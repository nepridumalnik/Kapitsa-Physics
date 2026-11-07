use super::row::RowDimension;
#[cfg(test)]
use super::workspace::JointSolverCall;
use super::{
    JointIterations, JointSolverOptions, JointSolverStep,
    row::ConstraintRow,
    warm_start::{JointImpulseCache, WarmStartConfig},
    workspace::JointWorkspace,
};
#[cfg(feature = "bench-internals")]
use crate::execution::StepExecutor;
use crate::{
    collision::CollisionError,
    constraint::{self, SolverBody, SolverBodyAccess},
    rigid_body::solver_mass::{SolverMass, SolverMassCacheStatus, SolverMassSlot},
    transform::Transform,
};
use kapitsa_math::{Cholesky3, Quat, Vec3};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct JointResidual {
    pub(crate) max_linear_m: f64,
    pub(crate) max_angular_rad: f64,
    pub(crate) max_velocity_mps: f64,
    pub(crate) max_angular_velocity_radps: f64,
}

impl JointResidual {
    pub(crate) const ZERO: Self = Self {
        max_linear_m: 0.0,
        max_angular_rad: 0.0,
        max_velocity_mps: 0.0,
        max_angular_velocity_radps: 0.0,
    };

    pub(crate) fn is_valid(self) -> bool {
        [
            self.max_linear_m,
            self.max_angular_rad,
            self.max_velocity_mps,
            self.max_angular_velocity_radps,
        ]
        .into_iter()
        .all(|value| value.is_finite() && value >= 0.0)
    }

    #[allow(dead_code)]
    pub(crate) fn converged(self, tolerance: super::JointTolerance) -> bool {
        self.is_valid()
            && tolerance.validate()
            && self.max_linear_m <= tolerance.linear_m
            && self.max_angular_rad <= tolerance.angular_rad
            && self.max_velocity_mps <= tolerance.velocity_mps
            && self.max_angular_velocity_radps <= tolerance.angular_velocity_radps
    }

    pub(crate) fn position_converged(self, tolerance: super::JointTolerance) -> bool {
        self.is_valid()
            && tolerance.validate()
            && self.max_linear_m <= tolerance.linear_m
            && self.max_angular_rad <= tolerance.angular_rad
    }

    pub(crate) fn velocity_converged(self, tolerance: super::JointTolerance) -> bool {
        self.is_valid()
            && tolerance.validate()
            && self.max_velocity_mps <= tolerance.velocity_mps
            && self.max_angular_velocity_radps <= tolerance.angular_velocity_radps
    }
}

fn projected_kkt_residual(gradient: f64, multiplier: f64, lower: f64, upper: f64) -> f64 {
    if lower == upper {
        // With a fixed multiplier every gradient is balanced by the bound
        // reactions; only a multiplier outside that fixed value is infeasible.
        (multiplier - lower).abs()
    } else if multiplier <= lower {
        (-gradient).max(0.0)
    } else if multiplier >= upper {
        gradient.max(0.0)
    } else {
        gradient.abs()
    }
}

/// Shared sequential Gauss–Seidel/XPBD solver for prepared joint rows.
pub(crate) struct JointSolver;

#[derive(Clone, Copy)]
pub(crate) struct JointConstraint {
    pub(crate) id: crate::world::JointId,
    pub(crate) body_a: usize,
    pub(crate) body_b: usize,
    pub(crate) joint: super::Joint,
}

#[derive(Clone, Copy)]
struct RowSettings {
    body_a: usize,
    body_b: usize,
    common: super::JointCommon,
    dt: f64,
    position_enabled: bool,
    velocity_bias: bool,
    velocity_target: f64,
    force_bias: f64,
    force_driven: bool,
    multiplier_bounds: Option<(f64, f64)>,
    impulse_bounds: Option<(f64, f64)>,
}

#[derive(Clone, Copy)]
struct FixedJointGeometry {
    anchor: FixedAnchorGeometry,
    frame: FixedFrameGeometry,
}

#[derive(Clone, Copy)]
struct FixedAnchorGeometry {
    anchor_a: Vec3,
    anchor_b: Vec3,
    position_error: Vec3,
}

#[derive(Clone, Copy)]
struct FixedFrameGeometry {
    rotation_a: Quat,
    rotation_error: Vec3,
}

/// Geometry keyed by the connector frames and both body poses it was derived
/// from. A cached entry stays valid until one of these inputs changes.
#[derive(Clone, Copy)]
pub(crate) struct FixedJointGeometryCache {
    frame_a: super::JointFrame,
    frame_b: super::JointFrame,
    transform_a: Transform,
    transform_b: Transform,
    geometry: FixedJointGeometry,
}

#[derive(Clone, Copy)]
enum JointRowBuffer {
    Current,
    Preparing,
}

#[derive(Clone, Copy)]
enum FixedJointRowBlock {
    Anchor,
    Frame,
}

/// Cached `J M^-1 J^T` operator for one fixed-joint velocity block, stored as a
/// Cholesky factorization. The operator only depends on the block's row
/// Jacobians and solver masses, so velocity sweeps that leave poses and masses
/// unchanged can reuse the factorization and only substitute per iteration.
#[derive(Clone, Copy, Debug)]
struct FixedBlockVelocityOperator {
    masses: [SolverMass; 2],
    factor: Cholesky3,
}

/// Per-joint velocity operators for the anchor (keys 0..=2) and frame (keys
/// 3..=5) blocks. Populated on the first velocity sweep of a step and cleared
/// when the step or joint geometry resets.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FixedBlockVelocityOperators {
    anchor: Option<FixedBlockVelocityOperator>,
    frame: Option<FixedBlockVelocityOperator>,
    #[cfg(test)]
    recomputes: usize,
}

impl FixedBlockVelocityOperators {
    const fn slot(&self, block: FixedJointRowBlock) -> Option<FixedBlockVelocityOperator> {
        match block {
            FixedJointRowBlock::Anchor => self.anchor,
            FixedJointRowBlock::Frame => self.frame,
        }
    }

    fn set(&mut self, block: FixedJointRowBlock, operator: FixedBlockVelocityOperator) {
        let slot = match block {
            FixedJointRowBlock::Anchor => &mut self.anchor,
            FixedJointRowBlock::Frame => &mut self.frame,
        };
        *slot = Some(operator);
        #[cfg(test)]
        {
            self.recomputes = self.recomputes.saturating_add(1);
        }
    }

    #[cfg(test)]
    pub(crate) const fn recomputes(&self) -> usize {
        self.recomputes
    }
}

impl JointRowBuffer {
    const fn is_preparing(self) -> bool {
        matches!(self, Self::Preparing)
    }
}

#[derive(Default)]
pub(super) struct SolverMassCacheStats {
    #[cfg(feature = "bench-internals")]
    hits: u64,
    #[cfg(feature = "bench-internals")]
    misses: u64,
    #[cfg(feature = "bench-internals")]
    inverse_inertia_updates: u64,
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy, Default)]
struct GeneralizedImpulse {
    linear: Vec3,
    angular: Vec3,
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct VelocityPcgResult {
    pub(crate) iterations: usize,
    pub(crate) converged: bool,
    pub(crate) linear_residual_mps: f64,
    pub(crate) angular_residual_radps: f64,
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct PositionPcgResult {
    pub(crate) iterations: usize,
    pub(crate) converged: bool,
    pub(crate) linear_residual_m: f64,
    pub(crate) angular_residual_rad: f64,
}

#[cfg(feature = "bench-internals")]
#[derive(Default)]
pub(crate) struct VelocityPcgScratch {
    pub(super) rows_prepared: bool,
    system_prepared: bool,
    unavailable: bool,
    uses_incomplete_cholesky: bool,
    uses_block_jacobi: bool,
    masses: Vec<SolverMass>,
    residual: Vec<f64>,
    diagonal: Vec<f64>,
    solution: Vec<f64>,
    preconditioned: Vec<f64>,
    direction: Vec<f64>,
    product: Vec<f64>,
    reduction_partials: Vec<f64>,
    body_impulses: Vec<GeneralizedImpulse>,
    body_responses: Vec<GeneralizedImpulse>,
    row_terms_by_body: Vec<Vec<VelocityRowTerm>>,
    lower_pattern: Vec<Vec<(usize, f64)>>,
    preconditioner: IncompleteCholeskyPreconditioner,
    block_jacobi: Vec<VelocityBlockFactor>,
    system_prepare_time: std::time::Duration,
    matrix_vector_time: std::time::Duration,
    preconditioner_time: std::time::Duration,
    reduction_time: std::time::Duration,
    position_system_prepare_time: std::time::Duration,
    position_matrix_vector_time: std::time::Duration,
    position_preconditioner_time: std::time::Duration,
    position_reduction_time: std::time::Duration,
}

#[cfg(feature = "bench-internals")]
impl VelocityPcgScratch {
    pub(super) const fn new() -> Self {
        Self {
            rows_prepared: false,
            system_prepared: false,
            unavailable: false,
            uses_incomplete_cholesky: false,
            uses_block_jacobi: false,
            masses: Vec::new(),
            residual: Vec::new(),
            diagonal: Vec::new(),
            solution: Vec::new(),
            preconditioned: Vec::new(),
            direction: Vec::new(),
            product: Vec::new(),
            reduction_partials: Vec::new(),
            body_impulses: Vec::new(),
            body_responses: Vec::new(),
            row_terms_by_body: Vec::new(),
            lower_pattern: Vec::new(),
            preconditioner: IncompleteCholeskyPreconditioner::new(),
            block_jacobi: Vec::new(),
            system_prepare_time: std::time::Duration::ZERO,
            matrix_vector_time: std::time::Duration::ZERO,
            preconditioner_time: std::time::Duration::ZERO,
            reduction_time: std::time::Duration::ZERO,
            position_system_prepare_time: std::time::Duration::ZERO,
            position_matrix_vector_time: std::time::Duration::ZERO,
            position_preconditioner_time: std::time::Duration::ZERO,
            position_reduction_time: std::time::Duration::ZERO,
        }
    }

    pub(super) fn begin_step(&mut self) {
        self.rows_prepared = false;
        self.system_prepared = false;
        self.unavailable = false;
        self.uses_incomplete_cholesky = false;
        self.uses_block_jacobi = false;
        self.system_prepare_time = std::time::Duration::ZERO;
        self.matrix_vector_time = std::time::Duration::ZERO;
        self.preconditioner_time = std::time::Duration::ZERO;
        self.reduction_time = std::time::Duration::ZERO;
        self.position_system_prepare_time = std::time::Duration::ZERO;
        self.position_matrix_vector_time = std::time::Duration::ZERO;
        self.position_preconditioner_time = std::time::Duration::ZERO;
        self.position_reduction_time = std::time::Duration::ZERO;
    }
}

impl SolverMassCacheStats {
    fn accumulate(&mut self, other: Self) {
        #[cfg(feature = "bench-internals")]
        {
            self.hits = self.hits.saturating_add(other.hits);
            self.misses = self.misses.saturating_add(other.misses);
            self.inverse_inertia_updates = self
                .inverse_inertia_updates
                .saturating_add(other.inverse_inertia_updates);
        }
        #[cfg(not(feature = "bench-internals"))]
        let _ = other;
    }

    pub(super) fn record(&mut self, status: SolverMassCacheStatus) {
        #[cfg(feature = "bench-internals")]
        match status {
            SolverMassCacheStatus::Hit => {
                self.hits = self.hits.saturating_add(1);
            }
            SolverMassCacheStatus::Updated { inverse_inertia } => {
                self.misses = self.misses.saturating_add(1);
                if inverse_inertia {
                    self.inverse_inertia_updates = self.inverse_inertia_updates.saturating_add(1);
                }
            }
        }
        #[cfg(not(feature = "bench-internals"))]
        let _ = status;
    }
}

#[derive(Clone, Copy, Default)]
struct ProjectionGroupResult {
    pose_changed: bool,
    block_rows: usize,
    fallback_rows: usize,
}

fn record_solver_mass_cache_stats(workspace: &mut JointWorkspace, stats: SolverMassCacheStats) {
    #[cfg(feature = "bench-internals")]
    workspace.record_solver_mass_cache(stats.hits, stats.misses, stats.inverse_inertia_updates);
    #[cfg(not(feature = "bench-internals"))]
    let _ = (workspace, stats);
}

fn record_projection_metrics(workspace: &mut JointWorkspace, result: ProjectionGroupResult) {
    #[cfg(feature = "bench-internals")]
    {
        workspace.record_block_rows_solved(result.block_rows);
        workspace.record_scalar_rows_solved(result.fallback_rows);
        workspace.record_block_fallback_rows(result.fallback_rows);
    }
    #[cfg(not(feature = "bench-internals"))]
    let _ = (workspace, result);
}

impl JointSolver {
    pub(crate) fn solve<F>(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        dt: f64,
        position_iterations: u32,
        velocity_iterations: u32,
        prepare_rows: F,
    ) -> Result<(), CollisionError>
    where
        F: FnMut(&[SolverBody], &mut JointWorkspace) -> Result<(), CollisionError>,
    {
        Self::solve_with_options(
            workspace,
            bodies,
            dt,
            position_iterations,
            velocity_iterations,
            JointSolverOptions::legacy(),
            prepare_rows,
        )
    }

    pub(crate) fn solve_with_options<F>(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        dt: f64,
        position_iterations: u32,
        velocity_iterations: u32,
        options: JointSolverOptions,
        mut prepare_rows: F,
    ) -> Result<(), CollisionError>
    where
        F: FnMut(&[SolverBody], &mut JointWorkspace) -> Result<(), CollisionError>,
    {
        if !dt.is_finite()
            || dt <= 0.0
            || position_iterations == 0
            || velocity_iterations == 0
            || options.warm_start().enabled
            || !options.iterations().validate()
        {
            return Err(CollisionError::InvalidConfig);
        }
        workspace.begin_step();
        if matches!(options.iterations(), JointIterations::Adaptive { .. }) {
            return Self::solve_adaptive(
                workspace,
                bodies,
                dt,
                options.iterations(),
                options,
                &mut prepare_rows,
            );
        }
        for _ in 0..position_iterations {
            #[cfg(feature = "bench-internals")]
            workspace.record_position_iteration();
            Self::prepare_iteration(workspace, bodies, &mut prepare_rows)?;
            Self::solve_position_iteration(workspace, bodies, dt, options)?;
        }
        for _ in 0..velocity_iterations {
            #[cfg(feature = "bench-internals")]
            workspace.record_velocity_iteration();
            Self::prepare_iteration(workspace, bodies, &mut prepare_rows)?;
            Self::solve_velocity_iteration(workspace, bodies, dt, options)?;
        }
        Ok(())
    }

    fn solve_adaptive<F>(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        dt: f64,
        iterations: JointIterations,
        options: JointSolverOptions,
        prepare_rows: &mut F,
    ) -> Result<(), CollisionError>
    where
        F: FnMut(&[SolverBody], &mut JointWorkspace) -> Result<(), CollisionError>,
    {
        let JointIterations::Adaptive {
            position,
            velocity,
            tolerance,
        } = iterations
        else {
            return Err(CollisionError::InvalidConfig);
        };
        for iteration in 0..position.max {
            #[cfg(feature = "bench-internals")]
            workspace.record_position_iteration();
            Self::prepare_iteration(workspace, bodies, prepare_rows)?;
            Self::solve_position_iteration(workspace, bodies, dt, options)?;
            Self::prepare_iteration(workspace, bodies, prepare_rows)?;
            let residual = measure_prepared_rows(workspace.rows(), bodies, dt)?;
            workspace.set_residual(residual);
            if iteration + 1 >= position.min && residual.position_converged(tolerance) {
                break;
            }
        }
        for iteration in 0..velocity.max {
            #[cfg(feature = "bench-internals")]
            workspace.record_velocity_iteration();
            Self::prepare_iteration(workspace, bodies, prepare_rows)?;
            Self::solve_velocity_iteration(workspace, bodies, dt, options)?;
            Self::prepare_iteration(workspace, bodies, prepare_rows)?;
            let residual = measure_prepared_rows(workspace.rows(), bodies, dt)?;
            workspace.set_residual(residual);
            if iteration + 1 >= velocity.min && residual.velocity_converged(tolerance) {
                break;
            }
        }
        Ok(())
    }

    pub(crate) fn begin_step(workspace: &mut JointWorkspace) {
        workspace.begin_step();
    }

    pub(crate) fn prepare(
        workspace: &mut JointWorkspace,
        bodies: &[SolverBody],
        joints: &[JointConstraint],
        dt: f64,
    ) -> Result<(), CollisionError> {
        workspace.begin_prepare();
        prepare_joint_rows(bodies, workspace, joints, dt)?;
        workspace.finish_prepare();
        Ok(())
    }

    pub(crate) fn solve_direct_velocity(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        mass_cache: bool,
    ) -> Result<bool, CollisionError> {
        if mass_cache {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        let mut cache_stats = SolverMassCacheStats::default();
        let result = {
            let (rows, slots, scratch) = workspace.direct_velocity_buffers_mut();
            scratch.solve(rows, bodies, slots, mass_cache, &mut cache_stats)
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        result
    }

    pub(crate) fn solve_position_iteration(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        Self::position_sweep(workspace, bodies, dt, options, JointRowBuffer::Current)
    }

    pub(crate) fn solve_velocity_iteration(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        Self::velocity_sweep(workspace, bodies, dt, options, JointRowBuffer::Current)
    }

    #[cfg(feature = "bench-internals")]
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn solve_velocity_pcg(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        joints: &[JointConstraint],
        dt: f64,
        max_iterations: u32,
        incomplete_cholesky_preconditioner: bool,
        block_jacobi_preconditioner: bool,
        executor: &impl StepExecutor,
        worker_count: usize,
    ) -> Result<Option<VelocityPcgResult>, CollisionError> {
        if !workspace.velocity_pcg_rows_prepared() {
            Self::prepare(workspace, bodies, joints, dt)?;
            workspace.mark_velocity_pcg_rows_prepared();
        }
        let tolerance = super::JointTolerance {
            linear_m: 1.0e-5,
            angular_rad: 1.0e-5,
            velocity_mps: 1.0e-4,
            angular_velocity_radps: 1.0e-4,
        };
        let (result, timings) = {
            let (rows, row_joint_indices, scratch) = workspace.velocity_pcg_rows_and_scratch_mut();
            let result = solve_velocity_pcg_prepared(
                rows,
                row_joint_indices,
                bodies,
                max_iterations as usize,
                tolerance,
                incomplete_cholesky_preconditioner,
                block_jacobi_preconditioner,
                executor,
                worker_count,
                scratch,
            )?;
            (
                result,
                (
                    std::mem::take(&mut scratch.system_prepare_time),
                    std::mem::take(&mut scratch.matrix_vector_time),
                    std::mem::take(&mut scratch.preconditioner_time),
                    std::mem::take(&mut scratch.reduction_time),
                ),
            )
        };
        workspace.record_velocity_pcg_timings(timings.0, timings.1, timings.2, timings.3);
        if let Some(result) = result {
            workspace.record_velocity_pcg_result(result);
        }
        Ok(result)
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn solve_position_pcg(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        joints: &[JointConstraint],
        dt: f64,
        max_iterations: u32,
        incomplete_cholesky_preconditioner: bool,
    ) -> Result<Option<PositionPcgResult>, CollisionError> {
        Self::prepare(workspace, bodies, joints, dt)?;
        workspace.ensure_solver_mass_slots(bodies.len())?;
        let tolerance = super::JointTolerance {
            linear_m: 1.0e-5,
            angular_rad: 1.0e-5,
            velocity_mps: 1.0e-4,
            angular_velocity_radps: 1.0e-4,
        };
        let mut cache_stats = SolverMassCacheStats::default();
        let (result, timings) = {
            let (rows, slots, scratch) = workspace.position_pcg_rows_slots_and_scratch_mut();
            let result = solve_position_pcg_prepared(
                rows,
                slots,
                bodies,
                max_iterations as usize,
                tolerance,
                incomplete_cholesky_preconditioner,
                scratch,
                &mut cache_stats,
            )?;
            (
                result,
                (
                    std::mem::take(&mut scratch.position_system_prepare_time),
                    std::mem::take(&mut scratch.position_matrix_vector_time),
                    std::mem::take(&mut scratch.position_preconditioner_time),
                    std::mem::take(&mut scratch.position_reduction_time),
                ),
            )
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        workspace.record_position_pcg_timings(timings.0, timings.1, timings.2, timings.3);
        if let Some(result) = result {
            workspace.record_position_pcg_result(result);
        }
        Ok(result)
    }

    pub(crate) fn solve_position_joint<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        constraint: JointConstraint,
        joint_index: usize,
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        Self::solve_position_joint_impl(
            workspace,
            bodies,
            constraint,
            joint_index,
            dt,
            options,
            true,
        )
    }

    fn solve_position_joint_impl<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        constraint: JointConstraint,
        joint_index: usize,
        dt: f64,
        options: JointSolverOptions,
        allow_in_place: bool,
    ) -> Result<(), CollisionError> {
        // A fixed joint with six committed rows can update those rows in place:
        // the row count never changes, so the generic prepare/commit copy is
        // redundant. The first call of a step (or any layout mismatch) falls
        // back to the generic path below.
        if allow_in_place
            && options.projection() == super::JointProjection::FixedBlock3
            && Self::position_fixed_block_in_place(
                workspace,
                bodies,
                constraint,
                joint_index,
                dt,
                options,
            )?
        {
            return Ok(());
        }
        workspace.begin_joint_prepare(joint_index, constraint.id);
        prepare_joint_rows_for_constraint(bodies, workspace, constraint, dt)?;
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::JointRowsPrepared(joint_index));
        if options.projection() == super::JointProjection::FixedBlock3 {
            Self::position_joint_block_sweep(workspace, bodies, constraint, dt, options)?;
        } else if matches!(constraint.joint, super::Joint::Fixed(_))
            && workspace.solver_buffers_mut(true).0.len() == 6
            && [constraint.body_a, constraint.body_b]
                .into_iter()
                .any(|body_index| {
                    bodies.get(body_index).is_some_and(|body| {
                        body.body().physics().body_type() == crate::rigid_body::BodyType::Dynamic
                    })
                })
        {
            Self::position_joint_fixed_scalar_sweep(workspace, bodies, constraint, dt, options)?;
        } else {
            Self::position_sweep(workspace, bodies, dt, options, JointRowBuffer::Preparing)?;
        }
        workspace.finish_joint_prepare()?;
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::JointRowsCommitted(joint_index));
        Ok(())
    }

    pub(crate) fn solve_velocity_joint<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        constraint: JointConstraint,
        joint_index: usize,
        step: JointSolverStep<'_>,
        first_velocity_iteration: bool,
    ) -> Result<(), CollisionError> {
        let dt = step.dt();
        let options = step.options();
        workspace.begin_joint_prepare(joint_index, constraint.id);
        prepare_joint_rows_for_constraint(bodies, workspace, constraint, dt)?;
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::JointRowsPrepared(joint_index));
        if first_velocity_iteration && options.warm_start().enabled {
            let warm_start_cache = step.impulse_cache().ok_or(CollisionError::InvalidConfig)?;
            apply_warm_start(
                workspace,
                bodies,
                constraint,
                dt,
                options.warm_start(),
                warm_start_cache,
                JointRowBuffer::Preparing,
            )?;
        }
        if options.projection() == super::JointProjection::FixedBlock3 {
            Self::velocity_joint_block_sweep(
                workspace,
                bodies,
                constraint,
                dt,
                options,
                JointRowBuffer::Preparing,
            )?;
        } else {
            Self::velocity_sweep(workspace, bodies, dt, options, JointRowBuffer::Preparing)?;
        }
        workspace.finish_joint_prepare()?;
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::JointRowsCommitted(joint_index));
        Ok(())
    }

    fn position_sweep<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        dt: f64,
        options: JointSolverOptions,
        row_buffer: JointRowBuffer,
    ) -> Result<(), CollisionError> {
        if options.mass_cache() && workspace.solver_rows_len(row_buffer.is_preparing()) > 0 {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        #[cfg(any(test, feature = "bench-internals"))]
        let row_count = workspace.solver_rows_len(row_buffer.is_preparing());
        let mut cache_stats = SolverMassCacheStats::default();
        let result = {
            let (rows, slots) = workspace.solver_buffers_mut(row_buffer.is_preparing());
            solve_position_rows(rows, slots, bodies, dt, options, &mut cache_stats)
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        result?;
        #[cfg(feature = "bench-internals")]
        workspace.record_scalar_rows_solved(row_count);
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::PositionSweep {
            preparing_rows: row_buffer.is_preparing(),
            row_count,
        });
        Ok(())
    }

    fn position_sweep_fixed_block<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        body_indices: [usize; 2],
        dt: f64,
        options: JointSolverOptions,
        block: FixedJointRowBlock,
    ) -> Result<bool, CollisionError> {
        if options.mass_cache() {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        let before = body_indices.map(|index| {
            bodies
                .get(index)
                .map(|body| body.state().transform())
                .ok_or(CollisionError::InvalidContact)
        });
        let [before_a, before_b] = [before[0]?, before[1]?];
        #[cfg(any(test, feature = "bench-internals"))]
        let row_count = 3;
        let mut cache_stats = SolverMassCacheStats::default();
        let result = {
            let (anchor_rows, frame_rows, slots) = workspace.preparing_fixed_row_blocks_mut()?;
            let rows = match block {
                FixedJointRowBlock::Anchor => anchor_rows.as_mut_slice(),
                FixedJointRowBlock::Frame => frame_rows.as_mut_slice(),
            };
            solve_position_rows(rows, slots, bodies, dt, options, &mut cache_stats)
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        result?;
        #[cfg(feature = "bench-internals")]
        workspace.record_scalar_rows_solved(row_count);
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::PositionSweep {
            preparing_rows: true,
            row_count,
        });
        let after_a = bodies
            .get(body_indices[0])
            .map(|body| body.state().transform())
            .ok_or(CollisionError::InvalidContact)?;
        let after_b = bodies
            .get(body_indices[1])
            .map(|body| body.state().transform())
            .ok_or(CollisionError::InvalidContact)?;
        Ok(before_a != after_a || before_b != after_b)
    }

    fn position_joint_fixed_scalar_sweep<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        constraint: JointConstraint,
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        let linear_changed = Self::position_sweep_fixed_block(
            workspace,
            bodies,
            [constraint.body_a, constraint.body_b],
            dt,
            options,
            FixedJointRowBlock::Anchor,
        )?;
        if linear_changed {
            refresh_fixed_joint_row_block(
                workspace,
                bodies,
                constraint,
                dt,
                FixedJointRowBlock::Frame,
            )?;
        }
        let angular_changed = Self::position_sweep_fixed_block(
            workspace,
            bodies,
            [constraint.body_a, constraint.body_b],
            dt,
            options,
            FixedJointRowBlock::Frame,
        )?;
        if angular_changed {
            refresh_fixed_joint_row_block(
                workspace,
                bodies,
                constraint,
                dt,
                FixedJointRowBlock::Anchor,
            )?;
            Self::position_sweep_fixed_block(
                workspace,
                bodies,
                [constraint.body_a, constraint.body_b],
                dt,
                options,
                FixedJointRowBlock::Anchor,
            )?;
        }
        Ok(())
    }

    fn velocity_sweep<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        dt: f64,
        options: JointSolverOptions,
        row_buffer: JointRowBuffer,
    ) -> Result<(), CollisionError> {
        if options.mass_cache() && workspace.solver_rows_len(row_buffer.is_preparing()) > 0 {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        #[cfg(any(test, feature = "bench-internals"))]
        let row_count = workspace.solver_rows_len(row_buffer.is_preparing());
        let mut cache_stats = SolverMassCacheStats::default();
        let result = {
            let (rows, slots) = workspace.solver_buffers_mut(row_buffer.is_preparing());
            solve_velocity_rows(rows, slots, bodies, dt, options, &mut cache_stats)
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        result?;
        #[cfg(feature = "bench-internals")]
        workspace.record_scalar_rows_solved(row_count);
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::VelocitySweep {
            preparing_rows: row_buffer.is_preparing(),
            row_count,
        });
        Ok(())
    }

    fn position_joint_block_sweep<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        constraint: JointConstraint,
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        if options.mass_cache() {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        let super::Joint::Fixed(_) = constraint.joint else {
            #[cfg(feature = "bench-internals")]
            let count = workspace.solver_buffers_mut(true).0.len();
            let mut cache_stats = SolverMassCacheStats::default();
            let result = {
                let (rows, slots) = workspace.solver_buffers_mut(true);
                solve_position_rows(rows, slots, bodies, dt, options, &mut cache_stats)
            };
            record_solver_mass_cache_stats(workspace, cache_stats);
            result?;
            #[cfg(feature = "bench-internals")]
            {
                workspace.record_scalar_rows_solved(count);
                workspace.record_block_fallback_rows(count);
            }
            return Ok(());
        };

        let linear = Self::position_projection_group(workspace, bodies, dt, options, [0, 1, 2])?;
        if linear.pose_changed {
            // A positional correction may rotate either body. Rebuild only the
            // angular block before the next group, preserving other rows.
            refresh_fixed_joint_row_block(
                workspace,
                bodies,
                constraint,
                dt,
                FixedJointRowBlock::Frame,
            )?;
        }
        let angular = Self::position_projection_group(workspace, bodies, dt, options, [3, 4, 5])?;
        if angular.pose_changed {
            // Angular correction changes offset connector positions. Refresh
            // only the point block before the final linear pass.
            refresh_fixed_joint_row_block(
                workspace,
                bodies,
                constraint,
                dt,
                FixedJointRowBlock::Anchor,
            )?;
            Self::position_projection_group(workspace, bodies, dt, options, [0, 1, 2])?;
        }
        Self::position_projection_remainder(workspace, bodies, dt, options)?;
        Ok(())
    }

    /// Updates the six committed FixedJoint rows in place for the block
    /// projection. Returns `false` when the committed layout does not match so
    /// the caller can use the generic prepare path.
    fn position_fixed_block_in_place<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        constraint: JointConstraint,
        joint_index: usize,
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<bool, CollisionError> {
        let super::Joint::Fixed(joint) = constraint.joint else {
            return Ok(false);
        };
        if !joint.common.enabled || workspace.committed_joint_id(joint_index) != Some(constraint.id)
        {
            return Ok(false);
        }
        let (start, end) = workspace.joint_row_range(joint_index);
        if end.saturating_sub(start) != 6 {
            return Ok(false);
        }
        {
            let Some(rows) = workspace.rows().get(start..end) else {
                return Ok(false);
            };
            for (offset, row) in rows.iter().enumerate() {
                if row.key != offset as u8
                    || row.body_a != constraint.body_a
                    || row.body_b != constraint.body_b
                    || workspace.row_owner(start + offset) != Some(joint_index)
                {
                    return Ok(false);
                }
            }
        }
        if options.mass_cache() {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        workspace.set_active_joint_index(joint_index);
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::JointRowsPrepared(joint_index));

        // Only the angular block is solved first after the linear rows; building
        // the frame rows here and after the linear pass would duplicate work.
        let masses = Self::fixed_position_masses(workspace, bodies, start, options)?;
        Self::write_fixed_position_block(
            workspace,
            bodies,
            constraint,
            dt,
            start,
            FixedJointRowBlock::Anchor,
            masses,
        )?;
        Self::position_projection_group_range(
            workspace,
            bodies,
            dt,
            options,
            [0, 1, 2],
            start..start + 3,
            masses,
        )?;
        // The previous iteration's frame rows predate the last angular solve, so
        // they must be rebuilt at the post-linear pose regardless of whether the
        // linear pass moved the bodies.
        let masses = Self::fixed_position_masses(workspace, bodies, start + 3, options)?;
        Self::write_fixed_position_block(
            workspace,
            bodies,
            constraint,
            dt,
            start + 3,
            FixedJointRowBlock::Frame,
            masses,
        )?;
        let angular = Self::position_projection_group_range(
            workspace,
            bodies,
            dt,
            options,
            [3, 4, 5],
            start + 3..start + 6,
            masses,
        )?;
        if angular.pose_changed {
            // Angular correction changes offset connector positions; refresh only
            // the anchor rows before the final linear pass.
            let masses = Self::fixed_position_masses(workspace, bodies, start, options)?;
            Self::write_fixed_position_block(
                workspace,
                bodies,
                constraint,
                dt,
                start,
                FixedJointRowBlock::Anchor,
                masses,
            )?;
            Self::position_projection_group_range(
                workspace,
                bodies,
                dt,
                options,
                [0, 1, 2],
                start..start + 3,
                masses,
            )?;
        }
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::JointRowsCommitted(joint_index));
        Ok(true)
    }

    fn write_fixed_position_block<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &B,
        constraint: JointConstraint,
        dt: f64,
        start: usize,
        block: FixedJointRowBlock,
        masses: [SolverMass; 2],
    ) -> Result<(), CollisionError> {
        let super::Joint::Fixed(joint) = constraint.joint else {
            return Err(CollisionError::InvalidContact);
        };
        #[cfg(feature = "bench-internals")]
        let preparation_started = std::time::Instant::now();
        let mut settings = row_settings(constraint.body_a, constraint.body_b, joint.common, dt);
        settings.velocity_bias = false;
        let frames = [joint.common.frame_a, joint.common.frame_b];
        let replacements = match block {
            FixedJointRowBlock::Anchor => {
                let mut centers = [Vec3::ZERO; 2];
                for (i, index) in [settings.body_a, settings.body_b].into_iter().enumerate() {
                    centers[i] = if masses[i].inverse_mass > 0.0 {
                        masses[i].center
                    } else {
                        center_of_mass(bodies.get(index).ok_or(CollisionError::InvalidContact)?)?
                    };
                }
                fixed_anchor_rows_with_centers(
                    settings,
                    fixed_anchor_geometry(bodies, settings, frames)?,
                    centers,
                )?
            }
            FixedJointRowBlock::Frame => {
                fixed_frame_rows(settings, fixed_frame_geometry(bodies, settings, frames)?)?
            }
        };
        let rows = workspace
            .rows_mut()
            .get_mut(start..start + replacements.len())
            .ok_or(CollisionError::InvalidContact)?;
        write_fixed_rows(rows, replacements)?;
        #[cfg(feature = "bench-internals")]
        {
            workspace.record_incremental_rows_prepared(replacements.len());
            workspace.record_joint_row_preparation_time(preparation_started.elapsed());
        }
        Ok(())
    }

    fn fixed_position_masses<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &B,
        start: usize,
        options: JointSolverOptions,
    ) -> Result<[SolverMass; 2], CollisionError> {
        let mut stats = SolverMassCacheStats::default();
        let masses = {
            let (rows, slots) = workspace.row_range_and_slots_mut(start, start + 3)?;
            projection_masses(
                bodies,
                slots,
                [rows[0], rows[1], rows[2]],
                options,
                &mut stats,
            )?
        };
        record_solver_mass_cache_stats(workspace, stats);
        Ok(masses)
    }

    fn position_projection_group_range<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        dt: f64,
        options: JointSolverOptions,
        keys: [u8; 3],
        range: std::ops::Range<usize>,
        masses: [SolverMass; 2],
    ) -> Result<ProjectionGroupResult, CollisionError> {
        let mut cache_stats = SolverMassCacheStats::default();
        let result = {
            let (rows, slots) = workspace.row_range_and_slots_mut(range.start, range.end)?;
            solve_position_projection_group(
                rows,
                slots,
                bodies,
                dt,
                options,
                (keys, Some(masses)),
                &mut cache_stats,
            )?
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        record_projection_metrics(workspace, result);
        Ok(result)
    }

    fn velocity_joint_block_sweep<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        constraint: JointConstraint,
        dt: f64,
        options: JointSolverOptions,
        row_buffer: JointRowBuffer,
    ) -> Result<(), CollisionError> {
        if options.mass_cache() {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        let super::Joint::Fixed(_) = constraint.joint else {
            #[cfg(feature = "bench-internals")]
            let count = workspace.solver_rows_len(row_buffer.is_preparing());
            let mut cache_stats = SolverMassCacheStats::default();
            let result = {
                let (rows, slots) = workspace.solver_buffers_mut(row_buffer.is_preparing());
                solve_velocity_rows(rows, slots, bodies, dt, options, &mut cache_stats)
            };
            record_solver_mass_cache_stats(workspace, cache_stats);
            result?;
            #[cfg(feature = "bench-internals")]
            {
                workspace.record_scalar_rows_solved(count);
                workspace.record_block_fallback_rows(count);
            }
            return Ok(());
        };

        workspace.ensure_fixed_velocity_operators(workspace.active_joint_index())?;
        Self::velocity_projection_group(workspace, bodies, dt, options, [0, 1, 2], row_buffer)?;
        Self::velocity_projection_group(workspace, bodies, dt, options, [3, 4, 5], row_buffer)?;
        Self::velocity_projection_remainder(workspace, bodies, dt, options, row_buffer)?;
        Ok(())
    }

    /// Prepares one joint's velocity rows and commits them without solving.
    pub(crate) fn prepare_velocity_joint<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &B,
        constraint: JointConstraint,
        joint_index: usize,
        dt: f64,
    ) -> Result<(), CollisionError> {
        workspace.begin_joint_prepare(joint_index, constraint.id);
        prepare_joint_rows_for_constraint(bodies, workspace, constraint, dt)?;
        workspace.finish_joint_prepare()?;
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::JointRowsPrepared(joint_index));
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::JointRowsCommitted(joint_index));
        Ok(())
    }

    /// Solves one velocity iteration against rows committed earlier in the step.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn solve_velocity_joint_prepared<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        constraint: JointConstraint,
        joint_index: usize,
        step: JointSolverStep<'_>,
        first_velocity_iteration: bool,
    ) -> Result<(), CollisionError> {
        let dt = step.dt();
        let options = step.options();
        workspace.set_active_joint_index(joint_index);
        if first_velocity_iteration && options.warm_start().enabled {
            let warm_start_cache = step.impulse_cache().ok_or(CollisionError::InvalidConfig)?;
            apply_warm_start(
                workspace,
                bodies,
                constraint,
                dt,
                options.warm_start(),
                warm_start_cache,
                JointRowBuffer::Current,
            )?;
        }
        let mut cache_stats = SolverMassCacheStats::default();
        Self::velocity_prepared_joint_rows(
            workspace,
            bodies,
            joint_index,
            matches!(constraint.joint, super::Joint::Fixed(_)),
            dt,
            options,
            &mut cache_stats,
        )?;
        record_solver_mass_cache_stats(workspace, cache_stats);
        Ok(())
    }

    /// Solves one velocity iteration against rows prepared once for the step.
    pub(crate) fn solve_velocity_prepared_iteration<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        joints: &[JointConstraint],
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        if options.projection() == super::JointProjection::FixedBlock3 {
            Self::velocity_block_prepared_sweep(workspace, bodies, joints, dt, options)
        } else {
            Self::velocity_sweep(workspace, bodies, dt, options, JointRowBuffer::Current)
        }
    }

    /// Applies cached 3x3 block solves to the prepared rows of every joint.
    fn velocity_block_prepared_sweep<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        joints: &[JointConstraint],
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        if options.mass_cache() {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        let mut cache_stats = SolverMassCacheStats::default();
        for (joint_index, constraint) in joints.iter().copied().enumerate() {
            let use_block = matches!(constraint.joint, super::Joint::Fixed(_));
            Self::velocity_prepared_joint_rows(
                workspace,
                bodies,
                joint_index,
                use_block,
                dt,
                options,
                &mut cache_stats,
            )?;
        }
        record_solver_mass_cache_stats(workspace, cache_stats);
        Ok(())
    }

    /// Solves the already-prepared row slice of one joint, using the 3x3 block
    /// path for fixed joints and the scalar fallback otherwise.
    fn velocity_prepared_joint_rows<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        joint_index: usize,
        is_fixed: bool,
        dt: f64,
        options: JointSolverOptions,
        cache_stats: &mut SolverMassCacheStats,
    ) -> Result<(), CollisionError> {
        let (start, end) = workspace.joint_row_range(joint_index);
        if start == end {
            return Ok(());
        }
        if options.mass_cache() {
            workspace.ensure_solver_mass_slots(bodies.len())?;
        }
        workspace.set_active_joint_index(joint_index);
        let row_count = end - start;
        if is_fixed && options.projection() == super::JointProjection::FixedBlock3 && row_count == 6
        {
            workspace.ensure_fixed_velocity_operators(joint_index)?;
            let (joint_stats, linear, angular, remainder) = {
                let (rows, slots, operators) =
                    workspace.solver_buffers_and_velocity_operators_mut(false);
                let rows = rows
                    .get_mut(start..end)
                    .ok_or(CollisionError::InvalidContact)?;
                let mut joint_stats = SolverMassCacheStats::default();
                let linear = solve_velocity_projection_group(
                    rows,
                    slots,
                    bodies,
                    dt,
                    options,
                    [0, 1, 2],
                    &mut joint_stats,
                    operators,
                )?;
                let angular = solve_velocity_projection_group(
                    rows,
                    slots,
                    bodies,
                    dt,
                    options,
                    [3, 4, 5],
                    &mut joint_stats,
                    operators,
                )?;
                let remainder = solve_velocity_projection_remainder(
                    rows,
                    slots,
                    bodies,
                    dt,
                    options,
                    &mut joint_stats,
                )?;
                (joint_stats, linear, angular, remainder)
            };
            cache_stats.accumulate(joint_stats);
            record_projection_metrics(workspace, linear);
            record_projection_metrics(workspace, angular);
            record_projection_metrics(workspace, remainder);
        } else {
            let mut joint_stats = SolverMassCacheStats::default();
            {
                let (rows, slots) = workspace.solver_buffers_mut(false);
                let rows = rows
                    .get_mut(start..end)
                    .ok_or(CollisionError::InvalidContact)?;
                solve_velocity_rows(rows, slots, bodies, dt, options, &mut joint_stats)?;
            }
            cache_stats.accumulate(joint_stats);
            #[cfg(feature = "bench-internals")]
            {
                workspace.record_scalar_rows_solved(row_count);
                workspace.record_block_fallback_rows(row_count);
            }
        }
        Ok(())
    }

    fn position_projection_group<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        dt: f64,
        options: JointSolverOptions,
        keys: [u8; 3],
    ) -> Result<ProjectionGroupResult, CollisionError> {
        let mut cache_stats = SolverMassCacheStats::default();
        let result = {
            let (rows, slots) = workspace.solver_buffers_mut(true);
            solve_position_projection_group(
                rows,
                slots,
                bodies,
                dt,
                options,
                (keys, None),
                &mut cache_stats,
            )?
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        record_projection_metrics(workspace, result);
        Ok(result)
    }

    fn velocity_projection_group<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        dt: f64,
        options: JointSolverOptions,
        keys: [u8; 3],
        row_buffer: JointRowBuffer,
    ) -> Result<(), CollisionError> {
        let mut cache_stats = SolverMassCacheStats::default();
        let result = {
            let (rows, slots, operators) =
                workspace.solver_buffers_and_velocity_operators_mut(row_buffer.is_preparing());
            solve_velocity_projection_group(
                rows,
                slots,
                bodies,
                dt,
                options,
                keys,
                &mut cache_stats,
                operators,
            )?
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        record_projection_metrics(workspace, result);
        Ok(())
    }

    fn position_projection_remainder<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        let mut cache_stats = SolverMassCacheStats::default();
        let stats = {
            let (rows, slots) = workspace.solver_buffers_mut(true);
            solve_position_projection_remainder(rows, slots, bodies, dt, options, &mut cache_stats)?
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        record_projection_metrics(workspace, stats);
        Ok(())
    }

    fn velocity_projection_remainder<B: SolverBodyAccess + ?Sized>(
        workspace: &mut JointWorkspace,
        bodies: &mut B,
        dt: f64,
        options: JointSolverOptions,
        row_buffer: JointRowBuffer,
    ) -> Result<(), CollisionError> {
        let mut cache_stats = SolverMassCacheStats::default();
        let stats = {
            let (rows, slots) = workspace.solver_buffers_mut(row_buffer.is_preparing());
            solve_velocity_projection_remainder(rows, slots, bodies, dt, options, &mut cache_stats)?
        };
        record_solver_mass_cache_stats(workspace, cache_stats);
        record_projection_metrics(workspace, stats);
        Ok(())
    }

    fn prepare_iteration<F>(
        workspace: &mut JointWorkspace,
        bodies: &[SolverBody],
        prepare_rows: &mut F,
    ) -> Result<(), CollisionError>
    where
        F: FnMut(&[SolverBody], &mut JointWorkspace) -> Result<(), CollisionError>,
    {
        workspace.begin_prepare();
        prepare_rows(bodies, workspace)?;
        workspace.finish_prepare();
        #[cfg(test)]
        workspace.record_solver_call(JointSolverCall::AllRowsPrepared);
        Ok(())
    }

    /// Rebuilds rows into the scratch buffer and measures them without touching
    /// the working rows, accumulated impulses, or velocity operator caches.
    pub(crate) fn measure_residual(
        workspace: &mut JointWorkspace,
        bodies: &[SolverBody],
        joints: &[JointConstraint],
        dt: f64,
    ) -> Result<(), CollisionError> {
        #[cfg(feature = "bench-internals")]
        {
            let metrics = workspace.metrics();
            workspace.begin_diagnostic_prepare();
            let result = prepare_joint_rows(bodies, workspace, joints, dt).and_then(|()| {
                let residual = measure_prepared_rows(workspace.preparing_rows(), bodies, dt)?;
                let geometry = measure_joint_geometry(
                    workspace.preparing_rows(),
                    workspace.preparing_row_joint_indices(),
                )?;
                Ok((residual, geometry))
            });
            workspace.abandon_prepare();
            let (residual, geometry) = result?;
            workspace.restore_metrics(metrics);
            workspace.set_geometry_residual(geometry);
            workspace.set_residual(residual);
            Ok(())
        }
        #[cfg(not(feature = "bench-internals"))]
        {
            workspace.begin_diagnostic_prepare();
            let result = prepare_joint_rows(bodies, workspace, joints, dt)
                .and_then(|()| measure_prepared_rows(workspace.preparing_rows(), bodies, dt));
            workspace.abandon_prepare();
            workspace.set_residual(result?);
            Ok(())
        }
    }

    /// Scratch-buffer residual measurement for adaptive convergence that skips
    /// the expensive bench-only geometry and work diagnostics.
    #[cfg(feature = "bench-internals")]
    pub(crate) fn measure_residual_light(
        workspace: &mut JointWorkspace,
        bodies: &[SolverBody],
        joints: &[JointConstraint],
        dt: f64,
    ) -> Result<(), CollisionError> {
        let metrics = workspace.metrics();
        workspace.begin_diagnostic_prepare();
        let result = prepare_joint_rows(bodies, workspace, joints, dt)
            .and_then(|()| measure_prepared_rows(workspace.preparing_rows(), bodies, dt));
        workspace.abandon_prepare();
        let residual = result?;
        // The preparation above is diagnostic noise, not solver work.
        workspace.restore_metrics(metrics);
        workspace.set_residual(residual);
        Ok(())
    }

    /// Measures the already-prepared working rows. Callers must only use this
    /// while row geometry still matches the current body poses.
    pub(crate) fn measure_current_residual(
        workspace: &mut JointWorkspace,
        bodies: &[SolverBody],
        dt: f64,
    ) -> Result<(), CollisionError> {
        let residual = measure_prepared_rows(workspace.rows(), bodies, dt)?;
        #[cfg(feature = "bench-internals")]
        {
            let geometry_residual =
                measure_joint_geometry(workspace.rows(), workspace.row_joint_indices())?;
            workspace.set_geometry_residual(geometry_residual);
        }
        workspace.set_residual(residual);
        Ok(())
    }
}

#[cfg(feature = "bench-internals")]
fn measure_joint_geometry(
    rows: &[ConstraintRow],
    row_joint_indices: &[usize],
) -> Result<super::diagnostics::JointGeometryResidual, CollisionError> {
    if rows.len() != row_joint_indices.len() {
        return Err(CollisionError::InvalidContact);
    }
    let mut residual = super::diagnostics::JointGeometryResidual::default();
    let mut row_index = 0;
    while row_index < rows.len() {
        let joint_index = row_joint_indices[row_index];
        let mut anchor_error_m: f64 = 0.0;
        let mut frame_error_rad: f64 = 0.0;
        while row_index < rows.len() && row_joint_indices[row_index] == joint_index {
            let row = rows[row_index];
            let value = row.error.abs();
            match row.dimension {
                RowDimension::Linear => anchor_error_m = anchor_error_m.hypot(value),
                RowDimension::Angular => frame_error_rad = frame_error_rad.hypot(value),
            }
            row_index += 1;
        }
        // FixedJoint rows are the three world-axis anchor components and three
        // components of the connector rotation log, so these norms match its
        // geometric residuals rather than the projected KKT residual.
        if !anchor_error_m.is_finite() || !frame_error_rad.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        residual.anchor_error_m = residual.anchor_error_m.max(anchor_error_m);
        residual.frame_error_rad = residual.frame_error_rad.max(frame_error_rad);
    }
    Ok(residual)
}

fn apply_warm_start<B: SolverBodyAccess + ?Sized>(
    workspace: &mut JointWorkspace,
    bodies: &mut B,
    constraint: JointConstraint,
    dt: f64,
    config: WarmStartConfig,
    cache: &JointImpulseCache,
    row_buffer: JointRowBuffer,
) -> Result<(), CollisionError> {
    if !config.validate() || !dt.is_finite() || dt <= 0.0 {
        return Err(CollisionError::InvalidConfig);
    }
    let mut applied_count = 0;
    {
        let (rows, _) = workspace.solver_buffers_mut(row_buffer.is_preparing());
        for index in 0..rows.len() {
            let row = *rows.get(index).ok_or(CollisionError::InvalidContact)?;
            if !row.is_valid(bodies.len()) {
                return Err(CollisionError::InvalidContact);
            }
            let Some(impulse) = cache.lookup(constraint.id, constraint.joint, row, dt, config)
            else {
                continue;
            };
            let next = (row.accumulated_impulse + impulse).clamp(row.min_impulse, row.max_impulse);
            let applied = next - row.accumulated_impulse;
            if !next.is_finite() || !applied.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            if applied != 0.0 {
                apply_velocity_row(bodies, row, applied, None)?;
                applied_count += 1;
            }
            rows[index].accumulated_impulse = next;
        }
    }
    #[cfg(feature = "bench-internals")]
    workspace.record_velocity_warm_starts(applied_count);
    #[cfg(not(feature = "bench-internals"))]
    let _ = applied_count;
    Ok(())
}

fn measure_prepared_rows(
    rows: &[ConstraintRow],
    bodies: &[SolverBody],
    dt: f64,
) -> Result<JointResidual, CollisionError> {
    let dt_squared = dt * dt;
    if !dt_squared.is_finite() || dt_squared <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let alpha_scale = dt_squared.recip();
    let mut residual = JointResidual::ZERO;
    for row in rows.iter().copied() {
        if !row.is_valid(bodies.len()) {
            return Err(CollisionError::InvalidContact);
        }
        if row.position_enabled {
            let gradient = row.error + row.compliance * alpha_scale * row.accumulated_multiplier;
            if !gradient.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            let position_error =
                if row.min_multiplier != f64::NEG_INFINITY || row.max_multiplier != f64::INFINITY {
                    projected_kkt_residual(
                        gradient,
                        row.accumulated_multiplier,
                        row.min_multiplier,
                        row.max_multiplier,
                    )
                } else {
                    gradient.abs()
                };
            update_position_residual(&mut residual, row.dimension, position_error)?;
        }

        let speed_error = row.target_velocity - row_velocity(bodies, row)?;
        if !speed_error.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let velocity_error =
            if row.min_impulse != f64::NEG_INFINITY || row.max_impulse != f64::INFINITY {
                projected_kkt_residual(
                    -speed_error,
                    row.accumulated_impulse,
                    row.min_impulse,
                    row.max_impulse,
                )
            } else {
                speed_error.abs()
            };
        update_velocity_residual(&mut residual, row.dimension, velocity_error)?;
    }
    if residual.is_valid() {
        Ok(residual)
    } else {
        Err(CollisionError::ArithmeticOverflow)
    }
}

#[cfg(feature = "bench-internals")]
#[allow(clippy::too_many_arguments)]
fn solve_position_pcg_prepared(
    rows: &mut [ConstraintRow],
    slots: &mut [SolverMassSlot],
    bodies: &mut [SolverBody],
    max_iterations: usize,
    tolerance: super::JointTolerance,
    incomplete_cholesky_preconditioner: bool,
    scratch: &mut VelocityPcgScratch,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<Option<PositionPcgResult>, CollisionError> {
    if rows.is_empty() {
        return Ok(Some(PositionPcgResult {
            converged: true,
            ..PositionPcgResult::default()
        }));
    }
    if max_iterations == 0
        || rows.iter().any(|row| {
            !row.position_enabled
                || row.compliance != 0.0
                || row.damping != 0.0
                || row.force_driven
                || row.min_multiplier != f64::NEG_INFINITY
                || row.max_multiplier != f64::INFINITY
        })
    {
        return Ok(None);
    }

    if slots.len() < bodies.len() {
        return Err(CollisionError::InvalidContact);
    }
    let preparation_started = std::time::Instant::now();
    scratch.masses.clear();
    scratch
        .masses
        .try_reserve(bodies.len())
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    for (body, slot) in bodies.iter().zip(slots) {
        let (mass, status) = slot.resolve(body.body())?;
        cache_stats.record(status);
        scratch.masses.push(mass);
    }

    resize_f64_buffer(&mut scratch.residual, rows.len())?;
    resize_f64_buffer(&mut scratch.diagonal, rows.len())?;
    for (index, row) in rows.iter().copied().enumerate() {
        if !row.is_valid(bodies.len()) {
            return Err(CollisionError::InvalidContact);
        }
        scratch.residual[index] = -row.error;
        let mass_a = *scratch
            .masses
            .get(row.body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let mass_b = *scratch
            .masses
            .get(row.body_b)
            .ok_or(CollisionError::InvalidContact)?;
        scratch.diagonal[index] = mass_a.diagonal(row.linear_a, row.angular_a)?
            + mass_b.diagonal(row.linear_b, row.angular_b)?;
        if !scratch.residual[index].is_finite() || !scratch.diagonal[index].is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if scratch.diagonal[index] <= f64::MIN_POSITIVE {
            if scratch.residual[index].abs() > position_row_tolerance(row, tolerance) {
                return Ok(None);
            }
            scratch.diagonal[index] = 1.0;
            scratch.residual[index] = 0.0;
        }
    }
    if position_vector_converged(rows, &scratch.residual, tolerance) {
        scratch.position_system_prepare_time += preparation_started.elapsed();
        return Ok(Some(position_pcg_result(rows, &scratch.residual, 0, true)));
    }

    if incomplete_cholesky_preconditioner
        && !build_incomplete_cholesky_preconditioner_into(
            rows,
            &scratch.masses,
            &scratch.diagonal,
            &mut scratch.row_terms_by_body,
            &mut scratch.lower_pattern,
            &mut scratch.preconditioner,
        )?
    {
        return Ok(None);
    }
    scratch.position_system_prepare_time += preparation_started.elapsed();

    resize_f64_buffer(&mut scratch.solution, rows.len())?;
    scratch.solution.fill(0.0);
    resize_f64_buffer(&mut scratch.preconditioned, rows.len())?;
    resize_f64_buffer(&mut scratch.direction, rows.len())?;
    resize_f64_buffer(&mut scratch.product, rows.len())?;
    let preconditioner = incomplete_cholesky_preconditioner.then_some(&scratch.preconditioner);
    let preconditioner_started = std::time::Instant::now();
    apply_velocity_preconditioner(
        &scratch.residual,
        &scratch.diagonal,
        preconditioner,
        &mut scratch.preconditioned,
    )?;
    scratch.position_preconditioner_time += preconditioner_started.elapsed();
    scratch.direction.copy_from_slice(&scratch.preconditioned);
    let reduction_started = std::time::Instant::now();
    let mut residual_product = checked_dot_product(&scratch.residual, &scratch.preconditioned)?;
    scratch.position_reduction_time += reduction_started.elapsed();
    if residual_product <= f64::MIN_POSITIVE {
        return Ok(None);
    }

    resize_impulse_buffer(&mut scratch.body_impulses, bodies.len())?;
    resize_impulse_buffer(&mut scratch.body_responses, bodies.len())?;
    let mut completed_iterations = 0;
    let mut converged = false;
    for iteration in 0..max_iterations {
        let matrix_vector_started = std::time::Instant::now();
        global_velocity_matrix_vector(
            rows,
            &scratch.direction,
            &scratch.masses,
            &mut scratch.body_impulses,
            &mut scratch.body_responses,
            &mut scratch.product,
        )?;
        scratch.position_matrix_vector_time += matrix_vector_started.elapsed();
        let reduction_started = std::time::Instant::now();
        let denominator = checked_dot_product(&scratch.direction, &scratch.product)?;
        scratch.position_reduction_time += reduction_started.elapsed();
        if denominator <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let alpha = residual_product / denominator;
        if !alpha.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        for index in 0..rows.len() {
            scratch.solution[index] += alpha * scratch.direction[index];
            scratch.residual[index] -= alpha * scratch.product[index];
            if !scratch.solution[index].is_finite() || !scratch.residual[index].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
        completed_iterations = iteration + 1;
        if position_vector_converged(rows, &scratch.residual, tolerance) {
            converged = true;
            break;
        }
        let preconditioner_started = std::time::Instant::now();
        apply_velocity_preconditioner(
            &scratch.residual,
            &scratch.diagonal,
            preconditioner,
            &mut scratch.preconditioned,
        )?;
        scratch.position_preconditioner_time += preconditioner_started.elapsed();
        let reduction_started = std::time::Instant::now();
        let next_residual_product =
            checked_dot_product(&scratch.residual, &scratch.preconditioned)?;
        scratch.position_reduction_time += reduction_started.elapsed();
        if next_residual_product <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let beta = next_residual_product / residual_product;
        if !beta.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        for index in 0..rows.len() {
            scratch.direction[index] =
                scratch.preconditioned[index] + beta * scratch.direction[index];
            if !scratch.direction[index].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
        residual_product = next_residual_product;
    }

    global_velocity_impulse(rows, &scratch.solution, &mut scratch.body_impulses)?;
    for ((body, impulse), mass) in bodies
        .iter_mut()
        .zip(scratch.body_impulses.iter().copied())
        .zip(scratch.masses.iter().copied())
    {
        if impulse.linear == Vec3::ZERO && impulse.angular == Vec3::ZERO {
            continue;
        }
        if body.body().physics().body_type() == crate::rigid_body::BodyType::Dynamic {
            body.body_mut()
                .apply_constraint_position_impulse_cached(impulse.linear, impulse.angular, mass)
                .map_err(map_rigid_body_error)?;
        }
    }
    for (row, delta) in rows.iter_mut().zip(scratch.solution.iter().copied()) {
        let next = row.accumulated_multiplier + delta;
        if !next.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        row.accumulated_multiplier = next;
    }
    Ok(Some(position_pcg_result(
        rows,
        &scratch.residual,
        completed_iterations,
        converged,
    )))
}

#[cfg(feature = "bench-internals")]
#[allow(clippy::too_many_arguments)]
fn solve_velocity_pcg_prepared(
    rows: &mut [ConstraintRow],
    row_joint_indices: &[usize],
    bodies: &mut [SolverBody],
    max_iterations: usize,
    tolerance: super::JointTolerance,
    incomplete_cholesky_preconditioner: bool,
    block_jacobi_preconditioner: bool,
    executor: &impl StepExecutor,
    worker_count: usize,
    scratch: &mut VelocityPcgScratch,
) -> Result<Option<VelocityPcgResult>, CollisionError> {
    if rows.is_empty() {
        return Ok(Some(VelocityPcgResult {
            converged: true,
            ..VelocityPcgResult::default()
        }));
    }
    if scratch.unavailable {
        return Ok(None);
    }
    if max_iterations == 0
        || rows.iter().any(|row| {
            row.compliance != 0.0
                || row.damping != 0.0
                || row.force_driven
                || row.min_impulse != f64::NEG_INFINITY
                || row.max_impulse != f64::INFINITY
        })
    {
        scratch.unavailable = true;
        return Ok(None);
    }

    resize_f64_buffer(&mut scratch.residual, rows.len())?;
    if rows.iter().any(|row| !row.is_valid(bodies.len())) {
        return Err(CollisionError::InvalidContact);
    }
    prepare_velocity_residual_parallel(
        rows,
        bodies,
        &mut scratch.residual,
        executor,
        worker_count,
    )?;
    let system_prepare_started = std::time::Instant::now();
    if !scratch.system_prepared {
        clear_mass_buffer(&mut scratch.masses, bodies.len())?;
        for body in bodies.iter() {
            scratch.masses.push(SolverMass::from_body(body.body())?);
        }
        resize_f64_buffer(&mut scratch.diagonal, rows.len())?;
    }
    if !scratch.system_prepared {
        for (index, row) in rows.iter().copied().enumerate() {
            let mass_a = *scratch
                .masses
                .get(row.body_a)
                .ok_or(CollisionError::InvalidContact)?;
            let mass_b = *scratch
                .masses
                .get(row.body_b)
                .ok_or(CollisionError::InvalidContact)?;
            scratch.diagonal[index] = mass_a.diagonal(row.linear_a, row.angular_a)?
                + mass_b.diagonal(row.linear_b, row.angular_b)?;
            if !scratch.diagonal[index].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            if scratch.diagonal[index] <= f64::MIN_POSITIVE {
                if scratch.residual[index].abs() > velocity_row_tolerance(row, tolerance) {
                    scratch.unavailable = true;
                    return Ok(None);
                }
                scratch.diagonal[index] = 1.0;
                scratch.residual[index] = 0.0;
            }
        }
    }
    if !scratch.system_prepared {
        scratch.uses_incomplete_cholesky = incomplete_cholesky_preconditioner;
        scratch.uses_block_jacobi = block_jacobi_preconditioner;
        if incomplete_cholesky_preconditioner && block_jacobi_preconditioner {
            scratch.unavailable = true;
            return Ok(None);
        }
        if block_jacobi_preconditioner
            && !build_velocity_block_jacobi_preconditioner(
                rows,
                row_joint_indices,
                &scratch.masses,
                &mut scratch.block_jacobi,
            )?
        {
            scratch.unavailable = true;
            return Ok(None);
        }
        if incomplete_cholesky_preconditioner
            && !build_incomplete_cholesky_preconditioner_into(
                rows,
                &scratch.masses,
                &scratch.diagonal,
                &mut scratch.row_terms_by_body,
                &mut scratch.lower_pattern,
                &mut scratch.preconditioner,
            )?
        {
            scratch.unavailable = true;
            return Ok(None);
        }
        if !incomplete_cholesky_preconditioner
            && !build_velocity_row_terms_by_body(
                rows,
                scratch.masses.len(),
                &mut scratch.row_terms_by_body,
            )?
        {
            scratch.unavailable = true;
            return Ok(None);
        }
        scratch.system_prepared = true;
        scratch.system_prepare_time += system_prepare_started.elapsed();
    }
    if velocity_vector_converged_parallel(
        rows,
        &scratch.residual,
        &mut scratch.reduction_partials,
        tolerance,
        executor,
        worker_count,
    )? {
        return Ok(Some(velocity_pcg_result(rows, &scratch.residual, 0, true)));
    }

    let preconditioner = if scratch.uses_incomplete_cholesky {
        Some(&scratch.preconditioner)
    } else {
        None
    };
    resize_f64_buffer(&mut scratch.solution, rows.len())?;
    resize_f64_buffer(&mut scratch.preconditioned, rows.len())?;
    resize_f64_buffer(&mut scratch.direction, rows.len())?;
    resize_f64_buffer(&mut scratch.product, rows.len())?;
    let stage_started = std::time::Instant::now();
    apply_velocity_pcg_preconditioner(
        &scratch.residual,
        &scratch.diagonal,
        preconditioner,
        scratch.uses_block_jacobi.then_some(&scratch.block_jacobi),
        &mut scratch.preconditioned,
        executor,
        worker_count,
    )?;
    scratch.preconditioner_time += stage_started.elapsed();
    scratch.direction.copy_from_slice(&scratch.preconditioned);
    let stage_started = std::time::Instant::now();
    let mut residual_product = checked_dot_product_parallel(
        &scratch.residual,
        &scratch.preconditioned,
        &mut scratch.reduction_partials,
        executor,
        worker_count,
    )?;
    scratch.reduction_time += stage_started.elapsed();
    if residual_product <= f64::MIN_POSITIVE {
        return Ok(None);
    }

    resize_impulse_buffer(&mut scratch.body_impulses, bodies.len())?;
    resize_impulse_buffer(&mut scratch.body_responses, bodies.len())?;
    let mut completed_iterations = 0;
    let mut converged = false;
    for iteration in 0..max_iterations {
        let stage_started = std::time::Instant::now();
        global_velocity_matrix_vector_parallel(
            rows,
            &scratch.direction,
            &scratch.masses,
            &scratch.row_terms_by_body,
            &mut scratch.body_impulses,
            &mut scratch.body_responses,
            &mut scratch.product,
            executor,
            worker_count,
        )?;
        scratch.matrix_vector_time += stage_started.elapsed();
        let stage_started = std::time::Instant::now();
        let denominator = checked_dot_product_parallel(
            &scratch.direction,
            &scratch.product,
            &mut scratch.reduction_partials,
            executor,
            worker_count,
        )?;
        scratch.reduction_time += stage_started.elapsed();
        if denominator <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let alpha = residual_product / denominator;
        if !alpha.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        velocity_pcg_scaled_add(
            &mut scratch.solution,
            &scratch.direction,
            alpha,
            executor,
            worker_count,
        )?;
        velocity_pcg_scaled_add(
            &mut scratch.residual,
            &scratch.product,
            -alpha,
            executor,
            worker_count,
        )?;
        completed_iterations = iteration + 1;
        if velocity_vector_converged_parallel(
            rows,
            &scratch.residual,
            &mut scratch.reduction_partials,
            tolerance,
            executor,
            worker_count,
        )? {
            converged = true;
            break;
        }

        let stage_started = std::time::Instant::now();
        apply_velocity_pcg_preconditioner(
            &scratch.residual,
            &scratch.diagonal,
            preconditioner,
            scratch.uses_block_jacobi.then_some(&scratch.block_jacobi),
            &mut scratch.preconditioned,
            executor,
            worker_count,
        )?;
        scratch.preconditioner_time += stage_started.elapsed();
        let stage_started = std::time::Instant::now();
        let next_residual_product = checked_dot_product_parallel(
            &scratch.residual,
            &scratch.preconditioned,
            &mut scratch.reduction_partials,
            executor,
            worker_count,
        )?;
        scratch.reduction_time += stage_started.elapsed();
        if next_residual_product <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let beta = next_residual_product / residual_product;
        if !beta.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        velocity_pcg_update_direction(
            &mut scratch.direction,
            &scratch.preconditioned,
            beta,
            executor,
            worker_count,
        )?;
        residual_product = next_residual_product;
    }

    global_velocity_impulse_parallel(
        &scratch.solution,
        &scratch.row_terms_by_body,
        &mut scratch.body_impulses,
        executor,
        worker_count,
    )?;
    for (body, impulse) in bodies.iter_mut().zip(scratch.body_impulses.iter().copied()) {
        if impulse.linear == Vec3::ZERO && impulse.angular == Vec3::ZERO {
            continue;
        }
        if body.body().physics().body_type() == crate::rigid_body::BodyType::Dynamic {
            body.body_mut()
                .apply_constraint_impulse(impulse.linear, impulse.angular)
                .map_err(map_rigid_body_error)?;
        }
    }
    for (row, delta) in rows.iter_mut().zip(scratch.solution.iter().copied()) {
        let next = row.accumulated_impulse + delta;
        if !next.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        row.accumulated_impulse = next;
    }
    Ok(Some(velocity_pcg_result(
        rows,
        &scratch.residual,
        completed_iterations,
        converged,
    )))
}

#[cfg(feature = "bench-internals")]
#[derive(Default)]
struct IncompleteCholeskyPreconditioner {
    lower_rows: Vec<Vec<(usize, f64)>>,
    lower_columns: Vec<Vec<(usize, f64)>>,
    diagonal: Vec<f64>,
}

#[cfg(feature = "bench-internals")]
impl IncompleteCholeskyPreconditioner {
    const fn new() -> Self {
        Self {
            lower_rows: Vec::new(),
            lower_columns: Vec::new(),
            diagonal: Vec::new(),
        }
    }
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy)]
struct VelocityRowTerm {
    row: usize,
    linear: Vec3,
    angular: Vec3,
}

#[cfg(feature = "bench-internals")]
fn build_velocity_row_terms_by_body(
    rows: &[ConstraintRow],
    body_count: usize,
    row_terms_by_body: &mut Vec<Vec<VelocityRowTerm>>,
) -> Result<bool, CollisionError> {
    resize_nested_buffer(row_terms_by_body, body_count)?;
    for (row_index, row) in rows.iter().copied().enumerate() {
        for (body_index, linear, angular) in [
            (row.body_a, row.linear_a, row.angular_a),
            (row.body_b, row.linear_b, row.angular_b),
        ] {
            let Some(terms) = row_terms_by_body.get_mut(body_index) else {
                return Ok(false);
            };
            terms.push(VelocityRowTerm {
                row: row_index,
                linear,
                angular,
            });
        }
    }
    Ok(true)
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy)]
struct VelocityBlockFactor {
    start: usize,
    len: usize,
    lower: [f64; 36],
}

#[cfg(feature = "bench-internals")]
fn build_velocity_block_jacobi_preconditioner(
    rows: &[ConstraintRow],
    row_joint_indices: &[usize],
    masses: &[SolverMass],
    blocks: &mut Vec<VelocityBlockFactor>,
) -> Result<bool, CollisionError> {
    if row_joint_indices.len() != rows.len() {
        return Ok(false);
    }
    let block_count = row_joint_indices
        .iter()
        .enumerate()
        .filter(|(index, owner)| *index == 0 || row_joint_indices[*index - 1] != **owner)
        .count();
    if block_count > blocks.capacity() {
        blocks
            .try_reserve_exact(block_count - blocks.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    blocks.clear();
    let mut start = 0;
    while start < rows.len() {
        let owner = row_joint_indices[start];
        let end = row_joint_indices[start..]
            .iter()
            .position(|candidate| *candidate != owner)
            .map_or(rows.len(), |offset| start + offset);
        let len = end - start;
        if len == 0 || len > 6 {
            return Ok(false);
        }
        let mut block = VelocityBlockFactor {
            start,
            len,
            lower: [0.0; 36],
        };
        for row in 0..len {
            let row_i = rows[start + row];
            for column in 0..=row {
                let row_j = rows[start + column];
                if row_i.body_a != row_j.body_a || row_i.body_b != row_j.body_b {
                    return Ok(false);
                }
                let mass_a = *masses
                    .get(row_i.body_a)
                    .ok_or(CollisionError::InvalidContact)?;
                let mass_b = *masses
                    .get(row_i.body_b)
                    .ok_or(CollisionError::InvalidContact)?;
                let mut value = mass_a.cross(
                    row_i.linear_a,
                    row_i.angular_a,
                    row_j.linear_a,
                    row_j.angular_a,
                )? + mass_b.cross(
                    row_i.linear_b,
                    row_i.angular_b,
                    row_j.linear_b,
                    row_j.angular_b,
                )?;
                for previous in 0..column {
                    value -= block.lower[row * 6 + previous] * block.lower[column * 6 + previous];
                }
                if !value.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                if row == column {
                    if value <= f64::MIN_POSITIVE {
                        return Ok(false);
                    }
                    block.lower[row * 6 + column] = value.sqrt();
                } else {
                    let factor = value / block.lower[column * 6 + column];
                    if !factor.is_finite() {
                        return Ok(false);
                    }
                    block.lower[row * 6 + column] = factor;
                }
            }
        }
        blocks.push(block);
        start = end;
    }
    Ok(true)
}

#[cfg(feature = "bench-internals")]
fn apply_velocity_pcg_preconditioner(
    residual: &[f64],
    diagonal: &[f64],
    incomplete_cholesky: Option<&IncompleteCholeskyPreconditioner>,
    block_jacobi: Option<&[VelocityBlockFactor]>,
    output: &mut [f64],
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if let Some(blocks) = block_jacobi {
        apply_block_jacobi_preconditioner(residual, blocks, output)
    } else if incomplete_cholesky.is_some() {
        apply_velocity_preconditioner(residual, diagonal, incomplete_cholesky, output)
    } else {
        if residual.len() != diagonal.len() || residual.len() != output.len() {
            return Err(CollisionError::InvalidContact);
        }
        let chunk_size = velocity_pcg_chunk_size(output.len(), worker_count);
        velocity_pcg_for_each_chunk_mut(output, chunk_size, executor, |chunk_index, chunk| {
            let start = chunk_index * chunk_size;
            for (offset, value) in chunk.iter_mut().enumerate() {
                let index = start + offset;
                *value = residual[index] / diagonal[index];
            }
        })?;
        if output.iter().any(|value| !value.is_finite()) {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(())
    }
}

#[cfg(feature = "bench-internals")]
fn apply_block_jacobi_preconditioner(
    residual: &[f64],
    blocks: &[VelocityBlockFactor],
    output: &mut [f64],
) -> Result<(), CollisionError> {
    if residual.len() != output.len() {
        return Err(CollisionError::InvalidContact);
    }
    for block in blocks {
        for row in 0..block.len {
            let mut value = residual[block.start + row];
            for column in 0..row {
                value -= block.lower[row * 6 + column] * output[block.start + column];
            }
            output[block.start + row] = value / block.lower[row * 6 + row];
        }
        for row in (0..block.len).rev() {
            let mut value = output[block.start + row];
            for lower_row in row + 1..block.len {
                value -= block.lower[lower_row * 6 + row] * output[block.start + lower_row];
            }
            output[block.start + row] = value / block.lower[row * 6 + row];
            if !output[block.start + row].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn build_incomplete_cholesky_preconditioner_into(
    rows: &[ConstraintRow],
    masses: &[SolverMass],
    diagonal: &[f64],
    row_terms_by_body: &mut Vec<Vec<VelocityRowTerm>>,
    lower_pattern: &mut Vec<Vec<(usize, f64)>>,
    preconditioner: &mut IncompleteCholeskyPreconditioner,
) -> Result<bool, CollisionError> {
    if diagonal.len() != rows.len() {
        return Ok(false);
    }
    if !build_velocity_row_terms_by_body(rows, masses.len(), row_terms_by_body)? {
        return Ok(false);
    }
    resize_nested_buffer(lower_pattern, rows.len())?;

    for (body_index, terms) in row_terms_by_body.iter().enumerate() {
        let mass = *masses
            .get(body_index)
            .ok_or(CollisionError::InvalidContact)?;
        for right in 0..terms.len() {
            for left in 0..right {
                let left_term = terms[left];
                let right_term = terms[right];
                let (row, column) = if left_term.row > right_term.row {
                    (left_term.row, right_term.row)
                } else {
                    (right_term.row, left_term.row)
                };
                let value = mass.cross(
                    left_term.linear,
                    left_term.angular,
                    right_term.linear,
                    right_term.angular,
                )?;
                if !value.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                lower_pattern[row].push((column, value));
            }
        }
    }

    for entries in lower_pattern.iter_mut() {
        entries.sort_unstable_by_key(|(column, _)| *column);
        let mut write = 0;
        for read in 0..entries.len() {
            let (column, value) = entries[read];
            if write > 0 && entries[write - 1].0 == column {
                entries[write - 1].1 += value;
                if !entries[write - 1].1.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
            } else {
                entries[write] = (column, value);
                write += 1;
            }
        }
        entries.truncate(write);
    }

    resize_nested_buffer(&mut preconditioner.lower_rows, rows.len())?;
    resize_nested_buffer(&mut preconditioner.lower_columns, rows.len())?;
    resize_f64_buffer(&mut preconditioner.diagonal, rows.len())?;

    for row in 0..rows.len() {
        for &(column, matrix_value) in &lower_pattern[row] {
            let mut value = matrix_value;
            let row_factors = &preconditioner.lower_rows[row];
            let column_factors = &preconditioner.lower_rows[column];
            let (mut row_factor_index, mut column_factor_index) = (0, 0);
            while row_factor_index < row_factors.len() && column_factor_index < column_factors.len()
            {
                let (row_column, row_factor) = row_factors[row_factor_index];
                if row_column >= column {
                    break;
                }
                let (column_column, column_factor) = column_factors[column_factor_index];
                match row_column.cmp(&column_column) {
                    std::cmp::Ordering::Less => row_factor_index += 1,
                    std::cmp::Ordering::Greater => column_factor_index += 1,
                    std::cmp::Ordering::Equal => {
                        value -= row_factor * column_factor;
                        row_factor_index += 1;
                        column_factor_index += 1;
                    }
                }
            }
            let factor = value / preconditioner.diagonal[column];
            if !factor.is_finite() {
                return Ok(false);
            }
            preconditioner.lower_rows[row].push((column, factor));
            preconditioner.lower_columns[column].push((row, factor));
        }
        let mut diagonal_value = diagonal[row];
        for &(_, factor) in &preconditioner.lower_rows[row] {
            diagonal_value -= factor * factor;
        }
        if !diagonal_value.is_finite() || diagonal_value <= f64::MIN_POSITIVE {
            return Ok(false);
        }
        preconditioner.diagonal[row] = diagonal_value.sqrt();
    }
    Ok(true)
}

#[cfg(feature = "bench-internals")]
fn apply_velocity_preconditioner(
    residual: &[f64],
    diagonal: &[f64],
    preconditioner: Option<&IncompleteCholeskyPreconditioner>,
    output: &mut [f64],
) -> Result<(), CollisionError> {
    if residual.len() != diagonal.len() || residual.len() != output.len() {
        return Err(CollisionError::InvalidContact);
    }
    let Some(preconditioner) = preconditioner else {
        for index in 0..residual.len() {
            output[index] = residual[index] / diagonal[index];
        }
        return Ok(());
    };
    if preconditioner.diagonal.len() != residual.len() {
        return Err(CollisionError::InvalidContact);
    }
    for row in 0..residual.len() {
        let mut value = residual[row];
        for &(column, factor) in &preconditioner.lower_rows[row] {
            value -= factor * output[column];
        }
        output[row] = value / preconditioner.diagonal[row];
        if !output[row].is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
    }
    for row in (0..residual.len()).rev() {
        let mut value = output[row];
        for &(lower_row, factor) in &preconditioner.lower_columns[row] {
            value -= factor * output[lower_row];
        }
        output[row] = value / preconditioner.diagonal[row];
        if !output[row].is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn global_velocity_matrix_vector(
    rows: &[ConstraintRow],
    vector: &[f64],
    masses: &[SolverMass],
    body_impulses: &mut [GeneralizedImpulse],
    body_responses: &mut [GeneralizedImpulse],
    result: &mut [f64],
) -> Result<(), CollisionError> {
    if rows.len() != vector.len() || rows.len() != result.len() {
        return Err(CollisionError::InvalidContact);
    }
    body_impulses.fill(GeneralizedImpulse::default());
    body_responses.fill(GeneralizedImpulse::default());
    for (row, coefficient) in rows.iter().copied().zip(vector.iter().copied()) {
        add_generalized_impulse(
            body_impulses
                .get_mut(row.body_a)
                .ok_or(CollisionError::InvalidContact)?,
            row.linear_a,
            row.angular_a,
            coefficient,
        )?;
        add_generalized_impulse(
            body_impulses
                .get_mut(row.body_b)
                .ok_or(CollisionError::InvalidContact)?,
            row.linear_b,
            row.angular_b,
            coefficient,
        )?;
    }
    for index in 0..masses.len() {
        let mass = masses[index];
        let impulse = body_impulses[index];
        body_responses[index] = GeneralizedImpulse {
            linear: impulse.linear * mass.inverse_mass,
            angular: mass
                .inverse_inertia
                .checked_mul_vector(impulse.angular)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        };
        if !body_responses[index].linear.is_finite() || !body_responses[index].angular.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
    }
    for (index, row) in rows.iter().copied().enumerate() {
        let response_a = body_responses
            .get(row.body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let response_b = body_responses
            .get(row.body_b)
            .ok_or(CollisionError::InvalidContact)?;
        let value = row
            .linear_a
            .checked_dot(response_a.linear)
            .ok_or(CollisionError::ArithmeticOverflow)?
            + row
                .angular_a
                .checked_dot(response_a.angular)
                .ok_or(CollisionError::ArithmeticOverflow)?
            + row
                .linear_b
                .checked_dot(response_b.linear)
                .ok_or(CollisionError::ArithmeticOverflow)?
            + row
                .angular_b
                .checked_dot(response_b.angular)
                .ok_or(CollisionError::ArithmeticOverflow)?;
        if !value.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        result[index] = value;
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn global_velocity_impulse(
    rows: &[ConstraintRow],
    vector: &[f64],
    body_impulses: &mut [GeneralizedImpulse],
) -> Result<(), CollisionError> {
    if rows.len() != vector.len() {
        return Err(CollisionError::InvalidContact);
    }
    body_impulses.fill(GeneralizedImpulse::default());
    for (row, coefficient) in rows.iter().copied().zip(vector.iter().copied()) {
        add_generalized_impulse(
            body_impulses
                .get_mut(row.body_a)
                .ok_or(CollisionError::InvalidContact)?,
            row.linear_a,
            row.angular_a,
            coefficient,
        )?;
        add_generalized_impulse(
            body_impulses
                .get_mut(row.body_b)
                .ok_or(CollisionError::InvalidContact)?,
            row.linear_b,
            row.angular_b,
            coefficient,
        )?;
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
#[allow(clippy::too_many_arguments)]
fn global_velocity_matrix_vector_parallel(
    rows: &[ConstraintRow],
    vector: &[f64],
    masses: &[SolverMass],
    row_terms_by_body: &[Vec<VelocityRowTerm>],
    body_impulses: &mut [GeneralizedImpulse],
    body_responses: &mut [GeneralizedImpulse],
    result: &mut [f64],
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if rows.len() != vector.len()
        || rows.len() != result.len()
        || masses.len() != row_terms_by_body.len()
        || masses.len() != body_impulses.len()
        || masses.len() != body_responses.len()
    {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(masses.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(body_impulses, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, impulse) in chunk.iter_mut().enumerate() {
            let mut sum = GeneralizedImpulse::default();
            for term in &row_terms_by_body[start + offset] {
                let coefficient = vector[term.row];
                sum.linear += term.linear * coefficient;
                sum.angular += term.angular * coefficient;
            }
            *impulse = sum;
        }
    })?;
    if body_impulses
        .iter()
        .any(|impulse| !impulse.linear.is_finite() || !impulse.angular.is_finite())
    {
        return Err(CollisionError::ArithmeticOverflow);
    }

    velocity_pcg_for_each_chunk_mut(
        body_responses,
        chunk_size,
        executor,
        |chunk_index, chunk| {
            let start = chunk_index * chunk_size;
            for (offset, response) in chunk.iter_mut().enumerate() {
                let index = start + offset;
                let mass = masses[index];
                let impulse = body_impulses[index];
                *response = GeneralizedImpulse {
                    linear: impulse.linear * mass.inverse_mass,
                    angular: mass
                        .inverse_inertia
                        .checked_mul_vector(impulse.angular)
                        .unwrap_or(Vec3::new(f64::NAN, f64::NAN, f64::NAN)),
                };
            }
        },
    )?;
    if body_responses
        .iter()
        .any(|response| !response.linear.is_finite() || !response.angular.is_finite())
    {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let chunk_size = velocity_pcg_chunk_size(rows.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(result, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, value) in chunk.iter_mut().enumerate() {
            let row = rows[start + offset];
            let response_a = body_responses[row.body_a];
            let response_b = body_responses[row.body_b];
            *value = row
                .linear_a
                .checked_dot(response_a.linear)
                .unwrap_or(f64::NAN)
                + row
                    .angular_a
                    .checked_dot(response_a.angular)
                    .unwrap_or(f64::NAN)
                + row
                    .linear_b
                    .checked_dot(response_b.linear)
                    .unwrap_or(f64::NAN)
                + row
                    .angular_b
                    .checked_dot(response_b.angular)
                    .unwrap_or(f64::NAN);
        }
    })?;
    if result.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn global_velocity_impulse_parallel(
    vector: &[f64],
    row_terms_by_body: &[Vec<VelocityRowTerm>],
    body_impulses: &mut [GeneralizedImpulse],
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if body_impulses.len() != row_terms_by_body.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(body_impulses.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(body_impulses, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, impulse) in chunk.iter_mut().enumerate() {
            let mut sum = GeneralizedImpulse::default();
            for term in &row_terms_by_body[start + offset] {
                let coefficient = vector[term.row];
                sum.linear += term.linear * coefficient;
                sum.angular += term.angular * coefficient;
            }
            *impulse = sum;
        }
    })?;
    if body_impulses
        .iter()
        .any(|impulse| !impulse.linear.is_finite() || !impulse.angular.is_finite())
    {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
const PCG_MIN_PARALLEL_CHUNK: usize = 256;

#[cfg(feature = "bench-internals")]
fn velocity_pcg_chunk_size(length: usize, worker_count: usize) -> usize {
    if worker_count <= 1 || length < PCG_MIN_PARALLEL_CHUNK {
        return length.max(1);
    }
    let task_count = worker_count.min((length / PCG_MIN_PARALLEL_CHUNK).max(1));
    length.div_ceil(task_count)
}

#[cfg(feature = "bench-internals")]
fn velocity_pcg_for_each_chunk_mut<T, F>(
    values: &mut [T],
    chunk_size: usize,
    executor: &impl StepExecutor,
    operation: F,
) -> Result<(), CollisionError>
where
    T: Send,
    F: Fn(usize, &mut [T]) + Send + Sync,
{
    if values.is_empty() {
        return Ok(());
    }
    if chunk_size >= values.len() {
        operation(0, values);
        return Ok(());
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        executor.for_each_chunk_mut(values, chunk_size, operation)
    }));
    if matches!(result, Ok(Ok(()))) {
        Ok(())
    } else {
        Err(CollisionError::WorkerFailed)
    }
}

#[cfg(feature = "bench-internals")]
fn velocity_pcg_scaled_add(
    destination: &mut [f64],
    source: &[f64],
    scale: f64,
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if destination.len() != source.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(destination.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(destination, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, value) in chunk.iter_mut().enumerate() {
            *value += scale * source[start + offset];
        }
    })?;
    if destination.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn velocity_pcg_update_direction(
    direction: &mut [f64],
    preconditioned: &[f64],
    beta: f64,
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if direction.len() != preconditioned.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(direction.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(direction, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, value) in chunk.iter_mut().enumerate() {
            *value = preconditioned[start + offset] + beta * *value;
        }
    })?;
    if direction.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn add_generalized_impulse(
    impulse: &mut GeneralizedImpulse,
    linear: Vec3,
    angular: Vec3,
    coefficient: f64,
) -> Result<(), CollisionError> {
    impulse.linear += linear * coefficient;
    impulse.angular += angular * coefficient;
    if impulse.linear.is_finite() && impulse.angular.is_finite() {
        Ok(())
    } else {
        Err(CollisionError::ArithmeticOverflow)
    }
}

#[cfg(feature = "bench-internals")]
fn velocity_vector_converged(
    rows: &[ConstraintRow],
    residual: &[f64],
    tolerance: super::JointTolerance,
) -> bool {
    rows.iter()
        .zip(residual)
        .all(|(row, value)| value.abs() <= velocity_row_tolerance(*row, tolerance))
}

#[cfg(feature = "bench-internals")]
fn velocity_vector_converged_parallel(
    rows: &[ConstraintRow],
    residual: &[f64],
    partial_results: &mut Vec<f64>,
    tolerance: super::JointTolerance,
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<bool, CollisionError> {
    if rows.len() != residual.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(rows.len(), worker_count);
    if chunk_size >= rows.len().max(1) {
        return Ok(velocity_vector_converged(rows, residual, tolerance));
    }
    let task_count = rows.len().div_ceil(chunk_size);
    resize_f64_buffer(partial_results, task_count)?;
    velocity_pcg_for_each_chunk_mut(partial_results, 1, executor, |task_index, output| {
        let start = task_index * chunk_size;
        let end = (start + chunk_size).min(rows.len());
        output[0] = if rows[start..end]
            .iter()
            .zip(&residual[start..end])
            .all(|(row, value)| value.abs() <= velocity_row_tolerance(*row, tolerance))
        {
            1.0
        } else {
            0.0
        };
    })?;
    Ok(partial_results[..task_count]
        .iter()
        .all(|result| *result == 1.0))
}

#[cfg(feature = "bench-internals")]
fn velocity_pcg_result(
    rows: &[ConstraintRow],
    residual: &[f64],
    iterations: usize,
    converged: bool,
) -> VelocityPcgResult {
    let mut result = VelocityPcgResult {
        iterations,
        converged,
        ..VelocityPcgResult::default()
    };
    for (row, residual) in rows.iter().zip(residual) {
        match row.dimension {
            RowDimension::Linear => {
                result.linear_residual_mps = result.linear_residual_mps.max(residual.abs())
            }
            RowDimension::Angular => {
                result.angular_residual_radps = result.angular_residual_radps.max(residual.abs())
            }
        }
    }
    result
}

#[cfg(feature = "bench-internals")]
fn position_pcg_result(
    rows: &[ConstraintRow],
    residual: &[f64],
    iterations: usize,
    converged: bool,
) -> PositionPcgResult {
    let mut result = PositionPcgResult {
        iterations,
        converged,
        ..PositionPcgResult::default()
    };
    for (row, residual) in rows.iter().zip(residual) {
        match row.dimension {
            RowDimension::Linear => {
                result.linear_residual_m = result.linear_residual_m.max(residual.abs())
            }
            RowDimension::Angular => {
                result.angular_residual_rad = result.angular_residual_rad.max(residual.abs())
            }
        }
    }
    result
}

#[cfg(feature = "bench-internals")]
fn prepare_velocity_residual_parallel(
    rows: &[ConstraintRow],
    bodies: &[SolverBody],
    residual: &mut [f64],
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if rows.len() != residual.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(rows.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(residual, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, value) in chunk.iter_mut().enumerate() {
            let row = rows[start + offset];
            *value = row.target_velocity - row_velocity(bodies, row).unwrap_or(f64::NAN);
        }
    })?;
    if residual.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn velocity_row_tolerance(row: ConstraintRow, tolerance: super::JointTolerance) -> f64 {
    match row.dimension {
        RowDimension::Linear => tolerance.velocity_mps,
        RowDimension::Angular => tolerance.angular_velocity_radps,
    }
}

#[cfg(feature = "bench-internals")]
fn position_vector_converged(
    rows: &[ConstraintRow],
    residual: &[f64],
    tolerance: super::JointTolerance,
) -> bool {
    rows.iter()
        .zip(residual)
        .all(|(row, value)| value.abs() <= position_row_tolerance(*row, tolerance))
}

#[cfg(feature = "bench-internals")]
fn position_row_tolerance(row: ConstraintRow, tolerance: super::JointTolerance) -> f64 {
    match row.dimension {
        RowDimension::Linear => tolerance.linear_m,
        RowDimension::Angular => tolerance.angular_rad,
    }
}

#[cfg(feature = "bench-internals")]
fn checked_dot_product(left: &[f64], right: &[f64]) -> Result<f64, CollisionError> {
    if left.len() != right.len() {
        return Err(CollisionError::InvalidContact);
    }
    left.iter().zip(right).try_fold(0.0, |sum, (left, right)| {
        let next = sum + left * right;
        next.is_finite()
            .then_some(next)
            .ok_or(CollisionError::ArithmeticOverflow)
    })
}

#[cfg(feature = "bench-internals")]
fn checked_dot_product_parallel(
    left: &[f64],
    right: &[f64],
    partial_sums: &mut Vec<f64>,
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<f64, CollisionError> {
    if left.len() != right.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(left.len(), worker_count);
    if chunk_size >= left.len().max(1) {
        return checked_dot_product(left, right);
    }
    let task_count = left.len().div_ceil(chunk_size);
    resize_f64_buffer(partial_sums, task_count)?;
    velocity_pcg_for_each_chunk_mut(partial_sums, 1, executor, |task_index, output| {
        let start = task_index * chunk_size;
        let end = (start + chunk_size).min(left.len());
        let mut sum = 0.0;
        for index in start..end {
            sum += left[index] * right[index];
        }
        output[0] = sum;
    })?;
    partial_sums[..task_count]
        .iter()
        .try_fold(0.0, |sum, partial| {
            let next = sum + partial;
            next.is_finite()
                .then_some(next)
                .ok_or(CollisionError::ArithmeticOverflow)
        })
}

#[cfg(feature = "bench-internals")]
fn zero_f64_buffer(length: usize) -> Result<Vec<f64>, CollisionError> {
    let mut buffer = Vec::new();
    buffer
        .try_reserve_exact(length)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    buffer.resize(length, 0.0);
    Ok(buffer)
}

#[cfg(feature = "bench-internals")]
fn zero_impulse_buffer(length: usize) -> Result<Vec<GeneralizedImpulse>, CollisionError> {
    let mut buffer = Vec::new();
    buffer
        .try_reserve_exact(length)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    buffer.resize(length, GeneralizedImpulse::default());
    Ok(buffer)
}

#[cfg(feature = "bench-internals")]
fn resize_f64_buffer(buffer: &mut Vec<f64>, length: usize) -> Result<(), CollisionError> {
    if length > buffer.capacity() {
        buffer
            .try_reserve_exact(length - buffer.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    buffer.resize(length, 0.0);
    buffer.fill(0.0);
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn resize_impulse_buffer(
    buffer: &mut Vec<GeneralizedImpulse>,
    length: usize,
) -> Result<(), CollisionError> {
    if length > buffer.capacity() {
        buffer
            .try_reserve_exact(length - buffer.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    buffer.resize(length, GeneralizedImpulse::default());
    buffer.fill(GeneralizedImpulse::default());
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn clear_mass_buffer(buffer: &mut Vec<SolverMass>, length: usize) -> Result<(), CollisionError> {
    buffer.clear();
    if length > buffer.capacity() {
        buffer
            .try_reserve_exact(length)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn resize_nested_buffer<T>(buffer: &mut Vec<Vec<T>>, length: usize) -> Result<(), CollisionError> {
    if length > buffer.len() {
        buffer
            .try_reserve_exact(length - buffer.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        buffer.resize_with(length, Vec::new);
    } else {
        buffer.truncate(length);
    }
    for entries in buffer {
        entries.clear();
    }
    Ok(())
}

fn update_position_residual(
    residual: &mut JointResidual,
    dimension: RowDimension,
    value: f64,
) -> Result<(), CollisionError> {
    if !value.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    match dimension {
        RowDimension::Linear => residual.max_linear_m = residual.max_linear_m.max(value),
        RowDimension::Angular => residual.max_angular_rad = residual.max_angular_rad.max(value),
    }
    Ok(())
}

fn update_velocity_residual(
    residual: &mut JointResidual,
    dimension: RowDimension,
    value: f64,
) -> Result<(), CollisionError> {
    if !value.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    match dimension {
        RowDimension::Linear => residual.max_velocity_mps = residual.max_velocity_mps.max(value),
        RowDimension::Angular => {
            residual.max_angular_velocity_radps = residual.max_angular_velocity_radps.max(value)
        }
    }
    Ok(())
}

pub(crate) fn prepare_joint_rows(
    bodies: &[SolverBody],
    workspace: &mut JointWorkspace,
    joints: &[JointConstraint],
    dt: f64,
) -> Result<(), CollisionError> {
    for (joint_index, constraint) in joints.iter().enumerate() {
        workspace.set_joint_index(joint_index, constraint.id);
        prepare_joint_rows_for_constraint(bodies, workspace, *constraint, dt)?;
    }
    Ok(())
}

fn prepare_joint_rows_for_constraint<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    constraint: JointConstraint,
    dt: f64,
) -> Result<(), CollisionError> {
    if !constraint.joint.common().enabled {
        return Ok(());
    }
    #[cfg(feature = "bench-internals")]
    let preparation_started = std::time::Instant::now();
    solver_bodies_at(bodies, constraint.body_a, constraint.body_b)?;
    let result = match constraint.joint {
        super::Joint::Fixed(joint) => append_fixed_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::Ball(joint) => append_ball_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::Distance(joint) => append_distance_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::Hinge(joint) => append_hinge_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::Slider(joint) => append_slider_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::Cylindrical(joint) => append_cylindrical_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::Planar(joint) => append_planar_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::D6(joint) => append_d6_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::Spring(joint) => append_spring_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
        super::Joint::ConeTwist(joint) => append_cone_twist_rows(
            bodies,
            workspace,
            constraint.body_a,
            constraint.body_b,
            joint,
            dt,
        ),
    };
    #[cfg(feature = "bench-internals")]
    workspace.record_joint_row_preparation_time(preparation_started.elapsed());
    result
}

fn append_fixed_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::FixedJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let geometry = fixed_joint_geometry(
        bodies,
        workspace,
        body_a,
        body_b,
        joint.common.frame_a,
        joint.common.frame_b,
    )?;
    let mut settings = row_settings(body_a, body_b, joint.common, dt);
    // Fixed-joint position projection already corrects connector drift. A
    // Baumgarte velocity target on the same error can add energy to large trees.
    settings.velocity_bias = false;
    for row in fixed_anchor_rows(bodies, settings, geometry.anchor)?.into_iter() {
        workspace.push(row);
    }
    for row in fixed_frame_rows(settings, geometry.frame)?.into_iter() {
        workspace.push(row);
    }
    Ok(())
}

fn fixed_joint_geometry<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    frame_a: super::JointFrame,
    frame_b: super::JointFrame,
) -> Result<FixedJointGeometry, CollisionError> {
    let transform_a = bodies
        .get(body_a)
        .ok_or(CollisionError::InvalidContact)?
        .state()
        .transform();
    let transform_b = bodies
        .get(body_b)
        .ok_or(CollisionError::InvalidContact)?
        .state()
        .transform();
    let joint_index = workspace.active_joint_index();
    if let Some(cached) = workspace.fixed_geometry(joint_index)
        && cached.frame_a == frame_a
        && cached.frame_b == frame_b
        && cached.transform_a == transform_a
        && cached.transform_b == transform_b
    {
        return Ok(cached.geometry);
    }

    let (anchor_a, rotation_a) = world_frame(bodies, body_a, frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, frame_b)?;
    let position_error = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let rotation_error = relative_rotation_error(rotation_a, rotation_b)?;
    let geometry = FixedJointGeometry {
        anchor: FixedAnchorGeometry {
            anchor_a,
            anchor_b,
            position_error,
        },
        frame: FixedFrameGeometry {
            rotation_a,
            rotation_error,
        },
    };
    if !workspace.diagnostic_prepare() {
        workspace.set_fixed_geometry(
            joint_index,
            FixedJointGeometryCache {
                frame_a,
                frame_b,
                transform_a,
                transform_b,
                geometry,
            },
        )?;
    }
    Ok(geometry)
}

fn fixed_anchor_geometry<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    settings: RowSettings,
    frames: [super::JointFrame; 2],
) -> Result<FixedAnchorGeometry, CollisionError> {
    let anchor = |index, frame: super::JointFrame| {
        Transform::new(frame.local_anchor, frame.local_rotation)
            .map_err(|_| CollisionError::InvalidContact)?;
        let point = bodies
            .get(index)
            .ok_or(CollisionError::InvalidContact)?
            .state()
            .transform()
            .transform_point(frame.local_anchor);
        if !point.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(point)
    };
    let anchor_a = anchor(settings.body_a, frames[0])?;
    let anchor_b = anchor(settings.body_b, frames[1])?;
    let position_error = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    Ok(FixedAnchorGeometry {
        anchor_a,
        anchor_b,
        position_error,
    })
}

fn fixed_frame_geometry<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    settings: RowSettings,
    frames: [super::JointFrame; 2],
) -> Result<FixedFrameGeometry, CollisionError> {
    let rotation = |index, frame: super::JointFrame| {
        Transform::new(frame.local_anchor, frame.local_rotation)
            .map_err(|_| CollisionError::InvalidContact)?;
        let body = bodies.get(index).ok_or(CollisionError::InvalidContact)?;
        (body.state().transform().rotation() * frame.local_rotation)
            .try_normalize()
            .filter(|rotation| rotation.is_normalized())
            .ok_or(CollisionError::ArithmeticOverflow)
    };
    let rotation_a = rotation(settings.body_a, frames[0])?;
    let rotation_b = rotation(settings.body_b, frames[1])?;
    let rotation_error = relative_rotation_error(rotation_a, rotation_b)?;
    Ok(FixedFrameGeometry {
        rotation_a,
        rotation_error,
    })
}

fn fixed_anchor_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    settings: RowSettings,
    geometry: FixedAnchorGeometry,
) -> Result<[ConstraintRow; 3], CollisionError> {
    // These rows share centers and anchors, so resolve them once per block.
    let centers = [
        center_of_mass(
            bodies
                .get(settings.body_a)
                .ok_or(CollisionError::InvalidContact)?,
        )?,
        center_of_mass(
            bodies
                .get(settings.body_b)
                .ok_or(CollisionError::InvalidContact)?,
        )?,
    ];
    fixed_anchor_rows_with_centers(settings, geometry, centers)
}

fn fixed_anchor_rows_with_centers(
    settings: RowSettings,
    geometry: FixedAnchorGeometry,
    centers: [Vec3; 2],
) -> Result<[ConstraintRow; 3], CollisionError> {
    let anchors = [geometry.anchor_a, geometry.anchor_b];
    Ok([
        make_linear_row(
            settings,
            anchors,
            Vec3::X,
            anchors[0],
            geometry
                .position_error
                .checked_dot(Vec3::X)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            0,
            bilateral_bounds(),
            centers,
        )?,
        make_linear_row(
            settings,
            anchors,
            Vec3::Y,
            anchors[0],
            geometry
                .position_error
                .checked_dot(Vec3::Y)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            1,
            bilateral_bounds(),
            centers,
        )?,
        make_linear_row(
            settings,
            anchors,
            Vec3::Z,
            anchors[0],
            geometry
                .position_error
                .checked_dot(Vec3::Z)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            2,
            bilateral_bounds(),
            centers,
        )?,
    ])
}

fn fixed_frame_rows(
    settings: RowSettings,
    geometry: FixedFrameGeometry,
) -> Result<[ConstraintRow; 3], CollisionError> {
    // The log-rotation derivative coefficient depends only on the rotation
    // error, so compute it once for all three axes.
    let coefficient = fixed_rotation_coefficient(geometry.rotation_error)?;
    let rotation_a = geometry.rotation_a;
    let rotation_error = geometry.rotation_error;
    Ok([
        make_angular_row(
            settings,
            fixed_rotation_row_axis(rotation_a, rotation_error, Vec3::X, coefficient)?,
            rotation_error
                .checked_dot(Vec3::X)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            3,
            bilateral_bounds(),
        )?,
        make_angular_row(
            settings,
            fixed_rotation_row_axis(rotation_a, rotation_error, Vec3::Y, coefficient)?,
            rotation_error
                .checked_dot(Vec3::Y)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            4,
            bilateral_bounds(),
        )?,
        make_angular_row(
            settings,
            fixed_rotation_row_axis(rotation_a, rotation_error, Vec3::Z, coefficient)?,
            rotation_error
                .checked_dot(Vec3::Z)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            5,
            bilateral_bounds(),
        )?,
    ])
}

fn refresh_fixed_joint_row_block<B: SolverBodyAccess + ?Sized>(
    workspace: &mut JointWorkspace,
    bodies: &B,
    constraint: JointConstraint,
    dt: f64,
    block: FixedJointRowBlock,
) -> Result<(), CollisionError> {
    let joint = match constraint.joint {
        super::Joint::Fixed(joint) => joint,
        _ => return Err(CollisionError::InvalidContact),
    };
    #[cfg(feature = "bench-internals")]
    let preparation_started = std::time::Instant::now();
    let geometry = fixed_joint_geometry(
        bodies,
        workspace,
        constraint.body_a,
        constraint.body_b,
        joint.common.frame_a,
        joint.common.frame_b,
    )?;
    let mut settings = row_settings(constraint.body_a, constraint.body_b, joint.common, dt);
    settings.velocity_bias = false;
    let replacements = match block {
        FixedJointRowBlock::Anchor => fixed_anchor_rows(bodies, settings, geometry.anchor)?,
        FixedJointRowBlock::Frame => fixed_frame_rows(settings, geometry.frame)?,
    };
    {
        let (anchor_rows, frame_rows, _) = workspace.preparing_fixed_row_blocks_mut()?;
        let rows = match block {
            FixedJointRowBlock::Anchor => anchor_rows,
            FixedJointRowBlock::Frame => frame_rows,
        };
        write_fixed_rows(rows, replacements)?;
    }
    #[cfg(feature = "bench-internals")]
    {
        workspace.record_incremental_rows_prepared(3);
        workspace.record_joint_row_preparation_time(preparation_started.elapsed());
    }
    Ok(())
}

/// Overwrites a three-row block, keeping the accumulated projection state.
fn write_fixed_rows(
    rows: &mut [ConstraintRow],
    replacements: [ConstraintRow; 3],
) -> Result<(), CollisionError> {
    if rows.len() != replacements.len() {
        return Err(CollisionError::InvalidContact);
    }
    for (row, mut replacement) in rows.iter_mut().zip(replacements) {
        if row.key != replacement.key {
            return Err(CollisionError::InvalidContact);
        }
        replacement.accumulated_multiplier = row.accumulated_multiplier;
        replacement.accumulated_impulse = row.accumulated_impulse;
        *row = replacement;
    }
    Ok(())
}

fn append_ball_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::BallJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let position_error = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    // The ball anchor is a hard bilateral lock; its swing/twist limits are
    // unilateral and handled separately below.
    let lock = lock_settings(settings);
    for (axis_index, axis) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
        let error = position_error
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_linear_row(
            bodies,
            workspace,
            lock,
            [anchor_a, anchor_b],
            axis,
            anchor_b, // Apply equal/opposite velocity impulses at one world point.
            error,
            axis_index as u8,
            bilateral_bounds(),
        )?;
    }
    append_ball_angular_limits(bodies, workspace, settings, rotation_a, rotation_b, joint)?;
    Ok(())
}

fn append_ball_angular_limits<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    rotation_a: Quat,
    rotation_b: Quat,
    joint: super::BallJoint,
) -> Result<(), CollisionError> {
    let relative = canonical_relative_rotation(rotation_a, rotation_b)?;
    if let Some(limit) = joint.swing_limit {
        let axis_a = rotation_a
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let axis_b = rotation_b
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let cosine = axis_a
            .checked_dot(axis_b)
            .ok_or(CollisionError::ArithmeticOverflow)?
            .clamp(-1.0, 1.0);
        let angle = cosine.acos();
        // A swing upper bound is unilateral. Derive its direction from the
        // relative orientation; when the swing is degenerate (zero or
        // antiparallel) fall back to the projected relative angular velocity
        // and then to a deterministic tangent so the row stays finite.
        if let Some(axis) = ball_swing_axis(bodies, settings, axis_a, axis_b, cosine)? {
            append_angular_axis_limit(
                bodies,
                workspace,
                settings,
                axis,
                angle,
                super::JointLimit::new(f64::NEG_INFINITY, limit),
                3,
            )?;
        }
    }
    if let (Some(limit), Some(twist)) = (joint.twist_limit, twist_angle(relative)?) {
        // The twist coordinate is undefined at the swing singularity; a
        // well-defined `twist` avoids the ±π wrap ambiguity because the limit is
        // expressed in the same `(-π, π]` domain returned by `twist_angle`.
        // Route through the shared angular limit so the row both reacts to an
        // existing violation and brakes a predicted crossing within the step,
        // while the collapsed interval still collapses to a bilateral lock.
        let axis = rotation_a
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        append_angular_axis_limit(bodies, workspace, settings, axis, twist, limit, 4)?;
    }
    Ok(())
}

fn append_distance_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::DistanceJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, _) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let delta = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let distance = delta
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let axis = if distance > f64::MIN_POSITIVE {
        delta / distance
    } else {
        rotation_a
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?
    };
    // Route the distance interval through the shared unilateral limit so that
    // the row activates both on an existing violation and when the radial
    // velocity is predicted to cross a boundary this step. The sign is chosen
    // so an upper bound never pushes the bodies together and a lower bound
    // never pulls them apart.
    append_linear_limit(
        bodies,
        workspace,
        settings,
        [anchor_a, anchor_b],
        axis,
        anchor_a, // The axis follows the current delta, not a body frame.
        distance,
        joint.distance_limit,
        0,
    )
}

fn append_hinge_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::HingeJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    append_anchor_lock(
        bodies, workspace, settings, anchor_a, anchor_b, rotation_a, 0,
    )?;
    append_axis_alignment(workspace, settings, rotation_a, rotation_b, 3)?;

    if let Some(angle) = twist_angle(canonical_relative_rotation(rotation_a, rotation_b)?)? {
        if let Some(settings) = drive_settings(settings, joint.drive)? {
            push_angular_row(
                workspace,
                settings,
                rotation_a
                    .rotate_vector(Vec3::Z)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
                angle - joint.drive.target_position,
                5,
                bilateral_bounds(),
            )?;
        }
        if let Some(limit) = joint.angle_limit {
            append_angular_limit(bodies, workspace, settings, rotation_a, angle, limit, 6)?;
        }
    }
    Ok(())
}

fn append_slider_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::SliderJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    append_linear_axes(
        bodies,
        workspace,
        settings,
        anchor_a,
        anchor_b,
        &perpendicular_basis(
            rotation_a
                .rotate_vector(Vec3::Z)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )?,
        0,
    )?;
    append_rotation_lock(workspace, settings, rotation_a, rotation_b, 2)?;

    let axis = rotation_a
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let coordinate = anchor_b
        .checked_sub(anchor_a)
        .and_then(|delta| delta.checked_dot(axis))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if let Some(drive) = drive_settings(settings, joint.drive)? {
        push_linear_row(
            bodies,
            workspace,
            drive,
            [anchor_a, anchor_b],
            axis,
            anchor_b,
            coordinate - joint.drive.target_position,
            5,
            bilateral_bounds(),
        )?;
    }
    if let Some(limit) = joint.translation_limit {
        append_linear_limit(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            anchor_b,
            coordinate,
            limit,
            6,
        )?;
    }
    Ok(())
}

fn append_cylindrical_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::CylindricalJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let axis = rotation_a
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    append_linear_axes(
        bodies,
        workspace,
        settings,
        anchor_a,
        anchor_b,
        &perpendicular_basis(axis)?,
        0,
    )?;
    append_axis_alignment(workspace, settings, rotation_a, rotation_b, 2)?;

    let coordinate = anchor_b
        .checked_sub(anchor_a)
        .and_then(|delta| delta.checked_dot(axis))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if let Some(drive) = drive_settings(settings, joint.translation_drive)? {
        push_linear_row(
            bodies,
            workspace,
            drive,
            [anchor_a, anchor_b],
            axis,
            anchor_b,
            coordinate - joint.translation_drive.target_position,
            4,
            bilateral_bounds(),
        )?;
    }
    if let Some(limit) = joint.translation_limit {
        append_linear_limit(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            anchor_b,
            coordinate,
            limit,
            5,
        )?;
    }

    if let Some(angle) = twist_angle(canonical_relative_rotation(rotation_a, rotation_b)?)? {
        if let Some(drive) = drive_settings(settings, joint.angular_drive)? {
            push_angular_row(
                workspace,
                drive,
                axis,
                angle - joint.angular_drive.target_position,
                6,
                bilateral_bounds(),
            )?;
        }
        if let Some(limit) = joint.angle_limit {
            append_angular_limit(bodies, workspace, settings, rotation_a, angle, limit, 7)?;
        }
    }
    Ok(())
}

fn append_planar_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::PlanarJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let axes = [Vec3::X, Vec3::Y, Vec3::Z].map(|axis| rotation_a.rotate_vector(axis));
    let axes = [
        axes[0].ok_or(CollisionError::ArithmeticOverflow)?,
        axes[1].ok_or(CollisionError::ArithmeticOverflow)?,
        axes[2].ok_or(CollisionError::ArithmeticOverflow)?,
    ];
    let delta = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    push_linear_row(
        bodies,
        workspace,
        settings,
        [anchor_a, anchor_b],
        axes[2],
        anchor_b,
        delta
            .checked_dot(axes[2])
            .ok_or(CollisionError::ArithmeticOverflow)?,
        0,
        bilateral_bounds(),
    )?;
    append_axis_alignment(workspace, settings, rotation_a, rotation_b, 1)?;

    for (axis_index, axis) in axes.iter().copied().take(2).enumerate() {
        let coordinate = delta
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        if let Some(drive) = drive_settings(settings, joint.translation_drives[axis_index])? {
            push_linear_row(
                bodies,
                workspace,
                drive,
                [anchor_a, anchor_b],
                axis,
                anchor_b,
                coordinate - joint.translation_drives[axis_index].target_position,
                3 + axis_index as u8,
                bilateral_bounds(),
            )?;
        }
        if let Some(limit) = joint.translation_limits[axis_index] {
            append_linear_limit(
                bodies,
                workspace,
                settings,
                [anchor_a, anchor_b],
                axis,
                anchor_b,
                coordinate,
                limit,
                5 + axis_index as u8,
            )?;
        }
    }

    let relative = canonical_relative_rotation(rotation_a, rotation_b)?;
    let rotation_error = relative_rotation_error(rotation_a, rotation_b)?;
    if let Some(drive) = drive_settings(settings, joint.rotation_drive)? {
        push_angular_row(
            workspace,
            drive,
            axes[2],
            rotation_error.z - joint.rotation_drive.target_position,
            7,
            bilateral_bounds(),
        )?;
    }
    if let (Some(limit), Some(angle)) = (joint.rotation_limit, twist_angle(relative)?) {
        append_angular_axis_limit(bodies, workspace, settings, axes[2], angle, limit, 8)?;
    }
    Ok(())
}

fn append_spring_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::SpringJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, _) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let delta = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let distance = delta
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let axis = match delta.try_normalize() {
        Some(axis) => axis,
        None => rotation_a
            .rotate_vector(Vec3::X)
            .ok_or(CollisionError::ArithmeticOverflow)?,
    };
    let settings = row_settings(body_a, body_b, joint.common, dt);
    if joint.stiffness > 0.0 || joint.spring_damping > 0.0 {
        let mut spring_settings = settings;
        spring_settings.common.compliance = 0.0;
        spring_settings.force_bias = joint.stiffness * (distance - joint.rest_length);
        if !spring_settings.force_bias.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        spring_settings.common.damping = joint.spring_damping;
        spring_settings.force_driven = true;
        spring_settings.position_enabled = false;
        spring_settings.velocity_bias = false;
        push_linear_row(
            bodies,
            workspace,
            spring_settings,
            [anchor_a, anchor_b],
            axis,
            anchor_a, // The axis follows the current delta, not a body frame.
            distance - joint.rest_length,
            0,
            bilateral_bounds(),
        )?;
    }
    if let Some(limit) = joint.distance_limit {
        append_linear_limit(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            anchor_a,
            distance,
            limit,
            1,
        )?;
    }
    Ok(())
}

fn append_cone_twist_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::ConeTwistJoint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    append_anchor_lock(
        bodies, workspace, settings, anchor_a, anchor_b, rotation_a, 0,
    )?;
    let relative = canonical_relative_rotation(rotation_a, rotation_b)?;
    let twist_length = relative.z.hypot(relative.w);
    if !twist_length.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let twist = if twist_length > TWIST_SINGULAR_TOLERANCE {
        Quat::new(
            0.0,
            0.0,
            relative.z / twist_length,
            relative.w / twist_length,
        )
    } else {
        // With no Z/W projection the twist coordinate is undefined. Treat the
        // whole relative rotation as swing and skip the separate twist limit.
        Quat::IDENTITY
    };
    let swing = (relative * twist.conjugate())
        .try_normalize()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let swing_error = relative_rotation_error(Quat::IDENTITY, swing)?;
    let axes = [Vec3::X, Vec3::Y, Vec3::Z].map(|axis| rotation_a.rotate_vector(axis));
    let axes = [
        axes[0].ok_or(CollisionError::ArithmeticOverflow)?,
        axes[1].ok_or(CollisionError::ArithmeticOverflow)?,
        axes[2].ok_or(CollisionError::ArithmeticOverflow)?,
    ];
    let swing_radius = swing_error.x.hypot(swing_error.y);
    let (swing_x_limit, swing_y_limit) = (joint.swing_limit[0], joint.swing_limit[1]);
    if swing_x_limit > 0.0 && swing_y_limit > 0.0 {
        let angular_velocity = bodies
            .get(body_b)
            .ok_or(CollisionError::InvalidContact)?
            .state()
            .angular_velocity()
            .checked_sub(
                bodies
                    .get(body_a)
                    .ok_or(CollisionError::InvalidContact)?
                    .state()
                    .angular_velocity(),
            )
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let local_velocity = rotation_a
            .conjugate()
            .rotate_vector(angular_velocity)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let predicted = Vec3::new(
            swing_error.x + local_velocity.x * dt,
            swing_error.y + local_velocity.y * dt,
            0.0,
        );
        let direction_source = if swing_radius > f64::MIN_POSITIVE {
            Vec3::new(swing_error.x, swing_error.y, 0.0)
        } else {
            predicted
        };
        if let Some(direction) = direction_source.try_normalize() {
            let boundary = 1.0
                / ((direction.x / swing_x_limit).powi(2) + (direction.y / swing_y_limit).powi(2))
                    .sqrt();
            if !boundary.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            let axis = rotation_a
                .rotate_vector(direction)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            append_angular_axis_limit(
                bodies,
                workspace,
                settings,
                axis,
                swing_radius,
                super::JointLimit::new(0.0, boundary),
                6,
            )?;
        }
    } else {
        for (axis_index, ((limit, local_axis), axis)) in joint
            .swing_limit
            .into_iter()
            .zip([Vec3::X, Vec3::Y])
            .zip(axes.into_iter().take(2))
            .enumerate()
        {
            append_angular_axis_limit(
                bodies,
                workspace,
                settings,
                axis,
                swing_error
                    .checked_dot(local_axis)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
                super::JointLimit::new(-limit, limit),
                3 + axis_index as u8,
            )?;
        }
    }
    if let Some(angle) = twist_angle(relative)? {
        append_angular_axis_limit(
            bodies,
            workspace,
            settings,
            axes[2],
            angle,
            joint.twist_limit,
            5,
        )?;
    }
    Ok(())
}

fn append_d6_rows<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    body_a: usize,
    body_b: usize,
    joint: super::D6Joint,
    dt: f64,
) -> Result<(), CollisionError> {
    let (anchor_a, rotation_a) = world_frame(bodies, body_a, joint.common.frame_a)?;
    let (anchor_b, rotation_b) = world_frame(bodies, body_b, joint.common.frame_b)?;
    let settings = row_settings(body_a, body_b, joint.common, dt);
    let lock = lock_settings(settings);
    let delta = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let rotation_error = relative_rotation_error(rotation_a, rotation_b)?;

    for (axis_index, local_axis) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
        let axis = rotation_a
            .rotate_vector(local_axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let coordinate = delta
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        match joint.linear_motion[axis_index] {
            super::JointMotion::Locked => push_linear_row(
                bodies,
                workspace,
                lock,
                [anchor_a, anchor_b],
                axis,
                anchor_b,
                coordinate,
                axis_index as u8,
                bilateral_bounds(),
            )?,
            super::JointMotion::Limited(limit) => append_linear_limit(
                bodies,
                workspace,
                settings,
                [anchor_a, anchor_b],
                axis,
                anchor_b,
                coordinate,
                limit,
                3 + axis_index as u8,
            )?,
            super::JointMotion::Free => {}
        }

        if !matches!(joint.linear_motion[axis_index], super::JointMotion::Locked)
            && let Some(drive) = drive_settings(settings, joint.linear_drives[axis_index])?
        {
            push_linear_row(
                bodies,
                workspace,
                drive,
                [anchor_a, anchor_b],
                axis,
                anchor_b,
                coordinate - joint.linear_drives[axis_index].target_position,
                6 + axis_index as u8,
                bilateral_bounds(),
            )?;
        }

        let angular_axis = axis;
        let angular_coordinate = rotation_error
            .checked_dot(local_axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        match joint.angular_motion[axis_index] {
            super::JointMotion::Locked => push_angular_row(
                workspace,
                lock,
                angular_axis,
                angular_coordinate,
                9 + axis_index as u8,
                bilateral_bounds(),
            )?,
            super::JointMotion::Limited(limit) => append_angular_axis_limit(
                bodies,
                workspace,
                settings,
                angular_axis,
                angular_coordinate,
                limit,
                12 + axis_index as u8,
            )?,
            super::JointMotion::Free => {}
        }

        if !matches!(joint.angular_motion[axis_index], super::JointMotion::Locked)
            && let Some(drive) = drive_settings(settings, joint.angular_drives[axis_index])?
        {
            push_angular_row(
                workspace,
                drive,
                angular_axis,
                angular_coordinate - joint.angular_drives[axis_index].target_position,
                15 + axis_index as u8,
                bilateral_bounds(),
            )?;
        }
    }
    Ok(())
}

fn append_anchor_lock<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    anchor_a: Vec3,
    anchor_b: Vec3,
    frame_rotation: Quat,
    first_key: u8,
) -> Result<(), CollisionError> {
    let axes = [Vec3::X, Vec3::Y, Vec3::Z].map(|axis| frame_rotation.rotate_vector(axis));
    let axes = [
        axes[0].ok_or(CollisionError::ArithmeticOverflow)?,
        axes[1].ok_or(CollisionError::ArithmeticOverflow)?,
        axes[2].ok_or(CollisionError::ArithmeticOverflow)?,
    ];
    append_linear_axes(
        bodies, workspace, settings, anchor_a, anchor_b, &axes, first_key,
    )
}

fn append_linear_axes<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    anchor_a: Vec3,
    anchor_b: Vec3,
    axes: &[Vec3],
    first_key: u8,
) -> Result<(), CollisionError> {
    let settings = lock_settings(settings);
    let delta = anchor_b
        .checked_sub(anchor_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    for (axis_index, axis) in axes.iter().copied().enumerate() {
        let error = delta
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_linear_row(
            bodies,
            workspace,
            settings,
            [anchor_a, anchor_b],
            axis,
            // Frame-derived axes rotate with body A: attach the axis to body
            // B's anchor so the Jacobian carries the exact rotating-axis term.
            anchor_b,
            error,
            first_key + axis_index as u8,
            bilateral_bounds(),
        )?;
    }
    Ok(())
}

fn append_rotation_lock(
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    rotation_a: Quat,
    rotation_b: Quat,
    first_key: u8,
) -> Result<(), CollisionError> {
    let settings = lock_settings(settings);
    let error = relative_rotation_error(rotation_a, rotation_b)?;
    for (axis_index, local_axis) in [Vec3::X, Vec3::Y, Vec3::Z].into_iter().enumerate() {
        let axis = rotation_a
            .rotate_vector(local_axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        push_angular_row(
            workspace,
            settings,
            axis,
            error
                .checked_dot(local_axis)
                .ok_or(CollisionError::ArithmeticOverflow)?,
            first_key + axis_index as u8,
            bilateral_bounds(),
        )?;
    }
    Ok(())
}

/// Deterministic swing axis for a ball joint. When `axis_a` and `axis_b` are
/// not parallel the cross product gives the rotation axis. In the degenerate
/// cases the direction comes from the component of the relative angular
/// velocity perpendicular to `axis_a`, or from a fixed tangent basis when no
/// motion distinguishes a direction. Returns `None` only when there is no
/// swing and no outward angular velocity, so no constraint is needed.
fn ball_swing_axis<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    settings: RowSettings,
    axis_a: Vec3,
    axis_b: Vec3,
    cosine: f64,
) -> Result<Option<Vec3>, CollisionError> {
    let cross = axis_a
        .checked_cross(axis_b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if let Some(axis) = cross.try_normalize() {
        return Ok(Some(axis));
    }
    if cosine < 0.0 {
        // Antiparallel swing: the rotation axis is undefined, so pick a
        // deterministic perpendicular to keep the limit well posed.
        let (first, _) = constraint::tangent_basis(axis_a).ok_or(CollisionError::InvalidContact)?;
        return Ok(Some(first));
    }
    let relative_velocity = bodies
        .get(settings.body_b)
        .ok_or(CollisionError::InvalidContact)?
        .state()
        .angular_velocity()
        .checked_sub(
            bodies
                .get(settings.body_a)
                .ok_or(CollisionError::InvalidContact)?
                .state()
                .angular_velocity(),
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let perpendicular = relative_velocity
        .checked_sub(
            axis_a
                * relative_velocity
                    .checked_dot(axis_a)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if let Some(axis) = perpendicular.try_normalize() {
        return Ok(Some(axis));
    }
    // No swing and no defined swing motion: nothing to constrain.
    Ok(None)
}

fn append_axis_alignment(
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    rotation_a: Quat,
    rotation_b: Quat,
    first_key: u8,
) -> Result<(), CollisionError> {
    let settings = lock_settings(settings);
    let axis_a = rotation_a
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let axis_b = rotation_b
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let dot = axis_a
        .checked_dot(axis_b)
        .ok_or(CollisionError::ArithmeticOverflow)?
        .clamp(-1.0, 1.0);
    let cross = axis_a
        .checked_cross(axis_b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let sine = cross
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let error = if sine > f64::MIN_POSITIVE {
        cross * (sine.atan2(dot) / sine)
    } else if dot < 0.0 {
        let (axis, _) = constraint::tangent_basis(axis_a).ok_or(CollisionError::InvalidContact)?;
        axis * std::f64::consts::PI
    } else {
        Vec3::ZERO
    };
    let (tangent_a, tangent_b) =
        constraint::tangent_basis(axis_a).ok_or(CollisionError::InvalidContact)?;
    for (axis_index, axis) in [tangent_a, tangent_b].into_iter().enumerate() {
        let component = error
            .checked_dot(axis)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        // A zero position error still leaves the relative angular velocity
        // unconstrained: the aligned axis must keep matching velocity around
        // both tangent axes. Emit both rows unconditionally so the velocity
        // solve cannot inject a forbidden angular velocity.
        push_angular_row(
            workspace,
            settings,
            axis,
            component,
            first_key + axis_index as u8,
            bilateral_bounds(),
        )?;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)] // Keep limit inputs explicit at the solver call sites.
fn append_linear_limit<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    anchors: [Vec3; 2],
    axis: Vec3,
    axis_anchor: Vec3,
    coordinate: f64,
    limit: super::JointLimit,
    key: u8,
) -> Result<(), CollisionError> {
    // When the two unilateral bounds coincide, their intersection is a
    // bilateral equality. Emit a hard, bias-free lock every step so the
    // collapsed limit cannot be pushed through by another row sharing the
    // degree of freedom.
    if let Some(value) = collapsed_limit_value(limit) {
        return push_linear_row(
            bodies,
            workspace,
            lock_settings(settings),
            anchors,
            axis,
            axis_anchor,
            coordinate - value,
            key,
            bilateral_bounds(),
        );
    }
    let mut row_settings = settings;
    let active = if let Some(violation) = limit_error(coordinate, limit) {
        // Position projection removes an existing violation. Disable the
        // Baumgarte velocity bias so the same error is not corrected twice.
        row_settings.velocity_bias = false;
        Some(violation)
    } else {
        let body_a = bodies
            .get(settings.body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let body_b = bodies
            .get(settings.body_b)
            .ok_or(CollisionError::InvalidContact)?;
        let speed =
            constraint::relative_velocity_at(body_a.body(), body_b.body(), anchors[0], anchors[1])
                .map_err(map_constraint_error)?
                .checked_dot(axis)
                .ok_or(CollisionError::ArithmeticOverflow)?;
        let predicted = coordinate + speed * settings.dt;
        if !predicted.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if speed > 0.0 && predicted > limit.upper {
            row_settings.position_enabled = false;
            row_settings.velocity_target = (limit.upper - coordinate) / settings.dt;
            Some((0.0, (f64::NEG_INFINITY, 0.0)))
        } else if speed < 0.0 && predicted < limit.lower {
            row_settings.position_enabled = false;
            row_settings.velocity_target = (limit.lower - coordinate) / settings.dt;
            Some((0.0, (0.0, f64::INFINITY)))
        } else {
            None
        }
    };
    if let Some((error, bounds)) = active {
        push_linear_row(
            bodies,
            workspace,
            row_settings,
            anchors,
            axis,
            axis_anchor,
            error,
            key,
            bounds,
        )?;
    }
    Ok(())
}

fn append_angular_limit<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    rotation_a: Quat,
    angle: f64,
    limit: super::JointLimit,
    key: u8,
) -> Result<(), CollisionError> {
    let axis = rotation_a
        .rotate_vector(Vec3::Z)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    append_angular_axis_limit(bodies, workspace, settings, axis, angle, limit, key)
}

fn append_angular_axis_limit<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    axis: Vec3,
    angle: f64,
    limit: super::JointLimit,
    key: u8,
) -> Result<(), CollisionError> {
    // A collapsed angular limit is a bilateral equality, exactly as for the
    // linear case: lock the angle every step instead of waiting for a
    // predicted crossing.
    if let Some(value) = collapsed_limit_value(limit) {
        return push_angular_row(
            workspace,
            lock_settings(settings),
            axis,
            angle - value,
            key,
            bilateral_bounds(),
        );
    }
    let mut row_settings = settings;
    let active = if let Some(violation) = limit_error(angle, limit) {
        // Position projection removes an existing violation. Disable the
        // Baumgarte velocity bias so the same error is not corrected twice.
        row_settings.velocity_bias = false;
        Some(violation)
    } else {
        let body_a = bodies
            .get(settings.body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let body_b = bodies
            .get(settings.body_b)
            .ok_or(CollisionError::InvalidContact)?;
        let speed = body_b
            .state()
            .angular_velocity()
            .checked_sub(body_a.state().angular_velocity())
            .and_then(|velocity| velocity.checked_dot(axis))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let predicted = angle + speed * settings.dt;
        if !predicted.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if speed > 0.0 && predicted > limit.upper {
            row_settings.position_enabled = false;
            row_settings.velocity_target = (limit.upper - angle) / settings.dt;
            Some((0.0, (f64::NEG_INFINITY, 0.0)))
        } else if speed < 0.0 && predicted < limit.lower {
            row_settings.position_enabled = false;
            row_settings.velocity_target = (limit.lower - angle) / settings.dt;
            Some((0.0, (0.0, f64::INFINITY)))
        } else {
            None
        }
    };
    if let Some((error, bounds)) = active {
        push_angular_row(workspace, row_settings, axis, error, key, bounds)?;
    }
    Ok(())
}

fn limit_error(value: f64, limit: super::JointLimit) -> Option<(f64, (f64, f64))> {
    if value < limit.lower {
        Some((value - limit.lower, (0.0, f64::INFINITY)))
    } else if value > limit.upper {
        Some((value - limit.upper, (f64::NEG_INFINITY, 0.0)))
    } else {
        None
    }
}

/// A limit whose lower and upper bounds coincide is the intersection of the two
/// unilateral constraints, i.e. a bilateral equality. Returns the fixed value
/// so callers can emit an unconditional lock row.
fn collapsed_limit_value(limit: super::JointLimit) -> Option<f64> {
    (limit.lower == limit.upper).then_some(limit.lower)
}

fn row_settings(body_a: usize, body_b: usize, common: super::JointCommon, dt: f64) -> RowSettings {
    RowSettings {
        body_a,
        body_b,
        common,
        dt,
        position_enabled: true,
        velocity_bias: true,
        velocity_target: 0.0,
        force_bias: 0.0,
        force_driven: false,
        multiplier_bounds: None,
        impulse_bounds: None,
    }
}

/// Row settings for a hard bilateral lock. Position projection (XPBD) already
/// removes the positional error, so these rows must not also carry a Baumgarte
/// velocity target: applying both corrects the same error twice and injects
/// energy into the relative motion. This mirrors the FixedJoint row builder.
fn lock_settings(mut settings: RowSettings) -> RowSettings {
    settings.velocity_bias = false;
    settings
}

fn perpendicular_basis(axis: Vec3) -> Result<[Vec3; 2], CollisionError> {
    let (first, second) = constraint::tangent_basis(axis).ok_or(CollisionError::InvalidContact)?;
    Ok([first, second])
}

fn drive_settings(
    mut settings: RowSettings,
    drive: super::JointDrive,
) -> Result<Option<RowSettings>, CollisionError> {
    if !drive.enabled || drive.max_force == 0.0 {
        return Ok(None);
    }
    let max_impulse = drive.max_force * settings.dt;
    let max_multiplier = max_impulse * settings.dt;
    if !max_impulse.is_finite() || !max_multiplier.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let compliance = if drive.stiffness > 0.0 {
        drive.stiffness.recip()
    } else {
        0.0
    };
    if !compliance.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    settings.common.compliance = compliance;
    settings.common.damping = drive.damping;
    settings.position_enabled = drive.stiffness > 0.0;
    settings.velocity_bias = drive.stiffness > 0.0;
    settings.velocity_target = drive.target_velocity;
    settings.multiplier_bounds = Some((-max_multiplier, max_multiplier));
    settings.impulse_bounds = Some((-max_impulse, max_impulse));
    Ok(Some(settings))
}

fn world_frame<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    body_index: usize,
    frame: super::JointFrame,
) -> Result<(Vec3, Quat), CollisionError> {
    let body = bodies
        .get(body_index)
        .ok_or(CollisionError::InvalidContact)?;
    let local = Transform::new(frame.local_anchor, frame.local_rotation)
        .map_err(|_| CollisionError::InvalidContact)?;
    let transform = body
        .state()
        .transform()
        .compose(&local)
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    Ok((transform.translation(), transform.rotation()))
}

pub(super) fn relative_rotation_error(
    rotation_a: Quat,
    rotation_b: Quat,
) -> Result<Vec3, CollisionError> {
    let relative = canonical_relative_rotation(rotation_a, rotation_b)?;
    let vector = Vec3::new(relative.x, relative.y, relative.z);
    let sine = vector
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if sine <= f64::MIN_POSITIVE {
        return Ok(Vec3::ZERO);
    }
    let angle = 2.0 * sine.atan2(relative.w);
    vector
        .checked_mul_scalar(angle / sine)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn canonical_relative_rotation(rotation_a: Quat, rotation_b: Quat) -> Result<Quat, CollisionError> {
    let mut relative = (rotation_a.conjugate() * rotation_b)
        .try_normalize()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if relative.w < 0.0 {
        relative = Quat::new(-relative.x, -relative.y, -relative.z, -relative.w);
    }
    Ok(relative)
}

pub(super) fn fixed_rotation_coefficient(rotation_error: Vec3) -> Result<f64, CollisionError> {
    let angle_squared = rotation_error
        .checked_dot(rotation_error)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if !angle_squared.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    // The FixedJoint error is log(R_a^-1 R_b). Its derivative for world-space
    // angular velocities uses J_l^-1(phi); row gradients use its transpose.
    let coefficient = if angle_squared < 1.0e-4 {
        1.0 / 12.0 + angle_squared / 720.0 + angle_squared * angle_squared / 30_240.0
    } else {
        let angle = angle_squared.sqrt();
        let half_angle = angle * 0.5;
        1.0 / angle_squared - half_angle.cos() / (2.0 * angle * half_angle.sin())
    };
    if !coefficient.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(coefficient)
}

pub(super) fn fixed_rotation_row_axis(
    rotation_a: Quat,
    rotation_error: Vec3,
    local_axis: Vec3,
    coefficient: f64,
) -> Result<Vec3, CollisionError> {
    if !local_axis.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let first_cross = rotation_error
        .checked_cross(local_axis)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let second_cross = rotation_error
        .checked_cross(first_cross)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let local_gradient = local_axis
        .checked_add(first_cross * 0.5)
        .and_then(|value| value.checked_add(second_cross * coefficient))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    rotation_a
        .rotate_vector(local_gradient)
        .filter(|axis| axis.is_finite())
        .ok_or(CollisionError::ArithmeticOverflow)
}

const TWIST_SINGULAR_TOLERANCE: f64 = 64.0 * f64::EPSILON;

fn twist_angle(relative: Quat) -> Result<Option<f64>, CollisionError> {
    let length = relative.z.hypot(relative.w);
    if !length.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if length <= TWIST_SINGULAR_TOLERANCE {
        // The swing is valid but its twist coordinate is undefined. Callers
        // skip only constraints that require this coordinate until it exists.
        return Ok(None);
    }
    let angle = 2.0 * (relative.z / length).atan2(relative.w / length);
    let wrapped =
        (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
    wrapped
        .is_finite()
        .then_some(Some(wrapped))
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn bilateral_bounds() -> (f64, f64) {
    (f64::NEG_INFINITY, f64::INFINITY)
}

#[allow(clippy::too_many_arguments)] // Keep row geometry explicit in the hot path.
fn push_linear_row<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    anchors: [Vec3; 2],
    axis: Vec3,
    axis_anchor: Vec3,
    error: f64,
    key: u8,
    bounds: (f64, f64),
) -> Result<(), CollisionError> {
    let centers = [
        center_of_mass(
            bodies
                .get(settings.body_a)
                .ok_or(CollisionError::InvalidContact)?,
        )?,
        center_of_mass(
            bodies
                .get(settings.body_b)
                .ok_or(CollisionError::InvalidContact)?,
        )?,
    ];
    workspace.push(make_linear_row(
        settings,
        anchors,
        axis,
        axis_anchor,
        error,
        key,
        bounds,
        centers,
    )?);
    Ok(())
}

/// Builds one linear constraint row.
///
/// `axis_anchor` is the world point the axis is rigidly attached to. For axes
/// that co-rotate with body A (frame-derived axes) that is body B's anchor:
/// the axis motion then contributes the exact `delta * (omega_a x axis)`
/// derivative term, and the impulse pair acts on both bodies at that one
/// point, which conserves angular momentum exactly. For world-aligned axes
/// (distance/spring directions) it is body A's anchor and the row
/// reduces to the classic anchor-pair Jacobian.
#[allow(clippy::too_many_arguments)] // Row construction stays explicit at call sites.
fn make_linear_row(
    settings: RowSettings,
    anchors: [Vec3; 2],
    axis: Vec3,
    axis_anchor: Vec3,
    error: f64,
    key: u8,
    bounds: (f64, f64),
    centers: [Vec3; 2],
) -> Result<ConstraintRow, CollisionError> {
    if !axis.is_finite()
        || !axis_anchor.is_finite()
        || !error.is_finite()
        || !settings.dt.is_finite()
        || settings.dt <= 0.0
    {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let offset_a = axis_anchor
        .checked_sub(centers[0])
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let position_offset_a = anchors[0]
        .checked_sub(centers[0])
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let offset_b = anchors[1]
        .checked_sub(centers[1])
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let angular_a = offset_a
        .checked_cross(axis)
        .and_then(|value| value.checked_mul_scalar(-1.0))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let position_angular_a = position_offset_a
        .checked_cross(axis)
        .and_then(|value| value.checked_mul_scalar(-1.0))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let angular_b = offset_b
        .checked_cross(axis)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    Ok(ConstraintRow {
        key,
        dimension: RowDimension::Linear,
        body_a: settings.body_a,
        body_b: settings.body_b,
        linear_a: -axis,
        angular_a,
        position_angular_a,
        linear_b: axis,
        angular_b,
        error,
        target_velocity: if settings.velocity_bias {
            -error * (0.2 / settings.dt) + settings.velocity_target
        } else {
            settings.velocity_target
        },
        force_bias: settings.force_bias,
        force_driven: settings.force_driven,
        position_enabled: settings.position_enabled,
        compliance: settings.common.compliance,
        damping: settings.common.damping,
        min_multiplier: settings.multiplier_bounds.map_or(bounds.0, |value| value.0),
        max_multiplier: settings.multiplier_bounds.map_or(bounds.1, |value| value.1),
        min_impulse: settings.impulse_bounds.map_or(bounds.0, |value| value.0),
        max_impulse: settings.impulse_bounds.map_or(bounds.1, |value| value.1),
        accumulated_multiplier: 0.0,
        accumulated_impulse: 0.0,
        cached_velocity_effective_mass: 0.0,
    })
}

fn push_angular_row(
    workspace: &mut JointWorkspace,
    settings: RowSettings,
    axis: Vec3,
    error: f64,
    key: u8,
    bounds: (f64, f64),
) -> Result<(), CollisionError> {
    workspace.push(make_angular_row(settings, axis, error, key, bounds)?);
    Ok(())
}

fn make_angular_row(
    settings: RowSettings,
    axis: Vec3,
    error: f64,
    key: u8,
    bounds: (f64, f64),
) -> Result<ConstraintRow, CollisionError> {
    if !axis.is_finite() || !error.is_finite() || !settings.dt.is_finite() || settings.dt <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(ConstraintRow {
        key,
        dimension: RowDimension::Angular,
        body_a: settings.body_a,
        body_b: settings.body_b,
        linear_a: Vec3::ZERO,
        angular_a: -axis,
        position_angular_a: -axis,
        linear_b: Vec3::ZERO,
        angular_b: axis,
        error,
        target_velocity: if settings.velocity_bias {
            -error * (0.2 / settings.dt) + settings.velocity_target
        } else {
            settings.velocity_target
        },
        force_bias: settings.force_bias,
        force_driven: settings.force_driven,
        position_enabled: settings.position_enabled,
        compliance: settings.common.compliance,
        damping: settings.common.damping,
        min_multiplier: settings.multiplier_bounds.map_or(bounds.0, |value| value.0),
        max_multiplier: settings.multiplier_bounds.map_or(bounds.1, |value| value.1),
        min_impulse: settings.impulse_bounds.map_or(bounds.0, |value| value.0),
        max_impulse: settings.impulse_bounds.map_or(bounds.1, |value| value.1),
        accumulated_multiplier: 0.0,
        accumulated_impulse: 0.0,
        cached_velocity_effective_mass: 0.0,
    })
}

fn center_of_mass(body: &SolverBody) -> Result<Vec3, CollisionError> {
    body.body()
        .world_center_of_mass()
        .map_err(|_| CollisionError::ArithmeticOverflow)
        .map(|center| center.unwrap_or(body.state().transform().translation()))
}

fn solve_position_iteration<B: SolverBodyAccess + ?Sized>(
    workspace: &mut JointWorkspace,
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
) -> Result<(), CollisionError> {
    if options.mass_cache() {
        workspace.ensure_solver_mass_slots(bodies.len())?;
    }
    let mut cache_stats = SolverMassCacheStats::default();
    let result = {
        let (rows, slots) = workspace.solver_buffers_mut(false);
        solve_position_rows(rows, slots, bodies, dt, options, &mut cache_stats)
    };
    record_solver_mass_cache_stats(workspace, cache_stats);
    result
}

fn solve_position_rows<B: SolverBodyAccess + ?Sized>(
    rows: &mut [ConstraintRow],
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<(), CollisionError> {
    let dt_squared = dt * dt;
    if !dt_squared.is_finite() || dt_squared <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let alpha_scale = dt_squared.recip();
    for row in rows {
        solve_position_row(row, slots, bodies, alpha_scale, options, cache_stats)?;
    }
    Ok(())
}

fn solve_position_row<B: SolverBodyAccess + ?Sized>(
    row: &mut ConstraintRow,
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    alpha_scale: f64,
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<bool, CollisionError> {
    if !row.is_valid(bodies.len()) {
        return Err(CollisionError::InvalidContact);
    }
    if !row.position_enabled {
        return Ok(false);
    }
    // The position projection follows the frozen-axis arm: see
    // `ConstraintRow::position_angular_a`.
    let mut position_row = *row;
    position_row.angular_a = row.position_angular_a;
    let (effective_mass, masses) =
        effective_mass_for_row(bodies, slots, position_row, options, cache_stats)?;
    let alpha = position_row.compliance * alpha_scale;
    if !alpha.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let denominator = effective_mass + alpha;
    if denominator <= f64::MIN_POSITIVE {
        return Ok(false);
    }
    let delta = -(position_row.error + alpha * position_row.accumulated_multiplier) / denominator;
    let next = (position_row.accumulated_multiplier + delta)
        .clamp(position_row.min_multiplier, position_row.max_multiplier);
    let applied = next - position_row.accumulated_multiplier;
    if !next.is_finite() || !applied.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if applied != 0.0 {
        apply_position_row(bodies, position_row, applied, masses)?;
        if options.mass_cache() {
            invalidate_solver_mass_pair(slots, row.body_a, row.body_b)?;
        }
        row.accumulated_multiplier = next;
    }
    Ok(applied != 0.0)
}

fn solve_velocity_iteration<B: SolverBodyAccess + ?Sized>(
    workspace: &mut JointWorkspace,
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
) -> Result<(), CollisionError> {
    if options.mass_cache() {
        workspace.ensure_solver_mass_slots(bodies.len())?;
    }
    let mut cache_stats = SolverMassCacheStats::default();
    let result = {
        let (rows, slots) = workspace.solver_buffers_mut(false);
        solve_velocity_rows(rows, slots, bodies, dt, options, &mut cache_stats)
    };
    record_solver_mass_cache_stats(workspace, cache_stats);
    result
}

fn solve_velocity_rows<B: SolverBodyAccess + ?Sized>(
    rows: &mut [ConstraintRow],
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<(), CollisionError> {
    // Rows of one joint share the same body pair and the velocity pass never
    // changes poses, so resolve each pair's mass response once per sweep.
    let mut pair = None;
    for row in rows {
        solve_velocity_row_with_pair(row, slots, bodies, dt, options, cache_stats, &mut pair)?;
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct VelocityMassPair {
    body_a: usize,
    body_b: usize,
    masses: [SolverMass; 2],
    statuses: [SolverMassCacheStatus; 2],
}

#[cfg(test)]
fn solve_velocity_row<B: SolverBodyAccess + ?Sized>(
    row: &mut ConstraintRow,
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<(), CollisionError> {
    solve_velocity_row_with_pair(row, slots, bodies, dt, options, cache_stats, &mut None)
}

fn solve_velocity_row_with_pair<B: SolverBodyAccess + ?Sized>(
    row: &mut ConstraintRow,
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
    pair: &mut Option<VelocityMassPair>,
) -> Result<(), CollisionError> {
    if !row.is_valid(bodies.len()) {
        return Err(CollisionError::InvalidContact);
    }
    let (effective_mass, masses) = if options.mass_cache() {
        let (masses, statuses) = match *pair {
            Some(cached) if cached.body_a == row.body_a && cached.body_b == row.body_b => {
                (cached.masses, cached.statuses)
            }
            _ => {
                let (masses, statuses) = resolve_solver_mass_pair_with_status(
                    bodies,
                    slots,
                    row.body_a,
                    row.body_b,
                    cache_stats,
                )?;
                *pair = Some(VelocityMassPair {
                    body_a: row.body_a,
                    body_b: row.body_b,
                    masses,
                    statuses,
                });
                (masses, statuses)
            }
        };
        let unchanged = statuses == [SolverMassCacheStatus::Hit; 2];
        let effective_mass = if unchanged && row.cached_velocity_effective_mass > 0.0 {
            row.cached_velocity_effective_mass
        } else {
            effective_mass_from_masses(*row, masses)?
        };
        row.cached_velocity_effective_mass = effective_mass;
        (effective_mass, Some(masses))
    } else {
        let (effective_mass, masses) =
            effective_mass_for_row(bodies, slots, *row, options, cache_stats)?;
        (effective_mass, masses)
    };
    if effective_mass <= f64::MIN_POSITIVE {
        return Ok(());
    }
    let velocity = row_velocity(bodies, *row)?;
    let speed_error = row.target_velocity - velocity;
    let alpha = row.compliance / (dt * dt);
    if !alpha.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let delta = if row.force_driven {
        // Integrate Hooke force and implicit viscous damping in this GS row.
        let damping = row.damping * dt;
        let spring_impulse = row.force_bias * dt;
        if damping > 0.0 {
            (-spring_impulse - damping * (velocity - row.target_velocity) - row.accumulated_impulse)
                / (1.0 + damping * effective_mass)
        } else {
            -spring_impulse - row.accumulated_impulse
        }
    } else if row.compliance == 0.0 && row.damping == 0.0 {
        speed_error / effective_mass
    } else if row.damping > 0.0 {
        let scaled_damping = row.damping * dt;
        let denominator = 1.0_f64 + scaled_damping * (effective_mass + alpha);
        scaled_damping * speed_error / denominator
    } else if row.target_velocity != 0.0 {
        speed_error / (effective_mass + alpha)
    } else {
        0.0
    } * options.velocity_relaxation();
    let next = (row.accumulated_impulse + delta).clamp(row.min_impulse, row.max_impulse);
    let applied = next - row.accumulated_impulse;
    if !delta.is_finite() || !next.is_finite() || !applied.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if applied != 0.0 {
        apply_velocity_row(bodies, *row, applied, masses)?;
        row.accumulated_impulse = next;
    }
    Ok(())
}

fn solve_position_projection_group<B: SolverBodyAccess + ?Sized>(
    rows: &mut [ConstraintRow],
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
    selection: ([u8; 3], Option<[SolverMass; 2]>),
    cache_stats: &mut SolverMassCacheStats,
) -> Result<ProjectionGroupResult, CollisionError> {
    let (keys, resolved_masses) = selection;
    validate_projection_rows(rows, bodies.len())?;
    // Resolved masses are passed only by the in-place FixedJoint path, whose
    // six-row layout and ownership were checked before selecting this slice.
    let indices = if resolved_masses.is_some() {
        Some([0, 1, 2])
    } else {
        projection_group_indices(rows, keys)
    };
    if let Some(indices) = indices {
        let block_rows = [rows[indices[0]], rows[indices[1]], rows[indices[2]]];
        let dimension = if keys[0] == 0 {
            RowDimension::Linear
        } else {
            RowDimension::Angular
        };
        if hard_bilateral_block(block_rows, keys, dimension, bodies.len()) {
            let masses = match resolved_masses {
                Some(masses) => masses,
                None => projection_masses(bodies, slots, block_rows, options, cache_stats)?,
            };
            if let Some(block) = prepared_block(block_rows, masses, dt, keys[0] == 0)? {
                match block.solve_position(dt) {
                    Ok(delta) => {
                        let pose_changed = apply_position_block(
                            rows,
                            indices,
                            delta,
                            bodies,
                            (resolved_masses.is_some() || options.mass_cache()).then_some(masses),
                        )?;
                        if options.mass_cache() && pose_changed {
                            invalidate_solver_mass_pair(
                                slots,
                                block_rows[0].body_a,
                                block_rows[0].body_b,
                            )?;
                        }
                        return Ok(ProjectionGroupResult {
                            pose_changed,
                            block_rows: 3,
                            fallback_rows: 0,
                        });
                    }
                    Err(CollisionError::NonConvergent) => {}
                    Err(error) => return Err(error),
                }
            }
        }
    }

    let dt_squared = dt * dt;
    if !dt_squared.is_finite() || dt_squared <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let alpha_scale = dt_squared.recip();
    let mut fallback_rows = 0;
    let mut pose_changed = false;
    for row in rows.iter_mut().filter(|row| keys.contains(&row.key)) {
        pose_changed |= solve_position_row(row, slots, bodies, alpha_scale, options, cache_stats)?;
        fallback_rows += 1;
    }
    Ok(ProjectionGroupResult {
        pose_changed,
        fallback_rows,
        ..ProjectionGroupResult::default()
    })
}

#[allow(clippy::too_many_arguments)]
fn solve_velocity_projection_group<B: SolverBodyAccess + ?Sized>(
    rows: &mut [ConstraintRow],
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
    keys: [u8; 3],
    cache_stats: &mut SolverMassCacheStats,
    operators: &mut FixedBlockVelocityOperators,
) -> Result<ProjectionGroupResult, CollisionError> {
    validate_projection_rows(rows, bodies.len())?;
    let indices = projection_group_indices(rows, keys);
    if let Some(indices) = indices {
        let block_rows = [rows[indices[0]], rows[indices[1]], rows[indices[2]]];
        let dimension = if keys[0] == 0 {
            RowDimension::Linear
        } else {
            RowDimension::Angular
        };
        if hard_bilateral_block(block_rows, keys, dimension, bodies.len()) {
            let masses = projection_masses(bodies, slots, block_rows, options, cache_stats)?;
            let block = if keys[0] == 0 {
                FixedJointRowBlock::Anchor
            } else {
                FixedJointRowBlock::Frame
            };
            if let Some(factor) = velocity_block_operator(block_rows, masses, block, operators)? {
                let velocity_error = Vec3::new(
                    block_rows[0].target_velocity - row_velocity(bodies, block_rows[0])?,
                    block_rows[1].target_velocity - row_velocity(bodies, block_rows[1])?,
                    block_rows[2].target_velocity - row_velocity(bodies, block_rows[2])?,
                );
                if !velocity_error.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                match factor.solve(velocity_error) {
                    Ok(delta) if delta.is_finite() => {
                        apply_velocity_block(
                            rows,
                            indices,
                            delta,
                            bodies,
                            options.mass_cache().then_some(masses),
                        )?;
                        return Ok(ProjectionGroupResult {
                            block_rows: 3,
                            ..ProjectionGroupResult::default()
                        });
                    }
                    _ => {}
                }
            }
        }
    }

    let mut fallback_rows = 0;
    let mut pair = None;
    for row in rows.iter_mut().filter(|row| keys.contains(&row.key)) {
        solve_velocity_row_with_pair(row, slots, bodies, dt, options, cache_stats, &mut pair)?;
        fallback_rows += 1;
    }
    Ok(ProjectionGroupResult {
        fallback_rows,
        ..ProjectionGroupResult::default()
    })
}

fn solve_position_projection_remainder<B: SolverBodyAccess + ?Sized>(
    rows: &mut [ConstraintRow],
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<ProjectionGroupResult, CollisionError> {
    validate_projection_rows(rows, bodies.len())?;
    let dt_squared = dt * dt;
    if !dt_squared.is_finite() || dt_squared <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let alpha_scale = dt_squared.recip();
    let mut fallback_rows = 0;
    for row in rows.iter_mut().filter(|row| row.key > 5) {
        solve_position_row(row, slots, bodies, alpha_scale, options, cache_stats)?;
        fallback_rows += 1;
    }
    Ok(ProjectionGroupResult {
        fallback_rows,
        ..ProjectionGroupResult::default()
    })
}

fn solve_velocity_projection_remainder<B: SolverBodyAccess + ?Sized>(
    rows: &mut [ConstraintRow],
    slots: &mut [SolverMassSlot],
    bodies: &mut B,
    dt: f64,
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<ProjectionGroupResult, CollisionError> {
    validate_projection_rows(rows, bodies.len())?;
    let mut fallback_rows = 0;
    let mut pair = None;
    for row in rows.iter_mut().filter(|row| row.key > 5) {
        solve_velocity_row_with_pair(row, slots, bodies, dt, options, cache_stats, &mut pair)?;
        fallback_rows += 1;
    }
    Ok(ProjectionGroupResult {
        fallback_rows,
        ..ProjectionGroupResult::default()
    })
}

fn validate_projection_rows(
    rows: &[ConstraintRow],
    body_count: usize,
) -> Result<(), CollisionError> {
    if rows.iter().all(|row| row.is_valid(body_count)) {
        Ok(())
    } else {
        Err(CollisionError::InvalidContact)
    }
}

fn projection_group_indices(rows: &[ConstraintRow], keys: [u8; 3]) -> Option<[usize; 3]> {
    let mut indices = [None; 3];
    let mut matching_rows = 0;
    let mut duplicate_key = false;
    for (row_index, row) in rows.iter().enumerate() {
        let Some(key_index) = keys.iter().position(|key| *key == row.key) else {
            continue;
        };
        matching_rows += 1;
        if indices[key_index].replace(row_index).is_some() {
            duplicate_key = true;
        }
    }
    if duplicate_key || matching_rows != 3 {
        return None;
    }
    match indices {
        [Some(a), Some(b), Some(c)] => Some([a, b, c]),
        _ => None,
    }
}

fn hard_bilateral_block(
    rows: [ConstraintRow; 3],
    keys: [u8; 3],
    dimension: RowDimension,
    body_count: usize,
) -> bool {
    let body_pair = (rows[0].body_a, rows[0].body_b);
    rows.iter().zip(keys).all(|(row, key)| {
        row.key == key
            && row.dimension == dimension
            && row.is_valid(body_count)
            && row.position_enabled
            && (row.body_a, row.body_b) == body_pair
            && row.compliance == 0.0
            && row.damping == 0.0
            && !row.force_driven
            && row.target_velocity == 0.0
            && row.min_multiplier == f64::NEG_INFINITY
            && row.max_multiplier == f64::INFINITY
            && row.min_impulse == f64::NEG_INFINITY
            && row.max_impulse == f64::INFINITY
    })
}

fn projection_masses<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    slots: &mut [SolverMassSlot],
    rows: [ConstraintRow; 3],
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<[SolverMass; 2], CollisionError> {
    let body_a = rows[0].body_a;
    let body_b = rows[0].body_b;
    if options.mass_cache() {
        resolve_solver_mass_pair(bodies, slots, body_a, body_b, cache_stats)
    } else {
        let (body_a, body_b) = solver_bodies_at(bodies, body_a, body_b)?;
        Ok([
            SolverMass::from_body(body_a.body())?,
            SolverMass::from_body(body_b.body())?,
        ])
    }
}

fn prepared_block(
    rows: [ConstraintRow; 3],
    masses: [SolverMass; 2],
    dt: f64,
    anchor: bool,
) -> Result<Option<super::block::Block3>, CollisionError> {
    match super::block::prepare_fixed_block3(&rows, masses, dt, anchor) {
        Ok(block) => Ok(Some(block)),
        Err(
            CollisionError::InvalidContact
            | CollisionError::InvalidConfig
            | CollisionError::NonConvergent,
        ) => Ok(None),
        Err(error) => Err(error),
    }
}

/// Returns the cached Cholesky factorization of `J M^-1 J^T` for a velocity
/// block, computing and factoring it once per step and reusing it while the
/// solver masses stay unchanged. Mirrors `prepared_block` fallback semantics:
/// invalid or non-SPD blocks yield `Ok(None)` so the caller drops to the scalar
/// path, while unexpected errors propagate.
fn velocity_block_operator(
    rows: [ConstraintRow; 3],
    masses: [SolverMass; 2],
    block: FixedJointRowBlock,
    operators: &mut FixedBlockVelocityOperators,
) -> Result<Option<Cholesky3>, CollisionError> {
    if let Some(cached) = operators
        .slot(block)
        .filter(|cached| cached.masses == masses)
    {
        return Ok(Some(cached.factor));
    }
    let inverse_effective_mass = match super::block::prepare_block3_operator(&rows, masses) {
        Ok(inverse_effective_mass) => inverse_effective_mass,
        Err(
            CollisionError::InvalidContact
            | CollisionError::InvalidConfig
            | CollisionError::NonConvergent,
        ) => return Ok(None),
        Err(error) => return Err(error),
    };
    let Ok(factor) = inverse_effective_mass.factor_spd() else {
        return Ok(None);
    };
    operators.set(block, FixedBlockVelocityOperator { masses, factor });
    Ok(Some(factor))
}

fn apply_position_block<B: SolverBodyAccess + ?Sized>(
    rows: &mut [ConstraintRow],
    indices: [usize; 3],
    delta: Vec3,
    bodies: &mut B,
    masses: Option<[SolverMass; 2]>,
) -> Result<bool, CollisionError> {
    if !delta.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let selected = [rows[indices[0]], rows[indices[1]], rows[indices[2]]];
    let linear_a = selected[0].linear_a * delta.x
        + selected[1].linear_a * delta.y
        + selected[2].linear_a * delta.z;
    let angular_a = selected[0].angular_a * delta.x
        + selected[1].angular_a * delta.y
        + selected[2].angular_a * delta.z;
    let linear_b = selected[0].linear_b * delta.x
        + selected[1].linear_b * delta.y
        + selected[2].linear_b * delta.z;
    let angular_b = selected[0].angular_b * delta.x
        + selected[1].angular_b * delta.y
        + selected[2].angular_b * delta.z;
    if !linear_a.is_finite()
        || !angular_a.is_finite()
        || !linear_b.is_finite()
        || !angular_b.is_finite()
    {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let next_multipliers = [
        rows[indices[0]].accumulated_multiplier + delta.x,
        rows[indices[1]].accumulated_multiplier + delta.y,
        rows[indices[2]].accumulated_multiplier + delta.z,
    ];
    if next_multipliers.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let applied = indices
        .into_iter()
        .zip(next_multipliers)
        .any(|(index, next)| next != rows[index].accumulated_multiplier);
    if applied {
        apply_position_impulse_at(
            bodies,
            selected[0].body_a,
            constraint::Jacobian {
                linear: linear_a,
                angular: angular_a,
            },
            masses.map(|m| m[0]),
        )?;
        apply_position_impulse_at(
            bodies,
            selected[0].body_b,
            constraint::Jacobian {
                linear: linear_b,
                angular: angular_b,
            },
            masses.map(|m| m[1]),
        )?;
    }
    for (index, next) in indices.into_iter().zip(next_multipliers) {
        rows[index].accumulated_multiplier = next;
    }
    Ok(applied)
}

fn apply_velocity_block<B: SolverBodyAccess + ?Sized>(
    rows: &mut [ConstraintRow],
    indices: [usize; 3],
    delta: Vec3,
    bodies: &mut B,
    masses: Option<[SolverMass; 2]>,
) -> Result<(), CollisionError> {
    if !delta.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let selected = [rows[indices[0]], rows[indices[1]], rows[indices[2]]];
    let impulses = [
        constraint::Jacobian {
            linear: selected[0].linear_a * delta.x
                + selected[1].linear_a * delta.y
                + selected[2].linear_a * delta.z,
            angular: selected[0].angular_a * delta.x
                + selected[1].angular_a * delta.y
                + selected[2].angular_a * delta.z,
        },
        constraint::Jacobian {
            linear: selected[0].linear_b * delta.x
                + selected[1].linear_b * delta.y
                + selected[2].linear_b * delta.z,
            angular: selected[0].angular_b * delta.x
                + selected[1].angular_b * delta.y
                + selected[2].angular_b * delta.z,
        },
    ];
    if impulses
        .iter()
        .any(|impulse| !impulse.linear.is_finite() || !impulse.angular.is_finite())
    {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let next_impulses = [
        rows[indices[0]].accumulated_impulse + delta.x,
        rows[indices[1]].accumulated_impulse + delta.y,
        rows[indices[2]].accumulated_impulse + delta.z,
    ];
    if next_impulses.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    apply_velocity_impulse_at(
        bodies,
        selected[0].body_a,
        impulses[0],
        masses.map(|m| m[0]),
    )?;
    apply_velocity_impulse_at(
        bodies,
        selected[0].body_b,
        impulses[1],
        masses.map(|m| m[1]),
    )?;
    for (index, next) in indices.into_iter().zip(next_impulses) {
        rows[index].accumulated_impulse = next;
    }
    Ok(())
}

fn effective_mass_for_row<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    slots: &mut [SolverMassSlot],
    row: ConstraintRow,
    options: JointSolverOptions,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<(f64, Option<[SolverMass; 2]>), CollisionError> {
    if options.mass_cache() {
        let masses = resolve_solver_mass_pair(bodies, slots, row.body_a, row.body_b, cache_stats)?;
        effective_mass_from_masses(row, masses).map(|effective_mass| (effective_mass, Some(masses)))
    } else {
        let jacobians = row_jacobians(row);
        let (body_a, body_b) = solver_bodies_at(bodies, row.body_a, row.body_b)?;
        constraint::effective_cross_mass(body_a.body(), body_b.body(), jacobians, jacobians)
            .map_err(map_constraint_error)
            .map(|effective_mass| (effective_mass, None))
    }
}

fn effective_mass_from_masses(
    row: ConstraintRow,
    masses: [SolverMass; 2],
) -> Result<f64, CollisionError> {
    let jacobians = row_jacobians(row);
    constraint::effective_cross_mass_cached(masses, jacobians, jacobians)
        .map_err(map_constraint_error)
}

fn row_jacobians(row: ConstraintRow) -> [constraint::Jacobian; 2] {
    [
        constraint::Jacobian {
            linear: row.linear_a,
            angular: row.angular_a,
        },
        constraint::Jacobian {
            linear: row.linear_b,
            angular: row.angular_b,
        },
    ]
}

pub(super) fn row_velocity<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    row: ConstraintRow,
) -> Result<f64, CollisionError> {
    let (body_a, body_b) = solver_bodies_at(bodies, row.body_a, row.body_b)?;
    let state_a = body_a.state();
    let state_b = body_b.state();
    let value = row.linear_a.dot(state_a.linear_velocity())
        + row.angular_a.dot(state_a.angular_velocity())
        + row.linear_b.dot(state_b.linear_velocity())
        + row.angular_b.dot(state_b.angular_velocity());
    value
        .is_finite()
        .then_some(value)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn apply_position_row<B: SolverBodyAccess + ?Sized>(
    bodies: &mut B,
    row: ConstraintRow,
    multiplier: f64,
    masses: Option<[SolverMass; 2]>,
) -> Result<(), CollisionError> {
    apply_position_impulse_at(
        bodies,
        row.body_a,
        constraint::Jacobian {
            linear: row.linear_a * multiplier,
            angular: row.angular_a * multiplier,
        },
        masses.map(|m| m[0]),
    )?;
    apply_position_impulse_at(
        bodies,
        row.body_b,
        constraint::Jacobian {
            linear: row.linear_b * multiplier,
            angular: row.angular_b * multiplier,
        },
        masses.map(|m| m[1]),
    )
}

fn apply_velocity_row<B: SolverBodyAccess + ?Sized>(
    bodies: &mut B,
    row: ConstraintRow,
    impulse: f64,
    masses: Option<[SolverMass; 2]>,
) -> Result<(), CollisionError> {
    let impulses = [
        constraint::Jacobian {
            linear: row.linear_a * impulse,
            angular: row.angular_a * impulse,
        },
        constraint::Jacobian {
            linear: row.linear_b * impulse,
            angular: row.angular_b * impulse,
        },
    ];
    apply_velocity_impulse_at(bodies, row.body_a, impulses[0], masses.map(|m| m[0]))?;
    apply_velocity_impulse_at(bodies, row.body_b, impulses[1], masses.map(|m| m[1]))
}

fn apply_position_impulse_at<B: SolverBodyAccess + ?Sized>(
    bodies: &mut B,
    body_index: usize,
    impulse: constraint::Jacobian,
    mass: Option<SolverMass>,
) -> Result<(), CollisionError> {
    let is_dynamic = bodies
        .get(body_index)
        .ok_or(CollisionError::InvalidContact)?
        .body()
        .physics()
        .body_type()
        == crate::rigid_body::BodyType::Dynamic;
    if is_dynamic {
        let body = bodies
            .get_dynamic_mut(body_index)
            .ok_or(CollisionError::InvalidContact)?;
        if let Some(mass) = mass {
            constraint::apply_generalized_position_impulse_cached(body.body_mut(), mass, impulse)
        } else {
            constraint::apply_generalized_position_impulse(body.body_mut(), impulse)
        }
        .map_err(map_constraint_error)?;
    }
    Ok(())
}

fn apply_velocity_impulse_at<B: SolverBodyAccess + ?Sized>(
    bodies: &mut B,
    body_index: usize,
    impulse: constraint::Jacobian,
    mass: Option<SolverMass>,
) -> Result<(), CollisionError> {
    let is_dynamic = bodies
        .get(body_index)
        .ok_or(CollisionError::InvalidContact)?
        .body()
        .physics()
        .body_type()
        == crate::rigid_body::BodyType::Dynamic;
    if is_dynamic {
        let body = bodies
            .get_dynamic_mut(body_index)
            .ok_or(CollisionError::InvalidContact)?;
        if let Some(mass) = mass {
            constraint::apply_generalized_impulse_cached(body.body_mut(), mass, impulse)
        } else {
            constraint::apply_generalized_impulse(body.body_mut(), impulse)
        }
        .map_err(map_constraint_error)?;
    }
    Ok(())
}

fn resolve_solver_mass_pair<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    slots: &mut [SolverMassSlot],
    body_a: usize,
    body_b: usize,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<[SolverMass; 2], CollisionError> {
    resolve_solver_mass_pair_with_status(bodies, slots, body_a, body_b, cache_stats)
        .map(|(masses, _)| masses)
}

fn resolve_solver_mass_pair_with_status<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    slots: &mut [SolverMassSlot],
    body_a: usize,
    body_b: usize,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<([SolverMass; 2], [SolverMassCacheStatus; 2]), CollisionError> {
    let (solver_body_a, solver_body_b) = solver_bodies_at(bodies, body_a, body_b)?;
    let (slot_a, slot_b) = solver_mass_slots_at_mut(slots, body_a, body_b)?;
    let (mass_a, status_a) = slot_a.resolve(solver_body_a.body())?;
    cache_stats.record(status_a);
    let (mass_b, status_b) = slot_b.resolve(solver_body_b.body())?;
    cache_stats.record(status_b);
    Ok(([mass_a, mass_b], [status_a, status_b]))
}

fn invalidate_solver_mass_pair(
    slots: &mut [SolverMassSlot],
    body_a: usize,
    body_b: usize,
) -> Result<(), CollisionError> {
    let (slot_a, slot_b) = solver_mass_slots_at_mut(slots, body_a, body_b)?;
    slot_a.invalidate_pose();
    slot_b.invalidate_pose();
    Ok(())
}

fn solver_mass_slots_at_mut(
    slots: &mut [SolverMassSlot],
    body_a: usize,
    body_b: usize,
) -> Result<(&mut SolverMassSlot, &mut SolverMassSlot), CollisionError> {
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    if body_a < body_b {
        let (left, right) = slots.split_at_mut(body_b);
        Ok((
            left.get_mut(body_a).ok_or(CollisionError::InvalidContact)?,
            right.first_mut().ok_or(CollisionError::InvalidContact)?,
        ))
    } else {
        let (left, right) = slots.split_at_mut(body_a);
        Ok((
            right.first_mut().ok_or(CollisionError::InvalidContact)?,
            left.get_mut(body_b).ok_or(CollisionError::InvalidContact)?,
        ))
    }
}

fn solver_bodies_at<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    body_a: usize,
    body_b: usize,
) -> Result<(&SolverBody, &SolverBody), CollisionError> {
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    Ok((
        bodies.get(body_a).ok_or(CollisionError::InvalidContact)?,
        bodies.get(body_b).ok_or(CollisionError::InvalidContact)?,
    ))
}

fn map_constraint_error(error: constraint::ConstraintError) -> CollisionError {
    match error {
        constraint::ConstraintError::InvalidMass => CollisionError::InvalidContact,
        constraint::ConstraintError::ArithmeticOverflow => CollisionError::ArithmeticOverflow,
        constraint::ConstraintError::NonConvergent => CollisionError::NonConvergent,
    }
}

pub(super) fn map_rigid_body_error(error: crate::rigid_body::BodyError) -> CollisionError {
    match error {
        crate::rigid_body::BodyError::ArithmeticOverflow
        | crate::rigid_body::BodyError::NonFiniteValue => CollisionError::ArithmeticOverflow,
        crate::rigid_body::BodyError::InvalidInertiaTensor => CollisionError::NonConvergent,
        crate::rigid_body::BodyError::BodyMustBeDynamic
        | crate::rigid_body::BodyError::StaticBodyCannotHaveVelocity => {
            CollisionError::InvalidContact
        }
    }
}

#[cfg(test)]
#[path = "solver_tests.rs"]
mod tests;

#[cfg(test)]
mod residual_tests {
    use super::{JointResidual, projected_kkt_residual};
    use crate::joint::JointTolerance;

    #[test]
    fn convergence_rejects_non_finite_residuals_and_keeps_units_independent() {
        let tolerance = JointTolerance {
            linear_m: 1.0e-3,
            angular_rad: 1.0e-2,
            velocity_mps: 1.0e-2,
            angular_velocity_radps: 1.0e-1,
        };
        let residual = JointResidual {
            max_linear_m: 5.0e-4,
            max_angular_rad: 5.0e-3,
            max_velocity_mps: 5.0e-3,
            max_angular_velocity_radps: 5.0e-2,
        };
        assert!(residual.converged(tolerance));
        assert!(
            !JointResidual {
                max_linear_m: 2.0e-3,
                ..residual
            }
            .converged(tolerance)
        );
        assert!(
            !JointResidual {
                max_angular_velocity_radps: f64::INFINITY,
                ..residual
            }
            .converged(tolerance)
        );
        assert!(residual.position_converged(tolerance));
        assert!(residual.velocity_converged(tolerance));
        assert!(
            JointResidual {
                max_velocity_mps: 1.0,
                ..residual
            }
            .position_converged(tolerance)
        );
        assert!(
            !JointResidual {
                max_linear_m: 1.0,
                ..residual
            }
            .position_converged(tolerance)
        );
        assert!(
            !JointResidual {
                max_velocity_mps: 1.0,
                ..residual
            }
            .velocity_converged(tolerance)
        );
    }

    #[test]
    fn projected_residual_obeys_lower_upper_and_bilateral_kkt_conditions() {
        assert_eq!(projected_kkt_residual(1.0, 0.0, 0.0, f64::INFINITY), 0.0);
        assert_eq!(projected_kkt_residual(-1.0, 0.0, 0.0, f64::INFINITY), 1.0);
        assert_eq!(
            projected_kkt_residual(-1.0, 0.0, f64::NEG_INFINITY, 0.0),
            0.0
        );
        assert_eq!(
            projected_kkt_residual(1.0, 0.0, f64::NEG_INFINITY, 0.0),
            1.0
        );
        assert_eq!(projected_kkt_residual(-2.0, 0.0, -1.0, 1.0), 2.0);
        assert_eq!(projected_kkt_residual(2.0, 1.0, 1.0, 1.0), 0.0);
        assert_eq!(projected_kkt_residual(-2.0, 1.0, 1.0, 1.0), 0.0);
        assert_eq!(projected_kkt_residual(2.0, 0.0, 1.0, 1.0), 1.0);
    }
}
