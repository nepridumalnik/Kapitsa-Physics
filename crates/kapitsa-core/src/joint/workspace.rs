use super::{
    JointBreakEvent, ReactionMode,
    reaction::wrench_from_impulse,
    row::ConstraintRow,
    solver::{FixedBlockVelocityOperators, FixedJointGeometryCache, JointConstraint},
    warm_start::JointImpulseCache,
};
use crate::collision::CollisionError;
use crate::rigid_body::solver_mass::SolverMassSlot;
use crate::rigid_body::{BodyError, BodyType};
use kapitsa_math::Vec3;

type FixedRowBlocksMut<'a> = (
    &'a mut [ConstraintRow; 3],
    &'a mut [ConstraintRow; 3],
    &'a mut [SolverMassSlot],
);

#[derive(Clone, Copy, Debug)]
enum ReactionSide {
    A,
    B,
}

#[derive(Clone, Copy, Debug)]
struct JointReaction {
    force_impulse: Vec3,
    torque_com_impulse: Vec3,
    com_world: Vec3,
    anchor_world: Vec3,
    side: ReactionSide,
}

/// Reusable rows for one solver island, retained between physics steps.
#[derive(Default)]
pub(crate) struct JointWorkspace {
    rows: Vec<ConstraintRow>,
    row_joint_indices: Vec<usize>,
    joint_ids: Vec<Option<crate::world::JointId>>,
    preparing_rows: Vec<ConstraintRow>,
    solver_mass_slots: Vec<SolverMassSlot>,
    direct_velocity_scratch: super::direct::DirectVelocityScratch,
    preparing_joint_indices: Vec<usize>,
    preparing_joint_ids: Vec<Option<crate::world::JointId>>,
    fixed_geometry: Vec<Option<FixedJointGeometryCache>>,
    fixed_velocity_operators: Vec<FixedBlockVelocityOperators>,
    prepared_rows: usize,
    active_joint_index: usize,
    active_joint_id: Option<crate::world::JointId>,
    sequential_joint_prepare: bool,
    diagnostic_prepare: bool,
    previous_row_start: usize,
    previous_row_end: usize,
    reactions: Vec<Option<JointReaction>>,
    break_events: Vec<JointBreakEvent>,
    #[cfg(feature = "bench-internals")]
    metrics: super::diagnostics::JointSolverMetrics,
    #[cfg(feature = "bench-internals")]
    geometry_residual: super::diagnostics::JointGeometryResidual,
    #[cfg(feature = "bench-internals")]
    velocity_pcg_scratch: super::solver::VelocityPcgScratch,
    residual: super::solver::JointResidual,
    #[cfg(test)]
    solver_calls: Vec<JointSolverCall>,
    #[cfg(test)]
    fixed_geometry_recomputes: usize,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum JointSolverCall {
    AllRowsPrepared,
    JointRowsPrepared(usize),
    JointRowsCommitted(usize),
    PositionSweep {
        preparing_rows: bool,
        row_count: usize,
    },
    VelocitySweep {
        preparing_rows: bool,
        row_count: usize,
    },
}

impl JointWorkspace {
    pub(crate) const fn new() -> Self {
        Self {
            rows: Vec::new(),
            row_joint_indices: Vec::new(),
            joint_ids: Vec::new(),
            preparing_rows: Vec::new(),
            solver_mass_slots: Vec::new(),
            direct_velocity_scratch: super::direct::DirectVelocityScratch::new(),
            preparing_joint_indices: Vec::new(),
            preparing_joint_ids: Vec::new(),
            fixed_geometry: Vec::new(),
            fixed_velocity_operators: Vec::new(),
            prepared_rows: 0,
            active_joint_index: 0,
            active_joint_id: None,
            sequential_joint_prepare: false,
            diagnostic_prepare: false,
            previous_row_start: 0,
            previous_row_end: 0,
            reactions: Vec::new(),
            break_events: Vec::new(),
            #[cfg(feature = "bench-internals")]
            metrics: super::diagnostics::JointSolverMetrics::ZERO,
            #[cfg(feature = "bench-internals")]
            geometry_residual: super::diagnostics::JointGeometryResidual {
                anchor_error_m: 0.0,
                frame_error_rad: 0.0,
            },
            #[cfg(feature = "bench-internals")]
            velocity_pcg_scratch: super::solver::VelocityPcgScratch::new(),
            residual: super::solver::JointResidual::ZERO,
            #[cfg(test)]
            solver_calls: Vec::new(),
            #[cfg(test)]
            fixed_geometry_recomputes: 0,
        }
    }

    pub(crate) fn begin_step(&mut self) {
        self.prepared_rows = 0;
        self.active_joint_index = 0;
        self.active_joint_id = None;
        self.sequential_joint_prepare = false;
        self.diagnostic_prepare = false;
        self.previous_row_start = 0;
        self.previous_row_end = 0;
        self.reactions.clear();
        self.break_events.clear();
        #[cfg(test)]
        self.solver_calls.clear();
        #[cfg(feature = "bench-internals")]
        {
            self.metrics = super::diagnostics::JointSolverMetrics::ZERO;
            self.geometry_residual = super::diagnostics::JointGeometryResidual::default();
            self.velocity_pcg_scratch.begin_step();
        }
        self.residual = super::solver::JointResidual::ZERO;
        for row in &mut self.rows {
            row.accumulated_multiplier = 0.0;
            row.accumulated_impulse = 0.0;
        }
        // Velocity block operators depend on row Jacobians and masses, which a
        // new step may change, so drop them; the first velocity sweep rebuilds.
        for operators in &mut self.fixed_velocity_operators {
            *operators = FixedBlockVelocityOperators::default();
        }
    }

    pub(crate) fn clear(&mut self) {
        self.rows.clear();
        self.row_joint_indices.clear();
        self.joint_ids.clear();
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
        self.fixed_geometry.clear();
        self.fixed_velocity_operators.clear();
        self.prepared_rows = 0;
        self.active_joint_index = 0;
        self.active_joint_id = None;
        self.sequential_joint_prepare = false;
        self.diagnostic_prepare = false;
        self.previous_row_start = 0;
        self.previous_row_end = 0;
        self.reactions.clear();
        self.break_events.clear();
        #[cfg(test)]
        self.solver_calls.clear();
        #[cfg(feature = "bench-internals")]
        {
            self.metrics = super::diagnostics::JointSolverMetrics::ZERO;
            self.geometry_residual = super::diagnostics::JointGeometryResidual::default();
        }
        self.residual = super::solver::JointResidual::ZERO;
    }

    /// Starts a diagnostic-only preparation. Rows are built in the scratch
    /// buffer and geometry caching is disabled so no solver state is touched.
    pub(crate) fn begin_diagnostic_prepare(&mut self) {
        self.begin_prepare();
        self.diagnostic_prepare = true;
    }

    /// Discards the diagnostic scratch rows without committing them.
    pub(crate) fn abandon_prepare(&mut self) {
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
        self.sequential_joint_prepare = false;
        self.diagnostic_prepare = false;
    }

    pub(crate) const fn diagnostic_prepare(&self) -> bool {
        self.diagnostic_prepare
    }

    pub(crate) fn preparing_rows(&self) -> &[ConstraintRow] {
        &self.preparing_rows
    }

    pub(crate) fn preparing_row_joint_indices(&self) -> &[usize] {
        &self.preparing_joint_indices
    }

    pub(crate) fn begin_prepare(&mut self) {
        self.prepared_rows = 0;
        self.active_joint_index = 0;
        self.active_joint_id = None;
        self.sequential_joint_prepare = false;
        self.previous_row_start = 0;
        self.previous_row_end = self.rows.len();
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
    }

    pub(crate) fn push(&mut self, mut row: ConstraintRow) {
        let previous = (self.previous_row_start..self.previous_row_end).find_map(|index| {
            (self.row_joint_indices.get(index) == Some(&self.active_joint_index)
                && self
                    .joint_ids
                    .get(self.active_joint_index)
                    .copied()
                    .flatten()
                    == self.active_joint_id
                && self
                    .rows
                    .get(index)
                    .is_some_and(|previous| previous.key == row.key))
            .then(|| self.rows[index])
        });
        if let Some(previous) = previous {
            row.accumulated_multiplier = previous.accumulated_multiplier;
            row.accumulated_impulse = previous.accumulated_impulse;
        }
        self.preparing_rows.push(row);
        if !self.sequential_joint_prepare {
            self.preparing_joint_indices.push(self.active_joint_index);
        }
        self.prepared_rows += 1;
    }

    pub(crate) fn finish_prepare(&mut self) {
        #[cfg(feature = "bench-internals")]
        {
            self.metrics.prepared_rows = self
                .metrics
                .prepared_rows
                .saturating_add(self.preparing_rows.len() as u64);
        }
        std::mem::swap(&mut self.rows, &mut self.preparing_rows);
        std::mem::swap(
            &mut self.row_joint_indices,
            &mut self.preparing_joint_indices,
        );
        std::mem::swap(&mut self.joint_ids, &mut self.preparing_joint_ids);
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
        self.sequential_joint_prepare = false;
    }

    pub(crate) fn begin_joint_prepare(
        &mut self,
        joint_index: usize,
        joint_id: crate::world::JointId,
    ) {
        let start = self
            .row_joint_indices
            .partition_point(|&owner| owner < joint_index);
        let end = self
            .row_joint_indices
            .partition_point(|&owner| owner <= joint_index);
        self.previous_row_start = start;
        self.previous_row_end = end;
        self.prepared_rows = 0;
        self.active_joint_index = joint_index;
        self.active_joint_id = Some(joint_id);
        self.sequential_joint_prepare = true;
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
    }

    pub(crate) fn ensure_solver_mass_slots(
        &mut self,
        body_count: usize,
    ) -> Result<(), CollisionError> {
        if body_count > self.solver_mass_slots.len() {
            self.solver_mass_slots
                .try_reserve(body_count - self.solver_mass_slots.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.solver_mass_slots
                .resize_with(body_count, SolverMassSlot::default);
        } else {
            self.solver_mass_slots.truncate(body_count);
        }
        Ok(())
    }

    pub(super) fn direct_velocity_buffers_mut(
        &mut self,
    ) -> (
        &mut [ConstraintRow],
        &mut [SolverMassSlot],
        &mut super::direct::DirectVelocityScratch,
    ) {
        (
            &mut self.rows,
            &mut self.solver_mass_slots,
            &mut self.direct_velocity_scratch,
        )
    }

    pub(crate) fn solver_buffers_mut(
        &mut self,
        preparing_rows: bool,
    ) -> (&mut [ConstraintRow], &mut [SolverMassSlot]) {
        let rows = if preparing_rows {
            &mut self.preparing_rows
        } else {
            &mut self.rows
        };
        (rows, &mut self.solver_mass_slots)
    }

    /// Grows the per-joint velocity operator cache so `joint_index` is addressable.
    pub(crate) fn ensure_fixed_velocity_operators(
        &mut self,
        joint_index: usize,
    ) -> Result<(), CollisionError> {
        if joint_index >= self.fixed_velocity_operators.len() {
            self.fixed_velocity_operators
                .try_reserve(joint_index + 1 - self.fixed_velocity_operators.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.fixed_velocity_operators
                .resize_with(joint_index + 1, FixedBlockVelocityOperators::default);
        }
        Ok(())
    }

    /// Rows, mass slots, and the active joint's velocity operator cache. The
    /// caller must have sized the cache with `ensure_fixed_velocity_operators`.
    pub(crate) fn solver_buffers_and_velocity_operators_mut(
        &mut self,
        preparing_rows: bool,
    ) -> (
        &mut [ConstraintRow],
        &mut [SolverMassSlot],
        &mut FixedBlockVelocityOperators,
    ) {
        let rows = if preparing_rows {
            &mut self.preparing_rows
        } else {
            &mut self.rows
        };
        let operators = &mut self.fixed_velocity_operators[self.active_joint_index];
        (rows, &mut self.solver_mass_slots, operators)
    }

    pub(crate) fn preparing_fixed_row_blocks_mut(
        &mut self,
    ) -> Result<FixedRowBlocksMut<'_>, CollisionError> {
        let (rows, slots) = (&mut self.preparing_rows, &mut self.solver_mass_slots);
        let rows: &mut [ConstraintRow; 6] = rows
            .as_mut_slice()
            .try_into()
            .map_err(|_| CollisionError::InvalidContact)?;
        let (anchor_rows, frame_rows) = rows.split_at_mut(3);
        let anchor_rows = anchor_rows
            .try_into()
            .map_err(|_| CollisionError::InvalidContact)?;
        let frame_rows = frame_rows
            .try_into()
            .map_err(|_| CollisionError::InvalidContact)?;
        Ok((anchor_rows, frame_rows, slots))
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn velocity_pcg_rows_and_scratch_mut(
        &mut self,
    ) -> (
        &mut [ConstraintRow],
        &[usize],
        &mut super::solver::VelocityPcgScratch,
    ) {
        (
            &mut self.rows,
            &self.row_joint_indices,
            &mut self.velocity_pcg_scratch,
        )
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn position_pcg_rows_slots_and_scratch_mut(
        &mut self,
    ) -> (
        &mut [ConstraintRow],
        &mut [SolverMassSlot],
        &mut super::solver::VelocityPcgScratch,
    ) {
        (
            &mut self.rows,
            &mut self.solver_mass_slots,
            &mut self.velocity_pcg_scratch,
        )
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) const fn velocity_pcg_rows_prepared(&self) -> bool {
        self.velocity_pcg_scratch.rows_prepared
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn mark_velocity_pcg_rows_prepared(&mut self) {
        self.velocity_pcg_scratch.rows_prepared = true;
    }

    pub(crate) fn stage_impulses(
        &self,
        cache: &mut JointImpulseCache,
        joints: &[JointConstraint],
        dt: f64,
    ) -> Result<(), CollisionError> {
        let mut row_start = 0;
        for (joint_index, joint) in joints.iter().copied().enumerate() {
            if self
                .row_joint_indices
                .get(row_start)
                .is_some_and(|owner| *owner < joint_index)
            {
                return Err(CollisionError::InvalidContact);
            }
            let row_end = self
                .row_joint_indices
                .partition_point(|owner| *owner <= joint_index);
            let rows = self
                .rows
                .get(row_start..row_end)
                .ok_or(CollisionError::InvalidContact)?;
            cache.stage_joint(joint.id, joint.joint, dt, rows)?;
            row_start = row_end;
        }
        if row_start != self.rows.len() {
            return Err(CollisionError::InvalidContact);
        }
        Ok(())
    }

    pub(crate) fn solver_rows_len(&self, preparing_rows: bool) -> usize {
        if preparing_rows {
            self.preparing_rows.len()
        } else {
            self.rows.len()
        }
    }

    pub(crate) const fn active_joint_index(&self) -> usize {
        self.active_joint_index
    }

    /// Selects the joint whose cached block operators a prepared sweep uses.
    pub(crate) const fn set_active_joint_index(&mut self, index: usize) {
        self.active_joint_index = index;
    }

    /// Row range `[start, end)` committed for `joint_index`.
    pub(crate) fn joint_row_range(&self, joint_index: usize) -> (usize, usize) {
        let start = self
            .row_joint_indices
            .partition_point(|&owner| owner < joint_index);
        let end = self
            .row_joint_indices
            .partition_point(|&owner| owner <= joint_index);
        (start, end)
    }

    pub(crate) fn committed_joint_id(&self, joint_index: usize) -> Option<crate::world::JointId> {
        self.joint_ids.get(joint_index).copied().flatten()
    }

    /// Owner joint index of a committed row.
    pub(crate) fn row_owner(&self, index: usize) -> Option<usize> {
        self.row_joint_indices.get(index).copied()
    }

    /// Mutable access to a committed row range together with the mass slots.
    pub(crate) fn row_range_and_slots_mut(
        &mut self,
        start: usize,
        end: usize,
    ) -> Result<(&mut [ConstraintRow], &mut [SolverMassSlot]), CollisionError> {
        let rows = self
            .rows
            .get_mut(start..end)
            .ok_or(CollisionError::InvalidContact)?;
        Ok((rows, &mut self.solver_mass_slots))
    }

    pub(crate) fn fixed_geometry(&self, joint_index: usize) -> Option<FixedJointGeometryCache> {
        self.fixed_geometry.get(joint_index).copied().flatten()
    }

    pub(crate) fn set_fixed_geometry(
        &mut self,
        joint_index: usize,
        geometry: FixedJointGeometryCache,
    ) -> Result<(), CollisionError> {
        if joint_index >= self.fixed_geometry.len() {
            self.fixed_geometry
                .try_reserve(joint_index + 1 - self.fixed_geometry.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.fixed_geometry.resize(joint_index + 1, None);
        }
        self.fixed_geometry[joint_index] = Some(geometry);
        #[cfg(test)]
        {
            self.fixed_geometry_recomputes = self.fixed_geometry_recomputes.saturating_add(1);
        }
        Ok(())
    }

    #[cfg(test)]
    pub(crate) const fn fixed_geometry_recomputes(&self) -> usize {
        self.fixed_geometry_recomputes
    }

    #[cfg(test)]
    pub(crate) fn fixed_block_velocity_recomputes(&self) -> usize {
        self.fixed_velocity_operators
            .iter()
            .map(FixedBlockVelocityOperators::recomputes)
            .sum()
    }

    #[cfg(test)]
    pub(crate) fn record_solver_call(&mut self, call: JointSolverCall) {
        self.solver_calls.push(call);
    }

    #[cfg(test)]
    pub(crate) fn solver_calls(&self) -> &[JointSolverCall] {
        &self.solver_calls
    }

    pub(crate) fn finish_joint_prepare(&mut self) -> Result<(), CollisionError> {
        let joint_id = self.active_joint_id.ok_or(CollisionError::InvalidContact)?;
        let new_count = self.preparing_rows.len();
        let old_count = self.previous_row_end - self.previous_row_start;
        if new_count == old_count {
            self.rows[self.previous_row_start..self.previous_row_end]
                .copy_from_slice(&self.preparing_rows);
            let owners =
                &mut self.row_joint_indices[self.previous_row_start..self.previous_row_end];
            if owners.first() != Some(&self.active_joint_index) {
                owners.fill(self.active_joint_index);
            }
            self.update_joint_id(joint_id)?;
            #[cfg(feature = "bench-internals")]
            {
                self.metrics.prepared_rows =
                    self.metrics.prepared_rows.saturating_add(new_count as u64);
            }
            self.preparing_rows.clear();
            self.preparing_joint_indices.clear();
            self.preparing_joint_ids.clear();
            self.sequential_joint_prepare = false;
            return Ok(());
        }
        if new_count > old_count {
            let additional = new_count - old_count;
            self.rows
                .try_reserve(additional)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.row_joint_indices
                .try_reserve(additional)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        self.rows.splice(
            self.previous_row_start..self.previous_row_end,
            self.preparing_rows.iter().copied(),
        );
        self.row_joint_indices.splice(
            self.previous_row_start..self.previous_row_end,
            std::iter::repeat_n(self.active_joint_index, new_count),
        );
        self.update_joint_id(joint_id)?;
        #[cfg(feature = "bench-internals")]
        {
            self.metrics.prepared_rows =
                self.metrics.prepared_rows.saturating_add(new_count as u64);
        }
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
        self.sequential_joint_prepare = false;
        Ok(())
    }

    pub(crate) fn rows(&self) -> &[ConstraintRow] {
        &self.rows
    }

    pub(crate) fn rows_mut(&mut self) -> &mut [ConstraintRow] {
        &mut self.rows
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_position_iteration(&mut self) {
        self.metrics.position_sweeps = self.metrics.position_sweeps.saturating_add(1);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_velocity_iteration(&mut self) {
        self.metrics.velocity_sweeps = self.metrics.velocity_sweeps.saturating_add(1);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_color_batches(&mut self, batches: usize) {
        self.metrics.color_batches_processed = self
            .metrics
            .color_batches_processed
            .saturating_add(batches as u64);
    }

    pub(crate) fn merge_colored_joint_from(
        &mut self,
        source: &Self,
        source_joint_index: usize,
        destination_joint_index: usize,
        joint_id: crate::world::JointId,
        body_indices: [usize; 2],
    ) -> Result<(), CollisionError> {
        let start = source
            .row_joint_indices
            .partition_point(|&owner| owner < source_joint_index);
        let end = source
            .row_joint_indices
            .partition_point(|&owner| owner <= source_joint_index);
        let rows = source
            .rows
            .get(start..end)
            .ok_or(CollisionError::InvalidContact)?;
        #[cfg(feature = "bench-internals")]
        let prepared_rows_before_merge = self.metrics.prepared_rows;
        self.begin_joint_prepare(destination_joint_index, joint_id);
        self.preparing_rows
            .try_reserve(rows.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for row in rows.iter().copied() {
            let mut row = row;
            row.body_a = body_indices[0];
            row.body_b = body_indices[1];
            self.preparing_rows.push(row);
        }
        self.finish_joint_prepare()?;
        #[cfg(feature = "bench-internals")]
        {
            self.metrics.prepared_rows = prepared_rows_before_merge;
        }
        Ok(())
    }

    pub(crate) fn accumulate_colored_metrics_from(&mut self, source: &Self) {
        #[cfg(feature = "bench-internals")]
        self.metrics.accumulate(source.metrics);
        #[cfg(not(feature = "bench-internals"))]
        let _ = source;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_velocity_pcg_result(&mut self, result: super::solver::VelocityPcgResult) {
        self.metrics.velocity_pcg_iterations = self
            .metrics
            .velocity_pcg_iterations
            .saturating_add(result.iterations as u64);
        if result.converged {
            self.metrics.velocity_pcg_converged_calls =
                self.metrics.velocity_pcg_converged_calls.saturating_add(1);
        } else {
            self.metrics.velocity_pcg_iteration_limit_calls = self
                .metrics
                .velocity_pcg_iteration_limit_calls
                .saturating_add(1);
        }
        self.metrics.velocity_pcg_linear_residual_mps = self
            .metrics
            .velocity_pcg_linear_residual_mps
            .max(result.linear_residual_mps);
        self.metrics.velocity_pcg_angular_residual_radps = self
            .metrics
            .velocity_pcg_angular_residual_radps
            .max(result.angular_residual_radps);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_constraint_coloring(
        &mut self,
        elapsed: std::time::Duration,
        constraint_count: usize,
        color_count: usize,
        rebuilt: bool,
    ) {
        self.metrics.constraint_coloring_prepare_time += elapsed;
        self.metrics.constraint_coloring_constraints = self
            .metrics
            .constraint_coloring_constraints
            .saturating_add(constraint_count as u64);
        self.metrics.max_colors_per_island = self
            .metrics
            .max_colors_per_island
            .max(color_count.min(u32::MAX as usize) as u32);
        if rebuilt {
            self.metrics.constraint_coloring_rebuilds =
                self.metrics.constraint_coloring_rebuilds.saturating_add(1);
        } else {
            self.metrics.constraint_coloring_cache_hits = self
                .metrics
                .constraint_coloring_cache_hits
                .saturating_add(1);
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_position_pcg_result(&mut self, result: super::solver::PositionPcgResult) {
        self.metrics.position_pcg_iterations = self
            .metrics
            .position_pcg_iterations
            .saturating_add(result.iterations as u64);
        if result.converged {
            self.metrics.position_pcg_converged_calls =
                self.metrics.position_pcg_converged_calls.saturating_add(1);
        } else {
            self.metrics.position_pcg_iteration_limit_calls = self
                .metrics
                .position_pcg_iteration_limit_calls
                .saturating_add(1);
        }
        self.metrics.position_pcg_linear_residual_m = self
            .metrics
            .position_pcg_linear_residual_m
            .max(result.linear_residual_m);
        self.metrics.position_pcg_angular_residual_rad = self
            .metrics
            .position_pcg_angular_residual_rad
            .max(result.angular_residual_rad);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_velocity_pcg_timings(
        &mut self,
        system_prepare: std::time::Duration,
        matrix_vector: std::time::Duration,
        preconditioner: std::time::Duration,
        reduction: std::time::Duration,
    ) {
        self.metrics.velocity_pcg_system_prepare_time += system_prepare;
        self.metrics.velocity_pcg_matrix_vector_time += matrix_vector;
        self.metrics.velocity_pcg_preconditioner_time += preconditioner;
        self.metrics.velocity_pcg_reduction_time += reduction;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_position_pcg_timings(
        &mut self,
        system_prepare: std::time::Duration,
        matrix_vector: std::time::Duration,
        preconditioner: std::time::Duration,
        reduction: std::time::Duration,
    ) {
        self.metrics.position_pcg_system_prepare_time += system_prepare;
        self.metrics.position_pcg_matrix_vector_time += matrix_vector;
        self.metrics.position_pcg_preconditioner_time += preconditioner;
        self.metrics.position_pcg_reduction_time += reduction;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_energy(
        &mut self,
        stage: super::diagnostics::EnergyStage,
        energy: super::diagnostics::EnergyComponents,
    ) {
        let target = match stage {
            super::diagnostics::EnergyStage::BeforeVelocityPcg => {
                &mut self.metrics.energy_before_velocity_pcg
            }
            super::diagnostics::EnergyStage::AfterVelocityPcg => {
                &mut self.metrics.energy_after_velocity_pcg
            }
            super::diagnostics::EnergyStage::VelocityPcgDelta => {
                &mut self.metrics.velocity_pcg_energy_delta
            }
            super::diagnostics::EnergyStage::AfterVelocitySolver => {
                &mut self.metrics.energy_after_velocity_solver
            }
            super::diagnostics::EnergyStage::BeforePositionPcg => {
                &mut self.metrics.energy_before_position_pcg
            }
            super::diagnostics::EnergyStage::AfterPositionPcg => {
                &mut self.metrics.energy_after_position_pcg
            }
            super::diagnostics::EnergyStage::PositionPcgDelta => {
                &mut self.metrics.position_pcg_energy_delta
            }
            super::diagnostics::EnergyStage::AfterPositionSolver => {
                &mut self.metrics.energy_after_position_solver
            }
        };
        if matches!(
            stage,
            super::diagnostics::EnergyStage::VelocityPcgDelta
                | super::diagnostics::EnergyStage::PositionPcgDelta
        ) {
            target.add_assign(energy);
        } else {
            *target = energy;
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_energy_delta(
        &mut self,
        stage: super::diagnostics::EnergyStage,
        before: super::diagnostics::EnergyComponents,
        after: super::diagnostics::EnergyComponents,
    ) {
        self.record_energy(stage, after.delta(before));
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_velocity_projection_work(
        &mut self,
        impulse_work_j: f64,
        quadratic_work_j: f64,
    ) {
        self.metrics.velocity_projection_impulse_work_j += impulse_work_j;
        self.metrics.velocity_projection_quadratic_work_j += quadratic_work_j;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_scalar_rows_solved(&mut self, rows: usize) {
        self.metrics.scalar_rows_solved =
            self.metrics.scalar_rows_solved.saturating_add(rows as u64);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_block_rows_solved(&mut self, rows: usize) {
        self.metrics.block_rows_solved = self.metrics.block_rows_solved.saturating_add(rows as u64);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_block_fallback_rows(&mut self, rows: usize) {
        self.metrics.block_fallback_rows =
            self.metrics.block_fallback_rows.saturating_add(rows as u64);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_solver_mass_cache(
        &mut self,
        hits: u64,
        misses: u64,
        inverse_inertia_updates: u64,
    ) {
        self.metrics.solver_mass_cache_hits =
            self.metrics.solver_mass_cache_hits.saturating_add(hits);
        self.metrics.solver_mass_cache_misses =
            self.metrics.solver_mass_cache_misses.saturating_add(misses);
        self.metrics.inverse_inertia_updates = self
            .metrics
            .inverse_inertia_updates
            .saturating_add(inverse_inertia_updates);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_incremental_rows_prepared(&mut self, count: usize) {
        self.metrics.prepared_rows = self.metrics.prepared_rows.saturating_add(count as u64);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_joint_row_preparation_time(&mut self, elapsed: std::time::Duration) {
        self.metrics.joint_row_preparation_time += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_velocity_warm_starts(&mut self, count: usize) {
        self.metrics.velocity_warm_starts = self
            .metrics
            .velocity_warm_starts
            .saturating_add(count as u64);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn metrics(&self) -> super::diagnostics::JointSolverMetrics {
        self.metrics
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn row_joint_indices(&self) -> &[usize] {
        &self.row_joint_indices
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn set_geometry_residual(
        &mut self,
        residual: super::diagnostics::JointGeometryResidual,
    ) {
        self.geometry_residual = residual;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_pre_velocity_geometry(&mut self) {
        self.metrics.pre_velocity_anchor_error_m = self.geometry_residual.anchor_error_m;
        self.metrics.pre_velocity_frame_error_rad = self.geometry_residual.frame_error_rad;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_post_velocity_geometry(&mut self) {
        self.metrics.post_velocity_anchor_error_m = self.geometry_residual.anchor_error_m;
        self.metrics.post_velocity_frame_error_rad = self.geometry_residual.frame_error_rad;
        self.metrics.post_velocity_linear_residual_mps = self.residual.max_velocity_mps;
        self.metrics.post_velocity_angular_residual_radps =
            self.residual.max_angular_velocity_radps;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_first_velocity_residual(&mut self) {
        self.metrics.first_velocity_linear_residual_mps = self.residual.max_velocity_mps;
        self.metrics.first_velocity_angular_residual_radps =
            self.residual.max_angular_velocity_radps;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_pre_joint_velocity_residual(&mut self) {
        self.metrics.pre_joint_velocity_linear_residual_mps = self.residual.max_velocity_mps;
        self.metrics.pre_joint_velocity_angular_residual_radps =
            self.residual.max_angular_velocity_radps;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_initial_velocity_residual(&mut self) {
        self.metrics.initial_velocity_linear_residual_mps = self.residual.max_velocity_mps;
        self.metrics.initial_velocity_angular_residual_radps =
            self.residual.max_angular_velocity_radps;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_pre_joint_geometry(&mut self) {
        self.metrics.pre_joint_anchor_error_m = self.geometry_residual.anchor_error_m;
        self.metrics.pre_joint_frame_error_rad = self.geometry_residual.frame_error_rad;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_post_joint_geometry(&mut self, first_sweep: bool) {
        self.metrics.post_joint_anchor_error_m = self.geometry_residual.anchor_error_m;
        self.metrics.post_joint_frame_error_rad = self.geometry_residual.frame_error_rad;
        self.metrics.post_position_linear_residual_mps = self.residual.max_velocity_mps;
        self.metrics.post_position_angular_residual_radps =
            self.residual.max_angular_velocity_radps;
        if first_sweep {
            self.metrics.first_post_joint_anchor_error_m = self.geometry_residual.anchor_error_m;
            self.metrics.first_post_joint_frame_error_rad = self.geometry_residual.frame_error_rad;
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn restore_metrics(&mut self, metrics: super::diagnostics::JointSolverMetrics) {
        self.metrics = metrics;
    }

    pub(crate) fn set_residual(&mut self, residual: super::solver::JointResidual) {
        self.residual = residual;
    }

    pub(crate) fn residual(&self) -> super::solver::JointResidual {
        self.residual
    }

    pub(crate) fn set_joint_index(&mut self, index: usize, id: crate::world::JointId) {
        self.previous_row_start = self
            .row_joint_indices
            .partition_point(|&owner| owner < index);
        self.previous_row_end = self
            .row_joint_indices
            .partition_point(|&owner| owner <= index);
        self.active_joint_index = index;
        self.active_joint_id = Some(id);
        set_joint_id(&mut self.preparing_joint_ids, index, id);
    }

    fn update_joint_id(&mut self, id: crate::world::JointId) -> Result<(), CollisionError> {
        if self.active_joint_index >= self.joint_ids.len() {
            self.joint_ids
                .try_reserve(self.active_joint_index + 1 - self.joint_ids.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.joint_ids.resize(self.active_joint_index + 1, None);
        }
        self.joint_ids[self.active_joint_index] = Some(id);
        Ok(())
    }

    pub(crate) fn collect_break_events(
        &mut self,
        joints: &[JointConstraint],
        bodies: &[crate::constraint::SolverBody],
        dt: f64,
        mode: ReactionMode,
    ) -> Result<(), CollisionError> {
        self.break_events.clear();
        if !joints.iter().any(|joint| {
            let common = joint.joint.common();
            common.enabled && (common.break_force.is_some() || common.break_torque.is_some())
        }) {
            return Ok(());
        }
        if mode != ReactionMode::VelocityOnly {
            return Err(CollisionError::InvalidConfig);
        }

        self.reactions.clear();
        self.reactions
            .try_reserve(joints.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.break_events
            .try_reserve(joints.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for joint in joints {
            let common = joint.joint.common();
            let reaction = if common.enabled
                && (common.break_force.is_some() || common.break_torque.is_some())
            {
                reaction_for_joint(*joint, bodies)?
            } else {
                None
            };
            self.reactions.push(reaction);
        }

        for (row, joint_index) in self.rows.iter().zip(&self.row_joint_indices) {
            let Some(reaction) = self
                .reactions
                .get_mut(*joint_index)
                .ok_or(CollisionError::InvalidContact)?
            else {
                continue;
            };
            let (linear_jacobian, angular_jacobian) = match reaction.side {
                ReactionSide::A => (row.linear_a, row.angular_a),
                ReactionSide::B => (row.linear_b, row.angular_b),
            };
            reaction.force_impulse += linear_jacobian * row.accumulated_impulse;
            reaction.torque_com_impulse += angular_jacobian * row.accumulated_impulse;
            if !reaction.force_impulse.is_finite() || !reaction.torque_com_impulse.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }

        for (joint, reaction) in joints.iter().zip(&self.reactions) {
            let common = joint.joint.common();
            let Some(reaction) = reaction else {
                continue;
            };
            let wrench = wrench_from_impulse(
                reaction.force_impulse,
                reaction.torque_com_impulse,
                reaction.com_world,
                reaction.anchor_world,
                dt,
            )?;
            let force = wrench.force.length();
            let torque = wrench.torque.length();
            if !force.is_finite() || !torque.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            if common
                .break_force
                .is_some_and(|threshold| force > threshold)
                || common
                    .break_torque
                    .is_some_and(|threshold| torque > threshold)
            {
                self.break_events
                    .push(JointBreakEvent::new(joint.id, force, torque));
            }
        }
        Ok(())
    }

    pub(crate) fn break_events(&self) -> &[JointBreakEvent] {
        &self.break_events
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        self.rows.capacity() * std::mem::size_of::<ConstraintRow>()
            + self.row_joint_indices.capacity() * std::mem::size_of::<usize>()
            + self.joint_ids.capacity() * std::mem::size_of::<Option<crate::world::JointId>>()
            + self.preparing_rows.capacity() * std::mem::size_of::<ConstraintRow>()
            + self.solver_mass_slots.capacity() * std::mem::size_of::<SolverMassSlot>()
            + self.direct_velocity_scratch.retained_capacity_bytes()
            + self.preparing_joint_indices.capacity() * std::mem::size_of::<usize>()
            + self.preparing_joint_ids.capacity()
                * std::mem::size_of::<Option<crate::world::JointId>>()
            + self.fixed_geometry.capacity()
                * std::mem::size_of::<Option<FixedJointGeometryCache>>()
            + self.fixed_velocity_operators.capacity()
                * std::mem::size_of::<FixedBlockVelocityOperators>()
            + self.reactions.capacity() * std::mem::size_of::<Option<JointReaction>>()
            + self.break_events.capacity() * std::mem::size_of::<JointBreakEvent>()
    }
}

fn reaction_for_joint(
    joint: JointConstraint,
    bodies: &[crate::constraint::SolverBody],
) -> Result<Option<JointReaction>, CollisionError> {
    let body_a = bodies
        .get(joint.body_a)
        .ok_or(CollisionError::InvalidContact)?;
    let body_b = bodies
        .get(joint.body_b)
        .ok_or(CollisionError::InvalidContact)?;
    let side = if body_a.body().physics().body_type() == BodyType::Dynamic {
        ReactionSide::A
    } else if body_b.body().physics().body_type() == BodyType::Dynamic {
        ReactionSide::B
    } else {
        return Ok(None);
    };
    let side_body = match side {
        ReactionSide::A => body_a,
        ReactionSide::B => body_b,
    };
    let com_world = side_body
        .body()
        .world_center_of_mass()
        .map_err(|error| match error {
            BodyError::ArithmeticOverflow => CollisionError::ArithmeticOverflow,
            BodyError::BodyMustBeDynamic
            | BodyError::StaticBodyCannotHaveVelocity
            | BodyError::NonFiniteValue
            | BodyError::InvalidInertiaTensor => CollisionError::InvalidContact,
        })?
        .unwrap_or_else(|| side_body.state().transform().translation());
    let anchor_world = body_a
        .state()
        .transform()
        .transform_point(joint.joint.common().frame_a.local_anchor);
    if !com_world.is_finite() || !anchor_world.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    Ok(Some(JointReaction {
        force_impulse: Vec3::ZERO,
        torque_com_impulse: Vec3::ZERO,
        com_world,
        anchor_world,
        side,
    }))
}

fn set_joint_id(
    ids: &mut Vec<Option<crate::world::JointId>>,
    index: usize,
    id: crate::world::JointId,
) {
    if index >= ids.len() {
        ids.resize(index + 1, None);
    }
    ids[index] = Some(id);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joint::row::RowDimension;
    use crate::{
        constraint::SolverBody,
        joint::{FixedJoint, Joint, JointCommon, JointFrame},
        mass::MassProperties,
        rigid_body::RigidBody,
        transform::Transform,
        world::PhysicsWorld,
    };
    use kapitsa_math::{Mat3, Vec3};

    fn test_joint_id() -> crate::world::JointId {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let body_b = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        world
            .add_joint(
                body_a,
                body_b,
                Joint::Fixed(FixedJoint::new(JointCommon::new(
                    JointFrame::IDENTITY,
                    JointFrame::IDENTITY,
                ))),
            )
            .unwrap()
    }

    fn row(key: u8) -> ConstraintRow {
        ConstraintRow {
            key,
            dimension: RowDimension::Linear,
            body_a: 0,
            body_b: 1,
            linear_a: -Vec3::X,
            angular_a: Vec3::ZERO,
            position_angular_a: Vec3::ZERO,
            linear_b: Vec3::X,
            angular_b: Vec3::ZERO,
            error: 0.0,
            target_velocity: 0.0,
            force_bias: 0.0,
            position_enabled: true,
            compliance: 0.0,
            damping: 0.0,
            min_multiplier: f64::NEG_INFINITY,
            max_multiplier: f64::INFINITY,
            min_impulse: f64::NEG_INFINITY,
            max_impulse: f64::INFINITY,
            accumulated_multiplier: 0.0,
            accumulated_impulse: 0.0,
            cached_velocity_effective_mass: 0.0,
        }
    }

    fn dynamic_solver_body(position: Vec3, local_com: Vec3) -> SolverBody {
        let mass = MassProperties::new(1.0, local_com, Mat3::IDENTITY).expect("valid mass");
        let body = RigidBody::new_dynamic(
            Transform::new(position, kapitsa_math::Quat::IDENTITY).expect("transform"),
            mass,
        )
        .expect("dynamic body");
        let state = *body.state();
        SolverBody::from_prediction(body, state)
    }

    fn static_solver_body(position: Vec3) -> SolverBody {
        let body = RigidBody::new_static(
            Transform::new(position, kapitsa_math::Quat::IDENTITY).expect("transform"),
        );
        let state = *body.state();
        SolverBody::from_prediction(body, state)
    }

    fn reaction_constraint(common: JointCommon) -> JointConstraint {
        JointConstraint {
            id: test_joint_id(),
            body_a: 0,
            body_b: 1,
            joint: Joint::Fixed(FixedJoint::new(common)),
        }
    }

    fn reaction_row(
        dimension: RowDimension,
        linear_a: Vec3,
        angular_a: Vec3,
        linear_b: Vec3,
        angular_b: Vec3,
        impulse: f64,
    ) -> ConstraintRow {
        ConstraintRow {
            key: 0,
            dimension,
            body_a: 0,
            body_b: 1,
            linear_a,
            angular_a,
            position_angular_a: angular_a,
            linear_b,
            angular_b,
            error: 0.0,
            target_velocity: 0.0,
            force_bias: 0.0,
            position_enabled: true,
            compliance: 0.0,
            damping: 0.0,
            min_multiplier: f64::NEG_INFINITY,
            max_multiplier: f64::INFINITY,
            min_impulse: f64::NEG_INFINITY,
            max_impulse: f64::INFINITY,
            accumulated_multiplier: 1.0e12,
            accumulated_impulse: impulse,
            cached_velocity_effective_mass: 0.0,
        }
    }

    #[test]
    fn break_reaction_sums_linear_and_angular_components_at_anchor() {
        let mut common = JointCommon::new(
            JointFrame::new(Vec3::Y, kapitsa_math::Quat::IDENTITY),
            JointFrame::IDENTITY,
        );
        common.break_force = Some(0.1);
        common.break_torque = Some(0.1);
        let joints = [reaction_constraint(common)];
        let bodies = [
            dynamic_solver_body(Vec3::ZERO, Vec3::ZERO),
            static_solver_body(Vec3::X),
        ];
        let mut workspace = JointWorkspace::new();
        workspace.rows.push(reaction_row(
            RowDimension::Linear,
            Vec3::X,
            Vec3::Z,
            -Vec3::X,
            -Vec3::Z,
            2.0,
        ));
        workspace.rows.push(reaction_row(
            RowDimension::Angular,
            Vec3::Y,
            Vec3::Y,
            -Vec3::Y,
            -Vec3::Y,
            3.0,
        ));
        workspace.row_joint_indices.extend([0, 0]);

        workspace
            .collect_break_events(&joints, &bodies, 1.0, ReactionMode::VelocityOnly)
            .expect("collect fixed-joint reaction");

        let event = workspace.break_events().first().expect("break event");
        assert!((event.force() - 13.0_f64.sqrt()).abs() <= 1.0e-12);
        assert!((event.torque() - 5.0).abs() <= 1.0e-12);
    }

    #[test]
    fn break_reaction_uses_dynamic_side_b_at_common_anchor() {
        let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        common.break_force = Some(0.1);
        common.break_torque = Some(0.1);
        let joints = [reaction_constraint(common)];
        let bodies = [
            static_solver_body(Vec3::ZERO),
            dynamic_solver_body(Vec3::ZERO, Vec3::Y),
        ];
        let mut workspace = JointWorkspace::new();
        workspace.rows.push(reaction_row(
            RowDimension::Linear,
            Vec3::X,
            Vec3::ZERO,
            -Vec3::X,
            -Vec3::new(0.0, 0.0, 2.5),
            2.0,
        ));
        workspace.row_joint_indices.push(0);

        workspace
            .collect_break_events(&joints, &bodies, 1.0, ReactionMode::VelocityOnly)
            .expect("collect side-B reaction");

        let event = workspace.break_events().first().expect("break event");
        assert!((event.force() - 2.0).abs() <= 1.0e-12);
        assert!((event.torque() - 3.0).abs() <= 1.0e-12);
    }

    #[test]
    fn break_threshold_is_strict_and_load_is_averaged_over_dt() {
        let bodies = [
            dynamic_solver_body(Vec3::ZERO, Vec3::ZERO),
            static_solver_body(Vec3::X),
        ];
        let mut workspace = JointWorkspace::new();
        workspace.rows.push(reaction_row(
            RowDimension::Linear,
            Vec3::X,
            Vec3::ZERO,
            -Vec3::X,
            Vec3::ZERO,
            2.0,
        ));
        workspace.row_joint_indices.push(0);

        let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        common.break_force = Some(2.0);
        let at_threshold = [reaction_constraint(common)];
        workspace
            .collect_break_events(&at_threshold, &bodies, 1.0, ReactionMode::VelocityOnly)
            .expect("threshold equality");
        assert!(workspace.break_events().is_empty());

        common.break_force = Some(1.999);
        let below_threshold = [reaction_constraint(common)];
        workspace
            .collect_break_events(&below_threshold, &bodies, 1.0, ReactionMode::VelocityOnly)
            .expect("strict threshold crossing");
        assert_eq!(workspace.break_events().len(), 1);
        assert!((workspace.break_events()[0].force() - 2.0).abs() <= 1.0e-12);

        common.break_force = Some(3.0);
        let timestep_threshold = [reaction_constraint(common)];
        workspace
            .collect_break_events(
                &timestep_threshold,
                &bodies,
                0.5,
                ReactionMode::VelocityOnly,
            )
            .expect("shorter timestep load");
        assert_eq!(workspace.break_events().len(), 1);
        assert!((workspace.break_events()[0].force() - 4.0).abs() <= 1.0e-12);
    }

    #[test]
    fn disabled_and_nondynamic_joints_do_not_break() {
        let bodies = [static_solver_body(Vec3::ZERO), static_solver_body(Vec3::X)];
        let mut workspace = JointWorkspace::new();
        workspace.rows.push(reaction_row(
            RowDimension::Linear,
            Vec3::X,
            Vec3::ZERO,
            -Vec3::X,
            Vec3::ZERO,
            100.0,
        ));
        workspace.row_joint_indices.push(0);

        let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        common.break_force = Some(0.1);
        common.enabled = false;
        let disabled = [reaction_constraint(common)];
        workspace
            .collect_break_events(&disabled, &bodies, 1.0, ReactionMode::VelocityOnly)
            .expect("disabled joint");
        assert!(workspace.break_events().is_empty());

        common.enabled = true;
        let nondynamic = [reaction_constraint(common)];
        workspace
            .collect_break_events(&nondynamic, &bodies, 1.0, ReactionMode::VelocityOnly)
            .expect("joint with no dynamic side");
        assert!(workspace.break_events().is_empty());
    }

    #[test]
    fn constraint_estimate_reaction_mode_is_inactive() {
        let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        common.break_force = Some(1.0);
        let joints = [reaction_constraint(common)];
        let bodies = [
            dynamic_solver_body(Vec3::ZERO, Vec3::ZERO),
            static_solver_body(Vec3::X),
        ];
        let mut workspace = JointWorkspace::new();

        assert_eq!(
            workspace.collect_break_events(&joints, &bodies, 1.0, ReactionMode::ConstraintEstimate),
            Err(CollisionError::InvalidConfig)
        );
        assert!(workspace.break_events().is_empty());
    }

    #[test]
    fn clear_reuses_row_and_owner_capacity_for_many_joints() {
        let mut workspace = JointWorkspace::new();
        let joint_id = test_joint_id();
        let add_rows = |workspace: &mut JointWorkspace| {
            workspace.begin_prepare();
            for joint_index in 0..32 {
                workspace.set_joint_index(joint_index, joint_id);
                workspace.push(ConstraintRow {
                    key: joint_index as u8,
                    dimension: RowDimension::Linear,
                    body_a: 0,
                    body_b: 1,
                    linear_a: -Vec3::X,
                    angular_a: Vec3::ZERO,
                    position_angular_a: Vec3::ZERO,
                    linear_b: Vec3::X,
                    angular_b: Vec3::ZERO,
                    error: 0.0,
                    target_velocity: 0.0,
                    force_bias: 0.0,
                    position_enabled: true,
                    compliance: 0.0,
                    damping: 0.0,
                    min_multiplier: f64::NEG_INFINITY,
                    max_multiplier: f64::INFINITY,
                    min_impulse: f64::NEG_INFINITY,
                    max_impulse: f64::INFINITY,
                    accumulated_multiplier: 0.0,
                    accumulated_impulse: 0.0,
                    cached_velocity_effective_mass: 0.0,
                });
            }
            workspace.finish_prepare();
        };
        add_rows(&mut workspace);
        let row_capacity = workspace.rows.capacity();
        let owner_capacity = workspace.row_joint_indices.capacity();
        let id_capacity = workspace.joint_ids.capacity();
        assert!(row_capacity >= 32);
        assert!(owner_capacity >= 32);
        assert!(id_capacity >= 32);

        workspace.clear();

        assert!(workspace.rows.is_empty());
        assert!(workspace.row_joint_indices.is_empty());
        assert!(workspace.joint_ids.is_empty());
        assert_eq!(workspace.rows.capacity(), row_capacity);
        assert_eq!(workspace.row_joint_indices.capacity(), owner_capacity);
        assert_eq!(workspace.joint_ids.capacity(), id_capacity);

        add_rows(&mut workspace);
        assert_eq!(workspace.rows.capacity(), row_capacity);
        assert_eq!(workspace.row_joint_indices.capacity(), owner_capacity);
        assert_eq!(workspace.joint_ids.capacity(), id_capacity);
    }

    #[test]
    fn multipliers_follow_joint_and_row_keys_and_reset_at_step_boundaries() {
        let mut workspace = JointWorkspace::new();
        let first_joint = test_joint_id();
        let replacement_joint = test_joint_id();

        workspace.begin_prepare();
        workspace.set_joint_index(0, first_joint);
        workspace.push(row(4));
        workspace.finish_prepare();
        workspace.rows_mut()[0].accumulated_multiplier = 3.0;
        workspace.rows_mut()[0].accumulated_impulse = 7.0;

        workspace.begin_prepare();
        workspace.set_joint_index(0, replacement_joint);
        workspace.push(row(4));
        workspace.finish_prepare();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
        assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);

        workspace.rows_mut()[0].accumulated_multiplier = 5.0;
        workspace.rows_mut()[0].accumulated_impulse = 11.0;
        workspace.begin_prepare();
        workspace.set_joint_index(0, replacement_joint);
        workspace.push(row(5));
        workspace.finish_prepare();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
        assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);

        workspace.rows_mut()[0].accumulated_multiplier = 13.0;
        workspace.rows_mut()[0].accumulated_impulse = 17.0;
        workspace.begin_prepare();
        workspace.set_joint_index(0, replacement_joint);
        workspace.push(row(5));
        workspace.finish_prepare();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 13.0);
        assert_eq!(workspace.rows()[0].accumulated_impulse, 17.0);

        workspace.begin_step();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
        assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);
    }

    #[test]
    fn sequential_updates_keep_row_owner_and_multiplier_identity() {
        let mut workspace = JointWorkspace::new();
        let first_joint = test_joint_id();
        let replacement_joint = test_joint_id();

        workspace.begin_joint_prepare(0, first_joint);
        workspace.push(row(4));
        workspace.finish_joint_prepare().unwrap();
        workspace.rows_mut()[0].accumulated_multiplier = 3.0;

        workspace.begin_joint_prepare(0, first_joint);
        workspace.push(row(4));
        workspace.finish_joint_prepare().unwrap();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 3.0);

        workspace.begin_joint_prepare(0, replacement_joint);
        workspace.push(row(4));
        workspace.finish_joint_prepare().unwrap();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
        assert_eq!(workspace.joint_ids[0], Some(replacement_joint));
    }
}
