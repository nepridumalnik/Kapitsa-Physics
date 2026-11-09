pub(crate) use crate::constraint::SolverBody;
use crate::{
    constraint::{self, SolverBodyAccess},
    gravity::CentralGravity,
    joint::{
        IterationRange, JointConstraint, JointIterations, JointSolver, JointSolverOptions,
        JointSolverStep, JointTolerance, JointWorkspace,
    },
    rigid_body::RigidBody,
};
#[cfg(feature = "bench-internals")]
use crate::{mass::MassProperties, rigid_body::BodyType, transform::Transform};
#[cfg(feature = "bench-internals")]
use kapitsa_math::Mat3;
use kapitsa_math::Vec3;
#[forbid(unsafe_code)]
mod colored_bodies;

use super::{
    CollisionConfig, CollisionError,
    manifold::{ContactManifold, ContactPoint},
};

const POSITION_CORRECTION_FRACTION: f64 = 0.2;
const LARGE_ISLAND_COLORING_THRESHOLD: usize = 128;
const LARGE_ISLAND_BATCH_SIZE: usize = 16;
const SMALL_COLOR_MERGE_THRESHOLD: usize = 32;
/// Estimated dispatch and barrier cost, in units of one constraint solve.
const COLORED_DISPATCH_COST: usize = 16;
/// Dispatching a single batch through an executor only adds scheduling cost.
const COLORED_MIN_PARALLEL_BATCHES: usize = 2;

/// Cheap deterministic estimate of how well the color schedule maps to workers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct ColoredWorkload {
    constraint_count: usize,
    color_count: usize,
    batch_count: usize,
    useful_batch_count: usize,
    parallel_constraint_count: usize,
    estimated_saved_work: usize,
    dispatch_count: usize,
}

impl ColoredWorkload {
    fn add_color(&mut self, constraints: usize, worker_count: usize) {
        let batches = constraints.div_ceil(LARGE_ISLAND_BATCH_SIZE);
        self.constraint_count = self.constraint_count.saturating_add(constraints);
        self.batch_count = self.batch_count.saturating_add(batches);
        self.useful_batch_count = self
            .useful_batch_count
            .saturating_add(batches.min(worker_count));
        if batches >= COLORED_MIN_PARALLEL_BATCHES && constraints > SMALL_COLOR_MERGE_THRESHOLD {
            self.parallel_constraint_count =
                self.parallel_constraint_count.saturating_add(constraints);
            let active_workers = batches.min(worker_count).max(1);
            // The slowest worker determines when the next color can begin.
            let waves = batches.div_ceil(active_workers);
            let parallel_work = waves
                .saturating_mul(LARGE_ISLAND_BATCH_SIZE)
                .min(constraints);
            self.estimated_saved_work = self
                .estimated_saved_work
                .saturating_add(constraints - parallel_work);
            self.dispatch_count = self.dispatch_count.saturating_add(1);
        }
    }

    fn rejection_reason(self, worker_count: usize) -> Option<&'static str> {
        if worker_count <= 1 || self.constraint_count < LARGE_ISLAND_COLORING_THRESHOLD {
            return Some("small-or-single-worker");
        }
        if self.color_count == 0 || self.batch_count == 0 {
            return Some("empty");
        }
        if self.estimated_saved_work <= self.dispatch_count.saturating_mul(COLORED_DISPATCH_COST) {
            return Some("dispatch-cost");
        }
        None
    }

    fn can_use_parallel_schedule(self, worker_count: usize) -> bool {
        self.rejection_reason(worker_count).is_none()
    }

    #[cfg(feature = "bench-internals")]
    fn can_use_partial_schedule(self, worker_count: usize) -> bool {
        worker_count > 1
            && self.parallel_constraint_count >= LARGE_ISLAND_COLORING_THRESHOLD
            && self.parallel_constraint_count >= self.constraint_count.div_ceil(2)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SolverMaterial {
    pub(crate) restitution: f64,
    pub(crate) static_friction: f64,
    pub(crate) dynamic_friction: f64,
}

#[derive(Clone, Copy)]
pub(crate) struct SolverManifold {
    pub(crate) body_a: usize,
    pub(crate) body_b: usize,
    pub(crate) manifold: ContactManifold,
    pub(crate) material: SolverMaterial,
}

#[derive(Clone, Copy, Default)]
struct SolverPoint {
    manifold: usize,
    point: usize,
    body_a: usize,
    body_b: usize,
    position_a: Vec3,
    position_b: Vec3,
    normal: Vec3,
    tangent_a: Vec3,
    tangent_b: Vec3,
    restitution_target: f64,
}

enum ColoredConstraintKind {
    Contact(SolverManifold),
    Joint(JointConstraint),
}

#[derive(Clone, Copy)]
enum ColoredConstraintPhase {
    PrepareContactPoints,
    WarmStartContacts,
    Velocity { first_iteration: bool },
    Position,
}

impl ColoredConstraintPhase {
    const fn is_contact_only(self) -> bool {
        matches!(self, Self::PrepareContactPoints | Self::WarmStartContacts)
    }
}

struct ColoredConstraintTask {
    constraint_index: usize,
    body_indices: [usize; 2],
    binding_positions: [usize; 2],
    points: [SolverPoint; 4],
    point_count: usize,
    kind: ColoredConstraintKind,
}

impl ColoredConstraintTask {
    fn new(
        constraint_index: usize,
        manifolds: &[SolverManifold],
        joints: &[JointConstraint],
    ) -> Result<Self, CollisionError> {
        let (body_indices, kind) = if let Some(manifold) = manifolds.get(constraint_index).copied()
        {
            (
                [manifold.body_a, manifold.body_b],
                ColoredConstraintKind::Contact(manifold),
            )
        } else {
            let joint_index = constraint_index
                .checked_sub(manifolds.len())
                .ok_or(CollisionError::InvalidContact)?;
            let joint = *joints
                .get(joint_index)
                .ok_or(CollisionError::InvalidContact)?;
            (
                [joint.body_a, joint.body_b],
                ColoredConstraintKind::Joint(joint),
            )
        };
        Ok(Self {
            constraint_index,
            body_indices,
            binding_positions: [usize::MAX; 2],
            points: [SolverPoint::default(); 4],
            point_count: 0,
            kind,
        })
    }

    fn update(
        &mut self,
        manifolds: &[SolverManifold],
        joints: &[JointConstraint],
    ) -> Result<(), CollisionError> {
        self.kind = if let Some(manifold) = manifolds.get(self.constraint_index).copied() {
            self.body_indices = [manifold.body_a, manifold.body_b];
            ColoredConstraintKind::Contact(manifold)
        } else {
            let joint_index = self
                .constraint_index
                .checked_sub(manifolds.len())
                .ok_or(CollisionError::InvalidContact)?;
            let joint = *joints
                .get(joint_index)
                .ok_or(CollisionError::InvalidContact)?;
            self.body_indices = [joint.body_a, joint.body_b];
            ColoredConstraintKind::Joint(joint)
        };
        self.point_count = 0;
        Ok(())
    }

    fn set_binding_positions(&mut self, positions: [usize; 2]) {
        self.binding_positions = positions;
    }
}

struct ColoredConstraintBatch {
    tasks: Vec<ColoredConstraintTask>,
    joint_workspace: JointWorkspace,
    contact_points: Vec<SolverPoint>,
    contact_task_count: usize,
    error: Option<CollisionError>,
}

struct BoundConstraint<'a> {
    task: &'a mut ColoredConstraintTask,
    bodies: [colored_bodies::BoundBodyRef<'a>; 2],
}

struct BoundColoredBatch<'a> {
    tasks: Vec<BoundConstraint<'a>>,
    joint_workspace: &'a mut JointWorkspace,
    contact_points: &'a mut Vec<SolverPoint>,
    error: &'a mut Option<CollisionError>,
}

fn bind_colored_batches<'a>(
    batches: &'a mut [ColoredConstraintBatch],
    bodies: &'a mut [SolverBody],
    indices: &[usize],
) -> Result<Vec<BoundColoredBatch<'a>>, CollisionError> {
    let mut binding = colored_bodies::BodyBinding::bind(bodies, indices)?;
    let mut result = Vec::with_capacity(batches.len());
    for batch in batches.iter_mut() {
        let ColoredConstraintBatch {
            tasks,
            joint_workspace,
            contact_points,
            error,
            ..
        } = batch;
        let mut bound_tasks = Vec::with_capacity(tasks.len());
        for task in tasks.iter_mut() {
            let bodies = binding.pair_at(task.binding_positions)?;
            bound_tasks.push(BoundConstraint { task, bodies });
        }
        result.push(BoundColoredBatch {
            tasks: bound_tasks,
            joint_workspace,
            contact_points,
            error,
        });
    }
    Ok(result)
}

impl ColoredConstraintBatch {
    fn new() -> Self {
        Self {
            tasks: Vec::new(),
            joint_workspace: JointWorkspace::new(),
            contact_points: Vec::new(),
            contact_task_count: 0,
            error: None,
        }
    }

    fn has_contacts(&self) -> bool {
        self.contact_task_count != 0
    }
}

#[derive(Default)]
pub(crate) struct SolverWorkspace {
    points: Vec<SolverPoint>,
    #[cfg(feature = "bench-internals")]
    velocity_snapshots: Vec<VelocitySnapshot>,
    constraint_coloring: super::constraint_coloring::ConstraintColoring,
    coloring_dynamic_bodies: Vec<bool>,
    coloring_endpoints: Vec<[usize; 2]>,
    colored_batches: Vec<ColoredConstraintBatch>,
    color_batch_offsets: Vec<usize>,
    joint_task_locations: Vec<[usize; 2]>,
    active_colored_batches: usize,
    #[cfg(feature = "bench-internals")]
    colored_schedule: super::ColoredScheduleMetrics,
    #[cfg(feature = "bench-internals")]
    colored_solver_selected: bool,
    #[cfg(feature = "bench-internals")]
    colored_groups_dispatched: usize,
    #[cfg(feature = "bench-internals")]
    colored_parallel_batches: usize,
    #[cfg(feature = "bench-internals")]
    reduced_fixed_tree_offsets: Vec<Transform>,
    #[cfg(feature = "bench-internals")]
    reduced_fixed_tree_signature: Vec<[usize; 2]>,
    #[cfg(feature = "bench-internals")]
    reduced_fixed_tree_dynamic_indices: Vec<usize>,
    #[cfg(feature = "bench-internals")]
    reduced_fixed_tree_assigned: Vec<bool>,
    #[cfg(feature = "bench-internals")]
    reduced_fixed_tree_mass: Option<MassProperties>,
    #[cfg(feature = "bench-internals")]
    reduced_fixed_tree_centers: Vec<Vec3>,
    #[cfg(feature = "bench-internals")]
    reduced_fixed_tree_target_centers: Vec<Vec3>,
    #[cfg(feature = "bench-internals")]
    reduced_contact_bodies: Vec<SolverBody>,
    #[cfg(feature = "bench-internals")]
    reduced_contact_manifolds: Vec<SolverManifold>,
    #[cfg(feature = "bench-internals")]
    reduced_contact_body_map: Vec<usize>,
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy)]
struct VelocitySnapshot {
    linear: Vec3,
    angular: Vec3,
}

/// Residual measurement used by the adaptive iteration policy. The switch only
/// removes bench-only geometry collection; solver state and the measured
/// residual are identical in both modes.
fn measure_joint_residual(
    joint_workspace: &mut JointWorkspace,
    bodies: &[SolverBody],
    joints: &[JointConstraint],
    dt: f64,
    full_diagnostics: bool,
) -> Result<(), CollisionError> {
    #[cfg(feature = "bench-internals")]
    if !full_diagnostics {
        return JointSolver::measure_residual_light(joint_workspace, bodies, joints, dt);
    }
    #[cfg(not(feature = "bench-internals"))]
    let _ = full_diagnostics;
    JointSolver::measure_residual(joint_workspace, bodies, joints, dt)
}

#[cfg(feature = "bench-internals")]
fn snapshot_velocities(
    bodies: &[SolverBody],
    snapshots: &mut Vec<VelocitySnapshot>,
) -> Result<(), CollisionError> {
    if snapshots.capacity() < bodies.len() {
        snapshots
            .try_reserve(bodies.len().saturating_sub(snapshots.len()))
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    snapshots.clear();
    snapshots.extend(bodies.iter().map(|solver_body| {
        let state = solver_body.state();
        VelocitySnapshot {
            linear: state.linear_velocity(),
            angular: state.angular_velocity(),
        }
    }));
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn velocity_projection_work(
    before: &[VelocitySnapshot],
    bodies: &[SolverBody],
) -> Result<(f64, f64), CollisionError> {
    if before.len() != bodies.len() {
        return Err(CollisionError::InvalidContact);
    }
    let mut impulse_work = 0.0;
    let mut quadratic_work = 0.0;
    for (before, solver_body) in before.iter().zip(bodies) {
        let body = solver_body.body();
        let Some(mass) = body.physics().mass() else {
            continue;
        };
        let rotation = solver_body.state().transform().rotation();
        let state = solver_body.state();
        let linear_delta = state
            .linear_velocity()
            .checked_sub(before.linear)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let angular_delta = state
            .angular_velocity()
            .checked_sub(before.angular)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let linear_impulse = linear_delta
            .checked_mul_scalar(mass.mass())
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let local_angular_delta = rotation
            .conjugate()
            .rotate_vector(angular_delta)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let angular_impulse = rotation
            .rotate_vector(mass.inertia() * local_angular_delta)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        impulse_work += before.linear.dot(linear_impulse) + before.angular.dot(angular_impulse);
        quadratic_work +=
            0.5 * (linear_delta.dot(linear_impulse) + angular_delta.dot(angular_impulse));
    }
    if impulse_work.is_finite() && quadratic_work.is_finite() {
        Ok((impulse_work, quadratic_work))
    } else {
        Err(CollisionError::ArithmeticOverflow)
    }
}

#[cfg(feature = "bench-internals")]
pub(crate) fn body_energy_components(
    body: &RigidBody,
    state: crate::rigid_body::BodyState,
    gravity: Option<CentralGravity>,
) -> crate::joint::diagnostics::EnergyComponents {
    let Some(mass) = body.physics().mass() else {
        return crate::joint::diagnostics::EnergyComponents::default();
    };
    let angular_velocity = state.angular_velocity();
    let rotation = state.transform().rotation();
    let rotational = rotation
        .conjugate()
        .rotate_vector(angular_velocity)
        .map(|local_omega| mass.inertia() * local_omega)
        .and_then(|local_i_omega| rotation.rotate_vector(local_i_omega))
        .map_or(0.0, |world_i_omega| {
            0.5 * angular_velocity.dot(world_i_omega)
        });
    let mut potential = 0.0;
    if let Some(gravity) = gravity {
        let center = state.transform().transform_point(mass.center_of_mass());
        let radius = (center - gravity.center()).length();
        if radius > 0.0 && radius.is_finite() {
            potential = -gravity.gravitational_parameter() * mass.mass() / radius;
        }
    }
    crate::joint::diagnostics::EnergyComponents {
        translational_j: 0.5 * mass.mass() * state.linear_velocity().length_squared(),
        rotational_j: rotational,
        potential_j: potential,
    }
}

#[cfg(feature = "bench-internals")]
fn energy_components(
    bodies: &[SolverBody],
    gravity: Option<CentralGravity>,
) -> crate::joint::diagnostics::EnergyComponents {
    let mut energy = crate::joint::diagnostics::EnergyComponents::default();
    for body in bodies {
        energy.add_assign(body_energy_components(body.body(), body.state(), gravity));
    }
    energy
}

pub(crate) struct SolverIsland {
    pub(crate) body_indices: Vec<usize>,
    pub(crate) manifold_indices: Vec<usize>,
    pub(crate) joint_indices: Vec<usize>,
    pub(crate) bodies: Vec<SolverBody>,
    pub(crate) manifolds: Vec<SolverManifold>,
    pub(crate) joints: Vec<JointConstraint>,
    pub(crate) workspace: SolverWorkspace,
    pub(crate) joint_workspace: JointWorkspace,
    pub(crate) worker_error: Option<CollisionError>,
    pub(crate) constrained_starts: Vec<SolverBody>,
    pub(crate) constrained_eligible: bool,
    pub(crate) constrained_workspace: crate::joint::constrained::ConstrainedWorkspace,
}

impl SolverIsland {
    pub(crate) fn new() -> Self {
        Self {
            body_indices: Vec::new(),
            manifold_indices: Vec::new(),
            joint_indices: Vec::new(),
            bodies: Vec::new(),
            manifolds: Vec::new(),
            joints: Vec::new(),
            workspace: SolverWorkspace::new(),
            joint_workspace: JointWorkspace::new(),
            worker_error: None,
            constrained_starts: Vec::new(),
            constrained_eligible: false,
            constrained_workspace: crate::joint::constrained::ConstrainedWorkspace::default(),
        }
    }

    pub(crate) fn clear(&mut self) {
        self.body_indices.clear();
        self.manifold_indices.clear();
        self.joint_indices.clear();
        self.bodies.clear();
        self.manifolds.clear();
        self.joints.clear();
        self.joint_workspace.clear();
        self.worker_error = None;
        self.constrained_starts.clear();
        self.constrained_eligible = true;
    }

    pub(crate) fn solve(
        &mut self,
        config: CollisionConfig,
        step: JointSolverStep<'_>,
        gravity: Option<CentralGravity>,
        executor: &impl crate::execution::StepExecutor,
        workers: usize,
    ) -> Result<(), CollisionError> {
        if self.constrained_eligible {
            crate::joint::constrained::ConstrainedIntegrator::solve(
                &mut self.constrained_starts,
                &self.joints,
                step.dt(),
                &mut self.constrained_workspace,
            )?;
            self.bodies.copy_from_slice(&self.constrained_starts);
            return Ok(());
        }
        self.workspace.solve_with_joints(
            &mut self.bodies,
            &mut self.manifolds,
            &mut self.joint_workspace,
            &self.joints,
            config,
            step,
            gravity,
            executor,
            workers,
        )
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        self.body_indices.capacity() * size_of::<usize>()
            + self.manifold_indices.capacity() * size_of::<usize>()
            + self.joint_indices.capacity() * size_of::<usize>()
            + self.bodies.capacity() * size_of::<SolverBody>()
            + self.manifolds.capacity() * size_of::<SolverManifold>()
            + self.joints.capacity() * size_of::<JointConstraint>()
            + self.workspace.retained_capacity_bytes()
            + self.joint_workspace.retained_capacity_bytes()
            + self.constrained_starts.capacity() * size_of::<SolverBody>()
            + self.constrained_workspace.retained_capacity_bytes()
    }
}

impl SolverWorkspace {
    pub(crate) const fn new() -> Self {
        Self {
            points: Vec::new(),
            #[cfg(feature = "bench-internals")]
            velocity_snapshots: Vec::new(),
            constraint_coloring: super::constraint_coloring::ConstraintColoring::new(),
            coloring_dynamic_bodies: Vec::new(),
            coloring_endpoints: Vec::new(),
            colored_batches: Vec::new(),
            color_batch_offsets: Vec::new(),
            joint_task_locations: Vec::new(),
            active_colored_batches: 0,
            #[cfg(feature = "bench-internals")]
            colored_schedule: super::ColoredScheduleMetrics {
                rejection: "not-evaluated",
                constraints: 0,
                colors: 0,
                batches: 0,
                useful_batches: 0,
                parallel_colors: 0,
                parallel_constraints: 0,
                sequential_constraints: 0,
                tasks_started: 0,
                velocity_sweeps: std::time::Duration::ZERO,
                position_sweeps: std::time::Duration::ZERO,
            },
            #[cfg(feature = "bench-internals")]
            colored_solver_selected: false,
            #[cfg(feature = "bench-internals")]
            colored_groups_dispatched: 0,
            #[cfg(feature = "bench-internals")]
            colored_parallel_batches: 0,
            #[cfg(feature = "bench-internals")]
            reduced_fixed_tree_offsets: Vec::new(),
            #[cfg(feature = "bench-internals")]
            reduced_fixed_tree_signature: Vec::new(),
            #[cfg(feature = "bench-internals")]
            reduced_fixed_tree_dynamic_indices: Vec::new(),
            #[cfg(feature = "bench-internals")]
            reduced_fixed_tree_assigned: Vec::new(),
            #[cfg(feature = "bench-internals")]
            reduced_fixed_tree_mass: None,
            #[cfg(feature = "bench-internals")]
            reduced_fixed_tree_centers: Vec::new(),
            #[cfg(feature = "bench-internals")]
            reduced_fixed_tree_target_centers: Vec::new(),
            #[cfg(feature = "bench-internals")]
            reduced_contact_bodies: Vec::new(),
            #[cfg(feature = "bench-internals")]
            reduced_contact_manifolds: Vec::new(),
            #[cfg(feature = "bench-internals")]
            reduced_contact_body_map: Vec::new(),
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn colored_dispatch_metrics(&self) -> (bool, usize, usize) {
        (
            self.colored_solver_selected,
            self.colored_groups_dispatched,
            self.colored_parallel_batches,
        )
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn colored_schedule_metrics(&self) -> super::ColoredScheduleMetrics {
        self.colored_schedule
    }

    #[cfg(feature = "bench-internals")]
    fn reset_colored_dispatch_metrics(&mut self) {
        self.colored_schedule = super::ColoredScheduleMetrics {
            rejection: "not-evaluated",
            ..Default::default()
        };
        self.colored_solver_selected = false;
        self.colored_groups_dispatched = 0;
        self.colored_parallel_batches = 0;
    }

    pub(crate) fn clear(&mut self) {
        self.points.clear();
        #[cfg(feature = "bench-internals")]
        self.velocity_snapshots.clear();
    }

    #[cfg(feature = "bench-internals")]
    fn solve_reduced_fixed_tree(
        &mut self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
        joints: &[JointConstraint],
        config: CollisionConfig,
    ) -> Result<bool, CollisionError> {
        self.reduced_fixed_tree_dynamic_indices.clear();
        self.reduced_fixed_tree_dynamic_indices
            .extend(bodies.iter().enumerate().filter_map(|(index, body)| {
                (body.body().physics().body_type() == BodyType::Dynamic).then_some(index)
            }));
        let dynamic_count = self.reduced_fixed_tree_dynamic_indices.len();
        if dynamic_count < 2
            || joints.len() != dynamic_count - 1
            || bodies.iter().any(|body| {
                !matches!(
                    body.body().physics().body_type(),
                    BodyType::Dynamic | BodyType::Static
                )
            })
            || joints.iter().any(|constraint| {
                !self
                    .reduced_fixed_tree_dynamic_indices
                    .contains(&constraint.body_a)
                    || !self
                        .reduced_fixed_tree_dynamic_indices
                        .contains(&constraint.body_b)
                    || !matches!(constraint.joint, crate::joint::Joint::Fixed(_))
                    || !constraint.joint.common().enabled
                    || constraint.body_a >= bodies.len()
                    || constraint.body_b >= bodies.len()
                    || constraint.body_a == constraint.body_b
            })
        {
            return Ok(false);
        }
        if manifolds.iter().any(|manifold| {
            self.reduced_fixed_tree_dynamic_indices
                .contains(&manifold.body_a)
                && self
                    .reduced_fixed_tree_dynamic_indices
                    .contains(&manifold.body_b)
        }) {
            return Ok(false);
        }
        if !manifolds.is_empty() {
            self.solve(bodies, manifolds, config)?;
        }

        let topology_changed = self.reduced_fixed_tree_signature.len() != joints.len()
            || joints.iter().enumerate().any(|(index, constraint)| {
                self.reduced_fixed_tree_signature.get(index)
                    != Some(&[constraint.body_a, constraint.body_b])
            })
            || self.reduced_fixed_tree_offsets.len() != bodies.len();
        if topology_changed {
            let mut roots = (0..dynamic_count).collect::<Vec<_>>();
            for constraint in joints {
                let a = constraint.body_a;
                let b = constraint.body_b;
                let Some(local_a) = self
                    .reduced_fixed_tree_dynamic_indices
                    .iter()
                    .position(|index| *index == a)
                else {
                    return Ok(false);
                };
                let Some(local_b) = self
                    .reduced_fixed_tree_dynamic_indices
                    .iter()
                    .position(|index| *index == b)
                else {
                    return Ok(false);
                };
                let mut root_a = local_a;
                while roots[root_a] != root_a {
                    root_a = roots[root_a];
                }
                let mut root_b = local_b;
                while roots[root_b] != root_b {
                    root_b = roots[root_b];
                }
                if root_a == root_b {
                    return Ok(false);
                }
                roots[root_b] = root_a;
            }
            let mut connected_root = 0;
            while roots[connected_root] != connected_root {
                connected_root = roots[connected_root];
            }
            for index in 0..roots.len() {
                let mut root = index;
                while roots[root] != root {
                    root = roots[root];
                }
                if root != connected_root {
                    return Ok(false);
                }
            }
            self.reduced_fixed_tree_offsets.clear();
            self.reduced_fixed_tree_offsets
                .try_reserve(bodies.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.reduced_fixed_tree_offsets
                .resize(bodies.len(), Transform::identity());
            self.reduced_fixed_tree_assigned.clear();
            self.reduced_fixed_tree_assigned
                .resize(dynamic_count, false);
            self.reduced_fixed_tree_assigned[0] = true;
            let mut assigned_count = 1;
            while assigned_count < dynamic_count {
                let mut progress = false;
                for constraint in joints {
                    let Some(local_a) = self
                        .reduced_fixed_tree_dynamic_indices
                        .iter()
                        .position(|index| *index == constraint.body_a)
                    else {
                        return Ok(false);
                    };
                    let Some(local_b) = self
                        .reduced_fixed_tree_dynamic_indices
                        .iter()
                        .position(|index| *index == constraint.body_b)
                    else {
                        return Ok(false);
                    };
                    let crate::joint::Joint::Fixed(fixed) = constraint.joint else {
                        return Ok(false);
                    };
                    if self.reduced_fixed_tree_assigned[local_a]
                        && !self.reduced_fixed_tree_assigned[local_b]
                    {
                        let frame_a = Transform::new(
                            fixed.common.frame_a.local_anchor,
                            fixed.common.frame_a.local_rotation,
                        )
                        .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let frame_b = Transform::new(
                            fixed.common.frame_b.local_anchor,
                            fixed.common.frame_b.local_rotation,
                        )
                        .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let frame_b_inverse = frame_b
                            .inverse()
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let relative = frame_a
                            .compose(&frame_b_inverse)
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        self.reduced_fixed_tree_offsets[constraint.body_b] = self
                            .reduced_fixed_tree_offsets[constraint.body_a]
                            .compose(&relative)
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        self.reduced_fixed_tree_assigned[local_b] = true;
                        assigned_count += 1;
                        progress = true;
                    } else if self.reduced_fixed_tree_assigned[local_b]
                        && !self.reduced_fixed_tree_assigned[local_a]
                    {
                        let frame_a = Transform::new(
                            fixed.common.frame_a.local_anchor,
                            fixed.common.frame_a.local_rotation,
                        )
                        .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let frame_b = Transform::new(
                            fixed.common.frame_b.local_anchor,
                            fixed.common.frame_b.local_rotation,
                        )
                        .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let frame_a_inverse = frame_a
                            .inverse()
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let relative = frame_b
                            .compose(&frame_a_inverse)
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        self.reduced_fixed_tree_offsets[constraint.body_a] = self
                            .reduced_fixed_tree_offsets[constraint.body_b]
                            .compose(&relative)
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        self.reduced_fixed_tree_assigned[local_a] = true;
                        assigned_count += 1;
                        progress = true;
                    }
                }
                if !progress {
                    return Ok(false);
                }
            }
            self.reduced_fixed_tree_signature.clear();
            self.reduced_fixed_tree_signature.extend(
                joints
                    .iter()
                    .map(|constraint| [constraint.body_a, constraint.body_b]),
            );
        }

        let root_index = self.reduced_fixed_tree_dynamic_indices[0];
        let root_transform = bodies[root_index].state().transform();
        let root_inverse = root_transform
            .inverse()
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let mut total_mass = 0.0;
        let mut center_numerator = Vec3::ZERO;
        for &body_index in &self.reduced_fixed_tree_dynamic_indices {
            let mass = bodies[body_index]
                .body()
                .physics()
                .mass()
                .ok_or(CollisionError::InvalidContact)?;
            let center =
                self.reduced_fixed_tree_offsets[body_index].transform_point(mass.center_of_mass());
            total_mass += mass.mass();
            center_numerator += center * mass.mass();
        }
        if !total_mass.is_finite() || total_mass <= 0.0 {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let center_of_mass = center_numerator / total_mass;
        let mut inertia_about_root = Mat3::ZERO;
        for &body_index in &self.reduced_fixed_tree_dynamic_indices {
            let mass = bodies[body_index]
                .body()
                .physics()
                .mass()
                .ok_or(CollisionError::InvalidContact)?;
            let offset = self.reduced_fixed_tree_offsets[body_index];
            let rotation = offset
                .rotation()
                .to_mat3()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let center = offset.transform_point(mass.center_of_mass());
            let inertia = rotation * mass.inertia() * rotation.transpose();
            inertia_about_root = inertia_about_root + inertia + parallel_axis(center, mass.mass());
        }
        let inertia_about_com = inertia_about_root - parallel_axis(center_of_mass, total_mass);
        self.reduced_fixed_tree_mass = Some(
            MassProperties::new(total_mass, center_of_mass, inertia_about_com)
                .map_err(|_| CollisionError::ArithmeticOverflow)?,
        );
        let aggregate_mass = self
            .reduced_fixed_tree_mass
            .ok_or(CollisionError::InvalidContact)?;
        let mut total_momentum = Vec3::ZERO;
        let mut current_com_numerator = Vec3::ZERO;
        self.reduced_fixed_tree_centers.clear();
        for &body_index in &self.reduced_fixed_tree_dynamic_indices {
            let body = &bodies[body_index];
            let mass = body
                .body()
                .physics()
                .mass()
                .ok_or(CollisionError::InvalidContact)?;
            let center = body
                .body()
                .world_center_of_mass()
                .map_err(|_| CollisionError::ArithmeticOverflow)?
                .ok_or(CollisionError::InvalidContact)?;
            total_momentum += body.state().linear_velocity() * mass.mass();
            current_com_numerator += center * mass.mass();
            self.reduced_fixed_tree_centers.push(center);
        }
        let total_mass = aggregate_mass.mass();
        let current_com = current_com_numerator / total_mass;
        let target_com = root_transform.transform_point(aggregate_mass.center_of_mass());
        let linear_velocity = total_momentum / total_mass;
        let mut angular_momentum = Vec3::ZERO;
        for (index, &body_index) in self.reduced_fixed_tree_dynamic_indices.iter().enumerate() {
            let body = &bodies[body_index];
            let mass = body
                .body()
                .physics()
                .mass()
                .ok_or(CollisionError::InvalidContact)?;
            let rotation_matrix = body
                .state()
                .transform()
                .rotation()
                .to_mat3()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let inertia_world = rotation_matrix * mass.inertia() * rotation_matrix.transpose();
            angular_momentum += inertia_world * body.state().angular_velocity();
            angular_momentum += (self.reduced_fixed_tree_centers[index] - current_com)
                .cross(body.state().linear_velocity() * mass.mass());
        }
        angular_momentum -= (target_com - current_com).cross(total_momentum);
        let root_rotation = root_transform
            .rotation()
            .to_mat3()
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let aggregate_inertia_world =
            root_rotation * aggregate_mass.inertia() * root_rotation.transpose();
        let angular_velocity = aggregate_inertia_world
            .solve_spd(angular_momentum)
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let mut aggregate_body = RigidBody::new_dynamic(root_transform, aggregate_mass)
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        aggregate_body
            .set_linear_velocity(linear_velocity)
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        aggregate_body
            .set_angular_velocity(angular_velocity)
            .map_err(|_| CollisionError::ArithmeticOverflow)?;

        self.reduced_contact_bodies.clear();
        self.reduced_contact_bodies
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.reduced_contact_bodies
            .push(SolverBody::from_prediction(
                aggregate_body,
                *aggregate_body.state(),
            ));
        if self.reduced_contact_body_map.len() < bodies.len() {
            self.reduced_contact_body_map
                .try_reserve(bodies.len() - self.reduced_contact_body_map.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        self.reduced_contact_body_map.clear();
        self.reduced_contact_body_map
            .resize(bodies.len(), usize::MAX);
        for &body_index in &self.reduced_fixed_tree_dynamic_indices {
            self.reduced_contact_body_map[body_index] = 0;
        }
        self.reduced_contact_manifolds.clear();
        self.reduced_contact_manifolds
            .try_reserve(manifolds.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for source in manifolds.iter().copied() {
            if source.body_a >= bodies.len() || source.body_b >= bodies.len() {
                return Ok(false);
            }
            let mut reduced = source;
            for original_index in [source.body_a, source.body_b] {
                if self.reduced_contact_body_map[original_index] != usize::MAX {
                    continue;
                }
                if bodies[original_index].body().physics().body_type() != BodyType::Static {
                    return Ok(false);
                }
                self.reduced_contact_body_map[original_index] = self.reduced_contact_bodies.len();
                self.reduced_contact_bodies
                    .push(SolverBody::from_prediction(
                        bodies[original_index].body,
                        bodies[original_index].state(),
                    ));
            }
            reduced.body_a = self.reduced_contact_body_map[source.body_a];
            reduced.body_b = self.reduced_contact_body_map[source.body_b];
            if reduced.body_a == reduced.body_b {
                return Ok(false);
            }
            for point_index in 0..reduced.manifold.len() {
                let point = reduced
                    .manifold
                    .point_mut(point_index)
                    .ok_or(CollisionError::InvalidContact)?;
                if self.reduced_contact_body_map[source.body_a] == 0 {
                    let target_transform = root_transform
                        .compose(&self.reduced_fixed_tree_offsets[source.body_a])
                        .map_err(|_| CollisionError::ArithmeticOverflow)?;
                    let world_point = target_transform.transform_point(point.local_a);
                    point.local_a = root_inverse.transform_point(world_point);
                }
                if self.reduced_contact_body_map[source.body_b] == 0 {
                    let target_transform = root_transform
                        .compose(&self.reduced_fixed_tree_offsets[source.body_b])
                        .map_err(|_| CollisionError::ArithmeticOverflow)?;
                    let world_point = target_transform.transform_point(point.local_b);
                    point.local_b = root_inverse.transform_point(world_point);
                }
            }
            self.reduced_contact_manifolds.push(reduced);
        }
        if !self.reduced_contact_manifolds.is_empty() {
            let mut reduced_bodies = std::mem::take(&mut self.reduced_contact_bodies);
            let mut reduced_manifolds = std::mem::take(&mut self.reduced_contact_manifolds);
            let solve_result = self.solve(&mut reduced_bodies, &mut reduced_manifolds, config);
            self.reduced_contact_bodies = reduced_bodies;
            self.reduced_contact_manifolds = reduced_manifolds;
            solve_result?;
            for (source, solved) in manifolds
                .iter_mut()
                .zip(self.reduced_contact_manifolds.iter())
            {
                for point_index in 0..source.manifold.len() {
                    let Some(source_point) = source.manifold.point_mut(point_index) else {
                        return Err(CollisionError::InvalidContact);
                    };
                    let solved_point = solved
                        .manifold
                        .point(point_index)
                        .ok_or(CollisionError::InvalidContact)?;
                    source_point.normal_impulse = solved_point.normal_impulse;
                    source_point.normal_impulse_increase_sum =
                        solved_point.normal_impulse_increase_sum;
                    source_point.tangent_impulse = solved_point.tangent_impulse;
                }
            }
        }

        let aggregate_state = self.reduced_contact_bodies[0].state();
        let target_com = aggregate_state
            .transform()
            .transform_point(aggregate_mass.center_of_mass());
        self.reduced_fixed_tree_target_centers.clear();
        for &body_index in &self.reduced_fixed_tree_dynamic_indices {
            let transform = aggregate_state
                .transform()
                .compose(&self.reduced_fixed_tree_offsets[body_index])
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            self.reduced_fixed_tree_target_centers.push(
                transform.transform_point(
                    bodies[body_index]
                        .body()
                        .physics()
                        .mass()
                        .ok_or(CollisionError::InvalidContact)?
                        .center_of_mass(),
                ),
            );
        }
        for (index, &body_index) in self.reduced_fixed_tree_dynamic_indices.iter().enumerate() {
            let body = &mut bodies[body_index];
            let transform = aggregate_state
                .transform()
                .compose(&self.reduced_fixed_tree_offsets[body_index])
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            let velocity = aggregate_state.linear_velocity()
                + aggregate_state
                    .angular_velocity()
                    .cross(self.reduced_fixed_tree_target_centers[index] - target_com);
            body.set_rigid_state(transform, velocity, aggregate_state.angular_velocity())
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
        }
        Ok(true)
    }

    fn prepare_constraint_coloring(
        &mut self,
        bodies: &[SolverBody],
        manifolds: &[SolverManifold],
        joints: &[JointConstraint],
    ) -> Result<bool, CollisionError> {
        let constraint_count = manifolds
            .len()
            .checked_add(joints.len())
            .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        if self.coloring_dynamic_bodies.capacity() < bodies.len() {
            self.coloring_dynamic_bodies
                .try_reserve(bodies.len() - self.coloring_dynamic_bodies.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        self.coloring_dynamic_bodies.clear();
        self.coloring_dynamic_bodies.extend(
            bodies.iter().map(|body| {
                body.body().physics().body_type() == crate::rigid_body::BodyType::Dynamic
            }),
        );
        if self.coloring_endpoints.capacity() < constraint_count {
            self.coloring_endpoints
                .try_reserve(constraint_count - self.coloring_endpoints.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        self.coloring_endpoints.clear();
        self.coloring_endpoints.extend(
            manifolds
                .iter()
                .map(|manifold| [manifold.body_a, manifold.body_b]),
        );
        self.coloring_endpoints
            .extend(joints.iter().map(|joint| [joint.body_a, joint.body_b]));
        self.constraint_coloring
            .update(&self.coloring_dynamic_bodies, &self.coloring_endpoints)
    }

    fn prepare_colored_batches(
        &mut self,
        manifolds: &[SolverManifold],
        joints: &[JointConstraint],
        graph_rebuilt: bool,
    ) -> Result<(), CollisionError> {
        let rebuild_batches = graph_rebuilt || self.active_colored_batches == 0;
        if rebuild_batches {
            let group_count = self.constraint_coloring.schedule_group_count();
            if self.color_batch_offsets.capacity() < group_count + 1 {
                self.color_batch_offsets
                    .try_reserve(group_count + 1 - self.color_batch_offsets.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            }
            self.color_batch_offsets.clear();
            self.color_batch_offsets.resize(group_count + 1, 0);
            let mut required_batches = 0_usize;
            for group in 0..group_count {
                let constraints = self
                    .constraint_coloring
                    .constraints_in_schedule_group(group)
                    .ok_or(CollisionError::InvalidContact)?;
                required_batches = required_batches
                    .checked_add(constraints.len().div_ceil(LARGE_ISLAND_BATCH_SIZE))
                    .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
            }
            if self.colored_batches.capacity() < required_batches {
                self.colored_batches
                    .try_reserve(required_batches - self.colored_batches.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            }
            for batch in &mut self.colored_batches {
                batch.tasks.clear();
                batch.contact_points.clear();
                batch.contact_task_count = 0;
                batch.error = None;
            }
            self.active_colored_batches = 0;
            if self.joint_task_locations.capacity() < joints.len() {
                self.joint_task_locations
                    .try_reserve(joints.len() - self.joint_task_locations.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            }
            self.joint_task_locations.resize(joints.len(), [0, 0]);
            for group in 0..group_count {
                let constraints = self
                    .constraint_coloring
                    .constraints_in_schedule_group(group)
                    .ok_or(CollisionError::InvalidContact)?;
                for chunk in constraints.chunks(LARGE_ISLAND_BATCH_SIZE) {
                    if self.active_colored_batches == self.colored_batches.len() {
                        self.colored_batches.push(ColoredConstraintBatch::new());
                    }
                    let batch = &mut self.colored_batches[self.active_colored_batches];
                    if batch.tasks.capacity() < chunk.len() {
                        batch
                            .tasks
                            .try_reserve(chunk.len() - batch.tasks.len())
                            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                    }
                    for constraint_index in chunk.iter().copied() {
                        let task_slot = batch.tasks.len();
                        batch.tasks.push(ColoredConstraintTask::new(
                            constraint_index,
                            manifolds,
                            joints,
                        )?);
                        let task = batch
                            .tasks
                            .last_mut()
                            .ok_or(CollisionError::InvalidContact)?;
                        let global_constraint = task.constraint_index;
                        let positions = self
                            .constraint_coloring
                            .body_positions_for_constraint(global_constraint)
                            .ok_or(CollisionError::InvalidContact)?;
                        task.set_binding_positions(positions);
                        if constraint_index < manifolds.len() {
                            batch.contact_task_count += 1;
                        } else {
                            let joint_index = constraint_index - manifolds.len();
                            self.joint_task_locations[joint_index] =
                                [self.active_colored_batches, task_slot];
                        }
                    }
                    batch.joint_workspace.begin_step();
                    self.active_colored_batches += 1;
                }
                self.color_batch_offsets[group + 1] = self.active_colored_batches;
            }
        } else {
            for batch in self
                .colored_batches
                .iter_mut()
                .take(self.active_colored_batches)
            {
                batch.error = None;
                batch.contact_points.clear();
                batch.joint_workspace.begin_step();
                let mut contact_task_count = 0;
                for task in &mut batch.tasks {
                    task.update(manifolds, joints)?;
                    if matches!(task.kind, ColoredConstraintKind::Contact(_)) {
                        contact_task_count += 1;
                    }
                }
                batch.contact_task_count = contact_task_count;
            }
        }
        Ok(())
    }

    fn colored_workload(&self, worker_count: usize) -> Option<ColoredWorkload> {
        let color_count = self.constraint_coloring.schedule_group_count();
        let worker_count = worker_count.max(1);
        let mut workload = ColoredWorkload {
            color_count,
            ..ColoredWorkload::default()
        };
        for group in 0..color_count {
            let constraints = self
                .constraint_coloring
                .constraints_in_schedule_group(group)?;
            workload.add_color(constraints.len(), worker_count);
        }
        Some(workload)
    }

    fn can_solve_colored_constraints(
        &self,
        options: JointSolverOptions,
        worker_count: usize,
        constraint_count: usize,
    ) -> bool {
        // Global multi-color Gauss-Seidel changes the scalar constraint order.
        // Keep it experimental until strict connected-island quality is proven.
        if !cfg!(feature = "bench-internals") {
            return false;
        }
        if worker_count <= 1
            || constraint_count < LARGE_ISLAND_COLORING_THRESHOLD
            || self.constraint_coloring.max_color_constraint_count() < SMALL_COLOR_MERGE_THRESHOLD
            || !matches!(options.iterations(), JointIterations::Fixed)
        {
            return false;
        }
        #[cfg(feature = "bench-internals")]
        if options.global_velocity_pcg_max_iterations() > 0
            || options.global_position_pcg_max_iterations() > 0
            || options.alternates_velocity_sweeps()
        {
            return false;
        }
        let Some(workload) = self.colored_workload(worker_count) else {
            return false;
        };
        #[cfg(feature = "bench-internals")]
        if options.partial_coloring() {
            return workload.can_use_partial_schedule(worker_count);
        }
        workload.can_use_parallel_schedule(worker_count)
    }

    #[allow(clippy::too_many_arguments)]
    fn solve_colored_constraints(
        &mut self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
        joint_workspace: &mut JointWorkspace,
        joints: &[JointConstraint],
        config: CollisionConfig,
        joint_solver_step: JointSolverStep<'_>,
        gravity: Option<CentralGravity>,
        executor: &impl crate::execution::StepExecutor,
        worker_count: usize,
    ) -> Result<(), CollisionError> {
        #[cfg(not(feature = "bench-internals"))]
        let _ = gravity;
        #[cfg(feature = "bench-internals")]
        let full_diagnostics = !joint_solver_step.light_diagnostics();
        if !manifolds.is_empty() {
            self.execute_colored_phase(
                bodies,
                manifolds,
                config,
                joint_solver_step,
                ColoredConstraintPhase::PrepareContactPoints,
                executor,
                worker_count,
            )?;
            self.execute_colored_phase(
                bodies,
                manifolds,
                config,
                joint_solver_step,
                ColoredConstraintPhase::WarmStartContacts,
                executor,
                worker_count,
            )?;
        }
        #[cfg(feature = "bench-internals")]
        if full_diagnostics {
            snapshot_velocities(bodies, &mut self.velocity_snapshots)?;
        }
        #[cfg(feature = "bench-internals")]
        {
            if full_diagnostics && !joints.is_empty() {
                JointSolver::measure_residual(
                    joint_workspace,
                    bodies,
                    joints,
                    joint_solver_step.dt(),
                )?;
                joint_workspace.record_initial_velocity_residual();
                joint_workspace.record_pre_velocity_geometry();
            }
        }

        #[cfg(feature = "bench-internals")]
        let sweep_started = std::time::Instant::now();
        for velocity_iteration in 0..config.velocity_iterations {
            #[cfg(feature = "bench-internals")]
            {
                joint_workspace.record_velocity_iteration();
                if full_diagnostics && !joints.is_empty() {
                    // Nothing changed since the previous measurement (the last
                    // post-velocity read or the initial one), so reuse it.
                    joint_workspace.record_pre_joint_velocity_residual();
                }
            }
            self.execute_colored_phase(
                bodies,
                manifolds,
                config,
                joint_solver_step,
                ColoredConstraintPhase::Velocity {
                    first_iteration: velocity_iteration == 0,
                },
                executor,
                worker_count,
            )?;
            #[cfg(feature = "bench-internals")]
            {
                joint_workspace.record_color_batches(self.active_colored_batches);
                if full_diagnostics && !joints.is_empty() {
                    // Velocity sweeps keep poses fixed, so the rows committed by
                    // each batch already describe the current geometry.
                    self.merge_colored_joint_rows(joint_workspace, joints)?;
                    JointSolver::measure_current_residual(
                        joint_workspace,
                        bodies,
                        joint_solver_step.dt(),
                    )?;
                    if velocity_iteration == 0 {
                        joint_workspace.record_first_velocity_residual();
                    }
                }
            }
        }
        #[cfg(feature = "bench-internals")]
        {
            self.colored_schedule.velocity_sweeps += sweep_started.elapsed();
        }

        #[cfg(feature = "bench-internals")]
        {
            if full_diagnostics {
                let (impulse_work, quadratic_work) =
                    velocity_projection_work(&self.velocity_snapshots, bodies)?;
                joint_workspace.record_velocity_projection_work(impulse_work, quadratic_work);
            }
            joint_workspace.record_energy(
                crate::joint::diagnostics::EnergyStage::AfterVelocitySolver,
                energy_components(bodies, gravity),
            );
            if full_diagnostics && !joints.is_empty() {
                // The last post-velocity read already reflects the final state.
                joint_workspace.record_post_velocity_geometry();
            }
        }

        #[cfg(feature = "bench-internals")]
        let sweep_started = std::time::Instant::now();
        for _position_iteration in 0..config.position_iterations {
            #[cfg(feature = "bench-internals")]
            {
                joint_workspace.record_position_iteration();
                if full_diagnostics && !joints.is_empty() {
                    // Nothing changed since the previous post-position read.
                    joint_workspace.record_pre_joint_geometry();
                }
            }
            self.execute_colored_phase(
                bodies,
                manifolds,
                config,
                joint_solver_step,
                ColoredConstraintPhase::Position,
                executor,
                worker_count,
            )?;
            #[cfg(feature = "bench-internals")]
            {
                joint_workspace.record_color_batches(self.active_colored_batches);
                if full_diagnostics && !joints.is_empty() {
                    JointSolver::measure_residual(
                        joint_workspace,
                        bodies,
                        joints,
                        joint_solver_step.dt(),
                    )?;
                    joint_workspace.record_post_joint_geometry(_position_iteration == 0);
                }
            }
        }
        #[cfg(feature = "bench-internals")]
        {
            self.colored_schedule.position_sweeps += sweep_started.elapsed();
        }

        self.merge_colored_joints(joint_workspace, joints, manifolds.len())?;
        #[cfg(feature = "bench-internals")]
        {
            joint_workspace.record_energy(
                crate::joint::diagnostics::EnergyStage::AfterPositionSolver,
                energy_components(bodies, gravity),
            );
            if full_diagnostics && !joints.is_empty() {
                if config.position_iterations == 0 {
                    JointSolver::measure_residual(
                        joint_workspace,
                        bodies,
                        joints,
                        joint_solver_step.dt(),
                    )?;
                }
                joint_workspace.record_post_joint_geometry(false);
            }
        }
        joint_workspace.collect_break_events(
            joints,
            bodies,
            joint_solver_step.dt(),
            joint_solver_step.options().reaction_mode(),
        )?;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn execute_colored_phase(
        &mut self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
        config: CollisionConfig,
        joint_solver_step: JointSolverStep<'_>,
        phase: ColoredConstraintPhase,
        executor: &impl crate::execution::StepExecutor,
        worker_count: usize,
    ) -> Result<(), CollisionError> {
        let contact_only = phase.is_contact_only();
        let group_count = self.color_batch_offsets.len().saturating_sub(1);
        for group in 0..group_count {
            let start = self.color_batch_offsets[group];
            let end = self.color_batch_offsets[group + 1];
            if start == end {
                continue;
            }
            let batches = &self.colored_batches[start..end];
            let active_batch_count = batches
                .iter()
                .filter(|batch| !contact_only || batch.has_contacts())
                .count();
            if active_batch_count == 0 {
                continue;
            }
            let constraint_count = batches
                .iter()
                .filter(|batch| !contact_only || batch.has_contacts())
                .map(|batch| {
                    if contact_only {
                        batch.contact_task_count
                    } else {
                        batch.tasks.len()
                    }
                })
                .sum::<usize>();
            // Contact constraints precede joints in each color, so inactive
            // joint batches form a suffix that does not need to be dispatched.
            let process_end = if contact_only {
                let prefix = batches
                    .iter()
                    .take_while(|batch| batch.has_contacts())
                    .count();
                if prefix == active_batch_count {
                    start + prefix
                } else {
                    end
                }
            } else {
                end
            };
            let use_parallel = active_batch_count >= COLORED_MIN_PARALLEL_BATCHES
                && constraint_count > SMALL_COLOR_MERGE_THRESHOLD;
            let indices = self
                .constraint_coloring
                .bodies_in_schedule_group(group)
                .ok_or(CollisionError::InvalidContact)?;
            let mut bound_batches = bind_colored_batches(
                &mut self.colored_batches[start..process_end],
                bodies,
                indices,
            )?;
            if !use_parallel {
                for batch in &mut bound_batches {
                    Self::process_colored_batch(batch, config, joint_solver_step, phase)?;
                }
            } else {
                #[cfg(feature = "bench-internals")]
                {
                    self.colored_groups_dispatched =
                        self.colored_groups_dispatched.saturating_add(1);
                    self.colored_parallel_batches = self
                        .colored_parallel_batches
                        .saturating_add(active_batch_count);
                }
                let batch_count = process_end - start;
                let task_count = worker_count
                    .min(31)
                    .saturating_mul(2)
                    .min(batch_count)
                    .max(1);
                let chunk_size = batch_count.div_ceil(task_count);
                #[cfg(feature = "bench-internals")]
                let started = std::sync::atomic::AtomicUsize::new(0);
                let batch_result =
                    executor.for_each_chunk_mut(&mut bound_batches, chunk_size, |_, batches| {
                        #[cfg(feature = "bench-internals")]
                        started.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        for batch in batches {
                            *batch.error = Self::process_colored_batch(
                                batch,
                                config,
                                joint_solver_step,
                                phase,
                            )
                            .err();
                        }
                    });
                #[cfg(feature = "bench-internals")]
                {
                    self.colored_schedule.tasks_started +=
                        started.load(std::sync::atomic::Ordering::Relaxed);
                }
                if batch_result.is_err() {
                    return Err(CollisionError::WorkerFailed);
                }
                if let Some(error) = bound_batches
                    .iter_mut()
                    .find_map(|batch| batch.error.take())
                {
                    return Err(error);
                }
            }

            drop(bound_batches);

            if matches!(phase, ColoredConstraintPhase::Velocity { .. }) {
                for batch in &self.colored_batches[start..end] {
                    for task in &batch.tasks {
                        if let ColoredConstraintKind::Contact(manifold) = &task.kind {
                            let destination = manifolds
                                .get_mut(task.constraint_index)
                                .ok_or(CollisionError::InvalidContact)?;
                            *destination = SolverManifold {
                                body_a: task.body_indices[0],
                                body_b: task.body_indices[1],
                                ..*manifold
                            };
                        }
                    }
                }
            }
        }
        Ok(())
    }

    fn process_colored_batch(
        batch: &mut BoundColoredBatch<'_>,
        config: CollisionConfig,
        joint_solver_step: JointSolverStep<'_>,
        phase: ColoredConstraintPhase,
    ) -> Result<(), CollisionError> {
        let dt = joint_solver_step.dt();
        let options = joint_solver_step.options();
        let contact_only = phase.is_contact_only();
        for (task_slot, bound) in batch.tasks.iter_mut().enumerate() {
            let task = &mut *bound.task;
            if contact_only && !matches!(task.kind, ColoredConstraintKind::Contact(_)) {
                continue;
            }
            let [a, b] = &mut bound.bodies;
            let mut bodies = colored_bodies::BoundPair([a.reborrow(), b.reborrow()]);
            match &mut task.kind {
                ColoredConstraintKind::Contact(manifold) => {
                    let mut local_manifold = SolverManifold {
                        body_a: 0,
                        body_b: 1,
                        ..*manifold
                    };
                    let mut local_manifolds = [local_manifold];
                    match phase {
                        ColoredConstraintPhase::PrepareContactPoints => {
                            Self::prepare_points_into(
                                batch.contact_points,
                                &bodies,
                                &local_manifolds,
                                config,
                            )?;
                            if batch.contact_points.len() > task.points.len() {
                                return Err(CollisionError::InvalidContact);
                            }
                            task.point_count = batch.contact_points.len();
                            task.points[..task.point_count].copy_from_slice(batch.contact_points);
                        }
                        ColoredConstraintPhase::WarmStartContacts => {
                            Self::warm_start_points(
                                &task.points[..task.point_count],
                                &mut bodies,
                                &local_manifolds,
                            )?;
                        }
                        ColoredConstraintPhase::Velocity { .. } => {
                            Self::solve_velocity_points(
                                &task.points[..task.point_count],
                                &mut bodies,
                                &mut local_manifolds,
                            )?;
                            local_manifold = local_manifolds[0];
                            *manifold = local_manifold;
                        }
                        ColoredConstraintPhase::Position => {
                            Self::solve_position_points(
                                &task.points[..task.point_count],
                                &mut bodies,
                                &local_manifolds,
                                config,
                            )?;
                        }
                    }
                }
                ColoredConstraintKind::Joint(joint) => {
                    let local_joint = JointConstraint {
                        body_a: 0,
                        body_b: 1,
                        ..*joint
                    };
                    match phase {
                        ColoredConstraintPhase::PrepareContactPoints
                        | ColoredConstraintPhase::WarmStartContacts => {}
                        ColoredConstraintPhase::Velocity {
                            first_iteration, ..
                        } => {
                            // Poses stay fixed during the velocity phase, so rows
                            // are built once and every later iteration reuses them.
                            if first_iteration {
                                JointSolver::prepare_velocity_joint(
                                    batch.joint_workspace,
                                    &bodies,
                                    local_joint,
                                    task_slot,
                                    dt,
                                )?;
                            }
                            JointSolver::solve_velocity_joint_prepared(
                                batch.joint_workspace,
                                &mut bodies,
                                local_joint,
                                task_slot,
                                joint_solver_step,
                                first_iteration,
                            )?;
                        }
                        ColoredConstraintPhase::Position => {
                            JointSolver::solve_position_joint(
                                batch.joint_workspace,
                                &mut bodies,
                                local_joint,
                                task_slot,
                                dt,
                                options,
                            )?;
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Copies the latest per-batch joint rows into the island workspace so
    /// diagnostics can measure them without rebuilding geometry.
    fn merge_colored_joint_rows(
        &self,
        destination: &mut JointWorkspace,
        joints: &[JointConstraint],
    ) -> Result<(), CollisionError> {
        for (joint_index, joint) in joints.iter().copied().enumerate() {
            let [batch_index, task_slot] = *self
                .joint_task_locations
                .get(joint_index)
                .ok_or(CollisionError::InvalidContact)?;
            let batch = self
                .colored_batches
                .get(batch_index)
                .ok_or(CollisionError::InvalidContact)?;
            destination.merge_colored_joint_from(
                &batch.joint_workspace,
                task_slot,
                joint_index,
                joint.id,
                [joint.body_a, joint.body_b],
            )?;
        }
        Ok(())
    }

    fn merge_colored_joints(
        &self,
        destination: &mut JointWorkspace,
        joints: &[JointConstraint],
        manifold_count: usize,
    ) -> Result<(), CollisionError> {
        self.merge_colored_joint_rows(destination, joints)?;
        for batch in self
            .colored_batches
            .iter()
            .take(self.active_colored_batches)
        {
            destination.accumulate_colored_metrics_from(&batch.joint_workspace);
        }
        if joints.len() + manifold_count != self.coloring_endpoints.len() {
            return Err(CollisionError::InvalidContact);
        }
        Ok(())
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        self.points.capacity() * std::mem::size_of::<SolverPoint>()
            + self.velocity_snapshots.capacity() * std::mem::size_of::<VelocitySnapshot>()
            + self.constraint_coloring.retained_capacity_bytes()
            + self.coloring_dynamic_bodies.capacity() * std::mem::size_of::<bool>()
            + self.coloring_endpoints.capacity() * std::mem::size_of::<[usize; 2]>()
            + self.color_batch_offsets.capacity() * std::mem::size_of::<usize>()
            + self.joint_task_locations.capacity() * std::mem::size_of::<[usize; 2]>()
            + self.colored_batches.capacity() * std::mem::size_of::<ColoredConstraintBatch>()
            + self
                .colored_batches
                .iter()
                .map(|batch| {
                    batch.tasks.capacity() * std::mem::size_of::<ColoredConstraintTask>()
                        + batch.joint_workspace.retained_capacity_bytes()
                        + batch.contact_points.capacity() * std::mem::size_of::<SolverPoint>()
                })
                .sum::<usize>()
    }

    pub(crate) fn solve(
        &mut self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        config.validate()?;
        self.prepare_points(bodies, manifolds, config)?;
        self.warm_start(bodies, manifolds)?;
        for _ in 0..config.velocity_iterations {
            self.solve_velocity_iteration(bodies, manifolds)?;
        }
        self.solve_position_constraints(bodies, manifolds, config)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn solve_with_joints(
        &mut self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
        joint_workspace: &mut JointWorkspace,
        joints: &[JointConstraint],
        config: CollisionConfig,
        joint_solver_step: JointSolverStep<'_>,
        gravity: Option<CentralGravity>,
        executor: &impl crate::execution::StepExecutor,
        worker_count: usize,
    ) -> Result<(), CollisionError> {
        #[cfg(not(feature = "bench-internals"))]
        let _ = gravity;
        #[cfg(feature = "bench-internals")]
        let full_diagnostics = !joint_solver_step.light_diagnostics();
        config.validate()?;
        let dt = joint_solver_step.dt();
        let joint_solver_options = joint_solver_step.options();
        if !dt.is_finite() || dt <= 0.0 || !joint_solver_options.iterations().validate() {
            return Err(CollisionError::InvalidConfig);
        }
        #[cfg(feature = "bench-internals")]
        self.reset_colored_dispatch_metrics();
        JointSolver::begin_step(joint_workspace);
        #[cfg(feature = "bench-internals")]
        if joint_solver_options.reduced_fixed_tree()
            && self.solve_reduced_fixed_tree(bodies, manifolds, joints, config)?
        {
            self.colored_schedule.constraints = manifolds.len().saturating_add(joints.len());
            self.colored_schedule.rejection = "reduced-fixed-tree";
            return Ok(());
        }
        let coloring_constraint_count = manifolds.len().saturating_add(joints.len());
        #[cfg(feature = "bench-internals")]
        {
            self.colored_schedule.constraints = coloring_constraint_count;
            self.colored_schedule.rejection = "small-or-single-worker";
        }
        if cfg!(feature = "bench-internals")
            && worker_count > 1
            && coloring_constraint_count >= LARGE_ISLAND_COLORING_THRESHOLD
        {
            #[cfg(feature = "bench-internals")]
            let coloring_started = std::time::Instant::now();
            let graph_rebuilt = self.prepare_constraint_coloring(bodies, manifolds, joints)?;
            #[cfg(feature = "bench-internals")]
            joint_workspace.record_constraint_coloring(
                coloring_started.elapsed(),
                self.constraint_coloring.constraint_count(),
                self.constraint_coloring.color_count(),
                graph_rebuilt,
            );
            #[cfg(feature = "bench-internals")]
            if let Some(workload) = self.colored_workload(worker_count) {
                let mut report = super::ColoredScheduleMetrics {
                    rejection: workload
                        .rejection_reason(worker_count)
                        .unwrap_or("configuration"),
                    constraints: workload.constraint_count,
                    colors: workload.color_count,
                    batches: workload.batch_count,
                    useful_batches: workload.useful_batch_count,
                    ..self.colored_schedule
                };
                for group in 0..workload.color_count {
                    if let Some(constraints) = self
                        .constraint_coloring
                        .constraints_in_schedule_group(group)
                    {
                        if constraints.len().div_ceil(LARGE_ISLAND_BATCH_SIZE)
                            >= COLORED_MIN_PARALLEL_BATCHES
                            && constraints.len() > SMALL_COLOR_MERGE_THRESHOLD
                        {
                            report.parallel_colors += 1;
                            report.parallel_constraints += constraints.len();
                        } else {
                            report.sequential_constraints += constraints.len();
                        }
                    }
                }
                self.colored_schedule = report;
            }
            if self.can_solve_colored_constraints(
                joint_solver_options,
                worker_count,
                coloring_constraint_count,
            ) {
                #[cfg(feature = "bench-internals")]
                {
                    self.colored_solver_selected = true;
                    self.colored_schedule.rejection = "selected";
                }
                self.prepare_colored_batches(manifolds, joints, graph_rebuilt)?;
                return self.solve_colored_constraints(
                    bodies,
                    manifolds,
                    joint_workspace,
                    joints,
                    config,
                    joint_solver_step,
                    gravity,
                    executor,
                    worker_count,
                );
            }
        }
        if joints.is_empty() {
            return self.solve(bodies, manifolds, config);
        }
        self.prepare_points(bodies, manifolds, config)?;
        self.warm_start(bodies, manifolds)?;
        #[cfg(feature = "bench-internals")]
        if full_diagnostics {
            snapshot_velocities(bodies, &mut self.velocity_snapshots)?;
        }
        #[cfg(feature = "bench-internals")]
        let mut velocity_pcg_before_recorded = false;
        #[cfg(feature = "bench-internals")]
        let mut position_pcg_before_recorded = false;

        if let JointIterations::Adaptive {
            position,
            velocity,
            tolerance,
        } = joint_solver_options.iterations()
        {
            self.solve_adaptive_joint_iterations(
                bodies,
                manifolds,
                joint_workspace,
                joints,
                config,
                joint_solver_step,
                position,
                velocity,
                tolerance,
            )?;
            joint_workspace.collect_break_events(
                joints,
                bodies,
                dt,
                joint_solver_options.reaction_mode(),
            )?;
            return Ok(());
        }

        // Velocity sweeps leave poses, masses, and the constraint set unchanged,
        // so rows can be prepared once and reused across every iteration.
        #[cfg(feature = "bench-internals")]
        let reuse_velocity_rows = matches!(
            joint_solver_options.projection(),
            crate::joint::JointProjection::ScalarGaussSeidel
                | crate::joint::JointProjection::FixedBlock3
        ) && !joint_solver_options.warm_start().enabled
            && !joint_solver_options.alternates_velocity_sweeps()
            && joint_solver_options.global_velocity_pcg_max_iterations() == 0;
        #[cfg(not(feature = "bench-internals"))]
        let reuse_velocity_rows = matches!(
            joint_solver_options.projection(),
            crate::joint::JointProjection::ScalarGaussSeidel
                | crate::joint::JointProjection::FixedBlock3
        ) && !joint_solver_options.warm_start().enabled;
        if reuse_velocity_rows {
            JointSolver::prepare(joint_workspace, bodies, joints, dt)?;
        }

        #[cfg(feature = "bench-internals")]
        {
            if full_diagnostics {
                if reuse_velocity_rows {
                    JointSolver::measure_current_residual(joint_workspace, bodies, dt)?;
                } else {
                    JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                }
                joint_workspace.record_initial_velocity_residual();
                joint_workspace.record_pre_velocity_geometry();
            }
        }
        #[cfg(feature = "bench-internals")]
        let sweep_started = std::time::Instant::now();
        let direct_velocity = manifolds.is_empty()
            && reuse_velocity_rows
            && joint_solver_options.projection()
                == crate::joint::JointProjection::ScalarGaussSeidel
            && JointSolver::solve_direct_velocity(
                joint_workspace,
                bodies,
                joint_solver_options.mass_cache(),
            )?;
        #[cfg(feature = "bench-internals")]
        if direct_velocity {
            joint_workspace.record_velocity_iteration();
            joint_workspace.record_scalar_rows_solved(joint_workspace.rows().len());
            if full_diagnostics {
                joint_workspace.record_pre_joint_velocity_residual();
                JointSolver::measure_current_residual(joint_workspace, bodies, dt)?;
                joint_workspace.record_first_velocity_residual();
            }
        }
        for velocity_iteration in 0..config.velocity_iterations {
            if direct_velocity {
                break;
            }
            #[cfg(feature = "bench-internals")]
            joint_workspace.record_velocity_iteration();
            self.solve_velocity_iteration(bodies, manifolds)?;
            #[cfg(feature = "bench-internals")]
            {
                if full_diagnostics {
                    // Velocity sweeps leave poses unchanged, so rows prepared by
                    // the previous sweep (or the one-time preparation) are current.
                    if reuse_velocity_rows || velocity_iteration > 0 {
                        JointSolver::measure_current_residual(joint_workspace, bodies, dt)?;
                    } else {
                        JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                    }
                    joint_workspace.record_pre_joint_velocity_residual();
                }
            }
            #[cfg(feature = "bench-internals")]
            let reverse_joint_sweep =
                joint_solver_options.alternates_velocity_sweeps() && velocity_iteration % 2 == 1;
            #[cfg(feature = "bench-internals")]
            {
                let pcg_max_iterations = joint_solver_options.global_velocity_pcg_max_iterations();
                if reuse_velocity_rows {
                    JointSolver::solve_velocity_prepared_iteration(
                        joint_workspace,
                        bodies,
                        joints,
                        dt,
                        joint_solver_options,
                    )?;
                } else if pcg_max_iterations > 0 {
                    let energy_before_pcg = energy_components(bodies, gravity);
                    #[cfg(feature = "bench-internals")]
                    if !velocity_pcg_before_recorded {
                        joint_workspace.record_energy(
                            crate::joint::diagnostics::EnergyStage::BeforeVelocityPcg,
                            energy_before_pcg,
                        );
                        velocity_pcg_before_recorded = true;
                    }
                    let incomplete_cholesky_preconditioner =
                        joint_solver_options.global_velocity_pcg_incomplete_cholesky();
                    let block_jacobi_preconditioner =
                        joint_solver_options.global_velocity_pcg_block_jacobi();
                    let pcg_result = JointSolver::solve_velocity_pcg(
                        joint_workspace,
                        bodies,
                        joints,
                        dt,
                        pcg_max_iterations,
                        incomplete_cholesky_preconditioner,
                        block_jacobi_preconditioner,
                        executor,
                        worker_count,
                    )?;
                    #[cfg(feature = "bench-internals")]
                    if pcg_result.is_some() {
                        let energy_after_pcg = energy_components(bodies, gravity);
                        joint_workspace.record_energy(
                            crate::joint::diagnostics::EnergyStage::AfterVelocityPcg,
                            energy_after_pcg,
                        );
                        joint_workspace.record_energy_delta(
                            crate::joint::diagnostics::EnergyStage::VelocityPcgDelta,
                            energy_before_pcg,
                            energy_after_pcg,
                        );
                    }
                    if pcg_result.is_none() {
                        self.solve_joint_velocity_sweep(
                            bodies,
                            joint_workspace,
                            joints,
                            joint_solver_step,
                            velocity_iteration == 0,
                            reverse_joint_sweep,
                        )?;
                    }
                } else {
                    self.solve_joint_velocity_sweep(
                        bodies,
                        joint_workspace,
                        joints,
                        joint_solver_step,
                        velocity_iteration == 0,
                        reverse_joint_sweep,
                    )?;
                }
            }
            #[cfg(not(feature = "bench-internals"))]
            if reuse_velocity_rows {
                JointSolver::solve_velocity_prepared_iteration(
                    joint_workspace,
                    bodies,
                    joints,
                    dt,
                    joint_solver_options,
                )?;
            } else {
                self.solve_joint_velocity_sweep(
                    bodies,
                    joint_workspace,
                    joints,
                    joint_solver_step,
                    velocity_iteration == 0,
                    false,
                )?;
            }
            #[cfg(feature = "bench-internals")]
            {
                if full_diagnostics {
                    // The sweep just committed rows for the current poses.
                    JointSolver::measure_current_residual(joint_workspace, bodies, dt)?;
                    if velocity_iteration == 0 {
                        joint_workspace.record_first_velocity_residual();
                    }
                }
            }
        }
        #[cfg(feature = "bench-internals")]
        {
            self.colored_schedule.velocity_sweeps += sweep_started.elapsed();
        }
        #[cfg(feature = "bench-internals")]
        {
            if full_diagnostics {
                let (impulse_work, quadratic_work) =
                    velocity_projection_work(&self.velocity_snapshots, bodies)?;
                joint_workspace.record_velocity_projection_work(impulse_work, quadratic_work);
            }
        }
        #[cfg(feature = "bench-internals")]
        joint_workspace.record_energy(
            crate::joint::diagnostics::EnergyStage::AfterVelocitySolver,
            energy_components(bodies, gravity),
        );
        #[cfg(feature = "bench-internals")]
        {
            if full_diagnostics {
                // The last velocity sweep committed rows without changing poses.
                JointSolver::measure_current_residual(joint_workspace, bodies, dt)?;
                joint_workspace.record_post_velocity_geometry();
            }
        }
        #[cfg(feature = "bench-internals")]
        let sweep_started = std::time::Instant::now();
        for _position_iteration in 0..config.position_iterations {
            #[cfg(feature = "bench-internals")]
            joint_workspace.record_position_iteration();
            self.solve_position_iteration(bodies, manifolds, config)?;
            #[cfg(feature = "bench-internals")]
            {
                if full_diagnostics {
                    // Contact position corrections may already have moved bodies.
                    JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                    joint_workspace.record_pre_joint_geometry();
                }
            }
            #[cfg(feature = "bench-internals")]
            {
                let pcg_max_iterations = joint_solver_options.global_position_pcg_max_iterations();
                if pcg_max_iterations > 0 {
                    let energy_before_pcg = energy_components(bodies, gravity);
                    #[cfg(feature = "bench-internals")]
                    if !position_pcg_before_recorded {
                        joint_workspace.record_energy(
                            crate::joint::diagnostics::EnergyStage::BeforePositionPcg,
                            energy_before_pcg,
                        );
                        position_pcg_before_recorded = true;
                    }
                    let incomplete_cholesky_preconditioner =
                        joint_solver_options.global_position_pcg_incomplete_cholesky();
                    let pcg_iterations = JointSolver::solve_position_pcg(
                        joint_workspace,
                        bodies,
                        joints,
                        dt,
                        pcg_max_iterations,
                        incomplete_cholesky_preconditioner,
                    )?;
                    #[cfg(feature = "bench-internals")]
                    if pcg_iterations.is_some() {
                        let energy_after_pcg = energy_components(bodies, gravity);
                        joint_workspace.record_energy(
                            crate::joint::diagnostics::EnergyStage::AfterPositionPcg,
                            energy_after_pcg,
                        );
                        joint_workspace.record_energy_delta(
                            crate::joint::diagnostics::EnergyStage::PositionPcgDelta,
                            energy_before_pcg,
                            energy_after_pcg,
                        );
                    }
                    if pcg_iterations.is_none() {
                        self.solve_joint_position_sweep(
                            bodies,
                            joint_workspace,
                            joints,
                            dt,
                            joint_solver_options,
                        )?;
                    }
                } else {
                    self.solve_joint_position_sweep(
                        bodies,
                        joint_workspace,
                        joints,
                        dt,
                        joint_solver_options,
                    )?;
                }
            }
            #[cfg(not(feature = "bench-internals"))]
            self.solve_joint_position_sweep(
                bodies,
                joint_workspace,
                joints,
                dt,
                joint_solver_options,
            )?;
            #[cfg(feature = "bench-internals")]
            {
                if full_diagnostics {
                    JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                    joint_workspace.record_post_joint_geometry(_position_iteration == 0);
                }
            }
        }
        #[cfg(feature = "bench-internals")]
        {
            self.colored_schedule.position_sweeps += sweep_started.elapsed();
        }
        #[cfg(feature = "bench-internals")]
        joint_workspace.record_energy(
            crate::joint::diagnostics::EnergyStage::AfterPositionSolver,
            energy_components(bodies, gravity),
        );
        joint_workspace.collect_break_events(
            joints,
            bodies,
            dt,
            joint_solver_options.reaction_mode(),
        )?;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn solve_adaptive_joint_iterations(
        &mut self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
        joint_workspace: &mut JointWorkspace,
        joints: &[JointConstraint],
        config: CollisionConfig,
        joint_solver_step: JointSolverStep<'_>,
        position: IterationRange,
        velocity: IterationRange,
        tolerance: JointTolerance,
    ) -> Result<(), CollisionError> {
        let dt = joint_solver_step.dt();
        #[cfg(feature = "bench-internals")]
        let full_diagnostics = !joint_solver_step.light_diagnostics();
        #[cfg(not(feature = "bench-internals"))]
        let full_diagnostics = true;
        let velocity_outer = config.velocity_iterations.max(velocity.max);
        let mut joint_velocity_sweeps = 0;
        #[cfg(feature = "bench-internals")]
        {
            if full_diagnostics {
                JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                joint_workspace.record_initial_velocity_residual();
                joint_workspace.record_pre_velocity_geometry();
            }
        }
        for outer_iteration in 0..velocity_outer {
            if outer_iteration < config.velocity_iterations {
                self.solve_velocity_iteration(bodies, manifolds)?;
            }
            #[cfg(feature = "bench-internals")]
            {
                if full_diagnostics {
                    JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                    joint_workspace.record_pre_joint_velocity_residual();
                }
            }
            let residual_converged = if joint_velocity_sweeps >= velocity.min {
                measure_joint_residual(joint_workspace, bodies, joints, dt, full_diagnostics)?;
                joint_workspace.residual().velocity_converged(tolerance)
            } else {
                false
            };
            if joint_velocity_sweeps == velocity.max || residual_converged {
                continue;
            }
            #[cfg(feature = "bench-internals")]
            joint_workspace.record_velocity_iteration();
            #[cfg(feature = "bench-internals")]
            let reverse_joint_sweep = joint_solver_step.options().alternates_velocity_sweeps()
                && joint_velocity_sweeps % 2 == 1;
            #[cfg(not(feature = "bench-internals"))]
            let reverse_joint_sweep = false;
            self.solve_joint_velocity_sweep(
                bodies,
                joint_workspace,
                joints,
                joint_solver_step,
                joint_velocity_sweeps == 0,
                reverse_joint_sweep,
            )?;
            joint_velocity_sweeps += 1;
            measure_joint_residual(joint_workspace, bodies, joints, dt, full_diagnostics)?;
            #[cfg(feature = "bench-internals")]
            if joint_velocity_sweeps == 1 {
                joint_workspace.record_first_velocity_residual();
            }
        }
        #[cfg(feature = "bench-internals")]
        {
            if full_diagnostics {
                JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                joint_workspace.record_post_velocity_geometry();
            }
        }

        let position_outer = config.position_iterations.max(position.max);
        let mut joint_position_sweeps = 0;
        for outer_iteration in 0..position_outer {
            if outer_iteration < config.position_iterations {
                self.solve_position_iteration(bodies, manifolds, config)?;
            }
            #[cfg(feature = "bench-internals")]
            {
                if full_diagnostics {
                    JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                    joint_workspace.record_pre_joint_geometry();
                }
            }
            let residual_converged = if joint_position_sweeps >= position.min {
                measure_joint_residual(joint_workspace, bodies, joints, dt, full_diagnostics)?;
                joint_workspace.residual().position_converged(tolerance)
            } else {
                false
            };
            if joint_position_sweeps == position.max || residual_converged {
                continue;
            }
            #[cfg(feature = "bench-internals")]
            joint_workspace.record_position_iteration();
            for (joint_index, joint) in joints.iter().copied().enumerate() {
                JointSolver::solve_position_joint(
                    joint_workspace,
                    bodies,
                    joint,
                    joint_index,
                    dt,
                    joint_solver_step.options(),
                )?;
            }
            joint_position_sweeps += 1;
            measure_joint_residual(joint_workspace, bodies, joints, dt, full_diagnostics)?;
            #[cfg(feature = "bench-internals")]
            joint_workspace.record_post_joint_geometry(joint_position_sweeps == 1);
        }
        #[cfg(feature = "bench-internals")]
        {
            if full_diagnostics {
                JointSolver::measure_residual(joint_workspace, bodies, joints, dt)?;
                joint_workspace.record_post_joint_geometry(false);
            }
        }
        Ok(())
    }

    fn solve_joint_velocity_sweep(
        &mut self,
        bodies: &mut [SolverBody],
        joint_workspace: &mut JointWorkspace,
        joints: &[JointConstraint],
        joint_solver_step: JointSolverStep<'_>,
        first_velocity_iteration: bool,
        reverse_joint_order: bool,
    ) -> Result<(), CollisionError> {
        for offset in 0..joints.len() {
            let joint_index = if reverse_joint_order {
                joints.len() - 1 - offset
            } else {
                offset
            };
            JointSolver::solve_velocity_joint(
                joint_workspace,
                bodies,
                joints[joint_index],
                joint_index,
                joint_solver_step,
                first_velocity_iteration,
            )?;
        }
        Ok(())
    }

    fn solve_joint_position_sweep(
        &mut self,
        bodies: &mut [SolverBody],
        joint_workspace: &mut JointWorkspace,
        joints: &[JointConstraint],
        dt: f64,
        options: JointSolverOptions,
    ) -> Result<(), CollisionError> {
        for (joint_index, joint) in joints.iter().copied().enumerate() {
            JointSolver::solve_position_joint(
                joint_workspace,
                bodies,
                joint,
                joint_index,
                dt,
                options,
            )?;
        }
        Ok(())
    }

    fn prepare_points(
        &mut self,
        bodies: &[SolverBody],
        manifolds: &[SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        Self::prepare_points_into(&mut self.points, bodies, manifolds, config)
    }

    fn prepare_points_into<B: SolverBodyAccess + ?Sized>(
        points: &mut Vec<SolverPoint>,
        bodies: &B,
        manifolds: &[SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        points.clear();
        let count = manifolds.iter().try_fold(0_usize, |count, manifold| {
            count
                .checked_add(manifold.manifold.len())
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)
        })?;
        points
            .try_reserve(count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;

        for (manifold_index, manifold) in manifolds.iter().enumerate() {
            if manifold.body_a == manifold.body_b
                || manifold.body_a >= bodies.len()
                || manifold.body_b >= bodies.len()
            {
                return Err(CollisionError::InvalidContact);
            }
            validate_material(manifold.material)?;
            let body_a = bodies
                .get(manifold.body_a)
                .ok_or(CollisionError::InvalidContact)?;
            let body_b = bodies
                .get(manifold.body_b)
                .ok_or(CollisionError::InvalidContact)?;
            for (point_index, contact) in manifold.manifold.points().iter().enumerate() {
                validate_contact(*contact)?;
                let normal = contact
                    .normal
                    .try_normalize()
                    .ok_or(CollisionError::InvalidContact)?;
                let (tangent_a, tangent_b) = tangent_basis(normal)?;
                let position_a = body_a
                    .body
                    .state()
                    .transform()
                    .transform_point(contact.local_a);
                let position_b = body_b
                    .body
                    .state()
                    .transform()
                    .transform_point(contact.local_b);
                let relative_velocity =
                    relative_velocity(&body_a.body, &body_b.body, position_a, position_b)?;
                let normal_speed = relative_velocity
                    .checked_dot(normal)
                    .ok_or(CollisionError::ArithmeticOverflow)?;
                let restitution_target = if normal_speed <= -config.restitution_threshold {
                    manifold.material.restitution * -normal_speed
                } else {
                    0.0
                };
                if !position_a.is_finite()
                    || !position_b.is_finite()
                    || !restitution_target.is_finite()
                {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                points.push(SolverPoint {
                    manifold: manifold_index,
                    point: point_index,
                    body_a: manifold.body_a,
                    body_b: manifold.body_b,
                    position_a,
                    position_b,
                    normal,
                    tangent_a,
                    tangent_b,
                    restitution_target,
                });
            }
        }
        Ok(())
    }

    fn warm_start(
        &self,
        bodies: &mut [SolverBody],
        manifolds: &[SolverManifold],
    ) -> Result<(), CollisionError> {
        Self::warm_start_points(&self.points, bodies, manifolds)
    }

    fn warm_start_points<B: SolverBodyAccess + ?Sized>(
        points: &[SolverPoint],
        bodies: &mut B,
        manifolds: &[SolverManifold],
    ) -> Result<(), CollisionError> {
        for point in points.iter().copied() {
            let contact = contact_at(manifolds, point)?;
            let impulse = point.normal * contact.normal_impulse
                + point.tangent_a * contact.tangent_impulse[0]
                + point.tangent_b * contact.tangent_impulse[1];
            apply_pair_impulse(
                bodies,
                point.body_a,
                point.body_b,
                point.position_a,
                point.position_b,
                impulse,
            )?;
        }
        Ok(())
    }

    fn solve_velocity_iteration(
        &self,
        bodies: &mut [SolverBody],
        manifolds: &mut [SolverManifold],
    ) -> Result<(), CollisionError> {
        Self::solve_velocity_points(&self.points, bodies, manifolds)
    }

    fn solve_velocity_points<B: SolverBodyAccess + ?Sized>(
        points: &[SolverPoint],
        bodies: &mut B,
        manifolds: &mut [SolverManifold],
    ) -> Result<(), CollisionError> {
        for point in points.iter().copied() {
            let contact = *contact_at(manifolds, point)?;
            let (body_a, body_b) = bodies_at(bodies, point.body_a, point.body_b)?;
            let denominator = effective_mass(
                &body_a.body,
                &body_b.body,
                point.position_a,
                point.position_b,
                point.normal,
            )?;
            if denominator <= f64::MIN_POSITIVE {
                return Err(CollisionError::InvalidContact);
            }
            let velocity = relative_velocity(
                &body_a.body,
                &body_b.body,
                point.position_a,
                point.position_b,
            )?;
            let normal_speed = velocity
                .checked_dot(point.normal)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let delta = (point.restitution_target - normal_speed) / denominator;
            if !delta.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            let next_normal_impulse = (contact.normal_impulse + delta).max(0.0);
            let normal_delta = next_normal_impulse - contact.normal_impulse;
            if normal_delta != 0.0 {
                apply_pair_impulse(
                    bodies,
                    point.body_a,
                    point.body_b,
                    point.position_a,
                    point.position_b,
                    point.normal * normal_delta,
                )?;
                let contact = contact_mut(manifolds, point)?;
                contact.normal_impulse = next_normal_impulse;
                if normal_delta > 0.0 {
                    let total_delta = contact.normal_impulse_increase_sum + normal_delta;
                    if !total_delta.is_finite() {
                        return Err(CollisionError::ArithmeticOverflow);
                    }
                    contact.normal_impulse_increase_sum = total_delta;
                }
            }

            let contact = *contact_at(manifolds, point)?;
            let material = manifolds
                .get(point.manifold)
                .ok_or(CollisionError::InvalidContact)?
                .material;
            let (body_a, body_b) = bodies_at(bodies, point.body_a, point.body_b)?;
            let velocity = relative_velocity(
                &body_a.body,
                &body_b.body,
                point.position_a,
                point.position_b,
            )?;
            let (tangent_impulse, delta_a) = solve_tangent_impulse(
                &body_a.body,
                &body_b.body,
                point,
                contact,
                material,
                velocity,
            )?;
            if delta_a != Vec3::ZERO {
                apply_pair_impulse(
                    bodies,
                    point.body_a,
                    point.body_b,
                    point.position_a,
                    point.position_b,
                    delta_a,
                )?;
            }
            contact_mut(manifolds, point)?.tangent_impulse = tangent_impulse;
        }
        Ok(())
    }

    fn solve_position_constraints(
        &self,
        bodies: &mut [SolverBody],
        manifolds: &[SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        for _ in 0..config.position_iterations {
            self.solve_position_iteration(bodies, manifolds, config)?;
        }
        Ok(())
    }

    fn solve_position_iteration(
        &self,
        bodies: &mut [SolverBody],
        manifolds: &[SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        Self::solve_position_points(&self.points, bodies, manifolds, config)
    }

    fn solve_position_points<B: SolverBodyAccess + ?Sized>(
        points: &[SolverPoint],
        bodies: &mut B,
        manifolds: &[SolverManifold],
        config: CollisionConfig,
    ) -> Result<(), CollisionError> {
        for point in points.iter().copied() {
            let contact = contact_at(manifolds, point)?;
            let (body_a, body_b) = bodies_at(bodies, point.body_a, point.body_b)?;
            let position_a = body_a
                .body
                .state()
                .transform()
                .transform_point(contact.local_a);
            let position_b = body_b
                .body
                .state()
                .transform()
                .transform_point(contact.local_b);
            let separation = position_b
                .checked_sub(position_a)
                .and_then(|delta| delta.checked_dot(point.normal))
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let correction =
                (-separation - config.penetration_slop).max(0.0) * POSITION_CORRECTION_FRACTION;
            if correction == 0.0 {
                continue;
            }
            let denominator = effective_mass(
                &body_a.body,
                &body_b.body,
                position_a,
                position_b,
                point.normal,
            )?;
            if denominator <= f64::MIN_POSITIVE {
                return Err(CollisionError::InvalidContact);
            }
            let impulse = point.normal * (correction / denominator);
            apply_pair_position_impulse(
                bodies,
                point.body_a,
                point.body_b,
                position_a,
                position_b,
                impulse,
            )?;
        }
        Ok(())
    }
}

#[cfg(feature = "bench-internals")]
fn parallel_axis(offset: Vec3, mass: f64) -> Mat3 {
    let squared = offset.length_squared() * mass;
    Mat3::from_diagonal(Vec3::splat(squared))
        - Mat3::from_rows([
            [
                offset.x * offset.x * mass,
                offset.x * offset.y * mass,
                offset.x * offset.z * mass,
            ],
            [
                offset.y * offset.x * mass,
                offset.y * offset.y * mass,
                offset.y * offset.z * mass,
            ],
            [
                offset.z * offset.x * mass,
                offset.z * offset.y * mass,
                offset.z * offset.z * mass,
            ],
        ])
}

fn solve_tangent_impulse(
    body_a: &RigidBody,
    body_b: &RigidBody,
    point: SolverPoint,
    contact: ContactPoint,
    material: SolverMaterial,
    velocity: Vec3,
) -> Result<([f64; 2], Vec3), CollisionError> {
    let k11 = effective_cross_mass(
        body_a,
        body_b,
        point.position_a,
        point.position_b,
        point.tangent_a,
        point.tangent_a,
    )?;
    let k12 = effective_cross_mass(
        body_a,
        body_b,
        point.position_a,
        point.position_b,
        point.tangent_a,
        point.tangent_b,
    )?;
    let k22 = effective_cross_mass(
        body_a,
        body_b,
        point.position_a,
        point.position_b,
        point.tangent_b,
        point.tangent_b,
    )?;
    // Check positive definiteness relative to the matrix scale before
    // inverting. A positive but nearly singular matrix amplifies velocity
    // error into an arbitrarily large friction impulse.
    let scale = k11.abs().max(k12.abs()).max(k22.abs());
    if !scale.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if scale == 0.0 {
        return Ok((contact.tangent_impulse, Vec3::ZERO));
    }
    let a = k11 / scale;
    let b = k12 / scale;
    let c = k22 / scale;
    let normalized_determinant = a * c - b * b;
    if a <= 0.0 || c <= 0.0 || normalized_determinant <= 64.0 * f64::EPSILON {
        return Ok((contact.tangent_impulse, Vec3::ZERO));
    }
    let determinant = k11 * k22 - k12 * k12;
    if !determinant.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if determinant <= 0.0 {
        return Ok((contact.tangent_impulse, Vec3::ZERO));
    }
    let speed_a = velocity
        .checked_dot(point.tangent_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let speed_b = velocity
        .checked_dot(point.tangent_b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let delta_a = (-k22 * speed_a + k12 * speed_b) / determinant;
    let delta_b = (k12 * speed_a - k11 * speed_b) / determinant;
    let candidate = [
        contact.tangent_impulse[0] + delta_a,
        contact.tangent_impulse[1] + delta_b,
    ];
    if !candidate[0].is_finite() || !candidate[1].is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let candidate_length = candidate[0].hypot(candidate[1]);
    let static_limit = material.static_friction * contact.normal_impulse;
    if !candidate_length.is_finite() || !static_limit.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let (next_a, next_b) = if candidate_length <= static_limit {
        (candidate[0], candidate[1])
    } else if candidate_length > 0.0 {
        let dynamic_limit = material.dynamic_friction * contact.normal_impulse;
        if !dynamic_limit.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let scale = dynamic_limit.min(candidate_length) / candidate_length;
        (candidate[0] * scale, candidate[1] * scale)
    } else {
        (0.0, 0.0)
    };
    let change_a = next_a - contact.tangent_impulse[0];
    let change_b = next_b - contact.tangent_impulse[1];
    let impulse = point.tangent_a * change_a + point.tangent_b * change_b;
    if !impulse.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(([next_a, next_b], impulse))
}

fn effective_mass(
    body_a: &RigidBody,
    body_b: &RigidBody,
    position_a: Vec3,
    position_b: Vec3,
    direction: Vec3,
) -> Result<f64, CollisionError> {
    effective_cross_mass(body_a, body_b, position_a, position_b, direction, direction)
}

fn effective_cross_mass(
    body_a: &RigidBody,
    body_b: &RigidBody,
    position_a: Vec3,
    position_b: Vec3,
    measurement: Vec3,
    impulse: Vec3,
) -> Result<f64, CollisionError> {
    constraint::effective_cross_mass_at(
        body_a,
        body_b,
        position_a,
        position_b,
        measurement,
        impulse,
    )
    .map_err(map_constraint_error)
}

fn relative_velocity(
    body_a: &RigidBody,
    body_b: &RigidBody,
    position_a: Vec3,
    position_b: Vec3,
) -> Result<Vec3, CollisionError> {
    constraint::relative_velocity_at(body_a, body_b, position_a, position_b)
        .map_err(map_constraint_error)
}

fn apply_pair_impulse<B: SolverBodyAccess + ?Sized>(
    bodies: &mut B,
    body_a: usize,
    body_b: usize,
    point_a: Vec3,
    point_b: Vec3,
    impulse: Vec3,
) -> Result<(), CollisionError> {
    let dynamic_a = bodies
        .get(body_a)
        .ok_or(CollisionError::InvalidContact)?
        .body()
        .physics()
        .body_type()
        == crate::rigid_body::BodyType::Dynamic;
    let dynamic_b = bodies
        .get(body_b)
        .ok_or(CollisionError::InvalidContact)?
        .body()
        .physics()
        .body_type()
        == crate::rigid_body::BodyType::Dynamic;
    if dynamic_a {
        let body = bodies
            .get_dynamic_mut(body_a)
            .ok_or(CollisionError::InvalidContact)?;
        constraint::apply_point_impulse(&mut body.body, point_a, -impulse)
            .map_err(map_constraint_error)?;
    }
    if dynamic_b {
        let body = bodies
            .get_dynamic_mut(body_b)
            .ok_or(CollisionError::InvalidContact)?;
        constraint::apply_point_impulse(&mut body.body, point_b, impulse)
            .map_err(map_constraint_error)?;
    }
    Ok(())
}

fn apply_pair_position_impulse<B: SolverBodyAccess + ?Sized>(
    bodies: &mut B,
    body_a: usize,
    body_b: usize,
    point_a: Vec3,
    point_b: Vec3,
    impulse: Vec3,
) -> Result<(), CollisionError> {
    let dynamic_a = bodies
        .get(body_a)
        .ok_or(CollisionError::InvalidContact)?
        .body()
        .physics()
        .body_type()
        == crate::rigid_body::BodyType::Dynamic;
    let dynamic_b = bodies
        .get(body_b)
        .ok_or(CollisionError::InvalidContact)?
        .body()
        .physics()
        .body_type()
        == crate::rigid_body::BodyType::Dynamic;
    if dynamic_a {
        let body = bodies
            .get_dynamic_mut(body_a)
            .ok_or(CollisionError::InvalidContact)?;
        constraint::apply_point_position_impulse(&mut body.body, point_a, -impulse)
            .map_err(map_constraint_error)?;
    }
    if dynamic_b {
        let body = bodies
            .get_dynamic_mut(body_b)
            .ok_or(CollisionError::InvalidContact)?;
        constraint::apply_point_position_impulse(&mut body.body, point_b, impulse)
            .map_err(map_constraint_error)?;
    }
    Ok(())
}

fn bodies_at<B: SolverBodyAccess + ?Sized>(
    bodies: &B,
    body_a: usize,
    body_b: usize,
) -> Result<(&SolverBody, &SolverBody), CollisionError> {
    let a = bodies.get(body_a).ok_or(CollisionError::InvalidContact)?;
    let b = bodies.get(body_b).ok_or(CollisionError::InvalidContact)?;
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    Ok((a, b))
}

fn contact_at(
    manifolds: &[SolverManifold],
    point: SolverPoint,
) -> Result<&ContactPoint, CollisionError> {
    manifolds
        .get(point.manifold)
        .and_then(|manifold| manifold.manifold.point(point.point))
        .ok_or(CollisionError::InvalidContact)
}

fn contact_mut(
    manifolds: &mut [SolverManifold],
    point: SolverPoint,
) -> Result<&mut ContactPoint, CollisionError> {
    manifolds
        .get_mut(point.manifold)
        .and_then(|manifold| manifold.manifold.point_mut(point.point))
        .ok_or(CollisionError::InvalidContact)
}

pub(crate) fn tangent_basis(normal: Vec3) -> Result<(Vec3, Vec3), CollisionError> {
    constraint::tangent_basis(normal).ok_or(CollisionError::InvalidContact)
}

fn validate_material(material: SolverMaterial) -> Result<(), CollisionError> {
    if !material.restitution.is_finite()
        || !(0.0..=1.0).contains(&material.restitution)
        || !material.static_friction.is_finite()
        || material.static_friction < 0.0
        || !material.dynamic_friction.is_finite()
        || material.dynamic_friction < 0.0
    {
        return Err(CollisionError::InvalidMaterial);
    }
    Ok(())
}

fn validate_contact(contact: ContactPoint) -> Result<(), CollisionError> {
    if !contact.local_a.is_finite()
        || !contact.local_b.is_finite()
        || !contact.normal.is_finite()
        || !contact.penetration.is_finite()
        || contact.penetration < 0.0
        || !contact.normal_impulse.is_finite()
        || contact.normal_impulse < 0.0
        || contact
            .tangent_impulse
            .iter()
            .any(|impulse| !impulse.is_finite())
    {
        return Err(CollisionError::InvalidContact);
    }
    Ok(())
}

fn map_constraint_error(error: crate::constraint::ConstraintError) -> CollisionError {
    match error {
        crate::constraint::ConstraintError::InvalidMass => CollisionError::InvalidContact,
        crate::constraint::ConstraintError::ArithmeticOverflow => {
            CollisionError::ArithmeticOverflow
        }
        crate::constraint::ConstraintError::NonConvergent => CollisionError::NonConvergent,
    }
}

#[cfg(test)]
#[path = "solver_tests.rs"]
mod tests;
