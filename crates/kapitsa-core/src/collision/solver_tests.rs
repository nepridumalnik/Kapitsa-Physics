use crate::execution::{ExecutionError, StepExecutor};
use crate::{
    collider::Collider,
    collision::{
        CollisionPair,
        cache::ContactCache,
        manifold::{ContactManifold, ContactPoint},
        solver::{SolverBody, SolverManifold, SolverMaterial, SolverWorkspace},
    },
    joint::{
        FixedJoint, IterationRange, Joint, JointCommon, JointConstraint, JointFrame,
        JointIterations, JointSolverCall, JointSolverOptions, JointSolverStep, JointTolerance,
        JointWorkspace,
    },
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Vec3};
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Default)]
struct CountingExecutor {
    calls: AtomicUsize,
    chunks: AtomicUsize,
}

impl CountingExecutor {
    fn calls(&self) -> usize {
        self.calls.load(Ordering::Relaxed)
    }

    fn chunks(&self) -> usize {
        self.chunks.load(Ordering::Relaxed)
    }
}

impl StepExecutor for CountingExecutor {
    fn for_each_chunk_mut<T, F>(
        &self,
        values: &mut [T],
        chunk_size: usize,
        operation: F,
    ) -> Result<(), ExecutionError>
    where
        T: Send,
        F: Fn(usize, &mut [T]) + Send + Sync,
    {
        self.calls.fetch_add(1, Ordering::Relaxed);
        if chunk_size == 0 {
            return Err(ExecutionError::InvalidChunkLayout);
        }
        for (index, chunk) in values.chunks_mut(chunk_size).enumerate() {
            self.chunks.fetch_add(1, Ordering::Relaxed);
            operation(index, chunk);
        }
        Ok(())
    }

    fn for_each_chunk_layout_mut<T, F>(
        &self,
        values: &mut [T],
        chunk_sizes: &[usize],
        operation: F,
    ) -> Result<(), ExecutionError>
    where
        T: Send,
        F: Fn(usize, &mut [T]) + Send + Sync,
    {
        self.calls.fetch_add(1, Ordering::Relaxed);
        let mut remaining = values;
        let mut start = 0;
        for &chunk_size in chunk_sizes {
            if chunk_size == 0 || chunk_size > remaining.len() {
                return Err(ExecutionError::InvalidChunkLayout);
            }
            let (chunk, tail) = remaining.split_at_mut(chunk_size);
            self.chunks.fetch_add(1, Ordering::Relaxed);
            operation(start, chunk);
            start += chunk_size;
            remaining = tail;
        }
        if !remaining.is_empty() {
            return Err(ExecutionError::InvalidChunkLayout);
        }
        Ok(())
    }
}

fn pair() -> CollisionPair {
    let mut world = PhysicsWorld::new();
    let body_a = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let body_b = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let collider_a = world
        .add_collider(
            body_a,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let collider_b = world
        .add_collider(
            body_b,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    CollisionPair::new(collider_a, collider_b).unwrap()
}

fn dynamic_body(translation: Vec3) -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    RigidBody::new_dynamic(
        Transform::new(translation, kapitsa_math::Quat::IDENTITY).unwrap(),
        mass,
    )
    .unwrap()
}

fn ball_on_plane(linear_velocity: Vec3) -> (Vec<SolverBody>, Vec<SolverManifold>) {
    let ground = RigidBody::new_static(Transform::identity());
    let mut ball = dynamic_body(Vec3::new(0.0, 0.4, 0.0));
    ball.set_linear_velocity(linear_velocity).unwrap();
    let ground_state = *ground.state();
    let ball_state = *ball.state();
    let point_a = Vec3::ZERO;
    let point_b = Vec3::new(0.0, -0.1, 0.0);
    let pair = pair();
    let contact = ContactPoint::from_world(
        &Transform::identity(),
        &ball.state().transform(),
        point_a,
        point_b,
        Vec3::Y,
        0.1,
        1,
    )
    .unwrap();
    let mut manifold = ContactManifold::new(pair);
    manifold
        .insert(contact, point_a * 0.5 + point_b * 0.5)
        .unwrap();
    (
        vec![
            SolverBody::from_prediction(ground, ground_state),
            SolverBody::from_prediction(ball, ball_state),
        ],
        vec![SolverManifold {
            body_a: 0,
            body_b: 1,
            manifold,
            material: SolverMaterial {
                restitution: 0.5,
                static_friction: 0.0,
                dynamic_friction: 0.0,
            },
        }],
    )
}

#[test]
fn normal_impulses_apply_restitution_and_separate_position_correction() {
    let (mut bodies, mut manifolds) = ball_on_plane(-Vec3::Y * 2.0);
    let ground_transform = bodies[0].body().state().transform();
    let mut solver = SolverWorkspace::default();
    solver
        .solve(
            &mut bodies,
            &mut manifolds,
            super::CollisionConfig::default(),
        )
        .unwrap();

    let ball = bodies[1].body();
    assert!((ball.state().linear_velocity().y - 1.0).abs() < 1.0e-10);
    assert!(ball.state().transform().translation().y > 0.4);
    assert_eq!(bodies[0].body().state().transform(), ground_transform);
    assert!(
        manifolds[0]
            .manifold
            .points()
            .first()
            .unwrap()
            .normal_impulse
            > 0.0
    );
}

#[test]
fn friction_limits_tangent_impulses_and_creates_angular_response() {
    let (mut bodies, mut manifolds) = ball_on_plane(Vec3::new(10.0, -1.0, 0.0));
    manifolds[0].material.static_friction = 0.4;
    manifolds[0].material.dynamic_friction = 0.3;
    let mut solver = SolverWorkspace::default();
    solver
        .solve(
            &mut bodies,
            &mut manifolds,
            super::CollisionConfig::default(),
        )
        .unwrap();

    let ball = bodies[1].body();
    assert!(ball.state().linear_velocity().x < 10.0);
    assert!(ball.state().angular_velocity().z.abs() > 0.0);
    let contact = manifolds[0].manifold.points().first().unwrap();
    let friction_limit = 0.3 * contact.normal_impulse;
    assert!(
        contact.tangent_impulse[0].hypot(contact.tangent_impulse[1]) <= friction_limit + 1.0e-12
    );
}

#[test]
fn rotated_contact_warm_start_does_not_increase_energy_vs_cold_start() {
    fn solve(warm_start: bool) -> (f64, f64) {
        let pair = pair();
        let mut previous = ContactManifold::new(pair);
        let mut cached_contact = ContactPoint::from_world(
            &Transform::identity(),
            &Transform::identity(),
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::Y,
            0.0,
            5,
        )
        .unwrap();
        cached_contact.normal_impulse = 0.1;
        cached_contact.tangent_impulse = [0.05, 0.0];
        previous.insert(cached_contact, Vec3::ZERO).unwrap();
        let mut cache = ContactCache::new();
        cache.stage(&[previous], 1.0).unwrap();
        cache.commit();

        let normal = kapitsa_math::Quat::from_axis_angle(Vec3::Z, 0.1)
            .unwrap()
            .rotate_vector(Vec3::Y)
            .unwrap();
        let body_transform = Transform::new(normal * 0.45, kapitsa_math::Quat::IDENTITY).unwrap();
        let mut ball = dynamic_body(body_transform.translation());
        ball.set_linear_velocity(Vec3::Z - normal * 0.1).unwrap();
        let point_a = Vec3::ZERO;
        let point_b = body_transform.translation() - normal * 0.5;
        let contact = ContactPoint::from_world(
            &Transform::identity(),
            &body_transform,
            point_a,
            point_b,
            normal,
            0.05,
            5,
        )
        .unwrap();
        let mut manifold = ContactManifold::new(pair);
        manifold.insert(contact, point_b).unwrap();
        let ground = RigidBody::new_static(Transform::identity());
        let ball_state = *ball.state();
        let mut bodies = vec![
            SolverBody::from_prediction(ground, *ground.state()),
            SolverBody::from_prediction(ball, ball_state),
        ];
        let mut manifolds = vec![SolverManifold {
            body_a: 0,
            body_b: 1,
            manifold,
            material: SolverMaterial {
                restitution: 0.0,
                static_friction: 0.6,
                dynamic_friction: 0.6,
            },
        }];
        if warm_start {
            cache.warm_start_solver(&mut manifolds, 1.0, 0.01).unwrap();
        }
        let initial_energy = kinetic_energy(bodies[1].body());
        SolverWorkspace::default()
            .solve(
                &mut bodies,
                &mut manifolds,
                super::CollisionConfig::default(),
            )
            .unwrap();
        (initial_energy, kinetic_energy(bodies[1].body()))
    }

    let (warm_initial, warm_final) = solve(true);
    let (cold_initial, cold_final) = solve(false);
    assert!((warm_initial - cold_initial).abs() < 1.0e-12);
    assert!(warm_final <= warm_initial + 1.0e-8);
    assert!(cold_final <= cold_initial + 1.0e-8);
    assert!(warm_final <= cold_final + 1.0e-5);
}

#[test]
fn solve_with_joints_reuses_scalar_velocity_rows_across_sweeps_in_phase_order() {
    let mut id_world = PhysicsWorld::new();
    let id_a = id_world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let id_b = id_world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let joint_id = id_world
        .add_joint(
            id_a,
            id_b,
            Joint::Fixed(FixedJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
        )
        .unwrap();
    let joint = JointConstraint {
        id: joint_id,
        body_a: 0,
        body_b: 1,
        joint: Joint::Fixed(FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    };
    let bodies = [
        dynamic_body(Vec3::ZERO),
        dynamic_body(Vec3::new(1.5, 0.0, 0.0)),
    ];
    let mut solver_bodies = bodies
        .into_iter()
        .map(|body| {
            let state = *body.state();
            SolverBody::from_prediction(body, state)
        })
        .collect::<Vec<_>>();
    let mut manifolds = Vec::new();
    let mut solver = SolverWorkspace::new();
    let mut joint_workspace = JointWorkspace::new();
    let config = super::CollisionConfig {
        velocity_iterations: 2,
        position_iterations: 1,
        ..super::CollisionConfig::default()
    };

    solver
        .solve_with_joints(
            &mut solver_bodies,
            &mut manifolds,
            &mut joint_workspace,
            &[joint],
            config,
            JointSolverStep::new(1.0 / 60.0, JointSolverOptions::legacy()),
            None,
            &crate::execution::SequentialExecutor,
            1,
        )
        .unwrap();

    assert_eq!(
        joint_workspace.solver_calls(),
        [
            JointSolverCall::VelocitySweep {
                preparing_rows: false,
                row_count: 6,
            },
            JointSolverCall::VelocitySweep {
                preparing_rows: false,
                row_count: 6,
            },
            JointSolverCall::JointRowsPrepared(0),
            JointSolverCall::PositionSweep {
                preparing_rows: true,
                row_count: 3,
            },
            JointSolverCall::PositionSweep {
                preparing_rows: true,
                row_count: 3,
            },
            JointSolverCall::JointRowsCommitted(0),
        ]
    );
}

fn fixed_joint_constraint(body_a: usize, body_b: usize) -> JointConstraint {
    let mut world = PhysicsWorld::new();
    let id_a = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let id_b = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let joint_id = world
        .add_joint(
            id_a,
            id_b,
            Joint::Fixed(FixedJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
        )
        .unwrap();
    JointConstraint {
        id: joint_id,
        body_a,
        body_b,
        joint: Joint::Fixed(FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    }
}

fn prepare_colored_workspace(
    workspace: &mut SolverWorkspace,
    bodies: &[SolverBody],
    manifolds: &[SolverManifold],
    joints: &[JointConstraint],
) {
    let rebuilt = workspace
        .prepare_constraint_coloring(bodies, manifolds, joints)
        .unwrap();
    workspace
        .prepare_colored_batches(manifolds, joints, rebuilt)
        .unwrap();
}

fn solve_colored(
    bodies: &mut Vec<SolverBody>,
    manifolds: &mut Vec<SolverManifold>,
    joints: &[JointConstraint],
    config: super::CollisionConfig,
) -> Result<(), crate::collision::CollisionError> {
    let mut workspace = SolverWorkspace::new();
    prepare_colored_workspace(&mut workspace, bodies, manifolds, joints);
    let mut joint_workspace = JointWorkspace::new();
    joint_workspace.begin_step();
    workspace.solve_colored_constraints(
        bodies,
        manifolds,
        &mut joint_workspace,
        joints,
        config,
        JointSolverStep::new(1.0 / 60.0, JointSolverOptions::legacy()),
        None,
        &crate::execution::SequentialExecutor,
        8,
    )
}

#[test]
fn contact_only_phases_do_not_touch_joints() {
    let bodies = [
        dynamic_body(Vec3::ZERO),
        dynamic_body(Vec3::new(1.5, 0.0, 0.0)),
    ];
    let mut solver_bodies = bodies
        .into_iter()
        .map(|body| {
            let state = *body.state();
            SolverBody::from_prediction(body, state)
        })
        .collect::<Vec<_>>();
    let manifolds = Vec::new();
    let joints = [fixed_joint_constraint(0, 1)];
    let mut workspace = SolverWorkspace::new();
    prepare_colored_workspace(&mut workspace, &solver_bodies, &manifolds, &joints);

    let before = solver_bodies
        .iter()
        .map(|body| {
            (
                body.body().state().transform(),
                body.body().state().linear_velocity(),
                body.body().state().angular_velocity(),
            )
        })
        .collect::<Vec<_>>();
    let step = JointSolverStep::new(1.0 / 60.0, JointSolverOptions::legacy());
    let executor = CountingExecutor::default();
    for phase in [
        super::ColoredConstraintPhase::PrepareContactPoints,
        super::ColoredConstraintPhase::WarmStartContacts,
    ] {
        workspace
            .execute_colored_phase(
                &mut solver_bodies,
                &mut [],
                super::CollisionConfig::default(),
                step,
                phase,
                &executor,
                8,
            )
            .unwrap();
    }
    assert_eq!(executor.calls(), 0);

    for (body, (transform, linear, angular)) in solver_bodies.iter().zip(before) {
        assert_eq!(body.body().state().transform(), transform);
        assert_eq!(body.body().state().linear_velocity(), linear);
        assert_eq!(body.body().state().angular_velocity(), angular);
    }
    assert!(
        workspace
            .colored_batches
            .iter()
            .all(|batch| batch.joint_workspace.solver_calls().is_empty())
    );
}

#[test]
fn contact_only_solution_matches_sequential_solver() {
    let config = super::CollisionConfig {
        velocity_iterations: 3,
        position_iterations: 2,
        ..super::CollisionConfig::default()
    };
    let (mut colored_bodies, mut colored_manifolds) = ball_on_plane(-Vec3::Y * 2.0);
    let (mut sequential_bodies, mut sequential_manifolds) = ball_on_plane(-Vec3::Y * 2.0);

    solve_colored(&mut colored_bodies, &mut colored_manifolds, &[], config).unwrap();
    SolverWorkspace::new()
        .solve(&mut sequential_bodies, &mut sequential_manifolds, config)
        .unwrap();

    let colored_velocity = colored_bodies[1].body().state().linear_velocity();
    let sequential_velocity = sequential_bodies[1].body().state().linear_velocity();
    assert!((colored_velocity - sequential_velocity).length() < 1.0e-12);
    assert!(
        (colored_bodies[1].body().state().transform().translation()
            - sequential_bodies[1]
                .body()
                .state()
                .transform()
                .translation())
        .length()
            < 1.0e-12
    );
    assert_eq!(
        colored_manifolds[0].manifold.points()[0].normal_impulse,
        sequential_manifolds[0].manifold.points()[0].normal_impulse
    );
}

fn manifold_impulses(manifolds: &[SolverManifold]) -> Vec<(f64, [f64; 2])> {
    manifolds
        .iter()
        .map(|manifold| {
            let point = &manifold.manifold.points()[0];
            (point.normal_impulse, point.tangent_impulse)
        })
        .collect()
}

#[test]
fn only_velocity_phase_writes_contact_impulses_back() {
    let (mut bodies, mut manifolds) = ball_on_plane(-Vec3::Y * 2.0);
    {
        let point = manifolds[0].manifold.point_mut(0).unwrap();
        point.normal_impulse = 0.25;
        point.tangent_impulse = [0.05, 0.02];
    }
    let mut workspace = SolverWorkspace::new();
    prepare_colored_workspace(&mut workspace, &bodies, &manifolds, &[]);
    let step = JointSolverStep::new(1.0 / 60.0, JointSolverOptions::legacy());
    let config = super::CollisionConfig::default();
    let initial = manifold_impulses(&manifolds);

    for phase in [
        super::ColoredConstraintPhase::PrepareContactPoints,
        super::ColoredConstraintPhase::WarmStartContacts,
    ] {
        workspace
            .execute_colored_phase(
                &mut bodies,
                &mut manifolds,
                config,
                step,
                phase,
                &crate::execution::SequentialExecutor,
                8,
            )
            .unwrap();
        assert_eq!(manifold_impulses(&manifolds), initial);
    }

    workspace
        .execute_colored_phase(
            &mut bodies,
            &mut manifolds,
            config,
            step,
            super::ColoredConstraintPhase::Velocity {
                first_iteration: true,
            },
            &crate::execution::SequentialExecutor,
            8,
        )
        .unwrap();
    let after_velocity = manifold_impulses(&manifolds);
    assert_ne!(after_velocity, initial);

    workspace
        .execute_colored_phase(
            &mut bodies,
            &mut manifolds,
            config,
            step,
            super::ColoredConstraintPhase::Position,
            &crate::execution::SequentialExecutor,
            8,
        )
        .unwrap();
    assert_eq!(manifold_impulses(&manifolds), after_velocity);
}

#[test]
fn contact_impulses_warm_start_the_next_solver_step() {
    let (mut bodies, mut manifolds) = ball_on_plane(-Vec3::Y * 2.0);
    solve_colored(
        &mut bodies,
        &mut manifolds,
        &[],
        super::CollisionConfig::default(),
    )
    .unwrap();
    let cached = manifold_impulses(&manifolds);
    assert!(cached[0].0 > 0.0);

    let mut next_workspace = SolverWorkspace::new();
    prepare_colored_workspace(&mut next_workspace, &bodies, &manifolds, &[]);
    let step = JointSolverStep::new(1.0 / 60.0, JointSolverOptions::legacy());
    let config = super::CollisionConfig::default();
    next_workspace
        .execute_colored_phase(
            &mut bodies,
            &mut manifolds,
            config,
            step,
            super::ColoredConstraintPhase::PrepareContactPoints,
            &crate::execution::SequentialExecutor,
            8,
        )
        .unwrap();
    let velocity_before_warm_start = bodies[1].body().state().linear_velocity();
    next_workspace
        .execute_colored_phase(
            &mut bodies,
            &mut manifolds,
            config,
            step,
            super::ColoredConstraintPhase::WarmStartContacts,
            &crate::execution::SequentialExecutor,
            8,
        )
        .unwrap();
    assert!(
        (bodies[1].body().state().linear_velocity() - velocity_before_warm_start).length()
            > 1.0e-12
    );
}

fn workload(constraints_per_color: &[usize], worker_count: usize) -> super::ColoredWorkload {
    let mut workload = super::ColoredWorkload {
        color_count: constraints_per_color.len(),
        ..super::ColoredWorkload::default()
    };
    for &constraint_count in constraints_per_color {
        let batch_count = constraint_count.div_ceil(super::LARGE_ISLAND_BATCH_SIZE);
        workload.constraint_count = workload.constraint_count.saturating_add(constraint_count);
        workload.batch_count = workload.batch_count.saturating_add(batch_count);
        workload.max_color_batch_count = workload.max_color_batch_count.max(batch_count);
        workload.useful_batch_count = workload
            .useful_batch_count
            .saturating_add(batch_count.min(worker_count));
    }
    workload
}

fn static_contact_fixture(count: usize) -> (Vec<SolverBody>, Vec<SolverManifold>) {
    let bodies = (0..count * 2)
        .map(|_| {
            let body = RigidBody::new_static(Transform::identity());
            let state = *body.state();
            SolverBody::from_prediction(body, state)
        })
        .collect::<Vec<_>>();
    let contact = ContactPoint::from_world(
        &Transform::identity(),
        &Transform::identity(),
        Vec3::ZERO,
        Vec3::new(0.0, -0.1, 0.0),
        Vec3::Y,
        0.1,
        1,
    )
    .unwrap();
    let mut manifolds = Vec::with_capacity(count);
    for index in 0..count {
        let mut manifold = ContactManifold::new(pair());
        manifold.insert(contact, Vec3::ZERO).unwrap();
        manifolds.push(SolverManifold {
            body_a: index * 2,
            body_b: index * 2 + 1,
            manifold,
            material: SolverMaterial {
                restitution: 0.0,
                static_friction: 0.0,
                dynamic_friction: 0.0,
            },
        });
    }
    (bodies, manifolds)
}

#[test]
fn colored_dispatch_uses_executor_only_for_multi_batch_groups() {
    for (contact_count, expected_calls) in [(1, 0), (20, 0), (33, 1)] {
        let (mut bodies, mut manifolds) = static_contact_fixture(contact_count);
        let mut workspace = SolverWorkspace::new();
        prepare_colored_workspace(&mut workspace, &bodies, &manifolds, &[]);
        let executor = CountingExecutor::default();
        workspace
            .execute_colored_phase(
                &mut bodies,
                &mut manifolds,
                super::CollisionConfig::default(),
                JointSolverStep::new(1.0 / 60.0, JointSolverOptions::legacy()),
                super::ColoredConstraintPhase::PrepareContactPoints,
                &executor,
                8,
            )
            .unwrap();
        assert_eq!(
            executor.calls(),
            expected_calls,
            "unexpected dispatch for {contact_count} contacts"
        );
        if expected_calls > 0 {
            assert!(executor.chunks() > 1);
            #[cfg(feature = "bench-internals")]
            {
                let (_, groups, batches) = workspace.colored_dispatch_metrics();
                assert_eq!(groups, 1);
                assert_eq!(batches, 3);
            }
        }
    }
}

#[test]
fn workload_heuristic_rejects_small_or_narrow_schedules() {
    let small = workload(&[16, 16], 8);
    assert!(!small.can_use_parallel_schedule(8));

    let narrow = workload(&[32; 64], 8);
    assert!(!narrow.can_use_parallel_schedule(8));

    let wide = workload(&[64; 64], 8);
    assert!(wide.can_use_parallel_schedule(8));

    assert!(!wide.can_use_parallel_schedule(1));
}

#[test]
fn thousand_constraints_in_27_colors_cannot_meet_eight_worker_width() {
    // ceil(n/16) <= (n+15)/16 for each of 27 nonempty colors.
    let max_batches = (1000 + 27 * 15) / super::LARGE_ISLAND_BATCH_SIZE;
    assert!(max_batches < 27 * 4);
    for counts in [
        {
            let mut c = [37; 27];
            c[0] = 38;
            c
        },
        {
            let mut c = [1; 27];
            c[0] = 974;
            c
        },
    ] {
        let schedule = workload(&counts, 8);
        assert_eq!(schedule.constraint_count, 1000);
        assert!(schedule.batch_count <= max_batches);
        assert!(schedule.useful_batch_count < 27 * 4);
        assert!(!schedule.can_use_parallel_schedule(8));
    }
}

#[test]
fn workload_heuristic_thresholds_follow_available_workers() {
    let two_batches_per_color = workload(&[32; 64], 8);
    assert_eq!(two_batches_per_color.parallel_fraction_percent(), 50);
    assert!(!two_batches_per_color.can_use_parallel_schedule(8));
    assert!(two_batches_per_color.can_use_parallel_schedule(4));

    let three_batches_per_color = workload(&[48; 64], 8);
    assert!(!three_batches_per_color.can_use_parallel_schedule(8));
    assert!(three_batches_per_color.can_use_parallel_schedule(4));

    let four_batches_per_color = workload(&[64; 64], 8);
    assert!(four_batches_per_color.can_use_parallel_schedule(8));

    let single_batch_per_color = workload(&[8; 64], 2);
    assert!(!single_batch_per_color.can_use_parallel_schedule(2));
}

#[test]
fn colored_solver_choice_respects_compatibility_configuration() {
    let bodies = (0..256)
        .map(|_| {
            let body = RigidBody::new_static(Transform::identity());
            let state = *body.state();
            SolverBody::from_prediction(body, state)
        })
        .collect::<Vec<_>>();
    let empty_manifold = ContactManifold::new(pair());
    let manifolds = (0..128)
        .map(|index| SolverManifold {
            body_a: index * 2,
            body_b: index * 2 + 1,
            manifold: empty_manifold,
            material: SolverMaterial {
                restitution: 0.0,
                static_friction: 0.0,
                dynamic_friction: 0.0,
            },
        })
        .collect::<Vec<_>>();
    let mut solver = SolverWorkspace::new();
    let rebuilt = solver
        .prepare_constraint_coloring(&bodies, &manifolds, &[])
        .unwrap();
    assert!(rebuilt);

    let legacy = JointSolverOptions::legacy();
    let expected = cfg!(feature = "bench-internals");
    assert_eq!(
        solver.can_solve_colored_constraints(legacy, 8, manifolds.len()),
        expected
    );
    assert!(!solver.can_solve_colored_constraints(legacy, 1, manifolds.len()));
    assert!(!solver.can_solve_colored_constraints(legacy, 8, 64));

    let adaptive = legacy.with_iterations(JointIterations::Adaptive {
        position: IterationRange::new(1, 1),
        velocity: IterationRange::new(1, 1),
        tolerance: JointTolerance {
            linear_m: 1.0e-6,
            angular_rad: 1.0e-6,
            velocity_mps: 1.0e-6,
            angular_velocity_radps: 1.0e-6,
        },
    });
    assert!(!solver.can_solve_colored_constraints(adaptive, 8, manifolds.len()));

    #[cfg(feature = "bench-internals")]
    {
        assert!(!solver.can_solve_colored_constraints(
            legacy.with_alternating_velocity_sweeps(true),
            8,
            manifolds.len()
        ));
        assert!(!solver.can_solve_colored_constraints(
            legacy.with_global_velocity_pcg(2, false),
            8,
            manifolds.len()
        ));
        assert!(!solver.can_solve_colored_constraints(
            legacy.with_global_position_pcg(2, false),
            8,
            manifolds.len()
        ));
    }
}

#[test]
fn mixed_joint_and_contact_solve_keeps_all_constraints_active() {
    let (mut bodies, mut manifolds) = ball_on_plane(-Vec3::Y);
    for position in [Vec3::ZERO, Vec3::new(1.5, 0.0, 0.0)] {
        let body = dynamic_body(position);
        let state = *body.state();
        bodies.push(SolverBody::from_prediction(body, state));
    }
    let joints = [fixed_joint_constraint(2, 3)];
    let initial_joint_distance = (bodies[3].body().state().transform().translation()
        - bodies[2].body().state().transform().translation())
    .length();

    solve_colored(
        &mut bodies,
        &mut manifolds,
        &joints,
        super::CollisionConfig::default(),
    )
    .unwrap();

    let ball = bodies[1].body();
    assert!(ball.state().linear_velocity().is_finite());
    assert!(ball.state().linear_velocity().y > 0.0);
    assert!(manifolds[0].manifold.points()[0].normal_impulse > 0.0);
    let joint_distance = (bodies[3].body().state().transform().translation()
        - bodies[2].body().state().transform().translation())
    .length();
    assert!(joint_distance.is_finite());
    assert!(joint_distance < initial_joint_distance);
    assert!(bodies.iter().all(|body| {
        body.body().state().transform().translation().is_finite()
            && body.body().state().linear_velocity().is_finite()
    }));
}

#[test]
fn contacts_sharing_one_dynamic_body_are_solved_without_conflicts() {
    let (mut bodies, mut manifolds) = ball_on_plane(-Vec3::Y * 2.0);
    let mut second = manifolds[0];
    second.manifold.point_mut(0).unwrap().normal_impulse = 0.0;
    manifolds.push(second);
    let config = super::CollisionConfig {
        velocity_iterations: 2,
        position_iterations: 1,
        ..super::CollisionConfig::default()
    };

    solve_colored(&mut bodies, &mut manifolds, &[], config).unwrap();

    let ball = bodies[1].body();
    assert!(ball.state().linear_velocity().is_finite());
    assert!(ball.state().transform().translation().is_finite());
    assert!(
        manifolds
            .iter()
            .all(|manifold| manifold.manifold.points()[0].normal_impulse >= 0.0)
    );
    assert!(
        manifolds
            .iter()
            .any(|manifold| manifold.manifold.points()[0].normal_impulse > 0.0)
    );
}

#[test]
fn adaptive_joint_iterations_stop_at_minimum_or_report_the_max_residual() {
    fn solve(offset: Vec3) -> (usize, usize, crate::joint::JointResidual) {
        let mut id_world = PhysicsWorld::new();
        let id_a = id_world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let id_b = id_world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let joint_id = id_world
            .add_joint(
                id_a,
                id_b,
                Joint::Fixed(FixedJoint::new(JointCommon::new(
                    JointFrame::IDENTITY,
                    JointFrame::IDENTITY,
                ))),
            )
            .unwrap();
        let joint = JointConstraint {
            id: joint_id,
            body_a: 0,
            body_b: 1,
            joint: Joint::Fixed(FixedJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
        };
        let body_a = RigidBody::new_static(Transform::identity());
        let body_b =
            RigidBody::new_static(Transform::new(offset, kapitsa_math::Quat::IDENTITY).unwrap());
        let state_a = *body_a.state();
        let state_b = *body_b.state();
        let mut bodies = vec![
            SolverBody::from_prediction(body_a, state_a),
            SolverBody::from_prediction(body_b, state_b),
        ];
        let tolerance = JointTolerance {
            linear_m: 1.0e-6,
            angular_rad: 1.0e-6,
            velocity_mps: 1.0e-5,
            angular_velocity_radps: 1.0e-5,
        };
        let options = JointSolverOptions::legacy().with_iterations(JointIterations::Adaptive {
            position: IterationRange::new(2, 4),
            velocity: IterationRange::new(2, 5),
            tolerance,
        });
        let config = super::CollisionConfig {
            velocity_iterations: 1,
            position_iterations: 1,
            ..super::CollisionConfig::default()
        };
        let mut collision_solver = SolverWorkspace::new();
        let mut joint_workspace = JointWorkspace::new();
        collision_solver
            .solve_with_joints(
                &mut bodies,
                &mut [],
                &mut joint_workspace,
                &[joint],
                config,
                JointSolverStep::new(1.0 / 60.0, options),
                None,
                &crate::execution::SequentialExecutor,
                1,
            )
            .unwrap();
        let velocity_sweeps = joint_workspace
            .solver_calls()
            .iter()
            .filter(|call| matches!(call, JointSolverCall::VelocitySweep { .. }))
            .count();
        let position_sweeps = joint_workspace
            .solver_calls()
            .iter()
            .filter(|call| matches!(call, JointSolverCall::PositionSweep { .. }))
            .count();
        (position_sweeps, velocity_sweeps, joint_workspace.residual())
    }

    let tolerance = JointTolerance {
        linear_m: 1.0e-6,
        angular_rad: 1.0e-6,
        velocity_mps: 1.0e-5,
        angular_velocity_radps: 1.0e-5,
    };
    let satisfied = solve(Vec3::ZERO);
    assert_eq!((satisfied.0, satisfied.1), (2, 2));
    assert!(satisfied.2.position_converged(tolerance));
    assert!(satisfied.2.velocity_converged(tolerance));

    let unsatisfiable = solve(Vec3::X);
    assert_eq!((unsatisfiable.0, unsatisfiable.1), (4, 2));
    assert!(!unsatisfiable.2.position_converged(tolerance));
    assert!(unsatisfiable.2.max_linear_m > tolerance.linear_m);
    assert_eq!(solve(Vec3::X), unsatisfiable);
}

#[test]
fn adaptive_joint_policy_leaves_contact_iteration_budget_unchanged() {
    let config = super::CollisionConfig {
        velocity_iterations: 4,
        position_iterations: 3,
        ..super::CollisionConfig::default()
    };
    let (mut baseline_bodies, mut baseline_manifolds) = ball_on_plane(Vec3::new(2.0, -1.0, 0.5));
    SolverWorkspace::new()
        .solve(&mut baseline_bodies, &mut baseline_manifolds, config)
        .unwrap();

    let (mut bodies, mut manifolds) = ball_on_plane(Vec3::new(2.0, -1.0, 0.5));
    for position in [Vec3::ZERO, Vec3::X * 2.0] {
        let body =
            RigidBody::new_static(Transform::new(position, kapitsa_math::Quat::IDENTITY).unwrap());
        let state = *body.state();
        bodies.push(SolverBody::from_prediction(body, state));
    }
    let mut id_world = PhysicsWorld::new();
    let id_a = id_world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let id_b = id_world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let joint_id = id_world
        .add_joint(
            id_a,
            id_b,
            Joint::Fixed(FixedJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
        )
        .unwrap();
    let joint = JointConstraint {
        id: joint_id,
        body_a: 2,
        body_b: 3,
        joint: Joint::Fixed(FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    };
    let tolerance = JointTolerance {
        linear_m: 1.0e-6,
        angular_rad: 1.0e-6,
        velocity_mps: 1.0e-6,
        angular_velocity_radps: 1.0e-6,
    };
    let options = JointSolverOptions::legacy().with_iterations(JointIterations::Adaptive {
        position: IterationRange::new(1, 1),
        velocity: IterationRange::new(1, 1),
        tolerance,
    });
    let mut collision_solver = SolverWorkspace::new();
    collision_solver
        .solve_with_joints(
            &mut bodies,
            &mut manifolds,
            &mut JointWorkspace::new(),
            &[joint],
            config,
            JointSolverStep::new(1.0 / 60.0, options),
            None,
            &crate::execution::SequentialExecutor,
            1,
        )
        .unwrap();

    assert_eq!(
        bodies[1].state().transform(),
        baseline_bodies[1].state().transform()
    );
    assert_eq!(
        bodies[1].state().linear_velocity(),
        baseline_bodies[1].state().linear_velocity()
    );
    assert_eq!(
        bodies[1].state().angular_velocity(),
        baseline_bodies[1].state().angular_velocity()
    );
    assert_eq!(
        manifolds[0].manifold.points()[0].normal_impulse,
        baseline_manifolds[0].manifold.points()[0].normal_impulse
    );
}

fn kinetic_energy(body: &RigidBody) -> f64 {
    let mass = body.physics().mass().unwrap();
    let state = body.state();
    let rotation = state.transform().rotation().to_mat3().unwrap();
    let inertia_world = rotation * mass.inertia() * rotation.transpose();
    0.5 * mass.mass() * state.linear_velocity().length_squared()
        + 0.5
            * state
                .angular_velocity()
                .dot(inertia_world * state.angular_velocity())
}

#[test]
fn invalid_contact_material_leaves_solver_bodies_unchanged() {
    let (mut bodies, mut manifolds) = ball_on_plane(-Vec3::Y);
    manifolds[0].material.static_friction = f64::NAN;
    let initial = bodies[1].body().state().linear_velocity();
    let mut solver = SolverWorkspace::default();
    assert_eq!(
        solver.solve(
            &mut bodies,
            &mut manifolds,
            super::CollisionConfig::default()
        ),
        Err(crate::collision::CollisionError::InvalidMaterial)
    );
    assert_eq!(bodies[1].body().state().linear_velocity(), initial);
}

#[test]
fn kinematic_velocity_contributes_without_kinematic_reaction() {
    let mut surface = RigidBody::new_kinematic(Transform::identity());
    surface.set_linear_velocity(Vec3::Y).unwrap();
    let ball = dynamic_body(Vec3::new(0.0, 0.4, 0.0));
    let surface_state = *surface.state();
    let ball_state = *ball.state();
    let point_a = Vec3::ZERO;
    let point_b = Vec3::new(0.0, -0.1, 0.0);
    let contact = ContactPoint::from_world(
        &surface_state.transform(),
        &ball_state.transform(),
        point_a,
        point_b,
        Vec3::Y,
        0.1,
        2,
    )
    .unwrap();
    let mut manifold = ContactManifold::new(pair());
    manifold.insert(contact, point_a).unwrap();
    let mut bodies = vec![
        SolverBody::from_prediction(surface, surface_state),
        SolverBody::from_prediction(ball, ball_state),
    ];
    let mut manifolds = vec![SolverManifold {
        body_a: 0,
        body_b: 1,
        manifold,
        material: SolverMaterial {
            restitution: 0.0,
            static_friction: 0.0,
            dynamic_friction: 0.0,
        },
    }];
    SolverWorkspace::default()
        .solve(
            &mut bodies,
            &mut manifolds,
            super::CollisionConfig::default(),
        )
        .unwrap();

    assert_eq!(bodies[0].body().state().linear_velocity(), Vec3::Y);
    assert_eq!(bodies[0].body().state().transform(), Transform::identity());
    assert!(bodies[1].body().state().linear_velocity().y > 0.0);
}
