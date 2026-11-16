use super::*;
use crate::joint::row::RowDimension;
use crate::joint::workspace::JointSolverCall;
use crate::{
    joint::{
        BallJoint, ConeTwistJoint, CylindricalJoint, D6Joint, HingeJoint, Joint, JointCommon,
        JointDrive, JointFrame, JointLimit, JointMotion, PlanarJoint, SliderJoint, SpringJoint,
    },
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
};
use kapitsa_math::{Quat, Vec3};

#[cfg(feature = "bench-internals")]
#[test]
fn joint_geometry_residual_uses_per_joint_linear_and_angular_norms() {
    let row = |dimension, error| ConstraintRow {
        key: 0,
        dimension,
        body_a: 0,
        body_b: 1,
        linear_a: Vec3::ZERO,
        angular_a: Vec3::ZERO,
        position_angular_a: Vec3::ZERO,
        linear_b: Vec3::ZERO,
        angular_b: Vec3::ZERO,
        error,
        target_velocity: 0.0,
        force_bias: 0.0,
        force_driven: false,
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
    };
    let rows = [
        row(RowDimension::Linear, 3.0),
        row(RowDimension::Linear, 4.0),
        row(RowDimension::Angular, 0.6),
        row(RowDimension::Angular, 0.8),
        row(RowDimension::Linear, 0.2),
        row(RowDimension::Angular, 0.3),
    ];
    let residual =
        measure_joint_geometry(&rows, &[0, 0, 0, 0, 1, 1]).expect("valid joint geometry residual");
    assert!((residual.anchor_error_m - 5.0).abs() < 1.0e-12);
    assert!((residual.frame_error_rad - 1.0).abs() < 1.0e-12);
}

fn solver_body(position: Vec3) -> SolverBody {
    solver_body_with_rotation(position, Quat::IDENTITY)
}

fn solver_body_with_rotation(position: Vec3, rotation: Quat) -> SolverBody {
    solver_body_with_velocities(position, rotation, Vec3::ZERO, Vec3::ZERO)
}

fn solver_body_with_velocities(
    position: Vec3,
    rotation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
) -> SolverBody {
    let mass =
        MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY).expect("valid mass");
    let mut body =
        RigidBody::new_dynamic(Transform::new(position, rotation).expect("transform"), mass)
            .expect("dynamic body");
    body.set_linear_velocity(linear_velocity)
        .expect("finite linear velocity");
    body.set_angular_velocity(angular_velocity)
        .expect("finite angular velocity");
    let state = *body.state();
    SolverBody::from_prediction(body.motion(), state)
}

fn solver_body_with_properties(
    position: Vec3,
    rotation: Quat,
    mass: f64,
    center: Vec3,
    inertia: kapitsa_math::Mat3,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
) -> SolverBody {
    let properties = MassProperties::new(mass, center, inertia).expect("valid mass");
    let mut body = RigidBody::new_dynamic(
        Transform::new(position, rotation).expect("transform"),
        properties,
    )
    .expect("dynamic body");
    body.set_linear_velocity(linear_velocity)
        .expect("finite linear velocity");
    body.set_angular_velocity(angular_velocity)
        .expect("finite angular velocity");
    let state = *body.state();
    SolverBody::from_prediction(body.motion(), state)
}

fn static_solver_body(position: Vec3, rotation: Quat) -> SolverBody {
    let body = RigidBody::new_static(Transform::new(position, rotation).expect("transform"));
    let state = *body.state();
    SolverBody::from_prediction(body.motion(), state)
}

fn solve_joint(joint: super::super::Joint, bodies: &mut [SolverBody]) {
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint,
    }];
    let mut workspace = JointWorkspace::new();
    let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
        prepare_joint_rows(bodies, workspace, &constraints, 0.01)
    };
    JointSolver::solve(&mut workspace, bodies, 0.01, 200, 8, prepare).expect("joint row solve");
}

fn test_joint_id() -> crate::world::JointId {
    use crate::{
        joint::{FixedJoint, Joint, JointCommon, JointFrame},
        rigid_body::RigidBody,
        transform::Transform,
        world::PhysicsWorld,
    };

    let mut world = PhysicsWorld::new();
    let a = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("static body");
    let b = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("static body");
    world
        .add_joint(
            a,
            b,
            Joint::Fixed(FixedJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
        )
        .expect("joint")
}

#[test]
fn fixed_joint_geometry_is_reused_until_a_body_pose_changes() -> Result<(), CollisionError> {
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            JointFrame::new(
                Vec3::new(0.3, 0.0, 0.1),
                Quat::from_axis_angle(Vec3::Z, 0.2).ok_or(CollisionError::InvalidContact)?,
            ),
            JointFrame::new(Vec3::new(-0.2, 0.1, 0.0), Quat::IDENTITY),
        ))),
    };
    let constraints = [constraint];
    let mut bodies = [
        solver_body_with_rotation(
            Vec3::ZERO,
            Quat::from_axis_angle(Vec3::X, 0.15).ok_or(CollisionError::InvalidContact)?,
        ),
        solver_body_with_rotation(
            Vec3::new(1.0, 0.2, -0.1),
            Quat::from_axis_angle(Vec3::Y, -0.25).ok_or(CollisionError::InvalidContact)?,
        ),
    ];
    let mut workspace = JointWorkspace::new();
    let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
        workspace.begin_prepare();
        prepare_joint_rows(bodies, workspace, &constraints, 0.01)?;
        workspace.finish_prepare();
        Ok::<(), CollisionError>(())
    };

    prepare(&bodies, &mut workspace)?;
    assert_eq!(workspace.fixed_geometry_recomputes(), 1);
    let first_rows = workspace.rows().to_vec();

    prepare(&bodies, &mut workspace)?;
    assert_eq!(workspace.fixed_geometry_recomputes(), 1);
    assert_eq!(workspace.rows(), first_rows.as_slice());

    bodies[1]
        .body_mut()
        .set_transform(
            Transform::new(
                Vec3::new(1.05, 0.25, -0.1),
                Quat::from_axis_angle(Vec3::Y, -0.35).ok_or(CollisionError::InvalidContact)?,
            )
            .map_err(|_| CollisionError::InvalidContact)?,
        )
        .map_err(|_| CollisionError::InvalidContact)?;
    prepare(&bodies, &mut workspace)?;
    assert_eq!(workspace.fixed_geometry_recomputes(), 2);
    assert_ne!(workspace.rows(), first_rows.as_slice());
    Ok(())
}

#[test]
fn prepared_velocity_reuse_matches_preparing_each_sweep_for_scalar_and_block3() {
    for projection in [
        super::super::JointProjection::ScalarGaussSeidel,
        super::super::JointProjection::FixedBlock3,
    ] {
        let constraints = [JointConstraint {
            id: test_joint_id(),
            body_a: 0,
            body_b: 1,
            joint: super::super::Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
                JointFrame::new(
                    Vec3::new(0.2, -0.1, 0.05),
                    Quat::from_axis_angle(Vec3::Z, 0.3).expect("rotation"),
                ),
                JointFrame::new(Vec3::new(-0.1, 0.15, 0.0), Quat::IDENTITY),
            ))),
        }];
        let bodies = [
            solver_body_with_velocities(
                Vec3::ZERO,
                Quat::from_axis_angle(Vec3::X, 0.2).expect("rotation"),
                Vec3::new(0.4, -0.2, 0.3),
                Vec3::new(0.1, 0.5, -0.2),
            ),
            solver_body_with_velocities(
                Vec3::new(1.1, 0.2, -0.1),
                Quat::from_axis_angle(Vec3::Y, -0.4).expect("rotation"),
                Vec3::new(-0.3, 0.1, 0.2),
                Vec3::new(-0.2, 0.3, 0.4),
            ),
        ];
        let options = JointSolverOptions::legacy().with_projection(projection);
        let step = JointSolverStep::new(0.01, options);

        // Reuse: prepare once, then run only prepared velocity sweeps.
        let mut reused = bodies;
        let mut reuse_workspace = JointWorkspace::new();
        JointSolver::prepare(&mut reuse_workspace, &reused, &constraints, 0.01)
            .expect("prepare once");
        for _ in 0..6 {
            JointSolver::solve_velocity_prepared_iteration(
                &mut reuse_workspace,
                &mut reused,
                &constraints,
                0.01,
                options,
            )
            .expect("prepared velocity sweep");
        }

        // Reference: rebuild and solve each sweep, as the incompatible path does.
        let mut rebuilt = bodies;
        let mut rebuild_workspace = JointWorkspace::new();
        for _ in 0..6 {
            JointSolver::solve_velocity_joint(
                &mut rebuild_workspace,
                &mut rebuilt,
                constraints[0],
                0,
                step,
                false,
            )
            .expect("per-sweep velocity solve");
        }

        for (reused_body, rebuilt_body) in reused.iter().zip(rebuilt.iter()) {
            let reused_state = reused_body.state();
            let rebuilt_state = rebuilt_body.state();
            assert_eq!(
                reused_state.linear_velocity(),
                rebuilt_state.linear_velocity()
            );
            assert_eq!(
                reused_state.angular_velocity(),
                rebuilt_state.angular_velocity()
            );
        }
    }
}

#[test]
fn new_step_invalidates_prepared_velocity_rows_and_operators() {
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    }];
    let bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 1.2)];
    let options =
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3);
    let mut workspace = JointWorkspace::new();

    JointSolver::prepare(&mut workspace, &bodies, &constraints, 0.01).expect("prepare");
    let mut solved = bodies;
    JointSolver::solve_velocity_prepared_iteration(
        &mut workspace,
        &mut solved,
        &constraints,
        0.01,
        options,
    )
    .expect("prepared sweep");
    assert_eq!(workspace.fixed_block_velocity_recomputes(), 2);

    // A second prepared sweep reuses the cached factorizations.
    JointSolver::solve_velocity_prepared_iteration(
        &mut workspace,
        &mut solved,
        &constraints,
        0.01,
        options,
    )
    .expect("second prepared sweep");
    assert_eq!(workspace.fixed_block_velocity_recomputes(), 2);

    // A new step clears cached operators and accumulated impulses.
    JointSolver::begin_step(&mut workspace);
    assert_eq!(workspace.fixed_block_velocity_recomputes(), 0);
    assert!(
        workspace
            .rows()
            .iter()
            .all(|row| row.accumulated_impulse == 0.0)
    );

    JointSolver::prepare(&mut workspace, &bodies, &constraints, 0.01).expect("prepare again");
    let mut solved_again = bodies;
    JointSolver::solve_velocity_prepared_iteration(
        &mut workspace,
        &mut solved_again,
        &constraints,
        0.01,
        options,
    )
    .expect("prepared sweep after reset");
    // Operators were rebuilt from scratch for the new step, not reused.
    assert_eq!(workspace.fixed_block_velocity_recomputes(), 2);
}

#[test]
fn solve_prepares_once_before_each_sweep_and_sweeps_each_row_once() {
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 1.5)];
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    }];
    let mut workspace = JointWorkspace::new();
    let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
        prepare_joint_rows(bodies, workspace, &constraints, 0.01)
    };

    JointSolver::solve(&mut workspace, &mut bodies, 0.01, 2, 3, prepare)
        .expect("solve fixed joint");

    assert_eq!(
        workspace.solver_calls(),
        [
            JointSolverCall::AllRowsPrepared,
            JointSolverCall::PositionSweep {
                preparing_rows: false,
                row_count: 6,
            },
            JointSolverCall::AllRowsPrepared,
            JointSolverCall::PositionSweep {
                preparing_rows: false,
                row_count: 6,
            },
            JointSolverCall::AllRowsPrepared,
            JointSolverCall::VelocitySweep {
                preparing_rows: false,
                row_count: 6,
            },
            JointSolverCall::AllRowsPrepared,
            JointSolverCall::VelocitySweep {
                preparing_rows: false,
                row_count: 6,
            },
            JointSolverCall::AllRowsPrepared,
            JointSolverCall::VelocitySweep {
                preparing_rows: false,
                row_count: 6,
            },
        ]
    );
}

#[test]
fn shared_solver_adaptive_policy_observes_minimum_sweeps() {
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
    let row = anchor_row(0.0);
    let mut workspace = JointWorkspace::new();
    let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
        let distance = bodies[1].state().transform().translation().x
            - bodies[0].state().transform().translation().x;
        workspace.push(ConstraintRow {
            error: distance - 1.0,
            ..row
        });
        Ok(())
    };
    let tolerance = super::super::JointTolerance {
        linear_m: 1.0e-8,
        angular_rad: 1.0e-8,
        velocity_mps: 1.0e-8,
        angular_velocity_radps: 1.0e-8,
    };
    let options =
        JointSolverOptions::legacy().with_iterations(super::super::JointIterations::Adaptive {
            position: super::super::IterationRange::new(2, 6),
            velocity: super::super::IterationRange::new(3, 5),
            tolerance,
        });
    JointSolver::solve_with_options(&mut workspace, &mut bodies, 0.01, 1, 1, options, prepare)
        .expect("adaptive shared solver");

    let position_sweeps = workspace
        .solver_calls()
        .iter()
        .filter(|call| matches!(call, JointSolverCall::PositionSweep { .. }))
        .count();
    let velocity_sweeps = workspace
        .solver_calls()
        .iter()
        .filter(|call| matches!(call, JointSolverCall::VelocitySweep { .. }))
        .count();
    assert_eq!((position_sweeps, velocity_sweeps), (2, 3));
    assert!(workspace.residual().position_converged(tolerance));
    assert!(workspace.residual().velocity_converged(tolerance));
}

#[cfg(feature = "bench-internals")]
#[test]
fn solver_mass_cache_metrics_report_reuse_and_pose_updates() {
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 1.5)];
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    };
    let constraints = [constraint];
    let mut workspace = JointWorkspace::new();
    let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
        prepare_joint_rows(bodies, workspace, &constraints, 0.01)
    };

    JointSolver::solve_with_options(
        &mut workspace,
        &mut bodies,
        0.01,
        2,
        2,
        JointSolverOptions::legacy().with_mass_cache(true),
        prepare,
    )
    .expect("solve fixed joint");

    let metrics = workspace.metrics();
    assert!(metrics.solver_mass_cache_hits > 0, "{metrics:?}");
    assert!(metrics.solver_mass_cache_misses > 0, "{metrics:?}");
    assert!(metrics.inverse_inertia_updates > 0, "{metrics:?}");
    assert!(metrics.inverse_inertia_updates <= metrics.solver_mass_cache_misses);
}

fn anchor_row(error: f64) -> ConstraintRow {
    ConstraintRow {
        key: 0,
        dimension: RowDimension::Linear,
        body_a: 0,
        body_b: 1,
        linear_a: -Vec3::X,
        angular_a: Vec3::ZERO,
        position_angular_a: Vec3::ZERO,
        linear_b: Vec3::X,
        angular_b: Vec3::ZERO,
        error,
        target_velocity: 0.0,
        force_bias: 0.0,
        force_driven: false,
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

#[cfg(feature = "bench-internals")]
#[test]
fn cached_velocity_effective_mass_refreshes_after_orientation_change() {
    let inertia = kapitsa_math::Mat3::from_diagonal(Vec3::new(1.0, 1.5, 2.0));
    let mut bodies = [
        solver_body_with_properties(
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
            Vec3::ZERO,
            inertia,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
        solver_body_with_properties(
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
            Vec3::ZERO,
            inertia,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
    ];
    let mut row = anchor_row(0.0);
    row.linear_a = Vec3::ZERO;
    row.linear_b = Vec3::ZERO;
    row.angular_a = Vec3::X;
    row.angular_b = -Vec3::X;
    let options = JointSolverOptions::legacy();
    let mut slots = vec![SolverMassSlot::default(); bodies.len()];
    let mut cache_stats = SolverMassCacheStats::default();
    solve_velocity_row(
        &mut row,
        &mut slots,
        &mut bodies,
        1.0 / 60.0,
        options,
        &mut cache_stats,
    )
    .expect("initial velocity row solve");
    let initial_effective_mass = row.cached_velocity_effective_mass;
    assert!(initial_effective_mass > 0.0);

    bodies[1]
        .body_mut()
        .apply_constraint_position_impulse(Vec3::ZERO, Vec3::Z)
        .expect("rotate body for cache invalidation");
    let mut cache_stats = SolverMassCacheStats::default();
    solve_velocity_row(
        &mut row,
        &mut slots,
        &mut bodies,
        1.0 / 60.0,
        options,
        &mut cache_stats,
    )
    .expect("velocity row solve after orientation change");
    let refreshed_effective_mass = row.cached_velocity_effective_mass;
    assert!(cache_stats.inverse_inertia_updates > 0);
    assert_ne!(refreshed_effective_mass, initial_effective_mass);

    let (current_effective_mass, _) = effective_mass_for_row(
        &bodies,
        &mut slots,
        row,
        options,
        &mut SolverMassCacheStats::default(),
    )
    .expect("current effective mass");
    assert!((refreshed_effective_mass - current_effective_mass).abs() < 1.0e-12);
}

#[cfg(feature = "bench-internals")]
#[test]
fn post_sweep_residual_uses_current_frames_without_double_counting_preparation() {
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    };
    let constraints = [constraint];
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &bodies, &constraints, 0.01)
        .expect("prepare initially separated frames");
    let prepared_rows = workspace.metrics().prepared_rows;
    bodies[1]
        .body_mut()
        .set_transform(Transform::identity())
        .expect("coincident frame");

    JointSolver::measure_residual(&mut workspace, &bodies, &constraints, 0.01)
        .expect("measure current frames");

    assert_eq!(workspace.residual(), JointResidual::ZERO);
    assert_eq!(workspace.metrics().prepared_rows, prepared_rows);
}

#[cfg(feature = "bench-internals")]
#[test]
fn residual_uses_absolute_geometry_and_keeps_linear_and_angular_units_separate() {
    let bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    let positive =
        measure_prepared_rows(&[anchor_row(0.25)], &bodies, 0.01).expect("positive residual");
    let negative =
        measure_prepared_rows(&[anchor_row(-0.25)], &bodies, 0.01).expect("negative residual");
    assert_eq!(positive.max_linear_m, 0.25);
    assert_eq!(negative.max_linear_m, positive.max_linear_m);

    let mut angular_row = anchor_row(0.125);
    angular_row.dimension = RowDimension::Angular;
    let angular = measure_prepared_rows(&[angular_row], &bodies, 0.01).expect("angular residual");
    assert_eq!(angular.max_linear_m, 0.0);
    assert_eq!(angular.max_angular_rad, 0.125);

    let moving_bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO, Vec3::X * 0.5),
    ];
    angular_row.linear_a = Vec3::ZERO;
    angular_row.linear_b = Vec3::ZERO;
    angular_row.angular_a = -Vec3::X;
    angular_row.angular_b = Vec3::X;
    let angular_velocity = measure_prepared_rows(&[angular_row], &moving_bodies, 0.01)
        .expect("angular velocity residual");
    assert_eq!(angular_velocity.max_velocity_mps, 0.0);
    assert_eq!(angular_velocity.max_angular_velocity_radps, 0.5);
}

#[cfg(feature = "bench-internals")]
#[test]
fn residual_applies_unilateral_kkt_conditions_and_rejects_overflow() {
    let bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    let mut satisfied_limit = anchor_row(0.25);
    satisfied_limit.min_multiplier = 0.0;
    let satisfied = measure_prepared_rows(&[satisfied_limit], &bodies, 0.01)
        .expect("satisfied lower-bound limit");
    assert_eq!(satisfied.max_linear_m, 0.0);

    let mut violated_limit = anchor_row(-0.25);
    violated_limit.min_multiplier = 0.0;
    let violated = measure_prepared_rows(&[violated_limit], &bodies, 0.01)
        .expect("violated lower-bound limit");
    assert_eq!(violated.max_linear_m, 0.25);

    let moving_bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(Vec3::ZERO, Quat::IDENTITY, Vec3::X * 0.25, Vec3::ZERO),
    ];
    let mut satisfied_impulse = anchor_row(0.0);
    satisfied_impulse.min_impulse = 0.0;
    assert_eq!(
        measure_prepared_rows(&[satisfied_impulse], &moving_bodies, 0.01)
            .expect("satisfied impulse lower bound")
            .max_velocity_mps,
        0.0
    );
    let mut violated_impulse = anchor_row(0.0);
    violated_impulse.min_impulse = 0.0;
    let violating_bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(Vec3::ZERO, Quat::IDENTITY, -Vec3::X * 0.25, Vec3::ZERO),
    ];
    assert_eq!(
        measure_prepared_rows(&[violated_impulse], &violating_bodies, 0.01)
            .expect("violated impulse lower bound")
            .max_velocity_mps,
        0.25
    );

    let fast_bodies = [
        solver_body_with_velocities(Vec3::ZERO, Quat::IDENTITY, Vec3::X * f64::MAX, Vec3::ZERO),
        solver_body(Vec3::ZERO),
    ];
    let mut overflow_row = anchor_row(0.0);
    overflow_row.target_velocity = f64::MAX;
    assert_eq!(
        measure_prepared_rows(&[overflow_row], &fast_bodies, 0.01),
        Err(CollisionError::ArithmeticOverflow)
    );
}

#[cfg(feature = "bench-internals")]
#[test]
fn residual_for_disabled_joint_is_zero() {
    let bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    let mut joint =
        super::super::FixedJoint::new(JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY));
    joint.common.enabled = false;
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Fixed(joint),
    }];
    let mut workspace = JointWorkspace::new();

    JointSolver::measure_residual(&mut workspace, &bodies, &constraints, 0.01)
        .expect("measure disabled joint");

    assert_eq!(workspace.residual(), JointResidual::ZERO);
}

#[test]
fn xpbd_row_corrects_position_and_reuses_rows_across_iterations() {
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
    let mut workspace = JointWorkspace::new();
    let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
        let distance = bodies[1].state().transform().translation().x
            - bodies[0].state().transform().translation().x;
        workspace.push(anchor_row(distance - 1.0));
        Ok(())
    };
    JointSolver::solve(&mut workspace, &mut bodies, 0.01, 4, 1, prepare).expect("solve row");
    let distance = bodies[1].state().transform().translation().x
        - bodies[0].state().transform().translation().x;
    assert!((distance - 1.0).abs() < 1.0e-12, "distance={distance}");
    assert_eq!(workspace.rows().len(), 1);
}

#[test]
fn d6_limit_drive_enable_and_dt_changes_keep_multiplier_identity() {
    let bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
    let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    let mut joint = D6Joint::new(common);
    let mut constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: Joint::D6(joint),
    };
    let mut workspace = JointWorkspace::new();

    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.01).unwrap();
    workspace
        .rows_mut()
        .iter_mut()
        .find(|row| row.key == 0)
        .unwrap()
        .accumulated_multiplier = 3.0;
    workspace
        .rows_mut()
        .iter_mut()
        .find(|row| row.key == 0)
        .unwrap()
        .accumulated_impulse = 4.0;

    joint.linear_motion[0] = JointMotion::Limited(JointLimit::new(-0.5, 0.5));
    constraint.joint = Joint::D6(joint);
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.01).unwrap();
    let limit_row = workspace.rows().iter().find(|row| row.key == 3).unwrap();
    assert!(limit_row.position_enabled);
    assert_eq!(limit_row.accumulated_multiplier, 0.0);
    assert_eq!(limit_row.accumulated_impulse, 0.0);
    workspace
        .rows_mut()
        .iter_mut()
        .find(|row| row.key == 3)
        .unwrap()
        .accumulated_multiplier = 5.0;
    workspace
        .rows_mut()
        .iter_mut()
        .find(|row| row.key == 3)
        .unwrap()
        .accumulated_impulse = 7.0;

    joint.linear_motion[0] = JointMotion::Free;
    joint.linear_drives[0] = JointDrive {
        enabled: true,
        target_position: 0.0,
        target_velocity: 0.0,
        stiffness: 1.0,
        damping: 0.0,
        max_force: 100.0,
    };
    constraint.joint = Joint::D6(joint);
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.01).unwrap();
    let drive_row = workspace.rows().iter().find(|row| row.key == 6).unwrap();
    assert_eq!(drive_row.accumulated_multiplier, 0.0);
    assert_eq!(drive_row.accumulated_impulse, 0.0);

    workspace
        .rows_mut()
        .iter_mut()
        .find(|row| row.key == 6)
        .unwrap()
        .accumulated_multiplier = 11.0;
    workspace
        .rows_mut()
        .iter_mut()
        .find(|row| row.key == 6)
        .unwrap()
        .accumulated_impulse = 13.0;
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.01).unwrap();
    let drive_row = workspace.rows().iter().find(|row| row.key == 6).unwrap();
    assert_eq!(drive_row.accumulated_multiplier, 11.0);
    assert_eq!(drive_row.accumulated_impulse, 13.0);

    JointSolver::begin_step(&mut workspace);
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.02).unwrap();
    let drive_row = workspace.rows().iter().find(|row| row.key == 6).unwrap();
    assert_eq!(drive_row.accumulated_multiplier, 0.0);
    assert_eq!(drive_row.accumulated_impulse, 0.0);

    joint.common.enabled = false;
    constraint.joint = Joint::D6(joint);
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.02).unwrap();
    assert!(workspace.rows().is_empty());

    joint.common.enabled = true;
    constraint.joint = Joint::D6(joint);
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.02).unwrap();
    let drive_row = workspace.rows().iter().find(|row| row.key == 6).unwrap();
    assert_eq!(drive_row.accumulated_multiplier, 0.0);
    assert_eq!(drive_row.accumulated_impulse, 0.0);
}

#[test]
fn compliant_row_scales_with_dt_and_preserves_finite_body_state() {
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
    let mut workspace = JointWorkspace::new();
    let mut row = anchor_row(0.0);
    row.compliance = 1.0e-3;
    let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
        let distance = bodies[1].state().transform().translation().x
            - bodies[0].state().transform().translation().x;
        workspace.push(ConstraintRow {
            error: distance - 1.0,
            force_bias: 0.0,
            force_driven: false,
            ..row
        });
        Ok(())
    };
    JointSolver::solve(&mut workspace, &mut bodies, 0.1, 8, 1, prepare)
        .expect("solve compliant row");
    assert!(bodies.iter().all(|body| {
        body.state().transform().translation().is_finite()
            && body.state().linear_velocity().is_finite()
            && body.state().angular_velocity().is_finite()
    }));
}

#[test]
fn fixed_joint_locks_connector_translation_and_rotation() {
    let rotation = Quat::from_axis_angle(Vec3::Z, 0.6).expect("rotation");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::new(2.0, -1.0, 0.5), rotation),
    ];
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    solve_joint(
        super::super::Joint::Fixed(super::super::FixedJoint::new(common)),
        &mut bodies,
    );
    let position_delta = bodies[1]
        .state()
        .transform()
        .translation()
        .checked_sub(bodies[0].state().transform().translation())
        .expect("finite delta");
    let angle_delta = relative_rotation_error(
        bodies[0].state().transform().rotation(),
        bodies[1].state().transform().rotation(),
    )
    .expect("relative angle");
    assert!(position_delta.length() < 1.0e-10);
    assert!(angle_delta.length() < 1.0e-10);
}

#[test]
fn fixed_joint_rotation_log_jacobian_matches_finite_differences() {
    fn component(value: Vec3, index: usize) -> f64 {
        match index {
            0 => value.x,
            1 => value.y,
            2 => value.z,
            _ => unreachable!("three rotation coordinates"),
        }
    }

    let rotation_a =
        Quat::from_axis_angle(Vec3::new(0.4, -0.3, 0.7), 0.73).expect("frame A rotation");
    let relative_axis = Vec3::new(0.3, -0.7, 0.2)
        .try_normalize()
        .expect("relative axis");
    let world_axes = [Vec3::X, Vec3::Y, Vec3::Z];
    let epsilon = 1.0e-7;

    for angle in [1.0e-5, 0.7, 2.8, std::f64::consts::PI - 1.0e-5] {
        let relative = Quat::from_axis_angle(relative_axis, angle).expect("relative rotation");
        let rotation_b = rotation_a * relative;
        let error = relative_rotation_error(rotation_a, rotation_b).expect("log map");
        let coefficient = fixed_rotation_coefficient(error).expect("finite rotation coefficient");

        for (row_index, local_axis) in world_axes.into_iter().enumerate() {
            let row_axis = fixed_rotation_row_axis(rotation_a, error, local_axis, coefficient)
                .expect("finite FixedJoint angular Jacobian");

            for world_axis in world_axes {
                let perturbation =
                    Quat::from_axis_angle(world_axis, epsilon).expect("small rotation");
                let inverse_perturbation = perturbation.conjugate();
                let expected_b = row_axis.dot(world_axis);
                let actual_b = (component(
                    relative_rotation_error(rotation_a, perturbation * rotation_b)
                        .expect("positive B perturbation"),
                    row_index,
                ) - component(
                    relative_rotation_error(rotation_a, inverse_perturbation * rotation_b)
                        .expect("negative B perturbation"),
                    row_index,
                )) / (2.0 * epsilon);
                assert!(
                    (actual_b - expected_b).abs() <= 3.0e-6,
                    "B Jacobian angle={angle} row={row_index} axis={world_axis:?}: {actual_b} != {expected_b}"
                );

                let expected_a = -expected_b;
                let actual_a = (component(
                    relative_rotation_error(perturbation * rotation_a, rotation_b)
                        .expect("positive A perturbation"),
                    row_index,
                ) - component(
                    relative_rotation_error(inverse_perturbation * rotation_a, rotation_b)
                        .expect("negative A perturbation"),
                    row_index,
                )) / (2.0 * epsilon);
                assert!(
                    (actual_a - expected_a).abs() <= 3.0e-6,
                    "A Jacobian angle={angle} row={row_index} axis={world_axis:?}: {actual_a} != {expected_a}"
                );
            }
        }
    }
}

#[test]
fn fixed_joint_point_jacobians_match_finite_differences_with_offset_com() {
    fn make_body(position: Vec3, rotation: Quat, center: Vec3, mass: f64) -> SolverBody {
        let properties = MassProperties::new(
            mass,
            center,
            kapitsa_math::Mat3::from_diagonal(Vec3::new(0.7, 1.1, 1.4)),
        )
        .expect("mass properties");
        let body = RigidBody::new_dynamic(
            Transform::new(position, rotation).expect("body transform"),
            properties,
        )
        .expect("dynamic body");
        let state = *body.state();
        SolverBody::from_prediction(body.motion(), state)
    }

    fn rotate_about_center(
        transform: Transform,
        local_center: Vec3,
        axis: Vec3,
        angle: f64,
    ) -> Transform {
        let center = transform.transform_point(local_center);
        let delta = Quat::from_axis_angle(axis, angle).expect("small rotation");
        let rotation = delta * transform.rotation();
        let rotated_local_center = rotation
            .rotate_vector(local_center)
            .expect("rotated center of mass");
        Transform::new(center - rotated_local_center, rotation).expect("rotated body transform")
    }

    let center_a = Vec3::new(0.17, -0.09, 0.12);
    let center_b = Vec3::new(-0.11, 0.14, 0.06);
    let rotation_a = Quat::from_axis_angle(Vec3::new(0.2, 0.8, -0.3), 0.37).expect("rotation A");
    let rotation_b = Quat::from_axis_angle(Vec3::new(-0.4, 0.1, 0.9), -0.29).expect("rotation B");
    let bodies = [
        make_body(Vec3::new(-0.3, 0.4, 0.2), rotation_a, center_a, 2.0),
        make_body(Vec3::new(0.9, -0.2, 0.5), rotation_b, center_b, 3.5),
    ];
    let frame_a = JointFrame::new(
        Vec3::new(0.3, -0.2, 0.1),
        Quat::from_axis_angle(Vec3::Y, 0.23).expect("frame A rotation"),
    );
    let frame_b = JointFrame::new(
        Vec3::new(-0.15, 0.25, -0.08),
        Quat::from_axis_angle(Vec3::X, -0.31).expect("frame B rotation"),
    );
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            frame_a, frame_b,
        ))),
    };
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.01)
        .expect("prepare fixed point rows");
    let row = workspace.rows()[0];
    let transform_a = bodies[0].state().transform();
    let transform_b = bodies[1].state().transform();
    let error = |transform_a: Transform, transform_b: Transform| {
        let anchor_a = transform_a.transform_point(frame_a.local_anchor);
        let anchor_b = transform_b.transform_point(frame_b.local_anchor);
        (anchor_b - anchor_a).dot(Vec3::X)
    };
    assert!((row.error - error(transform_a, transform_b)).abs() < 1.0e-12);

    let epsilon = 1.0e-7;
    for direction in [Vec3::X, Vec3::Y, Vec3::Z] {
        let plus_a = Transform::new(
            transform_a.translation() + direction * epsilon,
            transform_a.rotation(),
        )
        .expect("translated A");
        let minus_a = Transform::new(
            transform_a.translation() - direction * epsilon,
            transform_a.rotation(),
        )
        .expect("translated A");
        let translation_a =
            (error(plus_a, transform_b) - error(minus_a, transform_b)) / (2.0 * epsilon);
        assert!((translation_a - row.linear_a.dot(direction)).abs() < 3.0e-6);

        let plus_b = Transform::new(
            transform_b.translation() + direction * epsilon,
            transform_b.rotation(),
        )
        .expect("translated B");
        let minus_b = Transform::new(
            transform_b.translation() - direction * epsilon,
            transform_b.rotation(),
        )
        .expect("translated B");
        let translation_b =
            (error(transform_a, plus_b) - error(transform_a, minus_b)) / (2.0 * epsilon);
        assert!((translation_b - row.linear_b.dot(direction)).abs() < 3.0e-6);

        let rotation_a_delta = (error(
            rotate_about_center(transform_a, center_a, direction, epsilon),
            transform_b,
        ) - error(
            rotate_about_center(transform_a, center_a, direction, -epsilon),
            transform_b,
        )) / (2.0 * epsilon);
        assert!((rotation_a_delta - row.angular_a.dot(direction)).abs() < 3.0e-6);

        let rotation_b_delta = (error(
            transform_a,
            rotate_about_center(transform_b, center_b, direction, epsilon),
        ) - error(
            transform_a,
            rotate_about_center(transform_b, center_b, direction, -epsilon),
        )) / (2.0 * epsilon);
        assert!((rotation_b_delta - row.angular_b.dot(direction)).abs() < 3.0e-6);
    }
}

#[test]
fn fixed_joint_position_order_reports_anchor_rotation_coupling() {
    #[derive(Clone, Copy)]
    struct Residual {
        anchor: f64,
        frame: f64,
    }

    fn make_body(
        position: Vec3,
        rotation: Quat,
        mass: f64,
        center: Vec3,
        inertia: Vec3,
    ) -> SolverBody {
        let properties =
            MassProperties::new(mass, center, kapitsa_math::Mat3::from_diagonal(inertia))
                .expect("mass properties");
        let body = RigidBody::new_dynamic(
            Transform::new(position, rotation).expect("transform"),
            properties,
        )
        .expect("dynamic body");
        let state = *body.state();
        SolverBody::from_prediction(body.motion(), state)
    }

    fn make_constraint() -> JointConstraint {
        let common = JointCommon::new(
            JointFrame::new(Vec3::new(0.28, -0.16, 0.11), Quat::IDENTITY),
            JointFrame::new(Vec3::new(-0.12, 0.23, -0.09), Quat::IDENTITY),
        );
        JointConstraint {
            id: test_joint_id(),
            body_a: 0,
            body_b: 1,
            joint: Joint::Fixed(super::super::FixedJoint::new(common)),
        }
    }

    fn make_bodies(angle: f64) -> [SolverBody; 2] {
        let axis = Vec3::new(0.3, -0.6, 0.7)
            .try_normalize()
            .expect("rotation axis");
        let rotation_a =
            Quat::from_axis_angle(Vec3::new(0.2, 0.8, -0.3), 0.23).expect("rotation A");
        let relative = Quat::from_axis_angle(axis, angle).expect("relative rotation");
        let rotation_b = rotation_a * relative;
        [
            make_body(
                Vec3::new(-0.4, 0.2, -0.1),
                rotation_a,
                1.7,
                Vec3::new(0.12, -0.07, 0.09),
                Vec3::new(0.35, 0.8, 1.1),
            ),
            make_body(
                Vec3::new(0.9, -0.25, 0.3),
                rotation_b,
                3.4,
                Vec3::new(-0.08, 0.14, 0.04),
                Vec3::new(1.4, 0.6, 0.95),
            ),
        ]
    }

    fn residual(bodies: &[SolverBody], constraint: JointConstraint) -> Residual {
        let joint = match constraint.joint {
            Joint::Fixed(joint) => joint,
            _ => unreachable!("fixed diagnostic constraint"),
        };
        let (anchor_a, rotation_a) =
            world_frame(bodies, constraint.body_a, joint.common.frame_a).expect("frame A");
        let (anchor_b, rotation_b) =
            world_frame(bodies, constraint.body_b, joint.common.frame_b).expect("frame B");
        Residual {
            anchor: (anchor_b - anchor_a).length(),
            frame: relative_rotation_error(rotation_a, rotation_b)
                .expect("frame error")
                .length(),
        }
    }

    fn solve_subset(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        range: std::ops::Range<usize>,
    ) {
        workspace
            .ensure_solver_mass_slots(bodies.len())
            .expect("solver mass workspace");
        let mut cache_stats = SolverMassCacheStats::default();
        let (rows, slots) = workspace.solver_buffers_mut(false);
        solve_position_rows(
            &mut rows[range],
            slots,
            bodies,
            0.01,
            JointSolverOptions::legacy(),
            &mut cache_stats,
        )
        .expect("position row subset");
    }

    fn reprepare(
        workspace: &mut JointWorkspace,
        bodies: &[SolverBody],
        constraint: JointConstraint,
    ) {
        JointSolver::prepare(workspace, bodies, &[constraint], 0.01)
            .expect("refresh joint geometry");
    }

    fn run_world_steps(angle: f64, constraint: JointConstraint) -> [Residual; 3] {
        let bodies = make_bodies(angle);
        let mut world = crate::world::PhysicsWorld::new();
        let ids = bodies.map(|body| {
            world
                .add_body(RigidBody::from_motion(body.body))
                .expect("world dynamic body")
        });
        let joint = match constraint.joint {
            Joint::Fixed(joint) => Joint::Fixed(joint),
            _ => unreachable!("fixed diagnostic constraint"),
        };
        world
            .add_joint(ids[0], ids[1], joint)
            .expect("world fixed joint");
        let measure = |world: &crate::world::PhysicsWorld| {
            let body_a = world.body(ids[0]).expect("body A");
            let body_b = world.body(ids[1]).expect("body B");
            let frame_a = match constraint.joint {
                Joint::Fixed(joint) => joint.common.frame_a,
                _ => unreachable!("fixed diagnostic constraint"),
            };
            let frame_b = match constraint.joint {
                Joint::Fixed(joint) => joint.common.frame_b,
                _ => unreachable!("fixed diagnostic constraint"),
            };
            let transform_a = body_a.state().transform();
            let transform_b = body_b.state().transform();
            let anchor_a = transform_a.transform_point(frame_a.local_anchor);
            let anchor_b = transform_b.transform_point(frame_b.local_anchor);
            let rotation_a = transform_a.rotation() * frame_a.local_rotation;
            let rotation_b = transform_b.rotation() * frame_b.local_rotation;
            let relative = canonical_relative_rotation(rotation_a, rotation_b)
                .expect("normalized relative frame");
            Residual {
                anchor: (anchor_b - anchor_a).length(),
                frame: 2.0
                    * relative
                        .x
                        .hypot(relative.y)
                        .hypot(relative.z)
                        .atan2(relative.w),
            }
        };
        let initial = measure(&world);
        world.step(0.01).expect("first force-free physics step");
        let first = measure(&world);
        world.step(0.01).expect("second force-free physics step");
        [initial, first, measure(&world)]
    }

    for degrees in [5.0_f64, 15.0, 45.0, 90.0] {
        let angle = degrees.to_radians();
        let constraint = make_constraint();

        let mut bodies_a = make_bodies(angle);
        let mut workspace_a = JointWorkspace::new();
        reprepare(&mut workspace_a, &bodies_a, constraint);
        let initial = residual(&bodies_a, constraint);
        solve_subset(&mut workspace_a, &mut bodies_a, 0..3);
        let a_after_linear = residual(&bodies_a, constraint);
        solve_subset(&mut workspace_a, &mut bodies_a, 3..6);
        let a_after_full = residual(&bodies_a, constraint);

        let mut bodies_b = make_bodies(angle);
        let mut workspace_b = JointWorkspace::new();
        reprepare(&mut workspace_b, &bodies_b, constraint);
        solve_subset(&mut workspace_b, &mut bodies_b, 3..6);
        let b_after_angular = residual(&bodies_b, constraint);
        reprepare(&mut workspace_b, &bodies_b, constraint);
        solve_subset(&mut workspace_b, &mut bodies_b, 0..3);
        let b_after_full = residual(&bodies_b, constraint);

        let mut bodies_c = make_bodies(angle);
        let mut workspace_c = JointWorkspace::new();
        reprepare(&mut workspace_c, &bodies_c, constraint);
        solve_subset(&mut workspace_c, &mut bodies_c, 0..3);
        let c_after_linear = residual(&bodies_c, constraint);
        reprepare(&mut workspace_c, &bodies_c, constraint);
        solve_subset(&mut workspace_c, &mut bodies_c, 3..6);
        let c_after_angular = residual(&bodies_c, constraint);
        reprepare(&mut workspace_c, &bodies_c, constraint);
        solve_subset(&mut workspace_c, &mut bodies_c, 0..3);
        let c_after_full = residual(&bodies_c, constraint);

        let mut bodies_d = make_bodies(angle);
        let mut workspace_d = JointWorkspace::new();
        JointSolver::begin_step(&mut workspace_d);
        JointSolver::solve_position_joint(
            &mut workspace_d,
            &mut bodies_d,
            constraint,
            0,
            0.01,
            JointSolverOptions::legacy(),
        )
        .expect("incrementally refreshed fixed-joint projection");
        for (reference, actual) in bodies_c.iter().zip(&bodies_d) {
            let reference_transform = reference.state().transform();
            let actual_transform = actual.state().transform();
            assert!(
                (reference_transform.translation() - actual_transform.translation()).length()
                    <= 1.0e-12
            );
            let reference_rotation = reference_transform.rotation();
            let actual_rotation = actual_transform.rotation();
            let rotation_dot = reference_rotation.x * actual_rotation.x
                + reference_rotation.y * actual_rotation.y
                + reference_rotation.z * actual_rotation.z
                + reference_rotation.w * actual_rotation.w;
            assert!(rotation_dot.abs() >= 1.0 - 1.0e-13);
        }

        let world_steps = run_world_steps(angle, constraint);
        println!(
            "angle_deg={degrees:.0} start=({:.6e}m,{:.6e}rad) A_linear=({:.6e},{:.6e}) A_full=({:.6e},{:.6e}) A_angular_anchor_delta={:.6e}m B_angular=({:.6e},{:.6e}) B_full=({:.6e},{:.6e}) C_linear=({:.6e},{:.6e}) C_angular=({:.6e},{:.6e}) C_full=({:.6e},{:.6e}) world_step1=({:.6e},{:.6e}) world_step2=({:.6e},{:.6e})",
            initial.anchor,
            initial.frame,
            a_after_linear.anchor,
            a_after_linear.frame,
            a_after_full.anchor,
            a_after_full.frame,
            a_after_full.anchor - a_after_linear.anchor,
            b_after_angular.anchor,
            b_after_angular.frame,
            b_after_full.anchor,
            b_after_full.frame,
            c_after_linear.anchor,
            c_after_linear.frame,
            c_after_angular.anchor,
            c_after_angular.frame,
            c_after_full.anchor,
            c_after_full.frame,
            world_steps[1].anchor,
            world_steps[1].frame,
            world_steps[2].anchor,
            world_steps[2].frame,
        );

        for value in [
            initial,
            a_after_linear,
            a_after_full,
            b_after_angular,
            b_after_full,
            c_after_linear,
            c_after_angular,
            c_after_full,
            world_steps[0],
            world_steps[1],
            world_steps[2],
        ] {
            assert!(value.anchor.is_finite() && value.frame.is_finite());
        }
        assert!(c_after_full.anchor < initial.anchor);
        assert!(c_after_full.frame < initial.frame);
        assert!(world_steps[2].anchor <= world_steps[1].anchor);
        assert!(world_steps[2].frame <= world_steps[1].frame);
    }
}

#[test]
fn undefined_twist_does_not_fail_joint_preparation() {
    let swing = Quat::from_axis_angle(Vec3::X, std::f64::consts::PI).expect("180-degree swing");
    assert_eq!(twist_angle(swing), Ok(None));
    assert!(matches!(
        twist_angle(Quat::from_axis_angle(Vec3::Z, 0.7).expect("defined twist")),
        Ok(Some(angle)) if (angle - 0.7).abs() < 1.0e-12
    ));

    let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    let limit = super::super::JointLimit::new(-0.25, 0.25);
    let mut ball = super::super::BallJoint::new(common);
    ball.swing_limit = Some(0.5);
    ball.twist_limit = Some(limit);
    let joints = [
        Joint::Ball(ball),
        Joint::Hinge(super::super::HingeJoint {
            common,
            angle_limit: Some(limit),
            drive: super::super::JointDrive::DISABLED,
        }),
        Joint::Cylindrical(super::super::CylindricalJoint {
            angle_limit: Some(limit),
            ..super::super::CylindricalJoint::new(common)
        }),
        Joint::Planar(super::super::PlanarJoint {
            rotation_limit: Some(limit),
            ..super::super::PlanarJoint::new(common)
        }),
        Joint::ConeTwist(super::super::ConeTwistJoint::new(common, [0.5, 0.5], limit)),
    ];
    let bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::ZERO, swing),
    ];
    for joint in joints {
        let constraint = JointConstraint {
            id: test_joint_id(),
            body_a: 0,
            body_b: 1,
            joint,
        };
        let mut workspace = JointWorkspace::new();
        JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.01)
            .expect("undefined twist is not a failed physics step");
        assert!(workspace.rows().iter().all(|row| row.error.is_finite()));
    }
}

#[cfg(feature = "bench-internals")]
#[test]
fn fixed_block_projection_reaches_connector_tolerance_in_fewer_sweeps() {
    fn iterations_to_tolerance(projection: super::super::JointProjection) -> usize {
        let rotation_a = Quat::from_axis_angle(Vec3::Z, 0.23).expect("rotation");
        let rotation_b = Quat::from_axis_angle(Vec3::new(1.0, 0.6, -0.2), 0.51).expect("rotation");
        let mut bodies = [
            solver_body_with_properties(
                Vec3::ZERO,
                rotation_a,
                1.0,
                Vec3::new(0.08, -0.03, 0.04),
                kapitsa_math::Mat3::from_diagonal(Vec3::new(0.5, 1.2, 1.5)),
                Vec3::ZERO,
                Vec3::ZERO,
            ),
            solver_body_with_properties(
                Vec3::new(1.8, -0.7, 0.9),
                rotation_b,
                1_000.0,
                Vec3::new(-0.06, 0.04, 0.02),
                kapitsa_math::Mat3::from_diagonal(Vec3::new(400.0, 700.0, 900.0)),
                Vec3::ZERO,
                Vec3::ZERO,
            ),
        ];
        let frame_a = JointFrame::new(
            Vec3::new(0.2, -0.1, 0.08),
            Quat::from_axis_angle(Vec3::Y, 0.17).expect("frame rotation"),
        );
        let frame_b = JointFrame::new(
            Vec3::new(-0.15, 0.12, -0.04),
            Quat::from_axis_angle(Vec3::X, -0.28).expect("frame rotation"),
        );
        let constraint = JointConstraint {
            id: test_joint_id(),
            body_a: 0,
            body_b: 1,
            joint: Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
                frame_a, frame_b,
            ))),
        };
        let mut workspace = JointWorkspace::new();
        JointSolver::begin_step(&mut workspace);
        let options = JointSolverOptions::legacy()
            .with_mass_cache(true)
            .with_projection(projection);
        for iteration in 1..=32 {
            JointSolver::solve_position_joint(
                &mut workspace,
                &mut bodies,
                constraint,
                0,
                1.0 / 60.0,
                options,
            )
            .expect("position projection");
            JointSolver::prepare(&mut workspace, &bodies, &[constraint], 1.0 / 60.0)
                .expect("refresh rows");
            let residual = workspace
                .rows()
                .iter()
                .map(|row| row.error.abs())
                .fold(0.0_f64, f64::max);
            if residual < 1.0e-7 {
                return iteration;
            }
        }
        33
    }

    let scalar = iterations_to_tolerance(super::super::JointProjection::ScalarGaussSeidel);
    let block = iterations_to_tolerance(super::super::JointProjection::FixedBlock3);
    assert!(scalar <= 32, "scalar did not converge in 32 sweeps");
    assert!(block < scalar, "block={block} scalar={scalar}");
}

/// Builds an offset-anchor, rotated fixed joint pair with a large mass ratio
/// and unequal principal inertia so Jacobians and inertia tensors vary per body.
fn asymmetric_fixed_constraint() -> (JointConstraint, [SolverBody; 2]) {
    let rotation_a = Quat::from_axis_angle(Vec3::Z, 0.31).expect("rotation");
    let rotation_b = Quat::from_axis_angle(Vec3::new(0.4, -0.7, 0.2), 0.58).expect("rotation");
    let bodies = [
        solver_body_with_properties(
            Vec3::new(0.3, -0.2, 0.1),
            rotation_a,
            1.0,
            Vec3::new(0.07, -0.05, 0.03),
            kapitsa_math::Mat3::from_diagonal(Vec3::new(0.4, 1.1, 1.4)),
            Vec3::new(0.2, -0.1, 0.05),
            Vec3::new(0.03, 0.02, -0.01),
        ),
        solver_body_with_properties(
            Vec3::new(1.6, -0.9, 0.8),
            rotation_b,
            1_000.0,
            Vec3::new(-0.05, 0.03, 0.02),
            kapitsa_math::Mat3::from_diagonal(Vec3::new(450.0, 600.0, 950.0)),
            Vec3::new(-0.1, 0.2, 0.0),
            Vec3::new(-0.02, 0.01, 0.04),
        ),
    ];
    let frame_a = JointFrame::new(
        Vec3::new(0.2, -0.1, 0.08),
        Quat::from_axis_angle(Vec3::Y, 0.17).expect("frame rotation"),
    );
    let frame_b = JointFrame::new(
        Vec3::new(-0.15, 0.12, -0.04),
        Quat::from_axis_angle(Vec3::X, -0.28).expect("frame rotation"),
    );
    let mut common = JointCommon::new(frame_a, frame_b);
    common.compliance = 5.0e-4;
    common.damping = 0.1;
    common.break_force = Some(1.0e9);
    common.break_torque = Some(1.0e9);
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: Joint::Fixed(super::super::FixedJoint::new(common)),
    };
    (constraint, bodies)
}

/// Controlled one-step comparison. External impulse is along the connector
/// axis, so torque is zero and the exact assembly COM speed is 10/N m/s.
#[test]
#[ignore = "baseline A/B diagnostic for pose integration order"]
fn loaded_fixed_joint_prediction_precedes_velocity_projection() {
    let dt = 1.0 / 120.0;
    for count in [2_usize, 10, 50, 100, 1000] {
        for topology in ["chain", "tree"] {
            let mut initial_x = vec![0.0; count];
            for index in 1..count {
                let parent = if topology == "tree" {
                    (index - 1) / 2
                } else {
                    index - 1
                };
                initial_x[index] = initial_x[parent] + 1.0;
            }
            for projection in [
                super::super::JointProjection::ScalarGaussSeidel,
                super::super::JointProjection::FixedBlock3,
            ] {
                let common = JointCommon::new(
                    JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
                    JointFrame::new(-Vec3::X * 0.5, Quat::IDENTITY),
                );
                let joint = Joint::Fixed(super::super::FixedJoint::new(common));
                let mut world = crate::world::PhysicsWorld::new();
                let ids = (0..count)
                    .map(|index| {
                        let body = solver_body(Vec3::X * initial_x[index]);
                        world
                            .add_body(RigidBody::from_motion(*body.body()))
                            .expect("world body")
                    })
                    .collect::<Vec<_>>();
                world
                    .body_mut(ids[0])
                    .expect("root")
                    .apply_force_at_point(Vec3::X * 1200.0, Vec3::ZERO)
                    .expect("force");
                let joints = (1..count)
                    .map(|index| {
                        let parent = if topology == "tree" {
                            (index - 1) / 2
                        } else {
                            index - 1
                        };
                        let id = world
                            .add_joint(ids[parent], ids[index], joint)
                            .expect("world joint");
                        JointConstraint {
                            id,
                            body_a: parent,
                            body_b: index,
                            joint,
                        }
                    })
                    .collect::<Vec<_>>();
                let options = JointSolverOptions::legacy().with_projection(projection);
                let mut outputs = Vec::new();
                for early_pose in [true, false] {
                    let mut bodies = (0..count)
                        .map(|index| {
                            solver_body_with_velocities(
                                Vec3::X
                                    * (initial_x[index]
                                        + if early_pose && index == 0 {
                                            10.0 * dt
                                        } else {
                                            0.0
                                        }),
                                Quat::IDENTITY,
                                if index == 0 {
                                    Vec3::X * 10.0
                                } else {
                                    Vec3::ZERO
                                },
                                Vec3::ZERO,
                            )
                        })
                        .collect::<Vec<_>>();
                    let mut workspace = JointWorkspace::new();
                    JointSolver::begin_step(&mut workspace);
                    JointSolver::prepare(&mut workspace, &bodies, &joints, dt)
                        .expect("prepare velocity");
                    for _ in 0..8 {
                        JointSolver::solve_velocity_prepared_iteration(
                            &mut workspace,
                            &mut bodies[..],
                            &joints,
                            dt,
                            options,
                        )
                        .expect("velocity sweep");
                    }
                    if !early_pose {
                        for body in &mut bodies {
                            let state = body.state();
                            let transform = state.transform();
                            let next = Transform::new(
                                transform.translation() + state.linear_velocity() * dt,
                                transform.rotation(),
                            )
                            .expect("transform");
                            body.body_mut().set_transform(next).expect("integrate pose");
                        }
                    }
                    let max_anchor = |bodies: &[SolverBody]| {
                        joints
                            .iter()
                            .map(|joint| {
                                (bodies[joint.body_b].state().transform().translation()
                                    - bodies[joint.body_a].state().transform().translation()
                                    - Vec3::X)
                                    .length()
                            })
                            .fold(0.0_f64, f64::max)
                    };
                    let pre_position = max_anchor(&bodies);
                    for _ in 0..3 {
                        for (index, constraint) in joints.iter().copied().enumerate() {
                            JointSolver::solve_position_joint(
                                &mut workspace,
                                &mut bodies[..],
                                constraint,
                                index,
                                dt,
                                options,
                            )
                            .expect("position sweep");
                        }
                    }
                    let post_position = max_anchor(&bodies);
                    let momentum = bodies
                        .iter()
                        .map(|body| body.state().linear_velocity().x)
                        .sum::<f64>();
                    let kinetic = bodies
                        .iter()
                        .map(|body| 0.5 * body.state().linear_velocity().length_squared())
                        .sum::<f64>();
                    let velocity_residual = joints
                        .iter()
                        .map(|joint| {
                            (bodies[joint.body_b].state().linear_velocity()
                                - bodies[joint.body_a].state().linear_velocity())
                            .length()
                        })
                        .fold(0.0_f64, f64::max);
                    let rms_anchor = (joints
                        .iter()
                        .map(|joint| {
                            (bodies[joint.body_b].state().transform().translation()
                                - bodies[joint.body_a].state().transform().translation()
                                - Vec3::X)
                                .length_squared()
                        })
                        .sum::<f64>()
                        / joints.len() as f64)
                        .sqrt();
                    println!(
                        "topology={topology} count={count} projection={projection:?} early_pose={early_pose} pre_position={pre_position:.9e} post_max/rms={post_position:.9e}/{rms_anchor:.9e} velocity_residual={velocity_residual:.9e} momentum={momentum:.9e} com_accel={:.9e} kinetic={kinetic:.9e}",
                        momentum / count as f64 / dt
                    );
                    assert!((momentum - 10.0).abs() < 1.0e-10);
                    outputs.push((pre_position, post_position));
                }
                if projection == super::super::JointProjection::ScalarGaussSeidel {
                    world.step(dt).expect("world step");
                    let positions = ids
                        .iter()
                        .map(|id| {
                            world
                                .body(*id)
                                .expect("body")
                                .state()
                                .transform()
                                .translation()
                        })
                        .collect::<Vec<_>>();
                    let world_error = joints
                        .iter()
                        .map(|joint| {
                            (positions[joint.body_b] - positions[joint.body_a] - Vec3::X).length()
                        })
                        .fold(0.0_f64, f64::max);
                    assert!(
                        (world_error - outputs[0].1).abs() < 1.0e-10,
                        "world={world_error} test={}",
                        outputs[0].1
                    );
                }
                assert!(outputs[0].0 > 0.08);
                assert!(outputs[1].0 < 0.003);
                if count == 2 {
                    assert!(outputs[1].1 < 1.0e-12);
                }
            }
        }
    }
}

/// A/B on repeated, purely translational steps; no contacts or CCD.
#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "compare loaded graph integration order over 180 steps"]
fn diagnose_loaded_graph_integration_order_trajectory() {
    let dt = 1.0 / 120.0;
    for count in [2_usize, 10, 50, 100, 1000] {
        for topology in ["chain", "tree"] {
            let mut x = vec![0.0; count];
            for index in 1..count {
                let parent = if topology == "tree" {
                    (index - 1) / 2
                } else {
                    index - 1
                };
                x[index] = x[parent] + 1.0;
            }
            let common = JointCommon::new(
                JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
                JointFrame::new(-Vec3::X * 0.5, Quat::IDENTITY),
            );
            let joint = Joint::Fixed(super::super::FixedJoint::new(common));
            let mut world = crate::world::PhysicsWorld::new();
            let ids = x
                .iter()
                .map(|position| {
                    world
                        .add_body(RigidBody::from_motion(
                            *solver_body(Vec3::X * *position).body(),
                        ))
                        .expect("body")
                })
                .collect::<Vec<_>>();
            let joints = (1..count)
                .map(|index| {
                    let parent = if topology == "tree" {
                        (index - 1) / 2
                    } else {
                        index - 1
                    };
                    let id = world
                        .add_joint(ids[parent], ids[index], joint)
                        .expect("joint");
                    JointConstraint {
                        id,
                        body_a: parent,
                        body_b: index,
                        joint,
                    }
                })
                .collect::<Vec<_>>();
            for (projection, pcg) in [
                (super::super::JointProjection::ScalarGaussSeidel, false),
                (super::super::JointProjection::FixedBlock3, false),
                (super::super::JointProjection::ScalarGaussSeidel, true),
            ] {
                for early_pose in [true, false] {
                    let started = std::time::Instant::now();
                    let options = JointSolverOptions::legacy().with_projection(projection);
                    let mut bodies = x
                        .iter()
                        .map(|position| solver_body(Vec3::X * *position))
                        .collect::<Vec<_>>();
                    let mut workspace = JointWorkspace::new();
                    let mut max_anchor = 0.0_f64;
                    let mut anchor_square_sum = 0.0;
                    let mut max_velocity = 0.0_f64;
                    let mut first_limit = None;
                    let mut pcg_iterations = 0;
                    let mut pcg_limits = 0;
                    for tick in 0..180 {
                        let velocity = bodies[0].state().linear_velocity() + Vec3::X * 10.0;
                        bodies[0]
                            .body_mut()
                            .set_linear_velocity(velocity)
                            .expect("impulse");
                        let integrate = |bodies: &mut [SolverBody]| {
                            for body in bodies {
                                let state = body.state();
                                let transform = state.transform();
                                body.body_mut()
                                    .set_transform(
                                        Transform::new(
                                            transform.translation() + state.linear_velocity() * dt,
                                            transform.rotation(),
                                        )
                                        .expect("transform"),
                                    )
                                    .expect("pose");
                            }
                        };
                        if early_pose {
                            integrate(&mut bodies);
                        }
                        JointSolver::begin_step(&mut workspace);
                        JointSolver::prepare(&mut workspace, &bodies, &joints, dt).expect("rows");
                        for _ in 0..8 {
                            if pcg {
                                let result = JointSolver::solve_velocity_pcg(
                                    &mut workspace,
                                    &mut bodies,
                                    &joints,
                                    dt,
                                    64,
                                    true,
                                    false,
                                    &crate::execution::SequentialExecutor,
                                    1,
                                )
                                .expect("pcg")
                                .expect("eligible bilateral constraints");
                                pcg_iterations += result.iterations;
                                pcg_limits += usize::from(!result.converged);
                            } else {
                                JointSolver::solve_velocity_prepared_iteration(
                                    &mut workspace,
                                    &mut bodies[..],
                                    &joints,
                                    dt,
                                    options,
                                )
                                .expect("velocity");
                            }
                        }
                        if !early_pose {
                            integrate(&mut bodies);
                        }
                        for _ in 0..3 {
                            for (index, joint) in joints.iter().copied().enumerate() {
                                JointSolver::solve_position_joint(
                                    &mut workspace,
                                    &mut bodies[..],
                                    joint,
                                    index,
                                    dt,
                                    options,
                                )
                                .expect("position");
                            }
                        }
                        let mut tick_max = 0.0_f64;
                        for joint in &joints {
                            let a = bodies[joint.body_a].state();
                            let b = bodies[joint.body_b].state();
                            let error = (b.transform().translation()
                                - a.transform().translation()
                                - Vec3::X)
                                .length();
                            tick_max = tick_max.max(error);
                            anchor_square_sum += error * error;
                            max_velocity = max_velocity
                                .max((b.linear_velocity() - a.linear_velocity()).length());
                        }
                        max_anchor = max_anchor.max(tick_max);
                        if first_limit.is_none() && tick_max >= 0.03 {
                            first_limit = Some(tick);
                        }
                    }
                    let momentum = bodies
                        .iter()
                        .map(|body| body.state().linear_velocity().x)
                        .sum::<f64>();
                    let kinetic = bodies
                        .iter()
                        .map(|body| 0.5 * body.state().linear_velocity().length_squared())
                        .sum::<f64>();
                    let rms = (anchor_square_sum / ((count - 1) * 180) as f64).sqrt();
                    println!(
                        "topology={topology} count={count} projection={projection:?} pcg={pcg} early_pose={early_pose} anchor_max/rms={max_anchor:.9e}/{rms:.9e} frame=0 velocity_residual_max={max_velocity:.9e} momentum={momentum:.9e} kinetic={kinetic:.9e} com_accel={:.9e} first_limit={first_limit:?} pcg_iterations={pcg_iterations} pcg_limits={pcg_limits} elapsed_ms={}",
                        momentum / count as f64 / (180.0 * dt),
                        started.elapsed().as_millis()
                    );
                    assert!((momentum - 1800.0).abs() < 1.0e-7);
                    if early_pose
                        && !pcg
                        && projection == super::super::JointProjection::ScalarGaussSeidel
                    {
                        for _ in 0..180 {
                            world
                                .body_mut(ids[0])
                                .expect("root")
                                .apply_force_at_point(Vec3::X * 1200.0, Vec3::ZERO)
                                .expect("force");
                            world.step(dt).expect("world step");
                        }
                        for (id, body) in ids.iter().zip(&bodies) {
                            let state = world.body(*id).expect("body").state();
                            assert!(
                                (state.transform().translation()
                                    - body.state().transform().translation())
                                .length()
                                    < 1.0e-8
                            );
                            assert!(
                                (state.linear_velocity() - body.state().linear_velocity()).length()
                                    < 1.0e-8
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
#[ignore = "known loaded-chain drift before pose integration is corrected"]
fn loaded_fixed_joint_chain_stays_close_after_root_force() {
    let mut world = crate::world::PhysicsWorld::new();
    let ids = (0..10)
        .map(|index| {
            world
                .add_body(RigidBody::from_motion(
                    *solver_body(Vec3::X * index as f64).body(),
                ))
                .expect("body")
        })
        .collect::<Vec<_>>();
    let common = JointCommon::new(
        JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
        JointFrame::new(-Vec3::X * 0.5, Quat::IDENTITY),
    );
    for pair in ids.windows(2) {
        world
            .add_joint(
                pair[0],
                pair[1],
                Joint::Fixed(super::super::FixedJoint::new(common)),
            )
            .expect("joint");
    }
    world
        .body_mut(ids[0])
        .expect("root")
        .apply_force_at_point(Vec3::X * 1200.0, Vec3::ZERO)
        .expect("force");
    world.step(1.0 / 120.0).expect("step");
    let positions = ids
        .iter()
        .map(|id| {
            world
                .body(*id)
                .expect("body")
                .state()
                .transform()
                .translation()
        })
        .collect::<Vec<_>>();
    let max_error = positions
        .windows(2)
        .map(|pair| (pair[1] - pair[0] - Vec3::X).length())
        .fold(0.0_f64, f64::max);
    // For one-metre links, limit the drift to 0.3% of link length;
    // the exact hard-constraint value is zero.
    assert!(max_error < 0.003, "max anchor error {max_error:.9e} m");
}

#[test]
fn fixed_center_anchor_one_iteration_has_no_overshoot() {
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    };
    let options =
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3);
    for in_place in [true, false] {
        let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X)];
        let mut workspace = JointWorkspace::new();
        JointSolver::begin_step(&mut workspace);
        if in_place {
            JointSolver::prepare(&mut workspace, &bodies, &[constraint], 1.0 / 60.0)
                .expect("commit initial rows");
        }
        JointSolver::solve_position_joint_impl(
            &mut workspace,
            &mut bodies,
            constraint,
            0,
            1.0 / 60.0,
            options,
            in_place,
        )
        .expect("position projection");
        let error = (bodies[1].state().transform().translation()
            - bodies[0].state().transform().translation())
        .length();
        assert!(error < 1.0e-12, "in_place={in_place}, residual={error}");
    }
}

#[test]
fn compare_fixed_position_orders() {
    let dt = 1.0 / 60.0;
    let options =
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3);
    let identity = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    let make_constraint = |a, b, common| JointConstraint {
        id: test_joint_id(),
        body_a: a,
        body_b: b,
        joint: Joint::Fixed(super::super::FixedJoint::new(common)),
    };
    let (asymmetric, asymmetric_bodies) = asymmetric_fixed_constraint();
    let mut offset = identity;
    offset.frame_a = JointFrame::new(Vec3::new(0.4, 0.2, 0.0), Quat::IDENTITY);
    offset.frame_b = JointFrame::new(Vec3::new(-0.3, 0.1, 0.2), Quat::IDENTITY);
    let rotation = Quat::from_axis_angle(Vec3::Z, 1.2).expect("rotation");
    let big_rotation = Quat::from_axis_angle(Vec3::new(1.0, 0.4, 0.3), 2.6).expect("rotation");
    let cases = [
        (
            "linear",
            vec![solver_body(Vec3::ZERO), solver_body(Vec3::X)],
            vec![make_constraint(0, 1, identity)],
        ),
        (
            "rotation",
            vec![
                solver_body(Vec3::ZERO),
                solver_body_with_rotation(Vec3::ZERO, rotation),
            ],
            vec![make_constraint(0, 1, identity)],
        ),
        (
            "offset",
            vec![
                solver_body(Vec3::ZERO),
                solver_body_with_rotation(Vec3::new(1.0, 0.5, -0.2), rotation),
            ],
            vec![make_constraint(0, 1, offset)],
        ),
        ("asymmetric", asymmetric_bodies.to_vec(), vec![asymmetric]),
        (
            "static",
            vec![
                static_solver_body(Vec3::ZERO, Quat::IDENTITY),
                solver_body_with_rotation(Vec3::new(1.0, 0.5, -0.2), rotation),
            ],
            vec![make_constraint(0, 1, offset)],
        ),
        (
            "large",
            vec![
                asymmetric_bodies[0],
                solver_body_with_properties(
                    Vec3::new(1.6, -0.9, 0.8),
                    big_rotation,
                    1000.0,
                    Vec3::ZERO,
                    kapitsa_math::Mat3::from_diagonal(Vec3::new(450.0, 600.0, 950.0)),
                    Vec3::ZERO,
                    Vec3::ZERO,
                ),
            ],
            vec![make_constraint(0, 1, offset)],
        ),
        (
            "chain",
            vec![
                solver_body(Vec3::ZERO),
                solver_body_with_rotation(Vec3::new(1.0, 0.3, 0.0), rotation),
                solver_body(Vec3::new(2.0, -0.2, 0.4)),
            ],
            vec![make_constraint(0, 1, offset), make_constraint(1, 2, offset)],
        ),
    ];
    for (name, initial, constraints) in cases {
        for order in 0..3 {
            let mut bodies = initial.clone();
            let mut workspace = JointWorkspace::new();
            JointSolver::begin_step(&mut workspace);
            workspace
                .ensure_solver_mass_slots(bodies.len())
                .expect("mass slots");
            for iteration in 1..=8 {
                for (index, &constraint) in constraints.iter().enumerate() {
                    if order == 0 {
                        JointSolver::solve_position_joint_impl(
                            &mut workspace,
                            bodies.as_mut_slice(),
                            constraint,
                            index,
                            dt,
                            options,
                            false,
                        )
                        .expect("production order");
                    } else {
                        workspace.begin_joint_prepare(index, constraint.id);
                        prepare_joint_rows_for_constraint(
                            bodies.as_slice(),
                            &mut workspace,
                            constraint,
                            dt,
                        )
                        .expect("prepare");
                        if order == 2 {
                            JointSolver::position_projection_group(
                                &mut workspace,
                                bodies.as_mut_slice(),
                                dt,
                                options,
                                [0, 1, 2],
                            )
                            .expect("anchor");
                            refresh_fixed_joint_row_block(
                                &mut workspace,
                                bodies.as_slice(),
                                constraint,
                                dt,
                                FixedJointRowBlock::Frame,
                            )
                            .expect("frame refresh");
                        }
                        JointSolver::position_projection_group(
                            &mut workspace,
                            bodies.as_mut_slice(),
                            dt,
                            options,
                            [3, 4, 5],
                        )
                        .expect("frame");
                        refresh_fixed_joint_row_block(
                            &mut workspace,
                            bodies.as_slice(),
                            constraint,
                            dt,
                            FixedJointRowBlock::Anchor,
                        )
                        .expect("anchor refresh");
                        JointSolver::position_projection_group(
                            &mut workspace,
                            bodies.as_mut_slice(),
                            dt,
                            options,
                            [0, 1, 2],
                        )
                        .expect("anchor");
                        workspace.finish_joint_prepare().expect("commit");
                    }
                }
                assert!(
                    bodies.iter().all(|body| {
                        body.state().transform().translation().is_finite()
                            && body.state().transform().rotation().is_normalized()
                    }),
                    "invalid pose in {name} order {order} iteration {iteration}"
                );
                if iteration == 1 || iteration == 8 {
                    let mut linear = 0.0_f64;
                    let mut angular = 0.0_f64;
                    for &constraint in &constraints {
                        let mut measure = JointWorkspace::new();
                        JointSolver::prepare(&mut measure, bodies.as_slice(), &[constraint], dt)
                            .expect("measure");
                        let rows = measure.rows();
                        linear = linear.max(
                            rows[0..3]
                                .iter()
                                .map(|r| r.error * r.error)
                                .sum::<f64>()
                                .sqrt(),
                        );
                        angular = angular.max(
                            rows[3..6]
                                .iter()
                                .map(|r| r.error * r.error)
                                .sum::<f64>()
                                .sqrt(),
                        );
                    }
                    assert!(
                        linear.is_finite()
                            && angular.is_finite()
                            && linear < 100.0
                            && angular < 10.0,
                        "unstable {name} order {order}"
                    );
                    println!(
                        "{name} order={order} iteration={iteration} linear={linear:.9e} angular={angular:.9e}"
                    );
                }
            }
        }
    }
}

#[test]
fn fixed_specialized_mass_matches_generic_operator_and_projection() {
    let dt = 1.0 / 60.0;
    let (constraint, asymmetric) = asymmetric_fixed_constraint();
    let cases = [
        [solver_body(Vec3::ZERO), solver_body(Vec3::X)],
        [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            asymmetric[1],
        ],
        asymmetric,
        [
            asymmetric[0],
            solver_body_with_properties(
                Vec3::X,
                Quat::from_axis_angle(Vec3::Y, 2.5).expect("rotation"),
                1.0e-6,
                Vec3::ZERO,
                kapitsa_math::Mat3::from_diagonal(Vec3::new(1.0e-6, 2.0e-6, 3.0e-6)),
                Vec3::ZERO,
                Vec3::ZERO,
            ),
        ],
    ];
    for initial in cases {
        let mut workspace = JointWorkspace::new();
        JointSolver::prepare(&mut workspace, &initial, &[constraint], dt).expect("rows");
        let masses = initial.map(|body| SolverMass::from_body(body.body()).expect("mass"));
        for start in [0, 3] {
            let rows = [
                workspace.rows()[start],
                workspace.rows()[start + 1],
                workspace.rows()[start + 2],
            ];
            let reference =
                super::super::block::prepare_block3(&rows, masses, dt).expect("generic block");
            let specialized =
                super::super::block::prepare_fixed_block3(&rows, masses, dt, start == 0)
                    .expect("fixed block");
            for i in 0..3 {
                let expected = reference.inverse_effective_mass.row(i).expect("row");
                let actual = specialized.inverse_effective_mass.row(i).expect("row");
                for (a, b) in [
                    (actual.x, expected.x),
                    (actual.y, expected.y),
                    (actual.z, expected.z),
                ] {
                    assert!((a - b).abs() <= 1.0e-12 + 1.0e-12 * b.abs(), "{a} vs {b}");
                }
            }
            assert!(specialized.inverse_effective_mass.factor_spd().is_ok());
            let expected_delta = reference.solve_position(dt).expect("generic solve");
            let actual_delta = specialized.solve_position(dt).expect("fixed solve");
            assert!(
                (actual_delta - expected_delta).length()
                    <= 1.0e-11 * (1.0 + expected_delta.length())
            );
            let mut expected_bodies = initial;
            let mut actual_bodies = initial;
            let mut expected_rows = rows;
            let mut actual_rows = rows;
            apply_position_block(
                &mut expected_rows,
                [0, 1, 2],
                expected_delta,
                &mut expected_bodies,
                Some(masses),
            )
            .expect("generic apply");
            apply_position_block(
                &mut actual_rows,
                [0, 1, 2],
                actual_delta,
                &mut actual_bodies,
                Some(masses),
            )
            .expect("fixed apply");
            for (actual, expected) in actual_bodies.iter().zip(expected_bodies) {
                let a = actual.state().transform();
                let b = expected.state().transform();
                assert!((a.translation() - b.translation()).length() < 1.0e-10);
                for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                    assert!(
                        (a.rotation().rotate_vector(axis).expect("rotation")
                            - b.rotation().rotate_vector(axis).expect("rotation"))
                        .length()
                            < 1.0e-10
                    );
                }
            }
        }
    }
    let static_bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        static_solver_body(Vec3::X, Quat::IDENTITY),
    ];
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &static_bodies, &[constraint], dt).expect("static rows");
    let rows = [
        workspace.rows()[0],
        workspace.rows()[1],
        workspace.rows()[2],
    ];
    let singular =
        super::super::block::prepare_fixed_block3(&rows, [SolverMass::STATIC; 2], dt, true)
            .expect("singular block");
    assert!(singular.inverse_effective_mass.factor_spd().is_err());
}

#[test]
fn fixed_in_place_respects_committed_joint_identity() {
    let dt = 1.0 / 60.0;
    let (old, initial) = asymmetric_fixed_constraint();
    let options =
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3);
    for replace in [true, false] {
        let mut constraint = old;
        if replace {
            constraint.id = test_joint_id();
            assert_ne!(constraint.id, old.id);
        }
        let mut actual = JointWorkspace::new();
        let mut generic = JointWorkspace::new();
        for workspace in [&mut actual, &mut generic] {
            JointSolver::begin_step(workspace);
            JointSolver::prepare(workspace, &initial, &[old], dt).expect("commit old joint");
            for row in workspace.rows_mut() {
                row.accumulated_multiplier = 0.01 * (f64::from(row.key) + 1.0);
                row.accumulated_impulse = 2.0 + f64::from(row.key);
            }
        }
        let mut actual_bodies = initial;
        let mut generic_bodies = initial;
        JointSolver::solve_position_joint_impl(
            &mut actual,
            &mut actual_bodies,
            constraint,
            0,
            dt,
            options,
            true,
        )
        .expect("in-place or fallback");
        JointSolver::solve_position_joint_impl(
            &mut generic,
            &mut generic_bodies,
            constraint,
            0,
            dt,
            options,
            false,
        )
        .expect("generic");
        assert_eq!(actual.rows(), generic.rows(), "replace={replace}");
        for (a, b) in actual_bodies.iter().zip(generic_bodies) {
            assert_eq!(a.state().transform(), b.state().transform());
            assert!(a.state().transform().translation().is_finite());
            assert!(a.state().transform().rotation().is_normalized());
        }
        for row in actual.rows() {
            let impulse = if replace {
                0.0
            } else {
                2.0 + f64::from(row.key)
            };
            assert_eq!(row.accumulated_impulse, impulse);
        }
    }
}

#[test]
fn specialized_fixed_geometry_matches_full_rows() {
    let (constraint, initial) = asymmetric_fixed_constraint();
    let Joint::Fixed(joint) = constraint.joint else {
        panic!("fixed joint")
    };
    let settings = row_settings(0, 1, joint.common, 1.0 / 60.0);
    let frames = [joint.common.frame_a, joint.common.frame_b];
    for static_a in [false, true] {
        for large_rotation in [false, true] {
            let mut bodies = initial;
            if static_a {
                bodies[0] = static_solver_body(
                    Vec3::new(0.3, -0.2, 0.1),
                    Quat::from_axis_angle(Vec3::Z, 0.31).expect("rotation"),
                );
            }
            if large_rotation {
                bodies[1] = solver_body_with_properties(
                    Vec3::new(1.6, -0.9, 0.8),
                    Quat::from_axis_angle(Vec3::Y, 2.8).expect("rotation"),
                    1000.0,
                    Vec3::new(-0.05, 0.03, 0.02),
                    kapitsa_math::Mat3::from_diagonal(Vec3::new(450.0, 600.0, 950.0)),
                    Vec3::ZERO,
                    Vec3::ZERO,
                );
            }
            let mut workspace = JointWorkspace::new();
            let full = fixed_joint_geometry(&bodies, &mut workspace, 0, 1, frames[0], frames[1])
                .expect("full geometry");
            let anchor = fixed_anchor_geometry(&bodies, settings, frames).expect("anchor geometry");
            let frame = fixed_frame_geometry(&bodies, settings, frames).expect("frame geometry");
            assert_eq!(
                fixed_anchor_rows(&bodies, settings, full.anchor).expect("full anchor rows"),
                fixed_anchor_rows(&bodies, settings, anchor).expect("anchor rows")
            );
            assert_eq!(
                fixed_frame_rows(settings, full.frame).expect("full frame rows"),
                fixed_frame_rows(settings, frame).expect("frame rows")
            );
        }
    }
}

#[test]
fn in_place_fixed_position_projection_matches_generic_path() {
    let dt = 1.0 / 60.0;
    let (constraint, initial_bodies) = asymmetric_fixed_constraint();
    let options =
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3);
    let mut generic_bodies = initial_bodies;
    let mut in_place_bodies = initial_bodies;
    let mut generic = JointWorkspace::new();
    let mut in_place = JointWorkspace::new();
    JointSolver::begin_step(&mut generic);
    JointSolver::begin_step(&mut in_place);

    for sweep in 0..4 {
        JointSolver::solve_position_joint_impl(
            &mut generic,
            &mut generic_bodies,
            constraint,
            0,
            dt,
            options,
            false,
        )
        .expect("generic position projection");
        // The first sweep has no committed rows yet, so the in-place path
        // falls back to the generic builder exactly once per step.
        JointSolver::solve_position_joint_impl(
            &mut in_place,
            &mut in_place_bodies,
            constraint,
            0,
            dt,
            options,
            sweep > 0,
        )
        .expect("in-place position projection");

        assert_eq!(generic.rows().len(), 6);
        assert_eq!(in_place.rows().len(), 6);
        for (expected, actual) in generic.rows().iter().zip(in_place.rows()) {
            assert_eq!(expected, actual, "row state diverged on sweep {sweep}");
        }
        for (expected, actual) in generic_bodies.iter().zip(&in_place_bodies) {
            let expected_transform = expected.state().transform();
            let actual_transform = actual.state().transform();
            let translation_error =
                (expected_transform.translation() - actual_transform.translation()).length();
            assert!(translation_error <= 1.0e-14);
            let expected_axis = expected_transform
                .rotation()
                .rotate_vector(Vec3::X)
                .expect("finite rotation");
            let actual_axis = actual_transform
                .rotation()
                .rotate_vector(Vec3::X)
                .expect("finite rotation");
            assert!((expected_axis - actual_axis).length() <= 1.0e-14);
        }
    }
}

#[cfg(feature = "bench-internals")]
#[test]
fn fixed_in_place_mass_reuse_matches_generic_for_static_and_dynamic_bodies() {
    let dt = 1.0 / 60.0;
    for mass_cache in [false, true] {
        for static_a in [false, true] {
            for compliant in [false, true] {
                let (mut constraint, mut bodies) = asymmetric_fixed_constraint();
                if static_a {
                    bodies[0] = static_solver_body(
                        Vec3::new(0.3, -0.2, 0.1),
                        Quat::from_axis_angle(Vec3::Z, 0.31).expect("rotation"),
                    );
                }
                if let Joint::Fixed(ref mut joint) = constraint.joint
                    && !compliant
                {
                    joint.common.compliance = 0.0;
                    joint.common.damping = 0.0;
                }
                let options = JointSolverOptions::legacy()
                    .with_mass_cache(mass_cache)
                    .with_projection(super::super::JointProjection::FixedBlock3);
                let mut generic_bodies = bodies;
                let mut fast_bodies = bodies;
                let mut generic = JointWorkspace::new();
                let mut fast = JointWorkspace::new();
                JointSolver::begin_step(&mut generic);
                JointSolver::begin_step(&mut fast);
                for sweep in 0..4 {
                    JointSolver::solve_position_joint_impl(
                        &mut generic,
                        &mut generic_bodies,
                        constraint,
                        0,
                        dt,
                        options,
                        false,
                    )
                    .expect("generic");
                    JointSolver::solve_position_joint_impl(
                        &mut fast,
                        &mut fast_bodies,
                        constraint,
                        0,
                        dt,
                        options,
                        sweep > 0,
                    )
                    .expect("in-place");
                    for (a, b) in generic.rows().iter().zip(fast.rows()) {
                        for (x, y) in [
                            (a.linear_a, b.linear_a),
                            (a.angular_a, b.angular_a),
                            (a.linear_b, b.linear_b),
                            (a.angular_b, b.angular_b),
                        ] {
                            assert!((x - y).length() <= 1.0e-12);
                        }
                        assert!((a.error - b.error).abs() <= 1.0e-12);
                        assert!(
                            (a.accumulated_multiplier - b.accumulated_multiplier).abs() <= 1.0e-12
                        );
                        assert!((a.accumulated_impulse - b.accumulated_impulse).abs() <= 1.0e-12);
                    }
                    for (a, b) in generic_bodies.iter().zip(fast_bodies.iter()) {
                        let a = a.state().transform();
                        let b = b.state().transform();
                        assert!((a.translation() - b.translation()).length() <= 1.0e-12);
                        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                            assert!(
                                (a.rotation().rotate_vector(axis).expect("rotation")
                                    - b.rotation().rotate_vector(axis).expect("rotation"))
                                .length()
                                    <= 1.0e-12
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn in_place_fixed_position_refreshes_jacobians_after_pose_change() {
    let dt = 1.0 / 60.0;
    let (constraint, mut bodies) = asymmetric_fixed_constraint();
    let options =
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3);
    let mut workspace = JointWorkspace::new();
    JointSolver::begin_step(&mut workspace);
    JointSolver::solve_position_joint(&mut workspace, &mut bodies, constraint, 0, dt, options)
        .expect("initial position projection");
    let before = workspace.rows().to_vec();
    assert_eq!(before.len(), 6);

    // Move both props to a clearly different pose before the next sweep.
    bodies[0] = solver_body_with_properties(
        Vec3::new(-0.9, 0.7, -0.4),
        Quat::from_axis_angle(Vec3::new(0.3, 0.9, -0.5), -0.42).expect("rotation"),
        1.0,
        Vec3::new(0.07, -0.05, 0.03),
        kapitsa_math::Mat3::from_diagonal(Vec3::new(0.4, 1.1, 1.4)),
        Vec3::ZERO,
        Vec3::ZERO,
    );
    bodies[1] = solver_body_with_properties(
        Vec3::new(2.4, 0.5, -1.1),
        Quat::from_axis_angle(Vec3::new(-0.6, 0.2, 0.8), 0.71).expect("rotation"),
        1_000.0,
        Vec3::new(-0.05, 0.03, 0.02),
        kapitsa_math::Mat3::from_diagonal(Vec3::new(450.0, 600.0, 950.0)),
        Vec3::ZERO,
        Vec3::ZERO,
    );

    JointSolver::solve_position_joint(&mut workspace, &mut bodies, constraint, 0, dt, options)
        .expect("in-place position projection after pose change");

    let after = workspace.rows();
    assert_eq!(after.len(), 6);
    assert!(after.iter().all(|row| {
        row.error.is_finite()
            && row.linear_a.is_finite()
            && row.angular_a.is_finite()
            && row.angular_b.is_finite()
    }));
    let jacobians_refreshed = before.iter().zip(after).any(|(old, new)| {
        old.angular_a != new.angular_a || old.angular_b != new.angular_b || old.error != new.error
    });
    assert!(jacobians_refreshed, "pose change left stale joint rows");
}

#[test]
fn write_fixed_rows_updates_jacobians_and_preserves_accumulated_state() {
    let make_row = |key: u8, linear_a: Vec3, error: f64| ConstraintRow {
        key,
        dimension: crate::joint::row::RowDimension::Linear,
        body_a: 0,
        body_b: 1,
        linear_a,
        angular_a: Vec3::ZERO,
        position_angular_a: Vec3::ZERO,
        linear_b: -linear_a,
        angular_b: Vec3::ZERO,
        error,
        target_velocity: 0.0,
        force_bias: 0.0,
        force_driven: false,
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
    };
    let mut rows = [
        make_row(0, Vec3::X, 1.0),
        make_row(1, Vec3::Y, 2.0),
        make_row(2, Vec3::Z, 3.0),
    ];
    for (index, row) in rows.iter_mut().enumerate() {
        row.accumulated_multiplier = 0.25 * (index as f64 + 1.0);
        row.accumulated_impulse = -0.5 * (index as f64 + 1.0);
    }
    let replacements = [
        make_row(0, Vec3::Z, 4.0),
        make_row(1, Vec3::X, 5.0),
        make_row(2, Vec3::Y, 6.0),
    ];

    write_fixed_rows(&mut rows, replacements).expect("rewrite block");

    for (index, row) in rows.iter().enumerate() {
        assert_eq!(row.linear_a, replacements[index].linear_a);
        assert_eq!(row.error, replacements[index].error);
        assert_eq!(row.accumulated_multiplier, 0.25 * (index as f64 + 1.0));
        assert_eq!(row.accumulated_impulse, -0.5 * (index as f64 + 1.0));
    }
}

#[test]
fn scalar_fixed_position_fallback_stays_convergent_with_committed_rows() {
    let dt = 1.0 / 60.0;
    let (mut constraint, mut bodies) = asymmetric_fixed_constraint();
    // A hard constraint so the scalar fallback can drive the connector error
    // to zero; compliance would leave a force-proportional residual.
    if let Joint::Fixed(ref mut joint) = constraint.joint {
        joint.common.compliance = 0.0;
        joint.common.damping = 0.0;
    }
    let options = JointSolverOptions::legacy();
    assert_eq!(
        options.projection(),
        super::super::JointProjection::ScalarGaussSeidel
    );
    let mut workspace = JointWorkspace::new();
    JointSolver::begin_step(&mut workspace);
    for _ in 0..32 {
        JointSolver::solve_position_joint(&mut workspace, &mut bodies, constraint, 0, dt, options)
            .expect("scalar position projection");
    }
    assert_eq!(workspace.rows().len(), 6);
    let residual = workspace
        .rows()
        .iter()
        .map(|row| row.error.abs())
        .fold(0.0_f64, f64::max);
    assert!(residual < 1.0e-6, "scalar residual {residual}");
    assert!(bodies.iter().all(|body| {
        body.state().transform().translation().is_finite()
            && body.state().transform().rotation().is_normalized()
    }));
}

#[cfg(feature = "bench-internals")]
#[test]
fn fixed_block_handles_static_dynamic_swapped_order_and_damped_fallback() {
    let rotation = Quat::from_axis_angle(Vec3::Y, 0.37).expect("rotation");
    for swapped in [false, true] {
        let bodies = [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            solver_body_with_properties(
                Vec3::new(1.4, -0.8, 0.6),
                rotation,
                2.0,
                Vec3::new(0.1, -0.04, 0.03),
                kapitsa_math::Mat3::from_diagonal(Vec3::new(0.4, 1.1, 1.4)),
                Vec3::new(0.6, -0.3, 0.2),
                Vec3::new(-0.2, 0.4, 0.1),
            ),
        ];
        let mut bodies = bodies;
        let static_frame = JointFrame::new(Vec3::new(0.2, -0.1, 0.05), Quat::IDENTITY);
        let dynamic_frame = JointFrame::new(
            Vec3::new(-0.12, 0.08, 0.04),
            Quat::from_axis_angle(Vec3::X, 0.21).expect("frame rotation"),
        );
        let (body_a, body_b, frame_a, frame_b) = if swapped {
            (1, 0, dynamic_frame, static_frame)
        } else {
            (0, 1, static_frame, dynamic_frame)
        };
        let constraint = JointConstraint {
            id: test_joint_id(),
            body_a,
            body_b,
            joint: Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
                frame_a, frame_b,
            ))),
        };
        let mut workspace = JointWorkspace::new();
        JointSolver::begin_step(&mut workspace);
        let options = JointSolverOptions::legacy()
            .with_projection(super::super::JointProjection::FixedBlock3);
        JointSolver::solve_position_joint(
            &mut workspace,
            &mut bodies,
            constraint,
            0,
            1.0 / 60.0,
            options,
        )
        .expect("static-dynamic block projection");
        assert_eq!(workspace.metrics().block_rows_solved, 9);
        assert!(bodies.iter().all(|body| {
            body.state().transform().translation().is_finite()
                && body.state().transform().rotation().is_normalized()
        }));
    }

    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 1.5)];
    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.damping = 0.2;
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: Joint::Fixed(super::super::FixedJoint::new(common)),
    };
    let mut workspace = JointWorkspace::new();
    JointSolver::begin_step(&mut workspace);
    JointSolver::solve_position_joint(
        &mut workspace,
        &mut bodies,
        constraint,
        0,
        1.0 / 60.0,
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3),
    )
    .expect("damped scalar fallback");
    assert_eq!(workspace.metrics().block_rows_solved, 0);
    assert_eq!(workspace.metrics().block_fallback_rows, 6);
}

#[cfg(feature = "bench-internals")]
#[test]
fn fixed_block_velocity_sweep_supports_both_warm_start_modes() {
    for warm_start in [false, true] {
        let mut bodies = [
            solver_body_with_velocities(
                Vec3::ZERO,
                Quat::IDENTITY,
                Vec3::new(0.3, -0.4, 0.1),
                Vec3::new(-0.2, 0.5, 0.4),
            ),
            solver_body_with_velocities(
                Vec3::new(1.2, -0.6, 0.8),
                Quat::from_axis_angle(Vec3::Y, 0.31).expect("rotation"),
                Vec3::new(-0.6, 0.2, 0.3),
                Vec3::new(0.3, -0.1, 0.2),
            ),
        ];
        let constraint = JointConstraint {
            id: test_joint_id(),
            body_a: 0,
            body_b: 1,
            joint: Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
                JointFrame::new(Vec3::new(0.1, -0.05, 0.04), Quat::IDENTITY),
                JointFrame::new(
                    Vec3::new(-0.08, 0.03, 0.06),
                    Quat::from_axis_angle(Vec3::X, 0.18).expect("frame rotation"),
                ),
            ))),
        };
        let mut workspace = JointWorkspace::new();
        JointSolver::begin_step(&mut workspace);
        JointSolver::prepare(&mut workspace, &bodies, &[constraint], 1.0 / 60.0)
            .expect("prepare fixed rows");

        let mut cache = JointImpulseCache::new();
        let options = JointSolverOptions::legacy()
            .with_projection(super::super::JointProjection::FixedBlock3)
            .with_warm_start(if warm_start {
                super::super::WarmStartConfig::enabled()
            } else {
                super::super::WarmStartConfig::disabled()
            });
        if warm_start {
            for row in workspace.rows_mut() {
                row.accumulated_impulse = 0.05;
            }
            cache.begin_stage();
            cache
                .stage_joint(
                    constraint.id,
                    constraint.joint,
                    1.0 / 60.0,
                    workspace.rows(),
                )
                .expect("stage warm-start impulses");
            cache.commit();
        }
        let step = JointSolverStep::new(1.0 / 60.0, options).with_impulse_cache(&cache);
        JointSolver::solve_velocity_joint(&mut workspace, &mut bodies, constraint, 0, step, true)
            .expect("fixed block velocity sweep");

        assert_eq!(workspace.metrics().block_rows_solved, 6);
        assert_eq!(workspace.metrics().block_fallback_rows, 0);
        assert_eq!(workspace.metrics().velocity_warm_starts > 0, warm_start);
        assert!(
            workspace
                .rows()
                .iter()
                .all(|row| row.accumulated_impulse.is_finite())
        );
        assert!(bodies.iter().all(|body| {
            body.state().linear_velocity().is_finite()
                && body.state().angular_velocity().is_finite()
        }));
    }
}

#[cfg(feature = "bench-internals")]
#[test]
fn fixed_block_velocity_operators_are_reused_across_velocity_sweeps() {
    let mut bodies = [
        solver_body_with_velocities(
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::new(0.3, -0.4, 0.1),
            Vec3::new(-0.2, 0.5, 0.4),
        ),
        solver_body_with_velocities(
            Vec3::new(1.2, -0.6, 0.8),
            Quat::from_axis_angle(Vec3::Y, 0.31).expect("rotation"),
            Vec3::new(-0.6, 0.2, 0.3),
            Vec3::new(0.3, -0.1, 0.2),
        ),
    ];
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            JointFrame::new(Vec3::new(0.1, -0.05, 0.04), Quat::IDENTITY),
            JointFrame::new(
                Vec3::new(-0.08, 0.03, 0.06),
                Quat::from_axis_angle(Vec3::X, 0.18).expect("frame rotation"),
            ),
        ))),
    };
    let mut workspace = JointWorkspace::new();
    JointSolver::begin_step(&mut workspace);
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 1.0 / 60.0)
        .expect("prepare fixed rows");

    let cache = JointImpulseCache::new();
    let options =
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3);
    let step = JointSolverStep::new(1.0 / 60.0, options).with_impulse_cache(&cache);
    for _ in 0..4 {
        JointSolver::solve_velocity_joint(&mut workspace, &mut bodies, constraint, 0, step, true)
            .expect("fixed block velocity sweep");
    }

    // Velocity sweeps do not move bodies, so the anchor and frame block
    // operators are each prepared once and then reused across every sweep.
    assert_eq!(workspace.fixed_block_velocity_recomputes(), 2);
    assert!(bodies.iter().all(|body| {
        body.state().linear_velocity().is_finite() && body.state().angular_velocity().is_finite()
    }));
}

#[cfg(feature = "bench-internals")]
#[test]
fn fixed_block_velocity_falls_back_for_limits_motors_damping_and_incomplete_groups() {
    let mut bodies = [
        solver_body_with_velocities(
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::new(0.2, -0.1, 0.3),
            Vec3::new(0.1, 0.2, -0.2),
        ),
        solver_body(Vec3::X),
    ];
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: Joint::Fixed(super::super::FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        ))),
    };
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 1.0 / 60.0)
        .expect("prepare fixed rows");

    for change in 0..3 {
        let mut rows = workspace.rows().to_vec();
        match change {
            0 => rows[0].min_impulse = 0.0,
            1 => rows[0].target_velocity = 0.25,
            _ => rows[0].damping = 0.1,
        }
        let mut slots = vec![SolverMassSlot::default(); bodies.len()];
        let mut cache_stats = SolverMassCacheStats::default();
        let result = solve_velocity_projection_group(
            &mut rows,
            &mut slots,
            &mut bodies,
            1.0 / 60.0,
            JointSolverOptions::legacy(),
            [0, 1, 2],
            &mut cache_stats,
            &mut FixedBlockVelocityOperators::default(),
        )
        .expect("scalar fallback");
        assert_eq!(result.block_rows, 0);
        assert_eq!(result.fallback_rows, 3);
    }

    let mut incomplete = workspace.rows()[..2].to_vec();
    let mut slots = vec![SolverMassSlot::default(); bodies.len()];
    let mut cache_stats = SolverMassCacheStats::default();
    let result = solve_velocity_projection_group(
        &mut incomplete,
        &mut slots,
        &mut bodies,
        1.0 / 60.0,
        JointSolverOptions::legacy(),
        [0, 1, 2],
        &mut cache_stats,
        &mut FixedBlockVelocityOperators::default(),
    )
    .expect("incomplete scalar fallback");
    assert_eq!(result.block_rows, 0);
    assert_eq!(result.fallback_rows, 2);
}

#[test]
fn ball_joint_keeps_anchors_together_and_allows_free_rotation() {
    let rotation = Quat::from_axis_angle(Vec3::Y, 0.7).expect("rotation");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::X * 3.0, rotation),
    ];
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    solve_joint(
        super::super::Joint::Ball(super::super::BallJoint::new(common)),
        &mut bodies,
    );
    let position_delta = bodies[1]
        .state()
        .transform()
        .translation()
        .checked_sub(bodies[0].state().transform().translation())
        .expect("finite delta");
    assert!(position_delta.length() < 1.0e-10);
    assert!(
        relative_rotation_error(
            bodies[0].state().transform().rotation(),
            bodies[1].state().transform().rotation()
        )
        .expect("relative angle")
        .length()
            > 0.6
    );
}

#[test]
fn distance_joint_enforces_range_and_leaves_in_range_distance_unchanged() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let joint = super::super::Joint::Distance(super::super::DistanceJoint::new(
        common,
        super::super::JointLimit::new(1.0, 2.0),
    ));
    let mut outside = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 3.0)];
    solve_joint(joint, &mut outside);
    let distance = (outside[1].state().transform().translation()
        - outside[0].state().transform().translation())
    .length();
    assert!((distance - 2.0).abs() < 1.0e-10, "distance={distance}");

    let mut inside = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 1.5)];
    solve_joint(joint, &mut inside);
    let unchanged = (inside[1].state().transform().translation()
        - inside[0].state().transform().translation())
    .length();
    assert!((unchanged - 1.5).abs() < 1.0e-12);
}

#[test]
fn ball_joint_twist_limit_clamps_relative_rotation() {
    let rotation = Quat::from_axis_angle(Vec3::Z, 1.0).expect("rotation");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::ZERO, rotation),
    ];
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::BallJoint::new(common);
    joint.twist_limit = Some(super::super::JointLimit::new(-0.25, 0.25));
    solve_joint(super::super::Joint::Ball(joint), &mut bodies);
    let error = relative_rotation_error(
        bodies[0].state().transform().rotation(),
        bodies[1].state().transform().rotation(),
    )
    .expect("relative angle");
    assert!((error.z - 0.25).abs() < 1.0e-10, "twist={}", error.z);
}

#[test]
fn ball_joint_swing_limit_uses_the_connector_axes() {
    let swing = Quat::from_axis_angle(Vec3::Y, 0.8).expect("swing rotation");
    let twist = Quat::from_axis_angle(Vec3::Z, 1.0).expect("twist rotation");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::ZERO, swing * twist),
    ];
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::BallJoint::new(common);
    joint.swing_limit = Some(0.2);
    solve_joint(super::super::Joint::Ball(joint), &mut bodies);
    let axis_a = bodies[0]
        .state()
        .transform()
        .rotation()
        .rotate_vector(Vec3::Z)
        .expect("axis A");
    let axis_b = bodies[1]
        .state()
        .transform()
        .rotation()
        .rotate_vector(Vec3::Z)
        .expect("axis B");
    let swing_angle = axis_a.dot(axis_b).clamp(-1.0, 1.0).acos();
    assert!((swing_angle - 0.2).abs() < 1.0e-10, "swing={swing_angle}");
}

#[test]
fn hinge_joint_aligns_axes_and_enforces_twist_limit() {
    let tilt = Quat::from_axis_angle(Vec3::X, 0.4).expect("tilt");
    let twist = Quat::from_axis_angle(Vec3::Z, 0.7).expect("twist");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::new(2.0, 1.0, -1.0), tilt * twist),
    ];
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::HingeJoint::new(common);
    joint.angle_limit = Some(super::super::JointLimit::new(-0.2, 0.2));
    solve_joint(super::super::Joint::Hinge(joint), &mut bodies);

    let position_delta = bodies[1]
        .state()
        .transform()
        .translation()
        .checked_sub(bodies[0].state().transform().translation())
        .expect("finite delta");
    let rotation_error = relative_rotation_error(
        bodies[0].state().transform().rotation(),
        bodies[1].state().transform().rotation(),
    )
    .expect("relative rotation");
    assert!(position_delta.length() < 1.0e-10);
    assert!(rotation_error.x.abs() < 1.0e-10);
    assert!(rotation_error.y.abs() < 1.0e-10);
    assert!((rotation_error.z - 0.2).abs() < 1.0e-10);
}

#[test]
fn slider_joint_locks_sideways_motion_and_rotation() {
    let rotation = Quat::from_axis_angle(Vec3::X, 0.4).expect("rotation");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::new(2.0, 3.0, 4.0), rotation),
    ];
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::SliderJoint::new(common);
    joint.translation_limit = Some(super::super::JointLimit::new(-1.0, 2.0));
    solve_joint(super::super::Joint::Slider(joint), &mut bodies);

    let delta =
        bodies[1].state().transform().translation() - bodies[0].state().transform().translation();
    let rotation_error = relative_rotation_error(
        bodies[0].state().transform().rotation(),
        bodies[1].state().transform().rotation(),
    )
    .expect("relative rotation");
    let axis = bodies[0]
        .state()
        .transform()
        .rotation()
        .rotate_vector(Vec3::Z)
        .expect("slider axis");
    let axial = delta.dot(axis);
    let sideways = delta - axis * axial;
    assert!(sideways.length() < 1.0e-10, "slider sideways={sideways:?}");
    assert!((-1.0 - 1.0e-10..=2.0 + 1.0e-10).contains(&axial));
    assert!(rotation_error.length() < 1.0e-10);
}

#[test]
fn cylindrical_joint_limits_slide_and_twist_independently() {
    let tilt = Quat::from_axis_angle(Vec3::X, 0.5).expect("tilt");
    let twist = Quat::from_axis_angle(Vec3::Z, 0.7).expect("twist");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::new(2.0, 3.0, 4.0), tilt * twist),
    ];
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::CylindricalJoint::new(common);
    joint.translation_limit = Some(super::super::JointLimit::new(-1.0, 2.0));
    joint.angle_limit = Some(super::super::JointLimit::new(-0.25, 0.25));
    solve_joint(super::super::Joint::Cylindrical(joint), &mut bodies);

    let delta =
        bodies[1].state().transform().translation() - bodies[0].state().transform().translation();
    let rotation_error = relative_rotation_error(
        bodies[0].state().transform().rotation(),
        bodies[1].state().transform().rotation(),
    )
    .expect("relative rotation");
    let axis = bodies[0]
        .state()
        .transform()
        .rotation()
        .rotate_vector(Vec3::Z)
        .expect("cylindrical axis");
    let axial = delta.dot(axis);
    let sideways = delta - axis * axial;
    assert!(sideways.length() < 1.0e-10, "sideways={sideways:?}");
    assert!((-1.0 - 1.0e-10..=2.0 + 1.0e-10).contains(&axial));
    assert!(rotation_error.x.abs() < 1.0e-10);
    assert!(rotation_error.y.abs() < 1.0e-10);
    assert!((rotation_error.z - 0.25).abs() < 1.0e-10);
}

/// Runs only the velocity sweep of a joint so angular/linear constraints
/// can be inspected without the position projection perturbing the poses.
fn solve_joint_velocity_only(joint: super::super::Joint, bodies: &mut [SolverBody]) {
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint,
    }];
    let dt = 0.01;
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, bodies, &constraints, dt).expect("prepare rows");
    let options = JointSolverOptions::legacy();
    for _ in 0..8 {
        JointSolver::solve_velocity_prepared_iteration(
            &mut workspace,
            bodies,
            &constraints,
            dt,
            options,
        )
        .expect("velocity sweep");
    }
}

fn aligned_axis_common() -> super::super::JointCommon {
    super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    )
}

#[test]
fn aligned_axis_rows_exist_for_zero_orientation_error() {
    let bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Hinge(super::super::HingeJoint::new(aligned_axis_common())),
    }];
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &bodies, &constraints, 0.01).expect("prepare rows");
    let angular_keys = workspace
        .rows()
        .iter()
        .filter(|row| matches!(row.dimension, RowDimension::Angular))
        .map(|row| row.key)
        .collect::<Vec<_>>();
    assert!(
        angular_keys.contains(&3) && angular_keys.contains(&4),
        "expected tangent rows 3 and 4, got {angular_keys:?}"
    );
}

#[test]
fn hinge_aligned_axis_constrains_forbidden_angular_velocity() {
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(5.0, -3.0, 2.0),
        ),
    ];
    solve_joint_velocity_only(
        super::super::Joint::Hinge(super::super::HingeJoint::new(aligned_axis_common())),
        &mut bodies,
    );
    let angular_velocity = bodies[1].state().angular_velocity();
    assert!(
        angular_velocity.x.abs() <= 1.0e-10,
        "forbidden x velocity {}",
        angular_velocity.x
    );
    assert!(
        angular_velocity.y.abs() <= 1.0e-10,
        "forbidden y velocity {}",
        angular_velocity.y
    );
    assert!(
        (angular_velocity.z - 2.0).abs() <= 1.0e-10,
        "allowed z velocity should remain, got {}",
        angular_velocity.z
    );
}

#[test]
fn cylindrical_aligned_axis_constrains_forbidden_angular_velocity() {
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(-4.0, 6.0, 1.5),
        ),
    ];
    solve_joint_velocity_only(
        super::super::Joint::Cylindrical(
            super::super::CylindricalJoint::new(aligned_axis_common()),
        ),
        &mut bodies,
    );
    let angular_velocity = bodies[1].state().angular_velocity();
    assert!(
        angular_velocity.x.abs() <= 1.0e-10,
        "forbidden x velocity {}",
        angular_velocity.x
    );
    assert!(
        angular_velocity.y.abs() <= 1.0e-10,
        "forbidden y velocity {}",
        angular_velocity.y
    );
    assert!(
        (angular_velocity.z - 1.5).abs() <= 1.0e-10,
        "allowed z velocity should remain, got {}",
        angular_velocity.z
    );
}

#[test]
fn aligned_axis_solve_is_equivalent_under_body_swap() {
    let run = |swap: bool| {
        let (mut bodies, joint) = if swap {
            (
                [
                    solver_body_with_velocities(
                        Vec3::ZERO,
                        Quat::IDENTITY,
                        Vec3::ZERO,
                        Vec3::new(5.0, -3.0, 2.0),
                    ),
                    static_solver_body(Vec3::ZERO, Quat::IDENTITY),
                ],
                super::super::Joint::Hinge(super::super::HingeJoint::new(aligned_axis_common())),
            )
        } else {
            (
                [
                    static_solver_body(Vec3::ZERO, Quat::IDENTITY),
                    solver_body_with_velocities(
                        Vec3::ZERO,
                        Quat::IDENTITY,
                        Vec3::ZERO,
                        Vec3::new(5.0, -3.0, 2.0),
                    ),
                ],
                super::super::Joint::Hinge(super::super::HingeJoint::new(aligned_axis_common())),
            )
        };
        solve_joint_velocity_only(joint, &mut bodies);
        let dynamic = if swap { 0 } else { 1 };
        bodies[dynamic].state().angular_velocity()
    };
    let forward = run(false);
    let swapped = run(true);
    assert!((forward.x - swapped.x).abs() <= 1.0e-10);
    assert!((forward.y - swapped.y).abs() <= 1.0e-10);
    assert!((forward.z - swapped.z).abs() <= 1.0e-10);
}

fn solve_joint_velocity_only_with_dt(
    joint: super::super::Joint,
    bodies: &mut [SolverBody],
    dt: f64,
) {
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint,
    }];
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, bodies, &constraints, dt).expect("prepare rows");
    let options = JointSolverOptions::legacy();
    for _ in 0..8 {
        JointSolver::solve_velocity_prepared_iteration(
            &mut workspace,
            bodies,
            &constraints,
            dt,
            options,
        )
        .expect("velocity sweep");
    }
}

fn distance_joint_with_limit(lower: f64, upper: f64) -> super::super::Joint {
    super::super::Joint::Distance(super::super::DistanceJoint::new(
        aligned_axis_common(),
        super::super::JointLimit::new(lower, upper),
    ))
}

fn distance_between(bodies: &[SolverBody; 2]) -> f64 {
    bodies[1]
        .state()
        .transform()
        .translation()
        .checked_sub(bodies[0].state().transform().translation())
        .expect("finite delta")
        .length()
}

#[test]
fn distance_limit_predicts_boundary_crossing_for_all_timesteps() {
    for dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0] {
        // Upper bound: fast outward motion must be braked before it crosses.
        let mut bodies = [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            solver_body_with_velocities(
                Vec3::new(0.0, 0.0, 1.9),
                Quat::IDENTITY,
                Vec3::new(0.0, 0.0, 100.0),
                Vec3::ZERO,
            ),
        ];
        solve_joint_velocity_only_with_dt(distance_joint_with_limit(1.0, 2.0), &mut bodies, dt);
        let predicted = distance_between(&bodies) + bodies[1].state().linear_velocity().z * dt;
        assert!(
            predicted <= 2.0 + 1.0e-9,
            "dt={dt} predicted upper {predicted}"
        );

        // Lower bound: fast inward motion must be braked before it crosses.
        let mut bodies = [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            solver_body_with_velocities(
                Vec3::new(0.0, 0.0, 1.1),
                Quat::IDENTITY,
                Vec3::new(0.0, 0.0, -100.0),
                Vec3::ZERO,
            ),
        ];
        solve_joint_velocity_only_with_dt(distance_joint_with_limit(1.0, 2.0), &mut bodies, dt);
        let predicted = distance_between(&bodies) + bodies[1].state().linear_velocity().z * dt;
        assert!(
            predicted >= 1.0 - 1.0e-9,
            "dt={dt} predicted lower {predicted}"
        );
    }
}

#[test]
fn distance_limit_at_exact_boundary_is_enforced() {
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::new(0.0, 0.0, 2.0),
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::ZERO,
        ),
    ];
    solve_joint_velocity_only_with_dt(distance_joint_with_limit(1.0, 2.0), &mut bodies, 1.0 / 60.0);
    assert!(
        bodies[1].state().linear_velocity().z <= 1.0e-9,
        "outward velocity at the upper boundary {}",
        bodies[1].state().linear_velocity().z
    );
}

#[test]
fn distance_limit_ignores_motion_inside_the_interval() {
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::new(0.0, 0.0, 1.5),
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::ZERO,
        ),
    ];
    solve_joint_velocity_only_with_dt(distance_joint_with_limit(1.0, 2.0), &mut bodies, 1.0 / 60.0);
    assert!(
        (bodies[1].state().linear_velocity().z + 1.0).abs() <= 1.0e-10,
        "interior motion should not be constrained, got {}",
        bodies[1].state().linear_velocity().z
    );
}

#[test]
fn fixed_length_distance_joint_emits_bilateral_lock_at_exact_length() {
    // `fixed_length` collapses the distance interval to a single point.
    // The intersection of the lower and upper bounds is a bilateral
    // equality, so a row must exist even when the coordinate sits exactly
    // at the bound and the radial velocity is zero.
    let bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::new(0.0, 0.0, 2.0),
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
    ];
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Distance(super::super::DistanceJoint::fixed_length(
            aligned_axis_common(),
            2.0,
        )),
    }];
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &bodies, &constraints, 0.01).expect("prepare rows");
    let row = workspace
        .rows()
        .iter()
        .find(|row| row.key == 0)
        .expect("fixed-length distance must emit a lock row");
    assert_eq!(row.dimension, RowDimension::Linear);
    assert!(row.position_enabled);
    assert_eq!(row.min_multiplier, f64::NEG_INFINITY);
    assert_eq!(row.max_multiplier, f64::INFINITY);
    assert_eq!(
        row.target_velocity, 0.0,
        "the lock must not add a Baumgarte bias"
    );
}

#[test]
fn degenerate_translation_limit_locks_against_spring_for_prepared_rows() {
    // A single prepare builds the rows the velocity-only sweep reuses. A
    // spring force producer on the same degree of freedom must not push
    // through a collapsed (`lower == upper`) limit: the intersection of the
    // two unilateral bounds is a rigid lock and must be present from the
    // first prepared row.
    let common = aligned_axis_common();
    let mut spring = super::super::SpringJoint::new(common, 1.0, 500.0, 0.0);
    spring.distance_limit = Some(super::super::JointLimit::new(2.0, 2.0));
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::new(0.0, 0.0, 2.0),
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
    ];
    solve_joint_velocity_only_with_dt(super::super::Joint::Spring(spring), &mut bodies, 0.01);
    let radial_speed =
        bodies[1].state().linear_velocity().z - bodies[0].state().linear_velocity().z;
    assert!(
        radial_speed.abs() < 1.0e-9,
        "collapsed distance limit must block the spring, got {radial_speed}"
    );
}

#[test]
fn degenerate_angular_limit_locks_against_drive_for_prepared_rows() {
    let common = aligned_axis_common();
    let mut hinge = super::super::HingeJoint::new(common);
    hinge.angle_limit = Some(super::super::JointLimit::new(0.0, 0.0));
    hinge.drive = super::super::JointDrive {
        enabled: true,
        target_position: 0.0,
        target_velocity: 5.0,
        stiffness: 0.0,
        damping: 0.0,
        max_force: 1.0e7,
    };
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    solve_joint_velocity_only_with_dt(super::super::Joint::Hinge(hinge), &mut bodies, 0.01);
    let angular_speed =
        bodies[1].state().angular_velocity().z - bodies[0].state().angular_velocity().z;
    assert!(
        angular_speed.abs() < 1.0e-9,
        "collapsed angle limit must block the drive, got {angular_speed}"
    );
}

#[test]
fn degenerate_ball_twist_limit_emits_bilateral_lock() {
    let mut ball = super::super::BallJoint::new(aligned_axis_common());
    ball.twist_limit = Some(super::super::JointLimit::new(0.0, 0.0));
    let bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint: super::super::Joint::Ball(ball),
    }];
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &bodies, &constraints, 0.01).expect("prepare rows");
    let row = workspace
        .rows()
        .iter()
        .find(|row| row.key == 4)
        .expect("collapsed twist limit must emit a lock row");
    assert_eq!(row.dimension, RowDimension::Angular);
    assert!(row.position_enabled);
    assert_eq!(row.min_multiplier, f64::NEG_INFINITY);
    assert_eq!(row.max_multiplier, f64::INFINITY);
    assert_eq!(row.target_velocity, 0.0);
}

fn ball_twist_joint(lower: f64, upper: f64) -> super::super::Joint {
    let mut ball = super::super::BallJoint::new(aligned_axis_common());
    ball.twist_limit = Some(super::super::JointLimit::new(lower, upper));
    super::super::Joint::Ball(ball)
}

/// Recomputes the twist coordinate of a solved pair the same way the solver
/// does, without relying on internal state.
fn twist_between(bodies: &[SolverBody; 2]) -> f64 {
    let rotation_a = bodies[0].state().transform().rotation();
    let rotation_b = bodies[1].state().transform().rotation();
    let mut relative = (rotation_a.conjugate() * rotation_b)
        .try_normalize()
        .expect("normalized relative rotation");
    if relative.w < 0.0 {
        relative = Quat::new(-relative.x, -relative.y, -relative.z, -relative.w);
    }
    let length = relative.z.hypot(relative.w);
    let angle = 2.0 * (relative.z / length).atan2(relative.w / length);
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

#[test]
fn ball_twist_limit_predicts_outward_boundary_crossing_for_all_timesteps() {
    for dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0] {
        // The twist coordinate sits inside the interval but the relative
        // angular velocity would carry it past the upper bound within one
        // step. The velocity solve must brake it before the crossing.
        let rotation = Quat::from_axis_angle(Vec3::Z, 0.9).expect("twist rotation");
        let mut bodies = [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            solver_body_with_velocities(
                Vec3::ZERO,
                rotation,
                Vec3::ZERO,
                Vec3::new(0.0, 0.0, 100.0),
            ),
        ];
        solve_joint_velocity_only_with_dt(ball_twist_joint(-1.0, 1.0), &mut bodies, dt);
        let relative_speed =
            bodies[1].state().angular_velocity().z - bodies[0].state().angular_velocity().z;
        let predicted = twist_between(&bodies) + relative_speed * dt;
        assert!(
            predicted <= 1.0 + 1.0e-9,
            "dt={dt} predicted twist {predicted} crossed the upper bound"
        );

        // The symmetric lower-bound crossing must also be predicted.
        let rotation = Quat::from_axis_angle(Vec3::Z, -0.9).expect("twist rotation");
        let mut bodies = [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            solver_body_with_velocities(
                Vec3::ZERO,
                rotation,
                Vec3::ZERO,
                Vec3::new(0.0, 0.0, -100.0),
            ),
        ];
        solve_joint_velocity_only_with_dt(ball_twist_joint(-1.0, 1.0), &mut bodies, dt);
        let relative_speed =
            bodies[1].state().angular_velocity().z - bodies[0].state().angular_velocity().z;
        let predicted = twist_between(&bodies) + relative_speed * dt;
        assert!(
            predicted >= -1.0 - 1.0e-9,
            "dt={dt} predicted twist {predicted} crossed the lower bound"
        );
    }
}

#[test]
fn ball_twist_limit_at_exact_boundary_brakes_outward_velocity() {
    let rotation = Quat::from_axis_angle(Vec3::Z, 1.0).expect("twist rotation");
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(Vec3::ZERO, rotation, Vec3::ZERO, Vec3::new(0.0, 0.0, 5.0)),
    ];
    solve_joint_velocity_only_with_dt(ball_twist_joint(-1.0, 1.0), &mut bodies, 1.0 / 60.0);
    let relative_speed =
        bodies[1].state().angular_velocity().z - bodies[0].state().angular_velocity().z;
    assert!(
        relative_speed <= 1.0e-9,
        "outward velocity at the exact twist boundary must be removed, got {relative_speed}"
    );
}

#[test]
fn ball_twist_limit_ignores_inward_motion_inside_the_interval() {
    let rotation = Quat::from_axis_angle(Vec3::Z, 0.5).expect("twist rotation");
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(Vec3::ZERO, rotation, Vec3::ZERO, Vec3::new(0.0, 0.0, -3.0)),
    ];
    solve_joint_velocity_only_with_dt(ball_twist_joint(-1.0, 1.0), &mut bodies, 1.0 / 60.0);
    assert!(
        (bodies[1].state().angular_velocity().z + 3.0).abs() <= 1.0e-10,
        "inward motion inside the interval must be untouched, got {}",
        bodies[1].state().angular_velocity().z
    );
}

#[test]
fn ball_twist_limit_solve_is_equivalent_under_body_swap() {
    let run = |swap: bool| -> f64 {
        let dt = 1.0 / 120.0;
        let rotation = Quat::from_axis_angle(Vec3::Z, 0.9).expect("twist rotation");
        let dynamic = solver_body_with_velocities(
            Vec3::ZERO,
            rotation,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 100.0),
        );
        let fixed = static_solver_body(Vec3::ZERO, Quat::IDENTITY);
        let mut bodies = if swap {
            [dynamic, fixed]
        } else {
            [fixed, dynamic]
        };
        solve_joint_velocity_only_with_dt(ball_twist_joint(-1.0, 1.0), &mut bodies, dt);
        let axis = bodies[0]
            .state()
            .transform()
            .rotation()
            .rotate_vector(Vec3::Z)
            .expect("finite twist axis");
        let relative_speed = bodies[1]
            .state()
            .angular_velocity()
            .checked_sub(bodies[0].state().angular_velocity())
            .expect("finite relative velocity")
            .checked_dot(axis)
            .expect("finite dot");
        twist_between(&bodies) + relative_speed * dt
    };
    for predicted in [run(false), run(true)] {
        assert!(
            (-1.0 - 1.0e-9..=1.0 + 1.0e-9).contains(&predicted),
            "swapped twist predicted {predicted} left the interval"
        );
    }
}

#[test]
fn ball_twist_limit_stays_finite_near_pi() {
    for dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0] {
        let rotation =
            Quat::from_axis_angle(Vec3::Z, std::f64::consts::PI - 1.0e-6).expect("twist");
        let mut bodies = [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            solver_body_with_velocities(
                Vec3::ZERO,
                rotation,
                Vec3::ZERO,
                Vec3::new(0.0, 0.0, 200.0),
            ),
        ];
        solve_joint_velocity_only_with_dt(
            ball_twist_joint(-std::f64::consts::PI, std::f64::consts::PI),
            &mut bodies,
            dt,
        );
        let angular_velocity = bodies[1].state().angular_velocity();
        assert!(
            angular_velocity.is_finite(),
            "dt={dt} near-pi twist produced {angular_velocity:?}"
        );
        assert!(twist_between(&bodies).is_finite());
    }
}

#[test]
fn distance_limit_violation_is_removed_by_position_projection() {
    // A violated limit is corrected by XPBD position projection, not by a
    // Baumgarte velocity bias. A velocity-only sweep must therefore leave a
    // resting body at rest (no artificial energy injection)...
    let mut resting = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::new(0.0, 0.0, 2.5),
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
    ];
    solve_joint_velocity_only_with_dt(
        distance_joint_with_limit(1.0, 2.0),
        &mut resting,
        1.0 / 60.0,
    );
    assert_eq!(
        resting[1].state().linear_velocity(),
        Vec3::ZERO,
        "a resting body must not gain velocity from a position-only violation"
    );

    // ...while the full solve, which runs position projection, removes it.
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::new(0.0, 0.0, 2.5),
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
    ];
    solve_joint(distance_joint_with_limit(1.0, 2.0), &mut bodies);
    let distance = distance_between(&bodies);
    assert!(
        distance <= 2.0 + 1.0e-9,
        "position projection should remove the violation, got {distance}"
    );
}

#[test]
fn distance_limit_predicts_for_dynamic_dynamic_pair() {
    let mut bodies = [
        solver_body_with_velocities(
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, -3.0),
            Vec3::ZERO,
        ),
        solver_body_with_velocities(
            Vec3::new(0.0, 0.0, 1.9),
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, 3.0),
            Vec3::ZERO,
        ),
    ];
    let dt = 1.0 / 60.0;
    solve_joint_velocity_only_with_dt(distance_joint_with_limit(1.0, 2.0), &mut bodies, dt);
    let relative = bodies[1]
        .state()
        .linear_velocity()
        .checked_sub(bodies[0].state().linear_velocity())
        .expect("finite velocity");
    let predicted = distance_between(&bodies) + relative.z * dt;
    assert!(
        predicted <= 2.0 + 1.0e-9,
        "dynamic pair predicted {predicted}"
    );
    // Linear momentum is preserved: the pair has no external forces.
    let momentum = bodies[0].state().linear_velocity() + bodies[1].state().linear_velocity();
    assert!(momentum.length() <= 1.0e-10, "momentum {momentum:?}");
}

#[test]
fn ball_swing_limit_predicts_and_stays_finite_at_degenerate_swing() {
    for dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0] {
        // Near-zero swing with a fast outward swing velocity.
        let mut bodies = [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            solver_body_with_velocities(
                Vec3::ZERO,
                Quat::IDENTITY,
                Vec3::ZERO,
                Vec3::new(50.0, 0.0, 0.0),
            ),
        ];
        let mut joint = super::super::BallJoint::new(aligned_axis_common());
        joint.swing_limit = Some(0.1);
        solve_joint_velocity_only_with_dt(super::super::Joint::Ball(joint), &mut bodies, dt);
        let angular_velocity = bodies[1].state().angular_velocity();
        assert!(
            angular_velocity.is_finite(),
            "angular velocity {angular_velocity:?}"
        );

        // Near-pi swing must not fail on the degenerate cross product.
        let swing =
            Quat::from_axis_angle(Vec3::X, std::f64::consts::PI - 1.0e-9).expect("near-pi swing");
        let mut bodies = [
            static_solver_body(Vec3::ZERO, Quat::IDENTITY),
            solver_body_with_velocities(Vec3::ZERO, swing, Vec3::ZERO, Vec3::ZERO),
        ];
        let mut joint = super::super::BallJoint::new(aligned_axis_common());
        joint.swing_limit = Some(0.1);
        solve_joint_velocity_only_with_dt(super::super::Joint::Ball(joint), &mut bodies, dt);
        assert!(bodies[1].state().angular_velocity().is_finite());
        assert!(bodies[1].state().transform().translation().is_finite());
    }
}

fn assert_vec_close(actual: Vec3, expected: Vec3, tol: f64, context: &str) {
    let diff = actual - expected;
    assert!(
        diff.length() <= tol,
        "{context}: actual {actual:?} expected {expected:?}"
    );
}

fn velocity_only_bodies() -> [SolverBody; 2] {
    [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, 5.0, 6.0),
        ),
    ]
}

fn run_velocity_only(joint: Joint) -> [SolverBody; 2] {
    let mut bodies = velocity_only_bodies();
    solve_joint_velocity_only_with_dt(joint, &mut bodies, 1.0 / 60.0);
    bodies
}

#[test]
fn hinge_locks_translation_and_two_rotations_keeping_twist() {
    let bodies = run_velocity_only(Joint::Hinge(HingeJoint::new(aligned_axis_common())));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::ZERO,
        1.0e-10,
        "hinge translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::new(0.0, 0.0, 6.0),
        1.0e-10,
        "hinge rotation",
    );
}

#[test]
fn slider_locks_two_translations_and_all_rotations_keeping_axis() {
    let bodies = run_velocity_only(Joint::Slider(SliderJoint::new(aligned_axis_common())));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::new(0.0, 0.0, 3.0),
        1.0e-10,
        "slider translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::ZERO,
        1.0e-10,
        "slider rotation",
    );
}

/// Frame-derived constraint axes rotate with body A, so the velocity
/// Jacobian must carry the exact `delta * (omega_a x axis)` term. A
/// rigidly rotating pair that slides along its own axis satisfies every
/// locked coordinate exactly; after a velocity-only solve, a central
/// finite difference of those coordinates must therefore stay at zero.
/// A frozen-axis Jacobian injects impulses proportional to the free-axis
/// separation and fails this check.
#[test]
fn rotating_axis_jacobian_matches_finite_difference() {
    let dt = 1.0 / 60.0;
    let frame_rotation = Quat::from_axis_angle(Vec3::new(0.3, -0.4, 0.5), 0.7).expect("unit axis");
    let separation = 2.0_f64;
    let omega = Vec3::new(0.4, 0.8, -0.3);
    let slide = 1.5;
    let axis = frame_rotation
        .rotate_vector(Vec3::Z)
        .expect("normalized rotation");
    let in_plane = frame_rotation
        .rotate_vector(Vec3::X)
        .expect("normalized rotation");

    // Each case keeps the pair in a consistent rigid rotation plus a slide
    // along a genuinely free translation, so every locked coordinate has a
    // true derivative of zero. `locked` selects the measured coordinates.
    let cases: [(&str, Joint, Vec3, Vec3, [bool; 3]); 5] = [
        (
            "slider",
            Joint::Slider(SliderJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
            axis * separation,
            axis * slide,
            [true, true, false],
        ),
        (
            "cylindrical",
            Joint::Cylindrical(CylindricalJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
            axis * separation,
            axis * slide,
            [true, true, false],
        ),
        (
            "planar",
            Joint::Planar(PlanarJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            ))),
            in_plane * separation,
            in_plane * slide,
            [false, false, true],
        ),
        (
            "hinge",
            Joint::Hinge(HingeJoint::new(JointCommon::new(
                JointFrame::new(Vec3::Z * separation, Quat::IDENTITY),
                JointFrame::new(Vec3::Z * separation, Quat::IDENTITY),
            ))),
            Vec3::ZERO,
            Vec3::ZERO,
            [true, true, true],
        ),
        (
            "d6-locked",
            {
                let mut joint =
                    D6Joint::new(JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY));
                joint.linear_motion = [JointMotion::Locked, JointMotion::Locked, JointMotion::Free];
                joint.angular_motion = [JointMotion::Free, JointMotion::Free, JointMotion::Locked];
                Joint::D6(joint)
            },
            axis * separation,
            axis * slide,
            [true, true, false],
        ),
    ];

    for (name, joint, position_b, slide_velocity, locked) in cases {
        let mut bodies = [
            solver_body_with_velocities(Vec3::ZERO, frame_rotation, Vec3::ZERO, omega),
            solver_body_with_velocities(
                position_b,
                frame_rotation,
                omega.cross(position_b) + slide_velocity,
                omega,
            ),
        ];
        solve_joint_velocity_only_with_dt(joint, &mut bodies, dt);

        // Central difference of every locked linear coordinate under the
        // solved velocity field. The coordinates are recomputed at the
        // advanced poses, so their rotating frames are included.
        let h = 1.0e-6;
        let advance = |bodies: &[SolverBody; 2], sign: f64| {
            let mut advanced = *bodies;
            for body in &mut advanced {
                let state = body.state();
                let transform = state.transform();
                let translation = transform.translation() + state.linear_velocity() * (h * sign);
                let rotation = transform
                    .rotation()
                    .integrate_world_angular_velocity(state.angular_velocity(), h * sign)
                    .expect("finite rotation step");
                *body = solver_body_with_velocities(
                    translation,
                    rotation,
                    state.linear_velocity(),
                    state.angular_velocity(),
                );
            }
            advanced
        };
        let coordinate = |bodies: &[SolverBody; 2]| -> Vec3 {
            let rotation_a = bodies[0].state().transform().rotation();
            let delta = bodies[1].state().transform().translation()
                - bodies[0].state().transform().translation();
            let axis_a = rotation_a
                .rotate_vector(Vec3::Z)
                .expect("normalized rotation");
            let basis = perpendicular_basis(axis_a).expect("axis is normalized");
            Vec3::new(delta.dot(basis[0]), delta.dot(basis[1]), delta.dot(axis_a))
        };
        let forward = coordinate(&advance(&bodies, 1.0));
        let backward = coordinate(&advance(&bodies, -1.0));
        let derivative = (forward - backward) / (2.0 * h);
        let residual = Vec3::new(
            if locked[0] { derivative.x } else { 0.0 },
            if locked[1] { derivative.y } else { 0.0 },
            if locked[2] { derivative.z } else { 0.0 },
        );
        assert!(
            residual.length() <= 1.0e-9,
            "{name}: locked coordinate derivative {residual:?} exceeds 1e-9"
        );
    }
}

#[test]
fn ball_locks_translation_and_keeps_all_rotations() {
    let bodies = run_velocity_only(Joint::Ball(BallJoint::new(aligned_axis_common())));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::ZERO,
        1.0e-10,
        "ball translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::new(4.0, 5.0, 6.0),
        1.0e-10,
        "ball rotation should stay free",
    );
}

#[test]
fn cylindrical_locks_two_translations_and_two_rotations_keeping_axis() {
    let bodies = run_velocity_only(Joint::Cylindrical(CylindricalJoint::new(
        aligned_axis_common(),
    )));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::new(0.0, 0.0, 3.0),
        1.0e-10,
        "cylindrical translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::new(0.0, 0.0, 6.0),
        1.0e-10,
        "cylindrical rotation",
    );
}

#[test]
fn planar_locks_normal_and_two_rotations_keeping_in_plane_motion() {
    let bodies = run_velocity_only(Joint::Planar(PlanarJoint::new(aligned_axis_common())));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::new(1.0, 2.0, 0.0),
        1.0e-10,
        "planar translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::new(0.0, 0.0, 6.0),
        1.0e-10,
        "planar rotation",
    );
}

#[test]
fn cone_twist_without_active_limits_locks_translation_and_keeps_rotation() {
    let joint = ConeTwistJoint::new(
        aligned_axis_common(),
        [std::f64::consts::PI, std::f64::consts::PI],
        JointLimit::new(-std::f64::consts::PI, std::f64::consts::PI),
    );
    let bodies = run_velocity_only(Joint::ConeTwist(joint));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::ZERO,
        1.0e-10,
        "cone twist translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::new(4.0, 5.0, 6.0),
        1.0e-10,
        "cone twist rotation should stay free",
    );
}

#[test]
fn d6_free_axes_are_not_constrained() {
    let mut joint = D6Joint::new(aligned_axis_common());
    joint.linear_motion = [JointMotion::Free; 3];
    joint.angular_motion = [JointMotion::Free; 3];
    let bodies = run_velocity_only(Joint::D6(joint));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::new(1.0, 2.0, 3.0),
        1.0e-10,
        "d6 free translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::new(4.0, 5.0, 6.0),
        1.0e-10,
        "d6 free rotation",
    );
}

#[test]
fn d6_locks_only_declared_axes() {
    let mut joint = D6Joint::new(aligned_axis_common());
    joint.linear_motion = [JointMotion::Free, JointMotion::Free, JointMotion::Locked];
    joint.angular_motion = [JointMotion::Free, JointMotion::Free, JointMotion::Locked];
    let bodies = run_velocity_only(Joint::D6(joint));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::new(1.0, 2.0, 0.0),
        1.0e-10,
        "d6 mixed translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::new(4.0, 5.0, 0.0),
        1.0e-10,
        "d6 mixed rotation",
    );
}

#[test]
fn spring_without_force_or_limits_keeps_all_degrees_of_freedom() {
    let bodies = run_velocity_only(Joint::Spring(SpringJoint::new(
        aligned_axis_common(),
        1.0,
        0.0,
        0.0,
    )));
    assert_vec_close(
        bodies[1].state().linear_velocity(),
        Vec3::new(1.0, 2.0, 3.0),
        1.0e-10,
        "spring free translation",
    );
    assert_vec_close(
        bodies[1].state().angular_velocity(),
        Vec3::new(4.0, 5.0, 6.0),
        1.0e-10,
        "spring free rotation",
    );
}

#[test]
fn non_fixed_joints_remove_forbidden_velocity_with_offset_anchors() {
    // Nonzero local anchors and rotated connector frames must not change
    // which degrees of freedom are locked. Body B is positioned so the
    // world anchors coincide; only the forbidden relative motion remains.
    let frame_a = JointFrame::new(
        Vec3::new(0.1, -0.2, 0.3),
        Quat::from_axis_angle(Vec3::Z, 0.2).expect("frame a"),
    );
    let frame_b = JointFrame::new(
        Vec3::new(-0.4, 0.15, 0.2),
        Quat::from_axis_angle(Vec3::X, -0.3).expect("frame b"),
    );
    let common = JointCommon::new(frame_a, frame_b);
    // Body B keeps an identity world rotation, so the coincident-anchor
    // position is simply the difference of the two local anchors.
    let anchor_a = frame_a.local_anchor;
    let anchor_b_local = frame_b.local_anchor;
    let position_b = anchor_a - anchor_b_local;
    let mut bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_velocities(
            position_b,
            Quat::IDENTITY,
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, 5.0, 6.0),
        ),
    ];
    solve_joint_velocity_only_with_dt(Joint::Ball(BallJoint::new(common)), &mut bodies, 1.0 / 60.0);
    // The ball locks the anchor point: the relative velocity of the two
    // anchor points, including the rotational contribution, must vanish.
    let anchor_a_world = bodies[0].state().transform().translation()
        + bodies[0]
            .state()
            .transform()
            .rotation()
            .rotate_vector(anchor_a)
            .expect("anchor");
    let anchor_b_world = bodies[1].state().transform().translation()
        + bodies[1]
            .state()
            .transform()
            .rotation()
            .rotate_vector(anchor_b_local)
            .expect("anchor");
    let velocity_a = bodies[0].state().linear_velocity()
        + bodies[0]
            .state()
            .angular_velocity()
            .checked_cross(anchor_a_world - bodies[0].state().transform().translation())
            .expect("cross");
    let velocity_b = bodies[1].state().linear_velocity()
        + bodies[1]
            .state()
            .angular_velocity()
            .checked_cross(anchor_b_world - bodies[1].state().transform().translation())
            .expect("cross");
    let relative_anchor_velocity = velocity_b.checked_sub(velocity_a).expect("finite velocity");
    assert!(
        relative_anchor_velocity.length() <= 1.0e-8,
        "ball anchor velocity with offset anchors {relative_anchor_velocity:?}"
    );
}

#[test]
fn non_fixed_joint_solve_is_equivalent_under_body_swap() {
    let run = |swap: bool| -> Vec<Vec3> {
        let (mut bodies, joint): ([SolverBody; 2], Joint) = if swap {
            (
                [
                    solver_body_with_velocities(
                        Vec3::ZERO,
                        Quat::IDENTITY,
                        Vec3::new(1.0, 2.0, 3.0),
                        Vec3::new(4.0, 5.0, 6.0),
                    ),
                    static_solver_body(Vec3::ZERO, Quat::IDENTITY),
                ],
                Joint::Cylindrical(CylindricalJoint::new(aligned_axis_common())),
            )
        } else {
            (
                velocity_only_bodies(),
                Joint::Cylindrical(CylindricalJoint::new(aligned_axis_common())),
            )
        };
        solve_joint_velocity_only_with_dt(joint, &mut bodies, 1.0 / 60.0);
        let dynamic = if swap { 0 } else { 1 };
        vec![
            bodies[dynamic].state().linear_velocity(),
            bodies[dynamic].state().angular_velocity(),
        ]
    };
    let forward = run(false);
    let swapped = run(true);
    assert_vec_close(forward[0], swapped[0], 1.0e-10, "swap linear");
    assert_vec_close(forward[1], swapped[1], 1.0e-10, "swap angular");
}

#[test]
fn joint_drives_respect_force_limit_and_target_velocity() {
    let drive = super::super::JointDrive {
        enabled: true,
        target_position: 0.0,
        target_velocity: 10.0,
        stiffness: 0.0,
        damping: 0.0,
        max_force: 100.0,
    };
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );

    let mut hinge = super::super::HingeJoint::new(common);
    hinge.drive = drive;
    let mut hinge_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    solve_joint(super::super::Joint::Hinge(hinge), &mut hinge_bodies);
    let relative_angular_speed =
        hinge_bodies[1].state().angular_velocity().z - hinge_bodies[0].state().angular_velocity().z;
    assert!((relative_angular_speed - 2.0).abs() < 1.0e-10);

    let mut slider = super::super::SliderJoint::new(common);
    slider.drive = drive;
    let mut slider_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    solve_joint(super::super::Joint::Slider(slider), &mut slider_bodies);
    let relative_linear_speed =
        slider_bodies[1].state().linear_velocity().z - slider_bodies[0].state().linear_velocity().z;
    assert!((relative_linear_speed - 2.0).abs() < 1.0e-10);

    let mut cylindrical = super::super::CylindricalJoint::new(common);
    cylindrical.translation_drive = drive;
    cylindrical.angular_drive = drive;
    let mut cylindrical_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    solve_joint(
        super::super::Joint::Cylindrical(cylindrical),
        &mut cylindrical_bodies,
    );
    let linear_speed = cylindrical_bodies[1].state().linear_velocity().z
        - cylindrical_bodies[0].state().linear_velocity().z;
    let angular_speed = cylindrical_bodies[1].state().angular_velocity().z
        - cylindrical_bodies[0].state().angular_velocity().z;
    assert!((linear_speed - 2.0).abs() < 1.0e-10);
    assert!((angular_speed - 2.0).abs() < 1.0e-10);
}

#[test]
fn position_drives_move_hinge_and_slider_toward_targets() {
    let drive = super::super::JointDrive {
        enabled: true,
        target_position: 0.4,
        target_velocity: 0.0,
        stiffness: 100_000.0,
        damping: 0.0,
        max_force: 1.0e7,
    };
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );

    let mut hinge = super::super::HingeJoint::new(common);
    hinge.drive = drive;
    let mut hinge_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    solve_joint(super::super::Joint::Hinge(hinge), &mut hinge_bodies);
    let hinge_angle = relative_rotation_error(
        hinge_bodies[0].state().transform().rotation(),
        hinge_bodies[1].state().transform().rotation(),
    )
    .expect("hinge angle")
    .z;
    assert!(hinge_angle > 0.3, "hinge angle={hinge_angle}");

    let mut slider = super::super::SliderJoint::new(common);
    slider.drive = drive;
    let mut slider_bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    solve_joint(super::super::Joint::Slider(slider), &mut slider_bodies);
    let slider_position = slider_bodies[1].state().transform().translation().z
        - slider_bodies[0].state().transform().translation().z;
    assert!(slider_position > 0.3, "slider position={slider_position}");
}

#[test]
fn hinge_position_drive_respects_angular_limit() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut hinge = super::super::HingeJoint::new(common);
    hinge.angle_limit = Some(super::super::JointLimit::new(-0.2, 0.2));
    hinge.drive = super::super::JointDrive {
        enabled: true,
        target_position: 1.0,
        target_velocity: 0.0,
        stiffness: 100_000.0,
        damping: 0.0,
        max_force: 1.0e8,
    };
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];

    solve_joint(super::super::Joint::Hinge(hinge), &mut bodies);

    let angle = relative_rotation_error(
        bodies[0].state().transform().rotation(),
        bodies[1].state().transform().rotation(),
    )
    .expect("hinge angle")
    .z;
    assert!(angle <= 0.2 + 1.0e-10, "hinge angle={angle}");
    assert!(angle > 0.1, "drive did not approach its limit: {angle}");
}

#[test]
fn slider_limit_brakes_outward_velocity_before_crossing() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::SliderJoint::new(common);
    joint.translation_limit = Some(super::super::JointLimit::new(-1.0, 1.0));
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, 200.0),
            Vec3::ZERO,
        ),
    ];

    solve_joint(super::super::Joint::Slider(joint), &mut bodies);

    let relative_speed =
        bodies[1].state().linear_velocity().z - bodies[0].state().linear_velocity().z;
    assert!(
        relative_speed <= 100.0 + 1.0e-10,
        "relative speed={relative_speed}"
    );
}

#[test]
fn hinge_limit_brakes_outward_angular_velocity_before_crossing() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::HingeJoint::new(common);
    joint.angle_limit = Some(super::super::JointLimit::new(-1.0, 1.0));
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 200.0),
        ),
    ];

    solve_joint(super::super::Joint::Hinge(joint), &mut bodies);

    let relative_speed =
        bodies[1].state().angular_velocity().z - bodies[0].state().angular_velocity().z;
    assert!(
        relative_speed <= 100.0 + 1.0e-10,
        "relative speed={relative_speed}"
    );
}

#[test]
fn planar_joint_locks_normal_and_swing_but_keeps_in_plane_motion() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::PlanarJoint::new(common);
    joint.translation_limits = [
        Some(super::super::JointLimit::new(-0.5, 0.5)),
        Some(super::super::JointLimit::new(-1.0, 1.0)),
    ];
    let tilt = Quat::from_axis_angle(Vec3::X, 0.4).expect("tilt");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::new(2.0, 3.0, 4.0), tilt),
    ];

    solve_joint(super::super::Joint::Planar(joint), &mut bodies);

    let delta =
        bodies[1].state().transform().translation() - bodies[0].state().transform().translation();
    let frame_rotation = bodies[0].state().transform().rotation();
    let normal = frame_rotation
        .rotate_vector(Vec3::Z)
        .expect("planar normal");
    let x_axis = frame_rotation.rotate_vector(Vec3::X).expect("plane x");
    let y_axis = frame_rotation.rotate_vector(Vec3::Y).expect("plane y");
    assert!(delta.dot(normal).abs() < 1.0e-10);
    assert!(delta.dot(x_axis).abs() <= 0.5 + 1.0e-10);
    assert!(delta.dot(y_axis).abs() <= 1.0 + 1.0e-10);
    let rotation_error = relative_rotation_error(
        bodies[0].state().transform().rotation(),
        bodies[1].state().transform().rotation(),
    )
    .expect("rotation error");
    assert!(rotation_error.x.abs() < 1.0e-10);
    assert!(rotation_error.y.abs() < 1.0e-10);
}

#[test]
fn undamped_spring_at_rest_length_preserves_radial_velocity() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let spring = super::super::SpringJoint::new(common, 1.0, 100.0, 0.0);
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(Vec3::X, Quat::IDENTITY, Vec3::X * 2.0, Vec3::ZERO),
    ];
    let before = [bodies[0].state(), bodies[1].state()];
    solve_joint_velocity_only_with_dt(super::super::Joint::Spring(spring), &mut bodies, 0.01);
    assert_eq!(
        bodies[0].state().linear_velocity(),
        before[0].linear_velocity()
    );
    assert_eq!(
        bodies[1].state().linear_velocity(),
        before[1].linear_velocity()
    );
}

#[test]
fn zero_force_spring_keeps_radial_velocity_in_fixed_block_mode() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let joint =
        super::super::Joint::Spring(super::super::SpringJoint::new(common, 1.0, 100.0, 0.0));
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint,
    };
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(Vec3::X, Quat::IDENTITY, Vec3::X * 2.0, Vec3::ZERO),
    ];
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &bodies, &[constraint], 0.01).unwrap();
    let options =
        JointSolverOptions::legacy().with_projection(super::super::JointProjection::FixedBlock3);
    JointSolver::solve_velocity_prepared_iteration(
        &mut workspace,
        &mut bodies,
        &[constraint],
        0.01,
        options,
    )
    .unwrap();
    assert_eq!(bodies[0].state().linear_velocity(), Vec3::ZERO);
    assert_eq!(bodies[1].state().linear_velocity(), Vec3::X * 2.0);
}

#[cfg(feature = "bench-internals")]
#[test]
fn global_velocity_pcg_rejects_zero_force_spring_row() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let joint =
        super::super::Joint::Spring(super::super::SpringJoint::new(common, 1.0, 100.0, 0.0));
    let constraint = JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint,
    };
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(Vec3::X, Quat::IDENTITY, Vec3::X * 2.0, Vec3::ZERO),
    ];
    let before = [bodies[0].state(), bodies[1].state()];
    let mut workspace = JointWorkspace::new();
    let result = JointSolver::solve_velocity_pcg(
        &mut workspace,
        &mut bodies,
        &[constraint],
        0.01,
        8,
        false,
        false,
        &crate::execution::SequentialExecutor,
        1,
    )
    .unwrap();
    assert!(result.is_none());
    assert_eq!([bodies[0].state(), bodies[1].state()], before);
}

#[test]
fn undamped_spring_impulse_is_continuous_at_rest_length() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let spring =
        super::super::Joint::Spring(super::super::SpringJoint::new(common, 1.0, 100.0, 0.0));
    for offset in [-1.0e-8, 0.0, 1.0e-8] {
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_velocities(
                Vec3::X * (1.0 + offset),
                Quat::IDENTITY,
                Vec3::X * 2.0,
                Vec3::ZERO,
            ),
        ];
        solve_joint_velocity_only_with_dt(spring, &mut bodies, 0.01);
        let actual = bodies[1].state().linear_velocity().x - bodies[0].state().linear_velocity().x;
        let expected = 2.0 - 2.0 * 100.0 * offset * 0.01;
        assert!(
            (actual - expected).abs() < 1.0e-12,
            "offset={offset} actual={actual} expected={expected}"
        );
    }
}

#[test]
fn spring_joint_applies_hooke_impulse_and_damping() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let spring = super::super::SpringJoint::new(common, 1.0, 100.0, 0.0);
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body(Vec3::new(2.0, 0.0, 0.0)),
    ];
    solve_joint(super::super::Joint::Spring(spring), &mut bodies);
    let distance = (bodies[1].state().transform().translation()
        - bodies[0].state().transform().translation())
    .length();
    let relative_speed =
        bodies[1].state().linear_velocity().x - bodies[0].state().linear_velocity().x;
    assert_eq!(distance, 2.0);
    assert!(
        (relative_speed + 2.0).abs() < 1.0e-10,
        "speed={relative_speed}"
    );

    let damper = super::super::SpringJoint::new(common, 1.0, 0.0, 100.0);
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_velocities(
            Vec3::X,
            Quat::IDENTITY,
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::ZERO,
        ),
    ];
    solve_joint(super::super::Joint::Spring(damper), &mut bodies);
    let relative_speed =
        bodies[1].state().linear_velocity().x - bodies[0].state().linear_velocity().x;
    // One implicit damping impulse is shared by all solver sweeps:
    // v_after = v_before / (1 + dt * damping * inverse_effective_mass).
    let expected = 10.0 / (1.0 + 0.01 * 100.0 * 2.0);
    assert!(
        (relative_speed - expected).abs() < 1.0e-10,
        "damped speed={relative_speed}, expected={expected}"
    );
}

#[test]
fn cone_twist_joint_enforces_independent_swing_and_twist_limits() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let joint = super::super::ConeTwistJoint::new(
        common,
        [0.2, 0.3],
        super::super::JointLimit::new(-0.1, 0.1),
    );
    let swing = Quat::from_axis_angle(Vec3::X, 0.6).expect("swing");
    let mut swing_bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::ZERO, swing),
    ];
    solve_joint(super::super::Joint::ConeTwist(joint), &mut swing_bodies);
    let swing_error = relative_rotation_error(
        swing_bodies[0].state().transform().rotation(),
        swing_bodies[1].state().transform().rotation(),
    )
    .expect("swing error");
    assert!(
        (swing_error.x - 0.2).abs() < 1.0e-10,
        "swing={swing_error:?}"
    );

    let twist = Quat::from_axis_angle(Vec3::Z, 0.6).expect("twist");
    let mut twist_bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::ZERO, twist),
    ];
    solve_joint(super::super::Joint::ConeTwist(joint), &mut twist_bodies);
    let twist_error = relative_rotation_error(
        twist_bodies[0].state().transform().rotation(),
        twist_bodies[1].state().transform().rotation(),
    )
    .expect("twist error");
    assert!(
        (twist_error.z - 0.1).abs() < 1.0e-10,
        "twist={twist_error:?}"
    );

    let diagonal = Vec3::new(1.0, 1.0, 0.0)
        .try_normalize()
        .expect("diagonal swing axis");
    let swing = Quat::from_axis_angle(diagonal, 0.6).expect("diagonal swing");
    let mut diagonal_bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::ZERO, swing),
    ];
    solve_joint(super::super::Joint::ConeTwist(joint), &mut diagonal_bodies);
    let diagonal_error = relative_rotation_error(
        diagonal_bodies[0].state().transform().rotation(),
        diagonal_bodies[1].state().transform().rotation(),
    )
    .expect("diagonal swing error");
    let expected_radius = 1.0
        / ((diagonal.x / joint.swing_limit[0]).powi(2)
            + (diagonal.y / joint.swing_limit[1]).powi(2))
        .sqrt();
    assert!(
        (diagonal_error.x.hypot(diagonal_error.y) - expected_radius).abs() < 1.0e-10,
        "diagonal swing={diagonal_error:?}, boundary={expected_radius}"
    );
}

#[test]
fn d6_joint_applies_free_limited_and_locked_axes() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::D6Joint::new(common);
    joint.linear_motion = [
        super::super::JointMotion::Free,
        super::super::JointMotion::Limited(super::super::JointLimit::new(-1.0, 1.0)),
        super::super::JointMotion::Locked,
    ];
    joint.angular_motion = [
        super::super::JointMotion::Free,
        super::super::JointMotion::Limited(super::super::JointLimit::new(-0.2, 0.2)),
        super::super::JointMotion::Locked,
    ];
    let rotation = Quat::from_axis_angle(Vec3::Y, 0.5).expect("rotation");
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body_with_rotation(Vec3::new(3.0, 2.0, 4.0), rotation),
    ];

    solve_joint(super::super::Joint::D6(joint), &mut bodies);

    let delta =
        bodies[1].state().transform().translation() - bodies[0].state().transform().translation();
    let frame_rotation = bodies[0].state().transform().rotation();
    let local_delta = frame_rotation
        .conjugate()
        .rotate_vector(delta)
        .expect("joint frame delta");
    assert!(local_delta.x > 2.5, "free x={}", local_delta.x);
    assert!((-1.0 - 1.0e-10..=1.0 + 1.0e-10).contains(&local_delta.y));
    assert!(local_delta.z.abs() < 1.0e-10, "locked z={}", local_delta.z);
    let rotation_error = relative_rotation_error(
        bodies[0].state().transform().rotation(),
        bodies[1].state().transform().rotation(),
    )
    .expect("rotation error");
    assert!(rotation_error.x.abs() < 1.0e-10);
    assert!(rotation_error.y.abs() <= 0.2 + 1.0e-10);
    assert!(rotation_error.z.abs() < 1.0e-10);
}

#[test]
fn d6_drive_targets_a_free_axis() {
    let common = super::super::JointCommon::new(
        super::super::JointFrame::IDENTITY,
        super::super::JointFrame::IDENTITY,
    );
    let mut joint = super::super::D6Joint::new(common);
    joint.linear_motion[0] = super::super::JointMotion::Free;
    joint.linear_drives[0] = super::super::JointDrive {
        enabled: true,
        target_position: 1.0,
        target_velocity: 0.0,
        stiffness: 1.0e6,
        damping: 0.0,
        max_force: 1.0e8,
    };
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];

    solve_joint(super::super::Joint::D6(joint), &mut bodies);

    let separation = bodies[1].state().transform().translation().x
        - bodies[0].state().transform().translation().x;
    assert!(separation > 0.99, "driven separation={separation}");
}

fn d6_free_x_joint(drive: super::super::JointDrive) -> super::super::Joint {
    let mut joint = super::super::D6Joint::new(aligned_axis_common());
    joint.linear_motion[0] = super::super::JointMotion::Free;
    joint.linear_drives[0] = drive;
    super::super::Joint::D6(joint)
}

#[test]
fn disabled_and_zero_force_drives_leave_free_axes_unmoved() {
    // A disabled drive and an enabled drive with zero force budget must both
    // emit no row, so a free axis keeps its initial separation and velocity.
    let disabled = super::super::JointDrive {
        enabled: false,
        target_position: 5.0,
        target_velocity: 5.0,
        stiffness: 1.0e6,
        damping: 10.0,
        max_force: 1.0e8,
    };
    let zero_force = super::super::JointDrive {
        max_force: 0.0,
        ..disabled
    };
    for drive in [disabled, zero_force] {
        let mut bodies = [
            solver_body(Vec3::ZERO),
            solver_body_with_velocities(
                Vec3::new(0.3, 0.0, 0.0),
                Quat::IDENTITY,
                Vec3::new(2.0, 0.0, 0.0),
                Vec3::ZERO,
            ),
        ];
        solve_joint(d6_free_x_joint(drive), &mut bodies);
        let separation = bodies[1].state().transform().translation().x
            - bodies[0].state().transform().translation().x;
        assert!(
            (separation - 0.3).abs() <= 1.0e-12,
            "disabled drive moved a free axis to {separation}"
        );
        assert!(
            (bodies[1].state().linear_velocity().x - 2.0).abs() <= 1.0e-12,
            "disabled drive changed a free-axis velocity to {}",
            bodies[1].state().linear_velocity().x
        );
    }
}

#[test]
fn d6_linear_drive_moves_toward_negative_target() {
    let drive = super::super::JointDrive {
        enabled: true,
        target_position: -1.0,
        target_velocity: 0.0,
        stiffness: 1.0e6,
        damping: 0.0,
        max_force: 1.0e8,
    };
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    solve_joint(d6_free_x_joint(drive), &mut bodies);
    let separation = bodies[1].state().transform().translation().x
        - bodies[0].state().transform().translation().x;
    assert!(separation < -0.99, "driven separation={separation}");
}

#[test]
fn cylindrical_translation_drive_respects_signed_target() {
    let drive = super::super::JointDrive {
        enabled: true,
        target_position: -0.6,
        target_velocity: 0.0,
        stiffness: 1.0e6,
        damping: 0.0,
        max_force: 1.0e8,
    };
    let mut joint = super::super::CylindricalJoint::new(aligned_axis_common());
    joint.translation_drive = drive;
    let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
    solve_joint(super::super::Joint::Cylindrical(joint), &mut bodies);
    let axial = bodies[1].state().transform().translation().z
        - bodies[0].state().transform().translation().z;
    assert!(axial < -0.59, "cylindrical axial drive={axial}");
}

#[test]
fn drive_reaches_signed_target_with_asymmetric_masses() {
    // With two dynamic bodies of very different mass the drive must still
    // reach its target in the relative coordinate while conserving the
    // center of mass.
    let drive = super::super::JointDrive {
        enabled: true,
        target_position: 0.5,
        target_velocity: 0.0,
        stiffness: 1.0e6,
        damping: 0.0,
        max_force: 1.0e9,
    };
    let inertia = kapitsa_math::Mat3::from_diagonal(Vec3::new(1.0, 1.0, 1.0));
    let mut bodies = [
        solver_body_with_properties(
            Vec3::ZERO,
            Quat::IDENTITY,
            1.0,
            Vec3::ZERO,
            inertia,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
        solver_body_with_properties(
            Vec3::new(0.1, 0.0, 0.0),
            Quat::IDENTITY,
            5.0,
            Vec3::ZERO,
            inertia,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
    ];
    solve_joint(d6_free_x_joint(drive), &mut bodies);
    let separation = bodies[1].state().transform().translation().x
        - bodies[0].state().transform().translation().x;
    assert!(
        separation > 0.49,
        "asymmetric driven separation={separation}"
    );
    // The drive changes only the relative coordinate, so the mass-weighted
    // center of mass (m_a = 1, m_b = 5) must stay where it started.
    let com = (bodies[0].state().transform().translation().x
        + 5.0 * bodies[1].state().transform().translation().x)
        / 6.0;
    let initial_com = 5.0 * 0.1 / 6.0;
    assert!(
        (com - initial_com).abs() <= 1.0e-9,
        "drive moved the center of mass by {}",
        com - initial_com
    );
}

#[test]
fn compliant_drive_leaves_more_residual_than_rigid_drive() {
    let target = 1.0;
    let run = |stiffness: f64| -> f64 {
        let drive = super::super::JointDrive {
            enabled: true,
            target_position: target,
            target_velocity: 0.0,
            stiffness,
            damping: 0.0,
            max_force: 1.0e9,
        };
        let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::ZERO)];
        solve_joint(d6_free_x_joint(drive), &mut bodies);
        bodies[1].state().transform().translation().x
            - bodies[0].state().transform().translation().x
    };
    let rigid = run(1.0e7);
    let compliant = run(20.0);
    assert!(
        rigid > compliant + 0.05,
        "rigid drive residual {rigid} was not tighter than compliant {compliant}"
    );
    assert!(
        compliant.abs() > 1.0e-6,
        "compliant drive unexpectedly reached the target exactly: {compliant}"
    );
}

#[test]
fn spring_pulls_along_nonzero_anchor_line() {
    // Anchors are offset so the spring direction is the anchor-to-anchor
    // line, not the body-centre line. A stretched spring must pull body B
    // toward body A along that line.
    let mut common = super::super::JointCommon::new(
        super::super::JointFrame::new(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
        super::super::JointFrame::new(Vec3::new(-1.0, 0.0, 0.0), Quat::IDENTITY),
    );
    common.break_force = None;
    // Anchor A sits at (1,0,0); anchor B, with body B at (2,0,0), sits at
    // (1,0,0) too, so the anchors coincide: build the separation via body B.
    let spring = super::super::SpringJoint::new(common, 0.5, 100.0, 0.0);
    let mut bodies = [
        solver_body(Vec3::ZERO),
        solver_body(Vec3::new(3.0, 0.0, 0.0)),
    ];
    solve_joint(super::super::Joint::Spring(spring), &mut bodies);
    // anchor A = (1,0,0); anchor B = (2,0,0); distance 1.0 > rest 0.5.
    let relative_speed =
        bodies[1].state().linear_velocity().x - bodies[0].state().linear_velocity().x;
    assert!(
        relative_speed < -1.0e-6,
        "stretched anchored spring must pull B toward A, got {relative_speed}"
    );
}

fn hard_lock_row_targets_zero_velocity(joint: Joint) {
    let constraints = [JointConstraint {
        id: test_joint_id(),
        body_a: 0,
        body_b: 1,
        joint,
    }];
    let bodies = [
        static_solver_body(Vec3::ZERO, Quat::IDENTITY),
        solver_body_with_rotation(
            Vec3::new(0.3, -0.4, 0.5),
            Quat::from_axis_angle(Vec3::new(1.0, 2.0, -1.0), 0.7).expect("rotation"),
        ),
    ];
    let mut workspace = JointWorkspace::new();
    JointSolver::prepare(&mut workspace, &bodies, &constraints, 0.01).expect("prepare rows");
    // Every hard bilateral lock row must target zero relative velocity: a
    // Baumgarte bias would instead target a nonzero value proportional to
    // the position error and inject energy on top of position projection.
    for row in workspace.rows() {
        assert!(
            row.target_velocity.abs() <= 1.0e-12,
            "lock row key {} has bias target {} for error {}",
            row.key,
            row.target_velocity,
            row.error
        );
    }
}

#[test]
fn hard_bilateral_lock_rows_do_not_apply_baumgarte_bias() {
    hard_lock_row_targets_zero_velocity(Joint::Hinge(HingeJoint::new(aligned_axis_common())));
    hard_lock_row_targets_zero_velocity(Joint::Slider(SliderJoint::new(aligned_axis_common())));
    hard_lock_row_targets_zero_velocity(Joint::Ball(BallJoint::new(aligned_axis_common())));
    hard_lock_row_targets_zero_velocity(Joint::Cylindrical(CylindricalJoint::new(
        aligned_axis_common(),
    )));
    hard_lock_row_targets_zero_velocity(Joint::D6(D6Joint::new(aligned_axis_common())));
}

#[test]
fn resting_violated_lock_does_not_gain_velocity() {
    // Two dynamic bodies joined by a hinge whose anchors start apart and at
    // rest. Position projection removes the gap; the velocity solve must not
    // create motion from the position error.
    let frame_a = JointFrame::new(
        Vec3::new(0.1, -0.2, 0.3),
        Quat::from_axis_angle(Vec3::Z, 0.2).expect("frame a"),
    );
    let frame_b = JointFrame::new(
        Vec3::new(-0.4, 0.15, 0.2),
        Quat::from_axis_angle(Vec3::X, -0.3).expect("frame b"),
    );
    let inertia = kapitsa_math::Mat3::from_diagonal(Vec3::new(0.4, 1.1, 1.4));
    let joint = Joint::Hinge(HingeJoint::new(JointCommon::new(frame_a, frame_b)));
    let initial_b = frame_a.local_anchor - frame_b.local_anchor + Vec3::new(0.35, -0.2, 0.4);
    let mut bodies = [
        solver_body_with_properties(
            Vec3::new(0.2, 0.1, -0.3),
            Quat::from_axis_angle(Vec3::Y, 0.5).expect("rotation a"),
            1.0,
            Vec3::new(0.05, -0.02, 0.01),
            inertia,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
        solver_body_with_properties(
            initial_b,
            Quat::from_axis_angle(Vec3::X, 0.4).expect("rotation b"),
            2.0,
            Vec3::new(-0.03, 0.04, 0.02),
            inertia,
            Vec3::ZERO,
            Vec3::ZERO,
        ),
    ];

    solve_joint_velocity_only_with_dt(joint, &mut bodies, 1.0 / 60.0);
    assert_eq!(
        bodies[0].state().linear_velocity(),
        Vec3::ZERO,
        "body A gained velocity from a resting violation"
    );
    assert_eq!(
        bodies[1].state().linear_velocity(),
        Vec3::ZERO,
        "body B gained velocity from a resting violation"
    );

    solve_joint(joint, &mut bodies);
    let anchor_error = joint_anchor_error(&bodies, frame_a, frame_b);
    assert!(
        anchor_error <= 1.0e-3,
        "position projection should close the anchor gap, got {anchor_error}"
    );
}

fn joint_anchor_error(bodies: &[SolverBody; 2], frame_a: JointFrame, frame_b: JointFrame) -> f64 {
    let anchor = |body: &SolverBody, local: Vec3| {
        let transform = body.state().transform();
        transform.translation()
            + transform
                .rotation()
                .rotate_vector(local)
                .expect("anchor rotation")
    };
    (anchor(&bodies[1], frame_b.local_anchor) - anchor(&bodies[0], frame_a.local_anchor)).length()
}

fn body_momentum(body: &SolverBody) -> (Vec3, Vec3) {
    let mass = body.body().physics().mass().expect("dynamic mass");
    let state = body.state();
    let center = body
        .body()
        .world_center_of_mass()
        .expect("center of mass")
        .unwrap_or(state.transform().translation());
    let linear = state.linear_velocity() * mass.mass();
    let rotation = state.transform().rotation().to_mat3().expect("rotation");
    let inertia_world = rotation * mass.inertia() * rotation.transpose();
    let angular = center.checked_cross(linear).expect("angular momentum")
        + inertia_world * state.angular_velocity();
    (linear, angular)
}

fn kinetic_energy(body: &SolverBody) -> f64 {
    let mass = body.body().physics().mass().expect("dynamic mass");
    let state = body.state();
    let rotation = state.transform().rotation().to_mat3().expect("rotation");
    let inertia_world = rotation * mass.inertia() * rotation.transpose();
    0.5 * mass.mass() * state.linear_velocity().length_squared()
        + 0.5
            * state
                .angular_velocity()
                .dot(inertia_world * state.angular_velocity())
}

fn two_body_hinge_fixture() -> ([SolverBody; 2], Joint) {
    let frame_a = JointFrame::new(
        Vec3::new(0.1, -0.2, 0.3),
        Quat::from_axis_angle(Vec3::Z, 0.2).expect("frame a"),
    );
    let frame_b = JointFrame::new(
        Vec3::new(-0.4, 0.15, 0.2),
        Quat::from_axis_angle(Vec3::X, -0.3).expect("frame b"),
    );
    let inertia = kapitsa_math::Mat3::from_diagonal(Vec3::new(0.4, 1.1, 1.4));
    let joint = Joint::Hinge(HingeJoint::new(JointCommon::new(frame_a, frame_b)));
    let bodies = [
        solver_body_with_properties(
            Vec3::new(0.2, 0.1, -0.3),
            Quat::from_axis_angle(Vec3::Y, 0.5).expect("rotation a"),
            1.0,
            Vec3::new(0.05, -0.02, 0.01),
            inertia,
            Vec3::new(1.5, -0.7, 0.9),
            Vec3::new(-0.6, 1.2, 0.4),
        ),
        solver_body_with_properties(
            Vec3::new(0.3, -0.25, 0.6),
            Quat::from_axis_angle(Vec3::X, 0.4).expect("rotation b"),
            2.0,
            Vec3::new(-0.03, 0.04, 0.02),
            inertia,
            Vec3::new(-0.8, 0.5, -1.1),
            Vec3::new(0.7, -0.3, 1.3),
        ),
    ];
    (bodies, joint)
}

#[test]
fn joint_solve_conserves_linear_and_angular_momentum() {
    // Coincident anchors: the equal-and-opposite constraint forces act at
    // the same world point, so a velocity-only solve must leave total linear
    // and angular momentum untouched. Positions stay fixed, so the momentum
    // arms do not move.
    let frame_a = JointFrame::new(
        Vec3::new(0.1, -0.2, 0.3),
        Quat::from_axis_angle(Vec3::Z, 0.2).expect("frame a"),
    );
    let frame_b = JointFrame::new(
        Vec3::new(-0.4, 0.15, 0.2),
        Quat::from_axis_angle(Vec3::X, -0.3).expect("frame b"),
    );
    let inertia = kapitsa_math::Mat3::from_diagonal(Vec3::new(0.4, 1.1, 1.4));
    let pose_a = Transform::new(
        Vec3::new(0.2, 0.1, -0.3),
        Quat::from_axis_angle(Vec3::Y, 0.5).expect("rotation a"),
    )
    .expect("pose a");
    let rotation_b = Quat::from_axis_angle(Vec3::X, 0.4).expect("rotation b");
    let world_anchor_a = pose_a.translation()
        + pose_a
            .rotation()
            .rotate_vector(frame_a.local_anchor)
            .expect("anchor a");
    let pose_b = Transform::new(
        world_anchor_a
            - rotation_b
                .rotate_vector(frame_b.local_anchor)
                .expect("anchor b"),
        rotation_b,
    )
    .expect("pose b");
    let joint = Joint::Hinge(HingeJoint::new(JointCommon::new(frame_a, frame_b)));
    let mut bodies = [
        solver_body_with_properties(
            pose_a.translation(),
            pose_a.rotation(),
            1.0,
            Vec3::ZERO,
            inertia,
            Vec3::new(1.5, -0.7, 0.9),
            Vec3::new(-0.6, 1.2, 0.4),
        ),
        solver_body_with_properties(
            pose_b.translation(),
            pose_b.rotation(),
            2.0,
            Vec3::ZERO,
            inertia,
            Vec3::new(-0.8, 0.5, -1.1),
            Vec3::new(0.7, -0.3, 1.3),
        ),
    ];
    let before = [body_momentum(&bodies[0]), body_momentum(&bodies[1])];
    solve_joint_velocity_only_with_dt(joint, &mut bodies, 1.0 / 60.0);
    let after = [body_momentum(&bodies[0]), body_momentum(&bodies[1])];
    let linear_drift = (after[0].0 + after[1].0 - before[0].0 - before[1].0).length();
    let angular_drift = (after[0].1 + after[1].1 - before[0].1 - before[1].1).length();
    assert!(
        linear_drift <= 1.0e-10,
        "linear momentum drift {linear_drift}"
    );
    assert!(
        angular_drift <= 1.0e-10,
        "angular momentum drift {angular_drift}"
    );
}

#[test]
fn joint_solve_does_not_grow_kinetic_energy() {
    let (mut bodies, joint) = two_body_hinge_fixture();
    let energy_before = kinetic_energy(&bodies[0]) + kinetic_energy(&bodies[1]);
    solve_joint(joint, &mut bodies);
    let energy_after = kinetic_energy(&bodies[0]) + kinetic_energy(&bodies[1]);
    assert!(
        energy_after <= energy_before + 1.0e-9,
        "kinetic energy grew from {energy_before} to {energy_after}"
    );
}
