use kapitsa_math::{Mat3, Vec3};

use crate::{
    aerodynamics::{AerodynamicProperties, AtmosphericState},
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
    world::PhysicsWorld,
};

fn lift_trajectory(dt: f64, steps: usize, sharded: bool, drag: f64) -> Vec3 {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::X * 10.0).unwrap();
    let mut world = PhysicsWorld::new();
    let id = world.add_body(body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    world
        .set_aerodynamics(
            id,
            Some(
                AerodynamicProperties::new(1.0, drag, Vec3::ZERO)
                    .unwrap()
                    .with_lift(1.0, Vec3::Y)
                    .unwrap(),
            ),
        )
        .unwrap();
    for _ in 0..steps {
        if sharded {
            world
                .step_with_shards(
                    dt,
                    &mut [Vec::new()],
                    &crate::execution::SequentialExecutor,
                    1,
                )
                .unwrap();
        } else {
            world.step(dt).unwrap();
        }
    }
    world.body(id).unwrap().state().linear_velocity()
}

#[test]
fn lift_preserves_energy_over_many_steps_in_both_paths() {
    for sharded in [false, true] {
        let velocity = lift_trajectory(0.01, 1000, sharded, 0.0);
        assert!((velocity.length() - 10.0).abs() < 1e-10, "{velocity:?}");
        let with_drag = lift_trajectory(0.1, 10, sharded, 0.2);
        assert!(with_drag.length() <= 10.0 + 1e-10, "{with_drag:?}");
    }
}

#[test]
fn lift_trajectory_converges_as_step_shrinks() {
    let reference = lift_trajectory(0.0001, 5000, false, 0.0);
    let coarse = (lift_trajectory(0.02, 25, false, 0.0) - reference).length();
    let fine = (lift_trajectory(0.01, 50, false, 0.0) - reference).length();
    assert!(fine < coarse * 0.6, "coarse={coarse}, fine={fine}");
}

#[test]
fn offset_lift_preserves_energy_with_rotation_and_wind() {
    for sharded in [false, true] {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::X * 12.0).unwrap();
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        let wind = Vec3::X * 2.0;
        world.set_atmosphere(Some(AtmosphericState::new(1.0, wind).unwrap()));
        world
            .set_aerodynamics(
                id,
                Some(
                    AerodynamicProperties::new(1.0, 0.0, Vec3::X)
                        .unwrap()
                        .with_lift(0.1, Vec3::Y)
                        .unwrap(),
                ),
            )
            .unwrap();
        for _ in 0..2000 {
            if sharded {
                world
                    .step_with_shards(
                        0.001,
                        &mut [Vec::new()],
                        &crate::execution::SequentialExecutor,
                        1,
                    )
                    .unwrap();
            } else {
                world.step(0.001).unwrap();
            }
        }
        let state = world.body(id).unwrap().state();
        let relative = state.linear_velocity() - wind;
        let metric =
            relative.dot(relative) + state.angular_velocity().dot(state.angular_velocity());
        assert!((metric - 100.0).abs() < 1e-8, "{metric}");
    }
}

#[test]
fn aerodynamic_task_checks_without_mutating_the_world() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::X * 1e200).unwrap();
    let mut world = PhysicsWorld::new();
    let id = world.add_body(body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    assert!(
        crate::aerodynamics::AerodynamicsTask
            .execute(&world)
            .is_ok()
    );
    world
        .set_aerodynamics(
            id,
            Some(AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap()),
        )
        .unwrap();
    assert!(
        crate::aerodynamics::AerodynamicsTask
            .execute(&world)
            .is_err()
    );
    assert_eq!(*world.body(id).unwrap(), body);
}

#[test]
fn lift_energy_drift_converges_for_anisotropic_inertia() {
    fn drift(dt: f64, steps: usize) -> f64 {
        let inertia = Mat3::from_diagonal(Vec3::new(1.0, 1.5, 2.0));
        let mass = MassProperties::new(1.0, Vec3::ZERO, inertia).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::X * 5.0).unwrap();
        body.set_angular_velocity(Vec3::new(0.5, 0.3, 0.2)).unwrap();
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
        world
            .set_aerodynamics(
                id,
                Some(
                    AerodynamicProperties::new(0.2, 0.0, Vec3::X)
                        .unwrap()
                        .with_lift(0.3, Vec3::Y)
                        .unwrap(),
                ),
            )
            .unwrap();
        let energy = |world: &PhysicsWorld| {
            let state = world.body(id).unwrap().state();
            let omega = state
                .transform()
                .rotation()
                .conjugate()
                .rotate_vector(state.angular_velocity())
                .unwrap();
            state.linear_velocity().dot(state.linear_velocity()) + omega.dot(inertia * omega)
        };
        let initial = energy(&world);
        for _ in 0..steps {
            world.step(dt).unwrap();
        }
        (energy(&world) - initial).abs()
    }
    let coarse = drift(0.01, 100);
    let fine = drift(0.005, 200);
    assert!(
        fine < coarse * 0.7 && fine < 1e-3,
        "coarse={coarse}, fine={fine}"
    );
}

#[test]
fn atmosphere_without_surfaces_does_not_evaluate_relative_wind() {
    let mut results = Vec::new();
    for sharded in [false, true] {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::X * (-f64::MAX * 0.75))
            .unwrap();
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        world.set_atmosphere(Some(
            AtmosphericState::new(1.0, Vec3::X * (f64::MAX * 0.75)).unwrap(),
        ));
        if sharded {
            world
                .step_with_shards(
                    1e-310,
                    &mut [Vec::new()],
                    &crate::execution::SequentialExecutor,
                    1,
                )
                .unwrap();
        } else {
            world.step(1e-310).unwrap();
        }
        results.push(*world.body(id).unwrap());
    }
    assert_eq!(results[0].state(), results[1].state());
}

#[test]
fn aerodynamic_drag_is_applied_during_world_step() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    world
        .set_aerodynamics(
            body_id,
            Some(AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap()),
        )
        .unwrap();

    world.step(0.1).unwrap();

    assert_eq!(
        world.body(body_id).unwrap().state().linear_velocity(),
        Vec3::new(5.0, 0.0, 0.0)
    );
}

#[test]
fn aerodynamic_drag_produces_torque_at_an_offset_point() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    world
        .set_aerodynamics(
            body_id,
            Some(AerodynamicProperties::new(2.0, 0.5, Vec3::Y).unwrap()),
        )
        .unwrap();

    world.step(0.1).unwrap();

    let angular_velocity = world.body(body_id).unwrap().state().angular_velocity();
    assert!(angular_velocity.z > 0.0);
}

#[test]
fn body_can_combine_multiple_aerodynamic_surfaces() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    let surface = AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap();
    assert_eq!(world.add_aerodynamic_surface(body_id, surface).unwrap(), 0);
    assert_eq!(world.add_aerodynamic_surface(body_id, surface).unwrap(), 1);
    assert_eq!(world.aerodynamic_surfaces(body_id).unwrap().len(), 2);

    world.step(0.1).unwrap();

    assert_eq!(
        world.body(body_id).unwrap().state().linear_velocity(),
        Vec3::ZERO
    );
}

#[test]
fn strong_drag_does_not_reverse_linear_velocity_in_one_step() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    world
        .set_aerodynamics(
            body_id,
            Some(AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap()),
        )
        .unwrap();

    world.step(0.5).unwrap();

    assert_eq!(
        world.body(body_id).unwrap().state().linear_velocity(),
        Vec3::ZERO
    );
}

#[test]
fn ordinary_and_sharded_steps_match_with_aerodynamics() {
    let make_world = || {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();
        let mut world = PhysicsWorld::new();
        let body_id = world.add_body(body).unwrap();
        world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
        world
            .set_aerodynamics(
                body_id,
                Some(AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap()),
            )
            .unwrap();
        (world, body_id)
    };

    let (mut ordinary, ordinary_id) = make_world();
    let (mut sharded, sharded_id) = make_world();
    ordinary.step(0.1).unwrap();
    sharded
        .step_with_shards(
            0.1,
            &mut [Vec::new(), Vec::new()],
            &crate::execution::SequentialExecutor,
            2,
        )
        .unwrap();

    assert_eq!(
        ordinary.body(ordinary_id).unwrap(),
        sharded.body(sharded_id).unwrap()
    );
}

#[test]
fn failed_step_restores_body_after_aerodynamics_and_gravity_preparation() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    world
        .set_aerodynamics(
            body_id,
            Some(AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap()),
        )
        .unwrap();
    world.set_central_gravity(Some(
        crate::gravity::CentralGravity::new(Vec3::ZERO, 1.0).unwrap(),
    ));
    let before = *world.body(body_id).unwrap();

    assert!(world.step(0.1).is_err());
    assert_eq!(*world.body(body_id).unwrap(), before);
}

#[test]
fn wind_accelerates_a_stationary_body() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).unwrap())
        .unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::X * 10.0).unwrap()));
    world
        .set_aerodynamics(
            body,
            Some(AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap()),
        )
        .unwrap();
    world.step(0.5).unwrap();
    let velocity = world.body(body).unwrap().state().linear_velocity().x;
    assert!(velocity > 0.0 && velocity <= 10.0);
}

#[test]
fn rotating_body_experiences_drag_at_an_offset() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut rigid_body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    rigid_body.set_angular_velocity(Vec3::Z).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world.add_body(rigid_body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    world
        .set_aerodynamics(
            body,
            Some(AerodynamicProperties::new(1.0, 1.0, Vec3::X).unwrap()),
        )
        .unwrap();
    world.step(0.1).unwrap();
    assert!(world.body(body).unwrap().state().angular_velocity().z < 1.0);
}

#[test]
fn many_surfaces_cannot_reverse_motion_under_large_drag() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut rigid_body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    rigid_body.set_linear_velocity(Vec3::X * 10.0).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world.add_body(rigid_body).unwrap();
    world.set_atmosphere(Some(AtmosphericState::new(1.0, Vec3::ZERO).unwrap()));
    for _ in 0..3 {
        world
            .add_aerodynamic_surface(
                body,
                AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap(),
            )
            .unwrap();
    }
    world.step(0.5).unwrap();
    let velocity = world.body(body).unwrap().state().linear_velocity().x;
    assert!((-1e-12..=10.0).contains(&velocity));
}

#[test]
fn changing_body_aerodynamics_preserves_part_surfaces() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).unwrap())
        .unwrap();
    let part = world.enable_cluster(body).unwrap();
    let properties = AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap();
    world
        .add_aerodynamic_surface_for_part(part, properties)
        .unwrap();
    world.set_aerodynamics(body, Some(properties)).unwrap();
    assert_eq!(world.aerodynamic_surfaces(body).unwrap().len(), 2);
    world.set_aerodynamics(body, None).unwrap();
    let surfaces = world.aerodynamic_surfaces(body).unwrap();
    assert_eq!(surfaces.len(), 1);
    assert_eq!(
        surfaces[0].target(),
        crate::aerodynamics::AerodynamicTarget::Part(part)
    );
}
