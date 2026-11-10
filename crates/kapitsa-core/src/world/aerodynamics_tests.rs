use kapitsa_math::{Mat3, Vec3};

use crate::{
    aerodynamics::{AerodynamicProperties, AtmosphericState},
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
    world::PhysicsWorld,
};

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
