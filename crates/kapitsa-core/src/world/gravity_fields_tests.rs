use super::PhysicsWorld;
use crate::{
    execution::SequentialExecutor,
    gravity::{CentralGravity, SphericalGravity, UniformGravity},
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
};
use kapitsa_math::{Mat3, Vec3};

fn body() -> RigidBody {
    RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
    )
    .unwrap()
}

#[test]
fn fields_add_update_remove_and_work_with_shards() {
    for shards in [false, true] {
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body()).unwrap();
        let first = world
            .add_gravity(UniformGravity::new(Vec3::X).unwrap())
            .unwrap();
        let second = world
            .add_gravity(UniformGravity::new(Vec3::Y).unwrap())
            .unwrap();
        if shards {
            world
                .step_with_shards(0.1, &mut [Vec::new()], &SequentialExecutor, 1)
                .unwrap();
        } else {
            world.step(0.1).unwrap();
        }
        let velocity = world.body(id).unwrap().state().linear_velocity();
        assert!((velocity - Vec3::new(0.1, 0.1, 0.0)).length() < 1e-12);
        world
            .update_gravity(first, UniformGravity::new(Vec3::ZERO).unwrap())
            .unwrap();
        assert!(world.remove_gravity(second).is_some());
        assert!(
            world
                .update_gravity(second, UniformGravity::new(Vec3::X).unwrap())
                .is_err()
        );
        world.step(0.1).unwrap();
        assert!((world.body(id).unwrap().state().linear_velocity() - velocity).length() < 1e-12);
    }
}

#[test]
fn spherical_field_has_explicit_boundary() {
    let field = SphericalGravity::new(Vec3::ZERO, 100.0, 10.0).unwrap();
    assert_eq!(
        field.acceleration_at(Vec3::new(10.0, 0.0, 0.0)).unwrap(),
        -Vec3::X
    );
    assert_eq!(
        field.acceleration_at(Vec3::new(11.0, 0.0, 0.0)).unwrap(),
        Vec3::ZERO
    );
    assert!(field.acceleration_at(Vec3::ZERO).is_err());
    assert!(SphericalGravity::new(Vec3::ZERO, 1.0, 0.0).is_err());
    assert!(UniformGravity::new(Vec3::new(f64::NAN, 0.0, 0.0)).is_err());
}

#[test]
fn gravity_ids_are_scoped_to_world_and_reject_reuse() {
    let field = UniformGravity::new(Vec3::X).unwrap();
    let mut first = PhysicsWorld::new();
    let mut second = PhysicsWorld::new();
    let id = first.add_gravity(field).unwrap();
    let other = second.add_gravity(field).unwrap();
    assert!(second.gravity(id).is_none());
    assert!(first.remove_gravity(other).is_none());
    first.remove_gravity(id).unwrap();
    let replacement = first.add_gravity(field).unwrap();
    assert_ne!(id, replacement);
    assert!(first.gravity(id).is_none());
    assert!(first.update_gravity(id, field).is_err());
}

#[test]
fn legacy_and_managed_gravity_are_summed() {
    let mut world = PhysicsWorld::new();
    let id = world.add_body(body()).unwrap();
    world.set_central_gravity(Some(
        CentralGravity::new(Vec3::new(10.0, 0.0, 0.0), 100.0).unwrap(),
    ));
    world
        .add_gravity(UniformGravity::new(Vec3::Y).unwrap())
        .unwrap();
    world.step(0.1).unwrap();
    let velocity = world.body(id).unwrap().state().linear_velocity();
    assert!((velocity - Vec3::new(0.1, 0.1, 0.0)).length() < 1e-12);
}

#[test]
fn gravity_failure_rolls_back_a_step() {
    for shards in [false, true] {
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body()).unwrap();
        world
            .add_gravity(UniformGravity::new(Vec3::X).unwrap())
            .unwrap();
        let bad = world
            .add_gravity(CentralGravity::new(Vec3::ZERO, 1.0).unwrap())
            .unwrap();
        let result = if shards {
            world.step_with_shards(0.1, &mut [Vec::new()], &SequentialExecutor, 1)
        } else {
            world.step(0.1)
        };
        assert!(result.is_err());
        assert_eq!(
            world.body(id).unwrap().state().linear_velocity(),
            Vec3::ZERO
        );
        world.remove_gravity(bad).unwrap();
        world.step(0.1).unwrap();
        assert!(
            (world.body(id).unwrap().state().linear_velocity() - Vec3::X * 0.1).length() < 1e-12
        );
    }
}
