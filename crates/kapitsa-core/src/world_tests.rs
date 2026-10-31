use glam::{Mat3, Vec3};

use super::{BodyId, PhysicsWorld, WorldError};
use crate::{
    containers::{DensePool, DensePoolError},
    mass::MassProperties,
    rigid_body::{BodyType, RigidBody},
    transform::Transform,
};

fn dynamic_body() -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
    RigidBody::new_dynamic(Transform::identity(), mass).expect("valid dynamic body")
}

#[test]
fn stores_each_body_type_and_assigns_ids() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let static_id = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let kinematic_id = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;
    let dynamic_id = world.add_body(dynamic_body())?;

    assert_eq!(world.body_count(), 3);
    assert_eq!(
        world.body(static_id).map(|body| body.physics().body_type()),
        Some(BodyType::Static)
    );
    assert_eq!(
        world
            .body(kinematic_id)
            .map(|body| body.physics().body_type()),
        Some(BodyType::Kinematic)
    );
    assert_eq!(
        world
            .body(dynamic_id)
            .map(|body| body.physics().body_type()),
        Some(BodyType::Dynamic)
    );
    Ok(())
}

#[test]
fn body_mut_updates_the_stored_body() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::default();
    let id = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;
    let body = world.body_mut(id).ok_or(WorldError::InvalidBodyId)?;
    body.set_linear_velocity(Vec3::X)
        .expect("kinematic body accepts finite velocity");

    assert_eq!(
        world.body(id).map(|body| body.state().linear_velocity()),
        Some(Vec3::X)
    );
    Ok(())
}

#[test]
fn unknown_body_id_is_rejected() -> Result<(), DensePoolError> {
    let mut source = DensePool::new();
    let id = BodyId(source.insert(())?);
    let world = PhysicsWorld::new();

    assert!(world.body(id).is_none());
    Ok(())
}

#[test]
fn dense_pool_errors_preserve_their_variant() {
    assert_eq!(
        WorldError::from(DensePoolError::IndexOverflow),
        WorldError::DensePoolIndexOverflow
    );
    assert_eq!(
        WorldError::from(DensePoolError::CapacityExceeded),
        WorldError::DensePoolCapacityExceeded
    );
    assert_eq!(
        WorldError::from(DensePoolError::InvalidFreeList),
        WorldError::DensePoolInvalidFreeList
    );
}
