use glam::{Mat3, Vec3};

use super::{BodyId, ColliderId, PhysicsWorld, WorldError};
use crate::{
    collider::{Collider, CollisionFilter},
    containers::{DensePool, DensePoolError},
    mass::MassProperties,
    rigid_body::{BodyType, RigidBody},
    shape::{Shape, Sphere},
    transform::Transform,
};

fn dynamic_body() -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
    RigidBody::new_dynamic(Transform::identity(), mass).expect("valid dynamic body")
}

fn collider(radius: f32) -> Collider {
    let sphere = Sphere::new(radius).expect("valid sphere");
    Collider::new(Shape::Sphere(sphere), Transform::identity())
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

#[test]
fn adds_multiple_colliders_and_tracks_each_owner() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let first_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let second_body = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;
    let first = world.add_collider(first_body, collider(1.0))?;
    let second = world.add_collider(first_body, collider(2.0))?;
    let other = world.add_collider(second_body, collider(3.0))?;

    assert_eq!(world.collider_count(), 3);
    assert_eq!(world.collider_body(first), Some(first_body));
    assert_eq!(world.collider_body(second), Some(first_body));
    assert_eq!(world.collider_body(other), Some(second_body));
    assert!(world.collider(first).is_some());
    Ok(())
}

#[test]
fn collider_mut_changes_only_the_collider() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;
    let original_body = *world.body(body_id).ok_or(WorldError::InvalidBodyId)?;
    let collider_id = world.add_collider(body_id, collider(1.0))?;

    world
        .collider_mut(collider_id)
        .ok_or(WorldError::InvalidColliderId)?
        .set_filter(CollisionFilter::new(0b0100, 0b1000));

    assert_eq!(world.body(body_id), Some(&original_body));
    assert_eq!(
        world.collider(collider_id).map(Collider::filter),
        Some(CollisionFilter::new(0b0100, 0b1000))
    );
    Ok(())
}

#[test]
fn invalid_body_rejects_collider_without_changing_world() -> Result<(), DensePoolError> {
    let mut stale_ids = DensePool::new();
    let first = stale_ids.insert(())?;
    let _ = stale_ids.remove(first);
    let stale_body = BodyId(stale_ids.insert(())?);
    let mut world = PhysicsWorld::new();
    world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body insertion succeeds");

    assert_eq!(
        world.add_collider(stale_body, collider(1.0)),
        Err(WorldError::InvalidBodyId)
    );
    assert_eq!(world.body_count(), 1);
    assert_eq!(world.collider_count(), 0);
    Ok(())
}

#[test]
fn removing_colliders_in_arbitrary_order_updates_body_links() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let first = world.add_collider(body, collider(1.0))?;
    let second = world.add_collider(body, collider(2.0))?;
    let third = world.add_collider(body, collider(3.0))?;
    let fourth = world.add_collider(body, collider(4.0))?;

    assert!(world.remove_collider(second).is_some());
    assert!(world.remove_collider(fourth).is_some());
    assert!(world.remove_collider(first).is_some());
    assert!(world.remove_collider(third).is_some());

    assert_eq!(world.collider_count(), 0);
    assert!(world.collider(first).is_none());
    assert!(world.collider_body(second).is_none());
    assert!(world.remove_collider(third).is_none());
    Ok(())
}

#[test]
fn removed_collider_slot_reuses_generation_without_reviving_old_id() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let stale = world.add_collider(body, collider(1.0))?;
    assert!(world.remove_collider(stale).is_some());

    let replacement = world.add_collider(body, collider(2.0))?;

    assert_eq!(stale.0.index(), replacement.0.index());
    assert_ne!(stale.0.generation(), replacement.0.generation());
    assert!(world.collider(stale).is_none());
    assert_eq!(world.collider_body(replacement), Some(body));
    assert_eq!(world.collider_count(), 1);
    Ok(())
}

#[test]
fn body_state_is_unchanged_when_colliders_are_added_or_removed() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(dynamic_body())?;
    let original_body = *world.body(body_id).ok_or(WorldError::InvalidBodyId)?;
    let collider_id = world.add_collider(body_id, collider(1.0))?;
    assert!(world.remove_collider(collider_id).is_some());

    assert_eq!(world.body(body_id), Some(&original_body));
    Ok(())
}

#[test]
fn invalid_collider_ids_are_rejected() -> Result<(), DensePoolError> {
    let mut source = DensePool::new();
    let id = ColliderId(source.insert(())?);
    let mut world = PhysicsWorld::new();

    assert!(world.collider(id).is_none());
    assert!(world.collider_mut(id).is_none());
    assert!(world.collider_body(id).is_none());
    assert!(world.remove_collider(id).is_none());
    Ok(())
}

#[test]
fn removing_body_without_colliders_returns_it_and_invalidates_its_id() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let original = *world.body(body_id).ok_or(WorldError::InvalidBodyId)?;

    assert_eq!(world.remove_body(body_id), Some(original));
    assert_eq!(world.remove_body(body_id), None);
    assert!(world.body(body_id).is_none());
    assert_eq!(world.body_count(), 0);
    Ok(())
}

#[test]
fn removing_body_cascades_one_and_many_colliders() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let single_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let many_body = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;
    let other_body = world.add_body(dynamic_body())?;
    let single_collider = world.add_collider(single_body, collider(1.0))?;
    let many_colliders = [
        world.add_collider(many_body, collider(2.0))?,
        world.add_collider(many_body, collider(3.0))?,
        world.add_collider(many_body, collider(4.0))?,
    ];
    let other_collider = world.add_collider(other_body, collider(5.0))?;

    assert!(world.remove_body(single_body).is_some());
    assert!(world.collider(single_collider).is_none());
    assert_eq!(world.collider_count(), 4);

    assert!(world.remove_body(many_body).is_some());
    for id in many_colliders {
        assert!(world.collider(id).is_none());
        assert!(world.collider_body(id).is_none());
    }
    assert_eq!(world.body_count(), 1);
    assert_eq!(world.collider_count(), 1);
    assert!(world.body(other_body).is_some());
    assert_eq!(world.collider_body(other_collider), Some(other_body));
    Ok(())
}

#[test]
fn iterators_cover_empty_and_populated_worlds() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    assert_eq!(world.bodies().count(), 0);
    assert_eq!(world.colliders().count(), 0);

    let first_body = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;
    let second_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let first_collider = world.add_collider(first_body, collider(1.0))?;
    let second_collider = world.add_collider(second_body, collider(2.0))?;
    let extra_collider = world.add_collider(first_body, collider(3.0))?;

    let bodies: Vec<_> = world.bodies().collect();
    assert_eq!(bodies.len(), 2);
    assert!(bodies.iter().any(|(id, _)| *id == first_body));
    assert!(bodies.iter().any(|(id, _)| *id == second_body));
    for (id, _) in bodies {
        assert!(world.body(id).is_some());
    }

    let colliders: Vec<_> = world.colliders().collect();
    assert_eq!(colliders.len(), 3);
    assert!(
        colliders
            .iter()
            .any(|(id, owner, _)| *id == first_collider && *owner == first_body)
    );
    assert!(
        colliders
            .iter()
            .any(|(id, owner, _)| *id == extra_collider && *owner == first_body)
    );
    assert!(
        colliders
            .iter()
            .any(|(id, owner, _)| *id == second_collider && *owner == second_body)
    );
    Ok(())
}

#[test]
fn colliders_of_iterates_only_the_requested_body_list() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let first_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let second_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let first = world.add_collider(first_body, collider(1.0))?;
    let second = world.add_collider(first_body, collider(2.0))?;
    let other = world.add_collider(second_body, collider(3.0))?;

    let first_body_colliders: Vec<_> = world
        .colliders_of(first_body)
        .ok_or(WorldError::InvalidBodyId)?
        .map(|(id, _)| id)
        .collect();
    let second_body_colliders: Vec<_> = world
        .colliders_of(second_body)
        .ok_or(WorldError::InvalidBodyId)?
        .map(|(id, _)| id)
        .collect();

    assert_eq!(first_body_colliders.len(), 2);
    assert!(first_body_colliders.contains(&first));
    assert!(first_body_colliders.contains(&second));
    assert_eq!(second_body_colliders, [other]);
    Ok(())
}

#[test]
fn bodies_mut_visits_each_body_once() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let first = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;
    let second = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;

    for (_, body) in world.bodies_mut() {
        body.set_linear_velocity(Vec3::Y)
            .expect("kinematic body accepts finite velocity");
    }

    assert_eq!(
        world.body(first).map(|body| body.state().linear_velocity()),
        Some(Vec3::Y)
    );
    assert_eq!(
        world
            .body(second)
            .map(|body| body.state().linear_velocity()),
        Some(Vec3::Y)
    );
    Ok(())
}

#[test]
fn deleting_body_and_reusing_slots_does_not_revive_old_ids() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let old_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let old_collider = world.add_collider(old_body, collider(1.0))?;
    assert!(world.remove_body(old_body).is_some());

    let new_body = world.add_body(RigidBody::new_kinematic(Transform::identity()))?;
    let new_collider = world.add_collider(new_body, collider(2.0))?;

    assert_eq!(old_body.0.index(), new_body.0.index());
    assert_ne!(old_body.0.generation(), new_body.0.generation());
    assert!(world.body(old_body).is_none());
    assert!(world.collider(old_collider).is_none());
    assert_eq!(world.collider_body(new_collider), Some(new_body));
    assert_eq!(
        world
            .colliders_of(new_body)
            .ok_or(WorldError::InvalidBodyId)?
            .count(),
        1
    );
    Ok(())
}
