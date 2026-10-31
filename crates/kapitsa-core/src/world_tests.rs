use kapitsa_math::{Affine3, Mat3, Vec3};

use super::{BodyId, ColliderId, PhysicsWorld, StepError, WorldError, allocate_world_id};
use crate::{
    collider::{Collider, CollisionFilter},
    collision::CollisionError,
    containers::{DensePool, DensePoolError},
    mass::MassProperties,
    material::PhysicsMaterial,
    rigid_body::{BodyType, IntegrationError, RigidBody},
    shape::{BoneInfluence, Shape, SkinnedTriangleMesh, Sphere, TriangleMesh},
    transform::Transform,
};
use std::{collections::HashSet, sync::atomic::AtomicU64};

fn dynamic_body() -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
    RigidBody::new_dynamic(Transform::identity(), mass).expect("valid dynamic body")
}

fn collider(radius: f64) -> Collider {
    let sphere = Sphere::new(radius).expect("valid sphere");
    Collider::new(Shape::Sphere(sphere), Transform::identity())
}

#[test]
fn world_sets_skin_pose_and_rejects_non_skin_colliders() {
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let mesh =
        TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]).expect("valid mesh");
    let influences = (0..3)
        .map(|_| vec![BoneInfluence::new(0, 1.0).expect("valid influence")])
        .collect();
    let skinned = SkinnedTriangleMesh::new(
        mesh,
        vec![String::from("root")],
        influences,
        Affine3::IDENTITY,
        vec![Affine3::IDENTITY],
    )
    .expect("valid skinned mesh");
    let skinned_id = world
        .add_collider(
            body,
            Collider::new(Shape::SkinnedTriangleMesh(skinned), Transform::identity()),
        )
        .unwrap();
    world
        .set_skin_pose(
            skinned_id,
            &[Affine3::from_translation(Vec3::new(3.0, 4.0, 5.0))],
        )
        .unwrap();
    assert_eq!(
        world
            .collider(skinned_id)
            .unwrap()
            .world_bounds(&Transform::identity()),
        Ok(Some(
            crate::aabb::Aabb::new(Vec3::new(3.0, 4.0, 5.0), Vec3::new(4.0, 5.0, 5.0)).unwrap()
        ))
    );

    let sphere_id = world.add_collider(body, collider(1.0)).unwrap();
    assert_eq!(
        world.set_skin_pose(sphere_id, &[]),
        Err(CollisionError::InvalidSkinPose)
    );
    assert_eq!(
        world.set_skin_pose(skinned_id, &[]),
        Err(CollisionError::InvalidSkinPose)
    );
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
fn body_and_collider_ids_are_scoped_to_their_worlds() -> Result<(), WorldError> {
    let mut first_world = PhysicsWorld::new();
    let mut second_world = PhysicsWorld::new();
    let first_body = first_world.add_body(RigidBody::new_static(Transform::identity()))?;
    let second_body = second_world.add_body(RigidBody::new_static(Transform::identity()))?;
    let first_collider = first_world.add_collider(first_body, collider(1.0))?;
    let second_collider = second_world.add_collider(second_body, collider(2.0))?;

    assert_eq!(first_body.dense, second_body.dense);
    assert_ne!(first_body, second_body);
    assert_eq!(first_collider.dense, second_collider.dense);
    assert_ne!(first_collider, second_collider);
    assert_eq!(HashSet::from([first_body, second_body]).len(), 2);
    assert_eq!(HashSet::from([first_collider, second_collider]).len(), 2);

    assert!(second_world.body(first_body).is_none());
    assert!(second_world.body_mut(first_body).is_none());
    assert!(second_world.colliders_of(first_body).is_none());
    assert_eq!(
        second_world.add_collider(first_body, collider(3.0)),
        Err(WorldError::InvalidBodyId)
    );
    assert!(second_world.remove_body(first_body).is_none());
    assert_eq!(second_world.body_count(), 1);

    assert!(second_world.collider(first_collider).is_none());
    assert!(second_world.collider_mut(first_collider).is_none());
    assert!(second_world.collider_body(first_collider).is_none());
    assert!(second_world.remove_collider(first_collider).is_none());
    assert_eq!(second_world.collider_count(), 1);
    assert!(second_world.body(second_body).is_some());
    assert!(second_world.collider(second_collider).is_some());
    Ok(())
}

#[test]
fn material_ids_are_scoped_to_world_storage_and_in_use_materials_are_retained() {
    let mut first_world = PhysicsWorld::new();
    let mut second_world = PhysicsWorld::new();
    let material = PhysicsMaterial::default();
    let material_id = first_world.add_material(material).expect("insert material");
    let other_material_id = second_world
        .add_material(material)
        .expect("insert other world material");
    assert_ne!(material_id, other_material_id);
    assert_eq!(second_world.material(material_id), None);

    let body = first_world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("insert body");
    let collider_id = first_world
        .add_collider(body, collider(1.0))
        .expect("insert collider");
    assert_eq!(
        first_world.set_collider_material(collider_id, Some(other_material_id)),
        Err(WorldError::InvalidMaterialId)
    );
    first_world
        .set_collider_material(collider_id, Some(material_id))
        .expect("assign valid material");
    assert_eq!(
        first_world.remove_material(material_id),
        Err(WorldError::MaterialInUse)
    );
    assert_eq!(first_world.material(material_id), Some(&material));
    first_world
        .set_collider_material(collider_id, None)
        .expect("clear assignment");
    assert_eq!(first_world.remove_material(material_id), Ok(material));
    assert_eq!(
        first_world.remove_material(material_id),
        Err(WorldError::InvalidMaterialId)
    );
}

#[test]
fn dynamic_bodies_reject_surface_geometry_but_allow_convex_shapes() {
    let mut world = PhysicsWorld::new();
    let dynamic = world.add_body(dynamic_body()).expect("dynamic body");
    let triangle = Shape::Triangle(
        crate::shape::Triangle::new(Vec3::ZERO, Vec3::X, Vec3::Y).expect("triangle"),
    );
    assert_eq!(
        world.add_collider(dynamic, Collider::new(triangle, Transform::identity())),
        Err(WorldError::Collision(
            crate::collision::CollisionError::UnsupportedDynamicGeometry
        ))
    );
    assert!(world.add_collider(dynamic, collider(1.0)).is_ok());

    let static_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("static body");
    let triangle = Shape::Triangle(
        crate::shape::Triangle::new(Vec3::ZERO, Vec3::X, Vec3::Y).expect("triangle"),
    );
    assert!(
        world
            .add_collider(static_body, Collider::new(triangle, Transform::identity()))
            .is_ok()
    );
}

#[test]
fn invalid_material_fails_step_without_changing_body_or_forces() {
    let mut world = PhysicsWorld::new();
    let mut body = dynamic_body();
    body.apply_force(Vec3::X).expect("apply force");
    let body_id = world.add_body(body).expect("add body");
    let collider_id = world
        .add_collider(body_id, collider(1.0))
        .expect("add collider");
    let mut foreign_storage = crate::material::MaterialStorage::new();
    let foreign_material = foreign_storage
        .insert(PhysicsMaterial::default())
        .expect("add foreign material");
    world
        .collider_mut(collider_id)
        .expect("collider")
        .set_material(Some(foreign_material));
    let original = *world.body(body_id).expect("body");

    assert_eq!(
        world.step(0.1),
        Err(StepError::Collision(CollisionError::InvalidMaterial))
    );
    assert_eq!(world.body(body_id), Some(&original));
    assert!(world.collision_events().is_empty());
}

#[test]
fn invalid_material_fails_shard_commit_before_any_body_changes() {
    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(dynamic_body()).expect("add body");
    let collider_id = world
        .add_collider(body_id, collider(1.0))
        .expect("add collider");
    let mut foreign_storage = crate::material::MaterialStorage::new();
    let foreign_material = foreign_storage
        .insert(PhysicsMaterial::default())
        .expect("add foreign material");
    world
        .collider_mut(collider_id)
        .expect("collider")
        .set_material(Some(foreign_material));
    let original = *world.body(body_id).expect("body");
    let mut shards = vec![Vec::new()];
    world.fill_step_shards(&mut shards).expect("fill shards");
    shards[0][0].integrate(0.1).expect("predict body");

    assert_eq!(
        world.commit_step_shards(&shards),
        Err(StepError::Collision(CollisionError::InvalidMaterial))
    );
    assert_eq!(world.body(body_id), Some(&original));
}

#[test]
fn world_id_survives_moves_and_removing_all_bodies() -> Result<(), WorldError> {
    fn move_world(world: PhysicsWorld) -> PhysicsWorld {
        world
    }

    let mut world = PhysicsWorld::new();
    let first = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let world_id = world.world_id;
    world = move_world(world);
    assert!(world.body(first).is_some());
    assert!(world.remove_body(first).is_some());
    let second = world.add_body(RigidBody::new_static(Transform::identity()))?;

    assert_eq!(world.world_id, world_id);
    assert_eq!(second.world, world_id);
    assert!(world.body(first).is_none());
    assert!(world.body(second).is_some());
    Ok(())
}

#[test]
fn world_id_allocator_reports_exhaustion_without_wrapping() {
    let counter = AtomicU64::new(u64::MAX - 1);
    assert_eq!(allocate_world_id(&counter), Ok(u64::MAX - 1));
    assert_eq!(counter.load(std::sync::atomic::Ordering::Relaxed), u64::MAX);
    assert_eq!(
        allocate_world_id(&counter),
        Err(WorldError::WorldIdExhausted)
    );
    assert_eq!(counter.load(std::sync::atomic::Ordering::Relaxed), u64::MAX);
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
    let id = BodyId {
        world: u64::MAX,
        dense: source.insert(())?,
    };
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
    let stale_body = BodyId {
        world: u64::MAX,
        dense: stale_ids.insert(())?,
    };
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

    assert_eq!(stale.dense.index(), replacement.dense.index());
    assert_ne!(stale.dense.generation(), replacement.dense.generation());
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
    let id = ColliderId {
        world: u64::MAX,
        dense: source.insert(())?,
    };
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

    assert_eq!(old_body.dense.index(), new_body.dense.index());
    assert_ne!(old_body.dense.generation(), new_body.dense.generation());
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

#[test]
fn step_advances_each_body_type_and_clears_accumulated_forces() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let static_id = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let mut kinematic = RigidBody::new_kinematic(Transform::identity());
    kinematic
        .set_linear_velocity(Vec3::X)
        .expect("finite velocity");
    let kinematic_id = world.add_body(kinematic)?;

    let mut moving = dynamic_body();
    moving.apply_force(Vec3::X * 2.0).expect("finite force");
    let moving_id = world.add_body(moving)?;

    let mut sleeping = dynamic_body();
    sleeping
        .set_linear_velocity(Vec3::Y)
        .expect("finite velocity");
    sleeping.sleep().expect("dynamic body can sleep");
    let sleeping_id = world.add_body(sleeping)?;
    let sleeping_state = *world
        .body(sleeping_id)
        .ok_or(WorldError::InvalidBodyId)?
        .state();

    world.step(0.5).expect("valid physics step");

    assert_eq!(
        world
            .body(static_id)
            .ok_or(WorldError::InvalidBodyId)?
            .state()
            .transform(),
        Transform::identity()
    );
    let kinematic = world.body(kinematic_id).ok_or(WorldError::InvalidBodyId)?;
    assert_eq!(kinematic.state().transform().translation(), Vec3::X * 0.5);
    assert_eq!(kinematic.state().linear_velocity(), Vec3::X);

    let moving = world.body(moving_id).ok_or(WorldError::InvalidBodyId)?;
    assert_eq!(moving.state().linear_velocity(), Vec3::X);
    assert_eq!(moving.state().transform().translation(), Vec3::X * 0.5);
    assert_eq!(moving.forces().force(), Vec3::ZERO);
    assert_eq!(moving.forces().torque(), Vec3::ZERO);
    assert_eq!(
        world.body(sleeping_id).map(RigidBody::state),
        Some(&sleeping_state)
    );
    Ok(())
}

#[test]
fn invalid_timestep_preserves_all_body_state_and_forces() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let mut body = dynamic_body();
    body.set_linear_velocity(Vec3::X).expect("finite velocity");
    body.apply_force(Vec3::Y).expect("finite force");
    let id = world.add_body(body)?;
    let original = *world.body(id).ok_or(WorldError::InvalidBodyId)?;

    for dt in [0.0, -0.1, f64::NAN, f64::INFINITY] {
        assert_eq!(world.step(dt), Err(StepError::InvalidDeltaTime));
        assert_eq!(world.body(id), Some(&original));
    }
    Ok(())
}

#[test]
fn body_integration_failure_is_atomic_and_preserves_forces() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let mut first = dynamic_body();
    first.set_linear_velocity(Vec3::Y).expect("finite velocity");
    first.apply_force(Vec3::X).expect("finite force");
    let first_id = world.add_body(first)?;

    let mut failing = dynamic_body();
    failing
        .apply_force(Vec3::X * f64::MAX)
        .expect("finite force");
    let failing_id = world.add_body(failing)?;
    let first_original = *world.body(first_id).ok_or(WorldError::InvalidBodyId)?;
    let failing_original = *world.body(failing_id).ok_or(WorldError::InvalidBodyId)?;

    assert_eq!(
        world.step(2.0),
        Err(StepError::BodyIntegration {
            body_id: failing_id,
            cause: IntegrationError::ArithmeticOverflow,
        })
    );
    assert_eq!(world.body(first_id), Some(&first_original));
    assert_eq!(world.body(failing_id), Some(&failing_original));
    assert!(world.collision_system.context.predicted_states.is_empty());

    world
        .body_mut(failing_id)
        .ok_or(WorldError::InvalidBodyId)?
        .clear_forces();
    world
        .step(0.5)
        .expect("retry succeeds after clearing invalid force");
    assert_eq!(
        world
            .body(first_id)
            .ok_or(WorldError::InvalidBodyId)?
            .forces()
            .force(),
        Vec3::ZERO
    );
    Ok(())
}

#[test]
fn repeated_steps_are_deterministic_and_reuse_the_scratch_buffer() -> Result<(), WorldError> {
    fn make_world() -> Result<(PhysicsWorld, BodyId), WorldError> {
        let mut world = PhysicsWorld::new();
        let mut body = dynamic_body();
        body.set_linear_velocity(Vec3::new(1.0, -2.0, 0.5))
            .expect("finite velocity");
        body.set_angular_velocity(Vec3::new(0.25, -0.5, 0.75))
            .expect("finite velocity");
        body.apply_force(Vec3::new(2.0, 3.0, -1.0))
            .expect("finite force");
        body.apply_torque(Vec3::new(-0.5, 1.0, 2.0))
            .expect("finite torque");
        let id = world.add_body(body)?;
        Ok((world, id))
    }

    let (mut first, first_id) = make_world()?;
    let (mut second, second_id) = make_world()?;
    for _ in 0..8 {
        first.step(1.0 / 120.0).expect("valid step");
        second.step(1.0 / 120.0).expect("valid step");
        assert_eq!(first.body(first_id), second.body(second_id));
    }
    let capacity = first.collision_system.context.predicted_states.capacity();
    first.step(1.0 / 120.0).expect("valid step");
    assert_eq!(
        first.collision_system.context.predicted_states.capacity(),
        capacity
    );
    Ok(())
}

#[test]
fn ccd_swept_bounds_cover_offset_com_rotation_arcs() {
    let mass = MassProperties::new(1.0, Vec3::X, Mat3::IDENTITY).expect("mass properties");
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).expect("body");
    body.set_angular_velocity(Vec3::Z * (3.0 * std::f64::consts::FRAC_PI_2))
        .expect("angular velocity");
    let start = *body.state();
    let end = crate::rigid_body::predict(&body, 1.0).expect("integrator endpoint");
    let trajectory = super::ColliderTrajectory {
        body,
        base_state: start,
        base_time: 0.0,
        sample_start_time: 0.0,
        end_time: 1.0,
        local_transform: Transform::identity(),
    };
    let collider = Collider::new(
        Shape::Sphere(Sphere::new(0.1).expect("sphere")),
        Transform::identity(),
    );
    let bounds = super::swept_bounds_for_trajectory(&collider, &trajectory, end)
        .expect("trajectory bounds")
        .expect("finite bounds");
    let arc_sample = trajectory
        .transform_at(1.0 / 3.0)
        .expect("integrator trajectory sample");

    assert!(bounds.contains(arc_sample.translation()));
    assert!(bounds.contains(arc_sample.translation() + Vec3::Y * 0.1));
}

#[test]
fn skinned_mesh_bounds_track_pose_for_static_and_kinematic_bodies() -> Result<(), WorldError> {
    for body_type in [BodyType::Static, BodyType::Kinematic] {
        let mut world = PhysicsWorld::new();
        let mesh_body = match body_type {
            BodyType::Static => RigidBody::new_static(Transform::identity()),
            BodyType::Kinematic => RigidBody::new_kinematic(Transform::identity()),
            BodyType::Dynamic => unreachable!(),
        };
        let mesh_body_id = world.add_body(mesh_body)?;
        let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]])
            .expect("valid mesh");
        let influences = (0..3)
            .map(|_| vec![crate::shape::BoneInfluence::new(0, 1.0).expect("valid influence")])
            .collect();
        let skinned_mesh = SkinnedTriangleMesh::new(
            mesh,
            vec![String::from("root")],
            influences,
            Affine3::IDENTITY,
            vec![Affine3::IDENTITY],
        )
        .expect("valid skinned mesh");
        let mesh_collider = world.add_collider(
            mesh_body_id,
            Collider::new(
                Shape::SkinnedTriangleMesh(skinned_mesh),
                Transform::identity(),
            ),
        )?;
        let sphere_body = world.add_body(dynamic_body())?;
        let sphere_collider = world.add_collider(sphere_body, collider(0.5))?;

        for (pose_x, sphere_x) in [(10.0, 10.25), (20.0, 20.25)] {
            world
                .set_skin_pose(
                    mesh_collider,
                    &[Affine3::from_translation(Vec3::new(pose_x, 0.0, 0.0))],
                )
                .expect("valid skin pose");
            let sphere = world.body_mut(sphere_body).expect("sphere body");
            sphere
                .set_transform(
                    Transform::new(Vec3::new(sphere_x, 0.25, 0.4), kapitsa_math::Quat::IDENTITY)
                        .expect("valid transform"),
                )
                .expect("valid transform");
            sphere
                .set_linear_velocity(Vec3::ZERO)
                .expect("valid velocity");

            world.step(1.0 / 60.0).expect("contact step");

            let pair = crate::collision::CollisionPair::new(mesh_collider, sphere_collider)
                .expect("distinct colliders");
            assert!(
                (0..world.collision_system.candidate_count())
                    .filter_map(|index| world.collision_system.candidate(index))
                    .any(|candidate| candidate == pair),
                "broadphase missed the deformed mesh at x={pose_x} for {body_type:?}"
            );
            let event = world
                .collision_events()
                .iter()
                .find(|event| event.pair == pair)
                .expect("collision event");
            assert!(event.contact.is_some(), "narrowphase missed deformed mesh");
            assert!(
                world
                    .body(sphere_body)
                    .expect("sphere body")
                    .state()
                    .transform()
                    .translation()
                    .z
                    > 0.4,
                "solver did not separate the dynamic sphere"
            );
        }
    }
    Ok(())
}

#[test]
fn step_shards_are_balanced_and_commit_the_same_states() -> Result<(), Box<dyn std::error::Error>> {
    let mut parallel_world = PhysicsWorld::new();
    let mut sequential_world = PhysicsWorld::new();
    for index in 0..7 {
        let mut body = RigidBody::new_kinematic(Transform::identity());
        body.set_linear_velocity(Vec3::new(f64::from(index), 1.0, -1.0))?;
        parallel_world.add_body(body)?;
        sequential_world.add_body(body)?;
    }

    let mut shards = vec![Vec::new(); 3];
    parallel_world.fill_step_shards(&mut shards)?;
    assert_eq!(
        shards.iter().map(Vec::len).collect::<Vec<_>>(),
        vec![3, 2, 2]
    );
    for step in shards.iter_mut().flatten() {
        step.integrate(0.125)?;
    }
    assert_eq!(
        sequential_world.commit_step_shards(&shards),
        Err(StepError::InvalidStepResults)
    );
    parallel_world.commit_step_shards(&shards)?;
    sequential_world.step(0.125)?;

    for ((parallel_id, parallel), (sequential_id, sequential)) in
        parallel_world.bodies().zip(sequential_world.bodies())
    {
        assert_eq!(parallel_id.index(), sequential_id.index());
        assert_eq!(parallel_id.generation(), sequential_id.generation());
        assert_ne!(parallel_id, sequential_id);
        assert_eq!(parallel, sequential);
    }
    Ok(())
}

#[test]
fn incomplete_step_shards_do_not_change_the_world() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let first_id = world.add_body(dynamic_body())?;
    let second_id = world.add_body(dynamic_body())?;
    let original_first = *world.body(first_id).ok_or(WorldError::InvalidBodyId)?;
    let original_second = *world.body(second_id).ok_or(WorldError::InvalidBodyId)?;
    let mut shards = vec![Vec::new()];
    world.fill_step_shards(&mut shards)?;
    shards[0][0].integrate(0.1)?;

    assert_eq!(
        world.commit_step_shards(&shards),
        Err(StepError::InvalidStepResults)
    );
    assert_eq!(world.body(first_id), Some(&original_first));
    assert_eq!(world.body(second_id), Some(&original_second));
    Ok(())
}

#[test]
fn step_errors_expose_the_integration_cause() -> Result<(), WorldError> {
    let mut world = PhysicsWorld::new();
    let mut body = dynamic_body();
    body.apply_force(Vec3::X * f64::MAX).expect("finite force");
    world.add_body(body)?;

    let error = world.step(2.0).expect_err("integration overflows");
    assert!(error.to_string().contains("integration failed"));
    assert!(std::error::Error::source(&error).is_some());
    Ok(())
}
