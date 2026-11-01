use kapitsa_math::{Affine3, Mat3, Quat, Vec3};

use super::{
    BodyId, ColliderId, ForceSourceError, PhysicsWorld, StepError, WorldError, allocate_world_id,
    upsert_interaction,
};
use crate::{
    collider::{Collider, CollisionFilter, CollisionMode},
    collision::{
        CollisionError, CollisionInteraction, CollisionPair, ContactInfo, OverlapHit, QueryFilter,
        Ray, RayHit, ShapeHit,
    },
    containers::{DensePool, DensePoolError},
    mass::MassProperties,
    material::PhysicsMaterial,
    rigid_body::{BodyType, CcdMode, IntegrationError, RigidBody},
    shape::{BoneInfluence, Cuboid, Shape, SkinnedTriangleMesh, Sphere, TriangleMesh},
    transform::Transform,
};
use std::{collections::HashSet, sync::atomic::AtomicU64};

#[cfg(feature = "bench-internals")]
use std::alloc::{GlobalAlloc, Layout, System};
#[cfg(feature = "bench-internals")]
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
#[cfg(feature = "bench-internals")]
use std::time::Duration;

use crate::execution::{ExecutionError, StepExecutor};

struct ScopedThreadExecutor;

impl StepExecutor for ScopedThreadExecutor {
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
        if chunk_size == 0 {
            return Err(ExecutionError::InvalidChunkLayout);
        }
        std::thread::scope(|scope| {
            for (index, chunk) in values.chunks_mut(chunk_size).enumerate() {
                let operation = &operation;
                scope.spawn(move || operation(index, chunk));
            }
        });
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
        let total = chunk_sizes.iter().try_fold(0_usize, |total, &size| {
            (size > 0).then(|| total.checked_add(size)).flatten()
        });
        if total != Some(values.len()) {
            return Err(ExecutionError::InvalidChunkLayout);
        }

        std::thread::scope(|scope| {
            let mut remaining = values;
            let mut start = 0;
            for &chunk_size in chunk_sizes {
                let (chunk, tail) = remaining.split_at_mut(chunk_size);
                let operation = &operation;
                scope.spawn(move || operation(start, chunk));
                start += chunk_size;
                remaining = tail;
            }
        });
        Ok(())
    }
}

fn dynamic_body() -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
    RigidBody::new_dynamic(Transform::identity(), mass).expect("valid dynamic body")
}

fn collider(radius: f64) -> Collider {
    let sphere = Sphere::new(radius).expect("valid sphere");
    Collider::new(Shape::Sphere(sphere), Transform::identity())
}

fn independent_wall_ccd_world() -> Result<PhysicsWorld, Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let wall_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        wall_body,
        Collider::new(
            Shape::Cuboid(Cuboid::new(0.1, 100.0, 100.0)?),
            Transform::identity(),
        ),
    )?;
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    for index in 0..128 {
        let mut body = RigidBody::new_dynamic(
            Transform::new(
                Vec3::new(
                    -5.0,
                    f64::from(index % 16) * 3.0,
                    f64::from(index / 16) * 3.0,
                ),
                Quat::IDENTITY,
            )?,
            mass,
        )?;
        body.set_linear_velocity(Vec3::new(1000.0, 0.0, 0.0))?;
        body.set_ccd_mode(CcdMode::Continuous);
        let body = world.add_body(body)?;
        world.add_collider(body, collider(0.5))?;
    }
    Ok(world)
}

fn seeded_independent_ccd_world(mut seed: u64) -> Result<PhysicsWorld, Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let wall_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        wall_body,
        Collider::new(
            Shape::Cuboid(Cuboid::new(0.1, 100.0, 100.0)?),
            Transform::identity(),
        ),
    )?;
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    for index in 0..192 {
        let random = |state: &mut u64| {
            *state = (*state)
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (*state >> 32) as u32
        };
        let x_offset = f64::from(random(&mut seed) % 200) / 100.0;
        let speed = 1_200.0 + f64::from(random(&mut seed) % 600);
        let y = f64::from(index % 16) * 3.0;
        let z = f64::from(index / 16) * 3.0;
        let mut body = RigidBody::new_dynamic(
            Transform::new(Vec3::new(-6.0 - x_offset, y, z), Quat::IDENTITY)?,
            mass,
        )?;
        body.set_linear_velocity(Vec3::new(speed, 0.0, 0.0))?;
        body.set_ccd_mode(CcdMode::Continuous);
        let body = world.add_body(body)?;
        let mut collider = collider(0.5);
        if index % 11 == 0 {
            collider.set_filter(CollisionFilter::new(1, 0));
        }
        world.add_collider(body, collider)?;
    }
    Ok(world)
}

#[test]
fn interaction_index_preserves_duplicate_merge_rules_and_resets_with_step_scratch() {
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let colliders = (0..4)
        .map(|_| world.add_collider(body, collider(0.5)).unwrap())
        .collect::<Vec<_>>();
    let solid_pair = CollisionPair::new(colliders[0], colliders[1]).unwrap();
    let sensor_pair = CollisionPair::new(colliders[2], colliders[3]).unwrap();
    let mut interactions = Vec::new();
    let mut indices = None;
    let first_contact = ContactInfo {
        point: Vec3::X,
        normal: Vec3::X,
        penetration: 0.1,
        normal_impulse: 0.0,
    };
    let second_contact = ContactInfo {
        point: Vec3::Y,
        normal: Vec3::Y,
        penetration: 0.2,
        normal_impulse: 1.0,
    };

    upsert_interaction(
        &mut interactions,
        &mut indices,
        CollisionInteraction {
            pair: solid_pair,
            sensor: false,
            transient: false,
            contact: Some(first_contact),
            impact: false,
        },
    )
    .unwrap();
    upsert_interaction(
        &mut interactions,
        &mut indices,
        CollisionInteraction {
            pair: solid_pair,
            sensor: false,
            transient: false,
            contact: Some(second_contact),
            impact: true,
        },
    )
    .unwrap();
    assert_eq!(interactions.len(), 1);
    assert_eq!(
        indices.as_ref().and_then(|map| map.get(&solid_pair)),
        Some(&0)
    );
    assert_eq!(interactions[0].contact, Some(second_contact));
    assert!(interactions[0].impact);

    for transient in [true, false] {
        upsert_interaction(
            &mut interactions,
            &mut indices,
            CollisionInteraction {
                pair: sensor_pair,
                sensor: true,
                transient,
                contact: None,
                impact: false,
            },
        )
        .unwrap();
    }
    assert_eq!(interactions.len(), 2);
    assert_eq!(
        indices.as_ref().and_then(|map| map.get(&sensor_pair)),
        Some(&1)
    );
    assert!(!interactions[1].transient);

    assert_eq!(
        upsert_interaction(
            &mut interactions,
            &mut indices,
            CollisionInteraction {
                pair: solid_pair,
                sensor: true,
                transient: false,
                contact: None,
                impact: false,
            },
        ),
        Err(CollisionError::InvalidContact)
    );
    assert_eq!(interactions.len(), 2);

    let context = &mut world.collision_system.context;
    context.interactions = interactions;
    context.interaction_indices = indices;
    context.clear();
    assert!(context.interactions.is_empty());
    assert!(
        context
            .interaction_indices
            .as_ref()
            .is_none_or(|indices| indices.is_empty())
    );
}

fn world_query_results(
    world: &PhysicsWorld,
    ray: Ray,
    query_shape: &Shape,
    query_transform: Transform,
    displacement: Vec3,
    filter: QueryFilter,
) -> (Option<RayHit>, Option<ShapeHit>, Vec<OverlapHit>) {
    let ray_hit = world.raycast(ray, filter).unwrap();
    let sweep_hit = world
        .sweep(query_shape, query_transform, displacement, filter)
        .unwrap();
    let mut overlaps = Vec::new();
    world
        .overlaps(query_shape, query_transform, filter, &mut overlaps)
        .unwrap();
    (ray_hit, sweep_hit, overlaps)
}

fn assert_world_queries_match_full_scan(
    world: &mut PhysicsWorld,
    ray: Ray,
    query_shape: &Shape,
    query_transform: Transform,
    displacement: Vec3,
    filter: QueryFilter,
) {
    if world.query_index_dirty {
        let scanned = world_query_results(
            world,
            ray,
            query_shape,
            query_transform,
            displacement,
            filter,
        );
        world.rebuild_query_index();
        let indexed = world_query_results(
            world,
            ray,
            query_shape,
            query_transform,
            displacement,
            filter,
        );
        assert_eq!(indexed, scanned);
    } else {
        let indexed = world_query_results(
            world,
            ray,
            query_shape,
            query_transform,
            displacement,
            filter,
        );
        world.mark_query_index_dirty();
        let scanned = world_query_results(
            world,
            ray,
            query_shape,
            query_transform,
            displacement,
            filter,
        );
        assert_eq!(indexed, scanned);
        world.rebuild_query_index();
    }
}

#[test]
fn committed_query_index_matches_full_scan_across_queries_and_world_mutations() {
    let mut world = PhysicsWorld::new();
    let plane_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let plane = world
        .add_collider(
            plane_body,
            Collider::new(
                Shape::Plane(crate::shape::Plane::new(0.0, 1.0, 0.0, 0.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    let tie_transform = Transform::new(Vec3::new(5.0, 2.0, 0.0), Quat::IDENTITY).unwrap();
    let first_body = world
        .add_body(RigidBody::new_static(tie_transform))
        .unwrap();
    let first = world.add_collider(first_body, collider(0.5)).unwrap();
    let second_body = world
        .add_body(RigidBody::new_static(tie_transform))
        .unwrap();
    let second = world.add_collider(second_body, collider(0.5)).unwrap();

    let sensor_body = world
        .add_body(RigidBody::new_static(
            Transform::new(Vec3::new(2.0, 2.0, 0.0), Quat::IDENTITY).unwrap(),
        ))
        .unwrap();
    let mut sensor_collider = collider(0.5);
    sensor_collider.set_mode(CollisionMode::Sensor);
    let sensor = world.add_collider(sensor_body, sensor_collider).unwrap();

    let moving_body = world
        .add_body(
            RigidBody::new_dynamic(
                Transform::new(Vec3::new(20.0, 20.0, 0.0), Quat::IDENTITY).unwrap(),
                MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
            )
            .unwrap(),
        )
        .unwrap();
    world
        .body_mut(moving_body)
        .unwrap()
        .set_linear_velocity(Vec3::X * 10.0)
        .unwrap();
    let moving = world.add_collider(moving_body, collider(0.5)).unwrap();

    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]).unwrap();
    let skinned = SkinnedTriangleMesh::new(
        mesh,
        vec![String::from("root")],
        (0..3)
            .map(|_| vec![BoneInfluence::new(0, 1.0).unwrap()])
            .collect(),
        Affine3::IDENTITY,
        vec![Affine3::IDENTITY],
    )
    .unwrap();
    let mesh_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let mesh_collider = world
        .add_collider(
            mesh_body,
            Collider::new(Shape::SkinnedTriangleMesh(skinned), Transform::identity()),
        )
        .unwrap();

    let ray = Ray::new(Vec3::new(0.0, 2.0, 0.0), Vec3::X, 30.0).unwrap();
    let query_shape = Shape::Sphere(Sphere::new(0.25).unwrap());
    let query_transform = Transform::new(Vec3::new(5.0, 2.0, 0.0), Quat::IDENTITY).unwrap();
    let filter = QueryFilter::default();
    assert_world_queries_match_full_scan(
        &mut world,
        ray,
        &query_shape,
        query_transform,
        Vec3::X * 10.0,
        filter,
    );
    assert_eq!(
        world.raycast(ray, filter).unwrap().unwrap().collider,
        first.min(second)
    );

    let include_sensors = QueryFilter {
        include_sensors: true,
        ..filter
    };
    assert_world_queries_match_full_scan(
        &mut world,
        ray,
        &query_shape,
        query_transform,
        Vec3::X * 10.0,
        include_sensors,
    );
    assert_eq!(
        world
            .raycast(ray, include_sensors)
            .unwrap()
            .unwrap()
            .collider,
        sensor
    );
    let plane_ray = Ray::new(Vec3::new(0.0, 5.0, 0.0), -Vec3::Y, 10.0).unwrap();
    assert_eq!(
        world.raycast(plane_ray, filter).unwrap().unwrap().collider,
        plane
    );

    world.remove_collider(first).unwrap();
    let replacement_body = world
        .add_body(RigidBody::new_static(tie_transform))
        .unwrap();
    let replacement = world.add_collider(replacement_body, collider(0.5)).unwrap();
    assert_ne!(replacement, first);
    assert_world_queries_match_full_scan(
        &mut world,
        ray,
        &query_shape,
        query_transform,
        Vec3::X * 10.0,
        filter,
    );
    assert_eq!(
        world.raycast(ray, filter).unwrap().unwrap().collider,
        replacement
    );

    world
        .body_mut(replacement_body)
        .unwrap()
        .set_transform(Transform::new(Vec3::new(12.0, 2.0, 0.0), Quat::IDENTITY).unwrap())
        .unwrap();
    assert_world_queries_match_full_scan(
        &mut world,
        ray,
        &query_shape,
        query_transform,
        Vec3::X * 20.0,
        filter,
    );

    world
        .collider_mut(second)
        .unwrap()
        .set_filter(CollisionFilter::new(1, 0));
    assert_world_queries_match_full_scan(
        &mut world,
        ray,
        &query_shape,
        query_transform,
        Vec3::X * 20.0,
        filter,
    );

    world
        .set_skin_pose(
            mesh_collider,
            &[Affine3::from_translation(Vec3::new(50.0, 0.0, 0.0))],
        )
        .unwrap();
    let skin_query_transform = Transform::new(Vec3::new(50.2, 0.2, 0.0), Quat::IDENTITY).unwrap();
    assert_world_queries_match_full_scan(
        &mut world,
        ray,
        &query_shape,
        skin_query_transform,
        Vec3::ZERO,
        filter,
    );
    let mut skin_overlaps = Vec::new();
    world
        .overlaps(
            &query_shape,
            skin_query_transform,
            filter,
            &mut skin_overlaps,
        )
        .unwrap();
    assert!(
        skin_overlaps
            .iter()
            .any(|hit| hit.collider == mesh_collider)
    );

    world.step(0.001).unwrap();
    let small_move_ray = Ray::new(Vec3::new(0.0, 20.0, 0.0), Vec3::X, 30.0).unwrap();
    assert_world_queries_match_full_scan(
        &mut world,
        small_move_ray,
        &query_shape,
        Transform::new(Vec3::new(20.01, 20.0, 0.0), Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        filter,
    );
    assert_eq!(
        world
            .raycast(small_move_ray, filter)
            .unwrap()
            .unwrap()
            .collider,
        moving
    );

    world.step(0.1).unwrap();
    let moved_ray = Ray::new(Vec3::new(0.0, 20.0, 0.0), Vec3::X, 30.0).unwrap();
    assert_world_queries_match_full_scan(
        &mut world,
        moved_ray,
        &query_shape,
        Transform::new(Vec3::new(21.0, 20.0, 0.0), Quat::IDENTITY).unwrap(),
        Vec3::ZERO,
        filter,
    );
    assert_eq!(
        world.raycast(moved_ray, filter).unwrap().unwrap().collider,
        moving
    );
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
fn add_collider_rejects_a_material_from_another_world_before_insertion() {
    let mut world = PhysicsWorld::new();
    let mut other_world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body");
    let foreign_material = other_world
        .add_material(PhysicsMaterial::default())
        .expect("foreign material");
    let mut prepared = collider(1.0);
    prepared.set_material(Some(foreign_material));

    assert_eq!(
        world.add_collider(body, prepared),
        Err(WorldError::InvalidMaterialId)
    );
    assert_eq!(world.collider_count(), 0);

    let material = world
        .add_material(PhysicsMaterial::default())
        .expect("world material");
    let mut prepared = collider(1.0);
    prepared.set_material(Some(material));
    let id = world.add_collider(body, prepared).expect("valid material");
    assert_eq!(
        world.collider(id).expect("inserted collider").material(),
        Some(material)
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
fn persistent_force_source_uses_current_body_rotation_each_step() {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(dynamic_body()).expect("body");
    let source = world
        .add_force_source(body, Vec3::ZERO, Vec3::X)
        .expect("source");

    world.step(0.1).expect("first step");
    let rotation =
        Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2).expect("valid rotation");
    world
        .body_mut(body)
        .expect("body")
        .set_transform(Transform::new(Vec3::ZERO, rotation).expect("valid transform"))
        .expect("rotate body");
    world.step(0.1).expect("second step");

    let body = world.body(body).expect("body");
    assert!((body.state().linear_velocity().x - 0.1).abs() < 1.0e-12);
    assert!((body.state().linear_velocity().y - 0.1).abs() < 1.0e-12);
    assert_eq!(
        world.force_source(source).map(|source| source.is_enabled()),
        Some(true)
    );
}

#[test]
fn persistent_force_source_applies_torque_from_its_local_point() {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(dynamic_body()).expect("body");
    world
        .add_force_source(body, Vec3::Y, Vec3::X)
        .expect("source");

    world.step(0.1).expect("physics step");

    let body = world.body(body).expect("body");
    assert_eq!(body.state().linear_velocity(), Vec3::X * 0.1);
    assert_eq!(body.state().angular_velocity(), Vec3::Z * -0.1);
}

#[test]
fn indexed_force_sources_preserve_first_touch_and_accumulation_order() {
    let mut world = PhysicsWorld::new();
    let bodies = (0..=128)
        .map(|_| world.add_body(dynamic_body()).expect("body"))
        .collect::<Vec<_>>();
    let sources = bodies
        .iter()
        .map(|body| {
            world
                .add_force_source(*body, Vec3::ZERO, Vec3::ZERO)
                .expect("source")
        })
        .collect::<Vec<_>>();

    for force in [Vec3::X * 1.0e16, Vec3::X * -1.0e16, Vec3::X]
        .into_iter()
        .chain(std::iter::repeat_n(Vec3::X * 1.0e-3, 127))
    {
        world
            .add_force_source(bodies[0], Vec3::ZERO, force)
            .expect("repeated source");
    }

    world.step(1.0).expect("indexed mixed-source step");
    let velocity = world
        .body(bodies[0])
        .expect("first body")
        .state()
        .linear_velocity()
        .x;
    assert!((velocity - 1.127).abs() < 1.0e-10);
    assert!(bodies[1..].iter().all(|body| {
        world
            .body(*body)
            .is_some_and(|body| body.state().linear_velocity() == Vec3::ZERO)
    }));

    let stale_source = sources[1];
    assert!(world.remove_body(bodies[1]).is_some());
    assert_eq!(world.force_source(stale_source), None);
    assert_eq!(
        world.add_force_source(bodies[1], Vec3::ZERO, Vec3::X),
        Err(ForceSourceError::InvalidBodyId)
    );
    let replacement = world.add_body(dynamic_body()).expect("replacement body");
    assert_eq!(replacement.index(), bodies[1].index());
    assert_ne!(replacement.generation(), bodies[1].generation());
    world
        .add_force_source(replacement, Vec3::ZERO, Vec3::Y)
        .expect("replacement source");
    world.step(0.1).expect("step after slot reuse");
    assert_eq!(
        world
            .body(replacement)
            .expect("replacement")
            .state()
            .linear_velocity(),
        Vec3::Y * 0.1
    );
}

#[test]
fn indexed_force_source_preparation_errors_do_not_commit_staged_bodies() {
    let mut world = PhysicsWorld::new();
    let bodies = (0..=128)
        .map(|_| world.add_body(dynamic_body()).expect("body"))
        .collect::<Vec<_>>();
    for body in &bodies {
        world
            .add_force_source(*body, Vec3::ZERO, Vec3::X * 0.25)
            .expect("source");
    }
    let stale_body = world.add_body(dynamic_body()).expect("stale body");
    assert!(world.remove_body(stale_body).is_some());
    let replacement = world.add_body(dynamic_body()).expect("replacement body");
    let invalid_source = world
        .add_force_source(replacement, Vec3::ZERO, Vec3::Y)
        .expect("source to corrupt for test");
    world
        .force_sources
        .get_mut(invalid_source.dense)
        .expect("force source")
        .body = stale_body;
    let originals = bodies
        .iter()
        .map(|body| (*body, *world.body(*body).expect("body")))
        .collect::<Vec<_>>();
    let replacement_original = *world.body(replacement).expect("replacement");

    assert_eq!(
        world.prepare_force_source_updates(),
        Err((invalid_source, ForceSourceError::InvalidBodyId))
    );
    assert!(
        originals
            .iter()
            .all(|(id, original)| world.body(*id) == Some(original))
    );
    assert_eq!(world.body(replacement), Some(&replacement_original));
    assert!(world.force_source_body_updates.is_empty());
    assert!(
        world
            .force_source_body_indices
            .as_ref()
            .is_some_and(|indices| indices.is_empty())
    );
}

#[test]
fn indexed_force_source_step_failure_rolls_back_and_keeps_capacity() {
    let mut world = PhysicsWorld::new();
    let bodies = (0..=128)
        .map(|_| world.add_body(dynamic_body()).expect("body"))
        .collect::<Vec<_>>();
    for body in &bodies {
        world
            .add_force_source(*body, Vec3::ZERO, Vec3::X * 1.0e-3)
            .expect("source");
    }
    let overflow_source = world
        .add_force_source(bodies[0], Vec3::ZERO, Vec3::X * f64::MAX)
        .expect("overflow source");
    let originals = bodies
        .iter()
        .map(|body| (*body, *world.body(*body).expect("body")))
        .collect::<Vec<_>>();

    assert!(matches!(
        world.step(2.0),
        Err(StepError::BodyIntegration {
            body_id,
            cause: IntegrationError::ArithmeticOverflow,
        }) if body_id == bodies[0]
    ));
    assert!(
        originals
            .iter()
            .all(|(id, original)| world.body(*id) == Some(original))
    );
    assert!(world.force_source_body_updates.is_empty());
    let index_capacity = world
        .force_source_body_indices
        .as_ref()
        .map_or(0, |indices| indices.capacity());
    assert!(index_capacity > 0);

    world
        .remove_force_source(overflow_source)
        .expect("remove overflow source");
    world.step(0.1).expect("retry after rollback");
    assert!(
        world
            .force_source_body_indices
            .as_ref()
            .is_some_and(|indices| indices.is_empty())
    );
    assert_eq!(
        world
            .body(bodies[0])
            .expect("first body")
            .state()
            .linear_velocity(),
        Vec3::X * 1.0e-4
    );
}

#[test]
fn body_removal_invalidates_its_force_sources_and_disable_stops_force() {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(dynamic_body()).expect("body");
    let source = world
        .add_force_source(body, Vec3::ZERO, Vec3::X)
        .expect("source");

    world
        .set_force_source_enabled(source, false)
        .expect("disable source");
    world.step(0.1).expect("step without source");
    assert_eq!(
        world.body(body).expect("body").state().linear_velocity(),
        Vec3::ZERO
    );
    world
        .set_force_source_enabled(source, true)
        .expect("enable source");
    world.step(0.1).expect("step with source");
    assert_eq!(
        world.body(body).expect("body").state().linear_velocity(),
        Vec3::X * 0.1
    );

    assert!(world.remove_body(body).is_some());
    assert_eq!(world.force_source(source), None);
    assert_eq!(
        world.remove_force_source(source),
        Err(ForceSourceError::InvalidForceSourceId)
    );
}

#[test]
fn force_source_setup_rejects_static_bodies_and_non_finite_values() {
    let mut world = PhysicsWorld::new();
    let static_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body");
    assert_eq!(
        world.add_force_source(static_body, Vec3::ZERO, Vec3::X),
        Err(ForceSourceError::BodyMustBeDynamic)
    );

    let dynamic = world.add_body(dynamic_body()).expect("dynamic body");
    assert_eq!(
        world.add_force_source(dynamic, Vec3::ZERO, Vec3::splat(f64::NAN)),
        Err(ForceSourceError::NonFiniteValue)
    );
}

#[test]
fn force_source_step_error_preserves_existing_body_forces() {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(dynamic_body()).expect("body");
    world
        .body_mut(body)
        .expect("body")
        .apply_force(Vec3::new(1.0e308, 0.0, 0.0))
        .expect("initial finite force");
    let source = world
        .add_force_source(body, Vec3::ZERO, Vec3::new(1.0e308, 0.0, 0.0))
        .expect("source");

    assert_eq!(
        world.step(0.1),
        Err(StepError::ForceSource {
            source_id: source,
            cause: ForceSourceError::ArithmeticOverflow,
        })
    );
    assert_eq!(
        world.body(body).expect("body").forces().force(),
        Vec3::new(1.0e308, 0.0, 0.0)
    );
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
    let trajectory =
        super::CcdTrajectory::new(body, start, 0.0, 0.0, 1.0, Transform::identity(), false);
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
fn independent_single_pair_ccd_islands_resolve_across_workers()
-> Result<(), Box<dyn std::error::Error>> {
    let dt = 0.01;
    let mut sequential = independent_wall_ccd_world()?;
    sequential.step(dt)?;
    let expected_impacts = sequential
        .collision_events()
        .iter()
        .filter(|event| event.kind == crate::collision::CollisionEventKind::Impact)
        .count();
    assert_eq!(expected_impacts, 128);

    for workers in [2, 4] {
        let mut world = independent_wall_ccd_world()?;
        let mut shards = vec![Vec::new(); workers];
        world.fill_step_shards(&mut shards)?;
        for shard in &mut shards {
            for step in shard {
                step.integrate(dt)?;
            }
        }
        world.commit_step_shards_with_executor(&shards, &ScopedThreadExecutor, workers)?;

        for ((_, actual), (_, expected)) in world.bodies().zip(sequential.bodies()) {
            assert_eq!(
                actual.state().transform().translation(),
                expected.state().transform().translation()
            );
            assert_eq!(
                actual.state().linear_velocity(),
                expected.state().linear_velocity()
            );
        }
        let actual_impacts = world
            .collision_events()
            .iter()
            .filter(|event| event.kind == crate::collision::CollisionEventKind::Impact)
            .count();
        assert_eq!(actual_impacts, expected_impacts);
        for (actual, expected) in world
            .collision_events()
            .iter()
            .zip(sequential.collision_events())
        {
            assert_eq!(actual.kind, expected.kind);
            assert_eq!(actual.pair.a.dense.index(), expected.pair.a.dense.index());
            assert_eq!(actual.pair.b.dense.index(), expected.pair.b.dense.index());
            assert_eq!(actual.contact, expected.contact);
        }
        #[cfg(feature = "bench-internals")]
        assert!(
            world.collision_step_metrics().ccd_parallel_island_count >= workers,
            "the CCD island resolver should launch work for this scene"
        );
    }
    Ok(())
}

#[test]
fn seeded_independent_ccd_stress_matches_across_worker_counts()
-> Result<(), Box<dyn std::error::Error>> {
    const SEED: u64 = 0x5eed_c0de_2026;
    const DT: f64 = 1.0 / 60.0;

    let mut sequential = seeded_independent_ccd_world(SEED)?;
    sequential.step(DT)?;
    let expected_impacts = sequential
        .collision_events()
        .iter()
        .filter(|event| event.kind == crate::collision::CollisionEventKind::Impact)
        .count();
    assert!(expected_impacts >= 150);

    for workers in [1, 2, 4] {
        let mut world = seeded_independent_ccd_world(SEED)?;
        let mut shards = vec![Vec::new(); workers];
        world.fill_step_shards(&mut shards)?;
        for shard in &mut shards {
            for step in shard {
                step.integrate(DT)?;
            }
        }
        world.commit_step_shards_with_executor(&shards, &ScopedThreadExecutor, workers)?;

        for ((_, actual), (_, expected)) in world.bodies().zip(sequential.bodies()) {
            assert!(actual.state().transform().translation().is_finite());
            assert!(actual.state().linear_velocity().is_finite());
            assert!(actual.state().angular_velocity().is_finite());
            assert_eq!(
                actual.state().transform().translation(),
                expected.state().transform().translation()
            );
            assert_eq!(
                actual.state().linear_velocity(),
                expected.state().linear_velocity()
            );
        }
        assert_eq!(
            world.collision_events().len(),
            sequential.collision_events().len()
        );
        for (actual, expected) in world
            .collision_events()
            .iter()
            .zip(sequential.collision_events())
        {
            assert_eq!(actual.kind, expected.kind);
            assert_eq!(actual.pair.a.dense.index(), expected.pair.a.dense.index());
            assert_eq!(actual.pair.b.dense.index(), expected.pair.b.dense.index());
            assert_eq!(actual.contact, expected.contact);
        }
        #[cfg(feature = "bench-internals")]
        if workers > 1 {
            assert!(world.collision_step_metrics().ccd_parallel_island_count >= workers);
        }
    }
    Ok(())
}

#[test]
fn parallel_ccd_abort_does_not_publish_completed_islands() -> Result<(), Box<dyn std::error::Error>>
{
    const DT: f64 = 0.02;

    let mut world = PhysicsWorld::new();
    let restitution = world.add_material(PhysicsMaterial::new(0.0, 0.0, 1.0, 1.0)?)?;
    let front_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let front_wall = world.add_collider(
        front_body,
        Collider::new(
            Shape::Cuboid(Cuboid::new(0.1, 100.0, 100.0)?),
            Transform::identity(),
        ),
    )?;
    let rear_body = world.add_body(RigidBody::new_static(Transform::new(
        Vec3::new(-10.0, -3.0, 0.0),
        Quat::IDENTITY,
    )?))?;
    let rear_wall = world.add_collider(
        rear_body,
        Collider::new(
            Shape::Cuboid(Cuboid::new(0.1, 1.0, 1.0)?),
            Transform::identity(),
        ),
    )?;
    world.set_collider_material(front_wall, Some(restitution))?;
    world.set_collider_material(rear_wall, Some(restitution))?;

    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let mut body_ids = Vec::new();
    for index in 0..128 {
        let mut body = RigidBody::new_dynamic(
            Transform::new(
                Vec3::new(
                    -5.0,
                    3.0 + f64::from(index % 16) * 3.0,
                    f64::from(index / 16) * 3.0,
                ),
                Quat::IDENTITY,
            )?,
            mass,
        )?;
        body.set_linear_velocity(Vec3::new(1_200.0, 0.0, 0.0))?;
        body.set_ccd_mode(CcdMode::Continuous);
        let id = world.add_body(body)?;
        world.add_collider(id, collider(0.5))?;
        body_ids.push(id);
    }

    let mut rebound = RigidBody::new_dynamic(
        Transform::new(Vec3::new(-5.0, -3.0, 0.0), Quat::IDENTITY)?,
        mass,
    )?;
    rebound.set_linear_velocity(Vec3::new(1_200.0, 0.0, 0.0))?;
    rebound.set_ccd_mode(CcdMode::Continuous);
    let rebound_id = world.add_body(rebound)?;
    world.add_collider(rebound_id, collider(0.5))?;
    body_ids.push(rebound_id);

    world.set_collision_config(crate::collision::CollisionConfig {
        max_ccd_impacts: 1,
        ccd_limit_policy: crate::collision::CcdLimitPolicy::AbortStep,
        ..world.collision_config()
    })?;
    let before = body_ids
        .iter()
        .map(|&id| {
            world
                .body(id)
                .map(|body| *body.state())
                .ok_or(WorldError::InvalidBodyId)
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut shards = vec![Vec::new(); 4];
    world.fill_step_shards(&mut shards)?;
    for shard in &mut shards {
        for step in shard {
            step.integrate(DT)?;
        }
    }
    assert_eq!(
        world.commit_step_shards_with_executor(&shards, &ScopedThreadExecutor, 4),
        Err(StepError::Collision(CollisionError::CcdLimitExceeded))
    );

    for (&id, expected) in body_ids.iter().zip(before) {
        assert_eq!(
            world.body(id).ok_or(WorldError::InvalidBodyId)?.state(),
            &expected
        );
    }
    assert!(world.collision_events().is_empty());
    #[cfg(feature = "bench-internals")]
    assert!(world.collision_step_metrics().ccd_parallel_island_count >= 4);
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

#[cfg(feature = "bench-internals")]
static TRACK_WORLD_ALLOCATIONS: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "bench-internals")]
static WORLD_ALLOCATION_COUNT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "bench-internals")]
static WORLD_ALLOCATION_BYTES: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "bench-internals")]
struct WorldTrackingAllocator;

#[cfg(feature = "bench-internals")]
// SAFETY: all allocation operations delegate to the system allocator.
unsafe impl GlobalAlloc for WorldTrackingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() && TRACK_WORLD_ALLOCATIONS.load(Ordering::Relaxed) {
            WORLD_ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            WORLD_ALLOCATION_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() && TRACK_WORLD_ALLOCATIONS.load(Ordering::Relaxed) {
            WORLD_ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            WORLD_ALLOCATION_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let pointer = unsafe { System.realloc(pointer, layout, size) };
        if !pointer.is_null() && TRACK_WORLD_ALLOCATIONS.load(Ordering::Relaxed) {
            WORLD_ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            WORLD_ALLOCATION_BYTES.fetch_add(size, Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) };
    }
}

#[cfg(feature = "bench-internals")]
#[global_allocator]
static WORLD_TEST_ALLOCATOR: WorldTrackingAllocator = WorldTrackingAllocator;

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy)]
enum ForceSourceBenchScenario {
    OneBody,
    DistinctBodies,
    MixedBodies,
}

#[cfg(feature = "bench-internals")]
impl ForceSourceBenchScenario {
    const ALL: [Self; 3] = [Self::OneBody, Self::DistinctBodies, Self::MixedBodies];

    fn name(self) -> &'static str {
        match self {
            Self::OneBody => "one-body",
            Self::DistinctBodies => "distinct-bodies",
            Self::MixedBodies => "mixed-unbalanced",
        }
    }
}

#[cfg(feature = "bench-internals")]
fn force_source_bench_world(
    scenario: ForceSourceBenchScenario,
    source_count: usize,
) -> PhysicsWorld {
    let body_count = match scenario {
        ForceSourceBenchScenario::OneBody => 1,
        ForceSourceBenchScenario::DistinctBodies => source_count,
        ForceSourceBenchScenario::MixedBodies => (source_count / 8).max(2),
    };
    let mut world = PhysicsWorld::new();
    let mut bodies = Vec::with_capacity(body_count);
    let body_template = dynamic_body();
    for _ in 0..body_count {
        bodies.push(world.add_body(body_template).expect("benchmark body"));
    }

    let dominant_source_count = source_count.saturating_sub(body_count - 1);
    for source_index in 0..source_count {
        let body_index = match scenario {
            ForceSourceBenchScenario::OneBody => 0,
            ForceSourceBenchScenario::DistinctBodies => source_index,
            ForceSourceBenchScenario::MixedBodies => {
                if source_index < dominant_source_count {
                    0
                } else {
                    source_index - dominant_source_count + 1
                }
            }
        };
        let scale = (source_index % 5 + 1) as f64 * 1.0e-6;
        world
            .add_force_source(
                bodies[body_index],
                Vec3::Y * 0.01,
                Vec3::new(scale, scale * 2.0, -scale),
            )
            .expect("benchmark force source");
    }
    world
}

#[cfg(feature = "bench-internals")]
fn prepare_and_restore_force_sources(world: &mut PhysicsWorld) {
    world
        .prepare_force_source_updates()
        .expect("benchmark source preparation");
    let digest = world
        .force_source_body_updates
        .iter()
        .map(|update| update.prepared.forces().force().x)
        .sum::<f64>();
    std::hint::black_box(digest);
    world.rollback_force_source_updates();
}

#[cfg(feature = "bench-internals")]
fn calibrated_force_source_repeats(pilot: Duration) -> usize {
    const TARGET: Duration = Duration::from_millis(5);
    usize::try_from(TARGET.as_nanos() / pilot.as_nanos().max(1))
        .unwrap_or(5_000)
        .clamp(1, 5_000)
}

#[cfg(feature = "bench-internals")]
fn tracked_world_allocations(mut operation: impl FnMut()) -> (usize, usize) {
    WORLD_ALLOCATION_COUNT.store(0, Ordering::Relaxed);
    WORLD_ALLOCATION_BYTES.store(0, Ordering::Relaxed);
    TRACK_WORLD_ALLOCATIONS.store(true, Ordering::Relaxed);
    operation();
    TRACK_WORLD_ALLOCATIONS.store(false, Ordering::Relaxed);
    (
        WORLD_ALLOCATION_COUNT.load(Ordering::Relaxed),
        WORLD_ALLOCATION_BYTES.load(Ordering::Relaxed),
    )
}

#[cfg(feature = "bench-internals")]
fn force_source_preparation_samples(
    scenario: ForceSourceBenchScenario,
    source_count: usize,
) -> ([f64; 5], usize, usize, usize, usize, usize) {
    let mut pilot = force_source_bench_world(scenario, source_count);
    prepare_and_restore_force_sources(&mut pilot);
    let started = std::time::Instant::now();
    let result = pilot.prepare_force_source_updates();
    let pilot_time = started.elapsed();
    result.expect("pilot source preparation");
    pilot.rollback_force_source_updates();
    let repeats = calibrated_force_source_repeats(pilot_time);

    let mut samples = [0.0; 5];
    let mut update_capacity = 0;
    let mut index_capacity = 0;
    for sample in &mut samples {
        let mut world = force_source_bench_world(scenario, source_count);
        prepare_and_restore_force_sources(&mut world);
        let mut elapsed = Duration::ZERO;
        for _ in 0..repeats {
            let started = std::time::Instant::now();
            let result = world.prepare_force_source_updates();
            elapsed += started.elapsed();
            result.expect("benchmark source preparation");
            let digest = world
                .force_source_body_updates
                .iter()
                .map(|update| update.prepared.forces().force().x)
                .sum::<f64>();
            std::hint::black_box(digest);
            world.rollback_force_source_updates();
        }
        *sample = elapsed.as_nanos() as f64 / repeats as f64;
        update_capacity = world.force_source_body_updates.capacity();
        index_capacity = world
            .force_source_body_indices
            .as_ref()
            .map_or(0, |indices| indices.capacity());
    }
    samples.sort_by(f64::total_cmp);

    let mut allocation_world = force_source_bench_world(scenario, source_count);
    prepare_and_restore_force_sources(&mut allocation_world);
    let (allocations, bytes) = tracked_world_allocations(|| {
        for _ in 0..32 {
            prepare_and_restore_force_sources(&mut allocation_world);
        }
    });
    (
        samples,
        repeats,
        allocations / 32,
        bytes / 32,
        update_capacity,
        index_capacity,
    )
}

#[cfg(feature = "bench-internals")]
fn full_force_source_step_samples(
    scenario: ForceSourceBenchScenario,
    source_count: usize,
) -> ([f64; 5], usize, usize, usize) {
    const DT: f64 = 1.0 / 60.0;
    let mut pilot = force_source_bench_world(scenario, source_count);
    pilot.step(DT).expect("warm full step");
    let started = std::time::Instant::now();
    pilot.step(DT).expect("pilot full step");
    let repeats = calibrated_force_source_repeats(started.elapsed());

    let mut samples = [0.0; 5];
    for sample in &mut samples {
        let mut world = force_source_bench_world(scenario, source_count);
        world.step(DT).expect("warm full step");
        let started = std::time::Instant::now();
        for _ in 0..repeats {
            world.step(DT).expect("benchmark full step");
        }
        *sample = started.elapsed().as_nanos() as f64 / repeats as f64;
        std::hint::black_box(world.body(world.bodies().next().expect("body").0));
    }
    samples.sort_by(f64::total_cmp);

    let mut allocation_world = force_source_bench_world(scenario, source_count);
    allocation_world.step(DT).expect("warm full step");
    let (allocations, bytes) = tracked_world_allocations(|| {
        for _ in 0..32 {
            allocation_world
                .step(DT)
                .expect("benchmark full step allocation count");
        }
    });
    (samples, repeats, allocations / 32, bytes / 32)
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Release scaling benchmark; run with bench-internals"]
fn benchmark_force_source_update_scaling() {
    const SOURCE_COUNTS: [usize; 7] = [128, 256, 512, 1_024, 2_048, 4_096, 8_192];
    println!(
        "scenario,sources,prep_min_us,prep_median_us,prep_max_us,ns_per_source,growth_x,prep_repeats,prep_allocations,prep_bytes,update_vec_capacity,index_capacity,step_repeats,step_min_us,step_median_us,step_max_us,step_allocations,step_bytes"
    );
    for scenario in ForceSourceBenchScenario::ALL {
        let mut previous_median = None;
        for source_count in SOURCE_COUNTS {
            let (
                preparation,
                prep_repeats,
                prep_allocations,
                prep_bytes,
                update_capacity,
                index_capacity,
            ) = force_source_preparation_samples(scenario, source_count);
            let (full_step, step_repeats, step_allocations, step_bytes) =
                full_force_source_step_samples(scenario, source_count);
            let median = preparation[2];
            let growth = previous_median
                .map(|previous: f64| format!("{:.2}", median / previous))
                .unwrap_or_else(|| "-".to_owned());
            previous_median = Some(median);
            println!(
                "{},{source_count},{:.3},{:.3},{:.3},{:.2},{growth},{prep_repeats},{:.3},{:.1},{update_capacity},{index_capacity},{step_repeats},{:.3},{:.3},{:.3},{:.3},{:.1}",
                scenario.name(),
                preparation[0] / 1_000.0,
                preparation[2] / 1_000.0,
                preparation[4] / 1_000.0,
                median / source_count as f64,
                prep_allocations as f64,
                prep_bytes as f64,
                full_step[0] / 1_000.0,
                full_step[2] / 1_000.0,
                full_step[4] / 1_000.0,
                step_allocations as f64,
                step_bytes as f64,
            );
        }
    }
}
