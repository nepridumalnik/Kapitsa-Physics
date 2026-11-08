use super::*;
use crate::{
    chunk::ChunkBuilder,
    collider::Collider,
    collision::{QueryFilter, Ray},
    mass::MassProperties,
    rigid_body::{CcdMode, RigidBody},
    shape::{Shape, Sphere, TriangleMesh},
    transform::Transform,
};
use kapitsa_math::{Quat, Vec3};
use std::error::Error;

fn chunk(x: f64, count: usize) -> Result<Chunk, Box<dyn Error>> {
    let mut builder = ChunkBuilder::new();
    for _ in 0..count {
        builder.add_mesh(TriangleMesh::new(
            vec![
                Vec3::new(-5.0, 0.0, -5.0),
                Vec3::new(-5.0, 0.0, 5.0),
                Vec3::new(5.0, 0.0, -5.0),
                Vec3::new(5.0, 0.0, 5.0),
            ],
            vec![[0, 1, 2], [2, 1, 3]],
        )?)?;
    }
    Ok(builder.build(Transform::new(Vec3::new(x, 0.0, 0.0), Quat::IDENTITY)?)?)
}

fn body(world: &mut PhysicsWorld) -> Result<BodyId, WorldError> {
    world.add_body(RigidBody::new_static(Transform::identity()))
}

#[test]
fn batch_preserves_bvhs_order_and_rejects_stale_and_foreign_ids() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let body = body(&mut world)?;
    let chunks = vec![chunk(0.0, 2)?, chunk(20.0, 3)?];
    let pointers: Vec<_> = chunks
        .iter()
        .flat_map(|c| {
            c.colliders
                .iter()
                .filter_map(|c| c.query_bvh().map(|bvh| bvh as *const _))
        })
        .collect();
    let ids = world.add_chunk_batch(body, chunks)?;
    assert_eq!(world.collider_count(), 5);
    assert_eq!(world.chunk_count(), 2);
    assert_eq!(world.colliders_of(body).ok_or("missing body")?.count(), 5);
    for &id in &ids {
        assert_eq!(world.chunk_body(id), Some(body));
        for collider in world.chunk_colliders(id).ok_or("missing chunk")?.to_vec() {
            assert!(world.remove_collider(collider).is_none());
            assert!(world.collider_mut(collider).is_none());
        }
    }
    let removed = world.remove_chunk_batch(ids.clone())?;
    let returned: Vec<_> = removed
        .iter()
        .flat_map(|c| {
            c.colliders
                .iter()
                .filter_map(|c| c.query_bvh().map(|bvh| bvh as *const _))
        })
        .collect();
    assert_eq!(pointers, returned);
    assert_eq!(removed.first().map(|c| c.colliders.len()), Some(2));
    assert_eq!(world.collider_count(), 0);
    let replacements = world.add_chunk_batch(body, removed)?;
    assert!(ids.iter().all(|&id| world.remove_chunk(id).is_none()));
    let mut foreign = PhysicsWorld::new();
    let foreign_body = self::body(&mut foreign)?;
    let foreign_id = foreign.add_chunk(foreign_body, chunk(0.0, 1)?)?;
    assert!(matches!(
        world.remove_chunk_batch(vec![
            *replacements.first().ok_or("missing replacement")?,
            foreign_id
        ]),
        Err(ChunkOperationError::InvalidChunkId)
    ));
    assert_eq!(world.chunk_count(), 2);
    Ok(())
}

#[test]
fn invalid_batches_are_atomic_and_keep_input_ownership() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let body = body(&mut world)?;
    let id = world.add_chunk(body, chunk(0.0, 1)?)?;
    assert_eq!(
        world.remove_chunk_batch(vec![id, id]).err(),
        Some(ChunkOperationError::DuplicateChunkId)
    );
    assert_eq!(world.chunk_count(), 1);
    let mut other = PhysicsWorld::new();
    let other_body = self::body(&mut other)?;
    let error = world
        .add_chunk_batch(other_body, vec![chunk(10.0, 2)?, chunk(20.0, 1)?])
        .err()
        .ok_or("expected error")?;
    assert_eq!(error.chunks.len(), 2);
    assert_eq!(
        error.chunks.first().map(|chunk| chunk.colliders.len()),
        Some(2)
    );
    assert_eq!(world.collider_count(), 1);
    let mut invalid = chunk(20.0, 1)?;
    let material = other.add_material(crate::material::PhysicsMaterial::default())?;
    for collider in &mut invalid.colliders {
        collider.set_material(Some(material));
    }
    let error = world
        .add_chunk_batch(body, vec![chunk(10.0, 1)?, invalid])
        .err()
        .ok_or("expected material error")?;
    assert_eq!(error.cause, WorldError::InvalidMaterialId.into());
    assert_eq!(error.chunks.len(), 2);
    assert_eq!(world.collider_count(), 1);
    assert!(world.add_chunk_batch(other_body, Vec::new())?.is_empty());
    assert!(world.remove_chunk_batch(Vec::new())?.is_empty());
    let removed = world.remove_chunk(id).ok_or("missing chunk")?;
    let id = world.add_chunk(body, removed)?;
    assert!(world.remove_body(body).is_some());
    assert!(world.chunk_body(id).is_none());
    assert_eq!(world.chunk_count(), 0);
    assert_eq!(world.collider_count(), 0);
    Ok(())
}

#[test]
fn streamed_queries_sweeps_and_steps_find_only_registered_meshes() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let body = body(&mut world)?;
    let ray = Ray::new(Vec3::new(0.0, 3.0, 0.0), -Vec3::Y, 10.0)?;
    let sphere = Shape::Sphere(Sphere::new(0.5)?);
    for _ in 0..5 {
        let ids = world.add_chunk_batch(body, vec![chunk(0.0, 1)?, chunk(20.0, 1)?])?;
        world.step(0.02)?;
        assert!(world.raycast(ray, QueryFilter::default())?.is_some());
        assert!(
            world
                .sweep(
                    &sphere,
                    Transform::new(Vec3::new(0.0, 3.0, 0.0), Quat::IDENTITY)?,
                    -Vec3::Y * 4.0,
                    QueryFilter::default()
                )?
                .is_some()
        );
        let removed = world.remove_chunk_batch(ids)?;
        assert!(world.raycast(ray, QueryFilter::default())?.is_none());
        assert!(
            world
                .sweep(
                    &sphere,
                    Transform::identity(),
                    -Vec3::Y,
                    QueryFilter::default()
                )?
                .is_none()
        );
        world.step(0.02)?;
        let ids = world.add_chunk_batch(body, removed)?;
        world.remove_chunk_batch(ids)?;
    }
    Ok(())
}

#[test]
fn dynamic_mesh_rejection_and_cluster_guard_keep_chunk_ownership() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::from_shape(&Shape::Sphere(Sphere::new(0.5)?), 1.0)?;
    let dynamic = world.add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)?;
    let error = world
        .add_chunk(dynamic, chunk(0.0, 1)?)
        .err()
        .ok_or("expected rejection")?;
    assert_eq!(error.chunk.colliders.len(), 1);
    assert_eq!(world.collider_count(), 0);
    let body = body(&mut world)?;
    world.add_chunk(body, error.chunk)?;
    assert!(matches!(
        world.enable_cluster_with_mass(body, mass),
        Err(crate::world::ClusterError::UnsupportedGeometry)
    ));
    Ok(())
}

#[test]
fn continuous_dynamic_sphere_hits_streamed_mesh() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let terrain = body(&mut world)?;
    let id = world.add_chunk(terrain, chunk(0.0, 1)?)?;
    let mut rocket = RigidBody::new_dynamic(
        Transform::new(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY)?,
        MassProperties::from_shape(&Shape::Sphere(Sphere::new(0.5)?), 1.0)?,
    )?;
    rocket.set_linear_velocity(-Vec3::Y * 200.0)?;
    rocket.set_ccd_mode(CcdMode::Continuous);
    let rocket = world.add_body(rocket)?;
    world.add_collider(
        rocket,
        Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
    )?;
    world.step(0.02)?;
    assert!(
        world
            .body(rocket)
            .ok_or("missing rocket")?
            .state()
            .transform()
            .translation()
            .y
            >= 0.49
    );
    assert!(world.remove_chunk(id).is_some());
    world.step(0.02)?;
    Ok(())
}

#[test]
fn allocation_failure_during_registration_returns_every_prepared_mesh() -> Result<(), Box<dyn Error>>
{
    for fail_after in 0..5 {
        let mut world = PhysicsWorld::new();
        let body = body(&mut world)?;
        let survivor = world.add_chunk(body, chunk(0.0, 1)?)?;
        let input = vec![chunk(20.0, 2)?, chunk(40.0, 3)?];
        let pointers: Vec<_> = input
            .iter()
            .flat_map(|c| {
                c.colliders
                    .iter()
                    .filter_map(|c| c.query_bvh().map(|bvh| bvh as *const _))
            })
            .collect();
        let error = world
            .add_chunk_batch_inner(body, input, Some(fail_after))
            .err()
            .ok_or("expected allocation failure")?;
        let recovered: Vec<_> = error
            .chunks
            .iter()
            .flat_map(|c| {
                c.colliders
                    .iter()
                    .filter_map(|c| c.query_bvh().map(|bvh| bvh as *const _))
            })
            .collect();
        assert_eq!(pointers, recovered);
        assert_eq!(world.chunk_count(), 1);
        assert_eq!(world.collider_count(), 1);
        assert_eq!(world.chunk_body(survivor), Some(body));
        assert_eq!(world.colliders_of(body).ok_or("missing body")?.count(), 1);
        let ids = world.add_chunk_batch(body, error.chunks)?;
        assert_eq!(ids.len(), 2);
        world.step(0.02)?;
    }
    Ok(())
}
