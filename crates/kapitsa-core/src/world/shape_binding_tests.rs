//! Tests for body-bound shared shapes.

use std::sync::Arc;

use kapitsa_math::{Mat3, Quat, Vec3};

use super::{BodyId, PhysicsWorld};
use crate::aerodynamics::Aerodynamics;
use crate::aerodynamics::shape::{
    CookedSource, ShapeId, ShapeRegistry, ShapeRegistryError, assembly_pressure_config,
};
use crate::execution::SequentialExecutor;
use crate::mass::MassProperties;
use crate::rigid_body::RigidBody;
use crate::shape::TriangleMesh;
use crate::transform::Transform;

fn cube() -> Result<TriangleMesh, crate::Error> {
    TriangleMesh::new(
        vec![
            Vec3::ZERO,
            Vec3::X,
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::Y,
            Vec3::Z,
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::ONE,
            Vec3::new(0.0, 1.0, 1.0),
        ],
        vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ],
    )
}

fn add_body(world: &mut PhysicsWorld, model: Aerodynamics) -> Result<BodyId, crate::Error> {
    let body = world.add_body(RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
    )?)?;
    world.set_aerodynamics(body, model)?;
    Ok(body)
}

fn source(mesh: &Arc<TriangleMesh>, offset: Vec3, slot: u32) -> Result<CookedSource, crate::Error> {
    Ok(CookedSource {
        mesh: Arc::clone(mesh),
        local_transform: Transform::new(offset, Quat::IDENTITY)?,
        slot: crate::aerodynamics::geometry::OwnerSlot::new(slot),
    })
}

fn mesh_body(world: &mut PhysicsWorld) -> Result<BodyId, crate::Error> {
    add_body(world, Aerodynamics::Mesh(cube()?))
}

fn part_mesh(
    world: &PhysicsWorld,
    part: super::PartId,
) -> Result<Arc<TriangleMesh>, Box<dyn std::error::Error>> {
    match world
        .part_aerodynamic_geometry
        .as_ref()
        .and_then(|items| items.get(&part))
    {
        Some(super::assembly_aerodynamics::PartGeometry::Mesh(mesh)) => Ok(Arc::clone(mesh)),
        _ => Err("part has no mesh geometry".into()),
    }
}

#[test]
fn bind_accepts_matching_mesh_sources_and_unbinds() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let body = mesh_body(&mut world)?;
    let part = world.enable_cluster(body)?;
    let mesh = part_mesh(&world, part)?;
    let sources = vec![source(&mesh, Vec3::ZERO, 0)?];
    let shape: ShapeId = world.cook_shape(&sources, false)?;
    world.bind_shape(body, shape, vec![part])?;
    let binding = world.shape_binding(body).ok_or("binding")?;
    assert_eq!(binding.shape(), shape);
    assert_eq!(binding.parts(), &[part]);
    assert_eq!(world.shape_registry().len(), 1);
    let removed = world.unbind_shape(body).ok_or("removed binding")?;
    assert_eq!(removed.parts(), &[part]);
    assert!(world.shape_binding(body).is_none());
    Ok(())
}

#[test]
fn bind_rejects_foreign_registry_and_cavity_mismatches() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let body = mesh_body(&mut world)?;
    let part = world.enable_cluster(body)?;
    let mesh = part_mesh(&world, part)?;
    let sources = vec![source(&mesh, Vec3::ZERO, 0)?];
    let mut foreign = ShapeRegistry::new();
    let foreign_id = foreign.cook(&sources, assembly_pressure_config(), false)?;
    let error = world
        .bind_shape(body, foreign_id, vec![part])
        .err()
        .ok_or("foreign shape must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::StaleShape)
    );
    assert!(world.shape_binding(body).is_none());
    let shape = world.cook_shape(&sources, false)?;
    world.set_assembly_mesh_oriented_cavities(true);
    let error = world
        .bind_shape(body, shape, vec![part])
        .err()
        .ok_or("cavity mismatch must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::InvalidBinding)
    );
    Ok(())
}

#[test]
fn cook_shape_requires_the_world_cavity_policy() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let body = mesh_body(&mut world)?;
    let part = world.enable_cluster(body)?;
    let mesh = part_mesh(&world, part)?;
    let sources = vec![source(&mesh, Vec3::ZERO, 0)?];
    let error = world
        .cook_shape(&sources, true)
        .err()
        .ok_or("cook with a different policy must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::InvalidBinding)
    );
    assert!(world.shape_registry().is_empty());
    world.set_assembly_mesh_oriented_cavities(true);
    let shape = world.cook_shape(&sources, true)?;
    world.bind_shape(body, shape, vec![part])?;
    world.set_assembly_mesh_oriented_cavities(false);
    let binding = world.shape_binding(body).ok_or("binding")?;
    let cooked = world.shape_registry().get(shape).ok_or("shape")?;
    let entry = world.bodies.get(body.dense).ok_or("entry")?;
    let matches = super::assembly_aerodynamics::binding_matches(
        body,
        entry,
        &world.parts,
        world.part_aerodynamic_geometry.as_ref(),
        world.preserve_assembly_mesh_cavities,
        cooked,
        binding,
    );
    assert!(matches.is_err(), "changed policy must not match");
    Ok(())
}

#[test]
fn bind_rejects_foreign_parts_and_wrong_arity() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let body = mesh_body(&mut world)?;
    let part = world.enable_cluster(body)?;
    let mesh = part_mesh(&world, part)?;
    let other_body = mesh_body(&mut world)?;
    let other_part = world.enable_cluster(other_body)?;
    let sources = vec![source(&mesh, Vec3::ZERO, 0)?];
    let shape = world.cook_shape(&sources, false)?;
    let error = world
        .bind_shape(body, shape, vec![other_part])
        .err()
        .ok_or("foreign part must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::InvalidBinding)
    );
    let two = vec![source(&mesh, Vec3::ZERO, 0)?, source(&mesh, Vec3::Y, 1)?];
    let shape = world.cook_shape(&two, false)?;
    let error = world
        .bind_shape(body, shape, vec![part])
        .err()
        .ok_or("arity mismatch must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::InvalidBinding)
    );
    Ok(())
}

#[test]
fn bind_rejects_collider_sources() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let body = add_body(&mut world, Aerodynamics::Colliders)?;
    let part = world.enable_cluster(body)?;
    let mesh = Arc::new(cube()?);
    let sources = vec![source(&mesh, Vec3::ZERO, 0)?];
    let shape = world.cook_shape(&sources, false)?;
    let error = world
        .bind_shape(body, shape, vec![part])
        .err()
        .ok_or("collider source must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::UnsupportedSource)
    );
    Ok(())
}

#[test]
fn remove_body_drops_binding_and_foreign_ids_are_ignored() -> Result<(), Box<dyn std::error::Error>>
{
    let mut world = PhysicsWorld::new();
    let body = mesh_body(&mut world)?;
    let part = world.enable_cluster(body)?;
    let mesh = part_mesh(&world, part)?;
    let sources = vec![source(&mesh, Vec3::ZERO, 0)?];
    let shape = world.cook_shape(&sources, false)?;
    world.bind_shape(body, shape, vec![part])?;
    world.remove_body(body).ok_or("body removed")?;
    assert!(world.shape_binding(body).is_none());
    assert_eq!(
        world.bind_shape(body, shape, vec![part]),
        Err(crate::Error::from(super::ClusterError::InvalidBodyId))
    );
    Ok(())
}

fn wind_world() -> Result<PhysicsWorld, Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let state = crate::aerodynamics::AtmosphericState::new(1.0, Vec3::X * -2_000.0)?;
    world.set_atmosphere(super::test_atmosphere(state));
    Ok(world)
}

fn clustered_mesh_body(
    world: &mut PhysicsWorld,
    offset: Vec3,
) -> Result<(BodyId, super::PartId), crate::Error> {
    let body = world.add_body(RigidBody::new_dynamic(
        Transform::new(offset, Quat::IDENTITY)?,
        MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
    )?)?;
    world.set_aerodynamics(body, Aerodynamics::Mesh(cube()?))?;
    let part = world.enable_cluster(body)?;
    Ok((body, part))
}

fn body_velocity(world: &PhysicsWorld, body: BodyId) -> Result<Vec3, Box<dyn std::error::Error>> {
    Ok(world.body(body).ok_or("body")?.state().linear_velocity())
}

/// A cube scaled along Y so it differs from [`cube`].
fn taller_cube() -> Result<TriangleMesh, crate::Error> {
    let base = cube()?;
    TriangleMesh::new(
        base.vertices()
            .iter()
            .map(|v| Vec3::new(v.x, v.y * 2.0, v.z))
            .collect(),
        base.triangles().to_vec(),
    )
}

#[test]
fn bound_and_unbound_bodies_share_pressure_physics() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = wind_world()?;
    let (bound, bound_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let (local, _) = clustered_mesh_body(&mut world, Vec3::X * 100.0)?;
    let mesh = part_mesh(&world, bound_part)?;
    let shape = world.cook_shape(&[source(&mesh, Vec3::ZERO, 0)?], false)?;
    world.bind_shape(bound, shape, vec![bound_part])?;
    world.step(1e-8)?;
    assert!(
        world.shape_binding(bound).is_some(),
        "matching binding must survive the step"
    );
    let caches = world.assembly_aerodynamics.as_ref().ok_or("caches")?;
    let shared_entry = caches.get(&bound).ok_or("shared entry")?;
    assert!(
        shared_entry.local_source_count() == 0,
        "bound body must not rebuild per-body sources"
    );
    let local_entry = caches.get(&local).ok_or("local entry")?;
    assert_eq!(local_entry.local_source_count(), 1);
    let bound_velocity = body_velocity(&world, bound)?;
    let local_velocity = body_velocity(&world, local)?;
    assert!(
        bound_velocity.length() > 1e-6,
        "expected a nonzero pressure response, got {bound_velocity:?}"
    );
    assert!(
        (bound_velocity - local_velocity).length() < 1e-12,
        "velocity mismatch: bound {bound_velocity:?} vs local {local_velocity:?}"
    );
    Ok(())
}

#[test]
fn geometry_edit_invalidates_binding_and_falls_back_with_identical_physics()
-> Result<(), Box<dyn std::error::Error>> {
    let mut world = wind_world()?;
    let (bound, bound_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let (control, control_part) = clustered_mesh_body(&mut world, Vec3::X * 100.0)?;
    let mesh = part_mesh(&world, bound_part)?;
    let shape = world.cook_shape(&[source(&mesh, Vec3::ZERO, 0)?], false)?;
    world.bind_shape(bound, shape, vec![bound_part])?;
    world.step(1e-8)?;
    let before_bound = body_velocity(&world, bound)?;
    let before_control = body_velocity(&world, control)?;
    assert!((before_bound - before_control).length() < 1e-12);

    world.set_part_aerodynamic_mesh(bound_part, Some(taller_cube()?))?;
    world.set_part_aerodynamic_mesh(control_part, Some(taller_cube()?))?;
    world.step(1e-8)?;
    assert!(
        world.shape_binding(bound).is_none(),
        "edited geometry must invalidate the binding"
    );
    let caches = world.assembly_aerodynamics.as_ref().ok_or("caches")?;
    assert_eq!(
        caches
            .get(&bound)
            .ok_or("bound entry")?
            .local_source_count(),
        1,
        "fallback must rebuild body-local sources"
    );
    let after_bound = body_velocity(&world, bound)?;
    let after_control = body_velocity(&world, control)?;
    assert!(
        (after_bound - after_control).length() < 1e-12,
        "fallback velocity mismatch: {after_bound:?} vs {after_control:?}"
    );
    assert!(
        after_bound.length() > 1e-6,
        "fallback must still apply pressure, got {after_bound:?}"
    );
    Ok(())
}

#[test]
fn registry_replacement_stales_bindings_and_evaluation_falls_back()
-> Result<(), Box<dyn std::error::Error>> {
    let mut world = wind_world()?;
    let (bound, bound_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let (control, _) = clustered_mesh_body(&mut world, Vec3::X * 100.0)?;
    let mesh = part_mesh(&world, bound_part)?;
    let sources = vec![source(&mesh, Vec3::ZERO, 0)?];
    let shape = world.cook_shape(&sources, false)?;
    world.bind_shape(bound, shape, vec![bound_part])?;
    world.step(1e-8)?;
    let before_bound = body_velocity(&world, bound)?;
    let before_control = body_velocity(&world, control)?;
    assert!((before_bound - before_control).length() < 1e-12);

    assert_eq!(world.replace_shape(shape, &sources)?, 2);
    assert_eq!(
        world.shape_registry().get(shape).ok_or("shape")?.revision(),
        2
    );
    world.step(1e-8)?;
    assert!(
        world.shape_binding(bound).is_none(),
        "replaced shape must stale the binding"
    );
    let after_bound = body_velocity(&world, bound)?;
    let after_control = body_velocity(&world, control)?;
    assert!((after_bound - after_control).length() < 1e-12);
    Ok(())
}

fn simulate_bound(sharded: bool) -> Result<(Vec3, bool, bool), Box<dyn std::error::Error>> {
    let mut world = wind_world()?;
    let (body, part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let mesh = part_mesh(&world, part)?;
    let shape = world.cook_shape(&[source(&mesh, Vec3::ZERO, 0)?], false)?;
    world.bind_shape(body, shape, vec![part])?;
    if sharded {
        world.step_with_shards(1e-8, &mut [Vec::new()], &SequentialExecutor, 1)?;
    } else {
        world.step(1e-8)?;
    }
    let velocity = body_velocity(&world, body)?;
    let binding = world.shape_binding(body).is_some();
    let sources_empty = world
        .assembly_aerodynamics
        .as_ref()
        .and_then(|caches| caches.get(&body))
        .is_some_and(|entry| entry.local_source_count() == 0);
    Ok((velocity, binding, sources_empty))
}

#[test]
fn bound_body_ordinary_and_sharded_steps_agree() -> Result<(), Box<dyn std::error::Error>> {
    let (ordinary_velocity, ordinary_binding, ordinary_shared) = simulate_bound(false)?;
    let (sharded_velocity, sharded_binding, sharded_shared) = simulate_bound(true)?;
    assert!(ordinary_binding && sharded_binding);
    assert!(
        ordinary_shared && sharded_shared,
        "both paths must reuse the cooked shape"
    );
    assert!(
        (ordinary_velocity - sharded_velocity).length() < 1e-12,
        "velocity mismatch: ordinary {ordinary_velocity:?} vs sharded {sharded_velocity:?}"
    );
    Ok(())
}
