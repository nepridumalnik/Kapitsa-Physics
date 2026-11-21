//! Tests for body-bound shared shapes.

use std::sync::Arc;

use kapitsa_math::{Mat3, Quat, Vec3};

use super::{BodyId, PhysicsWorld};
use crate::aerodynamics::Aerodynamics;
use crate::aerodynamics::shape::{
    CookedSource, ShapeId, ShapeRegistry, ShapeRegistryError, assembly_pressure_config,
};
use crate::cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength};
use crate::execution::SequentialExecutor;
use crate::joint::JointFrame;
use crate::mass::MassProperties;
use crate::rigid_body::RigidBody;
use crate::shape::TriangleMesh;
use crate::transform::Transform;

pub(super) fn cube() -> Result<TriangleMesh, crate::Error> {
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

fn clustered_shared_body(
    world: &mut PhysicsWorld,
    offset: Vec3,
    mesh: &Arc<TriangleMesh>,
) -> Result<(BodyId, super::PartId), crate::Error> {
    let body = world.add_body(RigidBody::new_dynamic(
        Transform::new(offset, Quat::IDENTITY)?,
        MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
    )?)?;
    world.set_aerodynamics(body, Aerodynamics::None)?;
    let part = world.enable_cluster(body)?;
    world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(mesh)))?;
    Ok((body, part))
}

#[test]
fn bound_exposure_maps_cook_slots_and_rejects_stale_revision()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::geometry::GeometryOwner;
    let mut world = wind_world()?;
    let mesh = Arc::new(cube()?);
    let (first, first_part) = clustered_shared_body(&mut world, Vec3::ZERO, &mesh)?;
    let (second, second_part) = clustered_shared_body(&mut world, Vec3::X * 10.0, &mesh)?;
    let sources = [source(&mesh, Vec3::ZERO, 0)?];
    let shape = world.cook_shape(&sources, false)?;
    world.bind_shape(first, shape, vec![first_part])?;
    world.bind_shape(second, shape, vec![second_part])?;
    for (body, part) in [(first, first_part), (second, second_part)] {
        let patches = world.bound_shape_exposure(body, Vec3::X, 0.01)?;
        assert!(!patches.is_empty());
        assert!(
            patches
                .iter()
                .all(|patch| patch.owner == GeometryOwner::Part(part))
        );
    }
    world.replace_shape(shape, 1, &sources)?;
    assert!(matches!(world.bound_shape_exposure(first, Vec3::X, 0.01),
        Err(error) if error.downcast_ref::<ShapeRegistryError>() == Some(&ShapeRegistryError::StaleBinding)));
    Ok(())
}

#[test]
fn bound_exposure_respects_noncanonical_slot_order_and_rejects_split()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::geometry::GeometryOwner;
    let mut world = wind_world()?;
    let (body, root) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let offset = Vec3::Y * 4.0;
    let mut desc = part_desc(MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?);
    desc.local_transform = Transform::new(offset, Quat::IDENTITY)?;
    let child = world.add_cluster_part(
        root,
        desc,
        WeldDesc {
            frame_a: JointFrame::new(offset, Quat::IDENTITY),
            ..merge_weld()
        },
    )?;
    world.set_part_aerodynamic_mesh(child, Some(cube()?))?;
    let sources = [
        source(&part_mesh(&world, child)?, offset, 0)?,
        source(&part_mesh(&world, root)?, Vec3::ZERO, 1)?,
    ];
    let shape = world.cook_shape(&sources, false)?;
    world.bind_shape(body, shape, vec![child, root])?;
    let patches = world.bound_shape_exposure(body, Vec3::X, 0.01)?;
    for (part, upper) in [(root, false), (child, true)] {
        let mut owned = patches
            .iter()
            .filter(|patch| patch.owner == GeometryOwner::Part(part))
            .peekable();
        assert!(owned.peek().is_some());
        assert!(owned.all(|patch| (patch.center.y > offset.y / 2.0) == upper));
    }
    let weld = *world
        .cluster(body)
        .ok_or("missing cluster")?
        .welds()
        .first()
        .ok_or("missing weld")?;
    let split = world.break_weld(weld)?;
    assert_eq!(split.len(), 2);
    for body in split {
        assert!(matches!(world.bound_shape_exposure(body, Vec3::X, 0.01),
            Err(error) if error.downcast_ref::<ShapeRegistryError>() == Some(&ShapeRegistryError::InvalidBinding)));
    }
    Ok(())
}

#[test]
fn two_bodies_share_one_cooked_shape() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = wind_world()?;
    let shared_mesh = Arc::new(cube()?);
    let (first, first_part) = clustered_shared_body(&mut world, Vec3::ZERO, &shared_mesh)?;
    let (second, second_part) = clustered_shared_body(&mut world, Vec3::X * 100.0, &shared_mesh)?;
    let (local, _local_part) = clustered_shared_body(&mut world, Vec3::X * 200.0, &shared_mesh)?;
    let sources = vec![source(&shared_mesh, Vec3::ZERO, 0)?];
    let shape = world.cook_shape(&sources, false)?;
    world.bind_shape(first, shape, vec![first_part])?;
    world.bind_shape(second, shape, vec![second_part])?;
    world.step(1e-8)?;
    assert_eq!(world.shape_registry().len(), 1);
    assert!(world.shape_binding(first).is_some());
    assert!(world.shape_binding(second).is_some());
    assert!(world.shape_binding(local).is_none());
    let caches = world.assembly_aerodynamics.as_ref().ok_or("caches")?;
    for body in [first, second] {
        assert_eq!(
            caches
                .get(&body)
                .ok_or("shared entry")?
                .local_source_count(),
            0,
            "bound body must not rebuild per-body sources"
        );
    }
    assert_eq!(
        caches
            .get(&local)
            .ok_or("local entry")?
            .local_source_count(),
        1
    );
    let first_velocity = body_velocity(&world, first)?;
    let second_velocity = body_velocity(&world, second)?;
    let local_velocity = body_velocity(&world, local)?;
    assert!(
        first_velocity.length() > 1e-6,
        "expected a nonzero pressure response, got {first_velocity:?}"
    );
    assert!((first_velocity - second_velocity).length() < 1e-12);
    assert!((first_velocity - local_velocity).length() < 1e-12);
    use crate::aerodynamics::geometry::GeometryOwner;
    let first_load = world.owner_mesh_pressure(first).ok_or("first pressure")?;
    let second_load = world.owner_mesh_pressure(second).ok_or("second pressure")?;
    assert_eq!(first_load.len(), 1);
    assert_eq!(
        second_load,
        &[(GeometryOwner::Part(second_part), first_load[0].1)]
    );
    assert_eq!(first_load[0].0, GeometryOwner::Part(first_part));
    world.set_part_aerodynamic_mesh(first_part, Some(taller_cube()?))?;
    world.step(1e-8)?;
    assert!(world.shape_binding(first).is_none());
    assert_eq!(
        world
            .shape_binding(second)
            .ok_or("unaffected binding")?
            .shape(),
        shape
    );
    assert_eq!(
        body_velocity(&world, second)?,
        body_velocity(&world, local)?
    );
    Ok(())
}

#[test]
fn bind_rejects_equal_but_distinct_mesh_allocations() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let (first, first_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let (second, second_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let first_mesh = part_mesh(&world, first_part)?;
    let second_mesh = part_mesh(&world, second_part)?;
    assert!(!Arc::ptr_eq(&first_mesh, &second_mesh));
    let shape = world.cook_shape(&[source(&first_mesh, Vec3::ZERO, 0)?], false)?;
    world.bind_shape(first, shape, vec![first_part])?;
    let error = world
        .bind_shape(second, shape, vec![second_part])
        .err()
        .ok_or("distinct allocation must not bind")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::InvalidBinding)
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

    assert_eq!(world.replace_shape(shape, 1, &sources)?, 2);
    let previous_binding = world.shape_binding(bound).cloned();
    let error = world
        .replace_shape(shape, 1, &[])
        .err()
        .ok_or("stale revision accepted")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::StaleBinding)
    );
    assert_eq!(world.shape_binding(bound), previous_binding.as_ref());
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

pub(super) fn merge_weld() -> WeldDesc {
    WeldDesc {
        frame_a: JointFrame::IDENTITY,
        frame_b: JointFrame::IDENTITY,
        strength: WeldStrength::Unbreakable,
        stiffness: WeldStiffness {
            linear: 1.0,
            angular: 1.0,
        },
    }
}

fn part_desc(mass: MassProperties) -> ClusterPartDesc {
    ClusterPartDesc {
        local_transform: Transform::identity(),
        mass,
        colliders: Vec::new(),
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
    }
}

#[rstest::rstest]
#[case::current(false)]
#[case::stale(true)]
fn unstaged_shards_apply_bound_mesh_pressure(
    #[case] stale: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let mesh = Arc::new(cube()?);
    let mut world = wind_world()?;
    let (body, part) = clustered_shared_body(&mut world, Vec3::ZERO, &mesh)?;
    let sources = [source(&mesh, Vec3::ZERO, 0)?];
    let shape = world.cook_shape(&sources, false)?;
    world.bind_shape(body, shape, vec![part])?;
    if stale {
        world.set_part_aerodynamic_mesh(part, Some(taller_cube()?))?;
    }
    let mut shards = [Vec::new()];
    world.fill_step_shards(&mut shards)?;
    assert!(
        world.shape_binding(body).is_some(),
        "snapshot fill must not mutate bindings"
    );
    let step = shards[0].first_mut().ok_or("missing body snapshot")?;
    step.integrate(TOPOLOGY_DT)?;
    let predicted = step.predicted.ok_or("missing prediction")?;
    assert!(predicted.linear_velocity().length() > 1e-6);
    world.step(TOPOLOGY_DT)?;
    let actual = world.body(body).ok_or("missing body")?.state();
    assert_eq!(predicted.linear_velocity(), actual.linear_velocity());
    assert_eq!(predicted.angular_velocity(), actual.angular_velocity());
    Ok(())
}

const TOPOLOGY_DT: f64 = 1e-8;

#[test]
fn standalone_cooked_hull_matches_both_step_paths() -> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::{AtmosphericState, MeshPressureModel, geometry::GeometryOwner};
    fn run(
        sharded: bool,
        assembly: bool,
    ) -> Result<(Vec3, crate::aerodynamics::pressure::PressureWrench), Box<dyn std::error::Error>>
    {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::test_atmosphere(AtmosphericState::new(
            1.0,
            -Vec3::X * 1500.0,
        )?));
        world.set_mesh_pressure_model(MeshPressureModel::ModifiedNewtonianTriangleQuadrature);
        let body = add_body(&mut world, Aerodynamics::Mesh(cube()?))?;
        if assembly {
            let part = world.enable_cluster(body)?;
            let mesh = part_mesh(&world, part)?;
            let shape = world.cook_shape(&[source(&mesh, Vec3::ZERO, 0)?], false)?;
            world.bind_shape(body, shape, vec![part])?;
            topology_step(&mut world, sharded)?;
            let [(GeometryOwner::Part(owner), wrench)] =
                world.owner_mesh_pressure(body).ok_or("assembly pressure")?
            else {
                return Err("expected assembly owner".into());
            };
            assert_eq!(*owner, part);
            return Ok((body_velocity(&world, body)?, *wrench));
        }
        let before = world.body(body).ok_or("body")?.motion();
        assert!(topology_step(&mut world, sharded).is_err());
        assert_eq!(world.body(body).ok_or("body")?.motion(), before);
        world.cook_primary_hull(body)?;
        topology_step(&mut world, sharded)?;
        let [(GeometryOwner::Body, wrench)] = world.owner_mesh_pressure(body).ok_or("pressure")?
        else {
            return Err("expected standalone hull wrench".into());
        };
        assert!(wrench.force.x < 0.0);
        let result = (body_velocity(&world, body)?, *wrench);
        let before = world.body(body).ok_or("body")?.motion();
        world.set_body_mesh_oriented_cavities(true);
        assert!(topology_step(&mut world, sharded).is_err());
        assert_eq!(world.body(body).ok_or("body")?.motion(), before);
        world.cook_primary_hull(body)?;
        world.set_aerodynamics(body, Aerodynamics::Mesh(cube()?))?;
        assert!(topology_step(&mut world, sharded).is_err());
        Ok(result)
    }
    let baseline = run(false, false)?;
    assert_eq!(baseline, run(true, false)?);
    for sharded in [false, true] {
        let (velocity, wrench) = run(sharded, true)?;
        let force_scale = baseline.1.force.length();
        assert!((wrench.force - baseline.1.force).length() < 1e-10 * force_scale);
        // The reference length is the 1 m cube edge.
        assert!((wrench.moment - baseline.1.moment).length() < 1e-10 * force_scale);
        assert!((velocity - baseline.0).length() < 1e-10 * baseline.0.length());
    }
    Ok(())
}

#[test]
fn cooked_hull_and_controlled_wing_loads_do_not_overlap() -> Result<(), Box<dyn std::error::Error>>
{
    use crate::aerodynamics::{
        AtmosphericState, MeshPressureModel,
        wing::{FiniteWing, WingControl, WingSurface},
    };
    for (sharded, associated) in [(false, false), (false, true), (true, false), (true, true)] {
        let mesh = Arc::new(cube()?);
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::test_atmosphere(AtmosphericState::new(
            1.0,
            -Vec3::X * 100.0,
        )?));
        world.set_mesh_pressure_model(MeshPressureModel::ModifiedNewtonianTriangleQuadrature);
        let (body, root) = clustered_shared_body(&mut world, Vec3::ZERO, &mesh)?;
        let offset = if associated { Vec3::X } else { Vec3::Y } * 4.0;
        let mut desc = part_desc(MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?);
        desc.local_transform = Transform::new(offset, Quat::IDENTITY)?;
        let wing = world.add_cluster_part(
            root,
            desc,
            WeldDesc {
                frame_a: JointFrame::new(offset, Quat::IDENTITY),
                ..merge_weld()
            },
        )?;
        assert!(
            world
                .set_part_wing_shielding_mesh_shared(wing, Arc::clone(&mesh))
                .is_err()
        );
        world.set_part_aerodynamic_wings(
            wing,
            vec![
                WingSurface::new(
                    FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2)?,
                    if associated {
                        Transform::identity()
                    } else {
                        Transform::new(Vec3::Y * 4.0, Quat::IDENTITY)?
                    },
                    1.0,
                )?
                .with_control(WingControl::new(0.0, 0.2, 1.0)?)
                .with_spanwise_projection(0.01)?,
            ],
        )?;
        // Use geometry-only input for the independent wing's part too: the
        // displaced mesh does not represent the wing or overlap its footprint.
        if associated {
            world.set_part_wing_shielding_mesh_shared(wing, Arc::clone(&mesh))?;
        } else {
            world.set_part_wave_source_mesh_shared(wing, Some(Arc::clone(&mesh)))?;
        }
        let shape = world.cook_shape(
            &[source(&mesh, Vec3::ZERO, 0)?, source(&mesh, offset, 1)?],
            false,
        )?;
        world.bind_shape(body, shape, vec![root, wing])?;
        if !associated {
            let original = world
                .cluster_part(wing)
                .ok_or("wing part")?
                .aerodynamic_wings
                .clone();
            let surface = |z| -> Result<WingSurface, crate::Error> {
                WingSurface::new(
                    FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2)?,
                    Transform::new(Vec3::new(0.5, 0.5 - offset.y, z), Quat::IDENTITY)?,
                    0.25,
                )?
                .with_rectangular_footprint(Vec3::ZERO, 0.5, 0.5)
            };
            world.set_part_aerodynamic_wings(wing, vec![surface(1.0)?])?;
            let before = world.body(body).ok_or("body")?.motion();
            let error = topology_step(&mut world, sharded)
                .err()
                .ok_or("expected cross-part overlap rejection")?;
            assert!(
                matches!(error.downcast_ref::<super::StepError>(), Some(super::StepError::Aerodynamics { cause, .. }) if matches!(cause.downcast_ref::<crate::aerodynamics::AerodynamicsError>(), Some(crate::aerodynamics::AerodynamicsError::UnsupportedModel)))
            );
            assert_eq!(world.body(body).ok_or("body")?.motion(), before);
            world.set_part_aerodynamic_wings(wing, vec![surface(1.125)?])?;
            topology_step(&mut world, sharded)?;
            assert_eq!(world.part_wing_wrenches().len(), 1);
            world.set_part_aerodynamic_wings(wing, original)?;
        }
        let exposed = world.bound_shape_exposure(body, Vec3::X, 0.01)?;
        assert!(!exposed.is_empty());
        if associated {
            assert!(exposed.iter().all(
                |patch| patch.owner == crate::aerodynamics::geometry::GeometryOwner::Part(wing)
            ));
        }
        let id = world.part_wing_id(wing, 0).ok_or("missing wing ID")?;
        let mut initial_lift = 0.0;
        for deflection in [0.0, 0.1] {
            world.set_part_wing_deflection(id, deflection)?;
            topology_step(&mut world, sharded)?;
            let loads = world.owner_mesh_pressure(body).unwrap_or_default();
            if associated {
                assert!(loads.is_empty());
            } else {
                assert_eq!(loads.len(), 1);
                assert_eq!(
                    loads[0].0,
                    crate::aerodynamics::geometry::GeometryOwner::Part(root)
                );
                assert!(loads[0].1.force.x < 0.0);
            }
            let [load] = world.part_wing_wrenches() else {
                return Err("expected one wing wrench".into());
            };
            assert!(load.wrench.force.x < 0.0);
            let lift = load.wrench.force.z.abs();
            if deflection == 0.0 {
                initial_lift = lift;
            } else {
                assert!(lift > initial_lift);
            }
        }
        if associated {
            world.set_part_aerodynamic_wings(wing, Vec::new())?;
            let before = world.body(body).ok_or("body")?.motion();
            let error = topology_step(&mut world, sharded)
                .err()
                .ok_or("expected orphaned wing association rejection")?;
            assert!(
                matches!(error.downcast_ref::<super::StepError>(), Some(super::StepError::Aerodynamics { cause, .. }) if matches!(cause.downcast_ref::<crate::aerodynamics::AerodynamicsError>(), Some(crate::aerodynamics::AerodynamicsError::UnsupportedModel)))
            );
            assert_eq!(world.body(body).ok_or("body")?.motion(), before);
            // The same mesh can still serve as intentionally load-free geometry.
            world.set_part_wave_source_mesh_shared(wing, Some(Arc::clone(&mesh)))?;
            world.bind_shape(body, shape, vec![root, wing])?;
            topology_step(&mut world, sharded)?;
            assert!(world.part_wing_wrenches().is_empty());
            assert!(
                world
                    .owner_mesh_pressure(body)
                    .unwrap_or_default()
                    .iter()
                    .all(|(owner, _)| *owner
                        == crate::aerodynamics::geometry::GeometryOwner::Part(root))
            );
        }
    }
    Ok(())
}

#[test]
fn bound_triangle_quadrature_step_parity_and_rollback() -> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::MeshPressureModel;
    fn run(sharded: bool) -> Result<(Vec3, Vec3), Box<dyn std::error::Error>> {
        let mesh = Arc::new(cube()?);
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(crate::world::test_atmosphere(
            crate::aerodynamics::AtmosphericState::new(1.0, -Vec3::X * 1500.0)?,
        ));
        world.set_mesh_pressure_model(MeshPressureModel::ModifiedNewtonianTriangleQuadrature);
        let (body, part) = clustered_shared_body(&mut world, Vec3::ZERO, &mesh)?;
        let shape = world.cook_shape(&[source(&mesh, Vec3::ZERO, 0)?], false)?;
        world.bind_shape(body, shape, vec![part])?;
        topology_step(&mut world, sharded)?;
        let before = world.body(body).ok_or("body")?.motion();
        let loads = world.owner_mesh_pressure(body).ok_or("pressure")?.to_vec();
        assert_eq!(loads.len(), 1);
        assert!(loads[0].1.force.x < 0.0);
        world.set_mesh_sound_speed(200.0)?;
        assert!(topology_step(&mut world, sharded).is_err());
        assert_eq!(world.body(body).ok_or("body")?.motion(), before);
        assert_eq!(world.owner_mesh_pressure(body), Some(loads.as_slice()));
        world.set_mesh_sound_speed(340.0)?;
        use crate::aerodynamics::wing::{FiniteWing, WingSurface};
        world.set_part_aerodynamic_wings(
            part,
            vec![WingSurface::new(
                FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2)?,
                Transform::identity(),
                1.0,
            )?],
        )?;
        let error = topology_step(&mut world, sharded)
            .err()
            .ok_or("expected ambiguous geometry rejection")?;
        assert!(
            matches!(error.downcast_ref::<super::StepError>(), Some(super::StepError::Aerodynamics { cause, .. }) if matches!(cause.downcast_ref::<crate::aerodynamics::AerodynamicsError>(), Some(crate::aerodynamics::AerodynamicsError::UnsupportedModel)))
        );
        assert_eq!(world.body(body).ok_or("body")?.motion(), before);
        world.set_part_aerodynamic_wings(part, Vec::new())?;
        world.replace_shape(shape, 1, &[source(&mesh, Vec3::ZERO, 0)?])?;
        let error = topology_step(&mut world, sharded)
            .err()
            .ok_or("expected stale-binding rejection")?;
        assert!(
            matches!(error.downcast_ref::<super::StepError>(), Some(super::StepError::Aerodynamics { cause, .. }) if cause.downcast_ref::<ShapeRegistryError>() == Some(&ShapeRegistryError::InvalidBinding))
        );
        assert_eq!(world.body(body).ok_or("body")?.motion(), before);
        assert_eq!(world.owner_mesh_pressure(body), Some(loads.as_slice()));
        Ok((body_velocity(&world, body)?, loads[0].1.force))
    }
    assert_eq!(run(false)?, run(true)?);
    Ok(())
}

fn topology_step(world: &mut PhysicsWorld, sharded: bool) -> Result<(), crate::Error> {
    if sharded {
        world.step_with_shards(
            TOPOLOGY_DT,
            &mut [Vec::new(), Vec::new()],
            &SequentialExecutor,
            2,
        )
    } else {
        world.step(TOPOLOGY_DT)
    }
}

fn standalone_pressure()
-> Result<crate::aerodynamics::pressure::PressureWrench, Box<dyn std::error::Error>> {
    use crate::aerodynamics::geometry::GeometryOwner;
    let mut control = wind_world()?;
    let (reference, reference_part) = clustered_mesh_body(&mut control, Vec3::ZERO)?;
    topology_step(&mut control, false)?;
    let reference_load = control
        .owner_mesh_pressure(reference)
        .ok_or("reference load")?;
    assert_eq!(reference_load.len(), 1);
    assert_eq!(reference_load[0].0, GeometryOwner::Part(reference_part));
    let expected = reference_load[0].1;
    assert!(expected.force.length() > 1.0);
    assert!(expected.moment.length() > 1.0);
    Ok(expected)
}

#[rstest::rstest]
#[case::ordinary(false)]
#[case::sharded(true)]
fn merging_bodies_drops_absorbed_binding_and_stales_the_survivor(
    #[case] sharded: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::geometry::GeometryOwner;
    let expected = standalone_pressure()?;
    let mut world = wind_world()?;
    let (first, first_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let (second, second_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let first_mesh = part_mesh(&world, first_part)?;
    let first_shape = world.cook_shape(&[source(&first_mesh, Vec3::ZERO, 0)?], false)?;
    world.bind_shape(first, first_shape, vec![first_part])?;
    let second_mesh = part_mesh(&world, second_part)?;
    let second_shape = world.cook_shape(&[source(&second_mesh, Vec3::ZERO, 0)?], false)?;
    world.bind_shape(second, second_shape, vec![second_part])?;

    world.merge_clusters(first, second, merge_weld())?;
    assert!(
        world.shape_binding(second).is_none(),
        "absorbed body must not keep its binding"
    );
    assert!(
        world.shape_binding(first).is_some(),
        "binding is stored until the next step validates it"
    );
    topology_step(&mut world, sharded)?;
    assert!(
        world.shape_binding(first).is_none(),
        "membership change must stale the surviving binding"
    );
    let velocity = body_velocity(&world, first)?;
    assert!(
        velocity.length() > 1e-6,
        "merged cluster must still receive pressure, got {velocity:?}"
    );
    assert!(world.body(second).is_none());
    let loads = world.owner_mesh_pressure(first).ok_or("merged loads")?;
    assert_eq!(loads, &[(GeometryOwner::Part(first_part), expected)]);
    Ok(())
}

#[rstest::rstest]
#[case::ordinary(false)]
#[case::sharded(true)]
fn splitting_a_cluster_preserves_pressure_owners(
    #[case] sharded: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut world = wind_world()?;
    let (first, first_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let (second, second_part) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let first_mesh = part_mesh(&world, first_part)?;
    let first_shape = world.cook_shape(&[source(&first_mesh, Vec3::ZERO, 0)?], false)?;
    world.bind_shape(first, first_shape, vec![first_part])?;
    world.merge_clusters(first, second, merge_weld())?;
    let bridge = world.cluster(first).ok_or("cluster")?.welds()[0];
    let bodies = world.break_weld(bridge)?;
    assert_eq!(bodies[0], first);
    let new_body = *bodies.get(1).ok_or("split body")?;
    assert!(world.shape_binding(first).is_some());
    assert!(world.shape_binding(new_body).is_none());
    topology_step(&mut world, sharded)?;
    assert!(
        world.shape_binding(first).is_some(),
        "the surviving cluster is exactly the bound part, so sharing stays valid"
    );
    assert!(body_velocity(&world, first)?.length() > 1e-6);
    assert!(body_velocity(&world, new_body)?.length() > 1e-6);
    assert_eq!(world.part_body(second_part), Some(new_body));
    let first_load = world.owner_mesh_pressure(first).ok_or("first load")?;
    let second_load = world.owner_mesh_pressure(new_body).ok_or("split load")?;
    use crate::aerodynamics::geometry::GeometryOwner;
    assert_eq!(first_load.len(), 1);
    assert_eq!(second_load.len(), 1);
    assert_eq!(first_load[0].0, GeometryOwner::Part(first_part));
    assert_eq!(second_load[0].0, GeometryOwner::Part(second_part));
    let expected = standalone_pressure()?;
    assert_eq!(first_load[0].1, expected);
    assert_eq!(second_load[0].1, expected);
    Ok(())
}

#[rstest::rstest]
#[case::ordinary(false)]
#[case::sharded(true)]
fn removing_the_root_part_transfers_root_and_stales_binding(
    #[case] sharded: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut world = wind_world()?;
    let (body, root) = clustered_mesh_body(&mut world, Vec3::ZERO)?;
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let child = world.add_cluster_part(root, part_desc(mass), merge_weld())?;
    world.set_part_aerodynamic_mesh(child, Some(cube()?))?;
    let root_mesh = part_mesh(&world, root)?;
    let child_mesh = part_mesh(&world, child)?;
    let sources = vec![
        source(&root_mesh, Vec3::ZERO, 0)?,
        source(&child_mesh, Vec3::ZERO, 1)?,
    ];
    let shape = world.cook_shape(&sources, false)?;
    world.bind_shape(body, shape, vec![root, child])?;
    world.destroy_cluster_part(root)?;
    assert_eq!(
        world.cluster(body).map(|cluster| cluster.root()),
        Some(child)
    );
    topology_step(&mut world, sharded)?;
    assert!(
        world.shape_binding(body).is_none(),
        "part removal must stale the binding"
    );
    assert!(body_velocity(&world, body)?.length() > 1e-6);
    assert!(
        !world
            .part_aerodynamic_geometry
            .as_ref()
            .is_some_and(|geometry| geometry.contains_key(&root)),
        "destroyed root must not leave geometry behind"
    );
    assert!(world.cluster_part(root).is_none());
    use crate::aerodynamics::geometry::GeometryOwner;
    assert_eq!(
        world.owner_mesh_pressure(body),
        Some(&[(GeometryOwner::Part(child), standalone_pressure()?)][..])
    );
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn allocation_world(shared: bool) -> Result<PhysicsWorld, Box<dyn std::error::Error>> {
    const BODIES: usize = 4;
    let mut world = wind_world()?;
    if shared {
        let mesh = Arc::new(cube()?);
        let sources = vec![source(&mesh, Vec3::ZERO, 0)?];
        let shape = world.cook_shape(&sources, false)?;
        for index in 0..BODIES {
            let offset = Vec3::X * (100.0 * index as f64);
            let (body, part) = clustered_shared_body(&mut world, offset, &mesh)?;
            world.bind_shape(body, shape, vec![part])?;
        }
    } else {
        for index in 0..BODIES {
            clustered_mesh_body(&mut world, Vec3::X * (100.0 * index as f64))?;
        }
    }
    Ok(world)
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Run alone: process-wide allocation counter"]
fn shared_shape_warm_steps_add_no_allocations() -> Result<(), Box<dyn std::error::Error>> {
    const STEPS: usize = 16;
    const DT: f64 = 1e-8;
    let mut local = allocation_world(false)?;
    let mut shared = allocation_world(true)?;
    for _ in 0..STEPS {
        local.step(DT)?;
        shared.step(DT)?;
    }
    let measure = |world: &mut PhysicsWorld| {
        super::tests::tracked_world_allocations(|| {
            for _ in 0..STEPS {
                assert!(world.step(DT).is_ok());
            }
        })
    };
    let local_counts = measure(&mut local);
    let shared_counts = measure(&mut shared);
    println!(
        "{STEPS} warm steps, four vessels (allocations, bytes): local={local_counts:?}, shared={shared_counts:?}"
    );
    assert!(shared_counts.0 <= local_counts.0);
    assert!(shared_counts.1 <= local_counts.1);
    for ((_, local_body), (id, shared_body)) in local.bodies().zip(shared.bodies()) {
        assert!(shared.shape_binding(id).is_some());
        assert_eq!(
            local_body.state().linear_velocity(),
            shared_body.state().linear_velocity()
        );
        assert_eq!(
            local_body.state().angular_velocity(),
            shared_body.state().angular_velocity()
        );
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Run alone: compares cold one-step allocations for shared versus body-local vessels"]
fn shared_shape_cold_step_allocates_less_than_body_local() -> Result<(), Box<dyn std::error::Error>>
{
    let mut local = allocation_world(false)?;
    let mut shared = allocation_world(true)?;
    let (local_allocations, local_bytes) = super::tests::tracked_world_allocations(|| {
        assert!(local.step(1e-8).is_ok());
    });
    let (shared_allocations, shared_bytes) = super::tests::tracked_world_allocations(|| {
        assert!(shared.step(1e-8).is_ok());
    });
    println!(
        "cold step, 4 vessels: local={local_allocations} allocations/{local_bytes} bytes, \
         shared={shared_allocations} allocations/{shared_bytes} bytes"
    );
    assert!(
        shared_allocations < local_allocations,
        "shared cooking must avoid per-body preparation allocations: \
         shared={shared_allocations}, local={local_allocations}"
    );
    Ok(())
}
