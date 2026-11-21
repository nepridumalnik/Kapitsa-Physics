use super::*;
use crate::aerodynamics::geometry::OpenMeshPolicy;
use kapitsa_math::{Quat, Vec3};

fn config() -> GeometryConfig {
    GeometryConfig {
        cell_size: 0.25,
        max_cells: 100_000,
        max_work: 10_000_000,
        open_mesh_policy: OpenMeshPolicy::Reject,
        weld_tolerance: None,
        max_patch_work: 100_000,
    }
}

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

fn source(mesh: &Arc<TriangleMesh>, offset: Vec3, slot: u32) -> Result<CookedSource, crate::Error> {
    Ok(CookedSource {
        mesh: Arc::clone(mesh),
        local_transform: Transform::new(offset, Quat::IDENTITY)?,
        slot: OwnerSlot::new(slot),
    })
}

fn sources() -> Result<(Arc<TriangleMesh>, Vec<CookedSource>), crate::Error> {
    let mesh = Arc::new(cube()?);
    let list = vec![
        source(&mesh, Vec3::ZERO, 0)?,
        source(&mesh, Vec3::Y * 3.0, 1)?,
    ];
    Ok((mesh, list))
}

fn geometry_cause(error: &crate::Error) -> Option<&GeometryError> {
    error.downcast_ref::<GeometryError>()
}

#[test]
fn cook_matches_direct_slot_preparation() -> Result<(), Box<dyn std::error::Error>> {
    let (mesh, list) = sources()?;
    let shape = CookedShape::cook(&list, config(), false)?;
    let instances = [
        GeometryInstance {
            mesh: &mesh,
            local_transform: Transform::identity(),
            owner: GeometryOwner::Slot(OwnerSlot::new(0)),
        },
        GeometryInstance {
            mesh: &mesh,
            local_transform: Transform::new(Vec3::Y * 3.0, Quat::IDENTITY)?,
            owner: GeometryOwner::Slot(OwnerSlot::new(1)),
        },
    ];
    let direct = PreparedGeometry::prepare(&instances, config())?;
    assert_eq!(shape.geometry(), &direct);
    assert_eq!(shape.config(), config());
    assert!(!shape.oriented_cavities());
    assert_eq!(shape.revision(), 1);
    assert_eq!(shape.sources().len(), 2);
    assert!(Arc::ptr_eq(&shape.sources()[0].mesh, &mesh));
    assert!(Arc::ptr_eq(&shape.sources()[1].mesh, &mesh));
    assert_eq!(shape.sources()[1].slot, OwnerSlot::new(1));
    Ok(())
}

#[test]
fn cook_rejects_invalid_slot_layout() -> Result<(), Box<dyn std::error::Error>> {
    let mesh = Arc::new(cube()?);
    let duplicate = vec![
        source(&mesh, Vec3::ZERO, 0)?,
        source(&mesh, Vec3::Y * 3.0, 0)?,
    ];
    let error = CookedShape::cook(&duplicate, config(), false)
        .err()
        .ok_or("duplicate slots must fail")?;
    assert_eq!(
        geometry_cause(&error),
        Some(&GeometryError::InvalidGeometry)
    );
    let sparse = vec![source(&mesh, Vec3::ZERO, 1)?];
    let error = CookedShape::cook(&sparse, config(), false)
        .err()
        .ok_or("sparse slots must fail")?;
    assert_eq!(
        geometry_cause(&error),
        Some(&GeometryError::InvalidGeometry)
    );
    let unordered = vec![
        source(&mesh, Vec3::ZERO, 1)?,
        source(&mesh, Vec3::Y * 3.0, 0)?,
    ];
    let error = CookedShape::cook(&unordered, config(), false)
        .err()
        .ok_or("unordered slots must fail")?;
    assert_eq!(
        geometry_cause(&error),
        Some(&GeometryError::InvalidGeometry)
    );
    let error = CookedShape::cook(&[], config(), false)
        .err()
        .ok_or("empty sources must fail")?;
    assert_eq!(geometry_cause(&error), Some(&GeometryError::EmptyGeometry));
    Ok(())
}

#[test]
fn oriented_cavities_cook_matches_direct_preparation() -> Result<(), Box<dyn std::error::Error>> {
    let shell = cube()?;
    let vertices = shell
        .vertices()
        .iter()
        .copied()
        .chain(
            shell
                .vertices()
                .iter()
                .map(|v| Vec3::splat(0.25) + *v * 0.5),
        )
        .collect();
    let faces = shell
        .triangles()
        .iter()
        .copied()
        .chain(
            shell
                .triangles()
                .iter()
                .map(|&[a, b, c]| [a + 8, c + 8, b + 8]),
        )
        .collect();
    let mesh = Arc::new(TriangleMesh::new(vertices, faces)?);
    let list = vec![source(&mesh, Vec3::ZERO, 0)?];
    let oriented = CookedShape::cook(&list, config(), true)?;
    let instances = [GeometryInstance {
        mesh: &mesh,
        local_transform: Transform::identity(),
        owner: GeometryOwner::Slot(OwnerSlot::new(0)),
    }];
    let direct = PreparedGeometry::prepare_with_oriented_cavities(&instances, config())?;
    assert_eq!(oriented.geometry(), &direct);
    assert!(oriented.oriented_cavities());
    let filled = CookedShape::cook(&list, config(), false)?;
    assert!(oriented.geometry().volume() < filled.geometry().volume());
    Ok(())
}

#[test]
fn registry_identity_generations_and_foreign_rejection() -> Result<(), Box<dyn std::error::Error>> {
    let (_, list) = sources()?;
    let mut registry = ShapeRegistry::new();
    assert!(registry.is_empty());
    let first = registry.cook(&list, config(), false)?;
    assert_eq!(registry.len(), 1);
    assert!(registry.contains(first));
    assert!(registry.get(first).is_some());
    let removed = registry.remove(first).ok_or("removed shape")?;
    assert_eq!(removed.revision(), 1);
    assert!(!registry.contains(first));
    assert!(registry.get(first).is_none());
    assert!(registry.is_empty());
    let second = registry.cook(&list, config(), false)?;
    assert_ne!(first, second);
    assert!(registry.get(first).is_none());
    assert!(registry.get(second).is_some());
    let mut foreign = ShapeRegistry::new();
    let foreign_id = foreign.cook(&list, config(), false)?;
    assert!(registry.get(foreign_id).is_none());
    assert!(!registry.contains(foreign_id));
    assert!(registry.remove(foreign_id).is_none());
    assert_eq!(foreign.len(), 1);
    Ok(())
}

#[test]
fn replace_bumps_revision_and_preserves_identity() -> Result<(), Box<dyn std::error::Error>> {
    let (mesh, list) = sources()?;
    let mut registry = ShapeRegistry::new();
    let id = registry.cook(&list, config(), false)?;
    let moved = vec![
        source(&mesh, Vec3::ZERO, 0)?,
        source(&mesh, Vec3::Y * 4.0, 1)?,
    ];
    let replacement = CookedShape::cook(&moved, config(), false)?;
    let revision = registry.replace(id, replacement)?;
    assert_eq!(revision, 2);
    let stored = registry.get(id).ok_or("replaced shape")?;
    assert_eq!(stored.revision(), 2);
    assert_eq!(
        stored.sources()[1].local_transform,
        Transform::new(Vec3::Y * 4.0, Quat::IDENTITY)?
    );
    let again = CookedShape::cook(&list, config(), false)?;
    assert_eq!(registry.replace(id, again)?, 3);
    assert_eq!(registry.len(), 1);
    Ok(())
}

#[test]
fn replace_rejects_stale_and_foreign_without_mutation() -> Result<(), Box<dyn std::error::Error>> {
    let (_, list) = sources()?;
    let mut registry = ShapeRegistry::new();
    let id = registry.cook(&list, config(), false)?;
    let mut foreign = ShapeRegistry::new();
    let foreign_id = foreign.cook(&list, config(), false)?;
    let replacement = CookedShape::cook(&list, config(), false)?;
    let error = registry
        .replace(foreign_id, replacement)
        .err()
        .ok_or("foreign replace must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::StaleShape)
    );
    assert_eq!(registry.get(id).ok_or("shape")?.revision(), 1);
    registry.remove(id);
    let replacement = CookedShape::cook(&list, config(), false)?;
    let error = registry
        .replace(id, replacement)
        .err()
        .ok_or("stale replace must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::StaleShape)
    );
    assert!(registry.is_empty());
    assert_eq!(
        foreign.get(foreign_id).ok_or("foreign shape")?.revision(),
        1
    );
    Ok(())
}

#[test]
fn failed_cook_preserves_registry() -> Result<(), Box<dyn std::error::Error>> {
    let (_, list) = sources()?;
    let mut registry = ShapeRegistry::new();
    let id = registry.cook(&list, config(), false)?;
    let mut invalid = config();
    invalid.cell_size = 0.0;
    assert_eq!(
        registry.cook(&list, invalid, false),
        Err(crate::Error::from(GeometryError::InvalidConfig))
    );
    assert_eq!(registry.len(), 1);
    assert_eq!(registry.get(id).ok_or("shape")?.revision(), 1);
    let mesh = Arc::new(cube()?);
    let duplicate = vec![
        source(&mesh, Vec3::ZERO, 0)?,
        source(&mesh, Vec3::Y * 3.0, 0)?,
    ];
    assert_eq!(
        registry.cook(&duplicate, config(), false),
        Err(crate::Error::from(GeometryError::InvalidGeometry))
    );
    assert_eq!(registry.len(), 1);
    Ok(())
}

#[test]
fn revision_exhaustion_is_rejected_without_mutation() -> Result<(), Box<dyn std::error::Error>> {
    let (mesh, list) = sources()?;
    let instances = [GeometryInstance {
        mesh: &mesh,
        local_transform: Transform::identity(),
        owner: GeometryOwner::Slot(OwnerSlot::new(0)),
    }];
    let exhausted = CookedShape {
        geometry: PreparedGeometry::prepare(&instances, config())?,
        config: config(),
        oriented_cavities: false,
        sources: list.clone(),
        revision: u64::MAX,
    };
    let mut registry = ShapeRegistry::new();
    let id = registry.insert(exhausted)?;
    let replacement = CookedShape::cook(&list, config(), false)?;
    let error = registry
        .replace(id, replacement)
        .err()
        .ok_or("exhausted revision must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::RevisionExhausted)
    );
    let stored = registry.get(id).ok_or("shape")?;
    assert_eq!(stored.revision(), u64::MAX);
    assert_eq!(stored.sources().len(), 2);
    Ok(())
}

/// Creates `count` clustered bodies and returns their root parts.
fn bound_parts(
    count: usize,
) -> Result<(crate::world::PhysicsWorld, Vec<PartId>), Box<dyn std::error::Error>> {
    use crate::{mass::MassProperties, rigid_body::RigidBody, world::PhysicsWorld};
    use kapitsa_math::Mat3;
    let mut world = PhysicsWorld::new();
    let mut parts = Vec::new();
    for _ in 0..count {
        let body = world.add_body(RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
        )?)?;
        parts.push(world.enable_cluster(body)?);
    }
    Ok((world, parts))
}

#[test]
fn binding_maps_slots_and_writes_part_owned_instances() -> Result<(), Box<dyn std::error::Error>> {
    let (mesh, list) = sources()?;
    let mut registry = ShapeRegistry::new();
    let id = registry.cook(&list, config(), false)?;
    let (_world, parts) = bound_parts(2)?;
    let shape = registry.get(id).ok_or("shape")?;
    let binding = ShapeBinding::new(id, shape.revision(), parts.clone())?;
    assert_eq!(binding.shape(), id);
    assert_eq!(binding.revision(), 1);
    assert_eq!(binding.parts(), parts.as_slice());
    assert_eq!(binding.part(OwnerSlot::new(1)), Some(parts[1]));
    assert_eq!(binding.part(OwnerSlot::new(2)), None);
    assert_eq!(binding.slot_of(parts[0]), Some(OwnerSlot::new(0)));
    let mut instances = Vec::new();
    binding.write_instances(shape, &mut instances)?;
    assert_eq!(instances.len(), 2);
    assert_eq!(instances[0].owner, GeometryOwner::Part(parts[0]));
    assert_eq!(instances[1].owner, GeometryOwner::Part(parts[1]));
    assert!(std::ptr::eq(instances[0].mesh, &*shape.sources()[0].mesh));
    assert!(std::ptr::eq(instances[0].mesh, &*mesh));
    assert_eq!(
        instances[1].local_transform,
        Transform::new(Vec3::Y * 3.0, Quat::IDENTITY)?
    );
    Ok(())
}

#[test]
fn binding_rejects_empty_and_duplicate_parts() -> Result<(), Box<dyn std::error::Error>> {
    let (_, list) = sources()?;
    let mut registry = ShapeRegistry::new();
    let id = registry.cook(&list, config(), false)?;
    let (_, parts) = bound_parts(2)?;
    let error = ShapeBinding::new(id, 1, Vec::new())
        .err()
        .ok_or("empty binding must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::InvalidBinding)
    );
    let error = ShapeBinding::new(id, 1, vec![parts[0], parts[0]])
        .err()
        .ok_or("duplicate binding must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::InvalidBinding)
    );
    Ok(())
}

#[test]
fn binding_write_rejects_stale_revision_and_wrong_arity() -> Result<(), Box<dyn std::error::Error>>
{
    let (_, list) = sources()?;
    let mut registry = ShapeRegistry::new();
    let id = registry.cook(&list, config(), false)?;
    let (_, parts) = bound_parts(2)?;
    let binding = ShapeBinding::new(id, 1, parts.clone())?;
    {
        let mut instances = Vec::new();
        binding.write_instances(registry.get(id).ok_or("shape")?, &mut instances)?;
        assert_eq!(instances.len(), 2);
    }
    let replacement = CookedShape::cook(&list, config(), false)?;
    assert_eq!(registry.replace(id, replacement)?, 2);
    let mut instances = Vec::new();
    let error = binding
        .write_instances(registry.get(id).ok_or("shape")?, &mut instances)
        .err()
        .ok_or("stale binding must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::StaleBinding)
    );
    assert!(instances.is_empty());
    let mut instances = Vec::new();
    let fresh = ShapeBinding::new(id, 2, vec![parts[0]])?;
    let error = fresh
        .write_instances(registry.get(id).ok_or("shape")?, &mut instances)
        .err()
        .ok_or("wrong arity must fail")?;
    assert_eq!(
        error.downcast_ref::<ShapeRegistryError>(),
        Some(&ShapeRegistryError::InvalidBinding)
    );
    Ok(())
}

#[test]
fn assembly_pressure_config_is_the_documented_union_configuration() {
    let config = assembly_pressure_config();
    assert_eq!(config.cell_size, 0.125);
    assert_eq!(config.max_cells, 1_048_576);
    assert_eq!(config.max_work, 100_000_000);
    assert_eq!(config.open_mesh_policy, OpenMeshPolicy::SurfaceOnly);
    assert_eq!(config.weld_tolerance, None);
    assert_eq!(config.max_patch_work, 100_000_000);
}
