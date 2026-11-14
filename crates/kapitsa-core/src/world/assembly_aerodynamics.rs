use std::{collections::HashMap, sync::Arc};

use kapitsa_math::Vec3;

use super::{BodyEntry, BodyId, ClusterError, PartId, PhysicsWorld, StepError};
use crate::{
    aerodynamics::{
        AerodynamicsError, Atmosphere,
        cache::{GeometryCacheKey, PreparedGeometryCache},
        geometry::{
            GeometryConfig, GeometryError, GeometryInstance, GeometryOwner, OpenMeshPolicy,
            PreparedSurfacePatches,
        },
    },
    cluster::ClusterPart,
    containers::DensePool,
    shape::TriangleMesh,
    transform::Transform,
};

pub(super) enum PartGeometry {
    Mesh(Arc<TriangleMesh>),
    Colliders,
}

pub(super) struct AssemblyAerodynamics {
    sources: Vec<(PartId, Arc<TriangleMesh>, Transform)>,
    cache: PreparedGeometryCache,
    collider_parts: Vec<PartId>,
}

impl PhysicsWorld {
    /// Replaces the part's geometry with a Mesh in its local frame.
    /// `None` disables geometric loads. Explicit surfaces remain additive;
    /// configure each physical contribution once to avoid duplicated loads.
    /// Part ownership survives merge/split; no solver body is created.
    pub fn set_part_aerodynamic_mesh(
        &mut self,
        part: PartId,
        mesh: Option<TriangleMesh>,
    ) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        if let Some(mesh) = mesh {
            let geometry = self
                .part_aerodynamic_geometry
                .get_or_insert_with(HashMap::new);
            geometry
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
            geometry.insert(part, PartGeometry::Mesh(Arc::new(mesh)));
        } else if let Some(geometry) = &mut self.part_aerodynamic_geometry {
            geometry.remove(&part);
        }
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        self.invalidate_body_edit(body);
        Ok(())
    }

    /// Selects actual attached colliders, replacing the part's Mesh mode.
    /// `false` disables collider mode, preserving any already selected Mesh.
    /// Unsupported or empty geometry is an explicit preparation error in air.
    pub fn set_part_aerodynamic_colliders(
        &mut self,
        part: PartId,
        enabled: bool,
    ) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        if self.part_uses_aerodynamic_colliders(part) == enabled {
            return Ok(());
        }
        if enabled {
            let geometry = self
                .part_aerodynamic_geometry
                .get_or_insert_with(HashMap::new);
            geometry
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
            geometry.insert(part, PartGeometry::Colliders);
        } else if let Some(geometry) = &mut self.part_aerodynamic_geometry {
            geometry.remove(&part);
        }
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        self.invalidate_body_edit(body);
        Ok(())
    }

    /// Returns whether actual colliders define this part's geometry.
    pub fn part_uses_aerodynamic_colliders(&self, part: PartId) -> bool {
        self.cluster_part(part).is_some()
            && matches!(
                self.part_aerodynamic_geometry
                    .as_ref()
                    .and_then(|g| g.get(&part)),
                Some(PartGeometry::Colliders)
            )
    }

    /// Returns the configured physical mesh in the part's local frame.
    pub fn part_aerodynamic_mesh(&self, part: PartId) -> Option<&TriangleMesh> {
        self.cluster_part(part)?;
        match self.part_aerodynamic_geometry.as_ref()?.get(&part)? {
            PartGeometry::Mesh(mesh) => Some(mesh),
            PartGeometry::Colliders => None,
        }
    }
}

pub(super) fn configured(
    entry: &BodyEntry,
    geometry: Option<&HashMap<PartId, PartGeometry>>,
) -> bool {
    entry.cluster.as_ref().is_some_and(|cluster| {
        cluster
            .parts()
            .iter()
            .any(|part| geometry.is_some_and(|g| g.contains_key(part)))
    })
}

pub(super) fn prepare(
    body_id: BodyId,
    entry: &BodyEntry,
    parts: &DensePool<ClusterPart>,
    geometry: Option<&HashMap<PartId, PartGeometry>>,
    colliders: &DensePool<super::ColliderEntry>,
    caches: &mut Option<HashMap<BodyId, AssemblyAerodynamics>>,
    atmosphere: Atmosphere,
) -> Result<Option<PreparedSurfacePatches>, StepError> {
    let error = || StepError::Aerodynamics {
        body_id,
        cause: AerodynamicsError::ArithmeticOverflow,
    };
    let geometry_error = |cause| StepError::Aerodynamics {
        body_id,
        cause: AerodynamicsError::Geometry(cause),
    };
    let Some(geometry) = geometry else {
        return Ok(None);
    };
    let Some(cluster) = &entry.cluster else {
        return Ok(None);
    };
    if cluster.parts().len() > 100_000_000 {
        return Err(geometry_error(GeometryError::WorkBudgetExceeded));
    }
    if !configured(entry, Some(geometry)) {
        return Ok(None);
    }
    let center = entry
        .body
        .world_center_of_mass()
        .map_err(|_| error())?
        .ok_or_else(error)?;
    let state = atmosphere
        .state_at(center)
        .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
    if state.density() == 0.0 {
        return Ok(None);
    }
    let caches = caches.get_or_insert_with(HashMap::new);
    let count = cluster
        .parts()
        .iter()
        .filter(|part| geometry.contains_key(part))
        .count();
    let unchanged = caches.get(&body_id).is_some_and(|cached| {
        cached.sources.len() == count
            && cached.sources.iter().all(|(part, mesh, local)| {
                parts
                    .get(part.dense)
                    .is_some_and(|p| p.body() == body_id && p.local_transform() == *local)
                    && match geometry.get(part) {
                        Some(PartGeometry::Mesh(current)) => {
                            cached.collider_parts.binary_search(part).is_err()
                                && Arc::ptr_eq(mesh, current)
                        }
                        Some(PartGeometry::Colliders) => {
                            cached.collider_parts.binary_search(part).is_ok()
                        }
                        None => false,
                    }
            })
    });
    if unchanged {
        return evaluate(
            body_id,
            entry,
            state.wind_velocity(),
            caches.get_mut(&body_id).ok_or_else(error)?,
        )
        .map(Some)
        .map_err(|cause| StepError::Aerodynamics {
            body_id,
            cause: AerodynamicsError::Geometry(cause),
        });
    }
    let mut sources = Vec::new();
    let mut collider_parts = Vec::new();
    let mut triangle_count = 0usize;
    let mut collider_triangle_count = 0usize;
    for &part in cluster.parts() {
        let mesh = match geometry.get(&part) {
            Some(PartGeometry::Mesh(mesh)) => mesh.clone(),
            Some(PartGeometry::Colliders) => {
                let owner = parts
                    .get(part.dense)
                    .filter(|p| p.body() == body_id)
                    .ok_or_else(error)?;
                if owner.colliders().len() > entry.collider_tessellation.max_triangles() {
                    return Err(StepError::Aerodynamics {
                        body_id,
                        cause: AerodynamicsError::TessellationBudgetExceeded,
                    });
                }
                let mut attached = Vec::new();
                attached
                    .try_reserve(owner.colliders().len())
                    .map_err(|_| geometry_error(GeometryError::AllocationFailed))?;
                for collider in owner.colliders() {
                    attached.push(
                        &colliders
                            .get(collider.dense)
                            .filter(|c| c.body == body_id)
                            .ok_or_else(error)?
                            .collider,
                    );
                }
                let mesh = crate::aerodynamics::colliders::collider_mesh(
                    attached,
                    entry.collider_tessellation,
                )
                .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
                collider_triangle_count = collider_triangle_count
                    .checked_add(mesh.triangles().len())
                    .ok_or_else(error)?;
                if collider_triangle_count > entry.collider_tessellation.max_triangles() {
                    return Err(StepError::Aerodynamics {
                        body_id,
                        cause: AerodynamicsError::TessellationBudgetExceeded,
                    });
                }
                collider_parts
                    .try_reserve(1)
                    .map_err(|_| geometry_error(GeometryError::AllocationFailed))?;
                collider_parts.push(part);
                Arc::new(mesh)
            }
            None => continue,
        };
        triangle_count = triangle_count
            .checked_add(mesh.triangles().len())
            .ok_or_else(error)?;
        if triangle_count
            .checked_mul(triangle_count)
            .and_then(|pairs| pairs.checked_mul(128))
            .is_none_or(|work| work > 100_000_000)
        {
            return Err(geometry_error(GeometryError::WorkBudgetExceeded));
        }
        sources
            .try_reserve(1)
            .map_err(|_| geometry_error(GeometryError::AllocationFailed))?;
        let local = parts
            .get(part.dense)
            .filter(|p| p.body() == body_id)
            .ok_or_else(error)?
            .local_transform();
        sources.push((part, mesh, local));
    }
    sources.sort_unstable_by_key(|source| source.0);
    collider_parts.sort_unstable();
    caches
        .try_reserve(1)
        .map_err(|_| geometry_error(GeometryError::AllocationFailed))?;
    let mut candidate = AssemblyAerodynamics {
        sources,
        collider_parts,
        cache: PreparedGeometryCache::default(),
    };
    let patches =
        evaluate(body_id, entry, state.wind_velocity(), &mut candidate).map_err(|cause| {
            StepError::Aerodynamics {
                body_id,
                cause: AerodynamicsError::Geometry(cause),
            }
        })?;
    caches.insert(body_id, candidate);
    Ok(Some(patches))
}

fn evaluate(
    body_id: BodyId,
    entry: &BodyEntry,
    wind: Vec3,
    prepared: &mut AssemblyAerodynamics,
) -> Result<PreparedSurfacePatches, GeometryError> {
    let mut instances = Vec::new();
    instances
        .try_reserve(prepared.sources.len())
        .map_err(|_| GeometryError::AllocationFailed)?;
    for (part, mesh, local_transform) in &prepared.sources {
        instances.push(GeometryInstance {
            mesh,
            local_transform: if prepared.collider_parts.binary_search(part).is_ok() {
                Transform::identity()
            } else {
                *local_transform
            },
            owner: GeometryOwner::Part(*part),
        });
    }
    let config = GeometryConfig {
        cell_size: 0.125,
        max_cells: 1_048_576,
        max_work: 100_000_000,
        open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
        weld_tolerance: if prepared.collider_parts.is_empty() {
            None
        } else {
            Some(1e-10)
        },
        max_patch_work: 100_000_000,
    };
    let geometry = prepared.cache.prepare_instances(
        GeometryCacheKey {
            body: body_id,
            geometry_version: 0,
            config_version: 1,
        },
        &instances,
        config,
    )?;
    let inverse = entry
        .body
        .state()
        .transform()
        .inverse()
        .map_err(|_| GeometryError::ArithmeticOverflow)?;
    let relative = entry
        .body
        .state()
        .linear_velocity()
        .checked_sub(wind)
        .ok_or(GeometryError::ArithmeticOverflow)?;
    let velocity = inverse.transform_vector(relative);
    let omega = inverse.transform_vector(entry.body.state().angular_velocity());
    let center = entry
        .body
        .physics()
        .mass()
        .ok_or(GeometryError::InvalidGeometry)?
        .center_of_mass();
    geometry.surface_patches_with_flow(&instances, config.max_patch_work, |point| {
        point
            .checked_sub(center)
            .and_then(|arm| omega.checked_cross(arm))
            .and_then(|rotation| velocity.checked_add(rotation))
            .ok_or(GeometryError::ArithmeticOverflow)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        aerodynamics::AtmosphericState,
        cluster::{WeldDesc, WeldStiffness, WeldStrength},
        execution::SequentialExecutor,
        joint::JointFrame,
        mass::MassProperties,
        rigid_body::RigidBody,
    };
    use kapitsa_math::Mat3;

    fn weld() -> WeldDesc {
        WeldDesc {
            frame_a: JointFrame::new(Vec3::ZERO, kapitsa_math::Quat::IDENTITY),
            frame_b: JointFrame::new(Vec3::ZERO, kapitsa_math::Quat::IDENTITY),
            strength: WeldStrength::Unbreakable,
            stiffness: WeldStiffness {
                linear: 1.0,
                angular: 1.0,
            },
        }
    }

    fn plate() -> Result<TriangleMesh, crate::shape::ShapeError> {
        TriangleMesh::new(
            vec![
                Vec3::new(0.0, -0.5, -0.5),
                Vec3::new(0.0, 0.5, -0.5),
                Vec3::new(0.0, -0.5, 0.5),
                Vec3::new(0.0, 0.5, 0.5),
            ],
            vec![[0, 1, 2], [1, 3, 2]],
        )
    }

    #[test]
    fn part_mesh_loads_survive_merge_and_split_without_double_counting()
    -> Result<(), Box<dyn std::error::Error>> {
        for mode in 0..3 {
            for sharded in [false, true] {
                let mut world = PhysicsWorld::new();
                world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                    0.001,
                    Vec3::ZERO,
                )?));
                let mut bodies = Vec::new();
                let mut owners = Vec::new();
                for index in 0..2 {
                    let mut body = RigidBody::new_dynamic(
                        Transform::identity(),
                        MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
                    )?;
                    body.set_linear_velocity(Vec3::X * 2000.0)?;
                    let id = world.add_body(body)?;
                    let part = world.enable_cluster(id)?;
                    if mode == 1 || (mode == 2 && index == 1) {
                        let mut collider = crate::collider::Collider::new(
                            crate::shape::Shape::Cuboid(crate::shape::Cuboid::new(1.0, 1.0, 1.0)?),
                            Transform::identity(),
                        );
                        collider.set_mode(crate::collider::CollisionMode::Sensor);
                        world.add_part_collider(part, collider)?;
                        world.set_part_aerodynamic_colliders(part, true)?;
                    } else {
                        world.set_part_aerodynamic_mesh(part, Some(plate()?))?;
                    }
                    bodies.push(id);
                    owners.push(part);
                }
                let first = *bodies.first().ok_or("first body")?;
                let second = *bodies.get(1).ok_or("second body")?;
                let settings = crate::aerodynamics::ColliderTessellation::new(12, 6, 1024)?;
                world.set_collider_tessellation(first, settings)?;
                let before = world.body(first).ok_or("body")?.motion();
                assert_eq!(
                    world.merge_clusters(first, second, weld()),
                    Err(ClusterError::UnsupportedGeometry)
                );
                assert_eq!(world.bodies().count(), 2);
                assert_eq!(world.body(first).ok_or("body")?.motion(), before);
                world.set_collider_tessellation(second, settings)?;
                world.merge_clusters(first, second, weld())?;
                assert_eq!(world.bodies().count(), 1);
                world.set_collider_tessellation(
                    first,
                    crate::aerodynamics::ColliderTessellation::new(12, 6, 12)?,
                )?;
                if mode == 1 {
                    let before = world.body(first).ok_or("body")?.motion();
                    let failed = if sharded {
                        world.step_with_shards(1e-5, &mut [Vec::new()], &SequentialExecutor, 1)
                    } else {
                        world.step(1e-5)
                    };
                    assert_eq!(
                        failed,
                        Err(StepError::Aerodynamics {
                            body_id: first,
                            cause: AerodynamicsError::TessellationBudgetExceeded
                        })
                    );
                    assert_eq!(world.body(first).ok_or("body")?.motion(), before);
                } else {
                    super::super::AerodynamicsTask.execute(&world)?;
                }
                world.set_collider_tessellation(first, settings)?;
                super::super::AerodynamicsTask.execute(&world)?;
                if sharded {
                    world.step_with_shards(1e-5, &mut [Vec::new()], &SequentialExecutor, 1)?;
                } else {
                    world.step(1e-5)?;
                }
                let velocity = world
                    .body(first)
                    .ok_or("merged body")?
                    .state()
                    .linear_velocity()
                    .x;
                assert!(
                    (velocity - (2000.0 - 0.0002)).abs() < 1e-9,
                    "velocity={velocity}"
                );
                let expected_owner = owners.iter().copied().min().ok_or("owner")?;
                let prepared = world
                    .assembly_aerodynamics
                    .as_ref()
                    .and_then(|c| c.get(&first))
                    .ok_or("assembly cache")?;
                let occupancy = prepared.cache.get().ok_or("geometry")?.occupancy();
                if mode != 2 {
                    assert!(
                        occupancy
                            .iter()
                            .flatten()
                            .all(|owner| *owner == GeometryOwner::Part(expected_owner))
                    );
                } else {
                    assert!(occupancy.iter().flatten().all(|owner| {
                        owners
                            .iter()
                            .any(|part| *owner == GeometryOwner::Part(*part))
                    }));
                }
                let weld = *world
                    .cluster(first)
                    .ok_or("cluster")?
                    .welds()
                    .first()
                    .ok_or("weld")?;
                let split = world.break_weld(weld)?;
                assert_eq!(split.len(), 2);
                for body in &split {
                    assert_eq!(
                        world
                            .bodies
                            .get(body.dense)
                            .ok_or("split body")?
                            .collider_tessellation,
                        settings
                    );
                }
                for part in owners {
                    assert!(
                        world.part_aerodynamic_mesh(part).is_some()
                            || world.part_uses_aerodynamic_colliders(part)
                    );
                    assert!(
                        world
                            .body(world.cluster_part(part).ok_or("part")?.body())
                            .is_some()
                    );
                }
                if sharded {
                    world.step_with_shards(1e-5, &mut [Vec::new()], &SequentialExecutor, 1)?;
                } else {
                    world.step(1e-5)?;
                }
                assert_eq!(world.bodies().count(), 2);
            }
        }
        Ok(())
    }
    #[test]
    fn failed_part_preparation_preserves_motion_and_removal_cleans_configuration()
    -> Result<(), Box<dyn std::error::Error>> {
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                0.001,
                Vec3::ZERO,
            )?));
            let mut body = RigidBody::new_dynamic(
                Transform::identity(),
                MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY)?,
            )?;
            body.set_linear_velocity(Vec3::X * 2000.0)?;
            let id = world.add_body(body)?;
            let part = world.enable_cluster(id)?;
            let mesh =
                TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y * 1e10, Vec3::Z], vec![[0, 1, 2]])?;
            world.set_part_aerodynamic_mesh(part, Some(mesh))?;
            let before = world.body(id).ok_or("body")?.motion();
            let mut incompatible = RigidBody::from_motion(before);
            incompatible.set_aerodynamics(crate::aerodynamics::Aerodynamics::Mesh(plate()?));
            assert_eq!(
                world.replace_body_state(id, incompatible),
                Err(ClusterError::UnsupportedGeometry)
            );
            assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            let result = if sharded {
                world.step_with_shards(1e-5, &mut [Vec::new()], &SequentialExecutor, 1)
            } else {
                world.step(1e-5)
            };
            assert_eq!(
                result,
                Err(StepError::Aerodynamics {
                    body_id: id,
                    cause: AerodynamicsError::Geometry(GeometryError::CellBudgetExceeded)
                })
            );
            if let Err(StepError::Aerodynamics { cause, .. }) = result {
                assert_eq!(
                    std::error::Error::source(&cause)
                        .and_then(|source| source.downcast_ref::<GeometryError>()),
                    Some(&GeometryError::CellBudgetExceeded)
                );
            }
            assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            world.set_part_aerodynamic_mesh(part, None)?;
            world.step(1e-5)?;
            world.remove_body(id).ok_or("remove body")?;
            assert!(world.part_aerodynamic_mesh(part).is_none());
            assert!(
                !world
                    .part_aerodynamic_geometry
                    .as_ref()
                    .is_some_and(|m| m.contains_key(&part))
            );
        }
        Ok(())
    }
    #[test]
    fn motion_com_and_unrelated_edits_preserve_assembly_cache_ticket()
    -> Result<(), Box<dyn std::error::Error>> {
        for collider_mode in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                0.001,
                Vec3::ZERO,
            )?));
            let mut body = RigidBody::new_dynamic(
                Transform::identity(),
                MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
            )?;
            body.set_linear_velocity(Vec3::X * 2000.0)?;
            let id = world.add_body(body)?;
            let part = world.enable_cluster(id)?;
            if collider_mode {
                world.add_part_collider(
                    part,
                    crate::collider::Collider::new(
                        crate::shape::Shape::Cuboid(crate::shape::Cuboid::new(1.0, 1.0, 1.0)?),
                        Transform::identity(),
                    ),
                )?;
                world.set_part_aerodynamic_colliders(part, true)?;
            } else {
                world.set_part_aerodynamic_mesh(part, Some(plate()?))?;
            }
            world.step(1e-5)?;
            let cached = world
                .assembly_aerodynamics
                .as_mut()
                .and_then(|c| c.get_mut(&id))
                .ok_or("cache")?;
            let sources = cached.sources.clone();
            let weld_tolerance = if collider_mode { Some(1e-10) } else { None };
            let ticket = cached.cache.request(
                GeometryCacheKey {
                    body: id,
                    geometry_version: 0,
                    config_version: 1,
                },
                GeometryConfig {
                    cell_size: 0.125,
                    max_cells: 1_048_576,
                    max_work: 100_000_000,
                    open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
                    weld_tolerance,
                    max_patch_work: 100_000_000,
                },
            );
            let pose = Transform::new(
                Vec3::X,
                kapitsa_math::Quat::from_axis_angle(Vec3::X, 0.1).ok_or("rotation")?,
            )?;
            world.edit_body(id, |motion| motion.set_transform(pose))?;
            world.set_part_mass(
                part,
                MassProperties::new(200.0, Vec3::Y * 0.1, Mat3::IDENTITY * 200.0)?,
                crate::cluster::MassChangeMode::KeepVelocityField,
            )?;
            if collider_mode {
                world.set_part_aerodynamic_colliders(part, true)?;
            }
            world.add_body(RigidBody::new_static(Transform::identity()))?;
            world.step(1e-5)?;
            let instances: Vec<_> = sources
                .iter()
                .map(|(part, mesh, local)| GeometryInstance {
                    mesh,
                    local_transform: *local,
                    owner: GeometryOwner::Part(*part),
                })
                .collect();
            let prepared = ticket.prepare_instances(&instances)?;
            let cached = world
                .assembly_aerodynamics
                .as_mut()
                .and_then(|c| c.get_mut(&id))
                .ok_or("cache")?;
            assert!(
                cached.cache.publish(prepared),
                "geometry cache identity changed for motion/COM/unrelated edit"
            );
        }
        Ok(())
    }
    #[test]
    fn enabling_cluster_moves_existing_primary_geometry_without_copying()
    -> Result<(), Box<dyn std::error::Error>> {
        for collider_mode in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                0.001,
                Vec3::ZERO,
            )?));
            let mut body = RigidBody::new_dynamic(
                Transform::identity(),
                MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
            )?;
            body.set_linear_velocity(Vec3::X * 2000.0)?;
            let pointer = if collider_mode {
                body.set_aerodynamics(crate::aerodynamics::Aerodynamics::Colliders);
                None
            } else {
                let mesh = plate()?;
                let pointer = mesh.vertices().as_ptr();
                body.set_aerodynamics(crate::aerodynamics::Aerodynamics::Mesh(mesh));
                Some(pointer)
            };
            let id = world.add_body(body)?;
            if collider_mode {
                world.add_collider(
                    id,
                    crate::collider::Collider::new(
                        crate::shape::Shape::Cuboid(crate::shape::Cuboid::new(1.0, 1.0, 1.0)?),
                        Transform::identity(),
                    ),
                )?;
            }
            world.step(1e-5)?;
            let speed = world.body(id).ok_or("body")?.state().linear_velocity().x;
            let part = world.enable_cluster(id)?;
            assert!(matches!(
                world.body(id).ok_or("body")?.aerodynamics(),
                crate::aerodynamics::Aerodynamics::None
            ));
            if let Some(pointer) = pointer {
                assert_eq!(
                    world
                        .part_aerodynamic_mesh(part)
                        .ok_or("part mesh")?
                        .vertices()
                        .as_ptr(),
                    pointer
                );
            } else {
                assert!(world.part_uses_aerodynamic_colliders(part));
            }
            world.step(1e-5)?;
            let actual = world.body(id).ok_or("body")?.state().linear_velocity().x;
            let expected = speed - 0.001 * speed * speed * 1e-5 / 100.0;
            assert!(
                (actual - expected).abs() < 1e-9,
                "actual={actual}, expected={expected}"
            );
        }
        Ok(())
    }
    #[test]
    fn part_collider_edits_invalidate_preparation_and_errors_preserve_motion()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::{
            collider::Collider,
            shape::{Capsule, Cuboid, Shape},
        };
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                0.001,
                Vec3::ZERO,
            )?));
            let mut body = RigidBody::new_dynamic(
                Transform::identity(),
                MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
            )?;
            body.set_linear_velocity(Vec3::X * 2000.0)?;
            let id = world.add_body(body)?;
            let part = world.enable_cluster(id)?;
            let collider = world.add_part_collider(
                part,
                Collider::new(
                    Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0)?),
                    Transform::identity(),
                ),
            )?;
            world.set_part_aerodynamic_mesh(part, Some(plate()?))?;
            world.set_part_aerodynamic_colliders(part, true)?;
            assert!(world.part_aerodynamic_mesh(part).is_none());
            let step = |world: &mut PhysicsWorld| {
                if sharded {
                    world.step_with_shards(1e-5, &mut [Vec::new()], &SequentialExecutor, 1)
                } else {
                    world.step(1e-5)
                }
            };
            step(&mut world)?;
            let old_mesh = world
                .assembly_aerodynamics
                .as_ref()
                .and_then(|c| c.get(&id))
                .and_then(|c| c.sources.first())
                .ok_or("prepared source")?
                .1
                .clone();
            let other = world.add_body(RigidBody::new_static(Transform::identity()))?;
            let foreign_owner = world.enable_cluster_with_mass(
                other,
                MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
            )?;
            assert_eq!(
                world.set_part_collider_transform(foreign_owner, collider, Transform::identity()),
                Err(ClusterError::InvalidTopology)
            );
            assert!(
                world
                    .assembly_aerodynamics
                    .as_ref()
                    .is_some_and(|c| c.contains_key(&id))
            );
            world.set_part_collider_transform(
                part,
                collider,
                Transform::new(Vec3::Y, kapitsa_math::Quat::IDENTITY)?,
            )?;
            assert!(
                !world
                    .assembly_aerodynamics
                    .as_ref()
                    .is_some_and(|c| c.contains_key(&id))
            );
            step(&mut world)?;
            let mesh = &world
                .assembly_aerodynamics
                .as_ref()
                .and_then(|c| c.get(&id))
                .and_then(|c| c.sources.first())
                .ok_or("prepared source")?
                .1;
            assert!(!Arc::ptr_eq(&old_mesh, mesh));
            assert!(world.body(id).ok_or("body")?.state().angular_velocity().z > 0.0);
            world.set_collider_tessellation(
                id,
                crate::aerodynamics::ColliderTessellation::new(12, 6, 1024)?,
            )?;
            assert!(
                !world
                    .assembly_aerodynamics
                    .as_ref()
                    .is_some_and(|c| c.contains_key(&id))
            );
            step(&mut world)?;
            world.remove_part_collider(collider)?;
            let before = world.body(id).ok_or("body")?.motion();
            assert!(step(&mut world).is_err());
            assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            let unsupported = world.add_part_collider(
                part,
                Collider::new(
                    Shape::Capsule(Capsule::new(-Vec3::Z, Vec3::Z, 0.5)?),
                    Transform::identity(),
                ),
            )?;
            assert_eq!(
                step(&mut world),
                Err(StepError::Aerodynamics {
                    body_id: id,
                    cause: AerodynamicsError::UnsupportedModel
                })
            );
            assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            world.remove_part_collider(unsupported)?;
            world.set_part_aerodynamic_mesh(part, Some(plate()?))?;
            assert!(!world.part_uses_aerodynamic_colliders(part));
            step(&mut world)?;
        }
        Ok(())
    }
    #[test]
    fn transformed_part_geometry_has_correct_body_frame_force_and_com_moment()
    -> Result<(), Box<dyn std::error::Error>> {
        for collider_mode in [false, true] {
            for sharded in [false, true] {
                let mut world = PhysicsWorld::new();
                let atmosphere = crate::aerodynamics::AtmosphericState::new(0.001, Vec3::ZERO)?;
                world.set_atmosphere(super::super::test_atmosphere(atmosphere));
                let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
                let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
                body.set_linear_velocity(Vec3::X * 2000.0)?;
                let id = world.add_body(body)?;
                let root = world.enable_cluster(id)?;
                let angle = 0.3_f64;
                let rotation =
                    kapitsa_math::Quat::from_axis_angle(Vec3::X, angle).ok_or("rotation")?;
                let colliders = if collider_mode {
                    vec![crate::collider::Collider::new(
                        crate::shape::Shape::Cuboid(crate::shape::Cuboid::new(1.0, 1.0, 1.0)?),
                        Transform::new(Vec3::Y, kapitsa_math::Quat::IDENTITY)?,
                    )]
                } else {
                    Vec::new()
                };
                let part = world.add_cluster_part(
                    root,
                    crate::cluster::ClusterPartDesc {
                        local_transform: Transform::new(Vec3::Y * 2.0, rotation)?,
                        mass,
                        colliders,
                        linear_velocity: Vec3::X * 2000.0,
                        angular_velocity: Vec3::ZERO,
                    },
                    WeldDesc {
                        frame_a: JointFrame::new(Vec3::Y * 2.0, rotation),
                        ..weld()
                    },
                )?;
                if collider_mode {
                    world.set_part_aerodynamic_colliders(part, true)?;
                } else {
                    let original = plate()?;
                    let mesh = TriangleMesh::new(
                        original.vertices().iter().map(|p| *p + Vec3::Y).collect(),
                        original.triangles().to_vec(),
                    )?;
                    world.set_part_aerodynamic_mesh(part, Some(mesh))?;
                }
                let entry = world.bodies.get(id.dense).ok_or("body")?;
                let transform = entry.body.state().transform();
                let center_of_mass = entry.body.physics().mass().ok_or("mass")?.center_of_mass();
                let patches = prepare(
                    id,
                    entry,
                    &world.parts,
                    world.part_aerodynamic_geometry.as_ref(),
                    &world.colliders,
                    &mut world.assembly_aerodynamics,
                    world.atmosphere,
                )?
                .ok_or("patches")?;
                assert!(patches.iter().all(|p| p.owner == GeometryOwner::Part(part)));
                let wrench = crate::aerodynamics::pressure::evaluate_prepared_mesh(
                    &patches,
                    crate::aerodynamics::pressure::PressureFlow {
                        atmosphere,
                        sound_speed: 100.0,
                        transform,
                        center_of_mass,
                        linear_velocity: Vec3::X * 2000.0,
                        angular_velocity: Vec3::ZERO,
                    },
                )?;
                let moment = Vec3::new(0.0, -angle.sin() * 4000.0, (1.0 + angle.cos()) * 4000.0);
                assert!((wrench.force + Vec3::X * 4000.0).length() < 1e-8);
                assert!(
                    (wrench.moment - moment).length() < 1e-8,
                    "actual={:?}, expected={moment:?}",
                    wrench.moment
                );
                if sharded {
                    world.step_with_shards(1e-8, &mut [Vec::new()], &SequentialExecutor, 1)?;
                } else {
                    world.step(1e-8)?;
                }
                let state = world.body(id).ok_or("body")?.state();
                assert!((state.linear_velocity().x - (2000.0 - 2e-7)).abs() < 1e-9);
                let angular = Vec3::new(0.0, moment.y / 200.0, moment.z / 400.0) * 1e-8;
                assert!((state.angular_velocity() - angular).length() < 1e-12);
                assert_eq!(world.bodies().count(), 1);
            }
        }
        Ok(())
    }
}
