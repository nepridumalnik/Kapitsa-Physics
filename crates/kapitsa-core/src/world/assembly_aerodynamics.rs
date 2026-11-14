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

pub(super) struct AssemblyAerodynamics {
    sources: Vec<(PartId, Arc<TriangleMesh>, Transform)>,
    cache: PreparedGeometryCache,
}

impl PhysicsWorld {
    /// Replaces one part's physical aerodynamic mesh, in that part's local frame.
    /// `None` disables this part's geometry. Explicit aerodynamic surfaces remain
    /// additive: callers must avoid representing the same load twice.
    /// Part ownership survives assembly merge/split. No solver body is created.
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
            let meshes = self
                .part_aerodynamic_meshes
                .get_or_insert_with(HashMap::new);
            meshes
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
            meshes.insert(part, Arc::new(mesh));
        } else {
            if let Some(meshes) = &mut self.part_aerodynamic_meshes {
                meshes.remove(&part);
            }
        }
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        self.invalidate_body_edit(body);
        Ok(())
    }

    /// Returns the configured physical mesh in the part's local frame.
    pub fn part_aerodynamic_mesh(&self, part: PartId) -> Option<&TriangleMesh> {
        self.cluster_part(part)?;
        self.part_aerodynamic_meshes
            .as_ref()?
            .get(&part)
            .map(Arc::as_ref)
    }
}

pub(super) fn configured(
    entry: &BodyEntry,
    meshes: Option<&HashMap<PartId, Arc<TriangleMesh>>>,
) -> bool {
    let Some(meshes) = meshes else {
        return false;
    };
    entry
        .cluster
        .as_ref()
        .is_some_and(|cluster| cluster.parts().iter().any(|part| meshes.contains_key(part)))
}

pub(super) fn prepare(
    body_id: BodyId,
    entry: &BodyEntry,
    parts: &DensePool<ClusterPart>,
    meshes: Option<&HashMap<PartId, Arc<TriangleMesh>>>,
    caches: &mut Option<HashMap<BodyId, AssemblyAerodynamics>>,
    atmosphere: Atmosphere,
) -> Result<Option<PreparedSurfacePatches>, StepError> {
    let error = || StepError::Aerodynamics {
        body_id,
        cause: AerodynamicsError::ArithmeticOverflow,
    };
    let Some(meshes) = meshes else {
        return Ok(None);
    };
    let Some(cluster) = &entry.cluster else {
        return Ok(None);
    };
    if cluster.parts().len() > 100_000_000 {
        return Err(error());
    }
    if !configured(entry, Some(meshes)) {
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
    let mut sources = Vec::new();
    let mut triangle_count = 0usize;
    for &part in cluster.parts() {
        if let Some(mesh) = meshes.get(&part) {
            triangle_count = triangle_count
                .checked_add(mesh.triangles().len())
                .ok_or_else(error)?;
            if triangle_count
                .checked_mul(triangle_count)
                .and_then(|pairs| pairs.checked_mul(128))
                .is_none_or(|work| work > 100_000_000)
            {
                return Err(error());
            }
            sources.try_reserve(1).map_err(|_| error())?;
            let local = parts
                .get(part.dense)
                .filter(|p| p.body() == body_id)
                .ok_or_else(error)?
                .local_transform();
            sources.push((part, mesh.clone(), local));
        }
    }
    if sources.is_empty() {
        return Ok(None);
    }
    sources.sort_unstable_by_key(|source| source.0);
    let caches = caches.get_or_insert_with(HashMap::new);
    let unchanged = caches.get(&body_id).is_some_and(|cached| {
        cached.sources.len() == sources.len()
            && cached
                .sources
                .iter()
                .zip(&sources)
                .all(|(a, b)| a.0 == b.0 && Arc::ptr_eq(&a.1, &b.1) && a.2 == b.2)
    });
    if unchanged {
        let cached = caches.get_mut(&body_id).ok_or_else(error)?;
        return evaluate(body_id, entry, state.wind_velocity(), cached)
            .map(Some)
            .map_err(|_| error());
    }
    caches.try_reserve(1).map_err(|_| error())?;
    let mut candidate = AssemblyAerodynamics {
        sources,
        cache: PreparedGeometryCache::default(),
    };
    let patches =
        evaluate(body_id, entry, state.wind_velocity(), &mut candidate).map_err(|_| error())?;
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
            local_transform: *local_transform,
            owner: GeometryOwner::Part(*part),
        });
    }
    let config = GeometryConfig {
        cell_size: 0.125,
        max_cells: 1_048_576,
        max_work: 100_000_000,
        open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
        weld_tolerance: None,
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
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                0.001,
                Vec3::ZERO,
            )?));
            let mut bodies = Vec::new();
            let mut owners = Vec::new();
            for _ in 0..2 {
                let mut body = RigidBody::new_dynamic(
                    Transform::identity(),
                    MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
                )?;
                body.set_linear_velocity(Vec3::X * 2000.0)?;
                let id = world.add_body(body)?;
                let part = world.enable_cluster(id)?;
                world.set_part_aerodynamic_mesh(part, Some(plate()?))?;
                bodies.push(id);
                owners.push(part);
            }
            let first = *bodies.first().ok_or("first body")?;
            let second = *bodies.get(1).ok_or("second body")?;
            world.merge_clusters(
                first,
                second,
                WeldDesc {
                    frame_a: JointFrame::new(Vec3::ZERO, kapitsa_math::Quat::IDENTITY),
                    frame_b: JointFrame::new(Vec3::ZERO, kapitsa_math::Quat::IDENTITY),
                    strength: WeldStrength::Unbreakable,
                    stiffness: WeldStiffness {
                        linear: 1.0,
                        angular: 1.0,
                    },
                },
            )?;
            assert_eq!(world.bodies().count(), 1);
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
            assert!(
                prepared
                    .cache
                    .get()
                    .ok_or("geometry")?
                    .occupancy()
                    .iter()
                    .flatten()
                    .all(|owner| *owner == GeometryOwner::Part(expected_owner))
            );
            let weld = *world
                .cluster(first)
                .ok_or("cluster")?
                .welds()
                .first()
                .ok_or("weld")?;
            let split = world.break_weld(weld)?;
            assert_eq!(split.len(), 2);
            for part in owners {
                assert!(world.part_aerodynamic_mesh(part).is_some());
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
            assert!(result.is_err());
            assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            world.set_part_aerodynamic_mesh(part, None)?;
            world.step(1e-5)?;
            world.remove_body(id).ok_or("remove body")?;
            assert!(world.part_aerodynamic_mesh(part).is_none());
            assert!(
                !world
                    .part_aerodynamic_meshes
                    .as_ref()
                    .is_some_and(|m| m.contains_key(&part))
            );
        }
        Ok(())
    }
    #[test]
    fn motion_com_and_unrelated_edits_preserve_assembly_cache_ticket()
    -> Result<(), Box<dyn std::error::Error>> {
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
        world.set_part_aerodynamic_mesh(part, Some(plate()?))?;
        world.step(1e-5)?;
        let cached = world
            .assembly_aerodynamics
            .as_mut()
            .and_then(|c| c.get_mut(&id))
            .ok_or("cache")?;
        let sources = cached.sources.clone();
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
                weld_tolerance: None,
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
        Ok(())
    }
}
