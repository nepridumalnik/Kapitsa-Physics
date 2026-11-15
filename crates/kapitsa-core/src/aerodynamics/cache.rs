//! Cache for body-local meshes and unions of assembly-local instances.
//!
//! Integration boundary: the world must own one cache per body, supply its full
//! `BodyId`, and advance geometry/config versions when replacing a mesh or its
//! preparation settings. Assembly identity also covers membership, owners and
//! instance-local transforms. Versions must not be reused for different geometry.
//! Remove/drop the cache when the body is removed or switches models. Motion,
//! world pose and COM are deliberately absent from the identity. The world uses
//! prepared geometry for Mesh/Colliders pressure evaluation in nonzero density;
//! vacuum samples skip preparation and primary geometry force evaluation.
//!
//! A request borrows no body or mesh and may be sent to a worker. Preparation
//! borrows the mesh only while computing. Publish under the world's exclusive
//! cache access after selecting the latest identity: comparison and replacement
//! are one operation. A failed/stale result never replaces the last good entry.

use std::sync::Arc;

use super::geometry::{
    GeometryConfig, GeometryError, GeometryInstance, GeometryOwner, PreparedGeometry,
};
use crate::{shape::TriangleMesh, transform::Transform, world::BodyId};

/// Caller-maintained identity. `BodyId` includes world and slot generation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GeometryCacheKey {
    pub body: BodyId,
    pub geometry_version: u64,
    pub config_version: u64,
}

/// Owned preparation ticket. Its private incarnation also rejects A→B→A races.
#[derive(Clone, Debug)]
pub struct GeometryPreparation {
    key: GeometryCacheKey,
    config: GeometryConfig,
    incarnation: Arc<()>,
}

impl GeometryPreparation {
    pub fn key(&self) -> GeometryCacheKey {
        self.key
    }

    /// Prepare a mesh in the body's local frame, without cloning it.
    pub fn prepare(self, mesh: &TriangleMesh) -> Result<PreparedMeshGeometry, GeometryError> {
        self.prepare_instances(&[GeometryInstance {
            mesh,
            local_transform: Transform::identity(),
            owner: GeometryOwner::Body,
        }])
    }

    /// Prepares the union of assembly instances in their stable assembly frame.
    /// Owner IDs and local transforms are retained by the geometry preparation.
    /// The caller must change geometry identity for mesh, membership, owner or
    /// local-transform edits; world motion and COM changes are not shape edits.
    pub fn prepare_instances(
        self,
        instances: &[GeometryInstance<'_>],
    ) -> Result<PreparedMeshGeometry, GeometryError> {
        let geometry = PreparedGeometry::prepare(instances, self.config)?;
        Ok(PreparedMeshGeometry {
            request: self,
            geometry,
        })
    }
}

/// Completed immutable geometry and the unforgeable ticket it was built for.
#[derive(Debug)]
pub struct PreparedMeshGeometry {
    request: GeometryPreparation,
    geometry: PreparedGeometry,
}

/// Single-body, single-entry cache. The previous good entry survives failures,
/// but is never returned as a hit for a different key or configuration.
#[derive(Debug, Default)]
pub struct PreparedGeometryCache {
    requested: Option<GeometryPreparation>,
    cached: Option<PreparedMeshGeometry>,
    wave: Option<CachedWave>,
}

#[derive(Debug)]
struct CachedWave {
    incarnation: Arc<()>,
    axis: usize,
    smoothing_radius: usize,
    factor: super::wave::PreparedWaveDrag,
}

impl PreparedGeometryCache {
    /// Select the current identity before dispatching work. Actual config values
    /// are checked too, so forgetting to bump config_version cannot reuse a hit.
    /// Selecting a different identity invalidates all outstanding old tickets.
    pub fn request(
        &mut self,
        key: GeometryCacheKey,
        config: GeometryConfig,
    ) -> GeometryPreparation {
        if let Some(request) = &self.requested
            && request.key == key
            && request.config == config
        {
            return request.clone();
        }
        let request = GeometryPreparation {
            key,
            config,
            incarnation: Arc::new(()),
        };
        self.requested = Some(request.clone());
        request
    }

    /// Return a current hit only. Retained older geometry is not a current hit.
    pub fn get(&self) -> Option<&PreparedGeometry> {
        let request = self.requested.as_ref()?;
        let cached = self.cached.as_ref()?;
        Arc::ptr_eq(&request.incarnation, &cached.request.incarnation).then_some(&cached.geometry)
    }

    /// Configures a hull section from the current prepared voxel union.
    ///
    /// The wave integral is reused only for the same current geometry request,
    /// axis and smoothing radius. A geometry or configuration version change
    /// requires successfully preparing the new geometry first. A failed wave
    /// preparation does not replace the previous cached result. The returned
    /// section is a snapshot and must be refreshed after geometry changes.
    pub fn configure_voxel_wave_drag(
        &mut self,
        section: super::hull::HullSection,
        axis: usize,
        smoothing_radius: usize,
    ) -> Result<super::hull::HullSection, super::hull::HullError> {
        use super::hull::HullError;
        section.validate_voxel_axis(axis)?;
        let request = self
            .requested
            .as_ref()
            .ok_or(HullError::GeometryNotPrepared)?;
        let geometry = self.get().ok_or(HullError::GeometryNotPrepared)?;
        if let Some(cached) = &self.wave
            && Arc::ptr_eq(&cached.incarnation, &request.incarnation)
            && cached.axis == axis
            && cached.smoothing_radius == smoothing_radius
        {
            return Ok(section.with_prepared_wave_drag(cached.factor));
        }
        let factor = super::wave::prepare_voxel_wave_drag(geometry, axis, smoothing_radius)
            .map_err(HullError::Wave)?;
        let incarnation = Arc::clone(&request.incarnation);
        self.wave = Some(CachedWave {
            incarnation,
            axis,
            smoothing_radius,
            factor,
        });
        Ok(section.with_prepared_wave_drag(factor))
    }

    /// Inspect the last successful entry, even if no longer current.
    pub fn last_prepared(&self) -> Option<(GeometryCacheKey, &PreparedGeometry)> {
        self.cached
            .as_ref()
            .map(|cached| (cached.request.key, &cached.geometry))
    }

    /// Atomically check the ticket and replace the entry through exclusive access.
    /// Returns false for stale results (including results from another cache).
    pub fn publish(&mut self, prepared: PreparedMeshGeometry) -> bool {
        if !self
            .requested
            .as_ref()
            .is_some_and(|request| Arc::ptr_eq(&request.incarnation, &prepared.request.incarnation))
        {
            return false;
        }
        self.cached = Some(prepared);
        self.wave = None;
        true
    }

    /// Synchronous convenience path; successful hits do no geometry work.
    pub fn prepare(
        &mut self,
        key: GeometryCacheKey,
        mesh: &TriangleMesh,
        config: GeometryConfig,
    ) -> Result<&PreparedGeometry, GeometryError> {
        self.prepare_instances(
            key,
            &[GeometryInstance {
                mesh,
                local_transform: Transform::identity(),
                owner: GeometryOwner::Body,
            }],
            config,
        )
    }

    /// Prepares or reuses one assembly union. Identity covers every instance's
    /// geometry, membership, owner and assembly-local transform. Preparation is
    /// transactional: failure never publishes a partially prepared union.
    pub fn prepare_instances(
        &mut self,
        key: GeometryCacheKey,
        instances: &[GeometryInstance<'_>],
        config: GeometryConfig,
    ) -> Result<&PreparedGeometry, GeometryError> {
        let request = self.request(key, config);
        if self.get().is_none() {
            let prepared = request.prepare_instances(instances)?;
            self.publish(prepared);
        }
        self.get().ok_or(GeometryError::InvalidGeometry)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        aerodynamics::{Aerodynamics, geometry::OpenMeshPolicy},
        mass::MassProperties,
        rigid_body::RigidBody,
        world::PhysicsWorld,
    };
    use kapitsa_math::{Mat3, Quat, Vec3};

    fn config() -> GeometryConfig {
        GeometryConfig {
            cell_size: 0.5,
            max_cells: 4096,
            max_work: 100_000,
            max_patch_work: 100_000,
            open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
            weld_tolerance: None,
        }
    }

    fn mesh() -> TriangleMesh {
        TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]).unwrap()
    }

    fn key() -> GeometryCacheKey {
        let mut world = PhysicsWorld::new();
        GeometryCacheKey {
            body: world
                .add_body(RigidBody::new_static(Transform::identity()))
                .unwrap(),
            geometry_version: 1,
            config_version: 1,
        }
    }

    #[test]
    fn assembly_union_preserves_owners_and_rejects_stale_topology()
    -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]])?;
        let instances = [
            GeometryInstance {
                mesh: &mesh,
                local_transform: Transform::identity(),
                owner: GeometryOwner::Body,
            },
            GeometryInstance {
                mesh: &mesh,
                local_transform: Transform::new(Vec3::X * 2.0, Quat::IDENTITY)?,
                owner: GeometryOwner::Part(part),
            },
        ];
        let first = GeometryCacheKey {
            body,
            geometry_version: 1,
            config_version: 1,
        };
        let second = GeometryCacheKey {
            geometry_version: 2,
            ..first
        };
        let mut cache = PreparedGeometryCache::default();
        let expected = PreparedGeometry::prepare(&instances, config())?;
        assert_eq!(
            cache.prepare_instances(first, &instances, config())?,
            &expected
        );
        assert!(expected.occupancy().contains(&Some(GeometryOwner::Body)));
        assert!(
            expected
                .occupancy()
                .contains(&Some(GeometryOwner::Part(part)))
        );
        let incarnation = cache.request(first, config()).incarnation.clone();
        cache.prepare_instances(first, &instances, config())?;
        assert!(
            cache
                .requested
                .as_ref()
                .is_some_and(|r| Arc::ptr_eq(&incarnation, &r.incarnation))
        );
        let stale = cache
            .request(first, config())
            .prepare_instances(&instances)?;
        cache.request(second, config());
        assert!(!cache.publish(stale));
        assert!(cache.prepare_instances(second, &[], config()).is_err());
        assert!(cache.get().is_none());
        assert_eq!(
            cache.last_prepared().ok_or("missing retained union")?.1,
            &expected
        );
        let remaining = PreparedGeometry::prepare(&instances[..1], config())?;
        assert_eq!(
            cache.prepare_instances(second, &instances[..1], config())?,
            &remaining
        );
        Ok(())
    }

    #[test]
    fn wave_configuration_follows_current_geometry_and_filter()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::hull::{HullError, HullSection};
        let mesh = mesh();
        let first = key();
        let second = GeometryCacheKey {
            geometry_version: 2,
            ..first
        };
        let mut cache = PreparedGeometryCache::default();
        let section = HullSection::new(
            Transform::identity(),
            Vec3::X,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            None,
        )?;
        assert!(matches!(
            cache.configure_voxel_wave_drag(section.clone(), 0, 1),
            Err(HullError::GeometryNotPrepared)
        ));
        cache.prepare(first, &mesh, config())?;
        let configured = cache.configure_voxel_wave_drag(section.clone(), 0, 1)?;
        let cached = cache.wave.as_ref().ok_or("wave cache")?;
        let identity = Arc::clone(&cached.incarnation);
        let factor = cached.factor;
        assert_eq!(
            cache.configure_voxel_wave_drag(section.clone(), 0, 1)?,
            configured
        );
        let hit = cache.wave.as_ref().ok_or("wave cache")?;
        assert!(Arc::ptr_eq(&hit.incarnation, &identity));
        assert_eq!(hit.factor, factor);
        assert!(matches!(
            cache.configure_voxel_wave_drag(section.clone(), 1, 1),
            Err(HullError::InvalidParameters)
        ));
        assert_eq!(cache.wave.as_ref().ok_or("wave cache")?.smoothing_radius, 1);
        assert!(
            cache
                .configure_voxel_wave_drag(section.clone(), 0, usize::MAX)
                .is_err()
        );
        assert_eq!(cache.wave.as_ref().ok_or("wave cache")?.smoothing_radius, 1);
        cache.configure_voxel_wave_drag(section.clone(), 0, 2)?;
        assert_eq!(cache.wave.as_ref().ok_or("wave cache")?.smoothing_radius, 2);
        cache.request(second, config());
        assert!(matches!(
            cache.configure_voxel_wave_drag(section.clone(), 0, 1),
            Err(HullError::GeometryNotPrepared)
        ));
        let changed_mesh =
            TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y * 2.0], vec![[0, 1, 2]])?;
        cache.prepare(second, &changed_mesh, config())?;
        cache.configure_voxel_wave_drag(section, 0, 1)?;
        let fresh = cache.wave.as_ref().ok_or("wave cache")?;
        assert!(!Arc::ptr_eq(&fresh.incarnation, &identity));
        assert_ne!(fresh.factor, factor);
        Ok(())
    }

    #[test]
    fn publishing_geometry_clears_derived_wave_even_for_same_ticket()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::hull::HullSection;
        let mut cache = PreparedGeometryCache::default();
        let identity = key();
        let original = mesh();
        let changed = TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y * 2.0], vec![[0, 1, 2]])?;
        let section = HullSection::new(
            Transform::identity(),
            Vec3::X,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            None,
        )?;
        cache.prepare(identity, &original, config())?;
        cache.configure_voxel_wave_drag(section.clone(), 0, 1)?;
        let old_factor = cache.wave.as_ref().ok_or("wave cache")?.factor;
        let replacement = cache.request(identity, config()).prepare(&changed)?;
        assert!(cache.publish(replacement));
        assert!(cache.wave.is_none());
        cache.configure_voxel_wave_drag(section, 0, 1)?;
        assert_ne!(cache.wave.as_ref().ok_or("wave cache")?.factor, old_factor);
        Ok(())
    }

    #[test]
    fn prepare_success_and_reuses_cached_geometry() {
        let (key, mesh) = (key(), mesh());
        let mut cache = PreparedGeometryCache::default();
        let first = cache.prepare(key, &mesh, config()).unwrap() as *const _;
        let second = cache.prepare(key, &mesh, config()).unwrap() as *const _;
        assert_eq!(first, second);
        assert!(cache.get().is_some());
    }

    #[test]
    fn geometry_config_versions_and_actual_config_invalidate() {
        let mesh = mesh();
        let mut key = key();
        let mut cache = PreparedGeometryCache::default();
        cache.prepare(key, &mesh, config()).unwrap();
        key.geometry_version += 1;
        cache.request(key, config());
        assert!(cache.get().is_none());
        cache.prepare(key, &mesh, config()).unwrap();
        key.config_version += 1;
        cache.request(key, config());
        assert!(cache.get().is_none());
        cache.prepare(key, &mesh, config()).unwrap();
        let mut changed = config();
        changed.cell_size = 0.25;
        cache.request(key, changed);
        assert!(cache.get().is_none());
    }

    #[test]
    fn stale_publication_rejects_versions_aba_and_other_cache() {
        let mesh = mesh();
        let key = key();
        let mut cache = PreparedGeometryCache::default();
        cache.prepare(key, &mesh, config()).unwrap();
        let old = cache.request(key, config()).prepare(&mesh).unwrap();
        let newer_key = GeometryCacheKey {
            geometry_version: 2,
            ..key
        };
        let current = cache.request(newer_key, config()).prepare(&mesh).unwrap();
        assert!(!cache.publish(old));
        assert!(cache.publish(current));
        assert_eq!(cache.last_prepared().unwrap().0, newer_key);
        let old = cache.request(newer_key, config()).prepare(&mesh).unwrap();
        cache.request(key, config());
        cache.request(newer_key, config());
        assert!(!cache.publish(old));
        let mut other = PreparedGeometryCache::default();
        let foreign = other.request(newer_key, config()).prepare(&mesh).unwrap();
        assert!(!cache.publish(foreign));
    }

    #[test]
    fn body_identity_rejects_results_from_other_world() {
        let mesh = mesh();
        let mut cache = PreparedGeometryCache::default();
        let first = key();
        let second = key();
        assert_ne!(first.body, second.body);
        let old = cache.request(first, config()).prepare(&mesh).unwrap();
        cache.request(second, config());
        assert!(!cache.publish(old));
        assert!(cache.get().is_none());
    }

    #[test]
    fn failed_preparation_preserves_last_successful_entry() {
        let (key, mesh) = (key(), mesh());
        let mut cache = PreparedGeometryCache::default();
        let previous = cache.prepare(key, &mesh, config()).unwrap() as *const _;
        let mut invalid = config();
        invalid.cell_size = 0.0;
        assert_eq!(
            cache.prepare(key, &mesh, invalid),
            Err(GeometryError::InvalidConfig)
        );
        assert!(cache.get().is_none());
        let (retained_key, retained) = cache.last_prepared().unwrap();
        assert_eq!(retained_key, key);
        assert_eq!(retained as *const _, previous);
    }

    #[test]
    fn motion_and_com_changes_do_not_invalidate_body_local_mesh() {
        let mut body = RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
        )
        .unwrap();
        body.set_aerodynamics(Aerodynamics::Mesh(mesh()));
        let key = key();
        let mut cache = PreparedGeometryCache::default();
        let Aerodynamics::Mesh(mesh) = body.aerodynamics() else {
            panic!("mesh")
        };
        let previous = cache.prepare(key, mesh, config()).unwrap() as *const _;
        body.set_transform(Transform::new(Vec3::Y, Quat::IDENTITY).unwrap())
            .unwrap();
        body.set_linear_velocity(Vec3::X).unwrap();
        body.set_angular_velocity(Vec3::Z).unwrap();
        body.set_mass(MassProperties::new(2.0, Vec3::X, Mat3::IDENTITY).unwrap())
            .unwrap();
        let Aerodynamics::Mesh(mesh) = body.aerodynamics() else {
            panic!("mesh")
        };
        assert_eq!(
            cache.prepare(key, mesh, config()).unwrap() as *const _,
            previous
        );
    }
}
