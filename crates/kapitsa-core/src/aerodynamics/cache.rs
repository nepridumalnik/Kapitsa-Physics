//! Standalone cache for body-local `Aerodynamics::Mesh` preparation.
//!
//! Integration boundary: the world must own one cache per body, supply its full
//! `BodyId`, and advance geometry/config versions when replacing a mesh or its
//! preparation settings. Versions must not be reused for different geometry.
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
        let geometry = PreparedGeometry::prepare(
            &[GeometryInstance {
                mesh,
                local_transform: Transform::identity(),
                owner: GeometryOwner::Body,
            }],
            self.config,
        )?;
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
        true
    }

    /// Synchronous convenience path; successful hits do no geometry work.
    pub fn prepare(
        &mut self,
        key: GeometryCacheKey,
        mesh: &TriangleMesh,
        config: GeometryConfig,
    ) -> Result<&PreparedGeometry, GeometryError> {
        let request = self.request(key, config);
        if self.get().is_none() {
            let prepared = request.prepare(mesh)?;
            // Exclusive access and no intervening request make this current.
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
