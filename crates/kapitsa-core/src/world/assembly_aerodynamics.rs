use std::{collections::HashMap, sync::Arc};

use kapitsa_math::Vec3;

use crate::Error;

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
    Wave(Box<crate::aerodynamics::wave::VoxelWave>),
    WaveSourceMesh(Arc<TriangleMesh>),
    WaveSourceColliders,
}

pub(super) struct AssemblyAerodynamics {
    sources: Vec<(PartId, Arc<TriangleMesh>, Transform)>,
    cache: PreparedGeometryCache,
    collider_parts: Vec<PartId>,
    wave_only_parts: Vec<PartId>,
    wave_cache: PreparedGeometryCache,
    wing_cache: Vec<CachedWingVisibility>,
}

pub(super) struct PreparedAssembly {
    pub patches: Option<PreparedSurfacePatches>,
    pub wave: Option<(PartId, crate::aerodynamics::hull::HullSection)>,
    pub wing_visibility: Vec<PreparedWingVisibility>,
}

#[derive(Clone, Copy, PartialEq)]
struct WingVisibilityKey {
    part: PartId,
    index: usize,
    part_transform: Transform,
    wing_transform: Transform,
    footprint: crate::aerodynamics::wing::WingFootprint,
}

struct CachedWingVisibility {
    key: WingVisibilityKey,
    visibility: crate::aerodynamics::wing::WingVisibility,
}

pub(super) struct PreparedWingVisibility {
    pub part: PartId,
    pub index: usize,
    pub visibility: crate::aerodynamics::wing::WingVisibility,
}

impl PhysicsWorld {
    /// Replaces explicit finite wings in the stable part-local frame.
    /// Wing transforms map wing coordinates to part coordinates, independently
    /// of mass/COM edits. Empty clears wings. Loads are additive to hull and
    /// other explicit surfaces: callers must avoid counting a contribution twice.
    /// No interference or automatic hull/wing exclusion is inferred.
    /// Ownership follows the stable PartId through topology edits; no solver
    /// body is created. PhysicsClient exposes the corresponding queued setter.
    pub fn set_part_aerodynamic_wings(
        &mut self,
        part: PartId,
        wings: Vec<crate::aerodynamics::wing::WingSurface>,
    ) -> Result<(), Error> {
        self.set_part_aerodynamic_wings_inner(part, wings)
            .map_err(Error::from)
    }
    fn set_part_aerodynamic_wings_inner(
        &mut self,
        part: PartId,
        wings: Vec<crate::aerodynamics::wing::WingSurface>,
    ) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        self.parts
            .get_mut(part.dense)
            .ok_or(ClusterError::InvalidPartId)?
            .aerodynamic_wings = wings;
        self.invalidate_body_edit(body);
        Ok(())
    }

    /// Explicit wings in part-local physical coordinates, or None for a stale ID.
    pub fn part_aerodynamic_wings(
        &self,
        part: PartId,
    ) -> Option<&[crate::aerodynamics::wing::WingSurface]> {
        Some(&self.cluster_part(part)?.aerodynamic_wings)
    }

    /// Replaces explicit hull sections in the stable part-local frame.
    /// Sections are additive to wings and the body's primary aerodynamics.
    /// No shape inference or coefficient changes are performed.
    pub fn set_part_aerodynamic_hull(
        &mut self,
        part: PartId,
        sections: Vec<crate::aerodynamics::hull::HullSection>,
    ) -> Result<(), Error> {
        self.set_part_aerodynamic_hull_inner(part, sections)
            .map_err(Error::from)
    }
    fn set_part_aerodynamic_hull_inner(
        &mut self,
        part: PartId,
        sections: Vec<crate::aerodynamics::hull::HullSection>,
    ) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        self.parts
            .get_mut(part.dense)
            .ok_or(ClusterError::InvalidPartId)?
            .aerodynamic_hull = sections;
        self.invalidate_body_edit(body);
        Ok(())
    }

    /// Explicit hull sections in part-local physical coordinates.
    pub fn part_aerodynamic_hull(
        &self,
        part: PartId,
    ) -> Option<&[crate::aerodynamics::hull::HullSection]> {
        Some(&self.cluster_part(part)?.aerodynamic_hull)
    }

    /// Replaces the part's geometry with a Mesh in its local frame.
    /// `None` disables geometric loads. Explicit surfaces remain additive;
    /// configure each physical contribution once to avoid duplicated loads.
    /// Part ownership survives merge/split; no solver body is created.
    pub fn set_part_aerodynamic_mesh(
        &mut self,
        part: PartId,
        mesh: Option<TriangleMesh>,
    ) -> Result<(), Error> {
        self.set_part_aerodynamic_mesh_inner(part, mesh)
            .map_err(Error::from)
    }
    fn set_part_aerodynamic_mesh_inner(
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
    ) -> Result<(), Error> {
        self.set_part_aerodynamic_colliders_inner(part, enabled)
            .map_err(Error::from)
    }
    fn set_part_aerodynamic_colliders_inner(
        &mut self,
        part: PartId,
        enabled: bool,
    ) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        let pressure_colliders = matches!(
            self.part_aerodynamic_geometry
                .as_ref()
                .and_then(|g| g.get(&part)),
            Some(PartGeometry::Colliders)
        );
        if pressure_colliders == enabled {
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

    fn replace_part_wave_geometry(
        &mut self,
        part: PartId,
        value: Option<PartGeometry>,
    ) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        if let Some(value) = value {
            let geometry = self
                .part_aerodynamic_geometry
                .get_or_insert_with(HashMap::new);
            geometry
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
            geometry.insert(part, value);
        } else if let Some(geometry) = &mut self.part_aerodynamic_geometry {
            geometry.remove(&part);
        }
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        self.invalidate_body_edit(body);
        Ok(())
    }

    /// Sets the assembly's voxel wave model at a stable part-local application point.
    /// Only one part per rigid assembly may own this model. Geometry-only source
    /// parts participate in its voxel union. Pressure surfaces remain additive;
    /// select each contribution once to avoid double counting. `None` removes
    /// only an existing wave model from this part.
    pub fn set_part_voxel_wave(
        &mut self,
        part: PartId,
        model: Option<crate::aerodynamics::wave::VoxelWave>,
    ) -> Result<(), Error> {
        self.set_part_voxel_wave_inner(part, model)
            .map_err(Error::from)
    }
    fn set_part_voxel_wave_inner(
        &mut self,
        part: PartId,
        model: Option<crate::aerodynamics::wave::VoxelWave>,
    ) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        if model.is_some() {
            let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
            if cluster.parts().iter().any(|other| {
                *other != part
                    && matches!(
                        self.part_aerodynamic_geometry
                            .as_ref()
                            .and_then(|g| g.get(other)),
                        Some(PartGeometry::Wave(_))
                    )
            }) {
                return Err(ClusterError::UnsupportedGeometry);
            }
        } else if !matches!(
            self.part_aerodynamic_geometry
                .as_ref()
                .and_then(|g| g.get(&part)),
            Some(PartGeometry::Wave(_))
        ) {
            return Ok(());
        }
        self.replace_part_wave_geometry(
            part,
            model.map(|model| PartGeometry::Wave(Box::new(model))),
        )
    }

    /// Sets a part-local mesh used as geometry in an assembly wave union only.
    /// It does not independently produce Newtonian pressure loads. `None`
    /// removes only a wave-source mesh from this part.
    pub fn set_part_wave_source_mesh(
        &mut self,
        part: PartId,
        mesh: Option<TriangleMesh>,
    ) -> Result<(), Error> {
        self.set_part_wave_source_mesh_inner(part, mesh)
            .map_err(Error::from)
    }
    fn set_part_wave_source_mesh_inner(
        &mut self,
        part: PartId,
        mesh: Option<TriangleMesh>,
    ) -> Result<(), ClusterError> {
        if self.cluster_part(part).is_none() {
            return Err(ClusterError::InvalidPartId);
        }
        if mesh.is_none()
            && !matches!(
                self.part_aerodynamic_geometry
                    .as_ref()
                    .and_then(|g| g.get(&part)),
                Some(PartGeometry::WaveSourceMesh(_))
            )
        {
            return Ok(());
        }
        self.replace_part_wave_geometry(
            part,
            mesh.map(|mesh| PartGeometry::WaveSourceMesh(Arc::new(mesh))),
        )
    }

    /// Selects part-owned colliders as geometry-only input to the wave union.
    /// `false` removes only this mode; it leaves other geometry selections intact.
    pub fn set_part_wave_source_colliders(
        &mut self,
        part: PartId,
        enabled: bool,
    ) -> Result<(), Error> {
        self.set_part_wave_source_colliders_inner(part, enabled)
            .map_err(Error::from)
    }
    fn set_part_wave_source_colliders_inner(
        &mut self,
        part: PartId,
        enabled: bool,
    ) -> Result<(), ClusterError> {
        if self.cluster_part(part).is_none() {
            return Err(ClusterError::InvalidPartId);
        }
        let current = matches!(
            self.part_aerodynamic_geometry
                .as_ref()
                .and_then(|g| g.get(&part)),
            Some(PartGeometry::WaveSourceColliders)
        );
        if current == enabled {
            return Ok(());
        }
        self.replace_part_wave_geometry(part, enabled.then_some(PartGeometry::WaveSourceColliders))
    }

    /// Returns whether actual colliders define this part's geometry.
    pub fn part_uses_aerodynamic_colliders(&self, part: PartId) -> bool {
        self.cluster_part(part).is_some()
            && matches!(
                self.part_aerodynamic_geometry
                    .as_ref()
                    .and_then(|g| g.get(&part)),
                Some(PartGeometry::Colliders | PartGeometry::WaveSourceColliders)
            )
    }

    /// Returns the configured physical mesh in the part's local frame.
    pub fn part_aerodynamic_mesh(&self, part: PartId) -> Option<&TriangleMesh> {
        self.cluster_part(part)?;
        match self.part_aerodynamic_geometry.as_ref()?.get(&part)? {
            PartGeometry::Mesh(mesh) | PartGeometry::WaveSourceMesh(mesh) => Some(mesh),
            PartGeometry::Wave(model) => Some(&model.mesh),
            PartGeometry::Colliders | PartGeometry::WaveSourceColliders => None,
        }
    }
}

pub(super) fn wings_configured(entry: &BodyEntry, parts: &DensePool<ClusterPart>) -> bool {
    entry.cluster.as_ref().is_some_and(|cluster| {
        cluster.parts().iter().any(|id| {
            parts
                .get(id.dense)
                .is_some_and(|part| !part.aerodynamic_wings.is_empty())
        })
    })
}

pub(super) fn hull_configured(entry: &BodyEntry, parts: &DensePool<ClusterPart>) -> bool {
    entry.cluster.as_ref().is_some_and(|cluster| {
        cluster.parts().iter().any(|id| {
            parts
                .get(id.dense)
                .is_some_and(|part| !part.aerodynamic_hull.is_empty())
        })
    })
}

pub(super) fn wave_configured(
    entry: &BodyEntry,
    geometry: Option<&HashMap<PartId, PartGeometry>>,
) -> bool {
    entry.cluster.as_ref().is_some_and(|cluster| {
        cluster.parts().iter().any(|part| {
            matches!(
                geometry.and_then(|items| items.get(part)),
                Some(PartGeometry::Wave(_))
            )
        })
    })
}

pub(super) fn shielding_configured(
    entry: &BodyEntry,
    parts: &DensePool<ClusterPart>,
    geometry: Option<&HashMap<PartId, PartGeometry>>,
) -> bool {
    configured(entry, geometry)
        && entry.cluster.as_ref().is_some_and(|cluster| {
            cluster.parts().iter().any(|id| {
                parts.get(id.dense).is_some_and(|part| {
                    part.aerodynamic_wings
                        .iter()
                        .any(|wing| wing.has_footprint())
                })
            })
        })
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
) -> Result<Option<PreparedAssembly>, Error> {
    let error = || StepError::Aerodynamics {
        body_id,
        cause: crate::Error::from(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow),
    };
    let geometry_error = |cause| {
        Error::from(StepError::Aerodynamics {
            body_id,
            cause: crate::Error::from(crate::aerodynamics::AerodynamicsError::Geometry(
                crate::Error::from(cause),
            )),
        })
    };
    let Some(geometry) = geometry else {
        return Ok(None);
    };
    let Some(cluster) = &entry.cluster else {
        return Ok(None);
    };
    if entry.body.physics().body_type() != crate::rigid_body::BodyType::Dynamic {
        return Ok(None);
    }
    if cluster.parts().len() > 100_000_000 {
        return Err(geometry_error(GeometryError::WorkBudgetExceeded));
    }
    if !configured(entry, Some(geometry)) {
        return Ok(None);
    }
    let mut wave_model = None;
    let mut pressure_enabled = false;
    for &part in cluster.parts() {
        match geometry.get(&part) {
            Some(PartGeometry::Wave(model)) => {
                if wave_model.replace((part, model.as_ref())).is_some() {
                    return Err(Error::from(StepError::Aerodynamics {
                        body_id,
                        cause: crate::Error::from(
                            crate::aerodynamics::AerodynamicsError::UnsupportedModel,
                        ),
                    }));
                }
            }
            Some(PartGeometry::Mesh(_) | PartGeometry::Colliders) => pressure_enabled = true,
            _ => {}
        }
    }
    let pressure_wind = if pressure_enabled {
        let center = entry
            .body
            .world_center_of_mass()
            .map_err(|_| error())?
            .ok_or_else(error)?;
        let state = atmosphere.state_at(center).map_err(|cause| {
            Error::from(StepError::Aerodynamics {
                body_id,
                cause: crate::Error::from(cause),
            })
        })?;
        (state.density() != 0.0).then_some(state.wind_velocity())
    } else {
        None
    };
    let wave_model = if let Some((part, model)) = wave_model {
        let local = parts
            .get(part.dense)
            .filter(|p| p.body() == body_id)
            .ok_or_else(error)?
            .local_transform();
        let transform = entry
            .body
            .state()
            .transform()
            .compose(&local)
            .map_err(|_| error())?;
        let point = transform.transform_point(model.section.local_transform().translation());
        let state = atmosphere.state_at(point).map_err(|cause| {
            Error::from(StepError::Aerodynamics {
                body_id,
                cause: crate::Error::from(cause),
            })
        })?;
        (state.density() != 0.0).then_some((part, model, local))
    } else {
        None
    };
    let mut wing_requests = Vec::new();
    for &part_id in cluster.parts() {
        let part = parts
            .get(part_id.dense)
            .filter(|p| p.body() == body_id)
            .ok_or_else(error)?;
        let local = part.local_transform();
        for (index, wing) in part.aerodynamic_wings.iter().enumerate() {
            if !wing.has_footprint() {
                continue;
            }
            let transform = entry
                .body
                .state()
                .transform()
                .compose(&local)
                .map_err(|_| error())?;
            let point = transform.transform_point(wing.local_transform().translation());
            let state = atmosphere.state_at(point).map_err(|cause| {
                Error::from(StepError::Aerodynamics {
                    body_id,
                    cause: crate::Error::from(cause),
                })
            })?;
            if state.density() == 0.0 {
                continue;
            }
            if wing_requests.len() >= 1_000_000 {
                return Err(Error::from(StepError::Aerodynamics {
                    body_id,
                    cause: crate::Error::from(crate::aerodynamics::AerodynamicsError::Wing(
                        crate::Error::from(
                            crate::aerodynamics::wing::WingError::ShieldingBudgetExceeded,
                        ),
                    )),
                }));
            }
            wing_requests
                .try_reserve(1)
                .map_err(|_| geometry_error(GeometryError::AllocationFailed))?;
            wing_requests.push((part_id, index, wing, local));
        }
    }
    if pressure_wind.is_none() && wave_model.is_none() && wing_requests.is_empty() {
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
                                && cached.wave_only_parts.binary_search(part).is_err()
                                && Arc::ptr_eq(mesh, current)
                        }
                        Some(PartGeometry::WaveSourceMesh(current)) => {
                            cached.wave_only_parts.binary_search(part).is_ok()
                                && Arc::ptr_eq(mesh, current)
                        }
                        Some(PartGeometry::Wave(model)) => {
                            cached.wave_only_parts.binary_search(part).is_ok()
                                && Arc::ptr_eq(mesh, &model.mesh)
                        }
                        Some(PartGeometry::Colliders) => {
                            cached.collider_parts.binary_search(part).is_ok()
                                && cached.wave_only_parts.binary_search(part).is_err()
                        }
                        Some(PartGeometry::WaveSourceColliders) => {
                            cached.collider_parts.binary_search(part).is_ok()
                                && cached.wave_only_parts.binary_search(part).is_ok()
                        }
                        None => false,
                    }
            })
    });
    if unchanged {
        return evaluate(
            body_id,
            entry,
            pressure_wind,
            wave_model,
            &wing_requests,
            caches.get_mut(&body_id).ok_or_else(error)?,
        )
        .map(Some)
        .map_err(|cause| {
            Error::from(StepError::Aerodynamics {
                body_id,
                cause: crate::Error::from(cause),
            })
        });
    }
    let mut sources = Vec::new();
    let mut collider_parts = Vec::new();
    let mut wave_only_parts = Vec::new();
    let mut triangle_count = 0usize;
    let mut collider_triangle_count = 0usize;
    for &part in cluster.parts() {
        if matches!(
            geometry.get(&part),
            Some(
                PartGeometry::Wave(_)
                    | PartGeometry::WaveSourceMesh(_)
                    | PartGeometry::WaveSourceColliders
            )
        ) {
            wave_only_parts
                .try_reserve(1)
                .map_err(|_| geometry_error(GeometryError::AllocationFailed))?;
            wave_only_parts.push(part);
        }
        let mesh = match geometry.get(&part) {
            Some(PartGeometry::Mesh(mesh) | PartGeometry::WaveSourceMesh(mesh)) => mesh.clone(),
            Some(PartGeometry::Wave(model)) => Arc::clone(&model.mesh),
            Some(PartGeometry::Colliders | PartGeometry::WaveSourceColliders) => {
                let owner = parts
                    .get(part.dense)
                    .filter(|p| p.body() == body_id)
                    .ok_or_else(error)?;
                if owner.colliders().len() > entry.collider_tessellation.max_triangles() {
                    return Err(Error::from(StepError::Aerodynamics {
                        body_id,
                        cause: crate::Error::from(
                            crate::aerodynamics::AerodynamicsError::TessellationBudgetExceeded,
                        ),
                    }));
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
                .map_err(|cause| {
                    Error::from(StepError::Aerodynamics {
                        body_id,
                        cause: crate::Error::from(cause),
                    })
                })?;
                collider_triangle_count = collider_triangle_count
                    .checked_add(mesh.triangles().len())
                    .ok_or_else(error)?;
                if collider_triangle_count > entry.collider_tessellation.max_triangles() {
                    return Err(Error::from(StepError::Aerodynamics {
                        body_id,
                        cause: crate::Error::from(
                            crate::aerodynamics::AerodynamicsError::TessellationBudgetExceeded,
                        ),
                    }));
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
        if pressure_wind.is_some()
            && triangle_count
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
    wave_only_parts.sort_unstable();
    caches
        .try_reserve(1)
        .map_err(|_| geometry_error(GeometryError::AllocationFailed))?;
    let mut candidate = AssemblyAerodynamics {
        sources,
        collider_parts,
        wave_only_parts,
        cache: PreparedGeometryCache::default(),
        wave_cache: PreparedGeometryCache::default(),
        wing_cache: Vec::new(),
    };
    let prepared = evaluate(
        body_id,
        entry,
        pressure_wind,
        wave_model,
        &wing_requests,
        &mut candidate,
    )
    .map_err(|cause| {
        Error::from(StepError::Aerodynamics {
            body_id,
            cause: crate::Error::from(cause),
        })
    })?;
    caches.insert(body_id, candidate);
    Ok(Some(prepared))
}

fn evaluate(
    body_id: BodyId,
    entry: &BodyEntry,
    pressure_wind: Option<Vec3>,
    wave_model: Option<(PartId, &crate::aerodynamics::wave::VoxelWave, Transform)>,
    wing_requests: &[(
        PartId,
        usize,
        &crate::aerodynamics::wing::WingSurface,
        Transform,
    )],
    prepared: &mut AssemblyAerodynamics,
) -> Result<PreparedAssembly, AerodynamicsError> {
    let mut instances = Vec::new();
    instances.try_reserve(prepared.sources.len()).map_err(|_| {
        crate::aerodynamics::AerodynamicsError::Geometry(crate::Error::from(
            GeometryError::AllocationFailed,
        ))
    })?;
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
    let wave = if let Some((part, model, local)) = wave_model {
        let inverse = local.inverse().map_err(|_| {
            crate::aerodynamics::AerodynamicsError::Geometry(crate::Error::from(
                GeometryError::ArithmeticOverflow,
            ))
        })?;
        let mut wave_instances = Vec::new();
        wave_instances.try_reserve(instances.len()).map_err(|_| {
            crate::aerodynamics::AerodynamicsError::Geometry(crate::Error::from(
                GeometryError::AllocationFailed,
            ))
        })?;
        for instance in &instances {
            let local_transform = inverse.compose(&instance.local_transform).map_err(|_| {
                crate::aerodynamics::AerodynamicsError::Geometry(crate::Error::from(
                    GeometryError::ArithmeticOverflow,
                ))
            })?;
            wave_instances.push(GeometryInstance {
                mesh: instance.mesh,
                local_transform,
                owner: instance.owner,
            });
        }
        prepared
            .wave_cache
            .prepare_instances(
                GeometryCacheKey {
                    body: body_id,
                    geometry_version: 0,
                    config_version: 1,
                },
                &wave_instances,
                model.geometry_config,
            )
            .map_err(|cause| crate::aerodynamics::AerodynamicsError::Geometry(cause))?;
        let section = prepared
            .wave_cache
            .configure_voxel_wave_drag(model.section.clone(), model.axis, model.smoothing_radius)
            .map_err(|cause| crate::aerodynamics::AerodynamicsError::Hull(cause))?;
        Some((part, section))
    } else {
        None
    };
    let mut wing_visibility = Vec::new();
    if !wing_requests.is_empty() {
        let cache_matches = prepared.wing_cache.len() == wing_requests.len()
            && prepared.wing_cache.iter().zip(wing_requests).all(
                |(cached, &(part, index, wing, local))| {
                    wing.footprint_signature()
                        .is_some_and(|(wing_transform, footprint)| {
                            cached.key
                                == WingVisibilityKey {
                                    part,
                                    index,
                                    part_transform: local,
                                    wing_transform,
                                    footprint,
                                }
                        })
                },
            );
        if !cache_matches {
            let config = pressure_geometry_config(&prepared.collider_parts);
            let geometry = prepared
                .cache
                .prepare_instances(
                    GeometryCacheKey {
                        body: body_id,
                        geometry_version: 0,
                        config_version: 1,
                    },
                    &instances,
                    config,
                )
                .map_err(|cause| crate::aerodynamics::AerodynamicsError::Geometry(cause))?;
            let mut candidate = Vec::new();
            candidate
                .try_reserve_exact(wing_requests.len())
                .map_err(|_| {
                    crate::aerodynamics::AerodynamicsError::Geometry(crate::Error::from(
                        GeometryError::AllocationFailed,
                    ))
                })?;
            let mut remaining = 1_000_000_usize;
            for &(part, index, wing, local) in wing_requests {
                let (wing_transform, footprint) = wing.footprint_signature().ok_or(
                    crate::aerodynamics::AerodynamicsError::Wing(crate::Error::from(
                        crate::aerodynamics::wing::WingError::ShieldingNotPrepared,
                    )),
                )?;
                let visibility = wing
                    .volume_visibility(geometry, local, remaining)
                    .map_err(crate::aerodynamics::AerodynamicsError::Wing)?
                    .ok_or(crate::aerodynamics::AerodynamicsError::Wing(
                        crate::Error::from(
                            crate::aerodynamics::wing::WingError::ShieldingNotPrepared,
                        ),
                    ))?;
                remaining = remaining.checked_sub(visibility.samples).ok_or(
                    crate::aerodynamics::AerodynamicsError::Wing(crate::Error::from(
                        crate::aerodynamics::wing::WingError::ShieldingBudgetExceeded,
                    )),
                )?;
                candidate.push(CachedWingVisibility {
                    key: WingVisibilityKey {
                        part,
                        index,
                        part_transform: local,
                        wing_transform,
                        footprint,
                    },
                    visibility,
                });
            }
            prepared.wing_cache = candidate;
        }
        wing_visibility
            .try_reserve_exact(prepared.wing_cache.len())
            .map_err(|_| {
                crate::aerodynamics::AerodynamicsError::Geometry(crate::Error::from(
                    GeometryError::AllocationFailed,
                ))
            })?;
        for cached in &prepared.wing_cache {
            wing_visibility.push(PreparedWingVisibility {
                part: cached.key.part,
                index: cached.key.index,
                visibility: cached.visibility,
            });
        }
        wing_visibility.sort_unstable_by_key(|entry| (entry.part, entry.index));
    }
    let patches = match pressure_wind {
        Some(wind) => Some(
            evaluate_pressure(
                body_id,
                entry,
                wind,
                &instances,
                &prepared.collider_parts,
                &prepared.wave_only_parts,
                &mut prepared.cache,
            )
            .map_err(|cause| crate::aerodynamics::AerodynamicsError::Geometry(cause))?,
        ),
        None => None,
    };
    Ok(PreparedAssembly {
        patches,
        wave,
        wing_visibility,
    })
}

fn pressure_geometry_config(collider_parts: &[PartId]) -> GeometryConfig {
    GeometryConfig {
        cell_size: 0.125,
        max_cells: 1_048_576,
        max_work: 100_000_000,
        open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
        weld_tolerance: if collider_parts.is_empty() {
            None
        } else {
            Some(1e-10)
        },
        max_patch_work: 100_000_000,
    }
}

fn evaluate_pressure(
    body_id: BodyId,
    entry: &BodyEntry,
    wind: Vec3,
    instances: &[GeometryInstance<'_>],
    collider_parts: &[PartId],
    wave_only_parts: &[PartId],
    cache: &mut PreparedGeometryCache,
) -> Result<PreparedSurfacePatches, crate::Error> {
    let config = pressure_geometry_config(collider_parts);
    let geometry = cache.prepare_instances(
        GeometryCacheKey {
            body: body_id,
            geometry_version: 0,
            config_version: 1,
        },
        instances,
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
        .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?;
    let velocity = inverse.transform_vector(relative);
    let omega = inverse.transform_vector(entry.body.state().angular_velocity());
    let center = entry
        .body
        .physics()
        .mass()
        .ok_or(GeometryError::InvalidGeometry)?
        .center_of_mass();
    let mut patches =
        geometry.surface_patches_with_flow(instances, config.max_patch_work, |point| {
            point
                .checked_sub(center)
                .and_then(|arm| omega.checked_cross(arm))
                .and_then(|rotation| velocity.checked_add(rotation))
                .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))
        })?;
    patches.retain(|patch| match patch.owner {
        GeometryOwner::Part(part) => wave_only_parts.binary_search(&part).is_err(),
        GeometryOwner::Body => true,
    });
    Ok(patches)
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

    fn aero_cause(
        result: &Result<(), crate::Error>,
    ) -> Option<&crate::aerodynamics::AerodynamicsError> {
        match step_cause(result)? {
            crate::world::StepError::Aerodynamics { cause, .. } => cause.downcast_ref(),
            _ => None,
        }
    }
    fn aero_geometry(
        result: &Result<(), crate::Error>,
    ) -> Option<&crate::aerodynamics::geometry::GeometryError> {
        match aero_cause(result)? {
            crate::aerodynamics::AerodynamicsError::Geometry(cause) => cause.downcast_ref(),
            _ => None,
        }
    }

    fn aero_pressure(
        result: &Result<(), crate::Error>,
    ) -> Option<&crate::aerodynamics::pressure::PressureError> {
        match aero_cause(result)? {
            crate::aerodynamics::AerodynamicsError::Pressure(cause) => cause.downcast_ref(),
            _ => None,
        }
    }

    fn aero_wing(
        result: &Result<(), crate::Error>,
    ) -> Option<&crate::aerodynamics::wing::WingError> {
        match aero_cause(result)? {
            crate::aerodynamics::AerodynamicsError::Wing(cause) => cause.downcast_ref(),
            _ => None,
        }
    }

    fn step_cause(result: &Result<(), crate::Error>) -> Option<&crate::world::StepError> {
        result.as_ref().err().and_then(|error| error.downcast_ref())
    }

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

    fn plate() -> Result<TriangleMesh, crate::Error> {
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

    fn wave_model() -> Result<crate::aerodynamics::wave::VoxelWave, Box<dyn std::error::Error>> {
        let section = crate::aerodynamics::hull::HullSection::new(
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
        Ok(crate::aerodynamics::wave::VoxelWave::new(
            plate()?,
            section,
            0,
            1,
            GeometryConfig {
                cell_size: 0.25,
                max_cells: 100_000,
                max_work: 10_000_000,
                max_patch_work: 1_000_000,
                open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
                weld_tolerance: None,
            },
        )?)
    }

    #[test]
    fn part_wave_load_breaks_certified_weld() -> Result<(), Box<dyn std::error::Error>> {
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                1.2,
                Vec3::ZERO,
            )?));
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
            let mut rigid = RigidBody::new_dynamic(Transform::identity(), mass)?;
            rigid.set_linear_velocity(Vec3::X * 680.0)?;
            let body = world.add_body(rigid)?;
            let root = world.enable_cluster(body)?;
            let child = world.add_cluster_part(
                root,
                crate::cluster::ClusterPartDesc {
                    local_transform: Transform::identity(),
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::X * 680.0,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    strength: WeldStrength::Breakable {
                        max_force: Some(1.0),
                        max_torque: None,
                    },
                    ..weld()
                },
            )?;
            world.set_part_voxel_wave(child, Some(wave_model()?))?;
            world.step(1e-7)?;
            let reactions = world
                .part_aerodynamic_weld_reactions(body, 1e-8, 1e-8)
                .map_err(|e| format!("{e:?}"))?;
            assert!(reactions[0].force.x > 1.0);
            world
                .set_automatic_weld_failure(Some((1e-8, 1e-8)))
                .map_err(|e| format!("{e:?}"))?;
            if sharded {
                world.step_with_shards(
                    1e-7,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(1e-7)?;
            }
            assert!(world.cluster(body).ok_or("cluster")?.welds().is_empty());
            assert_ne!(world.cluster_part(child).ok_or("part")?.body(), body);
        }
        Ok(())
    }

    #[test]
    fn voxel_wave_transfers_to_cluster_and_matches_standalone()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::{Aerodynamics, AerodynamicsTask};
        let mut velocities = Vec::new();
        for clustered in [false, true] {
            for sharded in [false, true] {
                let mut world = PhysicsWorld::new();
                world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                    1.2,
                    Vec3::ZERO,
                )?));
                let mut body = RigidBody::new_dynamic(
                    Transform::identity(),
                    MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
                )?;
                body.set_linear_velocity(Vec3::X * 680.0)?;
                body.set_aerodynamics(Aerodynamics::VoxelWave(wave_model()?));
                let id = world.add_body(body)?;
                if clustered {
                    let part = world.enable_cluster(id)?;
                    assert!(matches!(
                        world
                            .part_aerodynamic_geometry
                            .as_ref()
                            .and_then(|g| g.get(&part)),
                        Some(PartGeometry::Wave(_))
                    ));
                }
                AerodynamicsTask.execute(&world)?;
                world.fill_step_shards(&mut [Vec::new()])?;
                if sharded {
                    world.step_with_shards(
                        1e-6,
                        &mut [Vec::new(), Vec::new()],
                        &SequentialExecutor,
                        1,
                    )?;
                } else {
                    world.step(1e-6)?;
                }
                let speed = world.body(id).ok_or("body")?.state().linear_velocity().x;
                assert!(speed < 680.0 && speed > 0.0);
                velocities.push(speed);
            }
        }
        assert!(
            velocities.windows(2).all(|pair| pair[0] == pair[1]),
            "{velocities:?}"
        );
        Ok(())
    }

    #[test]
    fn assembly_wave_rebuilds_for_source_changes_and_rejects_conflicting_merge()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::Aerodynamics;
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
            1.2,
            Vec3::ZERO,
        )?));
        let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
        let mut first = RigidBody::new_dynamic(Transform::identity(), mass)?;
        first.set_linear_velocity(Vec3::X * 680.0)?;
        first.set_aerodynamics(Aerodynamics::VoxelWave(wave_model()?));
        let id = world.add_body(first)?;
        let root = world.enable_cluster(id)?;
        let part = world.add_cluster_part(
            root,
            crate::cluster::ClusterPartDesc {
                local_transform: Transform::new(Vec3::X * 2.0, kapitsa_math::Quat::IDENTITY)?,
                mass,
                colliders: Vec::new(),
                linear_velocity: Vec3::X * 680.0,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame::new(Vec3::X * 2.0, kapitsa_math::Quat::IDENTITY),
                ..weld()
            },
        )?;
        world.set_part_wave_source_mesh(part, Some(plate()?))?;
        world.step(1e-6)?;
        let cache = world
            .assembly_aerodynamics
            .as_mut()
            .and_then(|c| c.get_mut(&id))
            .ok_or("assembly cache")?;
        let model = match world
            .part_aerodynamic_geometry
            .as_ref()
            .and_then(|g| g.get(&root))
        {
            Some(PartGeometry::Wave(model)) => model,
            _ => return Err("wave model missing".into()),
        };
        let first_section = cache.wave_cache.configure_voxel_wave_drag(
            model.section.clone(),
            model.axis,
            model.smoothing_radius,
        )?;
        let old_volume = cache.wave_cache.get().ok_or("prepared wave")?.volume();
        let changed_mesh = TriangleMesh::new(
            vec![
                Vec3::new(0.0, -1.0, -1.0),
                Vec3::new(0.0, 1.0, -1.0),
                Vec3::new(0.0, -1.0, 1.0),
            ],
            vec![[0, 1, 2]],
        )?;
        world.set_part_wave_source_mesh(part, Some(changed_mesh))?;
        world.step(1e-6)?;
        let cache = world
            .assembly_aerodynamics
            .as_mut()
            .and_then(|c| c.get_mut(&id))
            .ok_or("assembly cache")?;
        let model = match world
            .part_aerodynamic_geometry
            .as_ref()
            .and_then(|g| g.get(&root))
        {
            Some(PartGeometry::Wave(model)) => model,
            _ => return Err("wave model missing".into()),
        };
        let second_section = cache.wave_cache.configure_voxel_wave_drag(
            model.section.clone(),
            model.axis,
            model.smoothing_radius,
        )?;
        assert_ne!(first_section, second_section);
        assert_ne!(
            old_volume,
            cache.wave_cache.get().ok_or("prepared wave")?.volume()
        );
        let mut other = RigidBody::new_dynamic(Transform::identity(), mass)?;
        other.set_linear_velocity(world.body(id).ok_or("body")?.state().linear_velocity())?;
        other.set_aerodynamics(Aerodynamics::VoxelWave(wave_model()?));
        let other_id = world.add_body(other)?;
        world.enable_cluster(other_id)?;
        let before = world.body(id).ok_or("body")?.motion();
        assert_eq!(
            world.merge_clusters(id, other_id, weld()),
            Err(crate::Error::from(ClusterError::UnsupportedGeometry))
        );
        assert_eq!(world.body(id).ok_or("body")?.motion(), before);
        assert!(world.body(other_id).is_some());
        Ok(())
    }

    #[test]
    fn assembly_wave_samples_application_point_when_com_is_outside_atmosphere()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::{
            Aerodynamics, DensityProfile, SphericalAtmosphere, hull::HullSection, wave::VoxelWave,
        };
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(Atmosphere::Sphere(
                SphericalAtmosphere::builder()
                    .reference_radius(5.0)
                    .reference_density(1.0)
                    .height(5.0)
                    .profile(DensityProfile::Exponential { scale_height: 1.0 })
                    .build()?,
            ));
            let section = HullSection::new(
                Transform::new(Vec3::X * -20.0, kapitsa_math::Quat::IDENTITY)?,
                Vec3::X,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                None,
            )?;
            let wave = VoxelWave::new(
                plate()?,
                section,
                0,
                1,
                GeometryConfig {
                    cell_size: 0.25,
                    max_cells: 100_000,
                    max_work: 10_000_000,
                    max_patch_work: 1_000_000,
                    open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
                    weld_tolerance: None,
                },
            )?;
            let mut body = RigidBody::new_dynamic(
                Transform::new(Vec3::X * 20.0, kapitsa_math::Quat::IDENTITY)?,
                MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
            )?;
            body.set_linear_velocity(Vec3::X * 680.0)?;
            body.set_aerodynamics(Aerodynamics::VoxelWave(wave));
            let id = world.add_body(body)?;
            world.enable_cluster(id)?;
            if sharded {
                world.step_with_shards(
                    1e-8,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(1e-8)?;
            }
            assert!(world.body(id).ok_or("body")?.state().linear_velocity().x < 680.0);
        }
        Ok(())
    }

    #[test]
    fn collider_wave_source_rebuilds_after_transform_and_rolls_back_when_empty()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::Aerodynamics;
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
            1.2,
            Vec3::ZERO,
        )?));
        let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::X * 680.0)?;
        body.set_aerodynamics(Aerodynamics::VoxelWave(wave_model()?));
        let id = world.add_body(body)?;
        let root = world.enable_cluster(id)?;
        let part = world.add_cluster_part(
            root,
            crate::cluster::ClusterPartDesc {
                local_transform: Transform::new(Vec3::X * 2.0, kapitsa_math::Quat::IDENTITY)?,
                mass,
                colliders: Vec::new(),
                linear_velocity: Vec3::X * 680.0,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame::new(Vec3::X * 2.0, kapitsa_math::Quat::IDENTITY),
                ..weld()
            },
        )?;
        let mut collider = crate::collider::Collider::new(
            crate::shape::Shape::Cuboid(crate::shape::Cuboid::new(1.0, 1.0, 1.0)?),
            Transform::identity(),
        );
        collider.set_mode(crate::collider::CollisionMode::Sensor);
        let collider_id = world.add_part_collider(part, collider)?;
        world.set_part_wave_source_colliders(part, true)?;
        assert!(world.part_uses_aerodynamic_colliders(part));
        world.step(1e-7)?;
        let cache = world
            .assembly_aerodynamics
            .as_mut()
            .and_then(|c| c.get_mut(&id))
            .ok_or("cache")?;
        let model = match world
            .part_aerodynamic_geometry
            .as_ref()
            .and_then(|g| g.get(&root))
        {
            Some(PartGeometry::Wave(model)) => model,
            _ => return Err("model".into()),
        };
        let before_section = cache.wave_cache.configure_voxel_wave_drag(
            model.section.clone(),
            model.axis,
            model.smoothing_radius,
        )?;
        world.set_part_collider_transform(
            part,
            collider_id,
            Transform::new(Vec3::X, kapitsa_math::Quat::IDENTITY)?,
        )?;
        world.step(1e-7)?;
        let cache = world
            .assembly_aerodynamics
            .as_mut()
            .and_then(|c| c.get_mut(&id))
            .ok_or("cache")?;
        let model = match world
            .part_aerodynamic_geometry
            .as_ref()
            .and_then(|g| g.get(&root))
        {
            Some(PartGeometry::Wave(model)) => model,
            _ => return Err("model".into()),
        };
        let after_section = cache.wave_cache.configure_voxel_wave_drag(
            model.section.clone(),
            model.axis,
            model.smoothing_radius,
        )?;
        assert_ne!(before_section, after_section);
        world.remove_part_collider(collider_id)?;
        let before_motion = world.body(id).ok_or("body")?.motion();
        assert!(matches!(
            aero_cause(&world.step(1e-7)),
            Some(crate::aerodynamics::AerodynamicsError::UnsupportedModel)
        ));
        assert_eq!(world.body(id).ok_or("body")?.motion(), before_motion);
        Ok(())
    }

    #[test]
    fn changed_wave_source_error_rolls_back_and_recovers() -> Result<(), Box<dyn std::error::Error>>
    {
        use crate::aerodynamics::Aerodynamics;
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                1.2,
                Vec3::ZERO,
            )?));
            let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::X * 680.0)?;
            body.set_aerodynamics(Aerodynamics::VoxelWave(wave_model()?));
            let id = world.add_body(body)?;
            let root = world.enable_cluster(id)?;
            let source = world.add_cluster_part(
                root,
                crate::cluster::ClusterPartDesc {
                    local_transform: Transform::new(Vec3::X * 2.0, kapitsa_math::Quat::IDENTITY)?,
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::X * 680.0,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    frame_a: JointFrame::new(Vec3::X * 2.0, kapitsa_math::Quat::IDENTITY),
                    ..weld()
                },
            )?;
            world.set_part_wave_source_mesh(source, Some(plate()?))?;
            world.step(1e-7)?;
            let huge = TriangleMesh::new(
                vec![Vec3::ZERO, Vec3::X * 1e8, Vec3::Y * 1e8],
                vec![[0, 1, 2]],
            )?;
            world.set_part_wave_source_mesh(source, Some(huge))?;
            let before = world.body(id).ok_or("body")?.motion();
            let result = if sharded {
                world.step_with_shards(1e-7, &mut [Vec::new()], &SequentialExecutor, 1)
            } else {
                world.step(1e-7)
            };
            assert!(matches!(
                aero_geometry(&result),
                Some(GeometryError::CellBudgetExceeded)
            ));
            assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            world.set_part_wave_source_mesh(source, Some(plate()?))?;
            world.step(1e-7)?;
            assert!(
                world.body(id).ok_or("body")?.state().linear_velocity().x
                    < before.state().linear_velocity().x
            );
        }
        Ok(())
    }

    #[test]
    fn assembly_wave_uses_rotated_owner_frame() -> Result<(), Box<dyn std::error::Error>> {
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                1.2,
                Vec3::ZERO,
            )?));
            let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::Y * 680.0)?;
            let id = world.add_body(body)?;
            let root = world.enable_cluster(id)?;
            let rotation =
                kapitsa_math::Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2)
                    .ok_or("rotation")?;
            let local = Transform::new(Vec3::ZERO, rotation)?;
            let part = world.add_cluster_part(
                root,
                crate::cluster::ClusterPartDesc {
                    local_transform: local,
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::Y * 680.0,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    frame_a: JointFrame::new(Vec3::ZERO, rotation),
                    ..weld()
                },
            )?;
            world.set_part_voxel_wave(part, Some(wave_model()?))?;
            super::super::AerodynamicsTask.execute(&world)?;
            if sharded {
                world.step_with_shards(
                    1e-6,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(1e-6)?;
            }
            let velocity = world.body(id).ok_or("body")?.state().linear_velocity();
            assert!(velocity.y < 680.0 && velocity.y > 0.0);
            assert!(velocity.x.abs() < 1e-12);
        }
        Ok(())
    }

    #[test]
    fn wing_footprint_is_shielded_by_assembly_volume() -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::{
            AerodynamicsTask,
            wing::{FiniteWing, WingSurface},
        };
        for sharded in [false, true] {
            let mut gains = Vec::new();
            for (size, offset) in [(4.0, 0.0), (1.0, 0.0), (1.0, 10.0)] {
                let mut world = PhysicsWorld::new();
                world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                    1.2,
                    Vec3::ZERO,
                )?));
                let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
                let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
                body.set_linear_velocity(Vec3::new(50.0, 0.0, -5.0))?;
                let id = world.add_body(body)?;
                let part = world.enable_cluster(id)?;
                let wing = WingSurface::new(
                    FiniteWing::new(4.0, 0.8, 0.02, 0.0, 0.3)?,
                    Transform::identity(),
                    4.0,
                )?
                .with_rectangular_footprint(Vec3::ZERO, 2.0, 2.0)?;
                world.set_part_aerodynamic_wings(part, vec![wing])?;
                let mut collider = crate::collider::Collider::new(
                    crate::shape::Shape::Cuboid(crate::shape::Cuboid::new(size, size, size)?),
                    Transform::new(Vec3::Y * offset, kapitsa_math::Quat::IDENTITY)?,
                );
                collider.set_mode(crate::collider::CollisionMode::Sensor);
                world.add_part_collider(part, collider)?;
                world.set_part_wave_source_colliders(part, true)?;
                AerodynamicsTask.execute(&world)?;
                world.fill_step_shards(&mut [Vec::new()])?;
                if sharded {
                    world.step_with_shards(
                        1e-4,
                        &mut [Vec::new(), Vec::new()],
                        &SequentialExecutor,
                        1,
                    )?;
                } else {
                    world.step(1e-4)?;
                }
                gains.push(world.body(id).ok_or("body")?.state().linear_velocity().z + 5.0);
            }
            let blocked = *gains.first().ok_or("blocked")?;
            let partial = *gains.get(1).ok_or("partial")?;
            let clear = *gains.get(2).ok_or("clear")?;
            assert!(blocked.abs() < 1e-12, "{gains:?}");
            assert!(partial > 0.0 && partial < clear, "{gains:?}");
        }
        Ok(())
    }

    #[test]
    fn wing_shielding_tracks_collider_and_footprint_edits() -> Result<(), Box<dyn std::error::Error>>
    {
        use crate::aerodynamics::wing::{FiniteWing, WingSurface};
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
            1.2,
            Vec3::ZERO,
        )?));
        let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::new(50.0, 0.0, -5.0))?;
        let id = world.add_body(body)?;
        let part = world.enable_cluster(id)?;
        let model = FiniteWing::new(4.0, 0.8, 0.02, 0.0, 0.3)?;
        let wing = |center| -> Result<WingSurface, crate::Error> {
            WingSurface::new(model, Transform::identity(), 4.0)?
                .with_rectangular_footprint(center, 2.0, 2.0)
        };
        world.set_part_aerodynamic_wings(part, vec![wing(Vec3::ZERO)?])?;
        let mut collider = crate::collider::Collider::new(
            crate::shape::Shape::Cuboid(crate::shape::Cuboid::new(4.0, 4.0, 4.0)?),
            Transform::identity(),
        );
        collider.set_mode(crate::collider::CollisionMode::Sensor);
        let collider_id = world.add_part_collider(part, collider)?;
        world.set_part_wave_source_colliders(part, true)?;
        world.step(1e-4)?;
        let blocked = world.body(id).ok_or("body")?.state().linear_velocity().z;
        world.set_part_collider_transform(
            part,
            collider_id,
            Transform::new(Vec3::Y * 10.0, kapitsa_math::Quat::IDENTITY)?,
        )?;
        world.step(1e-4)?;
        let clear = world.body(id).ok_or("body")?.state().linear_velocity().z;
        assert!(clear > blocked);
        world.set_part_aerodynamic_wings(part, vec![wing(Vec3::Y * 10.0)?])?;
        world.step(1e-4)?;
        let blocked_again = world.body(id).ok_or("body")?.state().linear_velocity().z;
        assert!((blocked_again - clear).abs() < 1e-12);
        Ok(())
    }

    #[test]
    fn rotated_assembly_com_and_geometry_edit_matches_fresh_cache()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::{
            aerodynamics::wing::{FiniteWing, WingSurface},
            cluster::{ClusterPartDesc, MassChangeMode},
            collider::{Collider, CollisionMode},
            shape::{Cuboid, Shape},
        };
        use kapitsa_math::Quat;

        type RotatedAssemblySetup = (
            PhysicsWorld,
            BodyId,
            PartId,
            PartId,
            crate::world::ColliderId,
        );

        fn setup() -> Result<RotatedAssemblySetup, Box<dyn std::error::Error>> {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                1.2,
                Vec3::ZERO,
            )?));
            let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            let velocity = Vec3::new(50.0, 0.0, -5.0);
            body.set_linear_velocity(velocity)?;
            let id = world.add_body(body)?;
            let root = world.enable_cluster(id)?;
            let rotation = Quat::from_axis_angle(Vec3::Y, 0.05).ok_or("part rotation")?;
            let local = Transform::new(Vec3::X * 2.0, rotation)?;
            let part = world.add_cluster_part(
                root,
                ClusterPartDesc {
                    local_transform: local,
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: velocity,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    frame_a: JointFrame::new(Vec3::X * 2.0, rotation),
                    ..weld()
                },
            )?;
            let wing = WingSurface::new(
                FiniteWing::new(4.0, 0.8, 0.02, 0.0, 0.3)?,
                Transform::identity(),
                4.0,
            )?
            .with_rectangular_footprint(Vec3::ZERO, 2.0, 2.0)?
            .with_spanwise_projection(0.3)?;
            world.set_part_aerodynamic_wings(part, vec![wing])?;
            let mut collider = Collider::new(
                Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0)?),
                Transform::identity(),
            );
            collider.set_mode(CollisionMode::Sensor);
            let collider = world.add_part_collider(part, collider)?;
            world.set_part_wave_source_colliders(part, true)?;
            Ok((world, id, root, part, collider))
        }

        for sharded in [false, true] {
            let (mut warmed, id, root, part, collider) = setup()?;
            let (mut reference, reference_id, reference_root, reference_part, reference_collider) =
                setup()?;
            for world in [&mut warmed, &mut reference] {
                world.step(1e-5)?;
            }
            assert!(
                warmed
                    .assembly_aerodynamics
                    .as_ref()
                    .is_some_and(|cache| cache.contains_key(&id))
            );
            for (world, root, part, collider) in [
                (&mut warmed, root, part, collider),
                (
                    &mut reference,
                    reference_root,
                    reference_part,
                    reference_collider,
                ),
            ] {
                world.set_part_mass(
                    root,
                    MassProperties::new(120.0, Vec3::Y * 0.4, Mat3::IDENTITY * 120.0)?,
                    MassChangeMode::KeepVelocityField,
                )?;
                let rotation = Quat::from_axis_angle(Vec3::X, 0.3).ok_or("collider rotation")?;
                world.set_part_collider_transform(
                    part,
                    collider,
                    Transform::new(Vec3::Y * 10.0, rotation)?,
                )?;
            }
            // Reference must rebuild from current sources, not reuse the previous result.
            reference
                .assembly_aerodynamics
                .as_mut()
                .ok_or("reference cache")?
                .remove(&reference_id);
            let before = *warmed.body(id).ok_or("body")?.state();
            if sharded {
                warmed.step_with_shards(
                    1e-5,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
                reference.step_with_shards(
                    1e-5,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
            } else {
                warmed.step(1e-5)?;
                reference.step(1e-5)?;
            }
            let actual = warmed.body(id).ok_or("body")?.state();
            let expected = reference
                .body(reference_id)
                .ok_or("reference body")?
                .state();
            assert!((actual.linear_velocity() - expected.linear_velocity()).length() < 1e-12);
            assert!((actual.angular_velocity() - expected.angular_velocity()).length() < 1e-12);
            assert!((actual.linear_velocity() - before.linear_velocity()).length() > 1e-7);
            assert!((actual.angular_velocity() - before.angular_velocity()).length() > 1e-7);
            println!(
                "sharded={sharded} delta_v={:?} delta_omega={:?}",
                actual.linear_velocity() - before.linear_velocity(),
                actual.angular_velocity() - before.angular_velocity()
            );
        }
        Ok(())
    }

    #[test]
    fn partial_shielding_with_spanwise_projection_advances_multiple_steps()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::wing::{FiniteWing, WingError, WingSurface};
        for projection in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                1.2,
                Vec3::ZERO,
            )?));
            let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::new(50.0, 0.0, -5.0))?;
            let id = world.add_body(body)?;
            let part = world.enable_cluster(id)?;
            let wing = WingSurface::new(
                FiniteWing::new(4.0, 0.8, 0.02, 0.0, 0.3)?,
                Transform::identity(),
                4.0,
            )?
            .with_rectangular_footprint(Vec3::new(0.4, 0.2, 0.0), 2.0, 2.0)?;
            let wing = if projection {
                wing.with_spanwise_projection(0.3)?
            } else {
                wing
            };
            world.set_part_aerodynamic_wings(part, vec![wing])?;
            let mut collider = crate::collider::Collider::new(
                crate::shape::Shape::Cuboid(crate::shape::Cuboid::new(1.0, 1.0, 1.0)?),
                Transform::identity(),
            );
            collider.set_mode(crate::collider::CollisionMode::Sensor);
            world.add_part_collider(part, collider)?;
            world.set_part_wave_source_colliders(part, true)?;
            world.step(1e-4)?;
            assert!(
                world
                    .body(id)
                    .ok_or("body")?
                    .state()
                    .angular_velocity()
                    .length()
                    > 0.0
            );
            if projection {
                for _ in 0..10 {
                    world.step(1e-4)?;
                }
            } else {
                let before = world.body(id).ok_or("body")?.motion();
                assert!(matches!(
                    aero_wing(&world.step(1e-4)),
                    Some(WingError::UnsupportedFlow)
                ));
                assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            }
        }
        Ok(())
    }

    #[test]
    fn wing_shielding_budget_failure_rolls_back_world() -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::wing::{FiniteWing, WingError, WingSurface};
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                1.2,
                Vec3::ZERO,
            )?));
            let mut body = RigidBody::new_dynamic(
                Transform::identity(),
                MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
            )?;
            body.set_linear_velocity(Vec3::new(50.0, 0.0, -5.0))?;
            let id = world.add_body(body)?;
            let part = world.enable_cluster(id)?;
            let valid = WingSurface::new(
                FiniteWing::new(4.0, 0.8, 0.02, 0.0, 0.3)?,
                Transform::identity(),
                4.0,
            )?
            .with_rectangular_footprint(Vec3::ZERO, 2.0, 2.0)?;
            world.set_part_aerodynamic_wings(part, vec![valid.clone()])?;
            let source = plate()?;
            let distant = TriangleMesh::new(
                source
                    .vertices()
                    .iter()
                    .map(|v| *v + Vec3::Y * 10.0)
                    .collect(),
                source.triangles().to_vec(),
            )?;
            world.set_part_wave_source_mesh(part, Some(distant))?;
            world.step(1e-4)?;
            let wing = WingSurface::new(
                FiniteWing::new(4.0, 0.8, 0.02, 0.0, 0.3)?,
                Transform::identity(),
                1e12,
            )?
            .with_rectangular_footprint(Vec3::ZERO, 1e6, 1e6)?;
            world.set_part_aerodynamic_wings(part, vec![wing])?;
            let before = world.body(id).ok_or("body")?.motion();
            let result = if sharded {
                world.step_with_shards(1e-4, &mut [Vec::new()], &SequentialExecutor, 1)
            } else {
                world.step(1e-4)
            };
            assert!(matches!(
                aero_wing(&result),
                Some(WingError::ShieldingBudgetExceeded)
            ));
            assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            world.set_part_aerodynamic_wings(part, vec![valid])?;
            world.step(1e-4)?;
            assert_ne!(world.body(id).ok_or("body")?.motion(), before);
        }
        Ok(())
    }

    #[test]
    fn mixed_pressure_and_wave_geometry_have_distinct_load_owners()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::Aerodynamics;
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                0.001,
                Vec3::ZERO,
            )?));
            let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::X * 2000.0)?;
            body.set_aerodynamics(Aerodynamics::VoxelWave(wave_model()?));
            let id = world.add_body(body)?;
            let root = world.enable_cluster(id)?;
            let pressure_part = world.add_cluster_part(
                root,
                crate::cluster::ClusterPartDesc {
                    local_transform: Transform::new(Vec3::Y * 2.0, kapitsa_math::Quat::IDENTITY)?,
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::X * 2000.0,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    frame_a: JointFrame::new(Vec3::Y * 2.0, kapitsa_math::Quat::IDENTITY),
                    ..weld()
                },
            )?;
            world.set_part_aerodynamic_mesh(pressure_part, Some(plate()?))?;
            let entry = world.bodies.get(id.dense).ok_or("body")?;
            let prepared = prepare(
                id,
                entry,
                &world.parts,
                world.part_aerodynamic_geometry.as_ref(),
                &world.colliders,
                &mut world.assembly_aerodynamics,
                world.atmosphere,
            )?
            .ok_or("assembly")?;
            let patches = prepared.patches.ok_or("pressure patches")?;
            assert!(!patches.is_empty());
            assert!(
                patches
                    .iter()
                    .all(|patch| patch.owner == GeometryOwner::Part(pressure_part))
            );
            assert_eq!(prepared.wave.as_ref().map(|(part, _)| *part), Some(root));
            super::super::AerodynamicsTask.execute(&world)?;
            world.fill_step_shards(&mut [Vec::new()])?;
            if sharded {
                world.step_with_shards(
                    1e-7,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(1e-7)?;
            }
            assert!(world.body(id).ok_or("body")?.state().linear_velocity().x < 2000.0);
        }
        Ok(())
    }

    #[test]
    fn wave_model_follows_owner_through_split() -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::Aerodynamics;
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
            1.2,
            Vec3::ZERO,
        )?));
        let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::X * 680.0)?;
        body.set_aerodynamics(Aerodynamics::VoxelWave(wave_model()?));
        let id = world.add_body(body)?;
        let root = world.enable_cluster(id)?;
        let source = world.add_cluster_part(
            root,
            crate::cluster::ClusterPartDesc {
                local_transform: Transform::new(Vec3::X * 2.0, kapitsa_math::Quat::IDENTITY)?,
                mass,
                colliders: Vec::new(),
                linear_velocity: Vec3::X * 680.0,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame::new(Vec3::X * 2.0, kapitsa_math::Quat::IDENTITY),
                ..weld()
            },
        )?;
        world.set_part_wave_source_mesh(source, Some(plate()?))?;
        world.step(1e-7)?;
        let weld_id = *world
            .cluster(id)
            .ok_or("cluster")?
            .welds()
            .first()
            .ok_or("weld")?;
        let split = world.break_weld(weld_id)?;
        assert_eq!(split.len(), 2);
        let wave_body = world.cluster_part(root).ok_or("root")?.body();
        let source_body = world.cluster_part(source).ok_or("source")?.body();
        assert_ne!(wave_body, source_body);
        let before_wave = world
            .body(wave_body)
            .ok_or("wave body")?
            .state()
            .linear_velocity()
            .x;
        let before_source = world
            .body(source_body)
            .ok_or("source body")?
            .state()
            .linear_velocity()
            .x;
        world.step(1e-7)?;
        assert!(
            world
                .body(wave_body)
                .ok_or("wave body")?
                .state()
                .linear_velocity()
                .x
                < before_wave
        );
        assert_eq!(
            world
                .body(source_body)
                .ok_or("source body")?
                .state()
                .linear_velocity()
                .x,
            before_source
        );
        Ok(())
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
                    Err(crate::Error::from(ClusterError::UnsupportedGeometry))
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
                        Err(Error::from(StepError::Aerodynamics {
                            body_id: first,
                            cause: crate::Error::from(
                                crate::aerodynamics::AerodynamicsError::TessellationBudgetExceeded
                            )
                        }))
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
                Err(crate::Error::from(ClusterError::UnsupportedGeometry))
            );
            assert_eq!(world.body(id).ok_or("body")?.motion(), before);
            let result = if sharded {
                world.step_with_shards(1e-5, &mut [Vec::new()], &SequentialExecutor, 1)
            } else {
                world.step(1e-5)
            };
            assert_eq!(
                result,
                Err(Error::from(StepError::Aerodynamics {
                    body_id: id,
                    cause: crate::Error::from(crate::aerodynamics::AerodynamicsError::Geometry(
                        crate::Error::from(GeometryError::CellBudgetExceeded)
                    ))
                }))
            );
            assert_eq!(
                aero_geometry(&result),
                Some(&GeometryError::CellBudgetExceeded)
            );
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
                Err(crate::Error::from(ClusterError::InvalidTopology))
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
                Err(Error::from(StepError::Aerodynamics {
                    body_id: id,
                    cause: crate::Error::from(
                        crate::aerodynamics::AerodynamicsError::UnsupportedModel
                    )
                }))
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
    fn mesh_pressure_can_break_certified_tree_weld() -> Result<(), Box<dyn std::error::Error>> {
        for (sharded, policy) in [
            (false, super::super::ErrorPolicy::Realtime),
            (true, super::super::ErrorPolicy::Realtime),
            (false, super::super::ErrorPolicy::Strict),
            (true, super::super::ErrorPolicy::Strict),
        ] {
            let mut world = PhysicsWorld::new();
            world.set_error_policy(policy);
            world.set_atmosphere(super::super::test_atmosphere(
                crate::aerodynamics::AtmosphericState::new(0.001, Vec3::ZERO)?,
            ));
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
            let mut rigid = RigidBody::new_dynamic(Transform::identity(), mass)?;
            rigid.set_linear_velocity(Vec3::X * 2000.0)?;
            let body = world.add_body(rigid)?;
            let root = world.enable_cluster(body)?;
            let child = world.add_cluster_part(
                root,
                crate::cluster::ClusterPartDesc {
                    local_transform: Transform::identity(),
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::X * 2000.0,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    strength: WeldStrength::Breakable {
                        max_force: Some(100.0),
                        max_torque: None,
                    },
                    ..weld()
                },
            )?;
            world.set_part_aerodynamic_mesh(child, Some(plate()?))?;
            world.step(1e-8)?;
            let previous = world.owner_mesh_pressure(body).ok_or("pressure")?.to_vec();
            let reactions = world
                .part_aerodynamic_weld_reactions(body, 1e-8, 1e-8)
                .map_err(|e| format!("{e:?}"))?;
            assert!((reactions[0].force - Vec3::X * 2000.0).length() < 1e-6);
            let center = world
                .body(body)
                .ok_or("body")?
                .motion()
                .world_center_of_mass()?
                .ok_or("center")?;
            let bad_gravity =
                world.add_gravity(crate::gravity::CentralGravity::new(center, 1.0)?)?;
            let failed = if sharded {
                world.step_with_shards(1e-8, &mut [Vec::new(), Vec::new()], &SequentialExecutor, 1)
            } else {
                world.step(1e-8)
            };
            assert!(failed.is_err());
            assert_eq!(world.owner_mesh_pressure(body), Some(previous.as_slice()));
            assert_eq!(
                world
                    .part_aerodynamic_weld_reactions(body, 1e-8, 1e-8)
                    .map_err(|e| format!("{e:?}"))?,
                reactions
            );
            world.remove_gravity(bad_gravity).ok_or("gravity")?;
            world.set_mesh_sound_speed(1000.0)?;
            let before = *world.body(body).ok_or("body")?.state();
            let unsupported = if sharded {
                world.step_with_shards(1e-8, &mut [Vec::new(), Vec::new()], &SequentialExecutor, 1)
            } else {
                world.step(1e-8)
            };
            assert!(matches!(
                aero_pressure(&unsupported),
                Some(crate::aerodynamics::pressure::PressureError::UnsupportedMach)
            ));
            assert_eq!(*world.body(body).ok_or("body")?.state(), before);
            assert_eq!(world.owner_mesh_pressure(body), Some(previous.as_slice()));
            assert_eq!(
                world
                    .part_aerodynamic_weld_reactions(body, 1e-8, 1e-8)
                    .map_err(|e| format!("{e:?}"))?,
                reactions
            );
            world.set_mesh_sound_speed(340.0)?;
            world
                .set_automatic_weld_failure(Some((1e-8, 1e-8)))
                .map_err(|e| format!("{e:?}"))?;
            if sharded {
                world.step_with_shards(
                    1e-8,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(1e-8)?;
            }
            assert!(world.cluster(body).ok_or("cluster")?.welds().is_empty());
            assert_ne!(world.cluster_part(child).ok_or("part")?.body(), body);
        }
        Ok(())
    }

    #[test]
    fn vacuum_part_geometry_allows_certified_weld_balance() -> Result<(), Box<dyn std::error::Error>>
    {
        for sharded in [false, true] {
            let mut world = PhysicsWorld::new();
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
            let body = world.add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)?;
            let root = world.enable_cluster(body)?;
            let child = world.add_cluster_part(
                root,
                crate::cluster::ClusterPartDesc {
                    local_transform: Transform::identity(),
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::ZERO,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    strength: WeldStrength::Breakable {
                        max_force: Some(1.0),
                        max_torque: None,
                    },
                    ..weld()
                },
            )?;
            world.set_part_aerodynamic_mesh(child, Some(plate()?))?;
            world
                .set_automatic_weld_failure(Some((1e-9, 1e-9)))
                .map_err(|e| format!("{e:?}"))?;
            if sharded {
                world.step_with_shards(
                    0.01,
                    &mut [Vec::new(), Vec::new()],
                    &SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(0.01)?;
            }
            let reactions = world
                .part_aerodynamic_weld_reactions(body, 1e-9, 1e-9)
                .map_err(|e| format!("{e:?}"))?;
            assert_eq!(reactions.len(), 1);
            assert_eq!(reactions[0].force, Vec3::ZERO);
            assert_eq!(reactions[0].moment, Vec3::ZERO);
            assert_eq!(world.cluster(body).ok_or("cluster")?.welds().len(), 1);
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
                .ok_or("prepared assembly")?
                .patches
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
                let published = world
                    .owner_mesh_pressure(id)
                    .ok_or("published pressure")?
                    .to_vec();
                assert_eq!(published, vec![(GeometryOwner::Part(part), wrench)]);
                let state = world.body(id).ok_or("body")?.state();
                assert!((state.linear_velocity().x - (2000.0 - 2e-7)).abs() < 1e-9);
                let angular = Vec3::new(0.0, moment.y / 200.0, moment.z / 400.0) * 1e-8;
                assert!((state.angular_velocity() - angular).length() < 1e-12);
                assert_eq!(world.bodies().count(), 1);
                let center = world
                    .body(id)
                    .ok_or("body")?
                    .motion()
                    .world_center_of_mass()?
                    .ok_or("center")?;
                let bad = world.add_gravity(crate::gravity::CentralGravity::new(center, 1.0)?)?;
                let failed = if sharded {
                    world.step_with_shards(1e-8, &mut [Vec::new()], &SequentialExecutor, 1)
                } else {
                    world.step(1e-8)
                };
                assert!(failed.is_err());
                assert_eq!(world.owner_mesh_pressure(id), Some(published.as_slice()));
                world.remove_gravity(bad).ok_or("gravity")?;
                world.set_atmosphere(Atmosphere::Vacuum);
                if sharded {
                    world.step_with_shards(1e-8, &mut [Vec::new()], &SequentialExecutor, 1)?;
                } else {
                    world.step(1e-8)?;
                }
                assert!(world.owner_mesh_pressure(id).is_none());
            }
        }
        Ok(())
    }

    #[test]
    fn owner_pressure_survives_failed_owner_change_and_body_removal()
    -> Result<(), Box<dyn std::error::Error>> {
        use super::super::tests::ScopedThreadExecutor;
        use crate::cluster::ClusterPartDesc;

        for mode in 0..3 {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::super::test_atmosphere(AtmosphericState::new(
                0.001,
                Vec3::ZERO,
            )?));
            let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::X * 2000.0)?;
            let id = world.add_body(body)?;
            let root = world.enable_cluster(id)?;
            let child = world.add_cluster_part(
                root,
                ClusterPartDesc {
                    local_transform: Transform::identity(),
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::X * 2000.0,
                    angular_velocity: Vec3::ZERO,
                },
                weld(),
            )?;
            world.set_part_aerodynamic_mesh(root, Some(plate()?))?;
            // A second aerodynamic body ensures the executor has multiple work chunks.
            let mut other = RigidBody::new_dynamic(
                Transform::new(Vec3::Y * 10.0, kapitsa_math::Quat::IDENTITY)?,
                mass,
            )?;
            other.set_linear_velocity(Vec3::X * 2000.0)?;
            let other_id = world.add_body(other)?;
            let other_part = world.enable_cluster(other_id)?;
            world.set_part_aerodynamic_mesh(other_part, Some(plate()?))?;
            let step = |world: &mut PhysicsWorld| match mode {
                0 => world.step(1e-8),
                1 => world.step_with_executor(1e-8, &ScopedThreadExecutor, 2),
                _ => world.step_with_shards(
                    1e-8,
                    &mut [Vec::new(), Vec::new()],
                    &ScopedThreadExecutor,
                    2,
                ),
            };
            step(&mut world)?;
            let previous = world
                .owner_mesh_pressure(id)
                .ok_or("initial pressure")?
                .to_vec();
            assert_eq!(previous.len(), 1);
            assert_eq!(previous[0].0, GeometryOwner::Part(root));
            assert!(world.owner_mesh_pressure(other_id).is_some());
            world.set_part_aerodynamic_mesh(root, None)?;
            world.set_part_aerodynamic_mesh(child, Some(plate()?))?;
            let center = world
                .body(id)
                .ok_or("body")?
                .motion()
                .world_center_of_mass()?
                .ok_or("center")?;
            let bad = world.add_gravity(crate::gravity::CentralGravity::new(center, 1.0)?)?;
            assert!(step(&mut world).is_err());
            assert_eq!(world.owner_mesh_pressure(id), Some(previous.as_slice()));
            world.remove_gravity(bad).ok_or("gravity")?;
            step(&mut world)?;
            let current = world.owner_mesh_pressure(id).ok_or("updated pressure")?;
            assert_eq!(current.len(), 1);
            assert_eq!(current[0].0, GeometryOwner::Part(child));
            assert!(current[0].1.force.length() > 0.0);
            world.remove_body(id).ok_or("removed body")?;
            assert!(world.owner_mesh_pressure(id).is_none());
            assert!(
                !world
                    .owner_pressure
                    .as_ref()
                    .ok_or("pressure storage")?
                    .contains_key(&id)
            );
            step(&mut world)?;
            assert!(world.owner_mesh_pressure(id).is_none());
            assert!(world.owner_mesh_pressure(other_id).is_some());
        }
        Ok(())
    }
}
