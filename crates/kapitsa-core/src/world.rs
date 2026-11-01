use kapitsa_math::{Affine3, Vec3};
use std::{
    collections::HashMap,
    panic::{AssertUnwindSafe, catch_unwind},
    sync::atomic::{AtomicU64, Ordering},
};

#[cfg(feature = "bench-internals")]
use std::time::Instant;

mod error;
mod id;

pub use error::{ForceSourceError, StepError, WorldError};
pub use id::{BodyId, ColliderId, ForceSourceId};

static NEXT_WORLD_ID: AtomicU64 = AtomicU64::new(1);

fn allocate_world_id(counter: &AtomicU64) -> Result<u64, WorldError> {
    counter
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |next| {
            next.checked_add(1)
        })
        .map_err(|_| WorldError::WorldIdExhausted)
}

use crate::{
    aabb::Aabb,
    collider::Collider,
    collision::{
        CcdCandidateToi, CcdHit, CcdIslandBodyResult, CcdIslandCandidateResult,
        CcdIslandImpactResult, CcdIslandResult, CcdLimitPolicy, CcdMotion, CcdTrajectory,
        CollisionConfig, CollisionError, CollisionEvent, CollisionInteraction, CollisionPair,
        CollisionSystem, ContactInfo, ContactManifold, ContactPoint, NarrowPhase, OverlapHit,
        QueryError, QueryFilter, QueryScratch, QueryTarget, Ray, RayHit, ShapeHit, SolverBody,
        SolverManifold, SolverMaterial, StepContext, overlaps_shape_with_scratch,
        raycast_shape_with_scratch, spatial_index::SpatialIndex, sweep_shape_with_scratch,
    },
    containers::DensePool,
    execution::{SequentialExecutor, StepExecutor},
    material::{MaterialId, MaterialStorage, PhysicsMaterial},
    rigid_body::{BodyState, BodyType, CcdMode, RigidBody, predict, predict_from_state},
    shape::Shape,
};

struct BodyEntry {
    body: RigidBody,
    first_collider: Option<ColliderId>,
}

/// Persistent force applied at a point expressed in body-local coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ForceSource {
    body: BodyId,
    local_point: Vec3,
    local_force: Vec3,
    enabled: bool,
}

impl ForceSource {
    /// Returns the dynamic body receiving this force.
    pub fn body(&self) -> BodyId {
        self.body
    }

    /// Returns the application point relative to the body's origin, in meters.
    pub fn local_point(&self) -> Vec3 {
        self.local_point
    }

    /// Returns the force vector in body-local coordinates, in newtons.
    pub fn local_force(&self) -> Vec3 {
        self.local_force
    }

    /// Returns whether this source is applied on each physics step.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

struct ForceSourceBodyUpdate {
    id: BodyId,
    original: RigidBody,
    prepared: RigidBody,
}

const FORCE_SOURCE_BODY_LINEAR_SEARCH_LIMIT: usize = 128;

struct ColliderEntry {
    collider: Collider,
    body: BodyId,
    prev: Option<ColliderId>,
    next: Option<ColliderId>,
}

#[derive(Clone, Copy)]
struct CcdBodyReplaySnapshot {
    prediction: usize,
    ccd_state: BodyState,
    predicted_state: BodyState,
    elapsed: f64,
    impact_count: u32,
    clipped: bool,
    completed: bool,
}

fn swept_bounds_for_trajectory(
    collider: &Collider,
    trajectory: &CcdTrajectory,
    end: BodyState,
) -> Result<Option<crate::aabb::Aabb>, CollisionError> {
    let local_bounds = collider.local_bounds()?;
    let start_transform = trajectory.transform_at(0.0)?;
    let end_transform = end
        .transform()
        .compose(&trajectory.local_transform())
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    if trajectory.has_linear_collider_origin(end) {
        let motion = CcdMotion::linear(start_transform, end_transform)?;
        return crate::collision::swept_bounds_motion(local_bounds, &motion);
    }
    let (translation_bound, rotation_bound, _) = trajectory.path_bounds(end)?;
    let motion = CcdMotion::trajectory(
        start_transform,
        end_transform,
        translation_bound,
        false,
        rotation_bound,
        *trajectory,
    )?;
    crate::collision::swept_bounds_motion(local_bounds, &motion)
}

/// Owns rigid bodies and colliders and assigns IDs scoped to this world instance.
pub struct PhysicsWorld {
    world_id: u64,
    bodies: DensePool<BodyEntry>,
    colliders: DensePool<ColliderEntry>,
    force_sources: DensePool<ForceSource>,
    force_source_body_updates: Vec<ForceSourceBodyUpdate>,
    force_source_body_indices: Option<HashMap<BodyId, usize>>,
    materials: MaterialStorage,
    collision_system: CollisionSystem,
    query_index: Option<SpatialIndex>,
    query_index_dirty: bool,
    query_proxies: Vec<(ColliderId, Aabb)>,
    unbounded_query_colliders: Vec<ColliderId>,
}

/// Opaque copied body data used to predict and atomically commit a world step.
#[doc(hidden)]
#[derive(Clone, Copy, Debug)]
pub struct BodyStep {
    body_id: BodyId,
    source: RigidBody,
    predicted: Option<BodyState>,
    dt: Option<f64>,
    error: Option<StepError>,
}

impl BodyStep {
    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    pub fn integrate(&mut self, dt: f64) -> Result<(), StepError> {
        self.predicted = None;
        self.dt = None;
        self.error = None;
        let predicted = match predict(&self.source, dt) {
            Ok(predicted) => predicted,
            Err(cause) => {
                let error = StepError::BodyIntegration {
                    body_id: self.body_id,
                    cause,
                };
                self.error = Some(error);
                return Err(error);
            }
        };
        self.predicted = Some(predicted);
        self.dt = Some(dt);
        Ok(())
    }
}

impl PhysicsWorld {
    pub const fn new() -> Self {
        Self {
            world_id: 0,
            bodies: DensePool::new(),
            colliders: DensePool::new(),
            force_sources: DensePool::new(),
            force_source_body_updates: Vec::new(),
            force_source_body_indices: None,
            materials: MaterialStorage::new(),
            collision_system: CollisionSystem::new(),
            query_index: None,
            query_index_dirty: false,
            query_proxies: Vec::new(),
            unbounded_query_colliders: Vec::new(),
        }
    }

    pub fn collision_config(&self) -> CollisionConfig {
        self.collision_system.config()
    }

    pub fn set_collision_config(&mut self, config: CollisionConfig) -> Result<(), CollisionError> {
        self.collision_system.set_config(config)
    }

    pub fn collision_events(&self) -> &[CollisionEvent] {
        self.collision_system.events()
    }

    pub fn raycast(&self, ray: Ray, filter: QueryFilter) -> Result<Option<RayHit>, QueryError> {
        self.raycast_with_scratch(ray, filter, &mut QueryScratch::default())
    }

    fn raycast_with_scratch(
        &self,
        ray: Ray,
        filter: QueryFilter,
        scratch: &mut QueryScratch,
    ) -> Result<Option<RayHit>, QueryError> {
        scratch.clear();
        ray.validate()?;
        let mut closest = None;
        if self.query_index_dirty {
            for (dense_id, _) in self.colliders.iter() {
                let collider = ColliderId {
                    world: self.world_id,
                    dense: dense_id,
                };
                self.update_raycast_hit(collider, ray, filter, &mut closest, scratch)?;
            }
        } else {
            for collider in self.unbounded_query_colliders.iter().copied() {
                self.update_raycast_hit(collider, ray, filter, &mut closest, scratch)?;
            }
            if let Some(index) = &self.query_index {
                let mut stack = std::mem::take(&mut scratch.ray_stack);
                let max_distance = closest.map_or(ray.max_distance(), |hit: RayHit| hit.distance);
                let result = index.query_ray(
                    ray.origin(),
                    ray.direction(),
                    max_distance,
                    &mut stack,
                    |collider| {
                        self.update_raycast_hit(collider, ray, filter, &mut closest, scratch)
                            .map(|hit| hit.map(|hit| hit.distance))
                    },
                );
                scratch.ray_stack = stack;
                result?;
            }
        }
        Ok(closest)
    }

    pub fn sweep(
        &self,
        shape: &crate::shape::Shape,
        transform: crate::transform::Transform,
        displacement: Vec3,
        filter: QueryFilter,
    ) -> Result<Option<ShapeHit>, QueryError> {
        self.sweep_with_scratch(
            shape,
            transform,
            displacement,
            filter,
            &mut QueryScratch::default(),
        )
    }

    fn sweep_with_scratch(
        &self,
        shape: &crate::shape::Shape,
        transform: crate::transform::Transform,
        displacement: Vec3,
        filter: QueryFilter,
        scratch: &mut QueryScratch,
    ) -> Result<Option<ShapeHit>, QueryError> {
        scratch.clear();
        if !shape.is_convex() {
            return Err(QueryError::UnsupportedQueryShape);
        }
        if !displacement.is_finite() {
            return Err(QueryError::InvalidInput);
        }
        let end_translation = transform
            .translation()
            .checked_add(displacement)
            .ok_or(QueryError::ArithmeticOverflow)?;
        let end_transform = crate::transform::Transform::new(end_translation, transform.rotation())
            .map_err(|_| QueryError::ArithmeticOverflow)?;
        let start_bounds = shape
            .world_bounds(&transform)
            .map_err(|_| QueryError::Geometry(CollisionError::GeometryBounds))?
            .ok_or(QueryError::UnsupportedQueryShape)?;
        let end_bounds = shape
            .world_bounds(&end_transform)
            .map_err(|_| QueryError::Geometry(CollisionError::GeometryBounds))?
            .ok_or(QueryError::UnsupportedQueryShape)?;
        self.query_candidates(start_bounds.union(&end_bounds), scratch)?;
        let mut closest = None;
        for candidate_index in 0..scratch.colliders.len() {
            let collider_id = scratch.colliders[candidate_index];
            let entry = self
                .colliders
                .get(collider_id.dense)
                .filter(|_| collider_id.world == self.world_id)
                .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
            if !filter.allows(entry.body, entry.collider.filter(), entry.collider.mode()) {
                continue;
            }
            let body = self
                .bodies
                .get(entry.body.dense)
                .filter(|_| entry.body.world == self.world_id)
                .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
            let target_transform = body
                .body
                .state()
                .transform()
                .compose(&entry.collider.local_transform())
                .map_err(|_| QueryError::ArithmeticOverflow)?;
            let Some(hit) = sweep_shape_with_scratch(
                shape,
                transform,
                displacement,
                QueryTarget {
                    shape: entry.collider.shape(),
                    transform: target_transform,
                    bvh: entry.collider.query_bvh(),
                    deformed_vertices: entry.collider.query_vertices(),
                },
                scratch,
            )?
            else {
                continue;
            };
            let position = hit.point_a * 0.5 + hit.point_b * 0.5;
            if !position.is_finite() || !hit.normal.is_finite() || !hit.toi.is_finite() {
                return Err(QueryError::ArithmeticOverflow);
            }
            if closest.is_none_or(|current: ShapeHit| {
                hit.toi < current.toi || (hit.toi == current.toi && collider_id < current.collider)
            }) {
                closest = Some(ShapeHit {
                    body: entry.body,
                    collider: collider_id,
                    position,
                    normal: hit.normal,
                    toi: hit.toi,
                });
            }
        }
        Ok(closest)
    }

    pub fn overlaps(
        &self,
        shape: &crate::shape::Shape,
        transform: crate::transform::Transform,
        filter: QueryFilter,
        output: &mut Vec<OverlapHit>,
    ) -> Result<(), QueryError> {
        self.overlaps_with_scratch(
            shape,
            transform,
            filter,
            output,
            &mut QueryScratch::default(),
        )
    }

    fn overlaps_with_scratch(
        &self,
        shape: &crate::shape::Shape,
        transform: crate::transform::Transform,
        filter: QueryFilter,
        output: &mut Vec<OverlapHit>,
        scratch: &mut QueryScratch,
    ) -> Result<(), QueryError> {
        scratch.clear();
        if !shape.is_convex() {
            return Err(QueryError::UnsupportedQueryShape);
        }
        let bounds = shape
            .world_bounds(&transform)
            .map_err(|_| QueryError::Geometry(CollisionError::GeometryBounds))?
            .ok_or(QueryError::UnsupportedQueryShape)?;
        self.query_candidates(bounds, scratch)?;
        for candidate_index in 0..scratch.colliders.len() {
            let collider_id = scratch.colliders[candidate_index];
            let entry = self
                .colliders
                .get(collider_id.dense)
                .filter(|_| collider_id.world == self.world_id)
                .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
            if !filter.allows(entry.body, entry.collider.filter(), entry.collider.mode()) {
                continue;
            }
            let body = self
                .bodies
                .get(entry.body.dense)
                .filter(|_| entry.body.world == self.world_id)
                .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
            let target_transform = body
                .body
                .state()
                .transform()
                .compose(&entry.collider.local_transform())
                .map_err(|_| QueryError::ArithmeticOverflow)?;
            if overlaps_shape_with_scratch(
                shape,
                transform,
                QueryTarget {
                    shape: entry.collider.shape(),
                    transform: target_transform,
                    bvh: entry.collider.query_bvh(),
                    deformed_vertices: entry.collider.query_vertices(),
                },
                scratch,
            )? {
                scratch.overlaps.push(OverlapHit {
                    body: entry.body,
                    collider: collider_id,
                });
            }
        }
        scratch.overlaps.sort_by_key(|hit| (hit.body, hit.collider));
        output.clear();
        output.extend_from_slice(&scratch.overlaps);
        Ok(())
    }

    fn query_candidates(&self, bounds: Aabb, scratch: &mut QueryScratch) -> Result<(), QueryError> {
        let candidates = &mut scratch.colliders;
        if self.query_index_dirty || self.query_index.is_none() {
            candidates
                .try_reserve(self.colliders.len())
                .map_err(|_| QueryError::Geometry(CollisionError::SpatialIndexCapacityExceeded))?;
            candidates.extend(self.colliders.iter().map(|(dense, _)| ColliderId {
                world: self.world_id,
                dense,
            }));
            return Ok(());
        }

        self.query_index
            .as_ref()
            .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?
            .query_readonly(bounds, &mut scratch.tree_stack, candidates)
            .map_err(QueryError::Geometry)?;
        candidates
            .try_reserve(self.unbounded_query_colliders.len())
            .map_err(|_| QueryError::Geometry(CollisionError::SpatialIndexCapacityExceeded))?;
        candidates.extend(self.unbounded_query_colliders.iter().copied());
        Ok(())
    }

    fn update_raycast_hit(
        &self,
        collider_id: ColliderId,
        ray: Ray,
        filter: QueryFilter,
        closest: &mut Option<RayHit>,
        scratch: &mut QueryScratch,
    ) -> Result<Option<RayHit>, QueryError> {
        let entry = self
            .colliders
            .get(collider_id.dense)
            .filter(|_| collider_id.world == self.world_id)
            .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
        if !filter.allows(entry.body, entry.collider.filter(), entry.collider.mode()) {
            return Ok(None);
        }
        let body = self
            .bodies
            .get(entry.body.dense)
            .filter(|_| entry.body.world == self.world_id)
            .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
        let transform = body
            .body
            .state()
            .transform()
            .compose(&entry.collider.local_transform())
            .map_err(|_| QueryError::ArithmeticOverflow)?;
        let limited_ray =
            ray.with_max_distance(closest.map_or(ray.max_distance(), |hit| hit.distance));
        let Some((position, normal, distance)) = raycast_shape_with_scratch(
            entry.collider.shape(),
            transform,
            limited_ray,
            entry.collider.query_bvh(),
            entry.collider.query_vertices(),
            scratch,
        )?
        else {
            return Ok(None);
        };
        let hit = RayHit {
            body: entry.body,
            collider: collider_id,
            position,
            normal,
            distance,
        };
        if closest.is_none_or(|current| {
            distance < current.distance
                || (distance == current.distance && collider_id < current.collider)
        }) {
            *closest = Some(hit);
        }
        Ok(Some(hit))
    }

    fn mark_query_index_dirty(&mut self) {
        self.query_index_dirty = true;
        if let Some(index) = &mut self.query_index {
            index.mark_dirty();
        }
    }

    fn committed_collider_bounds(
        &self,
        collider_id: ColliderId,
    ) -> Result<Option<Aabb>, CollisionError> {
        let entry = self
            .colliders
            .get(collider_id.dense)
            .filter(|_| collider_id.world == self.world_id)
            .ok_or(CollisionError::InvalidContact)?;
        let body = self
            .bodies
            .get(entry.body.dense)
            .filter(|_| entry.body.world == self.world_id)
            .ok_or(CollisionError::InvalidContact)?;
        entry
            .collider
            .world_bounds(&body.body.state().transform())
            .map_err(|_| CollisionError::GeometryBounds)
    }

    fn update_query_index_for_collider(&mut self, collider_id: ColliderId) {
        if self.query_index_dirty {
            return;
        }
        let bounds = match self.committed_collider_bounds(collider_id) {
            Ok(bounds) => bounds,
            Err(_) => {
                self.mark_query_index_dirty();
                return;
            }
        };
        let Some(bounds) = bounds else {
            return;
        };
        let bounds = match expanded_query_bounds(bounds) {
            Ok(bounds) => bounds,
            Err(_) => {
                self.mark_query_index_dirty();
                return;
            }
        };
        let result = self
            .query_index
            .get_or_insert_with(SpatialIndex::new)
            .insert(collider_id, bounds);
        if result.is_err() {
            self.mark_query_index_dirty();
        }
    }

    fn rebuild_query_index(&mut self) {
        self.query_proxies.clear();
        let build_result = (|| {
            for (dense_id, entry) in self.colliders.iter() {
                let collider_id = ColliderId {
                    world: self.world_id,
                    dense: dense_id,
                };
                let body = self
                    .bodies
                    .get(entry.body.dense)
                    .filter(|_| entry.body.world == self.world_id)
                    .ok_or(CollisionError::InvalidContact)?;
                let bounds = entry
                    .collider
                    .world_bounds(&body.body.state().transform())
                    .map_err(|_| CollisionError::GeometryBounds)?;
                if let Some(bounds) = bounds {
                    self.query_proxies
                        .try_reserve(1)
                        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                    self.query_proxies
                        .push((collider_id, expanded_query_bounds(bounds)?));
                }
            }
            Ok::<(), CollisionError>(())
        })();
        if build_result.is_err() {
            self.mark_query_index_dirty();
            return;
        }
        if self.query_index.is_none() && self.query_proxies.is_empty() {
            self.query_index_dirty = false;
            return;
        }
        let result = self
            .query_index
            .get_or_insert_with(SpatialIndex::new)
            .synchronize(&self.query_proxies);
        if result.is_err() {
            self.mark_query_index_dirty();
        } else {
            self.query_index_dirty = false;
        }
    }

    pub fn add_material(&mut self, material: PhysicsMaterial) -> Result<MaterialId, WorldError> {
        self.materials.insert(material).map_err(WorldError::from)
    }

    pub fn material(&self, id: MaterialId) -> Option<&PhysicsMaterial> {
        self.materials.get(id)
    }

    pub fn material_mut(&mut self, id: MaterialId) -> Option<&mut PhysicsMaterial> {
        self.materials.get_mut(id)
    }

    pub fn remove_material(&mut self, id: MaterialId) -> Result<PhysicsMaterial, WorldError> {
        if !self.materials.contains(id) {
            return Err(WorldError::InvalidMaterialId);
        }
        if self
            .colliders
            .iter()
            .any(|(_, entry)| entry.collider.material() == Some(id))
        {
            return Err(WorldError::MaterialInUse);
        }
        self.materials
            .remove(id)
            .ok_or(WorldError::InvalidMaterialId)
    }

    pub fn set_collider_material(
        &mut self,
        id: ColliderId,
        material: Option<MaterialId>,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id || self.colliders.get(id.dense).is_none() {
            return Err(WorldError::InvalidColliderId);
        }
        if material.is_some_and(|material| !self.materials.contains(material)) {
            return Err(WorldError::InvalidMaterialId);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidColliderId)?;
        entry.collider.set_material(material);
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Ok(())
    }

    /// Sets whether an attached collider responds physically or only reports intersections.
    pub fn set_collider_mode(
        &mut self,
        id: ColliderId,
        mode: crate::collider::CollisionMode,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidColliderId);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidColliderId)?;
        if entry.collider.mode() != mode {
            entry.collider.set_mode(mode);
            self.collision_system.invalidate_collider_contacts(id);
        }
        Ok(())
    }

    pub fn add_body(&mut self, body: RigidBody) -> Result<BodyId, WorldError> {
        if self.world_id == 0 {
            self.world_id = allocate_world_id(&NEXT_WORLD_ID)?;
        }
        let dense = self
            .bodies
            .insert(BodyEntry {
                body,
                first_collider: None,
            })
            .map_err(WorldError::from)?;
        Ok(BodyId {
            world: self.world_id,
            dense,
        })
    }

    pub fn body(&self, id: BodyId) -> Option<&RigidBody> {
        if id.world != self.world_id {
            return None;
        }
        self.bodies.get(id.dense).map(|entry| &entry.body)
    }

    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut RigidBody> {
        if id.world != self.world_id {
            return None;
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.bodies.get_mut(id.dense).map(|entry| &mut entry.body)
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Adds a persistent force source to an existing dynamic body.
    pub fn add_force_source(
        &mut self,
        body: BodyId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<ForceSourceId, ForceSourceError> {
        let Some(body_state) = self.body(body) else {
            return Err(ForceSourceError::InvalidBodyId);
        };
        if body_state.physics().body_type() != BodyType::Dynamic {
            return Err(ForceSourceError::BodyMustBeDynamic);
        }
        validate_force_source_values(local_point, local_force)?;
        let dense = self
            .force_sources
            .insert(ForceSource {
                body,
                local_point,
                local_force,
                enabled: true,
            })
            .map_err(|_| ForceSourceError::StorageCapacityExceeded)?;
        Ok(ForceSourceId {
            world: self.world_id,
            dense,
        })
    }

    /// Looks up a force source by its world-scoped ID.
    pub fn force_source(&self, id: ForceSourceId) -> Option<&ForceSource> {
        (id.world == self.world_id)
            .then(|| self.force_sources.get(id.dense))
            .flatten()
    }

    /// Replaces the local application point and force vector of a source.
    pub fn set_force_source(
        &mut self,
        id: ForceSourceId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<(), ForceSourceError> {
        validate_force_source_values(local_point, local_force)?;
        let source = self
            .force_source_mut(id)
            .ok_or(ForceSourceError::InvalidForceSourceId)?;
        source.local_point = local_point;
        source.local_force = local_force;
        Ok(())
    }

    /// Enables or disables a source without removing it.
    pub fn set_force_source_enabled(
        &mut self,
        id: ForceSourceId,
        enabled: bool,
    ) -> Result<(), ForceSourceError> {
        self.force_source_mut(id)
            .ok_or(ForceSourceError::InvalidForceSourceId)?
            .enabled = enabled;
        Ok(())
    }

    /// Removes a source and returns its last configuration.
    pub fn remove_force_source(
        &mut self,
        id: ForceSourceId,
    ) -> Result<ForceSource, ForceSourceError> {
        if id.world != self.world_id {
            return Err(ForceSourceError::InvalidForceSourceId);
        }
        self.force_sources
            .remove(id.dense)
            .ok_or(ForceSourceError::InvalidForceSourceId)
    }

    fn force_source_mut(&mut self, id: ForceSourceId) -> Option<&mut ForceSource> {
        if id.world != self.world_id {
            return None;
        }
        self.force_sources.get_mut(id.dense)
    }

    /// Advances every body by one fixed timestep, atomically committing the results.
    pub fn step(&mut self, dt: f64) -> Result<(), StepError> {
        self.step_with_executor(dt, &SequentialExecutor, 1)
    }

    /// Advances one step using the supplied statically dispatched task executor.
    pub fn step_with_executor<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if worker_count <= 1 {
            return self.step_with_executor_inner(dt, executor, worker_count);
        }
        match catch_unwind(AssertUnwindSafe(|| {
            self.step_with_executor_inner(dt, executor, worker_count)
        })) {
            Ok(result) => result,
            Err(_) => {
                self.collision_system.abort_step();
                self.rollback_force_source_updates();
                Err(StepError::Collision(CollisionError::WorkerFailed))
            }
        }
    }

    fn step_with_executor_inner<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(StepError::InvalidDeltaTime);
        }
        self.validate_collision_inputs()
            .map_err(StepError::Collision)?;
        if let Err((source_id, cause)) = self.prepare_force_source_updates() {
            return Err(StepError::ForceSource { source_id, cause });
        }

        let result = self.step_prepared(dt, executor, worker_count);
        if result.is_err() {
            self.rollback_force_source_updates();
        } else {
            self.clear_force_source_updates();
        }
        result
    }

    fn step_prepared<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        self.collision_system.begin_step();

        for (dense_id, entry) in self.bodies.iter() {
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            match predict(&entry.body, dt) {
                Ok(state) => self.collision_system.stage_prediction(body_id, state),
                Err(cause) => {
                    self.collision_system.abort_step();
                    return Err(StepError::BodyIntegration { body_id, cause });
                }
            }
        }

        let worker_count = worker_count.max(1);
        if let Err(error) = self.prepare_collision_candidates(dt, worker_count, executor) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(worker_count, executor) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }

        #[cfg(feature = "bench-internals")]
        let commit_started = Instant::now();
        self.commit_predicted_states();
        self.collision_system.commit_step();
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_commit(commit_started.elapsed());
        Ok(())
    }

    fn prepare_force_source_updates(&mut self) -> Result<(), (ForceSourceId, ForceSourceError)> {
        self.clear_force_source_updates();
        if self.force_sources.is_empty() {
            return Ok(());
        }

        for (dense, source) in self.force_sources.iter() {
            let source_id = ForceSourceId {
                world: self.world_id,
                dense,
            };
            let source = *source;
            if !source.enabled {
                continue;
            }
            let Some(body) = self.body(source.body).copied() else {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::InvalidBodyId));
            };
            if body.physics().body_type() != BodyType::Dynamic {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::BodyMustBeDynamic));
            }

            let index_active = self
                .force_source_body_indices
                .as_ref()
                .is_some_and(|indices| !indices.is_empty());
            let update_index = if index_active {
                self.force_source_body_indices
                    .as_ref()
                    .and_then(|indices| indices.get(&source.body).copied())
            } else {
                self.force_source_body_updates
                    .iter()
                    .position(|update| update.id == source.body)
            };
            let update_index = match update_index {
                Some(index) => index,
                None => {
                    if self.force_source_body_updates.try_reserve(1).is_err() {
                        self.force_source_body_updates.clear();
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.clear();
                        }
                        return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                    }
                    if index_active {
                        let map_reserve_failed = self
                            .force_source_body_indices
                            .as_mut()
                            .is_none_or(|indices| indices.try_reserve(1).is_err());
                        if map_reserve_failed {
                            self.force_source_body_updates.clear();
                            if let Some(indices) = &mut self.force_source_body_indices {
                                indices.clear();
                            }
                            return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                        }
                    }
                    let index = self.force_source_body_updates.len();
                    self.force_source_body_updates.push(ForceSourceBodyUpdate {
                        id: source.body,
                        original: body,
                        prepared: body,
                    });
                    if index_active {
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.insert(source.body, index);
                        }
                    } else if self.force_source_body_updates.len()
                        > FORCE_SOURCE_BODY_LINEAR_SEARCH_LIMIT
                    {
                        let indices = self
                            .force_source_body_indices
                            .get_or_insert_with(HashMap::new);
                        if indices
                            .try_reserve(self.force_source_body_updates.len())
                            .is_err()
                        {
                            self.force_source_body_updates.clear();
                            indices.clear();
                            return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                        }
                        for (index, update) in self.force_source_body_updates.iter().enumerate() {
                            indices.insert(update.id, index);
                        }
                    }
                    index
                }
            };

            let body = &mut self.force_source_body_updates[update_index].prepared;
            let transform = body.state().transform();
            let force = transform.transform_vector(source.local_force);
            let point = transform.transform_point(source.local_point);
            if let Err(error) = body.apply_force_at_point(force, point) {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::from(error)));
            }
        }

        for update in &self.force_source_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body = update.prepared;
            }
        }
        Ok(())
    }

    fn clear_force_source_updates(&mut self) {
        self.force_source_body_updates.clear();
        if let Some(indices) = &mut self.force_source_body_indices {
            indices.clear();
        }
    }

    fn rollback_force_source_updates(&mut self) {
        for update in &self.force_source_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body = update.original;
            }
        }
        self.clear_force_source_updates();
    }

    fn stage_spatial_proxies(&mut self, dt: f64) -> Result<(), CollisionError> {
        for ((body_dense, body_entry), (predicted_id, predicted_state)) in self.bodies.iter().zip(
            self.collision_system
                .context
                .predicted_states
                .iter()
                .copied(),
        ) {
            let body_id = BodyId {
                world: self.world_id,
                dense: body_dense,
            };
            if predicted_id != body_id {
                return Err(CollisionError::InvalidContact);
            }
            let body_type = body_entry.body.physics().body_type();
            let mut next = body_entry.first_collider;
            while let Some(collider_id) = next {
                if collider_id.world != self.world_id {
                    return Err(CollisionError::InvalidContact);
                }
                let collider_entry = self
                    .colliders
                    .get(collider_id.dense)
                    .ok_or(CollisionError::InvalidContact)?;
                if collider_entry.body != body_id {
                    return Err(CollisionError::InvalidContact);
                }
                let bounds = if body_type != BodyType::Static {
                    let trajectory = CcdTrajectory::new(
                        body_entry.body,
                        *body_entry.body.state(),
                        0.0,
                        0.0,
                        dt,
                        collider_entry.collider.local_transform(),
                        false,
                    );
                    swept_bounds_for_trajectory(
                        &collider_entry.collider,
                        &trajectory,
                        predicted_state,
                    )?
                } else {
                    collider_entry
                        .collider
                        .world_bounds(&predicted_state.transform())
                        .map_err(|_| CollisionError::GeometryBounds)?
                };
                self.collision_system
                    .context
                    .colliders
                    .try_reserve(1)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                self.collision_system
                    .context
                    .colliders
                    .push(crate::collision::CollisionProxy {
                        collider: collider_id,
                        body: body_id,
                        bounds,
                        body_type,
                        mode: collider_entry.collider.mode(),
                        filter: collider_entry.collider.filter(),
                        convex: collider_entry.collider.shape().is_convex(),
                    });
                if let Some(bounds) = bounds {
                    self.collision_system
                        .context
                        .spatial_proxies
                        .try_reserve(1)
                        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                    self.collision_system
                        .context
                        .spatial_proxies
                        .push((collider_id, bounds));
                }
                next = collider_entry.next;
            }
        }
        Ok(())
    }

    fn commit_predicted_states(&mut self) {
        {
            let world_id = self.world_id;
            let colliders = &self.colliders;
            let query_index = &mut self.query_index;
            let query_index_dirty = &mut self.query_index_dirty;
            for ((dense_id, entry), (body_id, state)) in self.bodies.iter_mut().zip(
                self.collision_system
                    .context
                    .predicted_states
                    .iter()
                    .copied(),
            ) {
                let current_id = BodyId {
                    world: world_id,
                    dense: dense_id,
                };
                debug_assert_eq!(current_id, body_id);
                let transform_changed = entry.body.state().transform() != state.transform();
                entry.body.commit_integrated_state(state);
                if transform_changed
                    && !*query_index_dirty
                    && let Some(index) = query_index.as_mut()
                    && update_query_index_for_body(index, colliders, world_id, body_id, entry)
                        .is_err()
                {
                    *query_index_dirty = true;
                    index.mark_dirty();
                }
            }
        }
        if self.query_index_dirty {
            self.rebuild_query_index();
        }
    }

    fn prepare_collision_candidates(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
    ) -> Result<(), CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.stage_spatial_proxies(dt)?;
        self.collision_system.synchronize_spatial_index()?;
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_spatial_index(started.elapsed());

        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.collision_system.generate_candidates()?;
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_broad_phase(started.elapsed());

        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.process_continuous_collisions(dt, worker_count, executor)?;
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_ccd(started.elapsed());
        Ok(())
    }

    fn stage_collision_events(&mut self) -> Result<(), CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        let result = self.collision_system.stage_lifecycle_events();
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_events(started.elapsed());
        result
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn collision_step_metrics(&self) -> crate::collision::CollisionStepMetrics {
        let mut metrics = self.collision_system.benchmark_metrics();
        let query_index_bytes = self
            .query_index
            .as_ref()
            .map_or(0, SpatialIndex::retained_capacity_bytes)
            .saturating_add(
                self.query_proxies.capacity() * std::mem::size_of::<(ColliderId, Aabb)>(),
            )
            .saturating_add(
                self.unbounded_query_colliders.capacity() * std::mem::size_of::<ColliderId>(),
            );
        metrics.committed_query_index_bytes = query_index_bytes;
        metrics
    }

    fn process_continuous_collisions(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
    ) -> Result<(), CollisionError> {
        if dt <= 0.0 || self.collision_system.candidate_count() == 0 {
            return Ok(());
        }
        if !self.bodies.iter().any(|(_, entry)| {
            entry.body.physics().body_type() == BodyType::Dynamic
                && entry.body.ccd_mode() == CcdMode::Continuous
        }) {
            return Ok(());
        }
        Self::rebuild_body_indices(&mut self.collision_system.context)?;

        let prediction_count = self.collision_system.context.predicted_states.len();
        {
            let context = &mut self.collision_system.context;
            context.ccd_states.clear();
            context.ccd_elapsed.clear();
            context.ccd_impact_counts.clear();
            context.ccd_clipped_bodies.clear();
            context.ccd_completed_bodies.clear();
            #[cfg(feature = "bench-internals")]
            context.ccd_clipped_metrics.clear();
            context.ccd_body_types.clear();
            context.ccd_continuous_bodies.clear();
            context
                .ccd_states
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_elapsed
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_impact_counts
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_clipped_bodies
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_completed_bodies
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            #[cfg(feature = "bench-internals")]
            context
                .ccd_clipped_metrics
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_body_types
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_continuous_bodies
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        for index in 0..prediction_count {
            let body_id = self.collision_system.context.predicted_states[index].0;
            let body = self
                .bodies
                .get(body_id.dense)
                .filter(|_| body_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?;
            let context = &mut self.collision_system.context;
            context.ccd_states.push(*body.state());
            context.ccd_elapsed.push(0.0);
            context.ccd_impact_counts.push(0);
            context.ccd_clipped_bodies.push(false);
            context.ccd_completed_bodies.push(false);
            #[cfg(feature = "bench-internals")]
            context.ccd_clipped_metrics.push(false);
            context.ccd_body_types.push(body.physics().body_type());
            context.ccd_continuous_bodies.push(
                body.physics().body_type() == BodyType::Dynamic
                    && body.ccd_mode() == CcdMode::Continuous,
            );
        }
        #[cfg(feature = "bench-internals")]
        let ccd_prepare_started = Instant::now();
        self.rebuild_ccd_candidate_bodies()?;
        let mut force_single_island = false;
        self.collision_system
            .build_ccd_islands(force_single_island)?;
        self.collision_system.context.ccd_processed_pairs.clear();
        self.collision_system.context.ccd_processed_time = 0.0;
        #[cfg(feature = "bench-internals")]
        {
            let candidate_count = self.collision_system.candidate_count();
            let seen_pairs = self
                .collision_system
                .context
                .ccd_toi_seen_pairs
                .get_or_insert_with(std::collections::HashSet::new);
            seen_pairs
                .try_reserve(candidate_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        self.precompute_ccd_tois_parallel(dt, worker_count, executor)?;
        self.collision_system.build_ccd_islands(false)?;
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_island_prepare(ccd_prepare_started.elapsed());

        #[cfg(feature = "bench-internals")]
        let ccd_execute_started = Instant::now();
        let time_tolerance = f64::EPSILON * dt.max(1.0) * 128.0;
        self.resolve_single_candidate_ccd_islands_parallel(
            dt,
            worker_count,
            executor,
            time_tolerance,
            &mut force_single_island,
        )?;
        'passes: loop {
            let mut island_index = 0;
            while island_index < self.collision_system.ccd_island_count() {
                if self.collision_system.ccd_island_completed(island_index)? {
                    island_index += 1;
                    continue;
                }
                self.collision_system.context.ccd_processed_pairs.clear();
                self.collision_system.context.ccd_processed_time = 0.0;
                let mut elapsed = 0.0;
                loop {
                    let remaining = (dt - elapsed).max(0.0);
                    if remaining <= f64::EPSILON * dt.max(1.0) {
                        break;
                    }

                    let mut earliest: Option<(CollisionPair, CcdHit, f64)> = None;
                    let island_candidate_count = self
                        .collision_system
                        .ccd_island_candidates(island_index)
                        .map(<[usize]>::len)
                        .ok_or(CollisionError::InvalidContact)?;
                    #[cfg(feature = "bench-internals")]
                    let toi_search_started = Instant::now();
                    for island_candidate_index in 0..island_candidate_count {
                        let candidate_index = self
                            .collision_system
                            .ccd_island_candidates(island_index)
                            .and_then(|candidates| candidates.get(island_candidate_index))
                            .copied()
                            .ok_or(CollisionError::InvalidContact)?;
                        let pair = self
                            .collision_system
                            .ccd_candidate_pair(candidate_index)
                            .ok_or(CollisionError::InvalidContact)?;
                        let cached = self
                            .collision_system
                            .context
                            .ccd_toi_cache
                            .get(candidate_index)
                            .copied()
                            .ok_or(CollisionError::InvalidContact)?;
                        if cached.evaluated {
                            if let Some(hit) = cached.hit
                                && earliest.is_none_or(|(current_pair, _, current_time)| {
                                    cached.impact_time < current_time
                                        || (cached.impact_time == current_time
                                            && pair < current_pair)
                                })
                            {
                                earliest = Some((pair, hit, cached.impact_time));
                            }
                            continue;
                        }
                        let context = &self.collision_system.context;
                        if elapsed <= context.ccd_processed_time + time_tolerance
                            && context.ccd_processed_pairs.contains(&pair)
                        {
                            self.collision_system.context.ccd_toi_cache[candidate_index] =
                                super::collision::CcdCandidateToi {
                                    evaluated: true,
                                    ..Default::default()
                                };
                            continue;
                        }
                        let (collider_a, collider_b) = {
                            let collider_a = self
                                .colliders
                                .get(pair.a.dense)
                                .filter(|_| pair.a.world == self.world_id)
                                .ok_or(CollisionError::InvalidContact)?;
                            let collider_b = self
                                .colliders
                                .get(pair.b.dense)
                                .filter(|_| pair.b.world == self.world_id)
                                .ok_or(CollisionError::InvalidContact)?;
                            (collider_a, collider_b)
                        };
                        let body_a = self
                            .bodies
                            .get(collider_a.body.dense)
                            .filter(|_| collider_a.body.world == self.world_id)
                            .map(|entry| entry.body)
                            .ok_or(CollisionError::InvalidContact)?;
                        let body_b = self
                            .bodies
                            .get(collider_b.body.dense)
                            .filter(|_| collider_b.body.world == self.world_id)
                            .map(|entry| entry.body)
                            .ok_or(CollisionError::InvalidContact)?;
                        if ![body_a, body_b].into_iter().any(|body| {
                            body.physics().body_type() == BodyType::Dynamic
                                && body.ccd_mode() == CcdMode::Continuous
                        }) {
                            self.collision_system.context.ccd_toi_cache[candidate_index]
                                .evaluated = true;
                            continue;
                        }

                        let prediction_a = Self::predicted_body_index(
                            &self.collision_system.context.body_indices,
                            self.world_id,
                            collider_a.body,
                        )?;
                        let prediction_b = Self::predicted_body_index(
                            &self.collision_system.context.body_indices,
                            self.world_id,
                            collider_b.body,
                        )?;
                        let state_a =
                            self.body_state_at_ccd(collider_a.body, prediction_a, elapsed, dt)?;
                        let state_b =
                            self.body_state_at_ccd(collider_b.body, prediction_b, elapsed, dt)?;
                        let end_state_a = self
                            .collision_system
                            .context
                            .predicted_states
                            .get(prediction_a)
                            .map(|(_, state)| *state)
                            .ok_or(CollisionError::InvalidContact)?;
                        let end_state_b = self
                            .collision_system
                            .context
                            .predicted_states
                            .get(prediction_b)
                            .map(|(_, state)| *state)
                            .ok_or(CollisionError::InvalidContact)?;
                        let start_transform_a = state_a
                            .transform()
                            .compose(&collider_a.collider.local_transform())
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let start_transform_b = state_b
                            .transform()
                            .compose(&collider_b.collider.local_transform())
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let end_transform_a = end_state_a
                            .transform()
                            .compose(&collider_a.collider.local_transform())
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let end_transform_b = end_state_b
                            .transform()
                            .compose(&collider_b.collider.local_transform())
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let local_transform_a = collider_a.collider.local_transform();
                        let local_transform_b = collider_b.collider.local_transform();
                        let context = &self.collision_system.context;
                        let trajectory_a = CcdTrajectory::new(
                            body_a,
                            context.ccd_states[prediction_a],
                            context.ccd_elapsed[prediction_a],
                            elapsed,
                            dt,
                            local_transform_a,
                            context.ccd_clipped_bodies[prediction_a],
                        );
                        let trajectory_b = CcdTrajectory::new(
                            body_b,
                            context.ccd_states[prediction_b],
                            context.ccd_elapsed[prediction_b],
                            elapsed,
                            dt,
                            local_transform_b,
                            context.ccd_clipped_bodies[prediction_b],
                        );
                        let (translation_bound_a, rotation_bound_a, linear_translation_a) =
                            trajectory_a.path_bounds(end_state_a)?;
                        let (translation_bound_b, rotation_bound_b, linear_translation_b) =
                            trajectory_b.path_bounds(end_state_b)?;
                        let motion_a = CcdMotion::trajectory(
                            start_transform_a,
                            end_transform_a,
                            translation_bound_a,
                            linear_translation_a,
                            rotation_bound_a,
                            trajectory_a,
                        )?;
                        let motion_b = CcdMotion::trajectory(
                            start_transform_b,
                            end_transform_b,
                            translation_bound_b,
                            linear_translation_b,
                            rotation_bound_b,
                            trajectory_b,
                        )?;
                        let sensor = collider_a.collider.mode()
                            == crate::collider::CollisionMode::Sensor
                            || collider_b.collider.mode() == crate::collider::CollisionMode::Sensor;
                        #[cfg(feature = "bench-internals")]
                        let repeated_toi_call = if let Some(seen_pairs) =
                            &mut self.collision_system.context.ccd_toi_seen_pairs
                        {
                            !seen_pairs.insert(pair)
                        } else {
                            false
                        };
                        #[cfg(feature = "bench-internals")]
                        if repeated_toi_call {
                            self.collision_system.record_ccd_repeated_toi_call();
                        }
                        let hit = self.collision_system.time_of_impact(
                            collider_a.collider.shape(),
                            &motion_a,
                            collider_a.collider.query_bvh(),
                            collider_a.collider.query_vertices(),
                            collider_b.collider.shape(),
                            &motion_b,
                            collider_b.collider.query_bvh(),
                            collider_b.collider.query_vertices(),
                            sensor,
                        )?;
                        let cached = self
                            .collision_system
                            .context
                            .ccd_toi_cache
                            .get_mut(candidate_index)
                            .ok_or(CollisionError::InvalidContact)?;
                        cached.evaluated = true;
                        cached.hit = None;
                        let Some(hit) = hit else {
                            continue;
                        };
                        if !hit.toi.is_finite() || !(0.0..=1.0).contains(&hit.toi) {
                            return Err(CollisionError::InvalidContact);
                        }
                        if sensor {
                            upsert_interaction(
                                &mut self.collision_system.context.interactions,
                                &mut self.collision_system.context.interaction_indices,
                                CollisionInteraction {
                                    pair,
                                    sensor: true,
                                    transient: true,
                                    contact: None,
                                    impact: false,
                                },
                            )?;
                            continue;
                        }
                        if hit.toi <= f64::EPSILON * 128.0 {
                            let velocity_a = body_point_velocity(body_a, state_a, hit.point_a)?;
                            let velocity_b = body_point_velocity(body_b, state_b, hit.point_b)?;
                            let normal_speed = velocity_b
                                .checked_sub(velocity_a)
                                .and_then(|velocity| velocity.checked_dot(hit.normal))
                                .ok_or(CollisionError::ArithmeticOverflow)?;
                            if normal_speed >= 0.0 {
                                continue;
                            }
                        }
                        let impact_time = elapsed + remaining * hit.toi;
                        cached.impact_time = impact_time;
                        cached.hit = Some(hit);
                        self.collision_system
                            .set_ccd_candidate_active(candidate_index, true)?;
                        if earliest.is_none_or(|(current_pair, _, current_time)| {
                            impact_time < current_time
                                || (impact_time == current_time && pair < current_pair)
                        }) {
                            earliest = Some((pair, hit, impact_time));
                        }
                    }

                    #[cfg(feature = "bench-internals")]
                    self.collision_system
                        .record_ccd_toi_search(toi_search_started.elapsed());

                    let Some((pair, hit, impact_elapsed)) = earliest else {
                        break;
                    };

                    let (body_a_id, body_b_id) = {
                        let collider_a = self
                            .colliders
                            .get(pair.a.dense)
                            .filter(|_| pair.a.world == self.world_id)
                            .ok_or(CollisionError::InvalidContact)?;
                        let collider_b = self
                            .colliders
                            .get(pair.b.dense)
                            .filter(|_| pair.b.world == self.world_id)
                            .ok_or(CollisionError::InvalidContact)?;
                        (collider_a.body, collider_b.body)
                    };
                    let prediction_a = Self::predicted_body_index(
                        &self.collision_system.context.body_indices,
                        self.world_id,
                        body_a_id,
                    )?;
                    let prediction_b = Self::predicted_body_index(
                        &self.collision_system.context.body_indices,
                        self.world_id,
                        body_b_id,
                    )?;
                    if impact_elapsed < elapsed {
                        return Err(CollisionError::NonConvergent);
                    }
                    let (body_a, body_b) = {
                        (
                            self.bodies
                                .get(body_a_id.dense)
                                .filter(|_| body_a_id.world == self.world_id)
                                .map(|entry| entry.body)
                                .ok_or(CollisionError::InvalidContact)?,
                            self.bodies
                                .get(body_b_id.dense)
                                .filter(|_| body_b_id.world == self.world_id)
                                .map(|entry| entry.body)
                                .ok_or(CollisionError::InvalidContact)?,
                        )
                    };
                    #[cfg(feature = "bench-internals")]
                    let contact_resolution_started = Instant::now();
                    let state_a =
                        self.body_state_at_ccd(body_a_id, prediction_a, impact_elapsed, dt)?;
                    let state_b =
                        self.body_state_at_ccd(body_b_id, prediction_b, impact_elapsed, dt)?;
                    let body_transform_a = state_a.transform();
                    let body_transform_b = state_b.transform();
                    let separation = hit
                        .point_b
                        .checked_sub(hit.point_a)
                        .and_then(|delta| delta.checked_dot(hit.normal))
                        .ok_or(CollisionError::ArithmeticOverflow)?;
                    let contact = ContactPoint::from_world(
                        &body_transform_a,
                        &body_transform_b,
                        hit.point_a,
                        hit.point_b,
                        hit.normal,
                        (-separation).max(0.0),
                        0,
                    )?;
                    let contact_position = hit
                        .point_a
                        .checked_add(hit.point_b)
                        .and_then(|point| point.checked_mul_scalar(0.5))
                        .ok_or(CollisionError::ArithmeticOverflow)?;
                    let mut manifold = ContactManifold::new(pair);
                    manifold.insert(contact, contact_position)?;

                    let (body_a, body_b, mode_a, mode_b, material_a, material_b) = {
                        let collider_a = self
                            .colliders
                            .get(pair.a.dense)
                            .ok_or(CollisionError::InvalidContact)?;
                        let collider_b = self
                            .colliders
                            .get(pair.b.dense)
                            .ok_or(CollisionError::InvalidContact)?;
                        (
                            body_a,
                            body_b,
                            collider_a.collider.mode(),
                            collider_b.collider.mode(),
                            collider_a.collider.material(),
                            collider_b.collider.material(),
                        )
                    };
                    if mode_a == crate::collider::CollisionMode::Sensor
                        || mode_b == crate::collider::CollisionMode::Sensor
                    {
                        return Err(CollisionError::InvalidContact);
                    }
                    let mut budget_exhausted = false;
                    for (body, prediction) in [(body_a, prediction_a), (body_b, prediction_b)] {
                        if body.physics().body_type() == BodyType::Dynamic
                            && body.ccd_mode() == CcdMode::Continuous
                        {
                            let impacts =
                                self.collision_system.context.ccd_impact_counts[prediction];
                            budget_exhausted |=
                                impacts >= self.collision_system.config().max_ccd_impacts;
                        }
                    }
                    let already_clipped = self.collision_system.context.ccd_clipped_bodies
                        [prediction_a]
                        || self.collision_system.context.ccd_clipped_bodies[prediction_b];
                    let body_a_dynamic = body_a.physics().body_type() == BodyType::Dynamic;
                    let body_b_dynamic = body_b.physics().body_type() == BodyType::Dynamic;
                    let limit_policy = self.collision_system.config().ccd_limit_policy;
                    if budget_exhausted && limit_policy == CcdLimitPolicy::AbortStep {
                        return Err(CollisionError::CcdLimitExceeded);
                    }
                    if budget_exhausted || already_clipped {
                        self.clip_ccd_contact_bodies(
                            [
                                (body_a_id, prediction_a, body_a_dynamic),
                                (body_b_id, prediction_b, body_b_dynamic),
                            ],
                            impact_elapsed,
                            dt,
                        )?;
                        let has_prior_impact = self
                            .collision_system
                            .context
                            .interaction_indices
                            .as_ref()
                            .and_then(|indices| indices.get(&pair))
                            .and_then(|index| {
                                self.collision_system.context.interactions.get(*index)
                            })
                            .is_some_and(|interaction| interaction.impact);
                        if !has_prior_impact {
                            upsert_interaction(
                                &mut self.collision_system.context.interactions,
                                &mut self.collision_system.context.interaction_indices,
                                CollisionInteraction {
                                    pair,
                                    sensor: false,
                                    transient: false,
                                    contact: Some(ContactInfo {
                                        point: contact_position,
                                        normal: hit.normal,
                                        penetration: (-separation).max(0.0),
                                        normal_impulse: 0.0,
                                    }),
                                    impact: false,
                                },
                            )?;
                        }
                        #[cfg(feature = "bench-internals")]
                        self.collision_system.record_ccd_clipped_contact();
                        self.mark_ccd_pair_processed(pair, impact_elapsed, time_tolerance)?;
                        elapsed = impact_elapsed;
                        if dt - impact_elapsed > f64::EPSILON * dt.max(1.0)
                            && self.resweep_ccd_after_impact(
                                island_index,
                                [
                                    (body_a_id, prediction_a, body_a_dynamic),
                                    (body_b_id, prediction_b, body_b_dynamic),
                                ],
                                impact_elapsed,
                                dt,
                                time_tolerance,
                                &mut force_single_island,
                            )?
                        {
                            continue 'passes;
                        }
                        continue;
                    }
                    let material = self.combined_solver_material(material_a, material_b)?;
                    let (solved_a, solved_b, solved_manifold) = self
                        .collision_system
                        .solve_ccd_contact(body_a, state_a, body_b, state_b, manifold, material)?;
                    let mut normal_impulse = 0.0;
                    let mut impulse_delta = 0.0;
                    for point in solved_manifold.points() {
                        normal_impulse += point.normal_impulse;
                        impulse_delta += point.normal_impulse_delta;
                    }
                    if !normal_impulse.is_finite() || !impulse_delta.is_finite() {
                        return Err(CollisionError::ArithmeticOverflow);
                    }
                    if impulse_delta <= 0.0 {
                        return Err(CollisionError::NonConvergent);
                    }
                    let solved_point = solved_manifold
                        .points()
                        .first()
                        .ok_or(CollisionError::InvalidContact)?;
                    let point_a = solved_a.transform().transform_point(solved_point.local_a);
                    let point_b = solved_b.transform().transform_point(solved_point.local_b);
                    let point = point_a
                        .checked_add(point_b)
                        .and_then(|sum| sum.checked_mul_scalar(0.5))
                        .ok_or(CollisionError::ArithmeticOverflow)?;
                    #[cfg(feature = "bench-internals")]
                    let island_commit_started = Instant::now();
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        CollisionInteraction {
                            pair,
                            sensor: false,
                            transient: false,
                            contact: Some(ContactInfo {
                                point,
                                normal: solved_point.normal,
                                penetration: solved_point.penetration,
                                normal_impulse,
                            }),
                            impact: true,
                        },
                    )?;
                    #[cfg(feature = "bench-internals")]
                    {
                        self.collision_system.record_ccd_impact();
                        self.collision_system
                            .record_ccd_contact_resolution(contact_resolution_started.elapsed());
                    }
                    let remaining_after_impact = (dt - impact_elapsed).max(0.0);
                    for (body, prediction, solved) in [
                        (body_a, prediction_a, solved_a),
                        (body_b, prediction_b, solved_b),
                    ] {
                        if body.physics().body_type() != BodyType::Dynamic {
                            continue;
                        }
                        self.collision_system.context.ccd_states[prediction] = solved;
                        self.collision_system.context.ccd_elapsed[prediction] = impact_elapsed;
                        self.collision_system.context.predicted_states[prediction].1 =
                            if remaining_after_impact <= f64::EPSILON * dt.max(1.0) {
                                solved
                            } else {
                                let mut body = body;
                                body.set_state_preserving_forces(solved);
                                predict(&body, remaining_after_impact)
                                    .map_err(|_| CollisionError::ArithmeticOverflow)?
                            };
                    }
                    if body_a.physics().body_type() == BodyType::Dynamic
                        && body_a.ccd_mode() == CcdMode::Continuous
                    {
                        self.collision_system.context.ccd_impact_counts[prediction_a] += 1;
                    }
                    if body_b.physics().body_type() == BodyType::Dynamic
                        && body_b.ccd_mode() == CcdMode::Continuous
                    {
                        self.collision_system.context.ccd_impact_counts[prediction_b] += 1;
                    }
                    #[cfg(feature = "bench-internals")]
                    self.collision_system
                        .record_ccd_island_commit(island_commit_started.elapsed());
                    self.mark_ccd_pair_processed(pair, impact_elapsed, time_tolerance)?;
                    elapsed = impact_elapsed;
                    if remaining_after_impact > f64::EPSILON * dt.max(1.0)
                        && self.resweep_ccd_after_impact(
                            island_index,
                            [
                                (body_a_id, prediction_a, body_a_dynamic),
                                (body_b_id, prediction_b, body_b_dynamic),
                            ],
                            impact_elapsed,
                            dt,
                            time_tolerance,
                            &mut force_single_island,
                        )?
                    {
                        continue 'passes;
                    }
                }
                self.collision_system
                    .mark_ccd_island_completed(island_index)?;
                island_index += 1;
            }
            break;
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_island_execute(ccd_execute_started.elapsed());
        #[cfg(feature = "bench-internals")]
        {
            self.collision_system
                .record_ccd_final_candidate_count(self.collision_system.candidate_count());
            let impact_count = self
                .collision_system
                .context
                .interactions
                .iter()
                .filter(|interaction| interaction.impact)
                .count();
            self.collision_system
                .record_ccd_final_impact_count(impact_count);
        }
        Ok(())
    }

    fn resolve_single_candidate_ccd_islands_parallel(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
        time_tolerance: f64,
        force_single_island: &mut bool,
    ) -> Result<(), CollisionError> {
        const MIN_PARALLEL_ISLANDS: usize = 4;

        let island_count = self.collision_system.ccd_island_count();
        if worker_count <= 1 || island_count < MIN_PARALLEL_ISLANDS {
            return Ok(());
        }

        {
            let results = &mut self.collision_system.context.ccd_island_results;
            if results.len() < island_count {
                results
                    .try_reserve(island_count - results.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                results.resize_with(island_count, CcdIslandResult::default);
            }
            for result in results.iter_mut().take(island_count) {
                result.clear();
            }
        }
        let mut eligible_count = 0;
        let mut total_cost = 0_usize;
        for island in 0..island_count {
            let candidates = self
                .collision_system
                .ccd_island_candidates(island)
                .ok_or(CollisionError::InvalidContact)?;
            let cost = if candidates.len() == 1 {
                eligible_count += 1;
                self.ccd_candidate_work_cost(candidates[0])?
            } else {
                0
            };
            total_cost = total_cost.saturating_add(cost);
            self.collision_system.context.ccd_island_results[island].work_cost = cost;
        }
        if eligible_count < MIN_PARALLEL_ISLANDS || total_cost < 256 {
            return Ok(());
        }

        let active_workers = worker_count.min(eligible_count);
        self.collision_system
            .context
            .ccd_worker_chunks
            .try_reserve(active_workers)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let mut chunk_sizes = std::mem::take(&mut self.collision_system.context.ccd_worker_chunks);
        chunk_sizes.clear();
        let mut start = 0;
        let mut remaining_cost = total_cost;
        for worker in 0..active_workers {
            let workers_left = active_workers - worker;
            let target = remaining_cost.div_ceil(workers_left).max(1);
            let mut chunk_cost = 0;
            let mut end = start;
            while end < island_count
                && (chunk_cost < target || chunk_cost == 0)
                && island_count - end > workers_left - 1
            {
                chunk_cost = chunk_cost.saturating_add(
                    self.collision_system.context.ccd_island_results[end].work_cost,
                );
                end += 1;
            }
            if worker + 1 == active_workers {
                end = island_count;
                chunk_cost = remaining_cost;
            }
            if end == start {
                return Err(CollisionError::InvalidContact);
            }
            chunk_sizes.push(end - start);
            start = end;
            remaining_cost = remaining_cost.saturating_sub(chunk_cost);
        }

        let mut results = std::mem::take(&mut self.collision_system.context.ccd_island_results);
        let shared_world = &*self;
        let computed = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            executor.for_each_chunk_layout_mut(
                &mut results[..island_count],
                &chunk_sizes,
                |chunk_start, chunk| {
                    let mut narrow_phase = NarrowPhase::new();
                    for (offset, result) in chunk.iter_mut().enumerate() {
                        let island = chunk_start + offset;
                        let cost = result.work_cost;
                        if cost == 0 {
                            continue;
                        }
                        result.clear();
                        result.work_cost = cost;
                        if let Err(error) = shared_world.prepare_single_candidate_ccd_island(
                            island,
                            dt,
                            &mut narrow_phase,
                            result,
                        ) {
                            result.worker_error = Some(error);
                            break;
                        }
                    }
                },
            )
        }));
        self.collision_system.context.ccd_island_results = results;
        self.collision_system.context.ccd_worker_chunks = chunk_sizes;
        if !matches!(computed, Ok(Ok(()))) {
            return Err(CollisionError::WorkerFailed);
        }
        if let Some(error) = self
            .collision_system
            .context
            .ccd_island_results
            .iter()
            .take(island_count)
            .find_map(|result| result.worker_error)
        {
            return Err(error);
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_parallel_islands(eligible_count, active_workers);

        let mut layout_changed = false;
        for island in 0..island_count {
            let weight = self.collision_system.context.ccd_island_results[island].work_cost;
            if weight == 0 {
                continue;
            }
            let candidate_count = self.collision_system.context.ccd_island_results[island]
                .candidates
                .len();
            for index in 0..candidate_count {
                let candidate =
                    self.collision_system.context.ccd_island_results[island].candidates[index];
                let previous =
                    self.collision_system.context.ccd_candidate_active[candidate.candidate];
                self.collision_system.context.ccd_toi_cache[candidate.candidate] = candidate.toi;
                self.collision_system
                    .set_ccd_candidate_active(candidate.candidate, candidate.toi.hit.is_some())?;
                layout_changed |= previous != candidate.toi.hit.is_some();
                if let Some(interaction) = candidate.interaction {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        interaction,
                    )?;
                }
                #[cfg(feature = "bench-internals")]
                if candidate.evaluated_now {
                    self.collision_system
                        .record_ccd_toi_evaluation(candidate.pair);
                }
                #[cfg(not(feature = "bench-internals"))]
                let _ = candidate.evaluated_now;
            }
        }
        if layout_changed {
            self.collision_system.build_ccd_islands(false)?;
        }

        for island in 0..island_count {
            let weight = self.collision_system.context.ccd_island_results[island].work_cost;
            if weight == 0 || !self.collision_system.context.ccd_island_results[island].handled {
                continue;
            }
            let result = &self.collision_system.context.ccd_island_results[island];
            for body in &result.bodies {
                if let Some(completed) = self
                    .collision_system
                    .context
                    .ccd_completed_bodies
                    .get_mut(body.prediction)
                {
                    *completed = true;
                }
            }
            let Some(impact) = result.impact else {
                continue;
            };

            #[cfg(feature = "bench-internals")]
            let commit_started = Instant::now();
            upsert_interaction(
                &mut self.collision_system.context.interactions,
                &mut self.collision_system.context.interaction_indices,
                impact.interaction,
            )?;
            for ((body_id, prediction, dynamic), solved) in [
                (impact.bodies[0], impact.solved_a),
                (impact.bodies[1], impact.solved_b),
            ] {
                if !dynamic {
                    continue;
                }
                self.collision_system.context.ccd_states[prediction] = solved;
                self.collision_system.context.ccd_elapsed[prediction] = impact.impact_elapsed;
                let remaining_after_impact = (dt - impact.impact_elapsed).max(0.0);
                self.collision_system.context.predicted_states[prediction].1 =
                    if remaining_after_impact <= f64::EPSILON * dt.max(1.0) {
                        solved
                    } else {
                        let mut body = self
                            .bodies
                            .get(body_id.dense)
                            .filter(|_| body_id.world == self.world_id)
                            .map(|entry| entry.body)
                            .ok_or(CollisionError::InvalidContact)?;
                        body.set_state_preserving_forces(solved);
                        predict(&body, remaining_after_impact)
                            .map_err(|_| CollisionError::ArithmeticOverflow)?
                    };
            }
            for (body_id, prediction, _) in impact.bodies {
                let body = self
                    .bodies
                    .get(body_id.dense)
                    .filter(|_| body_id.world == self.world_id)
                    .map(|entry| entry.body)
                    .ok_or(CollisionError::InvalidContact)?;
                if body.physics().body_type() == BodyType::Dynamic
                    && body.ccd_mode() == CcdMode::Continuous
                {
                    self.collision_system.context.ccd_impact_counts[prediction] += 1;
                }
            }
            self.mark_ccd_pair_processed(impact.pair, impact.impact_elapsed, time_tolerance)?;
            #[cfg(feature = "bench-internals")]
            {
                self.collision_system.record_ccd_impact();
                self.collision_system
                    .record_ccd_contact_resolution(impact.solve_time);
                self.collision_system
                    .record_ccd_island_commit(commit_started.elapsed());
            }

            if dt - impact.impact_elapsed <= f64::EPSILON * dt.max(1.0) {
                continue;
            }
            let island_index = impact
                .bodies
                .iter()
                .filter(|body| body.2)
                .find_map(|body| self.collision_system.ccd_body_island(body.1))
                .ok_or(CollisionError::InvalidContact)?;
            let previous_candidate_count = self.collision_system.ccd_candidate_count();
            if self.resweep_ccd_after_impact(
                island_index,
                impact.bodies,
                impact.impact_elapsed,
                dt,
                time_tolerance,
                force_single_island,
            )? {
                return Ok(());
            }
            let current_island = impact
                .bodies
                .iter()
                .filter(|body| body.2)
                .find_map(|body| self.collision_system.ccd_body_island(body.1));
            if current_island.is_some_and(|current| {
                self.collision_system
                    .ccd_island_candidates(current)
                    .is_some_and(|candidates| {
                        self.collision_system.ccd_candidate_count() > previous_candidate_count
                            || candidates.len() > 1
                    })
            }) {
                let current_island = current_island.ok_or(CollisionError::InvalidContact)?;
                self.collision_system
                    .mark_ccd_island_incomplete(current_island)?;
            }
        }
        Ok(())
    }

    fn ccd_candidate_work_cost(&self, candidate: usize) -> Result<usize, CollisionError> {
        let pair = self
            .collision_system
            .ccd_candidate_pair(candidate)
            .ok_or(CollisionError::InvalidContact)?;
        let collider_a = self
            .colliders
            .get(pair.a.dense)
            .filter(|_| pair.a.world == self.world_id)
            .ok_or(CollisionError::InvalidContact)?;
        let collider_b = self
            .colliders
            .get(pair.b.dense)
            .filter(|_| pair.b.world == self.world_id)
            .ok_or(CollisionError::InvalidContact)?;
        let cost = match (collider_a.collider.shape(), collider_b.collider.shape()) {
            (Shape::Sphere(_), Shape::Sphere(_)) => 1,
            (Shape::Sphere(_), Shape::Cuboid(_)) | (Shape::Cuboid(_), Shape::Sphere(_)) => 2,
            (Shape::TriangleMesh(_), _) | (_, Shape::TriangleMesh(_)) => 16,
            (Shape::HeightField(_), _) | (_, Shape::HeightField(_)) => 12,
            (Shape::LevelSet(_), _) | (_, Shape::LevelSet(_)) => 12,
            _ => 8,
        };
        Ok(cost)
    }

    fn prepare_single_candidate_ccd_island(
        &self,
        island: usize,
        dt: f64,
        narrow_phase: &mut NarrowPhase,
        result: &mut CcdIslandResult,
    ) -> Result<(), CollisionError> {
        self.snapshot_ccd_island_bodies(island, result)?;
        let candidates = self
            .collision_system
            .ccd_island_candidates(island)
            .ok_or(CollisionError::InvalidContact)?;
        let [candidate] = candidates else {
            return Ok(());
        };
        let pair = self
            .collision_system
            .ccd_candidate_pair(*candidate)
            .ok_or(CollisionError::InvalidContact)?;
        let cached = self
            .collision_system
            .context
            .ccd_toi_cache
            .get(*candidate)
            .copied()
            .ok_or(CollisionError::InvalidContact)?;
        let evaluated_now = !cached.evaluated;
        let (toi, interaction) = if !evaluated_now {
            (cached, None)
        } else {
            self.evaluate_ccd_candidate(pair, 0.0, dt, narrow_phase)?
        };
        result
            .candidates
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        result.candidates.push(CcdIslandCandidateResult {
            candidate: *candidate,
            pair,
            toi,
            interaction,
            evaluated_now,
        });
        if let Some(interaction) = interaction
            && interaction.sensor
        {
            result.handled = true;
            return Ok(());
        }
        let Some(hit) = toi.hit else {
            result.handled = true;
            return Ok(());
        };

        let (body_a_id, body_b_id) = {
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            (collider_a.body, collider_b.body)
        };
        let prediction_a = Self::predicted_body_index(
            &self.collision_system.context.body_indices,
            self.world_id,
            body_a_id,
        )?;
        let prediction_b = Self::predicted_body_index(
            &self.collision_system.context.body_indices,
            self.world_id,
            body_b_id,
        )?;
        let (body_a, body_b) = (
            self.bodies
                .get(body_a_id.dense)
                .filter(|_| body_a_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?,
            self.bodies
                .get(body_b_id.dense)
                .filter(|_| body_b_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?,
        );
        let exhausted = [(body_a, prediction_a), (body_b, prediction_b)]
            .into_iter()
            .any(|(body, prediction)| {
                body.physics().body_type() == BodyType::Dynamic
                    && body.ccd_mode() == CcdMode::Continuous
                    && self.collision_system.context.ccd_impact_counts[prediction]
                        >= self.collision_system.config().max_ccd_impacts
            });
        if exhausted
            || self.collision_system.context.ccd_clipped_bodies[prediction_a]
            || self.collision_system.context.ccd_clipped_bodies[prediction_b]
        {
            return Ok(());
        }

        let impact_elapsed = toi.impact_time;
        let state_a = self.body_state_at_ccd(body_a_id, prediction_a, impact_elapsed, dt)?;
        let state_b = self.body_state_at_ccd(body_b_id, prediction_b, impact_elapsed, dt)?;
        let separation = hit
            .point_b
            .checked_sub(hit.point_a)
            .and_then(|delta| delta.checked_dot(hit.normal))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let contact = ContactPoint::from_world(
            &state_a.transform(),
            &state_b.transform(),
            hit.point_a,
            hit.point_b,
            hit.normal,
            (-separation).max(0.0),
            0,
        )?;
        let contact_position = hit
            .point_a
            .checked_add(hit.point_b)
            .and_then(|point| point.checked_mul_scalar(0.5))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let mut manifold = ContactManifold::new(pair);
        manifold.insert(contact, contact_position)?;
        let (mode_a, mode_b, material_a, material_b) = {
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .ok_or(CollisionError::InvalidContact)?;
            (
                collider_a.collider.mode(),
                collider_b.collider.mode(),
                collider_a.collider.material(),
                collider_b.collider.material(),
            )
        };
        if mode_a == crate::collider::CollisionMode::Sensor
            || mode_b == crate::collider::CollisionMode::Sensor
        {
            return Err(CollisionError::InvalidContact);
        }
        let material = self.combined_solver_material(material_a, material_b)?;
        #[cfg(feature = "bench-internals")]
        let solve_started = Instant::now();
        let (solved_a, solved_b, solved_manifold) =
            CollisionSystem::solve_ccd_contact_with_workspace(
                &mut result.solver,
                body_a,
                state_a,
                body_b,
                state_b,
                manifold,
                material,
                self.collision_system.config(),
            )?;
        #[cfg(feature = "bench-internals")]
        let solve_time = solve_started.elapsed();
        let mut normal_impulse = 0.0;
        let mut impulse_delta = 0.0;
        for point in solved_manifold.points() {
            normal_impulse += point.normal_impulse;
            impulse_delta += point.normal_impulse_delta;
        }
        if !normal_impulse.is_finite() || !impulse_delta.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if impulse_delta <= 0.0 {
            return Err(CollisionError::NonConvergent);
        }
        let solved_point = solved_manifold
            .points()
            .first()
            .ok_or(CollisionError::InvalidContact)?;
        let point_a = solved_a.transform().transform_point(solved_point.local_a);
        let point_b = solved_b.transform().transform_point(solved_point.local_b);
        let point = point_a
            .checked_add(point_b)
            .and_then(|sum| sum.checked_mul_scalar(0.5))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        result.impact = Some(CcdIslandImpactResult {
            pair,
            bodies: [
                (
                    body_a_id,
                    prediction_a,
                    body_a.physics().body_type() == BodyType::Dynamic,
                ),
                (
                    body_b_id,
                    prediction_b,
                    body_b.physics().body_type() == BodyType::Dynamic,
                ),
            ],
            impact_elapsed,
            solved_a,
            solved_b,
            interaction: CollisionInteraction {
                pair,
                sensor: false,
                transient: false,
                contact: Some(ContactInfo {
                    point,
                    normal: solved_point.normal,
                    penetration: solved_point.penetration,
                    normal_impulse,
                }),
                impact: true,
            },
            #[cfg(feature = "bench-internals")]
            solve_time,
        });
        result.handled = true;
        Ok(())
    }

    fn precompute_ccd_island(
        &self,
        island: usize,
        dt: f64,
        narrow_phase: &mut NarrowPhase,
        result: &mut CcdIslandResult,
    ) -> Result<(), CollisionError> {
        self.snapshot_ccd_island_bodies(island, result)?;
        let candidates = self
            .collision_system
            .ccd_island_candidates(island)
            .ok_or(CollisionError::InvalidContact)?;
        result
            .candidates
            .try_reserve(candidates.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for candidate_index in candidates.iter().copied() {
            let pair = self
                .collision_system
                .ccd_candidate_pair(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let (toi, interaction) = self.evaluate_ccd_candidate(pair, 0.0, dt, narrow_phase)?;
            result.candidates.push(CcdIslandCandidateResult {
                candidate: candidate_index,
                pair,
                toi,
                interaction,
                evaluated_now: true,
            });
        }
        Ok(())
    }

    fn snapshot_ccd_island_bodies(
        &self,
        island: usize,
        result: &mut CcdIslandResult,
    ) -> Result<(), CollisionError> {
        let bodies = self
            .collision_system
            .ccd_island_bodies(island)
            .ok_or(CollisionError::InvalidContact)?;
        result
            .bodies
            .try_reserve(bodies.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for prediction in bodies.iter().copied() {
            let context = &self.collision_system.context;
            result.bodies.push(CcdIslandBodyResult {
                prediction,
                ccd_state: *context
                    .ccd_states
                    .get(prediction)
                    .ok_or(CollisionError::InvalidContact)?,
                predicted_state: context
                    .predicted_states
                    .get(prediction)
                    .map(|(_, state)| *state)
                    .ok_or(CollisionError::InvalidContact)?,
                elapsed: *context
                    .ccd_elapsed
                    .get(prediction)
                    .ok_or(CollisionError::InvalidContact)?,
                impact_count: *context
                    .ccd_impact_counts
                    .get(prediction)
                    .ok_or(CollisionError::InvalidContact)?,
                clipped: context
                    .ccd_clipped_bodies
                    .get(prediction)
                    .copied()
                    .ok_or(CollisionError::InvalidContact)?,
            });
        }
        Ok(())
    }

    fn mark_ccd_pair_processed(
        &mut self,
        pair: CollisionPair,
        impact_elapsed: f64,
        time_tolerance: f64,
    ) -> Result<(), CollisionError> {
        let context = &mut self.collision_system.context;
        if impact_elapsed > context.ccd_processed_time + time_tolerance {
            context.ccd_processed_pairs.clear();
            context.ccd_processed_time = impact_elapsed;
        }
        if context.ccd_processed_pairs.contains(&pair) {
            return Err(CollisionError::NonConvergent);
        }
        context
            .ccd_processed_pairs
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context.ccd_processed_pairs.push(pair);
        Ok(())
    }

    fn resweep_ccd_after_impact(
        &mut self,
        island_index: usize,
        bodies: [(BodyId, usize, bool); 2],
        impact_elapsed: f64,
        dt: f64,
        time_tolerance: f64,
        force_single_island: &mut bool,
    ) -> Result<bool, CollisionError> {
        self.collision_system.invalidate_ccd_candidates(
            bodies[0].1,
            bodies[0].2,
            bodies[1].1,
            bodies[1].2,
        )?;
        let invalidated_count = self.collision_system.ccd_resweep_candidate_count();
        let new_ccd_candidates =
            self.resweep_ccd_bodies(island_index, bodies, impact_elapsed, dt, invalidated_count)?;
        let reevaluate_count = self.collision_system.ccd_resweep_candidate_count();
        if reevaluate_count == 0 {
            return Ok(false);
        }

        self.collision_system.sort_ccd_resweep_candidates();
        let reevaluate_count = self.collision_system.ccd_resweep_candidate_count();
        let mut narrow_phase = NarrowPhase::new();
        let mut layout_changed = false;
        for index in 0..reevaluate_count {
            let candidate_index = self
                .collision_system
                .ccd_resweep_candidate(index)
                .ok_or(CollisionError::InvalidContact)?;
            let candidate_pair = self
                .collision_system
                .ccd_candidate_pair(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let (candidate_a, candidate_b) = self
                .collision_system
                .ccd_candidate_body_pair(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let candidate_is_cross_island = self.collision_system.context.ccd_body_types
                [candidate_a]
                == BodyType::Dynamic
                && self.collision_system.context.ccd_body_types[candidate_b] == BodyType::Dynamic
                && self.collision_system.ccd_body_island(candidate_a)
                    != self.collision_system.ccd_body_island(candidate_b);
            if candidate_is_cross_island {
                self.replay_ccd_component(candidate_index, dt, force_single_island)?;
                return Ok(true);
            }
            let candidate_elapsed = impact_elapsed
                .max(self.collision_system.context.ccd_elapsed[candidate_a])
                .max(self.collision_system.context.ccd_elapsed[candidate_b]);
            if impact_elapsed <= self.collision_system.context.ccd_processed_time + time_tolerance
                && self
                    .collision_system
                    .context
                    .ccd_processed_pairs
                    .contains(&candidate_pair)
            {
                self.collision_system.context.ccd_toi_cache[candidate_index] = CcdCandidateToi {
                    evaluated: true,
                    ..Default::default()
                };
                continue;
            }
            remove_transient_sensor_interaction(
                &mut self.collision_system.context.interactions,
                &mut self.collision_system.context.interaction_indices,
                candidate_pair,
            )?;
            let (toi, interaction) = self.evaluate_ccd_candidate(
                candidate_pair,
                candidate_elapsed,
                dt,
                &mut narrow_phase,
            )?;
            #[cfg(feature = "bench-internals")]
            self.collision_system.record_ccd_local_recalculation(1);
            self.collision_system.context.ccd_toi_cache[candidate_index] = toi;
            if toi.hit.is_some() {
                layout_changed |=
                    !self.collision_system.context.ccd_candidate_active[candidate_index];
                self.collision_system
                    .set_ccd_candidate_active(candidate_index, true)?;
            }
            if let Some(interaction) = interaction {
                upsert_interaction(
                    &mut self.collision_system.context.interactions,
                    &mut self.collision_system.context.interaction_indices,
                    interaction,
                )?;
            }
            #[cfg(feature = "bench-internals")]
            self.collision_system
                .record_ccd_toi_evaluation(candidate_pair);
        }
        if new_ccd_candidates > 0 || layout_changed {
            self.collision_system
                .build_ccd_islands(*force_single_island)?;
        }
        Ok(false)
    }

    fn replay_ccd_component(
        &mut self,
        bridge_candidate: usize,
        dt: f64,
        force_single_island: &mut bool,
    ) -> Result<(), CollisionError> {
        let (body_a, _body_b) = self
            .collision_system
            .ccd_candidate_body_pair(bridge_candidate)
            .ok_or(CollisionError::InvalidContact)?;
        #[cfg(feature = "bench-internals")]
        let prior_island_a = self.collision_system.ccd_body_island(body_a);
        #[cfg(feature = "bench-internals")]
        let prior_island_b = self.collision_system.ccd_body_island(_body_b);
        if !self
            .collision_system
            .context
            .ccd_deferred_candidates
            .contains(&bridge_candidate)
        {
            self.collision_system
                .context
                .ccd_deferred_candidates
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.collision_system
                .context
                .ccd_deferred_candidates
                .push(bridge_candidate);
        }
        self.collision_system
            .set_ccd_candidate_active(bridge_candidate, true)?;
        self.collision_system.build_ccd_islands(false)?;

        let island = self
            .collision_system
            .ccd_body_island(body_a)
            .ok_or(CollisionError::InvalidContact)?;
        self.collision_system.context.ccd_replay_bodies.clear();
        self.collision_system
            .append_ccd_island_bodies_to_replay(island)?;

        let prediction_count = self.collision_system.context.predicted_states.len();
        for replay_pass in 0..=prediction_count {
            let replay_count = self.collision_system.context.ccd_replay_bodies.len();
            for index in 0..replay_count {
                let prediction = self.collision_system.context.ccd_replay_bodies[index];
                let body_id = self
                    .collision_system
                    .context
                    .predicted_states
                    .get(prediction)
                    .map(|(id, _)| *id)
                    .ok_or(CollisionError::InvalidContact)?;
                let body = self
                    .bodies
                    .get(body_id.dense)
                    .filter(|_| body_id.world == self.world_id)
                    .map(|entry| entry.body)
                    .ok_or(CollisionError::InvalidContact)?;
                let end_state =
                    predict(&body, dt).map_err(|_| CollisionError::ArithmeticOverflow)?;
                let context = &mut self.collision_system.context;
                context.predicted_states[prediction].1 = end_state;
                context.ccd_states[prediction] = *body.state();
                context.ccd_elapsed[prediction] = 0.0;
                context.ccd_impact_counts[prediction] = 0;
                context.ccd_clipped_bodies[prediction] = false;
                context.ccd_completed_bodies[prediction] = false;
            }

            for index in 0..replay_count {
                let prediction = self.collision_system.context.ccd_replay_bodies[index];
                let body_id = self
                    .collision_system
                    .context
                    .predicted_states
                    .get(prediction)
                    .map(|(id, _)| *id)
                    .ok_or(CollisionError::InvalidContact)?;
                self.collision_system
                    .invalidate_ccd_candidates(prediction, true, prediction, false)?;
                let invalidated = self.collision_system.ccd_resweep_candidate_count();
                self.resweep_ccd_bodies(
                    island,
                    [(body_id, prediction, true), (body_id, prediction, false)],
                    0.0,
                    dt,
                    invalidated,
                )?;
            }
            self.collision_system.build_ccd_islands(false)?;
            self.collision_system.collect_ccd_replay_candidates()?;
            let candidate_count = self.collision_system.ccd_resweep_candidate_count();
            let context = &mut self.collision_system.context;
            context.ccd_replay_pairs.clear();
            context
                .ccd_replay_pairs
                .try_reserve(candidate_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            for index in 0..candidate_count {
                let candidate = *context
                    .ccd_resweep_candidates
                    .get(index)
                    .ok_or(CollisionError::InvalidContact)?;
                let pair = context
                    .ccd_candidate_pairs
                    .get(candidate)
                    .copied()
                    .ok_or(CollisionError::InvalidContact)?;
                context.ccd_replay_pairs.push(pair);
                *context
                    .ccd_toi_cache
                    .get_mut(candidate)
                    .ok_or(CollisionError::InvalidContact)? = CcdCandidateToi::default();
                *context
                    .ccd_candidate_active
                    .get_mut(candidate)
                    .ok_or(CollisionError::InvalidContact)? = false;
            }
            context.ccd_replay_pairs.sort_unstable();
            context.ccd_replay_pairs.dedup();
            remove_interactions_for_pairs(
                &mut context.interactions,
                &mut context.interaction_indices,
                &context.ccd_replay_pairs,
            )?;
            context.ccd_processed_pairs.clear();
            context.ccd_processed_time = 0.0;

            let mut narrow_phase = NarrowPhase::new();
            let mut expanded = false;
            for index in 0..candidate_count {
                let candidate = *self
                    .collision_system
                    .context
                    .ccd_resweep_candidates
                    .get(index)
                    .ok_or(CollisionError::InvalidContact)?;
                let pair = self
                    .collision_system
                    .ccd_candidate_pair(candidate)
                    .ok_or(CollisionError::InvalidContact)?;
                let (candidate_a, candidate_b) = self
                    .collision_system
                    .ccd_candidate_body_pair(candidate)
                    .ok_or(CollisionError::InvalidContact)?;
                let outside = self.ccd_external_dynamic_body(candidate_a, candidate_b);
                let saved = if let Some(prediction) = outside {
                    Some(self.reset_ccd_body_for_replay(prediction, dt)?)
                } else {
                    None
                };
                let (toi, interaction) =
                    self.evaluate_ccd_candidate(pair, 0.0, dt, &mut narrow_phase)?;
                let deferred_bridge = toi.hit.is_none()
                    && self
                        .collision_system
                        .context
                        .ccd_deferred_candidates
                        .contains(&candidate);
                if toi.hit.is_some() || deferred_bridge {
                    self.collision_system
                        .set_ccd_candidate_active(candidate, true)?;
                    if let Some(prediction) = outside {
                        let bodies = &mut self.collision_system.context.ccd_replay_bodies;
                        if !bodies.contains(&prediction) {
                            bodies
                                .try_reserve(1)
                                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                            bodies.push(prediction);
                            expanded = true;
                        }
                    }
                } else {
                    self.collision_system
                        .set_ccd_candidate_active(candidate, false)?;
                    if let Some(saved) = saved {
                        self.restore_ccd_body_after_replay(saved)?;
                    }
                }
                self.collision_system.context.ccd_toi_cache[candidate] = if deferred_bridge {
                    CcdCandidateToi::default()
                } else {
                    toi
                };
                if let Some(interaction) = interaction {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        interaction,
                    )?;
                }
                #[cfg(feature = "bench-internals")]
                {
                    self.collision_system.record_ccd_local_recalculation(1);
                    self.collision_system.record_ccd_toi_evaluation(pair);
                }
                if expanded {
                    break;
                }
            }
            self.collision_system.build_ccd_islands(false)?;

            let replay_count = self.collision_system.context.ccd_replay_bodies.len();
            for index in 0..replay_count {
                let prediction = self.collision_system.context.ccd_replay_bodies[index];
                if let Some(island) = self.collision_system.ccd_body_island(prediction) {
                    let member_count = self
                        .collision_system
                        .ccd_island_bodies(island)
                        .ok_or(CollisionError::InvalidContact)?
                        .len();
                    for member_index in 0..member_count {
                        let member = self
                            .collision_system
                            .ccd_island_bodies(island)
                            .and_then(|members| members.get(member_index))
                            .copied()
                            .ok_or(CollisionError::InvalidContact)?;
                        if !self
                            .collision_system
                            .context
                            .ccd_replay_bodies
                            .contains(&member)
                        {
                            self.collision_system
                                .context
                                .ccd_replay_bodies
                                .try_reserve(1)
                                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                            self.collision_system.context.ccd_replay_bodies.push(member);
                            expanded = true;
                        }
                    }
                }
            }
            if !expanded {
                #[cfg(feature = "bench-internals")]
                self.collision_system.record_ccd_local_replay(
                    usize::from(prior_island_a.is_some()) + usize::from(prior_island_b.is_some()),
                );
                return Ok(());
            }
            self.collision_system
                .context
                .ccd_replay_bodies
                .sort_unstable();
            if replay_pass == prediction_count {
                *force_single_island = true;
                #[cfg(feature = "bench-internals")]
                self.collision_system.record_ccd_global_fallback(
                    self.collision_system.ccd_candidate_count(),
                    self.collision_system.ccd_island_count(),
                );
                self.reset_ccd_pass(dt, true)?;
                return Ok(());
            }
        }
        Err(CollisionError::NonConvergent)
    }

    fn ccd_external_dynamic_body(&self, body_a: usize, body_b: usize) -> Option<usize> {
        let bodies = &self.collision_system.context.ccd_replay_bodies;
        let selected_a = bodies.contains(&body_a);
        let selected_b = bodies.contains(&body_b);
        match (
            self.collision_system.context.ccd_body_types[body_a] == BodyType::Dynamic,
            self.collision_system.context.ccd_body_types[body_b] == BodyType::Dynamic,
            selected_a,
            selected_b,
        ) {
            (true, true, true, false) => Some(body_b),
            (true, true, false, true) => Some(body_a),
            _ => None,
        }
    }

    fn reset_ccd_body_for_replay(
        &mut self,
        prediction: usize,
        dt: f64,
    ) -> Result<CcdBodyReplaySnapshot, CollisionError> {
        let context = &self.collision_system.context;
        let body_id = context
            .predicted_states
            .get(prediction)
            .map(|(id, _)| *id)
            .ok_or(CollisionError::InvalidContact)?;
        let saved = CcdBodyReplaySnapshot {
            prediction,
            ccd_state: context.ccd_states[prediction],
            predicted_state: context.predicted_states[prediction].1,
            elapsed: context.ccd_elapsed[prediction],
            impact_count: context.ccd_impact_counts[prediction],
            clipped: context.ccd_clipped_bodies[prediction],
            completed: context.ccd_completed_bodies[prediction],
        };
        let body = self
            .bodies
            .get(body_id.dense)
            .filter(|_| body_id.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let end_state = predict(&body, dt).map_err(|_| CollisionError::ArithmeticOverflow)?;
        let context = &mut self.collision_system.context;
        context.ccd_states[prediction] = *body.state();
        context.predicted_states[prediction].1 = end_state;
        context.ccd_elapsed[prediction] = 0.0;
        context.ccd_impact_counts[prediction] = 0;
        context.ccd_clipped_bodies[prediction] = false;
        context.ccd_completed_bodies[prediction] = false;
        Ok(saved)
    }

    fn restore_ccd_body_after_replay(
        &mut self,
        saved: CcdBodyReplaySnapshot,
    ) -> Result<(), CollisionError> {
        let CcdBodyReplaySnapshot {
            prediction,
            ccd_state,
            predicted_state,
            elapsed,
            impact_count,
            clipped,
            completed,
        } = saved;
        let context = &mut self.collision_system.context;
        *context
            .ccd_states
            .get_mut(prediction)
            .ok_or(CollisionError::InvalidContact)? = ccd_state;
        context
            .predicted_states
            .get_mut(prediction)
            .ok_or(CollisionError::InvalidContact)?
            .1 = predicted_state;
        context.ccd_elapsed[prediction] = elapsed;
        context.ccd_impact_counts[prediction] = impact_count;
        context.ccd_clipped_bodies[prediction] = clipped;
        context.ccd_completed_bodies[prediction] = completed;
        Ok(())
    }

    fn precompute_ccd_tois_parallel(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
    ) -> Result<(), CollisionError> {
        let island_count = self.collision_system.ccd_island_count();
        if worker_count <= 1 || island_count <= 1 {
            return Ok(());
        }
        // Cuboid GJK sweeps first repay task-dispatch overhead at 256 candidates.
        const MIN_PARALLEL_EXPENSIVE_TOIS: usize = 256;
        if self.expensive_ccd_candidate_count()? < MIN_PARALLEL_EXPENSIVE_TOIS {
            return Ok(());
        }
        let worker_count = worker_count.min(island_count);
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        let chunk_size = island_count.div_ceil(worker_count);
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_worker_tasks_started(island_count.div_ceil(chunk_size));

        {
            let results = &mut self.collision_system.context.ccd_island_results;
            if results.len() < island_count {
                results
                    .try_reserve(island_count - results.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                results.resize_with(island_count, CcdIslandResult::default);
            }
            for result in results.iter_mut().take(island_count) {
                result.clear();
            }
        }
        let mut results = std::mem::take(&mut self.collision_system.context.ccd_island_results);
        debug_assert!(results.len() >= island_count);
        let shared_world = &*self;
        let evaluated = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            executor.for_each_chunk_mut(
                &mut results[..island_count],
                chunk_size,
                |worker_index, island_results| {
                    let start = worker_index * chunk_size;
                    let mut narrow_phase = NarrowPhase::new();
                    for (island, result) in
                        (start..start + island_results.len()).zip(island_results)
                    {
                        result.clear();
                        if let Err(error) = shared_world.precompute_ccd_island(
                            island,
                            dt,
                            &mut narrow_phase,
                            result,
                        ) {
                            result.worker_error = Some(error);
                            break;
                        }
                    }
                },
            )
        }));
        self.collision_system.context.ccd_island_results = results;
        if !matches!(evaluated, Ok(Ok(()))) {
            return Err(CollisionError::WorkerFailed);
        }
        if let Some(error) = self
            .collision_system
            .context
            .ccd_island_results
            .iter()
            .take(island_count)
            .find_map(|result| result.worker_error)
        {
            return Err(error);
        }

        for island in 0..island_count {
            let body_count = self.collision_system.context.ccd_island_results[island]
                .bodies
                .len();
            for body_index in 0..body_count {
                let body =
                    self.collision_system.context.ccd_island_results[island].bodies[body_index];
                let context = &mut self.collision_system.context;
                context.ccd_states[body.prediction] = body.ccd_state;
                context.predicted_states[body.prediction].1 = body.predicted_state;
                context.ccd_elapsed[body.prediction] = body.elapsed;
                context.ccd_impact_counts[body.prediction] = body.impact_count;
                context.ccd_clipped_bodies[body.prediction] = body.clipped;
            }
            let candidate_count = self.collision_system.context.ccd_island_results[island]
                .candidates
                .len();
            for candidate_index in 0..candidate_count {
                let result = self.collision_system.context.ccd_island_results[island].candidates
                    [candidate_index];
                *self
                    .collision_system
                    .context
                    .ccd_toi_cache
                    .get_mut(result.candidate)
                    .ok_or(CollisionError::InvalidContact)? = result.toi;
                self.collision_system
                    .set_ccd_candidate_active(result.candidate, result.toi.hit.is_some())?;
                if let Some(interaction) = result.interaction {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        interaction,
                    )?;
                }
                #[cfg(feature = "bench-internals")]
                self.collision_system.record_ccd_toi_evaluation(result.pair);
                #[cfg(not(feature = "bench-internals"))]
                let _ = result.pair;
            }
            debug_assert!(
                self.collision_system.context.ccd_island_results[island]
                    .dependencies
                    .is_empty()
            );
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_toi_search(started.elapsed());
        Ok(())
    }

    fn expensive_ccd_candidate_count(&self) -> Result<usize, CollisionError> {
        let mut count = 0_usize;
        for candidate_index in 0..self.collision_system.ccd_candidate_count() {
            let pair = self
                .collision_system
                .ccd_candidate_pair(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let mut rotational_motion = [false; 2];
            for (index, body_id) in [collider_a.body, collider_b.body].into_iter().enumerate() {
                let body = self
                    .bodies
                    .get(body_id.dense)
                    .filter(|_| body_id.world == self.world_id)
                    .map(|entry| entry.body)
                    .ok_or(CollisionError::InvalidContact)?;
                let prediction = Self::predicted_body_index(
                    &self.collision_system.context.body_indices,
                    self.world_id,
                    body_id,
                )?;
                let end_state = self
                    .collision_system
                    .context
                    .predicted_states
                    .get(prediction)
                    .map(|(_, state)| *state)
                    .ok_or(CollisionError::InvalidContact)?;
                rotational_motion[index] = body.state().angular_velocity() != Vec3::ZERO
                    || end_state.angular_velocity() != Vec3::ZERO
                    || body.state().transform().rotation() != end_state.transform().rotation();
            }
            let analytic_pair = match (collider_a.collider.shape(), collider_b.collider.shape()) {
                (Shape::Sphere(_), Shape::Sphere(_)) => true,
                (Shape::Sphere(_), Shape::Cuboid(_)) => !rotational_motion[1],
                (Shape::Cuboid(_), Shape::Sphere(_)) => !rotational_motion[0],
                _ => false,
            };
            if !analytic_pair {
                count = count.saturating_add(1);
            }
        }
        Ok(count)
    }

    fn evaluate_ccd_candidate(
        &self,
        pair: CollisionPair,
        elapsed: f64,
        dt: f64,
        narrow_phase: &mut NarrowPhase,
    ) -> Result<(CcdCandidateToi, Option<CollisionInteraction>), CollisionError> {
        let (collider_a, collider_b) = {
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            (collider_a, collider_b)
        };
        let body_a = self
            .bodies
            .get(collider_a.body.dense)
            .filter(|_| collider_a.body.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let body_b = self
            .bodies
            .get(collider_b.body.dense)
            .filter(|_| collider_b.body.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let mut cache = CcdCandidateToi {
            evaluated: true,
            ..Default::default()
        };
        if ![body_a, body_b].into_iter().any(|body| {
            body.physics().body_type() == BodyType::Dynamic
                && body.ccd_mode() == CcdMode::Continuous
        }) {
            return Ok((cache, None));
        }

        let prediction_a = Self::predicted_body_index(
            &self.collision_system.context.body_indices,
            self.world_id,
            collider_a.body,
        )?;
        let prediction_b = Self::predicted_body_index(
            &self.collision_system.context.body_indices,
            self.world_id,
            collider_b.body,
        )?;
        let state_a = self.body_state_at_ccd(collider_a.body, prediction_a, elapsed, dt)?;
        let state_b = self.body_state_at_ccd(collider_b.body, prediction_b, elapsed, dt)?;
        let end_state_a = self
            .collision_system
            .context
            .predicted_states
            .get(prediction_a)
            .map(|(_, state)| *state)
            .ok_or(CollisionError::InvalidContact)?;
        let end_state_b = self
            .collision_system
            .context
            .predicted_states
            .get(prediction_b)
            .map(|(_, state)| *state)
            .ok_or(CollisionError::InvalidContact)?;
        let start_transform_a = state_a
            .transform()
            .compose(&collider_a.collider.local_transform())
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let start_transform_b = state_b
            .transform()
            .compose(&collider_b.collider.local_transform())
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let end_transform_a = end_state_a
            .transform()
            .compose(&collider_a.collider.local_transform())
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let end_transform_b = end_state_b
            .transform()
            .compose(&collider_b.collider.local_transform())
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let trajectory_a = CcdTrajectory::new(
            body_a,
            self.collision_system.context.ccd_states[prediction_a],
            self.collision_system.context.ccd_elapsed[prediction_a],
            elapsed,
            dt,
            collider_a.collider.local_transform(),
            self.collision_system.context.ccd_clipped_bodies[prediction_a],
        );
        let trajectory_b = CcdTrajectory::new(
            body_b,
            self.collision_system.context.ccd_states[prediction_b],
            self.collision_system.context.ccd_elapsed[prediction_b],
            elapsed,
            dt,
            collider_b.collider.local_transform(),
            self.collision_system.context.ccd_clipped_bodies[prediction_b],
        );
        let (translation_bound_a, rotation_bound_a, linear_translation_a) =
            trajectory_a.path_bounds(end_state_a)?;
        let (translation_bound_b, rotation_bound_b, linear_translation_b) =
            trajectory_b.path_bounds(end_state_b)?;
        let motion_a = CcdMotion::trajectory(
            start_transform_a,
            end_transform_a,
            translation_bound_a,
            linear_translation_a,
            rotation_bound_a,
            trajectory_a,
        )?;
        let motion_b = CcdMotion::trajectory(
            start_transform_b,
            end_transform_b,
            translation_bound_b,
            linear_translation_b,
            rotation_bound_b,
            trajectory_b,
        )?;
        let sensor = collider_a.collider.mode() == crate::collider::CollisionMode::Sensor
            || collider_b.collider.mode() == crate::collider::CollisionMode::Sensor;
        let hit = narrow_phase.time_of_impact(
            collider_a.collider.shape(),
            &motion_a,
            collider_a.collider.query_bvh(),
            collider_a.collider.query_vertices(),
            collider_b.collider.shape(),
            &motion_b,
            collider_b.collider.query_bvh(),
            collider_b.collider.query_vertices(),
            sensor,
        )?;
        let Some(hit) = hit else {
            return Ok((cache, None));
        };
        if !hit.toi.is_finite() || !(0.0..=1.0).contains(&hit.toi) {
            return Err(CollisionError::InvalidContact);
        }
        if sensor {
            return Ok((
                cache,
                Some(CollisionInteraction {
                    pair,
                    sensor: true,
                    transient: true,
                    contact: None,
                    impact: false,
                }),
            ));
        }
        if hit.toi <= f64::EPSILON * 128.0 {
            let velocity_a = body_point_velocity(body_a, state_a, hit.point_a)?;
            let velocity_b = body_point_velocity(body_b, state_b, hit.point_b)?;
            let normal_speed = velocity_b
                .checked_sub(velocity_a)
                .and_then(|velocity| velocity.checked_dot(hit.normal))
                .ok_or(CollisionError::ArithmeticOverflow)?;
            if normal_speed >= 0.0 {
                return Ok((cache, None));
            }
        }
        cache.impact_time = elapsed + (dt - elapsed) * hit.toi;
        cache.hit = Some(hit);
        Ok((cache, None))
    }

    fn rebuild_ccd_candidate_bodies(&mut self) -> Result<(), CollisionError> {
        let candidate_count = self.collision_system.candidate_count();
        let context = &mut self.collision_system.context;
        context.ccd_candidate_bodies.clear();
        context.ccd_candidate_pairs.clear();
        context
            .ccd_candidate_bodies
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context
            .ccd_candidate_pairs
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let candidate_set = context
            .ccd_candidate_pair_set
            .get_or_insert_with(std::collections::HashSet::new);
        candidate_set.clear();
        candidate_set
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for candidate_index in 0..candidate_count {
            let pair = self
                .collision_system
                .candidate(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let body_a = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_a.body,
            )?;
            let body_b = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_b.body,
            )?;
            let context = &mut self.collision_system.context;
            context.ccd_candidate_pairs.push(pair);
            context.ccd_candidate_bodies.push((body_a, body_b));
            context
                .ccd_candidate_pair_set
                .as_mut()
                .ok_or(CollisionError::InvalidContact)?
                .insert(pair);
        }
        let context = &mut self.collision_system.context;
        let cache = &mut context.ccd_toi_cache;
        cache.clear();
        cache
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        cache.resize(
            candidate_count,
            super::collision::CcdCandidateToi::default(),
        );
        context.ccd_candidate_active.clear();
        context
            .ccd_candidate_active
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context.ccd_candidate_active.resize(candidate_count, true);
        context.ccd_resweep_candidates.clear();
        context.ccd_query_results.clear();
        Ok(())
    }

    fn resweep_ccd_bodies(
        &mut self,
        _island_index: usize,
        bodies: [(BodyId, usize, bool); 2],
        elapsed: f64,
        dt: f64,
        _invalidated_count: usize,
    ) -> Result<usize, CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        let mut new_ccd_candidates = 0;
        #[cfg(feature = "bench-internals")]
        let mut colliders_visited = 0_usize;
        #[cfg(feature = "bench-internals")]
        let mut proxies_updated = 0_usize;
        #[cfg(feature = "bench-internals")]
        let mut found_interisland_dependency = false;
        let world_id = self.world_id;
        let (body_pool, colliders, collision_system, unbounded_query_colliders) = (
            &self.bodies,
            &self.colliders,
            &mut self.collision_system,
            &self.unbounded_query_colliders,
        );

        for (body_id, prediction_index, changed) in bodies {
            if !changed {
                continue;
            }
            let body_entry = body_pool
                .get(body_id.dense)
                .filter(|_| body_id.world == world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let body = body_entry.body;
            let end_state = collision_system
                .context
                .predicted_states
                .get(prediction_index)
                .map(|(_, state)| *state)
                .ok_or(CollisionError::InvalidContact)?;

            let colliders_of_body = collider_entries_of(world_id, body_pool, colliders, body_id)
                .ok_or(CollisionError::InvalidContact)?;
            for (collider_id, collider_body, collider) in colliders_of_body {
                if collider_id.world != world_id || collider_body != body_id {
                    return Err(CollisionError::InvalidContact);
                }
                let proxy = collision_system.ccd_proxy(collider_id)?;
                if proxy.body != body_id {
                    return Err(CollisionError::InvalidContact);
                }
                #[cfg(feature = "bench-internals")]
                {
                    colliders_visited = colliders_visited.saturating_add(1);
                    proxies_updated = proxies_updated.saturating_add(1);
                }
                let bounds = {
                    let trajectory = CcdTrajectory::new(
                        body,
                        collision_system.context.ccd_states[prediction_index],
                        collision_system.context.ccd_elapsed[prediction_index],
                        elapsed,
                        dt,
                        collider.local_transform(),
                        collision_system.context.ccd_clipped_bodies[prediction_index],
                    );
                    swept_bounds_for_trajectory(collider, &trajectory, end_state)?
                };
                let (query_count, _unbounded_colliders_visited) = collision_system
                    .prepare_ccd_resweep_proxy(collider_id, bounds, unbounded_query_colliders)?;
                #[cfg(feature = "bench-internals")]
                {
                    colliders_visited =
                        colliders_visited.saturating_add(_unbounded_colliders_visited);
                }
                for query_index in 0..query_count {
                    let other_id = collision_system
                        .ccd_query_result(query_index)
                        .ok_or(CollisionError::InvalidContact)?;
                    let other_proxy = collision_system.ccd_proxy(other_id)?;
                    if bounds.is_some_and(|bounds| {
                        other_proxy
                            .bounds
                            .is_some_and(|other_bounds| !bounds.intersects(&other_bounds))
                    }) {
                        continue;
                    }
                    if !collision_system.ccd_pair_allowed(collider_id, other_id)? {
                        continue;
                    }
                    let pair = CollisionPair::new(collider_id, other_id)
                        .ok_or(CollisionError::InvalidContact)?;
                    let other_prediction = Self::predicted_body_index(
                        &collision_system.context.body_indices,
                        world_id,
                        other_proxy.body,
                    )?;
                    let (body_a, body_b) = if pair.a == collider_id {
                        (prediction_index, other_prediction)
                    } else {
                        (other_prediction, prediction_index)
                    };
                    let (added, is_ccd) =
                        collision_system.add_ccd_candidate(pair, body_a, body_b)?;
                    if added && is_ccd {
                        new_ccd_candidates += 1;
                        #[cfg(feature = "bench-internals")]
                        if collision_system.context.ccd_body_types[other_prediction]
                            == BodyType::Dynamic
                            && collision_system.ccd_body_island(other_prediction)
                                != Some(_island_index)
                        {
                            found_interisland_dependency = true;
                        }
                    }
                }
            }
        }

        #[cfg(feature = "bench-internals")]
        collision_system.record_ccd_resweep(
            started.elapsed(),
            _invalidated_count,
            colliders_visited,
            proxies_updated,
            found_interisland_dependency,
        );
        Ok(new_ccd_candidates)
    }

    fn reset_ccd_pass(&mut self, dt: f64, force_single_island: bool) -> Result<(), CollisionError> {
        let prediction_count = self.collision_system.context.predicted_states.len();
        for prediction in 0..prediction_count {
            let body_id = self.collision_system.context.predicted_states[prediction].0;
            let body = self
                .bodies
                .get(body_id.dense)
                .filter(|_| body_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?;
            let end_state = predict(&body, dt).map_err(|_| CollisionError::ArithmeticOverflow)?;
            let context = &mut self.collision_system.context;
            context.predicted_states[prediction].1 = end_state;
            context.ccd_states[prediction] = *body.state();
            context.ccd_elapsed[prediction] = 0.0;
            context.ccd_impact_counts[prediction] = 0;
            context.ccd_clipped_bodies[prediction] = false;
            context.ccd_completed_bodies[prediction] = false;
        }
        {
            let context = &mut self.collision_system.context;
            context.ccd_processed_pairs.clear();
            context.ccd_processed_time = 0.0;
            context.ccd_toi_cache.fill(Default::default());
            context.ccd_candidate_active.fill(true);
            context.interactions.clear();
            context.colliders.clear();
            context.spatial_proxies.clear();
            if let Some(indices) = &mut context.interaction_indices {
                indices.clear();
            }
        }

        self.stage_spatial_proxies(dt)?;
        self.collision_system.synchronize_spatial_index()?;
        self.collision_system.build_ccd_islands(force_single_island)
    }

    fn body_state_at_ccd(
        &self,
        id: BodyId,
        prediction_index: usize,
        elapsed: f64,
        dt: f64,
    ) -> Result<BodyState, CollisionError> {
        let body = self
            .bodies
            .get(id.dense)
            .filter(|_| id.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let state_start = *self
            .collision_system
            .context
            .ccd_states
            .get(prediction_index)
            .ok_or(CollisionError::InvalidContact)?;
        let start_time = *self
            .collision_system
            .context
            .ccd_elapsed
            .get(prediction_index)
            .ok_or(CollisionError::InvalidContact)?;
        if elapsed < start_time {
            return Err(CollisionError::InvalidContact);
        }
        if elapsed == start_time {
            return Ok(state_start);
        }
        if self
            .collision_system
            .context
            .ccd_clipped_bodies
            .get(prediction_index)
            .copied()
            .ok_or(CollisionError::InvalidContact)?
        {
            return Ok(state_start);
        }
        let duration = elapsed - start_time;
        if dt - start_time <= 0.0 {
            return Err(CollisionError::InvalidContact);
        }
        predict_from_state(&body, state_start, duration)
            .map_err(|_| CollisionError::ArithmeticOverflow)
    }

    fn clip_ccd_contact_bodies(
        &mut self,
        bodies: [(BodyId, usize, bool); 2],
        elapsed: f64,
        dt: f64,
    ) -> Result<(), CollisionError> {
        let first_prediction = bodies[0].1;
        for (body_index, (body_id, prediction, changed)) in bodies.into_iter().enumerate() {
            if !changed
                || self.collision_system.context.ccd_body_types[prediction] != BodyType::Dynamic
                || (body_index > 0 && first_prediction == prediction)
            {
                continue;
            }
            let state = self.body_state_at_ccd(body_id, prediction, elapsed, dt)?;
            let mut body = self
                .bodies
                .get(body_id.dense)
                .filter(|_| body_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?;
            body.set_state_preserving_forces(state);
            body.set_linear_velocity(Vec3::ZERO)
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            body.set_angular_velocity(Vec3::ZERO)
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            let clipped_state = *body.state();
            let context = &mut self.collision_system.context;
            #[cfg(feature = "bench-internals")]
            let count_clipped_body = !context.ccd_clipped_metrics[prediction];
            context.ccd_states[prediction] = clipped_state;
            context.ccd_elapsed[prediction] = elapsed;
            context.ccd_clipped_bodies[prediction] = true;
            context.predicted_states[prediction].1 = clipped_state;
            #[cfg(feature = "bench-internals")]
            {
                context.ccd_clipped_metrics[prediction] = true;
            }
            #[cfg(feature = "bench-internals")]
            if count_clipped_body {
                self.collision_system.record_ccd_clipped_body();
            }
        }
        Ok(())
    }

    fn required_body_index_slots(context: &StepContext) -> Result<usize, CollisionError> {
        context
            .predicted_states
            .iter()
            .try_fold(0_usize, |required, (body_id, _)| {
                let slot =
                    usize::try_from(body_id.index()).map_err(|_| CollisionError::GeometryBounds)?;
                let end = slot.checked_add(1).ok_or(CollisionError::GeometryBounds)?;
                Ok(required.max(end))
            })
    }

    fn rebuild_body_indices(context: &mut StepContext) -> Result<(), CollisionError> {
        if context.body_indices_ready {
            return Ok(());
        }
        let required_slots = Self::required_body_index_slots(context)?;
        if context.body_indices.len() < required_slots {
            context
                .body_indices
                .try_reserve(required_slots - context.body_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context.body_indices.resize(required_slots, None);
        }
        context
            .body_indices
            .iter_mut()
            .for_each(|slot| *slot = None);
        for (index, (body_id, _)) in context.predicted_states.iter().copied().enumerate() {
            let slot =
                usize::try_from(body_id.index()).map_err(|_| CollisionError::GeometryBounds)?;
            *context
                .body_indices
                .get_mut(slot)
                .ok_or(CollisionError::InvalidContact)? = Some((body_id, index));
        }
        context.body_indices_ready = true;
        Ok(())
    }

    fn prepare_solver_body_indices(context: &mut StepContext) -> Result<(), CollisionError> {
        let required_slots = Self::required_body_index_slots(context)?;
        if context.solver_body_indices.len() < required_slots {
            context
                .solver_body_indices
                .try_reserve(required_slots - context.solver_body_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context.solver_body_indices.resize(required_slots, None);
        }
        context
            .solver_body_indices
            .iter_mut()
            .for_each(|slot| *slot = None);
        context.body_types.clear();
        context.solver_bodies.clear();
        context.solver_body_predictions.clear();
        Ok(())
    }

    fn process_collision_candidates(
        &mut self,
        worker_count: usize,
        executor: &impl StepExecutor,
    ) -> Result<(), CollisionError> {
        let candidate_count = self.collision_system.candidate_count();
        if candidate_count == 0 {
            self.collision_system
                .solve_contacts(worker_count, executor)?;
            self.stage_collision_events()?;
            return Ok(());
        }
        let context = &mut self.collision_system.context;
        Self::rebuild_body_indices(context)?;
        Self::prepare_solver_body_indices(context)?;

        self.collision_system.context.manifolds.clear();
        self.collision_system
            .context
            .manifolds
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.collision_system
            .context
            .interactions
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for candidate_index in 0..candidate_count {
            let pair = self
                .collision_system
                .candidate(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let body_a_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_a.body,
            )?;
            let body_b_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_b.body,
            )?;
            let body_a = self
                .collision_system
                .context
                .predicted_states
                .get(body_a_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let body_b = self
                .collision_system
                .context
                .predicted_states
                .get(body_b_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let transform_a = body_a
                .compose(&collider_a.collider.local_transform())
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            let transform_b = body_b
                .compose(&collider_b.collider.local_transform())
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            #[cfg(feature = "bench-internals")]
            let started = Instant::now();
            let manifold = self.collision_system.detect(
                pair,
                collider_a.collider.shape(),
                &transform_a,
                &body_a,
                collider_a.collider.mode(),
                collider_a.collider.query_bvh(),
                collider_a.collider.query_vertices(),
                collider_b.collider.shape(),
                &transform_b,
                &body_b,
                collider_b.collider.mode(),
                collider_b.collider.query_bvh(),
                collider_b.collider.query_vertices(),
            )?;
            #[cfg(feature = "bench-internals")]
            self.collision_system.record_narrow_phase(
                started.elapsed(),
                manifold.as_ref().map_or(0, ContactManifold::len),
            );
            if let Some(manifold) = manifold {
                let sensor = collider_a.collider.mode() == crate::collider::CollisionMode::Sensor
                    || collider_b.collider.mode() == crate::collider::CollisionMode::Sensor;
                if sensor {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        CollisionInteraction {
                            pair,
                            sensor: true,
                            transient: false,
                            contact: None,
                            impact: false,
                        },
                    )?;
                } else {
                    self.collision_system.context.manifolds.push(manifold);
                }
            }
        }

        self.collision_system.context.solver_manifolds.clear();
        self.collision_system
            .context
            .solver_manifolds
            .try_reserve(self.collision_system.context.manifolds.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for manifold_index in 0..self.collision_system.context.manifolds.len() {
            let manifold = self.collision_system.context.manifolds[manifold_index];
            let (body_id_a, body_id_b, mode_a, mode_b, material_a, material_b) = {
                let collider_a = self
                    .colliders
                    .get(manifold.pair.a.dense)
                    .filter(|_| manifold.pair.a.world == self.world_id)
                    .ok_or(CollisionError::InvalidContact)?;
                let collider_b = self
                    .colliders
                    .get(manifold.pair.b.dense)
                    .filter(|_| manifold.pair.b.world == self.world_id)
                    .ok_or(CollisionError::InvalidContact)?;
                (
                    collider_a.body,
                    collider_b.body,
                    collider_a.collider.mode(),
                    collider_b.collider.mode(),
                    collider_a.collider.material(),
                    collider_b.collider.material(),
                )
            };
            let prediction_a = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                body_id_a,
            )?;
            let prediction_b = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                body_id_b,
            )?;
            let type_a = self
                .bodies
                .get(body_id_a.dense)
                .filter(|_| body_id_a.world == self.world_id)
                .map(|entry| entry.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            let type_b = self
                .bodies
                .get(body_id_b.dense)
                .filter(|_| body_id_b.world == self.world_id)
                .map(|entry| entry.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            if type_a != BodyType::Dynamic && type_b != BodyType::Dynamic {
                continue;
            }
            if mode_a == crate::collider::CollisionMode::Sensor
                || mode_b == crate::collider::CollisionMode::Sensor
            {
                return Err(CollisionError::InvalidContact);
            }
            let material = self.combined_solver_material(material_a, material_b)?;
            let body_a = self.solver_body_index(body_id_a, prediction_a)?;
            let body_b = self.solver_body_index(body_id_b, prediction_b)?;
            self.collision_system
                .context
                .solver_manifolds
                .push(SolverManifold {
                    body_a,
                    body_b,
                    manifold,
                    material,
                });
        }
        self.collision_system
            .solve_contacts(worker_count, executor)?;

        for manifold in self.collision_system.context.manifolds.iter().copied() {
            let collider_a = self
                .colliders
                .get(manifold.pair.a.dense)
                .filter(|_| manifold.pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(manifold.pair.b.dense)
                .filter(|_| manifold.pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let body_a_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_a.body,
            )?;
            let body_b_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_b.body,
            )?;
            let body_a = self
                .collision_system
                .context
                .predicted_states
                .get(body_a_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let body_b = self
                .collision_system
                .context
                .predicted_states
                .get(body_b_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let mut deepest = None;
            let mut total_impulse = 0.0;
            let mut impact = false;
            for point in manifold.points().iter().copied() {
                total_impulse += point.normal_impulse;
                if !total_impulse.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                impact |= point.normal_impulse_delta > 0.0;
                if deepest.is_none_or(|current: crate::collision::ContactPoint| {
                    point.penetration > current.penetration
                }) {
                    deepest = Some(point);
                }
            }
            let deepest = deepest.ok_or(CollisionError::InvalidContact)?;
            let point_a = body_a.transform_point(deepest.local_a);
            let point_b = body_b.transform_point(deepest.local_b);
            let point = point_a * 0.5 + point_b * 0.5;
            if !point.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            upsert_interaction(
                &mut self.collision_system.context.interactions,
                &mut self.collision_system.context.interaction_indices,
                CollisionInteraction {
                    pair: manifold.pair,
                    sensor: false,
                    transient: false,
                    contact: Some(ContactInfo {
                        point,
                        normal: deepest.normal,
                        penetration: deepest.penetration,
                        normal_impulse: total_impulse,
                    }),
                    impact,
                },
            )?;
        }
        self.stage_collision_events()?;
        Ok(())
    }

    fn predicted_body_index(
        body_indices: &[Option<(BodyId, usize)>],
        world_id: u64,
        id: BodyId,
    ) -> Result<usize, CollisionError> {
        if id.world != world_id {
            return Err(CollisionError::InvalidContact);
        }
        let slot = usize::try_from(id.index()).map_err(|_| CollisionError::InvalidContact)?;
        body_indices
            .get(slot)
            .and_then(|entry| *entry)
            .filter(|(current, _)| *current == id)
            .map(|(_, index)| index)
            .ok_or(CollisionError::InvalidContact)
    }

    fn solver_body_index(
        &mut self,
        body_id: BodyId,
        prediction_index: usize,
    ) -> Result<usize, CollisionError> {
        let slot = usize::try_from(body_id.index()).map_err(|_| CollisionError::InvalidContact)?;
        if let Some((current, index)) = self
            .collision_system
            .context
            .solver_body_indices
            .get(slot)
            .copied()
            .flatten()
            && current == body_id
        {
            return Ok(index);
        }
        let body = self
            .bodies
            .get(body_id.dense)
            .filter(|_| body_id.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let state = self
            .collision_system
            .context
            .predicted_states
            .get(prediction_index)
            .filter(|(current, _)| *current == body_id)
            .map(|(_, state)| *state)
            .ok_or(CollisionError::InvalidContact)?;
        let context = &mut self.collision_system.context;
        let index = context.solver_bodies.len();
        context
            .solver_bodies
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context
            .solver_body_predictions
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context
            .body_types
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        *context
            .solver_body_indices
            .get_mut(slot)
            .ok_or(CollisionError::InvalidContact)? = Some((body_id, index));
        context
            .solver_bodies
            .push(SolverBody::from_prediction(body, state));
        context.solver_body_predictions.push(prediction_index);
        context.body_types.push(body.physics().body_type());
        Ok(index)
    }

    fn combined_solver_material(
        &self,
        material_a: Option<MaterialId>,
        material_b: Option<MaterialId>,
    ) -> Result<SolverMaterial, CollisionError> {
        let fallback = PhysicsMaterial::default();
        let material_a = material_a
            .and_then(|id| self.materials.get(id))
            .copied()
            .unwrap_or(fallback);
        let material_b = material_b
            .and_then(|id| self.materials.get(id))
            .copied()
            .unwrap_or(fallback);
        let dynamic_friction = material_a.friction().sqrt() * material_b.friction().sqrt();
        let static_friction =
            material_a.static_friction().sqrt() * material_b.static_friction().sqrt();
        if !dynamic_friction.is_finite() || !static_friction.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(SolverMaterial {
            restitution: material_a.restitution().max(material_b.restitution()),
            static_friction,
            dynamic_friction,
        })
    }

    fn validate_collision_inputs(&self) -> Result<(), CollisionError> {
        self.collision_system.config().validate()?;
        for (_, entry) in self.colliders.iter() {
            if entry
                .collider
                .material()
                .is_some_and(|id| !self.materials.contains(id))
            {
                return Err(CollisionError::InvalidMaterial);
            }
            let body = self
                .bodies
                .get(entry.body.dense)
                .filter(|_| entry.body.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            if body.body.physics().body_type() == crate::rigid_body::BodyType::Dynamic
                && !entry.collider.shape().is_convex()
            {
                return Err(CollisionError::UnsupportedDynamicGeometry);
            }
        }
        Ok(())
    }

    /// Fills balanced, ordered body snapshots into reusable shard buffers.
    #[doc(hidden)]
    pub fn fill_step_shards(&self, shards: &mut [Vec<BodyStep>]) -> Result<(), StepError> {
        if shards.is_empty() {
            return Err(StepError::InvalidStepResults);
        }
        for shard in shards.iter_mut() {
            shard.clear();
        }

        let base_size = self.body_count() / shards.len();
        let remainder = self.body_count() % shards.len();
        for (index, shard) in shards.iter_mut().enumerate() {
            let required = base_size + usize::from(index < remainder);
            shard
                .try_reserve(required)
                .map_err(|_| StepError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        }
        let mut shard_index = 0;
        let mut shard_remaining = base_size + usize::from(remainder > 0);

        for (dense_id, entry) in self.bodies.iter() {
            while shard_remaining == 0 {
                shard_index += 1;
                shard_remaining = base_size + usize::from(shard_index < remainder);
            }
            shards[shard_index].push(BodyStep {
                body_id: BodyId {
                    world: self.world_id,
                    dense: dense_id,
                },
                source: entry.body,
                predicted: None,
                dt: None,
                error: None,
            });
            shard_remaining -= 1;
        }
        Ok(())
    }

    /// Validates all shard results before committing any body state.
    #[doc(hidden)]
    pub fn commit_step_shards(&mut self, shards: &[Vec<BodyStep>]) -> Result<(), StepError> {
        self.commit_step_shards_with_workers(shards, shards.len().max(1))
    }

    /// Validates shard results using the built-in sequential executor.
    ///
    /// Use [`Self::commit_step_shards_with_executor`] to select another executor.
    #[doc(hidden)]
    pub fn commit_step_shards_with_workers(
        &mut self,
        shards: &[Vec<BodyStep>],
        _worker_count: usize,
    ) -> Result<(), StepError> {
        self.commit_step_shards_with_executor(shards, &SequentialExecutor, 1)
    }

    /// Validates shard results and delegates independent collision work to an executor.
    #[doc(hidden)]
    pub fn commit_step_shards_with_executor<E: StepExecutor>(
        &mut self,
        shards: &[Vec<BodyStep>],
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        let mut steps = shards.iter().flatten();
        let mut step_dt = None;
        for (dense_id, entry) in self.bodies.iter() {
            let Some(step) = steps.next() else {
                return Err(StepError::InvalidStepResults);
            };
            if let Some(error) = step.error {
                return Err(error);
            }
            if step.body_id
                != (BodyId {
                    world: self.world_id,
                    dense: dense_id,
                })
                || step.source != entry.body
                || step.predicted.is_none()
                || step.dt.is_none_or(|dt| !dt.is_finite() || dt <= 0.0)
            {
                return Err(StepError::InvalidStepResults);
            }
            if step_dt.is_some_and(|dt| Some(dt) != step.dt) {
                return Err(StepError::InvalidStepResults);
            }
            step_dt = step.dt;
        }
        let dt = step_dt.unwrap_or(0.0);
        if steps.next().is_some() {
            return Err(StepError::InvalidStepResults);
        }

        self.validate_collision_inputs()
            .map_err(StepError::Collision)?;
        self.collision_system.begin_step();
        for step in shards.iter().flatten() {
            if let Some(state) = step.predicted {
                self.collision_system.stage_prediction(step.body_id, state);
            }
        }
        let worker_count = worker_count.max(1);
        if let Err(error) = self.prepare_collision_candidates(dt, worker_count, executor) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(worker_count, executor) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }

        #[cfg(feature = "bench-internals")]
        let commit_started = Instant::now();
        self.commit_predicted_states();
        self.collision_system.commit_step();
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_commit(commit_started.elapsed());
        Ok(())
    }

    pub fn add_collider(
        &mut self,
        body: BodyId,
        mut collider: Collider,
    ) -> Result<ColliderId, WorldError> {
        if body.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        if collider
            .material()
            .is_some_and(|material| !self.materials.contains(material))
        {
            return Err(WorldError::InvalidMaterialId);
        }
        let first = self
            .bodies
            .get(body.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .first_collider;
        if self.bodies.get(body.dense).is_some_and(|entry| {
            entry.body.physics().body_type() == crate::rigid_body::BodyType::Dynamic
                && !collider.shape().is_convex()
        }) {
            return Err(WorldError::Collision(
                CollisionError::UnsupportedDynamicGeometry,
            ));
        }
        if first.is_some_and(|id| self.colliders.get(id.dense).is_none()) {
            return Err(WorldError::InvalidColliderId);
        }
        collider.prepare_query_geometry()?;
        let has_finite_bounds = collider.local_bounds().ok().flatten().is_some();
        if !has_finite_bounds {
            self.unbounded_query_colliders
                .try_reserve(1)
                .map_err(|_| WorldError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        }

        let dense = self
            .colliders
            .insert(ColliderEntry {
                collider,
                body,
                prev: None,
                next: first,
            })
            .map_err(WorldError::from)?;
        let id = ColliderId {
            world: self.world_id,
            dense,
        };

        if let Some(first) = first
            && let Some(entry) = self.colliders.get_mut(first.dense)
        {
            entry.prev = Some(id);
        }
        if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.first_collider = Some(id);
        }
        if has_finite_bounds {
            self.update_query_index_for_collider(id);
        } else {
            self.unbounded_query_colliders.push(id);
        }
        self.collision_system.mark_spatial_dirty();
        Ok(id)
    }

    /// Updates the deformed local vertices for a skinned-mesh collider.
    ///
    /// The pose is fixed for each physics step. Changes between steps are
    /// discrete, and CCD does not interpolate unknown bone trajectories.
    pub fn set_skin_pose(
        &mut self,
        id: ColliderId,
        matrices: &[Affine3],
    ) -> Result<(), CollisionError> {
        if id.world != self.world_id {
            return Err(CollisionError::InvalidSkinPose);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(CollisionError::InvalidSkinPose)?;
        entry.collider.set_skin_pose(matrices)?;
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Ok(())
    }

    pub fn collider(&self, id: ColliderId) -> Option<&Collider> {
        if id.world != self.world_id {
            return None;
        }
        self.colliders.get(id.dense).map(|entry| &entry.collider)
    }

    pub fn collider_mut(&mut self, id: ColliderId) -> Option<&mut Collider> {
        if id.world != self.world_id {
            return None;
        }
        if self.colliders.get(id.dense).is_some() {
            self.collision_system.invalidate_collider_contacts(id);
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.colliders
            .get_mut(id.dense)
            .map(|entry| &mut entry.collider)
    }

    pub fn collider_body(&self, id: ColliderId) -> Option<BodyId> {
        if id.world != self.world_id {
            return None;
        }
        self.colliders.get(id.dense).map(|entry| entry.body)
    }

    pub fn remove_collider(&mut self, id: ColliderId) -> Option<Collider> {
        if id.world != self.world_id {
            return None;
        }
        let (body, prev, next) = {
            let entry = self.colliders.get(id.dense)?;
            (entry.body, entry.prev, entry.next)
        };
        let body_entry = self.bodies.get(body.dense)?;
        if prev.is_none() && body_entry.first_collider != Some(id) {
            return None;
        }
        if prev.is_some_and(|prev| {
            self.colliders
                .get(prev.dense)
                .is_none_or(|entry| entry.body != body || entry.next != Some(id))
        }) || next.is_some_and(|next| {
            self.colliders
                .get(next.dense)
                .is_none_or(|entry| entry.body != body || entry.prev != Some(id))
        }) {
            return None;
        }

        if !self.query_index_dirty {
            let remove_failed = self
                .query_index
                .as_mut()
                .is_some_and(|index| index.remove(id).is_err());
            if remove_failed {
                self.mark_query_index_dirty();
            }
        }
        self.unbounded_query_colliders
            .retain(|collider| *collider != id);

        let removed = self.colliders.remove(id.dense)?;
        if let Some(prev) = prev {
            if let Some(entry) = self.colliders.get_mut(prev.dense) {
                entry.next = next;
            }
        } else if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.first_collider = next;
        }
        if let Some(next) = next
            && let Some(entry) = self.colliders.get_mut(next.dense)
        {
            entry.prev = prev;
        }
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Some(removed.collider)
    }

    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    pub fn remove_body(&mut self, id: BodyId) -> Option<RigidBody> {
        if id.world != self.world_id {
            return None;
        }
        let mut next = self.bodies.get(id.dense)?.first_collider;
        while let Some(collider) = next {
            next = self.colliders.get(collider.dense)?.next;
            self.remove_collider(collider)?;
        }
        loop {
            let source = self.force_sources.iter().find_map(|(dense, source)| {
                (source.body == id).then_some(ForceSourceId {
                    world: self.world_id,
                    dense,
                })
            });
            let Some(source) = source else {
                break;
            };
            self.force_sources.remove(source.dense)?;
        }
        self.bodies.remove(id.dense).map(|entry| entry.body)
    }

    /// Iterates over bodies in dense storage order, which may change after removal.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &RigidBody)> {
        self.bodies.iter().map(|(dense, entry)| {
            (
                BodyId {
                    world: self.world_id,
                    dense,
                },
                &entry.body,
            )
        })
    }

    /// Mutably iterates over bodies in dense storage order, which may change after removal.
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = (BodyId, &mut RigidBody)> {
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.bodies.iter_mut().map(|(dense, entry)| {
            (
                BodyId {
                    world: self.world_id,
                    dense,
                },
                &mut entry.body,
            )
        })
    }

    /// Iterates over colliders in dense storage order, which may change after removal.
    pub fn colliders(&self) -> impl Iterator<Item = (ColliderId, BodyId, &Collider)> {
        self.colliders.iter().map(|(dense, entry)| {
            (
                ColliderId {
                    world: self.world_id,
                    dense,
                },
                entry.body,
                &entry.collider,
            )
        })
    }

    /// Iterates over the colliders attached to `body` in linked-list order.
    pub fn colliders_of(
        &self,
        body: BodyId,
    ) -> Option<impl Iterator<Item = (ColliderId, &Collider)> + '_> {
        collider_entries_of(self.world_id, &self.bodies, &self.colliders, body)
            .map(|entries| entries.map(|(id, _, collider)| (id, collider)))
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

fn validate_force_source_values(
    local_point: Vec3,
    local_force: Vec3,
) -> Result<(), ForceSourceError> {
    if !local_point.is_finite() || !local_force.is_finite() {
        return Err(ForceSourceError::NonFiniteValue);
    }
    Ok(())
}

fn body_point_velocity(
    mut body: RigidBody,
    state: BodyState,
    point: Vec3,
) -> Result<Vec3, CollisionError> {
    body.set_state_preserving_forces(state);
    let center = body
        .world_center_of_mass()
        .map_err(|_| CollisionError::ArithmeticOverflow)?
        .unwrap_or(state.transform().translation());
    let angular_velocity = state
        .angular_velocity()
        .checked_cross(
            point
                .checked_sub(center)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    state
        .linear_velocity()
        .checked_add(angular_velocity)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn collider_entries_of<'a>(
    world_id: u64,
    bodies: &DensePool<BodyEntry>,
    colliders: &'a DensePool<ColliderEntry>,
    body: BodyId,
) -> Option<impl Iterator<Item = (ColliderId, BodyId, &'a Collider)> + 'a> {
    if body.world != world_id {
        return None;
    }
    let mut next = bodies.get(body.dense)?.first_collider;
    Some(std::iter::from_fn(move || {
        let id = next?;
        let entry = colliders.get(id.dense)?;
        next = entry.next;
        Some((id, entry.body, &entry.collider))
    }))
}

fn update_query_index_for_body(
    index: &mut SpatialIndex,
    colliders: &DensePool<ColliderEntry>,
    world_id: u64,
    body_id: BodyId,
    body: &BodyEntry,
) -> Result<(), CollisionError> {
    let mut next = body.first_collider;
    while let Some(collider_id) = next {
        if collider_id.world != world_id {
            return Err(CollisionError::InvalidContact);
        }
        let entry = colliders
            .get(collider_id.dense)
            .filter(|entry| entry.body == body_id)
            .ok_or(CollisionError::InvalidContact)?;
        if let Some(bounds) = entry
            .collider
            .world_bounds(&body.body.state().transform())
            .map_err(|_| CollisionError::GeometryBounds)?
        {
            if !index.contains_bounds(collider_id, bounds)? {
                index.update(collider_id, expanded_query_bounds(bounds)?)?;
            }
        } else {
            index.remove(collider_id)?;
        }
        next = entry.next;
    }
    Ok(())
}

fn expanded_query_bounds(bounds: Aabb) -> Result<Aabb, CollisionError> {
    const MARGIN: f64 = 0.1;
    let margin = Vec3::splat(MARGIN);
    let min = bounds
        .min()
        .checked_sub(margin)
        .ok_or(CollisionError::GeometryBounds)?;
    let max = bounds
        .max()
        .checked_add(margin)
        .ok_or(CollisionError::GeometryBounds)?;
    Aabb::new(min, max).map_err(|_| CollisionError::GeometryBounds)
}

fn upsert_interaction(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    interaction: CollisionInteraction,
) -> Result<(), CollisionError> {
    let indices = indices.get_or_insert_with(HashMap::new);
    if let Some(&index) = indices.get(&interaction.pair) {
        let existing = interactions
            .get_mut(index)
            .ok_or(CollisionError::InvalidContact)?;
        merge_interaction(existing, interaction)?;
        return Ok(());
    }
    interactions
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    indices
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    let index = interactions.len();
    interactions.push(interaction);
    indices.insert(interaction.pair, index);
    Ok(())
}

fn remove_transient_sensor_interaction(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    pair: CollisionPair,
) -> Result<(), CollisionError> {
    let Some(index) = indices
        .as_ref()
        .and_then(|indices| indices.get(&pair).copied())
    else {
        return Ok(());
    };
    let interaction = interactions
        .get(index)
        .ok_or(CollisionError::InvalidContact)?;
    if !interaction.sensor || !interaction.transient {
        return Ok(());
    }
    interactions.remove(index);
    let indices = indices.as_mut().ok_or(CollisionError::InvalidContact)?;
    indices.clear();
    indices
        .try_reserve(interactions.len())
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    for (index, interaction) in interactions.iter().enumerate() {
        if indices.insert(interaction.pair, index).is_some() {
            return Err(CollisionError::InvalidContact);
        }
    }
    Ok(())
}

fn remove_interactions_for_pairs(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    pairs: &[CollisionPair],
) -> Result<(), CollisionError> {
    interactions.retain(|interaction| pairs.binary_search(&interaction.pair).is_err());
    let Some(indices) = indices else {
        return Ok(());
    };
    indices.clear();
    indices
        .try_reserve(interactions.len())
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    for (index, interaction) in interactions.iter().enumerate() {
        if indices.insert(interaction.pair, index).is_some() {
            return Err(CollisionError::InvalidContact);
        }
    }
    Ok(())
}

fn merge_interaction(
    existing: &mut CollisionInteraction,
    incoming: CollisionInteraction,
) -> Result<(), CollisionError> {
    if existing.sensor != incoming.sensor {
        return Err(CollisionError::InvalidContact);
    }
    existing.transient &= incoming.transient;
    if incoming.contact.is_some() {
        existing.contact = incoming.contact;
    }
    existing.impact |= incoming.impact;
    Ok(())
}

#[cfg(test)]
#[path = "world_tests.rs"]
mod tests;
