use kapitsa_math::{Affine3, Vec3};
use std::{
    collections::HashMap,
    sync::atomic::{AtomicU64, Ordering},
};

#[cfg(feature = "bench-internals")]
use std::time::Instant;

mod error;
mod id;

pub use error::{StepError, WorldError};
pub use id::{BodyId, ColliderId};

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
        CcdHit, CcdMotion, CollisionConfig, CollisionError, CollisionEvent, CollisionInteraction,
        CollisionPair, CollisionSystem, ContactInfo, ContactManifold, ContactPoint, OverlapHit,
        QueryError, QueryFilter, QueryScratch, QueryTarget, Ray, RayHit, ShapeHit, SolverBody,
        SolverManifold, SolverMaterial, StepContext, overlaps_shape_with_scratch,
        raycast_shape_with_scratch, spatial_index::SpatialIndex, sweep_shape_with_scratch,
    },
    containers::DensePool,
    material::{MaterialId, MaterialStorage, PhysicsMaterial},
    rigid_body::{
        BodyState, BodyType, CcdMode, RigidBody, predict, predict_from_state, trajectory_bounds,
    },
};

struct BodyEntry {
    body: RigidBody,
    first_collider: Option<ColliderId>,
}

struct ColliderEntry {
    collider: Collider,
    body: BodyId,
    prev: Option<ColliderId>,
    next: Option<ColliderId>,
}

#[derive(Clone, Copy)]
struct ColliderTrajectory {
    body: RigidBody,
    base_state: BodyState,
    base_time: f64,
    sample_start_time: f64,
    end_time: f64,
    local_transform: crate::transform::Transform,
}

impl ColliderTrajectory {
    fn transform_at(&self, amount: f64) -> Result<crate::transform::Transform, CollisionError> {
        let sample_time =
            self.sample_start_time + (self.end_time - self.sample_start_time) * amount;
        let duration = sample_time - self.base_time;
        let state = if duration == 0.0 {
            self.base_state
        } else {
            predict_from_state(&self.body, self.base_state, duration)
                .map_err(|_| CollisionError::ArithmeticOverflow)?
        };
        state
            .transform()
            .compose(&self.local_transform)
            .map_err(|_| CollisionError::ArithmeticOverflow)
    }

    fn path_bounds(&self, end: BodyState) -> Result<(f64, f64, bool), CollisionError> {
        let base_duration = self.end_time - self.base_time;
        if base_duration <= 0.0 {
            return Err(CollisionError::InvalidContact);
        }
        let bounds = trajectory_bounds(&self.body, self.base_state, end, base_duration)
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let linear_origin = self.has_linear_collider_origin(end);
        let segment_ratio = (self.end_time - self.sample_start_time) / base_duration;
        if !segment_ratio.is_finite() || segment_ratio < 0.0 || segment_ratio > 1.0 {
            return Err(CollisionError::InvalidContact);
        }
        let local_offset = self
            .local_transform
            .translation()
            .checked_length()
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let rotation = bounds.rotation * segment_ratio;
        let translation = (bounds.translation + local_offset * bounds.rotation) * segment_ratio;
        if !translation.is_finite() || !rotation.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok((translation, rotation, linear_origin))
    }

    fn has_linear_collider_origin(&self, end: BodyState) -> bool {
        if self.base_state.is_sleeping() || self.body.physics().body_type() == BodyType::Static {
            return true;
        }
        if self.base_state.linear_velocity() != end.linear_velocity() {
            return false;
        }
        if self.base_state.angular_velocity() == Vec3::ZERO && end.angular_velocity() == Vec3::ZERO
        {
            return true;
        }
        let center_of_mass = self
            .body
            .physics()
            .mass()
            .map_or(Vec3::ZERO, |mass| mass.center_of_mass());
        center_of_mass == Vec3::ZERO && self.local_transform.translation() == Vec3::ZERO
    }
}

fn swept_bounds_for_trajectory(
    collider: &Collider,
    trajectory: &ColliderTrajectory,
    end: BodyState,
) -> Result<Option<crate::aabb::Aabb>, CollisionError> {
    let local_bounds = collider.local_bounds()?;
    let start_transform = trajectory.transform_at(0.0)?;
    let end_transform = end
        .transform()
        .compose(&trajectory.local_transform)
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    if trajectory.has_linear_collider_origin(end) {
        let motion = CcdMotion::linear(start_transform, end_transform)?;
        return crate::collision::swept_bounds_motion(local_bounds, &motion);
    }
    let (translation_bound, rotation_bound, _) = trajectory.path_bounds(end)?;
    let sample = |amount| trajectory.transform_at(amount);
    let motion = CcdMotion::trajectory(
        start_transform,
        end_transform,
        translation_bound,
        false,
        rotation_bound,
        &sample,
    )?;
    crate::collision::swept_bounds_motion(local_bounds, &motion)
}

/// Owns rigid bodies and colliders and assigns IDs scoped to this world instance.
pub struct PhysicsWorld {
    world_id: u64,
    bodies: DensePool<BodyEntry>,
    colliders: DensePool<ColliderEntry>,
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
}

impl BodyStep {
    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    pub fn integrate(&mut self, dt: f64) -> Result<(), StepError> {
        self.predicted = None;
        self.dt = None;
        let predicted = predict(&self.source, dt).map_err(|cause| StepError::BodyIntegration {
            body_id: self.body_id,
            cause,
        })?;
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

    /// Advances every body by one fixed timestep, atomically committing the results.
    pub fn step(&mut self, dt: f64) -> Result<(), StepError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(StepError::InvalidDeltaTime);
        }
        self.validate_collision_inputs()
            .map_err(StepError::Collision)?;
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

        if let Err(error) = self.prepare_collision_candidates(dt) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(1) {
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
                    let trajectory = ColliderTrajectory {
                        body: body_entry.body,
                        base_state: *body_entry.body.state(),
                        base_time: 0.0,
                        sample_start_time: 0.0,
                        end_time: dt,
                        local_transform: collider_entry.collider.local_transform(),
                    };
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

    fn prepare_collision_candidates(&mut self, dt: f64) -> Result<(), CollisionError> {
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
        self.process_continuous_collisions(dt)?;
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

    fn process_continuous_collisions(&mut self, dt: f64) -> Result<(), CollisionError> {
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
            context.ccd_body_types.push(body.physics().body_type());
            context.ccd_continuous_bodies.push(
                body.physics().body_type() == BodyType::Dynamic
                    && body.ccd_mode() == CcdMode::Continuous,
            );
        }
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

        let time_tolerance = f64::EPSILON * dt.max(1.0) * 128.0;
        'passes: loop {
            let mut island_index = 0;
            let mut restart_pass = false;
            while island_index < self.collision_system.ccd_island_count() {
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
                        let trajectory_a = ColliderTrajectory {
                            body: body_a,
                            base_state: context.ccd_states[prediction_a],
                            base_time: context.ccd_elapsed[prediction_a],
                            sample_start_time: elapsed,
                            end_time: dt,
                            local_transform: local_transform_a,
                        };
                        let trajectory_b = ColliderTrajectory {
                            body: body_b,
                            base_state: context.ccd_states[prediction_b],
                            base_time: context.ccd_elapsed[prediction_b],
                            sample_start_time: elapsed,
                            end_time: dt,
                            local_transform: local_transform_b,
                        };
                        let (translation_bound_a, rotation_bound_a, linear_translation_a) =
                            trajectory_a.path_bounds(end_state_a)?;
                        let (translation_bound_b, rotation_bound_b, linear_translation_b) =
                            trajectory_b.path_bounds(end_state_b)?;
                        let sample_a = |amount| trajectory_a.transform_at(amount);
                        let sample_b = |amount| trajectory_b.transform_at(amount);
                        let motion_a = CcdMotion::trajectory(
                            start_transform_a,
                            end_transform_a,
                            translation_bound_a,
                            linear_translation_a,
                            rotation_bound_a,
                            &sample_a,
                        )?;
                        let motion_b = CcdMotion::trajectory(
                            start_transform_b,
                            end_transform_b,
                            translation_bound_b,
                            linear_translation_b,
                            rotation_bound_b,
                            &sample_b,
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
                    for (body, prediction) in [(body_a, prediction_a), (body_b, prediction_b)] {
                        if body.physics().body_type() == BodyType::Dynamic
                            && body.ccd_mode() == CcdMode::Continuous
                        {
                            let impacts =
                                self.collision_system.context.ccd_impact_counts[prediction];
                            if impacts >= self.collision_system.config().max_ccd_impacts {
                                return Err(CollisionError::CcdLimitExceeded);
                            }
                        }
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
                    let body_a_dynamic = body_a.physics().body_type() == BodyType::Dynamic;
                    let body_b_dynamic = body_b.physics().body_type() == BodyType::Dynamic;
                    if impact_elapsed
                        > self.collision_system.context.ccd_processed_time + time_tolerance
                    {
                        self.collision_system.context.ccd_processed_pairs.clear();
                        self.collision_system.context.ccd_processed_time = impact_elapsed;
                    }
                    if self
                        .collision_system
                        .context
                        .ccd_processed_pairs
                        .contains(&pair)
                    {
                        return Err(CollisionError::NonConvergent);
                    }
                    self.collision_system
                        .context
                        .ccd_processed_pairs
                        .try_reserve(1)
                        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                    self.collision_system.context.ccd_processed_pairs.push(pair);
                    elapsed = impact_elapsed;
                    if remaining_after_impact > f64::EPSILON * dt.max(1.0) {
                        self.collision_system.invalidate_ccd_candidates(
                            prediction_a,
                            body_a_dynamic,
                            prediction_b,
                            body_b_dynamic,
                        )?;
                        let invalidated_count = self.collision_system.ccd_resweep_candidate_count();
                        let (new_ccd_candidates, cross_island) = self.resweep_ccd_bodies(
                            [
                                (body_a_id, prediction_a, body_a_dynamic),
                                (body_b_id, prediction_b, body_b_dynamic),
                            ],
                            impact_elapsed,
                            dt,
                            force_single_island,
                            invalidated_count,
                        )?;
                        if new_ccd_candidates > 0 {
                            if cross_island && !force_single_island {
                                force_single_island = true;
                                #[cfg(feature = "bench-internals")]
                                self.collision_system.record_ccd_global_fallback();
                                self.reset_ccd_pass(dt, force_single_island)?;
                                restart_pass = true;
                                break;
                            }
                            self.collision_system
                                .build_ccd_islands(force_single_island)?;
                        }
                    }
                }
                if restart_pass {
                    continue 'passes;
                }
                island_index += 1;
            }
            break;
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_final_candidate_count(self.collision_system.candidate_count());
        Ok(())
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
        context.ccd_resweep_candidates.clear();
        context.ccd_query_results.clear();
        Ok(())
    }

    fn resweep_ccd_bodies(
        &mut self,
        bodies: [(BodyId, usize, bool); 2],
        elapsed: f64,
        dt: f64,
        force_single_island: bool,
        _invalidated_count: usize,
    ) -> Result<(usize, bool), CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        let mut new_ccd_candidates = 0;
        let mut cross_island = false;
        let proxy_count = self.collision_system.context.colliders.len();

        for (body_id, prediction_index, changed) in bodies {
            if !changed {
                continue;
            }
            let body = self
                .bodies
                .get(body_id.dense)
                .filter(|_| body_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?;
            let end_state = self
                .collision_system
                .context
                .predicted_states
                .get(prediction_index)
                .map(|(_, state)| *state)
                .ok_or(CollisionError::InvalidContact)?;

            for proxy_index in 0..proxy_count {
                let proxy = self
                    .collision_system
                    .context
                    .colliders
                    .get(proxy_index)
                    .copied()
                    .ok_or(CollisionError::InvalidContact)?;
                if proxy.body != body_id {
                    continue;
                }
                let bounds = {
                    let collider_entry = self
                        .colliders
                        .get(proxy.collider.dense)
                        .filter(|_| proxy.collider.world == self.world_id)
                        .ok_or(CollisionError::InvalidContact)?;
                    if collider_entry.body != body_id {
                        return Err(CollisionError::InvalidContact);
                    }
                    let trajectory = ColliderTrajectory {
                        body,
                        base_state: self.collision_system.context.ccd_states[prediction_index],
                        base_time: self.collision_system.context.ccd_elapsed[prediction_index],
                        sample_start_time: elapsed,
                        end_time: dt,
                        local_transform: collider_entry.collider.local_transform(),
                    };
                    swept_bounds_for_trajectory(&collider_entry.collider, &trajectory, end_state)?
                };
                let query_count = self
                    .collision_system
                    .prepare_ccd_resweep_proxy(proxy.collider, bounds)?;
                for query_index in 0..query_count {
                    let other_id = self
                        .collision_system
                        .ccd_query_result(query_index)
                        .ok_or(CollisionError::InvalidContact)?;
                    if !self
                        .collision_system
                        .ccd_pair_allowed(proxy.collider, other_id)?
                    {
                        continue;
                    }
                    let pair = CollisionPair::new(proxy.collider, other_id)
                        .ok_or(CollisionError::InvalidContact)?;
                    let other_proxy = self.collision_system.ccd_proxy(other_id)?;
                    let other_prediction = Self::predicted_body_index(
                        &self.collision_system.context.body_indices,
                        self.world_id,
                        other_proxy.body,
                    )?;
                    let (body_a, body_b) = if pair.a == proxy.collider {
                        (prediction_index, other_prediction)
                    } else {
                        (other_prediction, prediction_index)
                    };
                    let (added, is_ccd, joins_islands) = self.collision_system.add_ccd_candidate(
                        pair,
                        body_a,
                        body_b,
                        force_single_island,
                    )?;
                    if added && is_ccd {
                        new_ccd_candidates += 1;
                    }
                    cross_island |= joins_islands;
                }
            }
        }

        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_resweep(started.elapsed(), _invalidated_count);
        Ok((new_ccd_candidates, cross_island))
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
        }
        {
            let context = &mut self.collision_system.context;
            context.ccd_processed_pairs.clear();
            context.ccd_processed_time = 0.0;
            context.ccd_toi_cache.fill(Default::default());
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
        let duration = elapsed - start_time;
        if dt - start_time <= 0.0 {
            return Err(CollisionError::InvalidContact);
        }
        predict_from_state(&body, state_start, duration)
            .map_err(|_| CollisionError::ArithmeticOverflow)
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

    fn process_collision_candidates(&mut self, worker_count: usize) -> Result<(), CollisionError> {
        let candidate_count = self.collision_system.candidate_count();
        if candidate_count == 0 {
            self.collision_system.solve_contacts(worker_count)?;
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
        self.collision_system.solve_contacts(worker_count)?;

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
            });
            shard_remaining -= 1;
        }
        Ok(())
    }

    /// Validates all shard results before committing any body state.
    #[doc(hidden)]
    pub fn commit_step_shards(&mut self, shards: &[Vec<BodyStep>]) -> Result<(), StepError> {
        let mut steps = shards.iter().flatten();
        let mut step_dt = None;
        for (dense_id, entry) in self.bodies.iter() {
            let Some(step) = steps.next() else {
                return Err(StepError::InvalidStepResults);
            };
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
        if let Err(error) = self.prepare_collision_candidates(dt) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(shards.len().max(1)) {
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
        if body.world != self.world_id {
            return None;
        }
        let mut next = self.bodies.get(body.dense)?.first_collider;
        Some(std::iter::from_fn(move || {
            let id = next?;
            let entry = self.colliders.get(id.dense)?;
            next = entry.next;
            Some((id, &entry.collider))
        }))
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
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
