use kapitsa_math::{Affine3, Vec3};
use std::sync::atomic::{AtomicU64, Ordering};

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
    collider::Collider,
    collision::{
        CollisionConfig, CollisionError, CollisionEvent, CollisionInteraction, CollisionSystem,
        ContactInfo, OverlapHit, QueryError, QueryFilter, Ray, RayHit, ShapeHit, SolverBody,
        SolverManifold, SolverMaterial, overlaps_shape, raycast_shape, sweep_shape,
    },
    containers::DensePool,
    material::{MaterialId, MaterialStorage, PhysicsMaterial},
    rigid_body::{BodyState, BodyType, RigidBody, predict},
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

/// Owns rigid bodies and colliders and assigns IDs scoped to this world instance.
pub struct PhysicsWorld {
    world_id: u64,
    bodies: DensePool<BodyEntry>,
    colliders: DensePool<ColliderEntry>,
    materials: MaterialStorage,
    collision_system: CollisionSystem,
}

/// Opaque copied body data used to predict and atomically commit a world step.
#[doc(hidden)]
#[derive(Clone, Copy, Debug)]
pub struct BodyStep {
    body_id: BodyId,
    source: RigidBody,
    predicted: Option<BodyState>,
}

impl BodyStep {
    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    pub fn integrate(&mut self, dt: f64) -> Result<(), StepError> {
        self.predicted = None;
        self.predicted =
            Some(
                predict(&self.source, dt).map_err(|cause| StepError::BodyIntegration {
                    body_id: self.body_id,
                    cause,
                })?,
            );
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
        ray.validate()?;
        let mut closest = None;
        for (dense_id, entry) in self.colliders.iter() {
            if !filter.allows(entry.body, entry.collider.filter(), entry.collider.mode()) {
                continue;
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
            let Some((position, normal, distance)) = raycast_shape(
                entry.collider.shape(),
                transform,
                ray,
                entry.collider.query_bvh(),
                entry.collider.query_vertices(),
            )?
            else {
                continue;
            };
            if closest.is_none_or(|hit: RayHit| distance < hit.distance) {
                closest = Some(RayHit {
                    body: entry.body,
                    collider: ColliderId {
                        world: self.world_id,
                        dense: dense_id,
                    },
                    position,
                    normal,
                    distance,
                });
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
        if !shape.is_convex() {
            return Err(QueryError::UnsupportedQueryShape);
        }
        if !displacement.is_finite() {
            return Err(QueryError::InvalidInput);
        }
        let mut closest = None;
        for (dense_id, entry) in self.colliders.iter() {
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
            let Some(hit) = sweep_shape(
                shape,
                transform,
                displacement,
                entry.collider.shape(),
                target_transform,
                entry.collider.query_bvh(),
                entry.collider.query_vertices(),
            )?
            else {
                continue;
            };
            let position = hit.point_a * 0.5 + hit.point_b * 0.5;
            if !position.is_finite() || !hit.normal.is_finite() || !hit.toi.is_finite() {
                return Err(QueryError::ArithmeticOverflow);
            }
            if closest.is_none_or(|current: ShapeHit| hit.toi < current.toi) {
                closest = Some(ShapeHit {
                    body: entry.body,
                    collider: ColliderId {
                        world: self.world_id,
                        dense: dense_id,
                    },
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
        if !shape.is_convex() {
            return Err(QueryError::UnsupportedQueryShape);
        }
        let mut overlaps = Vec::new();
        for (dense_id, entry) in self.colliders.iter() {
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
            if overlaps_shape(
                shape,
                transform,
                entry.collider.shape(),
                target_transform,
                entry.collider.query_bvh(),
                entry.collider.query_vertices(),
            )? {
                overlaps.push(OverlapHit {
                    body: entry.body,
                    collider: ColliderId {
                        world: self.world_id,
                        dense: dense_id,
                    },
                });
            }
        }
        overlaps.sort_by_key(|hit| (hit.body, hit.collider));
        *output = overlaps;
        Ok(())
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

        if let Err(error) = self
            .stage_spatial_proxies()
            .and_then(|()| self.collision_system.synchronize_spatial_index())
            .and_then(|()| self.collision_system.generate_candidates())
        {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(1) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }

        for ((dense_id, entry), (body_id, state)) in self.bodies.iter_mut().zip(
            self.collision_system
                .context
                .predicted_states
                .iter()
                .copied(),
        ) {
            debug_assert_eq!(
                BodyId {
                    world: self.world_id,
                    dense: dense_id,
                },
                body_id
            );
            entry.body.commit_integrated_state(state);
        }
        self.collision_system.commit_step();
        Ok(())
    }

    fn stage_spatial_proxies(&mut self) -> Result<(), CollisionError> {
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
                let bounds = collider_entry
                    .collider
                    .world_bounds(&predicted_state.transform())
                    .map_err(|_| CollisionError::GeometryBounds)?;
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

    fn process_collision_candidates(&mut self, worker_count: usize) -> Result<(), CollisionError> {
        let candidate_count = self.collision_system.candidate_count();
        if candidate_count == 0 {
            self.collision_system.solve_contacts(worker_count)?;
            self.collision_system.stage_lifecycle_events()?;
            return Ok(());
        }
        let mut required_slots = 0;
        for (id, _) in &self.collision_system.context.predicted_states {
            let slot = usize::try_from(id.index()).map_err(|_| CollisionError::GeometryBounds)?;
            required_slots =
                required_slots.max(slot.checked_add(1).ok_or(CollisionError::GeometryBounds)?);
        }
        {
            let context = &mut self.collision_system.context;
            if context.body_indices.len() < required_slots {
                context
                    .body_indices
                    .try_reserve(required_slots - context.body_indices.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                context.body_indices.resize(required_slots, None);
            }
            if context.solver_body_indices.len() < required_slots {
                context
                    .solver_body_indices
                    .try_reserve(required_slots - context.solver_body_indices.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                context.solver_body_indices.resize(required_slots, None);
            }
            context
                .body_indices
                .iter_mut()
                .for_each(|slot| *slot = None);
            context
                .solver_body_indices
                .iter_mut()
                .for_each(|slot| *slot = None);
            context.body_types.clear();
            context.solver_bodies.clear();
            context.solver_body_predictions.clear();
            for (index, (body_id, _)) in context.predicted_states.iter().copied().enumerate() {
                let slot =
                    usize::try_from(body_id.index()).map_err(|_| CollisionError::GeometryBounds)?;
                *context
                    .body_indices
                    .get_mut(slot)
                    .ok_or(CollisionError::InvalidContact)? = Some((body_id, index));
            }
        }

        self.collision_system.context.manifolds.clear();
        self.collision_system
            .context
            .manifolds
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.collision_system.context.interactions.clear();
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
            if let Some(manifold) = self.collision_system.detect(
                pair,
                collider_a.collider.shape(),
                &transform_a,
                &body_a,
                collider_a.collider.mode(),
                collider_b.collider.shape(),
                &transform_b,
                &body_b,
                collider_b.collider.mode(),
            )? {
                let sensor = collider_a.collider.mode() == crate::collider::CollisionMode::Sensor
                    || collider_b.collider.mode() == crate::collider::CollisionMode::Sensor;
                if sensor {
                    self.collision_system
                        .context
                        .interactions
                        .push(CollisionInteraction {
                            pair,
                            sensor: true,
                            contact: None,
                            impact: false,
                        });
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
            for point in manifold.points() {
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
            self.collision_system
                .context
                .interactions
                .push(CollisionInteraction {
                    pair: manifold.pair,
                    sensor: false,
                    contact: Some(ContactInfo {
                        point,
                        normal: deepest.normal,
                        penetration: deepest.penetration,
                        normal_impulse: total_impulse,
                    }),
                    impact,
                });
        }
        self.collision_system.stage_lifecycle_events()?;
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
            });
            shard_remaining -= 1;
        }
        Ok(())
    }

    /// Validates all shard results before committing any body state.
    #[doc(hidden)]
    pub fn commit_step_shards(&mut self, shards: &[Vec<BodyStep>]) -> Result<(), StepError> {
        let mut steps = shards.iter().flatten();
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
            {
                return Err(StepError::InvalidStepResults);
            }
        }
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
        if let Err(error) = self
            .stage_spatial_proxies()
            .and_then(|()| self.collision_system.synchronize_spatial_index())
            .and_then(|()| self.collision_system.generate_candidates())
        {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(shards.len().max(1)) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }

        for ((dense_id, entry), (body_id, state)) in self.bodies.iter_mut().zip(
            self.collision_system
                .context
                .predicted_states
                .iter()
                .copied(),
        ) {
            debug_assert_eq!(
                BodyId {
                    world: self.world_id,
                    dense: dense_id,
                },
                body_id
            );
            entry.body.commit_integrated_state(state);
        }
        self.collision_system.commit_step();
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
        self.collision_system.mark_spatial_dirty();
        Ok(id)
    }

    /// Updates the deformed local vertices for a skinned-mesh collider.
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

#[cfg(test)]
#[path = "world_tests.rs"]
mod tests;
