use super::{ColliderId, PhysicsWorld, expanded_query_bounds};
use crate::{
    aabb::Aabb,
    collision::{
        CollisionError, OverlapHit, QueryError, QueryFilter, QueryScratch, QueryTarget, Ray,
        RayHit, ShapeHit, overlaps_shape_with_scratch, raycast_shape_with_scratch,
        spatial_index::SpatialIndex, sweep_shape_with_scratch,
    },
};
use kapitsa_math::Vec3;

impl PhysicsWorld {
    pub fn raycast(&self, ray: Ray, filter: QueryFilter) -> Result<Option<RayHit>, QueryError> {
        self.raycast_with_scratch(ray, filter, &mut QueryScratch::default())
    }

    /// Performs a raycast using caller-owned reusable query storage.
    pub fn raycast_with_scratch(
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

    /// Performs a sweep using caller-owned reusable query storage.
    pub fn sweep_with_scratch(
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

    /// Collects overlaps using caller-owned reusable query storage.
    pub fn overlaps_with_scratch(
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

    pub(super) fn mark_query_index_dirty(&mut self) {
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

    pub(super) fn update_query_index_for_collider(&mut self, collider_id: ColliderId) {
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

    pub(super) fn rebuild_query_index(&mut self) {
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
}
