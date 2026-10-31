use std::collections::HashMap;

use crate::{
    aabb::Aabb,
    collider::{CollisionFilter, CollisionMode},
    collision::{CollisionError, CollisionPair},
    rigid_body::BodyType,
    world::{BodyId, ColliderId},
};

use super::spatial_index::SpatialIndex;

#[derive(Clone, Copy)]
pub(crate) struct CollisionProxy {
    pub(crate) collider: ColliderId,
    pub(crate) body: BodyId,
    pub(crate) bounds: Option<Aabb>,
    pub(crate) body_type: BodyType,
    pub(crate) mode: CollisionMode,
    pub(crate) filter: CollisionFilter,
    pub(crate) convex: bool,
}

/// Forms deterministic candidate pairs from indexed bounds and infinite surfaces.
pub(crate) struct BroadPhase {
    lookup: Option<HashMap<ColliderId, usize>>,
    query_results: Vec<ColliderId>,
    candidates: Vec<CollisionPair>,
}

impl BroadPhase {
    pub(crate) const fn new() -> Self {
        Self {
            lookup: None,
            query_results: Vec::new(),
            candidates: Vec::new(),
        }
    }

    pub(crate) fn generate(
        &mut self,
        mut index: Option<&mut SpatialIndex>,
        proxies: &[CollisionProxy],
    ) -> Result<&[CollisionPair], CollisionError> {
        self.candidates.clear();
        if proxies.is_empty() {
            return Ok(&self.candidates);
        }
        let lookup = self.lookup.get_or_insert_with(HashMap::new);
        lookup.clear();
        lookup
            .try_reserve(proxies.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for (position, proxy) in proxies.iter().enumerate() {
            if lookup.insert(proxy.collider, position).is_some() {
                return Err(CollisionError::InvalidContact);
            }
        }

        let mut finite_count = 0;
        for proxy in proxies {
            if let Some(bounds) = proxy.bounds {
                finite_count += 1;
                let index = index.as_deref_mut().ok_or(CollisionError::InvalidContact)?;
                index.query(bounds, &mut self.query_results)?;
                for candidate_id in self.query_results.iter().copied() {
                    if proxy.collider >= candidate_id {
                        continue;
                    }
                    let position = lookup
                        .get(&candidate_id)
                        .copied()
                        .ok_or(CollisionError::InvalidContact)?;
                    let other = proxies
                        .get(position)
                        .ok_or(CollisionError::InvalidContact)?;
                    if Self::allows_pair(proxy, other) {
                        Self::push_pair(&mut self.candidates, proxy.collider, other.collider)?;
                    }
                }
            }
        }

        if finite_count > 0 {
            for surface in proxies.iter().filter(|proxy| proxy.bounds.is_none()) {
                for finite in proxies.iter().filter(|proxy| proxy.bounds.is_some()) {
                    if Self::allows_pair(surface, finite) {
                        Self::push_pair(&mut self.candidates, surface.collider, finite.collider)?;
                    }
                }
            }
        }

        self.candidates
            .sort_by(|left, right| left.a.cmp(&right.a).then(left.b.cmp(&right.b)));
        self.candidates.dedup();
        Ok(&self.candidates)
    }

    fn allows_pair(a: &CollisionProxy, b: &CollisionProxy) -> bool {
        if a.collider == b.collider || a.body == b.body || !a.filter.allows(&b.filter) {
            return false;
        }
        let sensor = a.mode == CollisionMode::Sensor || b.mode == CollisionMode::Sensor;
        if sensor {
            return a.convex || b.convex;
        }
        (a.body_type == BodyType::Dynamic || b.body_type == BodyType::Dynamic)
            && (a.convex || b.convex)
    }

    fn push_pair(
        candidates: &mut Vec<CollisionPair>,
        a: ColliderId,
        b: ColliderId,
    ) -> Result<(), CollisionError> {
        let pair = CollisionPair::new(a, b).ok_or(CollisionError::InvalidContact)?;
        candidates
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        candidates.push(pair);
        Ok(())
    }
}

#[cfg(test)]
#[path = "broad_phase_tests.rs"]
mod tests;
