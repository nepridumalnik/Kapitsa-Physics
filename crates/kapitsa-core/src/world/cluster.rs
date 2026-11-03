use std::collections::{HashMap, HashSet};

use super::{BodyId, ClusterError, ColliderId, PartId, PhysicsWorld, WeldId};
use crate::{
    cluster::{ClusterPart, RigidCluster, Weld},
    rigid_body::BodyType,
    transform::Transform,
};

impl PhysicsWorld {
    #[allow(dead_code)]
    pub(crate) fn rebind_collider(
        &mut self,
        collider: ColliderId,
        body: BodyId,
        local_transform: Transform,
    ) -> Result<(), ClusterError> {
        if collider.world != self.world_id || body.world != self.world_id {
            return Err(ClusterError::InvalidTopology);
        }
        let target = self
            .bodies
            .get(body.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        let target_first = target.first_collider;
        let entry = self
            .colliders
            .get(collider.dense)
            .ok_or(ClusterError::InvalidTopology)?;
        let old = entry.body;
        let prev = entry.prev;
        let next = entry.next;
        if target.body.physics().body_type() == BodyType::Dynamic
            && !entry.collider.shape().is_convex()
        {
            return Err(ClusterError::UnsupportedGeometry);
        }
        let old_body = self
            .bodies
            .get(old.dense)
            .ok_or(ClusterError::InvalidTopology)?;
        if old.world != self.world_id
            || (prev.is_none() && old_body.first_collider != Some(collider))
            || prev.is_some_and(|id| {
                self.colliders
                    .get(id.dense)
                    .is_none_or(|other| other.body != old || other.next != Some(collider))
            })
            || next.is_some_and(|id| {
                self.colliders
                    .get(id.dense)
                    .is_none_or(|other| other.body != old || other.prev != Some(collider))
            })
            || target_first.is_some_and(|id| {
                self.colliders
                    .get(id.dense)
                    .is_none_or(|other| other.body != body || other.prev.is_some())
            })
        {
            return Err(ClusterError::InvalidTopology);
        }
        if old != body {
            if let Some(id) = prev {
                if let Some(other) = self.colliders.get_mut(id.dense) {
                    other.next = next;
                }
            } else if let Some(old_body) = self.bodies.get_mut(old.dense) {
                old_body.first_collider = next;
            }
            if let Some(id) = next
                && let Some(other) = self.colliders.get_mut(id.dense)
            {
                other.prev = prev;
            }
            if let Some(id) = target_first
                && let Some(other) = self.colliders.get_mut(id.dense)
            {
                other.prev = Some(collider);
            }
            if let Some(target) = self.bodies.get_mut(body.dense) {
                target.first_collider = Some(collider);
            }
        }
        if let Some(entry) = self.colliders.get_mut(collider.dense) {
            if old != body {
                entry.body = body;
                entry.prev = None;
                entry.next = target_first;
            }
            entry.collider.set_local_transform(local_transform);
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(collider);
        Ok(())
    }

    pub fn cluster(&self, body: BodyId) -> Option<&RigidCluster> {
        if body.world != self.world_id {
            return None;
        }
        self.bodies.get(body.dense)?.cluster.as_ref()
    }

    pub fn cluster_part(&self, part: PartId) -> Option<&ClusterPart> {
        if part.world != self.world_id {
            return None;
        }
        self.parts.get(part.dense)
    }

    pub fn weld(&self, weld: WeldId) -> Option<&Weld> {
        if weld.world != self.world_id {
            return None;
        }
        self.welds.get(weld.dense)
    }

    pub fn part_body(&self, part: PartId) -> Option<BodyId> {
        Some(self.cluster_part(part)?.body())
    }

    pub fn validate_clusters(&self) -> Result<(), ClusterError> {
        let mut all_parts = HashSet::new();
        let mut all_welds = HashSet::new();
        let mut all_colliders = HashSet::new();
        for (dense, entry) in self.bodies.iter() {
            let Some(cluster) = &entry.cluster else {
                continue;
            };
            let body = BodyId {
                world: self.world_id,
                dense,
            };
            let mut adjacency = HashMap::new();
            for &part_id in cluster.parts() {
                let part = self
                    .cluster_part(part_id)
                    .ok_or(ClusterError::InvalidTopology)?;
                if part.body() != body || !all_parts.insert(part_id) {
                    return Err(ClusterError::InvalidTopology);
                }
                adjacency.insert(part_id, Vec::new());
                for &collider in part.colliders() {
                    if self.collider_body(collider) != Some(body) || !all_colliders.insert(collider)
                    {
                        return Err(ClusterError::InvalidTopology);
                    }
                }
            }
            if !adjacency.contains_key(&cluster.root()) {
                return Err(ClusterError::InvalidTopology);
            }
            for &weld_id in cluster.welds() {
                let weld = self.weld(weld_id).ok_or(ClusterError::InvalidTopology)?;
                let (a, b) = weld.parts();
                if a == b
                    || !all_welds.insert(weld_id)
                    || !adjacency.contains_key(&a)
                    || !adjacency.contains_key(&b)
                {
                    return Err(ClusterError::InvalidTopology);
                }
                if let Some(edges) = adjacency.get_mut(&a) {
                    edges.push(b);
                }
                if let Some(edges) = adjacency.get_mut(&b) {
                    edges.push(a);
                }
            }
            let mut visited = HashSet::new();
            let mut stack = vec![cluster.root()];
            while let Some(part) = stack.pop() {
                if visited.insert(part)
                    && let Some(neighbors) = adjacency.get(&part)
                {
                    stack.extend(neighbors.iter().copied());
                }
            }
            if visited.len() != adjacency.len() {
                return Err(ClusterError::InvalidTopology);
            }
            let mut collider = entry.first_collider;
            let mut linked_colliders = HashSet::new();
            while let Some(id) = collider {
                if !all_colliders.contains(&id) || !linked_colliders.insert(id) {
                    return Err(ClusterError::InvalidTopology);
                }
                let collider_entry = self
                    .colliders
                    .get(id.dense)
                    .ok_or(ClusterError::InvalidTopology)?;
                if collider_entry.body != body {
                    return Err(ClusterError::InvalidTopology);
                }
                collider = collider_entry.next;
            }
            if adjacency
                .keys()
                .filter_map(|id| self.cluster_part(*id))
                .map(|part| part.colliders().len())
                .sum::<usize>()
                != linked_colliders.len()
            {
                return Err(ClusterError::InvalidTopology);
            }
        }
        if all_parts.len() != self.parts.len() || all_welds.len() != self.welds.len() {
            return Err(ClusterError::InvalidTopology);
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "cluster_tests.rs"]
mod tests;
