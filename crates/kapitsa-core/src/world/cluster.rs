use std::collections::{HashMap, HashSet};

use super::{BodyId, ClusterError, PartId, PhysicsWorld, WeldId};
use crate::cluster::{ClusterPart, RigidCluster, Weld};

impl PhysicsWorld {
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
