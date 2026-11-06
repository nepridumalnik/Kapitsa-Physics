use std::collections::{HashMap, HashSet};

use crate::{aabb::Aabb, collision::CollisionError, world::ColliderId};
use kapitsa_math::Vec3;

use super::QueryError;

#[derive(Clone, Copy)]
struct Node {
    bounds: Aabb,
    parent: Option<usize>,
    left: Option<usize>,
    right: Option<usize>,
    height: u32,
    collider: Option<ColliderId>,
}

impl Node {
    fn leaf(collider: ColliderId, bounds: Aabb) -> Self {
        Self {
            bounds,
            parent: None,
            left: None,
            right: None,
            height: 0,
            collider: Some(collider),
        }
    }

    fn branch(left: usize, right: usize, bounds: Aabb, parent: Option<usize>) -> Self {
        Self {
            bounds,
            parent,
            left: Some(left),
            right: Some(right),
            height: 1,
            collider: None,
        }
    }

    fn is_leaf(self) -> bool {
        self.collider.is_some()
    }
}

/// Incremental dynamic AABB tree with stable node slots and reusable scratch.
pub(crate) struct SpatialIndex {
    nodes: Vec<Option<Node>>,
    free_nodes: Vec<usize>,
    leaves: HashMap<ColliderId, usize>,
    present: HashSet<ColliderId>,
    stale: Vec<ColliderId>,
    query_stack: Vec<usize>,
    root: Option<usize>,
    dirty: bool,
}

impl SpatialIndex {
    pub(crate) fn new() -> Self {
        Self {
            nodes: Vec::new(),
            free_nodes: Vec::new(),
            leaves: HashMap::new(),
            present: HashSet::new(),
            stale: Vec::new(),
            query_stack: Vec::new(),
            root: None,
            dirty: true,
        }
    }

    pub(crate) fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    pub(crate) fn insert(
        &mut self,
        collider: ColliderId,
        bounds: Aabb,
    ) -> Result<(), CollisionError> {
        if self.leaves.contains_key(&collider) {
            self.update(collider, bounds)?;
            return Ok(());
        }
        self.nodes
            .try_reserve(2)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.free_nodes
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.leaves
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let leaf = self.allocate_node(Node::leaf(collider, bounds))?;
        self.leaves.insert(collider, leaf);
        if let Err(error) = self.insert_leaf(leaf) {
            self.leaves.remove(&collider);
            self.free_nodes.push(leaf);
            if let Some(slot) = self.nodes.get_mut(leaf) {
                *slot = None;
            }
            return Err(error);
        }
        self.dirty = false;
        Ok(())
    }

    pub(crate) fn remove(&mut self, collider: ColliderId) -> Result<bool, CollisionError> {
        let Some(leaf) = self.leaves.get(&collider).copied() else {
            return Ok(false);
        };
        self.free_nodes
            .try_reserve(2)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.detach_leaf(leaf)?;
        self.free_node(leaf)?;
        self.leaves.remove(&collider);
        self.dirty = false;
        Ok(true)
    }

    pub(crate) fn update(
        &mut self,
        collider: ColliderId,
        bounds: Aabb,
    ) -> Result<bool, CollisionError> {
        let Some(leaf) = self.leaves.get(&collider).copied() else {
            self.insert(collider, bounds)?;
            return Ok(true);
        };
        if self.node(leaf)?.bounds == bounds {
            return Ok(false);
        }
        self.free_nodes
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.detach_leaf(leaf)?;
        self.node_mut(leaf)?.bounds = bounds;
        self.insert_leaf(leaf)?;
        self.dirty = false;
        Ok(true)
    }

    pub(crate) fn contains_bounds(
        &self,
        collider: ColliderId,
        bounds: Aabb,
    ) -> Result<bool, CollisionError> {
        let Some(leaf) = self.leaves.get(&collider).copied() else {
            return Ok(false);
        };
        let proxy = self.node(leaf)?.bounds;
        Ok(proxy.min().x <= bounds.min().x
            && proxy.min().y <= bounds.min().y
            && proxy.min().z <= bounds.min().z
            && proxy.max().x >= bounds.max().x
            && proxy.max().y >= bounds.max().y
            && proxy.max().z >= bounds.max().z)
    }

    pub(crate) fn query(
        &mut self,
        bounds: Aabb,
        output: &mut Vec<ColliderId>,
    ) -> Result<(), CollisionError> {
        output.clear();
        self.query_stack.clear();
        if let Some(root) = self.root {
            self.query_stack
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.query_stack.push(root);
        }
        while let Some(index) = self.query_stack.pop() {
            let node = *self.node(index)?;
            if !node.bounds.intersects(&bounds) {
                continue;
            }
            if let Some(collider) = node.collider {
                output
                    .try_reserve(1)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                output.push(collider);
            } else {
                let left = node.left.ok_or(CollisionError::InvalidContact)?;
                let right = node.right.ok_or(CollisionError::InvalidContact)?;
                self.query_stack
                    .try_reserve(2)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                self.query_stack.push(right);
                self.query_stack.push(left);
            }
        }
        Ok(())
    }

    pub(crate) fn query_readonly(
        &self,
        bounds: Aabb,
        stack: &mut Vec<usize>,
        output: &mut Vec<ColliderId>,
    ) -> Result<(), CollisionError> {
        stack.clear();
        output.clear();
        if let Some(root) = self.root {
            stack
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            stack.push(root);
        }
        while let Some(index) = stack.pop() {
            let node = *self.node(index)?;
            if !node.bounds.intersects(&bounds) {
                continue;
            }
            if let Some(collider) = node.collider {
                output
                    .try_reserve(1)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                output.push(collider);
            } else {
                let left = node.left.ok_or(CollisionError::InvalidContact)?;
                let right = node.right.ok_or(CollisionError::InvalidContact)?;
                stack
                    .try_reserve(2)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                stack.push(right);
                stack.push(left);
            }
        }
        Ok(())
    }

    pub(crate) fn query_ray(
        &self,
        origin: Vec3,
        direction: Vec3,
        mut max_distance: f64,
        stack: &mut Vec<(usize, f64)>,
        mut visit: impl FnMut(ColliderId) -> Result<Option<f64>, QueryError>,
    ) -> Result<(), QueryError> {
        stack.clear();
        if let Some(root) = self.root
            && let Some(entry) = ray_aabb_entry(
                self.node(root).map_err(QueryError::Geometry)?.bounds,
                origin,
                direction,
                max_distance,
            )
        {
            stack
                .try_reserve(1)
                .map_err(|_| QueryError::Geometry(CollisionError::SpatialIndexCapacityExceeded))?;
            stack.push((root, entry));
        }
        while let Some((index, entry)) = stack.pop() {
            if entry > max_distance {
                continue;
            }
            let node = *self.node(index).map_err(QueryError::Geometry)?;
            if let Some(collider) = node.collider {
                if let Some(hit_distance) = visit(collider)? {
                    if !hit_distance.is_finite()
                        || hit_distance < 0.0
                        || hit_distance > max_distance
                    {
                        return Err(QueryError::ArithmeticOverflow);
                    }
                    max_distance = hit_distance;
                }
                continue;
            }

            let left = node
                .left
                .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
            let right = node
                .right
                .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
            let left_entry = ray_aabb_entry(
                self.node(left).map_err(QueryError::Geometry)?.bounds,
                origin,
                direction,
                max_distance,
            );
            let right_entry = ray_aabb_entry(
                self.node(right).map_err(QueryError::Geometry)?.bounds,
                origin,
                direction,
                max_distance,
            );
            match (left_entry, right_entry) {
                (Some(left_entry), Some(right_entry)) => {
                    stack.try_reserve(2).map_err(|_| {
                        QueryError::Geometry(CollisionError::SpatialIndexCapacityExceeded)
                    })?;
                    if left_entry <= right_entry {
                        stack.push((right, right_entry));
                        stack.push((left, left_entry));
                    } else {
                        stack.push((left, left_entry));
                        stack.push((right, right_entry));
                    }
                }
                (Some(left_entry), None) => {
                    stack.try_reserve(1).map_err(|_| {
                        QueryError::Geometry(CollisionError::SpatialIndexCapacityExceeded)
                    })?;
                    stack.push((left, left_entry));
                }
                (None, Some(right_entry)) => {
                    stack.try_reserve(1).map_err(|_| {
                        QueryError::Geometry(CollisionError::SpatialIndexCapacityExceeded)
                    })?;
                    stack.push((right, right_entry));
                }
                (None, None) => {}
            }
        }
        Ok(())
    }

    pub(crate) fn synchronize(
        &mut self,
        proxies: &[(ColliderId, Aabb)],
    ) -> Result<(), CollisionError> {
        self.present.clear();
        self.present
            .try_reserve(proxies.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for (collider, _) in proxies {
            if !self.present.insert(*collider) {
                return Err(CollisionError::InvalidContact);
            }
        }

        self.stale.clear();
        if self.dirty || self.leaves.len() != self.present.len() {
            self.stale
                .try_reserve(self.leaves.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.stale.extend(
                self.leaves
                    .keys()
                    .filter(|id| !self.present.contains(id))
                    .copied(),
            );
        }
        self.free_nodes
            .try_reserve(self.stale.len().saturating_mul(2))
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let new_count = proxies
            .iter()
            .filter(|(id, _)| !self.leaves.contains_key(id))
            .count();
        self.nodes
            .try_reserve(new_count.saturating_mul(2))
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.free_nodes
            .try_reserve(new_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.leaves
            .try_reserve(new_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for position in 0..self.stale.len() {
            let collider = self
                .stale
                .get(position)
                .copied()
                .ok_or(CollisionError::InvalidContact)?;
            self.remove(collider)?;
        }
        for (collider, bounds) in proxies.iter().copied() {
            self.update(collider, bounds)?;
        }
        self.dirty = false;
        Ok(())
    }

    fn allocate_node(&mut self, node: Node) -> Result<usize, CollisionError> {
        if let Some(index) = self.free_nodes.pop() {
            let slot = self
                .nodes
                .get_mut(index)
                .ok_or(CollisionError::InvalidContact)?;
            if slot.replace(node).is_some() {
                return Err(CollisionError::InvalidContact);
            }
            return Ok(index);
        }
        self.nodes
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let index = self.nodes.len();
        self.nodes.push(Some(node));
        Ok(index)
    }

    fn free_node(&mut self, index: usize) -> Result<(), CollisionError> {
        let slot = self
            .nodes
            .get_mut(index)
            .ok_or(CollisionError::InvalidContact)?;
        if slot.take().is_none() {
            return Err(CollisionError::InvalidContact);
        }
        self.free_nodes.push(index);
        Ok(())
    }

    fn node(&self, index: usize) -> Result<&Node, CollisionError> {
        self.nodes
            .get(index)
            .and_then(Option::as_ref)
            .ok_or(CollisionError::InvalidContact)
    }

    fn node_mut(&mut self, index: usize) -> Result<&mut Node, CollisionError> {
        self.nodes
            .get_mut(index)
            .and_then(Option::as_mut)
            .ok_or(CollisionError::InvalidContact)
    }

    fn insert_leaf(&mut self, leaf: usize) -> Result<(), CollisionError> {
        let Some(root) = self.root else {
            self.root = Some(leaf);
            self.node_mut(leaf)?.parent = None;
            return Ok(());
        };

        let leaf_center = self.node(leaf)?.bounds.center();
        let mut sibling = root;
        loop {
            let node = *self.node(sibling)?;
            if node.is_leaf() {
                break;
            }
            let left = node.left.ok_or(CollisionError::InvalidContact)?;
            let right = node.right.ok_or(CollisionError::InvalidContact)?;
            let left_center = self.node(left)?.bounds.center();
            let right_center = self.node(right)?.bounds.center();
            let left_distance = (left_center - leaf_center).abs().max_element();
            let right_distance = (right_center - leaf_center).abs().max_element();
            sibling = if left_distance <= right_distance {
                left
            } else {
                right
            };
        }

        let old_parent = self.node(sibling)?.parent;
        let bounds = self.node(sibling)?.bounds.union(&self.node(leaf)?.bounds);
        let parent = self.allocate_node(Node::branch(sibling, leaf, bounds, old_parent))?;
        self.node_mut(sibling)?.parent = Some(parent);
        self.node_mut(leaf)?.parent = Some(parent);
        if let Some(old_parent) = old_parent {
            self.replace_child(old_parent, sibling, parent)?;
        } else {
            self.root = Some(parent);
        }
        self.refit_upwards(Some(parent))
    }

    fn detach_leaf(&mut self, leaf: usize) -> Result<(), CollisionError> {
        let Some(parent) = self.node(leaf)?.parent else {
            self.root = None;
            return Ok(());
        };
        let parent_node = *self.node(parent)?;
        let sibling = match (parent_node.left, parent_node.right) {
            (Some(left), Some(right)) if left == leaf => right,
            (Some(left), Some(right)) if right == leaf => left,
            _ => return Err(CollisionError::InvalidContact),
        };
        let grandparent = parent_node.parent;
        if let Some(grandparent) = grandparent {
            self.replace_child(grandparent, parent, sibling)?;
            self.node_mut(sibling)?.parent = Some(grandparent);
            self.free_node(parent)?;
            self.refit_upwards(Some(grandparent))
        } else {
            self.root = Some(sibling);
            self.node_mut(sibling)?.parent = None;
            self.free_node(parent)
        }
    }

    fn replace_child(
        &mut self,
        parent: usize,
        old: usize,
        new: usize,
    ) -> Result<(), CollisionError> {
        let node = self.node_mut(parent)?;
        if node.left == Some(old) {
            node.left = Some(new);
        } else if node.right == Some(old) {
            node.right = Some(new);
        } else {
            return Err(CollisionError::InvalidContact);
        }
        Ok(())
    }

    fn refit_upwards(&mut self, mut current: Option<usize>) -> Result<(), CollisionError> {
        while let Some(index) = current {
            let balanced = self.balance(index)?;
            current = self.node(balanced)?.parent;
        }
        Ok(())
    }

    fn balance(&mut self, index: usize) -> Result<usize, CollisionError> {
        let node = *self.node(index)?;
        if node.is_leaf() {
            return Ok(index);
        }
        let left = node.left.ok_or(CollisionError::InvalidContact)?;
        let right = node.right.ok_or(CollisionError::InvalidContact)?;
        let left_height = self.node(left)?.height;
        let right_height = self.node(right)?.height;
        if right_height > left_height.saturating_add(1) {
            let right_node = *self.node(right)?;
            let first = right_node.left.ok_or(CollisionError::InvalidContact)?;
            let second = right_node.right.ok_or(CollisionError::InvalidContact)?;
            let first_height = self.node(first)?.height;
            let second_height = self.node(second)?.height;
            self.replace_parent(index, right, node.parent)?;
            self.node_mut(right)?.left = Some(index);
            self.node_mut(right)?.parent = node.parent;
            self.node_mut(index)?.parent = Some(right);
            if first_height > second_height {
                self.node_mut(right)?.right = Some(first);
                self.node_mut(index)?.right = Some(second);
                self.node_mut(first)?.parent = Some(right);
                self.node_mut(second)?.parent = Some(index);
            } else {
                self.node_mut(right)?.right = Some(second);
                self.node_mut(index)?.right = Some(first);
                self.node_mut(second)?.parent = Some(right);
                self.node_mut(first)?.parent = Some(index);
            }
            self.recompute(index)?;
            self.recompute(right)?;
            return Ok(right);
        }
        if left_height > right_height.saturating_add(1) {
            let left_node = *self.node(left)?;
            let first = left_node.left.ok_or(CollisionError::InvalidContact)?;
            let second = left_node.right.ok_or(CollisionError::InvalidContact)?;
            let first_height = self.node(first)?.height;
            let second_height = self.node(second)?.height;
            self.replace_parent(index, left, node.parent)?;
            self.node_mut(left)?.right = Some(index);
            self.node_mut(left)?.parent = node.parent;
            self.node_mut(index)?.parent = Some(left);
            if first_height > second_height {
                self.node_mut(left)?.left = Some(first);
                self.node_mut(index)?.left = Some(second);
                self.node_mut(first)?.parent = Some(left);
                self.node_mut(second)?.parent = Some(index);
            } else {
                self.node_mut(left)?.left = Some(second);
                self.node_mut(index)?.left = Some(first);
                self.node_mut(second)?.parent = Some(left);
                self.node_mut(first)?.parent = Some(index);
            }
            self.recompute(index)?;
            self.recompute(left)?;
            return Ok(left);
        }
        self.recompute(index)?;
        Ok(index)
    }

    fn replace_parent(
        &mut self,
        old_child: usize,
        new_child: usize,
        parent: Option<usize>,
    ) -> Result<(), CollisionError> {
        if let Some(parent) = parent {
            self.replace_child(parent, old_child, new_child)
        } else {
            self.root = Some(new_child);
            Ok(())
        }
    }

    fn recompute(&mut self, index: usize) -> Result<(), CollisionError> {
        let node = *self.node(index)?;
        if node.is_leaf() {
            return Ok(());
        }
        let left = self.node(node.left.ok_or(CollisionError::InvalidContact)?)?;
        let right = self.node(node.right.ok_or(CollisionError::InvalidContact)?)?;
        let bounds = left.bounds.union(&right.bounds);
        let height = left
            .height
            .max(right.height)
            .checked_add(1)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let node = self.node_mut(index)?;
        node.bounds = bounds;
        node.height = height;
        Ok(())
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        self.nodes.capacity() * size_of::<Option<Node>>()
            + self.free_nodes.capacity() * size_of::<usize>()
            + self.leaves.capacity() * size_of::<(ColliderId, usize)>()
            + self.present.capacity() * size_of::<ColliderId>()
            + self.stale.capacity() * size_of::<ColliderId>()
            + self.query_stack.capacity() * size_of::<usize>()
    }
}

fn ray_aabb_entry(bounds: Aabb, origin: Vec3, direction: Vec3, max_distance: f64) -> Option<f64> {
    let origins = [origin.x, origin.y, origin.z];
    let directions = [direction.x, direction.y, direction.z];
    let minimum = [bounds.min().x, bounds.min().y, bounds.min().z];
    let maximum = [bounds.max().x, bounds.max().y, bounds.max().z];
    let mut near = 0.0_f64;
    let mut far = max_distance;

    for axis in 0..3 {
        let direction = directions[axis];
        if direction == 0.0 {
            if origins[axis] < minimum[axis] || origins[axis] > maximum[axis] {
                return None;
            }
            continue;
        }
        let mut first = (minimum[axis] - origins[axis]) / direction;
        let mut second = (maximum[axis] - origins[axis]) / direction;
        if first > second {
            std::mem::swap(&mut first, &mut second);
        }
        near = near.max(first);
        far = far.min(second);
        if near > far {
            return None;
        }
    }

    Some(near)
}

#[cfg(test)]
#[path = "spatial_index_tests.rs"]
mod tests;
