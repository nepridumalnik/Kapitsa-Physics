use crate::{aabb::Aabb, shape::TriangleMesh, transform::Transform};
use kapitsa_math::Vec3;

use super::CollisionError;

const LEAF_TRIANGLES: usize = 4;

#[derive(Clone, Copy)]
struct Primitive {
    triangle: usize,
    bounds: Aabb,
    center: Vec3,
}

#[derive(Clone, Copy)]
enum NodeKind {
    Leaf { start: usize, count: usize },
    Branch { left: usize, right: usize },
}

#[derive(Clone, Copy)]
struct Node {
    bounds: Aabb,
    kind: NodeKind,
}

/// Immutable local-space BVH for one triangle mesh.
pub(crate) struct TriangleBvh {
    nodes: Vec<Node>,
    triangles: Vec<usize>,
    triangle_bounds: Vec<Aabb>,
    triangle_count: usize,
}

/// Reusable traversal stack kept separate so queries can run concurrently.
#[derive(Default)]
pub(crate) struct MeshQueryWorkspace {
    stack: Vec<usize>,
}

impl MeshQueryWorkspace {
    pub(crate) const fn new() -> Self {
        Self { stack: Vec::new() }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        self.stack.capacity() * std::mem::size_of::<usize>()
    }
}

#[derive(Default)]
pub(crate) struct BvhRefitScratch {
    triangle_bounds: Vec<Aabb>,
    node_bounds: Vec<Aabb>,
}

impl BvhRefitScratch {
    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        self.triangle_bounds.capacity() * std::mem::size_of::<Aabb>()
            + self.node_bounds.capacity() * std::mem::size_of::<Aabb>()
    }
}

impl TriangleBvh {
    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        self.nodes.capacity() * std::mem::size_of::<Node>()
            + self.triangles.capacity() * std::mem::size_of::<usize>()
            + self.triangle_bounds.capacity() * std::mem::size_of::<Aabb>()
    }

    pub(crate) fn build(mesh: &TriangleMesh) -> Result<Self, CollisionError> {
        let triangle_count = mesh.triangles().len();
        let node_capacity = triangle_count
            .checked_mul(2)
            .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        let mut primitives = Vec::new();
        primitives
            .try_reserve_exact(triangle_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let mut bounds_by_triangle = Vec::new();
        bounds_by_triangle
            .try_reserve_exact(triangle_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for (triangle, indices) in mesh.triangles().iter().copied().enumerate() {
            let [a, b, c] = indices;
            let vertices = mesh.vertices();
            let a = *vertices.get(a).ok_or(CollisionError::GeometryBounds)?;
            let b = *vertices.get(b).ok_or(CollisionError::GeometryBounds)?;
            let c = *vertices.get(c).ok_or(CollisionError::GeometryBounds)?;
            let bounds = triangle_bounds([a, b, c])?;
            bounds_by_triangle.push(bounds);
            primitives.push(Primitive {
                triangle,
                bounds,
                center: bounds.center(),
            });
        }

        let mut nodes = Vec::new();
        nodes
            .try_reserve_exact(node_capacity)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let mut triangles = Vec::new();
        triangles
            .try_reserve_exact(triangle_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        build_node(
            &mut primitives,
            0,
            triangle_count,
            &mut nodes,
            &mut triangles,
        )?;
        Ok(Self {
            nodes,
            triangles,
            triangle_bounds: bounds_by_triangle,
            triangle_count,
        })
    }

    pub(crate) fn triangle_count(&self) -> usize {
        self.triangle_count
    }

    pub(crate) fn refit_vertices(
        &mut self,
        mesh: &TriangleMesh,
        vertices: &[Vec3],
    ) -> Result<(), CollisionError> {
        let mut scratch = BvhRefitScratch::default();
        self.prepare_refit(mesh, vertices, &mut scratch)?;
        self.commit_refit(&mut scratch);
        Ok(())
    }

    pub(crate) fn prepare_refit(
        &self,
        mesh: &TriangleMesh,
        vertices: &[Vec3],
        scratch: &mut BvhRefitScratch,
    ) -> Result<(), CollisionError> {
        if vertices.len() != mesh.vertices().len()
            || mesh.triangles().len() != self.triangle_count
            || !vertices.iter().all(|vertex| vertex.is_finite())
        {
            return Err(CollisionError::InvalidSkinPose);
        }

        scratch.triangle_bounds.clear();
        scratch
            .triangle_bounds
            .try_reserve_exact(self.triangle_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for [a, b, c] in mesh.triangles().iter().copied() {
            let a = *vertices.get(a).ok_or(CollisionError::GeometryBounds)?;
            let b = *vertices.get(b).ok_or(CollisionError::GeometryBounds)?;
            let c = *vertices.get(c).ok_or(CollisionError::GeometryBounds)?;
            scratch.triangle_bounds.push(triangle_bounds([a, b, c])?);
        }

        scratch.node_bounds.clear();
        scratch
            .node_bounds
            .try_reserve_exact(self.nodes.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        scratch
            .node_bounds
            .extend(self.nodes.iter().map(|node| node.bounds));
        for (index, node) in self.nodes.iter().copied().enumerate().rev() {
            let bounds = match node.kind {
                NodeKind::Leaf { start, count } => {
                    let end = start
                        .checked_add(count)
                        .ok_or(CollisionError::GeometryBounds)?;
                    let triangle_ids = self
                        .triangles
                        .get(start..end)
                        .ok_or(CollisionError::GeometryBounds)?;
                    let mut ids = triangle_ids.iter().copied();
                    let first = ids.next().ok_or(CollisionError::GeometryBounds)?;
                    let first_bounds = *scratch
                        .triangle_bounds
                        .get(first)
                        .ok_or(CollisionError::GeometryBounds)?;
                    ids.try_fold(first_bounds, |bounds, triangle| {
                        scratch
                            .triangle_bounds
                            .get(triangle)
                            .map(|triangle_bounds| bounds.union(triangle_bounds))
                            .ok_or(CollisionError::GeometryBounds)
                    })?
                }
                NodeKind::Branch { left, right } => {
                    let left = scratch
                        .node_bounds
                        .get(left)
                        .ok_or(CollisionError::GeometryBounds)?;
                    let right = scratch
                        .node_bounds
                        .get(right)
                        .ok_or(CollisionError::GeometryBounds)?;
                    left.union(right)
                }
            };
            *scratch
                .node_bounds
                .get_mut(index)
                .ok_or(CollisionError::GeometryBounds)? = bounds;
        }
        Ok(())
    }

    pub(crate) fn commit_refit(&mut self, scratch: &mut BvhRefitScratch) {
        std::mem::swap(&mut self.triangle_bounds, &mut scratch.triangle_bounds);
        for (node, bounds) in self
            .nodes
            .iter_mut()
            .zip(scratch.node_bounds.iter().copied())
        {
            node.bounds = bounds;
        }
    }

    pub(crate) fn query(
        &self,
        bounds: Aabb,
        workspace: &mut MeshQueryWorkspace,
        output: &mut Vec<usize>,
    ) -> Result<(), CollisionError> {
        output.clear();
        workspace.stack.clear();
        if self.nodes.is_empty() {
            return Ok(());
        }
        workspace
            .stack
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        workspace.stack.push(0);
        while let Some(index) = workspace.stack.pop() {
            let node = self
                .nodes
                .get(index)
                .ok_or(CollisionError::GeometryBounds)?;
            if !node.bounds.intersects(&bounds) {
                continue;
            }
            match node.kind {
                NodeKind::Leaf { start, count } => {
                    let end = start
                        .checked_add(count)
                        .ok_or(CollisionError::GeometryBounds)?;
                    let triangle_ids = self
                        .triangles
                        .get(start..end)
                        .ok_or(CollisionError::GeometryBounds)?;
                    for triangle in triangle_ids.iter().copied() {
                        let triangle_bounds = self
                            .triangle_bounds
                            .get(triangle)
                            .ok_or(CollisionError::GeometryBounds)?;
                        if triangle_bounds.intersects(&bounds) {
                            output
                                .try_reserve(1)
                                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                            output.push(triangle);
                        }
                    }
                }
                NodeKind::Branch { left, right } => {
                    workspace
                        .stack
                        .try_reserve(2)
                        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                    workspace.stack.push(right);
                    workspace.stack.push(left);
                }
            }
        }
        output.sort_unstable();
        Ok(())
    }
}

pub(crate) fn local_query_bounds(
    world_bounds: Aabb,
    world_transform: &Transform,
) -> Result<Aabb, CollisionError> {
    let inverse = world_transform
        .inverse()
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    let min = world_bounds.min();
    let max = world_bounds.max();
    let corners = [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(max.x, max.y, max.z),
    ];
    let first = inverse.transform_point(corners[0]);
    if !first.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let mut local_min = first;
    let mut local_max = first;
    for corner in corners.into_iter().skip(1) {
        let point = inverse.transform_point(corner);
        if !point.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        local_min = local_min.min(point);
        local_max = local_max.max(point);
    }
    Aabb::new(local_min, local_max).map_err(|_| CollisionError::GeometryBounds)
}

fn build_node(
    primitives: &mut [Primitive],
    start: usize,
    end: usize,
    nodes: &mut Vec<Node>,
    triangles: &mut Vec<usize>,
) -> Result<usize, CollisionError> {
    if start >= end || end > primitives.len() {
        return Err(CollisionError::GeometryBounds);
    }
    let first = primitives
        .get(start)
        .ok_or(CollisionError::GeometryBounds)?;
    let bounds = primitives
        .get(start + 1..end)
        .ok_or(CollisionError::GeometryBounds)?
        .iter()
        .fold(first.bounds, |bounds, primitive| {
            bounds.union(&primitive.bounds)
        });
    let node_index = nodes.len();
    nodes.push(Node {
        bounds,
        kind: NodeKind::Leaf { start: 0, count: 0 },
    });
    if end - start <= LEAF_TRIANGLES {
        let triangle_start = triangles.len();
        for primitive in primitives
            .get(start..end)
            .ok_or(CollisionError::GeometryBounds)?
        {
            triangles.push(primitive.triangle);
        }
        nodes
            .get_mut(node_index)
            .ok_or(CollisionError::GeometryBounds)?
            .kind = NodeKind::Leaf {
            start: triangle_start,
            count: end - start,
        };
        return Ok(node_index);
    }

    let center_bounds = primitives
        .get(start..end)
        .ok_or(CollisionError::GeometryBounds)?
        .iter()
        .map(|primitive| primitive.center)
        .reduce(|min, center| min.min(center))
        .ok_or(CollisionError::GeometryBounds)?;
    let center_max = primitives
        .get(start..end)
        .ok_or(CollisionError::GeometryBounds)?
        .iter()
        .map(|primitive| primitive.center)
        .reduce(|max, center| max.max(center))
        .ok_or(CollisionError::GeometryBounds)?;
    let extent = center_max - center_bounds;
    if !extent.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let axis = longest_axis(extent);
    primitives
        .get_mut(start..end)
        .ok_or(CollisionError::GeometryBounds)?
        .sort_by(|left, right| {
            component(left.center, axis)
                .total_cmp(&component(right.center, axis))
                .then(left.triangle.cmp(&right.triangle))
        });
    let middle = start + (end - start) / 2;
    let left = build_node(primitives, start, middle, nodes, triangles)?;
    let right = build_node(primitives, middle, end, nodes, triangles)?;
    nodes
        .get_mut(node_index)
        .ok_or(CollisionError::GeometryBounds)?
        .kind = NodeKind::Branch { left, right };
    Ok(node_index)
}

fn triangle_bounds(vertices: [Vec3; 3]) -> Result<Aabb, CollisionError> {
    let min = vertices[0].min(vertices[1]).min(vertices[2]);
    let max = vertices[0].max(vertices[1]).max(vertices[2]);
    Aabb::new(min, max).map_err(|_| CollisionError::GeometryBounds)
}

fn longest_axis(extent: Vec3) -> usize {
    if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    }
}

fn component(vector: Vec3, axis: usize) -> f64 {
    match axis {
        0 => vector.x,
        1 => vector.y,
        _ => vector.z,
    }
}

#[cfg(test)]
#[path = "mesh_tests.rs"]
mod tests;
