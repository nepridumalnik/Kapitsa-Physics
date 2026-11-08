//! Preparation of nonempty groups of static triangle-mesh colliders.

use std::fmt;

use crate::{
    aabb::Aabb,
    collider::Collider,
    collision::CollisionError,
    shape::{BoundsError, Shape, TriangleMesh},
    transform::Transform,
};

/// A prepared, nonempty group of colliders sharing a body-local transform.
pub struct Chunk {
    pub(crate) transform: Transform,
    pub(crate) bounds: Aabb,
    pub(crate) colliders: Vec<Collider>,
}

impl Chunk {
    /// Returns the prepared meshes without exposing mutable geometry.
    pub fn colliders(&self) -> &[Collider] {
        &self.colliders
    }

    /// Returns the bounds in chunk-local coordinates, before its transform.
    pub fn local_bounds(&self) -> Aabb {
        self.bounds
    }

    /// Returns the chunk's transform relative to its future body.
    pub fn transform(&self) -> Transform {
        self.transform
    }

    /// Returns the prepared colliders, preserving their BVHs.
    pub fn into_colliders(self) -> Vec<Collider> {
        self.colliders
    }
}

/// Incrementally collects meshes for a single chunk.
#[derive(Default)]
pub struct ChunkBuilder {
    meshes: Vec<TriangleMesh>,
    bounds: Option<Aabb>,
}

/// Failure to collect or prepare chunk geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkBuildError {
    Empty,
    Bounds(BoundsError),
    Preparation(CollisionError),
    CapacityExceeded,
}

impl fmt::Display for ChunkBuildError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => formatter.write_str("a chunk must contain at least one mesh"),
            Self::Bounds(error) => write!(formatter, "invalid mesh bounds: {error}"),
            Self::Preparation(error) => write!(formatter, "mesh preparation failed: {error}"),
            Self::CapacityExceeded => formatter.write_str("chunk capacity could not be reserved"),
        }
    }
}

impl std::error::Error for ChunkBuildError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Bounds(error) => Some(error),
            Self::Preparation(error) => Some(error),
            Self::Empty | Self::CapacityExceeded => None,
        }
    }
}

impl ChunkBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_mesh(&mut self, mesh: TriangleMesh) -> Result<(), ChunkBuildError> {
        let bounds = mesh.local_bounds().map_err(ChunkBuildError::Bounds)?;
        self.meshes
            .try_reserve(1)
            .map_err(|_| ChunkBuildError::CapacityExceeded)?;
        self.meshes.push(mesh);
        self.bounds = Some(self.bounds.map_or(bounds, |current| current.union(&bounds)));
        Ok(())
    }

    /// Builds the BVH of each mesh synchronously, independently of a world.
    pub fn build(self, transform: Transform) -> Result<Chunk, ChunkBuildError> {
        let bounds = self.bounds.ok_or(ChunkBuildError::Empty)?;
        let mut colliders = Vec::new();
        colliders
            .try_reserve_exact(self.meshes.len())
            .map_err(|_| ChunkBuildError::CapacityExceeded)?;
        for mesh in self.meshes {
            let mut collider = Collider::new(Shape::TriangleMesh(mesh), transform);
            collider.prepare().map_err(ChunkBuildError::Preparation)?;
            colliders.push(collider);
        }
        Ok(Chunk {
            transform,
            bounds,
            colliders,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_math::Vec3;
    use std::error::Error;

    #[test]
    fn rejects_empty_chunks() {
        assert!(matches!(
            ChunkBuilder::new().build(Transform::identity()),
            Err(ChunkBuildError::Empty)
        ));
    }

    #[test]
    fn prepares_all_meshes_and_combines_local_bounds() -> Result<(), Box<dyn Error>> {
        let mut builder = ChunkBuilder::new();
        builder.add_mesh(TriangleMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            vec![[0, 1, 2]],
        )?)?;
        builder.add_mesh(TriangleMesh::new(
            vec![Vec3::Z, Vec3::X + Vec3::Z, Vec3::Y + Vec3::Z],
            vec![[0, 1, 2]],
        )?)?;
        let chunk = builder.build(Transform::identity())?;
        assert_eq!(chunk.local_bounds().min(), Vec3::ZERO);
        assert_eq!(chunk.local_bounds().max(), Vec3::new(1.0, 1.0, 1.0));
        let colliders = chunk.into_colliders();
        assert_eq!(colliders.len(), 2);
        assert!(
            colliders
                .iter()
                .all(|collider| collider.query_bvh().is_some())
        );
        Ok(())
    }

    #[test]
    fn prepared_mesh_survives_world_registration_and_removal() -> Result<(), Box<dyn Error>> {
        let mut builder = ChunkBuilder::new();
        builder.add_mesh(TriangleMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            vec![[0, 1, 2]],
        )?)?;
        let mut colliders = builder.build(Transform::identity())?.into_colliders();
        let collider = colliders.pop().ok_or("missing collider")?;
        let original = collider.query_bvh().ok_or("missing BVH")? as *const _;
        let mut world = crate::world::PhysicsWorld::new();
        let body = world.add_body(crate::rigid_body::RigidBody::new_static(
            Transform::identity(),
        ))?;
        let id = world.add_collider(body, collider)?;
        let registered = world.collider(id).ok_or("missing registered collider")?;
        assert_eq!(
            original,
            registered.query_bvh().ok_or("missing BVH")? as *const _
        );
        let collider = world.remove_collider(id).ok_or("removal failed")?;
        assert_eq!(
            original,
            collider.query_bvh().ok_or("missing BVH")? as *const _
        );
        Ok(())
    }
}
