use crate::{
    aabb::Aabb,
    collision::{CollisionError, mesh::TriangleBvh},
    material::MaterialId,
    shape::{BoundsError, Shape},
    transform::Transform,
};
use kapitsa_math::{Affine3, Vec3};

pub mod error;
mod filter;
pub use error::ColliderError;
pub use filter::CollisionFilter;

/// Whether a collider responds physically or only reports intersections.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionMode {
    Solid,
    Sensor,
}

/// Collision geometry positioned relative to its owner.
pub struct Collider {
    shape: Shape,
    local_transform: Transform,
    filter: CollisionFilter,
    material: Option<MaterialId>,
    mode: CollisionMode,
    skin_pose: Option<SkinPose>,
    mesh_bvh: Option<Box<TriangleBvh>>,
}

struct SkinPose {
    vertices: Vec<Vec3>,
    bvh: TriangleBvh,
}

impl Collider {
    pub fn new(shape: Shape, local_transform: Transform) -> Self {
        Self {
            shape,
            local_transform,
            filter: CollisionFilter::default(),
            material: None,
            mode: CollisionMode::Solid,
            skin_pose: None,
            mesh_bvh: None,
        }
    }

    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    pub fn local_transform(&self) -> Transform {
        self.local_transform
    }

    pub fn filter(&self) -> CollisionFilter {
        self.filter
    }

    pub fn set_filter(&mut self, filter: CollisionFilter) {
        self.filter = filter;
    }

    /// Returns the assigned material ID.
    ///
    /// `PhysicsWorld` validates the ID before stepping. Prefer
    /// [`PhysicsWorld::set_collider_material`](crate::world::PhysicsWorld::set_collider_material)
    /// when assigning a material to a collider in a world.
    pub fn material(&self) -> Option<MaterialId> {
        self.material
    }

    /// Stores a material ID without checking that it exists in a world.
    ///
    /// Prefer [`PhysicsWorld::set_collider_material`](crate::world::PhysicsWorld::set_collider_material)
    /// for validated assignments.
    pub fn set_material(&mut self, material: Option<MaterialId>) {
        self.material = material;
    }

    pub fn mode(&self) -> CollisionMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: CollisionMode) {
        self.mode = mode;
    }

    pub fn world_bounds(
        &self,
        parent_transform: &Transform,
    ) -> Result<Option<Aabb>, ColliderError> {
        let transform = parent_transform.compose(&self.local_transform)?;
        if let Some(pose) = &self.skin_pose {
            return points_bounds(&pose.vertices)?
                .transformed(&transform)
                .map(Some)
                .map_err(|error| ColliderError::Bounds(BoundsError::from(error)));
        }
        self.shape
            .world_bounds(&transform)
            .map_err(ColliderError::from)
    }

    pub(crate) fn set_skin_pose(&mut self, matrices: &[Affine3]) -> Result<(), CollisionError> {
        let Shape::SkinnedTriangleMesh(skinned_mesh) = &self.shape else {
            return Err(CollisionError::InvalidSkinPose);
        };
        if matrices.len() != skinned_mesh.bone_names().len()
            || matrices.iter().any(|matrix| !matrix.is_finite())
        {
            return Err(CollisionError::InvalidSkinPose);
        }

        let reference_vertices = skinned_mesh.mesh().vertices();
        let influences = skinned_mesh.vertex_influences();
        if reference_vertices.len() != influences.len() {
            return Err(CollisionError::InvalidSkinPose);
        }
        let mut vertices = Vec::new();
        vertices
            .try_reserve_exact(reference_vertices.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for (reference, influences) in reference_vertices.iter().zip(influences) {
            let mut deformed = Vec3::ZERO;
            for influence in influences {
                let matrix = matrices
                    .get(influence.bone_index())
                    .ok_or(CollisionError::InvalidSkinPose)?;
                let transformed = matrix
                    .linear()
                    .checked_mul_vector(*reference)
                    .and_then(|point| point.checked_add(matrix.translation()))
                    .ok_or(CollisionError::ArithmeticOverflow)?;
                let weighted = transformed
                    .checked_mul_scalar(influence.weight())
                    .ok_or(CollisionError::ArithmeticOverflow)?;
                deformed = deformed
                    .checked_add(weighted)
                    .ok_or(CollisionError::ArithmeticOverflow)?;
            }
            if !deformed.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            vertices.push(deformed);
        }
        points_bounds(&vertices).map_err(|_| CollisionError::InvalidSkinPose)?;

        if let Some(pose) = &mut self.skin_pose {
            pose.bvh.refit_vertices(skinned_mesh.mesh(), &vertices)?;
            pose.vertices = vertices;
        } else {
            let mut bvh = TriangleBvh::build(skinned_mesh.mesh())?;
            bvh.refit_vertices(skinned_mesh.mesh(), &vertices)?;
            self.skin_pose = Some(SkinPose { vertices, bvh });
        }
        Ok(())
    }

    pub(crate) fn prepare_query_geometry(&mut self) -> Result<(), CollisionError> {
        let mesh = match &self.shape {
            Shape::TriangleMesh(mesh) => Some(mesh),
            Shape::SkinnedTriangleMesh(mesh) => Some(mesh.mesh()),
            _ => None,
        };
        if let Some(mesh) = mesh
            && self.mesh_bvh.is_none()
        {
            self.mesh_bvh = Some(Box::new(TriangleBvh::build(mesh)?));
        }
        Ok(())
    }

    pub(crate) fn query_bvh(&self) -> Option<&TriangleBvh> {
        self.skin_pose
            .as_ref()
            .map(|pose| &pose.bvh)
            .or(self.mesh_bvh.as_deref())
    }

    pub(crate) fn query_vertices(&self) -> Option<&[Vec3]> {
        self.skin_pose.as_ref().map(|pose| pose.vertices.as_slice())
    }

    #[allow(dead_code)]
    pub(crate) fn skinned_vertices(&self) -> Option<&[Vec3]> {
        self.skin_pose.as_ref().map(|pose| pose.vertices.as_slice())
    }

    #[allow(dead_code)]
    pub(crate) fn skinned_bvh(&self) -> Option<&TriangleBvh> {
        self.skin_pose.as_ref().map(|pose| &pose.bvh)
    }
}

fn points_bounds(points: &[Vec3]) -> Result<Aabb, BoundsError> {
    let (&first, rest) = points.split_first().ok_or(BoundsError::InvalidGeometry)?;
    let (min, max) = rest.iter().fold((first, first), |(min, max), &point| {
        (min.min(point), max.max(point))
    });
    Aabb::new(min, max).map_err(BoundsError::from)
}

#[cfg(test)]
#[path = "collider_tests.rs"]
mod tests;
