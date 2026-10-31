use crate::{
    aabb::Aabb,
    collision::{
        CollisionError,
        mesh::{BvhRefitScratch, TriangleBvh},
    },
    material::MaterialId,
    shape::{BoundsError, Shape, SkinnedTriangleMesh},
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
    staging_vertices: Vec<Vec3>,
    bounds: Aabb,
    bvh: Box<TriangleBvh>,
    refit_scratch: BvhRefitScratch,
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
    /// `PhysicsWorld` validates the ID on insertion and before stepping. Prefer
    /// [`PhysicsWorld::set_collider_material`](crate::world::PhysicsWorld::set_collider_material)
    /// when assigning a material to a collider in a world.
    pub fn material(&self) -> Option<MaterialId> {
        self.material
    }

    /// Assigns a material while preparing a collider that is not yet attached to a world.
    ///
    /// [`PhysicsWorld::add_collider`](crate::world::PhysicsWorld::add_collider) validates this
    /// assignment. For an attached collider, use
    /// [`PhysicsWorld::set_collider_material`](crate::world::PhysicsWorld::set_collider_material)
    /// so the material is checked against its world and contact caches are invalidated.
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
            return pose
                .bounds
                .transformed(&transform)
                .map(Some)
                .map_err(|error| ColliderError::Bounds(BoundsError::from(error)));
        }
        self.shape
            .world_bounds(&transform)
            .map_err(ColliderError::from)
    }

    pub(crate) fn local_bounds(&self) -> Result<Option<Aabb>, CollisionError> {
        if let Some(pose) = &self.skin_pose {
            return Ok(Some(pose.bounds));
        }
        self.shape
            .local_bounds()
            .map_err(|_| CollisionError::GeometryBounds)
    }

    /// Sets the skin pose used for collision during subsequent physics steps.
    ///
    /// Physics treats each pose as fixed for the duration of a step. Changes
    /// between steps are discrete; CCD does not interpolate unknown bone paths.
    pub(crate) fn set_skin_pose(&mut self, matrices: &[Affine3]) -> Result<(), CollisionError> {
        let Shape::SkinnedTriangleMesh(skinned_mesh) = &self.shape else {
            return Err(CollisionError::InvalidSkinPose);
        };
        if matrices.len() != skinned_mesh.bone_names().len()
            || matrices.iter().any(|matrix| !matrix.is_finite())
        {
            return Err(CollisionError::InvalidSkinPose);
        }

        if let Some(pose) = &mut self.skin_pose {
            fill_deformed_vertices(skinned_mesh, matrices, &mut pose.staging_vertices)?;
            let bounds = points_bounds(&pose.staging_vertices)
                .map_err(|_| CollisionError::InvalidSkinPose)?;
            pose.bvh.prepare_refit(
                skinned_mesh.mesh(),
                &pose.staging_vertices,
                &mut pose.refit_scratch,
            )?;
            pose.bvh.commit_refit(&mut pose.refit_scratch);
            std::mem::swap(&mut pose.vertices, &mut pose.staging_vertices);
            pose.bounds = bounds;
            return Ok(());
        }

        let vertex_count = skinned_mesh.mesh().vertices().len();
        let mut vertices = Vec::new();
        vertices
            .try_reserve_exact(vertex_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let mut staging_vertices = Vec::new();
        staging_vertices
            .try_reserve_exact(vertex_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        fill_deformed_vertices(skinned_mesh, matrices, &mut staging_vertices)?;
        let bounds =
            points_bounds(&staging_vertices).map_err(|_| CollisionError::InvalidSkinPose)?;
        let mut refit_scratch = BvhRefitScratch::default();
        let bvh = if let Some(bvh) = self.mesh_bvh.as_deref() {
            bvh.prepare_refit(skinned_mesh.mesh(), &staging_vertices, &mut refit_scratch)?;
            let Some(mut bvh) = self.mesh_bvh.take() else {
                return Err(CollisionError::InvalidContact);
            };
            bvh.commit_refit(&mut refit_scratch);
            bvh
        } else {
            let mut bvh = Box::new(TriangleBvh::build(skinned_mesh.mesh())?);
            bvh.prepare_refit(skinned_mesh.mesh(), &staging_vertices, &mut refit_scratch)?;
            bvh.commit_refit(&mut refit_scratch);
            bvh
        };
        std::mem::swap(&mut vertices, &mut staging_vertices);
        self.skin_pose = Some(SkinPose {
            vertices,
            staging_vertices,
            bounds,
            bvh,
            refit_scratch,
        });
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
            .map(|pose| pose.bvh.as_ref())
            .or(self.mesh_bvh.as_deref())
    }

    pub(crate) fn query_vertices(&self) -> Option<&[Vec3]> {
        self.skin_pose.as_ref().map(|pose| pose.vertices.as_slice())
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn skin_pose_retained_capacity_bytes(&self) -> usize {
        self.skin_pose.as_ref().map_or(0, |pose| {
            std::mem::size_of::<SkinPose>()
                + pose.vertices.capacity() * std::mem::size_of::<Vec3>()
                + pose.staging_vertices.capacity() * std::mem::size_of::<Vec3>()
                + pose.bvh.retained_capacity_bytes()
                + pose.refit_scratch.retained_capacity_bytes()
        })
    }

    #[allow(dead_code)]
    pub(crate) fn skinned_vertices(&self) -> Option<&[Vec3]> {
        self.skin_pose.as_ref().map(|pose| pose.vertices.as_slice())
    }

    #[allow(dead_code)]
    pub(crate) fn skinned_bvh(&self) -> Option<&TriangleBvh> {
        self.skin_pose.as_ref().map(|pose| pose.bvh.as_ref())
    }
}

fn fill_deformed_vertices(
    skinned_mesh: &SkinnedTriangleMesh,
    matrices: &[Affine3],
    output: &mut Vec<Vec3>,
) -> Result<(), CollisionError> {
    let reference_vertices = skinned_mesh.mesh().vertices();
    let influences = skinned_mesh.vertex_influences();
    if reference_vertices.len() != influences.len() {
        return Err(CollisionError::InvalidSkinPose);
    }
    output.clear();
    output
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
        output.push(deformed);
    }
    Ok(())
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
