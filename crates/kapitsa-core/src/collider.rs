use crate::{aabb::Aabb, material::MaterialId, shape::Shape, transform::Transform};

pub mod error;
mod filter;
pub use error::ColliderError;
pub use filter::CollisionFilter;

/// Collision geometry positioned relative to its owner.
pub struct Collider {
    shape: Shape,
    local_transform: Transform,
    filter: CollisionFilter,
    material: Option<MaterialId>,
}

impl Collider {
    pub fn new(shape: Shape, local_transform: Transform) -> Self {
        Self {
            shape,
            local_transform,
            filter: CollisionFilter::default(),
            material: None,
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

    /// Returns the assigned material ID, which may be stale after its material is removed.
    ///
    /// Resolving the ID and choosing a fallback are responsibilities of the future contact system.
    pub fn material(&self) -> Option<MaterialId> {
        self.material
    }

    /// Assigns a material ID without checking that it exists in any material storage.
    pub fn set_material(&mut self, material: Option<MaterialId>) {
        self.material = material;
    }

    pub fn world_bounds(
        &self,
        parent_transform: &Transform,
    ) -> Result<Option<Aabb>, ColliderError> {
        let transform = parent_transform.compose(&self.local_transform)?;
        self.shape
            .world_bounds(&transform)
            .map_err(ColliderError::from)
    }
}

#[cfg(test)]
#[path = "collider_tests.rs"]
mod tests;
