use crate::{aabb::Aabb, shape::Shape, transform::Transform};

pub mod error;
pub use error::ColliderError;

/// Collision geometry positioned relative to its owner.
pub struct Collider {
    shape: Shape,
    local_transform: Transform,
}

impl Collider {
    pub fn new(shape: Shape, local_transform: Transform) -> Self {
        Self {
            shape,
            local_transform,
        }
    }

    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    pub fn local_transform(&self) -> Transform {
        self.local_transform
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
