use crate::aabb::Aabb;
use glam::Vec3;

use super::{BoundsError, ShapeError};

/// Axis-aligned cuboid centered at the collider-local origin.
pub struct Cuboid {
    size: Vec3,
}

impl Cuboid {
    pub fn new(x: f32, y: f32, z: f32) -> Result<Self, ShapeError> {
        Self::from_vec(Vec3::new(x, y, z))
    }

    pub fn from_vec(size: Vec3) -> Result<Self, ShapeError> {
        if !size.is_finite() {
            return Err(ShapeError::NonFiniteParameter);
        }
        if size.min_element() <= 0.0 {
            return Err(ShapeError::InvalidDimension);
        }

        Ok(Self { size })
    }

    pub fn size(&self) -> Vec3 {
        self.size
    }

    pub fn half_extents(&self) -> Vec3 {
        self.size * 0.5
    }

    pub fn local_bounds(&self) -> Result<Aabb, BoundsError> {
        let half = self.half_extents();
        Aabb::from_f32_bounds(-half, half).map_err(BoundsError::InvalidBounds)
    }
}
