use crate::aabb::Aabb;
use kapitsa_math::Vec3;

use super::ShapeError;

/// Axis-aligned cuboid centered at the collider-local origin.
pub struct Cuboid {
    size: Vec3,
}

impl Cuboid {
    pub fn new(x: f64, y: f64, z: f64) -> Result<Self, crate::Error> {
        Self::from_vec(Vec3::new(x, y, z))
    }

    pub fn from_vec(size: Vec3) -> Result<Self, crate::Error> {
        if !size.is_finite() {
            return Err(crate::Error::from(ShapeError::NonFiniteParameter));
        }
        if size.min_element() <= 0.0 {
            return Err(crate::Error::from(ShapeError::InvalidDimension));
        }

        Ok(Self { size })
    }

    pub fn size(&self) -> Vec3 {
        self.size
    }

    pub fn half_extents(&self) -> Vec3 {
        self.size * 0.5
    }

    pub fn local_bounds(&self) -> Result<Aabb, crate::Error> {
        let half = self.half_extents();
        Aabb::new(-half, half)
    }
}
