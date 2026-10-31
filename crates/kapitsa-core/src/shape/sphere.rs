use super::{BoundsError, ShapeError};
use crate::aabb::Aabb;
use kapitsa_math::Vec3;

/// Sphere centered at the collider-local origin.
pub struct Sphere {
    radius: f64,
}

impl Sphere {
    pub fn new(radius: f64) -> Result<Self, ShapeError> {
        if !radius.is_finite() {
            return Err(ShapeError::NonFiniteParameter);
        }
        if radius <= 0.0 {
            return Err(ShapeError::InvalidDimension);
        }

        Ok(Self { radius })
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }

    pub fn local_bounds(&self) -> Result<Aabb, BoundsError> {
        let extent = Vec3::splat(self.radius);
        Aabb::new(-extent, extent).map_err(BoundsError::InvalidBounds)
    }
}
