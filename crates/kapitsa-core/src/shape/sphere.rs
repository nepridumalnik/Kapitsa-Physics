use super::ShapeError;
use crate::aabb::Aabb;
use glam::Vec3;

/// Sphere centered at the collider-local origin.
pub struct Sphere {
    radius: f32,
}

impl Sphere {
    pub fn new(radius: f32) -> Result<Self, ShapeError> {
        if !radius.is_finite() {
            return Err(ShapeError::NonFiniteParameter);
        }
        if radius <= 0.0 {
            return Err(ShapeError::InvalidDimension);
        }

        Ok(Self { radius })
    }

    pub fn radius(&self) -> f32 {
        self.radius
    }

    pub fn local_bounds(&self) -> Option<Aabb> {
        let extent = Vec3::splat(self.radius);
        Aabb::new(-extent, extent).ok()
    }
}
