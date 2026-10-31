use crate::aabb::Aabb;
use glam::Vec3;

use super::{BoundsError, ShapeError};

/// Capsule swept along a segment in collider-local coordinates.
pub struct Capsule {
    start: Vec3,
    end: Vec3,
    radius: f32,
}

impl Capsule {
    pub fn new(start: Vec3, end: Vec3, radius: f32) -> Result<Self, ShapeError> {
        if !start.is_finite() || !end.is_finite() || !radius.is_finite() {
            return Err(ShapeError::NonFiniteParameter);
        }

        let length = (end - start).length();
        if !length.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }
        if length <= 0.0 {
            return Err(ShapeError::DegenerateGeometry);
        }
        if radius <= 0.0 {
            return Err(ShapeError::InvalidDimension);
        }

        Ok(Self { start, end, radius })
    }

    pub fn start(&self) -> Vec3 {
        self.start
    }

    pub fn end(&self) -> Vec3 {
        self.end
    }

    pub fn radius(&self) -> f32 {
        self.radius
    }

    pub fn local_bounds(&self) -> Result<Aabb, BoundsError> {
        let radius = Vec3::splat(self.radius);

        Aabb::from_f32_bounds(
            self.start.min(self.end) - radius,
            self.start.max(self.end) + radius,
        )
        .map_err(BoundsError::InvalidBounds)
    }
}
