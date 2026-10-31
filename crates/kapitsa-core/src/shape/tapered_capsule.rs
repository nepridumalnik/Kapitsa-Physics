use crate::aabb::Aabb;
use glam::Vec3;

use super::{BoundsError, ShapeError};

/// Convex hull of two spheres at local-space endpoints.
/// Neither sphere may fully contain the other.
pub struct TaperedCapsule {
    start: Vec3,
    end: Vec3,
    start_radius: f32,
    end_radius: f32,
}

impl TaperedCapsule {
    pub fn new(
        start: Vec3,
        end: Vec3,
        start_radius: f32,
        end_radius: f32,
    ) -> Result<Self, ShapeError> {
        if !start.is_finite()
            || !end.is_finite()
            || !start_radius.is_finite()
            || !end_radius.is_finite()
        {
            return Err(ShapeError::NonFiniteParameter);
        }

        if start_radius <= 0.0 || end_radius <= 0.0 {
            return Err(ShapeError::InvalidDimension);
        }

        let length = (end - start).length();
        let radius_difference = (start_radius - end_radius).abs();
        if !length.is_finite() || !radius_difference.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }
        if length <= 0.0 || radius_difference >= length {
            return Err(ShapeError::DegenerateGeometry);
        }

        Ok(Self {
            start,
            end,
            start_radius,
            end_radius,
        })
    }

    pub fn start(&self) -> Vec3 {
        self.start
    }

    pub fn end(&self) -> Vec3 {
        self.end
    }

    pub fn start_radius(&self) -> f32 {
        self.start_radius
    }

    pub fn end_radius(&self) -> f32 {
        self.end_radius
    }

    pub fn local_bounds(&self) -> Result<Aabb, BoundsError> {
        let start_radius = Vec3::splat(self.start_radius);
        let end_radius = Vec3::splat(self.end_radius);
        Aabb::new(
            (self.start - start_radius).min(self.end - end_radius),
            (self.start + start_radius).max(self.end + end_radius),
        )
        .map_err(BoundsError::InvalidBounds)
    }
}
