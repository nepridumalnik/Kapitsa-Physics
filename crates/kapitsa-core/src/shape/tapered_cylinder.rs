use crate::aabb::Aabb;
use kapitsa_math::Vec3;

use super::{BoundsError, ShapeError};

/// Conical frustum with positive radii at both ends of its local axis.
pub struct TaperedCylinder {
    start: Vec3,
    end: Vec3,
    start_radius: f64,
    end_radius: f64,
}

impl TaperedCylinder {
    pub fn new(
        start: Vec3,
        end: Vec3,
        start_radius: f64,
        end_radius: f64,
    ) -> Result<Self, ShapeError> {
        if !start.is_finite()
            || !end.is_finite()
            || !start_radius.is_finite()
            || !end_radius.is_finite()
        {
            return Err(ShapeError::NonFiniteParameter);
        }

        let length = (end - start).length();
        if !length.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }
        if length <= 0.0 {
            return Err(ShapeError::DegenerateGeometry);
        }
        if start_radius <= 0.0 || end_radius <= 0.0 {
            return Err(ShapeError::InvalidDimension);
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

    pub fn start_radius(&self) -> f64 {
        self.start_radius
    }

    pub fn end_radius(&self) -> f64 {
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
