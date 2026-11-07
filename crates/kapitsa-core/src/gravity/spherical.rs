use super::{CentralGravity, GravityError};
use kapitsa_math::Vec3;

/// A central field limited to a spherical region.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphericalGravity {
    field: CentralGravity,
    radius: f64,
}

impl SphericalGravity {
    pub fn new(
        center: Vec3,
        gravitational_parameter: f64,
        radius: f64,
    ) -> Result<Self, GravityError> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(GravityError::InvalidRadius);
        }
        Ok(Self {
            field: CentralGravity::new(center, gravitational_parameter)?,
            radius,
        })
    }

    pub fn center(self) -> Vec3 {
        self.field.center()
    }
    pub fn gravitational_parameter(self) -> f64 {
        self.field.gravitational_parameter()
    }
    pub fn radius(self) -> f64 {
        self.radius
    }

    pub fn acceleration_at(self, position: Vec3) -> Result<Vec3, GravityError> {
        if !position.is_finite() {
            return Err(GravityError::NonFinitePosition);
        }
        let offset = position
            .checked_sub(self.center())
            .ok_or(GravityError::ArithmeticOverflow)?;
        if offset.length() > self.radius {
            return Ok(Vec3::ZERO);
        }
        self.field.acceleration_at(position)
    }
}
