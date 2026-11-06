use std::fmt;

use kapitsa_math::Vec3;

/// A point-mass gravitational field with gravitational parameter `μ = GM`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CentralGravity {
    center: Vec3,
    gravitational_parameter: f64,
}

impl CentralGravity {
    /// Creates a central field. A zero parameter is a valid disabled field.
    pub fn new(center: Vec3, gravitational_parameter: f64) -> Result<Self, GravityError> {
        if !center.is_finite() {
            return Err(GravityError::NonFiniteCenter);
        }
        if !gravitational_parameter.is_finite() || gravitational_parameter < 0.0 {
            return Err(GravityError::InvalidGravitationalParameter);
        }
        Ok(Self {
            center,
            gravitational_parameter,
        })
    }

    pub fn center(self) -> Vec3 {
        self.center
    }

    /// Returns `μ = GM` in m³/s².
    pub fn gravitational_parameter(self) -> f64 {
        self.gravitational_parameter
    }

    /// Evaluates acceleration at a world-space position in m/s².
    pub fn acceleration_at(self, position: Vec3) -> Result<Vec3, GravityError> {
        if !position.is_finite() {
            return Err(GravityError::NonFinitePosition);
        }
        if self.gravitational_parameter == 0.0 {
            return Ok(Vec3::ZERO);
        }

        let offset = position
            .checked_sub(self.center)
            .ok_or(GravityError::ArithmeticOverflow)?;
        let radius = offset.length();
        if radius == 0.0 {
            return Err(GravityError::AtFieldCenter);
        }
        if !radius.is_finite() {
            return Err(GravityError::ArithmeticOverflow);
        }
        let direction = offset
            .try_normalize()
            .ok_or(GravityError::ArithmeticOverflow)?;
        let magnitude = self.gravitational_parameter / radius / radius;
        direction
            .checked_mul_scalar(-magnitude)
            .ok_or(GravityError::ArithmeticOverflow)
    }
}

/// Invalid field parameters or an unrepresentable gravity calculation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GravityError {
    InvalidGravitationalParameter,
    NonFiniteCenter,
    NonFinitePosition,
    AtFieldCenter,
    ArithmeticOverflow,
    StorageCapacityExceeded,
}

impl fmt::Display for GravityError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidGravitationalParameter => {
                "gravitational parameter must be finite and non-negative"
            }
            Self::NonFiniteCenter => "gravity field center must be finite",
            Self::NonFinitePosition => "gravity evaluation position must be finite",
            Self::AtFieldCenter => "gravity is undefined at the field center",
            Self::ArithmeticOverflow => "gravity calculation exceeds the finite f64 range",
            Self::StorageCapacityExceeded => "gravity staging capacity could not be reserved",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for GravityError {}

#[cfg(test)]
mod tests {
    use kapitsa_math::Vec3;

    use super::{CentralGravity, GravityError};

    #[test]
    fn evaluates_inward_inverse_square_acceleration() {
        let gravity = CentralGravity::new(Vec3::ZERO, 100.0).expect("valid field");
        assert_eq!(
            gravity.acceleration_at(Vec3::new(10.0, 0.0, 0.0)),
            Ok(Vec3::new(-1.0, 0.0, 0.0))
        );
        assert_eq!(
            gravity.acceleration_at(Vec3::new(0.0, -10.0, 0.0)),
            Ok(Vec3::new(0.0, 1.0, 0.0))
        );
    }

    #[test]
    fn validates_parameters_and_handles_singular_and_overflow_positions() {
        assert_eq!(
            CentralGravity::new(Vec3::ZERO, f64::NAN),
            Err(GravityError::InvalidGravitationalParameter)
        );
        assert_eq!(
            CentralGravity::new(Vec3::ZERO, f64::INFINITY),
            Err(GravityError::InvalidGravitationalParameter)
        );
        assert_eq!(
            CentralGravity::new(Vec3::ZERO, -1.0),
            Err(GravityError::InvalidGravitationalParameter)
        );
        assert_eq!(
            CentralGravity::new(Vec3::new(f64::NAN, 0.0, 0.0), 1.0),
            Err(GravityError::NonFiniteCenter)
        );
        let gravity = CentralGravity::new(Vec3::ZERO, 1.0).expect("valid field");
        assert_eq!(
            gravity.acceleration_at(Vec3::ZERO),
            Err(GravityError::AtFieldCenter)
        );
        assert_eq!(
            gravity.acceleration_at(Vec3::new(f64::INFINITY, 0.0, 0.0)),
            Err(GravityError::NonFinitePosition)
        );
        let strong = CentralGravity::new(Vec3::ZERO, f64::MAX).expect("valid field");
        assert_eq!(
            strong.acceleration_at(Vec3::new(1.0e-200, 0.0, 0.0)),
            Err(GravityError::ArithmeticOverflow)
        );
    }
}
