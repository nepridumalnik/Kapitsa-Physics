use super::GravityError;
use kapitsa_math::Vec3;

/// Constant acceleration throughout the world, in m/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UniformGravity {
    acceleration: Vec3,
}

impl UniformGravity {
    pub fn new(acceleration: Vec3) -> Result<Self, GravityError> {
        if !acceleration.is_finite() {
            return Err(GravityError::InvalidAcceleration);
        }
        Ok(Self { acceleration })
    }

    pub fn acceleration(self) -> Vec3 {
        self.acceleration
    }

    pub fn acceleration_at(self, position: Vec3) -> Result<Vec3, GravityError> {
        if !position.is_finite() {
            return Err(GravityError::NonFinitePosition);
        }
        Ok(self.acceleration)
    }
}
