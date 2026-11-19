use super::GravityError;
use kapitsa_math::Vec3;

/// Constant acceleration throughout the world, in m/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UniformGravity {
    acceleration: Vec3,
}

impl UniformGravity {
    pub fn new(acceleration: Vec3) -> Result<Self, crate::Error> {
        if !acceleration.is_finite() {
            return Err(crate::Error::from(GravityError::InvalidAcceleration));
        }
        Ok(Self { acceleration })
    }

    pub fn acceleration(self) -> Vec3 {
        self.acceleration
    }

    pub fn acceleration_at(self, position: Vec3) -> Result<Vec3, crate::Error> {
        if !position.is_finite() {
            return Err(crate::Error::from(GravityError::NonFinitePosition));
        }
        Ok(self.acceleration)
    }
}
