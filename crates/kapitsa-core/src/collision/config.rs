use super::CollisionError;

/// Numerical and iteration limits for collision processing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionConfig {
    pub velocity_iterations: u32,
    pub position_iterations: u32,
    pub contact_offset: f64,
    pub penetration_slop: f64,
    pub restitution_threshold: f64,
    pub max_ccd_impacts: u32,
}

impl CollisionConfig {
    pub(crate) const fn default_const() -> Self {
        Self {
            velocity_iterations: 8,
            position_iterations: 3,
            contact_offset: 0.01,
            penetration_slop: 0.001,
            restitution_threshold: 1.0,
            max_ccd_impacts: 8,
        }
    }

    pub(crate) fn validate(self) -> Result<(), CollisionError> {
        if self.velocity_iterations == 0
            || self.position_iterations == 0
            || self.max_ccd_impacts == 0
            || !self.contact_offset.is_finite()
            || !self.penetration_slop.is_finite()
            || !self.restitution_threshold.is_finite()
            || self.contact_offset < 0.0
            || self.penetration_slop < 0.0
            || self.restitution_threshold < 0.0
            || self.penetration_slop > self.contact_offset
        {
            return Err(CollisionError::InvalidConfig);
        }
        Ok(())
    }
}

impl Default for CollisionConfig {
    fn default() -> Self {
        Self::default_const()
    }
}
