use super::CollisionError;

/// Numerical and iteration limits for collision processing.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionConfig {
    pub(crate) velocity_iterations: u32,
    pub(crate) position_iterations: u32,
    pub(crate) contact_offset: f64,
    pub(crate) penetration_slop: f64,
    pub(crate) restitution_threshold: f64,
    pub(crate) max_ccd_impacts: u32,
    pub(crate) ccd_limit_policy: CcdLimitPolicy,
}

/// Action taken when a continuous body reaches its per-step impact budget.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CcdLimitPolicy {
    /// Compatibility name: stop only dynamic participants in the unresolved contact.
    #[default]
    ClipIsland,
    /// Abort the step and preserve the last committed world state.
    AbortStep,
}

/// Builder for [`CollisionConfig`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CollisionConfigBuilder {
    config: CollisionConfig,
}

impl CollisionConfigBuilder {
    pub fn velocity_iterations(mut self, value: u32) -> Self {
        self.config.velocity_iterations = value;
        self
    }

    pub fn position_iterations(mut self, value: u32) -> Self {
        self.config.position_iterations = value;
        self
    }

    pub fn contact_offset(mut self, value: f64) -> Self {
        self.config.contact_offset = value;
        self
    }

    pub fn penetration_slop(mut self, value: f64) -> Self {
        self.config.penetration_slop = value;
        self
    }

    pub fn restitution_threshold(mut self, value: f64) -> Self {
        self.config.restitution_threshold = value;
        self
    }

    pub fn max_ccd_impacts(mut self, value: u32) -> Self {
        self.config.max_ccd_impacts = value;
        self
    }

    pub fn ccd_limit_policy(mut self, value: CcdLimitPolicy) -> Self {
        self.config.ccd_limit_policy = value;
        self
    }

    pub fn build(self) -> CollisionConfig {
        self.config
    }
}

impl CollisionConfig {
    pub fn builder() -> CollisionConfigBuilder {
        CollisionConfigBuilder::default()
    }

    pub fn velocity_iterations(&self) -> u32 {
        self.velocity_iterations
    }

    pub fn position_iterations(&self) -> u32 {
        self.position_iterations
    }

    pub fn contact_offset(&self) -> f64 {
        self.contact_offset
    }

    pub fn penetration_slop(&self) -> f64 {
        self.penetration_slop
    }

    pub fn restitution_threshold(&self) -> f64 {
        self.restitution_threshold
    }

    pub fn max_ccd_impacts(&self) -> u32 {
        self.max_ccd_impacts
    }

    pub fn ccd_limit_policy(&self) -> CcdLimitPolicy {
        self.ccd_limit_policy
    }

    pub(crate) const fn default_const() -> Self {
        Self {
            velocity_iterations: 8,
            position_iterations: 3,
            contact_offset: 0.01,
            penetration_slop: 0.001,
            restitution_threshold: 1.0,
            max_ccd_impacts: 8,
            ccd_limit_policy: CcdLimitPolicy::ClipIsland,
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
