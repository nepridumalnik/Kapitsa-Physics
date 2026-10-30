pub mod error;
mod storage;

pub use error::PhysicsMaterialError;
pub use storage::{MaterialId, MaterialStorage};

/// Surface friction, restitution, and density properties for a physics material.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicsMaterial {
    friction: f64,
    static_friction: f64,
    restitution: f64,
    density: f64,
}

impl PhysicsMaterial {
    pub fn new(
        friction: f64,
        static_friction: f64,
        restitution: f64,
        density: f64,
    ) -> Result<Self, PhysicsMaterialError> {
        if !friction.is_finite() {
            return Err(PhysicsMaterialError::NonFiniteFriction);
        }
        if !static_friction.is_finite() {
            return Err(PhysicsMaterialError::NonFiniteStaticFriction);
        }
        if !restitution.is_finite() {
            return Err(PhysicsMaterialError::NonFiniteRestitution);
        }
        if !density.is_finite() {
            return Err(PhysicsMaterialError::NonFiniteDensity);
        }
        if friction < 0.0 {
            return Err(PhysicsMaterialError::NegativeFriction);
        }
        if static_friction < 0.0 {
            return Err(PhysicsMaterialError::NegativeStaticFriction);
        }
        if !(0.0..=1.0).contains(&restitution) {
            return Err(PhysicsMaterialError::RestitutionOutOfRange);
        }
        if density <= 0.0 {
            return Err(PhysicsMaterialError::NonPositiveDensity);
        }

        Ok(Self {
            friction,
            static_friction,
            restitution,
            density,
        })
    }

    pub fn friction(&self) -> f64 {
        self.friction
    }

    pub fn static_friction(&self) -> f64 {
        self.static_friction
    }

    pub fn restitution(&self) -> f64 {
        self.restitution
    }

    pub fn density(&self) -> f64 {
        self.density
    }
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            friction: 0.5,
            static_friction: 0.0,
            restitution: 0.1,
            density: 1.0,
        }
    }
}

#[cfg(test)]
#[path = "material_tests.rs"]
mod tests;
