use glam::{Quat, Vec3};

pub mod error;
pub use error::TransformError;

/// Rigid transformation consisting of translation and rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    translation: Vec3,
    rotation: Quat,
}

impl Transform {
    pub fn new(translation: Vec3, rotation: Quat) -> Result<Self, TransformError> {
        if !translation.is_finite() {
            return Err(TransformError::NonFiniteTranslation);
        }
        if !rotation.is_finite() || !rotation.is_normalized() {
            return Err(TransformError::InvalidRotation);
        }

        Ok(Self {
            translation,
            rotation,
        })
    }

    pub const fn identity() -> Self {
        Self {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        }
    }

    pub fn translation(&self) -> Vec3 {
        self.translation
    }

    pub fn rotation(&self) -> Quat {
        self.rotation
    }

    pub fn transform_point(&self, point: Vec3) -> Vec3 {
        self.rotation * point + self.translation
    }

    pub fn transform_vector(&self, vector: Vec3) -> Vec3 {
        self.rotation * vector
    }

    /// Composes this transform with a local transform applied before it.
    pub fn compose(&self, local: &Self) -> Result<Self, TransformError> {
        let translation = self.rotation * local.translation + self.translation;
        if !translation.is_finite() {
            return Err(TransformError::CompositionOverflow);
        }

        let rotation = self.rotation * local.rotation;
        let rotation_length_squared = rotation.length_squared();
        if !rotation.is_finite()
            || !rotation_length_squared.is_finite()
            || rotation_length_squared <= 0.0
        {
            return Err(TransformError::CompositionOverflow);
        }

        let rotation = rotation.normalize();
        if !rotation.is_finite() || !rotation.is_normalized() {
            return Err(TransformError::CompositionOverflow);
        }

        Ok(Self {
            translation,
            rotation,
        })
    }

    pub fn inverse(&self) -> Result<Self, TransformError> {
        let rotation = self.rotation.conjugate();
        let translation = rotation * -self.translation;

        if !translation.is_finite() {
            return Err(TransformError::InverseOverflow);
        }

        Ok(Self {
            translation,
            rotation,
        })
    }
}

#[cfg(test)]
#[path = "transform_tests.rs"]
mod tests;
