use kapitsa_math::{Quat, Vec3};

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
        if !rotation.is_normalized() {
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
        match self.rotation.to_mat3() {
            Some(rotation) => rotation * point + self.translation,
            None => Vec3::splat(f64::NAN),
        }
    }

    pub fn transform_vector(&self, vector: Vec3) -> Vec3 {
        match self.rotation.to_mat3() {
            Some(rotation) => rotation * vector,
            None => Vec3::splat(f64::NAN),
        }
    }

    /// Composes this transform with a local transform applied before it.
    pub fn compose(&self, local: &Self) -> Result<Self, TransformError> {
        let Some(rotation_matrix) = self.rotation.to_mat3() else {
            return Err(TransformError::CompositionOverflow);
        };
        let translation = rotation_matrix * local.translation + self.translation;
        if !translation.is_finite() {
            return Err(TransformError::CompositionOverflow);
        }

        let rotation = (self.rotation * local.rotation)
            .try_normalize()
            .filter(|rotation| rotation.is_normalized())
            .ok_or(TransformError::CompositionOverflow)?;

        Ok(Self {
            translation,
            rotation,
        })
    }

    pub fn inverse(&self) -> Result<Self, TransformError> {
        let rotation = self.rotation.conjugate();
        let Some(rotation_matrix) = rotation.to_mat3() else {
            return Err(TransformError::InverseOverflow);
        };
        let translation = rotation_matrix * -self.translation;

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
