use glam::{Quat, Vec3};

/// Rigid transformation consisting of translation and rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    translation: Vec3,
    rotation: Quat,
}

impl Transform {
    pub fn new(translation: Vec3, rotation: Quat) -> Option<Self> {
        (translation.is_finite() && rotation.is_finite() && rotation.is_normalized()).then_some(
            Self {
                translation,
                rotation,
            },
        )
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

    pub fn inverse(&self) -> Option<Self> {
        let rotation = self.rotation.conjugate();
        let translation = rotation * -self.translation;

        Self::new(translation, rotation)
    }
}

#[cfg(test)]
#[path = "transform_tests.rs"]
mod tests;
