use crate::{Mat3, Vec3};

/// A 3D affine transform with a linear matrix and translation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Affine3 {
    linear: Mat3,
    translation: Vec3,
}

impl Affine3 {
    pub const IDENTITY: Self = Self::new(Mat3::IDENTITY, Vec3::ZERO);

    pub const fn new(linear: Mat3, translation: Vec3) -> Self {
        Self {
            linear,
            translation,
        }
    }

    pub const fn from_translation(translation: Vec3) -> Self {
        Self::new(Mat3::IDENTITY, translation)
    }

    pub const fn linear(self) -> Mat3 {
        self.linear
    }

    pub const fn translation(self) -> Vec3 {
        self.translation
    }

    pub const fn is_finite(self) -> bool {
        self.linear.is_finite() && self.translation.is_finite()
    }
}

#[cfg(test)]
mod tests {
    use super::Affine3;
    use crate::{Mat3, Vec3};

    #[test]
    fn identity_and_translation_are_finite_affine_transforms() {
        assert_eq!(Affine3::IDENTITY.linear(), Mat3::IDENTITY);
        assert_eq!(Affine3::IDENTITY.translation(), Vec3::ZERO);

        let transform = Affine3::from_translation(Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(transform.linear(), Mat3::IDENTITY);
        assert_eq!(transform.translation(), Vec3::new(1.0, 2.0, 3.0));
        assert!(transform.is_finite());
        assert!(!Affine3::from_translation(Vec3::new(f64::NAN, 0.0, 0.0)).is_finite());
    }
}
