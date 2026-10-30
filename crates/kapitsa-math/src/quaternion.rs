use std::ops::Mul;

use crate::{Mat3, Vec3};

/// A quaternion `(x, y, z, w)` representing a 3D rotation.
///
/// Multiplication uses the Hamilton product: `a * b` applies `b` first, then
/// `a`. Rotations use a right-handed coordinate system and angles are radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quat {
    pub const IDENTITY: Self = Self::new(0.0, 0.0, 0.0, 1.0);

    pub const fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Option<Self> {
        if !angle.is_finite() {
            return None;
        }
        let axis = axis.try_normalize()?;
        let half_angle = angle * 0.5;
        let (sin, cos) = half_angle.sin_cos();
        Some(Self::new(axis.x * sin, axis.y * sin, axis.z * sin, cos))
    }

    pub const fn conjugate(self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.w)
    }

    pub const fn length_squared(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    pub fn length(self) -> f64 {
        self.x.hypot(self.y).hypot(self.z).hypot(self.w)
    }

    pub fn try_normalize(self) -> Option<Self> {
        if !self.is_finite() {
            return None;
        }

        let scale = self
            .x
            .abs()
            .max(self.y.abs())
            .max(self.z.abs())
            .max(self.w.abs());
        if scale == 0.0 {
            return None;
        }

        let scaled = Self::new(
            self.x / scale,
            self.y / scale,
            self.z / scale,
            self.w / scale,
        );
        let length = scaled.length();
        if !length.is_finite() || length == 0.0 {
            return None;
        }

        Some(Self::new(
            scaled.x / length,
            scaled.y / length,
            scaled.z / length,
            scaled.w / length,
        ))
    }

    pub const fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite() && self.w.is_finite()
    }

    pub fn is_normalized(self) -> bool {
        self.is_finite() && (self.length() - 1.0).abs() <= 8.0 * f64::EPSILON
    }

    /// Rotates a vector, returning `None` for invalid inputs or overflow.
    pub fn rotate_vector(self, vector: Vec3) -> Option<Vec3> {
        if !vector.is_finite() {
            return None;
        }
        let rotation = self.try_normalize()?;
        let vector_part = Vec3::new(rotation.x, rotation.y, rotation.z);
        let first_cross = vector_part.cross(vector);
        let rotated =
            vector + first_cross * (2.0 * rotation.w) + vector_part.cross(first_cross) * 2.0;
        rotated.is_finite().then_some(rotated)
    }

    /// Returns the corresponding column-major rotation matrix.
    pub fn to_mat3(self) -> Option<Mat3> {
        let rotation = self.try_normalize()?;
        let (x, y, z, w) = (rotation.x, rotation.y, rotation.z, rotation.w);

        Some(Mat3::from_cols(
            Vec3::new(
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y + z * w),
                2.0 * (x * z - y * w),
            ),
            Vec3::new(
                2.0 * (x * y - z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z + x * w),
            ),
            Vec3::new(
                2.0 * (x * z + y * w),
                2.0 * (y * z - x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        ))
    }

    /// Integrates world-space angular velocity over `dt` radians-per-second.
    ///
    /// The world-space rotation increment is left-multiplied onto this
    /// orientation. Negative finite `dt` integrates backward in time.
    pub fn integrate_world_angular_velocity(self, angular_velocity: Vec3, dt: f64) -> Option<Self> {
        if !angular_velocity.is_finite() || !dt.is_finite() {
            return None;
        }

        let rotation_vector = angular_velocity * dt;
        if !rotation_vector.is_finite() {
            return None;
        }
        let angle = rotation_vector.length();
        if !angle.is_finite() {
            return None;
        }

        let delta = if angle < 1e-8 {
            let angle_squared = angle * angle;
            Self::new(
                rotation_vector.x * (0.5 - angle_squared / 48.0),
                rotation_vector.y * (0.5 - angle_squared / 48.0),
                rotation_vector.z * (0.5 - angle_squared / 48.0),
                1.0 - angle_squared / 8.0,
            )
        } else {
            let axis = rotation_vector / angle;
            let half_angle = angle * 0.5;
            let (sin, cos) = half_angle.sin_cos();
            Self::new(axis.x * sin, axis.y * sin, axis.z * sin, cos)
        };

        (delta * self.try_normalize()?).try_normalize()
    }
}

impl Mul for Quat {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(
            self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
            self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::Quat;
    use crate::{Mat3, Vec3};

    fn assert_vec3_close(actual: Vec3, expected: Vec3, tolerance: f64) {
        assert!((actual.x - expected.x).abs() <= tolerance);
        assert!((actual.y - expected.y).abs() <= tolerance);
        assert!((actual.z - expected.z).abs() <= tolerance);
    }

    #[test]
    fn axis_angle_rotation_matches_right_handed_convention() {
        let quarter_turn = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2)
            .expect("valid axis and angle");

        assert_vec3_close(
            quarter_turn.rotate_vector(Vec3::X).expect("valid rotation"),
            Vec3::Y,
            1e-15,
        );
        assert_vec3_close(
            quarter_turn.to_mat3().expect("valid rotation") * Vec3::X,
            Vec3::Y,
            1e-15,
        );
    }

    #[test]
    fn composition_applies_rhs_before_lhs() {
        let rotate_x = Quat::from_axis_angle(Vec3::X, std::f64::consts::FRAC_PI_2)
            .expect("valid axis and angle");
        let rotate_z = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2)
            .expect("valid axis and angle");
        let composed = rotate_z * rotate_x;

        let sequential = rotate_z
            .rotate_vector(rotate_x.rotate_vector(Vec3::Y).expect("valid rotation"))
            .expect("valid rotation");
        assert_vec3_close(
            composed.rotate_vector(Vec3::Y).expect("valid rotation"),
            sequential,
            1e-15,
        );
    }

    #[test]
    fn conjugate_reverses_a_unit_rotation() {
        let rotation =
            Quat::from_axis_angle(Vec3::new(1.0, 2.0, -3.0), 1.2).expect("valid axis and angle");
        let point = Vec3::new(4.0, -5.0, 6.0);
        let restored = rotation
            .conjugate()
            .rotate_vector(rotation.rotate_vector(point).expect("valid rotation"))
            .expect("valid rotation");

        assert_vec3_close(restored, point, 2e-15);
    }

    #[test]
    fn normalization_handles_zero_and_extreme_values() {
        assert_eq!(Quat::new(0.0, 0.0, 0.0, 0.0).try_normalize(), None);
        assert_eq!(Quat::new(f64::NAN, 0.0, 0.0, 1.0).try_normalize(), None);

        let normalized = Quat::new(f64::MAX, f64::MAX, 0.0, 0.0)
            .try_normalize()
            .expect("finite nonzero quaternion");
        assert!((normalized.length() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn integrates_angular_velocity_in_world_space() {
        let initial = Quat::from_axis_angle(Vec3::X, 0.4).expect("valid axis and angle");
        let integrated = initial
            .integrate_world_angular_velocity(Vec3::Z, std::f64::consts::FRAC_PI_2)
            .expect("finite orientation integration");
        let expected = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2)
            .expect("valid axis and angle")
            * initial;

        assert!((integrated.x - expected.x).abs() < 1e-15);
        assert!((integrated.y - expected.y).abs() < 1e-15);
        assert!((integrated.z - expected.z).abs() < 1e-15);
        assert!((integrated.w - expected.w).abs() < 1e-15);
        assert!(integrated.to_mat3().is_some());
    }

    #[test]
    fn rejects_invalid_axis_angle_and_integration_inputs() {
        assert_eq!(Quat::from_axis_angle(Vec3::ZERO, 1.0), None);
        assert_eq!(Quat::from_axis_angle(Vec3::X, f64::INFINITY), None);
        assert_eq!(
            Quat::IDENTITY.integrate_world_angular_velocity(Vec3::X, f64::NAN),
            None
        );
        assert_eq!(
            Quat::IDENTITY.integrate_world_angular_velocity(Vec3::splat(f64::MAX), 2.0),
            None
        );
    }

    #[test]
    fn matrix_conversion_is_rotation_preserving() {
        let rotation =
            Quat::from_axis_angle(Vec3::new(2.0, -1.0, 3.0), 2.1).expect("valid axis and angle");
        let point = Vec3::new(-2.0, 7.0, 0.5);
        let from_quaternion = rotation.rotate_vector(point).expect("valid rotation");
        let from_matrix = rotation.to_mat3().expect("valid rotation") * point;

        assert_vec3_close(from_matrix, from_quaternion, 2e-15);
        assert_eq!(Mat3::IDENTITY * point, point);
    }
}
