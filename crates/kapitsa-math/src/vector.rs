use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// A three-dimensional vector with `f64` components.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::splat(0.0);
    pub const ONE: Self = Self::splat(1.0);
    pub const X: Self = Self::new(1.0, 0.0, 0.0);
    pub const Y: Self = Self::new(0.0, 1.0, 0.0);
    pub const Z: Self = Self::new(0.0, 0.0, 1.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn splat(value: f64) -> Self {
        Self::new(value, value, value)
    }

    pub const fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub const fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    pub const fn length_squared(self) -> f64 {
        self.dot(self)
    }

    /// Returns the Euclidean length. The result can be infinite if the exact
    /// length is outside the representable `f64` range.
    pub fn length(self) -> f64 {
        self.x.hypot(self.y).hypot(self.z)
    }

    /// Returns a unit vector, or `None` for zero and non-finite inputs.
    /// Scaling before computing the norm avoids intermediate overflow and
    /// underflow for finite, nonzero vectors.
    pub fn try_normalize(self) -> Option<Self> {
        if !self.is_finite() {
            return None;
        }

        let scale = self.x.abs().max(self.y.abs()).max(self.z.abs());
        if scale == 0.0 {
            return None;
        }

        let scaled = self / scale;
        let length = scaled.length();
        if !length.is_finite() || length == 0.0 {
            return None;
        }

        Some(scaled / length)
    }

    pub const fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    pub fn min(self, rhs: Self) -> Self {
        Self::new(self.x.min(rhs.x), self.y.min(rhs.y), self.z.min(rhs.z))
    }

    pub fn max(self, rhs: Self) -> Self {
        Self::new(self.x.max(rhs.x), self.y.max(rhs.y), self.z.max(rhs.z))
    }

    pub fn checked_add(self, rhs: Self) -> Option<Self> {
        let result = self + rhs;
        result.is_finite().then_some(result)
    }

    pub fn checked_sub(self, rhs: Self) -> Option<Self> {
        let result = self - rhs;
        result.is_finite().then_some(result)
    }

    pub fn checked_mul_scalar(self, scalar: f64) -> Option<Self> {
        let result = self * scalar;
        (scalar.is_finite() && result.is_finite()).then_some(result)
    }

    pub fn checked_dot(self, rhs: Self) -> Option<f64> {
        let result = self.dot(rhs);
        (self.is_finite() && rhs.is_finite() && result.is_finite()).then_some(result)
    }

    pub fn checked_cross(self, rhs: Self) -> Option<Self> {
        let result = self.cross(rhs);
        (self.is_finite() && rhs.is_finite() && result.is_finite()).then_some(result)
    }

    pub fn checked_length(self) -> Option<f64> {
        let result = self.length();
        result.is_finite().then_some(result)
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Mul for Vec3 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<Vec3> for f64 {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        rhs * self
    }
}

impl MulAssign for Vec3 {
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
        self.z *= rhs.z;
    }
}

impl MulAssign<f64> for Vec3 {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Div for Vec3 {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Self::new(self.x / rhs.x, self.y / rhs.y, self.z / rhs.z)
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl DivAssign for Vec3 {
    fn div_assign(&mut self, rhs: Self) {
        self.x /= rhs.x;
        self.y /= rhs.y;
        self.z /= rhs.z;
    }
}

impl DivAssign<f64> for Vec3 {
    fn div_assign(&mut self, rhs: f64) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl Neg for Vec3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y, -self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::Vec3;

    #[test]
    fn constants_and_constructors() {
        assert_eq!(Vec3::ZERO, Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(Vec3::ONE, Vec3::splat(1.0));
        assert_eq!(Vec3::X, Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(Vec3::Y, Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(Vec3::Z, Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn vector_and_scalar_arithmetic() {
        let a = Vec3::new(2.0, -4.0, 6.0);
        let b = Vec3::new(1.0, 3.0, -2.0);

        assert_eq!(a + b, Vec3::new(3.0, -1.0, 4.0));
        assert_eq!(a - b, Vec3::new(1.0, -7.0, 8.0));
        assert_eq!(a * b, Vec3::new(2.0, -12.0, -12.0));
        assert_eq!(a / Vec3::new(2.0, -2.0, 3.0), Vec3::new(1.0, 2.0, 2.0));
        assert_eq!(a * 0.5, Vec3::new(1.0, -2.0, 3.0));
        assert_eq!(2.0 * b, Vec3::new(2.0, 6.0, -4.0));
        assert_eq!(a / 2.0, Vec3::new(1.0, -2.0, 3.0));
        assert_eq!(-a, Vec3::new(-2.0, 4.0, -6.0));

        let mut accumulated = a;
        accumulated += b;
        accumulated -= b;
        accumulated *= 2.0;
        accumulated /= 2.0;
        assert_eq!(accumulated, a);
    }

    #[test]
    fn dot_and_cross_products() {
        let a = Vec3::new(2.0, -3.0, 4.0);
        let b = Vec3::new(-1.0, 5.0, 2.0);
        let cross = a.cross(b);

        assert_eq!(a.dot(b), -9.0);
        assert_eq!(cross, Vec3::new(-26.0, -8.0, 7.0));
        assert_eq!(cross.dot(a), 0.0);
        assert_eq!(cross.dot(b), 0.0);
        assert_eq!(Vec3::X.cross(Vec3::Y), Vec3::Z);
    }

    #[test]
    fn length_and_normalization_handle_zero_and_extremes() {
        assert_eq!(Vec3::ZERO.length_squared(), 0.0);
        assert_eq!(Vec3::ZERO.length(), 0.0);
        assert_eq!(Vec3::ZERO.try_normalize(), None);

        let large = Vec3::splat(f64::MAX / 2.0);
        let normalized = large.try_normalize().expect("finite nonzero vector");
        assert!((normalized.length() - 1.0).abs() < 1e-15);

        let tiny = Vec3::new(f64::from_bits(1), 0.0, 0.0);
        assert_eq!(tiny.length(), f64::from_bits(1));
        assert_eq!(tiny.try_normalize(), Some(Vec3::X));

        assert!(Vec3::splat(f64::MAX).length().is_infinite());
    }

    #[test]
    fn rejects_non_finite_normalization_inputs() {
        assert!(!Vec3::new(f64::NAN, 0.0, 0.0).is_finite());
        assert!(!Vec3::new(0.0, f64::INFINITY, 0.0).is_finite());
        assert_eq!(Vec3::new(f64::NAN, 0.0, 0.0).try_normalize(), None);
        assert_eq!(Vec3::new(0.0, 0.0, f64::INFINITY).try_normalize(), None);
    }

    #[test]
    fn length_and_normalization_preserve_numeric_precision() {
        let vector = Vec3::new(3.0, 4.0, 12.0);
        assert_eq!(vector.length_squared(), 169.0);
        assert_eq!(vector.length(), 13.0);
        assert_eq!(
            vector.try_normalize(),
            Some(Vec3::new(3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0))
        );
    }

    #[test]
    fn checked_operations_reject_non_finite_results() {
        assert_eq!(
            Vec3::new(f64::MAX, 0.0, 0.0).checked_add(Vec3::new(f64::MAX, 0.0, 0.0)),
            None
        );
        assert_eq!(Vec3::new(f64::MAX, 0.0, 0.0).checked_mul_scalar(2.0), None);
        assert_eq!(
            Vec3::new(f64::MAX, 0.0, 0.0).checked_dot(Vec3::new(2.0, 0.0, 0.0)),
            None
        );
        assert_eq!(Vec3::new(f64::NAN, 0.0, 0.0).checked_length(), None);
    }
}
