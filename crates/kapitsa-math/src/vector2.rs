use std::ops::{Add, Div, Mul, Neg, Sub};

/// A two-dimensional vector with `f64` components.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const ZERO: Self = Self::splat(0.0);
    pub const ONE: Self = Self::splat(1.0);
    pub const X: Self = Self::new(1.0, 0.0);
    pub const Y: Self = Self::new(0.0, 1.0);

    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub const fn splat(value: f64) -> Self {
        Self::new(value, value)
    }

    pub const fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }

    pub fn min_element(self) -> f64 {
        self.x.min(self.y)
    }
}

impl Add for Vec2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul for Vec2 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(self.x * rhs.x, self.y * rhs.y)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl Mul<Vec2> for f64 {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Self::Output {
        rhs * self
    }
}

impl Div<f64> for Vec2 {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs)
    }
}

impl Neg for Vec2 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y)
    }
}

#[cfg(test)]
mod tests {
    use super::Vec2;

    #[test]
    fn constants_and_component_arithmetic() {
        assert_eq!(Vec2::ZERO, Vec2::new(0.0, 0.0));
        assert_eq!(Vec2::ONE, Vec2::splat(1.0));
        assert_eq!(Vec2::X + Vec2::Y, Vec2::ONE);
        assert_eq!(
            Vec2::new(2.0, 3.0) * Vec2::new(4.0, 5.0),
            Vec2::new(8.0, 15.0)
        );
        assert_eq!(Vec2::new(2.0, -4.0) * 0.5, Vec2::new(1.0, -2.0));
        assert_eq!(-Vec2::X, Vec2::new(-1.0, 0.0));
    }

    #[test]
    fn reports_non_finite_components() {
        assert!(!Vec2::new(f64::NAN, 0.0).is_finite());
        assert!(!Vec2::new(0.0, f64::INFINITY).is_finite());
    }
}
