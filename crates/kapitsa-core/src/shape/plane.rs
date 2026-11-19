use kapitsa_math::Vec3;

use super::ShapeError;

/// Infinite plane with equation `normal.dot(point) == distance` in local space.
/// The stored normal has unit length; distance is signed.
pub struct Plane {
    normal: Vec3,
    distance: f64,
}

impl Plane {
    pub fn new(nx: f64, ny: f64, nz: f64, distance: f64) -> Result<Self, crate::Error> {
        Self::from_vec(Vec3::new(nx, ny, nz), distance)
    }

    pub fn from_vec(normal: Vec3, distance: f64) -> Result<Self, crate::Error> {
        if !normal.is_finite() || !distance.is_finite() {
            return Err(crate::Error::from(ShapeError::NonFiniteParameter));
        }

        let length = normal.length();
        if !length.is_finite() {
            return Err(crate::Error::from(ShapeError::ArithmeticOverflow));
        }
        if length <= 0.0 {
            return Err(crate::Error::from(if normal == Vec3::ZERO {
                ShapeError::DegenerateGeometry
            } else {
                ShapeError::ArithmeticOverflow
            }));
        }

        let normal = normal
            .try_normalize()
            .ok_or_else(|| crate::Error::from(ShapeError::ArithmeticOverflow))?;
        let distance = distance / length;
        if !normal.is_finite() || !distance.is_finite() {
            return Err(crate::Error::from(ShapeError::ArithmeticOverflow));
        }

        Ok(Self { normal, distance })
    }

    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    pub fn distance(&self) -> f64 {
        self.distance
    }
}
