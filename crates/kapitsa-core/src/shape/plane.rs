use glam::Vec3;

use super::ShapeError;

/// Infinite plane with equation `normal.dot(point) == distance` in local space.
/// The stored normal has unit length; distance is signed.
pub struct Plane {
    normal: Vec3,
    distance: f32,
}

impl Plane {
    pub fn new(nx: f32, ny: f32, nz: f32, distance: f32) -> Result<Self, ShapeError> {
        Self::from_vec(Vec3::new(nx, ny, nz), distance)
    }

    pub fn from_vec(normal: Vec3, distance: f32) -> Result<Self, ShapeError> {
        if !normal.is_finite() || !distance.is_finite() {
            return Err(ShapeError::NonFiniteParameter);
        }

        let length = normal.length();
        if !length.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }
        if length <= 0.0 {
            return Err(if normal == Vec3::ZERO {
                ShapeError::DegenerateGeometry
            } else {
                ShapeError::ArithmeticOverflow
            });
        }

        let normal = normal / length;
        let distance = distance / length;
        if !normal.is_finite() || !distance.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }

        Ok(Self { normal, distance })
    }

    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    pub fn distance(&self) -> f32 {
        self.distance
    }
}
