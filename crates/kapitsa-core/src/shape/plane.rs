use glam::Vec3;

/// Infinite plane with equation `normal.dot(point) == distance` in local space.
/// The stored normal has unit length; distance is signed.
pub struct Plane {
    normal: Vec3,
    distance: f32,
}

impl Plane {
    pub fn new(nx: f32, ny: f32, nz: f32, distance: f32) -> Option<Self> {
        Self::from_vec(Vec3::new(nx, ny, nz), distance)
    }

    pub fn from_vec(normal: Vec3, distance: f32) -> Option<Self> {
        if !normal.is_finite() || !distance.is_finite() {
            return None;
        }

        let length = normal.length();
        if !length.is_finite() || length <= 0.0 {
            return None;
        }

        let normal = normal / length;
        let distance = distance / length;
        (normal.is_finite() && distance.is_finite()).then_some(Self { normal, distance })
    }

    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    pub fn distance(&self) -> f32 {
        self.distance
    }
}
