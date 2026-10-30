use glam::Vec3;

pub struct Plane {
    normal: Vec3,
    distance: f32,
}

impl Plane {
    pub fn new(nx: f32, ny: f32, nz: f32, distance: f32) -> Option<Self> {
        Self::from_vec(Vec3::new(nx, ny, nz), distance)
    }

    pub fn from_vec(normal: Vec3, distance: f32) -> Option<Self> {
        let length = normal.length();

        if !length.is_finite() || length <= 0.0 || !distance.is_finite() {
            return None;
        }

        Some(Self {
            normal: normal / length,
            distance: distance / length,
        })
    }

    pub fn normal(&self) -> Vec3 {
        self.normal
    }

    pub fn distance(&self) -> f32 {
        self.distance
    }
}
