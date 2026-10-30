use crate::aabb::Aabb;
use glam::Vec3;

/// Axis-aligned cuboid centered at the collider-local origin.
pub struct Cuboid {
    size: Vec3,
}

impl Cuboid {
    pub fn new(x: f32, y: f32, z: f32) -> Option<Self> {
        Self::from_vec(Vec3::new(x, y, z))
    }

    pub fn from_vec(size: Vec3) -> Option<Self> {
        (size.is_finite() && size.min_element() > 0.0).then_some(Self { size })
    }

    pub fn size(&self) -> Vec3 {
        self.size
    }

    pub fn half_extents(&self) -> Vec3 {
        self.size * 0.5
    }

    pub fn local_bounds(&self) -> Option<Aabb> {
        let half = self.half_extents();
        Aabb::new(-half, half)
    }
}
