use crate::aabb::Aabb;
use glam::Vec3;

/// Sphere centered at the collider-local origin.
pub struct Sphere {
    radius: f32,
}

impl Sphere {
    pub fn new(radius: f32) -> Option<Self> {
        (radius.is_finite() && radius > 0.0).then_some(Self { radius })
    }

    pub fn radius(&self) -> f32 {
        self.radius
    }

    pub fn local_bounds(&self) -> Option<Aabb> {
        let extent = Vec3::splat(self.radius);
        Aabb::new(-extent, extent).ok()
    }
}
