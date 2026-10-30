use glam::Vec3;

pub struct Capsule {
    start: Vec3,
    end: Vec3,
    radius: f32,
}

impl Capsule {
    pub fn new(start: Vec3, end: Vec3, radius: f32) -> Option<Self> {
        (start.is_finite() && end.is_finite() && radius.is_finite() && radius > 0.0)
            .then_some(Self { start, end, radius })
    }

    pub fn start(&self) -> Vec3 {
        self.start
    }

    pub fn end(&self) -> Vec3 {
        self.end
    }

    pub fn radius(&self) -> f32 {
        self.radius
    }
}
