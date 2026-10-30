use glam::Vec3;

pub struct Cylinder {
    start: Vec3,
    end: Vec3,
    radius: f32,
}

impl Cylinder {
    pub fn new(start: Vec3, end: Vec3, radius: f32) -> Option<Self> {
        let axis = end - start;

        (start.is_finite()
            && end.is_finite()
            && axis.length_squared().is_finite()
            && axis.length_squared() > 0.0
            && radius.is_finite()
            && radius > 0.0)
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
