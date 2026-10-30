use glam::Vec3;

/// Conical frustum with positive radii at both ends of its local axis.
pub struct TaperedCylinder {
    start: Vec3,
    end: Vec3,
    start_radius: f32,
    end_radius: f32,
}

impl TaperedCylinder {
    pub fn new(start: Vec3, end: Vec3, start_radius: f32, end_radius: f32) -> Option<Self> {
        let length = (end - start).length();
        (start.is_finite()
            && end.is_finite()
            && length.is_finite()
            && length > 0.0
            && start_radius.is_finite()
            && start_radius > 0.0
            && end_radius.is_finite()
            && end_radius > 0.0)
            .then_some(Self {
                start,
                end,
                start_radius,
                end_radius,
            })
    }

    pub fn start(&self) -> Vec3 {
        self.start
    }

    pub fn end(&self) -> Vec3 {
        self.end
    }

    pub fn start_radius(&self) -> f32 {
        self.start_radius
    }

    pub fn end_radius(&self) -> f32 {
        self.end_radius
    }
}
