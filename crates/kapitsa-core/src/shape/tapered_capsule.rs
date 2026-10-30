use glam::Vec3;

pub struct TaperedCapsule {
    start: Vec3,
    end: Vec3,
    start_radius: f32,
    end_radius: f32,
    split_axis: Vec3,
}

impl TaperedCapsule {
    pub fn new(
        start: Vec3,
        end: Vec3,
        start_radius: f32,
        end_radius: f32,
        split_axis: Vec3,
    ) -> Option<Self> {
        let axis = end - start;
        let axis_length = axis.length();
        let split_axis_length = split_axis.length();

        if !start.is_finite()
            || !end.is_finite()
            || !axis_length.is_finite()
            || axis_length <= 0.0
            || !start_radius.is_finite()
            || start_radius <= 0.0
            || !end_radius.is_finite()
            || end_radius <= 0.0
            || !split_axis_length.is_finite()
            || split_axis_length <= 0.0
        {
            return None;
        }

        let axis = axis / axis_length;
        let split_axis = split_axis / split_axis_length;
        if axis.dot(split_axis).abs() > 1.0e-5 {
            return None;
        }

        Some(Self {
            start,
            end,
            start_radius,
            end_radius,
            split_axis,
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

    pub fn split_axis(&self) -> Vec3 {
        self.split_axis
    }
}
