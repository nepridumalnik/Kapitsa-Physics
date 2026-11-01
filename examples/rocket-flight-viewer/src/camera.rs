use kapitsa::math::Vec3;

pub(crate) use crate::flight::{PLANET_RADIUS, SceneFrame};

pub(crate) struct Camera {
    half_width: f64,
    maximum_altitude: f64,
}

impl Camera {
    pub(crate) fn for_frames(frames: &[SceneFrame], width: u32, height: u32) -> Self {
        let maximum_altitude = frames
            .iter()
            .flat_map(|frame| [frame.stage_one, frame.stage_two])
            .flatten()
            .map(|state| altitude(state.transform().translation()))
            .fold(50.0_f64, f64::max);
        let aspect = f64::from(width.max(1)) / f64::from(height.max(1));
        Self {
            half_width: (maximum_altitude * aspect * 0.75).max(50.0),
            maximum_altitude: (maximum_altitude * 1.1).max(50.0),
        }
    }

    pub(crate) fn project(&self, position: Vec3) -> [f32; 2] {
        let x = position.x / self.half_width;
        let y = altitude(position) / self.maximum_altitude * 2.0 - 1.0;
        [x as f32, y as f32]
    }
}

fn altitude(position: Vec3) -> f64 {
    (position.length() - PLANET_RADIUS).max(-5.0)
}
