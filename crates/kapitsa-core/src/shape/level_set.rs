use glam::Vec3;

/// Row-major 3D signed-distance samples with X as the fastest-changing index.
pub struct LevelSet {
    origin: Vec3,
    dimensions: [usize; 3],
    grid_spacing: Vec3,
    signed_distances: Vec<f32>,
}

impl LevelSet {
    pub fn from_vec(
        origin: Vec3,
        dimensions: [usize; 3],
        grid_spacing: Vec3,
        signed_distances: Vec<f32>,
    ) -> Option<Self> {
        let count = dimensions[0]
            .checked_mul(dimensions[1])?
            .checked_mul(dimensions[2])?;
        if dimensions.iter().any(|&dimension| dimension < 2)
            || count != signed_distances.len()
            || !origin.is_finite()
            || !grid_spacing.is_finite()
            || grid_spacing.min_element() <= 0.0
            || !signed_distances.iter().all(|distance| distance.is_finite())
        {
            return None;
        }

        let span = Vec3::new(
            (dimensions[0] - 1) as f32,
            (dimensions[1] - 1) as f32,
            (dimensions[2] - 1) as f32,
        );
        if !(origin + grid_spacing * span).is_finite() {
            return None;
        }

        Some(Self {
            origin,
            dimensions,
            grid_spacing,
            signed_distances,
        })
    }

    pub fn origin(&self) -> Vec3 {
        self.origin
    }

    pub fn dimensions(&self) -> [usize; 3] {
        self.dimensions
    }

    pub fn grid_spacing(&self) -> Vec3 {
        self.grid_spacing
    }

    pub fn signed_distances(&self) -> &[f32] {
        &self.signed_distances
    }
}
