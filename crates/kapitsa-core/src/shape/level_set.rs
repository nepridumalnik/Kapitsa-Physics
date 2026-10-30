use glam::Vec3;

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
        let expected_count = dimensions[0]
            .checked_mul(dimensions[1])?
            .checked_mul(dimensions[2])?;

        if dimensions.iter().any(|dimension| *dimension < 2)
            || expected_count != signed_distances.len()
            || !origin.is_finite()
            || !grid_spacing.is_finite()
            || grid_spacing.min_element() <= 0.0
            || !signed_distances.iter().all(|distance| distance.is_finite())
        {
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
