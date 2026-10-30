use glam::Vec2;

pub struct HeightField {
    rows: usize,
    columns: usize,
    grid_spacing: Vec2,
    heights: Vec<f32>,
}

impl HeightField {
    pub fn from_vec(
        rows: usize,
        columns: usize,
        grid_spacing: Vec2,
        heights: Vec<f32>,
    ) -> Option<Self> {
        let expected_height_count = rows.checked_mul(columns)?;

        if rows < 2
            || columns < 2
            || expected_height_count != heights.len()
            || !grid_spacing.is_finite()
            || grid_spacing.min_element() <= 0.0
            || !heights.iter().all(|height| height.is_finite())
        {
            return None;
        }

        Some(Self {
            rows,
            columns,
            grid_spacing,
            heights,
        })
    }

    pub fn rows(&self) -> usize {
        self.rows
    }

    pub fn columns(&self) -> usize {
        self.columns
    }

    pub fn grid_spacing(&self) -> Vec2 {
        self.grid_spacing
    }

    pub fn heights(&self) -> &[f32] {
        &self.heights
    }
}
