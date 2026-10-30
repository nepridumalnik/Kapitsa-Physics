use crate::aabb::Aabb;
use glam::{Vec2, Vec3};

/// Row-major height samples on a regular local XY grid.
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
        let count = rows.checked_mul(columns)?;
        if rows < 2
            || columns < 2
            || count != heights.len()
            || !grid_spacing.is_finite()
            || grid_spacing.min_element() <= 0.0
            || !(grid_spacing * Vec2::new((columns - 1) as f32, (rows - 1) as f32)).is_finite()
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

    pub fn local_bounds(&self) -> Option<Aabb> {
        let (&first_height, remaining_heights) = self.heights.split_first()?;
        let (min_height, max_height) = remaining_heights
            .iter()
            .fold((first_height, first_height), |(min, max), &height| {
                (min.min(height), max.max(height))
            });
        let span = self.grid_spacing * Vec2::new((self.columns - 1) as f32, (self.rows - 1) as f32);

        Aabb::new(
            Vec3::new(0.0, 0.0, min_height),
            Vec3::new(span.x, span.y, max_height),
        )
    }
}
