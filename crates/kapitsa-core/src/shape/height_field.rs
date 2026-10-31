use crate::aabb::Aabb;
use kapitsa_math::{Vec2, Vec3};

use super::{BoundsError, ShapeError};

/// Row-major height samples on a regular local XY grid.
pub struct HeightField {
    rows: usize,
    columns: usize,
    grid_spacing: Vec2,
    heights: Vec<f64>,
}

impl HeightField {
    pub fn from_vec(
        rows: usize,
        columns: usize,
        grid_spacing: Vec2,
        heights: Vec<f64>,
    ) -> Result<Self, ShapeError> {
        let count = rows
            .checked_mul(columns)
            .ok_or(ShapeError::ArithmeticOverflow)?;
        if rows < 2 || columns < 2 {
            return Err(ShapeError::InvalidGridDimensions);
        }
        if count != heights.len() {
            return Err(ShapeError::DataLengthMismatch);
        }
        if !grid_spacing.is_finite() {
            return Err(ShapeError::NonFiniteParameter);
        }
        if grid_spacing.min_element() <= 0.0 {
            return Err(ShapeError::InvalidDimension);
        }
        let span = grid_spacing * Vec2::new((columns - 1) as f64, (rows - 1) as f64);
        if !span.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }
        if !heights.iter().all(|height| height.is_finite()) {
            return Err(ShapeError::NonFiniteParameter);
        }

        Ok(Self {
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

    pub fn heights(&self) -> &[f64] {
        &self.heights
    }

    pub fn local_bounds(&self) -> Result<Aabb, BoundsError> {
        let (&first_height, remaining_heights) = self
            .heights
            .split_first()
            .ok_or(BoundsError::InvalidGeometry)?;
        let (min_height, max_height) = remaining_heights
            .iter()
            .fold((first_height, first_height), |(min, max), &height| {
                (min.min(height), max.max(height))
            });
        let span = self.grid_spacing * Vec2::new((self.columns - 1) as f64, (self.rows - 1) as f64);

        Aabb::new(
            Vec3::new(0.0, 0.0, min_height),
            Vec3::new(span.x, span.y, max_height),
        )
        .map_err(BoundsError::InvalidBounds)
    }
}
