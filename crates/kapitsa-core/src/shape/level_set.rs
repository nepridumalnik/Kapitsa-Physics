use crate::aabb::Aabb;
use kapitsa_math::Vec3;

use super::{BoundsError, ShapeError};

/// Row-major 3D signed-distance samples with X as the fastest-changing index.
pub struct LevelSet {
    origin: Vec3,
    dimensions: [usize; 3],
    grid_spacing: Vec3,
    signed_distances: Vec<f64>,
}

impl LevelSet {
    pub fn from_vec(
        origin: Vec3,
        dimensions: [usize; 3],
        grid_spacing: Vec3,
        signed_distances: Vec<f64>,
    ) -> Result<Self, ShapeError> {
        let count = dimensions[0]
            .checked_mul(dimensions[1])
            .ok_or(ShapeError::ArithmeticOverflow)?
            .checked_mul(dimensions[2])
            .ok_or(ShapeError::ArithmeticOverflow)?;
        if dimensions.iter().any(|&dimension| dimension < 2) {
            return Err(ShapeError::InvalidGridDimensions);
        }
        if count != signed_distances.len() {
            return Err(ShapeError::DataLengthMismatch);
        }
        if !origin.is_finite() || !grid_spacing.is_finite() {
            return Err(ShapeError::NonFiniteParameter);
        }
        if grid_spacing.min_element() <= 0.0 {
            return Err(ShapeError::InvalidDimension);
        }
        if !signed_distances.iter().all(|distance| distance.is_finite()) {
            return Err(ShapeError::NonFiniteParameter);
        }

        let span = Vec3::new(
            (dimensions[0] - 1) as f64,
            (dimensions[1] - 1) as f64,
            (dimensions[2] - 1) as f64,
        );
        if !(origin + grid_spacing * span).is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }

        Ok(Self {
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

    pub fn signed_distances(&self) -> &[f64] {
        &self.signed_distances
    }

    pub fn local_bounds(&self) -> Result<Aabb, BoundsError> {
        let span = Vec3::new(
            (self.dimensions[0] - 1) as f64,
            (self.dimensions[1] - 1) as f64,
            (self.dimensions[2] - 1) as f64,
        );
        let last_node = self.origin + self.grid_spacing * span;

        Aabb::new(self.origin.min(last_node), self.origin.max(last_node))
            .map_err(BoundsError::InvalidBounds)
    }
}
