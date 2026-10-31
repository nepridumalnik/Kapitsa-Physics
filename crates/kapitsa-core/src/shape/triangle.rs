use crate::aabb::Aabb;
use glam::Vec3;

use super::ShapeError;

/// Nondegenerate triangle in collider-local coordinates.
pub struct Triangle {
    vertices: [Vec3; 3],
}

impl Triangle {
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Result<Self, ShapeError> {
        Self::from_vertices([a, b, c])
    }

    pub fn from_vertices(vertices: [Vec3; 3]) -> Result<Self, ShapeError> {
        if !vertices.iter().all(|vertex| vertex.is_finite()) {
            return Err(ShapeError::NonFiniteParameter);
        }

        let area_squared = (vertices[1] - vertices[0])
            .cross(vertices[2] - vertices[0])
            .length_squared();
        if !area_squared.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }
        if area_squared <= 0.0 {
            return Err(ShapeError::DegenerateGeometry);
        }

        Ok(Self { vertices })
    }

    pub fn vertices(&self) -> [Vec3; 3] {
        self.vertices
    }

    pub fn local_bounds(&self) -> Option<Aabb> {
        let (&first, rest) = self.vertices.split_first()?;
        let (min, max) = rest.iter().fold((first, first), |(min, max), &vertex| {
            (min.min(vertex), max.max(vertex))
        });
        Aabb::new(min, max).ok()
    }
}
