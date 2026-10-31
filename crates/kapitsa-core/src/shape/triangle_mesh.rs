use crate::aabb::Aabb;
use glam::Vec3;

use super::{BoundsError, ShapeError};

/// Triangulated surface in local coordinates; may be open or nonconvex.
pub struct TriangleMesh {
    vertices: Vec<Vec3>,
    triangles: Vec<[usize; 3]>,
}

impl TriangleMesh {
    pub fn new(vertices: Vec<Vec3>, triangles: Vec<[usize; 3]>) -> Result<Self, ShapeError> {
        if vertices.len() < 3 || triangles.is_empty() {
            return Err(ShapeError::InvalidTopology);
        }
        if !vertices.iter().all(|vertex| vertex.is_finite()) {
            return Err(ShapeError::NonFiniteParameter);
        }

        for &[a, b, c] in &triangles {
            if a >= vertices.len() || b >= vertices.len() || c >= vertices.len() {
                return Err(ShapeError::InvalidTopology);
            }
            if a == b || b == c || c == a {
                return Err(ShapeError::InvalidTopology);
            }

            let area_squared = (vertices[b] - vertices[a])
                .cross(vertices[c] - vertices[a])
                .length_squared();
            if !area_squared.is_finite() {
                return Err(ShapeError::ArithmeticOverflow);
            }
            if area_squared <= 0.0 {
                return Err(ShapeError::DegenerateGeometry);
            }
        }

        Ok(Self {
            vertices,
            triangles,
        })
    }

    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    pub fn triangles(&self) -> &[[usize; 3]] {
        &self.triangles
    }

    pub fn local_bounds(&self) -> Result<Aabb, BoundsError> {
        let (&first, rest) = self
            .vertices
            .split_first()
            .ok_or(BoundsError::InvalidGeometry)?;
        let (min, max) = rest.iter().fold((first, first), |(min, max), &vertex| {
            (min.min(vertex), max.max(vertex))
        });
        Aabb::new(min, max).map_err(BoundsError::InvalidBounds)
    }
}
