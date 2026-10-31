use std::collections::HashMap;

use crate::aabb::Aabb;
use glam::Vec3;

use super::{BoundsError, ShapeError};

/// Closed, consistently outward-wound, triangulated convex polyhedron.
pub struct Convex {
    vertices: Vec<Vec3>,
    faces: Vec<[usize; 3]>,
}

impl Convex {
    pub fn new(vertices: Vec<Vec3>, faces: Vec<[usize; 3]>) -> Result<Self, ShapeError> {
        if vertices.len() < 4 || faces.len() < 4 {
            return Err(ShapeError::InvalidTopology);
        }
        if !vertices.iter().all(|vertex| vertex.is_finite()) {
            return Err(ShapeError::NonFiniteParameter);
        }

        let (minimum, maximum) = vertices.iter().fold(
            (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
            |(min, max), &vertex| (min.min(vertex), max.max(vertex)),
        );
        let extent = (maximum - minimum).max_element();
        if !extent.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }
        if extent <= 0.0 {
            return Err(ShapeError::DegenerateGeometry);
        }

        let tolerance = extent * f32::EPSILON * 8.0;
        let mut used = vec![false; vertices.len()];
        let mut edges = HashMap::<(usize, usize), (usize, isize)>::new();
        let reference = vertices[0];
        let mut volume_six = 0.0_f64;

        for &[a, b, c] in &faces {
            if a >= vertices.len() || b >= vertices.len() || c >= vertices.len() {
                return Err(ShapeError::InvalidTopology);
            }
            if a == b || b == c || c == a {
                return Err(ShapeError::InvalidTopology);
            }

            let ab = vertices[b] - vertices[a];
            let ac = vertices[c] - vertices[a];
            let area = ab.cross(ac);
            let area_length = area.length();
            if !area_length.is_finite() {
                return Err(ShapeError::ArithmeticOverflow);
            }
            if area_length <= 0.0 {
                return Err(ShapeError::DegenerateGeometry);
            }

            // Every vertex must be on or behind the outward-facing plane.
            let normal = area / area_length;
            for &vertex in &vertices {
                let side = normal.dot(vertex - vertices[a]);
                if !side.is_finite() {
                    return Err(ShapeError::ArithmeticOverflow);
                }
                if side > tolerance {
                    return Err(ShapeError::InvalidTopology);
                }
            }

            used[a] = true;
            used[b] = true;
            used[c] = true;
            volume_six += (vertices[a] - reference)
                .dot((vertices[b] - reference).cross(vertices[c] - reference))
                as f64;

            for (from, to) in [(a, b), (b, c), (c, a)] {
                let key = (from.min(to), from.max(to));
                let (count, winding) = edges.entry(key).or_insert((0, 0));
                *count += 1;
                *winding += if from < to { 1 } else { -1 };
                if *count > 2 {
                    return Err(ShapeError::InvalidTopology);
                }
            }
        }

        if !used.into_iter().all(|value| value)
            || !edges
                .values()
                .all(|&(count, winding)| count == 2 && winding == 0)
        {
            return Err(ShapeError::InvalidTopology);
        }
        if !volume_six.is_finite() {
            return Err(ShapeError::ArithmeticOverflow);
        }
        if volume_six < 0.0 {
            return Err(ShapeError::InvalidTopology);
        }
        if volume_six == 0.0 {
            return Err(ShapeError::DegenerateGeometry);
        }

        Ok(Self { vertices, faces })
    }

    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    pub fn faces(&self) -> &[[usize; 3]] {
        &self.faces
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
