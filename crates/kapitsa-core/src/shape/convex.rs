use std::collections::HashMap;

use glam::Vec3;

pub struct Convex {
    vertices: Vec<Vec3>,
    faces: Vec<[usize; 3]>,
}

impl Convex {
    pub fn new(vertices: Vec<Vec3>, faces: Vec<[usize; 3]>) -> Option<Self> {
        if vertices.len() < 4
            || faces.len() < 4
            || !vertices.iter().all(|vertex| vertex.is_finite())
        {
            return None;
        }

        let mut minimum = Vec3::splat(f32::INFINITY);
        let mut maximum = Vec3::splat(f32::NEG_INFINITY);
        for vertex in &vertices {
            minimum = minimum.min(*vertex);
            maximum = maximum.max(*vertex);
        }
        let extent = (maximum - minimum).max_element();
        if !extent.is_finite() || extent <= 0.0 {
            return None;
        }
        let plane_tolerance = extent * f32::EPSILON * 8.0;

        let mut used_vertices = vec![false; vertices.len()];
        let mut edge_counts = HashMap::new();

        for [a, b, c] in &faces {
            if *a >= vertices.len() || *b >= vertices.len() || *c >= vertices.len() {
                return None;
            }
            if a == b || b == c || c == a {
                return None;
            }

            let area = (vertices[*b] - vertices[*a]).cross(vertices[*c] - vertices[*a]);
            let area_squared = area.length_squared();
            if !area_squared.is_finite() || area_squared == 0.0 {
                return None;
            }

            let normal = area / area_squared.sqrt();
            let mut has_positive_distance = false;
            let mut has_negative_distance = false;
            for vertex in &vertices {
                let distance = normal.dot(*vertex - vertices[*a]);
                has_positive_distance |= distance > plane_tolerance;
                has_negative_distance |= distance < -plane_tolerance;
            }
            if has_positive_distance && has_negative_distance {
                return None;
            }

            used_vertices[*a] = true;
            used_vertices[*b] = true;
            used_vertices[*c] = true;

            for (from, to) in [(*a, *b), (*b, *c), (*c, *a)] {
                let edge = if from < to { (from, to) } else { (to, from) };
                let (count, winding): &mut (usize, isize) =
                    edge_counts.entry(edge).or_insert((0, 0));
                *count += 1;
                *winding += if from < to { 1 } else { -1 };
            }
        }

        if !used_vertices.into_iter().all(|used| used)
            || !edge_counts
                .values()
                .all(|(count, winding)| *count == 2 && *winding == 0)
        {
            return None;
        }

        Some(Self { vertices, faces })
    }

    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    pub fn faces(&self) -> &[[usize; 3]] {
        &self.faces
    }
}
