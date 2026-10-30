use glam::Vec3;

/// Triangulated surface in local coordinates; may be open or nonconvex.
pub struct TriangleMesh {
    vertices: Vec<Vec3>,
    triangles: Vec<[usize; 3]>,
}

impl TriangleMesh {
    pub fn new(vertices: Vec<Vec3>, triangles: Vec<[usize; 3]>) -> Option<Self> {
        if vertices.len() < 3
            || triangles.is_empty()
            || !vertices.iter().all(|vertex| vertex.is_finite())
        {
            return None;
        }

        for &[a, b, c] in &triangles {
            if a >= vertices.len() || b >= vertices.len() || c >= vertices.len() {
                return None;
            }
            if a == b || b == c || c == a {
                return None;
            }

            let area_squared = (vertices[b] - vertices[a])
                .cross(vertices[c] - vertices[a])
                .length_squared();
            if !area_squared.is_finite() || area_squared <= 0.0 {
                return None;
            }
        }

        Some(Self {
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
}
