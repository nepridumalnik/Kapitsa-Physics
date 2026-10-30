use glam::Vec3;

/// Nondegenerate triangle in collider-local coordinates.
pub struct Triangle {
    vertices: [Vec3; 3],
}

impl Triangle {
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Option<Self> {
        Self::from_vertices([a, b, c])
    }

    pub fn from_vertices(vertices: [Vec3; 3]) -> Option<Self> {
        if !vertices.iter().all(|vertex| vertex.is_finite()) {
            return None;
        }

        let area_squared = (vertices[1] - vertices[0])
            .cross(vertices[2] - vertices[0])
            .length_squared();
        (area_squared.is_finite() && area_squared > 0.0).then_some(Self { vertices })
    }

    pub fn vertices(&self) -> [Vec3; 3] {
        self.vertices
    }
}
