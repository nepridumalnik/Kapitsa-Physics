//! Bounded, deterministic collider tessellation. No bounds-based substitutes.
use kapitsa_math::Vec3;

use super::AerodynamicsError;
use crate::{
    collider::Collider,
    shape::{Shape, TriangleMesh},
};

/// Resolution and aggregate output budget for one body's attached colliders.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ColliderTessellation {
    radial_segments: usize,
    sphere_rings: usize,
    max_triangles: usize,
}

impl Default for ColliderTessellation {
    fn default() -> Self {
        Self {
            radial_segments: 16,
            sphere_rings: 8,
            max_triangles: 16_384,
        }
    }
}

impl ColliderTessellation {
    /// Rings include the two poles. Hard limits bound both work and allocation.
    pub fn new(
        radial_segments: usize,
        sphere_rings: usize,
        max_triangles: usize,
    ) -> Result<Self, AerodynamicsError> {
        if !(3..=128).contains(&radial_segments)
            || !(3..=128).contains(&sphere_rings)
            || !(1..=65_536).contains(&max_triangles)
        {
            return Err(AerodynamicsError::InvalidTessellation);
        }
        Ok(Self {
            radial_segments,
            sphere_rings,
            max_triangles,
        })
    }
    pub fn radial_segments(self) -> usize {
        self.radial_segments
    }
    pub fn sphere_rings(self) -> usize {
        self.sphere_rings
    }
    pub fn max_triangles(self) -> usize {
        self.max_triangles
    }
}

/// Converts real surfaces into one body-local mesh. Collider transforms are baked
/// in, not world poses or COM offsets. Source winding is preserved for meshes.
/// Empty sets and unsupported shapes are rejected, never approximated by AABBs.
pub fn collider_mesh<'a>(
    colliders: impl IntoIterator<Item = &'a Collider>,
    config: ColliderTessellation,
) -> Result<TriangleMesh, AerodynamicsError> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for collider in colliders {
        let count = match collider.shape() {
            Shape::Cuboid(_) => 12,
            Shape::Sphere(_) => 2 * config.radial_segments * (config.sphere_rings - 2),
            Shape::Cylinder(_) => 4 * config.radial_segments,
            Shape::Triangle(_) => 1,
            Shape::Convex(shape) => shape.faces().len(),
            Shape::TriangleMesh(shape) => shape.triangles().len(),
            _ => return Err(AerodynamicsError::UnsupportedModel),
        };
        if count > config.max_triangles - triangles.len() {
            return Err(AerodynamicsError::TessellationBudgetExceeded);
        }
        // Use independent indexed triangles, welded explicitly by world geometry
        // preparation if closure is required. This also bounds unreferenced input.
        vertices
            .try_reserve(count * 3)
            .map_err(|_| AerodynamicsError::ArithmeticOverflow)?;
        triangles
            .try_reserve(count)
            .map_err(|_| AerodynamicsError::ArithmeticOverflow)?;
        let mut emit = |points: [Vec3; 3]| -> Result<(), AerodynamicsError> {
            let base = vertices.len();
            for point in points {
                let point = collider.local_transform().transform_point(point);
                if !point.is_finite() {
                    return Err(AerodynamicsError::ArithmeticOverflow);
                }
                vertices.push(point);
            }
            triangles.push([base, base + 1, base + 2]);
            Ok(())
        };
        match collider.shape() {
            Shape::Triangle(shape) => emit(shape.vertices())?,
            Shape::TriangleMesh(shape) => {
                for face in shape.triangles() {
                    emit(face.map(|i| shape.vertices()[i]))?;
                }
            }
            Shape::Convex(shape) => {
                for face in shape.faces() {
                    emit(face.map(|i| shape.vertices()[i]))?;
                }
            }
            Shape::Cuboid(shape) => {
                let h = shape.half_extents();
                for (normal, u, v) in [
                    (Vec3::X, Vec3::Y, Vec3::Z),
                    (-Vec3::X, Vec3::Z, Vec3::Y),
                    (Vec3::Y, Vec3::Z, Vec3::X),
                    (-Vec3::Y, Vec3::X, Vec3::Z),
                    (Vec3::Z, Vec3::X, Vec3::Y),
                    (-Vec3::Z, Vec3::Y, Vec3::X),
                ] {
                    let scale = |p: Vec3| Vec3::new(p.x * h.x, p.y * h.y, p.z * h.z);
                    let a = scale(normal - u - v);
                    let b = scale(normal + u - v);
                    let c = scale(normal + u + v);
                    let d = scale(normal - u + v);
                    emit([a, b, c])?;
                    emit([a, c, d])?;
                }
            }
            Shape::Sphere(shape) => {
                let point = |ring: usize, segment: usize| {
                    if ring == 0 {
                        return Vec3::Z * shape.radius();
                    }
                    if ring == config.sphere_rings - 1 {
                        return -Vec3::Z * shape.radius();
                    }
                    let theta =
                        std::f64::consts::PI * ring as f64 / (config.sphere_rings - 1) as f64;
                    let phi = std::f64::consts::TAU * (segment % config.radial_segments) as f64
                        / config.radial_segments as f64;
                    Vec3::new(
                        theta.sin() * phi.cos(),
                        theta.sin() * phi.sin(),
                        theta.cos(),
                    ) * shape.radius()
                };
                for ring in 0..config.sphere_rings - 1 {
                    for segment in 0..config.radial_segments {
                        let a = point(ring, segment);
                        let b = point(ring + 1, segment);
                        let c = point(ring + 1, segment + 1);
                        let d = point(ring, segment + 1);
                        if ring != config.sphere_rings - 2 {
                            emit([a, b, c])?;
                        }
                        if ring != 0 {
                            emit([a, c, d])?;
                        }
                    }
                }
            }
            Shape::Cylinder(shape) => {
                let axis = (shape.end() - shape.start())
                    .try_normalize()
                    .ok_or(AerodynamicsError::ArithmeticOverflow)?;
                let helper = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
                let u = axis
                    .cross(helper)
                    .try_normalize()
                    .ok_or(AerodynamicsError::ArithmeticOverflow)?;
                let v = axis.cross(u);
                let radial = |i: usize| {
                    let angle = std::f64::consts::TAU * (i % config.radial_segments) as f64
                        / config.radial_segments as f64;
                    (u * angle.cos() + v * angle.sin()) * shape.radius()
                };
                for i in 0..config.radial_segments {
                    let a = shape.start() + radial(i);
                    let b = shape.start() + radial(i + 1);
                    let c = shape.end() + radial(i + 1);
                    let d = shape.end() + radial(i);
                    emit([a, b, c])?;
                    emit([a, c, d])?;
                    emit([shape.start(), b, a])?;
                    emit([shape.end(), d, c])?;
                }
            }
            _ => return Err(AerodynamicsError::UnsupportedModel),
        }
    }
    if triangles.is_empty() {
        return Err(AerodynamicsError::UnsupportedModel);
    }
    TriangleMesh::new(vertices, triangles).map_err(|_| AerodynamicsError::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        shape::{Cuboid, Cylinder, Plane, Sphere},
        transform::Transform,
    };

    #[test]
    fn conversion_winding_budget_and_unsupported_shapes() -> Result<(), Box<dyn std::error::Error>>
    {
        let config = ColliderTessellation::default();
        assert!(ColliderTessellation::new(2, 8, 100).is_err());
        assert!(ColliderTessellation::new(16, 2, 100).is_err());
        assert!(ColliderTessellation::new(16, 8, 65_537).is_err());
        for shape in [
            Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0)?),
            Shape::Sphere(Sphere::new(1.0)?),
            Shape::Cylinder(Cylinder::new(-Vec3::Z, Vec3::Z, 1.0)?),
        ] {
            let collider = Collider::new(shape, Transform::identity());
            let mesh = collider_mesh([&collider], config)?;
            for face in mesh.triangles() {
                let [a, b, c] = face.map(|i| mesh.vertices()[i]);
                assert!((b - a).cross(c - a).dot((a + b + c) / 3.0) > 0.0);
            }
            assert_eq!(
                collider_mesh([&collider], ColliderTessellation::new(16, 8, 1)?).err(),
                Some(AerodynamicsError::TessellationBudgetExceeded)
            );
        }
        let plane = Collider::new(
            Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0)?),
            Transform::identity(),
        );
        assert_eq!(
            collider_mesh([&plane], config).err(),
            Some(AerodynamicsError::UnsupportedModel)
        );
        Ok(())
    }
}
