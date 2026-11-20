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
    ) -> Result<Self, crate::Error> {
        if !(3..=128).contains(&radial_segments)
            || !(3..=128).contains(&sphere_rings)
            || !(1..=65_536).contains(&max_triangles)
        {
            return Err(crate::Error::from(AerodynamicsError::InvalidTessellation));
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

fn emit_capped_cylinder(
    start: Vec3,
    end: Vec3,
    start_radius: f64,
    end_radius: f64,
    segments: usize,
    emit: &mut impl FnMut([Vec3; 3]) -> Result<(), crate::Error>,
) -> Result<(), crate::Error> {
    let axis = (end - start)
        .try_normalize()
        .ok_or(AerodynamicsError::ArithmeticOverflow)?;
    let helper = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let u = axis
        .cross(helper)
        .try_normalize()
        .ok_or(AerodynamicsError::ArithmeticOverflow)?;
    let v = axis.cross(u);
    let radial = |i: usize| {
        let angle = std::f64::consts::TAU * (i % segments) as f64 / segments as f64;
        u * angle.cos() + v * angle.sin()
    };
    for i in 0..segments {
        let first = radial(i);
        let second = radial(i + 1);
        let a = start + first * start_radius;
        let b = start + second * start_radius;
        let c = end + second * end_radius;
        let d = end + first * end_radius;
        emit([a, b, c])?;
        emit([a, c, d])?;
        emit([start, b, a])?;
        emit([end, d, c])?;
    }
    Ok(())
}

fn emit_capsule(
    start: Vec3,
    end: Vec3,
    start_radius: f64,
    end_radius: f64,
    config: ColliderTessellation,
    emit: &mut impl FnMut([Vec3; 3]) -> Result<(), crate::Error>,
) -> Result<(), crate::Error> {
    let segment = end - start;
    let length = segment.length();
    let axis = segment
        .try_normalize()
        .ok_or(AerodynamicsError::ArithmeticOverflow)?;
    let helper = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let u = axis
        .cross(helper)
        .try_normalize()
        .ok_or(AerodynamicsError::ArithmeticOverflow)?;
    let v = axis.cross(u);
    // A common supporting plane touches both endpoint spheres when its
    // normal n satisfies n·(end − start) = start_radius − end_radius.
    let seam_cos = (start_radius - end_radius) / length;
    if !seam_cos.is_finite() || seam_cos.abs() >= 1.0 {
        return Err(AerodynamicsError::ArithmeticOverflow.into());
    }
    let seam_sin = ((1.0 - seam_cos) * (1.0 + seam_cos)).sqrt();
    let seam_angle = seam_sin.atan2(seam_cos);
    let cap_steps = config.sphere_rings / 2;
    let last = 2 * cap_steps + 1;
    let point = |ring: usize, i: usize| {
        if ring == 0 {
            return end + axis * end_radius;
        }
        if ring == last {
            return start - axis * start_radius;
        }
        let phi = std::f64::consts::TAU * (i % config.radial_segments) as f64
            / config.radial_segments as f64;
        let radial = u * phi.cos() + v * phi.sin();
        let (center, radius, angle) = if ring <= cap_steps {
            (end, end_radius, seam_angle * ring as f64 / cap_steps as f64)
        } else {
            let fraction = (ring - cap_steps - 1) as f64 / cap_steps as f64;
            (
                start,
                start_radius,
                seam_angle + (std::f64::consts::PI - seam_angle) * fraction,
            )
        };
        let (sin, cos) = if ring == cap_steps || ring == cap_steps + 1 {
            (seam_sin, seam_cos)
        } else {
            angle.sin_cos()
        };
        center + (radial * sin + axis * cos) * radius
    };
    emit_latitude_mesh(last + 1, config.radial_segments, point, emit)
}

fn emit_latitude_mesh(
    rings: usize,
    segments: usize,
    point: impl Fn(usize, usize) -> Vec3,
    emit: &mut impl FnMut([Vec3; 3]) -> Result<(), crate::Error>,
) -> Result<(), crate::Error> {
    for ring in 0..rings - 1 {
        for segment in 0..segments {
            let a = point(ring, segment);
            let b = point(ring + 1, segment);
            let c = point(ring + 1, segment + 1);
            let d = point(ring, segment + 1);
            if ring != rings - 2 {
                emit([a, b, c])?;
            }
            if ring != 0 {
                emit([a, c, d])?;
            }
        }
    }
    Ok(())
}

/// Converts real surfaces into one body-local mesh. Collider transforms are baked
/// in, not world poses or COM offsets. Source winding is preserved for meshes.
/// Empty sets and unsupported shapes are rejected, never approximated by AABBs.
pub fn collider_mesh<'a>(
    colliders: impl IntoIterator<Item = &'a Collider>,
    config: ColliderTessellation,
) -> Result<TriangleMesh, crate::Error> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for collider in colliders {
        let count = match collider.shape() {
            Shape::Cuboid(_) => 12,
            Shape::Sphere(_) => 2 * config.radial_segments * (config.sphere_rings - 2),
            Shape::Cylinder(_) | Shape::TaperedCylinder(_) => 4 * config.radial_segments,
            Shape::Capsule(_) | Shape::TaperedCapsule(_) => {
                4 * config.radial_segments * (config.sphere_rings / 2)
            }
            Shape::Triangle(_) => 1,
            Shape::Convex(shape) => shape.faces().len(),
            Shape::TriangleMesh(shape) => shape.triangles().len(),
            Shape::SkinnedTriangleMesh(shape) => shape.mesh().triangles().len(),
            Shape::HeightField(shape) => (shape.rows() - 1)
                .checked_mul(shape.columns() - 1)
                .and_then(|cells| cells.checked_mul(2))
                .ok_or(AerodynamicsError::TessellationBudgetExceeded)?,
            _ => return Err(crate::Error::from(AerodynamicsError::UnsupportedModel)),
        };
        if count > config.max_triangles - triangles.len() {
            return Err(crate::Error::from(
                AerodynamicsError::TessellationBudgetExceeded,
            ));
        }
        // Use independent indexed triangles, welded explicitly by world geometry
        // preparation if closure is required. This also bounds unreferenced input.
        vertices
            .try_reserve(count * 3)
            .map_err(|_| crate::Error::from(AerodynamicsError::ArithmeticOverflow))?;
        triangles
            .try_reserve(count)
            .map_err(|_| crate::Error::from(AerodynamicsError::ArithmeticOverflow))?;
        let mut emit = |points: [Vec3; 3]| -> Result<(), crate::Error> {
            let base = vertices.len();
            for point in points {
                let point = collider.local_transform().transform_point(point);
                if !point.is_finite() {
                    return Err(crate::Error::from(AerodynamicsError::ArithmeticOverflow));
                }
                vertices.push(point);
            }
            triangles.push([base, base + 1, base + 2]);
            Ok(())
        };
        match collider.shape() {
            Shape::HeightField(shape) => {
                for row in 0..shape.rows() - 1 {
                    for column in 0..shape.columns() - 1 {
                        for triangle in
                            crate::collision::height_field::cell_triangles(shape, row, column)?
                        {
                            emit(triangle)?;
                        }
                    }
                }
            }
            Shape::Triangle(shape) => emit(shape.vertices())?,
            Shape::TriangleMesh(shape) => {
                for face in shape.triangles() {
                    emit(face.map(|i| shape.vertices()[i]))?;
                }
            }
            Shape::SkinnedTriangleMesh(shape) => {
                let mesh = shape.mesh();
                let vertices = collider.query_vertices().unwrap_or(mesh.vertices());
                for face in mesh.triangles() {
                    emit(face.map(|i| vertices[i]))?;
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
                emit_latitude_mesh(
                    config.sphere_rings,
                    config.radial_segments,
                    point,
                    &mut emit,
                )?;
            }
            Shape::Capsule(shape) => emit_capsule(
                shape.start(),
                shape.end(),
                shape.radius(),
                shape.radius(),
                config,
                &mut emit,
            )?,
            Shape::TaperedCapsule(shape) => emit_capsule(
                shape.start(),
                shape.end(),
                shape.start_radius(),
                shape.end_radius(),
                config,
                &mut emit,
            )?,
            Shape::Cylinder(shape) => emit_capped_cylinder(
                shape.start(),
                shape.end(),
                shape.radius(),
                shape.radius(),
                config.radial_segments,
                &mut emit,
            )?,
            Shape::TaperedCylinder(shape) => emit_capped_cylinder(
                shape.start(),
                shape.end(),
                shape.start_radius(),
                shape.end_radius(),
                config.radial_segments,
                &mut emit,
            )?,
            _ => return Err(crate::Error::from(AerodynamicsError::UnsupportedModel)),
        }
    }
    if triangles.is_empty() {
        return Err(crate::Error::from(AerodynamicsError::UnsupportedModel));
    }
    TriangleMesh::new(vertices, triangles)
        .map_err(|_| crate::Error::from(AerodynamicsError::ArithmeticOverflow))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        aerodynamics::geometry::{
            GeometryConfig, GeometryInstance, GeometryOwner, OpenMeshPolicy, PreparedGeometry,
        },
        shape::{
            Capsule, Cuboid, Cylinder, HeightField, Plane, Sphere, TaperedCapsule, TaperedCylinder,
        },
        transform::Transform,
    };
    use kapitsa_math::Affine3;

    #[test]
    fn skinned_mesh_conversion_uses_committed_pose() -> Result<(), crate::Error> {
        use crate::shape::{BoneInfluence, SkinnedTriangleMesh};

        let reference = vec![Vec3::ZERO, Vec3::X, Vec3::Y];
        let mesh = TriangleMesh::new(reference.clone(), vec![[0, 1, 2]])?;
        let skin = SkinnedTriangleMesh::new(
            mesh,
            vec![String::from("root")],
            (0..3)
                .map(|_| BoneInfluence::new(0, 1.0).map(|influence| vec![influence]))
                .collect::<Result<_, _>>()?,
            Affine3::IDENTITY,
            vec![Affine3::IDENTITY],
        )?;
        let mut collider = Collider::new(Shape::SkinnedTriangleMesh(skin), Transform::identity());
        let config = ColliderTessellation::new(3, 3, 1)?;
        assert_eq!(collider_mesh([&collider], config)?.vertices(), reference);
        let displacement = Vec3::Z * 2.0;
        collider.set_skin_pose(&[Affine3::from_translation(displacement)])?;
        let posed = collider_mesh([&collider], config)?;
        assert!(
            posed
                .vertices()
                .iter()
                .copied()
                .eq(reference.into_iter().map(|v| v + displacement))
        );
        assert_eq!(posed.triangles(), &[[0, 1, 2]]);
        Ok(())
    }

    #[test]
    fn height_field_mesh_uses_collision_diagonal_and_budget() -> Result<(), crate::Error> {
        let field = HeightField::from_vec(
            2,
            2,
            kapitsa_math::Vec2::new(2.0, 3.0),
            vec![0.0, 0.5, 0.25, 1.0],
        )?;
        let collider = Collider::new(Shape::HeightField(field), Transform::identity());
        let mesh = collider_mesh([&collider], ColliderTessellation::new(3, 3, 2)?)?;
        let a = Vec3::ZERO;
        let b = Vec3::new(2.0, 0.0, 0.5);
        let c = Vec3::new(2.0, 3.0, 1.0);
        let d = Vec3::new(0.0, 3.0, 0.25);
        assert!(
            mesh.triangles()
                .iter()
                .map(|face| face.map(|i| mesh.vertices()[i]))
                .eq([[a, b, c], [a, c, d]])
        );
        assert_eq!(
            collider_mesh([&collider], ColliderTessellation::new(3, 3, 1)?).err(),
            Some(AerodynamicsError::TessellationBudgetExceeded.into())
        );
        Ok(())
    }

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
            Shape::TaperedCylinder(TaperedCylinder::new(-Vec3::Z, Vec3::Z, 1.0, 0.5)?),
            Shape::Capsule(Capsule::new(-Vec3::Z, Vec3::Z, 1.0)?),
            Shape::TaperedCapsule(TaperedCapsule::new(-Vec3::Z, Vec3::Z, 1.0, 0.5)?),
        ] {
            let collider = Collider::new(shape, Transform::identity());
            let mesh = collider_mesh([&collider], config)?;
            for face in mesh.triangles() {
                let [a, b, c] = face.map(|i| mesh.vertices()[i]);
                assert!((b - a).cross(c - a).dot((a + b + c) / 3.0) > 0.0);
            }
            assert_eq!(
                collider_mesh([&collider], ColliderTessellation::new(16, 8, 1)?).err(),
                Some(crate::Error::from(
                    AerodynamicsError::TessellationBudgetExceeded
                ))
            );
        }
        let plane = Collider::new(
            Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0)?),
            Transform::identity(),
        );
        assert_eq!(
            collider_mesh([&plane], config).err(),
            Some(crate::Error::from(AerodynamicsError::UnsupportedModel))
        );
        Ok(())
    }

    #[test]
    fn tapered_capsule_mesh_joins_caps_on_common_tangent() -> Result<(), crate::Error> {
        let start = -Vec3::Z;
        let end = Vec3::Z;
        let collider = Collider::new(
            Shape::TaperedCapsule(TaperedCapsule::new(start, end, 1.0, 0.5)?),
            Transform::identity(),
        );
        let mesh = collider_mesh([&collider], ColliderTessellation::new(12, 8, 192)?)?;
        assert_eq!(mesh.triangles().len(), 192);
        let normal = Vec3::new(0.0, (15.0_f64 / 16.0).sqrt(), 0.25);
        for expected in [
            start + normal,
            end + normal * 0.5,
            start - Vec3::Z,
            end + Vec3::Z * 0.5,
        ] {
            assert!(
                mesh.vertices()
                    .iter()
                    .any(|point| (*point - expected).length() < 1e-14)
            );
        }
        let support = start.dot(normal) + 1.0;
        assert!(
            mesh.vertices()
                .iter()
                .all(|point| point.dot(normal) <= support + 1e-14)
        );
        Ok(())
    }

    #[test]
    fn tapered_cylinder_mesh_uses_distinct_radii_at_both_caps() -> Result<(), crate::Error> {
        let collider = Collider::new(
            Shape::TaperedCylinder(TaperedCylinder::new(-Vec3::Z, Vec3::Z, 1.0, 0.5)?),
            Transform::identity(),
        );
        let segments = 12;
        let mesh = collider_mesh(
            [&collider],
            ColliderTessellation::new(segments, 8, 4 * segments)?,
        )?;
        assert_eq!(mesh.triangles().len(), 4 * segments);
        let mut rim_points = [0; 2];
        for vertex in mesh.vertices() {
            assert_eq!(vertex.z.abs(), 1.0);
            let side = usize::from(vertex.z > 0.0);
            let radial = vertex.x.hypot(vertex.y);
            if radial != 0.0 {
                assert!((radial - [1.0, 0.5][side]).abs() < 1e-14);
                rim_points[side] += 1;
            }
        }
        assert_eq!(rim_points, [5 * segments; 2]);
        Ok(())
    }

    #[test]
    fn capsule_mesh_fills_both_caps_and_barrel_after_transform() -> Result<(), crate::Error> {
        let offset = Vec3::new(2.0, 3.0, 4.0);
        let collider = Collider::new(
            Shape::Capsule(Capsule::new(-Vec3::X, Vec3::X, 0.5)?),
            Transform::new(offset, kapitsa_math::Quat::IDENTITY)?,
        );
        let triangles = 4 * 12 * (8 / 2);
        let mesh = collider_mesh([&collider], ColliderTessellation::new(12, 8, triangles)?)?;
        assert_eq!(mesh.triangles().len(), triangles);
        assert_eq!(
            collider_mesh(
                [&collider],
                ColliderTessellation::new(12, 8, triangles - 1)?
            )
            .err(),
            Some(AerodynamicsError::TessellationBudgetExceeded.into())
        );
        let geometry = PreparedGeometry::prepare(
            &[GeometryInstance {
                mesh: &mesh,
                local_transform: Transform::identity(),
                owner: GeometryOwner::Body,
            }],
            GeometryConfig {
                cell_size: 0.05,
                max_cells: 100_000,
                max_work: 10_000_000,
                open_mesh_policy: OpenMeshPolicy::Reject,
                weld_tolerance: Some(1e-10),
                max_patch_work: 1,
            },
        )?;
        for x in [-1.25, 0.0, 1.25] {
            assert!(geometry.occupied_at(offset + Vec3::X * x)?);
        }
        assert!(!geometry.occupied_at(offset + Vec3::Y * 0.8)?);
        Ok(())
    }
}
