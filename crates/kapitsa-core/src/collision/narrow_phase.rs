use crate::{
    collider::CollisionMode,
    shape::{Plane, Shape, Triangle},
    transform::Transform,
};
use kapitsa_math::Vec3;

use super::{
    CollisionError, CollisionPair,
    epa::EpaWorkspace,
    gjk,
    manifold::{ContactManifold, ContactPoint, combine_feature_ids},
    support::{support_pair, support_world},
};

/// Reusable scratch for narrow-phase contact generation.
pub(crate) struct NarrowPhase {
    epa: EpaWorkspace,
}

impl NarrowPhase {
    pub(crate) const fn new() -> Self {
        Self {
            epa: EpaWorkspace::new(),
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn detect(
        &mut self,
        pair: CollisionPair,
        shape_a: &Shape,
        collider_a: &Transform,
        body_a: &Transform,
        mode_a: CollisionMode,
        shape_b: &Shape,
        collider_b: &Transform,
        body_b: &Transform,
        mode_b: CollisionMode,
        contact_offset: f64,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        if !contact_offset.is_finite() || contact_offset < 0.0 {
            return Err(CollisionError::InvalidConfig);
        }
        let contact_offset = if mode_a == CollisionMode::Sensor || mode_b == CollisionMode::Sensor {
            0.0
        } else {
            contact_offset
        };
        let pair_mode = if mode_a == CollisionMode::Sensor || mode_b == CollisionMode::Sensor {
            CollisionMode::Sensor
        } else {
            CollisionMode::Solid
        };

        match (shape_a, shape_b) {
            (Shape::Plane(plane), convex) if convex.is_convex() => self.plane_convex(
                pair,
                plane,
                collider_a,
                body_a,
                convex,
                collider_b,
                body_b,
                pair_mode,
                contact_offset,
                true,
            ),
            (convex, Shape::Plane(plane)) if convex.is_convex() => self.plane_convex(
                pair,
                plane,
                collider_b,
                body_b,
                convex,
                collider_a,
                body_a,
                pair_mode,
                contact_offset,
                false,
            ),
            (Shape::Triangle(triangle), convex) if convex.is_convex() => self.triangle_convex(
                pair,
                triangle,
                collider_a,
                convex,
                collider_b,
                body_a,
                body_b,
                true,
                contact_offset,
            ),
            (convex, Shape::Triangle(triangle)) if convex.is_convex() => self.triangle_convex(
                pair,
                triangle,
                collider_b,
                convex,
                collider_a,
                body_a,
                body_b,
                false,
                contact_offset,
            ),
            (a, b) if a.is_convex() && b.is_convex() => {
                self.convex_convex(pair, a, collider_a, body_a, b, collider_b, body_b)
            }
            _ => Err(CollisionError::UnsupportedGeometryPair),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn plane_convex(
        &self,
        pair: CollisionPair,
        plane: &Plane,
        plane_transform: &Transform,
        plane_body: &Transform,
        convex: &Shape,
        convex_transform: &Transform,
        convex_body: &Transform,
        mode: CollisionMode,
        contact_offset: f64,
        plane_is_a: bool,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        let local_normal = plane.normal();
        let plane_normal = plane_transform
            .transform_vector(local_normal)
            .try_normalize()
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let local_point = local_normal
            .checked_mul_scalar(plane.distance())
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let plane_point = plane_transform.transform_point(local_point);
        let center = shape_center(convex, convex_transform)?;
        if !plane_point.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let center_distance = center
            .checked_sub(plane_point)
            .and_then(|delta| delta.checked_dot(plane_normal))
            .ok_or(CollisionError::ArithmeticOverflow)?;

        if mode == CollisionMode::Solid && center_distance < 0.0 {
            return Ok(None);
        }
        let normal_plane_to_convex = if mode == CollisionMode::Sensor && center_distance < 0.0 {
            -plane_normal
        } else {
            plane_normal
        };
        let (convex_point, feature) =
            support_world(convex, convex_transform, -normal_plane_to_convex)?;
        let separation = convex_point
            .checked_sub(plane_point)
            .and_then(|delta| delta.checked_dot(normal_plane_to_convex))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        if separation > contact_offset {
            return Ok(None);
        }
        let plane_contact = convex_point
            .checked_sub(
                normal_plane_to_convex
                    .checked_mul_scalar(separation)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
            )
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let penetration = (-separation).max(0.0);
        let (point_a, point_b, normal, body_transform_a, body_transform_b, feature_id) =
            if plane_is_a {
                (
                    plane_contact,
                    convex_point,
                    normal_plane_to_convex,
                    plane_body,
                    convex_body,
                    combine_feature_ids(0, feature),
                )
            } else {
                (
                    convex_point,
                    plane_contact,
                    -normal_plane_to_convex,
                    convex_body,
                    plane_body,
                    combine_feature_ids(feature, 0),
                )
            };
        make_manifold(
            pair,
            body_transform_a,
            body_transform_b,
            point_a,
            point_b,
            normal,
            penetration,
            feature_id,
        )
        .map(Some)
    }

    #[allow(clippy::too_many_arguments)]
    fn triangle_convex(
        &self,
        pair: CollisionPair,
        triangle: &Triangle,
        triangle_transform: &Transform,
        convex: &Shape,
        convex_transform: &Transform,
        body_a: &Transform,
        body_b: &Transform,
        triangle_is_a: bool,
        contact_offset: f64,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        let vertices = triangle
            .vertices()
            .map(|vertex| triangle_transform.transform_point(vertex));
        if !vertices.iter().all(|vertex| vertex.is_finite()) {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let edge_ab = vertices[1]
            .checked_sub(vertices[0])
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let edge_ac = vertices[2]
            .checked_sub(vertices[0])
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let face_normal = edge_ab
            .checked_cross(edge_ac)
            .and_then(Vec3::try_normalize)
            .ok_or(CollisionError::InvalidContact)?;
        let center = shape_center(convex, convex_transform)?;
        let closest = closest_point_on_triangle(center, vertices)?;
        let radial = center
            .checked_sub(closest)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let normal_triangle_to_convex = radial.try_normalize().unwrap_or(face_normal);
        let (convex_point, convex_feature) =
            support_world(convex, convex_transform, -normal_triangle_to_convex)?;
        let triangle_point = closest_point_on_triangle(convex_point, vertices)?;
        let separation = convex_point
            .checked_sub(triangle_point)
            .and_then(|delta| delta.checked_dot(normal_triangle_to_convex))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        if separation > contact_offset {
            return Ok(None);
        }
        let penetration = (-separation).max(0.0);
        let (point_a, point_b, normal, body_transform_a, body_transform_b, feature_id) =
            if triangle_is_a {
                (
                    triangle_point,
                    convex_point,
                    normal_triangle_to_convex,
                    body_a,
                    body_b,
                    combine_feature_ids(0, convex_feature),
                )
            } else {
                (
                    convex_point,
                    triangle_point,
                    -normal_triangle_to_convex,
                    body_a,
                    body_b,
                    combine_feature_ids(convex_feature, 0),
                )
            };
        make_manifold(
            pair,
            body_transform_a,
            body_transform_b,
            point_a,
            point_b,
            normal,
            penetration,
            feature_id,
        )
        .map(Some)
    }

    #[allow(clippy::too_many_arguments)]
    fn convex_convex(
        &mut self,
        pair: CollisionPair,
        shape_a: &Shape,
        collider_a: &Transform,
        body_a: &Transform,
        shape_b: &Shape,
        collider_b: &Transform,
        body_b: &Transform,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        let result = gjk::intersect(shape_a, collider_a, shape_b, collider_b)?;
        if !result.intersects {
            return Ok(None);
        }
        let contact = if result.simplex.count == 4 {
            self.epa
                .solve(&result.simplex, shape_a, collider_a, shape_b, collider_b)?
        } else {
            let center_delta = collider_b
                .translation()
                .checked_sub(collider_a.translation())
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let normal = center_delta.try_normalize().unwrap_or(Vec3::X);
            let support = support_pair(shape_a, collider_a, shape_b, collider_b, normal)?;
            let distance = support
                .difference
                .checked_length()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let scale = support.difference.abs().max_element().max(1.0);
            if distance > scale * f64::EPSILON * 128.0 {
                return Err(CollisionError::NonConvergent);
            }
            super::epa::EpaResult {
                normal,
                penetration: 0.0,
                point_a: support.point_a,
                point_b: support.point_b,
                feature_a: support.feature_a,
                feature_b: support.feature_b,
            }
        };
        make_manifold(
            pair,
            body_a,
            body_b,
            contact.point_a,
            contact.point_b,
            contact.normal,
            contact.penetration,
            combine_feature_ids(contact.feature_a, contact.feature_b),
        )
        .map(Some)
    }
}

fn shape_center(shape: &Shape, transform: &Transform) -> Result<Vec3, CollisionError> {
    let bounds = shape
        .local_bounds()
        .map_err(|_| CollisionError::GeometryBounds)?
        .ok_or(CollisionError::UnsupportedGeometryPair)?;
    let center = transform.transform_point(bounds.center());
    center
        .is_finite()
        .then_some(center)
        .ok_or(CollisionError::ArithmeticOverflow)
}

#[allow(clippy::too_many_arguments)]
fn make_manifold(
    pair: CollisionPair,
    body_a: &Transform,
    body_b: &Transform,
    point_a: Vec3,
    point_b: Vec3,
    normal: Vec3,
    penetration: f64,
    feature_id: u64,
) -> Result<ContactManifold, CollisionError> {
    let contact = ContactPoint::from_world(
        body_a,
        body_b,
        point_a,
        point_b,
        normal,
        penetration,
        feature_id,
    )?;
    let position = point_a * 0.5 + point_b * 0.5;
    let mut manifold = ContactManifold::new(pair);
    manifold.insert(contact, position)?;
    Ok(manifold)
}

fn closest_point_on_triangle(point: Vec3, vertices: [Vec3; 3]) -> Result<Vec3, CollisionError> {
    let [a, b, c] = vertices;
    let ab = b.checked_sub(a).ok_or(CollisionError::ArithmeticOverflow)?;
    let ac = c.checked_sub(a).ok_or(CollisionError::ArithmeticOverflow)?;
    let ap = point
        .checked_sub(a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d1 = ab
        .checked_dot(ap)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d2 = ac
        .checked_dot(ap)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if d1 <= 0.0 && d2 <= 0.0 {
        return Ok(a);
    }

    let bp = point
        .checked_sub(b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d3 = ab
        .checked_dot(bp)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d4 = ac
        .checked_dot(bp)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if d3 >= 0.0 && d4 <= d3 {
        return Ok(b);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc.is_finite() && vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return interpolate(a, ab, v);
    }

    let cp = point
        .checked_sub(c)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d5 = ab
        .checked_dot(cp)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d6 = ac
        .checked_dot(cp)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if d6 >= 0.0 && d5 <= d6 {
        return Ok(c);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb.is_finite() && vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return interpolate(a, ac, w);
    }

    let va = d3 * d6 - d5 * d4;
    if va.is_finite() && va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let edge = c.checked_sub(b).ok_or(CollisionError::ArithmeticOverflow)?;
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return interpolate(b, edge, w);
    }

    let denominator = va + vb + vc;
    if !denominator.is_finite() || denominator.abs() <= f64::MIN_POSITIVE {
        return Err(CollisionError::NonConvergent);
    }
    let inverse = 1.0 / denominator;
    let v = vb * inverse;
    let w = vc * inverse;
    let result = a + ab * v + ac * w;
    result
        .is_finite()
        .then_some(result)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn interpolate(origin: Vec3, edge: Vec3, amount: f64) -> Result<Vec3, CollisionError> {
    let result = origin
        .checked_add(
            edge.checked_mul_scalar(amount)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    result
        .is_finite()
        .then_some(result)
        .ok_or(CollisionError::ArithmeticOverflow)
}

#[cfg(test)]
#[path = "narrow_phase_tests.rs"]
mod tests;
