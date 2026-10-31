use crate::{shape::Shape, transform::Transform};
use kapitsa_math::Vec3;

use super::CollisionError;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct SupportPoint {
    pub(crate) difference: Vec3,
    pub(crate) point_a: Vec3,
    pub(crate) point_b: Vec3,
    pub(crate) feature_a: u64,
    pub(crate) feature_b: u64,
}

#[derive(Clone, Copy)]
struct LocalSupport {
    point: Vec3,
    feature: u64,
}

pub(crate) fn support_world(
    shape: &Shape,
    transform: &Transform,
    direction: Vec3,
) -> Result<(Vec3, u64), CollisionError> {
    if !direction.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let direction = direction.try_normalize().unwrap_or(Vec3::X);
    let local_direction = transform
        .rotation()
        .conjugate()
        .rotate_vector(direction)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let local = support_local(shape, local_direction)?;
    let world = transform.transform_point(local.point);
    if world.is_finite() {
        Ok((world, local.feature))
    } else {
        Err(CollisionError::ArithmeticOverflow)
    }
}

pub(crate) fn support_pair(
    shape_a: &Shape,
    transform_a: &Transform,
    shape_b: &Shape,
    transform_b: &Transform,
    direction: Vec3,
) -> Result<SupportPoint, CollisionError> {
    let (point_a, feature_a) = support_world(shape_a, transform_a, direction)?;
    let (point_b, feature_b) = support_world(shape_b, transform_b, -direction)?;
    let difference = point_a
        .checked_sub(point_b)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    Ok(SupportPoint {
        difference,
        point_a,
        point_b,
        feature_a,
        feature_b,
    })
}

fn support_local(shape: &Shape, direction: Vec3) -> Result<LocalSupport, CollisionError> {
    let support = match shape {
        Shape::Sphere(shape) => {
            let point = direction
                .checked_mul_scalar(shape.radius())
                .ok_or(CollisionError::ArithmeticOverflow)?;
            LocalSupport { point, feature: 0 }
        }
        Shape::Cuboid(shape) => {
            let half = shape.half_extents();
            let feature = u64::from(direction.x >= 0.0)
                | (u64::from(direction.y >= 0.0) << 1)
                | (u64::from(direction.z >= 0.0) << 2);
            LocalSupport {
                point: Vec3::new(
                    if direction.x >= 0.0 { half.x } else { -half.x },
                    if direction.y >= 0.0 { half.y } else { -half.y },
                    if direction.z >= 0.0 { half.z } else { -half.z },
                ),
                feature,
            }
        }
        Shape::Capsule(shape) => {
            let start_score = direction
                .checked_dot(shape.start())
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let end_score = direction
                .checked_dot(shape.end())
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let (center, feature) = if end_score > start_score {
                (shape.end(), 1)
            } else {
                (shape.start(), 0)
            };
            let radial = direction
                .checked_mul_scalar(shape.radius())
                .ok_or(CollisionError::ArithmeticOverflow)?;
            LocalSupport {
                point: center
                    .checked_add(radial)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
                feature,
            }
        }
        Shape::Cylinder(shape) => {
            let (_, radial, axial) = decompose_axis(direction, shape.start(), shape.end())?;
            let center = if axial > 0.0 {
                shape.end()
            } else {
                shape.start()
            };
            let radial_direction = radial.try_normalize().unwrap_or(Vec3::ZERO);
            let radial = radial_direction
                .checked_mul_scalar(shape.radius())
                .ok_or(CollisionError::ArithmeticOverflow)?;
            LocalSupport {
                point: center
                    .checked_add(radial)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
                feature: u64::from(axial > 0.0),
            }
        }
        Shape::TaperedCapsule(shape) => {
            let start_score =
                weighted_endpoint_score(direction, shape.start(), shape.start_radius())?;
            let end_score = weighted_endpoint_score(direction, shape.end(), shape.end_radius())?;
            let (center, radius, feature) = if end_score > start_score {
                (shape.end(), shape.end_radius(), 1)
            } else {
                (shape.start(), shape.start_radius(), 0)
            };
            LocalSupport {
                point: center
                    .checked_add(
                        direction
                            .checked_mul_scalar(radius)
                            .ok_or(CollisionError::ArithmeticOverflow)?,
                    )
                    .ok_or(CollisionError::ArithmeticOverflow)?,
                feature,
            }
        }
        Shape::TaperedCylinder(shape) => {
            let (_, radial, _) = decompose_axis(direction, shape.start(), shape.end())?;
            let radial_length = radial
                .checked_length()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let start_score = weighted_cylinder_score(
                direction,
                shape.start(),
                radial_length,
                shape.start_radius(),
            )?;
            let end_score =
                weighted_cylinder_score(direction, shape.end(), radial_length, shape.end_radius())?;
            let (center, radius, feature) = if end_score > start_score {
                (shape.end(), shape.end_radius(), 1)
            } else {
                (shape.start(), shape.start_radius(), 0)
            };
            let radial_direction = radial.try_normalize().unwrap_or(Vec3::ZERO);
            let radial = radial_direction
                .checked_mul_scalar(radius)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            LocalSupport {
                point: center
                    .checked_add(radial)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
                feature,
            }
        }
        Shape::Convex(shape) => {
            let mut vertices = shape.vertices().iter().copied().enumerate();
            let (mut feature, mut point) = vertices.next().ok_or(CollisionError::InvalidContact)?;
            let mut best_score = direction
                .checked_dot(point)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            for (index, candidate) in vertices {
                let score = direction
                    .checked_dot(candidate)
                    .ok_or(CollisionError::ArithmeticOverflow)?;
                if score > best_score {
                    feature = index;
                    point = candidate;
                    best_score = score;
                }
            }
            LocalSupport {
                point,
                feature: u64::try_from(feature).map_err(|_| CollisionError::ArithmeticOverflow)?,
            }
        }
        Shape::Triangle(shape) => {
            let vertices = shape.vertices();
            let mut feature = 0;
            let mut point = vertices[0];
            let mut best_score = direction
                .checked_dot(point)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            for (index, candidate) in vertices.into_iter().enumerate().skip(1) {
                let score = direction
                    .checked_dot(candidate)
                    .ok_or(CollisionError::ArithmeticOverflow)?;
                if score > best_score {
                    feature = index;
                    point = candidate;
                    best_score = score;
                }
            }
            LocalSupport {
                point,
                feature: u64::try_from(feature).map_err(|_| CollisionError::ArithmeticOverflow)?,
            }
        }
        Shape::Plane(_)
        | Shape::TriangleMesh(_)
        | Shape::HeightField(_)
        | Shape::LevelSet(_)
        | Shape::SkinnedTriangleMesh(_) => {
            return Err(CollisionError::UnsupportedGeometryPair);
        }
    };
    if support.point.is_finite() {
        Ok(support)
    } else {
        Err(CollisionError::ArithmeticOverflow)
    }
}

fn decompose_axis(
    direction: Vec3,
    start: Vec3,
    end: Vec3,
) -> Result<(Vec3, Vec3, f64), CollisionError> {
    let axis = end
        .checked_sub(start)
        .and_then(Vec3::try_normalize)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let axial = direction
        .checked_dot(axis)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let radial = direction
        .checked_sub(axis * axial)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    Ok((axis, radial, axial))
}

fn weighted_endpoint_score(
    direction: Vec3,
    center: Vec3,
    radius: f64,
) -> Result<f64, CollisionError> {
    let value = direction
        .checked_dot(center)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let length = direction
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let score = value + radius * length;
    score
        .is_finite()
        .then_some(score)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn weighted_cylinder_score(
    direction: Vec3,
    center: Vec3,
    radial_length: f64,
    radius: f64,
) -> Result<f64, CollisionError> {
    let value = direction
        .checked_dot(center)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let score = value + radius * radial_length;
    score
        .is_finite()
        .then_some(score)
        .ok_or(CollisionError::ArithmeticOverflow)
}

#[cfg(test)]
#[path = "support_tests.rs"]
mod tests;
