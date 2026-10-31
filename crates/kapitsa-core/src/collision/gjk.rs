use crate::{shape::Shape, transform::Transform};
use kapitsa_math::Vec3;

use super::{CollisionError, support::SupportPoint, support::support_pair};

const MAX_GJK_ITERATIONS: usize = 64;

#[derive(Clone, Copy)]
pub(crate) struct Simplex {
    pub(crate) points: [SupportPoint; 4],
    pub(crate) count: usize,
}

impl Simplex {
    fn new(point: SupportPoint) -> Self {
        Self {
            points: [SupportPoint::default(); 4],
            count: 1,
        }
        .set1(point)
    }

    fn set1(mut self, a: SupportPoint) -> Self {
        self.points[0] = a;
        self.count = 1;
        self
    }

    fn set2(&mut self, b: SupportPoint, a: SupportPoint) {
        self.points[0] = b;
        self.points[1] = a;
        self.count = 2;
    }

    fn set3(&mut self, c: SupportPoint, b: SupportPoint, a: SupportPoint) {
        self.points[0] = c;
        self.points[1] = b;
        self.points[2] = a;
        self.count = 3;
    }

    fn push(&mut self, point: SupportPoint) -> Result<(), CollisionError> {
        let slot = self
            .points
            .get_mut(self.count)
            .ok_or(CollisionError::NonConvergent)?;
        *slot = point;
        self.count += 1;
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub(crate) struct GjkResult {
    pub(crate) intersects: bool,
    pub(crate) simplex: Simplex,
}

pub(crate) fn intersect(
    shape_a: &Shape,
    transform_a: &Transform,
    shape_b: &Shape,
    transform_b: &Transform,
) -> Result<GjkResult, CollisionError> {
    let mut direction = transform_b
        .translation()
        .checked_sub(transform_a.translation())
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if direction.try_normalize().is_none() {
        direction = Vec3::X;
    }
    let first = support_pair(shape_a, transform_a, shape_b, transform_b, direction)?;
    let mut simplex = Simplex::new(first);
    direction = -first.difference;

    for _ in 0..MAX_GJK_ITERATIONS {
        let Some(unit_direction) = direction.try_normalize() else {
            return Ok(GjkResult {
                intersects: true,
                simplex,
            });
        };
        let point = support_pair(shape_a, transform_a, shape_b, transform_b, unit_direction)?;
        let advance = point
            .difference
            .checked_dot(unit_direction)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let tolerance = scale_tolerance(point.difference);
        if advance <= tolerance {
            return Ok(GjkResult {
                intersects: false,
                simplex,
            });
        }
        simplex.push(point)?;
        if update_simplex(&mut simplex, &mut direction)? {
            return Ok(GjkResult {
                intersects: true,
                simplex,
            });
        }
    }
    Err(CollisionError::NonConvergent)
}

fn update_simplex(simplex: &mut Simplex, direction: &mut Vec3) -> Result<bool, CollisionError> {
    match simplex.count {
        1 => {
            let a = simplex.points[0];
            *direction = -a.difference;
            Ok(false)
        }
        2 => process_line(simplex, direction),
        3 => process_triangle(simplex, direction),
        4 => process_tetrahedron(simplex, direction),
        _ => Err(CollisionError::NonConvergent),
    }
}

fn process_line(simplex: &mut Simplex, direction: &mut Vec3) -> Result<bool, CollisionError> {
    let a = simplex.points[1];
    let b = simplex.points[0];
    let ab = b.difference - a.difference;
    let ao = -a.difference;
    if ab.dot(ao) > 0.0 {
        simplex.set2(b, a);
        *direction = triple_product(ab, ao, ab);
        if direction.try_normalize().is_none() {
            *direction = perpendicular(ab);
        }
    } else {
        simplex.set1(a);
        *direction = ao;
    }
    if !direction.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(false)
}

fn process_triangle(simplex: &mut Simplex, direction: &mut Vec3) -> Result<bool, CollisionError> {
    let a = simplex.points[2];
    let b = simplex.points[1];
    let c = simplex.points[0];
    let ab = b.difference - a.difference;
    let ac = c.difference - a.difference;
    let ao = -a.difference;
    let abc = ab.cross(ac);
    if !abc.is_finite() || abc.try_normalize().is_none() {
        return Err(CollisionError::NonConvergent);
    }

    if abc.cross(ac).dot(ao) > 0.0 {
        if ac.dot(ao) > 0.0 {
            simplex.set2(c, a);
            *direction = triple_product(ac, ao, ac);
        } else if ab.dot(ao) > 0.0 {
            simplex.set2(b, a);
            *direction = triple_product(ab, ao, ab);
        } else {
            simplex.set1(a);
            *direction = ao;
        }
    } else if ab.cross(abc).dot(ao) > 0.0 {
        if ab.dot(ao) > 0.0 {
            simplex.set2(b, a);
            *direction = triple_product(ab, ao, ab);
        } else {
            simplex.set1(a);
            *direction = ao;
        }
    } else if abc.dot(ao) > 0.0 {
        *direction = abc;
    } else {
        simplex.set3(b, c, a);
        *direction = -abc;
    }

    if direction.try_normalize().is_none() && *direction != Vec3::ZERO {
        return Err(CollisionError::NonConvergent);
    }
    Ok(false)
}

fn process_tetrahedron(
    simplex: &mut Simplex,
    direction: &mut Vec3,
) -> Result<bool, CollisionError> {
    let a = simplex.points[3];
    let b = simplex.points[2];
    let c = simplex.points[1];
    let d = simplex.points[0];
    let ao = -a.difference;
    for (face, opposite) in [([a, b, c], d), ([a, c, d], b), ([a, d, b], c)] {
        let ab = face[1].difference - face[0].difference;
        let ac = face[2].difference - face[0].difference;
        let mut normal = ab.cross(ac);
        if !normal.is_finite() || normal.try_normalize().is_none() {
            return Err(CollisionError::NonConvergent);
        }
        if normal.dot(opposite.difference - face[0].difference) > 0.0 {
            normal = -normal;
        }
        let outside = normal
            .checked_dot(ao)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        if outside > scale_tolerance(normal) {
            simplex.set3(face[2], face[1], face[0]);
            return process_triangle(simplex, direction);
        }
    }
    Ok(true)
}

fn triple_product(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    a.cross(b).cross(c)
}

fn perpendicular(vector: Vec3) -> Vec3 {
    let axis = if vector.x.abs() <= vector.y.abs() && vector.x.abs() <= vector.z.abs() {
        Vec3::X
    } else if vector.y.abs() <= vector.z.abs() {
        Vec3::Y
    } else {
        Vec3::Z
    };
    vector.cross(axis).try_normalize().unwrap_or(Vec3::X)
}

fn scale_tolerance(value: Vec3) -> f64 {
    value.abs().max_element() * f64::EPSILON * 64.0
}

#[cfg(test)]
#[path = "gjk_tests.rs"]
mod tests;
