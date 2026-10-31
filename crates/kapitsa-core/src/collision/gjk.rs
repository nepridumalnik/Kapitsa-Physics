use crate::{shape::Shape, transform::Transform};
use kapitsa_math::Vec3;

use super::{CollisionError, support::SupportPoint, support::support_pair};

const MAX_GJK_ITERATIONS: usize = 64;
const MAX_GJK_DISTANCE_ITERATIONS: usize = 128;

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

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GjkDistance {
    pub(crate) intersects: bool,
    pub(crate) distance: f64,
    pub(crate) lower_bound: f64,
    pub(crate) normal: Vec3,
    pub(crate) point_a: Vec3,
    pub(crate) point_b: Vec3,
}

#[derive(Clone, Copy)]
struct ClosestSimplex {
    simplex: Simplex,
    point: Vec3,
    point_a: Vec3,
    point_b: Vec3,
    distance_squared: f64,
}

pub(crate) fn distance(
    shape_a: &Shape,
    transform_a: &Transform,
    shape_b: &Shape,
    transform_b: &Transform,
) -> Result<GjkDistance, CollisionError> {
    let mut direction = transform_b
        .translation()
        .checked_sub(transform_a.translation())
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if direction.try_normalize().is_none() {
        direction = Vec3::X;
    }
    let first = support_pair(shape_a, transform_a, shape_b, transform_b, direction)?;
    let mut simplex = Simplex::new(first);
    let mut closest = closest_simplex(&simplex)?;
    let mut lower_bound = 0.0;

    for _ in 0..MAX_GJK_DISTANCE_ITERATIONS {
        let scale = closest
            .simplex
            .points
            .iter()
            .take(closest.simplex.count)
            .map(|point| point.difference.abs().max_element())
            .fold(1.0, f64::max);
        let tolerance = scale * f64::EPSILON * 128.0;
        let distance = closest.distance_squared.sqrt();
        if !distance.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if closest.simplex.count == 4 || distance <= tolerance {
            return distance_result(closest, true, 0.0, tolerance);
        }
        let search_direction = (-closest.point)
            .try_normalize()
            .ok_or(CollisionError::NonConvergent)?;
        let support = support_pair(shape_a, transform_a, shape_b, transform_b, search_direction)?;
        let support_projection = support
            .difference
            .checked_dot(search_direction)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let closest_projection = closest
            .point
            .checked_dot(search_direction)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        lower_bound = (-support_projection).clamp(0.0, distance);
        if support_projection - closest_projection <= tolerance {
            return distance_result(closest, false, lower_bound, tolerance);
        }
        if closest
            .simplex
            .points
            .iter()
            .take(closest.simplex.count)
            .any(|point| (point.difference - support.difference).length() <= tolerance)
        {
            if lower_bound > tolerance {
                return distance_result(closest, false, lower_bound, tolerance);
            }
            if distance <= scale * f64::EPSILON.sqrt() * 2.0 {
                return distance_result(closest, true, 0.0, tolerance);
            }
            return Err(CollisionError::NonConvergent);
        }
        simplex = closest.simplex;
        simplex.push(support)?;
        closest = closest_simplex(&simplex)?;
    }
    let scale = closest
        .simplex
        .points
        .iter()
        .take(closest.simplex.count)
        .map(|point| point.difference.abs().max_element())
        .fold(1.0, f64::max);
    let tolerance = scale * f64::EPSILON * 128.0;
    if lower_bound > tolerance {
        return distance_result(closest, false, lower_bound, tolerance);
    }
    if closest.distance_squared.sqrt() <= scale * f64::EPSILON.sqrt() * 2.0 {
        return distance_result(closest, true, 0.0, tolerance);
    }
    Err(CollisionError::NonConvergent)
}

fn distance_result(
    closest: ClosestSimplex,
    intersects: bool,
    lower_bound: f64,
    tolerance: f64,
) -> Result<GjkDistance, CollisionError> {
    let distance = if intersects {
        0.0
    } else {
        closest.distance_squared.sqrt()
    };
    if !distance.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if !lower_bound.is_finite() || lower_bound < 0.0 || lower_bound > distance + tolerance {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let center_direction = closest
        .point_b
        .checked_sub(closest.point_a)
        .and_then(Vec3::try_normalize)
        .unwrap_or(Vec3::X);
    let normal = if distance > tolerance {
        (closest.point_b - closest.point_a)
            .try_normalize()
            .unwrap_or(center_direction)
    } else {
        center_direction
    };
    Ok(GjkDistance {
        intersects,
        distance,
        lower_bound: lower_bound.min(distance),
        normal,
        point_a: closest.point_a,
        point_b: closest.point_b,
    })
}

fn closest_simplex(simplex: &Simplex) -> Result<ClosestSimplex, CollisionError> {
    if simplex.count == 0 || simplex.count > 4 {
        return Err(CollisionError::NonConvergent);
    }
    let mut best = None;
    for index in 0..simplex.count {
        let mut weights = [0.0; 4];
        weights[index] = 1.0;
        consider_weights(simplex, weights, &mut best)?;
    }
    for left in 0..simplex.count {
        for right in (left + 1)..simplex.count {
            let a = simplex.points[left].difference;
            let edge = simplex.points[right]
                .difference
                .checked_sub(a)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let denominator = edge
                .checked_dot(edge)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            if denominator <= f64::MIN_POSITIVE {
                continue;
            }
            let amount = (-a
                .checked_dot(edge)
                .ok_or(CollisionError::ArithmeticOverflow)?
                / denominator)
                .clamp(0.0, 1.0);
            let mut weights = [0.0; 4];
            weights[left] = 1.0 - amount;
            weights[right] = amount;
            consider_weights(simplex, weights, &mut best)?;
        }
    }
    for a in 0..simplex.count {
        for b in (a + 1)..simplex.count {
            for c in (b + 1)..simplex.count {
                consider_triangle(simplex, [a, b, c], &mut best)?;
            }
        }
    }
    if simplex.count == 4 {
        consider_tetrahedron(simplex, &mut best)?;
    }
    best.ok_or(CollisionError::NonConvergent)
}

fn consider_triangle(
    simplex: &Simplex,
    indices: [usize; 3],
    best: &mut Option<ClosestSimplex>,
) -> Result<(), CollisionError> {
    let [ia, ib, ic] = indices;
    let a = simplex.points[ia].difference;
    let b = simplex.points[ib].difference;
    let c = simplex.points[ic].difference;
    let ab = b.checked_sub(a).ok_or(CollisionError::ArithmeticOverflow)?;
    let ac = c.checked_sub(a).ok_or(CollisionError::ArithmeticOverflow)?;
    let normal = ab
        .checked_cross(ac)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let normal_squared = normal
        .checked_dot(normal)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if normal_squared <= f64::MIN_POSITIVE {
        return Ok(());
    }
    let projection = normal
        .checked_mul_scalar(
            -a.checked_dot(normal)
                .ok_or(CollisionError::ArithmeticOverflow)?
                / normal_squared,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let v0 = ab;
    let v1 = ac;
    let v2 = projection
        .checked_sub(a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d00 = v0
        .checked_dot(v0)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d01 = v0
        .checked_dot(v1)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d11 = v1
        .checked_dot(v1)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d20 = v2
        .checked_dot(v0)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let d21 = v2
        .checked_dot(v1)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let denominator = d00 * d11 - d01 * d01;
    if !denominator.is_finite() || denominator <= f64::MIN_POSITIVE {
        return Ok(());
    }
    let wb = (d11 * d20 - d01 * d21) / denominator;
    let wc = (d00 * d21 - d01 * d20) / denominator;
    let wa = 1.0 - wb - wc;
    let tolerance = f64::EPSILON * 128.0;
    if [wa, wb, wc]
        .iter()
        .any(|weight| !weight.is_finite() || *weight < -tolerance)
    {
        return Ok(());
    }
    let mut weights = [0.0; 4];
    weights[ia] = wa.max(0.0);
    weights[ib] = wb.max(0.0);
    weights[ic] = wc.max(0.0);
    normalize_weights(&mut weights)?;
    consider_weights(simplex, weights, best)
}

fn consider_tetrahedron(
    simplex: &Simplex,
    best: &mut Option<ClosestSimplex>,
) -> Result<(), CollisionError> {
    let a = simplex.points[0].difference;
    let ab = simplex.points[1]
        .difference
        .checked_sub(a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let ac = simplex.points[2]
        .difference
        .checked_sub(a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let ad = simplex.points[3]
        .difference
        .checked_sub(a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let determinant = ab
        .checked_dot(
            ac.checked_cross(ad)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if !determinant.is_finite() || determinant.abs() <= f64::MIN_POSITIVE {
        return Ok(());
    }
    let origin = -a;
    let wb = origin
        .checked_dot(
            ac.checked_cross(ad)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?
        / determinant;
    let wc = ab
        .checked_dot(
            origin
                .checked_cross(ad)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?
        / determinant;
    let wd = ab
        .checked_dot(
            ac.checked_cross(origin)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?
        / determinant;
    let wa = 1.0 - wb - wc - wd;
    let tolerance = f64::EPSILON * 128.0;
    let mut weights = [wa, wb, wc, wd];
    if weights
        .iter()
        .any(|weight| !weight.is_finite() || *weight < -tolerance)
    {
        return Ok(());
    }
    for weight in &mut weights {
        *weight = weight.max(0.0);
    }
    normalize_weights(&mut weights)?;
    consider_weights(simplex, weights, best)
}

fn normalize_weights(weights: &mut [f64; 4]) -> Result<(), CollisionError> {
    let sum: f64 = weights.iter().sum();
    if !sum.is_finite() || sum <= f64::MIN_POSITIVE {
        return Err(CollisionError::NonConvergent);
    }
    for weight in weights {
        *weight /= sum;
    }
    Ok(())
}

fn consider_weights(
    simplex: &Simplex,
    weights: [f64; 4],
    best: &mut Option<ClosestSimplex>,
) -> Result<(), CollisionError> {
    let mut point = Vec3::ZERO;
    let mut point_a = Vec3::ZERO;
    let mut point_b = Vec3::ZERO;
    let mut reduced = Simplex {
        points: [SupportPoint::default(); 4],
        count: 0,
    };
    for (index, weight) in weights.iter().copied().enumerate().take(simplex.count) {
        if !weight.is_finite() || weight < 0.0 {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if weight == 0.0 {
            continue;
        }
        let support = simplex.points[index];
        point = point
            .checked_add(
                support
                    .difference
                    .checked_mul_scalar(weight)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
            )
            .ok_or(CollisionError::ArithmeticOverflow)?;
        point_a = point_a
            .checked_add(
                support
                    .point_a
                    .checked_mul_scalar(weight)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
            )
            .ok_or(CollisionError::ArithmeticOverflow)?;
        point_b = point_b
            .checked_add(
                support
                    .point_b
                    .checked_mul_scalar(weight)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
            )
            .ok_or(CollisionError::ArithmeticOverflow)?;
        reduced.push(support)?;
    }
    let distance_squared = point
        .checked_dot(point)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if !distance_squared.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if best.is_none_or(|candidate| distance_squared < candidate.distance_squared) {
        *best = Some(ClosestSimplex {
            simplex: reduced,
            point,
            point_a,
            point_b,
            distance_squared,
        });
    }
    Ok(())
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
