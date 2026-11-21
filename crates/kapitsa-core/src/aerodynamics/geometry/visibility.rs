//! Shadow half-spaces evaluated in the source triangle's affine domain.
//! Adaptive orientation predicates come from GeoRust's MIT/Apache-2.0 `robust`.
use super::{GeometryError, SurfaceTriangle, Vec3, components};
use robust::{Coord, Coord3D, orient2d, orient3d};

mod exact_orientation;

pub(super) struct Shadow {
    pub planes: [[f64; 3]; 4],
    coplanar: bool,
}

// After a power-of-two scale per axis, this span keeps coordinates within
// [2^-127, 2^128). Including 53-bit significands, degree-three orientation
// expansions stay well inside the normal f64 range (roughly 2^-540..2^400).
// Unrepresentable cases are errors, never a distance epsilon or a missed hit.
const MAX_EXPONENT_SPAN: i32 = 254;

struct PredicateFrame {
    shifts: [i32; 3],
}
impl PredicateFrame {
    fn new(points: &[Vec3]) -> Result<Self, crate::Error> {
        let mut bounds: [Option<(i32, i32)>; 3] = [None; 3];
        for point in points {
            if !point.is_finite() {
                return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
            }
            for (bound, value) in bounds.iter_mut().zip([point.x, point.y, point.z]) {
                if value == 0.0 {
                    continue;
                }
                let bits = value.abs().to_bits();
                let biased = ((bits >> 52) & 0x7ff) as i32;
                let exponent = if biased == 0 {
                    (u64::BITS - 1 - bits.leading_zeros()) as i32 - 1074
                } else {
                    biased - 1023
                };
                *bound = Some(bound.map_or((exponent, exponent), |(low, high)| {
                    (low.min(exponent), high.max(exponent))
                }));
            }
        }
        let mut shifts = [0; 3];
        for (shift, bound) in shifts.iter_mut().zip(bounds) {
            if let Some((low, high)) = bound {
                if high - low > MAX_EXPONENT_SPAN {
                    return Err(crate::Error::from(GeometryError::PrecisionLoss));
                }
                *shift = -((low + high) / 2);
            }
        }
        // All orientations in this frame share the same positive determinant
        // factor, including those compared as affine distances.
        Ok(Self { shifts })
    }

    fn point(&self, point: Vec3) -> Result<Coord3D<f64>, crate::Error> {
        let point = Vec3::new(
            scale(point.x, self.shifts[0])?,
            scale(point.y, self.shifts[1])?,
            scale(point.z, self.shifts[2])?,
        );
        Ok(Coord3D {
            x: point.x,
            y: point.y,
            z: point.z,
        })
    }

    fn orientation(&self, a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Result<f64, crate::Error> {
        finite(orient3d(
            self.point(a)?,
            self.point(b)?,
            self.point(c)?,
            self.point(d)?,
        ))
    }

    fn fast_projection(
        &self,
        [start, end, point]: [Vec3; 3],
        edge: Vec3,
        offset: Vec3,
        direction: Vec3,
    ) -> Result<Option<f64>, crate::Error> {
        let e = self.point(edge)?;
        let p = self.point(offset)?;
        let d = self.point(direction)?;
        let zero = Coord3D {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let value = finite(orient3d(zero, e, d, p))?;
        let permanent = (e.x.abs() * p.y.abs() + e.y.abs() * p.x.abs()) * d.z.abs()
            + (e.y.abs() * p.z.abs() + e.z.abs() * p.y.abs()) * d.x.abs()
            + (e.z.abs() * p.x.abs() + e.x.abs() * p.z.abs()) * d.y.abs();
        // A normal subtraction changes each product by at most a few ulps;
        // differences in the subnormal range are exact. Use a conservative
        // margin for both product perturbations and determinant evaluation.
        let uncertain = value.abs() <= 32.0 * f64::EPSILON * permanent
            && projected_products_use_rounded_differences(start, end, point, direction);
        Ok((!uncertain).then_some(value))
    }
}

fn scale(mut value: f64, mut shift: i32) -> Result<f64, crate::Error> {
    while shift != 0 {
        let step = shift.clamp(-512, 512);
        let scaled = value * 2.0_f64.powi(step);
        if !scaled.is_finite() {
            return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
        }
        if value != 0.0 && scaled == 0.0 {
            return Err(crate::Error::from(GeometryError::PrecisionLoss));
        }
        value = scaled;
        shift -= step;
    }
    Ok(value)
}

// A rounded difference can erase a nonzero operand completely. In that
// case the original endpoint is needed to distinguish a very thin projection.
fn retains_endpoints(a: Vec3, b: Vec3, difference: Vec3) -> bool {
    difference.is_finite()
        && components(a)
            .into_iter()
            .zip(components(b))
            .zip(components(difference))
            .all(|((a, b), d)| (b == 0.0 || d != a) && (a == 0.0 || d != -b))
}

fn projected_products_use_rounded_differences(a: Vec3, b: Vec3, c: Vec3, direction: Vec3) -> bool {
    let [a, b, c, direction] = [a, b, c, direction].map(components);
    let exact_difference = |a: f64, b: f64| {
        let difference = a - b;
        let b_virtual = a - difference;
        let a_virtual = difference + b_virtual;
        (a - a_virtual) + (b_virtual - b) == 0.0
    };
    direction.into_iter().enumerate().any(|(k, direction)| {
        if direction == 0.0 {
            return false;
        }
        let i = (k + 1) % 3;
        let j = (k + 2) % 3;
        [(i, j), (j, i)].into_iter().any(|(i, j)| {
            b[i] != a[i]
                && c[j] != a[j]
                && (!exact_difference(b[i], a[i]) || !exact_difference(c[j], a[j]))
        })
    })
}

fn frame_or_exact(points: &[Vec3]) -> Result<Option<PredicateFrame>, crate::Error> {
    match PredicateFrame::new(points) {
        Ok(frame) => Ok(Some(frame)),
        Err(error)
            if error.downcast_ref::<GeometryError>() == Some(&GeometryError::PrecisionLoss) =>
        {
            Ok(None)
        }
        Err(error) => Err(error),
    }
}

fn finite(value: f64) -> Result<f64, crate::Error> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(crate::Error::from(GeometryError::ArithmeticOverflow))
    }
}

// Called only for strictly opposite signs. Check the smaller edge ratio so
// underflow detection is independent of endpoint order. A ratio can still
// round to one; rejecting that requires separate geometric analysis.
pub(super) fn crossing_fraction(a: f64, b: f64) -> Result<f64, crate::Error> {
    // Work with the smaller ratio to avoid overflowing the sum and to
    // preserve representable fractions when the distance scales differ.
    let a = a.abs();
    let b = b.abs();
    let (small, large) = if a <= b { (a, b) } else { (b, a) };
    let ratio = finite(small / large)?;
    let denominator = 1.0 + ratio;
    let denominator_error = ratio - (denominator - 1.0);
    let fraction = ratio / denominator;
    // Correct the rounded ratio and the rounded sum without forming an
    // underflowing product. All quantities in this correction are bounded.
    // Exact binary scaling keeps subnormal operands and their fused residual
    // representable during the ratio correction.
    const NORMALIZATION_SHIFT: i32 = 1022;
    let exponent = ((large.to_bits() >> 52) & 0x7ff) as i32;
    let shift = if exponent == 0 {
        52
    } else {
        NORMALIZATION_SHIFT - exponent
    };
    let (scaled_small, scaled_large) = if shift > 0 {
        (scale(small, shift)?, scale(large, shift)?)
    } else if shift < 0 {
        // Scaling a tiny numerator down can erase information needed for FMA.
        match (scale(small, shift), scale(large, shift)) {
            (Ok(a), Ok(b)) if a.is_normal() => (a, b),
            _ => (small, large),
        }
    } else {
        (small, large)
    };
    let ratio_error = (-ratio).mul_add(scaled_large, scaled_small) / scaled_large;
    let residual = (-fraction).mul_add(denominator, ratio);
    let smaller_fraction =
        fraction + ((residual + ratio_error) - fraction * denominator_error) / denominator;
    if smaller_fraction == 0.0 {
        return Err(crate::Error::from(GeometryError::PrecisionLoss));
    }
    let fraction = if a <= b {
        smaller_fraction
    } else {
        let reciprocal = 1.0 / denominator;
        let reciprocal_residual = (-reciprocal).mul_add(denominator, 1.0);
        reciprocal
            + (reciprocal_residual - reciprocal * (denominator_error + ratio_error)) / denominator
    };
    Ok(fraction)
}

pub(super) fn crossing_point(a: Vec3, b: Vec3, da: f64, db: f64) -> Result<Vec3, crate::Error> {
    let (near, far, d_near, d_far) = if da.abs() < db.abs() || (da.abs() == db.abs() && da > 0.0) {
        (a, b, da, db)
    } else {
        (b, a, db, da)
    };
    let fraction = crossing_fraction(d_near, d_far)?;
    let interpolate = |near: f64, far: f64| {
        let difference = far - near;
        if difference.is_finite() {
            difference.mul_add(fraction, near)
        } else {
            near * (1.0 - fraction) + far * fraction
        }
    };
    let point = Vec3::new(
        interpolate(near.x, far.x),
        interpolate(near.y, far.y),
        interpolate(near.z, far.z),
    );
    if !point.is_finite() {
        return Err(crate::Error::from(GeometryError::ArithmeticOverflow));
    }
    Ok(point)
}

fn normalize(values: [f64; 3]) -> Result<[f64; 3], crate::Error> {
    let maximum = values.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    if maximum == 0.0 {
        return Ok(values);
    }
    // Divide by a binary power to keep the stored half-space unchanged.
    // The minimum normal power also scales a subnormal maximum safely.
    let exponent_bits = maximum.to_bits() & (0x7ff_u64 << 52);
    let power = f64::from_bits(exponent_bits.max(1 << 52));
    let scaled = values.map(|v| v / power);
    if values.iter().zip(&scaled).any(|(a, b)| *a != *b * power) {
        return Err(crate::Error::from(GeometryError::PrecisionLoss));
    }
    Ok(scaled)
}

pub(super) fn distance(
    values: [f64; 3],
    point: Vec3,
    mut charge: impl FnMut(usize) -> Result<(), crate::Error>,
) -> Result<f64, crate::Error> {
    if !values.iter().all(|value| value.is_finite()) || !point.is_finite() {
        return Err(GeometryError::ArithmeticOverflow.into());
    }
    let x = values[1] - values[0];
    let y = values[2] - values[0];
    let x_virtual = x - values[1];
    let y_virtual = y - values[2];
    let x_error = (values[1] - (x - x_virtual)) - (values[0] + x_virtual);
    let y_error = (values[2] - (y - y_virtual)) - (values[0] + y_virtual);
    let xp = x * point.x;
    let yp = y * point.y;
    let xc = x_error * point.x;
    let yc = y_error * point.y;
    let xr = x.mul_add(point.x, -xp);
    let yr = y.mul_add(point.y, -yp);
    let sum = values[0] + xp;
    let v = sum - values[0];
    let x_sum_error = (values[0] - (sum - v)) + (xp - v);
    let result = sum + yp;
    let v = result - sum;
    let y_sum_error = (sum - (result - v)) + (yp - v);
    let corrections = [x_sum_error, y_sum_error, xr, yr, xc, yc];
    let value = result + corrections.iter().sum::<f64>();
    let correction_size: f64 = corrections.iter().map(|value| value.abs()).sum();

    // Below this product magnitude a binary64 product's exact low bits may
    // extend below the subnormal range, so its FMA residual may round.
    const MIN_PRODUCT: f64 = f64::MIN_POSITIVE / (f64::EPSILON * f64::EPSILON);
    let small_product = [
        (x, point.x, xp),
        (y, point.y, yp),
        (x_error, point.x, xc),
        (y_error, point.y, yc),
    ]
    .into_iter()
    .any(|(a, b, product)| a != 0.0 && b != 0.0 && product.abs() < MIN_PRODUCT);
    if !small_product
        && value.is_finite()
        && (correction_size == 0.0 || value.abs() > 16.0 * f64::EPSILON * correction_size)
    {
        return Ok(value);
    }
    charge(exact_orientation::AFFINE_WORK)?;
    exact_orientation::affine(values, point)
}

impl Shadow {
    pub fn new(
        source: &SurfaceTriangle,
        blocker: &SurfaceTriangle,
        source_index: usize,
        blocker_index: usize,
        upstream: Vec3,
        mut charge: impl FnMut(usize) -> Result<(), crate::Error>,
    ) -> Result<Option<Self>, crate::Error> {
        if source.vertices == blocker.vertices {
            return Ok(
                (blocker_index < source_index && blocker.normal.dot(upstream) > 0.0).then_some(
                    Self {
                        planes: [[1.0; 3], [1.0; 3], [1.0; 3], [0.0; 3]],
                        coplanar: true,
                    },
                ),
            );
        }
        if !upstream.is_finite()
            || blocker
                .vertices
                .iter()
                .chain(&source.vertices)
                .any(|point| !point.is_finite())
        {
            return Err(GeometryError::ArithmeticOverflow.into());
        }
        let [a, b, c] = blocker.vertices;
        let edge = b - a;
        let other = c - a;
        let size = edge.abs().max_element().max(other.abs().max_element());
        if size == 0.0 {
            return Err(crate::Error::from(GeometryError::InvalidGeometry));
        }
        // Direction may be positively rescaled without changing a shadow plane.
        // An exact power of two preserves the direction's component ratios.
        let size_exponent = size.to_bits() & (0x7ff_u64 << 52);
        let direction = upstream * f64::from_bits(size_exponent);
        let scaled_direction = size.is_finite()
            && direction.is_finite()
            && components(upstream)
                .into_iter()
                .zip(components(direction))
                .all(|(a, b)| a == 0.0 || b.is_normal());
        let direction_frame = if scaled_direction
            && retains_endpoints(b, a, edge)
            && retains_endpoints(c, a, other)
        {
            frame_or_exact(&[Vec3::ZERO, edge, other, direction])?
        } else {
            None
        };
        let denominator = if let Some(frame) = direction_frame {
            frame.fast_projection([a, b, c], edge, other, direction)?
        } else {
            None
        };
        let denominator = if let Some(value) = denominator {
            value
        } else {
            charge(exact_orientation::PROJECTED_WORK)?;
            exact_orientation::projected_sign(a, b, c, upstream)?
        };
        if denominator == 0.0 {
            return Ok(None);
        }
        let mut depth = [0.0; 3];
        // A coordinate-aligned plane has only one depth coordinate. The
        // common projected area factor cancels during normalization.
        let axial_depth = components(edge)
            .into_iter()
            .zip(components(other))
            .position(|(edge, other)| edge == 0.0 && other == 0.0);
        let axial_depth = axial_depth
            .map(|axis| {
                let plane = components(a)[axis];
                let sign = components(upstream)[axis].signum();
                source
                    .vertices
                    .map(|point| (plane - components(point)[axis]) * sign)
            })
            .filter(|depth| depth.iter().all(|value| value.is_finite()));
        if let Some(separations) = axial_depth {
            depth = separations;
        } else {
            if let Some(frame) = frame_or_exact(&[
                a,
                b,
                c,
                source.vertices[0],
                source.vertices[1],
                source.vertices[2],
            ])? {
                for (value, point) in depth.iter_mut().zip(source.vertices) {
                    // orient(a,b,c,p) is minus the signed plane distance numerator.
                    *value = frame.orientation(a, b, c, point)?;
                }
            } else {
                charge(3 * exact_orientation::WORK)?;
                depth = exact_orientation::orientations(blocker.vertices, source.vertices)?;
            }
            depth = depth.map(|value| value * denominator.signum());
        }
        let coplanar = depth.iter().all(|v| *v == 0.0);
        if coplanar {
            if blocker_index >= source_index || blocker.normal.dot(upstream) <= 0.0 {
                return Ok(None);
            }
        } else if !depth.iter().any(|v| *v > 0.0) {
            return Ok(None);
        }
        let mut planes = [[0.0; 3]; 4];
        planes[3] = normalize(depth)?;
        let axial_projection = match (upstream.x == 0.0, upstream.y == 0.0, upstream.z == 0.0) {
            (false, true, true) => Some(Vec3::new(0.0, 1.0, 1.0)),
            (true, false, true) => Some(Vec3::new(1.0, 0.0, 1.0)),
            (true, true, false) => Some(Vec3::new(1.0, 1.0, 0.0)),
            _ => None,
        };
        // The edge determinant already contains the axial direction.
        // Dropping that component from its other vectors leaves it unchanged.
        let (blocker_vertices, source_vertices) = if let Some(mask) = axial_projection {
            (
                blocker.vertices.map(|p| p * mask),
                source.vertices.map(|p| p * mask),
            )
        } else {
            (blocker.vertices, source.vertices)
        };
        let [a, b, c] = blocker_vertices;
        for (plane, (start, end, opposite)) in
            planes[..3]
                .iter_mut()
                .zip([(a, b, c), (b, c, a), (c, a, b)])
        {
            let edge = end - start;
            let offsets = source_vertices.map(|p| p - start);
            let opposite_offset = opposite - start;
            let frame = if scaled_direction
                && retains_endpoints(end, start, edge)
                && retains_endpoints(opposite, start, opposite_offset)
                && source_vertices
                    .into_iter()
                    .zip(offsets)
                    .all(|(point, offset)| retains_endpoints(point, start, offset))
            {
                frame_or_exact(&[
                    Vec3::ZERO,
                    edge,
                    direction,
                    opposite_offset,
                    offsets[0],
                    offsets[1],
                    offsets[2],
                ])?
            } else {
                None
            };
            let fast_reference = if let Some(frame) = &frame {
                frame.fast_projection([start, end, opposite], edge, opposite_offset, direction)?
            } else {
                None
            };
            let reference = if let Some(value) = fast_reference {
                value
            } else {
                charge(exact_orientation::PROJECTED_WORK)?;
                exact_orientation::projected_sign(start, end, opposite, upstream)?
            };
            if reference == 0.0 {
                return Err(crate::Error::from(GeometryError::PrecisionLoss));
            }
            let values = if let (Some(frame), Some(_)) = (&frame, fast_reference) {
                let mut values = [0.0; 3];
                let scale_exponent =
                    frame.shifts.iter().sum::<i32>() + (size_exponent >> 52) as i32 - 1023;
                for ((value, point), offset) in values.iter_mut().zip(source_vertices).zip(offsets)
                {
                    let points = [start, end, point];
                    *value = if let Some(value) =
                        frame.fast_projection(points, edge, offset, direction)?
                    {
                        value
                    } else {
                        charge(exact_orientation::PROJECTED_WORK)?;
                        exact_orientation::projected_coefficient(points, upstream, scale_exponent)?
                    };
                }
                values
            } else {
                charge(3 * exact_orientation::PROJECTED_WORK)?;
                exact_orientation::projected_coefficients(start, end, source_vertices, upstream)?
            };
            *plane = values.map(|value| value * reference.signum());
            if !plane.iter().any(|v| *v > 0.0) {
                return Ok(None);
            }
            *plane = normalize(*plane)?;
        }
        Ok(Some(Self { planes, coplanar }))
    }

    pub fn covers(
        &self,
        point: Vec3,
        mut charge: impl FnMut(usize) -> Result<(), crate::Error>,
    ) -> Result<bool, crate::Error> {
        for plane in &self.planes[..3] {
            if distance(*plane, point, &mut charge)? < 0.0 {
                return Ok(false);
            }
        }
        Ok(self.coplanar || distance(self.planes[3], point, charge)? > 0.0)
    }
}

/// Restrict an affine rigid-body flow to its genuinely windward region.
/// A stagnation point at the original centroid must not discard positive area.
pub(super) fn windward_triangle(
    normal: Vec3,
    velocities: [Vec3; 3],
) -> Result<Option<Vec<Vec3>>, crate::Error> {
    let mut values = [0.0; 3];
    for (value, velocity) in values.iter_mut().zip(velocities) {
        *value = normal
            .checked_dot(velocity)
            .ok_or_else(|| crate::Error::from(GeometryError::ArithmeticOverflow))?;
    }
    if !values.iter().any(|value| *value > 0.0) {
        return Ok(None);
    }
    let values = normalize(values)?;
    let vertices = [Vec3::ZERO, Vec3::X, Vec3::Y];
    let mut positive = Vec::new();
    positive
        .try_reserve_exact(4)
        .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
    for index in 0..3 {
        let next = (index + 1) % 3;
        let (a, b) = (values[index], values[next]);
        if a >= 0.0 {
            super::checked_push(&mut positive, vertices[index])?;
        }
        if (a > 0.0 && b < 0.0) || (a < 0.0 && b > 0.0) {
            let point = crossing_point(vertices[index], vertices[next], a, b)?;
            super::checked_push(&mut positive, point)?;
        }
    }
    if positive.len() < 3 {
        return Err(crate::Error::from(GeometryError::PrecisionLoss));
    }
    Ok(Some(positive))
}

/// Compensates accumulation roundoff only, not errors in clipped vertices.
#[derive(Default)]
pub(super) struct AreaSum {
    sum: f64,
    correction: f64,
}

impl AreaSum {
    pub fn add(&mut self, value: f64) {
        let adjusted = value - self.correction;
        let next = self.sum + adjusted;
        self.correction = (next - self.sum) - adjusted;
        self.sum = next;
    }

    pub fn value(&self) -> f64 {
        self.sum
    }
}

pub(super) fn polygon_fraction(
    points: &[Vec3],
    mut charge: impl FnMut(usize) -> Result<(), crate::Error>,
) -> Result<f64, crate::Error> {
    let mut area = AreaSum::default();
    for index in 1..points.len().saturating_sub(1) {
        area.add(triangle_fraction(
            [points[0], points[index], points[index + 1]],
            &mut charge,
        )?);
    }
    finite(area.value())
}

pub(super) fn triangle_fraction(
    points: [Vec3; 3],
    mut charge: impl FnMut(usize) -> Result<(), crate::Error>,
) -> Result<f64, crate::Error> {
    if points[0] == points[1] || points[1] == points[2] || points[2] == points[0] {
        return Ok(0.0);
    }
    let Some(frame) = frame_or_exact(&points)? else {
        charge(exact_orientation::WORK)?;
        return exact_orientation::area_fraction(points);
    };
    let [a, b, c] = [
        frame.point(points[0])?,
        frame.point(points[1])?,
        frame.point(points[2])?,
    ];
    let value = finite(orient2d(
        Coord { x: a.x, y: a.y },
        Coord { x: b.x, y: b.y },
        Coord { x: c.x, y: c.y },
    ))?;
    scale(value.abs(), -frame.shifts[0] - frame.shifts[1])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::geometry::GeometryOwner;

    fn distance(values: [f64; 3], point: Vec3) -> Result<f64, crate::Error> {
        super::distance(values, point, |_| Ok(()))
    }

    impl Shadow {
        fn covers_without_budget(&self, point: Vec3) -> Result<bool, crate::Error> {
            self.covers(point, |_| Ok(()))
        }
    }

    fn triangle_fraction(points: [Vec3; 3]) -> Result<f64, crate::Error> {
        super::triangle_fraction(points, |_| Ok(()))
    }

    fn polygon_fraction(points: &[Vec3]) -> Result<f64, crate::Error> {
        super::polygon_fraction(points, |_| Ok(()))
    }

    #[test]
    fn wide_triangle_area_preserves_absolute_scale_and_precharges_work() -> Result<(), crate::Error>
    {
        let tiny = f64::from_bits(1);
        let points = [Vec3::ZERO, Vec3::X * tiny, Vec3::X + Vec3::Y];
        let mut work = 0;
        let area = super::triangle_fraction(points, |amount| {
            work += amount;
            Ok(())
        })?;
        assert_eq!(area, tiny);
        assert_eq!(work, exact_orientation::WORK);
        assert_eq!(triangle_fraction([points[0], points[2], points[1]])?, tiny);
        assert_eq!(
            triangle_fraction([points[0], points[1], Vec3::X + Vec3::Y * 0.5]),
            Err(GeometryError::PrecisionLoss.into()),
        );
        assert_eq!(
            super::triangle_fraction(points, |_| Err(GeometryError::WorkBudgetExceeded.into())),
            Err(GeometryError::WorkBudgetExceeded.into()),
        );
        Ok(())
    }

    #[test]
    fn crossing_uses_near_endpoint_independent_of_edge_direction() {
        let small = f64::EPSILON / 4.0;
        let a = Vec3::ZERO;
        let b = Vec3::X;
        let forward = crossing_point(a, b, -1.0, small).unwrap();
        let reverse = crossing_point(b, a, small, -1.0).unwrap();
        assert_eq!(forward, reverse);
        assert_eq!(forward, b + (a - b) * small);
        assert_eq!(
            crossing_point(a, b, 1.0, -1.0),
            crossing_point(b, a, -1.0, 1.0)
        );
    }

    #[test]
    fn equal_area_sum_does_not_prove_partition_coverage() {
        // The left half is counted twice and the right half is missing.
        // All coordinates and fractions are exactly representable.
        let half = [Vec3::ZERO, Vec3::Y, Vec3::X * 0.5];
        let mut sum = AreaSum::default();
        sum.add(triangle_fraction(half).unwrap());
        sum.add(triangle_fraction(half).unwrap());
        assert_eq!(
            sum.value(),
            triangle_fraction([Vec3::ZERO, Vec3::X, Vec3::Y]).unwrap()
        );
    }

    #[test]
    fn crossing_keeps_representable_intersection_with_overflowing_differences()
    -> Result<(), crate::Error> {
        let largest = f64::MAX;
        assert_eq!(crossing_fraction(largest, -largest)?, 0.5);
        let a = Vec3::new(-largest, 2.0, -2.0);
        let b = Vec3::new(largest, 6.0, 2.0);
        assert_eq!(
            crossing_point(a, b, -largest, largest)?,
            Vec3::new(0.0, 4.0, 0.0)
        );
        assert_eq!(
            crossing_point(b, a, largest, -largest)?,
            Vec3::new(0.0, 4.0, 0.0)
        );
        Ok(())
    }

    #[test]
    fn crossing_interpolation_retains_subnormal_product_residual() -> Result<(), crate::Error> {
        let smallest = f64::from_bits(1);
        let near = Vec3::X * smallest;
        let far = -Vec3::X * smallest;
        let point = crossing_point(near, far, 1.0, -3.0)?;
        assert_eq!(point, Vec3::ZERO);
        assert_eq!(point, crossing_point(far, near, -3.0, 1.0)?);
        Ok(())
    }

    #[test]
    fn crossing_fraction_handles_subnormal_denominator_rounding() -> Result<(), crate::Error> {
        let m = f64::from_bits(1);
        assert_eq!(crossing_fraction(2.0 * m, -m)?, 2.0 / 3.0);
        assert_eq!(crossing_fraction(m, -2.0 * m)?, 1.0 / 3.0);
        Ok(())
    }

    #[test]
    fn crossing_fraction_corrects_ratio_at_rounding_boundary() -> Result<(), crate::Error> {
        let small = f64::from_bits(1);
        let large = f64::MIN_POSITIVE * 2.0;
        assert_eq!(
            crossing_fraction(small, -large)?,
            2.0_f64.powi(-53).next_down()
        );
        Ok(())
    }

    #[test]
    fn crossing_fraction_corrects_large_endpoint_reciprocal() -> Result<(), crate::Error> {
        // Expected value is the nearest binary64 to the exact rational a/(a+b).
        let a = f64::from_bits(0x4b18_5f64_cee3_79c6);
        let b = f64::from_bits(0x4ae1_f9d1_568c_8566);
        let expected = f64::from_bits(0x3fed_4c81_d7e1_b240);
        assert_eq!(crossing_fraction(a, -b)?, expected);
        assert_eq!(crossing_fraction(-a, b)?, expected);
        Ok(())
    }

    #[test]
    fn crossing_fraction_corrects_two_subnormal_distances() -> Result<(), crate::Error> {
        // Expected from the exact rational formed by the two binary64 inputs.
        let small = f64::from_bits(0x5d);
        let large = f64::from_bits(0x3c_f738);
        let expected = f64::from_bits(0x3ef8_6816_e38d_5625);
        assert_eq!(crossing_fraction(small, -large)?, expected);
        assert_eq!(crossing_fraction(-small, large)?, expected);
        Ok(())
    }

    #[test]
    fn crossing_fraction_corrects_normal_ratio_near_underflow() -> Result<(), crate::Error> {
        // Expected from exact rational arithmetic on the binary64 inputs.
        let small = f64::from_bits(0x2a38_b9be_9e64_a7b4);
        let large = f64::from_bits(0x6a04_e05d_a5d8_626a);
        let expected = f64::from_bits(0x0022_f335_6d15_4415);
        assert_eq!(crossing_fraction(small, -large)?, expected);
        Ok(())
    }

    #[test]
    fn crossing_fraction_rejects_underflow_but_accepts_representable_fraction() {
        let smallest = f64::from_bits(1);
        assert_eq!(crossing_fraction(smallest, -1.0), Ok(smallest));
        assert_eq!(
            crossing_fraction(smallest, -2.0),
            Err(crate::Error::from(GeometryError::PrecisionLoss))
        );
        assert_eq!(
            crossing_fraction(-2.0, smallest),
            Err(crate::Error::from(GeometryError::PrecisionLoss))
        );
        assert_eq!(crossing_fraction(-1.0, smallest), Ok(1.0));
        assert_eq!(crossing_fraction(1.0, -1.0), Ok(0.5));
    }

    #[test]
    fn dyadic_strip_partition_preserves_exact_unit_fraction() {
        // These triangles partition a triangle of unit area fraction,
        // with exactly representable binary coordinates.
        const MAX_POWER: u32 = 12;
        for power in 0..=MAX_POWER {
            let count = 1usize << power;
            let mut total = AreaSum::default();
            for i in 0..count {
                let y0 = i as f64 / count as f64;
                let y1 = (i + 1) as f64 / count as f64;
                let fraction = triangle_fraction([
                    Vec3::ZERO,
                    Vec3::new(1.0, y0, 0.0),
                    Vec3::new(1.0, y1, 0.0),
                ])
                .unwrap();
                assert_eq!(fraction, 1.0 / count as f64);
                total.add(fraction);
            }
            assert_eq!(total.value(), 1.0, "partition size {count}");
        }
    }

    #[test]
    fn affine_distance_rejects_product_underflow() {
        let smallest = f64::from_bits(1);
        assert_eq!(
            distance([0.0, smallest, 0.0], Vec3::X * 0.5),
            Err(crate::Error::from(GeometryError::PrecisionLoss))
        );
        assert_eq!(
            distance([0.0, 0.0, smallest], Vec3::Y * 0.5),
            Err(crate::Error::from(GeometryError::PrecisionLoss))
        );
        assert_eq!(distance([0.0, smallest, 0.0], Vec3::X), Ok(smallest));
    }

    #[test]
    fn affine_distance_recovers_after_coefficient_correction_underflow() {
        let smallest = f64::from_bits(1);
        // 1 - smallest rounds to 1; its exact residual is nonzero but its
        // product with one half cannot be represented in binary64.
        assert_eq!(distance([smallest, 1.0, 0.0], Vec3::X * 0.5), Ok(0.5));
    }

    #[test]
    fn shadow_coverage_recovers_from_intermediate_overflow() {
        let shadow = Shadow {
            planes: [[f64::MAX, -f64::MAX, 0.0]; 4],
            coplanar: false,
        };
        assert_eq!(
            shadow.covers_without_budget(Vec3::X * 2.0),
            Err(crate::Error::from(GeometryError::ArithmeticOverflow))
        );
        assert_eq!(shadow.covers_without_budget(Vec3::X), Ok(false));
    }

    #[test]
    fn affine_distance_precharges_exact_recovery_of_subnormal_sum() -> Result<(), crate::Error> {
        let smallest = f64::from_bits(1);
        let values = [0.0, smallest, smallest];
        let point = (Vec3::X + Vec3::Y) * 0.5;
        let mut work = 0;
        assert_eq!(
            super::distance(values, point, |amount| {
                work += amount;
                Ok(())
            })?,
            smallest
        );
        assert_eq!(work, exact_orientation::AFFINE_WORK);
        assert_eq!(
            super::distance(values, point, |_| Err(
                GeometryError::WorkBudgetExceeded.into()
            )),
            Err(GeometryError::WorkBudgetExceeded.into())
        );
        Ok(())
    }

    #[test]
    fn affine_distance_preserves_small_positive_value_through_cancellation() {
        // Exact affine value is 2 * offset.
        const OFFSET: f64 = 1.0 / ((1u64 << 55) as f64);
        let point = Vec3::new(0.5, OFFSET, 0.0);
        assert_eq!(distance([-1.0, 1.0, 1.0], point), Ok(2.0 * OFFSET));
    }

    #[test]
    fn affine_distance_compensates_coefficient_difference() {
        // At vertex 1 the exact barycentric value is values[1], even
        // when subtracting values[0] rounds away that small value.
        const SMALL: f64 = 1.0 / ((1u64 << 54) as f64);
        assert_eq!(distance([1.0, SMALL, 0.0], Vec3::X), Ok(SMALL));
        assert_eq!(distance([1.0, 0.0, SMALL], Vec3::Y), Ok(SMALL));
    }

    #[test]
    fn compensated_area_sum_retains_small_positive_contributions() {
        const SMALL: f64 = f64::EPSILON / 4.0;
        let mut sum = AreaSum::default();
        sum.add(1.0);
        for _ in 0..4 {
            sum.add(SMALL);
        }
        assert_eq!(sum.value(), 1.0 + f64::EPSILON);
    }

    #[test]
    fn binary_normalization_preserves_exact_boundary() -> Result<(), crate::Error> {
        let values = normalize([3.0, -1.0, 0.0])?;
        assert_eq!(values, [1.5, -0.5, 0.0]);
        assert_eq!(distance(values, Vec3::X * 0.75)?, 0.0);
        Ok(())
    }

    #[test]
    fn windward_clipping_rejects_normalization_that_erases_positive_velocity() {
        let smallest = f64::from_bits(1);
        let velocities = [Vec3::X * smallest, Vec3::X * -f64::MAX, Vec3::ZERO];
        assert_eq!(
            windward_triangle(Vec3::X, velocities),
            Err(crate::Error::from(GeometryError::PrecisionLoss))
        );
    }

    #[test]
    fn thin_triangle_fraction_preserves_small_representable_area() {
        // This offset is exactly representable in binary64.
        const HEIGHT: f64 = 1.0 / ((1u64 << 40) as f64);
        let points = [
            Vec3::ZERO,
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(1.0, 1.0 + HEIGHT, 0.0),
        ];
        assert_eq!(triangle_fraction(points).unwrap(), HEIGHT);
        assert_eq!(
            triangle_fraction([points[0], points[2], points[1]]).unwrap(),
            HEIGHT
        );
    }

    #[test]
    fn adaptive_triangle_fraction_resolves_cancelling_products() {
        // Exact determinant: 1 - (1 + delta)(1 - delta) = delta².
        // Both products round to 1 in a naive binary64 calculation.
        const DELTA: f64 = 1.0 / ((1u64 << 27) as f64);
        const EXPECTED: f64 = DELTA * DELTA;
        let points = [
            Vec3::ZERO,
            Vec3::new(1.0, 1.0 + DELTA, 0.0),
            Vec3::new(1.0 - DELTA, 1.0, 0.0),
        ];
        let naive = points[1].x * points[2].y - points[1].y * points[2].x;
        assert_eq!(naive, 0.0);
        assert_eq!(triangle_fraction(points).unwrap(), EXPECTED);
        assert_eq!(
            triangle_fraction([points[0], points[2], points[1]]).unwrap(),
            EXPECTED
        );
    }

    #[test]
    fn windward_triangle_clips_affine_sign_change_to_analytic_area() {
        const EXPECTED_FRACTION: f64 = 0.25;
        const TOLERANCE: f64 = 1e-14;
        for positive in 0..3 {
            let mut velocities = [-Vec3::X; 3];
            velocities[positive] = Vec3::X;
            let polygon = windward_triangle(Vec3::X, velocities).unwrap().unwrap();
            assert_eq!(polygon.len(), 3);
            let fraction = polygon_fraction(&polygon).unwrap();
            assert!((fraction - EXPECTED_FRACTION).abs() < TOLERANCE);
        }
    }

    #[test]
    fn windward_triangle_never_exceeds_four_vertices_for_all_sign_patterns() {
        const VALUES: [f64; 3] = [-1.0, 0.0, 1.0];
        const MAX_VERTICES: usize = 4;
        for a in VALUES {
            for b in VALUES {
                for c in VALUES {
                    let velocities = [a, b, c].map(|value| Vec3::X * value);
                    match windward_triangle(Vec3::X, velocities) {
                        Ok(Some(polygon)) => {
                            assert!([a, b, c].iter().any(|value| *value > 0.0));
                            assert!((3..=MAX_VERTICES).contains(&polygon.len()));
                        }
                        Ok(None) => assert!([a, b, c].iter().all(|value| *value <= 0.0)),
                        Err(error) => panic!("signs {a}, {b}, {c}: {error:?}"),
                    }
                }
            }
        }
    }

    #[test]
    fn affine_shadow_matches_scalar_rays_away_from_boundaries() -> Result<(), crate::Error> {
        let source = SurfaceTriangle {
            vertices: [Vec3::ZERO, Vec3::Y, Vec3::Z],
            normal: Vec3::X,
            area: 0.5,
            owner: GeometryOwner::Body,
        };
        for size in [1e-50, 1.0, 1e50] {
            let source = SurfaceTriangle {
                vertices: source.vertices.map(|point| point * size),
                area: 0.5 * size * size,
                ..source
            };
            for relative_separation in [1.0, 1e-14, 1e-70] {
                let separation = relative_separation * size;
                let blocker = SurfaceTriangle {
                    vertices: [
                        Vec3::X * separation,
                        Vec3::X * separation + Vec3::Y * (0.5 * size),
                        Vec3::X * separation + Vec3::Z * size,
                    ],
                    normal: Vec3::X,
                    area: 0.25,
                    owner: GeometryOwner::Body,
                };
                for direction in [Vec3::X, Vec3::new(1.0, 0.2, 0.1)] {
                    let direction = direction
                        .try_normalize()
                        .ok_or_else(|| crate::Error::from(GeometryError::InvalidGeometry))?;
                    let shadow = Shadow::new(&source, &blocker, 0, 1, direction, |_| Ok(()))?;
                    for u in [0.13, 0.27, 0.51, 0.73] {
                        for v in [0.11, 0.29, 0.53] {
                            if u + v >= 0.95 {
                                continue;
                            }
                            let point = Vec3::new(u, v, 0.0);
                            let origin = (Vec3::Y * u + Vec3::Z * v) * size;
                            let expected = super::super::ray_hits_triangle(
                                origin,
                                direction,
                                blocker.vertices,
                            )?;
                            assert_eq!(
                                shadow
                                    .as_ref()
                                    .is_some_and(|s| s.covers_without_budget(point).unwrap()),
                                expected
                            );
                        }
                    }
                }
            }
        }
        Ok(())
    }

    #[test]
    fn triangle_fraction_preserves_area_across_axis_scales() -> Result<(), crate::Error> {
        let points = [Vec3::ZERO, Vec3::X * 2.0_f64.powi(-500), Vec3::Y];
        assert_eq!(triangle_fraction(points)?, 2.0_f64.powi(-500));
        Ok(())
    }

    #[test]
    fn shadow_covers_thin_source_across_axis_scales() -> Result<(), crate::Error> {
        let width = 2.0_f64.powi(-500);
        let source = SurfaceTriangle {
            vertices: [Vec3::ZERO, Vec3::Y * width, Vec3::Z],
            normal: Vec3::X,
            area: width * 0.5,
            owner: GeometryOwner::Body,
        };
        let blocker = SurfaceTriangle {
            vertices: source.vertices.map(|point| point + Vec3::X),
            ..source
        };
        let shadow = Shadow::new(&source, &blocker, 0, 1, Vec3::X, |_| Ok(()))?
            .ok_or(GeometryError::PrecisionLoss)?;
        assert!(shadow.covers_without_budget(Vec3::new(0.25, 0.25, 0.0))?);
        Ok(())
    }

    #[test]
    fn axial_shadow_preserves_small_representable_depth() -> Result<(), crate::Error> {
        for (axis, u, v) in [
            (Vec3::X, Vec3::Y, Vec3::Z),
            (Vec3::Y, Vec3::Z, Vec3::X),
            (Vec3::Z, Vec3::X, Vec3::Y),
        ] {
            for upstream in [axis, -axis] {
                let source = SurfaceTriangle {
                    vertices: [Vec3::ZERO, u, v],
                    normal: upstream,
                    area: 0.5,
                    owner: GeometryOwner::Body,
                };
                let blocker = SurfaceTriangle {
                    vertices: source.vertices.map(|p| p + upstream * 1e-100),
                    ..source
                };
                let shadow = Shadow::new(&source, &blocker, 0, 1, upstream, |_| Ok(()))?
                    .ok_or(GeometryError::PrecisionLoss)?;
                assert!(shadow.covers_without_budget(Vec3::new(0.25, 0.25, 0.0))?);
                assert!(Shadow::new(&source, &blocker, 0, 1, -upstream, |_| Ok(()))?.is_none());
            }
        }
        Ok(())
    }

    #[test]
    fn axial_plane_depth_accepts_wide_coordinate_span() -> Result<(), crate::Error> {
        let source = SurfaceTriangle {
            vertices: [Vec3::ZERO, Vec3::Y + Vec3::X * 1e-100, Vec3::Z],
            normal: Vec3::X,
            area: 0.5,
            owner: GeometryOwner::Body,
        };
        let blocker = SurfaceTriangle {
            vertices: [Vec3::X, Vec3::X + Vec3::Y, Vec3::X + Vec3::Z],
            ..source
        };
        let shadow = Shadow::new(&source, &blocker, 0, 1, Vec3::X, |_| Ok(()))?
            .ok_or(GeometryError::PrecisionLoss)?;
        assert!(shadow.covers_without_budget(Vec3::new(0.25, 0.25, 0.0))?);
        assert!(Shadow::new(&source, &blocker, 0, 1, -Vec3::X, |_| Ok(()))?.is_none());
        Ok(())
    }

    #[test]
    fn projection_filter_detects_nonzero_sign_reversal() -> Result<(), crate::Error> {
        let a = Vec3::new(0.339_047_440_731_973_7, 0.384_338_398_802_053_34, 0.0);
        let b = Vec3::new(0.373_762_739_740_275_1, 0.886_031_065_600_735_5, 0.0);
        let c = Vec3::new(0.460_610_943_905_456_1, 2.141_129_203_856_350_7, 0.0);
        let edge = b - a;
        let offset = c - a;
        let frame = PredicateFrame::new(&[Vec3::ZERO, edge, offset, Vec3::Z])?;
        let rounded = frame.orientation(Vec3::ZERO, edge, Vec3::Z, offset)?;
        let exact = exact_orientation::projected_sign(a, b, c, Vec3::Z)?;
        assert_eq!(rounded.signum(), -exact);
        assert_ne!(exact, 0.0);
        assert!(
            frame
                .fast_projection([a, b, c], edge, offset, Vec3::Z)?
                .is_none()
        );
        Ok(())
    }

    #[test]
    fn partially_rounded_projection_does_not_hide_thin_blocker() -> Result<(), crate::Error> {
        let x = 0.1_f64;
        let a = Vec3::new(x, x.next_up(), 0.0);
        let b = Vec3::X + Vec3::Y;
        let c = b * 2.0;
        assert!(retains_endpoints(b, a, b - a));
        assert!(retains_endpoints(c, a, c - a));
        assert_eq!((b - a).cross(c - a), Vec3::ZERO);
        let blocker = SurfaceTriangle {
            vertices: [a, b, c],
            normal: Vec3::Z,
            area: (x.next_up() - x) * 0.5,
            owner: GeometryOwner::Body,
        };
        let source = SurfaceTriangle {
            vertices: blocker.vertices.map(|point| point - Vec3::Z),
            ..blocker
        };
        let shadow = Shadow::new(&source, &blocker, 0, 1, Vec3::Z, |_| Ok(()))?
            .ok_or(GeometryError::PrecisionLoss)?;
        assert!(shadow.covers_without_budget(Vec3::new(0.25, 0.25, 0.0))?);
        Ok(())
    }

    #[test]
    fn rounded_edge_difference_does_not_make_blocker_tangent() -> Result<(), crate::Error> {
        let start = Vec3::X * 2.0_f64.powi(-500);
        let end = Vec3::X + Vec3::Y;
        let blocker = SurfaceTriangle {
            vertices: [start, end, end * 2.0],
            normal: -Vec3::Z,
            area: start.x * 0.5,
            owner: GeometryOwner::Body,
        };
        let [a, b, c] = blocker.vertices;
        assert_eq!((b - a).cross(c - a), Vec3::ZERO);
        let source = SurfaceTriangle {
            vertices: blocker.vertices.map(|point| point - Vec3::Z),
            ..blocker
        };
        let shadow = Shadow::new(&source, &blocker, 0, 1, Vec3::Z, |_| Ok(()))?
            .ok_or(GeometryError::PrecisionLoss)?;
        assert!(shadow.covers_without_budget(Vec3::new(0.25, 0.25, 0.0))?);
        Ok(())
    }

    #[test]
    fn axial_depth_accepts_overflowing_separation() -> Result<(), crate::Error> {
        let side = Vec3::X * f64::MAX;
        let blocker = SurfaceTriangle {
            vertices: [side, side + Vec3::Y, side + Vec3::Z],
            normal: Vec3::X,
            area: 0.5,
            owner: GeometryOwner::Body,
        };
        let source = SurfaceTriangle {
            vertices: [-side, -side + Vec3::Y, -side + Vec3::Z],
            ..blocker
        };
        let shadow = Shadow::new(&source, &blocker, 0, 1, Vec3::X, |_| Ok(()))?
            .ok_or(GeometryError::PrecisionLoss)?;
        assert!(shadow.covers_without_budget(Vec3::new(0.25, 0.25, 0.0))?);
        assert!(Shadow::new(&source, &blocker, 0, 1, -Vec3::X, |_| Ok(()))?.is_none());
        Ok(())
    }

    #[test]
    fn wide_direction_and_edges_recover_projected_shadow() -> Result<(), crate::Error> {
        let tiny = 2.0_f64.powi(-500);
        let blocker = SurfaceTriangle {
            vertices: [Vec3::ZERO, Vec3::Y, Vec3::Z],
            normal: Vec3::X,
            area: 0.5,
            owner: GeometryOwner::Body,
        };
        for (separation, upstream) in [(1.0, Vec3::X + Vec3::Y * tiny), (tiny, Vec3::X + Vec3::Y)] {
            let source = SurfaceTriangle {
                vertices: blocker.vertices.map(|point| point - Vec3::X * separation),
                ..blocker
            };
            let mut work = 0;
            let shadow = Shadow::new(&source, &blocker, 0, 1, upstream, |cost| {
                work += cost;
                Ok(())
            })?
            .ok_or(GeometryError::PrecisionLoss)?;
            assert!(work >= 4 * exact_orientation::PROJECTED_WORK);
            assert!(shadow.covers_without_budget(Vec3::new(0.25, 0.25, 0.0))?);
            assert!(!shadow.covers_without_budget(Vec3::new(0.75, 0.25, 0.0))?);
        }
        Ok(())
    }

    #[test]
    fn wide_inclined_depth_charges_before_recovering_shadow() -> Result<(), crate::Error> {
        let source = SurfaceTriangle {
            vertices: [Vec3::ZERO, Vec3::Y + Vec3::X * 1e-100, Vec3::Z],
            normal: Vec3::X,
            area: 0.5,
            owner: GeometryOwner::Body,
        };
        let blocker = SurfaceTriangle {
            vertices: [Vec3::X, Vec3::X * 2.0 + Vec3::Y, Vec3::X + Vec3::Z],
            ..source
        };
        let charge = |_| Err(GeometryError::WorkBudgetExceeded.into());
        assert!(matches!(
            Shadow::new(&source, &blocker, 0, 1, Vec3::X, charge),
            Err(error) if error.downcast_ref::<GeometryError>() == Some(&GeometryError::WorkBudgetExceeded)
        ));
        let mut work = 0;
        let shadow = Shadow::new(&source, &blocker, 0, 1, Vec3::X, |cost| {
            work += cost;
            Ok(())
        })?
        .ok_or(GeometryError::PrecisionLoss)?;
        assert_eq!(work, 3 * exact_orientation::WORK);
        assert!(shadow.covers_without_budget(Vec3::new(0.25, 0.25, 0.0))?);
        Ok(())
    }
}
