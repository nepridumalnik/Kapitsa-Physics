//! Shadow half-spaces evaluated in the source triangle's affine domain.
//! Adaptive orientation predicates come from GeoRust's MIT/Apache-2.0 `robust`.
use super::{GeometryError, SurfaceTriangle, Vec3, components};
use robust::{Coord, Coord3D, orient2d, orient3d};

mod exact_depth;

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
    let difference = a - b;
    let (a, b, denominator) = if difference.is_infinite() && a.is_finite() && b.is_finite() {
        // Halving both distances preserves their ratio when their difference
        // would exceed the finite range.
        (a * 0.5, b * 0.5, a * 0.5 - b * 0.5)
    } else {
        (a, b, finite(difference)?)
    };
    if (a.abs().min(b.abs()) / denominator.abs()) == 0.0 {
        return Err(crate::Error::from(GeometryError::PrecisionLoss));
    }
    let fraction = finite(a / denominator)?;
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
            near + difference * fraction
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
    let scaled = values.map(|v| v / maximum);
    if values
        .iter()
        .zip(&scaled)
        .any(|(a, b)| *a != 0.0 && *b == 0.0)
    {
        return Err(crate::Error::from(GeometryError::PrecisionLoss));
    }
    Ok(scaled)
}

pub(super) fn distance(values: [f64; 3], point: Vec3) -> Result<f64, crate::Error> {
    // Evaluate the barycentric form with compensated products and sums.
    // This reduces cancellation but does not establish an exact sign bound.
    let x = values[1] - values[0];
    let y = values[2] - values[0];
    let x_virtual = x - values[1];
    let y_virtual = y - values[2];
    let x_error = (values[1] - (x - x_virtual)) - (values[0] + x_virtual);
    let y_error = (values[2] - (y - y_virtual)) - (values[0] + y_virtual);
    let checked_product = |a: f64, b: f64| {
        let product = a * b;
        if a != 0.0 && b != 0.0 && product == 0.0 {
            Err(crate::Error::from(GeometryError::PrecisionLoss))
        } else {
            Ok(product)
        }
    };
    let xp = checked_product(x, point.x)?;
    let yp = checked_product(y, point.y)?;
    let xc = checked_product(x_error, point.x)?;
    let yc = checked_product(y_error, point.y)?;
    let xr = x.mul_add(point.x, -xp);
    let yr = y.mul_add(point.y, -yp);
    let sum = values[0] + xp;
    let v = sum - values[0];
    let residual = (values[0] - (sum - v)) + (xp - v);
    let result = sum + yp;
    let v = result - sum;
    let residual = residual + ((sum - (result - v)) + (yp - v));
    finite(result + (residual + xr + yr + xc + yc))
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
        let [a, b, c] = blocker.vertices;
        let edge = b - a;
        let other = c - a;
        let size = edge.abs().max_element().max(other.abs().max_element());
        if !size.is_finite() || size == 0.0 {
            return Err(crate::Error::from(GeometryError::InvalidGeometry));
        }
        // Direction may be positively rescaled without changing a shadow plane.
        let direction = upstream * size;
        if !direction.is_finite()
            || [upstream.x, upstream.y, upstream.z]
                .iter()
                .zip([direction.x, direction.y, direction.z])
                .any(|(a, b)| *a != 0.0 && b == 0.0)
        {
            return Err(crate::Error::from(GeometryError::PrecisionLoss));
        }
        let frame = PredicateFrame::new(&[Vec3::ZERO, edge, other, direction])?;
        let denominator = -frame.orientation(Vec3::ZERO, edge, other, direction)?;
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
        if let Some(axis) = axial_depth {
            let plane = components(a)[axis];
            let sign = components(upstream)[axis].signum();
            for (value, point) in depth.iter_mut().zip(source.vertices) {
                *value = finite(plane - components(point)[axis])? * sign;
            }
        } else {
            let frame = PredicateFrame::new(&[
                a,
                b,
                c,
                source.vertices[0],
                source.vertices[1],
                source.vertices[2],
            ]);
            match frame {
                Ok(frame) => {
                    for (value, point) in depth.iter_mut().zip(source.vertices) {
                        // orient(a,b,c,p) is minus the signed plane distance numerator.
                        *value = frame.orientation(a, b, c, point)?;
                    }
                }
                Err(error)
                    if error.downcast_ref::<GeometryError>()
                        == Some(&GeometryError::PrecisionLoss) =>
                {
                    charge(exact_depth::WORK)?;
                    depth = exact_depth::depths(blocker.vertices, source.vertices)?;
                }
                Err(error) => return Err(error),
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
            let opposite = opposite - start;
            let frame = PredicateFrame::new(&[
                Vec3::ZERO,
                edge,
                direction,
                opposite,
                offsets[0],
                offsets[1],
                offsets[2],
            ])?;
            let reference = frame.orientation(Vec3::ZERO, edge, direction, opposite)?;
            if reference == 0.0 {
                return Err(crate::Error::from(GeometryError::PrecisionLoss));
            }
            for (value, offset) in plane.iter_mut().zip(offsets) {
                *value =
                    frame.orientation(Vec3::ZERO, edge, direction, offset)? * reference.signum();
            }
            if !plane.iter().any(|v| *v > 0.0) {
                return Ok(None);
            }
            *plane = normalize(*plane)?;
        }
        Ok(Some(Self { planes, coplanar }))
    }

    pub fn covers(&self, point: Vec3) -> Result<bool, crate::Error> {
        for plane in &self.planes[..3] {
            if distance(*plane, point)? < 0.0 {
                return Ok(false);
            }
        }
        Ok(self.coplanar || distance(self.planes[3], point)? > 0.0)
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

pub(super) fn polygon_fraction(points: &[Vec3]) -> Result<f64, crate::Error> {
    let mut area = AreaSum::default();
    for index in 1..points.len().saturating_sub(1) {
        area.add(triangle_fraction([
            points[0],
            points[index],
            points[index + 1],
        ])?);
    }
    finite(area.value())
}

pub(super) fn triangle_fraction(points: [Vec3; 3]) -> Result<f64, crate::Error> {
    if points[0] == points[1] || points[1] == points[2] || points[2] == points[0] {
        return Ok(0.0);
    }
    let frame = PredicateFrame::new(&points)?;
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
    fn affine_distance_rejects_coefficient_correction_underflow() {
        let smallest = f64::from_bits(1);
        // 1 - smallest rounds to 1; its exact residual is nonzero but its
        // product with one half cannot be represented in binary64.
        assert_eq!(
            distance([smallest, 1.0, 0.0], Vec3::X * 0.5),
            Err(crate::Error::from(GeometryError::PrecisionLoss))
        );
    }

    #[test]
    fn shadow_coverage_rejects_nonfinite_distance() {
        let shadow = Shadow {
            planes: [[f64::MAX, -f64::MAX, 0.0]; 4],
            coplanar: false,
        };
        assert_eq!(
            shadow.covers(Vec3::X),
            Err(crate::Error::from(GeometryError::ArithmeticOverflow))
        );
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
                                shadow.as_ref().is_some_and(|s| s.covers(point).unwrap()),
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
        assert!(shadow.covers(Vec3::new(0.25, 0.25, 0.0))?);
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
                assert!(shadow.covers(Vec3::new(0.25, 0.25, 0.0))?);
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
        assert!(shadow.covers(Vec3::new(0.25, 0.25, 0.0))?);
        assert!(Shadow::new(&source, &blocker, 0, 1, -Vec3::X, |_| Ok(()))?.is_none());
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
        assert_eq!(work, exact_depth::WORK);
        assert!(shadow.covers(Vec3::new(0.25, 0.25, 0.0))?);
        Ok(())
    }
}
