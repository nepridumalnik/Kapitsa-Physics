//! Shadow half-spaces evaluated in the source triangle's affine domain.
//! Adaptive orientation predicates come from GeoRust's MIT/Apache-2.0 `robust`.
use super::{GeometryError, SurfaceTriangle, Vec3};
use robust::{Coord, Coord3D, orient2d, orient3d};

pub(super) struct Shadow {
    pub planes: [[f64; 3]; 4],
    coplanar: bool,
}

// After a common power-of-two scale, this span keeps coordinates within
// [2^-127, 2^128). Including 53-bit significands, degree-three orientation
// expansions stay well inside the normal f64 range (roughly 2^-540..2^400).
// Unrepresentable cases are errors, never a distance epsilon or a missed hit.
const MAX_EXPONENT_SPAN: i32 = 254;

struct PredicateFrame {
    shift: i32,
}
impl PredicateFrame {
    fn new(points: &[Vec3]) -> Result<Self, GeometryError> {
        let mut bounds: Option<(i32, i32)> = None;
        for point in points {
            if !point.is_finite() {
                return Err(GeometryError::ArithmeticOverflow);
            }
            for value in [point.x, point.y, point.z] {
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
                bounds = Some(bounds.map_or((exponent, exponent), |(low, high)| {
                    (low.min(exponent), high.max(exponent))
                }));
            }
        }
        let Some((low, high)) = bounds else {
            return Ok(Self { shift: 0 });
        };
        if high - low > MAX_EXPONENT_SPAN {
            return Err(GeometryError::PrecisionLoss);
        }
        Ok(Self {
            shift: -((low + high) / 2),
        })
    }

    fn point(&self, point: Vec3) -> Result<Coord3D<f64>, GeometryError> {
        let point = Vec3::new(
            scale(point.x, self.shift)?,
            scale(point.y, self.shift)?,
            scale(point.z, self.shift)?,
        );
        Ok(Coord3D {
            x: point.x,
            y: point.y,
            z: point.z,
        })
    }

    fn orientation(&self, a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Result<f64, GeometryError> {
        finite(orient3d(
            self.point(a)?,
            self.point(b)?,
            self.point(c)?,
            self.point(d)?,
        ))
    }
}

fn scale(mut value: f64, mut shift: i32) -> Result<f64, GeometryError> {
    while shift != 0 {
        let step = shift.clamp(-512, 512);
        let scaled = value * 2.0_f64.powi(step);
        if !scaled.is_finite() {
            return Err(GeometryError::ArithmeticOverflow);
        }
        if value != 0.0 && scaled == 0.0 {
            return Err(GeometryError::PrecisionLoss);
        }
        value = scaled;
        shift -= step;
    }
    Ok(value)
}

fn finite(value: f64) -> Result<f64, GeometryError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(GeometryError::ArithmeticOverflow)
    }
}

fn normalize(values: [f64; 3]) -> Result<[f64; 3], GeometryError> {
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
        return Err(GeometryError::PrecisionLoss);
    }
    Ok(scaled)
}

pub(super) fn distance(values: [f64; 3], point: Vec3) -> f64 {
    // Evaluate the barycentric form with compensated products and sums.
    // This reduces cancellation but does not establish an exact sign bound.
    let x = values[1] - values[0];
    let y = values[2] - values[0];
    let x_virtual = x - values[1];
    let y_virtual = y - values[2];
    let x_error = (values[1] - (x - x_virtual)) - (values[0] + x_virtual);
    let y_error = (values[2] - (y - y_virtual)) - (values[0] + y_virtual);
    let xp = x * point.x;
    let yp = y * point.y;
    let xr = x.mul_add(point.x, -xp);
    let yr = y.mul_add(point.y, -yp);
    let sum = values[0] + xp;
    let v = sum - values[0];
    let residual = (values[0] - (sum - v)) + (xp - v);
    let result = sum + yp;
    let v = result - sum;
    let residual = residual + ((sum - (result - v)) + (yp - v));
    result + (residual + xr + yr + x_error * point.x + y_error * point.y)
}

impl Shadow {
    pub fn new(
        source: &SurfaceTriangle,
        blocker: &SurfaceTriangle,
        source_index: usize,
        blocker_index: usize,
        upstream: Vec3,
    ) -> Result<Option<Self>, GeometryError> {
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
            return Err(GeometryError::InvalidGeometry);
        }
        // Direction may be positively rescaled without changing a shadow plane.
        let direction = upstream * size;
        if !direction.is_finite()
            || [upstream.x, upstream.y, upstream.z]
                .iter()
                .zip([direction.x, direction.y, direction.z])
                .any(|(a, b)| *a != 0.0 && b == 0.0)
        {
            return Err(GeometryError::PrecisionLoss);
        }
        let frame = PredicateFrame::new(&[Vec3::ZERO, edge, other, direction])?;
        let denominator = -frame.orientation(Vec3::ZERO, edge, other, direction)?;
        if denominator == 0.0 {
            return Ok(None);
        }
        let frame = PredicateFrame::new(&[
            a,
            b,
            c,
            source.vertices[0],
            source.vertices[1],
            source.vertices[2],
        ])?;
        let mut depth = [0.0; 3];
        for (value, point) in depth.iter_mut().zip(source.vertices) {
            // orient(a,b,c,p) is minus the signed plane distance numerator.
            *value = frame.orientation(a, b, c, point)? * denominator.signum();
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
        for (plane, (start, end, opposite)) in
            planes[..3]
                .iter_mut()
                .zip([(a, b, c), (b, c, a), (c, a, b)])
        {
            let edge = end - start;
            let offsets = source.vertices.map(|p| p - start);
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
                return Err(GeometryError::PrecisionLoss);
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

    pub fn covers(&self, point: Vec3) -> bool {
        self.planes[..3]
            .iter()
            .all(|plane| distance(*plane, point) >= 0.0)
            && (self.coplanar || distance(self.planes[3], point) > 0.0)
    }
}

/// Restrict an affine rigid-body flow to its genuinely windward region.
/// A stagnation point at the original centroid must not discard positive area.
pub(super) fn windward_triangle(
    normal: Vec3,
    velocities: [Vec3; 3],
) -> Result<Option<Vec<Vec3>>, GeometryError> {
    let mut values = [0.0; 3];
    for (value, velocity) in values.iter_mut().zip(velocities) {
        *value = normal
            .checked_dot(velocity)
            .ok_or(GeometryError::ArithmeticOverflow)?;
    }
    if !values.iter().any(|value| *value > 0.0) {
        return Ok(None);
    }
    let values = normalize(values)?;
    let vertices = [Vec3::ZERO, Vec3::X, Vec3::Y];
    let mut positive = Vec::new();
    positive
        .try_reserve_exact(4)
        .map_err(|_| GeometryError::AllocationFailed)?;
    for index in 0..3 {
        let next = (index + 1) % 3;
        let (a, b) = (values[index], values[next]);
        if a >= 0.0 {
            super::checked_push(&mut positive, vertices[index])?;
        }
        if (a > 0.0 && b < 0.0) || (a < 0.0 && b > 0.0) {
            let point = vertices[index] + (vertices[next] - vertices[index]) * (a / (a - b));
            if !point.is_finite() {
                return Err(GeometryError::ArithmeticOverflow);
            }
            super::checked_push(&mut positive, point)?;
        }
    }
    if positive.len() < 3 {
        return Err(GeometryError::PrecisionLoss);
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

pub(super) fn polygon_fraction(points: &[Vec3]) -> Result<f64, GeometryError> {
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

pub(super) fn triangle_fraction(points: [Vec3; 3]) -> Result<f64, GeometryError> {
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
    scale(value.abs(), -2 * frame.shift)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::geometry::GeometryOwner;

    #[test]
    fn affine_distance_preserves_small_positive_value_through_cancellation() {
        // Exact affine value is 2 * offset.
        const OFFSET: f64 = 1.0 / ((1u64 << 55) as f64);
        let point = Vec3::new(0.5, OFFSET, 0.0);
        assert_eq!(distance([-1.0, 1.0, 1.0], point), 2.0 * OFFSET);
    }

    #[test]
    fn affine_distance_compensates_coefficient_difference() {
        // At vertex 1 the exact barycentric value is values[1], even
        // when subtracting values[0] rounds away that small value.
        const SMALL: f64 = 1.0 / ((1u64 << 54) as f64);
        assert_eq!(distance([1.0, SMALL, 0.0], Vec3::X), SMALL);
        assert_eq!(distance([1.0, 0.0, SMALL], Vec3::Y), SMALL);
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
            Err(GeometryError::PrecisionLoss)
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
    fn affine_shadow_matches_scalar_rays_away_from_boundaries() -> Result<(), GeometryError> {
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
                        .ok_or(GeometryError::InvalidGeometry)?;
                    let shadow = Shadow::new(&source, &blocker, 0, 1, direction)?;
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
                            assert_eq!(shadow.as_ref().is_some_and(|s| s.covers(point)), expected);
                        }
                    }
                }
            }
        }
        Ok(())
    }

    #[test]
    fn unsupported_precision_is_an_error_instead_of_a_missed_blocker() {
        let source = SurfaceTriangle {
            vertices: [Vec3::ZERO, Vec3::Y, Vec3::Z],
            normal: Vec3::X,
            area: 0.5,
            owner: GeometryOwner::Body,
        };
        let blocker = SurfaceTriangle {
            vertices: source.vertices.map(|p| p + Vec3::X * 1e-100),
            ..source
        };
        assert!(matches!(
            Shadow::new(&source, &blocker, 0, 1, Vec3::X),
            Err(GeometryError::PrecisionLoss)
        ));
    }
}
