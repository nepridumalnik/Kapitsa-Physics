use crate::{
    aabb::Aabb,
    shape::{HeightField, LevelSet, Plane, Shape, Triangle, TriangleMesh},
    transform::Transform,
};
use kapitsa_math::{Quat, Vec3, numeric};

use super::{
    CollisionError,
    gjk::{self, GjkDistance},
    height_field::{self, HeightFieldTriangle},
    level_set::{self, LevelSetCell},
    mesh::{MeshQueryWorkspace, TriangleBvh},
    support::{support_pair, support_world},
};

const MAX_CCD_ITERATIONS: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CcdHit {
    pub(crate) toi: f64,
    pub(crate) normal: Vec3,
    pub(crate) point_a: Vec3,
    pub(crate) point_b: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct CcdCandidateToi {
    pub(crate) evaluated: bool,
    pub(crate) impact_time: f64,
    pub(crate) hit: Option<CcdHit>,
}

impl Default for CcdCandidateToi {
    fn default() -> Self {
        Self {
            evaluated: false,
            impact_time: 0.0,
            hit: None,
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct CcdMotion<'a> {
    start: Transform,
    end: Transform,
    translation_bound: f64,
    linear_translation: bool,
    rotation_bound: f64,
    sample: Option<&'a dyn Fn(f64) -> Result<Transform, CollisionError>>,
}

impl CcdMotion<'_> {
    pub(crate) fn linear(start: Transform, end: Transform) -> Result<Self, CollisionError> {
        Ok(Self {
            start,
            end,
            translation_bound: translation_delta(start, end)?.length(),
            linear_translation: true,
            rotation_bound: rotation_angle(start.rotation(), end.rotation())?,
            sample: None,
        })
    }

    pub(crate) fn trajectory<'a>(
        start: Transform,
        end: Transform,
        translation_bound: f64,
        linear_translation: bool,
        rotation_bound: f64,
        sample: &'a dyn Fn(f64) -> Result<Transform, CollisionError>,
    ) -> Result<CcdMotion<'a>, CollisionError> {
        if !translation_bound.is_finite()
            || translation_bound < 0.0
            || !rotation_bound.is_finite()
            || rotation_bound < 0.0
        {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(CcdMotion {
            start,
            end,
            translation_bound,
            linear_translation,
            rotation_bound,
            sample: Some(sample),
        })
    }

    fn at(&self, amount: f64) -> Result<Transform, CollisionError> {
        if let Some(sample) = self.sample {
            sample(amount)
        } else {
            interpolate(self.start, self.end, amount)
        }
    }

    fn is_linear(&self) -> bool {
        self.sample.is_none()
    }
}

/// Finds the first contact between two shapes following their start-to-end poses.
pub(crate) fn convex_toi(
    shape_a: &Shape,
    start_a: Transform,
    end_a: Transform,
    shape_b: &Shape,
    start_b: Transform,
    end_b: Transform,
) -> Result<Option<CcdHit>, CollisionError> {
    let motion_a = CcdMotion::linear(start_a, end_a)?;
    let motion_b = CcdMotion::linear(start_b, end_b)?;
    convex_toi_motion(shape_a, &motion_a, shape_b, &motion_b)
}

pub(crate) fn convex_toi_motion(
    shape_a: &Shape,
    motion_a: &CcdMotion<'_>,
    shape_b: &Shape,
    motion_b: &CcdMotion<'_>,
) -> Result<Option<CcdHit>, CollisionError> {
    if !shape_a.is_convex() || !(shape_b.is_convex() || matches!(shape_b, Shape::Triangle(_))) {
        return Err(CollisionError::UnsupportedGeometryPair);
    }
    if motion_a.linear_translation
        && motion_b.linear_translation
        && let (Shape::Sphere(sphere_a), Shape::Sphere(sphere_b)) = (shape_a, shape_b)
    {
        return sphere_sphere_toi(
            sphere_a.radius(),
            motion_a.start,
            motion_a.end,
            sphere_b.radius(),
            motion_b.start,
            motion_b.end,
        );
    }
    match (shape_a, shape_b) {
        (Shape::Sphere(sphere), Shape::Cuboid(cuboid))
            if motion_b.rotation_bound == 0.0
                && motion_a.linear_translation
                && motion_b.linear_translation =>
        {
            return sphere_cuboid_toi(sphere.radius(), motion_a, cuboid, motion_b);
        }
        (Shape::Sphere(sphere), Shape::Cuboid(cuboid)) if motion_b.rotation_bound == 0.0 => {
            return sphere_cuboid_toi_sampled(sphere.radius(), motion_a, cuboid, motion_b);
        }
        (Shape::Cuboid(cuboid), Shape::Sphere(sphere))
            if motion_a.rotation_bound == 0.0
                && motion_a.linear_translation
                && motion_b.linear_translation =>
        {
            return sphere_cuboid_toi(sphere.radius(), motion_b, cuboid, motion_a)
                .map(|hit| hit.map(reverse_hit));
        }
        (Shape::Cuboid(cuboid), Shape::Sphere(sphere)) if motion_a.rotation_bound == 0.0 => {
            return sphere_cuboid_toi_sampled(sphere.radius(), motion_b, cuboid, motion_a)
                .map(|hit| hit.map(reverse_hit));
        }
        _ => {}
    }
    let radius_a = rotation_radius(shape_a)?;
    let radius_b = rotation_radius(shape_b)?;
    let translation_bound = if motion_a.linear_translation && motion_b.linear_translation {
        translation_delta(motion_b.start, motion_b.end)?
            .checked_sub(translation_delta(motion_a.start, motion_a.end)?)
            .ok_or(CollisionError::ArithmeticOverflow)?
            .length()
    } else {
        motion_a.translation_bound + motion_b.translation_bound
    };
    let motion_bound =
        translation_bound + motion_a.rotation_bound * radius_a + motion_b.rotation_bound * radius_b;
    if !motion_bound.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let mut toi = 0.0;
    for _ in 0..MAX_CCD_ITERATIONS {
        let transform_a = motion_a.at(toi)?;
        let transform_b = motion_b.at(toi)?;
        let distance = match gjk::distance(shape_a, &transform_a, shape_b, &transform_b) {
            Ok(distance) => distance,
            Err(CollisionError::NonConvergent) => {
                let intersection = gjk::intersect(shape_a, &transform_a, shape_b, &transform_b)?;
                if intersection.intersects {
                    return Ok(Some(intersection_hit(
                        toi,
                        shape_a,
                        transform_a,
                        shape_b,
                        transform_b,
                    )?));
                }
                return Err(CollisionError::NonConvergent);
            }
            Err(error) => return Err(error),
        };
        if distance.intersects {
            return Ok(Some(distance_hit(toi, distance, transform_a, transform_b)));
        }
        if motion_bound == 0.0 {
            return Ok(None);
        }
        let remaining = 1.0 - toi;
        let advance = distance.lower_bound / motion_bound;
        if !advance.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if advance > remaining {
            return Ok(None);
        }
        let next_toi = toi + advance;
        if next_toi <= toi {
            return Err(CollisionError::NonConvergent);
        }
        toi = next_toi.min(1.0);
    }
    Err(CollisionError::NonConvergent)
}

fn sphere_sphere_toi(
    radius_a: f64,
    start_a: Transform,
    end_a: Transform,
    radius_b: f64,
    start_b: Transform,
    end_b: Transform,
) -> Result<Option<CcdHit>, CollisionError> {
    let relative_start = start_b
        .translation()
        .checked_sub(start_a.translation())
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let motion_a = translation_delta(start_a, end_a)?;
    let motion_b = translation_delta(start_b, end_b)?;
    let relative_motion = motion_b
        .checked_sub(motion_a)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let radius = radius_a + radius_b;
    let a = relative_motion
        .checked_dot(relative_motion)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let b = 2.0
        * relative_start
            .checked_dot(relative_motion)
            .ok_or(CollisionError::ArithmeticOverflow)?;
    let c = relative_start
        .checked_dot(relative_start)
        .ok_or(CollisionError::ArithmeticOverflow)?
        - radius * radius;
    if !radius.is_finite() || !a.is_finite() || !b.is_finite() || !c.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let toi = if c <= 0.0 {
        0.0
    } else {
        if a <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let discriminant = b * b - 4.0 * a * c;
        if !discriminant.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let tolerance = (b * b).max((4.0 * a * c).abs()).max(1.0) * f64::EPSILON * 128.0;
        if discriminant < -tolerance {
            return Ok(None);
        }
        let toi = (-b - discriminant.max(0.0).sqrt()) / (2.0 * a);
        if !toi.is_finite() || !(0.0..=1.0).contains(&toi) {
            return Ok(None);
        }
        toi
    };

    let center_a = start_a
        .translation()
        .checked_add(
            motion_a
                .checked_mul_scalar(toi)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let center_b = start_b
        .translation()
        .checked_add(
            motion_b
                .checked_mul_scalar(toi)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let normal = center_b
        .checked_sub(center_a)
        .and_then(Vec3::try_normalize)
        .unwrap_or(Vec3::X);
    let point_a = center_a
        .checked_add(
            normal
                .checked_mul_scalar(radius_a)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let point_b = center_b
        .checked_sub(
            normal
                .checked_mul_scalar(radius_b)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    Ok(Some(CcdHit {
        toi,
        normal,
        point_a,
        point_b,
    }))
}

fn reverse_hit(hit: CcdHit) -> CcdHit {
    CcdHit {
        toi: hit.toi,
        normal: -hit.normal,
        point_a: hit.point_b,
        point_b: hit.point_a,
    }
}

fn sphere_cuboid_toi(
    radius: f64,
    sphere_motion: &CcdMotion<'_>,
    cuboid: &crate::shape::Cuboid,
    cuboid_motion: &CcdMotion<'_>,
) -> Result<Option<CcdHit>, CollisionError> {
    let inverse_start = cuboid_motion
        .start
        .inverse()
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    let relative_start = inverse_start.transform_point(sphere_motion.start.translation());
    let sphere_delta = translation_delta(sphere_motion.start, sphere_motion.end)?;
    let cuboid_delta = translation_delta(cuboid_motion.start, cuboid_motion.end)?;
    let relative_delta = inverse_start.transform_vector(
        sphere_delta
            .checked_sub(cuboid_delta)
            .ok_or(CollisionError::ArithmeticOverflow)?,
    );
    if !relative_start.is_finite() || !relative_delta.is_finite() || !radius.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let half = cuboid.half_extents();
    let start = [relative_start.x, relative_start.y, relative_start.z];
    let delta = [relative_delta.x, relative_delta.y, relative_delta.z];
    let half = [half.x, half.y, half.z];
    let radius_squared = radius * radius;
    if !radius_squared.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let mut boundaries = [0.0; 8];
    let mut boundary_count = 1;
    for axis in 0..3 {
        if delta[axis] == 0.0 {
            continue;
        }
        for face in [-half[axis], half[axis]] {
            let crossing = (face - start[axis]) / delta[axis];
            if !crossing.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            if crossing > 0.0 && crossing < 1.0 {
                boundaries[boundary_count] = crossing;
                boundary_count += 1;
            }
        }
    }
    boundaries[boundary_count] = 1.0;
    boundary_count += 1;
    for index in 1..boundary_count {
        let mut cursor = index;
        while cursor > 0 && boundaries[cursor] < boundaries[cursor - 1] {
            boundaries.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }

    for interval in 0..boundary_count - 1 {
        let start_time = boundaries[interval];
        let end_time = boundaries[interval + 1];
        if end_time <= start_time {
            continue;
        }
        let at_start = point_on_segment(start, delta, start_time)?;
        if let Some(contact) = sphere_cuboid_contact(at_start, half, radius_squared)? {
            return Ok(Some(sphere_cuboid_hit(
                start_time,
                contact,
                radius,
                sphere_motion,
                cuboid_motion,
            )?));
        }

        let midpoint = point_on_segment(start, delta, (start_time + end_time) * 0.5)?;
        let mut a = 0.0;
        let mut b = 0.0;
        let mut c = -radius_squared;
        for axis in 0..3 {
            let face = if midpoint[axis] < -half[axis] {
                Some(-half[axis])
            } else if midpoint[axis] > half[axis] {
                Some(half[axis])
            } else {
                None
            };
            let Some(face) = face else {
                continue;
            };
            let offset = at_start[axis] - face;
            a += delta[axis] * delta[axis];
            b += 2.0 * offset * delta[axis];
            c += offset * offset;
        }
        if !a.is_finite() || !b.is_finite() || !c.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if a <= f64::MIN_POSITIVE {
            continue;
        }
        let discriminant = b * b - 4.0 * a * c;
        if !discriminant.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let tolerance = (b * b).max((4.0 * a * c).abs()).max(1.0) * f64::EPSILON * 128.0;
        if discriminant < -tolerance {
            continue;
        }
        let local_time = (-b - discriminant.max(0.0).sqrt()) / (2.0 * a);
        let interval_duration = end_time - start_time;
        if !local_time.is_finite()
            || local_time < 0.0
            || local_time > interval_duration + f64::EPSILON * 128.0
        {
            continue;
        }
        let toi = (start_time + local_time).min(end_time);
        let at_toi = point_on_segment(start, delta, toi)?;
        if let Some(contact) = sphere_cuboid_contact(at_toi, half, radius_squared)? {
            return Ok(Some(sphere_cuboid_hit(
                toi,
                contact,
                radius,
                sphere_motion,
                cuboid_motion,
            )?));
        }
    }
    Ok(None)
}

fn sphere_cuboid_toi_sampled(
    radius: f64,
    sphere_motion: &CcdMotion<'_>,
    cuboid: &crate::shape::Cuboid,
    cuboid_motion: &CcdMotion<'_>,
) -> Result<Option<CcdHit>, CollisionError> {
    let motion_bound = sphere_motion.translation_bound + cuboid_motion.translation_bound;
    let radius_squared = radius * radius;
    if !motion_bound.is_finite() || !radius.is_finite() || !radius_squared.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let half = cuboid.half_extents();
    let half = [half.x, half.y, half.z];
    let mut toi = 0.0;
    for _ in 0..MAX_CCD_ITERATIONS {
        let sphere_transform = sphere_motion.at(toi)?;
        let cuboid_transform = cuboid_motion.at(toi)?;
        let inverse = cuboid_transform
            .inverse()
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let center = inverse.transform_point(sphere_transform.translation());
        if !center.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let center = [center.x, center.y, center.z];
        let closest = [
            center[0].clamp(-half[0], half[0]),
            center[1].clamp(-half[1], half[1]),
            center[2].clamp(-half[2], half[2]),
        ];
        let difference = Vec3::new(
            center[0] - closest[0],
            center[1] - closest[1],
            center[2] - closest[2],
        );
        let distance_squared = difference
            .checked_dot(difference)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let tolerance = distance_squared.max(radius_squared).max(1.0) * f64::EPSILON * 128.0;
        if distance_squared <= radius_squared + tolerance {
            let contact = sphere_cuboid_contact(center, half, radius_squared)?
                .ok_or(CollisionError::InvalidContact)?;
            return sphere_cuboid_hit(toi, contact, radius, sphere_motion, cuboid_motion).map(Some);
        }
        if motion_bound == 0.0 {
            return Ok(None);
        }
        let distance = distance_squared.sqrt();
        let advance = (distance - radius) / motion_bound;
        if !advance.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let remaining = 1.0 - toi;
        if advance > remaining {
            return Ok(None);
        }
        let next_toi = toi + advance;
        if next_toi <= toi {
            return Err(CollisionError::NonConvergent);
        }
        toi = next_toi.min(1.0);
    }
    Err(CollisionError::NonConvergent)
}

fn point_on_segment(
    start: [f64; 3],
    delta: [f64; 3],
    time: f64,
) -> Result<[f64; 3], CollisionError> {
    let point = [
        start[0] + delta[0] * time,
        start[1] + delta[1] * time,
        start[2] + delta[2] * time,
    ];
    point
        .iter()
        .all(|coordinate| coordinate.is_finite())
        .then_some(point)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn sphere_cuboid_contact(
    center: [f64; 3],
    half: [f64; 3],
    radius_squared: f64,
) -> Result<Option<(Vec3, Vec3)>, CollisionError> {
    let closest = [
        center[0].clamp(-half[0], half[0]),
        center[1].clamp(-half[1], half[1]),
        center[2].clamp(-half[2], half[2]),
    ];
    let difference = Vec3::new(
        center[0] - closest[0],
        center[1] - closest[1],
        center[2] - closest[2],
    );
    let distance_squared = difference
        .checked_dot(difference)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let tolerance = distance_squared.max(radius_squared).max(1.0) * f64::EPSILON * 128.0;
    if distance_squared > radius_squared + tolerance {
        return Ok(None);
    }
    if distance_squared > f64::MIN_POSITIVE {
        let normal = -difference
            .try_normalize()
            .ok_or(CollisionError::InvalidContact)?;
        return Ok(Some((
            normal,
            Vec3::new(closest[0], closest[1], closest[2]),
        )));
    }

    let face_distances = [
        (center[0] + half[0], Vec3::new(-1.0, 0.0, 0.0)),
        (half[0] - center[0], Vec3::X),
        (center[1] + half[1], Vec3::new(0.0, -1.0, 0.0)),
        (half[1] - center[1], Vec3::Y),
        (center[2] + half[2], Vec3::new(0.0, 0.0, -1.0)),
        (half[2] - center[2], Vec3::Z),
    ];
    let (face_distance, normal) = face_distances
        .into_iter()
        .min_by(|left, right| left.0.total_cmp(&right.0))
        .ok_or(CollisionError::InvalidContact)?;
    let point = Vec3::new(
        if normal.x < 0.0 {
            -half[0]
        } else if normal.x > 0.0 {
            half[0]
        } else {
            center[0]
        },
        if normal.y < 0.0 {
            -half[1]
        } else if normal.y > 0.0 {
            half[1]
        } else {
            center[1]
        },
        if normal.z < 0.0 {
            -half[2]
        } else if normal.z > 0.0 {
            half[2]
        } else {
            center[2]
        },
    );
    if face_distance < 0.0 {
        return Err(CollisionError::InvalidContact);
    }
    Ok(Some((normal, point)))
}

fn sphere_cuboid_hit(
    toi: f64,
    (normal_local, point_local): (Vec3, Vec3),
    radius: f64,
    sphere_motion: &CcdMotion<'_>,
    cuboid_motion: &CcdMotion<'_>,
) -> Result<CcdHit, CollisionError> {
    let sphere_transform = sphere_motion.at(toi)?;
    let cuboid_transform = cuboid_motion.at(toi)?;
    let normal = cuboid_transform
        .transform_vector(normal_local)
        .try_normalize()
        .ok_or(CollisionError::InvalidContact)?;
    let point_a = sphere_transform
        .translation()
        .checked_add(
            normal
                .checked_mul_scalar(radius)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let point_b = cuboid_transform.transform_point(point_local);
    if !point_a.is_finite() || !point_b.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(CcdHit {
        toi,
        normal,
        point_a,
        point_b,
    })
}

/// Finds a convex shape's first contact with a moving plane.
#[allow(clippy::too_many_arguments)]
pub(crate) fn plane_toi(
    convex: &Shape,
    start_convex: Transform,
    end_convex: Transform,
    plane: &Plane,
    start_plane: Transform,
    end_plane: Transform,
    two_sided: bool,
    plane_is_a: bool,
) -> Result<Option<CcdHit>, CollisionError> {
    let convex_motion = CcdMotion::linear(start_convex, end_convex)?;
    let plane_motion = CcdMotion::linear(start_plane, end_plane)?;
    plane_toi_motion(
        convex,
        &convex_motion,
        plane,
        &plane_motion,
        two_sided,
        plane_is_a,
    )
}

pub(crate) fn plane_toi_motion(
    convex: &Shape,
    convex_motion: &CcdMotion<'_>,
    plane: &Plane,
    plane_motion: &CcdMotion<'_>,
    two_sided: bool,
    plane_is_a: bool,
) -> Result<Option<CcdHit>, CollisionError> {
    if !convex.is_convex() {
        return Err(CollisionError::UnsupportedGeometryPair);
    }
    let radius = shape_radius(convex)?;
    let relative_center = convex_motion
        .start
        .translation()
        .checked_sub(plane_motion.start.translation())
        .and_then(Vec3::checked_length)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let plane_pivot_radius = relative_center
        + radius
        + plane.distance().abs()
        + convex_motion.translation_bound
        + plane_motion.translation_bound;
    let motion_bound = convex_motion.translation_bound
        + plane_motion.translation_bound
        + convex_motion.rotation_bound * radius
        + plane_motion.rotation_bound * plane_pivot_radius;
    if !motion_bound.is_finite() || !plane_pivot_radius.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let initial = plane_gap(
        convex,
        convex_motion.start,
        plane,
        plane_motion.start,
        two_sided,
    )?;
    if !two_sided && initial.center_side < 0.0 {
        return Ok(None);
    }
    let mut toi = 0.0;
    for _ in 0..MAX_CCD_ITERATIONS {
        let convex_transform = convex_motion.at(toi)?;
        let plane_transform = plane_motion.at(toi)?;
        let gap = plane_gap(convex, convex_transform, plane, plane_transform, two_sided)?;
        if gap.separation <= tolerance(gap.separation) {
            let (normal, point_a, point_b) = if plane_is_a {
                (gap.normal, gap.plane_point, gap.convex_point)
            } else {
                (-gap.normal, gap.convex_point, gap.plane_point)
            };
            return Ok(Some(CcdHit {
                toi,
                normal,
                point_a,
                point_b,
            }));
        }
        if motion_bound == 0.0 {
            return Ok(None);
        }
        let advance = gap.separation / motion_bound;
        let remaining = 1.0 - toi;
        if !advance.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if advance > remaining {
            return Ok(None);
        }
        let next_toi = toi + advance;
        if next_toi <= toi {
            return Err(CollisionError::NonConvergent);
        }
        toi = next_toi.min(1.0);
    }
    Err(CollisionError::NonConvergent)
}

/// Searches a mesh BVH using the swept convex's conservative local-space envelope.
#[allow(clippy::too_many_arguments)]
pub(crate) fn triangle_mesh_toi(
    convex: &Shape,
    start_convex: Transform,
    end_convex: Transform,
    mesh: &TriangleMesh,
    deformed_vertices: Option<&[Vec3]>,
    bvh: &TriangleBvh,
    start_mesh: Transform,
    end_mesh: Transform,
    workspace: &mut MeshQueryWorkspace,
    candidates: &mut Vec<usize>,
) -> Result<Option<CcdHit>, CollisionError> {
    let convex_motion = CcdMotion::linear(start_convex, end_convex)?;
    let mesh_motion = CcdMotion::linear(start_mesh, end_mesh)?;
    triangle_mesh_toi_motion(
        convex,
        &convex_motion,
        mesh,
        deformed_vertices,
        bvh,
        &mesh_motion,
        workspace,
        candidates,
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn triangle_mesh_toi_motion(
    convex: &Shape,
    convex_motion: &CcdMotion<'_>,
    mesh: &TriangleMesh,
    deformed_vertices: Option<&[Vec3]>,
    bvh: &TriangleBvh,
    mesh_motion: &CcdMotion<'_>,
    workspace: &mut MeshQueryWorkspace,
    candidates: &mut Vec<usize>,
) -> Result<Option<CcdHit>, CollisionError> {
    let vertices = deformed_vertices.unwrap_or_else(|| mesh.vertices());
    if vertices.len() != mesh.vertices().len() || bvh.triangle_count() != mesh.triangles().len() {
        return Err(CollisionError::GeometryBounds);
    }
    let query_bounds = relative_swept_bounds(convex, convex_motion, mesh_motion)?;
    bvh.query(query_bounds, workspace, candidates)?;
    let mut best = None;
    for triangle_id in candidates.iter().copied() {
        let [a, b, c] = *mesh
            .triangles()
            .get(triangle_id)
            .ok_or(CollisionError::GeometryBounds)?;
        let triangle = Triangle::from_vertices([
            *vertices.get(a).ok_or(CollisionError::GeometryBounds)?,
            *vertices.get(b).ok_or(CollisionError::GeometryBounds)?,
            *vertices.get(c).ok_or(CollisionError::GeometryBounds)?,
        ])
        .map_err(|_| CollisionError::InvalidContact)?;
        consider_triangle_toi(&mut best, convex, convex_motion, triangle, mesh_motion)?;
    }
    Ok(best)
}

/// Searches only swept height-field cells and their two fixed-diagonal triangles.
pub(crate) fn height_field_toi(
    convex: &Shape,
    start_convex: Transform,
    end_convex: Transform,
    field: &HeightField,
    start_field: Transform,
    end_field: Transform,
    candidates: &mut Vec<HeightFieldTriangle>,
) -> Result<Option<CcdHit>, CollisionError> {
    let convex_motion = CcdMotion::linear(start_convex, end_convex)?;
    let field_motion = CcdMotion::linear(start_field, end_field)?;
    height_field_toi_motion(convex, &convex_motion, field, &field_motion, candidates)
}

pub(crate) fn height_field_toi_motion(
    convex: &Shape,
    convex_motion: &CcdMotion<'_>,
    field: &HeightField,
    field_motion: &CcdMotion<'_>,
    candidates: &mut Vec<HeightFieldTriangle>,
) -> Result<Option<CcdHit>, CollisionError> {
    let query_bounds = relative_swept_bounds(convex, convex_motion, field_motion)?;
    height_field::query_triangles(field, query_bounds, candidates)?;
    let mut best = None;
    for candidate in candidates.iter().copied() {
        let triangle = Triangle::from_vertices(candidate.vertices)
            .map_err(|_| CollisionError::InvalidContact)?;
        consider_triangle_toi(&mut best, convex, convex_motion, triangle, field_motion)?;
    }
    Ok(best)
}

/// Advances against the trilinear zero surface using a cell-wise gradient bound.
pub(crate) fn level_set_toi(
    convex: &Shape,
    start_convex: Transform,
    end_convex: Transform,
    field: &LevelSet,
    start_field: Transform,
    end_field: Transform,
    candidates: &mut Vec<LevelSetCell>,
) -> Result<Option<CcdHit>, CollisionError> {
    let convex_motion = CcdMotion::linear(start_convex, end_convex)?;
    let field_motion = CcdMotion::linear(start_field, end_field)?;
    level_set_toi_motion(convex, &convex_motion, field, &field_motion, candidates)
}

pub(crate) fn level_set_toi_motion(
    convex: &Shape,
    convex_motion: &CcdMotion<'_>,
    field: &LevelSet,
    field_motion: &CcdMotion<'_>,
    candidates: &mut Vec<LevelSetCell>,
) -> Result<Option<CcdHit>, CollisionError> {
    let query_bounds = relative_swept_bounds(convex, convex_motion, field_motion)?;
    level_set::query_surface_cells(field, query_bounds, candidates)?;
    if candidates.is_empty() {
        return Ok(None);
    }
    let convex_radius = shape_radius(convex)?;
    let field_bounds = field
        .local_bounds()
        .map_err(|_| CollisionError::GeometryBounds)?;
    let field_radius = bounds_radius(field_bounds)?;
    let motion_bound = convex_motion.translation_bound
        + field_motion.translation_bound
        + convex_motion.rotation_bound * convex_radius
        + field_motion.rotation_bound * field_radius;
    if !motion_bound.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let mut toi = 0.0;
    for _ in 0..MAX_CCD_ITERATIONS {
        let convex_transform = convex_motion.at(toi)?;
        let field_transform = field_motion.at(toi)?;
        let mut closest = None;
        for cell in candidates.iter().copied() {
            let candidate = level_cell_gap(convex, convex_transform, field, field_transform, cell)?;
            if closest.is_none_or(|best: LevelSetCandidateGap| candidate.gap < best.gap) {
                closest = Some(candidate);
            }
        }
        let closest = closest.ok_or(CollisionError::GeometryBounds)?;
        if closest.gap <= f64::EPSILON * 128.0 {
            let hit = level_cell_hit(convex, convex_transform, field, field_transform, closest)?;
            return Ok(Some(CcdHit { toi, ..hit }));
        }
        if motion_bound == 0.0 {
            return Ok(None);
        }
        let advance = closest.gap / motion_bound;
        let remaining = 1.0 - toi;
        if !advance.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if advance > remaining {
            return Ok(None);
        }
        let next_toi = toi + advance;
        if next_toi <= toi {
            return Err(CollisionError::NonConvergent);
        }
        toi = next_toi.min(1.0);
    }
    Err(CollisionError::NonConvergent)
}

#[derive(Clone, Copy)]
struct LevelSetCandidateGap {
    gap: f64,
    cell: LevelSetCell,
    nearest_local: Vec3,
    sample: level_set::LevelSetSample,
}

fn level_cell_gap(
    convex: &Shape,
    convex_transform: Transform,
    field: &LevelSet,
    field_transform: Transform,
    cell: LevelSetCell,
) -> Result<LevelSetCandidateGap, CollisionError> {
    let center = convex_transform.translation();
    let world_bounds = cell
        .bounds
        .transformed(&field_transform)
        .map_err(|_| CollisionError::GeometryBounds)?;
    let separation = point_aabb_distance(center, world_bounds)?;
    let radius = shape_radius(convex)?;
    let center_local = field_transform
        .inverse()
        .map_err(|_| CollisionError::ArithmeticOverflow)?
        .transform_point(center);
    let nearest_local = center_local.max(cell.bounds.min()).min(cell.bounds.max());
    let sample = level_set::sample(field, nearest_local)?.ok_or(CollisionError::GeometryBounds)?;
    let gradient_bound = level_set::cell_lipschitz_bound(field, cell.index)?;
    let nearest_world = field_transform.transform_point(nearest_local);
    if !nearest_world.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let center_to_cell = center
        .checked_sub(nearest_world)
        .and_then(Vec3::checked_length)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let field_gap = if gradient_bound > 0.0 {
        sample.value.abs() / gradient_bound - center_to_cell - radius
    } else if sample.value == 0.0 {
        0.0
    } else {
        f64::INFINITY
    };
    let gap = (separation - radius).max(field_gap).max(0.0);
    if !gap.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(LevelSetCandidateGap {
        gap,
        cell,
        nearest_local,
        sample,
    })
}

fn level_cell_hit(
    convex: &Shape,
    convex_transform: Transform,
    field: &LevelSet,
    field_transform: Transform,
    candidate: LevelSetCandidateGap,
) -> Result<CcdHit, CollisionError> {
    let gradient = field_transform
        .transform_vector(candidate.sample.gradient)
        .try_normalize()
        .ok_or(CollisionError::NonConvergent)?;
    let side = if candidate.sample.value < 0.0 {
        -1.0
    } else {
        1.0
    };
    let normal = -gradient * side;
    let convex_point = support_world(convex, &convex_transform, normal)?.0;
    let surface_local =
        project_to_level_set(field, candidate.cell.bounds, candidate.nearest_local)?;
    let surface_point = field_transform.transform_point(surface_local);
    if !surface_point.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(CcdHit {
        toi: 0.0,
        normal,
        point_a: convex_point,
        point_b: surface_point,
    })
}

fn project_to_level_set(
    field: &LevelSet,
    cell_bounds: Aabb,
    mut point: Vec3,
) -> Result<Vec3, CollisionError> {
    for _ in 0..16 {
        let sample = level_set::sample(field, point)?.ok_or(CollisionError::GeometryBounds)?;
        if sample.value.abs() <= f64::EPSILON * point.abs().max_element().max(1.0) * 128.0 {
            return Ok(point);
        }
        let gradient_squared = sample
            .gradient
            .checked_dot(sample.gradient)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        if gradient_squared <= f64::MIN_POSITIVE {
            return Err(CollisionError::NonConvergent);
        }
        point = point
            .checked_sub(
                sample
                    .gradient
                    .checked_mul_scalar(sample.value / gradient_squared)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
            )
            .ok_or(CollisionError::ArithmeticOverflow)?
            .max(cell_bounds.min())
            .min(cell_bounds.max());
    }
    let sample = level_set::sample(field, point)?.ok_or(CollisionError::GeometryBounds)?;
    if sample.value.abs() <= f64::EPSILON * point.abs().max_element().max(1.0) * 128.0 {
        Ok(point)
    } else {
        Err(CollisionError::NonConvergent)
    }
}

fn point_aabb_distance(point: Vec3, bounds: Aabb) -> Result<f64, CollisionError> {
    let min = bounds.min();
    let max = bounds.max();
    let offset = Vec3::new(
        (min.x - point.x).max(0.0).max(point.x - max.x),
        (min.y - point.y).max(0.0).max(point.y - max.y),
        (min.z - point.z).max(0.0).max(point.z - max.z),
    );
    offset
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn consider_triangle_toi(
    best: &mut Option<CcdHit>,
    convex: &Shape,
    convex_motion: &CcdMotion<'_>,
    triangle: Triangle,
    triangle_motion: &CcdMotion<'_>,
) -> Result<(), CollisionError> {
    let triangle = Shape::Triangle(triangle);
    if let Some(hit) = convex_toi_motion(convex, convex_motion, &triangle, triangle_motion)?
        && best.is_none_or(|current| hit.toi < current.toi)
    {
        *best = Some(hit);
    }
    Ok(())
}

fn relative_swept_bounds(
    convex: &Shape,
    convex_motion: &CcdMotion<'_>,
    target_motion: &CcdMotion<'_>,
) -> Result<Aabb, CollisionError> {
    let inverse = target_motion
        .start
        .inverse()
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    let local_center = inverse.transform_point(convex_motion.start.translation());
    let relative_length = local_center
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let convex_delta = convex_motion.translation_bound;
    let target_delta = target_motion.translation_bound;
    let radius = shape_radius(convex)?;
    let target_rotation = target_motion.rotation_bound;
    let expansion = radius
        + convex_delta
        + target_delta
        + target_rotation * (relative_length + convex_delta + target_delta);
    if !local_center.is_finite() || !expansion.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let extent = Vec3::splat(expansion);
    let min = local_center
        .checked_sub(extent)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let max = local_center
        .checked_add(extent)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let margin = local_center.abs().max_element().max(expansion).max(1.0) * f64::EPSILON * 128.0;
    let min = Vec3::new(
        numeric::round_down(min.x - margin).ok_or(CollisionError::ArithmeticOverflow)?,
        numeric::round_down(min.y - margin).ok_or(CollisionError::ArithmeticOverflow)?,
        numeric::round_down(min.z - margin).ok_or(CollisionError::ArithmeticOverflow)?,
    );
    let max = Vec3::new(
        numeric::round_up(max.x + margin).ok_or(CollisionError::ArithmeticOverflow)?,
        numeric::round_up(max.y + margin).ok_or(CollisionError::ArithmeticOverflow)?,
        numeric::round_up(max.z + margin).ok_or(CollisionError::ArithmeticOverflow)?,
    );
    Aabb::new(min, max).map_err(|_| CollisionError::GeometryBounds)
}

/// Conservative bounds for a shape over any rotation between two poses.
pub(crate) fn swept_bounds(
    bounds: Option<Aabb>,
    start: Transform,
    end: Transform,
) -> Result<Option<Aabb>, CollisionError> {
    let Some(bounds) = bounds else {
        return Ok(None);
    };
    let radius = bounds
        .min()
        .abs()
        .max(bounds.max().abs())
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let radius = numeric::round_up(radius).ok_or(CollisionError::ArithmeticOverflow)?;
    let center_min = start.translation().min(end.translation());
    let center_max = start.translation().max(end.translation());
    let min = Vec3::new(
        numeric::round_down(center_min.x - radius).ok_or(CollisionError::ArithmeticOverflow)?,
        numeric::round_down(center_min.y - radius).ok_or(CollisionError::ArithmeticOverflow)?,
        numeric::round_down(center_min.z - radius).ok_or(CollisionError::ArithmeticOverflow)?,
    );
    let max = Vec3::new(
        numeric::round_up(center_max.x + radius).ok_or(CollisionError::ArithmeticOverflow)?,
        numeric::round_up(center_max.y + radius).ok_or(CollisionError::ArithmeticOverflow)?,
        numeric::round_up(center_max.z + radius).ok_or(CollisionError::ArithmeticOverflow)?,
    );
    Aabb::new(min, max)
        .map(Some)
        .map_err(|_| CollisionError::GeometryBounds)
}

pub(crate) fn swept_bounds_motion(
    bounds: Option<Aabb>,
    motion: &CcdMotion<'_>,
) -> Result<Option<Aabb>, CollisionError> {
    if motion.is_linear() {
        return swept_bounds(bounds, motion.start, motion.end);
    }
    let Some(bounds) = bounds else {
        return Ok(None);
    };
    let radius = bounds_radius(bounds)?;
    let segment_travel = numeric::round_up(motion.translation_bound / 16.0)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let extent =
        numeric::round_up(radius + segment_travel).ok_or(CollisionError::ArithmeticOverflow)?;
    let extent = Vec3::splat(extent);
    let mut combined = None;
    for segment in 0..16 {
        let amount = f64::from(segment) / 16.0;
        let center = motion.at(amount)?.translation();
        let min = center
            .checked_sub(extent)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let max = center
            .checked_add(extent)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let bounds = Aabb::new(
            Vec3::new(
                numeric::round_down(min.x).ok_or(CollisionError::ArithmeticOverflow)?,
                numeric::round_down(min.y).ok_or(CollisionError::ArithmeticOverflow)?,
                numeric::round_down(min.z).ok_or(CollisionError::ArithmeticOverflow)?,
            ),
            Vec3::new(
                numeric::round_up(max.x).ok_or(CollisionError::ArithmeticOverflow)?,
                numeric::round_up(max.y).ok_or(CollisionError::ArithmeticOverflow)?,
                numeric::round_up(max.z).ok_or(CollisionError::ArithmeticOverflow)?,
            ),
        )
        .map_err(|_| CollisionError::GeometryBounds)?;
        combined = Some(combined.map_or(bounds, |current: Aabb| current.union(&bounds)));
    }
    Ok(combined)
}

struct PlaneGap {
    separation: f64,
    center_side: f64,
    normal: Vec3,
    convex_point: Vec3,
    plane_point: Vec3,
}

fn plane_gap(
    convex: &Shape,
    convex_transform: Transform,
    plane: &Plane,
    plane_transform: Transform,
    two_sided: bool,
) -> Result<PlaneGap, CollisionError> {
    let normal = plane_transform
        .transform_vector(plane.normal())
        .try_normalize()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let plane_point = plane_transform.transform_point(plane.normal() * plane.distance());
    let center = convex_transform.transform_point(
        convex
            .local_bounds()
            .map_err(|_| CollisionError::GeometryBounds)?
            .ok_or(CollisionError::UnsupportedGeometryPair)?
            .center(),
    );
    if !plane_point.is_finite() || !center.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let center_side = center
        .checked_sub(plane_point)
        .and_then(|offset| offset.checked_dot(normal))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let side_normal = if two_sided && center_side < 0.0 {
        -normal
    } else {
        normal
    };
    let (convex_point, _) = support_world(convex, &convex_transform, -side_normal)?;
    let separation = convex_point
        .checked_sub(plane_point)
        .and_then(|offset| offset.checked_dot(side_normal))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let projected = convex_point
        .checked_sub(
            side_normal
                .checked_mul_scalar(separation)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    Ok(PlaneGap {
        separation,
        center_side,
        normal: side_normal,
        convex_point,
        plane_point: projected,
    })
}

fn distance_hit(
    toi: f64,
    distance: GjkDistance,
    transform_a: Transform,
    transform_b: Transform,
) -> CcdHit {
    let center_direction = transform_b
        .translation()
        .checked_sub(transform_a.translation())
        .and_then(Vec3::try_normalize);
    CcdHit {
        toi,
        normal: center_direction.unwrap_or(distance.normal),
        point_a: distance.point_a,
        point_b: distance.point_b,
    }
}

fn intersection_hit(
    toi: f64,
    shape_a: &Shape,
    transform_a: Transform,
    shape_b: &Shape,
    transform_b: Transform,
) -> Result<CcdHit, CollisionError> {
    let direction = transform_b
        .translation()
        .checked_sub(transform_a.translation())
        .and_then(Vec3::try_normalize)
        .unwrap_or(Vec3::X);
    let support = support_pair(shape_a, &transform_a, shape_b, &transform_b, direction)?;
    Ok(CcdHit {
        toi,
        normal: direction,
        point_a: support.point_a,
        point_b: support.point_b,
    })
}

fn shape_radius(shape: &Shape) -> Result<f64, CollisionError> {
    if let Shape::Sphere(sphere) = shape {
        return Ok(sphere.radius());
    }
    let bounds = shape
        .local_bounds()
        .map_err(|_| CollisionError::GeometryBounds)?
        .ok_or(CollisionError::UnsupportedGeometryPair)?;
    bounds_radius(bounds)
}

fn rotation_radius(shape: &Shape) -> Result<f64, CollisionError> {
    if matches!(shape, Shape::Sphere(_)) {
        Ok(0.0)
    } else {
        shape_radius(shape)
    }
}

fn bounds_radius(bounds: Aabb) -> Result<f64, CollisionError> {
    bounds
        .min()
        .abs()
        .max(bounds.max().abs())
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn translation_delta(start: Transform, end: Transform) -> Result<Vec3, CollisionError> {
    end.translation()
        .checked_sub(start.translation())
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn rotation_angle(start: Quat, end: Quat) -> Result<f64, CollisionError> {
    let dot = start.x * end.x + start.y * end.y + start.z * end.z + start.w * end.w;
    if !dot.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(2.0 * dot.abs().clamp(0.0, 1.0).acos())
}

pub(crate) fn interpolate(
    start: Transform,
    end: Transform,
    amount: f64,
) -> Result<Transform, CollisionError> {
    let translation = start
        .translation()
        .checked_add(
            translation_delta(start, end)?
                .checked_mul_scalar(amount)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let mut end_rotation = end.rotation();
    let start_rotation = start.rotation();
    let dot = start_rotation.x * end_rotation.x
        + start_rotation.y * end_rotation.y
        + start_rotation.z * end_rotation.z
        + start_rotation.w * end_rotation.w;
    if !dot.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if dot < 0.0 {
        end_rotation = Quat::new(
            -end_rotation.x,
            -end_rotation.y,
            -end_rotation.z,
            -end_rotation.w,
        );
    }
    let dot = dot.abs().clamp(-1.0, 1.0);
    let rotation = if dot > 0.9995 {
        Quat::new(
            start_rotation.x + (end_rotation.x - start_rotation.x) * amount,
            start_rotation.y + (end_rotation.y - start_rotation.y) * amount,
            start_rotation.z + (end_rotation.z - start_rotation.z) * amount,
            start_rotation.w + (end_rotation.w - start_rotation.w) * amount,
        )
        .try_normalize()
    } else {
        let angle = dot.acos();
        let inverse_sine = 1.0 / angle.sin();
        let start_weight = ((1.0 - amount) * angle).sin() * inverse_sine;
        let end_weight = (amount * angle).sin() * inverse_sine;
        Quat::new(
            start_rotation.x * start_weight + end_rotation.x * end_weight,
            start_rotation.y * start_weight + end_rotation.y * end_weight,
            start_rotation.z * start_weight + end_rotation.z * end_weight,
            start_rotation.w * start_weight + end_rotation.w * end_weight,
        )
        .try_normalize()
    }
    .filter(|rotation| rotation.is_normalized())
    .ok_or(CollisionError::ArithmeticOverflow)?;
    Transform::new(translation, rotation).map_err(|_| CollisionError::ArithmeticOverflow)
}

fn tolerance(value: f64) -> f64 {
    value.abs().max(1.0) * f64::EPSILON * 128.0
}

#[cfg(test)]
#[path = "ccd_tests.rs"]
mod tests;
