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

/// Finds the first contact between two shapes following their start-to-end poses.
pub(crate) fn convex_toi(
    shape_a: &Shape,
    start_a: Transform,
    end_a: Transform,
    shape_b: &Shape,
    start_b: Transform,
    end_b: Transform,
) -> Result<Option<CcdHit>, CollisionError> {
    if !shape_a.is_convex() || !(shape_b.is_convex() || matches!(shape_b, Shape::Triangle(_))) {
        return Err(CollisionError::UnsupportedGeometryPair);
    }
    let radius_a = shape_radius(shape_a)?;
    let radius_b = shape_radius(shape_b)?;
    let motion_bound = translation_delta(start_a, end_a)?.length()
        + translation_delta(start_b, end_b)?.length()
        + rotation_angle(start_a.rotation(), end_a.rotation())? * radius_a
        + rotation_angle(start_b.rotation(), end_b.rotation())? * radius_b;
    if !motion_bound.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let mut toi = 0.0;
    for _ in 0..MAX_CCD_ITERATIONS {
        let transform_a = interpolate(start_a, end_a, toi)?;
        let transform_b = interpolate(start_b, end_b, toi)?;
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
    if !convex.is_convex() {
        return Err(CollisionError::UnsupportedGeometryPair);
    }
    let radius = shape_radius(convex)?;
    let convex_translation = translation_delta(start_convex, end_convex)?;
    let plane_translation = translation_delta(start_plane, end_plane)?;
    let relative_center = start_convex
        .translation()
        .checked_sub(start_plane.translation())
        .and_then(Vec3::checked_length)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let plane_pivot_radius = relative_center
        + radius
        + plane.distance().abs()
        + convex_translation.length()
        + plane_translation.length();
    let motion_bound = convex_translation.length()
        + plane_translation.length()
        + rotation_angle(start_convex.rotation(), end_convex.rotation())? * radius
        + rotation_angle(start_plane.rotation(), end_plane.rotation())? * plane_pivot_radius;
    if !motion_bound.is_finite() || !plane_pivot_radius.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let initial = plane_gap(convex, start_convex, plane, start_plane, two_sided)?;
    if !two_sided && initial.center_side < 0.0 {
        return Ok(None);
    }
    let mut toi = 0.0;
    for _ in 0..MAX_CCD_ITERATIONS {
        let convex_transform = interpolate(start_convex, end_convex, toi)?;
        let plane_transform = interpolate(start_plane, end_plane, toi)?;
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
    let vertices = deformed_vertices.unwrap_or_else(|| mesh.vertices());
    if vertices.len() != mesh.vertices().len() || bvh.triangle_count() != mesh.triangles().len() {
        return Err(CollisionError::GeometryBounds);
    }
    let query_bounds =
        relative_swept_bounds(convex, start_convex, end_convex, start_mesh, end_mesh)?;
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
        consider_triangle_toi(
            &mut best,
            convex,
            start_convex,
            end_convex,
            triangle,
            start_mesh,
            end_mesh,
        )?;
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
    let query_bounds =
        relative_swept_bounds(convex, start_convex, end_convex, start_field, end_field)?;
    height_field::query_triangles(field, query_bounds, candidates)?;
    let mut best = None;
    for candidate in candidates.iter().copied() {
        let triangle = Triangle::from_vertices(candidate.vertices)
            .map_err(|_| CollisionError::InvalidContact)?;
        consider_triangle_toi(
            &mut best,
            convex,
            start_convex,
            end_convex,
            triangle,
            start_field,
            end_field,
        )?;
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
    let query_bounds =
        relative_swept_bounds(convex, start_convex, end_convex, start_field, end_field)?;
    level_set::query_surface_cells(field, query_bounds, candidates)?;
    if candidates.is_empty() {
        return Ok(None);
    }
    let convex_radius = shape_radius(convex)?;
    let field_bounds = field
        .local_bounds()
        .map_err(|_| CollisionError::GeometryBounds)?;
    let field_radius = bounds_radius(field_bounds)?;
    let motion_bound = translation_delta(start_convex, end_convex)?.length()
        + translation_delta(start_field, end_field)?.length()
        + rotation_angle(start_convex.rotation(), end_convex.rotation())? * convex_radius
        + rotation_angle(start_field.rotation(), end_field.rotation())? * field_radius;
    if !motion_bound.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let mut toi = 0.0;
    for _ in 0..MAX_CCD_ITERATIONS {
        let convex_transform = interpolate(start_convex, end_convex, toi)?;
        let field_transform = interpolate(start_field, end_field, toi)?;
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
    start_convex: Transform,
    end_convex: Transform,
    triangle: Triangle,
    start_triangle: Transform,
    end_triangle: Transform,
) -> Result<(), CollisionError> {
    let triangle = Shape::Triangle(triangle);
    if let Some(hit) = convex_toi(
        convex,
        start_convex,
        end_convex,
        &triangle,
        start_triangle,
        end_triangle,
    )? && best.is_none_or(|current| hit.toi < current.toi)
    {
        *best = Some(hit);
    }
    Ok(())
}

fn relative_swept_bounds(
    convex: &Shape,
    start_convex: Transform,
    end_convex: Transform,
    start_target: Transform,
    end_target: Transform,
) -> Result<Aabb, CollisionError> {
    let inverse = start_target
        .inverse()
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    let local_center = inverse.transform_point(start_convex.translation());
    let relative_length = local_center
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let convex_delta = translation_delta(start_convex, end_convex)?.length();
    let target_delta = translation_delta(start_target, end_target)?.length();
    let radius = shape_radius(convex)?;
    let target_rotation = rotation_angle(start_target.rotation(), end_target.rotation())?;
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
    shape: &Shape,
    start: Transform,
    end: Transform,
) -> Result<Option<Aabb>, CollisionError> {
    let Some(bounds) = shape
        .local_bounds()
        .map_err(|_| CollisionError::GeometryBounds)?
    else {
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
