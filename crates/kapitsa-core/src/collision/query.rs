use crate::{
    aabb::Aabb,
    collider::{CollisionFilter, CollisionMode},
    shape::{LevelSet, Plane, Shape, Triangle, TriangleMesh},
    transform::Transform,
    world::{BodyId, ColliderId},
};
use kapitsa_math::Vec3;

use super::{
    CollisionError, QueryError,
    ccd::{self, CcdHit},
    gjk,
    height_field::{self, HeightFieldTriangle},
    level_set,
    mesh::{MeshQueryWorkspace, TriangleBvh},
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
    pub max_distance: f64,
}

impl Ray {
    pub fn new(origin: Vec3, direction: Vec3, max_distance: f64) -> Result<Self, QueryError> {
        if !origin.is_finite() || !max_distance.is_finite() || max_distance < 0.0 {
            return Err(QueryError::InvalidInput);
        }
        let direction = direction.try_normalize().ok_or(QueryError::InvalidInput)?;
        Ok(Self {
            origin,
            direction,
            max_distance,
        })
    }

    pub(crate) fn validate(self) -> Result<(), QueryError> {
        if !self.origin.is_finite()
            || !self.direction.is_finite()
            || (self.direction.length() - 1.0).abs() > 1.0e-12
            || !self.max_distance.is_finite()
            || self.max_distance < 0.0
        {
            return Err(QueryError::InvalidInput);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QueryFilter {
    pub layers: u32,
    pub mask: u32,
    pub exclude_body: Option<BodyId>,
    pub include_sensors: bool,
}

impl QueryFilter {
    pub(crate) fn allows(
        self,
        body: BodyId,
        collider: CollisionFilter,
        mode: CollisionMode,
    ) -> bool {
        (self.include_sensors || mode != CollisionMode::Sensor)
            && self.exclude_body != Some(body)
            && self.layers & collider.mask() != 0
            && collider.layers() & self.mask != 0
    }
}

impl Default for QueryFilter {
    fn default() -> Self {
        Self {
            layers: 1,
            mask: u32::MAX,
            exclude_body: None,
            include_sensors: false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    pub body: BodyId,
    pub collider: ColliderId,
    pub position: Vec3,
    pub normal: Vec3,
    pub distance: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShapeHit {
    pub body: BodyId,
    pub collider: ColliderId,
    pub position: Vec3,
    pub normal: Vec3,
    pub toi: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OverlapHit {
    pub body: BodyId,
    pub collider: ColliderId,
}

pub(crate) fn raycast_shape(
    shape: &Shape,
    transform: Transform,
    ray: Ray,
    bvh: Option<&TriangleBvh>,
    deformed_vertices: Option<&[Vec3]>,
) -> Result<Option<(Vec3, Vec3, f64)>, QueryError> {
    ray.validate()?;
    if shape.is_convex() {
        return raycast_convex(shape, transform, ray);
    }
    let inverse = transform
        .inverse()
        .map_err(|_| QueryError::ArithmeticOverflow)?;
    let local_origin = inverse.transform_point(ray.origin);
    let local_direction = inverse.transform_vector(ray.direction);
    if !local_origin.is_finite() || !local_direction.is_finite() {
        return Err(QueryError::ArithmeticOverflow);
    }
    let (local_normal, distance) = match shape {
        Shape::Plane(plane) => ray_plane(plane, local_origin, local_direction, ray.max_distance)?,
        Shape::Triangle(triangle) => ray_triangle(
            local_origin,
            local_direction,
            ray.max_distance,
            triangle.vertices(),
        )?,
        Shape::TriangleMesh(mesh) => ray_mesh(
            mesh,
            deformed_vertices.unwrap_or_else(|| mesh.vertices()),
            bvh.ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            local_origin,
            local_direction,
            ray.max_distance,
        )?,
        Shape::HeightField(field) => {
            ray_height_field(field, local_origin, local_direction, ray.max_distance)?
        }
        Shape::LevelSet(field) => {
            ray_level_set(field, local_origin, local_direction, ray.max_distance)?
        }
        Shape::SkinnedTriangleMesh(skinned) => ray_mesh(
            skinned.mesh(),
            deformed_vertices.unwrap_or_else(|| skinned.mesh().vertices()),
            bvh.ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            local_origin,
            local_direction,
            ray.max_distance,
        )?,
        Shape::Sphere(_)
        | Shape::Cuboid(_)
        | Shape::Capsule(_)
        | Shape::Cylinder(_)
        | Shape::TaperedCapsule(_)
        | Shape::TaperedCylinder(_)
        | Shape::Convex(_) => unreachable!("convex shapes use GJK ray casting"),
    };
    let Some(local_normal) = local_normal else {
        return Ok(None);
    };
    let normal = transform
        .transform_vector(local_normal)
        .try_normalize()
        .ok_or(QueryError::ArithmeticOverflow)?;
    let position = ray.origin + ray.direction * distance;
    if !position.is_finite() {
        return Err(QueryError::ArithmeticOverflow);
    }
    Ok(Some((position, normal, distance)))
}

pub(crate) fn sweep_shape(
    query_shape: &Shape,
    query_transform: Transform,
    displacement: Vec3,
    target_shape: &Shape,
    target_transform: Transform,
    bvh: Option<&TriangleBvh>,
    deformed_vertices: Option<&[Vec3]>,
) -> Result<Option<CcdHit>, QueryError> {
    if !query_shape.is_convex() {
        return Err(QueryError::UnsupportedQueryShape);
    }
    if !displacement.is_finite() {
        return Err(QueryError::InvalidInput);
    }
    let end_translation = query_transform
        .translation()
        .checked_add(displacement)
        .ok_or(QueryError::ArithmeticOverflow)?;
    let end_transform = Transform::new(end_translation, query_transform.rotation())
        .map_err(|_| QueryError::ArithmeticOverflow)?;
    let hit = match target_shape {
        Shape::Plane(plane) => ccd::plane_toi(
            query_shape,
            query_transform,
            end_transform,
            plane,
            target_transform,
            target_transform,
            true,
            false,
        ),
        Shape::Triangle(_) => ccd::convex_toi(
            query_shape,
            query_transform,
            end_transform,
            target_shape,
            target_transform,
            target_transform,
        ),
        Shape::TriangleMesh(mesh) => ccd::triangle_mesh_toi(
            query_shape,
            query_transform,
            end_transform,
            mesh,
            deformed_vertices,
            bvh.ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            target_transform,
            target_transform,
            &mut MeshQueryWorkspace::default(),
            &mut Vec::new(),
        ),
        Shape::HeightField(field) => ccd::height_field_toi(
            query_shape,
            query_transform,
            end_transform,
            field,
            target_transform,
            target_transform,
            &mut Vec::new(),
        ),
        Shape::LevelSet(field) => ccd::level_set_toi(
            query_shape,
            query_transform,
            end_transform,
            field,
            target_transform,
            target_transform,
            &mut Vec::new(),
        ),
        Shape::SkinnedTriangleMesh(skinned) => ccd::triangle_mesh_toi(
            query_shape,
            query_transform,
            end_transform,
            skinned.mesh(),
            deformed_vertices,
            bvh.ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            target_transform,
            target_transform,
            &mut MeshQueryWorkspace::default(),
            &mut Vec::new(),
        ),
        target if target.is_convex() => ccd::convex_toi(
            query_shape,
            query_transform,
            end_transform,
            target,
            target_transform,
            target_transform,
        ),
        _ => return Err(QueryError::UnsupportedQueryShape),
    }
    .map_err(query_collision)?;
    Ok(hit)
}

pub(crate) fn overlaps_shape(
    query_shape: &Shape,
    query_transform: Transform,
    target_shape: &Shape,
    target_transform: Transform,
    bvh: Option<&TriangleBvh>,
    deformed_vertices: Option<&[Vec3]>,
) -> Result<bool, QueryError> {
    if !query_shape.is_convex() {
        return Err(QueryError::UnsupportedQueryShape);
    }
    match target_shape {
        Shape::Plane(plane) => {
            overlaps_plane(query_shape, query_transform, plane, target_transform)
        }
        Shape::Triangle(_) => {
            overlaps_convex(query_shape, query_transform, target_shape, target_transform)
        }
        Shape::TriangleMesh(mesh) => overlaps_mesh(
            query_shape,
            query_transform,
            mesh,
            deformed_vertices.unwrap_or_else(|| mesh.vertices()),
            bvh.ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            target_transform,
        ),
        Shape::HeightField(field) => {
            overlaps_height_field(query_shape, query_transform, field, target_transform)
        }
        Shape::LevelSet(field) => {
            overlaps_level_set(query_shape, query_transform, field, target_transform)
        }
        Shape::SkinnedTriangleMesh(skinned) => overlaps_mesh(
            query_shape,
            query_transform,
            skinned.mesh(),
            deformed_vertices.unwrap_or_else(|| skinned.mesh().vertices()),
            bvh.ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            target_transform,
        ),
        target if target.is_convex() => {
            overlaps_convex(query_shape, query_transform, target, target_transform)
        }
        _ => Err(QueryError::UnsupportedQueryShape),
    }
}

fn overlaps_convex(
    query_shape: &Shape,
    query_transform: Transform,
    target_shape: &Shape,
    target_transform: Transform,
) -> Result<bool, QueryError> {
    gjk::intersect(
        query_shape,
        &query_transform,
        target_shape,
        &target_transform,
    )
    .map(|result| result.intersects)
    .map_err(query_collision)
}

fn overlaps_plane(
    query_shape: &Shape,
    query_transform: Transform,
    plane: &Plane,
    plane_transform: Transform,
) -> Result<bool, QueryError> {
    let normal = plane_transform
        .transform_vector(plane.normal())
        .try_normalize()
        .ok_or(QueryError::ArithmeticOverflow)?;
    let point = plane_transform.transform_point(plane.normal() * plane.distance());
    let minimum = super::support::support_world(query_shape, &query_transform, -normal)
        .map_err(query_collision)?
        .0;
    let maximum = super::support::support_world(query_shape, &query_transform, normal)
        .map_err(query_collision)?
        .0;
    let minimum = (minimum - point)
        .checked_dot(normal)
        .ok_or(QueryError::ArithmeticOverflow)?;
    let maximum = (maximum - point)
        .checked_dot(normal)
        .ok_or(QueryError::ArithmeticOverflow)?;
    Ok(minimum <= 0.0 && maximum >= 0.0)
}

fn overlaps_mesh(
    query_shape: &Shape,
    query_transform: Transform,
    mesh: &TriangleMesh,
    vertices: &[Vec3],
    bvh: &TriangleBvh,
    target_transform: Transform,
) -> Result<bool, QueryError> {
    if vertices.len() != mesh.vertices().len() || bvh.triangle_count() != mesh.triangles().len() {
        return Err(QueryError::Geometry(CollisionError::InvalidSkinPose));
    }
    let world_bounds = query_world_bounds(query_shape, query_transform)?;
    let local_bounds = super::mesh::local_query_bounds(world_bounds, &target_transform)
        .map_err(query_collision)?;
    let mut workspace = MeshQueryWorkspace::default();
    let mut candidates = Vec::new();
    bvh.query(local_bounds, &mut workspace, &mut candidates)
        .map_err(query_collision)?;
    for triangle_id in candidates {
        let [a, b, c] = *mesh
            .triangles()
            .get(triangle_id)
            .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?;
        let triangle = Shape::Triangle(
            Triangle::from_vertices([
                *vertices
                    .get(a)
                    .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
                *vertices
                    .get(b)
                    .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
                *vertices
                    .get(c)
                    .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            ])
            .map_err(|_| QueryError::Geometry(CollisionError::InvalidContact))?,
        );
        if overlaps_convex(query_shape, query_transform, &triangle, target_transform)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn overlaps_height_field(
    query_shape: &Shape,
    query_transform: Transform,
    field: &crate::shape::HeightField,
    target_transform: Transform,
) -> Result<bool, QueryError> {
    let world_bounds = query_world_bounds(query_shape, query_transform)?;
    let mut candidates: Vec<HeightFieldTriangle> = Vec::new();
    height_field::query_world_bounds(field, &target_transform, world_bounds, &mut candidates)
        .map_err(query_collision)?;
    for candidate in candidates {
        let triangle = Shape::Triangle(
            Triangle::from_vertices(candidate.vertices)
                .map_err(|_| QueryError::Geometry(CollisionError::InvalidContact))?,
        );
        if overlaps_convex(query_shape, query_transform, &triangle, target_transform)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn overlaps_level_set(
    query_shape: &Shape,
    query_transform: Transform,
    field: &LevelSet,
    target_transform: Transform,
) -> Result<bool, QueryError> {
    let world_bounds = query_world_bounds(query_shape, query_transform)?;
    let local_bounds = super::mesh::local_query_bounds(world_bounds, &target_transform)
        .map_err(query_collision)?;
    let mut candidates = Vec::new();
    level_set::query_surface_cells(field, local_bounds, &mut candidates)
        .map_err(query_collision)?;
    if candidates.is_empty() {
        return Ok(false);
    }

    let inverse = target_transform
        .inverse()
        .map_err(|_| QueryError::ArithmeticOverflow)?;
    let shape_center = query_transform.transform_point(
        query_shape
            .local_bounds()
            .map_err(|_| QueryError::Geometry(CollisionError::GeometryBounds))?
            .ok_or(QueryError::UnsupportedQueryShape)?
            .center(),
    );
    let mut signs = [false; 2];
    for direction in support_directions() {
        let point = super::support::support_world(query_shape, &query_transform, direction)
            .map_err(query_collision)?
            .0;
        if let Some(sample) =
            level_set::sample(field, inverse.transform_point(point)).map_err(query_collision)?
        {
            signs[usize::from(sample.value >= 0.0)] = true;
            if sample.value == 0.0 {
                return Ok(true);
            }
        }
    }
    if signs[0] && signs[1] {
        return Ok(true);
    }

    for cell in candidates {
        let seed = inverse
            .transform_point(shape_center)
            .max(cell.bounds.min())
            .min(cell.bounds.max());
        if let Some(surface) = project_level_set_point(field, cell.bounds, seed)?
            && gjk::distance_to_point(
                query_shape,
                &query_transform,
                target_transform.transform_point(surface),
            )
            .map_err(query_collision)?
            .intersects
        {
            return Ok(true);
        }
        if cell_edge_root_inside(query_shape, query_transform, field, target_transform, cell)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn cell_edge_root_inside(
    query_shape: &Shape,
    query_transform: Transform,
    field: &LevelSet,
    field_transform: Transform,
    cell: level_set::LevelSetCell,
) -> Result<bool, QueryError> {
    let min = cell.bounds.min();
    let max = cell.bounds.max();
    let corners = [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, max.y, max.z),
    ];
    for (left, right) in [
        (0, 1),
        (0, 2),
        (1, 3),
        (2, 3),
        (4, 5),
        (4, 6),
        (5, 7),
        (6, 7),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ] {
        let first = level_set::sample(field, corners[left])
            .map_err(query_collision)?
            .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?;
        let second = level_set::sample(field, corners[right])
            .map_err(query_collision)?
            .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?;
        if first.value == 0.0
            && level_set_point_inside(
                query_shape,
                query_transform,
                field_transform.transform_point(corners[left]),
            )?
        {
            return Ok(true);
        }
        if second.value == 0.0
            && level_set_point_inside(
                query_shape,
                query_transform,
                field_transform.transform_point(corners[right]),
            )?
        {
            return Ok(true);
        }
        if first.value.is_sign_positive() == second.value.is_sign_positive() {
            continue;
        }
        let fraction = first.value.abs() / (first.value.abs() + second.value.abs());
        let root = corners[left] + (corners[right] - corners[left]) * fraction;
        if level_set_point_inside(
            query_shape,
            query_transform,
            field_transform.transform_point(root),
        )? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn level_set_point_inside(
    shape: &Shape,
    transform: Transform,
    point: Vec3,
) -> Result<bool, QueryError> {
    gjk::distance_to_point(shape, &transform, point)
        .map(|distance| distance.intersects)
        .map_err(query_collision)
}

fn project_level_set_point(
    field: &LevelSet,
    bounds: Aabb,
    mut point: Vec3,
) -> Result<Option<Vec3>, QueryError> {
    for _ in 0..24 {
        let sample = level_set::sample(field, point)
            .map_err(query_collision)?
            .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?;
        let tolerance = point.abs().max_element().max(1.0) * f64::EPSILON * 128.0;
        if sample.value.abs() <= tolerance {
            return Ok(Some(point));
        }
        let gradient_squared = sample
            .gradient
            .checked_dot(sample.gradient)
            .ok_or(QueryError::ArithmeticOverflow)?;
        if gradient_squared <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        point = point
            .checked_sub(
                sample
                    .gradient
                    .checked_mul_scalar(sample.value / gradient_squared)
                    .ok_or(QueryError::ArithmeticOverflow)?,
            )
            .ok_or(QueryError::ArithmeticOverflow)?
            .max(bounds.min())
            .min(bounds.max());
    }
    Ok(None)
}

fn support_directions() -> [Vec3; 26] {
    let mut directions = [Vec3::ZERO; 26];
    let mut index = 0;
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                if x == 0 && y == 0 && z == 0 {
                    continue;
                }
                directions[index] = Vec3::new(x as f64, y as f64, z as f64);
                index += 1;
            }
        }
    }
    directions
}

fn query_world_bounds(shape: &Shape, transform: Transform) -> Result<Aabb, QueryError> {
    shape
        .world_bounds(&transform)
        .map_err(|_| QueryError::Geometry(CollisionError::GeometryBounds))?
        .ok_or(QueryError::UnsupportedQueryShape)
}

fn raycast_convex(
    shape: &Shape,
    transform: Transform,
    ray: Ray,
) -> Result<Option<(Vec3, Vec3, f64)>, QueryError> {
    let mut distance = 0.0;
    let mut previous_normal = None;
    for _ in 0..128 {
        let point = ray.origin + ray.direction * distance;
        let result = gjk::distance_to_point(shape, &transform, point).map_err(query_collision)?;
        if result.intersects {
            let normal = previous_normal.unwrap_or(-ray.direction);
            let position = if distance == 0.0 {
                ray.origin
            } else {
                result.point_a
            };
            return Ok(Some((position, normal, distance)));
        }
        let closing_speed = -ray
            .direction
            .checked_dot(result.normal)
            .ok_or(QueryError::ArithmeticOverflow)?;
        if closing_speed <= f64::EPSILON {
            return Ok(None);
        }
        let advance = result.lower_bound / closing_speed;
        if !advance.is_finite() {
            return Err(QueryError::ArithmeticOverflow);
        }
        if advance <= 0.0 {
            return Err(QueryError::NonConvergent);
        }
        previous_normal = Some(result.normal);
        distance += advance;
        if distance > ray.max_distance {
            return Ok(None);
        }
    }
    Err(QueryError::NonConvergent)
}

fn ray_plane(
    plane: &Plane,
    origin: Vec3,
    direction: Vec3,
    max_distance: f64,
) -> Result<(Option<Vec3>, f64), QueryError> {
    let normal = plane.normal();
    let denominator = direction
        .checked_dot(normal)
        .ok_or(QueryError::ArithmeticOverflow)?;
    if denominator.abs() <= f64::EPSILON * 32.0 {
        return Ok((None, 0.0));
    }
    let distance = (plane.distance()
        - origin
            .checked_dot(normal)
            .ok_or(QueryError::ArithmeticOverflow)?)
        / denominator;
    if !distance.is_finite() {
        return Err(QueryError::ArithmeticOverflow);
    }
    if distance < 0.0 || distance > max_distance {
        return Ok((None, 0.0));
    }
    Ok((
        Some(if denominator > 0.0 { -normal } else { normal }),
        distance,
    ))
}

fn ray_triangle(
    origin: Vec3,
    direction: Vec3,
    max_distance: f64,
    vertices: [Vec3; 3],
) -> Result<(Option<Vec3>, f64), QueryError> {
    let edge_a = vertices[1] - vertices[0];
    let edge_b = vertices[2] - vertices[0];
    let p = direction.cross(edge_b);
    let determinant = edge_a
        .checked_dot(p)
        .ok_or(QueryError::ArithmeticOverflow)?;
    let scale = edge_a
        .abs()
        .max_element()
        .max(edge_b.abs().max_element())
        .max(1.0);
    if determinant.abs() <= scale * f64::EPSILON * 32.0 {
        return Ok((None, 0.0));
    }
    let inverse = 1.0 / determinant;
    let offset = origin - vertices[0];
    let u = offset.dot(p) * inverse;
    if !(0.0..=1.0).contains(&u) {
        return Ok((None, 0.0));
    }
    let q = offset.cross(edge_a);
    let v = direction.dot(q) * inverse;
    if v < 0.0 || u + v > 1.0 {
        return Ok((None, 0.0));
    }
    let distance = edge_b.dot(q) * inverse;
    if distance < 0.0 || distance > max_distance {
        return Ok((None, 0.0));
    }
    let normal = edge_a
        .cross(edge_b)
        .try_normalize()
        .ok_or(QueryError::Geometry(CollisionError::InvalidContact))?;
    Ok((
        Some(if direction.dot(normal) > 0.0 {
            -normal
        } else {
            normal
        }),
        distance,
    ))
}

fn ray_mesh(
    mesh: &TriangleMesh,
    vertices: &[Vec3],
    bvh: &TriangleBvh,
    origin: Vec3,
    direction: Vec3,
    max_distance: f64,
) -> Result<(Option<Vec3>, f64), QueryError> {
    if vertices.len() != mesh.vertices().len() {
        return Err(QueryError::Geometry(CollisionError::InvalidSkinPose));
    }
    let end = origin + direction * max_distance;
    if !end.is_finite() {
        return Err(QueryError::ArithmeticOverflow);
    }
    let segment_bounds = Aabb::new(origin.min(end), origin.max(end))
        .map_err(|_| QueryError::Geometry(CollisionError::GeometryBounds))?;
    let mut workspace = MeshQueryWorkspace::default();
    let mut candidates = Vec::new();
    bvh.query(segment_bounds, &mut workspace, &mut candidates)
        .map_err(query_collision)?;
    let mut best = None;
    for triangle in candidates {
        let [a, b, c] = *mesh
            .triangles()
            .get(triangle)
            .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?;
        let vertices = [
            *vertices
                .get(a)
                .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            *vertices
                .get(b)
                .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
            *vertices
                .get(c)
                .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?,
        ];
        let (normal, distance) = ray_triangle(origin, direction, max_distance, vertices)?;
        if let Some(normal) = normal
            && best.is_none_or(|(_, best_distance)| distance < best_distance)
        {
            best = Some((normal, distance));
        }
    }
    Ok(best.map_or((None, 0.0), |(normal, distance)| (Some(normal), distance)))
}

fn ray_height_field(
    field: &crate::shape::HeightField,
    origin: Vec3,
    direction: Vec3,
    max_distance: f64,
) -> Result<(Option<Vec3>, f64), QueryError> {
    let end = origin + direction * max_distance;
    if !end.is_finite() {
        return Err(QueryError::ArithmeticOverflow);
    }
    let bounds = Aabb::new(origin.min(end), origin.max(end))
        .map_err(|_| QueryError::Geometry(CollisionError::GeometryBounds))?;
    let mut candidates: Vec<HeightFieldTriangle> = Vec::new();
    height_field::query_triangles(field, bounds, &mut candidates).map_err(query_collision)?;
    let mut best = None;
    for triangle in candidates {
        let (normal, distance) = ray_triangle(origin, direction, max_distance, triangle.vertices)?;
        if let Some(normal) = normal
            && best.is_none_or(|(_, best_distance)| distance < best_distance)
        {
            best = Some((normal, distance));
        }
    }
    Ok(best.map_or((None, 0.0), |(normal, distance)| (Some(normal), distance)))
}

fn ray_level_set(
    field: &LevelSet,
    origin: Vec3,
    direction: Vec3,
    max_distance: f64,
) -> Result<(Option<Vec3>, f64), QueryError> {
    let bounds = field
        .local_bounds()
        .map_err(|_| QueryError::Geometry(CollisionError::GeometryBounds))?;
    let Some((mut current, exit)) = ray_aabb_interval(origin, direction, max_distance, bounds)?
    else {
        return Ok((None, 0.0));
    };
    let dimensions = field.dimensions();
    let max_cells = dimensions[0]
        .checked_add(dimensions[1])
        .and_then(|count| count.checked_add(dimensions[2]))
        .and_then(|count| count.checked_add(4))
        .ok_or(QueryError::ArithmeticOverflow)?;
    for _ in 0..max_cells {
        if current > exit {
            return Ok((None, 0.0));
        }
        let point = origin + direction * current;
        let indices = ray_cell_indices(field, point, direction)?;
        let next = next_cell_boundary(field, current, point, direction, indices, exit)?;
        if next <= current {
            let sample = level_set::sample(field, point)
                .map_err(query_collision)?
                .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?;
            if sample.value == 0.0 {
                let normal = sample
                    .gradient
                    .try_normalize()
                    .ok_or(QueryError::NonConvergent)?;
                return Ok((Some(normal), current));
            }
            return Ok((None, 0.0));
        }
        let end = next.min(exit);
        let values = [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0].map(|fraction| {
            let sample_point = origin + direction * (current + (end - current) * fraction);
            level_set::sample(field, sample_point)
                .map_err(query_collision)?
                .map(|sample| sample.value)
                .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))
        });
        let [y0, y1, y2, y3] = [values[0]?, values[1]?, values[2]?, values[3]?];
        if let Some(fraction) = first_cubic_root(y0, y1, y2, y3)? {
            let distance = current + (end - current) * fraction;
            let hit_point = origin + direction * distance;
            let sample = level_set::sample(field, hit_point)
                .map_err(query_collision)?
                .ok_or(QueryError::Geometry(CollisionError::GeometryBounds))?;
            let normal = sample
                .gradient
                .try_normalize()
                .ok_or(QueryError::NonConvergent)?;
            return Ok((Some(normal), distance));
        }
        if end >= exit {
            return Ok((None, 0.0));
        }
        current = end;
    }
    Err(QueryError::NonConvergent)
}

fn ray_aabb_interval(
    origin: Vec3,
    direction: Vec3,
    max_distance: f64,
    bounds: Aabb,
) -> Result<Option<(f64, f64)>, QueryError> {
    let min = bounds.min();
    let max = bounds.max();
    let mut entry: f64 = 0.0;
    let mut exit = max_distance;
    for (origin, direction, min, max) in [
        (origin.x, direction.x, min.x, max.x),
        (origin.y, direction.y, min.y, max.y),
        (origin.z, direction.z, min.z, max.z),
    ] {
        if direction.abs() <= f64::MIN_POSITIVE {
            if origin < min || origin > max {
                return Ok(None);
            }
            continue;
        }
        let first = (min - origin) / direction;
        let second = (max - origin) / direction;
        entry = entry.max(first.min(second));
        exit = exit.min(first.max(second));
        if entry > exit {
            return Ok(None);
        }
    }
    if exit < 0.0 || entry > max_distance {
        return Ok(None);
    }
    Ok(Some((entry.max(0.0), exit)))
}

fn ray_cell_indices(
    field: &LevelSet,
    point: Vec3,
    direction: Vec3,
) -> Result<[usize; 3], QueryError> {
    let origin = field.origin();
    let spacing = field.grid_spacing();
    let dimensions = field.dimensions();
    let coordinates = [
        ((point.x - origin.x) / spacing.x, direction.x, dimensions[0]),
        ((point.y - origin.y) / spacing.y, direction.y, dimensions[1]),
        ((point.z - origin.z) / spacing.z, direction.z, dimensions[2]),
    ];
    let mut indices = [0; 3];
    for (axis, (coordinate, direction, dimension)) in coordinates.into_iter().enumerate() {
        if !coordinate.is_finite() {
            return Err(QueryError::ArithmeticOverflow);
        }
        let nearest = coordinate.round();
        let index = if direction < 0.0
            && (coordinate - nearest).abs() <= f64::EPSILON * coordinate.abs().max(1.0) * 16.0
            && nearest > 0.0
        {
            nearest as usize - 1
        } else {
            coordinate.floor() as usize
        }
        .min(dimension - 2);
        indices[axis] = index;
    }
    Ok(indices)
}

fn next_cell_boundary(
    field: &LevelSet,
    current: f64,
    point: Vec3,
    direction: Vec3,
    cell: [usize; 3],
    exit: f64,
) -> Result<f64, QueryError> {
    let origin = field.origin();
    let spacing = field.grid_spacing();
    let coordinates = [
        (point.x, direction.x, origin.x, spacing.x, cell[0]),
        (point.y, direction.y, origin.y, spacing.y, cell[1]),
        (point.z, direction.z, origin.z, spacing.z, cell[2]),
    ];
    let mut next = exit;
    for (point, direction, origin, spacing, index) in coordinates {
        if direction == 0.0 {
            continue;
        }
        let boundary_index = if direction > 0.0 { index + 1 } else { index };
        let boundary = origin + spacing * boundary_index as f64;
        let distance = (boundary - point) / direction;
        if !distance.is_finite() {
            return Err(QueryError::ArithmeticOverflow);
        }
        if distance > f64::EPSILON * point.abs().max(1.0) * 16.0 {
            next = next.min(current + distance);
        }
    }
    Ok(next)
}

fn first_cubic_root(y0: f64, y1: f64, y2: f64, y3: f64) -> Result<Option<f64>, QueryError> {
    let delta1 = y1 - y0;
    let delta2 = y2 - 2.0 * y1 + y0;
    let delta3 = y3 - 3.0 * y2 + 3.0 * y1 - y0;
    let coefficients = [
        4.5 * delta3,
        4.5 * (delta2 - delta3),
        3.0 * delta1 - 1.5 * delta2 + delta3,
        y0,
    ];
    if !coefficients.iter().all(|value| value.is_finite()) {
        return Err(QueryError::ArithmeticOverflow);
    }
    let [a, b, c, d] = coefficients;
    let mut points = [0.0, 1.0, 0.0, 0.0];
    let mut count = 2;
    let derivative_a = 3.0 * a;
    let derivative_b = 2.0 * b;
    if derivative_a.abs() <= f64::EPSILON * b.abs().max(1.0) {
        if derivative_b != 0.0 {
            let root = -c / derivative_b;
            if root > 0.0 && root < 1.0 {
                points[count] = root;
                count += 1;
            }
        }
    } else {
        let discriminant = derivative_b * derivative_b - 4.0 * derivative_a * c;
        if discriminant >= 0.0 {
            let root = discriminant.sqrt();
            for candidate in [
                (-derivative_b - root) / (2.0 * derivative_a),
                (-derivative_b + root) / (2.0 * derivative_a),
            ] {
                if candidate > 0.0 && candidate < 1.0 {
                    points[count] = candidate;
                    count += 1;
                }
            }
        }
    }
    points[..count].sort_by(f64::total_cmp);
    let evaluate = |u: f64| ((a * u + b) * u + c) * u + d;
    let scale = coefficients
        .iter()
        .map(|value| value.abs())
        .fold(1.0, f64::max);
    let tolerance = scale * f64::EPSILON * 64.0;
    for interval in points[..count].windows(2) {
        let mut low = interval[0];
        let mut high = interval[1];
        let mut low_value = evaluate(low);
        let high_value = evaluate(high);
        if low_value.abs() <= tolerance {
            return Ok(Some(low));
        }
        if high_value.abs() <= tolerance {
            return Ok(Some(high));
        }
        if low_value.is_sign_positive() == high_value.is_sign_positive() {
            continue;
        }
        for _ in 0..64 {
            let middle = low * 0.5 + high * 0.5;
            let value = evaluate(middle);
            if value.abs() <= tolerance {
                return Ok(Some(middle));
            }
            if value.is_sign_positive() == low_value.is_sign_positive() {
                low = middle;
                low_value = value;
            } else {
                high = middle;
            }
        }
        return Ok(Some(low * 0.5 + high * 0.5));
    }
    Ok(None)
}

fn query_collision(error: CollisionError) -> QueryError {
    match error {
        CollisionError::ArithmeticOverflow => QueryError::ArithmeticOverflow,
        CollisionError::NonConvergent => QueryError::NonConvergent,
        other => QueryError::Geometry(other),
    }
}

#[cfg(test)]
#[path = "query_tests.rs"]
mod tests;
