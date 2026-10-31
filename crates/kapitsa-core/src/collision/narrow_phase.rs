use std::cmp::Ordering;

use crate::{
    aabb::Aabb,
    collider::CollisionMode,
    shape::{HeightField, LevelSet, Plane, Shape, Sphere, Triangle, TriangleMesh},
    transform::Transform,
};
use kapitsa_math::Vec3;

use super::{
    CollisionError, CollisionPair,
    ccd::{CcdHit, CcdMotion},
    epa::EpaWorkspace,
    gjk,
    height_field::{self, HeightFieldTriangle},
    manifold::{ContactManifold, ContactPoint, combine_feature_ids},
    mesh::{self, MeshQueryWorkspace, TriangleBvh},
    support::{support_pair, support_world},
};

/// Reusable scratch for narrow-phase contact generation.
pub(crate) struct NarrowPhase {
    epa: EpaWorkspace,
    mesh_workspace: MeshQueryWorkspace,
    mesh_candidates: Vec<usize>,
    height_field_candidates: Vec<HeightFieldTriangle>,
    level_set_candidates: Vec<super::level_set::LevelSetCell>,
}

struct SurfacePair<'a> {
    surface: &'a Shape,
    convex: &'a Shape,
    surface_transform: &'a Transform,
    convex_transform: &'a Transform,
    surface_bvh: Option<&'a TriangleBvh>,
    surface_vertices: Option<&'a [Vec3]>,
    body_a: &'a Transform,
    body_b: &'a Transform,
    surface_is_a: bool,
}

impl<'a> SurfacePair<'a> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        shape_a: &'a Shape,
        collider_a: &'a Transform,
        bvh_a: Option<&'a TriangleBvh>,
        vertices_a: Option<&'a [Vec3]>,
        shape_b: &'a Shape,
        collider_b: &'a Transform,
        bvh_b: Option<&'a TriangleBvh>,
        vertices_b: Option<&'a [Vec3]>,
        body_a: &'a Transform,
        body_b: &'a Transform,
    ) -> Option<Self> {
        let a_is_surface = is_surface(shape_a);
        let b_is_surface = is_surface(shape_b);
        if a_is_surface && shape_b.is_convex() {
            Some(Self {
                surface: shape_a,
                convex: shape_b,
                surface_transform: collider_a,
                convex_transform: collider_b,
                surface_bvh: bvh_a,
                surface_vertices: vertices_a,
                body_a,
                body_b,
                surface_is_a: true,
            })
        } else if b_is_surface && shape_a.is_convex() {
            Some(Self {
                surface: shape_b,
                convex: shape_a,
                surface_transform: collider_b,
                convex_transform: collider_a,
                surface_bvh: bvh_b,
                surface_vertices: vertices_b,
                body_a,
                body_b,
                surface_is_a: false,
            })
        } else {
            None
        }
    }
}

fn is_surface(shape: &Shape) -> bool {
    matches!(
        shape,
        Shape::Plane(_)
            | Shape::Triangle(_)
            | Shape::TriangleMesh(_)
            | Shape::SkinnedTriangleMesh(_)
            | Shape::HeightField(_)
            | Shape::LevelSet(_)
    )
}

impl NarrowPhase {
    pub(crate) const fn new() -> Self {
        Self {
            epa: EpaWorkspace::new(),
            mesh_workspace: MeshQueryWorkspace::new(),
            mesh_candidates: Vec::new(),
            height_field_candidates: Vec::new(),
            level_set_candidates: Vec::new(),
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[cfg(test)]
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
        self.detect_with_geometry(
            pair,
            shape_a,
            collider_a,
            body_a,
            mode_a,
            None,
            None,
            shape_b,
            collider_b,
            body_b,
            mode_b,
            None,
            None,
            contact_offset,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn detect_with_geometry(
        &mut self,
        pair: CollisionPair,
        shape_a: &Shape,
        collider_a: &Transform,
        body_a: &Transform,
        mode_a: CollisionMode,
        bvh_a: Option<&TriangleBvh>,
        vertices_a: Option<&[Vec3]>,
        shape_b: &Shape,
        collider_b: &Transform,
        body_b: &Transform,
        mode_b: CollisionMode,
        bvh_b: Option<&TriangleBvh>,
        vertices_b: Option<&[Vec3]>,
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

        if let Some(surface_pair) = SurfacePair::new(
            shape_a, collider_a, bvh_a, vertices_a, shape_b, collider_b, bvh_b, vertices_b, body_a,
            body_b,
        ) {
            return self.detect_surface_pair(pair, surface_pair, pair_mode, contact_offset);
        }

        match (shape_a, shape_b) {
            (Shape::Sphere(sphere_a), Shape::Sphere(sphere_b)) => sphere_sphere(
                pair, sphere_a, collider_a, body_a, sphere_b, collider_b, body_b,
            ),
            (a, b) if a.is_convex() && b.is_convex() => {
                self.convex_convex(pair, a, collider_a, body_a, b, collider_b, body_b)
            }
            _ => Err(CollisionError::UnsupportedGeometryPair),
        }
    }

    fn detect_surface_pair(
        &mut self,
        pair: CollisionPair,
        routing: SurfacePair<'_>,
        mode: CollisionMode,
        contact_offset: f64,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        match routing.surface {
            Shape::Plane(plane) => self.plane_convex(
                pair,
                plane,
                routing.surface_transform,
                routing.convex,
                routing.convex_transform,
                routing.body_a,
                routing.body_b,
                mode,
                contact_offset,
                routing.surface_is_a,
            ),
            Shape::Triangle(triangle) => self.triangle_convex(
                pair,
                triangle,
                routing.surface_transform,
                routing.convex,
                routing.convex_transform,
                routing.body_a,
                routing.body_b,
                routing.surface_is_a,
                contact_offset,
            ),
            Shape::TriangleMesh(mesh) => self.mesh_convex(
                pair,
                mesh,
                routing.surface_vertices.unwrap_or_else(|| mesh.vertices()),
                routing.surface_bvh.ok_or(CollisionError::GeometryBounds)?,
                routing.surface_transform,
                routing.convex,
                routing.convex_transform,
                routing.body_a,
                routing.body_b,
                routing.surface_is_a,
                contact_offset,
            ),
            Shape::SkinnedTriangleMesh(mesh) => self.mesh_convex(
                pair,
                mesh.mesh(),
                routing
                    .surface_vertices
                    .unwrap_or_else(|| mesh.mesh().vertices()),
                routing.surface_bvh.ok_or(CollisionError::GeometryBounds)?,
                routing.surface_transform,
                routing.convex,
                routing.convex_transform,
                routing.body_a,
                routing.body_b,
                routing.surface_is_a,
                contact_offset,
            ),
            Shape::HeightField(field) => self.height_field_convex(
                pair,
                field,
                routing.surface_transform,
                routing.convex,
                routing.convex_transform,
                routing.body_a,
                routing.body_b,
                routing.surface_is_a,
                contact_offset,
            ),
            Shape::LevelSet(field) => self.level_set_convex(
                pair,
                field,
                routing.surface_transform,
                routing.convex,
                routing.convex_transform,
                routing.body_a,
                routing.body_b,
                routing.surface_is_a,
            ),
            _ => Err(CollisionError::UnsupportedGeometryPair),
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn time_of_impact(
        &mut self,
        shape_a: &Shape,
        motion_a: &CcdMotion<'_>,
        bvh_a: Option<&TriangleBvh>,
        vertices_a: Option<&[Vec3]>,
        shape_b: &Shape,
        motion_b: &CcdMotion<'_>,
        bvh_b: Option<&TriangleBvh>,
        vertices_b: Option<&[Vec3]>,
        sensor: bool,
    ) -> Result<Option<CcdHit>, CollisionError> {
        match (shape_a, shape_b) {
            (Shape::Plane(plane), convex) if convex.is_convex() => {
                super::ccd::plane_toi_motion(convex, motion_b, plane, motion_a, sensor, true)
            }
            (convex, Shape::Plane(plane)) if convex.is_convex() => {
                super::ccd::plane_toi_motion(convex, motion_a, plane, motion_b, sensor, false)
            }
            (Shape::Triangle(triangle), convex) if convex.is_convex() => {
                let target = Shape::Triangle(
                    Triangle::from_vertices(triangle.vertices())
                        .map_err(|_| CollisionError::InvalidContact)?,
                );
                super::ccd::convex_toi_motion(convex, motion_b, &target, motion_a)
                    .map(|hit| hit.map(reverse_hit))
            }
            (convex, Shape::Triangle(triangle)) if convex.is_convex() => {
                let target = Shape::Triangle(
                    Triangle::from_vertices(triangle.vertices())
                        .map_err(|_| CollisionError::InvalidContact)?,
                );
                super::ccd::convex_toi_motion(convex, motion_a, &target, motion_b)
            }
            (Shape::TriangleMesh(mesh), convex) if convex.is_convex() => {
                super::ccd::triangle_mesh_toi_motion(
                    convex,
                    motion_b,
                    mesh,
                    vertices_a,
                    bvh_a.ok_or(CollisionError::GeometryBounds)?,
                    motion_a,
                    &mut self.mesh_workspace,
                    &mut self.mesh_candidates,
                )
                .map(|hit| hit.map(reverse_hit))
            }
            (convex, Shape::TriangleMesh(mesh)) if convex.is_convex() => {
                super::ccd::triangle_mesh_toi_motion(
                    convex,
                    motion_a,
                    mesh,
                    vertices_b,
                    bvh_b.ok_or(CollisionError::GeometryBounds)?,
                    motion_b,
                    &mut self.mesh_workspace,
                    &mut self.mesh_candidates,
                )
            }
            (Shape::SkinnedTriangleMesh(mesh), convex) if convex.is_convex() => {
                super::ccd::triangle_mesh_toi_motion(
                    convex,
                    motion_b,
                    mesh.mesh(),
                    vertices_a,
                    bvh_a.ok_or(CollisionError::GeometryBounds)?,
                    motion_a,
                    &mut self.mesh_workspace,
                    &mut self.mesh_candidates,
                )
                .map(|hit| hit.map(reverse_hit))
            }
            (convex, Shape::SkinnedTriangleMesh(mesh)) if convex.is_convex() => {
                super::ccd::triangle_mesh_toi_motion(
                    convex,
                    motion_a,
                    mesh.mesh(),
                    vertices_b,
                    bvh_b.ok_or(CollisionError::GeometryBounds)?,
                    motion_b,
                    &mut self.mesh_workspace,
                    &mut self.mesh_candidates,
                )
            }
            (Shape::HeightField(field), convex) if convex.is_convex() => {
                super::ccd::height_field_toi_motion(
                    convex,
                    motion_b,
                    field,
                    motion_a,
                    &mut self.height_field_candidates,
                )
                .map(|hit| hit.map(reverse_hit))
            }
            (convex, Shape::HeightField(field)) if convex.is_convex() => {
                super::ccd::height_field_toi_motion(
                    convex,
                    motion_a,
                    field,
                    motion_b,
                    &mut self.height_field_candidates,
                )
            }
            (Shape::LevelSet(field), convex) if convex.is_convex() => {
                super::ccd::level_set_toi_motion(
                    convex,
                    motion_b,
                    field,
                    motion_a,
                    &mut self.level_set_candidates,
                )
                .map(|hit| hit.map(reverse_hit))
            }
            (convex, Shape::LevelSet(field)) if convex.is_convex() => {
                super::ccd::level_set_toi_motion(
                    convex,
                    motion_a,
                    field,
                    motion_b,
                    &mut self.level_set_candidates,
                )
            }
            (a, b) if a.is_convex() && b.is_convex() => {
                super::ccd::convex_toi_motion(a, motion_a, b, motion_b)
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
        convex: &Shape,
        convex_transform: &Transform,
        body_a: &Transform,
        body_b: &Transform,
        mode: CollisionMode,
        contact_offset: f64,
        surface_is_a: bool,
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
        make_surface_manifold(
            pair,
            surface_is_a,
            body_a,
            body_b,
            plane_contact,
            convex_point,
            normal_plane_to_convex,
            penetration,
            0,
            feature,
        )
        .map(Some)
    }

    #[allow(clippy::too_many_arguments)]
    fn triangle_convex(
        &mut self,
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
        let normal_for_side = triangle_normal_for_side(center, vertices[0], face_normal)?;
        let (
            triangle_point,
            convex_point,
            normal_triangle_to_convex,
            penetration,
            triangle_feature,
            convex_feature,
        ) = if let Shape::Sphere(sphere) = convex {
            let triangle_point = closest_point_on_triangle(center, vertices)?;
            let radial = center
                .checked_sub(triangle_point)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let distance = radial
                .checked_length()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let separation = distance - sphere.radius();
            if separation > contact_offset {
                return Ok(None);
            }
            let normal = radial.try_normalize().unwrap_or(normal_for_side);
            let convex_point = center
                .checked_sub(
                    normal
                        .checked_mul_scalar(sphere.radius())
                        .ok_or(CollisionError::ArithmeticOverflow)?,
                )
                .ok_or(CollisionError::ArithmeticOverflow)?;
            (
                triangle_point,
                convex_point,
                normal,
                (-separation).max(0.0),
                0,
                0,
            )
        } else {
            let triangle_shape = Shape::Triangle(
                Triangle::from_vertices(triangle.vertices())
                    .map_err(|_| CollisionError::InvalidContact)?,
            );
            let distance = gjk::distance(
                &triangle_shape,
                triangle_transform,
                convex,
                convex_transform,
            )?;
            if distance.distance > contact_offset {
                return Ok(None);
            }
            if distance.intersects {
                let intersection = gjk::intersect(
                    &triangle_shape,
                    triangle_transform,
                    convex,
                    convex_transform,
                )?;
                if intersection.intersects && simplex_has_resolvable_volume(&intersection.simplex)?
                {
                    let contact = self.epa.solve(
                        &intersection.simplex,
                        &triangle_shape,
                        triangle_transform,
                        convex,
                        convex_transform,
                    )?;
                    (
                        contact.point_a,
                        contact.point_b,
                        contact.normal,
                        contact.penetration,
                        contact.feature_a,
                        contact.feature_b,
                    )
                } else {
                    // The GJK witnesses prove an exact or tolerance-level contact.
                    // A lower-dimensional simplex does not prove penetration depth.
                    (
                        distance.point_a,
                        distance.point_b,
                        normal_for_side,
                        0.0,
                        0,
                        0,
                    )
                }
            } else {
                (
                    distance.point_a,
                    distance.point_b,
                    distance.normal,
                    0.0,
                    0,
                    0,
                )
            }
        };
        make_surface_manifold(
            pair,
            triangle_is_a,
            body_a,
            body_b,
            triangle_point,
            convex_point,
            normal_triangle_to_convex,
            penetration,
            triangle_feature,
            convex_feature,
        )
        .map(Some)
    }

    #[allow(clippy::too_many_arguments)]
    fn mesh_convex(
        &mut self,
        pair: CollisionPair,
        mesh: &TriangleMesh,
        vertices: &[Vec3],
        bvh: &TriangleBvh,
        mesh_transform: &Transform,
        convex: &Shape,
        convex_transform: &Transform,
        body_a: &Transform,
        body_b: &Transform,
        mesh_is_a: bool,
        contact_offset: f64,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        if vertices.len() != mesh.vertices().len() || bvh.triangle_count() != mesh.triangles().len()
        {
            return Err(CollisionError::GeometryBounds);
        }
        let bounds = expanded_convex_bounds(convex, convex_transform, contact_offset)?;
        let local_bounds = mesh::local_query_bounds(bounds, mesh_transform)?;
        bvh.query(
            local_bounds,
            &mut self.mesh_workspace,
            &mut self.mesh_candidates,
        )?;
        let mut manifold = None;
        for candidate_index in 0..self.mesh_candidates.len() {
            let triangle_index = self.mesh_candidates[candidate_index];
            let [a, b, c] = *mesh
                .triangles()
                .get(triangle_index)
                .ok_or(CollisionError::GeometryBounds)?;
            let triangle = Triangle::from_vertices([
                *vertices.get(a).ok_or(CollisionError::GeometryBounds)?,
                *vertices.get(b).ok_or(CollisionError::GeometryBounds)?,
                *vertices.get(c).ok_or(CollisionError::GeometryBounds)?,
            ])
            .map_err(|_| CollisionError::InvalidContact)?;
            let Some(contact) = self.triangle_convex(
                pair,
                &triangle,
                mesh_transform,
                convex,
                convex_transform,
                body_a,
                body_b,
                mesh_is_a,
                contact_offset,
            )?
            else {
                continue;
            };
            let feature =
                u64::try_from(triangle_index).map_err(|_| CollisionError::ArithmeticOverflow)?;
            merge_contact_manifold(&mut manifold, contact, feature)?;
        }
        Ok(manifold)
    }

    #[allow(clippy::too_many_arguments)]
    fn height_field_convex(
        &mut self,
        pair: CollisionPair,
        field: &HeightField,
        field_transform: &Transform,
        convex: &Shape,
        convex_transform: &Transform,
        body_a: &Transform,
        body_b: &Transform,
        surface_is_a: bool,
        contact_offset: f64,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        let bounds = expanded_convex_bounds(convex, convex_transform, contact_offset)?;
        height_field::query_world_bounds(
            field,
            field_transform,
            bounds,
            &mut self.height_field_candidates,
        )?;
        let mut manifold = None;
        for candidate_index in 0..self.height_field_candidates.len() {
            let candidate = self.height_field_candidates[candidate_index];
            let triangle = Triangle::from_vertices(candidate.vertices)
                .map_err(|_| CollisionError::InvalidContact)?;
            let Some(contact) = self.triangle_convex(
                pair,
                &triangle,
                field_transform,
                convex,
                convex_transform,
                body_a,
                body_b,
                surface_is_a,
                contact_offset,
            )?
            else {
                continue;
            };
            merge_contact_manifold(&mut manifold, contact, candidate.feature_id)?;
        }
        Ok(manifold)
    }

    #[allow(clippy::too_many_arguments)]
    fn level_set_convex(
        &mut self,
        pair: CollisionPair,
        field: &LevelSet,
        field_transform: &Transform,
        convex: &Shape,
        convex_transform: &Transform,
        body_a: &Transform,
        body_b: &Transform,
        surface_is_a: bool,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        let Some(hit) = super::ccd::level_set_toi(
            convex,
            *convex_transform,
            *convex_transform,
            field,
            *field_transform,
            *field_transform,
            &mut self.level_set_candidates,
        )?
        else {
            return Ok(None);
        };
        if !gjk::distance_to_point(convex, convex_transform, hit.point_b)?.intersects {
            return Err(CollisionError::NonConvergent);
        }
        let surface_point = hit.point_b;
        let convex_point = hit.point_a;
        let surface_to_convex = -hit.normal;
        let separation = convex_point
            .checked_sub(surface_point)
            .and_then(|delta| delta.checked_dot(surface_to_convex))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        make_surface_manifold(
            pair,
            surface_is_a,
            body_a,
            body_b,
            surface_point,
            convex_point,
            surface_to_convex,
            (-separation).max(0.0),
            0,
            0,
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
        if compare_convex_geometry(shape_a, collider_a, shape_b, collider_b)? == Ordering::Greater {
            return self
                .convex_convex(
                    pair, shape_b, collider_b, body_b, shape_a, collider_a, body_a,
                )?
                .map(|manifold| reverse_manifold(manifold, body_a, body_b))
                .transpose();
        }
        // Recover GJK's valid overlap result if its initial simplex differs by order.
        match self.convex_convex_ordered(
            pair, shape_a, collider_a, body_a, shape_b, collider_b, body_b,
        ) {
            Err(error @ CollisionError::NonConvergent) => {
                if let Ok(Some(manifold)) = self.convex_convex_ordered(
                    pair, shape_b, collider_b, body_b, shape_a, collider_a, body_a,
                ) {
                    return reverse_manifold(manifold, body_a, body_b).map(Some);
                }
                Err(error)
            }
            result => result,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn convex_convex_ordered(
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
        // Use a support contact when GJK confirms overlap without a tetrahedron.
        let contact = if result.simplex.count == 4 {
            self.epa
                .solve(&result.simplex, shape_a, collider_a, shape_b, collider_b)?
        } else {
            support_contact_for_degenerate_simplex(shape_a, collider_a, shape_b, collider_b)?
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

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        self.epa.retained_capacity_bytes()
            + self.mesh_workspace.retained_capacity_bytes()
            + self.mesh_candidates.capacity() * size_of::<usize>()
            + self.height_field_candidates.capacity() * size_of::<HeightFieldTriangle>()
            + self.level_set_candidates.capacity() * size_of::<super::level_set::LevelSetCell>()
    }
}

fn expanded_convex_bounds(
    shape: &Shape,
    transform: &Transform,
    expansion: f64,
) -> Result<Aabb, CollisionError> {
    let bounds = shape
        .world_bounds(transform)
        .map_err(|_| CollisionError::GeometryBounds)?
        .ok_or(CollisionError::UnsupportedGeometryPair)?;
    let expansion = Vec3::splat(expansion);
    let min = bounds
        .min()
        .checked_sub(expansion)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let max = bounds
        .max()
        .checked_add(expansion)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    Aabb::new(min, max).map_err(|_| CollisionError::GeometryBounds)
}

fn reverse_hit(hit: CcdHit) -> CcdHit {
    CcdHit {
        toi: hit.toi,
        normal: -hit.normal,
        point_a: hit.point_b,
        point_b: hit.point_a,
    }
}

fn merge_contact_manifold(
    output: &mut Option<ContactManifold>,
    source: ContactManifold,
    feature: u64,
) -> Result<(), CollisionError> {
    let manifold = output.get_or_insert_with(|| ContactManifold::new(source.pair));
    for (point_index, mut point) in source.points().iter().copied().enumerate() {
        point.feature_id = combine_feature_ids(feature, point.feature_id);
        let world_position = source
            .world_point(point_index)
            .ok_or(CollisionError::InvalidContact)?;
        let mut duplicate = false;
        for index in 0..manifold.len() {
            let existing = *manifold
                .point(index)
                .ok_or(CollisionError::InvalidContact)?;
            let existing_position = manifold
                .world_point(index)
                .ok_or(CollisionError::InvalidContact)?;
            let distance = world_position
                .checked_sub(existing_position)
                .and_then(Vec3::checked_length)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let scale = world_position
                .abs()
                .max_element()
                .max(existing_position.abs().max_element())
                .max(1.0);
            if distance <= scale * f64::EPSILON * 512.0
                && point.normal.dot(existing.normal) >= 0.995
            {
                if point.penetration > existing.penetration {
                    manifold.replace_point(index, point, world_position)?;
                }
                duplicate = true;
                break;
            }
        }
        if !duplicate {
            manifold.insert(point, world_position)?;
        }
    }
    Ok(())
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

#[allow(clippy::too_many_arguments)]
fn make_surface_manifold(
    pair: CollisionPair,
    surface_is_a: bool,
    body_a: &Transform,
    body_b: &Transform,
    surface_point: Vec3,
    convex_point: Vec3,
    surface_to_convex: Vec3,
    penetration: f64,
    surface_feature: u64,
    convex_feature: u64,
) -> Result<ContactManifold, CollisionError> {
    let (point_a, point_b, normal, feature_id) = if surface_is_a {
        (
            surface_point,
            convex_point,
            surface_to_convex,
            combine_feature_ids(surface_feature, convex_feature),
        )
    } else {
        (
            convex_point,
            surface_point,
            -surface_to_convex,
            combine_feature_ids(convex_feature, surface_feature),
        )
    };
    make_manifold(
        pair,
        body_a,
        body_b,
        point_a,
        point_b,
        normal,
        penetration,
        feature_id,
    )
}

fn reverse_manifold(
    manifold: ContactManifold,
    body_a: &Transform,
    body_b: &Transform,
) -> Result<ContactManifold, CollisionError> {
    let mut reversed = ContactManifold::new(manifold.pair);
    for (index, point) in manifold.points().iter().copied().enumerate() {
        let world_a = body_a.transform_point(point.local_a);
        let world_b = body_b.transform_point(point.local_b);
        let reversed_point = ContactPoint::from_world(
            body_b,
            body_a,
            world_b,
            world_a,
            -point.normal,
            point.penetration,
            point.feature_id,
        )?;
        let world_position = manifold
            .world_point(index)
            .ok_or(CollisionError::InvalidContact)?;
        reversed.insert(reversed_point, world_position)?;
    }
    Ok(reversed)
}

fn compare_convex_geometry(
    shape_a: &Shape,
    transform_a: &Transform,
    shape_b: &Shape,
    transform_b: &Transform,
) -> Result<Ordering, CollisionError> {
    let rank_a = convex_shape_rank(shape_a);
    let rank_b = convex_shape_rank(shape_b);
    match rank_a.cmp(&rank_b) {
        Ordering::Equal => {}
        order => return Ok(order),
    }
    let centers_a = transform_a.translation();
    let centers_b = transform_b.translation();
    for (a, b) in [
        (centers_a.x, centers_b.x),
        (centers_a.y, centers_b.y),
        (centers_a.z, centers_b.z),
    ] {
        match a.total_cmp(&b) {
            Ordering::Equal => {}
            order => return Ok(order),
        }
    }
    let bounds_a = shape_a
        .world_bounds(transform_a)
        .map_err(|_| CollisionError::GeometryBounds)?
        .ok_or(CollisionError::UnsupportedGeometryPair)?;
    let bounds_b = shape_b
        .world_bounds(transform_b)
        .map_err(|_| CollisionError::GeometryBounds)?
        .ok_or(CollisionError::UnsupportedGeometryPair)?;
    let values_a = [
        bounds_a.min().x,
        bounds_a.min().y,
        bounds_a.min().z,
        bounds_a.max().x,
        bounds_a.max().y,
        bounds_a.max().z,
    ];
    let values_b = [
        bounds_b.min().x,
        bounds_b.min().y,
        bounds_b.min().z,
        bounds_b.max().x,
        bounds_b.max().y,
        bounds_b.max().z,
    ];
    for (a, b) in values_a.into_iter().zip(values_b) {
        match a.total_cmp(&b) {
            Ordering::Equal => {}
            order => return Ok(order),
        }
    }
    Ok(Ordering::Equal)
}

fn convex_shape_rank(shape: &Shape) -> u8 {
    match shape {
        Shape::Sphere(_) => 0,
        Shape::Cuboid(_) => 1,
        Shape::Capsule(_) => 2,
        Shape::Cylinder(_) => 3,
        Shape::TaperedCapsule(_) => 4,
        Shape::TaperedCylinder(_) => 5,
        Shape::Convex(_) => 6,
        _ => u8::MAX,
    }
}

fn support_contact_for_degenerate_simplex(
    shape_a: &Shape,
    transform_a: &Transform,
    shape_b: &Shape,
    transform_b: &Transform,
) -> Result<super::epa::EpaResult, CollisionError> {
    let normal = transform_b
        .translation()
        .checked_sub(transform_a.translation())
        .and_then(Vec3::try_normalize)
        .unwrap_or(Vec3::X);
    let support = support_pair(shape_a, transform_a, shape_b, transform_b, normal)?;
    let penetration = support
        .difference
        .checked_dot(normal)
        .ok_or(CollisionError::ArithmeticOverflow)?;
    if penetration < 0.0 {
        return Err(CollisionError::NonConvergent);
    }
    Ok(super::epa::EpaResult {
        normal,
        penetration,
        point_a: support.point_a,
        point_b: support.point_b,
        feature_a: support.feature_a,
        feature_b: support.feature_b,
    })
}

#[allow(clippy::too_many_arguments)]
fn sphere_sphere(
    pair: CollisionPair,
    sphere_a: &Sphere,
    collider_a: &Transform,
    body_a: &Transform,
    sphere_b: &Sphere,
    collider_b: &Transform,
    body_b: &Transform,
) -> Result<Option<ContactManifold>, CollisionError> {
    let center_delta = collider_b
        .translation()
        .checked_sub(collider_a.translation())
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let distance = center_delta
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let radius_sum = sphere_a.radius() + sphere_b.radius();
    if !radius_sum.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if distance > radius_sum {
        return Ok(None);
    }

    let normal = center_delta.try_normalize().unwrap_or(Vec3::X);
    let point_a = collider_a
        .translation()
        .checked_add(
            normal
                .checked_mul_scalar(sphere_a.radius())
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    let point_b = collider_b
        .translation()
        .checked_sub(
            normal
                .checked_mul_scalar(sphere_b.radius())
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;

    make_manifold(
        pair,
        body_a,
        body_b,
        point_a,
        point_b,
        normal,
        (radius_sum - distance).max(0.0),
        0,
    )
    .map(Some)
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

fn triangle_normal_for_side(
    center: Vec3,
    vertex: Vec3,
    normal: Vec3,
) -> Result<Vec3, CollisionError> {
    let signed_distance = center
        .checked_sub(vertex)
        .and_then(|offset| offset.checked_dot(normal))
        .ok_or(CollisionError::ArithmeticOverflow)?;
    Ok(if signed_distance < 0.0 {
        -normal
    } else {
        normal
    })
}

fn simplex_has_resolvable_volume(simplex: &gjk::Simplex) -> Result<bool, CollisionError> {
    if simplex.count != 4 {
        return Ok(false);
    }
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
    let scale = ab
        .abs()
        .max_element()
        .max(ac.abs().max_element())
        .max(ad.abs().max_element());
    if !scale.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    if scale == 0.0 {
        return Ok(false);
    }
    let normalize = |vector: Vec3| Vec3::new(vector.x / scale, vector.y / scale, vector.z / scale);
    let relative_volume = normalize(ab)
        .checked_dot(
            normalize(ac)
                .checked_cross(normalize(ad))
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?
        .abs();
    Ok(relative_volume.is_finite() && relative_volume > f64::EPSILON * 128.0)
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
