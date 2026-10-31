use crate::aabb::Aabb;
use glam::{Affine3A, Vec2, Vec3};
use rstest::{fixture, rstest};

use super::{
    BoneInfluence, Capsule, Convex, Cuboid, Cylinder, HeightField, LevelSet, Plane, Shape,
    ShapeError, SkinnedTriangleMesh, Sphere, TaperedCapsule, TaperedCylinder, Triangle,
    TriangleMesh,
};

#[fixture]
fn tetrahedron() -> (Vec<Vec3>, Vec<[usize; 3]>) {
    let vertices = vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z];
    let faces = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    (vertices, faces)
}

#[fixture]
fn triangle_mesh() -> TriangleMesh {
    TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]])
        .expect("valid triangle mesh")
}

#[fixture]
fn vertex_influences() -> Vec<Vec<BoneInfluence>> {
    (0..3)
        .map(|_| vec![BoneInfluence::new(0, 1.0).expect("valid bone influence")])
        .collect()
}

#[rstest]
fn sphere_accepts_positive_finite_radius() {
    assert!(Sphere::new(1.0).is_ok());
}

#[rstest]
#[case::zero(0.0, ShapeError::InvalidDimension)]
#[case::negative(-1.0, ShapeError::InvalidDimension)]
#[case::nan(f32::NAN, ShapeError::NonFiniteParameter)]
#[case::infinity(f32::INFINITY, ShapeError::NonFiniteParameter)]
fn sphere_rejects_invalid_radius(#[case] radius: f32, #[case] error: ShapeError) {
    assert_eq!(Sphere::new(radius).err(), Some(error));
}

#[rstest]
fn cuboid_stores_size_and_half_extents() {
    let cuboid = Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid");
    assert_eq!(cuboid.size(), Vec3::new(2.0, 4.0, 6.0));
    assert_eq!(cuboid.half_extents(), Vec3::new(1.0, 2.0, 3.0));
}

#[rstest]
#[case::zero_component(Vec3::new(0.0, 1.0, 1.0), ShapeError::InvalidDimension)]
#[case::negative_component(Vec3::new(1.0, -1.0, 1.0), ShapeError::InvalidDimension)]
#[case::infinite_component(Vec3::splat(f32::INFINITY), ShapeError::NonFiniteParameter)]
fn cuboid_rejects_invalid_size(#[case] size: Vec3, #[case] error: ShapeError) {
    assert_eq!(Cuboid::from_vec(size).err(), Some(error));
}

#[rstest]
#[case(Vec3::new(0.0, 2.0, 0.0), 10.0, Vec3::Y, 5.0)]
fn plane_normalizes_its_equation(
    #[case] normal: Vec3,
    #[case] distance: f32,
    #[case] expected_normal: Vec3,
    #[case] expected_distance: f32,
) {
    let plane = Plane::from_vec(normal, distance).expect("valid plane");
    assert_eq!(plane.normal(), expected_normal);
    assert_eq!(plane.distance(), expected_distance);
}

#[rstest]
#[case::zero_normal(Vec3::ZERO, 0.0, ShapeError::DegenerateGeometry)]
#[case::nan_distance(Vec3::X, f32::NAN, ShapeError::NonFiniteParameter)]
#[case::infinite_normal(Vec3::splat(f32::INFINITY), 1.0, ShapeError::NonFiniteParameter)]
#[case::overflowing_distance(
    Vec3::splat(f32::MIN_POSITIVE),
    f32::MAX,
    ShapeError::ArithmeticOverflow
)]
fn plane_rejects_invalid_equation(
    #[case] normal: Vec3,
    #[case] distance: f32,
    #[case] error: ShapeError,
) {
    assert_eq!(Plane::from_vec(normal, distance).err(), Some(error));
}

#[rstest]
fn capsule_accepts_distinct_finite_endpoints() {
    assert!(Capsule::new(Vec3::ZERO, Vec3::Z, 0.5).is_ok());
}

#[rstest]
#[case::coincident_endpoints(Vec3::ZERO, Vec3::ZERO, 0.5, ShapeError::DegenerateGeometry)]
#[case::zero_radius(Vec3::ZERO, Vec3::Z, 0.0, ShapeError::InvalidDimension)]
fn capsule_rejects_invalid_geometry(
    #[case] start: Vec3,
    #[case] end: Vec3,
    #[case] radius: f32,
    #[case] error: ShapeError,
) {
    assert_eq!(Capsule::new(start, end, radius).err(), Some(error));
}

#[rstest]
fn cylinder_accepts_distinct_finite_endpoints() {
    assert!(Cylinder::new(Vec3::ZERO, Vec3::Z, 0.5).is_ok());
}

#[rstest]
#[case::coincident_endpoints(Vec3::ZERO, Vec3::ZERO, 0.5, ShapeError::DegenerateGeometry)]
#[case::negative_radius(Vec3::ZERO, Vec3::Z, -0.5, ShapeError::InvalidDimension)]
fn cylinder_rejects_invalid_geometry(
    #[case] start: Vec3,
    #[case] end: Vec3,
    #[case] radius: f32,
    #[case] error: ShapeError,
) {
    assert_eq!(Cylinder::new(start, end, radius).err(), Some(error));
}

#[rstest]
fn tapered_cylinder_accepts_positive_end_radii() {
    assert!(TaperedCylinder::new(Vec3::ZERO, Vec3::Z, 0.25, 0.5).is_ok());
}

#[rstest]
#[case::zero_radius(0.0, 0.5, Vec3::Z, ShapeError::InvalidDimension)]
#[case::negative_radius(0.5, -0.5, Vec3::Z, ShapeError::InvalidDimension)]
#[case::coincident_endpoints(0.5, 0.5, Vec3::ZERO, ShapeError::DegenerateGeometry)]
fn tapered_cylinder_rejects_invalid_geometry(
    #[case] start_radius: f32,
    #[case] end_radius: f32,
    #[case] end: Vec3,
    #[case] error: ShapeError,
) {
    assert_eq!(
        TaperedCylinder::new(Vec3::ZERO, end, start_radius, end_radius).err(),
        Some(error)
    );
}

#[rstest]
fn tapered_capsule_accepts_uncontained_end_spheres() {
    assert!(TaperedCapsule::new(Vec3::ZERO, Vec3::Z, 0.5, 0.25).is_ok());
}

#[rstest]
#[case::contained_end_sphere(Vec3::ZERO, Vec3::Z, 2.0, 0.5, ShapeError::DegenerateGeometry)]
#[case::coincident_endpoints(Vec3::ZERO, Vec3::ZERO, 0.5, 0.5, ShapeError::DegenerateGeometry)]
fn tapered_capsule_rejects_invalid_geometry(
    #[case] start: Vec3,
    #[case] end: Vec3,
    #[case] start_radius: f32,
    #[case] end_radius: f32,
    #[case] error: ShapeError,
) {
    assert_eq!(
        TaperedCapsule::new(start, end, start_radius, end_radius).err(),
        Some(error)
    );
}

#[rstest]
fn triangle_accepts_non_degenerate_vertices() {
    assert!(Triangle::new(Vec3::ZERO, Vec3::X, Vec3::Y).is_ok());
}

#[rstest]
#[case::collinear(Vec3::ZERO, Vec3::X, Vec3::X * 2.0, ShapeError::DegenerateGeometry)]
#[case::non_finite(
    Vec3::ZERO,
    Vec3::X,
    Vec3::splat(f32::NAN),
    ShapeError::NonFiniteParameter
)]
fn triangle_rejects_invalid_vertices(
    #[case] a: Vec3,
    #[case] b: Vec3,
    #[case] c: Vec3,
    #[case] error: ShapeError,
) {
    assert_eq!(Triangle::new(a, b, c).err(), Some(error));
}

#[rstest]
fn triangle_mesh_accepts_open_surface() {
    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]);
    assert!(mesh.is_some());
}

#[rstest]
#[case::out_of_bounds(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 3]])]
#[case::repeated_index(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 1]])]
#[case::degenerate_triangle(vec![Vec3::ZERO, Vec3::X, Vec3::X * 2.0], vec![[0, 1, 2]])]
fn triangle_mesh_rejects_invalid_triangles(
    #[case] vertices: Vec<Vec3>,
    #[case] triangles: Vec<[usize; 3]>,
) {
    assert!(TriangleMesh::new(vertices, triangles).is_none());
}

#[rstest]
fn convex_accepts_outward_tetrahedron(tetrahedron: (Vec<Vec3>, Vec<[usize; 3]>)) {
    let (vertices, faces) = tetrahedron;
    assert!(Convex::new(vertices, faces).is_some());
}

#[rstest]
fn convex_rejects_inward_tetrahedron(tetrahedron: (Vec<Vec3>, Vec<[usize; 3]>)) {
    let (vertices, mut faces) = tetrahedron;
    for face in &mut faces {
        face.swap(1, 2);
    }
    assert!(Convex::new(vertices, faces).is_none());
}

#[rstest]
fn convex_rejects_open_or_mixed_winding(tetrahedron: (Vec<Vec3>, Vec<[usize; 3]>)) {
    let (vertices, faces) = tetrahedron;
    let mut open_faces = faces.clone();
    open_faces.pop();
    assert!(Convex::new(vertices.clone(), open_faces).is_none());

    let mut mixed_faces = faces;
    mixed_faces[0].swap(1, 2);
    assert!(Convex::new(vertices, mixed_faces).is_none());
}

#[rstest]
fn height_field_accepts_valid_grid() {
    assert!(HeightField::from_vec(2, 2, Vec2::ONE, vec![0.0; 4]).is_some());
}

#[rstest]
#[case::count_mismatch(2, 2, Vec2::ONE, vec![0.0; 3])]
#[case::zero_spacing(2, 2, Vec2::ZERO, vec![0.0; 4])]
#[case::dimension_overflow(usize::MAX, 2, Vec2::ONE, vec![])]
fn height_field_rejects_invalid_grid(
    #[case] rows: usize,
    #[case] columns: usize,
    #[case] grid_spacing: Vec2,
    #[case] heights: Vec<f32>,
) {
    assert!(HeightField::from_vec(rows, columns, grid_spacing, heights).is_none());
}

#[rstest]
fn level_set_accepts_valid_grid() {
    assert!(LevelSet::from_vec(Vec3::ZERO, [2, 2, 2], Vec3::ONE, vec![0.0; 8]).is_some());
}

#[rstest]
#[case::count_mismatch([2, 2, 2], Vec3::ONE, vec![0.0; 7])]
#[case::zero_dimension([0, 2, 2], Vec3::ONE, vec![])]
#[case::dimension_overflow([usize::MAX, 2, 2], Vec3::ONE, vec![])]
#[case::zero_spacing([2, 2, 2], Vec3::ZERO, vec![0.0; 8])]
fn level_set_rejects_invalid_grid(
    #[case] dimensions: [usize; 3],
    #[case] grid_spacing: Vec3,
    #[case] signed_distances: Vec<f32>,
) {
    assert!(LevelSet::from_vec(Vec3::ZERO, dimensions, grid_spacing, signed_distances).is_none());
}

#[rstest]
fn skinned_triangle_mesh_accepts_matching_vertex_influences(
    triangle_mesh: TriangleMesh,
    vertex_influences: Vec<Vec<BoneInfluence>>,
) {
    let skinned = SkinnedTriangleMesh::new(
        triangle_mesh,
        vec!["root".into()],
        vertex_influences,
        Affine3A::IDENTITY,
        vec![Affine3A::IDENTITY],
    )
    .expect("valid skinned mesh");
    let shape = Shape::SkinnedTriangleMesh(skinned);
    assert!(matches!(shape, Shape::SkinnedTriangleMesh(_)));
}

#[rstest]
#[case::negative_weight(-1.0)]
#[case::weight_above_one(1.5)]
fn bone_influence_rejects_invalid_weight(#[case] weight: f32) {
    assert!(BoneInfluence::new(0, weight).is_none());
}

#[rstest]
#[case::small(0.5)]
#[case::large(2.0)]
#[case::maximum(f32::MAX)]
fn sphere_local_bounds(#[case] radius: f32) {
    let sphere = Sphere::new(radius).expect("valid sphere");
    let extent = Vec3::splat(radius);

    assert_eq!(sphere.local_bounds(), Aabb::new(-extent, extent).ok());
}

#[test]
fn cuboid_local_bounds() {
    let cuboid = Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid");

    assert_eq!(
        cuboid.local_bounds(),
        Aabb::new(Vec3::new(-1.0, -2.0, -3.0), Vec3::new(1.0, 2.0, 3.0),).ok()
    );
}

#[test]
fn cuboid_local_bounds_with_maximum_size() {
    let cuboid = Cuboid::from_vec(Vec3::splat(f32::MAX)).expect("valid cuboid");

    let half = Vec3::splat(f32::MAX * 0.5);

    assert_eq!(cuboid.local_bounds(), Aabb::new(-half, half).ok());
}

#[test]
fn capsule_local_bounds() {
    let capsule = Capsule::new(Vec3::new(1.0, -2.0, 3.0), Vec3::new(-4.0, 5.0, -6.0), 0.5)
        .expect("valid capsule");

    assert_eq!(
        capsule.local_bounds(),
        Aabb::new(Vec3::new(-4.5, -2.5, -6.5), Vec3::new(1.5, 5.5, 3.5),).ok()
    );
}

#[test]
fn capsule_local_bounds_rejects_overflow() {
    let capsule = Capsule::new(
        Vec3::new(f32::MAX, 0.0, 0.0),
        Vec3::new(f32::MAX, 0.0, 1.0),
        f32::MAX,
    )
    .expect("valid capsule");

    assert!(capsule.local_bounds().is_none());
}

#[rstest]
#[case::tilted_axis(
    Vec3::new(2.0, -3.0, 4.0),
    Vec3::new(-4.0, 5.0, -6.0),
    Vec3::new(-4.5, -3.5, -6.5),
    Vec3::new(2.5, 5.5, 4.5),
)]
#[case::reversed_axis(
    Vec3::new(-4.0, 5.0, -6.0),
    Vec3::new(2.0, -3.0, 4.0),
    Vec3::new(-4.5, -3.5, -6.5),
    Vec3::new(2.5, 5.5, 4.5),
)]
fn cylinder_local_bounds(
    #[case] start: Vec3,
    #[case] end: Vec3,
    #[case] min: Vec3,
    #[case] max: Vec3,
) {
    let cylinder = Cylinder::new(start, end, 0.5).expect("valid cylinder");

    assert_eq!(cylinder.local_bounds(), Aabb::new(min, max).ok());
}

#[rstest]
#[case::forward(
    Vec3::new(2.0, -3.0, 4.0),
    Vec3::new(-4.0, 5.0, -6.0),
    0.5,
    2.0,
    Vec3::new(-6.0, -3.5, -8.0),
    Vec3::new(2.5, 7.0, 4.5),
)]
#[case::reversed_with_radii(
    Vec3::new(-4.0, 5.0, -6.0),
    Vec3::new(2.0, -3.0, 4.0),
    2.0,
    0.5,
    Vec3::new(-6.0, -3.5, -8.0),
    Vec3::new(2.5, 7.0, 4.5),
)]
fn tapered_capsule_local_bounds(
    #[case] start: Vec3,
    #[case] end: Vec3,
    #[case] start_radius: f32,
    #[case] end_radius: f32,
    #[case] min: Vec3,
    #[case] max: Vec3,
) {
    let capsule =
        TaperedCapsule::new(start, end, start_radius, end_radius).expect("valid tapered capsule");

    assert_eq!(capsule.local_bounds(), Aabb::new(min, max).ok());
}

#[rstest]
#[case::forward(
    Vec3::new(2.0, -3.0, 4.0),
    Vec3::new(-4.0, 5.0, -6.0),
    0.5,
    2.0,
    Vec3::new(-6.0, -3.5, -8.0),
    Vec3::new(2.5, 7.0, 4.5),
)]
#[case::reversed_with_radii(
    Vec3::new(-4.0, 5.0, -6.0),
    Vec3::new(2.0, -3.0, 4.0),
    2.0,
    0.5,
    Vec3::new(-6.0, -3.5, -8.0),
    Vec3::new(2.5, 7.0, 4.5),
)]
fn tapered_cylinder_local_bounds(
    #[case] start: Vec3,
    #[case] end: Vec3,
    #[case] start_radius: f32,
    #[case] end_radius: f32,
    #[case] min: Vec3,
    #[case] max: Vec3,
) {
    let cylinder =
        TaperedCylinder::new(start, end, start_radius, end_radius).expect("valid tapered cylinder");

    assert_eq!(cylinder.local_bounds(), Aabb::new(min, max).ok());
}

#[rstest]
fn cylindrical_local_bounds_reject_overflow() {
    let start = Vec3::new(f32::MAX, 0.0, 0.0);
    let end = Vec3::new(f32::MAX, 0.0, 1.0);

    let cylinder = Cylinder::new(start, end, f32::MAX).expect("valid cylinder");
    assert!(cylinder.local_bounds().is_none());

    let tapered_capsule =
        TaperedCapsule::new(start, end, f32::MAX, f32::MAX).expect("valid tapered capsule");
    assert!(tapered_capsule.local_bounds().is_none());

    let tapered_cylinder =
        TaperedCylinder::new(start, end, f32::MAX, f32::MAX).expect("valid tapered cylinder");
    assert!(tapered_cylinder.local_bounds().is_none());
}

#[rstest]
#[case::arbitrary_coordinates(
    [
        Vec3::new(-4.0, 2.0, 3.0),
        Vec3::new(1.0, 5.0, -2.0),
        Vec3::new(3.0, -1.0, 4.0),
    ],
    Vec3::new(-4.0, -1.0, -2.0),
    Vec3::new(3.0, 5.0, 4.0),
)]
#[case::flat_on_z(
    [
        Vec3::new(-1.0, 2.0, 0.0),
        Vec3::new(3.0, -4.0, 0.0),
        Vec3::new(2.0, 5.0, 0.0),
    ],
    Vec3::new(-1.0, -4.0, 0.0),
    Vec3::new(3.0, 5.0, 0.0),
)]
fn triangle_local_bounds(#[case] vertices: [Vec3; 3], #[case] min: Vec3, #[case] max: Vec3) {
    let triangle = Triangle::from_vertices(vertices).expect("valid triangle");

    assert_eq!(triangle.local_bounds(), Aabb::new(min, max).ok());
}

#[rstest]
fn convex_local_bounds_cover_all_vertices() {
    let translation = Vec3::new(-3.0, -4.0, -5.0);
    let vertices = vec![
        translation,
        translation + Vec3::new(2.0, 1.0, 0.0),
        translation + Vec3::new(0.0, 3.0, 1.0),
        translation + Vec3::new(1.0, 0.0, 4.0),
    ];
    let faces = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    let convex = Convex::new(vertices, faces).expect("valid convex tetrahedron");

    assert_eq!(
        convex.local_bounds(),
        Aabb::new(Vec3::new(-3.0, -4.0, -5.0), Vec3::splat(-1.0)).ok()
    );
}

#[rstest]
fn triangle_mesh_local_bounds_include_unindexed_vertices() {
    let vertices = vec![
        Vec3::new(-4.0, 1.0, -2.0),
        Vec3::new(3.0, 1.0, -2.0),
        Vec3::new(0.0, 5.0, -2.0),
        Vec3::new(1.0, -3.0, 4.0),
        Vec3::new(-2.0, 0.0, 6.0),
        Vec3::new(9.0, 0.0, 0.0),
        Vec3::new(0.0, 8.0, 0.0),
        Vec3::new(0.0, 0.0, 15.0),
    ];
    let triangles = vec![[0, 1, 2], [3, 4, 0]];
    let mesh = TriangleMesh::new(vertices, triangles).expect("valid triangle mesh");

    assert_eq!(
        mesh.local_bounds(),
        Aabb::new(Vec3::new(-4.0, -3.0, -2.0), Vec3::new(9.0, 8.0, 15.0)).ok()
    );
}

#[rstest]
fn height_field_local_bounds_use_grid_extent_and_height_range() {
    let heights = vec![-4.0, 2.0, -1.0, 6.0, 3.0, 5.0];
    let height_field =
        HeightField::from_vec(3, 2, Vec2::new(2.0, 0.5), heights).expect("valid height field");

    assert_eq!(
        height_field.local_bounds(),
        Aabb::new(Vec3::new(0.0, 0.0, -4.0), Vec3::new(2.0, 1.0, 6.0)).ok()
    );
}

#[rstest]
fn level_set_local_bounds_cover_only_grid_nodes() {
    let level_set = LevelSet::from_vec(
        Vec3::new(-3.0, 2.0, -5.0),
        [3, 2, 4],
        Vec3::new(0.5, 2.0, 3.0),
        vec![1000.0; 24],
    )
    .expect("valid level set");

    assert_eq!(
        level_set.local_bounds(),
        Aabb::new(Vec3::new(-3.0, 2.0, -5.0), Vec3::new(-2.0, 4.0, 4.0)).ok()
    );
}

#[rstest]
fn shape_local_bounds_dispatches_to_finite_shapes(
    tetrahedron: (Vec<Vec3>, Vec<[usize; 3]>),
    triangle_mesh: TriangleMesh,
) {
    let (vertices, faces) = tetrahedron;
    let convex = Convex::new(vertices, faces).expect("valid convex tetrahedron");
    let height_field = HeightField::from_vec(2, 2, Vec2::ONE, vec![0.0, 0.0, 0.0, 1.0])
        .expect("valid height field");
    let level_set = LevelSet::from_vec(Vec3::ZERO, [2, 2, 2], Vec3::ONE, vec![0.0; 8])
        .expect("valid level set");

    let shapes = [
        (
            Shape::Sphere(Sphere::new(1.0).expect("valid sphere")),
            Vec3::splat(-1.0),
            Vec3::ONE,
        ),
        (
            Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("valid cuboid")),
            Vec3::splat(-1.0),
            Vec3::ONE,
        ),
        (
            Shape::Capsule(Capsule::new(Vec3::ZERO, Vec3::Z, 0.5).expect("valid capsule")),
            Vec3::splat(-0.5),
            Vec3::new(0.5, 0.5, 1.5),
        ),
        (
            Shape::Cylinder(Cylinder::new(Vec3::ZERO, Vec3::Z, 0.5).expect("valid cylinder")),
            Vec3::splat(-0.5),
            Vec3::new(0.5, 0.5, 1.5),
        ),
        (
            Shape::TaperedCapsule(
                TaperedCapsule::new(Vec3::ZERO, Vec3::Z, 0.25, 0.5).expect("valid tapered capsule"),
            ),
            Vec3::new(-0.5, -0.5, -0.25),
            Vec3::new(0.5, 0.5, 1.5),
        ),
        (
            Shape::TaperedCylinder(
                TaperedCylinder::new(Vec3::ZERO, Vec3::Z, 0.25, 0.5)
                    .expect("valid tapered cylinder"),
            ),
            Vec3::new(-0.5, -0.5, -0.25),
            Vec3::new(0.5, 0.5, 1.5),
        ),
        (
            Shape::Triangle(Triangle::new(Vec3::ZERO, Vec3::X, Vec3::Y).expect("valid triangle")),
            Vec3::ZERO,
            Vec3::new(1.0, 1.0, 0.0),
        ),
        (Shape::Convex(convex), Vec3::ZERO, Vec3::ONE),
        (
            Shape::TriangleMesh(triangle_mesh),
            Vec3::ZERO,
            Vec3::new(1.0, 1.0, 0.0),
        ),
        (Shape::HeightField(height_field), Vec3::ZERO, Vec3::ONE),
        (Shape::LevelSet(level_set), Vec3::ZERO, Vec3::ONE),
    ];

    for (shape, min, max) in shapes {
        assert_eq!(shape.local_bounds(), Aabb::new(min, max).ok());
    }
}

#[rstest]
fn shape_local_bounds_returns_none_for_plane() {
    let plane = Plane::new(0.0, 1.0, 0.0, 0.0).expect("valid plane");

    assert!(Shape::Plane(plane).local_bounds().is_none());
}

#[rstest]
fn shape_local_bounds_returns_none_for_skinned_mesh(
    triangle_mesh: TriangleMesh,
    vertex_influences: Vec<Vec<BoneInfluence>>,
) {
    let skinned_mesh = SkinnedTriangleMesh::new(
        triangle_mesh,
        vec!["root".into()],
        vertex_influences,
        Affine3A::IDENTITY,
        vec![Affine3A::IDENTITY],
    )
    .expect("valid skinned mesh");

    assert!(
        Shape::SkinnedTriangleMesh(skinned_mesh)
            .local_bounds()
            .is_none()
    );
}

#[rstest]
fn shape_local_bounds_propagates_overflow() {
    let start = Vec3::new(f32::MAX, 0.0, 0.0);
    let end = Vec3::new(f32::MAX, 0.0, 1.0);
    let cylinder = Cylinder::new(start, end, f32::MAX).expect("valid cylinder");

    assert!(Shape::Cylinder(cylinder).local_bounds().is_none());
}
