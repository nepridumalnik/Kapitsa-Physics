use glam::{Affine3A, Vec2, Vec3};
use rstest::{fixture, rstest};

use super::{
    BoneInfluence, Capsule, Convex, Cuboid, Cylinder, HeightField, LevelSet, Plane, Shape,
    SkinnedTriangleMesh, Sphere, TaperedCapsule, TaperedCylinder, Triangle, TriangleMesh,
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
    assert!(Sphere::new(1.0).is_some());
}

#[rstest]
#[case::zero(0.0)]
#[case::negative(-1.0)]
#[case::nan(f32::NAN)]
#[case::infinity(f32::INFINITY)]
fn sphere_rejects_invalid_radius(#[case] radius: f32) {
    assert!(Sphere::new(radius).is_none());
}

#[rstest]
fn cuboid_stores_size_and_half_extents() {
    let cuboid = Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid");
    assert_eq!(cuboid.size(), Vec3::new(2.0, 4.0, 6.0));
    assert_eq!(cuboid.half_extents(), Vec3::new(1.0, 2.0, 3.0));
}

#[rstest]
#[case::zero_component(Vec3::new(0.0, 1.0, 1.0))]
#[case::negative_component(Vec3::new(1.0, -1.0, 1.0))]
#[case::infinite_component(Vec3::splat(f32::INFINITY))]
fn cuboid_rejects_invalid_size(#[case] size: Vec3) {
    assert!(Cuboid::from_vec(size).is_none());
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
#[case::zero_normal(Vec3::ZERO, 0.0)]
#[case::nan_distance(Vec3::X, f32::NAN)]
#[case::infinite_normal(Vec3::splat(f32::INFINITY), 1.0)]
#[case::overflowing_distance(Vec3::splat(f32::MIN_POSITIVE), f32::MAX)]
fn plane_rejects_invalid_equation(#[case] normal: Vec3, #[case] distance: f32) {
    assert!(Plane::from_vec(normal, distance).is_none());
}

#[rstest]
fn capsule_accepts_distinct_finite_endpoints() {
    assert!(Capsule::new(Vec3::ZERO, Vec3::Z, 0.5).is_some());
}

#[rstest]
#[case::coincident_endpoints(Vec3::ZERO, Vec3::ZERO, 0.5)]
#[case::zero_radius(Vec3::ZERO, Vec3::Z, 0.0)]
fn capsule_rejects_invalid_geometry(#[case] start: Vec3, #[case] end: Vec3, #[case] radius: f32) {
    assert!(Capsule::new(start, end, radius).is_none());
}

#[rstest]
fn cylinder_accepts_distinct_finite_endpoints() {
    assert!(Cylinder::new(Vec3::ZERO, Vec3::Z, 0.5).is_some());
}

#[rstest]
#[case::coincident_endpoints(Vec3::ZERO, Vec3::ZERO, 0.5)]
#[case::negative_radius(Vec3::ZERO, Vec3::Z, -0.5)]
fn cylinder_rejects_invalid_geometry(#[case] start: Vec3, #[case] end: Vec3, #[case] radius: f32) {
    assert!(Cylinder::new(start, end, radius).is_none());
}

#[rstest]
fn tapered_cylinder_accepts_positive_end_radii() {
    assert!(TaperedCylinder::new(Vec3::ZERO, Vec3::Z, 0.25, 0.5).is_some());
}

#[rstest]
#[case::zero_radius(0.0, 0.5, Vec3::Z)]
#[case::negative_radius(0.5, -0.5, Vec3::Z)]
#[case::coincident_endpoints(0.5, 0.5, Vec3::ZERO)]
fn tapered_cylinder_rejects_invalid_geometry(
    #[case] start_radius: f32,
    #[case] end_radius: f32,
    #[case] end: Vec3,
) {
    assert!(TaperedCylinder::new(Vec3::ZERO, end, start_radius, end_radius).is_none());
}

#[rstest]
fn tapered_capsule_accepts_uncontained_end_spheres() {
    assert!(TaperedCapsule::new(Vec3::ZERO, Vec3::Z, 0.5, 0.25).is_some());
}

#[rstest]
#[case::contained_end_sphere(Vec3::ZERO, Vec3::Z, 2.0, 0.5)]
#[case::coincident_endpoints(Vec3::ZERO, Vec3::ZERO, 0.5, 0.5)]
fn tapered_capsule_rejects_invalid_geometry(
    #[case] start: Vec3,
    #[case] end: Vec3,
    #[case] start_radius: f32,
    #[case] end_radius: f32,
) {
    assert!(TaperedCapsule::new(start, end, start_radius, end_radius).is_none());
}

#[rstest]
fn triangle_accepts_non_degenerate_vertices() {
    assert!(Triangle::new(Vec3::ZERO, Vec3::X, Vec3::Y).is_some());
}

#[rstest]
#[case::collinear(Vec3::ZERO, Vec3::X, Vec3::X * 2.0)]
fn triangle_rejects_degenerate_vertices(#[case] a: Vec3, #[case] b: Vec3, #[case] c: Vec3) {
    assert!(Triangle::new(a, b, c).is_none());
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
