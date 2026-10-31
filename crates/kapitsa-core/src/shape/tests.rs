use crate::aabb::{Aabb, AabbError};
use crate::transform::Transform;
use glam::{Affine3A, Quat, Vec2, Vec3};
use kapitsa_math::{Quat as MathQuat, Vec3 as MathVec3};
use rstest::{fixture, rstest};

use super::{
    BoneInfluence, BoundsError, Capsule, Convex, Cuboid, Cylinder, HeightField, LevelSet, Plane,
    Shape, ShapeError, SkinnedTriangleMesh, Sphere, TaperedCapsule, TaperedCylinder, Triangle,
    TriangleMesh,
};

fn expected_aabb(min: Vec3, max: Vec3) -> Aabb {
    Aabb::from_f32_bounds(min, max).expect("valid test bounds")
}

fn transform(translation: Vec3, rotation: Quat) -> Transform {
    let translation = MathVec3::new(
        f64::from(translation.x),
        f64::from(translation.y),
        f64::from(translation.z),
    );
    let rotation = MathQuat::new(
        f64::from(rotation.x),
        f64::from(rotation.y),
        f64::from(rotation.z),
        f64::from(rotation.w),
    )
    .try_normalize()
    .expect("valid test rotation");
    Transform::new(translation, rotation).expect("valid transform")
}

#[test]
fn bounds_error_exposes_nested_aabb_error_as_source() {
    let error = BoundsError::InvalidBounds(AabbError::NonFiniteBounds);

    assert_eq!(
        std::error::Error::source(&error).and_then(|source| source.downcast_ref::<AabbError>()),
        Some(&AabbError::NonFiniteBounds)
    );
}

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
    assert!(mesh.is_ok());
}

#[rstest]
#[case::out_of_bounds(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 3]], ShapeError::InvalidTopology)]
#[case::repeated_index(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 1]], ShapeError::InvalidTopology)]
#[case::degenerate_triangle(vec![Vec3::ZERO, Vec3::X, Vec3::X * 2.0], vec![[0, 1, 2]], ShapeError::DegenerateGeometry)]
fn triangle_mesh_rejects_invalid_triangles(
    #[case] vertices: Vec<Vec3>,
    #[case] triangles: Vec<[usize; 3]>,
    #[case] error: ShapeError,
) {
    assert_eq!(TriangleMesh::new(vertices, triangles).err(), Some(error));
}

#[test]
fn triangle_mesh_distinguishes_non_finite_vertices_and_area_overflow() {
    assert_eq!(
        TriangleMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::splat(f32::NAN)],
            vec![[0, 1, 2]],
        )
        .err(),
        Some(ShapeError::NonFiniteParameter)
    );
    assert_eq!(
        TriangleMesh::new(
            vec![Vec3::ZERO, Vec3::X * f32::MAX, Vec3::Y * f32::MAX],
            vec![[0, 1, 2]],
        )
        .err(),
        Some(ShapeError::ArithmeticOverflow)
    );
}

#[rstest]
fn convex_accepts_outward_tetrahedron(tetrahedron: (Vec<Vec3>, Vec<[usize; 3]>)) {
    let (vertices, faces) = tetrahedron;
    assert!(Convex::new(vertices, faces).is_ok());
}

#[rstest]
fn convex_rejects_inward_tetrahedron(tetrahedron: (Vec<Vec3>, Vec<[usize; 3]>)) {
    let (vertices, mut faces) = tetrahedron;
    for face in &mut faces {
        face.swap(1, 2);
    }
    assert_eq!(
        Convex::new(vertices, faces).err(),
        Some(ShapeError::InvalidTopology)
    );
}

#[rstest]
fn convex_rejects_open_or_mixed_winding(tetrahedron: (Vec<Vec3>, Vec<[usize; 3]>)) {
    let (vertices, faces) = tetrahedron;
    let mut open_faces = faces.clone();
    open_faces.pop();
    assert_eq!(
        Convex::new(vertices.clone(), open_faces).err(),
        Some(ShapeError::InvalidTopology)
    );

    let mut mixed_faces = faces;
    mixed_faces[0].swap(1, 2);
    assert_eq!(
        Convex::new(vertices, mixed_faces).err(),
        Some(ShapeError::InvalidTopology)
    );
}

#[rstest]
fn convex_rejects_non_finite_vertices(tetrahedron: (Vec<Vec3>, Vec<[usize; 3]>)) {
    let (mut vertices, faces) = tetrahedron;
    vertices[0].x = f32::NAN;

    assert_eq!(
        Convex::new(vertices, faces).err(),
        Some(ShapeError::NonFiniteParameter)
    );
}

#[rstest]
fn height_field_accepts_valid_grid() {
    assert!(HeightField::from_vec(2, 2, Vec2::ONE, vec![0.0; 4]).is_ok());
}

#[rstest]
#[case::count_mismatch(2, 2, Vec2::ONE, vec![0.0; 3], ShapeError::DataLengthMismatch)]
#[case::zero_spacing(2, 2, Vec2::ZERO, vec![0.0; 4], ShapeError::InvalidDimension)]
#[case::dimension_overflow(usize::MAX, 2, Vec2::ONE, vec![], ShapeError::ArithmeticOverflow)]
#[case::too_few_rows(1, 2, Vec2::ONE, vec![0.0; 2], ShapeError::InvalidGridDimensions)]
fn height_field_rejects_invalid_grid(
    #[case] rows: usize,
    #[case] columns: usize,
    #[case] grid_spacing: Vec2,
    #[case] heights: Vec<f32>,
    #[case] error: ShapeError,
) {
    assert_eq!(
        HeightField::from_vec(rows, columns, grid_spacing, heights).err(),
        Some(error)
    );
}

#[test]
fn height_field_distinguishes_non_finite_data_and_span_overflow() {
    assert_eq!(
        HeightField::from_vec(2, 2, Vec2::new(f32::NAN, 1.0), vec![0.0; 4]).err(),
        Some(ShapeError::NonFiniteParameter)
    );
    assert_eq!(
        HeightField::from_vec(2, 2, Vec2::ONE, vec![f32::INFINITY; 4]).err(),
        Some(ShapeError::NonFiniteParameter)
    );
    assert_eq!(
        HeightField::from_vec(3, 2, Vec2::splat(f32::MAX), vec![0.0; 6]).err(),
        Some(ShapeError::ArithmeticOverflow)
    );
}

#[rstest]
fn level_set_accepts_valid_grid() {
    assert!(LevelSet::from_vec(Vec3::ZERO, [2, 2, 2], Vec3::ONE, vec![0.0; 8]).is_ok());
}

#[rstest]
#[case::count_mismatch([2, 2, 2], Vec3::ONE, vec![0.0; 7], ShapeError::DataLengthMismatch)]
#[case::zero_dimension([0, 2, 2], Vec3::ONE, vec![], ShapeError::InvalidGridDimensions)]
#[case::dimension_overflow([usize::MAX, 2, 2], Vec3::ONE, vec![], ShapeError::ArithmeticOverflow)]
#[case::zero_spacing([2, 2, 2], Vec3::ZERO, vec![0.0; 8], ShapeError::InvalidDimension)]
fn level_set_rejects_invalid_grid(
    #[case] dimensions: [usize; 3],
    #[case] grid_spacing: Vec3,
    #[case] signed_distances: Vec<f32>,
    #[case] error: ShapeError,
) {
    assert_eq!(
        LevelSet::from_vec(Vec3::ZERO, dimensions, grid_spacing, signed_distances).err(),
        Some(error)
    );
}

#[test]
fn level_set_distinguishes_non_finite_data_and_coordinate_overflow() {
    assert_eq!(
        LevelSet::from_vec(Vec3::splat(f32::NAN), [2, 2, 2], Vec3::ONE, vec![0.0; 8]).err(),
        Some(ShapeError::NonFiniteParameter)
    );
    assert_eq!(
        LevelSet::from_vec(Vec3::ZERO, [2, 2, 2], Vec3::ONE, vec![f32::NAN; 8]).err(),
        Some(ShapeError::NonFiniteParameter)
    );
    assert_eq!(
        LevelSet::from_vec(
            Vec3::splat(f32::MAX),
            [2, 2, 2],
            Vec3::splat(f32::MAX),
            vec![0.0; 8],
        )
        .err(),
        Some(ShapeError::ArithmeticOverflow)
    );
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
fn skinned_triangle_mesh_rejects_invalid_bone_names(
    triangle_mesh: TriangleMesh,
    vertex_influences: Vec<Vec<BoneInfluence>>,
) {
    assert_eq!(
        SkinnedTriangleMesh::new(
            triangle_mesh,
            vec![" ".into()],
            vertex_influences,
            Affine3A::IDENTITY,
            vec![Affine3A::IDENTITY],
        )
        .err(),
        Some(ShapeError::InvalidBoneNames)
    );
}

#[rstest]
fn skinned_triangle_mesh_rejects_mismatched_arrays(
    triangle_mesh: TriangleMesh,
    vertex_influences: Vec<Vec<BoneInfluence>>,
) {
    assert_eq!(
        SkinnedTriangleMesh::new(
            triangle_mesh,
            vec!["root".into()],
            vertex_influences.into_iter().take(2).collect(),
            Affine3A::IDENTITY,
            vec![Affine3A::IDENTITY],
        )
        .err(),
        Some(ShapeError::DataLengthMismatch)
    );
}

#[rstest]
fn skinned_triangle_mesh_rejects_invalid_influence_indices(triangle_mesh: TriangleMesh) {
    let influences = (0..3)
        .map(|_| vec![BoneInfluence::new(1, 1.0).expect("valid influence weight")])
        .collect();

    assert_eq!(
        SkinnedTriangleMesh::new(
            triangle_mesh,
            vec!["root".into()],
            influences,
            Affine3A::IDENTITY,
            vec![Affine3A::IDENTITY],
        )
        .err(),
        Some(ShapeError::InvalidInfluenceIndex)
    );
}

#[rstest]
fn skinned_triangle_mesh_rejects_duplicate_influence_indices(triangle_mesh: TriangleMesh) {
    let influences = (0..3)
        .map(|_| {
            vec![
                BoneInfluence::new(0, 0.5).expect("valid influence weight"),
                BoneInfluence::new(0, 0.5).expect("valid influence weight"),
            ]
        })
        .collect();

    assert_eq!(
        SkinnedTriangleMesh::new(
            triangle_mesh,
            vec!["root".into()],
            influences,
            Affine3A::IDENTITY,
            vec![Affine3A::IDENTITY],
        )
        .err(),
        Some(ShapeError::InvalidInfluenceIndex)
    );
}

#[rstest]
fn skinned_triangle_mesh_rejects_invalid_weight_totals(triangle_mesh: TriangleMesh) {
    let influences = (0..3)
        .map(|_| vec![BoneInfluence::new(0, 0.5).expect("valid influence weight")])
        .collect();

    assert_eq!(
        SkinnedTriangleMesh::new(
            triangle_mesh,
            vec!["root".into()],
            influences,
            Affine3A::IDENTITY,
            vec![Affine3A::IDENTITY],
        )
        .err(),
        Some(ShapeError::InvalidWeight)
    );
}

#[rstest]
fn skinned_triangle_mesh_rejects_non_finite_transforms(
    triangle_mesh: TriangleMesh,
    vertex_influences: Vec<Vec<BoneInfluence>>,
) {
    assert_eq!(
        SkinnedTriangleMesh::new(
            triangle_mesh,
            vec!["root".into()],
            vertex_influences,
            Affine3A::from_translation(Vec3::splat(f32::NAN)),
            vec![Affine3A::IDENTITY],
        )
        .err(),
        Some(ShapeError::NonFiniteTransform)
    );
}

#[rstest]
#[case::negative_weight(-1.0)]
#[case::weight_above_one(1.5)]
#[case::non_finite(f32::NAN)]
fn bone_influence_rejects_invalid_weight(#[case] weight: f32) {
    assert_eq!(
        BoneInfluence::new(0, weight).err(),
        Some(ShapeError::InvalidWeight)
    );
}

#[rstest]
#[case::small(0.5)]
#[case::large(2.0)]
#[case::maximum(f32::MAX)]
fn sphere_local_bounds(#[case] radius: f32) {
    let sphere = Sphere::new(radius).expect("valid sphere");
    let extent = Vec3::splat(radius);

    assert_eq!(sphere.local_bounds(), Ok(expected_aabb(-extent, extent)));
}

#[test]
fn cuboid_local_bounds() {
    let cuboid = Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid");

    assert_eq!(
        cuboid.local_bounds(),
        Ok(expected_aabb(
            Vec3::new(-1.0, -2.0, -3.0),
            Vec3::new(1.0, 2.0, 3.0),
        ))
    );
}

#[test]
fn cuboid_local_bounds_with_maximum_size() {
    let cuboid = Cuboid::from_vec(Vec3::splat(f32::MAX)).expect("valid cuboid");

    let half = Vec3::splat(f32::MAX * 0.5);

    assert_eq!(cuboid.local_bounds(), Ok(expected_aabb(-half, half)));
}

#[test]
fn capsule_local_bounds() {
    let capsule = Capsule::new(Vec3::new(1.0, -2.0, 3.0), Vec3::new(-4.0, 5.0, -6.0), 0.5)
        .expect("valid capsule");

    assert_eq!(
        capsule.local_bounds(),
        Ok(expected_aabb(
            Vec3::new(-4.5, -2.5, -6.5),
            Vec3::new(1.5, 5.5, 3.5),
        ))
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

    assert_eq!(
        capsule.local_bounds(),
        Err(BoundsError::InvalidBounds(AabbError::NonFiniteBounds))
    );
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

    assert_eq!(cylinder.local_bounds(), Ok(expected_aabb(min, max)));
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

    assert_eq!(capsule.local_bounds(), Ok(expected_aabb(min, max)));
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

    assert_eq!(cylinder.local_bounds(), Ok(expected_aabb(min, max)));
}

#[rstest]
fn cylindrical_local_bounds_reject_overflow() {
    let start = Vec3::new(f32::MAX, 0.0, 0.0);
    let end = Vec3::new(f32::MAX, 0.0, 1.0);

    let cylinder = Cylinder::new(start, end, f32::MAX).expect("valid cylinder");
    assert_eq!(
        cylinder.local_bounds(),
        Err(BoundsError::InvalidBounds(AabbError::NonFiniteBounds))
    );

    let tapered_capsule =
        TaperedCapsule::new(start, end, f32::MAX, f32::MAX).expect("valid tapered capsule");
    assert_eq!(
        tapered_capsule.local_bounds(),
        Err(BoundsError::InvalidBounds(AabbError::NonFiniteBounds))
    );

    let tapered_cylinder =
        TaperedCylinder::new(start, end, f32::MAX, f32::MAX).expect("valid tapered cylinder");
    assert_eq!(
        tapered_cylinder.local_bounds(),
        Err(BoundsError::InvalidBounds(AabbError::NonFiniteBounds))
    );
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

    assert_eq!(triangle.local_bounds(), Ok(expected_aabb(min, max)));
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
        Ok(expected_aabb(
            Vec3::new(-3.0, -4.0, -5.0),
            Vec3::splat(-1.0),
        ))
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
        Ok(expected_aabb(
            Vec3::new(-4.0, -3.0, -2.0),
            Vec3::new(9.0, 8.0, 15.0),
        ))
    );
}

#[rstest]
fn height_field_local_bounds_use_grid_extent_and_height_range() {
    let heights = vec![-4.0, 2.0, -1.0, 6.0, 3.0, 5.0];
    let height_field =
        HeightField::from_vec(3, 2, Vec2::new(2.0, 0.5), heights).expect("valid height field");

    assert_eq!(
        height_field.local_bounds(),
        Ok(expected_aabb(
            Vec3::new(0.0, 0.0, -4.0),
            Vec3::new(2.0, 1.0, 6.0),
        ))
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
        Ok(expected_aabb(
            Vec3::new(-3.0, 2.0, -5.0),
            Vec3::new(-2.0, 4.0, 4.0),
        ))
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
        assert_eq!(shape.local_bounds(), Ok(Some(expected_aabb(min, max))));
    }
}

#[rstest]
fn shape_local_bounds_returns_none_for_plane() {
    let plane = Plane::new(0.0, 1.0, 0.0, 0.0).expect("valid plane");

    assert_eq!(Shape::Plane(plane).local_bounds(), Ok(None));
}

#[rstest]
fn shape_local_bounds_reports_unimplemented_skinned_mesh_bounds(
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

    assert_eq!(
        Shape::SkinnedTriangleMesh(skinned_mesh).local_bounds(),
        Err(BoundsError::NotImplemented)
    );
}

#[rstest]
fn shape_local_bounds_propagates_overflow() {
    let start = Vec3::new(f32::MAX, 0.0, 0.0);
    let end = Vec3::new(f32::MAX, 0.0, 1.0);
    let cylinder = Cylinder::new(start, end, f32::MAX).expect("valid cylinder");

    assert_eq!(
        Shape::Cylinder(cylinder).local_bounds(),
        Err(BoundsError::InvalidBounds(AabbError::NonFiniteBounds))
    );
}

#[test]
fn sphere_world_bounds_ignore_rotation() {
    let sphere = Shape::Sphere(Sphere::new(2.0).expect("valid sphere"));
    let transform = transform(Vec3::new(10.0, 5.0, -3.0), Quat::from_rotation_z(0.7));

    let bounds = sphere
        .world_bounds(&transform)
        .expect("valid bounds")
        .expect("sphere has finite bounds");
    let expected_min = MathVec3::new(8.0, 3.0, -5.0);
    let expected_max = MathVec3::new(12.0, 7.0, -1.0);

    assert!(bounds.min().x <= expected_min.x);
    assert!(bounds.min().y <= expected_min.y);
    assert!(bounds.min().z <= expected_min.z);
    assert!(bounds.max().x >= expected_max.x);
    assert!(bounds.max().y >= expected_max.y);
    assert!(bounds.max().z >= expected_max.z);
    assert!((bounds.min() - expected_min).abs().max_element() < 1.0e-14);
    assert!((bounds.max() - expected_max).abs().max_element() < 1.0e-14);
}

#[test]
fn sphere_world_bounds_round_outward() {
    let sphere = Shape::Sphere(Sphere::new(1.0).expect("valid sphere"));
    let transform = transform(Vec3::new(100_000_000.0, 0.0, 0.0), Quat::IDENTITY);

    let bounds = sphere
        .world_bounds(&transform)
        .expect("valid bounds")
        .expect("finite bounds");

    assert!(bounds.min().x <= 99_999_999.0);
    assert!(bounds.max().x >= 100_000_001.0);
    assert!(bounds.min().x > 99_999_998.999);
    assert!(bounds.max().x < 100_000_001.001);
}

#[test]
fn cuboid_world_bounds_apply_rotation_and_translation() {
    let cuboid = Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid"));
    let transform = transform(
        Vec3::new(10.0, 5.0, 0.0),
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
    );

    let bounds = cuboid
        .world_bounds(&transform)
        .expect("valid bounds")
        .expect("finite bounds");

    let expected_min = MathVec3::new(8.0, 4.0, -3.0);
    let expected_max = MathVec3::new(12.0, 6.0, 3.0);

    assert!(bounds.min().x <= expected_min.x);
    assert!(bounds.min().y <= expected_min.y);
    assert!(bounds.min().z <= expected_min.z);
    assert!(bounds.max().x >= expected_max.x);
    assert!(bounds.max().y >= expected_max.y);
    assert!(bounds.max().z >= expected_max.z);

    assert!((bounds.min() - expected_min).abs().max_element() < 1.0e-4);
    assert!((bounds.max() - expected_max).abs().max_element() < 1.0e-4);
}

#[test]
fn plane_world_bounds_are_unbounded() {
    let plane = Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).expect("valid plane"));

    assert_eq!(plane.world_bounds(&Transform::identity()), Ok(None));
}

#[test]
fn sphere_world_bounds_report_overflow() {
    let sphere = Shape::Sphere(Sphere::new(f32::MAX).expect("valid sphere"));
    let transform =
        Transform::new(MathVec3::splat(f64::MAX), MathQuat::IDENTITY).expect("valid transform");

    assert_eq!(
        sphere.world_bounds(&transform),
        Err(BoundsError::InvalidBounds(
            crate::aabb::AabbError::TransformOverflow
        ))
    );
}
