use crate::{
    collision::{
        CollisionError,
        gjk::{GjkResult, distance, intersect},
    },
    shape::{Capsule, Cuboid, Shape, Sphere},
    transform::Transform,
};
use kapitsa_math::Vec3;

#[test]
fn gjk_detects_separated_and_overlapping_spheres() {
    let sphere = Shape::Sphere(Sphere::new(1.0).expect("sphere"));
    let first = Transform::identity();
    let separated =
        Transform::new(Vec3::new(3.0, 0.0, 0.0), kapitsa_math::Quat::IDENTITY).expect("transform");
    assert!(matches!(
        intersect(&sphere, &first, &sphere, &separated),
        Ok(GjkResult {
            intersects: false,
            ..
        })
    ));

    let overlapping =
        Transform::new(Vec3::new(1.5, 0.0, 0.0), kapitsa_math::Quat::IDENTITY).expect("transform");
    assert!(matches!(
        intersect(&sphere, &first, &sphere, &overlapping),
        Ok(GjkResult {
            intersects: true,
            ..
        })
    ));
}

#[test]
fn gjk_distance_reports_witnesses_and_handles_intersection() {
    let sphere = Shape::Sphere(Sphere::new(1.0).expect("sphere"));
    let first = Transform::identity();
    let separated =
        Transform::new(Vec3::new(5.0, 0.0, 0.0), kapitsa_math::Quat::IDENTITY).expect("transform");
    let result = distance(&sphere, &first, &sphere, &separated).expect("distance");
    assert!(!result.intersects);
    assert!((result.distance - 3.0).abs() < 1.0e-12);
    assert!((result.normal - Vec3::X).length() < 1.0e-12);
    assert!((result.point_a - Vec3::X).length() < 1.0e-12);
    assert!((result.point_b - Vec3::new(4.0, 0.0, 0.0)).length() < 1.0e-12);

    let overlapping =
        Transform::new(Vec3::new(1.5, 0.0, 0.0), kapitsa_math::Quat::IDENTITY).expect("transform");
    let result = distance(&sphere, &first, &sphere, &overlapping).expect("overlap");
    assert!(result.intersects);
    assert_eq!(result.distance, 0.0);
}

#[test]
fn gjk_distance_tolerance_is_independent_of_world_translation() {
    let sphere = Shape::Sphere(Sphere::new(1.0).expect("sphere"));
    let first = Transform::new(Vec3::new(1.0e12, 0.0, 0.0), kapitsa_math::Quat::IDENTITY)
        .expect("transform");
    let second = Transform::new(
        Vec3::new(1.0e12 + 5.0, 0.0, 0.0),
        kapitsa_math::Quat::IDENTITY,
    )
    .expect("transform");
    let result = distance(&sphere, &first, &sphere, &second).expect("distance");
    assert!(!result.intersects);
    assert!((result.distance - 3.0).abs() < 1.0e-3);
}

#[test]
fn gjk_handles_rotated_polyhedra_and_reports_bad_geometry() {
    let cuboid = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("cuboid"));
    let first = Transform::identity();
    let second = Transform::new(
        Vec3::new(1.0, 0.0, 0.0),
        kapitsa_math::Quat::from_axis_angle(Vec3::Z, 0.35).expect("rotation"),
    )
    .expect("transform");
    assert!(matches!(
        intersect(&cuboid, &first, &cuboid, &second),
        Ok(GjkResult {
            intersects: true,
            ..
        })
    ));

    let invalid = Transform::new(Vec3::new(f64::MAX, 0.0, 0.0), kapitsa_math::Quat::IDENTITY)
        .expect("large transform");
    let sphere = Shape::Sphere(crate::shape::Sphere::new(f64::MAX).expect("sphere"));
    assert!(matches!(
        intersect(&sphere, &invalid, &sphere, &first),
        Err(CollisionError::ArithmeticOverflow)
    ));
}

#[test]
fn identical_convexes_and_nested_sphere_produce_finite_overlap_results() {
    let cuboid = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("cuboid"));
    let sphere = Shape::Sphere(Sphere::new(0.25).expect("sphere"));
    let identity = Transform::identity();
    let identical = intersect(&cuboid, &identity, &cuboid, &identity).expect("identical GJK");
    assert!(identical.intersects);
    assert!((1..=4).contains(&identical.simplex.count));

    let nested = distance(&cuboid, &identity, &sphere, &identity).expect("nested distance");
    assert!(nested.intersects);
    assert_eq!(nested.distance, 0.0);
    assert!(nested.point_a.is_finite() && nested.point_b.is_finite());
    assert!(nested.normal.is_finite());
}

#[test]
fn gjk_separates_near_touching_boxes_and_reports_touching_consistently() {
    let box_a = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("box A"));
    let box_b = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("box B"));
    let identity = Transform::identity();
    let separated = Transform::new(
        Vec3::new(2.0 + 1.0e-7, 0.0, 0.0),
        kapitsa_math::Quat::IDENTITY,
    )
    .expect("separated transform");
    let gap = distance(&box_a, &identity, &box_b, &separated).expect("separation");
    assert!(!gap.intersects);
    assert!((gap.distance - 1.0e-7).abs() < 2.0e-8);
    assert!((gap.normal - Vec3::X).length() < 1.0e-8);
    assert!(gap.point_a.is_finite() && gap.point_b.is_finite());

    let touching = Transform::new(Vec3::new(2.0, 0.0, 0.0), kapitsa_math::Quat::IDENTITY)
        .expect("touching transform");
    let result = distance(&box_a, &identity, &box_b, &touching).expect("touching distance");
    assert!(result.distance <= 1.0e-12);
    assert!(result.point_a.is_finite() && result.point_b.is_finite());
    assert!(result.normal.is_finite());
}

#[test]
fn gjk_handles_thin_boxes_perpendicular_capsules_and_degenerate_initial_directions() {
    let thin = Shape::Cuboid(Cuboid::new(2.0, 1.0e-6, 1.0e-6).expect("thin box"));
    let overlapping_thin = Transform::new(Vec3::new(1.9, 0.0, 0.0), kapitsa_math::Quat::IDENTITY)
        .expect("thin overlap transform");
    let thin_result =
        intersect(&thin, &Transform::identity(), &thin, &overlapping_thin).expect("thin-box GJK");
    assert!(thin_result.intersects);

    let horizontal = Shape::Capsule(
        Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.25)
            .expect("horizontal capsule"),
    );
    let vertical = Shape::Capsule(
        Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.25)
            .expect("vertical capsule"),
    );
    let crossed = intersect(
        &horizontal,
        &Transform::identity(),
        &vertical,
        &Transform::identity(),
    )
    .expect("perpendicular capsule GJK");
    assert!(crossed.intersects);
    assert!((1..=4).contains(&crossed.simplex.count));
}

#[test]
fn gjk_distance_remains_finite_for_large_and_small_coordinates() {
    let large_sphere = Shape::Sphere(Sphere::new(1.0).expect("large sphere"));
    let large_a = Transform::new(Vec3::splat(1.0e12), kapitsa_math::Quat::IDENTITY)
        .expect("large transform A");
    let large_b = Transform::new(
        Vec3::new(1.0e12 + 5.0, 1.0e12, 1.0e12),
        kapitsa_math::Quat::IDENTITY,
    )
    .expect("large transform B");
    let large = distance(&large_sphere, &large_a, &large_sphere, &large_b).expect("large GJK");
    assert!(!large.intersects);
    assert!((large.distance - 3.0).abs() < 1.0e-3);
    assert!(large.normal.is_finite() && large.point_a.is_finite() && large.point_b.is_finite());

    let small_sphere = Shape::Sphere(Sphere::new(1.0e-12).expect("small sphere"));
    let small_offset = Transform::new(Vec3::new(5.0e-12, 0.0, 0.0), kapitsa_math::Quat::IDENTITY)
        .expect("small transform");
    let small = distance(
        &small_sphere,
        &Transform::identity(),
        &small_sphere,
        &small_offset,
    )
    .expect("small GJK");
    assert!(!small.intersects);
    assert!((small.distance - 3.0e-12).abs() < 1.0e-14);
    assert!(small.normal.is_finite() && small.point_a.is_finite() && small.point_b.is_finite());
}
