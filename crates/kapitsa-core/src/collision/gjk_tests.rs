use crate::{
    collision::{
        CollisionError,
        gjk::{GjkResult, distance, intersect},
    },
    shape::{Cuboid, Shape, Sphere},
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
