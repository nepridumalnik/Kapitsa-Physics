use glam::{Quat, Vec3};
use rstest::rstest;

use super::Transform;

fn assert_vec3_close(actual: Vec3, expected: Vec3) {
    const EPSILON: f32 = 1.0e-5;

    assert!(
        (actual - expected).abs().max_element() <= EPSILON,
        "expected {expected:?}, got {actual:?}"
    );
}

#[test]
fn identity_preserves_points_and_vectors() {
    let transform = Transform::identity();
    let value = Vec3::new(1.0, -2.0, 3.0);

    assert_eq!(transform.translation(), Vec3::ZERO);
    assert_eq!(transform.rotation(), Quat::IDENTITY);
    assert_eq!(transform.transform_point(value), value);
    assert_eq!(transform.transform_vector(value), value);
}

#[test]
fn stores_translation_and_rotation() {
    let translation = Vec3::new(1.0, 2.0, 3.0);
    let rotation = Quat::from_rotation_z(0.5);

    let transform = Transform::new(translation, rotation).expect("valid transform");

    assert_eq!(transform.translation(), translation);
    assert_eq!(transform.rotation(), rotation);
}

#[rstest]
#[case::nan_x(Vec3::new(f32::NAN, 0.0, 0.0))]
#[case::positive_infinity(Vec3::new(0.0, f32::INFINITY, 0.0))]
#[case::negative_infinity(Vec3::new(0.0, 0.0, f32::NEG_INFINITY))]
fn rejects_non_finite_translation(#[case] translation: Vec3) {
    assert!(Transform::new(translation, Quat::IDENTITY).is_none());
}

#[rstest]
#[case::zero(Quat::from_xyzw(0.0, 0.0, 0.0, 0.0))]
#[case::non_unit(Quat::from_xyzw(0.0, 0.0, 0.0, 2.0))]
#[case::nan(Quat::from_xyzw(0.0, 0.0, 0.0, f32::NAN))]
#[case::infinity(Quat::from_xyzw(0.0, 0.0, f32::INFINITY, 1.0))]
fn rejects_invalid_rotation(#[case] rotation: Quat) {
    assert!(Transform::new(Vec3::ZERO, rotation).is_none());
}

#[test]
fn transform_point_applies_translation() {
    let transform =
        Transform::new(Vec3::new(3.0, -2.0, 5.0), Quat::IDENTITY).expect("valid transform");

    assert_eq!(
        transform.transform_point(Vec3::new(1.0, 2.0, 3.0)),
        Vec3::new(4.0, 0.0, 8.0)
    );
}

#[test]
fn transform_point_applies_rotation_and_translation() {
    let transform = Transform::new(
        Vec3::new(10.0, 5.0, -2.0),
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
    )
    .expect("valid transform");

    assert_vec3_close(
        transform.transform_point(Vec3::X),
        Vec3::new(10.0, 6.0, -2.0),
    );
}

#[test]
fn transform_vector_ignores_translation() {
    let transform = Transform::new(
        Vec3::new(10.0, 20.0, 30.0),
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
    )
    .expect("valid transform");

    assert_vec3_close(transform.transform_vector(Vec3::X), Vec3::Y);

    assert_vec3_close(
        transform.transform_point(Vec3::X),
        Vec3::new(10.0, 21.0, 30.0),
    );
}

#[test]
fn inverse_of_identity_is_identity() {
    let inverse = Transform::identity()
        .inverse()
        .expect("invertible identity");

    assert_eq!(inverse, Transform::identity());
}

#[test]
fn inverse_reverses_translation() {
    let transform =
        Transform::new(Vec3::new(3.0, -4.0, 5.0), Quat::IDENTITY).expect("valid transform");

    let inverse = transform.inverse().expect("invertible transform");

    assert_eq!(inverse.translation(), Vec3::new(-3.0, 4.0, -5.0));
    assert_eq!(inverse.rotation(), Quat::IDENTITY);
}

#[test]
fn inverse_reverses_rotation_and_translation() {
    let transform = Transform::new(
        Vec3::new(2.0, 0.0, 0.0),
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
    )
    .expect("valid transform");

    let inverse = transform.inverse().expect("invertible transform");

    assert_vec3_close(inverse.translation(), Vec3::new(0.0, 2.0, 0.0));

    assert_vec3_close(inverse.transform_point(Vec3::new(2.0, 1.0, 0.0)), Vec3::X);
}

#[rstest]
#[case::origin(Vec3::ZERO)]
#[case::positive(Vec3::new(1.0, 2.0, 3.0))]
#[case::negative(Vec3::new(-4.0, -2.0, -1.0))]
#[case::mixed(Vec3::new(3.0, -5.0, 2.0))]
fn inverse_restores_original_point(#[case] point: Vec3) {
    let transform = Transform::new(Vec3::new(3.0, -5.0, 2.0), Quat::from_rotation_y(0.7))
        .expect("valid transform");

    let inverse = transform.inverse().expect("invertible transform");

    let transformed = transform.transform_point(point);
    let restored = inverse.transform_point(transformed);

    assert_vec3_close(restored, point);
}

#[test]
fn inverse_restores_original_vector() {
    let transform = Transform::new(Vec3::new(10.0, -5.0, 3.0), Quat::from_rotation_x(0.8))
        .expect("valid transform");

    let inverse = transform.inverse().expect("invertible transform");
    let vector = Vec3::new(2.0, -3.0, 4.0);

    assert_vec3_close(
        inverse.transform_vector(transform.transform_vector(vector)),
        vector,
    );
}

#[test]
fn inverse_rejects_overflow() {
    let transform = Transform::new(
        Vec3::new(f32::MAX, f32::MAX, 0.0),
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_4),
    )
    .expect("valid transform");

    assert!(transform.inverse().is_none());
}
