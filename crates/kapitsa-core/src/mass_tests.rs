use glam::{Mat3, Quat, Vec3};
use kapitsa_math::{Quat as MathQuat, Vec3 as MathVec3};

use super::{MassError, MassProperties};
use crate::shape::{Cuboid, Plane, Shape, Sphere};
use crate::transform::Transform;

const EPSILON: f32 = 1.0e-5;

fn assert_close(actual: f32, expected: f32) {
    let tolerance = EPSILON * expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected}, got {actual}"
    );
}

fn properties(mass: f32, center: Vec3, inertia: Mat3) -> MassProperties {
    MassProperties::new(mass, center, inertia).expect("valid mass properties")
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

fn assert_vec3_close(actual: Vec3, expected: Vec3) {
    assert_close(actual.x, expected.x);
    assert_close(actual.y, expected.y);
    assert_close(actual.z, expected.z);
}

fn assert_mat3_close(actual: Mat3, expected: Mat3) {
    for (actual, expected) in actual
        .to_cols_array()
        .into_iter()
        .zip(expected.to_cols_array())
    {
        assert_close(actual, expected);
    }
}

#[test]
fn creates_valid_mass_properties() -> Result<(), MassError> {
    let center = Vec3::new(1.0, -2.0, 3.0);
    let inertia = Mat3::from_cols(
        Vec3::new(2.0, 0.25, 0.0),
        Vec3::new(0.25, 3.0, 0.0),
        Vec3::new(0.0, 0.0, 4.0),
    );

    let properties = MassProperties::new(2.5, center, inertia)?;

    assert_eq!(properties.mass(), 2.5);
    assert_eq!(properties.center_of_mass(), center);
    assert_eq!(properties.inertia(), inertia);
    Ok(())
}

#[test]
fn rejects_invalid_mass_values() {
    for (mass, expected) in [
        (-1.0, MassError::NonPositiveMass),
        (0.0, MassError::NonPositiveMass),
        (f32::INFINITY, MassError::NonFiniteMass),
        (f32::NAN, MassError::NonFiniteMass),
    ] {
        assert_eq!(
            MassProperties::new(mass, Vec3::ZERO, Mat3::IDENTITY),
            Err(expected)
        );
    }
}

#[test]
fn rejects_non_finite_center_of_mass() {
    assert_eq!(
        MassProperties::new(1.0, Vec3::new(f32::NAN, 0.0, 0.0), Mat3::IDENTITY),
        Err(MassError::NonFiniteCenterOfMass)
    );
    assert_eq!(
        MassProperties::new(1.0, Vec3::new(0.0, f32::INFINITY, 0.0), Mat3::IDENTITY),
        Err(MassError::NonFiniteCenterOfMass)
    );
}

#[test]
fn rejects_non_finite_inertia() {
    let inertia = Mat3::from_diagonal(Vec3::new(1.0, f32::INFINITY, 1.0));

    assert_eq!(
        MassProperties::new(1.0, Vec3::ZERO, inertia),
        Err(MassError::NonFiniteInertia)
    );
}

#[test]
fn rejects_asymmetric_and_non_positive_definite_inertia() {
    let asymmetric = Mat3::from_cols(
        Vec3::new(2.0, 0.2, 0.0),
        Vec3::new(0.4, 3.0, 0.0),
        Vec3::new(0.0, 0.0, 4.0),
    );
    let invalid = Mat3::from_diagonal(Vec3::new(1.0, -1.0, 1.0));

    assert_eq!(
        MassProperties::new(1.0, Vec3::ZERO, asymmetric),
        Err(MassError::NonSymmetricInertia)
    );
    assert_eq!(
        MassProperties::new(1.0, Vec3::ZERO, invalid),
        Err(MassError::NonPositiveDefiniteInertia)
    );
}

#[test]
fn accepts_f32_rounding_in_symmetric_components() -> Result<(), MassError> {
    let inertia = Mat3::from_cols(
        Vec3::new(2.0, 0.2, 0.0),
        Vec3::new(0.2 + f32::EPSILON, 3.0, 0.0),
        Vec3::new(0.0, 0.0, 4.0),
    );

    let properties = MassProperties::new(1.0, Vec3::ZERO, inertia)?;
    assert_eq!(properties.inertia().x_axis.y, properties.inertia().y_axis.x);
    Ok(())
}

#[test]
fn rejects_positive_definite_but_physically_impossible_inertia() {
    let inertia = Mat3::from_diagonal(Vec3::new(10.0, 1.0, 1.0));

    assert_eq!(
        MassProperties::new(1.0, Vec3::ZERO, inertia),
        Err(MassError::NonPhysicalInertia)
    );
}

#[test]
fn accepts_physical_inertia_boundaries() -> Result<(), MassError> {
    for inertia in [
        Mat3::from_diagonal(Vec3::new(2.0, 2.0, 4.0)),
        Mat3::IDENTITY,
    ] {
        MassProperties::new(1.0, Vec3::ZERO, inertia)?;
    }
    Ok(())
}

#[test]
fn physical_inertia_validation_is_scale_independent() -> Result<(), MassError> {
    let smallest = f32::from_bits(1);
    let largest = f32::MAX / 4.0;

    for scale in [smallest, largest] {
        let inertia = Mat3::from_diagonal(Vec3::new(scale, scale, 2.0 * scale));
        MassProperties::new(1.0, Vec3::ZERO, inertia)?;
    }
    Ok(())
}

#[test]
fn computes_unit_sphere_mass_and_inertia() -> Result<(), MassError> {
    let shape = Shape::Sphere(Sphere::new(1.0).expect("valid unit sphere"));
    let properties = MassProperties::from_shape(&shape, 1.0)?;
    let expected_mass = (4.0 / 3.0) * std::f32::consts::PI;
    let expected_inertia = (2.0 / 5.0) * expected_mass;

    assert_close(properties.mass(), expected_mass);
    assert_eq!(properties.center_of_mass(), Vec3::ZERO);
    assert_close(properties.inertia().x_axis.x, expected_inertia);
    assert_close(properties.inertia().y_axis.y, expected_inertia);
    assert_close(properties.inertia().z_axis.z, expected_inertia);
    Ok(())
}

#[test]
fn computes_cuboid_mass_and_inertia_for_different_side_lengths() -> Result<(), MassError> {
    let shape = Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid"));
    let properties = MassProperties::from_shape(&shape, 2.0)?;
    let inertia = properties.inertia();

    assert_close(properties.mass(), 96.0);
    assert_close(inertia.x_axis.x, 416.0);
    assert_close(inertia.y_axis.y, 320.0);
    assert_close(inertia.z_axis.z, 160.0);
    assert_eq!(properties.center_of_mass(), Vec3::ZERO);
    Ok(())
}

#[test]
fn density_scales_sphere_mass_and_inertia() -> Result<(), MassError> {
    let shape = Shape::Sphere(Sphere::new(1.0).expect("valid sphere"));
    let unit_density = MassProperties::from_shape(&shape, 1.0)?;
    let triple_density = MassProperties::from_shape(&shape, 3.0)?;

    assert_close(triple_density.mass(), unit_density.mass() * 3.0);
    assert_close(
        triple_density.inertia().x_axis.x,
        unit_density.inertia().x_axis.x * 3.0,
    );
    Ok(())
}

#[test]
fn rejects_invalid_density() {
    let shape = Shape::Sphere(Sphere::new(1.0).expect("valid sphere"));

    assert_eq!(
        MassProperties::from_shape(&shape, f32::NAN),
        Err(MassError::NonFiniteDensity)
    );
    assert_eq!(
        MassProperties::from_shape(&shape, 0.0),
        Err(MassError::NonPositiveDensity)
    );
    assert_eq!(
        MassProperties::from_shape(&shape, -1.0),
        Err(MassError::NonPositiveDensity)
    );
}

#[test]
fn rejects_overflow_and_positive_mass_underflow() {
    let large_sphere = Shape::Sphere(Sphere::new(f64::MAX).expect("finite sphere"));
    let large_cuboid =
        Shape::Cuboid(Cuboid::new(f64::MAX, f64::MAX, f64::MAX).expect("finite cuboid"));
    let tiny_sphere =
        Shape::Sphere(Sphere::new(f64::from_bits(1)).expect("positive subnormal sphere"));

    assert_eq!(
        MassProperties::from_shape(&large_sphere, f32::MAX),
        Err(MassError::ArithmeticOverflow)
    );
    assert_eq!(
        MassProperties::from_shape(&large_cuboid, f32::MAX),
        Err(MassError::ArithmeticOverflow)
    );
    assert_eq!(
        MassProperties::from_shape(&tiny_sphere, f32::from_bits(1)),
        Err(MassError::ArithmeticOverflow)
    );
}

#[test]
fn rejects_unsupported_shapes() {
    let shape = Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).expect("valid plane"));

    assert_eq!(
        MassProperties::from_shape(&shape, 1.0),
        Err(MassError::UnsupportedShape)
    );
}

#[test]
fn combines_identical_bodies_at_identical_transforms() -> Result<(), MassError> {
    let body = properties(
        2.0,
        Vec3::ZERO,
        Mat3::from_diagonal(Vec3::new(1.0, 2.0, 3.0)),
    );
    let placement = transform(Vec3::new(3.0, -2.0, 1.0), Quat::from_rotation_z(0.4));
    let single = MassProperties::combine(&[(body, placement)])?;
    let combined = MassProperties::combine(&[(body, placement), (body, placement)])?;

    assert_close(combined.mass(), 2.0 * body.mass());
    assert_vec3_close(combined.center_of_mass(), Vec3::new(3.0, -2.0, 1.0));
    assert_mat3_close(combined.inertia(), single.inertia() * 2.0);
    Ok(())
}

#[test]
fn combines_symmetric_spheres_about_the_origin() -> Result<(), MassError> {
    let shape = Shape::Sphere(Sphere::new(1.0).expect("valid sphere"));
    let sphere = MassProperties::from_shape(&shape, 1.0)?;
    let left = transform(-2.0 * Vec3::X, Quat::IDENTITY);
    let right = transform(2.0 * Vec3::X, Quat::IDENTITY);

    let combined = MassProperties::combine(&[(sphere, left), (sphere, right)])?;
    let sphere_inertia = sphere.inertia().x_axis.x;
    let expected_transverse = 2.0 * sphere_inertia + 8.0 * sphere.mass();

    assert_vec3_close(combined.center_of_mass(), Vec3::ZERO);
    assert_close(combined.inertia().x_axis.x, 2.0 * sphere_inertia);
    assert_close(combined.inertia().y_axis.y, expected_transverse);
    assert_close(combined.inertia().z_axis.z, expected_transverse);
    Ok(())
}

#[test]
fn combines_different_masses_at_the_expected_center() -> Result<(), MassError> {
    let first = properties(1.0, Vec3::ZERO, Mat3::IDENTITY);
    let second = properties(3.0, Vec3::ZERO, Mat3::IDENTITY);
    let parts = [
        (first, Transform::identity()),
        (second, transform(4.0 * Vec3::X, Quat::IDENTITY)),
    ];

    let combined = MassProperties::combine(&parts)?;

    assert_close(combined.mass(), 4.0);
    assert_vec3_close(combined.center_of_mass(), 3.0 * Vec3::X);
    assert_close(combined.inertia().x_axis.x, 2.0);
    assert_close(combined.inertia().y_axis.y, 14.0);
    assert_close(combined.inertia().z_axis.z, 14.0);
    Ok(())
}

#[test]
fn applies_the_parallel_axis_theorem() -> Result<(), MassError> {
    let body = properties(2.0, Vec3::ZERO, Mat3::IDENTITY);
    let offset = 1.5 * Vec3::X;
    let combined = MassProperties::combine(&[
        (body, transform(-offset, Quat::IDENTITY)),
        (body, transform(offset, Quat::IDENTITY)),
    ])?;

    assert_vec3_close(combined.center_of_mass(), Vec3::ZERO);
    assert_close(combined.inertia().x_axis.x, 2.0);
    assert_close(combined.inertia().y_axis.y, 11.0);
    assert_close(combined.inertia().z_axis.z, 11.0);
    Ok(())
}

#[test]
fn rotates_cuboid_inertia_by_ninety_degrees() -> Result<(), MassError> {
    let shape = Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0).expect("valid cuboid"));
    let cuboid = MassProperties::from_shape(&shape, 2.0)?;
    let rotation = transform(
        Vec3::ZERO,
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
    );
    let combined = MassProperties::combine(&[(cuboid, rotation)])?;

    assert_close(combined.inertia().x_axis.x, 320.0);
    assert_close(combined.inertia().y_axis.y, 416.0);
    assert_close(combined.inertia().z_axis.z, 160.0);
    Ok(())
}

#[test]
fn arbitrary_rotation_creates_off_diagonal_inertia() -> Result<(), MassError> {
    let body = properties(
        1.0,
        Vec3::ZERO,
        Mat3::from_diagonal(Vec3::new(1.0, 2.0, 3.0)),
    );
    let rotation = transform(
        Vec3::ZERO,
        Quat::from_rotation_z(std::f32::consts::FRAC_PI_4),
    );
    let combined = MassProperties::combine(&[(body, rotation)])?;

    assert_close(combined.inertia().x_axis.x, 1.5);
    assert_close(combined.inertia().y_axis.y, 1.5);
    assert_close(combined.inertia().z_axis.z, 3.0);
    assert_close(combined.inertia().x_axis.y, -0.5);
    assert_close(combined.inertia().y_axis.x, -0.5);
    Ok(())
}

#[test]
fn push_matches_combine_without_modifying_parts() -> Result<(), MassError> {
    let first = properties(
        2.0,
        Vec3::new(0.2, -0.5, 0.1),
        Mat3::from_diagonal(Vec3::new(1.0, 2.0, 3.0)),
    );
    let second = properties(
        3.0,
        Vec3::new(-0.4, 0.2, 0.1),
        Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0)),
    );
    let transform = transform(Vec3::new(2.0, -1.0, 0.5), Quat::from_rotation_y(0.7));
    let original_transform = transform;
    let initial = first;
    let part = second;
    let mut pushed = first;

    pushed.push(&second, &transform)?;
    let combined = MassProperties::combine(&[(first, Transform::identity()), (second, transform)])?;

    assert_close(pushed.mass(), combined.mass());
    assert_vec3_close(pushed.center_of_mass(), combined.center_of_mass());
    assert_mat3_close(pushed.inertia(), combined.inertia());
    assert_eq!(first, initial);
    assert_eq!(second, part);
    assert_eq!(transform, original_transform);
    Ok(())
}

#[test]
fn set_combined_replaces_existing_properties() -> Result<(), MassError> {
    let first = properties(1.0, Vec3::ZERO, Mat3::IDENTITY);
    let second = properties(2.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::splat(2.0)));
    let parts = [
        (first, Transform::identity()),
        (second, transform(2.0 * Vec3::Y, Quat::IDENTITY)),
    ];
    let expected = MassProperties::combine(&parts)?;
    let mut actual = properties(5.0, Vec3::X, Mat3::from_diagonal(Vec3::splat(5.0)));

    actual.set_combined(&parts)?;

    assert_eq!(actual, expected);
    Ok(())
}

#[test]
fn empty_composition_and_failed_updates_preserve_existing_properties() {
    let mut properties = properties(3.0, Vec3::X, Mat3::from_diagonal(Vec3::splat(2.0)));
    let original = properties;

    assert_eq!(
        MassProperties::combine(&[]),
        Err(MassError::EmptyComposition)
    );
    assert_eq!(
        properties.set_combined(&[]),
        Err(MassError::EmptyComposition)
    );
    assert_eq!(properties, original);
}

#[test]
fn overflow_does_not_modify_the_existing_properties() {
    let large = properties(f32::MAX, Vec3::ZERO, Mat3::IDENTITY);
    let mut accumulated = large;

    assert_eq!(
        accumulated.push(&large, &Transform::identity()),
        Err(MassError::ArithmeticOverflow)
    );
    assert_eq!(accumulated, large);

    let ordinary = properties(1.0, Vec3::ZERO, Mat3::IDENTITY);
    let far_away = transform(Vec3::splat(f32::MAX), Quat::IDENTITY);
    assert_eq!(
        MassProperties::combine(&[(ordinary, Transform::identity()), (ordinary, far_away)]),
        Err(MassError::ArithmeticOverflow)
    );

    let mut replacement = ordinary;
    assert_eq!(
        replacement.set_combined(&[
            (large, Transform::identity()),
            (large, Transform::identity()),
        ]),
        Err(MassError::ArithmeticOverflow)
    );
    assert_eq!(replacement, ordinary);
}
