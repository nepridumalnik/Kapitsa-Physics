use glam::{Mat3, Vec3};

use super::{MassError, MassProperties};
use crate::shape::{Cuboid, Plane, Shape, Sphere};

const EPSILON: f32 = 1.0e-5;

fn assert_close(actual: f32, expected: f32) {
    let tolerance = EPSILON * expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected}, got {actual}"
    );
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

    MassProperties::new(1.0, Vec3::ZERO, inertia)?;
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
    let large_sphere = Shape::Sphere(Sphere::new(f32::MAX).expect("finite sphere"));
    let large_cuboid =
        Shape::Cuboid(Cuboid::new(f32::MAX, f32::MAX, f32::MAX).expect("finite cuboid"));
    let tiny_sphere =
        Shape::Sphere(Sphere::new(f32::from_bits(1)).expect("positive subnormal sphere"));

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
