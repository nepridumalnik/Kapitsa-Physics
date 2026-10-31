use super::{PhysicsMaterial, PhysicsMaterialError};

#[test]
fn creates_material_with_valid_parameters() -> Result<(), PhysicsMaterialError> {
    let material = PhysicsMaterial::new(0.8, 0.3, 0.6, 2.5)?;

    assert_eq!(material.friction(), 0.8);
    assert_eq!(material.static_friction(), 0.3);
    assert_eq!(material.restitution(), 0.6);
    assert_eq!(material.density(), 2.5);
    Ok(())
}

#[test]
fn default_uses_chaos_material_values() {
    let material = PhysicsMaterial::default();

    assert_eq!(material.friction(), 0.5);
    assert_eq!(material.static_friction(), 0.0);
    assert_eq!(material.restitution(), 0.1);
    assert_eq!(material.density(), 1.0);
}

#[test]
fn rejects_nan_for_each_parameter() {
    assert_eq!(
        PhysicsMaterial::new(f32::NAN, 0.0, 0.5, 1.0),
        Err(PhysicsMaterialError::NonFiniteFriction)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, f32::NAN, 0.5, 1.0),
        Err(PhysicsMaterialError::NonFiniteStaticFriction)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, f32::NAN, 1.0),
        Err(PhysicsMaterialError::NonFiniteRestitution)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, 0.5, f32::NAN),
        Err(PhysicsMaterialError::NonFiniteDensity)
    );
}

#[test]
fn rejects_positive_and_negative_infinity_for_each_parameter() {
    let cases = [
        (
            [f32::INFINITY, 0.0, 0.5, 1.0],
            PhysicsMaterialError::NonFiniteFriction,
        ),
        (
            [f32::NEG_INFINITY, 0.0, 0.5, 1.0],
            PhysicsMaterialError::NonFiniteFriction,
        ),
        (
            [0.5, f32::INFINITY, 0.5, 1.0],
            PhysicsMaterialError::NonFiniteStaticFriction,
        ),
        (
            [0.5, f32::NEG_INFINITY, 0.5, 1.0],
            PhysicsMaterialError::NonFiniteStaticFriction,
        ),
        (
            [0.5, 0.0, f32::INFINITY, 1.0],
            PhysicsMaterialError::NonFiniteRestitution,
        ),
        (
            [0.5, 0.0, f32::NEG_INFINITY, 1.0],
            PhysicsMaterialError::NonFiniteRestitution,
        ),
        (
            [0.5, 0.0, 0.5, f32::INFINITY],
            PhysicsMaterialError::NonFiniteDensity,
        ),
        (
            [0.5, 0.0, 0.5, f32::NEG_INFINITY],
            PhysicsMaterialError::NonFiniteDensity,
        ),
    ];

    for (parameters, expected) in cases {
        assert_eq!(
            PhysicsMaterial::new(parameters[0], parameters[1], parameters[2], parameters[3]),
            Err(expected)
        );
    }
}

#[test]
fn rejects_negative_friction_coefficients() {
    assert_eq!(
        PhysicsMaterial::new(-0.1, 0.0, 0.5, 1.0),
        Err(PhysicsMaterialError::NegativeFriction)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, -0.1, 0.5, 1.0),
        Err(PhysicsMaterialError::NegativeStaticFriction)
    );
}

#[test]
fn rejects_restitution_outside_unit_interval() {
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, -0.1, 1.0),
        Err(PhysicsMaterialError::RestitutionOutOfRange)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, 1.1, 1.0),
        Err(PhysicsMaterialError::RestitutionOutOfRange)
    );
}

#[test]
fn rejects_non_positive_density() {
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, 0.5, 0.0),
        Err(PhysicsMaterialError::NonPositiveDensity)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, 0.5, -1.0),
        Err(PhysicsMaterialError::NonPositiveDensity)
    );
}

#[test]
fn accepts_boundaries_without_friction_ordering() -> Result<(), PhysicsMaterialError> {
    let material = PhysicsMaterial::new(0.8, 0.0, 0.0, f32::MIN_POSITIVE)?;
    let maximum_restitution = PhysicsMaterial::new(0.0, 0.5, 1.0, 1.0)?;

    assert_eq!(material.friction(), 0.8);
    assert_eq!(material.static_friction(), 0.0);
    assert_eq!(material.restitution(), 0.0);
    assert_eq!(material.density(), f32::MIN_POSITIVE);
    assert_eq!(maximum_restitution.restitution(), 1.0);
    Ok(())
}
