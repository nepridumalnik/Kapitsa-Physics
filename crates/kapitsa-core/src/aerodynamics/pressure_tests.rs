use super::*;
use kapitsa_math::Quat;

fn flow() -> PressureFlow {
    PressureFlow {
        atmosphere: AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
        sound_speed: 100.0,
        transform: Transform::identity(),
        center_of_mass: Vec3::ZERO,
        linear_velocity: Vec3::X * 1000.0,
        angular_velocity: Vec3::ZERO,
    }
}

#[test]
fn explicit_base_pressure_has_correct_force_moment_and_domain() {
    let base = BasePressure::new(-0.4, 5.0, 10.0).unwrap();
    let patch = PressurePatch::new(Vec3::Y, -Vec3::X, 2.0).unwrap();
    let load = patch.evaluate_base(flow(), base).unwrap();
    assert_eq!(load.force, -Vec3::X * 400_000.0);
    assert_eq!(load.moment, Vec3::Z * 400_000.0);
    assert!(
        patch
            .evaluate_base(
                PressureFlow {
                    linear_velocity: Vec3::X * 500.0,
                    ..flow()
                },
                base
            )
            .is_ok()
    );
    assert_eq!(
        patch.evaluate_base(
            PressureFlow {
                linear_velocity: Vec3::X * 499.0,
                ..flow()
            },
            base
        ),
        Err(PressureError::UnsupportedMach)
    );
    assert_eq!(
        patch.evaluate_base(
            PressureFlow {
                linear_velocity: Vec3::X * 1001.0,
                ..flow()
            },
            base
        ),
        Err(PressureError::UnsupportedMach)
    );
    assert_eq!(
        patch
            .evaluate_base(flow(), BasePressure::new(0.0, 0.0, 20.0).unwrap())
            .unwrap()
            .force,
        Vec3::ZERO
    );
}

#[test]
fn base_pressure_rejects_invalid_coefficients_and_domains() {
    for (cp, min, max) in [
        (0.1, 0.0, 1.0),
        (f64::NAN, 0.0, 1.0),
        (-0.1, -1.0, 1.0),
        (-0.1, 2.0, 1.0),
        (-0.1, 0.0, f64::INFINITY),
    ] {
        assert_eq!(
            BasePressure::new(cp, min, max),
            Err(PressureError::InvalidCoefficient)
        );
    }
}

#[test]
fn base_pressure_rejects_windward_surface_in_local_flow() {
    let base = BasePressure::new(-0.4, 0.0, 20.0).unwrap();
    let windward = PressurePatch::new(Vec3::ZERO, Vec3::X, 1.0).unwrap();
    assert_eq!(
        windward.evaluate_base(flow(), base),
        Err(PressureError::InvalidBaseOrientation)
    );
    let rotating = PressurePatch::new(Vec3::Y, -Vec3::X, 1.0).unwrap();
    let reversed = PressureFlow {
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::Z * 1000.0,
        ..flow()
    };
    assert_eq!(
        rotating.evaluate_base(reversed, base),
        Err(PressureError::InvalidBaseOrientation)
    );
}

#[test]
fn normal_plate_matches_analytic_pressure_and_offset_moment() {
    let p = PressurePatch::new(Vec3::Y, Vec3::X, 2.0).unwrap();
    let w = p.evaluate(flow()).unwrap();
    assert_eq!(w.force, Vec3::X * (-2_000_000.0));
    assert_eq!(w.moment, Vec3::Z * 2_000_000.0);
    assert_eq!(p.center(), Vec3::Y);
    assert_eq!(p.normal(), Vec3::X);
    assert_eq!(p.area(), 2.0);
}

#[test]
fn small_patch_preserves_force_when_dynamic_pressure_overflows() {
    let patch = PressurePatch::new(Vec3::ZERO, Vec3::X, 1e-200).unwrap();
    let load = patch
        .evaluate(PressureFlow {
            atmosphere: AtmosphericState::new(1e200, Vec3::ZERO).unwrap(),
            ..flow()
        })
        .unwrap();
    assert!((load.force.x + 1e6).abs() < 1e-8);
}

#[test]
fn grazing_leeward_and_oblique_patch_loads() {
    for normal in [Vec3::Y, -Vec3::X] {
        let p = PressurePatch::new(Vec3::ZERO, normal, 1.0).unwrap();
        assert_eq!(p.evaluate(flow()).unwrap().force, Vec3::ZERO);
    }
    let p = PressurePatch::new(Vec3::ZERO, Vec3::X + Vec3::Y, 1.0).unwrap();
    let w = p.evaluate(flow()).unwrap();
    assert!((w.force.length() - 500_000.0).abs() < 1e-8);
    assert!(w.force.dot(flow().linear_velocity) < 0.0);
}

#[test]
fn com_change_only_changes_moment_arm() {
    let p = PressurePatch::new(Vec3::Y, Vec3::X, 1.0).unwrap();
    let a = p.evaluate(flow()).unwrap();
    let b = p
        .evaluate(PressureFlow {
            center_of_mass: Vec3::Y,
            ..flow()
        })
        .unwrap();
    assert_eq!(a.force, b.force);
    assert_eq!(b.moment, Vec3::ZERO);
}

#[test]
fn rigid_transform_rotates_force_and_moment_without_translation_effect() {
    let p = PressurePatch::new(Vec3::Y, Vec3::X, 1.0).unwrap();
    let a = p.evaluate(flow()).unwrap();
    let transform = Transform::new(
        Vec3::splat(1e12),
        Quat::from_axis_angle(Vec3::Z, 0.7).unwrap(),
    )
    .unwrap();
    let b = p
        .evaluate(PressureFlow {
            transform,
            linear_velocity: transform.transform_vector(flow().linear_velocity),
            ..flow()
        })
        .unwrap();
    assert!((b.force - transform.transform_vector(a.force)).length() < 1e-8);
    assert!((b.moment - transform.transform_vector(a.moment)).length() < 1e-8);
}

#[test]
fn rotational_velocity_generates_dissipative_torque() {
    let p = PressurePatch::new(Vec3::Y, Vec3::X, 1.0).unwrap();
    let f = PressureFlow {
        linear_velocity: Vec3::ZERO,
        angular_velocity: -Vec3::Z * 1000.0,
        ..flow()
    };
    let w = p.evaluate(f).unwrap();
    assert!(w.moment.dot(f.angular_velocity) < 0.0);
    assert!(w.force.dot(f.angular_velocity.cross(Vec3::Y)) < 0.0);
}

#[test]
fn limits_vacuum_zero_speed_and_mach_boundary() {
    let p = PressurePatch::new(Vec3::ZERO, Vec3::X, 1.0).unwrap();
    assert_eq!(
        p.evaluate(PressureFlow {
            linear_velocity: Vec3::X * 499.0,
            ..flow()
        }),
        Err(PressureError::UnsupportedMach)
    );
    assert!(
        p.evaluate(PressureFlow {
            linear_velocity: Vec3::X * 500.0,
            ..flow()
        })
        .is_ok()
    );
    assert_eq!(
        p.evaluate(PressureFlow {
            linear_velocity: Vec3::ZERO,
            ..flow()
        })
        .unwrap()
        .force,
        Vec3::ZERO
    );
    assert_eq!(
        p.evaluate(PressureFlow {
            atmosphere: AtmosphericState::new(0.0, Vec3::ZERO).unwrap(),
            linear_velocity: Vec3::X,
            ..flow()
        })
        .unwrap()
        .force,
        Vec3::ZERO
    );
}

#[test]
fn rejects_invalid_input_and_overflow_without_mutation() {
    assert_eq!(
        PressurePatch::new(Vec3::ZERO, Vec3::ZERO, 1.0),
        Err(PressureError::InvalidPatch)
    );
    assert_eq!(
        PressurePatch::new(Vec3::ZERO, Vec3::X, -1.0),
        Err(PressureError::InvalidPatch)
    );
    let p = PressurePatch::new(Vec3::Y, Vec3::X, 1.0).unwrap();
    assert_eq!(
        p.evaluate(PressureFlow {
            sound_speed: 0.0,
            ..flow()
        }),
        Err(PressureError::InvalidSoundSpeed)
    );
    assert_eq!(
        p.evaluate(PressureFlow {
            angular_velocity: Vec3::splat(f64::NAN),
            ..flow()
        }),
        Err(PressureError::InvalidMotion)
    );
    assert_eq!(
        p.evaluate(PressureFlow {
            linear_velocity: Vec3::X * f64::MAX,
            ..flow()
        }),
        Err(PressureError::ArithmeticOverflow)
    );
}
