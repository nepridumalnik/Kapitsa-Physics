use glam::{Mat3, Quat, Vec3};
use kapitsa_math::{Quat as MathQuat, Vec3 as MathVec3};

use super::{BodyType, MassProperties, RigidBody};
use crate::transform::Transform;

fn transform() -> Transform {
    transform_at(Vec3::new(1.0, 2.0, 3.0), Quat::IDENTITY)
}

fn transform_at(translation: Vec3, rotation: Quat) -> Transform {
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

fn mass() -> MassProperties {
    MassProperties::new(2.0, Vec3::new(0.0, 0.5, 0.0), Mat3::IDENTITY)
        .expect("valid mass properties")
}

#[test]
fn constructors_set_body_types_and_mass_properties() {
    let transform = transform();
    let static_body = RigidBody::new_static(transform);
    let kinematic_body = RigidBody::new_kinematic(transform);
    let dynamic_body = RigidBody::new_dynamic(transform, mass()).expect("valid dynamic body");

    assert_eq!(static_body.physics().body_type(), BodyType::Static);
    assert_eq!(static_body.physics().mass(), None);
    assert_eq!(kinematic_body.physics().body_type(), BodyType::Kinematic);
    assert_eq!(kinematic_body.physics().mass(), None);
    assert_eq!(dynamic_body.physics().body_type(), BodyType::Dynamic);
    assert_eq!(dynamic_body.physics().mass(), Some(mass()));
}

#[test]
fn constructors_initialize_motion_and_accumulators_to_zero() {
    for body in [
        RigidBody::new_static(transform()),
        RigidBody::new_kinematic(transform()),
        RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body"),
    ] {
        assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
        assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
        assert!(!body.state().is_sleeping());
        assert_eq!(body.forces().force(), Vec3::ZERO);
        assert_eq!(body.forces().torque(), Vec3::ZERO);
    }
}

#[test]
fn state_and_physics_getters_return_initial_values() {
    let transform = transform();
    let body = RigidBody::new_dynamic(transform, mass()).expect("valid dynamic body");

    assert_eq!(body.state().transform(), transform);
    assert_eq!(body.physics().body_type(), BodyType::Dynamic);
    assert_eq!(body.physics().mass(), Some(mass()));
}

#[test]
fn body_error_has_display_and_error_implementations() {
    let error = super::BodyError::BodyMustBeDynamic;

    assert_eq!(error.to_string(), "operation requires a dynamic body");
    let _: &dyn std::error::Error = &error;
}

#[test]
fn world_center_of_mass_applies_rotation_and_translation() {
    let rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    let body = RigidBody::new_dynamic(transform_at(Vec3::new(1.0, 2.0, 3.0), rotation), mass())
        .expect("valid dynamic body");

    assert_eq!(
        body.world_center_of_mass(),
        Ok(Some(Vec3::new(0.5, 2.0, 3.0)))
    );
    assert_eq!(
        RigidBody::new_static(transform()).world_center_of_mass(),
        Ok(None)
    );
}

#[test]
fn dynamic_constructor_accepts_an_offset_local_center_of_mass() {
    let local_center = Vec3::new(2.0, -1.0, 0.5);
    let mass_properties =
        MassProperties::new(1.0, local_center, Mat3::IDENTITY).expect("valid mass properties");
    let body = RigidBody::new_dynamic(Transform::identity(), mass_properties)
        .expect("finite world center of mass");

    assert_eq!(body.world_center_of_mass(), Ok(Some(local_center)));
}

#[test]
fn dynamic_constructor_rejects_world_center_of_mass_overflow() {
    let transform = transform_at(Vec3::X * f32::MAX, Quat::IDENTITY);
    let mass_properties = MassProperties::new(1.0, Vec3::X * f32::MAX, Mat3::IDENTITY)
        .expect("valid mass properties");

    assert_eq!(
        RigidBody::new_dynamic(transform, mass_properties),
        Err(super::BodyError::ArithmeticOverflow)
    );
}

#[test]
fn velocities_are_allowed_for_dynamic_and_kinematic_bodies() -> Result<(), super::BodyError> {
    let mut dynamic = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    let mut kinematic = RigidBody::new_kinematic(transform());
    let linear = Vec3::new(1.0, 2.0, 3.0);
    let angular = Vec3::new(-1.0, 0.5, 2.0);

    dynamic.set_linear_velocity(linear)?;
    dynamic.set_angular_velocity(angular)?;
    kinematic.set_linear_velocity(linear)?;
    kinematic.set_angular_velocity(angular)?;

    assert_eq!(dynamic.state().linear_velocity(), linear);
    assert_eq!(dynamic.state().angular_velocity(), angular);
    assert_eq!(kinematic.state().linear_velocity(), linear);
    assert_eq!(kinematic.state().angular_velocity(), angular);
    Ok(())
}

#[test]
fn static_bodies_reject_velocity_changes() {
    let mut body = RigidBody::new_static(transform());

    assert_eq!(
        body.set_linear_velocity(Vec3::X),
        Err(super::BodyError::StaticBodyCannotHaveVelocity)
    );
    assert_eq!(
        body.set_angular_velocity(Vec3::Y),
        Err(super::BodyError::StaticBodyCannotHaveVelocity)
    );
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
}

#[test]
fn rejects_non_finite_velocities_without_changing_state() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.set_linear_velocity(Vec3::X).expect("finite velocity");
    body.set_angular_velocity(Vec3::Y).expect("finite velocity");

    assert_eq!(
        body.set_linear_velocity(Vec3::new(f32::NAN, 0.0, 0.0)),
        Err(super::BodyError::NonFiniteValue)
    );
    assert_eq!(
        body.set_angular_velocity(Vec3::new(0.0, f32::INFINITY, 0.0)),
        Err(super::BodyError::NonFiniteValue)
    );
    assert_eq!(body.state().linear_velocity(), Vec3::X);
    assert_eq!(body.state().angular_velocity(), Vec3::Y);
}

#[test]
fn changed_transform_preserves_velocity_and_wakes_dynamic_body() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.set_linear_velocity(Vec3::X).expect("finite velocity");
    body.set_angular_velocity(Vec3::Y).expect("finite velocity");

    let updated_transform = Transform::identity();
    body.set_transform(updated_transform)
        .expect("valid transformed center of mass");
    assert_eq!(body.state().linear_velocity(), Vec3::X);
    assert_eq!(body.state().angular_velocity(), Vec3::Y);

    body.sleep().expect("dynamic body can sleep");

    body.set_transform(updated_transform)
        .expect("valid transformed center of mass");

    assert!(body.state().is_sleeping());
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);

    body.set_linear_velocity(Vec3::X).expect("finite velocity");
    body.set_angular_velocity(Vec3::Y).expect("finite velocity");
    body.sleep().expect("dynamic body can sleep");
    body.set_transform(transform())
        .expect("valid transformed center of mass");

    assert!(!body.state().is_sleeping());
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
    assert_eq!(body.state().transform(), transform());
}

#[test]
fn transform_overflow_does_not_change_body() {
    let large_center = MassProperties::new(1.0, Vec3::new(f32::MAX, 0.0, 0.0), Mat3::IDENTITY)
        .expect("finite mass properties");
    let mut body =
        RigidBody::new_dynamic(Transform::identity(), large_center).expect("valid dynamic body");
    body.set_linear_velocity(Vec3::X).expect("finite velocity");
    let original = body.state().transform();

    assert_eq!(
        body.set_transform(transform_at(Vec3::X * f32::MAX, Quat::IDENTITY)),
        Err(super::BodyError::ArithmeticOverflow)
    );
    assert_eq!(body.state().transform(), original);
    assert_eq!(body.state().linear_velocity(), Vec3::X);
}

#[test]
fn set_mass_only_changes_dynamic_body_and_preserves_pose_and_velocity() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    let velocity = Vec3::new(1.0, 2.0, 3.0);
    body.set_linear_velocity(velocity).expect("finite velocity");
    let replacement =
        MassProperties::new(3.0, Vec3::ZERO, Mat3::IDENTITY * 2.0).expect("valid mass properties");

    body.set_mass(replacement).expect("dynamic body has mass");

    assert_eq!(body.physics().mass(), Some(replacement));
    assert_eq!(body.state().transform(), transform());
    assert_eq!(body.state().linear_velocity(), velocity);
    assert!(!body.state().is_sleeping());

    body.sleep().expect("dynamic body can sleep");
    let second_replacement =
        MassProperties::new(4.0, Vec3::ZERO, Mat3::IDENTITY * 3.0).expect("valid mass properties");
    body.set_mass(second_replacement)
        .expect("dynamic body has mass");
    assert!(!body.state().is_sleeping());
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert_eq!(
        RigidBody::new_static(transform()).set_mass(replacement),
        Err(super::BodyError::BodyMustBeDynamic)
    );
}

#[test]
fn sleep_and_wake_up_follow_dynamic_body_rules() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.set_linear_velocity(Vec3::X).expect("finite velocity");
    body.set_angular_velocity(Vec3::Y).expect("finite velocity");
    body.sleep().expect("dynamic body can sleep");
    body.sleep().expect("sleep is idempotent");

    assert!(body.state().is_sleeping());
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert_eq!(body.state().angular_velocity(), Vec3::ZERO);

    body.wake_up();
    assert!(!body.state().is_sleeping());
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert_eq!(body.sleep(), Ok(()));

    for mut non_dynamic in [
        RigidBody::new_static(transform()),
        RigidBody::new_kinematic(transform()),
    ] {
        assert_eq!(
            non_dynamic.sleep(),
            Err(super::BodyError::BodyMustBeDynamic)
        );
        non_dynamic.wake_up();
        assert!(!non_dynamic.state().is_sleeping());
    }
}

#[test]
fn nonzero_velocity_wakes_but_zero_velocity_does_not() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");

    body.sleep().expect("dynamic body can sleep");
    body.set_linear_velocity(Vec3::ZERO)
        .expect("zero velocity is valid");
    assert!(body.state().is_sleeping());

    body.set_angular_velocity(Vec3::Z).expect("finite velocity");
    assert!(!body.state().is_sleeping());
}

#[test]
fn forces_and_torques_accumulate_and_can_be_cleared() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.apply_force(Vec3::X)
        .expect("dynamic body accepts force");
    body.apply_force(Vec3::Y)
        .expect("dynamic body accepts force");
    body.apply_torque(Vec3::Z)
        .expect("dynamic body accepts torque");
    body.apply_torque(Vec3::Z)
        .expect("dynamic body accepts torque");

    assert_eq!(body.forces().force(), Vec3::X + Vec3::Y);
    assert_eq!(body.forces().torque(), 2.0 * Vec3::Z);
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert_eq!(body.state().angular_velocity(), Vec3::ZERO);

    body.clear_forces();
    assert_eq!(body.forces().force(), Vec3::ZERO);
    assert_eq!(body.forces().torque(), Vec3::ZERO);
}

#[test]
fn force_at_center_of_mass_adds_no_torque() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    let center = body
        .world_center_of_mass()
        .expect("valid center of mass")
        .expect("dynamic body has mass");

    body.apply_force_at_point(Vec3::new(0.0, 2.0, 0.0), center)
        .expect("dynamic body accepts force");

    assert_eq!(body.forces().force(), Vec3::new(0.0, 2.0, 0.0));
    assert_eq!(body.forces().torque(), Vec3::ZERO);
}

#[test]
fn force_with_lever_arm_adds_torque_in_the_expected_direction() {
    let centered_mass =
        MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
    let mut body =
        RigidBody::new_dynamic(Transform::identity(), centered_mass).expect("valid dynamic body");

    body.apply_force_at_point(Vec3::Y, Vec3::X)
        .expect("dynamic body accepts force");

    assert_eq!(body.forces().force(), Vec3::Y);
    assert_eq!(body.forces().torque(), Vec3::Z);
}

#[test]
fn force_and_torque_operations_require_dynamic_body() {
    for mut body in [
        RigidBody::new_static(transform()),
        RigidBody::new_kinematic(transform()),
    ] {
        assert_eq!(
            body.apply_force(Vec3::X),
            Err(super::BodyError::BodyMustBeDynamic)
        );
        assert_eq!(
            body.apply_force_at_point(Vec3::X, Vec3::Y),
            Err(super::BodyError::BodyMustBeDynamic)
        );
        assert_eq!(
            body.apply_torque(Vec3::Z),
            Err(super::BodyError::BodyMustBeDynamic)
        );
        assert_eq!(body.forces().force(), Vec3::ZERO);
        assert_eq!(body.forces().torque(), Vec3::ZERO);
    }
}

#[test]
fn invalid_force_inputs_do_not_change_accumulators_or_sleep_state() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.apply_force(Vec3::X).expect("valid force");

    assert_eq!(
        body.apply_force(Vec3::new(f32::NAN, 0.0, 0.0)),
        Err(super::BodyError::NonFiniteValue)
    );
    assert_eq!(
        body.apply_force_at_point(Vec3::Y, Vec3::new(f32::INFINITY, 0.0, 0.0)),
        Err(super::BodyError::NonFiniteValue)
    );
    assert_eq!(
        body.apply_torque(Vec3::new(0.0, 0.0, f32::NEG_INFINITY)),
        Err(super::BodyError::NonFiniteValue)
    );
    assert_eq!(body.forces().force(), Vec3::X);
    assert_eq!(body.forces().torque(), Vec3::ZERO);

    body.sleep().expect("dynamic body can sleep");
    assert_eq!(
        body.apply_torque(Vec3::new(0.0, 0.0, f32::NEG_INFINITY)),
        Err(super::BodyError::NonFiniteValue)
    );
    assert_eq!(body.forces().force(), Vec3::ZERO);
    assert_eq!(body.forces().torque(), Vec3::ZERO);
    assert!(body.state().is_sleeping());
}

#[test]
fn force_accumulation_overflow_is_atomic() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    let large_force = Vec3::splat(f32::MAX * 0.75);
    body.apply_force(large_force)
        .expect("force is representable");

    assert_eq!(
        body.apply_force(large_force),
        Err(super::BodyError::ArithmeticOverflow)
    );
    assert_eq!(body.forces().force(), large_force);

    let mut body =
        RigidBody::new_dynamic(Transform::identity(), mass()).expect("valid dynamic body");
    assert_eq!(
        body.apply_force_at_point(Vec3::X * f32::MAX, Vec3::new(0.0, -f32::MAX, 0.0)),
        Err(super::BodyError::ArithmeticOverflow)
    );
    assert_eq!(body.forces().force(), Vec3::ZERO);
    assert_eq!(body.forces().torque(), Vec3::ZERO);
}

#[test]
fn nonzero_force_wakes_and_zero_force_does_not() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.sleep().expect("dynamic body can sleep");
    body.apply_force(Vec3::ZERO).expect("zero force is valid");
    assert!(body.state().is_sleeping());

    body.apply_torque(Vec3::Z)
        .expect("dynamic body accepts torque");
    assert!(!body.state().is_sleeping());
}

#[test]
fn changing_mass_clears_accumulated_forces_and_torques() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.apply_force(Vec3::X).expect("valid force");
    body.apply_torque(Vec3::Y).expect("valid torque");
    let replacement =
        MassProperties::new(3.0, Vec3::ZERO, Mat3::IDENTITY * 2.0).expect("valid mass properties");

    body.set_mass(replacement).expect("dynamic body has mass");

    assert_eq!(body.forces().force(), Vec3::ZERO);
    assert_eq!(body.forces().torque(), Vec3::ZERO);
}

#[test]
fn linear_impulse_changes_velocity_by_impulse_over_mass() {
    let mut light = RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties"),
    )
    .expect("valid dynamic body");
    let mut heavy = RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(4.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties"),
    )
    .expect("valid dynamic body");

    light
        .apply_impulse(Vec3::X)
        .expect("dynamic body accepts impulse");
    heavy
        .apply_impulse(Vec3::X)
        .expect("dynamic body accepts impulse");

    assert_eq!(light.state().linear_velocity(), Vec3::X);
    assert_eq!(heavy.state().linear_velocity(), Vec3::X * 0.25);
    assert_eq!(light.state().angular_velocity(), Vec3::ZERO);
}

#[test]
fn impulse_at_center_of_mass_does_not_change_angular_velocity() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    let center = body
        .world_center_of_mass()
        .expect("valid center of mass")
        .expect("dynamic body has mass");

    body.apply_impulse_at_point(Vec3::new(2.0, 0.0, 0.0), center)
        .expect("dynamic body accepts impulse");

    assert_eq!(body.state().linear_velocity(), Vec3::X);
    assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
}

#[test]
fn impulse_with_lever_arm_changes_linear_and_angular_velocity() {
    let properties =
        MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
    let mut body =
        RigidBody::new_dynamic(Transform::identity(), properties).expect("valid dynamic body");

    body.apply_impulse_at_point(Vec3::Z, Vec3::Y)
        .expect("dynamic body accepts impulse");

    assert_eq!(body.state().linear_velocity(), Vec3::Z * 0.5);
    assert_eq!(body.state().angular_velocity(), Vec3::X);
}

#[test]
fn rotated_inertia_tensor_rotates_the_angular_response() {
    let inertia = Mat3::from_diagonal(Vec3::new(2.0, 4.0, 5.0));
    let properties = MassProperties::new(1.0, Vec3::ZERO, inertia).expect("valid mass properties");
    let rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
    let mut body = RigidBody::new_dynamic(transform_at(Vec3::ZERO, rotation), properties)
        .expect("valid dynamic body");

    body.apply_impulse_at_point(Vec3::Z, Vec3::Y)
        .expect("dynamic body accepts impulse");

    assert!((body.state().angular_velocity().x - 0.25).abs() < 1.0e-6);
    assert!(body.state().angular_velocity().y.abs() < 1.0e-6);
    assert!(body.state().angular_velocity().z.abs() < 1.0e-6);
}

#[test]
fn off_diagonal_inertia_is_used_for_angular_impulses() {
    let inertia = Mat3::from_cols(
        Vec3::new(2.0, 0.5, 0.0),
        Vec3::new(0.5, 3.0, 0.0),
        Vec3::new(0.0, 0.0, 4.0),
    );
    let properties = MassProperties::new(1.0, Vec3::ZERO, inertia).expect("valid mass properties");
    let mut body =
        RigidBody::new_dynamic(Transform::identity(), properties).expect("valid dynamic body");

    body.apply_impulse_at_point(Vec3::Z, Vec3::Y)
        .expect("dynamic body accepts impulse");

    let angular_velocity = body.state().angular_velocity();
    assert!((angular_velocity.x - 12.0 / 23.0).abs() < 1.0e-6);
    assert!((angular_velocity.y + 2.0 / 23.0).abs() < 1.0e-6);
    assert!(angular_velocity.z.abs() < 1.0e-6);
}

#[test]
fn impulses_wake_dynamic_bodies_only_when_nonzero() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.sleep().expect("dynamic body can sleep");
    body.apply_impulse(Vec3::ZERO)
        .expect("zero impulse is valid");
    body.apply_impulse_at_point(Vec3::ZERO, Vec3::X)
        .expect("zero impulse is valid");
    assert!(body.state().is_sleeping());

    body.apply_impulse(Vec3::X)
        .expect("dynamic body accepts impulse");
    assert!(!body.state().is_sleeping());
}

#[test]
fn impulses_require_a_dynamic_body() {
    for mut body in [
        RigidBody::new_static(transform()),
        RigidBody::new_kinematic(transform()),
    ] {
        assert_eq!(
            body.apply_impulse(Vec3::X),
            Err(super::BodyError::BodyMustBeDynamic)
        );
        assert_eq!(
            body.apply_impulse_at_point(Vec3::X, Vec3::Y),
            Err(super::BodyError::BodyMustBeDynamic)
        );
        assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
        assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
    }
}

#[test]
fn invalid_impulses_do_not_change_body_state() {
    let mut body = RigidBody::new_dynamic(transform(), mass()).expect("valid dynamic body");
    body.set_linear_velocity(Vec3::Y).expect("finite velocity");
    body.sleep().expect("dynamic body can sleep");

    assert_eq!(
        body.apply_impulse(Vec3::new(f32::NAN, 0.0, 0.0)),
        Err(super::BodyError::NonFiniteValue)
    );
    assert_eq!(
        body.apply_impulse_at_point(Vec3::X, Vec3::new(0.0, f32::INFINITY, 0.0)),
        Err(super::BodyError::NonFiniteValue)
    );
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
    assert!(body.state().is_sleeping());
}

#[test]
fn impulse_overflow_does_not_partially_update_velocity_or_sleep_state() {
    let tiny_mass = MassProperties::new(f32::from_bits(1), Vec3::ZERO, Mat3::IDENTITY)
        .expect("valid mass properties");
    let mut body =
        RigidBody::new_dynamic(Transform::identity(), tiny_mass).expect("valid dynamic body");
    body.sleep().expect("dynamic body can sleep");

    assert_eq!(
        body.apply_impulse(Vec3::X * f32::MAX),
        Err(super::BodyError::ArithmeticOverflow)
    );
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert!(body.state().is_sleeping());

    let mut body =
        RigidBody::new_dynamic(Transform::identity(), mass()).expect("valid dynamic body");
    body.sleep().expect("dynamic body can sleep");
    assert_eq!(
        body.apply_impulse_at_point(Vec3::X * f32::MAX, Vec3::new(0.0, -f32::MAX, 0.0)),
        Err(super::BodyError::ArithmeticOverflow)
    );
    assert_eq!(body.state().linear_velocity(), Vec3::ZERO);
    assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
    assert!(body.state().is_sleeping());
}

#[test]
fn mass_update_overflow_preserves_existing_properties_and_state() {
    let existing =
        MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
    let replacement = MassProperties::new(2.0, Vec3::new(f32::MAX, 0.0, 0.0), Mat3::IDENTITY)
        .expect("valid mass properties");
    let pose = transform_at(Vec3::X * f32::MAX, Quat::IDENTITY);
    let mut body = RigidBody::new_dynamic(pose, existing).expect("valid dynamic body");
    body.set_linear_velocity(Vec3::X).expect("finite velocity");
    body.apply_force(Vec3::Y).expect("valid force");
    body.sleep().expect("dynamic body can sleep");

    assert_eq!(
        body.set_mass(replacement),
        Err(super::BodyError::ArithmeticOverflow)
    );
    assert_eq!(body.physics().mass(), Some(existing));
    assert_eq!(body.state().transform(), pose);
    assert!(body.state().is_sleeping());
    assert_eq!(body.forces().force(), Vec3::ZERO);
}
