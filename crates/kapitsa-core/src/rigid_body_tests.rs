use glam::{Mat3, Quat, Vec3};

use super::{BodyType, MassProperties, RigidBody};
use crate::transform::Transform;

fn transform() -> Transform {
    Transform::new(Vec3::new(1.0, 2.0, 3.0), Quat::IDENTITY).expect("valid transform")
}

fn transform_at(translation: Vec3, rotation: Quat) -> Transform {
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
    let dynamic_body = RigidBody::new_dynamic(transform, mass());

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
        RigidBody::new_dynamic(transform(), mass()),
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
    let body = RigidBody::new_dynamic(transform, mass());

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
    let body = RigidBody::new_dynamic(transform_at(Vec3::new(1.0, 2.0, 3.0), rotation), mass());

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
fn velocities_are_allowed_for_dynamic_and_kinematic_bodies() -> Result<(), super::BodyError> {
    let mut dynamic = RigidBody::new_dynamic(transform(), mass());
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
    let mut body = RigidBody::new_dynamic(transform(), mass());
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
    let mut body = RigidBody::new_dynamic(transform(), mass());
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
    let mut body = RigidBody::new_dynamic(Transform::identity(), large_center);
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
    let mut body = RigidBody::new_dynamic(transform(), mass());
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
    let mut body = RigidBody::new_dynamic(transform(), mass());
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
    let mut body = RigidBody::new_dynamic(transform(), mass());

    body.sleep().expect("dynamic body can sleep");
    body.set_linear_velocity(Vec3::ZERO)
        .expect("zero velocity is valid");
    assert!(body.state().is_sleeping());

    body.set_angular_velocity(Vec3::Z).expect("finite velocity");
    assert!(!body.state().is_sleeping());
}
