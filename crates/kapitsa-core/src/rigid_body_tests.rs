use glam::{Mat3, Quat, Vec3};

use super::{BodyType, MassProperties, RigidBody};
use crate::transform::Transform;

fn transform() -> Transform {
    Transform::new(Vec3::new(1.0, 2.0, 3.0), Quat::IDENTITY).expect("valid transform")
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
