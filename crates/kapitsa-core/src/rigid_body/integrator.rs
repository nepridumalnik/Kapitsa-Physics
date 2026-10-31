use std::fmt;

use kapitsa_math::{CholeskyError, Mat3, Vec3};

use super::{BodyState, BodyType, RigidBody};
use crate::transform::Transform;

/// Failure returned when a rigid-body state cannot be predicted for one timestep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum IntegrationError {
    InvalidDeltaTime,
    MissingMassProperties,
    InvalidInertiaTensor,
    ArithmeticOverflow,
}

impl fmt::Display for IntegrationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidDeltaTime => "timestep must be positive and finite",
            Self::MissingMassProperties => "dynamic body requires mass properties",
            Self::InvalidInertiaTensor => "inertia tensor could not be solved",
            Self::ArithmeticOverflow => "integration result exceeds the finite f64 range",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for IntegrationError {}

/// Predicts a body's state without mutating the source body.
#[allow(dead_code)] // PhysicsWorld::step consumes this in the next implementation stage.
pub(crate) fn predict(body: &RigidBody, dt: f64) -> Result<BodyState, IntegrationError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(IntegrationError::InvalidDeltaTime);
    }

    if body.state().is_sleeping() {
        return Ok(*body.state());
    }

    match body.physics().body_type() {
        BodyType::Static => Ok(*body.state()),
        BodyType::Kinematic => predict_kinematic(body, dt),
        BodyType::Dynamic => predict_dynamic(body, dt),
    }
}

fn predict_kinematic(body: &RigidBody, dt: f64) -> Result<BodyState, IntegrationError> {
    let state = *body.state();
    let rotation = state
        .transform
        .rotation()
        .integrate_world_angular_velocity(state.angular_velocity, dt)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let translation_delta = state
        .linear_velocity
        .checked_mul_scalar(dt)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let translation = if let Some(mass) = body.physics().mass() {
        let center = world_center_of_mass(state.transform, mass.center_of_mass())?
            .checked_add(translation_delta)
            .ok_or(IntegrationError::ArithmeticOverflow)?;
        center
            .checked_sub(
                rotation
                    .rotate_vector(mass.center_of_mass())
                    .ok_or(IntegrationError::ArithmeticOverflow)?,
            )
            .ok_or(IntegrationError::ArithmeticOverflow)?
    } else {
        state
            .transform
            .translation()
            .checked_add(translation_delta)
            .ok_or(IntegrationError::ArithmeticOverflow)?
    };
    let transform =
        Transform::new(translation, rotation).map_err(|_| IntegrationError::ArithmeticOverflow)?;

    Ok(BodyState { transform, ..state })
}

fn predict_dynamic(body: &RigidBody, dt: f64) -> Result<BodyState, IntegrationError> {
    let state = *body.state();
    let physics = body.physics();
    let mass = physics
        .mass()
        .ok_or(IntegrationError::MissingMassProperties)?;

    let linear_acceleration = body.forces().force() / mass.mass();
    if !linear_acceleration.is_finite() {
        return Err(IntegrationError::ArithmeticOverflow);
    }
    let linear_velocity = state
        .linear_velocity
        .checked_add(
            linear_acceleration
                .checked_mul_scalar(dt)
                .ok_or(IntegrationError::ArithmeticOverflow)?,
        )
        .ok_or(IntegrationError::ArithmeticOverflow)?;

    let rotation = state.transform.rotation();
    let local_inertia = mass.inertia();
    let rotation_matrix = rotation
        .to_mat3()
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let world_inertia = rotation_matrix
        .checked_mul_matrix(local_inertia)
        .and_then(|rotated| rotated.checked_mul_matrix(rotation_matrix.transpose()))
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let angular_momentum = world_inertia
        .checked_mul_vector(state.angular_velocity)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let gyroscopic_torque = state
        .angular_velocity
        .checked_cross(angular_momentum)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let net_torque = body
        .forces()
        .torque()
        .checked_sub(gyroscopic_torque)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let angular_acceleration = solve_spd(world_inertia, net_torque)?;
    let angular_velocity = state
        .angular_velocity
        .checked_add(
            angular_acceleration
                .checked_mul_scalar(dt)
                .ok_or(IntegrationError::ArithmeticOverflow)?,
        )
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let new_rotation = rotation
        .integrate_world_angular_velocity(angular_velocity, dt)
        .ok_or(IntegrationError::ArithmeticOverflow)?;

    let center_of_mass = world_center_of_mass(state.transform, mass.center_of_mass())?;
    let new_center_of_mass = center_of_mass
        .checked_add(
            linear_velocity
                .checked_mul_scalar(dt)
                .ok_or(IntegrationError::ArithmeticOverflow)?,
        )
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let rotated_local_center = new_rotation
        .rotate_vector(mass.center_of_mass())
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let translation = new_center_of_mass
        .checked_sub(rotated_local_center)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let transform = Transform::new(translation, new_rotation)
        .map_err(|_| IntegrationError::ArithmeticOverflow)?;

    Ok(BodyState {
        transform,
        linear_velocity,
        angular_velocity,
        sleeping: state.sleeping,
    })
}

fn world_center_of_mass(
    transform: Transform,
    local_center: Vec3,
) -> Result<Vec3, IntegrationError> {
    transform
        .rotation()
        .rotate_vector(local_center)
        .and_then(|center| center.checked_add(transform.translation()))
        .ok_or(IntegrationError::ArithmeticOverflow)
}

fn solve_spd(matrix: Mat3, rhs: Vec3) -> Result<Vec3, IntegrationError> {
    match matrix.solve_spd(rhs) {
        Ok(solution) => Ok(solution),
        Err(CholeskyError::NotSymmetric | CholeskyError::NotPositiveDefinite) => {
            Err(IntegrationError::InvalidInertiaTensor)
        }
        Err(CholeskyError::NonFiniteInput | CholeskyError::ResultOutOfRange) => {
            Err(IntegrationError::ArithmeticOverflow)
        }
    }
}

#[cfg(test)]
mod tests {
    use kapitsa_math::{Mat3, Quat, Vec3};

    use super::{IntegrationError, predict};
    use crate::{
        mass::MassProperties,
        rigid_body::{BodyType, RigidBody},
        transform::Transform,
    };

    fn body(mass: MassProperties) -> RigidBody {
        RigidBody::new_dynamic(Transform::identity(), mass).expect("valid dynamic body")
    }

    fn mass(center_of_mass: Vec3, inertia: Mat3) -> MassProperties {
        MassProperties::new(1.0, center_of_mass, inertia).expect("valid mass properties")
    }

    fn rotation_z(angle: f64) -> Quat {
        Quat::from_axis_angle(Vec3::Z, angle).expect("valid rotation")
    }

    fn assert_vec3_close(actual: Vec3, expected: Vec3) {
        assert!((actual.x - expected.x).abs() <= 1.0e-12);
        assert!((actual.y - expected.y).abs() <= 1.0e-12);
        assert!((actual.z - expected.z).abs() <= 1.0e-12);
    }

    #[test]
    fn static_body_state_is_unchanged() {
        let body = RigidBody::new_static(Transform::identity());
        let state = *body.state();

        assert_eq!(predict(&body, 0.1), Ok(state));
    }

    #[test]
    fn sleeping_body_state_is_unchanged() {
        let mut body = body(mass(Vec3::ZERO, Mat3::IDENTITY));
        body.set_linear_velocity(Vec3::X).expect("finite velocity");
        body.apply_force(Vec3::X).expect("finite force");
        body.sleep().expect("dynamic body can sleep");
        let state = *body.state();

        assert_eq!(predict(&body, 0.5), Ok(state));
    }

    #[test]
    fn kinematic_body_moves_its_origin_using_velocity_only() {
        let mut body = RigidBody::new_kinematic(Transform::identity());
        body.set_linear_velocity(Vec3::X * 2.0)
            .expect("finite velocity");
        body.set_angular_velocity(Vec3::Z).expect("finite velocity");
        body.forces.force = Vec3::Y * 100.0;
        body.forces.torque = Vec3::Y * 100.0;
        let original = *body.state();

        let predicted = predict(&body, 0.5).expect("finite prediction");
        assert_vec3_close(predicted.transform.translation(), Vec3::X);
        assert_vec3_close(
            predicted.transform.transform_vector(Vec3::X),
            rotation_z(0.5)
                .rotate_vector(Vec3::X)
                .expect("rotated vector"),
        );
        assert_eq!(predicted.linear_velocity(), Vec3::X * 2.0);
        assert_eq!(predicted.angular_velocity(), Vec3::Z);
        assert_eq!(body.state(), &original);
        assert_eq!(body.physics().body_type(), BodyType::Kinematic);
    }

    #[test]
    fn kinematic_mass_center_stays_fixed_while_the_body_rotates() {
        let mut body = RigidBody::new_kinematic(Transform::identity());
        body.physics.mass = Some(mass(Vec3::X, Mat3::IDENTITY));
        body.set_angular_velocity(Vec3::Z * std::f64::consts::FRAC_PI_2)
            .expect("finite velocity");

        let predicted = predict(&body, 1.0).expect("finite prediction");
        assert_vec3_close(predicted.transform.translation(), Vec3::new(1.0, -1.0, 0.0));
        assert_vec3_close(predicted.transform.transform_point(Vec3::X), Vec3::X);
    }

    #[test]
    fn dynamic_body_moves_freely_with_semi_implicit_euler() {
        let mut body = body(mass(Vec3::ZERO, Mat3::IDENTITY));
        body.set_linear_velocity(Vec3::X * 3.0)
            .expect("finite velocity");

        let predicted = predict(&body, 0.5).expect("finite prediction");
        assert_vec3_close(predicted.transform.translation(), Vec3::X * 1.5);
        assert_eq!(predicted.linear_velocity(), Vec3::X * 3.0);
    }

    #[test]
    fn constant_force_updates_velocity_before_position() {
        let properties =
            MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
        let mut body = body(properties);
        body.apply_force(Vec3::X * 4.0).expect("finite force");

        let predicted = predict(&body, 0.5).expect("finite prediction");
        assert_vec3_close(predicted.linear_velocity(), Vec3::X);
        assert_vec3_close(predicted.transform.translation(), Vec3::X * 0.5);
    }

    #[test]
    fn constant_torque_updates_angular_velocity_and_orientation() {
        let mut body = body(mass(Vec3::ZERO, Mat3::IDENTITY));
        body.apply_torque(Vec3::Z * 2.0).expect("finite torque");

        let predicted = predict(&body, 0.5).expect("finite prediction");
        assert_vec3_close(predicted.angular_velocity(), Vec3::Z);
        assert_vec3_close(
            predicted.transform.transform_vector(Vec3::X),
            rotation_z(0.5)
                .rotate_vector(Vec3::X)
                .expect("rotated vector"),
        );
    }

    #[test]
    fn uses_the_full_inertia_tensor_for_angular_acceleration() {
        let inertia = Mat3::from_rows([[2.0, 0.5, 0.0], [0.5, 3.0, 0.0], [0.0, 0.0, 4.0]]);
        let mut body = body(mass(Vec3::ZERO, inertia));
        body.apply_torque(Vec3::new(1.0, -2.0, 0.0))
            .expect("finite torque");

        let predicted = predict(&body, 0.25).expect("finite prediction");
        let expected = inertia
            .solve_spd(Vec3::new(1.0, -2.0, 0.0))
            .expect("valid SPD solve")
            * 0.25;
        assert_vec3_close(predicted.angular_velocity(), expected);
    }

    #[test]
    fn rotates_inertia_into_world_space_for_torque_response() {
        let inertia = Mat3::from_diagonal(Vec3::new(2.0, 4.0, 5.0));
        let transform = Transform::new(Vec3::ZERO, rotation_z(std::f64::consts::FRAC_PI_2))
            .expect("valid transform");
        let mut body = RigidBody::new_dynamic(transform, mass(Vec3::ZERO, inertia))
            .expect("valid dynamic body");
        body.apply_torque(Vec3::X * 4.0).expect("finite torque");

        let predicted = predict(&body, 0.5).expect("finite prediction");
        assert_vec3_close(predicted.angular_velocity(), Vec3::X * 0.5);
    }

    #[test]
    fn gyroscopic_torque_changes_angular_velocity_without_external_torque() {
        let inertia = Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0));
        let mut body = body(mass(Vec3::ZERO, inertia));
        let initial = Vec3::new(1.0, 2.0, 3.0);
        body.set_angular_velocity(initial).expect("finite velocity");

        let predicted = predict(&body, 0.1).expect("finite prediction");
        assert_vec3_close(predicted.angular_velocity(), Vec3::new(0.7, 2.2, 2.95));
    }

    #[test]
    fn offset_center_of_mass_remains_fixed_while_the_body_rotates() {
        let mut body = body(mass(Vec3::X, Mat3::IDENTITY));
        body.set_angular_velocity(Vec3::Z * std::f64::consts::FRAC_PI_2)
            .expect("finite velocity");

        let predicted = predict(&body, 1.0).expect("finite prediction");
        assert_vec3_close(predicted.transform.translation(), Vec3::new(1.0, -1.0, 0.0));
        let center = predicted.transform.transform_point(Vec3::X);
        assert_vec3_close(center, Vec3::X);
    }

    #[test]
    fn rejects_invalid_timestep() {
        let body = body(mass(Vec3::ZERO, Mat3::IDENTITY));

        for dt in [0.0, -0.1, f64::NAN, f64::INFINITY] {
            assert_eq!(predict(&body, dt), Err(IntegrationError::InvalidDeltaTime));
        }
    }

    #[test]
    fn overflow_does_not_mutate_the_source_body() {
        let mut body = body(mass(Vec3::ZERO, Mat3::IDENTITY));
        body.apply_force(Vec3::X * f64::MAX).expect("finite force");
        let original = body;

        assert_eq!(
            predict(&body, 2.0),
            Err(IntegrationError::ArithmeticOverflow)
        );
        assert_eq!(body, original);
    }
}
