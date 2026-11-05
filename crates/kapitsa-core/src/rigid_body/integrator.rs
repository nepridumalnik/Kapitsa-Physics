use std::fmt;

use kapitsa_math::{CholeskyError, Mat3, Vec3};

use super::{BodyState, BodyType, RigidBody};
use crate::transform::Transform;

/// Failure returned when a rigid-body state cannot be predicted for one timestep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegrationError {
    InvalidDeltaTime,
    MissingMassProperties,
    InvalidInertiaTensor,
    ArithmeticOverflow,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct TrajectoryBounds {
    pub(crate) translation: f64,
    pub(crate) rotation: f64,
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
pub(crate) fn predict(body: &RigidBody, dt: f64) -> Result<BodyState, IntegrationError> {
    predict_from_state(body, *body.state(), dt)
}

/// Predicts from an explicit segment-start state using the body's current forces and properties.
pub(crate) fn predict_from_state(
    body: &RigidBody,
    state: BodyState,
    dt: f64,
) -> Result<BodyState, IntegrationError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(IntegrationError::InvalidDeltaTime);
    }

    if state.is_sleeping() {
        return Ok(state);
    }

    match body.physics().body_type() {
        BodyType::Static => Ok(state),
        BodyType::Kinematic => predict_kinematic(body, state, dt),
        BodyType::Dynamic => predict_dynamic(body, state, dt),
    }
}

/// Returns conservative path-length bounds for the integrator's continuous sample trajectory.
pub(crate) fn trajectory_bounds(
    body: &RigidBody,
    start: BodyState,
    end: BodyState,
    dt: f64,
) -> Result<TrajectoryBounds, IntegrationError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(IntegrationError::InvalidDeltaTime);
    }
    if start.is_sleeping() || body.physics().body_type() == BodyType::Static {
        return Ok(TrajectoryBounds {
            translation: 0.0,
            rotation: 0.0,
        });
    }

    let linear_delta = end
        .linear_velocity()
        .checked_sub(start.linear_velocity())
        .and_then(Vec3::checked_length)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let angular_delta = end
        .angular_velocity()
        .checked_sub(start.angular_velocity())
        .and_then(Vec3::checked_length)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let linear_speed_bound = start
        .linear_velocity()
        .checked_length()
        .ok_or(IntegrationError::ArithmeticOverflow)?
        + 2.0 * linear_delta;
    let angular_speed_bound = start
        .angular_velocity()
        .checked_length()
        .ok_or(IntegrationError::ArithmeticOverflow)?
        + 2.0 * angular_delta;
    let rotation = angular_speed_bound * dt;
    let center_of_mass_radius = if let Some(mass) = body.physics().mass() {
        mass.center_of_mass()
            .checked_length()
            .ok_or(IntegrationError::ArithmeticOverflow)?
    } else {
        0.0
    };
    let translation = linear_speed_bound * dt + center_of_mass_radius * rotation;
    if !linear_speed_bound.is_finite()
        || !angular_speed_bound.is_finite()
        || !rotation.is_finite()
        || !translation.is_finite()
    {
        return Err(IntegrationError::ArithmeticOverflow);
    }
    Ok(TrajectoryBounds {
        translation,
        rotation,
    })
}

fn predict_kinematic(
    body: &RigidBody,
    state: BodyState,
    dt: f64,
) -> Result<BodyState, IntegrationError> {
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

fn predict_dynamic(
    body: &RigidBody,
    state: BodyState,
    dt: f64,
) -> Result<BodyState, IntegrationError> {
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
    let torque = body.forces().torque();
    let initial_angular_acceleration =
        world_angular_acceleration(rotation, state.angular_velocity, local_inertia, torque)?;
    let midpoint_angular_velocity = state
        .angular_velocity
        .checked_add(
            initial_angular_acceleration
                .checked_mul_scalar(dt * 0.5)
                .ok_or(IntegrationError::ArithmeticOverflow)?,
        )
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let midpoint_rotation = rotation
        .integrate_world_angular_velocity(midpoint_angular_velocity, dt * 0.5)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let midpoint_angular_acceleration = world_angular_acceleration(
        midpoint_rotation,
        midpoint_angular_velocity,
        local_inertia,
        torque,
    )?;
    let angular_velocity = state
        .angular_velocity
        .checked_add(
            midpoint_angular_acceleration
                .checked_mul_scalar(dt)
                .ok_or(IntegrationError::ArithmeticOverflow)?,
        )
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let new_rotation = rotation
        .integrate_world_angular_velocity(midpoint_angular_velocity, dt)
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

fn world_angular_acceleration(
    rotation: kapitsa_math::Quat,
    angular_velocity: Vec3,
    local_inertia: Mat3,
    torque: Vec3,
) -> Result<Vec3, IntegrationError> {
    let rotation_matrix = rotation
        .to_mat3()
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let world_inertia = rotation_matrix
        .checked_mul_matrix(local_inertia)
        .and_then(|rotated| rotated.checked_mul_matrix(rotation_matrix.transpose()))
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let angular_momentum = world_inertia
        .checked_mul_vector(angular_velocity)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let gyroscopic_torque = angular_velocity
        .checked_cross(angular_momentum)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    let net_torque = torque
        .checked_sub(gyroscopic_torque)
        .ok_or(IntegrationError::ArithmeticOverflow)?;
    solve_spd(world_inertia, net_torque)
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

    fn rotational_energy(body: &RigidBody) -> f64 {
        let mass = body.physics().mass().expect("dynamic body mass");
        let rotation = body.state().transform().rotation();
        let local_omega = rotation
            .conjugate()
            .rotate_vector(body.state().angular_velocity())
            .expect("local angular velocity");
        0.5 * local_omega.dot(mass.inertia() * local_omega)
    }

    fn world_angular_momentum(body: &RigidBody) -> Vec3 {
        let mass = body.physics().mass().expect("dynamic body mass");
        let rotation = body.state().transform().rotation();
        let local_omega = rotation
            .conjugate()
            .rotate_vector(body.state().angular_velocity())
            .expect("local angular velocity");
        rotation
            .rotate_vector(mass.inertia() * local_omega)
            .expect("world angular momentum")
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
    fn resumed_ccd_segment_applies_force_only_for_remaining_time() {
        let properties = MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
        let mut body = body(properties);
        body.apply_force(Vec3::X * 4.0).expect("force");
        let mut impact =
            super::predict_from_state(&body, *body.state(), 0.25).expect("first segment");
        assert_vec3_close(impact.linear_velocity(), Vec3::X * 0.5);
        // An impact changes velocity, while the force remains active for the
        // remaining segment. Reapplying a full-step force would yield 1.8 m/s.
        impact.linear_velocity += Vec3::X * 0.3;
        let resumed = super::predict_from_state(&body, impact, 0.25).expect("remaining segment");
        assert_vec3_close(resumed.linear_velocity(), Vec3::X * 1.3);
        assert_vec3_close(resumed.transform().translation(), Vec3::X * 0.45);
    }

    #[test]
    fn constant_torque_updates_angular_velocity_and_orientation() {
        let mut body = body(mass(Vec3::ZERO, Mat3::IDENTITY));
        body.apply_torque(Vec3::Z * 2.0).expect("finite torque");

        let predicted = predict(&body, 0.5).expect("finite prediction");
        assert_vec3_close(predicted.angular_velocity(), Vec3::Z);
        assert_vec3_close(
            predicted.transform.transform_vector(Vec3::X),
            rotation_z(0.25)
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

        let expected_acceleration = inertia
            .solve_spd(Vec3::new(1.0, -2.0, 0.0))
            .expect("valid SPD solve");
        let acceleration_error = |dt| {
            let response = predict(&body, dt)
                .expect("finite prediction")
                .angular_velocity()
                / dt;
            (response - expected_acceleration).length()
        };
        let coarse_error = acceleration_error(0.25);
        let fine_error = acceleration_error(0.125);

        assert!(expected_acceleration.x > 0.0);
        assert!(expected_acceleration.y < 0.0);
        assert!(fine_error < coarse_error * 0.35);
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

        let dt = 1.0e-7;
        let predicted = predict(&body, dt).expect("finite prediction");
        let expected = Vec3::new(1.0, 2.0, 3.0) + Vec3::new(-3.0, 2.0, -0.5) * dt;
        assert_vec3_close(predicted.angular_velocity(), expected);
    }

    #[test]
    fn free_asymmetric_rotation_preserves_energy_and_angular_momentum() {
        let inertia = Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0));
        let properties = mass(Vec3::ZERO, inertia);
        for dt in [1.0_f64 / 120.0, 1.0 / 240.0] {
            let mut body = body(properties);
            body.set_angular_velocity(Vec3::new(1.0, 2.0, 3.0))
                .expect("finite angular velocity");
            let initial_energy = rotational_energy(&body);
            let initial_momentum = world_angular_momentum(&body);
            let steps = (10.0 / dt).round() as usize;
            for _ in 0..steps {
                let state = predict(&body, dt).expect("finite free rotation");
                body.commit_integrated_state(state);
            }
            let final_energy = rotational_energy(&body);
            let final_momentum = world_angular_momentum(&body);
            println!(
                "free_asymmetric_rotation dt={dt:.9e}s steps={steps} energy_delta={:+.9e}J energy_drift={:+.9e}% angular_momentum_drift={:.9e}",
                final_energy - initial_energy,
                (final_energy - initial_energy) / initial_energy * 100.0,
                (final_momentum - initial_momentum).length() / initial_momentum.length(),
            );
            assert!((final_energy - initial_energy).abs() / initial_energy * 100.0 < 5.0e-4);
            assert!(
                (final_momentum - initial_momentum).length() / initial_momentum.length() < 5.0e-5
            );
        }
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
