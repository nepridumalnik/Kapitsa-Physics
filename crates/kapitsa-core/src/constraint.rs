use kapitsa_math::{CholeskyError, Vec3};

use crate::{
    mass::MassProperties,
    rigid_body::{BodyType, RigidBody},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ConstraintError {
    InvalidMass,
    ArithmeticOverflow,
    NonConvergent,
}

#[derive(Clone, Copy)]
pub(crate) struct Jacobian {
    pub(crate) linear: Vec3,
    pub(crate) angular: Vec3,
}

fn inverse_inertia_world_with_mass(
    body: &RigidBody,
    mass: MassProperties,
    impulse: Vec3,
) -> Result<Vec3, ConstraintError> {
    let rotation = body.state().transform().rotation();
    let local = rotation
        .conjugate()
        .rotate_vector(impulse)
        .ok_or(ConstraintError::ArithmeticOverflow)?;
    let local_response = mass
        .inertia()
        .solve_spd(local)
        .map_err(map_cholesky_error)?;
    rotation
        .rotate_vector(local_response)
        .ok_or(ConstraintError::ArithmeticOverflow)
}

pub(crate) fn effective_cross_mass_at(
    body_a: &RigidBody,
    body_b: &RigidBody,
    position_a: Vec3,
    position_b: Vec3,
    measurement: Vec3,
    impulse: Vec3,
) -> Result<f64, ConstraintError> {
    let mut result = 0.0;
    for (body, position) in [(body_a, position_a), (body_b, position_b)] {
        if body.physics().body_type() != BodyType::Dynamic {
            continue;
        }
        let mass = body.physics().mass().ok_or(ConstraintError::InvalidMass)?;
        let linear_term = mass.mass().recip()
            * measurement
                .checked_dot(impulse)
                .ok_or(ConstraintError::ArithmeticOverflow)?;
        let center = body
            .world_center_of_mass()
            .map_err(map_body_error)?
            .unwrap_or(body.state().transform().translation());
        let offset = position
            .checked_sub(center)
            .ok_or(ConstraintError::ArithmeticOverflow)?;
        let torque = offset
            .checked_cross(impulse)
            .ok_or(ConstraintError::ArithmeticOverflow)?;
        let response = inverse_inertia_world_with_mass(body, mass, torque)?;
        let angular_velocity = response
            .checked_cross(offset)
            .ok_or(ConstraintError::ArithmeticOverflow)?;
        let angular_term = measurement
            .checked_dot(angular_velocity)
            .ok_or(ConstraintError::ArithmeticOverflow)?;
        result += linear_term + angular_term;
        if !result.is_finite() {
            return Err(ConstraintError::ArithmeticOverflow);
        }
    }
    Ok(result)
}

/// Generalized inverse-mass matrix entry for two scalar Jacobian rows.
pub(crate) fn effective_cross_mass(
    body_a: &RigidBody,
    body_b: &RigidBody,
    measurement: [Jacobian; 2],
    impulse: [Jacobian; 2],
) -> Result<f64, ConstraintError> {
    let mut result = 0.0;
    for (body, measurement, impulse) in [
        (body_a, measurement[0], impulse[0]),
        (body_b, measurement[1], impulse[1]),
    ] {
        if body.physics().body_type() != BodyType::Dynamic {
            continue;
        }
        let mass = body.physics().mass().ok_or(ConstraintError::InvalidMass)?;
        result += mass.mass().recip()
            * measurement
                .linear
                .checked_dot(impulse.linear)
                .ok_or(ConstraintError::ArithmeticOverflow)?;
        result += measurement
            .angular
            .checked_dot(inverse_inertia_world_with_mass(
                body,
                mass,
                impulse.angular,
            )?)
            .ok_or(ConstraintError::ArithmeticOverflow)?;
        if !result.is_finite() {
            return Err(ConstraintError::ArithmeticOverflow);
        }
    }
    Ok(result)
}

pub(crate) fn point_velocity(body: &RigidBody, position: Vec3) -> Result<Vec3, ConstraintError> {
    let center = body
        .world_center_of_mass()
        .map_err(map_body_error)?
        .unwrap_or(body.state().transform().translation());
    let offset = position
        .checked_sub(center)
        .ok_or(ConstraintError::ArithmeticOverflow)?;
    body.state()
        .angular_velocity()
        .checked_cross(offset)
        .and_then(|angular| angular.checked_add(body.state().linear_velocity()))
        .ok_or(ConstraintError::ArithmeticOverflow)
}

pub(crate) fn relative_velocity_at(
    body_a: &RigidBody,
    body_b: &RigidBody,
    position_a: Vec3,
    position_b: Vec3,
) -> Result<Vec3, ConstraintError> {
    point_velocity(body_b, position_b)?
        .checked_sub(point_velocity(body_a, position_a)?)
        .ok_or(ConstraintError::ArithmeticOverflow)
}

pub(crate) fn apply_point_impulse_pair(
    body_a: &mut RigidBody,
    body_b: &mut RigidBody,
    point_a: Vec3,
    point_b: Vec3,
    impulse_on_b: Vec3,
) -> Result<(), ConstraintError> {
    if body_a.physics().body_type() == BodyType::Dynamic {
        body_a
            .apply_impulse_at_point(-impulse_on_b, point_a)
            .map_err(map_body_error)?;
    }
    if body_b.physics().body_type() == BodyType::Dynamic {
        body_b
            .apply_impulse_at_point(impulse_on_b, point_b)
            .map_err(map_body_error)?;
    }
    Ok(())
}

pub(crate) fn apply_point_position_impulse_pair(
    body_a: &mut RigidBody,
    body_b: &mut RigidBody,
    point_a: Vec3,
    point_b: Vec3,
    impulse_on_b: Vec3,
) -> Result<(), ConstraintError> {
    if body_a.physics().body_type() == BodyType::Dynamic {
        body_a
            .apply_position_impulse(-impulse_on_b, point_a)
            .map_err(map_body_error)?;
    }
    if body_b.physics().body_type() == BodyType::Dynamic {
        body_b
            .apply_position_impulse(impulse_on_b, point_b)
            .map_err(map_body_error)?;
    }
    Ok(())
}

pub(crate) fn apply_generalized_impulse_pair(
    body_a: &mut RigidBody,
    body_b: &mut RigidBody,
    linear_a: Vec3,
    angular_a: Vec3,
    linear_b: Vec3,
    angular_b: Vec3,
) -> Result<(), ConstraintError> {
    if body_a.physics().body_type() == BodyType::Dynamic {
        body_a
            .apply_constraint_impulse(linear_a, angular_a)
            .map_err(map_body_error)?;
    }
    if body_b.physics().body_type() == BodyType::Dynamic {
        body_b
            .apply_constraint_impulse(linear_b, angular_b)
            .map_err(map_body_error)?;
    }
    Ok(())
}

pub(crate) fn apply_generalized_position_impulse_pair(
    body_a: &mut RigidBody,
    body_b: &mut RigidBody,
    linear_a: Vec3,
    angular_a: Vec3,
    linear_b: Vec3,
    angular_b: Vec3,
) -> Result<(), ConstraintError> {
    if body_a.physics().body_type() == BodyType::Dynamic {
        body_a
            .apply_constraint_position_impulse(linear_a, angular_a)
            .map_err(map_body_error)?;
    }
    if body_b.physics().body_type() == BodyType::Dynamic {
        body_b
            .apply_constraint_position_impulse(linear_b, angular_b)
            .map_err(map_body_error)?;
    }
    Ok(())
}

fn map_body_error(error: crate::rigid_body::BodyError) -> ConstraintError {
    match error {
        crate::rigid_body::BodyError::ArithmeticOverflow
        | crate::rigid_body::BodyError::NonFiniteValue => ConstraintError::ArithmeticOverflow,
        crate::rigid_body::BodyError::InvalidInertiaTensor => ConstraintError::NonConvergent,
        crate::rigid_body::BodyError::BodyMustBeDynamic
        | crate::rigid_body::BodyError::StaticBodyCannotHaveVelocity => {
            ConstraintError::InvalidMass
        }
    }
}

fn map_cholesky_error(error: CholeskyError) -> ConstraintError {
    match error {
        CholeskyError::NonFiniteInput | CholeskyError::ResultOutOfRange => {
            ConstraintError::ArithmeticOverflow
        }
        CholeskyError::NotSymmetric | CholeskyError::NotPositiveDefinite => {
            ConstraintError::NonConvergent
        }
    }
}
