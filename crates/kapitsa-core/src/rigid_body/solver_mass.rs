use kapitsa_math::{CholeskyError, Mat3, Vec3};

use crate::{
    collision::CollisionError,
    rigid_body::{BodyError, BodyType, RigidBody},
};

/// Derived from the body's current transform and mass; never authoritative.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SolverMass {
    pub(crate) inverse_mass: f64,
    pub(crate) inverse_inertia: Mat3,
    pub(crate) center: Vec3,
}

impl SolverMass {
    pub(crate) const STATIC: Self = Self {
        inverse_mass: 0.0,
        inverse_inertia: Mat3::ZERO,
        center: Vec3::ZERO,
    };

    pub(crate) fn from_body(body: &RigidBody) -> Result<Self, CollisionError> {
        if body.physics().body_type() != BodyType::Dynamic {
            return Ok(Self::STATIC);
        }
        let mass = body
            .physics()
            .mass()
            .ok_or(CollisionError::InvalidContact)?;
        let rotation = body.state().transform().rotation();
        let inverse_local_inertia = inverse_spd(mass.inertia())?;
        let x = rotation
            .rotate_vector(Vec3::X)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let y = rotation
            .rotate_vector(Vec3::Y)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let z = rotation
            .rotate_vector(Vec3::Z)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let rotation = Mat3::from_cols(x, y, z);
        let inverse_inertia = rotation
            .checked_mul_matrix(inverse_local_inertia)
            .and_then(|matrix| matrix.checked_mul_matrix(rotation.transpose()))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let center = body
            .world_center_of_mass()
            .map_err(|_| CollisionError::ArithmeticOverflow)?
            .ok_or(CollisionError::InvalidContact)?;
        let inverse_mass = 1.0 / mass.mass();
        if !inverse_mass.is_finite() || inverse_mass <= 0.0 {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(Self {
            inverse_mass,
            inverse_inertia,
            center,
        })
    }

    /// For generalized Jacobian J=(linear, angular): J M^-1 J^T.
    pub(crate) fn diagonal(self, linear: Vec3, angular: Vec3) -> Result<f64, CollisionError> {
        self.cross(linear, angular, linear, angular)
    }

    pub(crate) fn cross(
        self,
        linear_a: Vec3,
        angular_a: Vec3,
        linear_b: Vec3,
        angular_b: Vec3,
    ) -> Result<f64, CollisionError> {
        let angular_response = self
            .inverse_inertia
            .checked_mul_vector(angular_b)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let value = self.inverse_mass * linear_a.dot(linear_b) + angular_a.dot(angular_response);
        value
            .is_finite()
            .then_some(value)
            .ok_or(CollisionError::ArithmeticOverflow)
    }
}

fn inverse_spd(matrix: Mat3) -> Result<Mat3, CollisionError> {
    let x = matrix.solve_spd(Vec3::X).map_err(map_cholesky_error)?;
    let y = matrix.solve_spd(Vec3::Y).map_err(map_cholesky_error)?;
    let z = matrix.solve_spd(Vec3::Z).map_err(map_cholesky_error)?;
    let inverse = Mat3::from_cols(x, y, z);
    inverse
        .is_finite()
        .then_some(inverse)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn map_cholesky_error(error: CholeskyError) -> CollisionError {
    match error {
        CholeskyError::NonFiniteInput | CholeskyError::ResultOutOfRange => {
            CollisionError::ArithmeticOverflow
        }
        CholeskyError::NotSymmetric | CholeskyError::NotPositiveDefinite => {
            CollisionError::InvalidContact
        }
    }
}

impl RigidBody {
    pub(crate) fn apply_constraint_impulse_cached(
        &mut self,
        linear_impulse: Vec3,
        angular_impulse: Vec3,
        cached: SolverMass,
    ) -> Result<(), BodyError> {
        if self.physics.body_type != BodyType::Dynamic {
            return Err(BodyError::BodyMustBeDynamic);
        }
        if !linear_impulse.is_finite() || !angular_impulse.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }
        let delta_linear_velocity = linear_impulse
            .checked_mul_scalar(cached.inverse_mass)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let delta_angular_velocity = cached
            .inverse_inertia
            .checked_mul_vector(angular_impulse)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let linear_velocity = self
            .state
            .linear_velocity
            .checked_add(delta_linear_velocity)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let angular_velocity = self
            .state
            .angular_velocity
            .checked_add(delta_angular_velocity)
            .ok_or(BodyError::ArithmeticOverflow)?;

        self.state.linear_velocity = linear_velocity;
        self.state.angular_velocity = angular_velocity;
        if linear_impulse != Vec3::ZERO || angular_impulse != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        constraint::{self, Jacobian},
        mass::MassProperties,
        rigid_body::CcdMode,
        transform::Transform,
    };
    use kapitsa_math::{Quat, Vec3};

    fn mass(inertia: Mat3, center: Vec3) -> MassProperties {
        MassProperties::new(2.5, center, inertia).expect("valid physical mass properties")
    }

    fn dynamic_body(transform: Transform, mass: MassProperties) -> RigidBody {
        RigidBody::new_dynamic(transform, mass).expect("valid dynamic body")
    }

    fn assert_close(actual: f64, expected: f64) {
        let error = (actual - expected).abs();
        let tolerance = 2.0e-10 * expected.abs().max(1.0);
        assert!(
            error <= tolerance,
            "actual={actual}, expected={expected}, error={error}"
        );
    }

    fn jacobian(linear: Vec3, arm: Vec3) -> Jacobian {
        Jacobian {
            linear,
            angular: arm.cross(linear),
        }
    }

    fn assert_pair_matches_reference(body_a: &RigidBody, body_b: &RigidBody) {
        let measurement = [
            jacobian(Vec3::new(0.3, -0.7, 0.2), Vec3::new(0.4, 0.1, -0.3)),
            jacobian(Vec3::new(-0.2, 0.5, 0.9), Vec3::new(-0.1, 0.6, 0.2)),
        ];
        let impulse = [
            jacobian(Vec3::new(-0.6, 0.1, 0.4), Vec3::new(0.2, -0.5, 0.7)),
            jacobian(Vec3::new(0.8, -0.3, 0.1), Vec3::new(0.5, 0.2, -0.4)),
        ];
        let cached_a = SolverMass::from_body(body_a).expect("mass A");
        let cached_b = SolverMass::from_body(body_b).expect("mass B");
        assert_eq!(
            cached_a.center,
            body_a
                .world_center_of_mass()
                .expect("center A")
                .unwrap_or(Vec3::ZERO)
        );
        assert_eq!(
            cached_b.center,
            body_b
                .world_center_of_mass()
                .expect("center B")
                .unwrap_or(Vec3::ZERO)
        );
        let actual = cached_a
            .cross(
                measurement[0].linear,
                measurement[0].angular,
                impulse[0].linear,
                impulse[0].angular,
            )
            .expect("cached contribution A")
            + cached_b
                .cross(
                    measurement[1].linear,
                    measurement[1].angular,
                    impulse[1].linear,
                    impulse[1].angular,
                )
                .expect("cached contribution B");
        let expected = constraint::effective_cross_mass(body_a, body_b, measurement, impulse)
            .expect("reference effective mass");
        assert_close(actual, expected);

        let actual_diagonal = cached_a
            .diagonal(measurement[0].linear, measurement[0].angular)
            .expect("cached diagonal A")
            + cached_b
                .diagonal(measurement[1].linear, measurement[1].angular)
                .expect("cached diagonal B");
        let expected_diagonal =
            constraint::effective_cross_mass(body_a, body_b, measurement, measurement)
                .expect("reference diagonal");
        assert_close(actual_diagonal, expected_diagonal);
    }

    #[test]
    fn solver_mass_matches_reference_for_identity_rotated_and_off_center_bodies() {
        let anisotropic = Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0));
        let rotation =
            Quat::from_axis_angle(Vec3::new(1.0, 2.0, -0.5), 0.73).expect("valid rotation");
        let bodies = [
            dynamic_body(Transform::identity(), mass(anisotropic, Vec3::ZERO)),
            dynamic_body(
                Transform::new(Vec3::new(2.0, -1.0, 0.5), rotation).expect("transform"),
                mass(anisotropic, Vec3::new(0.3, -0.2, 0.4)),
            ),
            dynamic_body(
                Transform::new(Vec3::new(-1.0, 0.5, 3.0), Quat::IDENTITY).expect("transform"),
                mass(
                    Mat3::from_diagonal(Vec3::new(1.0e-6, 1.0e6, 1.0e6)),
                    Vec3::ZERO,
                ),
            ),
        ];
        for body in &bodies {
            assert_pair_matches_reference(body, &RigidBody::new_static(Transform::identity()));
        }
    }

    #[test]
    fn solver_mass_handles_static_kinematic_and_dynamic_pairs() {
        let dynamic = dynamic_body(Transform::identity(), mass(Mat3::IDENTITY, Vec3::ZERO));
        let static_body = RigidBody::new_static(Transform::identity());
        let kinematic_body = RigidBody::new_kinematic(Transform::identity());
        assert_eq!(
            SolverMass::from_body(&static_body).expect("static mass"),
            SolverMass::STATIC
        );
        assert_eq!(
            SolverMass::from_body(&kinematic_body).expect("kinematic mass"),
            SolverMass::STATIC
        );
        assert_pair_matches_reference(&static_body, &dynamic);
        assert_pair_matches_reference(&dynamic, &kinematic_body);
        assert_pair_matches_reference(&dynamic, &dynamic);
    }

    #[test]
    fn inverse_spd_rejects_non_positive_definite_inputs() {
        assert_eq!(inverse_spd(Mat3::ZERO), Err(CollisionError::InvalidContact));
    }

    #[test]
    fn cached_impulse_application_matches_the_reference_body_operation() {
        let inertia = Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0));
        let rotation =
            Quat::from_axis_angle(Vec3::new(0.2, 1.0, -0.3), 0.91).expect("valid rotation");
        let mut initial = dynamic_body(
            Transform::new(Vec3::new(1.0, -2.0, 0.5), rotation).expect("transform"),
            mass(inertia, Vec3::new(0.2, -0.4, 0.3)),
        );
        initial
            .set_linear_velocity(Vec3::new(0.5, -1.0, 0.25))
            .expect("linear velocity");
        initial
            .set_angular_velocity(Vec3::new(-0.2, 0.4, 0.1))
            .expect("angular velocity");
        initial.set_ccd_mode(CcdMode::Continuous);
        let linear_impulse = Vec3::new(0.7, -0.1, 0.3);
        let angular_impulse = Vec3::new(-0.4, 0.6, 0.2);
        let cached = SolverMass::from_body(&initial).expect("cached mass");
        let mut reference = initial;
        let mut optimized = initial;

        reference
            .apply_constraint_impulse(linear_impulse, angular_impulse)
            .expect("reference impulse");
        optimized
            .apply_constraint_impulse_cached(linear_impulse, angular_impulse, cached)
            .expect("cached impulse");

        for (actual, expected) in [
            (
                optimized.state().linear_velocity().x,
                reference.state().linear_velocity().x,
            ),
            (
                optimized.state().linear_velocity().y,
                reference.state().linear_velocity().y,
            ),
            (
                optimized.state().linear_velocity().z,
                reference.state().linear_velocity().z,
            ),
            (
                optimized.state().angular_velocity().x,
                reference.state().angular_velocity().x,
            ),
            (
                optimized.state().angular_velocity().y,
                reference.state().angular_velocity().y,
            ),
            (
                optimized.state().angular_velocity().z,
                reference.state().angular_velocity().z,
            ),
        ] {
            assert_close(actual, expected);
        }
        assert_eq!(
            optimized.state().is_sleeping(),
            reference.state().is_sleeping()
        );
        assert_eq!(optimized.ccd_mode(), reference.ccd_mode());
    }
}
