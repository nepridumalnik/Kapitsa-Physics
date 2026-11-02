use kapitsa_math::{CholeskyError, Mat3, Quat, Vec3};

use crate::{
    collision::CollisionError,
    mass::MassProperties,
    rigid_body::{BodyError, BodyType, RigidBody},
    transform::Transform,
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

    #[cfg(test)]
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
        let linear_term = self.inverse_mass
            * linear_a
                .checked_dot(linear_b)
                .ok_or(CollisionError::ArithmeticOverflow)?;
        let angular_term = angular_a
            .checked_dot(angular_response)
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let value = linear_term + angular_term;
        value
            .is_finite()
            .then_some(value)
            .ok_or(CollisionError::ArithmeticOverflow)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SolverMassCacheStatus {
    Hit,
    Updated { inverse_inertia: bool },
}

/// Reusable derived mass for one solver-body slot. The body remains the source
/// of truth; this value is valid only while its mass and transform match.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SolverMassSlot {
    cached_mass: Option<MassProperties>,
    cached_transform: Option<Transform>,
    cached_rotation: Option<Quat>,
    inverse_local_inertia: Mat3,
    resolved: SolverMass,
}

impl Default for SolverMassSlot {
    fn default() -> Self {
        Self {
            cached_mass: None,
            cached_transform: None,
            cached_rotation: None,
            inverse_local_inertia: Mat3::ZERO,
            resolved: SolverMass::STATIC,
        }
    }
}

impl SolverMassSlot {
    pub(crate) fn invalidate(&mut self) {
        self.cached_mass = None;
        self.cached_transform = None;
        self.cached_rotation = None;
        self.inverse_local_inertia = Mat3::ZERO;
        self.resolved = SolverMass::STATIC;
    }

    /// Invalidates pose-derived values while retaining the mass factorization.
    pub(crate) fn invalidate_pose(&mut self) {
        self.cached_transform = None;
    }

    pub(crate) fn resolve(
        &mut self,
        body: &RigidBody,
    ) -> Result<(SolverMass, SolverMassCacheStatus), CollisionError> {
        if body.physics().body_type() != BodyType::Dynamic {
            if self.cached_mass.is_some()
                || self.cached_transform.is_some()
                || self.cached_rotation.is_some()
                || self.resolved != SolverMass::STATIC
            {
                self.invalidate();
                return Ok((
                    SolverMass::STATIC,
                    SolverMassCacheStatus::Updated {
                        inverse_inertia: false,
                    },
                ));
            }
            return Ok((SolverMass::STATIC, SolverMassCacheStatus::Hit));
        }

        let mass = body
            .physics()
            .mass()
            .ok_or(CollisionError::InvalidContact)?;
        if self.cached_mass != Some(mass) {
            self.inverse_local_inertia = inverse_spd(mass.inertia())?;
            self.cached_mass = Some(mass);
            self.cached_transform = None;
            self.cached_rotation = None;
        }

        let transform = body.state().transform();
        if self.cached_transform == Some(transform) {
            return Ok((self.resolved, SolverMassCacheStatus::Hit));
        }

        let rotation_changed = self.cached_rotation != Some(transform.rotation());
        let inverse_inertia = if rotation_changed {
            let rotation = transform
                .rotation()
                .to_mat3()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            rotation
                .checked_mul_matrix(self.inverse_local_inertia)
                .and_then(|matrix| matrix.checked_mul_matrix(rotation.transpose()))
                .ok_or(CollisionError::ArithmeticOverflow)?
        } else {
            self.resolved.inverse_inertia
        };
        let inverse_mass = 1.0 / mass.mass();
        if !inverse_mass.is_finite() || inverse_mass <= 0.0 {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let center = body
            .world_center_of_mass()
            .map_err(|_| CollisionError::ArithmeticOverflow)?
            .ok_or(CollisionError::InvalidContact)?;
        self.resolved = SolverMass {
            inverse_mass,
            inverse_inertia,
            center,
        };
        if rotation_changed {
            self.cached_rotation = Some(transform.rotation());
        }
        self.cached_transform = Some(transform);
        Ok((
            self.resolved,
            SolverMassCacheStatus::Updated {
                inverse_inertia: rotation_changed,
            },
        ))
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

    fn assert_solver_mass_close(actual: SolverMass, expected: SolverMass) {
        assert_close(actual.inverse_mass, expected.inverse_mass);
        for (actual, expected) in actual
            .inverse_inertia
            .to_cols_array()
            .into_iter()
            .zip(expected.inverse_inertia.to_cols_array())
        {
            assert_close(actual, expected);
        }
        for (actual, expected) in [
            (actual.center.x, expected.center.x),
            (actual.center.y, expected.center.y),
            (actual.center.z, expected.center.z),
        ] {
            assert_close(actual, expected);
        }
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
        let mut slot_a = SolverMassSlot::default();
        let mut slot_b = SolverMassSlot::default();
        let cached_a = slot_a.resolve(body_a).expect("slot mass A").0;
        let cached_b = slot_b.resolve(body_b).expect("slot mass B").0;
        assert_solver_mass_close(
            cached_a,
            SolverMass::from_body(body_a).expect("reference mass A"),
        );
        assert_solver_mass_close(
            cached_b,
            SolverMass::from_body(body_b).expect("reference mass B"),
        );
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
        let actual =
            constraint::effective_cross_mass_cached([cached_a, cached_b], measurement, impulse)
                .expect("cached effective mass");
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
    fn solver_mass_slot_hits_for_unchanged_mass_and_pose() {
        let body = dynamic_body(
            Transform::identity(),
            mass(Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0)), Vec3::ZERO),
        );
        let mut slot = SolverMassSlot::default();

        let (first, first_status) = slot.resolve(&body).expect("initial slot resolve");
        let (second, second_status) = slot.resolve(&body).expect("cached slot resolve");

        assert_eq!(
            first_status,
            SolverMassCacheStatus::Updated {
                inverse_inertia: true
            }
        );
        assert_eq!(second_status, SolverMassCacheStatus::Hit);
        assert_eq!(first, second);
    }

    #[test]
    fn solver_mass_slot_keeps_static_and_kinematic_response_zero() {
        let mut static_slot = SolverMassSlot::default();
        let mut kinematic_slot = SolverMassSlot::default();
        let static_body = RigidBody::new_static(Transform::identity());
        let kinematic_body = RigidBody::new_kinematic(Transform::identity());
        let (static_mass, _) = static_slot.resolve(&static_body).expect("static mass");
        let (kinematic_mass, _) = kinematic_slot
            .resolve(&kinematic_body)
            .expect("kinematic mass");
        let jacobians = [
            Jacobian {
                linear: Vec3::X,
                angular: Vec3::Y,
            },
            Jacobian {
                linear: Vec3::Z,
                angular: Vec3::X,
            },
        ];

        assert_eq!(static_mass, SolverMass::STATIC);
        assert_eq!(kinematic_mass, SolverMass::STATIC);
        assert_eq!(
            constraint::effective_cross_mass_cached(
                [static_mass, kinematic_mass],
                jacobians,
                jacobians,
            )
            .expect("zero response"),
            0.0
        );
    }

    #[test]
    fn solver_mass_slot_translation_refreshes_center_without_inertia_update() {
        let mut body = dynamic_body(
            Transform::identity(),
            mass(Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0)), Vec3::ZERO),
        );
        let mut slot = SolverMassSlot::default();
        slot.resolve(&body).expect("initial slot resolve");
        body.apply_constraint_position_impulse(Vec3::X, Vec3::ZERO)
            .expect("translation correction");
        slot.invalidate_pose();

        let (actual, status) = slot.resolve(&body).expect("updated slot resolve");
        assert_eq!(
            status,
            SolverMassCacheStatus::Updated {
                inverse_inertia: false
            }
        );
        assert_solver_mass_close(
            actual,
            SolverMass::from_body(&body).expect("reference mass"),
        );
    }

    #[test]
    fn solver_mass_slot_updates_world_inertia_after_orientation_change() {
        let inertia = Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0));
        let mut body = dynamic_body(
            Transform::identity(),
            mass(inertia, Vec3::new(0.2, -0.4, 0.3)),
        );
        let mut slot = SolverMassSlot::default();
        slot.resolve(&body).expect("initial slot resolve");
        body.apply_constraint_position_impulse(Vec3::new(0.1, -0.2, 0.3), Vec3::new(0.0, 0.0, 0.4))
            .expect("position correction");
        slot.invalidate_pose();

        let (actual, status) = slot.resolve(&body).expect("updated slot resolve");
        assert_eq!(
            status,
            SolverMassCacheStatus::Updated {
                inverse_inertia: true
            }
        );
        assert_solver_mass_close(
            actual,
            SolverMass::from_body(&body).expect("reference mass"),
        );
    }

    #[test]
    fn solver_mass_slot_rebuilds_after_mass_change() {
        let mut body = dynamic_body(
            Transform::identity(),
            mass(Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0)), Vec3::ZERO),
        );
        let mut slot = SolverMassSlot::default();
        slot.resolve(&body).expect("initial slot resolve");
        let changed = MassProperties::new(
            4.0,
            Vec3::new(0.1, -0.3, 0.2),
            Mat3::from_diagonal(Vec3::new(5.0, 7.0, 9.0)),
        )
        .expect("changed mass");
        body.set_mass(changed).expect("set changed mass");

        let (actual, status) = slot.resolve(&body).expect("updated slot resolve");
        assert_eq!(
            status,
            SolverMassCacheStatus::Updated {
                inverse_inertia: true
            }
        );
        assert_solver_mass_close(
            actual,
            SolverMass::from_body(&body).expect("reference mass"),
        );
    }

    #[test]
    fn solver_mass_slot_handles_mass_and_orientation_change_together() {
        let mut body = dynamic_body(
            Transform::identity(),
            mass(Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0)), Vec3::ZERO),
        );
        let mut slot = SolverMassSlot::default();
        slot.resolve(&body).expect("initial slot resolve");
        body.set_mass(
            MassProperties::new(
                3.5,
                Vec3::new(-0.2, 0.1, 0.4),
                Mat3::from_diagonal(Vec3::new(4.0, 6.0, 8.0)),
            )
            .expect("changed mass"),
        )
        .expect("set changed mass");
        body.apply_constraint_position_impulse(Vec3::ZERO, Vec3::new(0.2, -0.1, 0.3))
            .expect("position correction");

        let (actual, status) = slot.resolve(&body).expect("updated slot resolve");
        assert_eq!(
            status,
            SolverMassCacheStatus::Updated {
                inverse_inertia: true
            }
        );
        assert_solver_mass_close(
            actual,
            SolverMass::from_body(&body).expect("reference mass"),
        );
    }

    #[test]
    fn solver_mass_slot_reuse_resolves_the_new_body_state() {
        let first = dynamic_body(
            Transform::identity(),
            mass(Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0)), Vec3::ZERO),
        );
        let second = dynamic_body(
            Transform::new(
                Vec3::new(3.0, -2.0, 1.0),
                Quat::from_axis_angle(Vec3::Y, 0.7).expect("rotation"),
            )
            .expect("transform"),
            MassProperties::new(
                4.5,
                Vec3::new(0.1, 0.2, -0.3),
                Mat3::from_diagonal(Vec3::new(5.0, 6.0, 7.0)),
            )
            .expect("mass"),
        );
        let mut slot = SolverMassSlot::default();
        slot.resolve(&first).expect("first body resolve");

        let (actual, status) = slot.resolve(&second).expect("reused body slot resolve");
        assert_eq!(
            status,
            SolverMassCacheStatus::Updated {
                inverse_inertia: true
            }
        );
        assert_solver_mass_close(
            actual,
            SolverMass::from_body(&second).expect("reference mass"),
        );
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
