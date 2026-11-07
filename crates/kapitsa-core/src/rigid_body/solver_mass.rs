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

pub(crate) fn inverse_spd(matrix: Mat3) -> Result<Mat3, CollisionError> {
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
    /// Position correction driven by a mass resolved for the current pose.
    ///
    /// The caller must guarantee that `cached` belongs to this body's current
    /// transform; otherwise the correction uses stale centers or inertia.
    pub(crate) fn apply_constraint_position_impulse_cached(
        &mut self,
        linear_impulse: Vec3,
        angular_impulse: Vec3,
        cached: SolverMass,
    ) -> Result<(), BodyError> {
        if self.physics.body_type != BodyType::Dynamic {
            return Ok(());
        }
        if !linear_impulse.is_finite() || !angular_impulse.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }
        if linear_impulse == Vec3::ZERO && angular_impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let rotation = self.state.transform.rotation();
        let rotation_delta = cached
            .inverse_inertia
            .checked_mul_vector(angular_impulse)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let angle = rotation_delta
            .checked_length()
            .ok_or(BodyError::ArithmeticOverflow)?;
        let rotation_delta = if angle > 0.0 {
            Quat::from_axis_angle(rotation_delta / angle, angle)
                .ok_or(BodyError::ArithmeticOverflow)?
        } else {
            Quat::IDENTITY
        };
        let next_rotation = (rotation_delta * rotation)
            .try_normalize()
            .filter(|rotation| rotation.is_normalized())
            .ok_or(BodyError::ArithmeticOverflow)?;
        let linear_delta = linear_impulse
            .checked_mul_scalar(cached.inverse_mass)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let next_center = cached
            .center
            .checked_add(linear_delta)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let next_translation = next_center
            .checked_sub(
                next_rotation
                    .rotate_vector(mass.center_of_mass())
                    .ok_or(BodyError::ArithmeticOverflow)?,
            )
            .ok_or(BodyError::ArithmeticOverflow)?;
        let next_transform = Transform::new(next_translation, next_rotation)
            .map_err(|_| BodyError::ArithmeticOverflow)?;
        self.set_position_projected_transform(next_transform, Some(cached.inverse_inertia))
    }

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
#[path = "solver_mass_tests.rs"]
mod tests;
