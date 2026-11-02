pub mod error;
mod forces;
pub(crate) mod integrator;
mod physics;
#[allow(dead_code)] // This reference kernel is integrated by the next solver stage.
pub(crate) mod solver_mass;
mod state;

pub use error::BodyError;
pub use forces::BodyForces;
pub use integrator::IntegrationError;
pub use physics::{BodyPhysics, BodyType};
pub use state::BodyState;

/// Continuous collision detection policy for a rigid body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CcdMode {
    Discrete,
    Continuous,
}

pub(crate) use integrator::{predict, predict_from_state, trajectory_bounds};

use kapitsa_math::{CholeskyError, Quat, Vec3};

use crate::{mass::MassProperties, transform::Transform};

/// A rigid body's state, physical properties, and accumulated world-space forces.
///
/// The transform describes the body's world-space local origin. Its center of mass
/// may be offset from that origin by [`MassProperties::center_of_mass`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidBody {
    state: BodyState,
    physics: BodyPhysics,
    forces: BodyForces,
    ccd_mode: CcdMode,
}

impl RigidBody {
    /// Creates a stationary body with no mass properties.
    pub fn new_static(transform: Transform) -> Self {
        Self::new(BodyType::Static, transform, None)
    }

    /// Creates a stationary kinematic body with no mass properties.
    pub fn new_kinematic(transform: Transform) -> Self {
        Self::new(BodyType::Kinematic, transform, None)
    }

    /// Creates a stationary dynamic body with the supplied mass properties.
    pub fn new_dynamic(transform: Transform, mass: MassProperties) -> Result<Self, BodyError> {
        let body = Self::new(BodyType::Dynamic, transform, Some(mass));
        body.world_center_of_mass()?;
        Ok(body)
    }

    fn new(body_type: BodyType, transform: Transform, mass: Option<MassProperties>) -> Self {
        Self {
            state: BodyState {
                transform,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
                sleeping: false,
            },
            physics: BodyPhysics { body_type, mass },
            forces: BodyForces {
                force: Vec3::ZERO,
                torque: Vec3::ZERO,
            },
            ccd_mode: CcdMode::Discrete,
        }
    }

    pub fn state(&self) -> &BodyState {
        &self.state
    }

    pub fn physics(&self) -> &BodyPhysics {
        &self.physics
    }

    pub fn forces(&self) -> &BodyForces {
        &self.forces
    }

    pub fn ccd_mode(&self) -> CcdMode {
        self.ccd_mode
    }

    pub fn set_ccd_mode(&mut self, mode: CcdMode) {
        self.ccd_mode = mode;
    }

    /// Replaces the world transform while preserving velocities.
    ///
    /// A changed transform wakes a sleeping dynamic body. The update is rejected
    /// if its world-space center of mass overflows.
    pub fn set_transform(&mut self, transform: Transform) -> Result<(), BodyError> {
        world_center_of_mass_for(transform, self.physics.mass)?;

        if transform != self.state.transform {
            self.state.transform = transform;
            if self.physics.body_type == BodyType::Dynamic {
                self.state.sleeping = false;
            }
        }
        Ok(())
    }

    /// Sets the world-space linear velocity of the center of mass, in m/s.
    pub fn set_linear_velocity(&mut self, velocity: Vec3) -> Result<(), BodyError> {
        self.ensure_velocity_is_allowed()?;
        if !velocity.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }

        self.state.linear_velocity = velocity;
        if self.physics.body_type == BodyType::Dynamic && velocity != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Sets the world-space angular velocity, in rad/s.
    pub fn set_angular_velocity(&mut self, velocity: Vec3) -> Result<(), BodyError> {
        self.ensure_velocity_is_allowed()?;
        if !velocity.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }

        self.state.angular_velocity = velocity;
        if self.physics.body_type == BodyType::Dynamic && velocity != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Replaces dynamic mass properties without changing position or velocity.
    ///
    /// Changed mass properties wake the body and clear accumulated forces and
    /// torques, whose torque reference may otherwise be stale.
    pub fn set_mass(&mut self, mass: MassProperties) -> Result<(), BodyError> {
        if self.physics.body_type != BodyType::Dynamic {
            return Err(BodyError::BodyMustBeDynamic);
        }
        world_center_of_mass_for(self.state.transform, Some(mass))?;
        if self.physics.mass != Some(mass) {
            self.physics.mass = Some(mass);
            self.state.sleeping = false;
            self.forces.force = Vec3::ZERO;
            self.forces.torque = Vec3::ZERO;
        }
        Ok(())
    }

    /// Returns the center of mass in world space, or `None` for bodies without mass.
    pub fn world_center_of_mass(&self) -> Result<Option<Vec3>, BodyError> {
        world_center_of_mass_for(self.state.transform, self.physics.mass)
    }

    /// Puts a dynamic body to sleep, clearing its velocities and accumulated forces.
    pub fn sleep(&mut self) -> Result<(), BodyError> {
        if self.physics.body_type != BodyType::Dynamic {
            return Err(BodyError::BodyMustBeDynamic);
        }

        self.state.sleeping = true;
        self.state.linear_velocity = Vec3::ZERO;
        self.state.angular_velocity = Vec3::ZERO;
        self.forces.force = Vec3::ZERO;
        self.forces.torque = Vec3::ZERO;
        Ok(())
    }

    /// Wakes a dynamic body. Other body types are unchanged.
    pub fn wake_up(&mut self) {
        if self.physics.body_type == BodyType::Dynamic {
            self.state.sleeping = false;
        }
    }

    fn ensure_velocity_is_allowed(&self) -> Result<(), BodyError> {
        if self.physics.body_type == BodyType::Static {
            Err(BodyError::StaticBodyCannotHaveVelocity)
        } else {
            Ok(())
        }
    }

    /// Accumulates a world-space force in newtons without changing velocity immediately.
    pub fn apply_force(&mut self, force: Vec3) -> Result<(), BodyError> {
        self.ensure_dynamic()?;
        if !force.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }

        let next_force = add_checked(self.forces.force, force)?;
        self.forces.force = next_force;
        if force != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Accumulates force in newtons and torque about the center of mass in N·m.
    ///
    /// `point` is expressed in world-space meters.
    pub fn apply_force_at_point(&mut self, force: Vec3, point: Vec3) -> Result<(), BodyError> {
        self.ensure_dynamic()?;
        if !force.is_finite() || !point.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }

        let center = world_center_of_mass_for(self.state.transform, self.physics.mass)?
            .ok_or(BodyError::BodyMustBeDynamic)?;
        let torque_delta = point
            .checked_sub(center)
            .and_then(|offset| offset.checked_cross(force))
            .ok_or(BodyError::ArithmeticOverflow)?;
        let next_force = add_checked(self.forces.force, force)?;
        let next_torque = add_checked(self.forces.torque, torque_delta)?;

        self.forces.force = next_force;
        self.forces.torque = next_torque;
        if force != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Accumulates a world-space torque about the center of mass, in N·m.
    pub fn apply_torque(&mut self, torque: Vec3) -> Result<(), BodyError> {
        self.ensure_dynamic()?;
        if !torque.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }

        let next_torque = add_checked(self.forces.torque, torque)?;
        self.forces.torque = next_torque;
        if torque != Vec3::ZERO {
            self.state.sleeping = false;
        }
        Ok(())
    }

    /// Clears accumulated forces and torques without changing motion or sleep state.
    pub fn clear_forces(&mut self) {
        self.forces.force = Vec3::ZERO;
        self.forces.torque = Vec3::ZERO;
    }

    pub(crate) fn commit_integrated_state(&mut self, state: BodyState) {
        self.state = state;
        self.clear_forces();
    }

    pub(crate) fn set_state_preserving_forces(&mut self, state: BodyState) {
        self.state = state;
    }

    fn ensure_dynamic(&self) -> Result<(), BodyError> {
        if self.physics.body_type == BodyType::Dynamic {
            Ok(())
        } else {
            Err(BodyError::BodyMustBeDynamic)
        }
    }

    /// Applies a world-space impulse immediately to the center of mass, in kg·m/s.
    pub fn apply_impulse(&mut self, impulse: Vec3) -> Result<(), BodyError> {
        self.ensure_dynamic()?;
        if !impulse.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }
        if impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let delta_velocity = impulse / mass.mass();
        let next_velocity = add_checked(self.state.linear_velocity, delta_velocity)?;

        self.state.linear_velocity = next_velocity;
        self.state.sleeping = false;
        Ok(())
    }

    /// Applies a world-space impulse in kg·m/s at a world-space point in meters.
    pub fn apply_impulse_at_point(&mut self, impulse: Vec3, point: Vec3) -> Result<(), BodyError> {
        self.ensure_dynamic()?;
        if !impulse.is_finite() || !point.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }
        if impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let center = world_center_of_mass_for(self.state.transform, Some(mass))?
            .ok_or(BodyError::BodyMustBeDynamic)?;
        let angular_impulse = point
            .checked_sub(center)
            .and_then(|offset| offset.checked_cross(impulse))
            .ok_or(BodyError::ArithmeticOverflow)?;
        self.apply_constraint_impulse(impulse, angular_impulse)
    }

    pub(crate) fn apply_constraint_impulse(
        &mut self,
        linear_impulse: Vec3,
        angular_impulse: Vec3,
    ) -> Result<(), BodyError> {
        self.ensure_dynamic()?;
        if !linear_impulse.is_finite() || !angular_impulse.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }
        if linear_impulse == Vec3::ZERO && angular_impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let rotation = self.state.transform.rotation();
        let local_angular_impulse = rotation
            .conjugate()
            .rotate_vector(angular_impulse)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let local_delta_angular_velocity = solve_inertia(mass.inertia(), local_angular_impulse)?;
        let delta_angular_velocity = rotation
            .rotate_vector(local_delta_angular_velocity)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let delta_linear_velocity = linear_impulse / mass.mass();
        let next_linear_velocity = add_checked(self.state.linear_velocity, delta_linear_velocity)?;
        let next_angular_velocity =
            add_checked(self.state.angular_velocity, delta_angular_velocity)?;

        self.state.linear_velocity = next_linear_velocity;
        self.state.angular_velocity = next_angular_velocity;
        self.state.sleeping = false;
        Ok(())
    }

    pub(crate) fn apply_position_impulse(
        &mut self,
        impulse: Vec3,
        point: Vec3,
    ) -> Result<(), BodyError> {
        if self.physics.body_type != BodyType::Dynamic {
            return Ok(());
        }
        if !impulse.is_finite() || !point.is_finite() {
            return Err(BodyError::NonFiniteValue);
        }
        if impulse == Vec3::ZERO {
            return Ok(());
        }

        let mass = self.physics.mass.ok_or(BodyError::BodyMustBeDynamic)?;
        let center = world_center_of_mass_for(self.state.transform, Some(mass))?
            .ok_or(BodyError::BodyMustBeDynamic)?;
        let angular_impulse = point
            .checked_sub(center)
            .and_then(|offset| offset.checked_cross(impulse))
            .ok_or(BodyError::ArithmeticOverflow)?;
        self.apply_constraint_position_impulse(impulse, angular_impulse)
    }

    pub(crate) fn apply_constraint_position_impulse(
        &mut self,
        linear_impulse: Vec3,
        angular_impulse: Vec3,
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
        let center = world_center_of_mass_for(self.state.transform, Some(mass))?
            .ok_or(BodyError::BodyMustBeDynamic)?;
        let rotation = self.state.transform.rotation();
        let local_angular_impulse = rotation
            .conjugate()
            .rotate_vector(angular_impulse)
            .ok_or(BodyError::ArithmeticOverflow)?;
        let local_rotation_delta = solve_inertia(mass.inertia(), local_angular_impulse)?;
        let rotation_delta = rotation
            .rotate_vector(local_rotation_delta)
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
        let next_center = center
            .checked_add(linear_impulse / mass.mass())
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
        self.set_transform(next_transform)
    }
}

fn world_center_of_mass_for(
    transform: Transform,
    mass: Option<MassProperties>,
) -> Result<Option<Vec3>, BodyError> {
    let Some(mass) = mass else {
        return Ok(None);
    };

    let center = transform
        .rotation()
        .rotate_vector(mass.center_of_mass())
        .and_then(|center| center.checked_add(transform.translation()))
        .ok_or(BodyError::ArithmeticOverflow)?;
    Ok(Some(center))
}

fn solve_inertia(inertia: kapitsa_math::Mat3, rhs: Vec3) -> Result<Vec3, BodyError> {
    match inertia.solve_spd(rhs) {
        Ok(solution) => Ok(solution),
        Err(CholeskyError::NotSymmetric | CholeskyError::NotPositiveDefinite) => {
            Err(BodyError::InvalidInertiaTensor)
        }
        Err(CholeskyError::NonFiniteInput) => Err(BodyError::NonFiniteValue),
        Err(CholeskyError::ResultOutOfRange) => Err(BodyError::ArithmeticOverflow),
    }
}

fn add_checked(current: Vec3, delta: Vec3) -> Result<Vec3, BodyError> {
    current
        .checked_add(delta)
        .ok_or(BodyError::ArithmeticOverflow)
}

#[cfg(test)]
#[path = "rigid_body_tests.rs"]
mod tests;
