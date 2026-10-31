pub mod error;
mod forces;
mod physics;
mod state;

pub use error::BodyError;
pub use forces::BodyForces;
pub use physics::{BodyPhysics, BodyType};
pub use state::BodyState;

use glam::{DQuat, DVec3, Vec3};

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
    pub fn new_dynamic(transform: Transform, mass: MassProperties) -> Self {
        Self::new(BodyType::Dynamic, transform, Some(mass))
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

    /// Replaces the world transform while preserving velocities.
    ///
    /// A changed transform wakes a sleeping dynamic body. The update is rejected
    /// if its world-space center of mass cannot be represented as `f32`.
    pub fn set_transform(&mut self, transform: Transform) -> Result<(), BodyError> {
        world_center_of_mass_for(transform, self.physics.mass)?
            .map(checked_vec3)
            .transpose()?;

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
        world_center_of_mass_for(self.state.transform, Some(mass))?
            .map(checked_vec3)
            .transpose()?;
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
        world_center_of_mass_for(self.state.transform, self.physics.mass)?
            .map(checked_vec3)
            .transpose()
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

        let next_force = add_checked(self.forces.force, to_dvec3(force))?;
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
        let torque_delta = (to_dvec3(point) - center).cross(to_dvec3(force));
        let next_force = add_checked(self.forces.force, to_dvec3(force))?;
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

        let next_torque = add_checked(self.forces.torque, to_dvec3(torque))?;
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
        let delta_velocity = to_dvec3(impulse) / f64::from(mass.mass());
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
        let angular_impulse = (to_dvec3(point) - center).cross(to_dvec3(impulse));
        let rotation = rotation_quat(self.state.transform);
        let local_angular_impulse = rotation.conjugate().mul_vec3(angular_impulse);
        let local_delta_angular_velocity = solve_inertia(mass.inertia(), local_angular_impulse)?;
        let delta_angular_velocity = rotation.mul_vec3(local_delta_angular_velocity);
        let delta_linear_velocity = to_dvec3(impulse) / f64::from(mass.mass());
        let next_linear_velocity = add_checked(self.state.linear_velocity, delta_linear_velocity)?;
        let next_angular_velocity =
            add_checked(self.state.angular_velocity, delta_angular_velocity)?;

        self.state.linear_velocity = next_linear_velocity;
        self.state.angular_velocity = next_angular_velocity;
        self.state.sleeping = false;
        Ok(())
    }
}

fn world_center_of_mass_for(
    transform: Transform,
    mass: Option<MassProperties>,
) -> Result<Option<DVec3>, BodyError> {
    let Some(mass) = mass else {
        return Ok(None);
    };

    let rotation = rotation_quat(transform);
    let center =
        rotation.mul_vec3(to_dvec3(mass.center_of_mass())) + to_dvec3(transform.translation());
    if !center.is_finite()
        || center.x.abs() > f64::from(f32::MAX)
        || center.y.abs() > f64::from(f32::MAX)
        || center.z.abs() > f64::from(f32::MAX)
    {
        return Err(BodyError::ArithmeticOverflow);
    }

    Ok(Some(center))
}

fn rotation_quat(transform: Transform) -> DQuat {
    let rotation = transform.rotation();
    DQuat::from_xyzw(
        f64::from(rotation.x),
        f64::from(rotation.y),
        f64::from(rotation.z),
        f64::from(rotation.w),
    )
    .normalize()
}

fn solve_inertia(inertia: glam::Mat3, right_hand_side: DVec3) -> Result<DVec3, BodyError> {
    let columns = inertia.to_cols_array();
    let matrix = [
        [
            f64::from(columns[0]),
            f64::from(columns[3]),
            f64::from(columns[6]),
        ],
        [
            f64::from(columns[1]),
            f64::from(columns[4]),
            f64::from(columns[7]),
        ],
        [
            f64::from(columns[2]),
            f64::from(columns[5]),
            f64::from(columns[8]),
        ],
    ];
    let scale = matrix
        .iter()
        .flatten()
        .fold(0.0_f64, |scale, value| scale.max(value.abs()));
    if !scale.is_finite() || scale <= 0.0 || !right_hand_side.is_finite() {
        return Err(BodyError::InvalidInertiaTensor);
    }

    let normalized = matrix.map(|row| row.map(|value| value / scale));
    let first_pivot = normalized[0][0];
    if first_pivot <= 0.0 {
        return Err(BodyError::InvalidInertiaTensor);
    }
    let l00 = first_pivot.sqrt();
    let l10 = normalized[1][0] / l00;
    let l20 = normalized[2][0] / l00;
    let second_pivot = normalized[1][1] - l10 * l10;
    if !second_pivot.is_finite() || second_pivot <= 0.0 {
        return Err(BodyError::InvalidInertiaTensor);
    }
    let l11 = second_pivot.sqrt();
    let l21 = (normalized[2][1] - l20 * l10) / l11;
    let third_pivot = normalized[2][2] - l20 * l20 - l21 * l21;
    if !third_pivot.is_finite() || third_pivot <= 0.0 {
        return Err(BodyError::InvalidInertiaTensor);
    }
    let l22 = third_pivot.sqrt();

    let rhs = right_hand_side / scale;
    if !rhs.is_finite() {
        return Err(BodyError::ArithmeticOverflow);
    }
    let y0 = rhs.x / l00;
    let y1 = (rhs.y - l10 * y0) / l11;
    let y2 = (rhs.z - l20 * y0 - l21 * y1) / l22;
    let x2 = y2 / l22;
    let x1 = (y1 - l21 * x2) / l11;
    let x0 = (y0 - l10 * x1 - l20 * x2) / l00;
    let solution = DVec3::new(x0, x1, x2);
    if !solution.is_finite() {
        return Err(BodyError::ArithmeticOverflow);
    }
    Ok(solution)
}

fn to_dvec3(value: Vec3) -> DVec3 {
    DVec3::new(f64::from(value.x), f64::from(value.y), f64::from(value.z))
}

fn checked_vec3(value: DVec3) -> Result<Vec3, BodyError> {
    if !value.is_finite()
        || value.x.abs() > f64::from(f32::MAX)
        || value.y.abs() > f64::from(f32::MAX)
        || value.z.abs() > f64::from(f32::MAX)
    {
        return Err(BodyError::ArithmeticOverflow);
    }

    Ok(Vec3::new(value.x as f32, value.y as f32, value.z as f32))
}

fn add_checked(current: Vec3, delta: DVec3) -> Result<Vec3, BodyError> {
    checked_vec3(to_dvec3(current) + delta)
}

#[cfg(test)]
#[path = "rigid_body_tests.rs"]
mod tests;
