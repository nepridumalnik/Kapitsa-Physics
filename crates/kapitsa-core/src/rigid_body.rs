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
    /// A changed mass or center of mass wakes the body and clears accumulated
    /// forces and torques, whose torque reference may otherwise be stale.
    pub fn set_mass(&mut self, mass: MassProperties) -> Result<(), BodyError> {
        if self.physics.body_type != BodyType::Dynamic {
            return Err(BodyError::BodyMustBeDynamic);
        }
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
}

fn world_center_of_mass_for(
    transform: Transform,
    mass: Option<MassProperties>,
) -> Result<Option<Vec3>, BodyError> {
    let Some(mass) = mass else {
        return Ok(None);
    };

    let rotation = transform.rotation();
    let rotation = DQuat::from_xyzw(
        f64::from(rotation.x),
        f64::from(rotation.y),
        f64::from(rotation.z),
        f64::from(rotation.w),
    )
    .normalize();
    let center = rotation.mul_vec3(DVec3::new(
        f64::from(mass.center_of_mass().x),
        f64::from(mass.center_of_mass().y),
        f64::from(mass.center_of_mass().z),
    )) + DVec3::new(
        f64::from(transform.translation().x),
        f64::from(transform.translation().y),
        f64::from(transform.translation().z),
    );
    if !center.is_finite()
        || center.x.abs() > f64::from(f32::MAX)
        || center.y.abs() > f64::from(f32::MAX)
        || center.z.abs() > f64::from(f32::MAX)
    {
        return Err(BodyError::ArithmeticOverflow);
    }

    Ok(Some(Vec3::new(
        center.x as f32,
        center.y as f32,
        center.z as f32,
    )))
}

#[cfg(test)]
#[path = "rigid_body_tests.rs"]
mod tests;
