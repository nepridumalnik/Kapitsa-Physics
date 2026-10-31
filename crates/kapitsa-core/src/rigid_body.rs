pub mod error;
mod forces;
mod physics;
mod state;

pub use error::BodyError;
pub use forces::BodyForces;
pub use physics::{BodyPhysics, BodyType};
pub use state::BodyState;

use glam::Vec3;

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
}

#[cfg(test)]
#[path = "rigid_body_tests.rs"]
mod tests;
