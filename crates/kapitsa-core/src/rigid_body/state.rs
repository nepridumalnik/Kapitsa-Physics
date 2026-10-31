use kapitsa_math::Vec3;

use crate::transform::Transform;

/// Mutable kinematic state of a rigid body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyState {
    pub(super) transform: Transform,
    pub(super) linear_velocity: Vec3,
    pub(super) angular_velocity: Vec3,
    pub(super) sleeping: bool,
}

impl BodyState {
    /// Returns the body's world transform, which locates its local origin.
    pub fn transform(&self) -> Transform {
        self.transform
    }

    /// Returns the world-space linear velocity of the center of mass, in m/s.
    pub fn linear_velocity(&self) -> Vec3 {
        self.linear_velocity
    }

    /// Returns the world-space angular velocity, in rad/s.
    pub fn angular_velocity(&self) -> Vec3 {
        self.angular_velocity
    }

    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }

    pub(crate) fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
    }
}
