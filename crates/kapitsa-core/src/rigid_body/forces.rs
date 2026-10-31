use glam::Vec3;

/// Accumulated force and torque in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyForces {
    pub(super) force: Vec3,
    pub(super) torque: Vec3,
}

impl BodyForces {
    /// Returns the accumulated world-space force, in newtons.
    pub fn force(&self) -> Vec3 {
        self.force
    }

    /// Returns the accumulated world-space torque about the center of mass, in N·m.
    pub fn torque(&self) -> Vec3 {
        self.torque
    }
}
