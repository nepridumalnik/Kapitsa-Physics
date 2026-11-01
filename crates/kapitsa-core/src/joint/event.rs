use crate::world::JointId;

/// A joint removed after its measured solver load exceeded a break threshold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointBreakEvent {
    joint: JointId,
    force: f64,
    torque: f64,
}

impl JointBreakEvent {
    pub(crate) const fn new(joint: JointId, force: f64, torque: f64) -> Self {
        Self {
            joint,
            force,
            torque,
        }
    }

    /// Returns the generational ID of the removed joint.
    pub fn joint(&self) -> JointId {
        self.joint
    }

    /// Returns the measured reaction force in newtons.
    pub fn force(&self) -> f64 {
        self.force
    }

    /// Returns the measured reaction torque in newton-meters.
    pub fn torque(&self) -> f64 {
        self.torque
    }
}
