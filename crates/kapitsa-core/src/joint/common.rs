use kapitsa_math::{Quat, Vec3};

use super::JointError;

/// A connector frame expressed in a body's local coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointFrame {
    pub local_anchor: Vec3,
    pub local_rotation: Quat,
}

impl JointFrame {
    pub const IDENTITY: Self = Self {
        local_anchor: Vec3::ZERO,
        local_rotation: Quat::IDENTITY,
    };

    pub const fn new(local_anchor: Vec3, local_rotation: Quat) -> Self {
        Self {
            local_anchor,
            local_rotation,
        }
    }

    pub fn validate(self) -> Result<(), JointError> {
        if !self.local_anchor.is_finite() || !self.local_rotation.is_normalized() {
            return Err(JointError::InvalidFrame);
        }
        Ok(())
    }
}

/// Inclusive lower and upper bounds in meters or radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimit {
    pub lower: f64,
    pub upper: f64,
}

impl JointLimit {
    pub const fn new(lower: f64, upper: f64) -> Self {
        Self { lower, upper }
    }

    pub fn validate(self) -> Result<(), JointError> {
        if !self.lower.is_finite() || !self.upper.is_finite() || self.lower > self.upper {
            return Err(JointError::InvalidLimit);
        }
        Ok(())
    }
}

/// A one-axis motor. `target_position` is meters or radians and velocity is
/// meters/second or radians/second, according to the driven degree of freedom.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointDrive {
    pub enabled: bool,
    pub target_position: f64,
    pub target_velocity: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub max_force: f64,
}

impl JointDrive {
    pub const DISABLED: Self = Self {
        enabled: false,
        target_position: 0.0,
        target_velocity: 0.0,
        stiffness: 0.0,
        damping: 0.0,
        max_force: 0.0,
    };

    pub fn validate(self) -> Result<(), JointError> {
        if !self.target_position.is_finite()
            || !self.target_velocity.is_finite()
            || !self.stiffness.is_finite()
            || self.stiffness < 0.0
            || !self.damping.is_finite()
            || self.damping < 0.0
            || !self.max_force.is_finite()
            || self.max_force < 0.0
        {
            return Err(JointError::InvalidDrive);
        }
        Ok(())
    }
}

/// Common connector and solver settings shared by every joint kind.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointCommon {
    pub frame_a: JointFrame,
    pub frame_b: JointFrame,
    /// XPBD compliance in inverse newtons (or inverse newton-meters).
    pub compliance: f64,
    /// Relative velocity damping coefficient.
    pub damping: f64,
    /// Optional force threshold in newtons.
    pub break_force: Option<f64>,
    /// Optional torque threshold in newton-meters.
    pub break_torque: Option<f64>,
    pub enabled: bool,
    pub collide_connected: bool,
}

impl JointCommon {
    pub const fn new(frame_a: JointFrame, frame_b: JointFrame) -> Self {
        Self {
            frame_a,
            frame_b,
            compliance: 0.0,
            damping: 0.0,
            break_force: None,
            break_torque: None,
            enabled: true,
            collide_connected: false,
        }
    }

    pub fn validate(self) -> Result<(), JointError> {
        self.frame_a.validate()?;
        self.frame_b.validate()?;
        if !self.compliance.is_finite() || self.compliance < 0.0 {
            return Err(JointError::InvalidStiffness);
        }
        if !self.damping.is_finite() || self.damping < 0.0 {
            return Err(JointError::InvalidDamping);
        }
        for threshold in [self.break_force, self.break_torque].into_iter().flatten() {
            if !threshold.is_finite() || threshold <= 0.0 {
                return Err(JointError::InvalidBreakThreshold);
            }
        }
        Ok(())
    }
}

/// Motion allowed along one degree of freedom in a D6 joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointMotion {
    Free,
    Locked,
    Limited(JointLimit),
}

impl JointMotion {
    pub(crate) fn validate(self) -> Result<(), JointError> {
        if let Self::Limited(limit) = self {
            limit.validate()?;
        }
        Ok(())
    }
}
