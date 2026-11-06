use std::fmt;

/// Invalid joint configuration or constraint data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointError {
    InvalidFrame,
    InvalidLimit,
    InvalidDrive,
    InvalidStiffness,
    InvalidDamping,
    InvalidBreakThreshold,
    InvalidJoint,
}

impl fmt::Display for JointError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidFrame => {
                "joint frame must contain finite coordinates and a normalized rotation"
            }
            Self::InvalidLimit => "joint limits must be finite and ordered",
            Self::InvalidDrive => "joint drive settings must be finite and non-negative",
            Self::InvalidStiffness => "joint stiffness must be finite and non-negative",
            Self::InvalidDamping => "joint damping must be finite and non-negative",
            Self::InvalidBreakThreshold => "joint break thresholds must be positive and finite",
            Self::InvalidJoint => "joint configuration is invalid",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for JointError {}
