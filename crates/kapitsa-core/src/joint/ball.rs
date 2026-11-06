use super::{JointCommon, JointLimit};

/// Keeps connector anchors together while allowing limited or free rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BallJoint {
    pub common: JointCommon,
    /// Maximum swing angle from the connector axis, in radians.
    pub swing_limit: Option<f64>,
    /// Twist interval around the connector's local Z axis, in radians.
    pub twist_limit: Option<JointLimit>,
}

impl BallJoint {
    pub const fn new(common: JointCommon) -> Self {
        Self {
            common,
            swing_limit: None,
            twist_limit: None,
        }
    }
}
