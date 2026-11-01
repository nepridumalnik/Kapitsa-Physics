use super::{JointCommon, JointLimit};

/// Keeps anchors together and constrains two swing angles and one twist angle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConeTwistJoint {
    pub common: JointCommon,
    /// Maximum swing angles around local X and Y, in radians.
    pub swing_limit: [f64; 2],
    pub twist_limit: JointLimit,
}

impl ConeTwistJoint {
    pub const fn new(common: JointCommon, swing_limit: [f64; 2], twist_limit: JointLimit) -> Self {
        Self {
            common,
            swing_limit,
            twist_limit,
        }
    }
}
