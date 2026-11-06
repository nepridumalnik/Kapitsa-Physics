use super::JointCommon;

/// Locks all relative translation and rotation between two connector frames.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixedJoint {
    pub common: JointCommon,
}

impl FixedJoint {
    pub const fn new(common: JointCommon) -> Self {
        Self { common }
    }
}
