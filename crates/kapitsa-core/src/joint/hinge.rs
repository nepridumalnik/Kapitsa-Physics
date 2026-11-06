use super::{JointCommon, JointDrive, JointLimit};

/// Allows rotation around the connector frames' local Z axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HingeJoint {
    pub common: JointCommon,
    pub angle_limit: Option<JointLimit>,
    pub drive: JointDrive,
}

impl HingeJoint {
    pub const fn new(common: JointCommon) -> Self {
        Self {
            common,
            angle_limit: None,
            drive: JointDrive::DISABLED,
        }
    }
}
