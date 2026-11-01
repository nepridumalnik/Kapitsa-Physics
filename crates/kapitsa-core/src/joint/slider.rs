use super::{JointCommon, JointDrive, JointLimit};

/// Allows translation along the connector frames' local Z axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SliderJoint {
    pub common: JointCommon,
    pub translation_limit: Option<JointLimit>,
    pub drive: JointDrive,
}

impl SliderJoint {
    pub const fn new(common: JointCommon) -> Self {
        Self {
            common,
            translation_limit: None,
            drive: JointDrive::DISABLED,
        }
    }
}
