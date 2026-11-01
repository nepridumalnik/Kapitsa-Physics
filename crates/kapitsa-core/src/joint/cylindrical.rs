use super::{JointCommon, JointDrive, JointLimit};

/// Allows independent translation and rotation along/around the connector Z axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylindricalJoint {
    pub common: JointCommon,
    pub translation_limit: Option<JointLimit>,
    pub angle_limit: Option<JointLimit>,
    pub translation_drive: JointDrive,
    pub angular_drive: JointDrive,
}

impl CylindricalJoint {
    pub const fn new(common: JointCommon) -> Self {
        Self {
            common,
            translation_limit: None,
            angle_limit: None,
            translation_drive: JointDrive::DISABLED,
            angular_drive: JointDrive::DISABLED,
        }
    }
}
