use super::{JointCommon, JointDrive, JointLimit};

/// Constrains normal translation and tilt while allowing motion in the connector plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarJoint {
    pub common: JointCommon,
    /// Optional bounds along local X and Y, in meters.
    pub translation_limits: [Option<JointLimit>; 2],
    pub rotation_limit: Option<JointLimit>,
    pub translation_drives: [JointDrive; 2],
    pub rotation_drive: JointDrive,
}

impl PlanarJoint {
    pub const fn new(common: JointCommon) -> Self {
        Self {
            common,
            translation_limits: [None, None],
            rotation_limit: None,
            translation_drives: [JointDrive::DISABLED; 2],
            rotation_drive: JointDrive::DISABLED,
        }
    }
}
