use super::{JointCommon, JointDrive, JointMotion};

/// Independently configures all six relative degrees of freedom.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct D6Joint {
    pub common: JointCommon,
    /// Local X/Y/Z linear axes.
    pub linear_motion: [JointMotion; 3],
    /// Local X/Y/Z angular axes, represented as rotation-vector components.
    pub angular_motion: [JointMotion; 3],
    pub linear_drives: [JointDrive; 3],
    pub angular_drives: [JointDrive; 3],
}

impl D6Joint {
    pub const fn new(common: JointCommon) -> Self {
        Self {
            common,
            linear_motion: [JointMotion::Locked; 3],
            angular_motion: [JointMotion::Locked; 3],
            linear_drives: [JointDrive::DISABLED; 3],
            angular_drives: [JointDrive::DISABLED; 3],
        }
    }
}
