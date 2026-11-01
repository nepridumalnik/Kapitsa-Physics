use super::{JointCommon, JointLimit};

/// Constrains the distance between connector anchors to an interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistanceJoint {
    pub common: JointCommon,
    pub distance_limit: JointLimit,
}

impl DistanceJoint {
    pub const fn new(common: JointCommon, distance_limit: JointLimit) -> Self {
        Self {
            common,
            distance_limit,
        }
    }

    pub const fn fixed_length(common: JointCommon, length: f64) -> Self {
        Self::new(common, JointLimit::new(length, length))
    }
}
