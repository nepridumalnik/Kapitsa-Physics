use super::{JointCommon, JointLimit};

/// Applies a compliant distance constraint with optional hard distance bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpringJoint {
    pub common: JointCommon,
    pub rest_length: f64,
    /// Hooke stiffness in newtons per meter.
    pub stiffness: f64,
    /// Viscous damping in newton-seconds per meter.
    pub spring_damping: f64,
    pub distance_limit: Option<JointLimit>,
}

impl SpringJoint {
    pub const fn new(
        common: JointCommon,
        rest_length: f64,
        stiffness: f64,
        spring_damping: f64,
    ) -> Self {
        Self {
            common,
            rest_length,
            stiffness,
            spring_damping,
            distance_limit: None,
        }
    }
}
