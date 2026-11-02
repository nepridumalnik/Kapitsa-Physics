mod ball;
mod common;
mod cone_twist;
mod cylindrical;
mod d6;
#[cfg(feature = "bench-internals")]
pub mod diagnostics;
mod distance;
mod error;
mod event;
mod fixed;
mod hinge;
mod planar;
mod row;
mod slider;
#[allow(dead_code)] // Integrated into collision islands in the dedicated solver integration step.
mod solver;
mod spring;
#[cfg(test)]
mod stability_tests;
#[allow(dead_code)] // Used by the shared solver before island integration is wired in.
mod workspace;

pub use ball::BallJoint;
pub use common::{JointCommon, JointDrive, JointFrame, JointLimit, JointMotion};
pub use cone_twist::ConeTwistJoint;
pub use cylindrical::CylindricalJoint;
pub use d6::D6Joint;
pub use distance::DistanceJoint;
pub use error::JointError;
pub use event::JointBreakEvent;
pub use fixed::FixedJoint;
pub use hinge::HingeJoint;
pub use planar::PlanarJoint;
pub use slider::SliderJoint;
pub(crate) use solver::{JointConstraint, JointSolver};
pub use spring::SpringJoint;
pub(crate) use workspace::JointWorkspace;

/// Statically dispatched constraint kinds supported by Kapitsa.
// Inline Copy settings avoid per-joint heap allocations in the solver path.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Joint {
    Fixed(FixedJoint),
    Hinge(HingeJoint),
    Slider(SliderJoint),
    Ball(BallJoint),
    Cylindrical(CylindricalJoint),
    Planar(PlanarJoint),
    Distance(DistanceJoint),
    Spring(SpringJoint),
    ConeTwist(ConeTwistJoint),
    D6(D6Joint),
}

impl Joint {
    pub fn common(&self) -> &JointCommon {
        match self {
            Self::Fixed(joint) => &joint.common,
            Self::Hinge(joint) => &joint.common,
            Self::Slider(joint) => &joint.common,
            Self::Ball(joint) => &joint.common,
            Self::Cylindrical(joint) => &joint.common,
            Self::Planar(joint) => &joint.common,
            Self::Distance(joint) => &joint.common,
            Self::Spring(joint) => &joint.common,
            Self::ConeTwist(joint) => &joint.common,
            Self::D6(joint) => &joint.common,
        }
    }

    /// Validates all common and type-specific settings before storage or solving.
    pub fn validate(&self) -> Result<(), JointError> {
        self.common().validate()?;
        match self {
            Self::Fixed(_) => {}
            Self::Hinge(joint) => {
                validate_limit(joint.angle_limit)?;
                joint.drive.validate()?;
            }
            Self::Slider(joint) => {
                validate_limit(joint.translation_limit)?;
                joint.drive.validate()?;
            }
            Self::Ball(joint) => {
                if joint.swing_limit.is_some_and(|angle| {
                    !angle.is_finite() || !(0.0..=std::f64::consts::PI).contains(&angle)
                }) {
                    return Err(JointError::InvalidLimit);
                }
                validate_limit(joint.twist_limit)?;
            }
            Self::Cylindrical(joint) => {
                validate_limit(joint.translation_limit)?;
                validate_limit(joint.angle_limit)?;
                joint.translation_drive.validate()?;
                joint.angular_drive.validate()?;
            }
            Self::Planar(joint) => {
                for limit in joint.translation_limits {
                    validate_limit(limit)?;
                }
                validate_limit(joint.rotation_limit)?;
                for drive in joint.translation_drives {
                    drive.validate()?;
                }
                joint.rotation_drive.validate()?;
            }
            Self::Distance(joint) => joint.distance_limit.validate()?,
            Self::Spring(joint) => {
                if !joint.rest_length.is_finite() || joint.rest_length < 0.0 {
                    return Err(JointError::InvalidLimit);
                }
                if !joint.stiffness.is_finite() || joint.stiffness < 0.0 {
                    return Err(JointError::InvalidStiffness);
                }
                if !joint.spring_damping.is_finite() || joint.spring_damping < 0.0 {
                    return Err(JointError::InvalidDamping);
                }
                validate_limit(joint.distance_limit)?;
            }
            Self::ConeTwist(joint) => {
                if joint.swing_limit.iter().any(|angle| {
                    !angle.is_finite() || !(0.0..=std::f64::consts::PI).contains(angle)
                }) {
                    return Err(JointError::InvalidLimit);
                }
                joint.twist_limit.validate()?;
            }
            Self::D6(joint) => {
                for motion in joint.linear_motion.into_iter().chain(joint.angular_motion) {
                    motion.validate()?;
                }
                for drive in joint.linear_drives.into_iter().chain(joint.angular_drives) {
                    drive.validate()?;
                }
            }
        }
        Ok(())
    }

    pub fn set_enabled(&mut self, enabled: bool) {
        match self {
            Self::Fixed(joint) => joint.common.enabled = enabled,
            Self::Hinge(joint) => joint.common.enabled = enabled,
            Self::Slider(joint) => joint.common.enabled = enabled,
            Self::Ball(joint) => joint.common.enabled = enabled,
            Self::Cylindrical(joint) => joint.common.enabled = enabled,
            Self::Planar(joint) => joint.common.enabled = enabled,
            Self::Distance(joint) => joint.common.enabled = enabled,
            Self::Spring(joint) => joint.common.enabled = enabled,
            Self::ConeTwist(joint) => joint.common.enabled = enabled,
            Self::D6(joint) => joint.common.enabled = enabled,
        }
    }
}

fn validate_limit(limit: Option<JointLimit>) -> Result<(), JointError> {
    if let Some(limit) = limit {
        limit.validate()?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_math::{Quat, Vec3};

    fn common() -> JointCommon {
        JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY)
    }

    #[test]
    fn every_joint_kind_has_a_valid_default_configuration() {
        let common = common();
        let joints = [
            Joint::Fixed(FixedJoint::new(common)),
            Joint::Hinge(HingeJoint::new(common)),
            Joint::Slider(SliderJoint::new(common)),
            Joint::Ball(BallJoint::new(common)),
            Joint::Cylindrical(CylindricalJoint::new(common)),
            Joint::Planar(PlanarJoint::new(common)),
            Joint::Distance(DistanceJoint::fixed_length(common, 1.0)),
            Joint::Spring(SpringJoint::new(common, 1.0, 20.0, 0.5)),
            Joint::ConeTwist(ConeTwistJoint::new(
                common,
                [0.5, 0.75],
                JointLimit::new(-1.0, 1.0),
            )),
            Joint::D6(D6Joint::new(common)),
        ];
        assert!(joints.iter().all(|joint| joint.validate().is_ok()));
    }

    #[test]
    fn joint_validation_rejects_bad_frames_limits_and_drives() {
        let invalid_frame = JointFrame::new(Vec3::new(f64::NAN, 0.0, 0.0), Quat::IDENTITY);
        assert_eq!(invalid_frame.validate(), Err(JointError::InvalidFrame));

        let common = common();
        let mut hinge = HingeJoint::new(common);
        hinge.angle_limit = Some(JointLimit::new(1.0, -1.0));
        assert_eq!(
            Joint::Hinge(hinge).validate(),
            Err(JointError::InvalidLimit)
        );

        let mut slider = SliderJoint::new(common);
        slider.drive.enabled = true;
        slider.drive.max_force = f64::INFINITY;
        assert_eq!(
            Joint::Slider(slider).validate(),
            Err(JointError::InvalidDrive)
        );
    }
}
