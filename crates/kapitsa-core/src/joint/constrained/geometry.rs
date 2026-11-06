use super::{MomentumBody, finite, rotate};
use crate::{
    collision::CollisionError,
    joint::{Joint, JointConstraint},
};
use kapitsa_math::Vec3;

pub(crate) const MAX_EQUALITY_ROWS: usize = 6;

#[derive(Clone, Copy, Debug)]
pub(crate) struct EqualityRow {
    pub(crate) a: usize,
    pub(crate) b: usize,
    pub(crate) linear_a: Vec3,
    pub(crate) angular_a: Vec3,
    pub(crate) linear_b: Vec3,
    pub(crate) angular_b: Vec3,
    pub(crate) value: f64,
    pub(crate) tolerance: f64,
}
impl EqualityRow {
    pub(crate) const ZERO: Self = Self {
        a: 0,
        b: 0,
        linear_a: Vec3::ZERO,
        angular_a: Vec3::ZERO,
        linear_b: Vec3::ZERO,
        angular_b: Vec3::ZERO,
        value: 0.0,
        tolerance: 1.0e-13,
    };
}

pub(crate) fn write_equality_rows(
    joint: JointConstraint,
    bodies: &[MomentumBody],
    out: &mut [EqualityRow; MAX_EQUALITY_ROWS],
) -> Result<usize, CollisionError> {
    use crate::joint::{
        JointMotion,
        solver::{fixed_rotation_coefficient, fixed_rotation_row_axis, relative_rotation_error},
    };
    if !joint.joint.supports_free_bilateral() || joint.body_a == joint.body_b {
        return Err(CollisionError::InvalidContact);
    }
    let a = *bodies
        .get(joint.body_a)
        .ok_or(CollisionError::InvalidContact)?;
    let b = *bodies
        .get(joint.body_b)
        .ok_or(CollisionError::InvalidContact)?;
    let common = joint.joint.common();
    let pa = a.anchor(common.frame_a.local_anchor)?;
    let pb = b.anchor(common.frame_b.local_anchor)?;
    let delta = finite(pb - pa)?;
    let frame_a = a.rotation * common.frame_a.local_rotation;
    let frame_b = b.rotation * common.frame_b.local_rotation;
    let mut linear = [true; 3];
    let mut angular = [false; 3];
    let mut align = false;
    let mut target = frame_a;
    match joint.joint {
        Joint::Ball(_) => {}
        Joint::Fixed(_) => angular = [true; 3],
        Joint::Hinge(_) => align = true,
        Joint::Slider(_) => {
            linear[2] = false;
            angular = [true; 3];
        }
        Joint::Cylindrical(_) => {
            linear[2] = false;
            align = true;
        }
        Joint::Planar(_) => {
            linear = [false, false, true];
            align = true;
        }
        Joint::D6(joint) => {
            linear = joint
                .linear_motion
                .map(|motion| matches!(motion, JointMotion::Locked));
            angular = joint
                .angular_motion
                .map(|motion| matches!(motion, JointMotion::Locked));
        }
        Joint::Distance(distance_joint) => {
            let distance = super::length(delta)?;
            let axis = delta.try_normalize().ok_or(CollisionError::NonConvergent)?;
            out[0] = EqualityRow {
                a: joint.body_a,
                b: joint.body_b,
                ..EqualityRow::ZERO
            };
            // Filled below using the common scalar point-distance differential.
            out[0].linear_a = -axis;
            out[0].linear_b = axis;
            out[0].angular_a = -(pa - a.center).cross(axis);
            out[0].angular_b = (pb - b.center).cross(axis);
            out[0].value = distance - distance_joint.distance_limit.lower;
            return Ok(1);
        }
        Joint::ConeTwist(joint) => {
            if joint.swing_limit == [0.0, 0.0] {
                if joint.twist_limit.lower == joint.twist_limit.upper {
                    angular = [true; 3];
                    target = frame_a
                        * kapitsa_math::Quat::from_axis_angle(Vec3::Z, joint.twist_limit.lower)
                            .ok_or(CollisionError::ArithmeticOverflow)?;
                } else {
                    align = true;
                }
            }
        }
        _ => return Err(CollisionError::InvalidContact),
    }
    let mut count = 0;
    for (locked, local_axis) in linear.into_iter().zip([Vec3::X, Vec3::Y, Vec3::Z]) {
        if !locked {
            continue;
        }
        let axis = rotate(frame_a, local_axis)?;
        out[count] = EqualityRow {
            a: joint.body_a,
            b: joint.body_b,
            linear_a: -axis,
            linear_b: axis,
            angular_a: finite(axis.cross(delta) - (pa - a.center).cross(axis))?,
            angular_b: finite((pb - b.center).cross(axis))?,
            value: delta.dot(axis),
            tolerance: (32.0 * f64::EPSILON * (super::length(pa)? + super::length(pb)?))
                .max(1.0e-13),
        };
        count += 1;
    }
    if align {
        let z = rotate(frame_b, Vec3::Z)?;
        if rotate(frame_a, Vec3::Z)?.dot(z) <= 1.0e-10 {
            return Err(CollisionError::NonConvergent);
        }
        for axis in [Vec3::X, Vec3::Y] {
            let axis = rotate(frame_a, axis)?;
            let gradient = finite(axis.cross(z))?;
            out[count] = EqualityRow {
                a: joint.body_a,
                b: joint.body_b,
                angular_a: gradient,
                angular_b: -gradient,
                value: axis.dot(z),
                ..EqualityRow::ZERO
            };
            count += 1;
        }
    } else if angular.iter().any(|locked| *locked) {
        let error = relative_rotation_error(target, frame_b)?;
        if super::length(error)? >= std::f64::consts::PI - 1.0e-10 {
            return Err(CollisionError::NonConvergent);
        }
        let coefficient = fixed_rotation_coefficient(error)?;
        for ((locked, axis), value) in angular
            .into_iter()
            .zip([Vec3::X, Vec3::Y, Vec3::Z])
            .zip([error.x, error.y, error.z])
        {
            if !locked {
                continue;
            }
            let axis = fixed_rotation_row_axis(target, error, axis, coefficient)?;
            out[count] = EqualityRow {
                a: joint.body_a,
                b: joint.body_b,
                angular_a: -axis,
                angular_b: axis,
                value,
                ..EqualityRow::ZERO
            };
            count += 1;
        }
    }
    if out[..count].iter().any(|row| !row.value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(count)
}
