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
    };
}

pub(crate) fn write_equality_rows(
    joint: JointConstraint,
    bodies: &[MomentumBody],
    out: &mut [EqualityRow; MAX_EQUALITY_ROWS],
) -> Result<usize, CollisionError> {
    if !matches!(joint.joint, Joint::Ball(_)) {
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
    let frame = a.rotation * common.frame_a.local_rotation;
    for (row, axis) in out.iter_mut().zip([Vec3::X, Vec3::Y, Vec3::Z]) {
        let axis = rotate(frame, axis)?;
        *row = EqualityRow {
            a: joint.body_a,
            b: joint.body_b,
            linear_a: -axis,
            linear_b: axis,
            angular_a: finite(axis.cross(delta) - (pa - a.center).cross(axis))?,
            angular_b: finite((pb - b.center).cross(axis))?,
            value: delta.dot(axis),
        };
    }
    Ok(3)
}
