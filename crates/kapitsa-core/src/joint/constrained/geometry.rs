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
    let target = frame_a;
    let mut cone = None;
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
            out[0].tolerance =
                (32.0 * f64::EPSILON * (super::length(pa)? + super::length(pb)?)).max(1.0e-13);
            return Ok(1);
        }
        Joint::ConeTwist(joint) => cone = Some(joint),
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
    if let Some(cone) = cone {
        let relative = (frame_a.conjugate() * frame_b)
            .try_normalize()
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let norm_squared = relative.w * relative.w + relative.z * relative.z;
        if norm_squared <= (64.0 * f64::EPSILON).powi(2)
            && (cone.swing_limit.contains(&0.0) || cone.twist_limit.lower == cone.twist_limit.upper)
        {
            return Err(CollisionError::NonConvergent);
        }
        let norm = norm_squared.sqrt();
        let vector = Vec3::new(relative.x, relative.y, relative.z);
        for (axis, limit) in cone.swing_limit.into_iter().enumerate() {
            if limit != 0.0 {
                continue;
            }
            // swing = relative * conjugate(normalize(relative.zw)).
            let (numerator, gradient, scalar) = if axis == 0 {
                (
                    relative.w * relative.x - relative.z * relative.y,
                    Vec3::new(relative.w, -relative.z, -relative.y),
                    relative.x,
                )
            } else {
                (
                    relative.w * relative.y + relative.z * relative.x,
                    Vec3::new(relative.z, relative.w, relative.x),
                    relative.y,
                )
            };
            let value = 2.0 * numerator / norm;
            let gradient = gradient * (2.0 / norm)
                - Vec3::Z * (2.0 * numerator * relative.z / (norm_squared * norm));
            let scalar = 2.0 * scalar / norm - 2.0 * numerator * relative.w / (norm_squared * norm);
            let local = (gradient * relative.w + vector.cross(gradient) - vector * scalar) * 0.5;
            let angular = rotate(frame_a, local)?;
            out[count] = EqualityRow {
                a: joint.body_a,
                b: joint.body_b,
                angular_a: -angular,
                angular_b: angular,
                value,
                ..EqualityRow::ZERO
            };
            count += 1;
        }
        if cone.twist_limit.lower == cone.twist_limit.upper {
            let angle = 2.0 * relative.z.atan2(relative.w);
            let value = (angle - cone.twist_limit.lower + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            if value.abs() >= std::f64::consts::PI - 1.0e-10 {
                return Err(CollisionError::NonConvergent);
            }
            let gradient = Vec3::Z * (2.0 * relative.w / norm_squared);
            let scalar = -2.0 * relative.z / norm_squared;
            let angular = rotate(
                frame_a,
                (gradient * relative.w + vector.cross(gradient) - vector * scalar) * 0.5,
            )?;
            out[count] = EqualityRow {
                a: joint.body_a,
                b: joint.body_b,
                angular_a: -angular,
                angular_b: angular,
                value,
                ..EqualityRow::ZERO
            };
            count += 1;
        }
    } else if align {
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
