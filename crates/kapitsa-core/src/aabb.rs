use kapitsa_math::{Mat3, Vec3, numeric};

use crate::transform::Transform;

pub mod error;
pub use error::AabbError;

/// Axis-aligned bounding box defined by minimum and maximum corners.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    min: Vec3,
    max: Vec3,
}

impl Aabb {
    pub fn new(min: Vec3, max: Vec3) -> Result<Self, AabbError> {
        if !min.is_finite() || !max.is_finite() {
            return Err(AabbError::NonFiniteBounds);
        }
        if min.x > max.x || min.y > max.y || min.z > max.z {
            return Err(AabbError::InvertedBounds);
        }

        Ok(Self { min, max })
    }

    /// Converts legacy shape bounds while those shapes are migrated to `f64`.
    pub(crate) fn from_f32_bounds(min: glam::Vec3, max: glam::Vec3) -> Result<Self, AabbError> {
        Self::new(to_f64_vec3(min), to_f64_vec3(max))
    }

    pub fn min(&self) -> Vec3 {
        self.min
    }

    pub fn max(&self) -> Vec3 {
        self.max
    }

    pub fn center(&self) -> Vec3 {
        self.min * 0.5 + self.max * 0.5
    }

    pub fn size(&self) -> Result<Vec3, AabbError> {
        let size = self.max - self.min;
        if size.is_finite() {
            Ok(size)
        } else {
            Err(AabbError::SizeOverflow)
        }
    }

    pub fn contains(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.y >= self.min.y
            && point.z >= self.min.z
            && point.x <= self.max.x
            && point.y <= self.max.y
            && point.z <= self.max.z
    }

    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.min.y <= other.max.y
            && self.min.z <= other.max.z
            && self.max.x >= other.min.x
            && self.max.y >= other.min.y
            && self.max.z >= other.min.z
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Transforms these local-space bounds by a rigid transform and rounds
    /// the result outward to keep it conservative.
    pub fn transformed(&self, transform: &Transform) -> Result<Self, AabbError> {
        let translation = transform.translation();
        let rotation = transform.rotation();

        if is_identity_rotation(rotation) {
            let min = translated(self.min, translation, false)?;
            let max = translated(self.max, translation, true)?;
            return Self::new(min, max).map_err(|_| AabbError::TransformOverflow);
        }

        if self.min == Vec3::ZERO && self.max == Vec3::ZERO {
            return Self::new(translation, translation).map_err(|_| AabbError::TransformOverflow);
        }

        let center = self.center();
        let half_extents = self.max * 0.5 - self.min * 0.5;
        if !center.is_finite() || !half_extents.is_finite() {
            return Err(AabbError::TransformOverflow);
        }

        let rotation = rotation.to_mat3().ok_or(AabbError::TransformOverflow)?;
        let absolute_rotation: Mat3 = rotation.abs();
        let transformed_center = rotation
            .checked_mul_vector(center)
            .and_then(|value| value.checked_add(translation))
            .ok_or(AabbError::TransformOverflow)?;
        let transformed_half_extents = absolute_rotation
            .checked_mul_vector(half_extents)
            .ok_or(AabbError::TransformOverflow)?;

        let scale = center
            .abs()
            .max_element()
            .max(half_extents.abs().max_element())
            .max(translation.abs().max_element());
        let error = numeric::roundoff_bound(scale, 64).ok_or(AabbError::TransformOverflow)?
            + 64.0 * f64::from_bits(1);
        if !error.is_finite() {
            return Err(AabbError::TransformOverflow);
        }

        let error = Vec3::splat(error);
        let min = transformed_center
            .checked_sub(transformed_half_extents)
            .and_then(|value| value.checked_sub(error))
            .ok_or(AabbError::TransformOverflow)?;
        let max = transformed_center
            .checked_add(transformed_half_extents)
            .and_then(|value| value.checked_add(error))
            .ok_or(AabbError::TransformOverflow)?;
        let min = round_down_vec3(min)?;
        let max = round_up_vec3(max)?;

        Self::new(min, max).map_err(|_| AabbError::TransformOverflow)
    }
}

fn is_identity_rotation(rotation: kapitsa_math::Quat) -> bool {
    rotation.x == 0.0 && rotation.y == 0.0 && rotation.z == 0.0 && rotation.w.abs() == 1.0
}

fn translated(value: Vec3, offset: Vec3, round_up: bool) -> Result<Vec3, AabbError> {
    Ok(Vec3::new(
        translated_component(value.x, offset.x, round_up)?,
        translated_component(value.y, offset.y, round_up)?,
        translated_component(value.z, offset.z, round_up)?,
    ))
}

fn translated_component(value: f64, offset: f64, round_up: bool) -> Result<f64, AabbError> {
    let sum = value + offset;
    if !sum.is_finite() {
        return Err(AabbError::TransformOverflow);
    }

    let virtual_offset = sum - value;
    let error = (value - (sum - virtual_offset)) + (offset - virtual_offset);
    if round_up && error > 0.0 {
        numeric::round_up(sum).ok_or(AabbError::TransformOverflow)
    } else if !round_up && error < 0.0 {
        numeric::round_down(sum).ok_or(AabbError::TransformOverflow)
    } else {
        Ok(sum)
    }
}

fn round_down_vec3(value: Vec3) -> Result<Vec3, AabbError> {
    Ok(Vec3::new(
        numeric::round_down(value.x).ok_or(AabbError::TransformOverflow)?,
        numeric::round_down(value.y).ok_or(AabbError::TransformOverflow)?,
        numeric::round_down(value.z).ok_or(AabbError::TransformOverflow)?,
    ))
}

fn round_up_vec3(value: Vec3) -> Result<Vec3, AabbError> {
    Ok(Vec3::new(
        numeric::round_up(value.x).ok_or(AabbError::TransformOverflow)?,
        numeric::round_up(value.y).ok_or(AabbError::TransformOverflow)?,
        numeric::round_up(value.z).ok_or(AabbError::TransformOverflow)?,
    ))
}

fn to_f64_vec3(value: glam::Vec3) -> Vec3 {
    Vec3::new(f64::from(value.x), f64::from(value.y), f64::from(value.z))
}

#[cfg(test)]
#[path = "aabb_tests.rs"]
mod tests;
