use glam::{DMat3, DQuat, DVec3, Vec3};

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
        if !min.cmple(max).all() {
            return Err(AabbError::InvertedBounds);
        }

        Ok(Self { min, max })
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
        match self.max - self.min {
            size if size.is_finite() => Ok(size),
            _ => Err(AabbError::SizeOverflow),
        }
    }

    pub fn contains(&self, point: Vec3) -> bool {
        point.cmpge(self.min).all() && point.cmple(self.max).all()
    }

    pub fn intersects(&self, other: &Self) -> bool {
        self.min.cmple(other.max).all() && self.max.cmpge(other.min).all()
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Transforms these local-space bounds by a rigid transform.
    ///
    /// The result is rounded outward to account for `f32` transform and
    /// conversion rounding without widening exact translations.
    pub fn transformed(&self, transform: &Transform) -> Result<Self, AabbError> {
        let min = DVec3::new(
            f64::from(self.min.x),
            f64::from(self.min.y),
            f64::from(self.min.z),
        );
        let max = DVec3::new(
            f64::from(self.max.x),
            f64::from(self.max.y),
            f64::from(self.max.z),
        );
        let center = min * 0.5 + max * 0.5;
        let half_extents = max * 0.5 - min * 0.5;
        if !center.is_finite() || !half_extents.is_finite() {
            return Err(AabbError::TransformOverflow);
        }

        let quat = transform.rotation();
        let quat = DQuat::from_xyzw(
            f64::from(quat.x),
            f64::from(quat.y),
            f64::from(quat.z),
            f64::from(quat.w),
        );
        let vector_part = DVec3::new(quat.x, quat.y, quat.z);
        let vector_length_squared = vector_part.length_squared();
        let scalar_squared = quat.w * quat.w;
        let rotation = DMat3::from_cols(
            DVec3::new(
                scalar_squared + quat.x * quat.x - quat.y * quat.y - quat.z * quat.z,
                2.0 * (quat.x * quat.y + quat.w * quat.z),
                2.0 * (quat.x * quat.z - quat.w * quat.y),
            ),
            DVec3::new(
                2.0 * (quat.x * quat.y - quat.w * quat.z),
                scalar_squared - quat.x * quat.x + quat.y * quat.y - quat.z * quat.z,
                2.0 * (quat.y * quat.z + quat.w * quat.x),
            ),
            DVec3::new(
                2.0 * (quat.x * quat.z + quat.w * quat.y),
                2.0 * (quat.y * quat.z - quat.w * quat.x),
                scalar_squared - quat.x * quat.x - quat.y * quat.y + quat.z * quat.z,
            ),
        );
        let absolute_rotation = DMat3::from_cols(
            rotation.x_axis.abs(),
            rotation.y_axis.abs(),
            rotation.z_axis.abs(),
        );
        if !rotation.x_axis.is_finite()
            || !rotation.y_axis.is_finite()
            || !rotation.z_axis.is_finite()
        {
            return Err(AabbError::TransformOverflow);
        }

        let translation = transform.translation();
        let translation = DVec3::new(
            f64::from(translation.x),
            f64::from(translation.y),
            f64::from(translation.z),
        );
        let transformed_center = rotation * center + translation;
        let transformed_half_extents = absolute_rotation * half_extents;
        let mut transformed_min = transformed_center - transformed_half_extents;
        let mut transformed_max = transformed_center + transformed_half_extents;
        if !transformed_center.is_finite()
            || !transformed_half_extents.is_finite()
            || !transformed_min.is_finite()
            || !transformed_max.is_finite()
        {
            return Err(AabbError::TransformOverflow);
        }

        let source_magnitude = center.abs() + half_extents;
        if source_magnitude != DVec3::ZERO && (vector_part != DVec3::ZERO || quat.w.abs() != 1.0) {
            // Bound f32 rounding in the absolute quaternion terms, before cancellation.
            let absolute_vector_part = vector_part.abs();
            let dot_scale = source_magnitude.dot(absolute_vector_part);
            let term_one_scale = source_magnitude * (scalar_squared - vector_length_squared).abs();
            let term_two_scale = absolute_vector_part * (2.0 * dot_scale);
            let term_three_scale = DVec3::new(
                absolute_vector_part.y * source_magnitude.z
                    + absolute_vector_part.z * source_magnitude.y,
                absolute_vector_part.z * source_magnitude.x
                    + absolute_vector_part.x * source_magnitude.z,
                absolute_vector_part.x * source_magnitude.y
                    + absolute_vector_part.y * source_magnitude.x,
            ) * (2.0 * quat.w.abs());
            let error_scale = term_one_scale + term_two_scale + term_three_scale;
            let operation_count = 12.0;
            let epsilon = f64::from(f32::EPSILON);
            let relative_error = operation_count * epsilon / (1.0 - operation_count * epsilon);
            let underflow_error = operation_count * f64::from(f32::from_bits(1));
            let error = error_scale * relative_error + DVec3::splat(underflow_error);
            if !error_scale.is_finite() || !error.is_finite() {
                return Err(AabbError::TransformOverflow);
            }

            transformed_min -= error;
            transformed_max += error;
        }
        if !transformed_min.is_finite() || !transformed_max.is_finite() {
            return Err(AabbError::TransformOverflow);
        }

        let min = Vec3::new(
            round_down_to_f32(transformed_min.x)?,
            round_down_to_f32(transformed_min.y)?,
            round_down_to_f32(transformed_min.z)?,
        );
        let max = Vec3::new(
            round_up_to_f32(transformed_max.x)?,
            round_up_to_f32(transformed_max.y)?,
            round_up_to_f32(transformed_max.z)?,
        );

        Self::new(min, max).map_err(|_| AabbError::TransformOverflow)
    }
}

pub(crate) fn round_down_to_f32(value: f64) -> Result<f32, AabbError> {
    if !value.is_finite() || value < -(f32::MAX as f64) || value > f32::MAX as f64 {
        return Err(AabbError::TransformOverflow);
    }

    let rounded = value as f32;
    Ok(if f64::from(rounded) > value {
        rounded.next_down()
    } else {
        rounded
    })
}

pub(crate) fn round_up_to_f32(value: f64) -> Result<f32, AabbError> {
    if !value.is_finite() || value < -(f32::MAX as f64) || value > f32::MAX as f64 {
        return Err(AabbError::TransformOverflow);
    }

    let rounded = value as f32;
    Ok(if f64::from(rounded) < value {
        rounded.next_up()
    } else {
        rounded
    })
}

#[cfg(test)]
#[path = "aabb_tests.rs"]
mod tests;
