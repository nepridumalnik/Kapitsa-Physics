mod body;
mod geometry;
mod solver;

pub(crate) use body::MomentumBody;
pub(crate) use geometry::{EqualityRow, MAX_EQUALITY_ROWS, write_equality_rows};
pub(crate) use solver::{ConstrainedIntegrator, ConstrainedWorkspace, IslandEligibility};

use crate::collision::CollisionError;
use kapitsa_math::{Quat, Vec3};

fn finite(value: Vec3) -> Result<Vec3, CollisionError> {
    value
        .is_finite()
        .then_some(value)
        .ok_or(CollisionError::ArithmeticOverflow)
}
fn length(value: Vec3) -> Result<f64, CollisionError> {
    value
        .checked_length()
        .ok_or(CollisionError::ArithmeticOverflow)
}
fn rotate(rotation: Quat, value: Vec3) -> Result<Vec3, CollisionError> {
    rotation
        .rotate_vector(value)
        .ok_or(CollisionError::ArithmeticOverflow)
}

#[cfg(test)]
mod tests;
