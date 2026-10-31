use std::fmt;

use crate::collision::CollisionError;
use crate::containers::DensePoolError;
use crate::rigid_body::IntegrationError;

use super::BodyId;

/// Failure returned when a world timestep cannot be completed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepError {
    InvalidDeltaTime,
    InvalidStepResults,
    Collision(CollisionError),
    BodyIntegration {
        body_id: BodyId,
        cause: IntegrationError,
    },
}

impl fmt::Display for StepError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDeltaTime => formatter.write_str("timestep must be positive and finite"),
            Self::InvalidStepResults => {
                formatter.write_str("predicted body states do not match the world")
            }
            Self::Collision(error) => write!(formatter, "collision step failed: {error}"),
            Self::BodyIntegration { body_id, cause } => {
                write!(formatter, "integration failed for {body_id:?}: {cause}")
            }
        }
    }
}

impl std::error::Error for StepError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Collision(error) => Some(error),
            Self::InvalidDeltaTime | Self::InvalidStepResults => None,
            Self::BodyIntegration { cause, .. } => Some(cause),
        }
    }
}

/// Reason an object could not be added or a world ID could not be resolved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorldError {
    DensePoolIndexOverflow,
    DensePoolCapacityExceeded,
    DensePoolInvalidFreeList,
    WorldIdExhausted,
    InvalidBodyId,
    InvalidColliderId,
}

impl fmt::Display for WorldError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DensePoolIndexOverflow => formatter.write_str("world storage index exceeds u32"),
            Self::DensePoolCapacityExceeded => {
                formatter.write_str("world storage capacity could not be reserved")
            }
            Self::DensePoolInvalidFreeList => {
                formatter.write_str("world storage free list is invalid")
            }
            Self::WorldIdExhausted => formatter.write_str("physics world ID space is exhausted"),
            Self::InvalidBodyId => formatter.write_str("body ID is invalid for this world"),
            Self::InvalidColliderId => formatter.write_str("collider ID is invalid for this world"),
        }
    }
}

impl std::error::Error for WorldError {}

impl From<DensePoolError> for WorldError {
    fn from(error: DensePoolError) -> Self {
        match error {
            DensePoolError::IndexOverflow => Self::DensePoolIndexOverflow,
            DensePoolError::CapacityExceeded => Self::DensePoolCapacityExceeded,
            DensePoolError::InvalidFreeList => Self::DensePoolInvalidFreeList,
        }
    }
}
