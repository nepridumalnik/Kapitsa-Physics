use std::fmt;

use crate::containers::DensePoolError;

/// Reason an object could not be added or a world ID could not be resolved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorldError {
    DensePoolIndexOverflow,
    DensePoolCapacityExceeded,
    DensePoolInvalidFreeList,
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
