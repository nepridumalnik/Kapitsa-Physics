use std::fmt;

use crate::{shape::BoundsError, transform::TransformError};

/// Reason collider world bounds could not be computed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ColliderError {
    Transform(TransformError),
    Bounds(BoundsError),
}

impl fmt::Display for ColliderError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Transform(error) => write!(formatter, "invalid collider transform: {error}"),
            Self::Bounds(error) => write!(formatter, "could not compute collider bounds: {error}"),
        }
    }
}

impl std::error::Error for ColliderError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Transform(error) => Some(error),
            Self::Bounds(error) => Some(error),
        }
    }
}

impl From<TransformError> for ColliderError {
    fn from(error: TransformError) -> Self {
        Self::Transform(error)
    }
}

impl From<BoundsError> for ColliderError {
    fn from(error: BoundsError) -> Self {
        Self::Bounds(error)
    }
}
