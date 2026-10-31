use std::fmt;

/// Reason an axis-aligned bounding box could not be constructed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AabbError {
    NonFiniteBounds,
    InvertedBounds,
}

impl fmt::Display for AabbError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFiniteBounds => formatter.write_str("AABB bounds must be finite"),
            Self::InvertedBounds => formatter.write_str("AABB minimum must not exceed maximum"),
        }
    }
}

impl std::error::Error for AabbError {}
