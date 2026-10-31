use std::fmt;

/// Reason an axis-aligned bounding box could not be constructed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AabbError {
    NonFiniteBounds,
    InvertedBounds,
    SizeOverflow,
    TransformOverflow,
}

impl fmt::Display for AabbError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFiniteBounds => formatter.write_str("AABB bounds must be finite"),
            Self::InvertedBounds => formatter.write_str("AABB minimum must not exceed maximum"),
            Self::SizeOverflow => formatter.write_str("AABB size exceeds the finite range"),
            Self::TransformOverflow => {
                formatter.write_str("transformed AABB exceeds the finite range")
            }
        }
    }
}

impl std::error::Error for AabbError {}
