use std::fmt;

use crate::aabb::AabbError;

/// Reason shape geometry or its input data could not be constructed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeError {
    NonFiniteParameter,
    InvalidDimension,
    DegenerateGeometry,
    InvalidTopology,
    InvalidGridDimensions,
    DataLengthMismatch,
    InvalidBoneNames,
    InvalidInfluenceIndex,
    InvalidWeight,
    NonFiniteTransform,
    ArithmeticOverflow,
}

impl fmt::Display for ShapeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::NonFiniteParameter => "shape parameters must be finite",
            Self::InvalidDimension => "shape dimensions must be positive",
            Self::DegenerateGeometry => "shape geometry is degenerate",
            Self::InvalidTopology => "shape topology is invalid",
            Self::InvalidGridDimensions => "grid dimensions are invalid",
            Self::DataLengthMismatch => "shape data length does not match its dimensions",
            Self::InvalidBoneNames => "bone names must be nonempty and unique",
            Self::InvalidInfluenceIndex => "bone influence index is invalid",
            Self::InvalidWeight => "bone influence weight is invalid",
            Self::NonFiniteTransform => "reference transforms must be finite",
            Self::ArithmeticOverflow => "shape dimensions or calculations overflow",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for ShapeError {}

/// Reason shape bounds could not be computed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoundsError {
    InvalidBounds(AabbError),
    InvalidGeometry,
    NotImplemented,
}

impl fmt::Display for BoundsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidBounds(error) => write!(formatter, "invalid shape bounds: {error}"),
            Self::InvalidGeometry => formatter.write_str("shape geometry invariant is invalid"),
            Self::NotImplemented => formatter.write_str("shape bounds are not implemented"),
        }
    }
}

impl std::error::Error for BoundsError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidBounds(error) => Some(error),
            Self::InvalidGeometry | Self::NotImplemented => None,
        }
    }
}
