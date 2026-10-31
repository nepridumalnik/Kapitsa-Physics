use std::fmt;

/// Reason a rigid transform could not be constructed or inverted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransformError {
    NonFiniteTranslation,
    InvalidRotation,
    InverseOverflow,
}

impl fmt::Display for TransformError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFiniteTranslation => formatter.write_str("translation must be finite"),
            Self::InvalidRotation => {
                formatter.write_str("rotation must be a finite unit quaternion")
            }
            Self::InverseOverflow => formatter.write_str("inverse translation is not finite"),
        }
    }
}

impl std::error::Error for TransformError {}
