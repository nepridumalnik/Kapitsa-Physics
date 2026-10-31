use std::fmt;

/// Reason a rigid-body operation could not be completed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyError {
    BodyMustBeDynamic,
    StaticBodyCannotHaveVelocity,
    NonFiniteValue,
    ArithmeticOverflow,
    InvalidInertiaTensor,
}

impl fmt::Display for BodyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::BodyMustBeDynamic => "operation requires a dynamic body",
            Self::StaticBodyCannotHaveVelocity => "static bodies cannot have velocity",
            Self::NonFiniteValue => "value must be finite",
            Self::ArithmeticOverflow => "rigid-body calculation exceeds the supported range",
            Self::InvalidInertiaTensor => "inertia tensor could not be inverted",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for BodyError {}
