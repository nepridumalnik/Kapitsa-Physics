use std::fmt;

/// Reason a physics material parameter is invalid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhysicsMaterialError {
    NonFiniteFriction,
    NonFiniteStaticFriction,
    NonFiniteRestitution,
    NonFiniteDensity,
    NegativeFriction,
    NegativeStaticFriction,
    RestitutionOutOfRange,
    NonPositiveDensity,
}

impl fmt::Display for PhysicsMaterialError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::NonFiniteFriction => "friction must be finite",
            Self::NonFiniteStaticFriction => "static friction must be finite",
            Self::NonFiniteRestitution => "restitution must be finite",
            Self::NonFiniteDensity => "density must be finite",
            Self::NegativeFriction => "friction must be nonnegative",
            Self::NegativeStaticFriction => "static friction must be nonnegative",
            Self::RestitutionOutOfRange => "restitution must be in the inclusive range [0, 1]",
            Self::NonPositiveDensity => "density must be positive",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for PhysicsMaterialError {}
