use std::fmt;

/// Reason mass properties could not be constructed or calculated.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MassError {
    NonFiniteMass,
    NonPositiveMass,
    NonFiniteCenterOfMass,
    NonFiniteInertia,
    NonSymmetricInertia,
    NonPositiveDefiniteInertia,
    NonPhysicalInertia,
    NonFiniteDensity,
    NonPositiveDensity,
    UnsupportedShape,
    ArithmeticOverflow,
    EmptyComposition,
}

impl fmt::Display for MassError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::NonFiniteMass => "mass must be finite",
            Self::NonPositiveMass => "mass must be positive",
            Self::NonFiniteCenterOfMass => "center of mass must be finite",
            Self::NonFiniteInertia => "inertia tensor must be finite",
            Self::NonSymmetricInertia => "inertia tensor must be symmetric",
            Self::NonPositiveDefiniteInertia => "inertia tensor must be positive definite",
            Self::NonPhysicalInertia => "inertia tensor is not physically realizable",
            Self::NonFiniteDensity => "density must be finite",
            Self::NonPositiveDensity => "density must be positive",
            Self::UnsupportedShape => "mass properties are not implemented for this shape",
            Self::ArithmeticOverflow => "mass property calculation exceeds the supported range",
            Self::EmptyComposition => "cannot combine an empty set of parts",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for MassError {}
