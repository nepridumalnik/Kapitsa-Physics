use std::fmt;

/// Failure while validating or evaluating a collision step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionError {
    InvalidConfig,
    UnsupportedDynamicGeometry,
    InvalidSkinPose,
    InvalidMaterial,
    UnsupportedGeometryPair,
    NonConvergent,
    CcdLimitExceeded,
    ArithmeticOverflow,
    SpatialIndexCapacityExceeded,
    GeometryBounds,
    InvalidContact,
}

impl fmt::Display for CollisionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidConfig => "collision configuration is invalid",
            Self::UnsupportedDynamicGeometry => "dynamic bodies require convex collision geometry",
            Self::InvalidSkinPose => "skin pose does not match the collider geometry",
            Self::InvalidMaterial => "collider references an invalid physics material",
            Self::UnsupportedGeometryPair => "collision geometry pair is unsupported",
            Self::NonConvergent => "collision calculation did not converge",
            Self::CcdLimitExceeded => "continuous collision impact limit was exceeded",
            Self::ArithmeticOverflow => "collision calculation exceeds the supported range",
            Self::SpatialIndexCapacityExceeded => "spatial index capacity could not be reserved",
            Self::GeometryBounds => "collision geometry bounds could not be computed",
            Self::InvalidContact => "collision calculation produced an invalid contact",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for CollisionError {}

/// Failure while evaluating a spatial query.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueryError {
    InvalidInput,
    UnsupportedQueryShape,
    Geometry(CollisionError),
    ArithmeticOverflow,
    NonConvergent,
}

impl fmt::Display for QueryError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput => formatter.write_str("query input is invalid"),
            Self::UnsupportedQueryShape => formatter.write_str("query shape is unsupported"),
            Self::Geometry(error) => write!(formatter, "query geometry failed: {error}"),
            Self::ArithmeticOverflow => formatter.write_str("query exceeds the supported range"),
            Self::NonConvergent => formatter.write_str("query calculation did not converge"),
        }
    }
}

impl std::error::Error for QueryError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Geometry(error) => Some(error),
            Self::InvalidInput
            | Self::UnsupportedQueryShape
            | Self::ArithmeticOverflow
            | Self::NonConvergent => None,
        }
    }
}
