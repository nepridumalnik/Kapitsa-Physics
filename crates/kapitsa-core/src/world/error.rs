use std::fmt;

use crate::collision::CollisionError;
use crate::containers::DensePoolError;
use crate::gravity::GravityError;
use crate::rigid_body::{BodyError, IntegrationError};

use super::{BodyId, ForceSourceId};
use crate::joint::JointError;

/// Failure while configuring or applying a persistent body force.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ForceSourceError {
    InvalidBodyId,
    InvalidForceSourceId,
    BodyMustBeDynamic,
    NonFiniteValue,
    ArithmeticOverflow,
    StorageCapacityExceeded,
}

impl fmt::Display for ForceSourceError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidBodyId => "force source body ID is invalid",
            Self::InvalidForceSourceId => "force source ID is invalid",
            Self::BodyMustBeDynamic => "force sources require a dynamic body",
            Self::NonFiniteValue => "force source values must be finite",
            Self::ArithmeticOverflow => "force source calculation exceeds the supported range",
            Self::StorageCapacityExceeded => "force source storage capacity could not be reserved",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for ForceSourceError {}

impl From<BodyError> for ForceSourceError {
    fn from(error: BodyError) -> Self {
        match error {
            BodyError::BodyMustBeDynamic => Self::BodyMustBeDynamic,
            BodyError::NonFiniteValue => Self::NonFiniteValue,
            BodyError::ArithmeticOverflow | BodyError::InvalidInertiaTensor => {
                Self::ArithmeticOverflow
            }
            BodyError::StaticBodyCannotHaveVelocity => Self::BodyMustBeDynamic,
        }
    }
}

/// Failure returned when a world timestep cannot be completed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepError {
    InvalidDeltaTime,
    InvalidStepResults,
    Collision(CollisionError),
    BodyIntegration {
        body_id: BodyId,
        cause: IntegrationError,
    },
    ForceSource {
        source_id: ForceSourceId,
        cause: ForceSourceError,
    },
    Gravity {
        body_id: BodyId,
        cause: GravityError,
    },
}

impl fmt::Display for StepError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDeltaTime => formatter.write_str("timestep must be positive and finite"),
            Self::InvalidStepResults => {
                formatter.write_str("predicted body states do not match the world")
            }
            Self::Collision(error) => write!(formatter, "collision step failed: {error}"),
            Self::BodyIntegration { body_id, cause } => {
                write!(formatter, "integration failed for {body_id:?}: {cause}")
            }
            Self::ForceSource { source_id, cause } => {
                write!(formatter, "force source {source_id:?} failed: {cause}")
            }
            Self::Gravity { body_id, cause } => {
                write!(formatter, "gravity failed for {body_id:?}: {cause}")
            }
        }
    }
}

impl std::error::Error for StepError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Collision(error) => Some(error),
            Self::InvalidDeltaTime | Self::InvalidStepResults => None,
            Self::BodyIntegration { cause, .. } => Some(cause),
            Self::ForceSource { cause, .. } => Some(cause),
            Self::Gravity { cause, .. } => Some(cause),
        }
    }
}

/// Reason an object could not be added or a world ID could not be resolved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorldError {
    DensePoolIndexOverflow,
    DensePoolCapacityExceeded,
    DensePoolInvalidFreeList,
    WorldIdExhausted,
    InvalidBodyId,
    InvalidColliderId,
    InvalidJointId,
    InvalidJointBodies,
    InvalidMaterialId,
    MaterialInUse,
    Collision(CollisionError),
    Joint(JointError),
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
            Self::WorldIdExhausted => formatter.write_str("physics world ID space is exhausted"),
            Self::InvalidBodyId => formatter.write_str("body ID is invalid for this world"),
            Self::InvalidColliderId => formatter.write_str("collider ID is invalid for this world"),
            Self::InvalidJointId => formatter.write_str("joint ID is invalid for this world"),
            Self::InvalidJointBodies => {
                formatter.write_str("joint body IDs must identify two bodies in this world")
            }
            Self::InvalidMaterialId => formatter.write_str("material ID is invalid for this world"),
            Self::MaterialInUse => {
                formatter.write_str("physics material is assigned to a collider")
            }
            Self::Collision(error) => write!(formatter, "collision configuration failed: {error}"),
            Self::Joint(error) => write!(formatter, "joint configuration failed: {error}"),
        }
    }
}

impl std::error::Error for WorldError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Collision(error) => Some(error),
            Self::Joint(error) => Some(error),
            Self::DensePoolIndexOverflow
            | Self::DensePoolCapacityExceeded
            | Self::DensePoolInvalidFreeList
            | Self::WorldIdExhausted
            | Self::InvalidBodyId
            | Self::InvalidColliderId
            | Self::InvalidJointId
            | Self::InvalidJointBodies
            | Self::InvalidMaterialId
            | Self::MaterialInUse => None,
        }
    }
}

impl From<DensePoolError> for WorldError {
    fn from(error: DensePoolError) -> Self {
        match error {
            DensePoolError::IndexOverflow => Self::DensePoolIndexOverflow,
            DensePoolError::CapacityExceeded => Self::DensePoolCapacityExceeded,
            DensePoolError::InvalidFreeList => Self::DensePoolInvalidFreeList,
            DensePoolError::IdentifierExhausted => Self::DensePoolCapacityExceeded,
        }
    }
}

impl From<CollisionError> for WorldError {
    fn from(error: CollisionError) -> Self {
        Self::Collision(error)
    }
}

impl From<JointError> for WorldError {
    fn from(error: JointError) -> Self {
        Self::Joint(error)
    }
}
