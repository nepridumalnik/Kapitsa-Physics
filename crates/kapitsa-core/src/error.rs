//! Unified public error surface.
//!
//! `Error` preserves the original typed cause as its `source`, so callers can
//! inspect details with [`Error::downcast_ref`] while matching broad categories
//! through [`Error::kind`]. Structured runtime context such as a command index or
//! a failed step attempt is available through typed accessors.

use std::{fmt, sync::Arc};

use crate::{
    aabb::AabbError,
    aerodynamics::{
        AerodynamicsError, geometry::GeometryError, hull::HullError, pressure::PressureError,
        sphere_drag::SphereDragError, wave::WaveDragError, wing::WingError,
    },
    chunk::ChunkBuildError,
    cluster::{ClusterError, balance::BalanceError},
    collider::ColliderError,
    collision::{CollisionError, QueryError},
    command::{CommandBatchError, CommandError},
    constraint::ConstraintError,
    containers::DensePoolError,
    execution::ExecutionError,
    gravity::GravityError,
    joint::JointError,
    mass::MassError,
    material::PhysicsMaterialError,
    rigid_body::{BodyError, IntegrationError},
    shape::{BoundsError, ShapeError},
    transform::TransformError,
    world::SimpleSurfaceAttributionError,
    world::{
        AddChunkBatchError, AddChunkError, ChunkOperationError, ForceSourceError, StepError,
        WorldError,
    },
};

/// Broad failure domain of an [`Error`].
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ErrorKind {
    /// Invalid input, identifier, geometry, or numeric domain.
    Validation,
    /// Collision detection, contact resolution, or spatial queries.
    Collision,
    /// A physics step failed before committing.
    Step,
    /// Body integration failed.
    Integration,
    /// Gravity field evaluation failed.
    Gravity,
    /// Chunk construction or streaming.
    Chunk,
    /// Rigid-cluster topology or mass management.
    Cluster,
    /// Joint validation or solving.
    Joint,
    /// Material storage or assignment.
    Material,
    /// Runtime or simulation configuration.
    Config,
    /// Spatial or collision query failure.
    Query,
    /// Aerodynamic model or geometry failure.
    Aerodynamics,
    /// Asynchronous runtime, command submission, or snapshot failure.
    Runtime,
    /// A bounded resource was exhausted.
    ResourceExhausted,
    /// Host input/output failure.
    Io,
}

impl fmt::Display for ErrorKind {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Self::Validation => "validation",
            Self::Collision => "collision",
            Self::Step => "step",
            Self::Integration => "integration",
            Self::Gravity => "gravity",
            Self::Chunk => "chunk",
            Self::Cluster => "cluster",
            Self::Joint => "joint",
            Self::Material => "material",
            Self::Config => "config",
            Self::Query => "query",
            Self::Aerodynamics => "aerodynamics",
            Self::Runtime => "runtime",
            Self::ResourceExhausted => "resource exhaustion",
            Self::Io => "I/O",
        };
        formatter.write_str(name)
    }
}

/// The engine's unified error.
///
/// The display message describes the concrete failure. The original typed error
/// remains reachable through [`std::error::Error::source`] and
/// [`Error::downcast_ref`]. Structured context that callers previously matched
/// directly is available through [`Error::command_index`] and
/// [`Error::failed_step`].
#[derive(Clone, Debug)]
pub struct Error {
    kind: ErrorKind,
    message: String,
    cause: Option<Arc<dyn std::error::Error + Send + Sync + 'static>>,
    command_index: Option<usize>,
    failed_step: Option<u64>,
    lagged_batches: Option<u64>,
}

impl Error {
    /// Creates an error with a static message and no typed cause.
    pub fn new(kind: ErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            cause: None,
            command_index: None,
            failed_step: None,
            lagged_batches: None,
        }
    }

    /// Creates an error from a typed cause, using its display text as the message.
    pub fn from_source<E>(kind: ErrorKind, cause: E) -> Self
    where
        E: std::error::Error + Send + Sync + 'static,
    {
        Self {
            kind,
            message: cause.to_string(),
            cause: Some(Arc::new(cause)),
            command_index: None,
            failed_step: None,
            lagged_batches: None,
        }
    }

    /// Broad failure domain.
    pub fn kind(&self) -> ErrorKind {
        self.kind
    }

    /// Index of the command that rejected a buffer, if applicable.
    pub fn command_index(&self) -> Option<usize> {
        self.command_index
    }

    /// Failed step attempt number, if applicable.
    pub fn failed_step(&self) -> Option<u64> {
        self.failed_step
    }

    /// Number of discarded stream batches, if applicable.
    pub fn lagged_batches(&self) -> Option<u64> {
        self.lagged_batches
    }

    /// Adds command-index context.
    pub fn with_command_index(mut self, command_index: usize) -> Self {
        self.command_index = Some(command_index);
        self
    }

    /// Adds failed-step context.
    pub fn with_failed_step(mut self, step: u64) -> Self {
        self.failed_step = Some(step);
        self
    }

    /// Adds discarded-stream-batch context.
    pub fn with_lagged_batches(mut self, missed_batches: u64) -> Self {
        self.lagged_batches = Some(missed_batches);
        self
    }

    /// Reclassifies the error domain while preserving its cause.
    pub fn with_kind(mut self, kind: ErrorKind) -> Self {
        self.kind = kind;
        self
    }

    /// Downcasts the preserved typed cause.
    pub fn downcast_ref<T>(&self) -> Option<&T>
    where
        T: std::error::Error + 'static,
    {
        self.cause.as_ref()?.downcast_ref::<T>()
    }
}

impl PartialEq for Error {
    fn eq(&self, other: &Self) -> bool {
        self.kind == other.kind
            && self.message == other.message
            && self.command_index == other.command_index
            && self.failed_step == other.failed_step
            && self.lagged_batches == other.lagged_batches
    }
}

impl Eq for Error {}

impl fmt::Display for Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        self.cause
            .as_ref()
            .map(|cause| cause.as_ref() as &(dyn std::error::Error + 'static))
    }
}

const _: () = {
    const fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<Error>();
};

macro_rules! impl_error_from {
    ($ty:ty, $kind:ident) => {
        impl From<$ty> for Error {
            fn from(error: $ty) -> Self {
                Self::from_source(ErrorKind::$kind, error)
            }
        }
    };
}

impl_error_from!(AabbError, Validation);
impl_error_from!(AerodynamicsError, Aerodynamics);
impl_error_from!(GeometryError, Aerodynamics);
impl_error_from!(HullError, Aerodynamics);
impl_error_from!(PressureError, Aerodynamics);
impl_error_from!(SphereDragError, Aerodynamics);
impl_error_from!(WaveDragError, Aerodynamics);
impl_error_from!(WingError, Aerodynamics);
impl_error_from!(AddChunkBatchError, Chunk);
impl_error_from!(AddChunkError, Chunk);
impl_error_from!(ChunkBuildError, Chunk);
impl_error_from!(ChunkOperationError, Chunk);
impl_error_from!(ClusterError, Cluster);
impl_error_from!(BalanceError, Cluster);
impl_error_from!(SimpleSurfaceAttributionError, Validation);
impl_error_from!(ColliderError, Validation);
impl_error_from!(CollisionError, Collision);
impl_error_from!(ConstraintError, Runtime);
impl_error_from!(QueryError, Query);
impl_error_from!(CommandError, Validation);
impl_error_from!(DensePoolError, ResourceExhausted);
impl_error_from!(ExecutionError, Runtime);
impl_error_from!(GravityError, Gravity);
impl_error_from!(JointError, Joint);
impl_error_from!(ForceSourceError, Validation);
impl_error_from!(MassError, Validation);
impl_error_from!(PhysicsMaterialError, Material);
impl_error_from!(BodyError, Validation);
impl_error_from!(IntegrationError, Integration);
impl_error_from!(BoundsError, Validation);
impl_error_from!(ShapeError, Validation);
impl_error_from!(TransformError, Validation);
impl_error_from!(StepError, Step);
impl From<WorldError> for Error {
    fn from(error: WorldError) -> Self {
        let kind = match &error {
            WorldError::DensePoolIndexOverflow
            | WorldError::DensePoolCapacityExceeded
            | WorldError::DensePoolInvalidFreeList
            | WorldError::DensePoolIdentifierExhausted
            | WorldError::WorldIdExhausted => ErrorKind::ResourceExhausted,
            WorldError::Collision(_) => ErrorKind::Collision,
            _ => ErrorKind::Validation,
        };
        Self::from_source(kind, error)
    }
}

impl From<CommandBatchError> for Error {
    fn from(error: CommandBatchError) -> Self {
        let mut unified = Self::from_source(ErrorKind::Validation, error.error);
        unified.command_index = error.command_index;
        unified
    }
}

#[cfg(test)]
mod tests {
    use super::{Error, ErrorKind};

    #[test]
    fn preserves_kind_message_and_typed_source() {
        let error = Error::from(crate::rigid_body::BodyError::NonFiniteValue);
        assert_eq!(error.kind(), ErrorKind::Validation);
        assert!(!error.to_string().is_empty());
        assert!(
            error
                .downcast_ref::<crate::rigid_body::BodyError>()
                .is_some()
        );
        assert!(std::error::Error::source(&error).is_some());
    }

    #[test]
    fn unified_error_is_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Error>();
    }

    #[test]
    fn command_index_survives_batch_conversion() {
        let batch = crate::command::CommandBatchError {
            command_index: Some(3),
            error: crate::command::CommandError::UnsupportedBatchCommand,
        };
        let error = Error::from(batch);
        assert_eq!(error.kind(), ErrorKind::Validation);
        assert_eq!(error.command_index(), Some(3));
    }
}
