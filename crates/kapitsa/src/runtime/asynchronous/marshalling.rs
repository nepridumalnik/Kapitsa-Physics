mod buffer_pool;
mod chunk;
mod client;
mod command;
mod events;
mod snapshot;

pub(crate) use buffer_pool::CommandBufferPool;
pub use chunk::ChunkClientError;
#[cfg(test)]
pub(crate) use chunk::ChunkCommand;
pub(crate) use chunk::{ChunkOutcome, ChunkReply, apply_chunk};
pub use client::PhysicsClient;
#[cfg(all(test, feature = "bench-internals"))]
pub(crate) use command::tests::allocation_tracking;
pub use command::{
    BatchId, PhysicsCommand, PhysicsCommandBuffer, PhysicsCommandResult, PhysicsSubmitError,
};
pub(crate) use command::{BatchScratch, CommandRequest, apply_command, apply_command_batch};
pub use events::{
    CollisionEventBatch, CollisionEvents, JointBreakEventBatch, JointBreakEvents,
    PhysicsBatchResult, PhysicsBatchResults,
};
pub(crate) use events::{
    CollisionEventStreamError, JointBreakEventStreamError, PhysicsBatchResultStreamError,
};
pub use snapshot::PhysicsSnapshot;

use std::fmt;

use kapitsa_core::{
    Error,
    collision::QueryError,
    rigid_body::BodyError,
    world::{ClusterError, ForceSourceError, WorldError},
};

/// Failure returned while submitting or completing a physics command.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum PhysicsClientError {
    NotRunning,
    ChannelClosed,
    World(WorldError),
    Body(BodyError),
    Cluster(ClusterError),
    Query(QueryError),
    ForceSource(ForceSourceError),
    /// The command changed the world, but the associated step failed before
    /// publishing a snapshot. `step` is the failed step attempt.
    StepFailedAfterApply {
        step: u64,
        cause: Error,
    },
    UnsupportedBatchCommand,
    TooManyCommands {
        max_commands: usize,
    },
    UnexpectedResult,
    Unified(Error),
}

impl fmt::Display for PhysicsClientError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotRunning => formatter.write_str("physics service is not running"),
            Self::ChannelClosed => formatter.write_str("physics command channel is closed"),
            Self::World(error) => write!(formatter, "physics world command failed: {error}"),
            Self::Body(error) => write!(formatter, "rigid body command failed: {error}"),
            Self::Cluster(error) => write!(formatter, "rigid cluster command failed: {error}"),
            Self::Query(error) => write!(formatter, "physics query failed: {error}"),
            Self::ForceSource(error) => write!(formatter, "force source command failed: {error}"),
            Self::StepFailedAfterApply { step, cause } => write!(
                formatter,
                "physics step {step} failed after the command was applied: {cause}"
            ),
            Self::UnsupportedBatchCommand => {
                formatter.write_str("command cannot be used in an atomic body batch")
            }
            Self::TooManyCommands { max_commands } => write!(
                formatter,
                "command batch exceeds the maximum of {max_commands} commands"
            ),
            Self::UnexpectedResult => {
                formatter.write_str("physics command returned an unexpected result")
            }
            Self::Unified(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for PhysicsClientError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::World(error) => Some(error),
            Self::Body(error) => Some(error),
            Self::Cluster(error) => Some(error),
            Self::Query(error) => Some(error),
            Self::ForceSource(error) => Some(error),
            Self::StepFailedAfterApply { cause, .. } => Some(cause),
            Self::Unified(error) => Some(error),
            Self::NotRunning
            | Self::ChannelClosed
            | Self::UnsupportedBatchCommand
            | Self::TooManyCommands { .. }
            | Self::UnexpectedResult => None,
        }
    }
}

impl From<WorldError> for PhysicsClientError {
    fn from(error: WorldError) -> Self {
        Self::World(error)
    }
}

impl From<BodyError> for PhysicsClientError {
    fn from(error: BodyError) -> Self {
        Self::Body(error)
    }
}

impl From<ClusterError> for PhysicsClientError {
    fn from(error: ClusterError) -> Self {
        match error {
            ClusterError::InvalidBodyId => Self::World(WorldError::InvalidBodyId),
            ClusterError::Body(error) => Self::Body(error),
            error => Self::Cluster(error),
        }
    }
}

impl From<QueryError> for PhysicsClientError {
    fn from(error: QueryError) -> Self {
        Self::Query(error)
    }
}

impl From<ForceSourceError> for PhysicsClientError {
    fn from(error: ForceSourceError) -> Self {
        Self::ForceSource(error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cluster_body_errors_preserve_existing_client_errors() {
        assert_eq!(
            PhysicsClientError::from(ClusterError::InvalidBodyId),
            PhysicsClientError::World(WorldError::InvalidBodyId)
        );
        assert_eq!(
            PhysicsClientError::from(ClusterError::Body(BodyError::NonFiniteValue)),
            PhysicsClientError::Body(BodyError::NonFiniteValue)
        );
        assert_eq!(
            PhysicsClientError::from(ClusterError::ManagedMass),
            PhysicsClientError::Cluster(ClusterError::ManagedMass)
        );
    }
}
