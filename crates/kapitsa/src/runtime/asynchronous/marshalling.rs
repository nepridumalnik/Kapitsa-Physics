mod client;
mod command;
mod events;
mod snapshot;

pub use client::PhysicsClient;
pub(crate) use command::{CommandRequest, apply_command};
pub use command::{PhysicsCommand, PhysicsCommandResult};
pub use events::{CollisionEventBatch, CollisionEventStreamError, CollisionEvents};
pub use snapshot::PhysicsSnapshot;

use std::fmt;

use kapitsa_core::{collision::QueryError, rigid_body::BodyError, world::WorldError};

/// Failure returned while submitting or applying a physics command.
#[derive(Debug)]
pub enum PhysicsClientError {
    NotRunning,
    ChannelClosed,
    World(WorldError),
    Body(BodyError),
    Query(QueryError),
    UnexpectedResult,
}

impl fmt::Display for PhysicsClientError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotRunning => formatter.write_str("physics service is not running"),
            Self::ChannelClosed => formatter.write_str("physics command channel is closed"),
            Self::World(error) => write!(formatter, "physics world command failed: {error}"),
            Self::Body(error) => write!(formatter, "rigid body command failed: {error}"),
            Self::Query(error) => write!(formatter, "physics query failed: {error}"),
            Self::UnexpectedResult => {
                formatter.write_str("physics command returned an unexpected result")
            }
        }
    }
}

impl std::error::Error for PhysicsClientError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::World(error) => Some(error),
            Self::Body(error) => Some(error),
            Self::Query(error) => Some(error),
            Self::NotRunning | Self::ChannelClosed | Self::UnexpectedResult => None,
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

impl From<QueryError> for PhysicsClientError {
    fn from(error: QueryError) -> Self {
        Self::Query(error)
    }
}
