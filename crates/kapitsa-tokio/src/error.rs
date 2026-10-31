use std::fmt;

use kapitsa_core::{
    rigid_body::BodyError,
    world::{StepError, WorldError},
};

/// Failure returned while submitting or applying a physics command.
#[derive(Debug)]
pub enum PhysicsClientError {
    /// The simulation is stopped or is stopping.
    NotRunning,
    /// A command or snapshot channel closed unexpectedly.
    ChannelClosed,
    /// An operation failed while accessing the physics world.
    World(WorldError),
    /// A rigid-body operation rejected its input or current body type.
    Body(BodyError),
    /// A command handler returned a result variant inconsistent with the request.
    UnexpectedResult,
}

impl fmt::Display for PhysicsClientError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotRunning => formatter.write_str("physics simulation is not running"),
            Self::ChannelClosed => formatter.write_str("physics command channel is closed"),
            Self::World(error) => write!(formatter, "physics world command failed: {error}"),
            Self::Body(error) => write!(formatter, "rigid body command failed: {error}"),
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

/// Failure returned while configuring or stepping the physics scheduler.
#[derive(Debug)]
pub enum SchedulerError {
    InvalidConfig(&'static str),
    MissingRuntime,
    WorldUnavailable,
    WorldLost,
    BlockingTaskFailed(String),
    PhysicsStep(StepError),
}

impl fmt::Display for SchedulerError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig(reason) => write!(formatter, "invalid scheduler config: {reason}"),
            Self::MissingRuntime => {
                formatter.write_str("physics steps require an active Tokio runtime")
            }
            Self::WorldUnavailable => {
                formatter.write_str("physics world is currently owned by a blocking task")
            }
            Self::WorldLost => {
                formatter.write_str("physics world was lost after a blocking task failure")
            }
            Self::BlockingTaskFailed(reason) => {
                write!(formatter, "blocking physics task failed: {reason}")
            }
            Self::PhysicsStep(error) => write!(formatter, "physics step failed: {error}"),
        }
    }
}

impl std::error::Error for SchedulerError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::PhysicsStep(error) => Some(error),
            _ => None,
        }
    }
}

impl From<StepError> for SchedulerError {
    fn from(error: StepError) -> Self {
        Self::PhysicsStep(error)
    }
}

/// Failure returned by the autonomous physics simulation lifecycle.
#[derive(Debug)]
pub enum SimulationError {
    MissingRuntime,
    SimulationTimeOverflow,
    WorldUnavailable,
    TaskRequiresStop,
    Scheduler(SchedulerError),
    TaskFailed(String),
}

impl fmt::Display for SimulationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingRuntime => formatter
                .write_str("starting a physics simulation requires an active Tokio runtime"),
            Self::SimulationTimeOverflow => {
                formatter.write_str("simulation time accumulator overflowed")
            }
            Self::WorldUnavailable => formatter
                .write_str("physics world is only accessible while the simulation is stopped"),
            Self::TaskRequiresStop => {
                formatter.write_str("stop the completed simulation task before restarting")
            }
            Self::Scheduler(error) => write!(formatter, "physics simulation failed: {error}"),
            Self::TaskFailed(reason) => {
                write!(formatter, "physics simulation task failed: {reason}")
            }
        }
    }
}

impl std::error::Error for SimulationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Scheduler(error) => Some(error),
            _ => None,
        }
    }
}

impl From<SchedulerError> for SimulationError {
    fn from(error: SchedulerError) -> Self {
        Self::Scheduler(error)
    }
}
