use std::fmt;

use kapitsa_core::world::StepError;

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
