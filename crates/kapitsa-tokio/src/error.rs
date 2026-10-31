use std::fmt;

use kapitsa_core::world::StepError;

/// Failure returned while configuring or advancing the physics scheduler.
#[derive(Debug)]
pub enum SchedulerError {
    InvalidConfig(&'static str),
    MissingRuntime,
    SimulationTimeOverflow,
    WorldUnavailable,
    WorldLost,
    BlockingTaskFailed(String),
    PhysicsStep(StepError),
}

impl fmt::Display for SchedulerError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig(reason) => write!(formatter, "invalid scheduler config: {reason}"),
            Self::MissingRuntime => formatter.write_str("advance requires an active Tokio runtime"),
            Self::SimulationTimeOverflow => {
                formatter.write_str("simulation time accumulator overflowed")
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
