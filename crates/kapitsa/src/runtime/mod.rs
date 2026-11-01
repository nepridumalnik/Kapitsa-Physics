//! Simulation runtime configuration and execution APIs.

mod asynchronous;
mod config;
mod engine;
mod executor;
pub mod metrics;
mod planner;
#[cfg(test)]
mod sync;

pub use asynchronous::{
    CollisionEventBatch, CollisionEventStreamError, CollisionEvents, PhysicsClient,
    PhysicsClientError, PhysicsCommand, PhysicsCommandResult, PhysicsSnapshot, Simulation,
    SimulationError,
};
pub use config::{ConfigError, SchedulerConfig, SimulationConfig};
pub use metrics::{SimulationMetrics, SimulationState, StepMeasurement, StepStats};
pub use planner::{WorkerConfig, WorkerConfigError};
