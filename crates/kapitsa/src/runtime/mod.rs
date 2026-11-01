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
    BatchId, CollisionEventBatch, CollisionEventStreamError, CollisionEvents, PhysicsBatchError,
    PhysicsBatchResult, PhysicsBatchResultStreamError, PhysicsBatchResults, PhysicsClient,
    PhysicsClientError, PhysicsCommand, PhysicsCommandBuffer, PhysicsCommandResult,
    PhysicsSnapshot, PhysicsSubmitError, Simulation, SimulationError,
};
pub use config::{ConfigError, SchedulerConfig, SimulationConfig};
pub use metrics::{SimulationMetrics, SimulationState, StepMeasurement, StepStats};
pub use planner::{WorkerConfig, WorkerConfigError};
