//! Simulation runtime configuration and execution APIs.

#[cfg(feature = "async")]
mod asynchronous;
mod config;
#[cfg(any(test, feature = "async"))]
mod engine;
#[cfg(feature = "async")]
mod executor;
pub mod metrics;
mod planner;
#[cfg(test)]
mod sync;

#[cfg(feature = "async")]
pub use asynchronous::{
    CollisionEventBatch, CollisionEventStreamError, CollisionEvents, PhysicsClient,
    PhysicsClientError, PhysicsCommand, PhysicsCommandResult, PhysicsSnapshot, Simulation,
    SimulationError,
};
pub use config::{ConfigError, SchedulerConfig, SimulationConfig};
#[cfg(feature = "async")]
pub use executor::RayonPoolError;
pub use metrics::{SimulationMetrics, SimulationState, StepMeasurement, StepStats};
pub use planner::{WorkerConfig, WorkerConfigError};
