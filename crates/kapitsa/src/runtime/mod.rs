//! Simulation runtime configuration and execution APIs.

#[cfg(feature = "async")]
pub mod asynchronous;
mod config;
mod engine;
#[cfg(feature = "async")]
pub mod executor;
pub mod metrics;
#[cfg(feature = "async")]
mod planner;
pub mod sync;

#[cfg(feature = "async")]
pub use asynchronous::{
    CollisionEventBatch, CollisionEventStreamError, CollisionEvents, PhysicsClient,
    PhysicsClientError, PhysicsCommand, PhysicsCommandResult, PhysicsService, PhysicsServiceError,
    PhysicsSnapshot, Simulation as AsyncSimulation, SimulationError as AsyncSimulationError,
};
pub use config::{ConfigError, SchedulerConfig, SimulationConfig};
#[cfg(feature = "async")]
pub use executor::{RayonExecutor, RayonPool, RayonPoolError};
pub use metrics::{SimulationMetrics, SimulationState, StepMeasurement, StepStats};
#[cfg(feature = "async")]
pub use planner::{AdaptiveScheduler, AdaptiveStatus, WorkerConfig, WorkerConfigError};
