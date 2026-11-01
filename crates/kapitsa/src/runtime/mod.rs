//! Simulation runtime configuration and execution APIs.

mod config;
mod engine;
#[cfg(feature = "async")]
pub mod executor;
pub mod metrics;
pub mod sync;

pub use config::{ConfigError, SchedulerConfig, SimulationConfig};
#[cfg(feature = "async")]
pub use executor::{RayonExecutor, RayonPool, RayonPoolError};
pub use metrics::{SimulationMetrics, SimulationState, StepMeasurement, StepStats};
