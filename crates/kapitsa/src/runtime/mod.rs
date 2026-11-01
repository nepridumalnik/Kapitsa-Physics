//! Simulation runtime configuration and execution APIs.

mod config;
mod engine;
pub mod metrics;
pub mod sync;

pub use config::{ConfigError, SchedulerConfig, SimulationConfig};
pub use metrics::{SimulationMetrics, SimulationState, StepMeasurement, StepStats};
