use std::time::Duration;

use kapitsa_core::gravity::CentralGravity;

use super::planner::{WorkerConfig, WorkerConfigError};

const MAX_TICK_RATE: u32 = 1_000_000_000;

/// Shared timestep and batching settings.
#[derive(Clone, Debug, PartialEq)]
pub struct SimulationConfig {
    pub tick_rate: u32,
    pub max_catch_up_steps: usize,
    pub time_step_policy: TimeStepPolicy,
    pub scheduler: SchedulerConfig,
    pub workers: WorkerConfig,
    /// Optional point-mass field configured as `μ = GM`.
    pub central_gravity: Option<CentralGravity>,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            tick_rate: 60,
            max_catch_up_steps: 4,
            time_step_policy: TimeStepPolicy::Fixed,
            scheduler: SchedulerConfig::default(),
            workers: WorkerConfig::default(),
            central_gravity: None,
        }
    }
}

impl SimulationConfig {
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.tick_rate == 0 || self.tick_rate > MAX_TICK_RATE {
            return Err(ConfigError::InvalidTickRate);
        }
        if self.max_catch_up_steps == 0 {
            return Err(ConfigError::ZeroCatchUpSteps);
        }
        if let TimeStepPolicy::Adaptive(config) = self.time_step_policy {
            config.validate()?;
        }
        self.scheduler.validate()?;
        self.workers
            .validate()
            .map_err(ConfigError::InvalidWorkerConfig)
    }

    pub fn fixed_step_seconds(&self) -> f64 {
        1.0 / f64::from(self.tick_rate)
    }

    pub fn step_budget(&self) -> Duration {
        Duration::from_secs_f64(self.fixed_step_seconds())
    }
}

/// Selects fixed-step compatibility or physics-bounded adaptive stepping.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum TimeStepPolicy {
    #[default]
    Fixed,
    Adaptive(AdaptiveTimeStepConfig),
}

/// Accuracy and safety bounds used by the adaptive Time Warp policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdaptiveTimeStepConfig {
    /// Refuse a smaller step and report the requested warp as unattainable.
    pub minimum_step: Duration,
    /// Absolute safety cap; physics estimates can choose a smaller step.
    pub maximum_step: Duration,
    /// Dimensionless safety factor for motion, acceleration, and joint bounds, in `(0, 1]`.
    pub relative_error_tolerance: f64,
}

impl Default for AdaptiveTimeStepConfig {
    fn default() -> Self {
        Self {
            minimum_step: Duration::from_micros(10),
            maximum_step: Duration::from_millis(500),
            relative_error_tolerance: 0.01,
        }
    }
}

impl AdaptiveTimeStepConfig {
    fn validate(self) -> Result<(), ConfigError> {
        if self.minimum_step.is_zero()
            || self.maximum_step < self.minimum_step
            || !self.relative_error_tolerance.is_finite()
            || !(0.0..=1.0).contains(&self.relative_error_tolerance)
            || self.relative_error_tolerance == 0.0
        {
            return Err(ConfigError::InvalidAdaptiveTimeStep);
        }
        Ok(())
    }
}

/// Shared task batching settings used by runtime executors.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SchedulerConfig {
    pub min_batch_size: usize,
    pub max_batches_per_worker: usize,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            min_batch_size: 128,
            max_batches_per_worker: 4,
        }
    }
}

impl SchedulerConfig {
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.min_batch_size == 0 {
            return Err(ConfigError::ZeroBatchSize);
        }
        if self.max_batches_per_worker == 0 {
            return Err(ConfigError::ZeroBatchesPerWorker);
        }
        Ok(())
    }
}

/// Invalid fixed-step or batching configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    InvalidTickRate,
    ZeroCatchUpSteps,
    InvalidAdaptiveTimeStep,
    ZeroBatchSize,
    ZeroBatchesPerWorker,
    InvalidWorkerConfig(WorkerConfigError),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidTickRate => formatter.write_str("tick_rate must be in 1..=1,000,000,000"),
            Self::ZeroCatchUpSteps => formatter.write_str("max_catch_up_steps must be positive"),
            Self::InvalidAdaptiveTimeStep => formatter.write_str(
                "adaptive timestep requires 0 < minimum_step <= maximum_step and relative_error_tolerance in (0, 1]",
            ),
            Self::ZeroBatchSize => formatter.write_str("min_batch_size must be positive"),
            Self::ZeroBatchesPerWorker => {
                formatter.write_str("max_batches_per_worker must be positive")
            }
            Self::InvalidWorkerConfig(error) => write!(formatter, "worker config: {error}"),
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidWorkerConfig(error) => Some(error),
            _ => None,
        }
    }
}

impl From<WorkerConfigError> for ConfigError {
    fn from(error: WorkerConfigError) -> Self {
        Self::InvalidWorkerConfig(error)
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{
        AdaptiveTimeStepConfig, ConfigError, SchedulerConfig, SimulationConfig, TimeStepPolicy,
    };

    #[test]
    fn defaults_use_a_fixed_sixty_hertz_step() {
        let config = SimulationConfig::default();

        assert_eq!(config.tick_rate, 60);
        assert_eq!(config.max_catch_up_steps, 4);
        assert_eq!(config.time_step_policy, TimeStepPolicy::Fixed);
        assert_eq!(config.scheduler.min_batch_size, 128);
        assert_eq!(config.scheduler.max_batches_per_worker, 4);
        assert!((config.fixed_step_seconds() - 1.0 / 60.0).abs() < f64::EPSILON);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn rejects_zero_and_unrepresentable_tick_rates() {
        for tick_rate in [0, u32::MAX] {
            let config = SimulationConfig {
                tick_rate,
                ..SimulationConfig::default()
            };
            assert_eq!(config.validate(), Err(ConfigError::InvalidTickRate));
        }
    }

    #[test]
    fn rejects_zero_catch_up_and_batch_limits() {
        let config = SimulationConfig {
            max_catch_up_steps: 0,
            ..SimulationConfig::default()
        };
        assert_eq!(config.validate(), Err(ConfigError::ZeroCatchUpSteps));

        let config = SimulationConfig {
            scheduler: SchedulerConfig {
                min_batch_size: 0,
                ..SchedulerConfig::default()
            },
            ..SimulationConfig::default()
        };
        assert_eq!(config.validate(), Err(ConfigError::ZeroBatchSize));

        let config = SimulationConfig {
            scheduler: SchedulerConfig {
                max_batches_per_worker: 0,
                ..SchedulerConfig::default()
            },
            ..SimulationConfig::default()
        };
        assert_eq!(config.validate(), Err(ConfigError::ZeroBatchesPerWorker));
    }

    #[test]
    fn validates_adaptive_timestep_bounds() {
        let config = SimulationConfig {
            time_step_policy: TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default()),
            ..SimulationConfig::default()
        };
        assert!(config.validate().is_ok());

        for adaptive in [
            AdaptiveTimeStepConfig {
                minimum_step: Duration::ZERO,
                ..AdaptiveTimeStepConfig::default()
            },
            AdaptiveTimeStepConfig {
                maximum_step: Duration::from_nanos(1),
                ..AdaptiveTimeStepConfig::default()
            },
            AdaptiveTimeStepConfig {
                relative_error_tolerance: 0.0,
                ..AdaptiveTimeStepConfig::default()
            },
            AdaptiveTimeStepConfig {
                relative_error_tolerance: f64::NAN,
                ..AdaptiveTimeStepConfig::default()
            },
        ] {
            let config = SimulationConfig {
                time_step_policy: TimeStepPolicy::Adaptive(adaptive),
                ..SimulationConfig::default()
            };
            assert_eq!(config.validate(), Err(ConfigError::InvalidAdaptiveTimeStep));
        }
    }
}
