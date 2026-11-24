use crate::runtime::constants::{
    DEFAULT_MAX_BATCHES_PER_WORKER, DEFAULT_MAX_CATCH_UP_STEPS, DEFAULT_MIN_BATCH_SIZE,
    DEFAULT_TICK_RATE, MAX_TICK_RATE,
};
use std::time::Duration;

use kapitsa_core::world::ErrorPolicy;

use super::planner::{WorkerConfig, WorkerConfigError};

/// Shared timestep and batching settings.
#[non_exhaustive]
#[derive(Clone, Debug, PartialEq)]
pub struct SimulationConfig {
    /// Policy for explicitly supported approximations during physics steps.
    pub(crate) error_policy: ErrorPolicy,
    pub(crate) tick_rate: u32,
    pub(crate) max_catch_up_steps: usize,
    pub(crate) time_step_policy: TimeStepPolicy,
    pub(crate) step_recovery_policy: StepRecoveryPolicy,
    pub(crate) scheduler: SchedulerConfig,
    pub(crate) workers: WorkerConfig,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            error_policy: ErrorPolicy::default(),
            tick_rate: DEFAULT_TICK_RATE,
            max_catch_up_steps: DEFAULT_MAX_CATCH_UP_STEPS,
            time_step_policy: TimeStepPolicy::Fixed,
            step_recovery_policy: StepRecoveryPolicy::FailFast,
            scheduler: SchedulerConfig::default(),
            workers: WorkerConfig::default(),
        }
    }
}

/// Builder for [`SimulationConfig`]. Defaults match [`SimulationConfig::default`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SimulationConfigBuilder {
    config: SimulationConfig,
}

impl SimulationConfigBuilder {
    pub fn error_policy(mut self, value: ErrorPolicy) -> Self {
        self.config.error_policy = value;
        self
    }

    /// Sets the base simulation frequency in Hz.
    #[must_use]
    pub fn frequency(mut self, value: u32) -> Self {
        self.config.tick_rate = value;
        self
    }

    #[must_use]
    pub fn tick_rate(mut self, value: u32) -> Self {
        self.config.tick_rate = value;
        self
    }

    pub fn max_catch_up_steps(mut self, value: usize) -> Self {
        self.config.max_catch_up_steps = value;
        self
    }

    pub fn time_step_policy(mut self, value: TimeStepPolicy) -> Self {
        self.config.time_step_policy = value;
        self
    }

    pub fn step_recovery_policy(mut self, value: StepRecoveryPolicy) -> Self {
        self.config.step_recovery_policy = value;
        self
    }

    pub fn scheduler(mut self, value: SchedulerConfig) -> Self {
        self.config.scheduler = value;
        self
    }

    pub fn workers(mut self, value: WorkerConfig) -> Self {
        self.config.workers = value;
        self
    }

    pub fn build(self) -> SimulationConfig {
        self.config
    }
}

impl SimulationConfig {
    pub fn builder() -> SimulationConfigBuilder {
        SimulationConfigBuilder::default()
    }

    pub fn error_policy(&self) -> ErrorPolicy {
        self.error_policy
    }

    /// Base simulation frequency in Hz.
    pub fn frequency(&self) -> u32 {
        self.tick_rate
    }

    pub fn tick_rate(&self) -> u32 {
        self.tick_rate
    }

    pub fn max_catch_up_steps(&self) -> usize {
        self.max_catch_up_steps
    }

    pub fn time_step_policy(&self) -> TimeStepPolicy {
        self.time_step_policy
    }

    pub fn step_recovery_policy(&self) -> StepRecoveryPolicy {
        self.step_recovery_policy
    }

    pub fn scheduler(&self) -> SchedulerConfig {
        self.scheduler.clone()
    }

    pub fn workers(&self) -> WorkerConfig {
        self.workers.clone()
    }

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
        if let StepRecoveryPolicy::Resilient(recovery) = self.step_recovery_policy {
            let TimeStepPolicy::Adaptive(adaptive) = self.time_step_policy else {
                return Err(ConfigError::InvalidRecoveryConfig);
            };
            if recovery.minimum_step.is_zero()
                || recovery.minimum_step > adaptive.minimum_step
                || recovery.max_retries == 0
                || recovery.max_simulation_debt < self.step_budget()
                || recovery.worker_retry_backoff.is_zero()
            {
                return Err(ConfigError::InvalidRecoveryConfig);
            }
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

/// Recovery mode for the autonomous service; synchronous updates remain single attempts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub enum StepRecoveryPolicy {
    #[default]
    FailFast,
    Resilient(RecoveryConfig),
}

/// Resource bounds for opt-in adaptive recovery.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct RecoveryConfig {
    pub(crate) minimum_step: Duration,
    pub(crate) max_retries: usize,
    pub(crate) max_simulation_debt: Duration,
    pub(crate) worker_retry_backoff: Duration,
}

impl Default for RecoveryConfig {
    fn default() -> Self {
        Self {
            minimum_step: Duration::from_nanos(1),
            max_retries: 8,
            max_simulation_debt: Duration::from_secs(1),
            worker_retry_backoff: Duration::from_millis(10),
        }
    }
}

/// Builds recovery resource bounds; the containing simulation validates them before spawn.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RecoveryConfigBuilder {
    config: RecoveryConfig,
}

impl RecoveryConfigBuilder {
    pub fn minimum_step(mut self, value: Duration) -> Self {
        self.config.minimum_step = value;
        self
    }
    pub fn max_retries(mut self, value: usize) -> Self {
        self.config.max_retries = value;
        self
    }
    pub fn max_simulation_debt(mut self, value: Duration) -> Self {
        self.config.max_simulation_debt = value;
        self
    }
    pub fn worker_retry_backoff(mut self, value: Duration) -> Self {
        self.config.worker_retry_backoff = value;
        self
    }
    pub fn build(self) -> RecoveryConfig {
        self.config
    }
}

impl RecoveryConfig {
    pub fn builder() -> RecoveryConfigBuilder {
        RecoveryConfigBuilder::default()
    }
    pub fn minimum_step(&self) -> Duration {
        self.minimum_step
    }
    pub fn max_retries(&self) -> usize {
        self.max_retries
    }
    pub fn max_simulation_debt(&self) -> Duration {
        self.max_simulation_debt
    }
    pub fn worker_retry_backoff(&self) -> Duration {
        self.worker_retry_backoff
    }
}

/// Selects fixed-step compatibility or physics-bounded adaptive stepping.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub enum TimeStepPolicy {
    #[default]
    Fixed,
    Adaptive(AdaptiveTimeStepConfig),
}

/// Accuracy and safety bounds used by the adaptive Time Warp policy.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdaptiveTimeStepConfig {
    /// Refuse a smaller step and report the requested warp as unattainable.
    pub(crate) minimum_step: Duration,
    /// Absolute safety cap; physics estimates can choose a smaller step.
    pub(crate) maximum_step: Duration,
    /// Dimensionless safety factor for motion, acceleration, and joint bounds, in `(0, 1]`.
    pub(crate) relative_error_tolerance: f64,
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

/// Builder for [`AdaptiveTimeStepConfig`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveTimeStepConfigBuilder {
    config: AdaptiveTimeStepConfig,
}

impl AdaptiveTimeStepConfigBuilder {
    pub fn minimum_step(mut self, value: Duration) -> Self {
        self.config.minimum_step = value;
        self
    }

    pub fn maximum_step(mut self, value: Duration) -> Self {
        self.config.maximum_step = value;
        self
    }

    pub fn relative_error_tolerance(mut self, value: f64) -> Self {
        self.config.relative_error_tolerance = value;
        self
    }

    pub fn build(self) -> AdaptiveTimeStepConfig {
        self.config
    }
}

impl AdaptiveTimeStepConfig {
    pub fn builder() -> AdaptiveTimeStepConfigBuilder {
        AdaptiveTimeStepConfigBuilder::default()
    }

    pub fn minimum_step(&self) -> Duration {
        self.minimum_step
    }

    pub fn maximum_step(&self) -> Duration {
        self.maximum_step
    }

    pub fn relative_error_tolerance(&self) -> f64 {
        self.relative_error_tolerance
    }

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
#[non_exhaustive]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SchedulerConfig {
    pub(crate) min_batch_size: usize,
    pub(crate) max_batches_per_worker: usize,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            min_batch_size: DEFAULT_MIN_BATCH_SIZE,
            max_batches_per_worker: DEFAULT_MAX_BATCHES_PER_WORKER,
        }
    }
}

/// Builder for [`SchedulerConfig`].
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SchedulerConfigBuilder {
    config: SchedulerConfig,
}

impl SchedulerConfigBuilder {
    pub fn min_batch_size(mut self, value: usize) -> Self {
        self.config.min_batch_size = value;
        self
    }

    pub fn max_batches_per_worker(mut self, value: usize) -> Self {
        self.config.max_batches_per_worker = value;
        self
    }

    pub fn build(self) -> SchedulerConfig {
        self.config
    }
}

impl SchedulerConfig {
    pub fn builder() -> SchedulerConfigBuilder {
        SchedulerConfigBuilder::default()
    }

    pub fn min_batch_size(&self) -> usize {
        self.min_batch_size
    }

    pub fn max_batches_per_worker(&self) -> usize {
        self.max_batches_per_worker
    }

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
    InvalidRecoveryConfig,
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
            Self::InvalidRecoveryConfig => formatter.write_str("resilient recovery requires adaptive stepping, a valid step floor, positive retry and backoff bounds, and debt of at least one base step"),
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
        AdaptiveTimeStepConfig, ConfigError, RecoveryConfig, SchedulerConfig, SimulationConfig,
        StepRecoveryPolicy, TimeStepPolicy,
    };

    #[test]
    fn resilient_policy_requires_valid_adaptive_resource_bounds() {
        assert_eq!(
            SimulationConfig::default().step_recovery_policy(),
            StepRecoveryPolicy::FailFast
        );
        let adaptive = TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default());
        let valid = SimulationConfig::builder()
            .time_step_policy(adaptive)
            .step_recovery_policy(StepRecoveryPolicy::Resilient(RecoveryConfig::default()))
            .build();
        assert_eq!(valid.validate(), Ok(()));
        let fixed = SimulationConfig::builder()
            .step_recovery_policy(StepRecoveryPolicy::Resilient(RecoveryConfig::default()))
            .build();
        assert_eq!(fixed.validate(), Err(ConfigError::InvalidRecoveryConfig));
        let invalid = [
            RecoveryConfig::builder()
                .minimum_step(Duration::ZERO)
                .build(),
            RecoveryConfig::builder()
                .minimum_step(Duration::from_secs(1))
                .build(),
            RecoveryConfig::builder().max_retries(0).build(),
            RecoveryConfig::builder()
                .max_simulation_debt(Duration::ZERO)
                .build(),
            RecoveryConfig::builder()
                .worker_retry_backoff(Duration::ZERO)
                .build(),
        ];
        for recovery in invalid {
            let config = SimulationConfig::builder()
                .time_step_policy(adaptive)
                .step_recovery_policy(StepRecoveryPolicy::Resilient(recovery))
                .build();
            assert_eq!(config.validate(), Err(ConfigError::InvalidRecoveryConfig));
        }
    }

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
    fn frequency_alias_matches_tick_rate() {
        let config = SimulationConfig::builder().frequency(120).build();
        assert_eq!(config.frequency(), 120);
        assert_eq!(config.tick_rate(), 120);
        assert_eq!(config.frequency(), config.tick_rate());
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
