use std::time::Duration;

use crate::SchedulerError;

const MAX_FREQUENCY_HZ: u32 = 1_000_000_000;

/// Configuration for fixed-step scheduling and scheduler measurements.
#[derive(Clone, Debug, PartialEq)]
pub struct SchedulerConfig {
    pub target_hz: u32,
    pub min_hz: u32,
    pub target_utilization: f64,
    pub overload_tolerance: f64,
    pub max_workers: usize,
    pub max_catch_up_steps: usize,
    pub adaptive: bool,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            target_hz: 60,
            min_hz: 30,
            target_utilization: 0.75,
            overload_tolerance: 0.10,
            max_workers: std::thread::available_parallelism()
                .map(usize::from)
                .unwrap_or(1),
            max_catch_up_steps: 4,
            adaptive: true,
        }
    }
}

impl SchedulerConfig {
    pub fn validate(&self) -> Result<(), SchedulerError> {
        if self.target_hz == 0 || self.target_hz > MAX_FREQUENCY_HZ {
            return Err(SchedulerError::InvalidConfig(
                "target_hz must be in 1..=1,000,000,000",
            ));
        }
        if self.min_hz == 0 || self.min_hz > self.target_hz {
            return Err(SchedulerError::InvalidConfig(
                "min_hz must be positive and no greater than target_hz",
            ));
        }
        if !self.target_utilization.is_finite()
            || self.target_utilization <= 0.0
            || self.target_utilization > 1.0
        {
            return Err(SchedulerError::InvalidConfig(
                "target_utilization must be finite and in (0, 1]",
            ));
        }
        if !self.overload_tolerance.is_finite() || self.overload_tolerance < 0.0 {
            return Err(SchedulerError::InvalidConfig(
                "overload_tolerance must be finite and non-negative",
            ));
        }
        if self.max_workers == 0 {
            return Err(SchedulerError::InvalidConfig(
                "max_workers must be positive",
            ));
        }
        if self.max_catch_up_steps == 0 {
            return Err(SchedulerError::InvalidConfig(
                "max_catch_up_steps must be positive",
            ));
        }
        Ok(())
    }

    pub(crate) fn step_duration(&self, frequency_hz: u32) -> Duration {
        Duration::from_secs_f64(1.0 / f64::from(frequency_hz))
    }
}
