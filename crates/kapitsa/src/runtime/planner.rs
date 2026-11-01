use std::num::NonZeroUsize;
#[cfg(feature = "async")]
use std::time::Duration;

const EARLY_WARNING_AT: f64 = 0.75;
#[cfg(feature = "async")]
const MIN_SCALE_UP_STEPS: u32 = 3;
#[cfg(feature = "async")]
const MIN_SCALE_DOWN_STEPS: u32 = 24;
#[cfg(feature = "async")]
const STABILIZATION_STEPS: u32 = 12;
#[cfg(feature = "async")]
const WORKER_PROBE_STEPS: usize = 12;
#[cfg(feature = "async")]
const REQUIRED_SPEEDUP: f64 = 0.05;
#[cfg(feature = "async")]
const SIGNIFICANT_LOAD_INCREASE: f64 = 1.25;
#[cfg(feature = "async")]
const SCALE_UP_HYSTERESIS: f64 = 0.10;
#[cfg(feature = "async")]
const SCALE_DOWN_HYSTERESIS: f64 = 1.15;

/// Worker pool limits and adaptive thresholds.
#[derive(Clone, Debug, PartialEq)]
pub struct WorkerConfig {
    /// Fixed Rayon pool size; `None` uses the host's available parallelism.
    pub max_workers: Option<NonZeroUsize>,
    /// Smoothed step-time ratio that starts a scale-up probe. Defaults to `0.85`.
    pub scale_up_at: f64,
    /// Per-worker load ratio that permits scaling down. Defaults to `0.30`.
    pub scale_down_at: f64,
    /// EWMA weight for the latest completed step, in `(0, 1]`.
    pub smoothing: f64,
}

impl Default for WorkerConfig {
    fn default() -> Self {
        Self {
            max_workers: None,
            scale_up_at: 0.85,
            scale_down_at: 0.30,
            smoothing: 0.20,
        }
    }
}

impl WorkerConfig {
    pub fn validate(&self) -> Result<(), WorkerConfigError> {
        if !self.scale_up_at.is_finite()
            || self.scale_up_at <= EARLY_WARNING_AT
            || self.scale_up_at > 1.0
        {
            return Err(WorkerConfigError::InvalidScaleUpThreshold);
        }
        if !self.scale_down_at.is_finite()
            || self.scale_down_at <= 0.0
            || self.scale_down_at >= EARLY_WARNING_AT
            || self.scale_down_at >= self.scale_up_at
        {
            return Err(WorkerConfigError::InvalidScaleDownThreshold);
        }
        if !self.smoothing.is_finite() || self.smoothing <= 0.0 || self.smoothing > 1.0 {
            return Err(WorkerConfigError::InvalidSmoothing);
        }
        Ok(())
    }

    #[cfg(feature = "async")]
    pub(crate) fn resolve_max_workers(&self, available: usize) -> usize {
        self.max_workers
            .map(NonZeroUsize::get)
            .unwrap_or(available)
            .min(available)
            .max(1)
    }
}

/// Invalid async worker or scaling settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorkerConfigError {
    InvalidScaleUpThreshold,
    InvalidScaleDownThreshold,
    InvalidSmoothing,
    ActiveWorkersOutOfRange,
}

impl std::fmt::Display for WorkerConfigError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::InvalidScaleUpThreshold => "scale_up_at must be in (0.75, 1]",
            Self::InvalidScaleDownThreshold => "scale_down_at must be in (0, 0.75)",
            Self::InvalidSmoothing => "smoothing must be finite and in (0, 1]",
            Self::ActiveWorkersOutOfRange => "active workers must be in 1..=max_workers",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for WorkerConfigError {}

/// Current adaptive worker and step-budget status.
#[cfg(feature = "async")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdaptiveStatus {
    pub active_workers: usize,
    pub max_workers: usize,
    pub budget_warning: bool,
    pub overloaded: bool,
    pub smoothed_step_duration: Duration,
}

#[cfg(feature = "async")]
struct WorkerProbe {
    previous_workers: usize,
    candidate_workers: usize,
    baseline_seconds: f64,
    candidate_total_seconds: f64,
    candidate_samples: usize,
}

#[cfg(feature = "async")]
struct RejectedIncrease {
    candidate_workers: usize,
    baseline_seconds: f64,
}

/// Adapts active Rayon work while keeping the fixed pool size unchanged.
#[cfg(feature = "async")]
pub struct AdaptiveScheduler {
    config: WorkerConfig,
    max_workers: usize,
    active_workers: usize,
    smoothed_seconds: Option<f64>,
    scale_up_streak: u32,
    scale_down_streak: u32,
    stabilization_steps: u32,
    budget_warning: bool,
    overloaded: bool,
    worker_probe: Option<WorkerProbe>,
    rejected_increase: Option<RejectedIncrease>,
}

#[cfg(feature = "async")]
impl AdaptiveScheduler {
    #[cfg(test)]
    pub fn new(config: WorkerConfig, max_workers: usize) -> Result<Self, WorkerConfigError> {
        config.validate()?;
        if max_workers == 0 {
            return Err(WorkerConfigError::ActiveWorkersOutOfRange);
        }
        Ok(Self::with_valid_config(config, max_workers))
    }

    pub(crate) fn with_valid_config(config: WorkerConfig, max_workers: usize) -> Self {
        Self {
            config,
            max_workers,
            active_workers: 1,
            smoothed_seconds: None,
            scale_up_streak: 0,
            scale_down_streak: 0,
            stabilization_steps: 0,
            budget_warning: false,
            overloaded: false,
            worker_probe: None,
            rejected_increase: None,
        }
    }

    #[cfg(test)]
    pub fn active_workers(&self) -> usize {
        self.active_workers
    }

    pub fn observe(&mut self, step_duration: Duration, step_budget: Duration) -> AdaptiveStatus {
        let sample_seconds = step_duration.as_secs_f64();
        let smoothed_seconds = self.smoothed_seconds.map_or(sample_seconds, |previous| {
            previous + self.config.smoothing * (sample_seconds - previous)
        });
        self.smoothed_seconds = Some(smoothed_seconds);

        let budget_seconds = step_budget.as_secs_f64();
        self.budget_warning = step_duration.as_secs_f64() >= budget_seconds * EARLY_WARNING_AT;
        self.overloaded = smoothed_seconds >= budget_seconds * self.config.scale_up_at;

        if self.overloaded {
            self.scale_up_streak = self.scale_up_streak.saturating_add(1);
        } else if smoothed_seconds
            < budget_seconds * (self.config.scale_up_at - SCALE_UP_HYSTERESIS)
        {
            self.scale_up_streak = 0;
        }

        let average_worker_seconds = smoothed_seconds / self.active_workers as f64;
        if average_worker_seconds < budget_seconds * self.config.scale_down_at {
            self.scale_down_streak = self.scale_down_streak.saturating_add(1);
        } else if average_worker_seconds
            > budget_seconds * self.config.scale_down_at * SCALE_DOWN_HYSTERESIS
        {
            self.scale_down_streak = 0;
        }

        if self.worker_probe.is_some() {
            self.observe_worker_probe(sample_seconds);
            return self.status();
        }
        if self.stabilization_steps > 0 {
            self.stabilization_steps -= 1;
            return self.status();
        }

        if self.scale_up_streak >= MIN_SCALE_UP_STEPS && self.try_increase_workers() {
            return self.status();
        }
        if self.scale_down_streak >= MIN_SCALE_DOWN_STEPS
            && self.try_decrease_workers(budget_seconds)
        {
            return self.status();
        }
        self.status()
    }

    #[cfg(test)]
    pub fn set_active_workers(&mut self, workers: usize) -> Result<(), WorkerConfigError> {
        if workers == 0 || workers > self.max_workers {
            return Err(WorkerConfigError::ActiveWorkersOutOfRange);
        }
        self.active_workers = workers;
        self.scale_up_streak = 0;
        self.scale_down_streak = 0;
        self.stabilization_steps = STABILIZATION_STEPS;
        self.worker_probe = None;
        self.rejected_increase = None;
        Ok(())
    }

    fn observe_worker_probe(&mut self, sample_seconds: f64) {
        let Some(probe) = &mut self.worker_probe else {
            return;
        };
        probe.candidate_total_seconds += sample_seconds;
        probe.candidate_samples += 1;
        if probe.candidate_samples < WORKER_PROBE_STEPS {
            return;
        }

        let Some(probe) = self.worker_probe.take() else {
            return;
        };
        let candidate_seconds = probe.candidate_total_seconds / probe.candidate_samples as f64;
        if candidate_seconds <= probe.baseline_seconds * (1.0 - REQUIRED_SPEEDUP) {
            self.rejected_increase = None;
        } else {
            self.active_workers = probe.previous_workers;
            self.rejected_increase = Some(RejectedIncrease {
                candidate_workers: probe.candidate_workers,
                baseline_seconds: probe.baseline_seconds,
            });
        }
        self.finish_adjustment();
    }

    fn try_increase_workers(&mut self) -> bool {
        if self.active_workers >= self.max_workers {
            return false;
        }
        let candidate_workers = self.active_workers + 1;
        let baseline_seconds = self.smoothed_seconds.unwrap_or(0.0);
        if self.rejected_increase.as_ref().is_some_and(|rejected| {
            rejected.candidate_workers == candidate_workers
                && baseline_seconds <= rejected.baseline_seconds * SIGNIFICANT_LOAD_INCREASE
        }) {
            self.scale_up_streak = 0;
            return false;
        }

        self.rejected_increase = None;
        self.worker_probe = Some(WorkerProbe {
            previous_workers: self.active_workers,
            candidate_workers,
            baseline_seconds,
            candidate_total_seconds: 0.0,
            candidate_samples: 0,
        });
        self.active_workers = candidate_workers;
        self.scale_up_streak = 0;
        self.scale_down_streak = 0;
        true
    }

    fn try_decrease_workers(&mut self, budget_seconds: f64) -> bool {
        if self.active_workers <= 1 {
            return false;
        }
        let smoothed_seconds = self.smoothed_seconds.unwrap_or(0.0);
        let estimated_seconds =
            smoothed_seconds * self.active_workers as f64 / (self.active_workers - 1) as f64;
        if estimated_seconds >= budget_seconds {
            self.scale_down_streak = 0;
            return false;
        }

        self.active_workers -= 1;
        self.finish_adjustment();
        true
    }

    fn finish_adjustment(&mut self) {
        self.stabilization_steps = STABILIZATION_STEPS;
        self.scale_up_streak = 0;
        self.scale_down_streak = 0;
    }

    fn status(&self) -> AdaptiveStatus {
        AdaptiveStatus {
            active_workers: self.active_workers,
            max_workers: self.max_workers,
            budget_warning: self.budget_warning,
            overloaded: self.overloaded,
            smoothed_step_duration: Duration::from_secs_f64(self.smoothed_seconds.unwrap_or(0.0)),
        }
    }
}

#[cfg(all(test, feature = "async"))]
mod tests {
    use std::time::Duration;

    use super::{
        AdaptiveScheduler, EARLY_WARNING_AT, WORKER_PROBE_STEPS, WorkerConfig, WorkerConfigError,
    };

    fn scheduler(max_workers: usize) -> AdaptiveScheduler {
        AdaptiveScheduler::new(WorkerConfig::default(), max_workers).expect("valid scheduler")
    }

    fn observe_many(
        scheduler: &mut AdaptiveScheduler,
        duration: Duration,
        steps: usize,
    ) -> super::AdaptiveStatus {
        let budget = Duration::from_nanos(16_666_667);
        let mut status = scheduler.observe(duration, budget);
        for _ in 1..steps {
            status = scheduler.observe(duration, budget);
        }
        status
    }

    #[test]
    fn thresholds_warn_at_seventy_five_percent_and_scale_up_after_sustained_load() {
        let budget = Duration::from_millis(20);
        let mut warning_scheduler = scheduler(4);
        let warning = warning_scheduler.observe(Duration::from_millis(15), budget);
        assert!(warning.budget_warning);
        assert_eq!(warning.active_workers, 1);

        let mut scheduler = scheduler(4);
        observe_many(&mut scheduler, Duration::from_millis(18), 3);
        let status = scheduler.observe(Duration::from_millis(18), budget);
        assert!(status.overloaded);
        assert_eq!(status.active_workers, 2);
    }

    #[test]
    fn a_single_overload_spike_does_not_change_worker_count() {
        let mut scheduler = scheduler(4);
        let status = scheduler.observe(Duration::from_millis(40), Duration::from_nanos(16_666_667));
        assert!(status.overloaded);
        assert_eq!(status.active_workers, 1);
    }

    #[test]
    fn alternating_load_near_scale_up_threshold_does_not_trigger_a_probe() {
        let mut scheduler = scheduler(4);
        let budget = Duration::from_nanos(16_666_667);
        let lower = budget.mul_f64(0.84);
        let upper = budget.mul_f64(0.86);

        for index in 0..120 {
            scheduler.observe(if index % 2 == 0 { lower } else { upper }, budget);
        }

        assert_eq!(scheduler.active_workers(), 1);
    }

    #[test]
    fn useful_worker_probe_is_retained_and_does_not_double_worker_count() {
        let mut scheduler = scheduler(4);
        observe_many(&mut scheduler, Duration::from_millis(15), 3);
        assert_eq!(scheduler.active_workers(), 2);

        observe_many(&mut scheduler, Duration::from_millis(7), WORKER_PROBE_STEPS);
        assert_eq!(scheduler.active_workers(), 2);
    }

    #[test]
    fn ineffective_worker_probe_rolls_back_and_is_not_repeated() {
        let mut scheduler = scheduler(4);
        observe_many(&mut scheduler, Duration::from_millis(15), 3);
        assert_eq!(scheduler.active_workers(), 2);
        observe_many(
            &mut scheduler,
            Duration::from_millis(15),
            WORKER_PROBE_STEPS,
        );
        assert_eq!(scheduler.active_workers(), 1);
        observe_many(&mut scheduler, Duration::from_millis(15), 40);
        assert_eq!(scheduler.active_workers(), 1);
    }

    #[test]
    fn sustained_low_load_reduces_one_worker_without_changing_pool_size() {
        let mut scheduler = scheduler(4);
        scheduler
            .set_active_workers(4)
            .expect("valid active workers");
        let status = observe_many(&mut scheduler, Duration::from_millis(2), 40);
        assert_eq!(status.active_workers, 3);
        assert_eq!(status.max_workers, 4);
    }

    #[test]
    fn refuses_to_reduce_workers_when_remaining_load_would_miss_budget() {
        let mut scheduler = scheduler(4);
        scheduler
            .set_active_workers(4)
            .expect("valid active workers");
        let status = observe_many(&mut scheduler, Duration::from_millis(20), 64);
        assert_eq!(status.active_workers, 4);
    }

    #[test]
    fn rejects_invalid_thresholds_and_smoothing() {
        assert_eq!(
            WorkerConfig {
                max_workers: None,
                scale_up_at: 0.75,
                scale_down_at: 0.30,
                smoothing: 0.20,
            }
            .validate(),
            Err(WorkerConfigError::InvalidScaleUpThreshold)
        );
        assert_eq!(
            WorkerConfig {
                max_workers: None,
                scale_up_at: 0.85,
                scale_down_at: 0.75,
                smoothing: 0.20,
            }
            .validate(),
            Err(WorkerConfigError::InvalidScaleDownThreshold)
        );
        assert_eq!(
            WorkerConfig {
                max_workers: None,
                scale_up_at: 0.85,
                scale_down_at: 0.30,
                smoothing: f64::NAN,
            }
            .validate(),
            Err(WorkerConfigError::InvalidSmoothing)
        );
    }

    #[test]
    fn early_warning_threshold_is_the_shared_budget_policy() {
        assert_eq!(EARLY_WARNING_AT, 0.75);
    }
}
