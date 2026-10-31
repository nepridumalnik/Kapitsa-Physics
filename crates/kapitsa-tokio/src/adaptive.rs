use std::collections::VecDeque;

use crate::{SchedulerConfig, SchedulerError, StepMeasurement};

const ERROR_MAX: f64 = 4.0;
const OVERLOAD_LIMIT: f64 = 2.2;
const CLEAR_LIMIT: f64 = 0.4;
const MIN_OVERLOAD_STEPS: u32 = 3;
const RECOVERY_STEPS: u32 = 24;
const STABILIZATION_STEPS: u32 = 12;
const WORKER_PROBE_STEPS: usize = 12;
const REQUIRED_SPEEDUP: f64 = 0.05;
const SIGNIFICANT_LOAD_INCREASE: f64 = 1.25;
const UNDERLOAD_RATIO: f64 = 0.55;
const UNDERLOAD_HYSTERESIS_RATIO: f64 = 0.65;
const AVERAGE_WINDOW: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdaptiveStatus {
    pub current_hz: u32,
    pub active_workers: usize,
    pub overloaded: bool,
    pub overload_integral: f64,
}

struct WorkerProbe {
    previous_workers: usize,
    candidate_workers: usize,
    baseline_seconds: f64,
    candidate_total_seconds: f64,
    candidate_samples: usize,
}

struct RejectedWorkerIncrease {
    candidate_workers: usize,
    baseline_seconds: f64,
}

/// Adjusts worker count and fixed-step frequency from measured step timings.
///
/// Feed measurements through [`Self::observe`] so regulation can be tested with
/// deterministic timing sequences.
pub struct AdaptiveScheduler {
    config: SchedulerConfig,
    current_hz: u32,
    active_workers: usize,
    overload_integral: f64,
    overload_streak: u32,
    underload_streak: u32,
    stabilization_steps: u32,
    overloaded: bool,
    recent_step_seconds: VecDeque<f64>,
    worker_probe: Option<WorkerProbe>,
    rejected_worker_increase: Option<RejectedWorkerIncrease>,
}

impl AdaptiveScheduler {
    pub fn new(config: SchedulerConfig) -> Result<Self, SchedulerError> {
        config.validate()?;
        Ok(Self::with_valid_config(config))
    }

    pub(crate) fn with_valid_config(config: SchedulerConfig) -> Self {
        Self {
            current_hz: config.target_hz,
            active_workers: 1,
            overload_integral: 0.0,
            overload_streak: 0,
            underload_streak: 0,
            stabilization_steps: 0,
            overloaded: false,
            recent_step_seconds: VecDeque::with_capacity(AVERAGE_WINDOW),
            worker_probe: None,
            rejected_worker_increase: None,
            config,
        }
    }

    pub fn current_hz(&self) -> u32 {
        self.current_hz
    }

    pub fn active_workers(&self) -> usize {
        self.active_workers
    }

    pub fn overload_integral(&self) -> f64 {
        self.overload_integral
    }

    pub fn observe(&mut self, measurement: StepMeasurement) -> AdaptiveStatus {
        let duration_seconds = measurement.step_duration.as_secs_f64();
        self.record_duration(duration_seconds);

        let budget_seconds = measurement.step_budget.as_secs_f64();
        let threshold_seconds = budget_seconds * (1.0 + self.config.overload_tolerance);
        let ratio = if threshold_seconds == 0.0 {
            if duration_seconds == 0.0 {
                1.0
            } else {
                f64::INFINITY
            }
        } else {
            duration_seconds / threshold_seconds
        };
        let overrun = (ratio - 1.0).clamp(0.0, 1.0);
        let recovery = (1.0 - ratio).max(0.0);
        self.overloaded = ratio > 1.0;
        self.overload_integral =
            (self.overload_integral + overrun - recovery).clamp(0.0, ERROR_MAX);

        if self.overloaded {
            self.overload_streak = self.overload_streak.saturating_add(1);
        } else if duration_seconds < threshold_seconds * 0.9 {
            self.overload_streak = 0;
        }

        if duration_seconds <= budget_seconds * UNDERLOAD_RATIO {
            self.underload_streak = self.underload_streak.saturating_add(1);
        } else if duration_seconds > budget_seconds * UNDERLOAD_HYSTERESIS_RATIO {
            self.underload_streak = 0;
        }

        if self.worker_probe.is_some() {
            self.observe_worker_probe(duration_seconds);
            return self.status();
        }
        if self.overload_integral < CLEAR_LIMIT {
            self.rejected_worker_increase = None;
        }
        if !self.config.adaptive {
            return self.status();
        }
        if self.stabilization_steps > 0 {
            self.stabilization_steps -= 1;
            return self.status();
        }

        if self.overload_streak >= MIN_OVERLOAD_STEPS && self.overload_integral >= OVERLOAD_LIMIT {
            if self.try_increase_workers() {
                return self.status();
            }
            self.decrease_frequency();
            return self.status();
        }

        if self.underload_streak >= RECOVERY_STEPS {
            if self.increase_frequency() {
                self.finish_adjustment();
            } else if self.active_workers > 1 {
                self.active_workers = self.active_workers.div_ceil(2);
                self.finish_adjustment();
            }
        }
        self.status()
    }

    pub(crate) fn set_active_workers(&mut self, workers: usize) {
        self.active_workers = workers;
        self.worker_probe = None;
        self.rejected_worker_increase = None;
        self.stabilization_steps = STABILIZATION_STEPS;
    }

    fn observe_worker_probe(&mut self, duration_seconds: f64) {
        let Some(probe) = &mut self.worker_probe else {
            return;
        };
        probe.candidate_total_seconds += duration_seconds;
        probe.candidate_samples += 1;
        if probe.candidate_samples < WORKER_PROBE_STEPS {
            return;
        }

        let Some(probe) = self.worker_probe.take() else {
            return;
        };
        let candidate_seconds = probe.candidate_total_seconds / probe.candidate_samples as f64;
        if candidate_seconds <= probe.baseline_seconds * (1.0 - REQUIRED_SPEEDUP) {
            self.rejected_worker_increase = None;
        } else {
            self.active_workers = probe.previous_workers;
            self.rejected_worker_increase = Some(RejectedWorkerIncrease {
                candidate_workers: probe.candidate_workers,
                baseline_seconds: probe.baseline_seconds,
            });
        }
        self.finish_adjustment();
    }

    fn try_increase_workers(&mut self) -> bool {
        if self.active_workers >= self.config.max_workers {
            return false;
        }
        let candidate_workers = self
            .active_workers
            .saturating_mul(2)
            .min(self.config.max_workers);
        let baseline_seconds = self.average_step_seconds();
        if let Some(rejected) = &self.rejected_worker_increase {
            let load_increased =
                baseline_seconds > rejected.baseline_seconds * SIGNIFICANT_LOAD_INCREASE;
            if rejected.candidate_workers == candidate_workers && !load_increased {
                return false;
            }
        }

        self.rejected_worker_increase = None;
        self.worker_probe = Some(WorkerProbe {
            previous_workers: self.active_workers,
            candidate_workers,
            baseline_seconds,
            candidate_total_seconds: 0.0,
            candidate_samples: 0,
        });
        self.active_workers = candidate_workers;
        self.underload_streak = 0;
        true
    }

    fn decrease_frequency(&mut self) {
        let levels = self.frequency_levels();
        if let Some(index) = levels
            .iter()
            .position(|frequency| *frequency == self.current_hz)
            && let Some(next) = levels.get(index + 1)
        {
            self.current_hz = *next;
            self.finish_adjustment();
        }
    }

    fn increase_frequency(&mut self) -> bool {
        let levels = self.frequency_levels();
        if let Some(index) = levels
            .iter()
            .position(|frequency| *frequency == self.current_hz)
            && index > 0
        {
            self.current_hz = levels[index - 1];
            return true;
        }
        false
    }

    fn frequency_levels(&self) -> Vec<u32> {
        let mut levels = vec![self.config.target_hz, 60, 50, 40, 30, self.config.min_hz];
        levels.sort_unstable_by(|lhs, rhs| rhs.cmp(lhs));
        levels.dedup();
        levels.retain(|frequency| {
            *frequency >= self.config.min_hz && *frequency <= self.config.target_hz
        });
        levels
    }

    fn record_duration(&mut self, duration_seconds: f64) {
        if self.recent_step_seconds.len() == AVERAGE_WINDOW {
            self.recent_step_seconds.pop_front();
        }
        self.recent_step_seconds.push_back(duration_seconds);
    }

    fn average_step_seconds(&self) -> f64 {
        if self.recent_step_seconds.is_empty() {
            return 0.0;
        }
        self.recent_step_seconds.iter().sum::<f64>() / self.recent_step_seconds.len() as f64
    }

    fn finish_adjustment(&mut self) {
        self.stabilization_steps = STABILIZATION_STEPS;
        self.overload_streak = 0;
        self.underload_streak = 0;
    }

    fn status(&self) -> AdaptiveStatus {
        AdaptiveStatus {
            current_hz: self.current_hz,
            active_workers: self.active_workers,
            overloaded: self.overloaded,
            overload_integral: self.overload_integral,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use crate::{SchedulerConfig, StepMeasurement};

    use super::AdaptiveScheduler;

    fn config(max_workers: usize) -> SchedulerConfig {
        SchedulerConfig {
            max_workers,
            ..SchedulerConfig::default()
        }
    }

    fn scheduler(max_workers: usize) -> AdaptiveScheduler {
        AdaptiveScheduler::new(config(max_workers)).expect("valid config")
    }

    fn measurement(scheduler: &AdaptiveScheduler, duration: Duration) -> StepMeasurement {
        let step = Duration::from_secs_f64(1.0 / f64::from(scheduler.current_hz()));
        StepMeasurement {
            step_duration: duration,
            task_wait_duration: duration,
            step_budget: step.mul_f64(0.75),
            deadline: step,
        }
    }

    fn observe(scheduler: &mut AdaptiveScheduler, duration: Duration) -> super::AdaptiveStatus {
        scheduler.observe(measurement(scheduler, duration))
    }

    fn feed_overload(scheduler: &mut AdaptiveScheduler, count: usize) {
        for _ in 0..count {
            observe(scheduler, Duration::from_millis(50));
        }
    }

    fn feed_underload(scheduler: &mut AdaptiveScheduler, count: usize) {
        for _ in 0..count {
            observe(scheduler, Duration::from_millis(1));
        }
    }

    #[test]
    fn stable_load_keeps_the_target_configuration() {
        let mut scheduler = scheduler(4);
        feed_underload(&mut scheduler, 64);

        assert_eq!(scheduler.current_hz(), 60);
        assert_eq!(scheduler.active_workers(), 1);
        assert_eq!(scheduler.overload_integral(), 0.0);
    }

    #[test]
    fn one_slow_step_does_not_change_the_configuration() {
        let mut scheduler = scheduler(4);
        observe(&mut scheduler, Duration::from_millis(100));
        feed_underload(&mut scheduler, 32);

        assert_eq!(scheduler.current_hz(), 60);
        assert_eq!(scheduler.active_workers(), 1);
        assert!(scheduler.overload_integral() < 0.1);
    }

    #[test]
    fn overload_integral_accumulates_overrun_and_recovers_by_ratio() {
        let mut scheduler = scheduler(1);
        let measurement = StepMeasurement {
            step_duration: Duration::from_millis(22),
            task_wait_duration: Duration::from_millis(22),
            step_budget: Duration::from_millis(10),
            deadline: Duration::from_millis(20),
        };

        assert!((scheduler.observe(measurement).overload_integral - 1.0).abs() < 1.0e-12);
        assert!((scheduler.observe(measurement).overload_integral - 2.0).abs() < 1.0e-12);
        assert!((scheduler.observe(measurement).overload_integral - 3.0).abs() < 1.0e-12);

        let recovery = StepMeasurement {
            step_duration: Duration::from_millis(5),
            task_wait_duration: Duration::from_millis(5),
            ..measurement
        };
        let status = scheduler.observe(recovery);
        assert!((status.overload_integral - (3.0 - (1.0 - 0.5 / 1.1))).abs() < 1.0e-12);
    }

    #[test]
    fn fixed_duration_regressions_cover_threshold_and_sustained_overload() {
        let mut no_overload = scheduler(4);
        let below = observe(&mut no_overload, Duration::from_millis(12));
        assert!(!below.overloaded);
        assert_eq!(below.overload_integral, 0.0);

        let mut boundary = scheduler(4);
        let exact = observe(&mut boundary, Duration::from_micros(13_750));
        assert!(!exact.overloaded);
        assert_eq!(exact.overload_integral, 0.0);

        for (duration_micros, max_steps) in [(15_000, 64), (16_667, 20)] {
            let mut scheduler = scheduler(4);
            for _ in 0..max_steps {
                observe(&mut scheduler, Duration::from_micros(duration_micros));
                if scheduler.active_workers() > 1 {
                    break;
                }
            }
            assert_eq!(
                scheduler.active_workers(),
                2,
                "duration={duration_micros} µs"
            );
            assert_eq!(scheduler.current_hz(), 60);
        }
    }

    #[test]
    fn one_50_ms_spike_does_not_reconfigure() {
        let mut scheduler = scheduler(4);
        let status = observe(&mut scheduler, Duration::from_millis(50));

        assert!(status.overloaded);
        assert_eq!(status.overload_integral, 1.0);
        assert_eq!(scheduler.active_workers(), 1);
        assert_eq!(scheduler.current_hz(), 60);
    }

    #[test]
    fn sustained_overload_probes_a_larger_worker_count() {
        let mut scheduler = scheduler(4);
        feed_overload(&mut scheduler, 3);

        assert_eq!(scheduler.active_workers(), 2);
        assert_eq!(scheduler.current_hz(), 60);
    }

    #[test]
    fn faster_worker_probe_is_kept_and_can_grow_again() {
        let mut scheduler = scheduler(4);
        feed_overload(&mut scheduler, 3);
        feed_underload(&mut scheduler, 12);
        assert_eq!(scheduler.active_workers(), 2);

        feed_overload(&mut scheduler, 16);
        assert_eq!(scheduler.active_workers(), 4);
    }

    #[test]
    fn ineffective_worker_probe_rolls_back_and_is_not_repeated() {
        let mut scheduler = scheduler(4);
        feed_overload(&mut scheduler, 3);
        feed_overload(&mut scheduler, 12);
        assert_eq!(scheduler.active_workers(), 1);

        feed_overload(&mut scheduler, 48);
        assert_eq!(scheduler.active_workers(), 1);
        assert!(scheduler.current_hz() < 60);
    }

    #[test]
    fn worker_growth_respects_the_configured_limit() {
        let mut scheduler = scheduler(2);
        feed_overload(&mut scheduler, 3);
        assert_eq!(scheduler.active_workers(), 2);

        feed_underload(&mut scheduler, 12);
        feed_overload(&mut scheduler, 64);
        assert_eq!(scheduler.active_workers(), 2);
        assert!(scheduler.current_hz() < 60);
    }

    #[test]
    fn sustained_overload_reduces_frequency_in_defined_levels() {
        let mut scheduler = scheduler(1);
        for expected in [50, 40, 30] {
            for _ in 0..64 {
                if scheduler.current_hz() == expected {
                    break;
                }
                observe(&mut scheduler, Duration::from_millis(50));
            }
            assert_eq!(scheduler.current_hz(), expected);
        }

        assert_eq!(scheduler.current_hz(), 30);
        assert_eq!(scheduler.active_workers(), 1);
    }

    #[test]
    fn recovery_restores_frequency_and_reduces_workers() {
        let mut scheduler = scheduler(4);
        feed_overload(&mut scheduler, 200);
        scheduler.set_active_workers(4);
        feed_underload(&mut scheduler, 300);

        assert_eq!(scheduler.current_hz(), 60);
        assert_eq!(scheduler.active_workers(), 1);
    }

    #[test]
    fn oscillating_load_near_the_threshold_does_not_reconfigure() {
        let mut scheduler = scheduler(4);
        for index in 0..120 {
            observe(
                &mut scheduler,
                Duration::from_micros(if index % 2 == 0 { 13_740 } else { 13_760 }),
            );
        }

        assert_eq!(scheduler.current_hz(), 60);
        assert_eq!(scheduler.active_workers(), 1);
    }

    #[test]
    fn minimum_frequency_still_reports_unresolved_overload() {
        let mut scheduler = scheduler(1);
        feed_overload(&mut scheduler, 200);
        let status = scheduler.observe(measurement(&scheduler, Duration::from_millis(50)));

        assert_eq!(status.current_hz, 30);
        assert!(status.overloaded);
        assert!(status.overload_integral.is_finite());
    }

    #[test]
    fn invalid_extreme_metrics_do_not_overflow_the_integral() {
        let mut scheduler = scheduler(4);
        let status = scheduler.observe(StepMeasurement {
            step_duration: Duration::MAX,
            task_wait_duration: Duration::MAX,
            step_budget: Duration::ZERO,
            deadline: Duration::from_millis(20),
        });

        assert!(status.overloaded);
        assert_eq!(status.overload_integral, 1.0);
        assert!(status.overload_integral.is_finite());
    }

    #[test]
    fn disabled_adaptation_keeps_worker_count_and_frequency_fixed() {
        let mut scheduler = AdaptiveScheduler::new(SchedulerConfig {
            max_workers: 4,
            adaptive: false,
            ..SchedulerConfig::default()
        })
        .expect("valid config");
        feed_overload(&mut scheduler, 200);

        assert_eq!(scheduler.current_hz(), 60);
        assert_eq!(scheduler.active_workers(), 1);
    }
}
