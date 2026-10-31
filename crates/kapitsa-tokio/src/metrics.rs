use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

use crate::AdaptiveStatus;

const METRICS_WINDOW: usize = 120;

/// Timing data for one completed physics step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepMeasurement {
    pub step_duration: Duration,
    pub task_wait_duration: Duration,
    pub step_budget: Duration,
    pub deadline: Duration,
}

/// Lifecycle state reflected in a simulation metrics snapshot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SimulationState {
    #[default]
    Stopped,
    Running,
    Error,
}

/// Ready-to-read status and rolling measurements for a physics simulation.
#[derive(Clone, Debug, PartialEq)]
pub struct SimulationMetrics {
    pub state: SimulationState,
    pub target_hz: u32,
    pub current_hz: u32,
    pub actual_hz: f64,
    pub step_duration: Duration,
    pub average_step_duration: Duration,
    pub max_step_duration: Duration,
    pub step_budget: Duration,
    pub budget_usage: f64,
    pub active_workers: usize,
    pub total_steps: u64,
    pub budget_overruns: u64,
    pub deadline_misses: u64,
    pub overload_integral: f64,
    pub simulation_time_debt: Duration,
}

impl SimulationMetrics {
    pub(crate) fn new(
        target_hz: u32,
        current_hz: u32,
        active_workers: usize,
        step_budget: Duration,
    ) -> Self {
        Self {
            state: SimulationState::Stopped,
            target_hz,
            current_hz,
            actual_hz: 0.0,
            step_duration: Duration::ZERO,
            average_step_duration: Duration::ZERO,
            max_step_duration: Duration::ZERO,
            step_budget,
            budget_usage: 0.0,
            active_workers,
            total_steps: 0,
            budget_overruns: 0,
            deadline_misses: 0,
            overload_integral: 0.0,
            simulation_time_debt: Duration::ZERO,
        }
    }

    pub(crate) fn record_step(
        &mut self,
        measurement: StepMeasurement,
        adaptive: AdaptiveStatus,
        completed_at: Instant,
        step_durations: &mut VecDeque<Duration>,
        completion_times: &mut VecDeque<Instant>,
    ) -> (bool, bool) {
        self.total_steps = self.total_steps.saturating_add(1);
        self.step_duration = measurement.step_duration;
        self.step_budget = measurement.step_budget;
        self.budget_usage = budget_usage(measurement.step_duration, measurement.step_budget);
        let budget_overrun = measurement.step_duration > measurement.step_budget;
        let deadline_missed = measurement.step_duration > measurement.deadline;
        if budget_overrun {
            self.budget_overruns = self.budget_overruns.saturating_add(1);
        }
        if deadline_missed {
            self.deadline_misses = self.deadline_misses.saturating_add(1);
        }
        self.current_hz = adaptive.current_hz;
        self.active_workers = adaptive.active_workers;
        self.overload_integral = adaptive.overload_integral;
        if self.state == SimulationState::Error {
            self.state = SimulationState::Stopped;
        }

        push_window(step_durations, measurement.step_duration);
        self.average_step_duration = average_duration(step_durations);
        self.max_step_duration = step_durations
            .iter()
            .copied()
            .max()
            .unwrap_or(Duration::ZERO);

        push_window(completion_times, completed_at);
        self.actual_hz = actual_frequency(completion_times);
        (budget_overrun, deadline_missed)
    }

    pub(crate) fn update_scheduler_state(
        &mut self,
        current_hz: u32,
        active_workers: usize,
        step_budget: Duration,
        overload_integral: f64,
    ) {
        self.current_hz = current_hz;
        self.active_workers = active_workers;
        self.step_budget = step_budget;
        self.overload_integral = overload_integral;
    }

    pub(crate) fn record_error(&mut self) {
        self.state = SimulationState::Error;
    }
}

/// Statistics for one completed call to [`crate::PhysicsController::step`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepStats {
    pub step_duration: Duration,
    pub task_wait_duration: Duration,
    pub step_budget: Duration,
    pub deadline: Duration,
    pub budget_usage: f64,
    pub budget_overrun: bool,
    pub deadline_missed: bool,
    pub overloaded: bool,
    pub current_hz: u32,
    pub active_workers: usize,
    pub average_step_duration: Duration,
    pub max_step_duration: Duration,
}

impl StepStats {
    pub(crate) fn snapshot(
        measurement: StepMeasurement,
        budget_overrun: bool,
        deadline_missed: bool,
        adaptive: AdaptiveStatus,
        metrics: &SimulationMetrics,
    ) -> Self {
        Self {
            step_duration: measurement.step_duration,
            task_wait_duration: measurement.task_wait_duration,
            step_budget: measurement.step_budget,
            deadline: measurement.deadline,
            budget_usage: metrics.budget_usage,
            budget_overrun,
            deadline_missed,
            overloaded: adaptive.overloaded,
            current_hz: metrics.current_hz,
            active_workers: metrics.active_workers,
            average_step_duration: metrics.average_step_duration,
            max_step_duration: metrics.max_step_duration,
        }
    }
}

pub(crate) struct MetricsCollector {
    snapshot: SimulationMetrics,
    step_durations: VecDeque<Duration>,
    completion_times: VecDeque<Instant>,
}

impl MetricsCollector {
    pub(crate) fn new(
        target_hz: u32,
        current_hz: u32,
        active_workers: usize,
        step_budget: Duration,
    ) -> Self {
        Self {
            snapshot: SimulationMetrics::new(target_hz, current_hz, active_workers, step_budget),
            step_durations: VecDeque::with_capacity(METRICS_WINDOW),
            completion_times: VecDeque::with_capacity(METRICS_WINDOW),
        }
    }

    pub(crate) fn snapshot(&self) -> SimulationMetrics {
        self.snapshot.clone()
    }

    pub(crate) fn record_step(
        &mut self,
        measurement: StepMeasurement,
        adaptive: AdaptiveStatus,
        completed_at: Instant,
    ) -> (bool, bool) {
        self.snapshot.record_step(
            measurement,
            adaptive,
            completed_at,
            &mut self.step_durations,
            &mut self.completion_times,
        )
    }

    pub(crate) fn update_scheduler_state(
        &mut self,
        current_hz: u32,
        active_workers: usize,
        step_budget: Duration,
        overload_integral: f64,
    ) {
        self.snapshot.update_scheduler_state(
            current_hz,
            active_workers,
            step_budget,
            overload_integral,
        );
    }

    pub(crate) fn record_error(&mut self) {
        self.snapshot.record_error();
    }
}

fn budget_usage(duration: Duration, budget: Duration) -> f64 {
    match (duration.is_zero(), budget.is_zero()) {
        (true, _) => 0.0,
        (false, true) => f64::MAX,
        (false, false) => duration.as_secs_f64() / budget.as_secs_f64(),
    }
}

fn push_window<T>(window: &mut VecDeque<T>, value: T) {
    if window.len() == METRICS_WINDOW {
        window.pop_front();
    }
    window.push_back(value);
}

fn average_duration(durations: &VecDeque<Duration>) -> Duration {
    if durations.is_empty() {
        return Duration::ZERO;
    }

    let seconds = durations.iter().map(Duration::as_secs_f64).sum::<f64>() / durations.len() as f64;
    if seconds >= u64::MAX as f64 {
        Duration::MAX
    } else {
        Duration::from_secs_f64(seconds)
    }
}

fn actual_frequency(completions: &VecDeque<Instant>) -> f64 {
    let Some(first) = completions.front() else {
        return 0.0;
    };
    let Some(last) = completions.back() else {
        return 0.0;
    };
    let span = last.duration_since(*first).as_secs_f64();
    if span == 0.0 || completions.len() < 2 {
        0.0
    } else {
        (completions.len() - 1) as f64 / span
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use crate::{AdaptiveStatus, SimulationState};

    use super::{MetricsCollector, StepMeasurement};

    #[test]
    fn records_budgets_deadlines_actual_rate_and_rolling_durations() {
        let mut metrics = MetricsCollector::new(60, 60, 1, Duration::from_millis(12));
        let status = AdaptiveStatus {
            current_hz: 60,
            active_workers: 1,
            overloaded: false,
            overload_integral: 0.0,
        };
        let start = Instant::now();
        let (budget_overrun, deadline_missed) = metrics.record_step(
            StepMeasurement {
                step_duration: Duration::from_millis(15),
                task_wait_duration: Duration::from_millis(10),
                step_budget: Duration::from_millis(12),
                deadline: Duration::from_millis(17),
            },
            status,
            start,
        );
        assert!(budget_overrun);
        assert!(!deadline_missed);

        let (budget_overrun, deadline_missed) = metrics.record_step(
            StepMeasurement {
                step_duration: Duration::from_millis(20),
                task_wait_duration: Duration::from_millis(11),
                step_budget: Duration::from_millis(12),
                deadline: Duration::from_millis(17),
            },
            AdaptiveStatus {
                overloaded: true,
                overload_integral: 1.0,
                ..status
            },
            start + Duration::from_millis(20),
        );
        assert!(budget_overrun);
        assert!(deadline_missed);

        let snapshot = metrics.snapshot();
        assert_eq!(snapshot.state, SimulationState::Stopped);
        assert_eq!(snapshot.target_hz, 60);
        assert_eq!(snapshot.current_hz, 60);
        assert_eq!(snapshot.actual_hz, 50.0);
        assert_eq!(snapshot.step_duration, Duration::from_millis(20));
        assert_eq!(
            snapshot.average_step_duration,
            Duration::from_micros(17_500)
        );
        assert_eq!(snapshot.max_step_duration, Duration::from_millis(20));
        assert_eq!(snapshot.step_budget, Duration::from_millis(12));
        assert!(snapshot.budget_usage > 1.6);
        assert_eq!(snapshot.total_steps, 2);
        assert_eq!(snapshot.budget_overruns, 2);
        assert_eq!(snapshot.deadline_misses, 1);
        assert_eq!(snapshot.overload_integral, 1.0);
        assert_eq!(snapshot.simulation_time_debt, Duration::ZERO);
    }
}
