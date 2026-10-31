use std::time::Duration;

const STEP_WINDOW_CAPACITY: usize = 120;

/// Measurements for one completed synchronous physics step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepMeasurement {
    pub step_duration: Duration,
    pub task_wait_duration: Duration,
    pub step_budget: Duration,
}

/// Cumulative and rolling scheduler measurements.
#[derive(Clone, Debug)]
pub struct SchedulerMetrics {
    total_steps: u64,
    budget_overruns: u64,
    current_hz: u32,
    active_workers: usize,
    simulation_time_debt: Duration,
    last_step_duration: Duration,
    last_task_wait_duration: Duration,
    total_task_wait_duration: Duration,
    step_durations: std::collections::VecDeque<Duration>,
    rolling_average_step_duration: Duration,
    rolling_max_step_duration: Duration,
}

impl SchedulerMetrics {
    pub(crate) fn new(current_hz: u32, active_workers: usize) -> Self {
        Self {
            total_steps: 0,
            budget_overruns: 0,
            current_hz,
            active_workers,
            simulation_time_debt: Duration::ZERO,
            last_step_duration: Duration::ZERO,
            last_task_wait_duration: Duration::ZERO,
            total_task_wait_duration: Duration::ZERO,
            step_durations: std::collections::VecDeque::with_capacity(STEP_WINDOW_CAPACITY),
            rolling_average_step_duration: Duration::ZERO,
            rolling_max_step_duration: Duration::ZERO,
        }
    }

    pub(crate) fn record_step(&mut self, measurement: StepMeasurement) -> bool {
        self.total_steps = self.total_steps.saturating_add(1);
        self.last_step_duration = measurement.step_duration;
        self.last_task_wait_duration = measurement.task_wait_duration;
        self.total_task_wait_duration = self
            .total_task_wait_duration
            .saturating_add(measurement.task_wait_duration);
        let over_budget = measurement.step_duration > measurement.step_budget;
        if over_budget {
            self.budget_overruns = self.budget_overruns.saturating_add(1);
        }

        if self.step_durations.len() == STEP_WINDOW_CAPACITY {
            self.step_durations.pop_front();
        }
        self.step_durations.push_back(measurement.step_duration);
        self.rolling_average_step_duration = average(&self.step_durations);
        self.rolling_max_step_duration = self
            .step_durations
            .iter()
            .copied()
            .max()
            .unwrap_or(Duration::ZERO);
        over_budget
    }

    pub(crate) fn update_scheduler_state(
        &mut self,
        current_hz: u32,
        active_workers: usize,
        simulation_time_debt: Duration,
    ) {
        self.current_hz = current_hz;
        self.active_workers = active_workers;
        self.simulation_time_debt = simulation_time_debt;
    }

    pub fn total_steps(&self) -> u64 {
        self.total_steps
    }

    pub fn budget_overruns(&self) -> u64 {
        self.budget_overruns
    }

    pub fn current_hz(&self) -> u32 {
        self.current_hz
    }

    pub fn active_workers(&self) -> usize {
        self.active_workers
    }

    pub fn simulation_time_debt(&self) -> Duration {
        self.simulation_time_debt
    }

    pub fn last_step_duration(&self) -> Duration {
        self.last_step_duration
    }

    pub fn last_task_wait_duration(&self) -> Duration {
        self.last_task_wait_duration
    }

    pub fn total_task_wait_duration(&self) -> Duration {
        self.total_task_wait_duration
    }

    pub fn rolling_average_step_duration(&self) -> Duration {
        self.rolling_average_step_duration
    }

    pub fn rolling_max_step_duration(&self) -> Duration {
        self.rolling_max_step_duration
    }
}

/// Per-call result returned by [`crate::PhysicsController::advance`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AdvanceStats {
    pub steps_completed: usize,
    pub budget_overruns: usize,
    pub step_duration: Duration,
    pub task_wait_duration: Duration,
    pub current_hz: u32,
    pub active_workers: usize,
    pub simulation_time_debt: Duration,
    pub rolling_average_step_duration: Duration,
    pub rolling_max_step_duration: Duration,
}

impl AdvanceStats {
    pub(crate) fn snapshot(
        steps_completed: usize,
        budget_overruns: usize,
        step_duration: Duration,
        task_wait_duration: Duration,
        metrics: &SchedulerMetrics,
    ) -> Self {
        Self {
            steps_completed,
            budget_overruns,
            step_duration,
            task_wait_duration,
            current_hz: metrics.current_hz(),
            active_workers: metrics.active_workers(),
            simulation_time_debt: metrics.simulation_time_debt(),
            rolling_average_step_duration: metrics.rolling_average_step_duration(),
            rolling_max_step_duration: metrics.rolling_max_step_duration(),
        }
    }
}

fn average(durations: &std::collections::VecDeque<Duration>) -> Duration {
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

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{SchedulerMetrics, StepMeasurement};

    #[test]
    fn records_budget_overruns_and_rolling_step_statistics() {
        let mut metrics = SchedulerMetrics::new(60, 1);

        assert!(!metrics.record_step(StepMeasurement {
            step_duration: Duration::from_millis(5),
            task_wait_duration: Duration::from_millis(6),
            step_budget: Duration::from_millis(10),
        }));
        assert!(metrics.record_step(StepMeasurement {
            step_duration: Duration::from_millis(15),
            task_wait_duration: Duration::from_millis(16),
            step_budget: Duration::from_millis(10),
        }));

        assert_eq!(metrics.total_steps(), 2);
        assert_eq!(metrics.budget_overruns(), 1);
        assert_eq!(metrics.last_step_duration(), Duration::from_millis(15));
        assert_eq!(metrics.last_task_wait_duration(), Duration::from_millis(16));
        assert_eq!(
            metrics.total_task_wait_duration(),
            Duration::from_millis(22)
        );
        assert_eq!(
            metrics.rolling_average_step_duration(),
            Duration::from_millis(10)
        );
        assert_eq!(
            metrics.rolling_max_step_duration(),
            Duration::from_millis(15)
        );
    }
}
