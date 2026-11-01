use std::time::Duration;

/// Measurements collected for one completed fixed physics step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StepMeasurement {
    pub step_duration: Duration,
    pub task_wait_duration: Duration,
}

/// Lifecycle state reported by a simulation runtime.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SimulationState {
    #[default]
    Stopped,
    Running,
    Error,
}

/// Snapshot of fixed-step timing, workload, and deadline metrics.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SimulationMetrics {
    pub state: SimulationState,
    pub last_step_duration: Duration,
    pub task_wait_duration: Duration,
    pub average_step_duration: Duration,
    pub max_step_duration: Duration,
    pub step_budget: Duration,
    pub budget_usage: f64,
    pub budget_warning: bool,
    pub overloaded: bool,
    pub active_workers: usize,
    pub rayon_pool_size: Option<usize>,
    pub total_steps: u64,
    pub deadline_misses: u64,
    pub simulation_time_debt: Duration,
}

/// Statistics for one completed manual `Simulation::step()` call.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepStats {
    pub step_duration: Duration,
    pub task_wait_duration: Duration,
    pub step_budget: Duration,
    pub budget_usage: f64,
    pub budget_warning: bool,
    pub overloaded: bool,
    pub deadline_missed: bool,
    pub active_workers: usize,
    pub average_step_duration: Duration,
    pub max_step_duration: Duration,
}
