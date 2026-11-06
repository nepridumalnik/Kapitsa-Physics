use std::time::Duration;

/// Measurements collected for one completed physics step.
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

/// Snapshot of simulation timing, workload, and deadline metrics.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SimulationMetrics {
    pub state: SimulationState,
    /// Simulation-time interval integrated by the latest completed physics step.
    pub last_physics_step: Duration,
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
    /// Accumulated simulation time beyond one configured base interval.
    pub simulation_time_debt: Duration,
    /// True when adaptive precision bounds are below the configured minimum step.
    pub adaptive_step_limited: bool,
    /// Time multiplier requested through `Simulation::set_time_multiplier`.
    pub requested_time_multiplier: f64,
    /// Simulated seconds per wall second measured since the latest multiplier change.
    pub actual_time_multiplier: f64,
}

/// Statistics for one completed manual `Simulation::step()` call.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepStats {
    pub physics_step: Duration,
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
