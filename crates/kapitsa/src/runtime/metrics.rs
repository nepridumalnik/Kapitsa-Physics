use std::time::Duration;

pub use kapitsa_core::world::StepDiagnostics;

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

/// Resilient service progress independently of its lifecycle state.
#[derive(Clone, Debug, Default, PartialEq)]
pub enum RecoveryStatus {
    #[default]
    Healthy,
    Retrying {
        cause: kapitsa_core::Error,
        next_step: Duration,
        retry_count: usize,
    },
    Blocked(RecoveryBlock),
    Fatal(kapitsa_core::Error),
}

/// Why an autonomous resilient service cannot currently advance.
#[derive(Clone, Debug, PartialEq)]
pub enum RecoveryBlock {
    PhysicalStepLimit,
    RejectedStep(kapitsa_core::Error),
}

/// Snapshot of simulation timing, workload, and deadline metrics.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SimulationMetrics {
    pub state: SimulationState,
    pub recovery_status: RecoveryStatus,
    /// Number of integration attempts, including failed attempts and retries.
    pub total_attempts: u64,
    pub total_retries: u64,
    pub total_rejections: u64,
    pub consecutive_rejections: u64,
    /// Most recent failure, retained after a later successful step.
    pub last_failure: Option<kapitsa_core::Error>,
    /// Most recently selected safe step and actual attempted duration.
    pub last_safe_step: Duration,
    pub last_attempted_step: Duration,
    pub committed_simulation_time: Duration,
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
    /// Requested simulated time discarded by admission limits or horizon reduction.
    pub discarded_simulation_demand: Duration,
    /// Whether discarded demand exceeded the representable duration range.
    pub discarded_demand_saturated: bool,
    /// True when adaptive precision bounds are below the configured minimum step.
    pub adaptive_step_limited: bool,
    /// Time multiplier requested through `Simulation::set_time_multiplier`.
    pub requested_time_multiplier: f64,
    /// Simulated seconds per wall second measured since the latest multiplier change.
    pub actual_time_multiplier: f64,
    /// Current load-induced admission factor, between zero and one while running.
    pub load_speed_factor: f64,
    /// Degradations recorded by the latest successfully committed physics step.
    pub step_diagnostics: StepDiagnostics,
}

impl SimulationMetrics {
    pub(crate) fn copy_runtime_diagnostics_from(&mut self, source: &Self) {
        self.total_attempts = source.total_attempts;
        self.total_retries = source.total_retries;
        self.total_rejections = source.total_rejections;
        self.consecutive_rejections = source.consecutive_rejections;
        self.last_failure = source.last_failure.clone();
        self.last_safe_step = source.last_safe_step;
        self.last_attempted_step = source.last_attempted_step;
        self.committed_simulation_time = source.committed_simulation_time;
        self.discarded_simulation_demand = source.discarded_simulation_demand;
        self.discarded_demand_saturated = source.discarded_demand_saturated;
    }
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
    /// Degradations recorded by this successfully committed physics step.
    pub step_diagnostics: StepDiagnostics,
}
