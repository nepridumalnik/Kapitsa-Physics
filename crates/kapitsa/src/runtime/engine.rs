use std::collections::VecDeque;
use std::time::{Duration, Instant};

use kapitsa_core::{
    execution::StepExecutor,
    world::{PhysicsWorld, StepError},
};

use super::{
    config::{ConfigError, SimulationConfig},
    metrics::{SimulationMetrics, SimulationState, StepStats},
};

const METRICS_WINDOW: usize = 120;

pub(crate) struct StepEngine<E: StepExecutor> {
    world: PhysicsWorld,
    executor: E,
    config: SimulationConfig,
    active_workers: usize,
    rayon_pool_size: Option<usize>,
    recent_step_durations: VecDeque<Duration>,
    metrics: SimulationMetrics,
}

impl<E: StepExecutor> StepEngine<E> {
    pub(crate) fn new(
        world: PhysicsWorld,
        config: SimulationConfig,
        executor: E,
    ) -> Result<Self, ConfigError> {
        config.validate()?;
        let step_budget = config.step_budget();
        let mut metrics = SimulationMetrics::default();
        metrics.step_budget = step_budget;
        metrics.active_workers = 1;
        Ok(Self {
            world,
            executor,
            config,
            active_workers: 1,
            rayon_pool_size: None,
            recent_step_durations: VecDeque::with_capacity(METRICS_WINDOW),
            metrics,
        })
    }

    pub(crate) fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    pub(crate) fn world_mut(&mut self) -> &mut PhysicsWorld {
        &mut self.world
    }

    pub(crate) fn config(&self) -> &SimulationConfig {
        &self.config
    }

    pub(crate) fn metrics(&self) -> &SimulationMetrics {
        &self.metrics
    }

    pub(crate) fn step(&mut self, task_wait_duration: Duration) -> Result<StepStats, StepError> {
        let started = Instant::now();
        let result = self.world.step_with_executor(
            self.config.fixed_step_seconds(),
            &self.executor,
            self.active_workers,
        );
        let step_duration = started.elapsed();
        if let Err(error) = result {
            self.metrics.state = SimulationState::Error;
            return Err(error);
        }

        if self.recent_step_durations.len() == METRICS_WINDOW {
            self.recent_step_durations.pop_front();
        }
        self.recent_step_durations.push_back(step_duration);

        let average_step_duration = average_duration(&self.recent_step_durations);
        let max_step_duration = self
            .recent_step_durations
            .iter()
            .copied()
            .max()
            .unwrap_or(Duration::ZERO);
        let step_budget = self.config.step_budget();
        let budget_usage = step_duration.as_secs_f64() / step_budget.as_secs_f64();
        let deadline_missed = step_duration > step_budget;

        self.metrics.state = SimulationState::Stopped;
        self.metrics.last_step_duration = step_duration;
        self.metrics.task_wait_duration = task_wait_duration;
        self.metrics.average_step_duration = average_step_duration;
        self.metrics.max_step_duration = max_step_duration;
        self.metrics.step_budget = step_budget;
        self.metrics.budget_usage = budget_usage;
        self.metrics.active_workers = self.active_workers;
        self.metrics.rayon_pool_size = self.rayon_pool_size;
        self.metrics.total_steps = self.metrics.total_steps.saturating_add(1);
        self.metrics.deadline_misses = self
            .metrics
            .deadline_misses
            .saturating_add(u64::from(deadline_missed));

        Ok(StepStats {
            step_duration,
            task_wait_duration,
            step_budget,
            budget_usage,
            deadline_missed,
            active_workers: self.active_workers,
            average_step_duration,
            max_step_duration,
        })
    }
}

fn average_duration(durations: &VecDeque<Duration>) -> Duration {
    let total = durations
        .iter()
        .copied()
        .fold(Duration::ZERO, Duration::saturating_add);
    if durations.is_empty() {
        Duration::ZERO
    } else {
        total.div_f64(durations.len() as f64)
    }
}
