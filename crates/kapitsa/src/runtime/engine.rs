use std::collections::VecDeque;
use std::time::{Duration, Instant};

use kapitsa_core::{
    collision::CollisionError,
    execution::StepExecutor,
    rigid_body::BodyType,
    world::{BodyStep, PhysicsWorld, StepError},
};

#[cfg(test)]
use super::config::ConfigError;
use super::{
    config::SimulationConfig,
    metrics::{SimulationMetrics, SimulationState, StepStats},
};

const METRICS_WINDOW: usize = 120;

pub(crate) struct StepEngine<E: StepExecutor> {
    world: PhysicsWorld,
    executor: E,
    config: SimulationConfig,
    active_workers: usize,
    rayon_pool_size: Option<usize>,
    bucket_buffers: Vec<Vec<BodyStep>>,
    recent_step_durations: VecDeque<Duration>,
    metrics: SimulationMetrics,
}

impl<E: StepExecutor> StepEngine<E> {
    #[cfg(test)]
    pub(crate) fn new(
        world: PhysicsWorld,
        config: SimulationConfig,
        executor: E,
    ) -> Result<Self, ConfigError> {
        config.validate()?;
        Ok(Self::new_validated(world, config, executor))
    }

    pub(crate) fn new_validated(
        mut world: PhysicsWorld,
        config: SimulationConfig,
        executor: E,
    ) -> Self {
        if let Some(gravity) = config.central_gravity {
            world.set_central_gravity(Some(gravity));
        }
        let step_budget = config.step_budget();
        let metrics = SimulationMetrics {
            step_budget,
            active_workers: 1,
            requested_time_multiplier: 1.0,
            ..SimulationMetrics::default()
        };
        Self {
            world,
            executor,
            config,
            active_workers: 1,
            rayon_pool_size: None,
            bucket_buffers: Vec::new(),
            recent_step_durations: VecDeque::with_capacity(METRICS_WINDOW),
            metrics,
        }
    }

    pub(crate) fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    pub(crate) fn world_mut(&mut self) -> &mut PhysicsWorld {
        &mut self.world
    }

    pub(crate) fn metrics(&self) -> &SimulationMetrics {
        &self.metrics
    }

    pub(crate) fn set_parallelism(&mut self, active_workers: usize, pool_size: Option<usize>) {
        self.active_workers = active_workers.max(1);
        self.rayon_pool_size = pool_size;
        self.metrics.active_workers = self.active_workers;
        self.metrics.rayon_pool_size = pool_size;
    }

    pub(crate) fn record_task_wait(&mut self, stats: &mut StepStats, duration: Duration) {
        stats.task_wait_duration = duration;
        self.metrics.task_wait_duration = duration;
    }

    pub(crate) fn set_budget_status(&mut self, warning: bool, overloaded: bool) {
        self.metrics.budget_warning = warning;
        self.metrics.overloaded = overloaded;
    }

    pub(crate) fn set_runtime_status(
        &mut self,
        state: SimulationState,
        simulation_time_debt: Duration,
    ) {
        self.metrics.state = state;
        self.metrics.simulation_time_debt = simulation_time_debt;
    }

    #[cfg(test)]
    pub(crate) fn into_world(self) -> PhysicsWorld {
        self.world
    }

    pub(crate) fn step(&mut self, task_wait_duration: Duration) -> Result<StepStats, StepError> {
        let started = Instant::now();
        let result = self.run_step();
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
        let budget_warning = step_duration >= step_budget.mul_f64(0.75);
        let overloaded = step_duration >= step_budget.mul_f64(0.85);
        let deadline_missed = step_duration > step_budget;

        self.metrics.state = SimulationState::Stopped;
        self.metrics.last_step_duration = step_duration;
        self.metrics.task_wait_duration = task_wait_duration;
        self.metrics.average_step_duration = average_step_duration;
        self.metrics.max_step_duration = max_step_duration;
        self.metrics.step_budget = step_budget;
        self.metrics.budget_usage = budget_usage;
        self.metrics.budget_warning = budget_warning;
        self.metrics.overloaded = overloaded;
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
            budget_warning,
            overloaded,
            deadline_missed,
            active_workers: self.active_workers,
            average_step_duration,
            max_step_duration,
        })
    }

    fn run_step(&mut self) -> Result<(), StepError> {
        let body_count = self.world.body_count();
        let worker_count = self.active_workers.min(body_count).max(1);
        if worker_count <= 1 {
            return self.world.step_with_executor(
                self.config.fixed_step_seconds(),
                &self.executor,
                worker_count,
            );
        }

        let mut active_bodies = 0;
        let threshold = body_count / 2;
        for (_, body) in self.world.bodies() {
            if body.physics().body_type() != BodyType::Static && !body.state().is_sleeping() {
                active_bodies += 1;
                if active_bodies > threshold {
                    break;
                }
            }
        }
        if active_bodies <= threshold {
            return self.world.step_with_executor(
                self.config.fixed_step_seconds(),
                &self.executor,
                worker_count,
            );
        }

        let max_buckets = worker_count.saturating_mul(self.config.scheduler.max_batches_per_worker);
        let size_limited_buckets = body_count
            .div_ceil(self.config.scheduler.min_batch_size)
            .max(worker_count);
        let bucket_count = size_limited_buckets.min(max_buckets);
        if self.bucket_buffers.len() < bucket_count {
            self.bucket_buffers
                .try_reserve(bucket_count - self.bucket_buffers.len())
                .map_err(|_| StepError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
            self.bucket_buffers.resize_with(bucket_count, Vec::new);
        }
        let buckets = &mut self.bucket_buffers[..bucket_count];
        let dt = self.config.fixed_step_seconds();
        self.world.fill_step_shards(buckets)?;
        self.executor
            .for_each_chunk_mut(buckets, 1, |_, chunk| {
                for shard in chunk {
                    for step in shard {
                        if step.integrate(dt).is_err() {
                            break;
                        }
                    }
                }
            })
            .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
        self.world
            .commit_step_shards_with_executor(buckets, &self.executor, worker_count)
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
