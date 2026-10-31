use std::{
    collections::VecDeque,
    mem,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use kapitsa_core::{
    rigid_body::BodyType,
    world::{BodyStep, PhysicsWorld, StepError},
};
use tokio::{runtime::Handle, task::JoinHandle};

use crate::metrics::MetricsCollector;
use crate::{
    AdaptiveScheduler, SchedulerConfig, SchedulerError, SimulationMetrics, StepMeasurement,
    StepStats,
};

enum PrepareOutput {
    Completed(
        PhysicsWorld,
        Vec<Vec<BodyStep>>,
        Result<(), StepError>,
        Instant,
    ),
    Shards(PhysicsWorld, Vec<Vec<BodyStep>>, Result<(), StepError>),
}
type CommitOutput = (
    PhysicsWorld,
    Vec<Vec<BodyStep>>,
    Result<(), StepError>,
    Instant,
);
type SequentialOutput = (PhysicsWorld, Result<(), StepError>, Instant);

const BUCKETS_PER_WORKER: usize = 4;
const MIN_BUCKET_SIZE: usize = 128;

struct BucketOutput {
    index: usize,
    shard: Vec<BodyStep>,
    error: Option<StepError>,
}

struct WorkerOutput {
    buckets: Vec<BucketOutput>,
}

enum StepWork {
    Sequential {
        task: JoinHandle<SequentialOutput>,
    },
    Preparing {
        task: JoinHandle<PrepareOutput>,
    },
    Integrating {
        tasks: Vec<Option<JoinHandle<WorkerOutput>>>,
        results: Vec<Option<Result<WorkerOutput, String>>>,
        next_task: usize,
        bucket_count: usize,
    },
    Committing {
        task: JoinHandle<CommitOutput>,
    },
}

struct InFlightStep {
    world: Option<PhysicsWorld>,
    work: StepWork,
    started: Instant,
    task_wait_duration: Duration,
    step_duration: Duration,
    step_budget: Duration,
}

struct TaskWaitTimer<'a> {
    total: &'a mut Duration,
    started: Instant,
}

impl Drop for TaskWaitTimer<'_> {
    fn drop(&mut self) {
        *self.total = self.total.saturating_add(self.started.elapsed());
    }
}

async fn wait_for_task<T>(
    task: &mut JoinHandle<T>,
    total_wait: &mut Duration,
) -> Result<T, tokio::task::JoinError> {
    let _timer = TaskWaitTimer {
        total: total_wait,
        started: Instant::now(),
    };
    task.await
}

enum WorldState {
    Available(PhysicsWorld),
    InFlight(InFlightStep),
    Lost,
}

/// Owns a physics world and runs its fixed steps on the caller's Tokio runtime.
pub struct PhysicsController {
    world: WorldState,
    config: SchedulerConfig,
    current_hz: u32,
    active_workers: usize,
    bucket_buffers: Vec<Vec<BodyStep>>,
    adaptive: AdaptiveScheduler,
    metrics: MetricsCollector,
    #[cfg(test)]
    step_gate: Option<(
        tokio::sync::oneshot::Sender<()>,
        std::sync::mpsc::Receiver<()>,
    )>,
}

impl PhysicsController {
    pub fn new(world: PhysicsWorld, config: SchedulerConfig) -> Result<Self, SchedulerError> {
        config.validate()?;
        Ok(Self::with_config(world, config))
    }

    pub(crate) fn with_default_config(world: PhysicsWorld) -> Self {
        Self::with_config(world, SchedulerConfig::default())
    }

    fn with_config(world: PhysicsWorld, config: SchedulerConfig) -> Self {
        let current_hz = config.target_hz;
        let active_workers = 1;
        let step_budget = config
            .step_duration(current_hz)
            .mul_f64(config.target_utilization);
        let adaptive = AdaptiveScheduler::with_valid_config(config.clone());
        let metrics =
            MetricsCollector::new(config.target_hz, current_hz, active_workers, step_budget);
        Self {
            world: WorldState::Available(world),
            config,
            current_hz,
            active_workers,
            bucket_buffers: vec![Vec::new()],
            adaptive,
            metrics,
            #[cfg(test)]
            step_gate: None,
        }
    }

    /// Borrows the world between calls to [`Self::step`].
    pub fn world(&self) -> Result<&PhysicsWorld, SchedulerError> {
        match &self.world {
            WorldState::Available(world) => Ok(world),
            WorldState::InFlight(_) => Err(SchedulerError::WorldUnavailable),
            WorldState::Lost => Err(SchedulerError::WorldLost),
        }
    }

    /// Mutates the world between calls to [`Self::step`].
    pub fn world_mut(&mut self) -> Result<&mut PhysicsWorld, SchedulerError> {
        match &mut self.world {
            WorldState::Available(world) => Ok(world),
            WorldState::InFlight(_) => Err(SchedulerError::WorldUnavailable),
            WorldState::Lost => Err(SchedulerError::WorldLost),
        }
    }

    pub fn config(&self) -> &SchedulerConfig {
        &self.config
    }

    pub fn metrics(&self) -> SimulationMetrics {
        self.metrics.snapshot()
    }

    pub fn current_hz(&self) -> u32 {
        self.current_hz
    }

    pub fn active_workers(&self) -> usize {
        self.active_workers
    }

    /// Selects a bounded worker count for non-adaptive scheduling or tests.
    pub fn set_active_workers(&mut self, workers: usize) -> Result<(), SchedulerError> {
        if workers == 0 || workers > self.config.max_workers {
            return Err(SchedulerError::InvalidConfig(
                "active_workers must be in 1..=max_workers",
            ));
        }
        match self.world {
            WorldState::Available(_) => {}
            WorldState::InFlight(_) => return Err(SchedulerError::WorldUnavailable),
            WorldState::Lost => return Err(SchedulerError::WorldLost),
        }
        self.active_workers = workers;
        self.adaptive.set_active_workers(workers);
        self.update_metrics_state();
        Ok(())
    }

    /// Runs exactly one fixed physics step on the Tokio blocking pool.
    ///
    /// If a previous call was cancelled while its step was in flight, this call
    /// waits for and reports that step instead of starting another one.
    pub async fn step(&mut self) -> Result<StepStats, SchedulerError> {
        if Handle::try_current().is_err() {
            self.metrics.record_error();
            return Err(SchedulerError::MissingRuntime);
        }
        if !matches!(self.world, WorldState::InFlight(_))
            && let Err(error) = self.start_step(self.step_duration())
        {
            self.metrics.record_error();
            return Err(error);
        }

        let (measurement, completed_at) = match self.complete_pending_step().await {
            Ok(result) => result,
            Err(error) => {
                self.metrics.record_error();
                self.update_metrics_state();
                return Err(error);
            }
        };
        let adaptive = self.adaptive.observe(measurement);
        self.current_hz = adaptive.current_hz;
        self.active_workers = adaptive.active_workers;
        let (budget_overrun, deadline_missed) =
            self.metrics
                .record_step(measurement, adaptive, completed_at);
        self.update_metrics_state();
        Ok(StepStats::snapshot(
            measurement,
            budget_overrun,
            deadline_missed,
            adaptive,
            &self.metrics.snapshot(),
        ))
    }

    pub(crate) fn step_duration(&self) -> Duration {
        self.config.step_duration(self.current_hz)
    }

    pub(crate) fn set_simulation_state(&mut self, state: crate::SimulationState) {
        self.metrics.set_state(state);
    }

    pub(crate) fn reset_actual_frequency(&mut self) {
        self.metrics.reset_actual_frequency();
    }

    pub(crate) fn set_simulation_time_debt(&mut self, debt: Duration) {
        self.metrics.set_simulation_time_debt(debt);
    }

    #[cfg(test)]
    pub(crate) fn gate_next_step(
        &mut self,
        entered: tokio::sync::oneshot::Sender<()>,
        release: std::sync::mpsc::Receiver<()>,
    ) {
        self.step_gate = Some((entered, release));
    }

    fn start_step(&mut self, step_duration: Duration) -> Result<(), SchedulerError> {
        let mut world = match mem::replace(&mut self.world, WorldState::Lost) {
            WorldState::Available(world) => world,
            state @ WorldState::InFlight(_) => {
                self.world = state;
                return Err(SchedulerError::WorldUnavailable);
            }
            state @ WorldState::Lost => {
                self.world = state;
                return Err(SchedulerError::WorldLost);
            }
        };

        let started = Instant::now();
        let step_budget = step_duration.mul_f64(self.config.target_utilization);
        let worker_count = self.active_workers.min(world.body_count()).max(1);
        if worker_count == 1 {
            #[cfg(test)]
            let step_gate = self.step_gate.take();
            let task = tokio::task::spawn_blocking(move || {
                #[cfg(test)]
                if let Some((entered, release)) = step_gate {
                    let _ = entered.send(());
                    let _ = release.recv();
                }
                let result = world.step(step_duration.as_secs_f64());
                let completed_at = Instant::now();
                (world, result, completed_at)
            });
            self.world = WorldState::InFlight(InFlightStep {
                world: None,
                work: StepWork::Sequential { task },
                started,
                task_wait_duration: Duration::ZERO,
                step_duration,
                step_budget,
            });
            return Ok(());
        }

        let max_buckets = worker_count.saturating_mul(BUCKETS_PER_WORKER);
        let size_limited_buckets = world
            .body_count()
            .div_ceil(MIN_BUCKET_SIZE)
            .max(worker_count);
        let bucket_count = size_limited_buckets.min(max_buckets);
        self.bucket_buffers.resize_with(bucket_count, Vec::new);
        self.bucket_buffers.truncate(bucket_count);
        let buffers = mem::take(&mut self.bucket_buffers);
        let task = tokio::task::spawn_blocking(move || {
            let mut buffers = buffers;
            // Snapshotting inactive bodies costs more than the sequential step in measured workloads.
            let mut active = 0;
            let threshold = world.body_count() / 2;
            for (_, body) in world.bodies() {
                if body.physics().body_type() != BodyType::Static && !body.state().is_sleeping() {
                    active += 1;
                    if active > threshold {
                        break;
                    }
                }
            }
            if active <= threshold {
                let result = world.step(step_duration.as_secs_f64());
                return PrepareOutput::Completed(world, buffers, result, Instant::now());
            }
            let result = world.fill_step_shards(&mut buffers);
            PrepareOutput::Shards(world, buffers, result)
        });
        self.world = WorldState::InFlight(InFlightStep {
            world: None,
            work: StepWork::Preparing { task },
            started,
            task_wait_duration: Duration::ZERO,
            step_duration,
            step_budget,
        });
        Ok(())
    }

    async fn complete_pending_step(
        &mut self,
    ) -> Result<(StepMeasurement, Instant), SchedulerError> {
        loop {
            let phase = match &self.world {
                WorldState::Available(_) => return Err(SchedulerError::WorldUnavailable),
                WorldState::InFlight(in_flight) => match in_flight.work {
                    StepWork::Sequential { .. } => 0,
                    StepWork::Preparing { .. } => 1,
                    StepWork::Integrating { .. } => 2,
                    StepWork::Committing { .. } => 3,
                },
                WorldState::Lost => return Err(SchedulerError::WorldLost),
            };

            match phase {
                0 => match &mut self.world {
                    WorldState::InFlight(InFlightStep {
                        work: StepWork::Sequential { task },
                        task_wait_duration,
                        ..
                    }) => {
                        let completion = wait_for_task(task, task_wait_duration).await;
                        let in_flight = self.take_in_flight()?;
                        let StepWork::Sequential { .. } = in_flight.work else {
                            self.world = WorldState::Lost;
                            return Err(SchedulerError::WorldLost);
                        };
                        match completion {
                            Ok((world, Ok(()), completed_at)) => {
                                self.restore_world(world);
                                return Ok((
                                    StepMeasurement {
                                        step_duration: completed_at
                                            .saturating_duration_since(in_flight.started),
                                        task_wait_duration: in_flight.task_wait_duration,
                                        step_budget: in_flight.step_budget,
                                        deadline: in_flight.step_duration,
                                    },
                                    completed_at,
                                ));
                            }
                            Ok((world, Err(error), _)) => {
                                self.restore_world(world);
                                return Err(SchedulerError::from(error));
                            }
                            Err(error) => {
                                self.world = WorldState::Lost;
                                self.update_metrics_state();
                                return Err(SchedulerError::BlockingTaskFailed(error.to_string()));
                            }
                        }
                    }
                    WorldState::Available(_) => return Err(SchedulerError::WorldUnavailable),
                    WorldState::InFlight(_) => continue,
                    WorldState::Lost => return Err(SchedulerError::WorldLost),
                },
                1 => {
                    let completion = match &mut self.world {
                        WorldState::InFlight(InFlightStep {
                            work: StepWork::Preparing { task },
                            task_wait_duration,
                            ..
                        }) => wait_for_task(task, task_wait_duration).await,
                        WorldState::Available(_) => {
                            return Err(SchedulerError::WorldUnavailable);
                        }
                        WorldState::InFlight(_) => continue,
                        WorldState::Lost => return Err(SchedulerError::WorldLost),
                    };
                    let in_flight = self.take_in_flight()?;
                    let StepWork::Preparing { .. } = in_flight.work else {
                        self.world = WorldState::Lost;
                        return Err(SchedulerError::WorldLost);
                    };
                    match completion {
                        Ok(PrepareOutput::Completed(world, buffers, result, completed_at)) => {
                            self.restore_available(world, buffers);
                            result.map_err(SchedulerError::from)?;
                            return Ok((
                                StepMeasurement {
                                    step_duration: completed_at
                                        .saturating_duration_since(in_flight.started),
                                    task_wait_duration: in_flight.task_wait_duration,
                                    step_budget: in_flight.step_budget,
                                    deadline: in_flight.step_duration,
                                },
                                completed_at,
                            ));
                        }
                        Ok(PrepareOutput::Shards(world, buffers, Ok(()))) => {
                            let bucket_count = buffers.len();
                            let worker_count = self.active_workers.min(bucket_count).max(1);
                            let queue = Arc::new(Mutex::new(
                                buffers.into_iter().enumerate().collect::<VecDeque<_>>(),
                            ));
                            let mut tasks = Vec::with_capacity(worker_count);
                            for _ in 0..worker_count {
                                let queue = Arc::clone(&queue);
                                let dt = in_flight.step_duration.as_secs_f64();
                                tasks.push(Some(tokio::task::spawn_blocking(move || {
                                    let mut buckets = Vec::new();
                                    loop {
                                        let next = queue
                                            .lock()
                                            .unwrap_or_else(std::sync::PoisonError::into_inner)
                                            .pop_front();
                                        let Some((index, mut shard)) = next else {
                                            break;
                                        };
                                        let mut error = None;
                                        for body in &mut shard {
                                            if let Err(step_error) = body.integrate(dt) {
                                                error = Some(step_error);
                                                break;
                                            }
                                        }
                                        buckets.push(BucketOutput {
                                            index,
                                            shard,
                                            error,
                                        });
                                    }
                                    WorkerOutput { buckets }
                                })));
                            }
                            self.world = WorldState::InFlight(InFlightStep {
                                world: Some(world),
                                work: StepWork::Integrating {
                                    tasks,
                                    results: std::iter::repeat_with(|| None)
                                        .take(worker_count)
                                        .collect(),
                                    next_task: 0,
                                    bucket_count,
                                },
                                started: in_flight.started,
                                task_wait_duration: in_flight.task_wait_duration,
                                step_duration: in_flight.step_duration,
                                step_budget: in_flight.step_budget,
                            });
                        }
                        Ok(PrepareOutput::Shards(world, buffers, Err(error))) => {
                            self.restore_available(world, buffers);
                            return Err(SchedulerError::from(error));
                        }
                        Err(error) => {
                            self.world = WorldState::Lost;
                            self.update_metrics_state();
                            return Err(SchedulerError::BlockingTaskFailed(error.to_string()));
                        }
                    }
                }
                2 => {
                    let task_index = match &self.world {
                        WorldState::InFlight(InFlightStep {
                            work:
                                StepWork::Integrating {
                                    tasks, next_task, ..
                                },
                            ..
                        }) if *next_task < tasks.len() => Some(*next_task),
                        WorldState::InFlight(_) => None,
                        WorldState::Available(_) => {
                            return Err(SchedulerError::WorldUnavailable);
                        }
                        WorldState::Lost => return Err(SchedulerError::WorldLost),
                    };
                    if let Some(task_index) = task_index {
                        let completion = match &mut self.world {
                            WorldState::InFlight(InFlightStep {
                                work: StepWork::Integrating { tasks, .. },
                                task_wait_duration,
                                ..
                            }) => match tasks.get_mut(task_index).and_then(Option::as_mut) {
                                Some(task) => Some(wait_for_task(task, task_wait_duration).await),
                                None => None,
                            },
                            WorldState::Available(_) => {
                                return Err(SchedulerError::WorldUnavailable);
                            }
                            WorldState::InFlight(_) => continue,
                            WorldState::Lost => return Err(SchedulerError::WorldLost),
                        };
                        let Some(completion) = completion else {
                            return Err(SchedulerError::BlockingTaskFailed(
                                "worker task state was missing".to_owned(),
                            ));
                        };
                        if let WorldState::InFlight(InFlightStep {
                            work:
                                StepWork::Integrating {
                                    tasks,
                                    results,
                                    next_task,
                                    ..
                                },
                            ..
                        }) = &mut self.world
                        {
                            tasks[task_index] = None;
                            results[task_index] =
                                Some(completion.map_err(|error| error.to_string()));
                            *next_task = task_index + 1;
                        }
                    } else {
                        self.start_commit_step()?;
                    }
                }
                _ => {
                    let completion = match &mut self.world {
                        WorldState::InFlight(InFlightStep {
                            work: StepWork::Committing { task },
                            task_wait_duration,
                            ..
                        }) => wait_for_task(task, task_wait_duration).await,
                        WorldState::Available(_) => {
                            return Err(SchedulerError::WorldUnavailable);
                        }
                        WorldState::InFlight(_) => continue,
                        WorldState::Lost => return Err(SchedulerError::WorldLost),
                    };
                    let in_flight = self.take_in_flight()?;
                    match completion {
                        Ok((world, buffers, Ok(()), completed_at)) => {
                            self.restore_available(world, buffers);
                            return Ok((
                                StepMeasurement {
                                    step_duration: completed_at
                                        .saturating_duration_since(in_flight.started),
                                    task_wait_duration: in_flight.task_wait_duration,
                                    step_budget: in_flight.step_budget,
                                    deadline: in_flight.step_duration,
                                },
                                completed_at,
                            ));
                        }
                        Ok((world, buffers, Err(error), _)) => {
                            self.restore_available(world, buffers);
                            return Err(SchedulerError::from(error));
                        }
                        Err(error) => {
                            self.world = WorldState::Lost;
                            self.update_metrics_state();
                            return Err(SchedulerError::BlockingTaskFailed(error.to_string()));
                        }
                    }
                }
            }
        }
    }

    fn start_commit_step(&mut self) -> Result<(), SchedulerError> {
        let in_flight = self.take_in_flight()?;
        let StepWork::Integrating {
            results,
            bucket_count,
            ..
        } = in_flight.work
        else {
            self.world = WorldState::Lost;
            return Err(SchedulerError::WorldLost);
        };
        let Some(mut world) = in_flight.world else {
            self.world = WorldState::Lost;
            return Err(SchedulerError::WorldLost);
        };
        let mut buckets = std::iter::repeat_with(|| None)
            .take(bucket_count)
            .collect::<Vec<_>>();
        let mut error = None;
        for result in results {
            match result {
                Some(Ok(output)) => {
                    for bucket in output.buckets {
                        match buckets.get_mut(bucket.index) {
                            Some(slot) if slot.is_none() => *slot = Some(bucket.shard),
                            Some(_) => {
                                if error.is_none() {
                                    error = Some(SchedulerError::BlockingTaskFailed(
                                        "duplicate bucket result".to_owned(),
                                    ));
                                }
                            }
                            None => {
                                if error.is_none() {
                                    error = Some(SchedulerError::BlockingTaskFailed(
                                        "bucket result index was out of range".to_owned(),
                                    ));
                                }
                            }
                        }
                        if error.is_none() {
                            error = bucket.error.map(SchedulerError::from);
                        }
                    }
                }
                Some(Err(reason)) => {
                    if error.is_none() {
                        error = Some(SchedulerError::BlockingTaskFailed(reason));
                    }
                }
                None => {
                    if error.is_none() {
                        error = Some(SchedulerError::BlockingTaskFailed(
                            "worker result was missing".to_owned(),
                        ));
                    }
                }
            }
        }
        if error.is_none() && buckets.iter().any(Option::is_none) {
            error = Some(SchedulerError::BlockingTaskFailed(
                "bucket result was missing".to_owned(),
            ));
        }
        let buffers = buckets
            .into_iter()
            .map(Option::unwrap_or_default)
            .collect::<Vec<_>>();
        if let Some(error) = error {
            self.restore_available(world, buffers);
            return Err(error);
        }

        let task = tokio::task::spawn_blocking(move || {
            let result = world.commit_step_shards(&buffers);
            let completed_at = Instant::now();
            (world, buffers, result, completed_at)
        });
        self.world = WorldState::InFlight(InFlightStep {
            world: None,
            work: StepWork::Committing { task },
            started: in_flight.started,
            task_wait_duration: in_flight.task_wait_duration,
            step_duration: in_flight.step_duration,
            step_budget: in_flight.step_budget,
        });
        Ok(())
    }

    fn take_in_flight(&mut self) -> Result<InFlightStep, SchedulerError> {
        match mem::replace(&mut self.world, WorldState::Lost) {
            WorldState::InFlight(in_flight) => Ok(in_flight),
            state @ WorldState::Available(_) => {
                self.world = state;
                Err(SchedulerError::WorldUnavailable)
            }
            WorldState::Lost => Err(SchedulerError::WorldLost),
        }
    }

    fn restore_available(&mut self, world: PhysicsWorld, buffers: Vec<Vec<BodyStep>>) {
        self.restore_world(world);
        self.bucket_buffers = buffers;
    }

    fn restore_world(&mut self, world: PhysicsWorld) {
        self.world = WorldState::Available(world);
        self.update_metrics_state();
    }

    fn update_metrics_state(&mut self) {
        let step_duration = self.step_duration();
        let step_budget = step_duration.mul_f64(self.config.target_utilization);
        self.metrics.update_scheduler_state(
            self.current_hz,
            self.active_workers,
            step_budget,
            self.adaptive.overload_integral(),
        );
    }
}

#[cfg(test)]
mod tests {
    use std::{
        future::Future,
        task::{Context, Poll, Waker},
        time::Duration,
    };

    use kapitsa_core::{
        mass::MassProperties, rigid_body::RigidBody, transform::Transform, world::PhysicsWorld,
    };
    use kapitsa_math::{Mat3, Vec3};
    use tokio::runtime::Builder;

    use crate::{PhysicsController, SchedulerConfig, SchedulerError};

    fn runtime() -> tokio::runtime::Runtime {
        Builder::new_current_thread().build().expect("test runtime")
    }

    fn moving_kinematic_world() -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        let mut body = RigidBody::new_kinematic(Transform::identity());
        body.set_linear_velocity(Vec3::X).expect("finite velocity");
        world.add_body(body).expect("body insertion");
        world
    }

    fn config(max_catch_up_steps: usize) -> SchedulerConfig {
        SchedulerConfig {
            max_catch_up_steps,
            max_workers: 4,
            ..SchedulerConfig::default()
        }
    }

    fn dynamic_world(body_count: usize) -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        let mass =
            MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass properties");
        for index in 0..body_count {
            let mut body =
                RigidBody::new_dynamic(Transform::identity(), mass).expect("valid dynamic body");
            let scale = index as f64 + 1.0;
            body.set_linear_velocity(Vec3::new(scale, -scale, 0.5 * scale))
                .expect("finite velocity");
            body.set_angular_velocity(Vec3::new(0.1 * scale, 0.2, -0.3))
                .expect("finite velocity");
            body.apply_force(Vec3::new(0.25, -0.5 * scale, 0.75))
                .expect("finite force");
            body.apply_torque(Vec3::new(-0.1 * scale, 0.2, 0.05))
                .expect("finite torque");
            world.add_body(body).expect("body insertion");
        }
        world
    }

    #[tokio::test(start_paused = true)]
    async fn catch_up_uses_changed_frequency_without_losing_time() {
        let mut config = config(1);
        config.adaptive = false;
        let mut controller =
            PhysicsController::new(moving_kinematic_world(), config).expect("controller");
        let first_quantum = controller.step_duration();
        let (_stop_tx, stop_rx) = tokio::sync::watch::channel(false);
        let (metrics_tx, _) = tokio::sync::watch::channel(controller.metrics());
        let mut accumulator = Duration::from_secs(1);
        let mut previous = tokio::time::Instant::now();
        crate::simulation::run_catch_up_batch(
            &mut controller,
            &stop_rx,
            &metrics_tx,
            &mut accumulator,
            &mut previous,
        )
        .await
        .expect("first batch");
        controller.current_hz = 30;
        let second_quantum = controller.step_duration();
        crate::simulation::run_catch_up_batch(
            &mut controller,
            &stop_rx,
            &metrics_tx,
            &mut accumulator,
            &mut previous,
        )
        .await
        .expect("second batch");
        assert_eq!(
            accumulator,
            Duration::from_secs(1) - first_quantum - second_quantum
        );
        let body = controller
            .world()
            .expect("world")
            .bodies()
            .next()
            .expect("body")
            .1;
        let expected = first_quantum.as_secs_f64() + second_quantum.as_secs_f64();
        assert!((body.state().transform().translation().x - expected).abs() < 1.0e-9);
    }

    #[test]
    fn step_runs_once_per_call_on_the_blocking_pool() {
        let runtime = runtime();
        let mut controller =
            PhysicsController::new(moving_kinematic_world(), config(4)).expect("valid controller");
        let id = controller
            .world()
            .expect("world available")
            .bodies()
            .next()
            .expect("one body")
            .0;

        for expected_steps in 1..=2 {
            let stats = runtime.block_on(controller.step()).expect("one fixed step");
            assert!(stats.step_duration > Duration::ZERO);
            assert!(stats.step_budget > Duration::ZERO);
            assert_eq!(controller.metrics().total_steps, expected_steps);
            let translation = controller
                .world()
                .expect("world available")
                .body(id)
                .expect("body exists")
                .state()
                .transform()
                .translation();
            assert!((translation.x - expected_steps as f64 / 60.0).abs() <= 1.0e-9);
        }
    }

    #[test]
    fn physics_error_keeps_world_and_failed_step_time_available_for_retry() {
        let runtime = runtime();
        let mut world = PhysicsWorld::new();
        let tiny_mass = MassProperties::new(f64::from_bits(1), Vec3::ZERO, Mat3::IDENTITY)
            .expect("valid mass properties");
        let mut body =
            RigidBody::new_dynamic(Transform::identity(), tiny_mass).expect("valid dynamic body");
        body.apply_force(Vec3::X * f64::MAX).expect("finite force");
        let id = world.add_body(body).expect("body insertion");
        let mut controller = PhysicsController::new(world, config(2)).expect("valid controller");
        assert!(matches!(
            runtime.block_on(controller.step()),
            Err(SchedulerError::PhysicsStep(_))
        ));
        assert!(
            controller
                .world()
                .expect("world restored")
                .body(id)
                .is_some()
        );
        assert_ne!(
            controller
                .world()
                .expect("world restored")
                .body(id)
                .expect("body exists")
                .forces()
                .force(),
            Vec3::ZERO
        );
    }

    #[test]
    fn world_can_be_configured_between_steps() {
        let runtime = runtime();
        let mut controller =
            PhysicsController::new(PhysicsWorld::new(), config(4)).expect("valid controller");

        let id = controller
            .world_mut()
            .expect("world available")
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("body insertion");
        runtime.block_on(controller.step()).expect("valid step");

        assert!(
            controller
                .world()
                .expect("world available")
                .body(id)
                .is_some()
        );
    }

    #[test]
    fn parallel_worker_counts_match_sequential_integration() {
        let runtime = runtime();
        let config = config(1);
        let quantum = config.step_duration(config.target_hz);
        let mut sequential = dynamic_world(17);
        sequential
            .step(quantum.as_secs_f64())
            .expect("sequential step");

        for workers in [1, 2, 4] {
            let mut controller =
                PhysicsController::new(dynamic_world(17), config.clone()).expect("controller");
            controller
                .set_active_workers(workers)
                .expect("worker count within limit");
            let stats = runtime.block_on(controller.step()).expect("parallel step");

            assert_eq!(stats.active_workers, workers);
            let parallel = controller.world().expect("world available");
            for ((parallel_id, parallel_body), (sequential_id, sequential_body)) in
                parallel.bodies().zip(sequential.bodies())
            {
                assert_eq!(parallel_id, sequential_id);
                assert_eq!(parallel_body, sequential_body);
            }
        }
    }

    #[test]
    fn dynamic_buckets_preserve_body_order_and_results_across_reused_steps() {
        let runtime = runtime();
        let config = config(1);
        let quantum = config.step_duration(config.target_hz);
        let mut controller =
            PhysicsController::new(dynamic_world(2_048), config.clone()).expect("controller");
        controller
            .set_active_workers(2)
            .expect("two active workers");
        let ids = controller
            .world()
            .expect("world available")
            .bodies()
            .map(|(id, _)| id)
            .collect::<Vec<_>>();
        let mut sequential = dynamic_world(2_048);

        for _ in 0..3 {
            runtime.block_on(controller.step()).expect("parallel step");
            sequential
                .step(quantum.as_secs_f64())
                .expect("sequential step");
            for id in &ids {
                assert_eq!(
                    controller.world().expect("world available").body(*id),
                    sequential.body(*id)
                );
            }
        }
    }

    #[test]
    fn inactive_body_fallback_matches_world_step_after_slot_reuse() {
        fn mixed_world() -> PhysicsWorld {
            let mut world = dynamic_world(12);
            let ids = world.bodies().map(|(id, _)| id).collect::<Vec<_>>();
            for id in &ids[..10] {
                world.body_mut(*id).expect("body").sleep().expect("sleep");
            }
            world.remove_body(ids[0]).expect("remove body");
            world
                .add_body(RigidBody::new_static(Transform::identity()))
                .expect("reuse slot");
            world
        }
        let runtime = runtime();
        let config = config(1);
        let quantum = config.step_duration(config.target_hz);
        let mut sequential = mixed_world();
        let mut controller = PhysicsController::new(mixed_world(), config).expect("controller");
        controller.set_active_workers(2).expect("workers");
        sequential
            .step(quantum.as_secs_f64())
            .expect("sequential step");
        runtime.block_on(controller.step()).expect("scheduled step");
        assert!(
            controller
                .world()
                .expect("world")
                .bodies()
                .zip(sequential.bodies())
                .all(|((id, body), (expected_id, expected))| id == expected_id && body == expected)
        );
    }

    #[test]
    fn inactive_body_fallback_preserves_world_on_integration_error() {
        let runtime = runtime();
        let mut world = dynamic_world(8);
        let ids = world.bodies().map(|(id, _)| id).collect::<Vec<_>>();
        for id in &ids[..7] {
            world.body_mut(*id).expect("body").sleep().expect("sleep");
        }
        let body = world.body_mut(ids[7]).expect("active body");
        body.set_mass(
            MassProperties::new(f64::from_bits(1), Vec3::ZERO, Mat3::IDENTITY).expect("mass"),
        )
        .expect("set mass");
        body.apply_force(Vec3::X * f64::MAX).expect("force");
        let original = world
            .bodies()
            .map(|(id, body)| (id, *body))
            .collect::<Vec<_>>();
        let mut controller = PhysicsController::new(world, config(1)).expect("controller");
        controller.set_active_workers(2).expect("workers");
        assert!(matches!(
            runtime.block_on(controller.step()),
            Err(SchedulerError::PhysicsStep(_))
        ));
        assert!(controller.world().expect("world").bodies().zip(original)
            .all(|((id, body), (original_id, original))| id == original_id && body == &original));
    }

    #[test]
    fn parallel_integration_failure_waits_for_workers_and_preserves_world() {
        let runtime = runtime();
        let config = config(1);
        let mut world = dynamic_world(5);
        let failing_id = world.bodies().nth(2).expect("third body").0;
        let failing = world.body_mut(failing_id).expect("body exists");
        failing
            .set_mass(
                MassProperties::new(f64::from_bits(1), Vec3::ZERO, Mat3::IDENTITY)
                    .expect("valid tiny mass"),
            )
            .expect("dynamic mass replacement");
        failing
            .apply_force(Vec3::X * f64::MAX)
            .expect("finite force");
        let originals = world
            .bodies()
            .map(|(id, body)| (id, *body))
            .collect::<Vec<_>>();
        let mut controller = PhysicsController::new(world, config).expect("controller");
        controller
            .set_active_workers(4)
            .expect("worker count within limit");

        assert!(matches!(
            runtime.block_on(controller.step()),
            Err(SchedulerError::PhysicsStep(_))
        ));
        assert!(controller
            .world()
            .expect("world restored")
            .bodies()
            .zip(originals)
            .all(|((id, body), (original_id, original))| id == original_id && body == &original));
    }

    #[test]
    fn cancelling_step_keeps_the_in_flight_world_and_tasks() {
        let runtime = runtime();
        let config = config(1);
        let mut controller =
            PhysicsController::new(dynamic_world(10_000), config).expect("controller");

        runtime.block_on(async {
            let mut step = Box::pin(controller.step());
            let waker = Waker::noop();
            let mut context = Context::from_waker(waker);
            assert!(matches!(step.as_mut().poll(&mut context), Poll::Pending));
            drop(step);

            assert!(matches!(
                controller.world(),
                Err(SchedulerError::WorldUnavailable)
            ));
            controller.step().await.expect("in-flight step completes");
            assert_eq!(controller.metrics().total_steps, 1);
            assert_eq!(
                controller.world().expect("world restored").body_count(),
                10_000
            );
        });
    }
}
