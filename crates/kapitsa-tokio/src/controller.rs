use std::{
    mem,
    time::{Duration, Instant},
};

use kapitsa_core::world::{BodyStep, PhysicsWorld, StepError};
use tokio::{runtime::Handle, task::JoinHandle};

use crate::{
    AdaptiveScheduler, SchedulerConfig, SchedulerError, SchedulerMetrics, StepMeasurement,
    StepStats,
};

type PrepareOutput = (PhysicsWorld, Vec<Vec<BodyStep>>, Result<(), StepError>);
type CommitOutput = (PhysicsWorld, Vec<Vec<BodyStep>>, Result<(), StepError>);

struct WorkerOutput {
    shard: Vec<BodyStep>,
    error: Option<StepError>,
}

enum StepWork {
    Preparing {
        task: JoinHandle<PrepareOutput>,
    },
    Integrating {
        tasks: Vec<Option<JoinHandle<WorkerOutput>>>,
        results: Vec<Option<Result<WorkerOutput, String>>>,
        next_task: usize,
    },
    Committing {
        task: JoinHandle<CommitOutput>,
    },
}

struct InFlightStep {
    world: Option<PhysicsWorld>,
    work: StepWork,
    started: Instant,
    wait_started: Instant,
    step_duration: Duration,
    step_budget: Duration,
}

enum WorldState {
    Available(PhysicsWorld),
    InFlight(InFlightStep),
    Lost,
}

/// Owns a physics world and advances it using the caller's Tokio runtime.
pub struct PhysicsController {
    world: WorldState,
    config: SchedulerConfig,
    current_hz: u32,
    active_workers: usize,
    worker_buffers: Vec<Vec<BodyStep>>,
    adaptive: AdaptiveScheduler,
    metrics: SchedulerMetrics,
}

impl PhysicsController {
    pub fn new(world: PhysicsWorld, config: SchedulerConfig) -> Result<Self, SchedulerError> {
        config.validate()?;
        let current_hz = config.target_hz;
        let active_workers = 1;
        let adaptive = AdaptiveScheduler::new(config.clone())?;
        Ok(Self {
            world: WorldState::Available(world),
            config,
            current_hz,
            active_workers,
            worker_buffers: vec![Vec::new()],
            adaptive,
            metrics: SchedulerMetrics::new(current_hz, active_workers),
        })
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

    pub fn metrics(&self) -> &SchedulerMetrics {
        &self.metrics
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
        Handle::try_current().map_err(|_| SchedulerError::MissingRuntime)?;
        if !matches!(self.world, WorldState::InFlight(_)) {
            self.start_step(self.step_duration())?;
        }

        let measurement = self.complete_pending_step().await?;
        let adaptive = self.adaptive.observe(measurement);
        self.current_hz = adaptive.current_hz;
        self.active_workers = adaptive.active_workers;
        let budget_overrun = self.metrics.record_step(measurement, adaptive);
        self.update_metrics_state();
        Ok(StepStats::snapshot(
            measurement,
            budget_overrun,
            adaptive,
            &self.metrics,
        ))
    }

    fn step_duration(&self) -> Duration {
        self.config.step_duration(self.current_hz)
    }

    fn start_step(&mut self, step_duration: Duration) -> Result<(), SchedulerError> {
        let world = match mem::replace(&mut self.world, WorldState::Lost) {
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

        let worker_count = self.active_workers.min(world.body_count()).max(1);
        self.worker_buffers.resize_with(worker_count, Vec::new);
        self.worker_buffers.truncate(worker_count);
        let buffers = mem::take(&mut self.worker_buffers);
        let started = Instant::now();
        let step_budget = step_duration.mul_f64(self.config.target_utilization);
        let task = tokio::task::spawn_blocking(move || {
            let mut buffers = buffers;
            let result = world.fill_step_shards(&mut buffers);
            (world, buffers, result)
        });
        self.world = WorldState::InFlight(InFlightStep {
            world: None,
            work: StepWork::Preparing { task },
            started,
            wait_started: started,
            step_duration,
            step_budget,
        });
        Ok(())
    }

    async fn complete_pending_step(&mut self) -> Result<StepMeasurement, SchedulerError> {
        loop {
            let phase = match &self.world {
                WorldState::Available(_) => return Err(SchedulerError::WorldUnavailable),
                WorldState::InFlight(in_flight) => match in_flight.work {
                    StepWork::Preparing { .. } => 0,
                    StepWork::Integrating { .. } => 1,
                    StepWork::Committing { .. } => 2,
                },
                WorldState::Lost => return Err(SchedulerError::WorldLost),
            };

            match phase {
                0 => {
                    let completion = match &mut self.world {
                        WorldState::InFlight(InFlightStep {
                            work: StepWork::Preparing { task },
                            ..
                        }) => task.await,
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
                        Ok((world, buffers, Ok(()))) => {
                            let tasks = buffers
                                .into_iter()
                                .map(|mut shard| {
                                    let task = tokio::task::spawn_blocking(move || {
                                        let mut error = None;
                                        for body in &mut shard {
                                            if let Err(step_error) = body
                                                .integrate(in_flight.step_duration.as_secs_f64())
                                            {
                                                error = Some(step_error);
                                                break;
                                            }
                                        }
                                        WorkerOutput { shard, error }
                                    });
                                    Some(task)
                                })
                                .collect::<Vec<_>>();
                            let result_count = tasks.len();
                            self.world = WorldState::InFlight(InFlightStep {
                                world: Some(world),
                                work: StepWork::Integrating {
                                    tasks,
                                    results: std::iter::repeat_with(|| None)
                                        .take(result_count)
                                        .collect(),
                                    next_task: 0,
                                },
                                started: in_flight.started,
                                wait_started: in_flight.wait_started,
                                step_duration: in_flight.step_duration,
                                step_budget: in_flight.step_budget,
                            });
                        }
                        Ok((world, buffers, Err(error))) => {
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
                1 => {
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
                                ..
                            }) => match tasks.get_mut(task_index).and_then(Option::as_mut) {
                                Some(task) => Some(task.await),
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
                            ..
                        }) => task.await,
                        WorldState::Available(_) => {
                            return Err(SchedulerError::WorldUnavailable);
                        }
                        WorldState::InFlight(_) => continue,
                        WorldState::Lost => return Err(SchedulerError::WorldLost),
                    };
                    let in_flight = self.take_in_flight()?;
                    match completion {
                        Ok((world, buffers, Ok(()))) => {
                            self.restore_available(world, buffers);
                            return Ok(StepMeasurement {
                                step_duration: in_flight.started.elapsed(),
                                task_wait_duration: in_flight.wait_started.elapsed(),
                                step_budget: in_flight.step_budget,
                            });
                        }
                        Ok((world, buffers, Err(error))) => {
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
        let StepWork::Integrating { results, .. } = in_flight.work else {
            self.world = WorldState::Lost;
            return Err(SchedulerError::WorldLost);
        };
        let Some(mut world) = in_flight.world else {
            self.world = WorldState::Lost;
            return Err(SchedulerError::WorldLost);
        };
        let mut buffers = std::iter::repeat_with(Vec::new)
            .take(results.len())
            .collect::<Vec<_>>();
        let mut error = None;
        for (index, result) in results.into_iter().enumerate() {
            match result {
                Some(Ok(output)) => {
                    buffers[index] = output.shard;
                    if error.is_none() {
                        error = output.error.map(SchedulerError::from);
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
        if let Some(error) = error {
            self.restore_available(world, buffers);
            return Err(error);
        }

        let task = tokio::task::spawn_blocking(move || {
            let result = world.commit_step_shards(&buffers);
            (world, buffers, result)
        });
        self.world = WorldState::InFlight(InFlightStep {
            world: None,
            work: StepWork::Committing { task },
            started: in_flight.started,
            wait_started: in_flight.wait_started,
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
        self.world = WorldState::Available(world);
        self.worker_buffers = buffers;
        self.update_metrics_state();
    }

    fn update_metrics_state(&mut self) {
        self.metrics
            .update_scheduler_state(self.current_hz, self.active_workers, Duration::ZERO);
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
            assert_eq!(controller.metrics().total_steps(), expected_steps);
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
            assert_eq!(controller.metrics().total_steps(), 1);
            assert_eq!(
                controller.world().expect("world restored").body_count(),
                10_000
            );
        });
    }
}
