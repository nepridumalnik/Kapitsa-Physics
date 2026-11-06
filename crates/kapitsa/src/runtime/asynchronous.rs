use std::{
    mem,
    panic::{AssertUnwindSafe, catch_unwind},
    time::{Duration, Instant},
};

use kapitsa_core::{
    collision::CollisionError,
    world::{PhysicsWorld, StepError},
};
use tokio::{runtime::Handle, sync::oneshot};

use super::{
    config::{ConfigError, SimulationConfig},
    engine::StepEngine,
    executor::{RayonExecutor, RayonPool, RayonPoolError},
    metrics::{SimulationMetrics, StepStats},
    planner::{AdaptiveScheduler, WorkerConfigError},
};

pub mod marshalling;
mod service;

pub use marshalling::{
    BatchId, CollisionEventBatch, CollisionEventStreamError, CollisionEvents, JointBreakEventBatch,
    JointBreakEventStreamError, JointBreakEvents, PhysicsBatchError, PhysicsBatchResult,
    PhysicsBatchResultStreamError, PhysicsBatchResults, PhysicsClient, PhysicsClientError,
    PhysicsCommand, PhysicsCommandBuffer, PhysicsCommandResult, PhysicsSnapshot,
    PhysicsSubmitError,
};
pub use service::{Simulation, SimulationError};

enum EngineState {
    Available(Box<StepEngine<RayonExecutor>>),
    InFlight {
        receiver: oneshot::Receiver<StepOutput>,
        started_at: Instant,
    },
    Lost,
}

struct StepOutput {
    engine: Box<StepEngine<RayonExecutor>>,
    result: Result<StepStats, StepError>,
}

/// Error returned by the Tokio backed simulation.
#[derive(Debug)]
enum StepRunnerError {
    InvalidConfig(ConfigError),
    InvalidWorkerConfig(WorkerConfigError),
    RayonPool(RayonPoolError),
    Physics(StepError),
    WorldUnavailable,
    WorldLost,
}

impl std::fmt::Display for StepRunnerError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfig(error) => write!(formatter, "invalid simulation config: {error}"),
            Self::InvalidWorkerConfig(error) => {
                write!(formatter, "invalid async worker config: {error}")
            }
            Self::RayonPool(error) => write!(formatter, "Rayon runtime failed: {error}"),
            Self::Physics(error) => write!(formatter, "physics step failed: {error}"),
            Self::WorldUnavailable => {
                formatter.write_str("the world is unavailable while a step is in flight")
            }
            Self::WorldLost => formatter.write_str("the in-flight physics world was lost"),
        }
    }
}

impl std::error::Error for StepRunnerError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidConfig(error) => Some(error),
            Self::InvalidWorkerConfig(error) => Some(error),
            Self::RayonPool(error) => Some(error),
            Self::Physics(error) => Some(error),
            Self::WorldUnavailable | Self::WorldLost => None,
        }
    }
}

impl From<ConfigError> for StepRunnerError {
    fn from(error: ConfigError) -> Self {
        Self::InvalidConfig(error)
    }
}

impl From<RayonPoolError> for StepRunnerError {
    fn from(error: RayonPoolError) -> Self {
        Self::RayonPool(error)
    }
}

impl From<WorkerConfigError> for StepRunnerError {
    fn from(error: WorkerConfigError) -> Self {
        Self::InvalidWorkerConfig(error)
    }
}

impl From<StepError> for StepRunnerError {
    fn from(error: StepError) -> Self {
        Self::Physics(error)
    }
}

/// Owns a world and runs one fixed step at a time on a reusable Rayon pool.
///
/// The pool's workers are hosted by the caller's Tokio blocking pool. Call
/// [`Self::shutdown`] before stopping that Tokio runtime to await worker exit.
struct StepRunner {
    state: EngineState,
    #[allow(dead_code)] // Retains the pool while the simulation is stopped.
    pool: Option<RayonPool>,
    dispatch: Option<RayonExecutor>,
    config: SimulationConfig,
    adaptive: AdaptiveScheduler,
    #[cfg(test)]
    max_workers: usize,
    last_metrics: SimulationMetrics,
    #[cfg(test)]
    step_gate: Option<(oneshot::Sender<()>, std::sync::mpsc::Receiver<()>)>,
}

impl StepRunner {
    fn new_on_validated(
        world: PhysicsWorld,
        config: SimulationConfig,
        handle: Handle,
    ) -> Result<Self, StepRunnerError> {
        let available = std::thread::available_parallelism()
            .map(usize::from)
            .unwrap_or(1)
            .max(1);
        let worker_count = config.workers.resolve_max_workers(available);
        let pool = RayonPool::new(&handle, worker_count)?;
        let executor = pool.executor()?;
        Self::with_executor_validated(world, config, executor, Some(pool))
    }

    #[cfg(test)]
    fn new_with_executor(
        world: PhysicsWorld,
        config: SimulationConfig,
        executor: RayonExecutor,
    ) -> Result<Self, StepRunnerError> {
        config.validate()?;
        Self::with_executor_validated(world, config, executor, None)
    }

    fn with_executor_validated(
        world: PhysicsWorld,
        config: SimulationConfig,
        executor: RayonExecutor,
        pool: Option<RayonPool>,
    ) -> Result<Self, StepRunnerError> {
        let worker_count = executor.worker_count();
        let max_workers = config.workers.resolve_max_workers(worker_count);
        let adaptive = AdaptiveScheduler::with_valid_config(config.workers.clone(), max_workers);
        let dispatch = executor.clone();
        let mut engine = Box::new(StepEngine::new_validated(world, config.clone(), executor));
        engine.set_parallelism(1, Some(worker_count));
        let last_metrics = engine.metrics().clone();
        Ok(Self {
            state: EngineState::Available(engine),
            pool,
            dispatch: Some(dispatch),
            config,
            adaptive,
            #[cfg(test)]
            max_workers,
            last_metrics,
            #[cfg(test)]
            step_gate: None,
        })
    }

    /// Runs exactly one fixed physics step without blocking a Tokio worker thread.
    ///
    /// If this future is cancelled while work is running, the simulation retains
    /// the in-flight result. The next `step().await` waits for and reports it.
    async fn step(&mut self) -> Result<StepStats, StepRunnerError> {
        self.step_with_dt(self.config.fixed_step_seconds()).await
    }

    async fn step_with_dt(&mut self, physics_dt: f64) -> Result<StepStats, StepRunnerError> {
        if matches!(self.state, EngineState::Available(_)) {
            self.start_step(physics_dt)?;
        }
        self.complete_step().await
    }

    fn world(&self) -> Result<&PhysicsWorld, StepRunnerError> {
        match &self.state {
            EngineState::Available(engine) => Ok(engine.world()),
            EngineState::InFlight { .. } => Err(StepRunnerError::WorldUnavailable),
            EngineState::Lost => Err(StepRunnerError::WorldLost),
        }
    }

    fn world_mut(&mut self) -> Result<&mut PhysicsWorld, StepRunnerError> {
        match &mut self.state {
            EngineState::Available(engine) => Ok(engine.world_mut()),
            EngineState::InFlight { .. } => Err(StepRunnerError::WorldUnavailable),
            EngineState::Lost => Err(StepRunnerError::WorldLost),
        }
    }

    fn config(&self) -> &SimulationConfig {
        &self.config
    }

    #[cfg(test)]
    fn max_workers(&self) -> usize {
        self.max_workers
    }

    /// Sets the active compute limit between completed steps.
    #[cfg(test)]
    fn set_active_workers(&mut self, workers: usize) -> Result<(), StepRunnerError> {
        match &self.state {
            EngineState::Available(_) => {}
            EngineState::InFlight { .. } => return Err(StepRunnerError::WorldUnavailable),
            EngineState::Lost => return Err(StepRunnerError::WorldLost),
        }
        self.adaptive.set_active_workers(workers)?;
        let pool_size = self.last_metrics.rayon_pool_size;
        if let EngineState::Available(engine) = &mut self.state {
            engine.set_parallelism(workers, pool_size);
        }
        self.last_metrics.active_workers = workers;
        Ok(())
    }

    /// Returns the last completed snapshot while a step is in flight.
    fn metrics(&self) -> &SimulationMetrics {
        match &self.state {
            EngineState::Available(engine) => engine.metrics(),
            EngineState::InFlight { .. } | EngineState::Lost => &self.last_metrics,
        }
    }

    fn set_runtime_status(
        &mut self,
        state: super::metrics::SimulationState,
        simulation_time_debt: Duration,
    ) {
        if let EngineState::Available(engine) = &mut self.state {
            engine.set_runtime_status(state, simulation_time_debt);
            self.last_metrics = engine.metrics().clone();
        }
    }

    /// Completes any in-flight step, stops the worker pool, and returns the world.
    #[cfg(test)]
    async fn shutdown(mut self) -> Result<PhysicsWorld, StepRunnerError> {
        if matches!(self.state, EngineState::InFlight { .. }) {
            let _ = self.complete_step().await;
        }
        let state = mem::replace(&mut self.state, EngineState::Lost);
        let engine = match state {
            EngineState::Available(engine) => engine,
            EngineState::InFlight { .. } | EngineState::Lost => {
                return Err(StepRunnerError::WorldLost);
            }
        };
        let world = engine.into_world();
        drop(self.dispatch.take());
        if let Some(pool) = self.pool.as_mut() {
            pool.shutdown().await?;
        }
        self.pool = None;
        Ok(world)
    }

    fn start_step(&mut self, physics_dt: f64) -> Result<(), StepRunnerError> {
        let state = mem::replace(&mut self.state, EngineState::Lost);
        let engine = match state {
            EngineState::Available(engine) => engine,
            other => {
                self.state = other;
                return Ok(());
            }
        };
        let (result_sender, result_receiver) = oneshot::channel();
        let started_at = Instant::now();
        #[cfg(test)]
        let step_gate = self.step_gate.take();

        self.dispatch
            .as_ref()
            .ok_or(StepRunnerError::WorldLost)?
            .spawn(move || {
                #[cfg(test)]
                if let Some((entered, release)) = step_gate {
                    let _ = entered.send(());
                    let _ = release.recv();
                }

                let mut engine = engine;
                let result = match catch_unwind(AssertUnwindSafe(|| {
                    engine.step_with_dt(Duration::ZERO, physics_dt)
                })) {
                    Ok(result) => result,
                    Err(_) => Err(StepError::Collision(CollisionError::WorkerFailed)),
                };
                let _ = result_sender.send(StepOutput { engine, result });
            })?;

        self.state = EngineState::InFlight {
            receiver: result_receiver,
            started_at,
        };
        Ok(())
    }

    async fn complete_step(&mut self) -> Result<StepStats, StepRunnerError> {
        let (received, started_at) = match &mut self.state {
            EngineState::InFlight {
                receiver,
                started_at,
            } => ((&mut *receiver).await, *started_at),
            EngineState::Available(_) => return Err(StepRunnerError::WorldUnavailable),
            EngineState::Lost => return Err(StepRunnerError::WorldLost),
        };

        let mut output = match received {
            Ok(output) => output,
            Err(_) => {
                self.state = EngineState::Lost;
                return Err(StepRunnerError::WorldLost);
            }
        };
        if let Ok(stats) = &mut output.result {
            let status = self
                .adaptive
                .observe(stats.step_duration, stats.step_budget);
            stats.budget_warning = status.budget_warning;
            stats.overloaded = status.overloaded;
            let task_wait_duration = started_at.elapsed().saturating_sub(stats.step_duration);
            output.engine.record_task_wait(stats, task_wait_duration);
            let pool_size = output.engine.metrics().rayon_pool_size;
            output
                .engine
                .set_parallelism(status.active_workers, pool_size);
            output
                .engine
                .set_budget_status(status.budget_warning, status.overloaded);
        }
        self.last_metrics = output.engine.metrics().clone();
        self.state = EngineState::Available(output.engine);
        output.result.map_err(StepRunnerError::Physics)
    }

    #[cfg(test)]
    fn gate_next_step(
        &mut self,
        entered: oneshot::Sender<()>,
        release: std::sync::mpsc::Receiver<()>,
    ) {
        self.step_gate = Some((entered, release));
    }
}

#[cfg(test)]
mod tests {
    use std::{
        future::Future,
        num::NonZeroUsize,
        sync::mpsc,
        task::{Context, Poll, Waker},
        time::Duration,
    };

    use kapitsa_core::{
        mass::MassProperties,
        rigid_body::RigidBody,
        shape::{Shape, Sphere},
        transform::Transform,
        world::{PhysicsWorld, StepError},
    };
    use kapitsa_math::{Mat3, Quat, Vec3};
    use tokio::{
        runtime::{Builder, Handle},
        sync::oneshot,
        time::timeout,
    };

    use crate::runtime::{SimulationConfig, WorkerConfig, executor::RayonPool};

    use super::{Simulation, SimulationError, StepRunner, StepRunnerError};

    fn runtime() -> tokio::runtime::Runtime {
        Builder::new_multi_thread()
            .worker_threads(2)
            .max_blocking_threads(1)
            .enable_all()
            .build()
            .expect("runtime should build")
    }

    fn world() -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("static body should be valid");
        let shape = Shape::Sphere(Sphere::new(0.5).expect("sphere should be valid"));
        let mass = MassProperties::from_shape(&shape, 1.0).expect("mass should be valid");
        for _ in 0..3 {
            let mut body =
                RigidBody::new_dynamic(Transform::identity(), mass).expect("body should be valid");
            body.set_linear_velocity(Vec3::X)
                .expect("velocity should be valid");
            world.add_body(body).expect("dynamic body should be valid");
        }
        world
    }

    fn step_runner(world: PhysicsWorld, config: SimulationConfig) -> StepRunner {
        config.validate().expect("valid configuration");
        StepRunner::new_on_validated(
            world,
            config,
            Handle::try_current().expect("host runtime is entered"),
        )
        .expect("step runner")
    }

    #[test]
    fn async_step_runs_on_the_pool_and_shutdown_returns_the_world() {
        let runtime = runtime();
        runtime.block_on(async {
            let mut simulation = step_runner(world(), SimulationConfig::default());
            let config = simulation.config().clone();
            let mut expected = world();
            expected
                .step(config.fixed_step_seconds())
                .expect("reference step should succeed");
            let stats = timeout(Duration::from_secs(3), simulation.step())
                .await
                .expect("step should not starve the one-slot Tokio blocking pool")
                .expect("step should succeed");
            assert_eq!(stats.active_workers, 1);
            assert_eq!(simulation.metrics().total_steps, 1);
            assert!(
                simulation
                    .world()
                    .expect("world should be available")
                    .bodies()
                    .map(|(_, body)| body)
                    .zip(expected.bodies())
                    .all(|(body, (_, expected_body))| body == expected_body)
            );

            let world = timeout(Duration::from_secs(3), simulation.shutdown())
                .await
                .expect("shutdown should finish before the Tokio runtime")
                .expect("worker pool should shut down");
            assert_eq!(world.body_count(), 4);
        });
        drop(runtime);
    }

    #[test]
    fn cancelling_sharded_force_source_step_keeps_the_world_and_in_flight_result() {
        let runtime = runtime();
        runtime.block_on(async {
            let mut force_world = PhysicsWorld::new();
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)
                .expect("valid mass properties");
            let bodies = (0..4)
                .map(|index| {
                    force_world
                        .add_body(
                            RigidBody::new_dynamic(
                                Transform::new(Vec3::X * (index as f64 * 4.0), Quat::IDENTITY)
                                    .expect("valid transform"),
                                mass,
                            )
                            .expect("dynamic body"),
                        )
                        .expect("body id")
                })
                .collect::<Vec<_>>();
            force_world
                .add_force_source(bodies[0], Vec3::ZERO, Vec3::Y)
                .expect("force source");
            let config = SimulationConfig {
                workers: WorkerConfig {
                    max_workers: NonZeroUsize::new(4),
                    ..WorkerConfig::default()
                },
                ..SimulationConfig::default()
            };
            let mut simulation = step_runner(force_world, config);
            simulation
                .set_active_workers(4)
                .expect("activate sharded step path");
            let (entered_tx, entered_rx) = oneshot::channel();
            let (release_tx, release_rx) = mpsc::channel();
            simulation.gate_next_step(entered_tx, release_rx);

            let mut step = Box::pin(simulation.step());
            let waker = Waker::noop();
            let mut context = Context::from_waker(waker);
            assert!(matches!(step.as_mut().poll(&mut context), Poll::Pending));
            timeout(Duration::from_secs(3), entered_rx)
                .await
                .expect("step worker should start")
                .expect("worker should signal start");
            drop(step);

            assert!(matches!(
                simulation.world(),
                Err(StepRunnerError::WorldUnavailable)
            ));
            release_tx.send(()).expect("release step worker");
            let stats = timeout(Duration::from_secs(3), simulation.step())
                .await
                .expect("in-flight step should finish")
                .expect("cancelled step should be recovered");
            assert_eq!(stats.active_workers, 4);
            assert_eq!(simulation.metrics().total_steps, 1);

            let world = simulation.shutdown().await.expect("clean shutdown");
            assert_eq!(world.body_count(), 4);
            assert_eq!(
                world
                    .body(bodies[0])
                    .expect("body")
                    .state()
                    .linear_velocity(),
                Vec3::Y / 60.0
            );
        });
    }

    #[test]
    fn public_constructor_requires_a_host_runtime_and_does_not_create_one() {
        assert!(matches!(
            Simulation::new(),
            Err(SimulationError::MissingRuntime)
        ));
    }

    #[test]
    fn simulations_can_share_a_host_owned_pool_without_global_state() {
        let runtime = runtime();
        runtime.block_on(async {
            let mut pool = RayonPool::new(runtime.handle(), 4).expect("shared pool");
            let executor = pool.executor().expect("shared executor");
            let config = SimulationConfig::default();
            let mut first =
                StepRunner::new_with_executor(world(), config.clone(), executor.clone())
                    .expect("first simulation");
            let mut second = StepRunner::new_with_executor(world(), config, executor.clone())
                .expect("second simulation");
            drop(executor);

            let (first_result, second_result) = tokio::join!(first.step(), second.step());
            first_result.expect("first step");
            second_result.expect("second step");
            assert_eq!(first.metrics().total_steps, 1);
            assert_eq!(second.metrics().total_steps, 1);

            first.shutdown().await.expect("first shutdown");
            second.shutdown().await.expect("second shutdown");
            pool.shutdown().await.expect("shared pool shutdown");
        });
    }

    #[test]
    fn worker_config_caps_the_pool_and_active_parallelism() {
        let runtime = runtime();
        runtime.block_on(async {
            let config = SimulationConfig {
                workers: WorkerConfig {
                    max_workers: NonZeroUsize::new(2),
                    ..WorkerConfig::default()
                },
                ..SimulationConfig::default()
            };
            let mut simulation = step_runner(world(), config);
            assert_eq!(simulation.max_workers(), 2);
            assert!(simulation.set_active_workers(3).is_err());
            simulation
                .set_active_workers(2)
                .expect("worker count within pool limit");
            let stats = simulation.step().await.expect("step should succeed");
            assert_eq!(stats.active_workers, 2);
            assert_eq!(simulation.metrics().rayon_pool_size, Some(2));
            simulation.shutdown().await.expect("clean shutdown");
        });
    }

    #[test]
    fn parallel_integration_error_keeps_the_world_unmodified() {
        let runtime = runtime();
        runtime.block_on(async {
            let mut world = world();
            let failing_id = world.bodies().nth(3).expect("fourth body").0;
            let failing_body = world.body_mut(failing_id).expect("body exists");
            failing_body
                .set_mass(
                    MassProperties::new(f64::from_bits(1), Vec3::ZERO, Mat3::IDENTITY)
                        .expect("tiny mass should be valid"),
                )
                .expect("body mass should update");
            failing_body
                .apply_force(Vec3::X * f64::MAX)
                .expect("force should be finite");
            let original = world
                .bodies()
                .map(|(id, body)| (id, *body))
                .collect::<Vec<_>>();
            let config = SimulationConfig {
                workers: WorkerConfig {
                    max_workers: NonZeroUsize::new(2),
                    ..WorkerConfig::default()
                },
                ..SimulationConfig::default()
            };
            let mut simulation = step_runner(world, config);
            simulation
                .set_active_workers(2)
                .expect("worker limit should permit two workers");

            assert!(matches!(
                simulation.step().await,
                Err(StepRunnerError::Physics(StepError::BodyIntegration { .. }))
            ));
            assert!(
                simulation
                    .world()
                    .expect("world should be restored")
                    .bodies()
                    .zip(original)
                    .all(|((id, body), (original_id, original_body))| {
                        id == original_id && body == &original_body
                    })
            );
            simulation.shutdown().await.expect("clean shutdown");
        });
    }
}
