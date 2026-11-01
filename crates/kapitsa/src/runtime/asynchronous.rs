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
};

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
pub enum SimulationError {
    InvalidConfig(ConfigError),
    MissingRuntime,
    RayonPool(RayonPoolError),
    Physics(StepError),
    WorldUnavailable,
    WorldLost,
}

impl std::fmt::Display for SimulationError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfig(error) => write!(formatter, "invalid simulation config: {error}"),
            Self::MissingRuntime => {
                formatter.write_str("an active Tokio runtime is required for async simulation")
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

impl std::error::Error for SimulationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidConfig(error) => Some(error),
            Self::RayonPool(error) => Some(error),
            Self::Physics(error) => Some(error),
            Self::MissingRuntime | Self::WorldUnavailable | Self::WorldLost => None,
        }
    }
}

impl From<ConfigError> for SimulationError {
    fn from(error: ConfigError) -> Self {
        Self::InvalidConfig(error)
    }
}

impl From<RayonPoolError> for SimulationError {
    fn from(error: RayonPoolError) -> Self {
        Self::RayonPool(error)
    }
}

impl From<StepError> for SimulationError {
    fn from(error: StepError) -> Self {
        Self::Physics(error)
    }
}

/// Owns a world and runs one fixed step at a time on a reusable Rayon pool.
///
/// The pool's workers are hosted by the caller's Tokio blocking pool. Call
/// [`Self::shutdown`] before stopping that Tokio runtime to await worker exit.
pub struct Simulation {
    state: EngineState,
    pool: Option<RayonPool>,
    dispatch: Option<RayonExecutor>,
    config: SimulationConfig,
    last_metrics: SimulationMetrics,
    #[cfg(test)]
    step_gate: Option<(oneshot::Sender<()>, std::sync::mpsc::Receiver<()>)>,
}

impl Simulation {
    /// Creates a simulation from the currently entered Tokio runtime.
    pub fn new(world: PhysicsWorld, config: SimulationConfig) -> Result<Self, SimulationError> {
        config.validate()?;
        let handle = Handle::try_current().map_err(|_| SimulationError::MissingRuntime)?;
        Self::new_on(world, config, handle)
    }

    /// Creates a simulation using an explicit handle supplied by the host application.
    pub fn new_on(
        world: PhysicsWorld,
        config: SimulationConfig,
        handle: Handle,
    ) -> Result<Self, SimulationError> {
        config.validate()?;
        let worker_count = std::thread::available_parallelism()
            .map(usize::from)
            .unwrap_or(1)
            .max(1);
        let pool = RayonPool::new(&handle, worker_count)?;
        let executor = pool.executor()?;
        Self::with_executor(world, config, executor, Some(pool))
    }

    /// Creates a simulation using a host-owned executor shared with other simulations.
    pub fn new_with_executor(
        world: PhysicsWorld,
        config: SimulationConfig,
        executor: RayonExecutor,
    ) -> Result<Self, SimulationError> {
        Self::with_executor(world, config, executor, None)
    }

    fn with_executor(
        world: PhysicsWorld,
        config: SimulationConfig,
        executor: RayonExecutor,
        pool: Option<RayonPool>,
    ) -> Result<Self, SimulationError> {
        config.validate()?;
        let worker_count = executor.worker_count();
        let dispatch = executor.clone();
        let mut engine = Box::new(StepEngine::new(world, config.clone(), executor)?);
        engine.set_parallelism(1, Some(worker_count));
        let last_metrics = engine.metrics().clone();
        Ok(Self {
            state: EngineState::Available(engine),
            pool,
            dispatch: Some(dispatch),
            config,
            last_metrics,
            #[cfg(test)]
            step_gate: None,
        })
    }

    /// Runs exactly one fixed physics step without blocking a Tokio worker thread.
    ///
    /// If this future is cancelled while work is running, the simulation retains
    /// the in-flight result. The next `step().await` waits for and reports it.
    pub async fn step(&mut self) -> Result<StepStats, SimulationError> {
        if matches!(self.state, EngineState::Available(_)) {
            self.start_step()?;
        }
        self.complete_step().await
    }

    pub fn world(&self) -> Result<&PhysicsWorld, SimulationError> {
        match &self.state {
            EngineState::Available(engine) => Ok(engine.world()),
            EngineState::InFlight { .. } => Err(SimulationError::WorldUnavailable),
            EngineState::Lost => Err(SimulationError::WorldLost),
        }
    }

    pub fn world_mut(&mut self) -> Result<&mut PhysicsWorld, SimulationError> {
        match &mut self.state {
            EngineState::Available(engine) => Ok(engine.world_mut()),
            EngineState::InFlight { .. } => Err(SimulationError::WorldUnavailable),
            EngineState::Lost => Err(SimulationError::WorldLost),
        }
    }

    pub fn config(&self) -> &SimulationConfig {
        &self.config
    }

    /// Returns the last completed snapshot while a step is in flight.
    pub fn metrics(&self) -> &SimulationMetrics {
        match &self.state {
            EngineState::Available(engine) => engine.metrics(),
            EngineState::InFlight { .. } | EngineState::Lost => &self.last_metrics,
        }
    }

    /// Completes any in-flight step, stops the worker pool, and returns the world.
    pub async fn shutdown(mut self) -> Result<PhysicsWorld, SimulationError> {
        if matches!(self.state, EngineState::InFlight { .. }) {
            let _ = self.complete_step().await;
        }
        let state = mem::replace(&mut self.state, EngineState::Lost);
        let engine = match state {
            EngineState::Available(engine) => engine,
            EngineState::InFlight { .. } | EngineState::Lost => {
                return Err(SimulationError::WorldLost);
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

    fn start_step(&mut self) -> Result<(), SimulationError> {
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
            .ok_or(SimulationError::WorldLost)?
            .spawn(move || {
                #[cfg(test)]
                if let Some((entered, release)) = step_gate {
                    let _ = entered.send(());
                    let _ = release.recv();
                }

                let mut engine = engine;
                let result = match catch_unwind(AssertUnwindSafe(|| engine.step(Duration::ZERO))) {
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

    async fn complete_step(&mut self) -> Result<StepStats, SimulationError> {
        let (received, started_at) = match &mut self.state {
            EngineState::InFlight {
                receiver,
                started_at,
            } => ((&mut *receiver).await, *started_at),
            EngineState::Available(_) => return Err(SimulationError::WorldUnavailable),
            EngineState::Lost => return Err(SimulationError::WorldLost),
        };

        let mut output = match received {
            Ok(output) => output,
            Err(_) => {
                self.state = EngineState::Lost;
                return Err(SimulationError::WorldLost);
            }
        };
        if let Ok(stats) = &mut output.result {
            let task_wait_duration = started_at.elapsed().saturating_sub(stats.step_duration);
            output.engine.record_task_wait(stats, task_wait_duration);
        }
        self.last_metrics = output.engine.metrics().clone();
        self.state = EngineState::Available(output.engine);
        output.result.map_err(SimulationError::Physics)
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
        sync::mpsc,
        task::{Context, Poll, Waker},
        time::Duration,
    };

    use kapitsa_core::{
        mass::MassProperties,
        rigid_body::RigidBody,
        shape::{Shape, Sphere},
        transform::Transform,
        world::PhysicsWorld,
    };
    use kapitsa_math::Vec3;
    use tokio::{runtime::Builder, sync::oneshot, time::timeout};

    use crate::runtime::{RayonPool, SimulationConfig};

    use super::{Simulation, SimulationError};

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
        let mut body =
            RigidBody::new_dynamic(Transform::identity(), mass).expect("body should be valid");
        body.set_linear_velocity(Vec3::X)
            .expect("velocity should be valid");
        world.add_body(body).expect("dynamic body should be valid");
        world
    }

    #[test]
    fn async_step_runs_on_the_pool_and_shutdown_returns_the_world() {
        let runtime = runtime();
        runtime.block_on(async {
            let mut simulation =
                Simulation::new(world(), SimulationConfig::default()).expect("simulation");
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
            assert_eq!(world.body_count(), 2);
        });
        drop(runtime);
    }

    #[test]
    fn cancelling_step_keeps_the_world_and_in_flight_result() {
        let runtime = runtime();
        runtime.block_on(async {
            let mut simulation =
                Simulation::new(world(), SimulationConfig::default()).expect("simulation");
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
                Err(SimulationError::WorldUnavailable)
            ));
            release_tx.send(()).expect("release step worker");
            let stats = timeout(Duration::from_secs(3), simulation.step())
                .await
                .expect("in-flight step should finish")
                .expect("cancelled step should be recovered");
            assert_eq!(stats.active_workers, 1);
            assert_eq!(simulation.metrics().total_steps, 1);

            let world = simulation.shutdown().await.expect("clean shutdown");
            assert_eq!(world.body_count(), 2);
        });
    }

    #[test]
    fn constructor_requires_a_host_runtime_and_does_not_create_one() {
        assert!(matches!(
            Simulation::new(PhysicsWorld::new(), SimulationConfig::default()),
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
                Simulation::new_with_executor(world(), config.clone(), executor.clone())
                    .expect("first simulation");
            let mut second = Simulation::new_with_executor(world(), config, executor.clone())
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
}
