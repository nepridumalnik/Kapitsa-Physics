use crate::runtime::constants::{
    COLLISION_EVENT_BATCH_CAPACITY, COMMAND_QUEUE_CAPACITY, JOINT_BREAK_EVENT_BATCH_CAPACITY,
    MAX_COMMANDS_PER_STEP, MIN_LOAD_SPEED, PHYSICS_BATCH_RESULT_CAPACITY,
};
use std::{
    sync::{Arc, atomic::AtomicU64},
    time::Duration,
};

use kapitsa_core::{
    aerodynamics::Atmosphere,
    collision::CollisionError,
    world::{PhysicsWorld, StepError},
};
use tokio::{
    runtime::Handle,
    sync::{broadcast, mpsc, watch},
    task::JoinHandle,
    time::Instant,
};

use super::{
    StepRunner, StepRunnerError,
    marshalling::{
        BatchScratch, ChunkOutcome, ChunkReply, CollisionEventBatch, CommandBufferPool,
        CommandRequest, JointBreakEventBatch, PhysicsBatchError, PhysicsBatchResult, PhysicsClient,
        PhysicsClientError, PhysicsCommand, PhysicsCommandResult, PhysicsSnapshot, apply_chunk,
        apply_command, apply_command_batch,
    },
};
use crate::runtime::{
    ConfigError, SimulationConfig, SimulationMetrics, SimulationState, TimeStepPolicy,
    adaptive_step::{StepChoice, choose_step},
    executor::RayonExecutor,
};

struct ServiceTaskOutput {
    simulation: StepRunner,
    simulation_time: Duration,
    result: Result<(), SimulationError>,
}

struct RuntimeControl {
    stop: watch::Receiver<bool>,
    time_multiplier: watch::Receiver<f64>,
}

#[derive(Clone)]
struct SimulationEvents {
    collisions: broadcast::Sender<CollisionEventBatch>,
    joint_breaks: broadcast::Sender<JointBreakEventBatch>,
    batch_results: broadcast::Sender<PhysicsBatchResult>,
}

struct CommandQueue {
    receiver: mpsc::Receiver<CommandRequest>,
    pending: Option<CommandRequest>,
    batch_scratch: BatchScratch,
    pending_results: Vec<DeferredCommandResult>,
    buffer_pool: Arc<CommandBufferPool>,
}

enum DeferredCommandResult {
    Single {
        reply: tokio::sync::oneshot::Sender<Result<PhysicsCommandResult, PhysicsClientError>>,
        result: Result<PhysicsCommandResult, PhysicsClientError>,
        is_read_only_query: bool,
    },
    Batch(PhysicsBatchResult),
    Chunk {
        reply: tokio::sync::oneshot::Sender<ChunkReply>,
        outcome: ChunkOutcome,
    },
}

impl CommandQueue {
    fn new(receiver: mpsc::Receiver<CommandRequest>, buffer_pool: Arc<CommandBufferPool>) -> Self {
        Self {
            receiver,
            pending: None,
            batch_scratch: BatchScratch::default(),
            pending_results: Vec::with_capacity(MAX_COMMANDS_PER_STEP),
            buffer_pool,
        }
    }
}

/// Owns a clock-driven physics simulation and exposes its client API.
pub struct Simulation {
    simulation: Option<StepRunner>,
    task: Option<JoinHandle<ServiceTaskOutput>>,
    stop: Option<watch::Sender<bool>>,
    command_tx: watch::Sender<Option<mpsc::Sender<CommandRequest>>>,
    command_rx: watch::Receiver<Option<mpsc::Sender<CommandRequest>>>,
    time_multiplier_tx: watch::Sender<f64>,
    snapshot_tx: watch::Sender<Arc<PhysicsSnapshot>>,
    snapshot_rx: watch::Receiver<Arc<PhysicsSnapshot>>,
    collision_events_tx: broadcast::Sender<CollisionEventBatch>,
    joint_break_events_tx: broadcast::Sender<JointBreakEventBatch>,
    batch_results_tx: broadcast::Sender<PhysicsBatchResult>,
    batch_id_sequence: Arc<AtomicU64>,
    buffer_pool: Arc<CommandBufferPool>,
    simulation_time: Duration,
    metrics_tx: watch::Sender<SimulationMetrics>,
    metrics_rx: watch::Receiver<SimulationMetrics>,
    runtime: Handle,
    paused_multiplier: Option<f64>,
    owned_runtime: Option<OwnedRuntime>,
}

struct OwnedRuntime {
    shutdown: Option<tokio::sync::oneshot::Sender<()>>,
}

impl Drop for OwnedRuntime {
    fn drop(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
    }
}

impl OwnedRuntime {
    fn start() -> Result<(Self, Handle), SimulationError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(1);
        let (shutdown, stopped) = tokio::sync::oneshot::channel();
        std::thread::Builder::new()
            .name("kapitsa-clock".into())
            .spawn(move || {
                match tokio::runtime::Builder::new_current_thread()
                    .enable_time()
                    .build()
                {
                    Ok(runtime) => {
                        if sender.send(Ok(runtime.handle().clone())).is_ok() {
                            runtime.block_on(async {
                                let _ = stopped.await;
                            });
                        }
                    }
                    Err(error) => {
                        let _ = sender.send(Err(error.to_string()));
                    }
                }
            })
            .map_err(|error| SimulationError::TaskFailed(error.to_string()))?;
        let runtime = receiver
            .recv()
            .map_err(|error| SimulationError::TaskFailed(error.to_string()))?
            .map_err(SimulationError::TaskFailed)?;
        Ok((
            Self {
                shutdown: Some(shutdown),
            },
            runtime,
        ))
    }
}

/// Settings for starting a clock-driven simulation.
#[derive(Default)]
pub struct SimulationBuilder {
    atmosphere: Atmosphere,
    mesh_sound_speed: Option<f64>,
    config: SimulationConfig,
    invalid_worker_count: bool,
}

impl SimulationBuilder {
    /// Configures the atmosphere before the simulation starts. Defaults to vacuum.
    ///
    /// ```no_run
    /// use kapitsa::{Simulation, aerodynamics::{Atmosphere, DensityProfile, SphericalAtmosphere}};
    ///
    /// # async fn example() -> Result<(), Box<dyn std::error::Error>> {
    /// let sphere = SphericalAtmosphere::builder()
    ///     .reference_radius(600_000.0)
    ///     .reference_density(1.225)
    ///     .height(80_000.0)
    ///     .profile(DensityProfile::Exponential { scale_height: 5_600.0 })
    ///     .build()?;
    /// let mut simulation = Simulation::builder()
    ///     .atmosphere(Atmosphere::Sphere(sphere))
    ///     .spawn_on_tokio(&tokio::runtime::Handle::current())?;
    /// simulation.stop().await?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn atmosphere(mut self, atmosphere: Atmosphere) -> Self {
        self.atmosphere = atmosphere;
        self
    }

    /// Sets uniform Mesh/Colliders sound speed in m/s (compatibility default: 340).
    /// Supply a finite positive value matching the selected atmosphere.
    /// Invalid values are rejected when spawning, before starting any service.
    /// Example: `Simulation::builder().mesh_sound_speed(295.0)`.
    pub fn mesh_sound_speed(mut self, sound_speed: f64) -> Self {
        self.mesh_sound_speed = Some(sound_speed);
        self
    }

    /// Sets the base physics frequency in hertz.
    pub fn frequency(mut self, frequency: u32) -> Self {
        self.config.tick_rate = frequency;
        self
    }

    /// Selects fixed or physics-bounded adaptive stepping.
    pub fn time_step_policy(mut self, policy: TimeStepPolicy) -> Self {
        self.config.time_step_policy = policy;
        self
    }

    /// Limits the number of workers used by this simulation.
    pub fn max_workers(mut self, workers: usize) -> Self {
        self.invalid_worker_count = workers == 0;
        self.config.workers.max_workers = std::num::NonZeroUsize::new(workers);
        self
    }

    /// Starts a simulation on the supplied Tokio runtime.
    pub fn spawn_on_tokio(self, runtime: &Handle) -> Result<Simulation, SimulationError> {
        if self.invalid_worker_count {
            return Err(SimulationError::InvalidWorkerCount);
        }
        let _entered = runtime.enter();
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(self.atmosphere);
        if let Some(sound_speed) = self.mesh_sound_speed {
            world
                .set_mesh_sound_speed(sound_speed)
                .map_err(SimulationError::InvalidMeshSoundSpeed)?;
        }
        let mut simulation = Simulation::new_with_world(world, self.config)?;
        simulation.start()?;
        Ok(simulation)
    }

    /// Starts a simulation using the supplied Rayon pool for computation.
    pub fn spawn_on_rayon(
        self,
        pool: Arc<rayon::ThreadPool>,
    ) -> Result<Simulation, SimulationError> {
        if self.invalid_worker_count {
            return Err(SimulationError::InvalidWorkerCount);
        }
        self.config
            .validate()
            .map_err(SimulationError::InvalidConfig)?;
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(self.atmosphere);
        if let Some(sound_speed) = self.mesh_sound_speed {
            world
                .set_mesh_sound_speed(sound_speed)
                .map_err(SimulationError::InvalidMeshSoundSpeed)?;
        }
        let (owned_runtime, runtime) = OwnedRuntime::start()?;
        let runner =
            StepRunner::new_with_executor(world, self.config, RayonExecutor::from_pool(pool))?;
        let mut simulation = Simulation::from_runner(runner, runtime)?;
        simulation.owned_runtime = Some(owned_runtime);
        simulation.start()?;
        Ok(simulation)
    }
}

/// Failure returned by a clock-driven physics service.
#[derive(Debug)]
pub enum SimulationError {
    InvalidConfig(ConfigError),
    MissingRuntime,
    SimulationTimeOverflow,
    InvalidTimeMultiplier,
    InvalidWorkerCount,
    InvalidMeshSoundSpeed(kapitsa_core::aerodynamics::pressure::PressureError),
    AlreadyRunning,
    TaskRequiresStop,
    Physics(kapitsa_core::world::StepError),
    WorkerPoolInitializationFailed,
    WorldUnavailable,
    WorldLost,
    TaskFailed(String),
}

impl std::fmt::Display for SimulationError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidMeshSoundSpeed(error) => {
                write!(formatter, "invalid mesh flow configuration: {error}")
            }
            Self::InvalidConfig(error) => write!(formatter, "invalid simulation config: {error}"),
            Self::MissingRuntime => {
                formatter.write_str("starting a physics service requires a Tokio runtime")
            }
            Self::SimulationTimeOverflow => {
                formatter.write_str("simulation time accumulator overflowed")
            }
            Self::InvalidTimeMultiplier => {
                formatter.write_str("time multiplier must be finite and non-negative")
            }
            Self::InvalidWorkerCount => formatter.write_str("worker count must be positive"),
            Self::AlreadyRunning => formatter.write_str("simulation is already running"),
            Self::TaskRequiresStop => {
                formatter.write_str("stop the completed simulation before restarting")
            }
            Self::Physics(error) => write!(formatter, "physics step failed: {error}"),
            Self::WorkerPoolInitializationFailed => {
                formatter.write_str("physics worker pool could not be initialized")
            }
            Self::WorldUnavailable => formatter.write_str("physics world is unavailable"),
            Self::WorldLost => formatter.write_str("the physics world was lost"),
            Self::TaskFailed(reason) => write!(formatter, "simulation task failed: {reason}"),
        }
    }
}

impl std::error::Error for SimulationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidConfig(error) => Some(error),
            Self::InvalidMeshSoundSpeed(error) => Some(error),
            Self::Physics(error) => Some(error),
            _ => None,
        }
    }
}

impl From<StepRunnerError> for SimulationError {
    fn from(error: StepRunnerError) -> Self {
        match error {
            StepRunnerError::InvalidConfig(error) => Self::InvalidConfig(error),
            StepRunnerError::InvalidWorkerConfig(error) => {
                Self::InvalidConfig(ConfigError::InvalidWorkerConfig(error))
            }
            StepRunnerError::RayonPool(_) => Self::WorkerPoolInitializationFailed,
            StepRunnerError::Physics(error) => Self::Physics(error),
            StepRunnerError::WorldUnavailable => Self::WorldUnavailable,
            StepRunnerError::WorldLost => Self::WorldLost,
        }
    }
}

impl Simulation {
    pub fn builder() -> SimulationBuilder {
        SimulationBuilder::default()
    }
    /// Creates an empty simulation using the default fixed-step configuration.
    pub fn new() -> Result<Self, SimulationError> {
        Self::new_from_config(SimulationConfig::default())
    }

    /// Creates an empty simulation using the supplied configuration.
    pub fn new_from_config(config: SimulationConfig) -> Result<Self, SimulationError> {
        Self::new_with_world(PhysicsWorld::new(), config)
    }

    pub(crate) fn new_with_world(
        world: PhysicsWorld,
        config: SimulationConfig,
    ) -> Result<Self, SimulationError> {
        config.validate().map_err(SimulationError::InvalidConfig)?;
        let runtime = Handle::try_current().map_err(|_| SimulationError::MissingRuntime)?;
        let simulation = StepRunner::new_on_validated(world, config, runtime.clone())?;
        Self::from_runner(simulation, runtime)
    }

    fn from_runner(simulation: StepRunner, runtime: Handle) -> Result<Self, SimulationError> {
        let world = simulation.world().map_err(SimulationError::from)?;
        let initial_snapshot = Arc::new(PhysicsSnapshot::from_world(
            world,
            simulation.metrics().total_steps,
            Duration::ZERO,
        ));
        let (metrics_tx, metrics_rx) = watch::channel(simulation.metrics().clone());
        let (command_tx, command_rx) = watch::channel(None);
        let (time_multiplier_tx, _) = watch::channel(1.0);
        let (snapshot_tx, snapshot_rx) = watch::channel(initial_snapshot);
        let (collision_events_tx, _) = broadcast::channel(COLLISION_EVENT_BATCH_CAPACITY);
        let (joint_break_events_tx, _) = broadcast::channel(JOINT_BREAK_EVENT_BATCH_CAPACITY);
        let (batch_results_tx, _) = broadcast::channel(PHYSICS_BATCH_RESULT_CAPACITY);
        let buffer_pool = Arc::new(CommandBufferPool::default());
        Ok(Self {
            simulation: Some(simulation),
            task: None,
            stop: None,
            command_tx,
            command_rx,
            time_multiplier_tx,
            snapshot_tx,
            snapshot_rx,
            collision_events_tx,
            joint_break_events_tx,
            batch_results_tx,
            batch_id_sequence: Arc::new(AtomicU64::new(1)),
            buffer_pool,
            simulation_time: Duration::ZERO,
            metrics_tx,
            metrics_rx,
            runtime,
            paused_multiplier: None,
            owned_runtime: None,
        })
    }

    /// Starts clock-driven stepping on the configured Tokio runtime.
    pub fn start(&mut self) -> Result<(), SimulationError> {
        if let Some(task) = &self.task {
            return if task.is_finished() {
                Err(SimulationError::TaskRequiresStop)
            } else {
                Err(SimulationError::AlreadyRunning)
            };
        }

        let snapshot = {
            let simulation = self
                .simulation
                .as_ref()
                .ok_or(SimulationError::TaskRequiresStop)?;
            Arc::new(PhysicsSnapshot::from_world(
                simulation.world().map_err(SimulationError::from)?,
                simulation.metrics().total_steps,
                self.simulation_time,
            ))
        };
        let simulation = self
            .simulation
            .take()
            .ok_or(SimulationError::TaskRequiresStop)?;
        self.snapshot_tx.send_replace(snapshot);

        let (stop_tx, stop_rx) = watch::channel(false);
        let (command_sender, command_receiver) = mpsc::channel(COMMAND_QUEUE_CAPACITY);
        self.command_tx.send_replace(Some(command_sender));
        let mut simulation = simulation;
        simulation.set_runtime_status(SimulationState::Running, Duration::ZERO);
        let mut initial_metrics = simulation.metrics().clone();
        initial_metrics.requested_time_multiplier = *self.time_multiplier_tx.borrow();
        initial_metrics.actual_time_multiplier = 0.0;
        initial_metrics.load_speed_factor = 1.0;
        initial_metrics.adaptive_step_limited = false;
        self.metrics_tx.send_replace(initial_metrics);

        let metrics_tx = self.metrics_tx.clone();
        let command_state_tx = self.command_tx.clone();
        let snapshot_tx = self.snapshot_tx.clone();
        let events = SimulationEvents {
            collisions: self.collision_events_tx.clone(),
            joint_breaks: self.joint_break_events_tx.clone(),
            batch_results: self.batch_results_tx.clone(),
        };
        let mut simulation_time = self.simulation_time;
        let buffer_pool = Arc::clone(&self.buffer_pool);
        let time_multiplier = self.time_multiplier_tx.subscribe();
        self.stop = Some(stop_tx);
        self.task = Some(self.runtime.spawn(async move {
            let result = run_simulation(
                &mut simulation,
                CommandQueue::new(command_receiver, buffer_pool),
                RuntimeControl {
                    stop: stop_rx,
                    time_multiplier,
                },
                &metrics_tx,
                &snapshot_tx,
                &events,
                &mut simulation_time,
            )
            .await;
            command_state_tx.send_replace(None);
            let state = if result.is_ok() {
                SimulationState::Stopped
            } else {
                SimulationState::Error
            };
            simulation.set_runtime_status(state, Duration::ZERO);
            metrics_tx.send_modify(|metrics| {
                let requested_multiplier = metrics.requested_time_multiplier;
                let actual_multiplier = metrics.actual_time_multiplier;
                let adaptive_step_limited = metrics.adaptive_step_limited;
                let load_speed_factor = metrics.load_speed_factor;
                *metrics = simulation.metrics().clone();
                metrics.requested_time_multiplier = requested_multiplier;
                metrics.actual_time_multiplier = actual_multiplier;
                metrics.adaptive_step_limited = adaptive_step_limited;
                metrics.load_speed_factor = load_speed_factor;
            });
            ServiceTaskOutput {
                simulation,
                simulation_time,
                result,
            }
        }));
        Ok(())
    }

    /// Stops scheduling steps and waits for any active step to finish.
    pub async fn stop(&mut self) -> Result<(), SimulationError> {
        let Some(task) = &mut self.task else {
            return Ok(());
        };
        self.command_tx.send_replace(None);
        if let Some(stop) = &self.stop {
            stop.send_replace(true);
        }
        let joined = task.await;
        self.task = None;
        self.stop = None;
        match joined {
            Ok(output) => {
                self.simulation = Some(output.simulation);
                self.simulation_time = output.simulation_time;
                output.result
            }
            Err(error) => {
                self.metrics_tx.send_modify(|metrics| {
                    metrics.state = SimulationState::Error;
                });
                Err(SimulationError::TaskFailed(error.to_string()))
            }
        }
    }

    pub fn state(&self) -> SimulationState {
        self.metrics().state
    }

    pub fn metrics(&self) -> SimulationMetrics {
        self.metrics_rx.borrow().clone()
    }

    /// Changes the requested simulation-time rate without changing timestep policy.
    ///
    /// Zero pauses simulated-time accumulation; positive values request that
    /// many simulated seconds per wall second. `Fixed` keeps `dt` at
    /// `1 / tick_rate`; `Adaptive` scales the target interval and substeps it
    /// according to physics bounds. Achieved scaling is reported in
    /// [`SimulationMetrics::actual_time_multiplier`].
    pub fn set_time_multiplier(&self, multiplier: f64) -> Result<(), SimulationError> {
        if !multiplier.is_finite() || multiplier < 0.0 {
            return Err(SimulationError::InvalidTimeMultiplier);
        }
        self.time_multiplier_tx.send_replace(multiplier);
        self.metrics_tx.send_modify(|metrics| {
            metrics.requested_time_multiplier = multiplier;
            metrics.actual_time_multiplier = 0.0;
            metrics.adaptive_step_limited = false;
        });
        Ok(())
    }

    /// Pauses simulated time, retaining the requested speed for resuming.
    pub fn pause(&mut self) -> Result<(), SimulationError> {
        if self.paused_multiplier.is_none() {
            self.paused_multiplier = Some(*self.time_multiplier_tx.borrow());
            self.set_time_multiplier(0.0)?;
        }
        Ok(())
    }

    /// Resumes at the speed requested before pausing.
    pub fn resume(&mut self) -> Result<(), SimulationError> {
        if let Some(multiplier) = self.paused_multiplier.take() {
            self.set_time_multiplier(multiplier)?;
        }
        Ok(())
    }

    pub fn client(&self) -> PhysicsClient {
        PhysicsClient::new(
            self.command_rx.clone(),
            self.snapshot_rx.clone(),
            self.collision_events_tx.clone(),
            self.joint_break_events_tx.clone(),
            self.batch_results_tx.clone(),
            Arc::clone(&self.batch_id_sequence),
            Arc::clone(&self.buffer_pool),
        )
    }
}

impl Drop for Simulation {
    fn drop(&mut self) {
        self.command_tx.send_replace(None);
        if let Some(stop) = &self.stop {
            stop.send_replace(true);
        }
    }
}

fn stopping(stop: &watch::Receiver<bool>) -> bool {
    *stop.borrow() || stop.has_changed().is_err()
}

async fn run_simulation(
    simulation: &mut StepRunner,
    mut commands: CommandQueue,
    controls: RuntimeControl,
    metrics: &watch::Sender<SimulationMetrics>,
    snapshots: &watch::Sender<Arc<PhysicsSnapshot>>,
    events: &SimulationEvents,
    simulation_time: &mut Duration,
) -> Result<(), SimulationError> {
    let result = run_simulation_loop(
        simulation,
        &mut commands,
        controls,
        metrics,
        snapshots,
        events,
        simulation_time,
    )
    .await;
    commands.receiver.close();
    let stopped_at_step = simulation.metrics().total_steps;
    if let Some(request) = commands.pending.take() {
        reject_request(request, stopped_at_step, events, &commands.buffer_pool);
    }
    while let Ok(request) = commands.receiver.try_recv() {
        reject_request(request, stopped_at_step, events, &commands.buffer_pool);
    }
    result
}

async fn run_simulation_loop(
    simulation: &mut StepRunner,
    commands: &mut CommandQueue,
    controls: RuntimeControl,
    metrics: &watch::Sender<SimulationMetrics>,
    snapshots: &watch::Sender<Arc<PhysicsSnapshot>>,
    events: &SimulationEvents,
    simulation_time: &mut Duration,
) -> Result<(), SimulationError> {
    let RuntimeControl {
        mut stop,
        mut time_multiplier,
    } = controls;
    let step_duration = simulation.config().step_budget();
    let adaptive_policy = matches!(
        simulation.config().time_step_policy,
        TimeStepPolicy::Adaptive(_)
    );
    let max_catch_up_steps = simulation.config().max_catch_up_steps;
    let mut previous = Instant::now();
    let mut multiplier_started_at = previous;
    let mut multiplier_started_simulation_time = *simulation_time;
    let mut current_multiplier = *time_multiplier.borrow_and_update();
    let mut load_speed = 1.0;
    let mut time_updates_open = true;
    let mut accumulator = Duration::ZERO;
    let mut reusable_snapshot = None;

    loop {
        if current_multiplier == 0.0 && commands.pending.is_some() {
            apply_queued_commands(simulation, commands)?;
            let step = simulation.metrics().total_steps;
            for result in &mut commands.pending_results {
                if let DeferredCommandResult::Batch(batch) = result {
                    batch.step = step;
                }
            }
            let world = simulation.world().map_err(SimulationError::from)?;
            let snapshot = PhysicsSnapshot::from_world(world, step, *simulation_time);
            snapshots.send_replace(Arc::new(snapshot));
            publish_command_results(commands, events);
            continue;
        }
        let (mut ready_step, mut step_limited) =
            choose_runtime_step(simulation, accumulator, current_multiplier, step_duration)?;
        if current_multiplier != 0.0 && step_limited && has_queued_request(commands) {
            apply_queued_commands(simulation, commands)?;
            (ready_step, step_limited) =
                choose_runtime_step(simulation, accumulator, current_multiplier, step_duration)?;
        }
        let has_ready_step = current_multiplier != 0.0
            && (ready_step.is_some()
                || (adaptive_policy && step_limited && commands.pending.is_some()));
        let remaining = if adaptive_policy {
            step_duration
        } else {
            step_duration.saturating_sub(accumulator)
        };
        let scheduler_multiplier = if adaptive_policy {
            if current_multiplier > 0.0 { 1.0 } else { 0.0 }
        } else {
            current_multiplier * load_speed
        };
        let step_deadline_reached = tokio::select! {
            biased;
            changed = stop.changed() => {
                if changed.is_err() || *stop.borrow() {
                    break;
                }
                false
            }
            changed = time_multiplier.changed(), if time_updates_open => {
                let now = Instant::now();
                add_scaled_elapsed(
                    &mut accumulator,
                    now.duration_since(previous),
                    current_multiplier * load_speed,
                )?;
                previous = now;
                match changed {
                    Ok(()) => {
                        current_multiplier = *time_multiplier.borrow_and_update();
                        multiplier_started_at = now;
                        multiplier_started_simulation_time = *simulation_time;
                    }
                    Err(_) => time_updates_open = false,
                }
                false
            }
            request = commands.receiver.recv(),
                if (current_multiplier == 0.0 || (adaptive_policy && step_limited)) && commands.pending.is_none() && !commands.receiver.is_closed() =>
            {
                let now = Instant::now();
                add_scaled_elapsed(
                    &mut accumulator,
                    now.duration_since(previous),
                    current_multiplier * load_speed,
                )?;
                previous = now;
                commands.pending = request;
                false
            }
            deadline_reached = wait_for_scheduler_turn(
                has_ready_step,
                remaining,
                scheduler_multiplier,
            ) => deadline_reached,
        };
        if stopping(&stop) {
            break;
        }
        if current_multiplier == 0.0 {
            continue;
        }

        let now = Instant::now();
        add_scaled_elapsed(
            &mut accumulator,
            now.duration_since(previous),
            current_multiplier * load_speed,
        )?;
        if step_deadline_reached && !adaptive_policy {
            accumulator = accumulator.max(step_duration);
        }
        previous = now;

        for _ in 0..max_catch_up_steps {
            if stopping(&stop) {
                break;
            }
            let (physics_step, limited) =
                choose_runtime_step(simulation, accumulator, current_multiplier, step_duration)?;
            let Some(physics_step) = physics_step else {
                if limited {
                    if has_queued_request(commands) {
                        apply_queued_commands(simulation, commands)?;
                    }
                    let (_, still_limited) = choose_runtime_step(
                        simulation,
                        accumulator,
                        current_multiplier,
                        step_duration,
                    )?;
                    if still_limited {
                        let completed_at = Instant::now();
                        let actual_multiplier = measured_time_multiplier(
                            *simulation_time,
                            multiplier_started_simulation_time,
                            completed_at.duration_since(multiplier_started_at),
                        );
                        simulation.set_runtime_status(
                            SimulationState::Running,
                            simulation_debt(accumulator, step_duration),
                        );
                        publish_runtime_metrics(
                            simulation,
                            metrics,
                            *time_multiplier.borrow(),
                            actual_multiplier,
                            true,
                            load_speed,
                        );
                    }
                }
                break;
            };
            let next_simulation_time = simulation_time
                .checked_add(physics_step)
                .ok_or(SimulationError::SimulationTimeOverflow)?;
            let attempted_step = simulation.metrics().total_steps.saturating_add(1);
            apply_queued_commands(simulation, commands)?;
            let step_result = if adaptive_policy {
                simulation.step_with_dt(physics_step.as_secs_f64()).await
            } else {
                simulation.step().await
            };
            if let Err(error) = step_result {
                publish_failed_command_results(
                    commands,
                    events,
                    attempted_step,
                    step_error_for_result(&error),
                );
                return Err(error.into());
            }
            *simulation_time = next_simulation_time;

            let (mut snapshot, _) = PhysicsSnapshot::take_reusable(&mut reusable_snapshot);
            let world = match simulation.world() {
                Ok(world) => world,
                Err(error) => {
                    publish_failed_command_results(
                        commands,
                        events,
                        attempted_step,
                        step_error_for_result(&error),
                    );
                    return Err(error.into());
                }
            };
            snapshot.rebuild(world, simulation.metrics().total_steps, *simulation_time);
            let previous_snapshot = snapshots.send_replace(Arc::new(snapshot));
            PhysicsSnapshot::recycle(previous_snapshot, &mut reusable_snapshot);
            publish_command_results(commands, events);
            if events.collisions.receiver_count() > 0 {
                let batch = CollisionEventBatch::new(
                    simulation.metrics().total_steps,
                    simulation
                        .world()
                        .map_err(SimulationError::from)?
                        .collision_events()
                        .to_vec(),
                );
                let _ = events.collisions.send(batch);
            }
            if events.joint_breaks.receiver_count() > 0 {
                let joint_events = simulation
                    .world()
                    .map_err(SimulationError::from)?
                    .joint_events();
                if !joint_events.is_empty() {
                    let batch = JointBreakEventBatch::new(
                        simulation.metrics().total_steps,
                        joint_events.to_vec(),
                    );
                    let _ = events.joint_breaks.send(batch);
                }
            }

            accumulator = accumulator.saturating_sub(physics_step);
            let completed_at = Instant::now();
            let actual_multiplier = measured_time_multiplier(
                *simulation_time,
                multiplier_started_simulation_time,
                completed_at.duration_since(multiplier_started_at),
            );
            add_scaled_elapsed(
                &mut accumulator,
                completed_at.duration_since(previous),
                current_multiplier * load_speed,
            )?;
            previous = completed_at;
            let measured = simulation
                .metrics()
                .average_step_duration
                .saturating_add(simulation.metrics().task_wait_duration);
            load_speed = load_speed_for_step(
                step_duration,
                measured,
                if adaptive_policy {
                    1.0
                } else {
                    current_multiplier
                },
            );
            let debt = simulation_debt(accumulator, step_duration);
            simulation.set_runtime_status(SimulationState::Running, debt);
            publish_runtime_metrics(
                simulation,
                metrics,
                *time_multiplier.borrow(),
                actual_multiplier,
                false,
                load_speed,
            );
        }

        if stopping(&stop) {
            break;
        }
    }

    Ok(())
}

fn load_speed_for_step(budget: Duration, average: Duration, multiplier: f64) -> f64 {
    if average.is_zero() {
        1.0
    } else {
        (budget.as_secs_f64() / average.as_secs_f64() / multiplier).clamp(MIN_LOAD_SPEED, 1.0)
    }
}

fn choose_runtime_step(
    simulation: &StepRunner,
    accumulated: Duration,
    multiplier: f64,
    fixed_step: Duration,
) -> Result<(Option<Duration>, bool), SimulationError> {
    match simulation.config().time_step_policy {
        TimeStepPolicy::Fixed => Ok(((accumulated >= fixed_step).then_some(fixed_step), false)),
        TimeStepPolicy::Adaptive(_) => {
            let target =
                Duration::try_from_secs_f64(simulation.config().fixed_step_seconds() * multiplier)
                    .map_err(|_| SimulationError::SimulationTimeOverflow)?;
            let world = simulation.world().map_err(SimulationError::from)?;
            Ok(
                match choose_step(world, simulation.config(), target, accumulated, fixed_step) {
                    StepChoice::Wait => (None, false),
                    StepChoice::Limited => (None, true),
                    StepChoice::Step(step) => (Some(step), false),
                },
            )
        }
    }
}

fn has_queued_request(commands: &mut CommandQueue) -> bool {
    if commands.pending.is_some() {
        return true;
    }
    match commands.receiver.try_recv() {
        Ok(request) => {
            commands.pending = Some(request);
            true
        }
        Err(mpsc::error::TryRecvError::Empty | mpsc::error::TryRecvError::Disconnected) => false,
    }
}

fn simulation_debt(accumulated: Duration, fixed_step: Duration) -> Duration {
    accumulated.saturating_sub(fixed_step)
}

fn apply_queued_commands(
    simulation: &mut StepRunner,
    commands: &mut CommandQueue,
) -> Result<(), SimulationError> {
    let step = simulation.metrics().total_steps.saturating_add(1);
    let world = simulation.world_mut().map_err(SimulationError::from)?;
    let mut processed_commands = 0;
    while processed_commands < MAX_COMMANDS_PER_STEP {
        let request = match commands.pending.take() {
            Some(request) => request,
            None => match commands.receiver.try_recv() {
                Ok(request) => request,
                Err(mpsc::error::TryRecvError::Empty | mpsc::error::TryRecvError::Disconnected) => {
                    break;
                }
            },
        };
        let command_count = request.command_count();
        if command_count > MAX_COMMANDS_PER_STEP - processed_commands {
            commands.pending = Some(request);
            break;
        }
        match request {
            CommandRequest::Chunk { command, reply } => {
                let outcome = apply_chunk(world, command);
                commands
                    .pending_results
                    .push(DeferredCommandResult::Chunk { reply, outcome });
            }
            CommandRequest::Single { command, reply } => {
                let is_read_only_query = matches!(
                    &command,
                    PhysicsCommand::Raycast { .. }
                        | PhysicsCommand::Sweep { .. }
                        | PhysicsCommand::Overlaps { .. }
                        | PhysicsCommand::GetMaterial { .. }
                        | PhysicsCommand::GetCluster { .. }
                        | PhysicsCommand::GetClusterPart { .. }
                        | PhysicsCommand::GetWeld { .. }
                );
                let result = apply_command(world, command);
                commands
                    .pending_results
                    .push(DeferredCommandResult::Single {
                        reply,
                        result,
                        is_read_only_query,
                    });
            }
            CommandRequest::Batch { id, buffer } => {
                let result =
                    apply_command_batch(world, &buffer.commands, &mut commands.batch_scratch);
                commands.buffer_pool.recycle(buffer);
                commands
                    .pending_results
                    .push(DeferredCommandResult::Batch(PhysicsBatchResult {
                        batch_id: id,
                        step,
                        result,
                    }));
            }
        }
        processed_commands += command_count;
    }
    Ok(())
}

fn publish_command_results(commands: &mut CommandQueue, events: &SimulationEvents) {
    for pending in commands.pending_results.drain(..) {
        match pending {
            DeferredCommandResult::Chunk { reply, outcome } => {
                let _ = reply.send(ChunkReply::Completed {
                    outcome,
                    step_failure: None,
                });
            }
            DeferredCommandResult::Single { reply, result, .. } => {
                let _ = reply.send(result);
            }
            DeferredCommandResult::Batch(result) => {
                let _ = events.batch_results.send(result);
            }
        }
    }
}

fn publish_failed_command_results(
    commands: &mut CommandQueue,
    events: &SimulationEvents,
    attempted_step: u64,
    cause: StepError,
) {
    for pending in commands.pending_results.drain(..) {
        match pending {
            DeferredCommandResult::Chunk { reply, outcome } => {
                let step_failure = outcome.applied().then_some((attempted_step, cause));
                let _ = reply.send(ChunkReply::Completed {
                    outcome,
                    step_failure,
                });
            }
            DeferredCommandResult::Single {
                reply,
                mut result,
                is_read_only_query,
            } => {
                if result.is_ok() && !is_read_only_query {
                    result = Err(PhysicsClientError::StepFailedAfterApply {
                        step: attempted_step,
                        cause,
                    });
                }
                let _ = reply.send(result);
            }
            DeferredCommandResult::Batch(mut result) => {
                result.step = attempted_step;
                if result.result.is_ok() {
                    result.result = Err(PhysicsBatchError {
                        command_index: None,
                        error: PhysicsClientError::StepFailedAfterApply {
                            step: attempted_step,
                            cause,
                        },
                    });
                }
                let _ = events.batch_results.send(result);
            }
        }
    }
}

fn step_error_for_result(error: &StepRunnerError) -> StepError {
    match error {
        StepRunnerError::Physics(error) => *error,
        StepRunnerError::InvalidConfig(_)
        | StepRunnerError::InvalidWorkerConfig(_)
        | StepRunnerError::RayonPool(_)
        | StepRunnerError::WorldUnavailable
        | StepRunnerError::WorldLost => StepError::Collision(CollisionError::WorkerFailed),
    }
}

fn reject_request(
    request: CommandRequest,
    step: u64,
    events: &SimulationEvents,
    buffer_pool: &CommandBufferPool,
) {
    match request {
        CommandRequest::Chunk { command, reply } => {
            let _ = reply.send(ChunkReply::NotSubmitted {
                command,
                cause: PhysicsClientError::NotRunning,
            });
        }
        CommandRequest::Single { reply, .. } => {
            let _ = reply.send(Err(PhysicsClientError::NotRunning));
        }
        CommandRequest::Batch { id, buffer } => {
            buffer_pool.recycle(buffer);
            let _ = events.batch_results.send(PhysicsBatchResult {
                batch_id: id,
                step,
                result: Err(PhysicsBatchError {
                    command_index: None,
                    error: PhysicsClientError::NotRunning,
                }),
            });
        }
    }
}

fn add_elapsed(accumulator: &mut Duration, elapsed: Duration) -> Result<(), SimulationError> {
    *accumulator = accumulator
        .checked_add(elapsed)
        .ok_or(SimulationError::SimulationTimeOverflow)?;
    Ok(())
}

fn add_scaled_elapsed(
    accumulator: &mut Duration,
    elapsed: Duration,
    multiplier: f64,
) -> Result<(), SimulationError> {
    let scaled_seconds = elapsed.as_secs_f64() * multiplier;
    let scaled = Duration::try_from_secs_f64(scaled_seconds)
        .map_err(|_| SimulationError::SimulationTimeOverflow)?;
    add_elapsed(accumulator, scaled)
}

fn measured_time_multiplier(
    simulation_time: Duration,
    start_simulation_time: Duration,
    wall_elapsed: Duration,
) -> f64 {
    if wall_elapsed.is_zero() {
        0.0
    } else {
        simulation_time
            .saturating_sub(start_simulation_time)
            .as_secs_f64()
            / wall_elapsed.as_secs_f64()
    }
}

async fn wait_for_scaled_step(remaining: Duration, multiplier: f64) {
    if multiplier == 0.0 {
        std::future::pending::<()>().await;
    } else {
        tokio::time::sleep(scaled_step_delay(remaining, multiplier)).await;
    }
}

async fn wait_for_scheduler_turn(
    has_ready_step: bool,
    remaining: Duration,
    multiplier: f64,
) -> bool {
    if has_ready_step {
        tokio::task::yield_now().await;
        false
    } else {
        wait_for_scaled_step(remaining, multiplier).await;
        true
    }
}

fn scaled_step_delay(remaining: Duration, multiplier: f64) -> Duration {
    let seconds = remaining.as_secs_f64() / multiplier;
    match Duration::try_from_secs_f64(seconds) {
        Ok(delay) => delay,
        Err(_) => Duration::from_secs(86_400),
    }
}

fn publish_runtime_metrics(
    simulation: &StepRunner,
    metrics: &watch::Sender<SimulationMetrics>,
    requested_multiplier: f64,
    actual_multiplier: f64,
    adaptive_step_limited: bool,
    load_speed: f64,
) {
    let mut snapshot = simulation.metrics().clone();
    snapshot.requested_time_multiplier = requested_multiplier;
    snapshot.actual_time_multiplier = actual_multiplier;
    snapshot.adaptive_step_limited = adaptive_step_limited;
    snapshot.load_speed_factor = load_speed;
    metrics.send_replace(snapshot);
}

#[cfg(test)]
#[path = "service/chunk_tests.rs"]
mod chunk_tests;

#[cfg(test)]
mod tests {
    #[tokio::test]
    async fn builder_installs_atmosphere_on_both_executors()
    -> Result<(), Box<dyn std::error::Error>> {
        use kapitsa_core::aerodynamics::{Atmosphere, DensityProfile, SphericalAtmosphere};
        let atmosphere = Atmosphere::Sphere(
            SphericalAtmosphere::builder()
                .reference_radius(100.0)
                .reference_density(1.0)
                .height(20.0)
                .profile(DensityProfile::Exponential { scale_height: 10.0 })
                .build()?,
        );
        assert_eq!(
            super::SimulationBuilder::default().atmosphere,
            Atmosphere::Vacuum
        );
        for rayon in [false, true] {
            let builder = super::Simulation::builder()
                .atmosphere(atmosphere)
                .mesh_sound_speed(295.0);
            let mut simulation = if rayon {
                builder.spawn_on_rayon(std::sync::Arc::new(
                    rayon::ThreadPoolBuilder::new().num_threads(1).build()?,
                ))?
            } else {
                builder.spawn_on_tokio(&tokio::runtime::Handle::current())?
            };
            simulation.stop().await?;
            let runner = simulation
                .simulation
                .as_ref()
                .ok_or("missing stopped simulation")?;
            assert_eq!(runner.world()?.atmosphere(), atmosphere);
            assert_eq!(runner.world()?.mesh_sound_speed(), 295.0);
        }
        Ok(())
    }

    #[tokio::test]
    async fn builder_rejects_invalid_sound_speed_on_both_executors()
    -> Result<(), Box<dyn std::error::Error>> {
        let pool = std::sync::Arc::new(rayon::ThreadPoolBuilder::new().num_threads(1).build()?);
        for invalid in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            for rayon in [false, true] {
                let builder = super::Simulation::builder().mesh_sound_speed(invalid);
                let result = if rayon {
                    builder.spawn_on_rayon(pool.clone())
                } else {
                    builder.spawn_on_tokio(&tokio::runtime::Handle::current())
                };
                assert!(matches!(
                    result,
                    Err(super::SimulationError::InvalidMeshSoundSpeed(_))
                ));
            }
        }
        Ok(())
    }

    use std::{
        future::Future,
        num::NonZeroUsize,
        pin::Pin,
        sync::mpsc,
        task::{Context, Poll, Waker},
        time::Duration,
    };

    use kapitsa_core::{
        collider::Collider,
        collision::{QueryFilter, Ray},
        gravity::CentralGravity,
        joint::{FixedJoint, Joint, JointCommon, JointFrame},
        mass::MassProperties,
        rigid_body::RigidBody,
        shape::{Shape, Sphere},
        transform::Transform,
        world::{StepError, WorldError},
    };
    use kapitsa_math::{Mat3, Vec3};
    use tokio::{sync::oneshot, time::sleep};

    use crate::runtime::{
        AdaptiveTimeStepConfig, PhysicsClientError, SimulationConfig, SimulationError,
        SimulationState, TimeStepPolicy, WorkerConfig,
    };

    use super::{
        PhysicsBatchError, Simulation, add_elapsed, add_scaled_elapsed, load_speed_for_step,
        measured_time_multiplier, wait_for_scaled_step, wait_for_scheduler_turn,
    };

    fn dynamic_body() -> RigidBody {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass");
        RigidBody::new_dynamic(Transform::identity(), mass).expect("dynamic body")
    }

    #[test]
    fn overload_slowdown_keeps_fixed_step_and_accumulates_below_floor() {
        let step = Duration::from_millis(10);
        assert_eq!(
            load_speed_for_step(step, Duration::from_millis(5), 1.0),
            1.0
        );
        assert_eq!(
            load_speed_for_step(step, Duration::from_millis(20), 1.0),
            0.5
        );
        assert_eq!(
            load_speed_for_step(step, Duration::from_millis(100), 1.0),
            0.3
        );
        assert_eq!(
            load_speed_for_step(step, Duration::from_millis(10), 2.0),
            0.5
        );
        let mut elapsed = Duration::ZERO;
        add_scaled_elapsed(&mut elapsed, Duration::from_millis(100), 0.3).unwrap();
        assert_eq!(elapsed - step, Duration::from_millis(20));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn overloaded_step_keeps_dt_and_reports_speed_floor() {
        let mut simulation = Simulation::new_from_config(config(100)).unwrap();
        let (entered_tx, entered_rx) = oneshot::channel();
        let (release_tx, release_rx) = mpsc::channel();
        simulation
            .simulation
            .as_mut()
            .unwrap()
            .gate_next_step(entered_tx, release_rx);
        simulation.start().unwrap();
        entered_rx.await.unwrap();
        sleep(Duration::from_millis(80)).await;
        simulation.pause().unwrap();
        release_tx.send(()).unwrap();
        tokio::time::timeout(Duration::from_secs(2), async {
            while simulation.metrics().total_steps == 0 {
                tokio::task::yield_now().await;
            }
        })
        .await
        .unwrap();
        let metrics = simulation.metrics();
        assert_eq!(metrics.last_physics_step, Duration::from_millis(10));
        assert_eq!(metrics.load_speed_factor, 0.3);
        simulation.stop().await.unwrap();
    }

    fn config(tick_rate: u32) -> SimulationConfig {
        let workers = WorkerConfig {
            max_workers: NonZeroUsize::new(2),
            ..WorkerConfig::default()
        };
        SimulationConfig {
            tick_rate,
            workers,
            ..SimulationConfig::default()
        }
    }

    fn poll_once<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
        future.poll(&mut Context::from_waker(Waker::noop()))
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn paused_commands_publish_without_advancing_time() {
        let mut simulation = Simulation::new_from_config(config(120)).unwrap();
        simulation.pause().unwrap();
        simulation.start().unwrap();
        let client = simulation.client();
        let id = tokio::time::timeout(Duration::from_secs(2), client.add_body(dynamic_body()))
            .await
            .unwrap()
            .unwrap();
        let snapshot = client.snapshot();
        assert!(snapshot.body(id).is_some());
        assert_eq!(snapshot.step(), 0);
        assert_eq!(snapshot.simulation_time(), Duration::ZERO);
        let gravity = client
            .add_gravity(kapitsa_core::gravity::UniformGravity::new(Vec3::Y).unwrap())
            .await
            .unwrap();
        client
            .update_gravity(
                gravity,
                kapitsa_core::gravity::UniformGravity::new(Vec3::X).unwrap(),
            )
            .await
            .unwrap();
        client.remove_gravity(gravity).await.unwrap();
        assert!(client.remove_gravity(gravity).await.is_err());
        assert_eq!(client.snapshot().step(), 0);
        simulation.resume().unwrap();
        tokio::time::timeout(Duration::from_secs(2), async {
            while simulation.metrics().total_steps == 0 {
                tokio::task::yield_now().await;
            }
        })
        .await
        .unwrap();
        simulation.stop().await.unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn client_gravity_changes_motion_after_resuming() {
        let mut simulation = Simulation::new_from_config(config(100)).unwrap();
        simulation.pause().unwrap();
        simulation.start().unwrap();
        let mut client = simulation.client();
        let id = client.add_body(dynamic_body()).await.unwrap();
        client
            .add_gravity(kapitsa_core::gravity::UniformGravity::new(Vec3::Y).unwrap())
            .await
            .unwrap();
        simulation.resume().unwrap();
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let state = client.next_snapshot().await.unwrap();
                if state.body(id).unwrap().linear_velocity().y > 0.0 {
                    break;
                }
            }
        })
        .await
        .unwrap();
        simulation.stop().await.unwrap();
    }

    #[tokio::test]
    async fn builder_rejects_zero_worker_limit() {
        let result = Simulation::builder()
            .max_workers(0)
            .spawn_on_tokio(&tokio::runtime::Handle::current());
        assert!(matches!(result, Err(SimulationError::InvalidWorkerCount)));
    }

    #[tokio::test]
    async fn builder_rejects_zero_frequency() {
        let result = Simulation::builder()
            .frequency(0)
            .spawn_on_tokio(&tokio::runtime::Handle::current());
        assert!(matches!(result, Err(SimulationError::InvalidConfig(_))));
    }

    #[tokio::test]
    async fn resume_restores_requested_time_multiplier() {
        let mut simulation = Simulation::new_from_config(config(120)).unwrap();
        simulation.set_time_multiplier(2.0).unwrap();
        simulation.pause().unwrap();
        simulation.pause().unwrap();
        assert_eq!(simulation.metrics().requested_time_multiplier, 0.0);
        simulation.resume().unwrap();
        assert_eq!(simulation.metrics().requested_time_multiplier, 2.0);
    }

    #[test]
    fn rayon_builder_starts_without_a_tokio_runtime() {
        let pool = std::sync::Arc::new(
            rayon::ThreadPoolBuilder::new()
                .num_threads(2)
                .build()
                .unwrap(),
        );
        let mut simulation = Simulation::builder()
            .frequency(120)
            .max_workers(1)
            .spawn_on_rayon(pool.clone())
            .unwrap();
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .unwrap();
        runtime.block_on(async {
            let client = simulation.client();
            let id = tokio::time::timeout(Duration::from_secs(2), client.add_body(dynamic_body()))
                .await
                .unwrap()
                .unwrap();
            assert!(client.snapshot().body(id).is_some());
            simulation.stop().await.unwrap();
        });
        assert_eq!(pool.install(|| 2 + 2), 4);
    }

    #[test]
    fn simulations_share_rayon_pool_without_sharing_lifecycle() {
        let pool = std::sync::Arc::new(
            rayon::ThreadPoolBuilder::new()
                .num_threads(2)
                .build()
                .unwrap(),
        );
        let mut first = Simulation::builder()
            .max_workers(1)
            .spawn_on_rayon(pool.clone())
            .unwrap();
        let mut second = Simulation::builder()
            .max_workers(2)
            .spawn_on_rayon(pool.clone())
            .unwrap();
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .unwrap();
        runtime.block_on(async {
            let id = first.client().add_body(dynamic_body()).await.unwrap();
            assert!(first.client().snapshot().body(id).is_some());
            first.stop().await.unwrap();
            let id = tokio::time::timeout(
                Duration::from_secs(2),
                second.client().add_body(dynamic_body()),
            )
            .await
            .unwrap()
            .unwrap();
            assert!(second.client().snapshot().body(id).is_some());
            second.stop().await.unwrap();
        });
        assert_eq!(pool.install(|| 7), 7);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn simulation_start_stop_restart_and_client_commands_work() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        let client = simulation.client();
        simulation.start().expect("start");
        assert_eq!(simulation.state(), SimulationState::Running);
        let id = client.add_body(dynamic_body()).await.expect("add body");
        client.apply_force(id, Vec3::X).await.expect("apply force");
        sleep(Duration::from_millis(35)).await;
        simulation.stop().await.expect("stop");
        assert_eq!(simulation.state(), SimulationState::Stopped);
        assert!(simulation.metrics().total_steps >= 1);
        assert!(client.snapshot().body(id).is_some());

        let previous_steps = simulation.metrics().total_steps;
        simulation.start().expect("restart");
        sleep(Duration::from_millis(25)).await;
        simulation.stop().await.expect("second stop");
        assert!(simulation.metrics().total_steps > previous_steps);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn runtime_joint_commands_publish_break_events_after_the_snapshot() {
        let mut simulation = Simulation::new_from_config(config(500)).expect("simulation");
        let client = simulation.client();
        let mut breaks = client.subscribe_joint_break_events();
        let mut results = client.subscribe_batch_results();
        simulation.start().expect("start");

        let body_a = client.add_body(dynamic_body()).await.expect("body A");
        let body_b = client.add_body(dynamic_body()).await.expect("body B");
        let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        common.break_force = Some(100.0);
        let joint = client
            .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
            .await
            .expect("joint");

        let mut commands = client.commands();
        commands.set_joint_enabled(joint, false);
        commands.update_joint(joint, Joint::Fixed(FixedJoint::new(common)));
        commands.set_joint_enabled(joint, true);
        let batch_id = client.try_submit(commands).expect("joint batch accepted");
        let batch = results.next_result().await.expect("joint batch result");
        assert_eq!(batch.batch_id, batch_id);
        assert_eq!(batch.result, Ok(()));

        client
            .set_linear_velocity(body_b, Vec3::X * 10.0)
            .await
            .expect("load joint");
        let event_batch = tokio::time::timeout(Duration::from_secs(1), breaks.next_batch())
            .await
            .expect("break event arrives")
            .expect("break event stream");
        assert_eq!(event_batch.events().len(), 1);
        assert_eq!(event_batch.events()[0].joint(), joint);
        assert!(client.snapshot().step() >= event_batch.step());
        assert!(matches!(
            client.remove_joint(joint).await,
            Err(PhysicsClientError::World(WorldError::InvalidJointId))
        ));
        simulation.stop().await.expect("stop");
    }

    #[tokio::test(start_paused = true)]
    async fn client_rejects_commands_before_simulation_starts() {
        let simulation = Simulation::new_from_config(config(60)).expect("simulation");
        assert!(matches!(
            simulation
                .client()
                .add_body(RigidBody::new_static(Transform::identity()))
                .await,
            Err(crate::runtime::PhysicsClientError::NotRunning)
        ));
    }

    #[tokio::test(start_paused = true)]
    async fn stop_completes_when_called_before_the_first_tick() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        simulation.start().expect("start");
        tokio::time::advance(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;
        simulation.stop().await.expect("stop");
        assert_eq!(simulation.state(), SimulationState::Stopped);
        assert_eq!(simulation.metrics().total_steps, 0);
    }

    #[tokio::test(start_paused = true)]
    async fn stopped_queue_batches_are_recycled_and_available_after_restart() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        let body = simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .world_mut()
            .expect("world")
            .add_body(dynamic_body())
            .expect("body ID");
        let client = simulation.client();
        let mut results = client.subscribe_batch_results();
        simulation.start().expect("start");

        let mut buffer = client.commands();
        buffer.reserve(15);
        buffer.set_linear_velocity(body, Vec3::X);
        let capacity = buffer.commands.capacity();
        let batch_id = client.try_submit(buffer).expect("accepted batch");
        simulation.stop().await.expect("stop");

        let result = results.next_result().await.expect("rejected batch result");
        assert_eq!(result.batch_id, batch_id);
        assert!(result.result.is_err());
        assert_eq!(simulation.buffer_pool.stats().0, 1);

        simulation.start().expect("restart");
        let reused = client.commands();
        assert!(reused.is_empty());
        assert_eq!(reused.commands.capacity(), capacity);
        drop(reused);
        simulation.stop().await.expect("second stop");
    }

    #[tokio::test(start_paused = true)]
    async fn applied_and_failed_batches_are_recycled() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        let body = simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .world_mut()
            .expect("world")
            .add_body(dynamic_body())
            .expect("body ID");
        let client = simulation.client();
        let mut results = client.subscribe_batch_results();
        simulation.start().expect("start");
        tokio::task::yield_now().await;

        let mut first = client.commands();
        first.reserve(15);
        first.set_linear_velocity(body, Vec3::X);
        let first_capacity = first.commands.capacity();
        let first_id = client.try_submit(first).expect("accepted batch");
        tokio::time::advance(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;
        let result = results.next_result().await.expect("applied batch result");
        assert_eq!(result.batch_id, first_id);
        assert!(result.result.is_ok());
        assert_eq!(simulation.buffer_pool.stats().0, 1);

        let mut second = client.commands();
        assert_eq!(second.commands.capacity(), first_capacity);
        second.set_linear_velocity(body, Vec3::new(f64::NAN, 0.0, 0.0));
        let second_id = client.try_submit(second).expect("accepted invalid batch");
        tokio::time::advance(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;
        let result = results.next_result().await.expect("failed batch result");
        assert_eq!(result.batch_id, second_id);
        assert!(result.result.is_err());
        assert_eq!(simulation.buffer_pool.stats().0, 1);

        simulation.stop().await.expect("stop");
    }

    #[tokio::test(start_paused = true)]
    async fn batch_results_follow_fifo_and_wait_for_snapshot_publication() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        let body = simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .world_mut()
            .expect("world")
            .add_body(dynamic_body())
            .expect("body ID");
        let client = simulation.client();
        let mut results = client.subscribe_batch_results();
        let (entered_tx, entered_rx) = oneshot::channel();
        let (release_tx, release_rx) = mpsc::channel();
        simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .gate_next_step(entered_tx, release_rx);
        simulation.start().expect("start");
        tokio::task::yield_now().await;

        let mut valid = client.commands();
        valid.set_linear_velocity(body, Vec3::Y);
        let valid_id = client.try_submit(valid).expect("valid batch");
        let mut invalid = client.commands();
        invalid.set_linear_velocity(body, Vec3::X);
        invalid.set_linear_velocity(body, Vec3::new(f64::NAN, 0.0, 0.0));
        let invalid_id = client.try_submit(invalid).expect("invalid batch");

        tokio::time::advance(Duration::from_millis(10)).await;
        entered_rx.await.expect("step worker entered gate");
        assert_eq!(client.snapshot().step(), 0);
        assert_eq!(
            client
                .snapshot()
                .body(body)
                .expect("initial body")
                .linear_velocity(),
            Vec3::ZERO
        );
        tokio::select! {
            biased;
            result = results.next_result() => panic!("batch result arrived before step completion: {result:?}"),
            _ = tokio::task::yield_now() => {}
        }

        release_tx.send(()).expect("release step worker");
        let first = results.next_result().await.expect("first batch result");
        let second = results.next_result().await.expect("second batch result");
        assert_eq!((first.batch_id, second.batch_id), (valid_id, invalid_id));
        assert!(first.result.is_ok());
        assert_eq!(
            second.result,
            Err(PhysicsBatchError {
                command_index: Some(1),
                error: PhysicsClientError::Body(
                    kapitsa_core::rigid_body::BodyError::NonFiniteValue
                ),
            })
        );
        assert_eq!(first.step, second.step);
        assert!(client.snapshot().step() >= first.step);
        assert_eq!(
            client
                .snapshot()
                .body(body)
                .expect("published body")
                .linear_velocity(),
            Vec3::Y
        );
        simulation.stop().await.expect("stop");
    }

    #[tokio::test(start_paused = true)]
    async fn cancelling_stop_during_step_preserves_applied_batch_completion() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        let body = simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .world_mut()
            .expect("world")
            .add_body(dynamic_body())
            .expect("body ID");
        let client = simulation.client();
        let mut results = client.subscribe_batch_results();
        let (entered_tx, entered_rx) = oneshot::channel();
        let (release_tx, release_rx) = mpsc::channel();
        simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .gate_next_step(entered_tx, release_rx);
        simulation.start().expect("start");
        tokio::task::yield_now().await;

        let mut buffer = client.commands();
        buffer.set_linear_velocity(body, Vec3::X);
        let batch_id = client.try_submit(buffer).expect("accepted batch");
        tokio::time::advance(Duration::from_millis(10)).await;
        entered_rx.await.expect("step worker entered gate");

        let mut stop = Box::pin(simulation.stop());
        tokio::select! {
            biased;
            result = &mut stop => panic!("stop returned while its step was gated: {result:?}"),
            _ = tokio::task::yield_now() => {}
        }
        drop(stop);
        assert!(matches!(
            client
                .add_body(RigidBody::new_static(Transform::identity()))
                .await,
            Err(PhysicsClientError::NotRunning)
        ));

        release_tx.send(()).expect("release step worker");
        simulation.stop().await.expect("join cancelled stop");
        let result = results.next_result().await.expect("applied batch result");
        assert_eq!(result.batch_id, batch_id);
        assert!(result.result.is_ok());
        assert!(client.snapshot().step() >= result.step);
    }

    #[tokio::test(start_paused = true)]
    async fn applied_batch_reports_step_failure_without_rolling_back_commands() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        let tiny_mass =
            MassProperties::new(f64::from_bits(1), Vec3::ZERO, Mat3::IDENTITY).expect("tiny mass");
        let body = simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .world_mut()
            .expect("world")
            .add_body(RigidBody::new_dynamic(Transform::identity(), tiny_mass).expect("body"))
            .expect("body ID");
        let client = simulation.client();
        let mut results = client.subscribe_batch_results();
        let (entered_tx, entered_rx) = oneshot::channel();
        let (release_tx, release_rx) = mpsc::channel();
        simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .gate_next_step(entered_tx, release_rx);
        simulation.start().expect("start");
        tokio::task::yield_now().await;

        let mut buffer = client.commands();
        buffer.set_linear_velocity(body, Vec3::Y);
        buffer.apply_force(body, Vec3::X * f64::MAX);
        let batch_id = client.try_submit(buffer).expect("accepted batch");
        tokio::time::advance(Duration::from_millis(10)).await;
        entered_rx.await.expect("step worker entered gate");
        assert_eq!(simulation.buffer_pool.stats().0, 1);
        release_tx.send(()).expect("release step worker");

        let result = results.next_result().await.expect("failed step result");
        assert_eq!(result.batch_id, batch_id);
        assert_eq!(result.step, 1);
        assert_eq!(
            result.result,
            Err(PhysicsBatchError {
                command_index: None,
                error: PhysicsClientError::StepFailedAfterApply {
                    step: 1,
                    cause: StepError::BodyIntegration {
                        body_id: body,
                        cause: kapitsa_core::rigid_body::IntegrationError::ArithmeticOverflow,
                    },
                },
            })
        );
        assert!(matches!(
            simulation.stop().await,
            Err(SimulationError::Physics(StepError::BodyIntegration { .. }))
        ));
        assert_eq!(client.snapshot().step(), 0);
        assert_eq!(
            simulation
                .simulation
                .as_ref()
                .expect("runner returned after failed step")
                .world()
                .expect("world restored")
                .body(body)
                .expect("body")
                .state()
                .linear_velocity(),
            Vec3::Y
        );
    }

    #[tokio::test(start_paused = true)]
    async fn single_mutations_wait_for_snapshot_and_report_failed_step() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        let tiny_mass =
            MassProperties::new(f64::from_bits(1), Vec3::ZERO, Mat3::IDENTITY).expect("tiny mass");
        let body = simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .world_mut()
            .expect("world")
            .add_body(RigidBody::new_dynamic(Transform::identity(), tiny_mass).expect("body"))
            .expect("body ID");
        let world = simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .world_mut()
            .expect("world");
        let fixed = world
            .add_body(RigidBody::new_static(
                Transform::new(Vec3::X * 4.0, kapitsa_math::Quat::IDENTITY).expect("transform"),
            ))
            .expect("fixed body");
        world
            .add_collider(
                fixed,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                    Transform::identity(),
                ),
            )
            .expect("fixed collider");
        let material = world
            .add_material(kapitsa_core::material::PhysicsMaterial::default())
            .unwrap();
        let root = world
            .enable_cluster_with_mass(
                fixed,
                MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
            )
            .unwrap();
        let client = simulation.client();
        let (entered_tx, entered_rx) = oneshot::channel();
        let (release_tx, release_rx) = mpsc::channel();
        simulation
            .simulation
            .as_mut()
            .expect("simulation runner")
            .gate_next_step(entered_tx, release_rx);
        simulation.start().expect("start");
        tokio::task::yield_now().await;

        let mut velocity = Box::pin(client.set_linear_velocity(body, Vec3::Y));
        assert!(matches!(poll_once(velocity.as_mut()), Poll::Pending));
        let mut force = Box::pin(client.apply_force(body, Vec3::X * f64::MAX));
        assert!(matches!(poll_once(force.as_mut()), Poll::Pending));
        let ray = Ray::new(Vec3::new(4.0, 0.0, -2.0), Vec3::Z, 10.0).expect("ray");
        let mut query = Box::pin(client.raycast(ray, QueryFilter::default()));
        assert!(matches!(poll_once(query.as_mut()), Poll::Pending));
        let mut material_query = Box::pin(client.material(material));
        assert!(matches!(poll_once(material_query.as_mut()), Poll::Pending));
        let mut part_query = Box::pin(client.cluster_part(root));
        assert!(matches!(poll_once(part_query.as_mut()), Poll::Pending));
        tokio::time::advance(Duration::from_millis(10)).await;
        entered_rx.await.expect("step worker entered gate");
        assert_eq!(client.snapshot().step(), 0);
        tokio::select! {
            biased;
            result = &mut velocity => panic!("mutation completed before the step: {result:?}"),
            _ = tokio::task::yield_now() => {}
        }
        release_tx.send(()).expect("release step worker");

        let velocity_error = velocity.await.expect_err("failed step is reported");
        let force_error = force.await.expect_err("failed step is reported");
        assert!(matches!(
            velocity_error,
            PhysicsClientError::StepFailedAfterApply {
                step: 1,
                cause: StepError::BodyIntegration { body_id, .. },
            } if body_id == body
        ));
        assert!(matches!(
            force_error,
            PhysicsClientError::StepFailedAfterApply {
                step: 1,
                cause: StepError::BodyIntegration { body_id, .. },
            } if body_id == body
        ));
        assert!(
            query
                .await
                .expect("query result remains distinct")
                .is_some()
        );
        assert_eq!(
            material_query.await.unwrap(),
            Some(kapitsa_core::material::PhysicsMaterial::default())
        );
        assert_eq!(part_query.await.unwrap().unwrap().body(), fixed);
        assert!(matches!(
            simulation.stop().await,
            Err(SimulationError::Physics(StepError::BodyIntegration { .. }))
        ));
        let world = simulation
            .simulation
            .as_ref()
            .expect("runner returned after failure")
            .world()
            .expect("world restored");
        let body = world.body(body).expect("body");
        assert_eq!(body.state().linear_velocity(), Vec3::Y);
        assert_eq!(body.forces().force(), Vec3::X * f64::MAX);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn runtime_config_applies_central_gravity() {
        let mut config = config(1_000);
        config.central_gravity =
            Some(CentralGravity::new(Vec3::ZERO, 100.0).expect("valid central gravity"));
        let mut simulation = Simulation::new_from_config(config).expect("simulation");
        let client = simulation.client();
        simulation.start().expect("start");
        let mass = MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
        let body = RigidBody::new_dynamic(
            Transform::new(Vec3::Y * 10.0, kapitsa_math::Quat::IDENTITY).expect("transform"),
            mass,
        )
        .expect("body");
        let id = client.add_body(body).await.expect("body added");

        assert!(
            client
                .snapshot()
                .body(id)
                .expect("body published")
                .linear_velocity()
                .y
                < 0.0
        );
        simulation.stop().await.expect("stop");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn time_multiplier_changes_scheduler_without_changing_fixed_dt() {
        let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
        let client = simulation.client();
        for multiplier in [0.1, 1.0, 10.0, 100.0, 1_000.0] {
            simulation
                .set_time_multiplier(multiplier)
                .expect("finite non-negative multiplier");
            assert_eq!(simulation.metrics().requested_time_multiplier, multiplier);
        }
        for multiplier in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            assert!(matches!(
                simulation.set_time_multiplier(multiplier),
                Err(SimulationError::InvalidTimeMultiplier)
            ));
        }
        simulation
            .set_time_multiplier(0.0)
            .expect("pause simulation");
        simulation.start().expect("start");
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(simulation.metrics().total_steps, 0);

        simulation.set_time_multiplier(1.0).expect("resume at 1x");
        tokio::time::sleep(Duration::from_millis(100)).await;
        let first_step = simulation.metrics().total_steps;
        assert!(first_step >= 1);
        assert_eq!(simulation.metrics().step_budget, Duration::from_millis(10));

        simulation.set_time_multiplier(10.0).expect("switch to 10x");
        tokio::time::sleep(Duration::from_millis(100)).await;
        let metrics = simulation.metrics();
        assert!(metrics.total_steps > first_step);
        assert_eq!(metrics.requested_time_multiplier, 10.0);
        assert_eq!(metrics.step_budget, Duration::from_millis(10));
        assert_eq!(
            client.snapshot().simulation_time(),
            Duration::from_millis(10 * client.snapshot().step())
        );
        assert!(metrics.actual_time_multiplier.is_finite());
        simulation.stop().await.expect("stop");
    }

    #[tokio::test(start_paused = true)]
    async fn adaptive_time_warp_scales_physics_step_and_reports_unattainable_warp() {
        let mut adaptive = config(100);
        adaptive.time_step_policy = TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default());
        let mut simulation = Simulation::new_from_config(adaptive).expect("simulation");
        simulation.set_time_multiplier(5.0).expect("adaptive warp");
        simulation.start().expect("start");
        tokio::task::yield_now().await;
        tokio::time::advance(Duration::from_millis(10)).await;
        while simulation.metrics().total_steps == 0 {
            tokio::task::yield_now().await;
        }
        assert_eq!(
            simulation.metrics().last_physics_step,
            Duration::from_millis(50)
        );
        assert_eq!(
            simulation.client().snapshot().simulation_time(),
            Duration::from_millis(50)
        );
        assert!(!simulation.metrics().adaptive_step_limited);
        simulation.stop().await.expect("stop");

        let mut limited = config(100);
        limited.time_step_policy = TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig {
            minimum_step: Duration::from_millis(5),
            maximum_step: Duration::from_millis(10),
            relative_error_tolerance: 0.01,
        });
        let mut simulation = Simulation::new_from_config(limited).expect("limited simulation");
        let world = simulation
            .simulation
            .as_mut()
            .expect("runner")
            .world_mut()
            .expect("world");
        let body_id = world.add_body(dynamic_body()).expect("body");
        world
            .add_collider(
                body_id,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                    Transform::identity(),
                ),
            )
            .expect("collider");
        world
            .body_mut(body_id)
            .expect("body")
            .set_linear_velocity(Vec3::X * 1_000.0)
            .expect("velocity");
        let client = simulation.client();
        let mut results = client.subscribe_batch_results();
        simulation.set_time_multiplier(10.0).expect("adaptive warp");
        simulation.start().expect("start limited simulation");
        tokio::task::yield_now().await;
        tokio::time::advance(Duration::from_millis(10)).await;
        while !simulation.metrics().adaptive_step_limited {
            tokio::task::yield_now().await;
        }
        assert_eq!(simulation.metrics().total_steps, 0);
        assert_eq!(client.snapshot().simulation_time(), Duration::ZERO);
        simulation
            .set_time_multiplier(0.0)
            .expect("pause before recovery");
        let mut recovery = client.commands();
        recovery.set_linear_velocity(body_id, Vec3::ZERO);
        let batch_id = client.try_submit(recovery).expect("recovery command");
        tokio::time::advance(Duration::from_millis(10)).await;
        let result = results.next_result().await.expect("recovery result");
        assert_eq!(result.batch_id, batch_id);
        assert!(result.result.is_ok());
        assert_eq!(result.step, 0);
        assert_eq!(client.snapshot().simulation_time(), Duration::ZERO);
        assert_eq!(
            client
                .snapshot()
                .body(body_id)
                .expect("recovered body")
                .linear_velocity(),
            Vec3::ZERO
        );
        assert!(!simulation.metrics().adaptive_step_limited);
        simulation.stop().await.expect("stop limited simulation");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn high_warp_keeps_commands_and_shutdown_responsive_with_time_debt() {
        let mut simulation = Simulation::new_from_config(config(120)).expect("simulation");
        {
            let world = simulation
                .simulation
                .as_mut()
                .expect("runner")
                .world_mut()
                .expect("world");
            for _ in 0..512 {
                world.add_body(dynamic_body()).expect("body");
            }
        }
        let client = simulation.client();
        simulation.set_time_multiplier(1_000.0).expect("high warp");
        simulation.start().expect("start");
        tokio::time::sleep(Duration::from_millis(20)).await;
        assert!(simulation.metrics().total_steps > 0);
        assert!(!simulation.metrics().simulation_time_debt.is_zero());

        let body = tokio::time::timeout(Duration::from_secs(2), client.add_body(dynamic_body()))
            .await
            .expect("command was not starved by accumulated debt")
            .expect("body command applied");
        assert!(client.snapshot().body(body).is_some());

        tokio::time::timeout(Duration::from_secs(2), simulation.stop())
            .await
            .expect("shutdown was not responsive under high warp")
            .expect("stop");
    }

    #[tokio::test(start_paused = true)]
    async fn scaled_scheduler_wait_uses_the_tokio_clock() {
        let start = tokio::time::Instant::now();
        let mut wait = Box::pin(wait_for_scaled_step(Duration::from_millis(100), 10.0));
        assert!(matches!(poll_once(wait.as_mut()), Poll::Pending));
        tokio::time::advance(Duration::from_millis(9)).await;
        assert!(matches!(poll_once(wait.as_mut()), Poll::Pending));
        tokio::time::advance(Duration::from_millis(1)).await;
        wait.await;
        assert_eq!(
            tokio::time::Instant::now() - start,
            Duration::from_millis(10)
        );

        let mut paused = Box::pin(wait_for_scaled_step(Duration::from_millis(10), 0.0));
        assert!(matches!(poll_once(paused.as_mut()), Poll::Pending));
        tokio::time::advance(Duration::from_secs(1)).await;
        assert!(matches!(poll_once(paused.as_mut()), Poll::Pending));
    }

    #[tokio::test(start_paused = true)]
    async fn scheduler_yields_without_timer_wait_when_step_debt_is_ready() {
        let start = tokio::time::Instant::now();
        assert!(!wait_for_scheduler_turn(true, Duration::from_secs(10), 1_000.0).await);
        assert_eq!(tokio::time::Instant::now() - start, Duration::ZERO);

        assert!(wait_for_scheduler_turn(false, Duration::from_secs(10), 1.0).await);
        assert_eq!(tokio::time::Instant::now() - start, Duration::from_secs(10));
    }

    #[test]
    fn large_time_debt_is_retained_and_duration_overflow_is_reported() {
        let mut accumulator = Duration::ZERO;
        add_scaled_elapsed(&mut accumulator, Duration::from_millis(125), 1_000.0)
            .expect("large scaled duration remains representable");
        assert_eq!(accumulator, Duration::from_millis(125_000));

        let mut overflow = Duration::ZERO;
        assert!(matches!(
            add_scaled_elapsed(&mut overflow, Duration::from_secs(1), f64::MAX),
            Err(SimulationError::SimulationTimeOverflow)
        ));
        assert_eq!(overflow, Duration::ZERO);

        assert!(matches!(add_elapsed(&mut overflow, Duration::MAX), Ok(())));
        assert!(matches!(
            add_elapsed(&mut overflow, Duration::from_nanos(1)),
            Err(SimulationError::SimulationTimeOverflow)
        ));
        assert_eq!(overflow, Duration::MAX);
    }

    #[test]
    fn actual_time_multiplier_uses_elapsed_simulation_time_since_change() {
        assert_eq!(
            measured_time_multiplier(
                Duration::from_secs(3),
                Duration::from_secs(2),
                Duration::from_millis(500),
            ),
            2.0,
        );
        assert_eq!(
            measured_time_multiplier(
                Duration::from_secs(2),
                Duration::from_secs(2),
                Duration::ZERO
            ),
            0.0,
        );
    }

    #[tokio::test]
    async fn invalid_time_multiplier_is_rejected_without_changing_metrics() {
        let simulation = Simulation::new().expect("simulation");
        assert_eq!(simulation.metrics().requested_time_multiplier, 1.0);
        assert!(matches!(
            simulation.set_time_multiplier(-0.5),
            Err(SimulationError::InvalidTimeMultiplier)
        ));
        assert_eq!(simulation.metrics().requested_time_multiplier, 1.0);
    }

    #[tokio::test]
    async fn start_twice_reports_an_invalid_transition() {
        let mut simulation = Simulation::new().expect("simulation");
        simulation.start().expect("start");
        assert!(matches!(
            simulation.start(),
            Err(SimulationError::AlreadyRunning)
        ));
        simulation.stop().await.expect("stop");
    }
}

#[cfg(all(test, feature = "bench-internals"))]
mod benchmarks;
