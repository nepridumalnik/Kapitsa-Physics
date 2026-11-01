use std::{
    sync::{Arc, atomic::AtomicU64},
    time::Duration,
};

use kapitsa_core::{
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
        BatchScratch, CollisionEventBatch, CommandBufferPool, CommandRequest, PhysicsBatchError,
        PhysicsBatchResult, PhysicsClient, PhysicsClientError, PhysicsCommand,
        PhysicsCommandResult, PhysicsSnapshot, apply_command, apply_command_batch,
    },
};
use crate::runtime::{ConfigError, SimulationConfig, SimulationMetrics, SimulationState};

const COMMAND_QUEUE_CAPACITY: usize = 1024;
const MAX_COMMANDS_PER_STEP: usize = 1024;
const COLLISION_EVENT_BATCH_CAPACITY: usize = 64;
const PHYSICS_BATCH_RESULT_CAPACITY: usize = 256;

struct ServiceTaskOutput {
    simulation: StepRunner,
    simulation_time: Duration,
    result: Result<(), SimulationError>,
}

#[derive(Clone)]
struct SimulationEvents {
    collisions: broadcast::Sender<CollisionEventBatch>,
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
        is_spatial_query: bool,
    },
    Batch(PhysicsBatchResult),
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
    snapshot_tx: watch::Sender<Arc<PhysicsSnapshot>>,
    snapshot_rx: watch::Receiver<Arc<PhysicsSnapshot>>,
    collision_events_tx: broadcast::Sender<CollisionEventBatch>,
    batch_results_tx: broadcast::Sender<PhysicsBatchResult>,
    batch_id_sequence: Arc<AtomicU64>,
    buffer_pool: Arc<CommandBufferPool>,
    simulation_time: Duration,
    metrics_tx: watch::Sender<SimulationMetrics>,
    metrics_rx: watch::Receiver<SimulationMetrics>,
    runtime: Handle,
}

/// Failure returned by a clock-driven physics service.
#[derive(Debug)]
pub enum SimulationError {
    InvalidConfig(ConfigError),
    MissingRuntime,
    SimulationTimeOverflow,
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
            Self::InvalidConfig(error) => write!(formatter, "invalid simulation config: {error}"),
            Self::MissingRuntime => {
                formatter.write_str("starting a physics service requires a Tokio runtime")
            }
            Self::SimulationTimeOverflow => {
                formatter.write_str("simulation time accumulator overflowed")
            }
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
    /// Creates an empty simulation using the default fixed-step configuration.
    pub fn new() -> Result<Self, SimulationError> {
        Self::new_from_config(SimulationConfig::default())
    }

    /// Creates an empty simulation using the supplied configuration.
    pub fn new_from_config(config: SimulationConfig) -> Result<Self, SimulationError> {
        config.validate().map_err(SimulationError::InvalidConfig)?;
        let runtime = Handle::try_current().map_err(|_| SimulationError::MissingRuntime)?;
        let simulation =
            StepRunner::new_on_validated(PhysicsWorld::new(), config, runtime.clone())?;
        let world = simulation.world().map_err(SimulationError::from)?;
        let initial_snapshot = Arc::new(PhysicsSnapshot::from_world(
            world,
            simulation.metrics().total_steps,
            Duration::ZERO,
        ));
        let (metrics_tx, metrics_rx) = watch::channel(simulation.metrics().clone());
        let (command_tx, command_rx) = watch::channel(None);
        let (snapshot_tx, snapshot_rx) = watch::channel(initial_snapshot);
        let (collision_events_tx, _) = broadcast::channel(COLLISION_EVENT_BATCH_CAPACITY);
        let (batch_results_tx, _) = broadcast::channel(PHYSICS_BATCH_RESULT_CAPACITY);
        let buffer_pool = Arc::new(CommandBufferPool::default());
        Ok(Self {
            simulation: Some(simulation),
            task: None,
            stop: None,
            command_tx,
            command_rx,
            snapshot_tx,
            snapshot_rx,
            collision_events_tx,
            batch_results_tx,
            batch_id_sequence: Arc::new(AtomicU64::new(1)),
            buffer_pool,
            simulation_time: Duration::ZERO,
            metrics_tx,
            metrics_rx,
            runtime,
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
        self.metrics_tx.send_replace(simulation.metrics().clone());

        let metrics_tx = self.metrics_tx.clone();
        let command_state_tx = self.command_tx.clone();
        let snapshot_tx = self.snapshot_tx.clone();
        let events = SimulationEvents {
            collisions: self.collision_events_tx.clone(),
            batch_results: self.batch_results_tx.clone(),
        };
        let mut simulation_time = self.simulation_time;
        let buffer_pool = Arc::clone(&self.buffer_pool);
        self.stop = Some(stop_tx);
        self.task = Some(self.runtime.spawn(async move {
            let result = run_simulation(
                &mut simulation,
                CommandQueue::new(command_receiver, buffer_pool),
                stop_rx,
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
            metrics_tx.send_replace(simulation.metrics().clone());
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

    pub fn client(&self) -> PhysicsClient {
        PhysicsClient::new(
            self.command_rx.clone(),
            self.snapshot_rx.clone(),
            self.collision_events_tx.clone(),
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
    stop: watch::Receiver<bool>,
    metrics: &watch::Sender<SimulationMetrics>,
    snapshots: &watch::Sender<Arc<PhysicsSnapshot>>,
    events: &SimulationEvents,
    simulation_time: &mut Duration,
) -> Result<(), SimulationError> {
    let result = run_simulation_loop(
        simulation,
        &mut commands,
        stop,
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
    mut stop: watch::Receiver<bool>,
    metrics: &watch::Sender<SimulationMetrics>,
    snapshots: &watch::Sender<Arc<PhysicsSnapshot>>,
    events: &SimulationEvents,
    simulation_time: &mut Duration,
) -> Result<(), SimulationError> {
    let step_duration = simulation.config().step_budget();
    let max_catch_up_steps = simulation.config().max_catch_up_steps;
    let mut previous = Instant::now();
    let mut accumulator = Duration::ZERO;
    let mut reusable_snapshot = None;

    loop {
        let delay = step_duration.saturating_sub(accumulator);
        tokio::select! {
            changed = stop.changed() => {
                if changed.is_err() || *stop.borrow() {
                    break;
                }
            }
            _ = tokio::time::sleep(delay) => {}
        }
        if stopping(&stop) {
            break;
        }

        let now = Instant::now();
        add_elapsed(&mut accumulator, now.duration_since(previous))?;
        previous = now;

        for _ in 0..max_catch_up_steps {
            if accumulator < step_duration || stopping(&stop) {
                break;
            }
            let next_simulation_time = simulation_time
                .checked_add(step_duration)
                .ok_or(SimulationError::SimulationTimeOverflow)?;
            let attempted_step = simulation.metrics().total_steps.saturating_add(1);
            apply_queued_commands(simulation, commands)?;
            if let Err(error) = simulation.step().await {
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

            accumulator = accumulator.saturating_sub(step_duration);
            let completed_at = Instant::now();
            add_elapsed(&mut accumulator, completed_at.duration_since(previous))?;
            previous = completed_at;
            let debt = accumulator.saturating_sub(step_duration);
            simulation.set_runtime_status(SimulationState::Running, debt);
            metrics.send_replace(simulation.metrics().clone());
        }

        if stopping(&stop) {
            break;
        }
        if accumulator >= step_duration {
            tokio::task::yield_now().await;
        }
    }

    Ok(())
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
            CommandRequest::Single { command, reply } => {
                let is_spatial_query = matches!(
                    &command,
                    PhysicsCommand::Raycast { .. }
                        | PhysicsCommand::Sweep { .. }
                        | PhysicsCommand::Overlaps { .. }
                );
                let result = apply_command(world, command);
                commands
                    .pending_results
                    .push(DeferredCommandResult::Single {
                        reply,
                        result,
                        is_spatial_query,
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
            DeferredCommandResult::Single {
                reply,
                mut result,
                is_spatial_query,
            } => {
                if result.is_ok() && !is_spatial_query {
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

#[cfg(test)]
mod tests {
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
        mass::MassProperties,
        rigid_body::RigidBody,
        shape::{Shape, Sphere},
        transform::Transform,
        world::StepError,
    };
    use kapitsa_math::{Mat3, Vec3};
    use tokio::{sync::oneshot, time::sleep};

    use crate::runtime::{
        PhysicsClientError, SimulationConfig, SimulationError, SimulationState, WorkerConfig,
    };

    use super::{PhysicsBatchError, Simulation};

    fn dynamic_body() -> RigidBody {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass");
        RigidBody::new_dynamic(Transform::identity(), mass).expect("dynamic body")
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
