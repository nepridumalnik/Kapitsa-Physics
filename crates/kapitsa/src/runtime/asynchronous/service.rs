use std::{sync::Arc, time::Duration};

use kapitsa_core::world::PhysicsWorld;
use tokio::{
    runtime::Handle,
    sync::{broadcast, mpsc, watch},
    task::JoinHandle,
    time::Instant,
};

use super::{
    StepRunner, StepRunnerError,
    marshalling::{
        CollisionEventBatch, CommandRequest, PhysicsClient, PhysicsClientError, PhysicsSnapshot,
        apply_command,
    },
};
use crate::runtime::{ConfigError, SimulationConfig, SimulationMetrics, SimulationState};

const COMMAND_QUEUE_CAPACITY: usize = 1024;
const MAX_COMMANDS_PER_STEP: usize = 1024;
const COLLISION_EVENT_BATCH_CAPACITY: usize = 64;

struct ServiceTaskOutput {
    simulation: StepRunner,
    simulation_time: Duration,
    result: Result<(), SimulationError>,
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
    RayonPool(super::super::executor::RayonPoolError),
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
            Self::RayonPool(error) => write!(formatter, "Rayon runtime failed: {error}"),
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
            Self::RayonPool(error) => Some(error),
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
            StepRunnerError::RayonPool(error) => Self::RayonPool(error),
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
        Ok(Self {
            simulation: Some(simulation),
            task: None,
            stop: None,
            command_tx,
            command_rx,
            snapshot_tx,
            snapshot_rx,
            collision_events_tx,
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
        let collision_events_tx = self.collision_events_tx.clone();
        let mut simulation_time = self.simulation_time;
        self.stop = Some(stop_tx);
        self.task = Some(self.runtime.spawn(async move {
            let result = run_simulation(
                &mut simulation,
                command_receiver,
                stop_rx,
                &metrics_tx,
                &snapshot_tx,
                &collision_events_tx,
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
    mut commands: mpsc::Receiver<CommandRequest>,
    stop: watch::Receiver<bool>,
    metrics: &watch::Sender<SimulationMetrics>,
    snapshots: &watch::Sender<Arc<PhysicsSnapshot>>,
    collision_events: &broadcast::Sender<CollisionEventBatch>,
    simulation_time: &mut Duration,
) -> Result<(), SimulationError> {
    let result = run_simulation_loop(
        simulation,
        &mut commands,
        stop,
        metrics,
        snapshots,
        collision_events,
        simulation_time,
    )
    .await;
    while let Ok(request) = commands.try_recv() {
        let _ = request.reply.send(Err(PhysicsClientError::NotRunning));
    }
    result
}

async fn run_simulation_loop(
    simulation: &mut StepRunner,
    commands: &mut mpsc::Receiver<CommandRequest>,
    mut stop: watch::Receiver<bool>,
    metrics: &watch::Sender<SimulationMetrics>,
    snapshots: &watch::Sender<Arc<PhysicsSnapshot>>,
    collision_events: &broadcast::Sender<CollisionEventBatch>,
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
            apply_queued_commands(simulation, commands)?;
            simulation.step().await?;
            *simulation_time = next_simulation_time;

            let (mut snapshot, _) = PhysicsSnapshot::take_reusable(&mut reusable_snapshot);
            snapshot.rebuild(
                simulation.world().map_err(SimulationError::from)?,
                simulation.metrics().total_steps,
                *simulation_time,
            );
            let previous_snapshot = snapshots.send_replace(Arc::new(snapshot));
            PhysicsSnapshot::recycle(previous_snapshot, &mut reusable_snapshot);
            if collision_events.receiver_count() > 0 {
                let batch = CollisionEventBatch::new(
                    simulation.metrics().total_steps,
                    simulation
                        .world()
                        .map_err(SimulationError::from)?
                        .collision_events()
                        .to_vec(),
                );
                let _ = collision_events.send(batch);
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
    commands: &mut mpsc::Receiver<CommandRequest>,
) -> Result<(), SimulationError> {
    let world = simulation.world_mut().map_err(SimulationError::from)?;
    for _ in 0..MAX_COMMANDS_PER_STEP {
        let request = match commands.try_recv() {
            Ok(request) => request,
            Err(mpsc::error::TryRecvError::Empty | mpsc::error::TryRecvError::Disconnected) => {
                break;
            }
        };
        let result = apply_command(world, request.command);
        let _ = request.reply.send(result);
    }
    Ok(())
}

fn add_elapsed(accumulator: &mut Duration, elapsed: Duration) -> Result<(), SimulationError> {
    *accumulator = accumulator
        .checked_add(elapsed)
        .ok_or(SimulationError::SimulationTimeOverflow)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroUsize;
    use std::time::Duration;

    use kapitsa_core::{mass::MassProperties, rigid_body::RigidBody, transform::Transform};
    use kapitsa_math::{Mat3, Vec3};
    use tokio::time::sleep;

    use crate::runtime::{SimulationConfig, SimulationError, SimulationState, WorkerConfig};

    use super::Simulation;

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
