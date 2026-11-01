use std::{sync::Arc, time::Duration};

use kapitsa_core::world::PhysicsWorld;
use tokio::{
    runtime::Handle,
    sync::{broadcast, mpsc, watch},
    task::JoinHandle,
    time::Instant,
};

use super::{
    Simulation, SimulationError,
    marshalling::{
        CollisionEventBatch, CommandRequest, PhysicsClient, PhysicsSnapshot, apply_command,
    },
};
use crate::runtime::{SimulationConfig, SimulationMetrics, SimulationState, WorkerConfig};

const COMMAND_QUEUE_CAPACITY: usize = 1024;
const MAX_COMMANDS_PER_STEP: usize = 1024;
const COLLISION_EVENT_BATCH_CAPACITY: usize = 64;

struct ServiceTaskOutput {
    simulation: Simulation,
    simulation_time: Duration,
    result: Result<(), PhysicsServiceError>,
}

/// Drives a simulation from a fixed-rate Tokio timer and exposes its client API.
///
/// [`Simulation::step`] remains a manual one-step operation. Use this service
/// when commands, snapshots, events, and clock-driven catch-up are needed.
pub struct PhysicsService {
    simulation: Option<Simulation>,
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
pub enum PhysicsServiceError {
    MissingRuntime,
    SimulationTimeOverflow,
    WorldUnavailable,
    TaskRequiresStop,
    Simulation(SimulationError),
    TaskFailed(String),
}

impl std::fmt::Display for PhysicsServiceError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingRuntime => {
                formatter.write_str("starting a physics service requires a Tokio runtime")
            }
            Self::SimulationTimeOverflow => {
                formatter.write_str("simulation time accumulator overflowed")
            }
            Self::WorldUnavailable => {
                formatter.write_str("physics world is available only while the service is stopped")
            }
            Self::TaskRequiresStop => {
                formatter.write_str("stop the completed physics service before restarting")
            }
            Self::Simulation(error) => write!(formatter, "physics service failed: {error}"),
            Self::TaskFailed(reason) => write!(formatter, "physics service task failed: {reason}"),
        }
    }
}

impl std::error::Error for PhysicsServiceError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Simulation(error) => Some(error),
            _ => None,
        }
    }
}

impl From<SimulationError> for PhysicsServiceError {
    fn from(error: SimulationError) -> Self {
        Self::Simulation(error)
    }
}

impl PhysicsService {
    /// Creates a service from the currently entered Tokio runtime.
    pub fn new(world: PhysicsWorld, config: SimulationConfig) -> Result<Self, PhysicsServiceError> {
        Self::new_with_worker_config(world, config, WorkerConfig::default())
    }

    /// Creates a service with explicit worker-pool settings.
    pub fn new_with_worker_config(
        world: PhysicsWorld,
        config: SimulationConfig,
        worker_config: WorkerConfig,
    ) -> Result<Self, PhysicsServiceError> {
        let runtime = Handle::try_current().map_err(|_| PhysicsServiceError::MissingRuntime)?;
        let simulation = Simulation::new_with_worker_config(world, config, worker_config)?;
        Self::from_simulation_on(simulation, runtime)
    }

    /// Creates a service on the supplied host runtime.
    pub fn new_on(
        world: PhysicsWorld,
        config: SimulationConfig,
        runtime: Handle,
    ) -> Result<Self, PhysicsServiceError> {
        Self::new_on_with_worker_config(world, config, WorkerConfig::default(), runtime)
    }

    /// Creates a service on the supplied host runtime with explicit worker settings.
    pub fn new_on_with_worker_config(
        world: PhysicsWorld,
        config: SimulationConfig,
        worker_config: WorkerConfig,
        runtime: Handle,
    ) -> Result<Self, PhysicsServiceError> {
        let simulation =
            Simulation::new_on_with_worker_config(world, config, worker_config, runtime.clone())?;
        Self::from_simulation_on(simulation, runtime)
    }

    /// Wraps a manually configured simulation as a clock-driven service.
    pub fn from_simulation(simulation: Simulation) -> Result<Self, PhysicsServiceError> {
        let runtime = Handle::try_current().map_err(|_| PhysicsServiceError::MissingRuntime)?;
        Self::from_simulation_on(simulation, runtime)
    }

    /// Wraps a simulation and schedules the service on the supplied Tokio runtime.
    pub fn from_simulation_on(
        simulation: Simulation,
        runtime: Handle,
    ) -> Result<Self, PhysicsServiceError> {
        let world = simulation.world().map_err(PhysicsServiceError::from)?;
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
    pub fn start(&mut self) -> Result<(), PhysicsServiceError> {
        if let Some(task) = &self.task {
            return if task.is_finished() {
                Err(PhysicsServiceError::TaskRequiresStop)
            } else {
                Ok(())
            };
        }

        let snapshot = {
            let simulation = self
                .simulation
                .as_ref()
                .ok_or(PhysicsServiceError::TaskRequiresStop)?;
            Arc::new(PhysicsSnapshot::from_world(
                simulation.world().map_err(PhysicsServiceError::from)?,
                simulation.metrics().total_steps,
                self.simulation_time,
            ))
        };
        let simulation = self
            .simulation
            .take()
            .ok_or(PhysicsServiceError::TaskRequiresStop)?;
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
    pub async fn stop(&mut self) -> Result<(), PhysicsServiceError> {
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
                Err(PhysicsServiceError::TaskFailed(error.to_string()))
            }
        }
    }

    /// Stops the service, shuts down its owned Rayon pool, and returns the world.
    pub async fn shutdown(mut self) -> Result<PhysicsWorld, PhysicsServiceError> {
        let stop_result = self.stop().await;
        let simulation = self
            .simulation
            .take()
            .ok_or(PhysicsServiceError::WorldUnavailable)?;
        let world = simulation
            .shutdown()
            .await
            .map_err(PhysicsServiceError::from);
        stop_result?;
        world
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

    pub fn world(&self) -> Result<&PhysicsWorld, PhysicsServiceError> {
        self.simulation
            .as_ref()
            .ok_or(PhysicsServiceError::WorldUnavailable)?
            .world()
            .map_err(PhysicsServiceError::from)
    }

    pub fn world_mut(&mut self) -> Result<&mut PhysicsWorld, PhysicsServiceError> {
        self.simulation
            .as_mut()
            .ok_or(PhysicsServiceError::WorldUnavailable)?
            .world_mut()
            .map_err(PhysicsServiceError::from)
    }
}

impl Drop for PhysicsService {
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
    simulation: &mut Simulation,
    mut commands: mpsc::Receiver<CommandRequest>,
    mut stop: watch::Receiver<bool>,
    metrics: &watch::Sender<SimulationMetrics>,
    snapshots: &watch::Sender<Arc<PhysicsSnapshot>>,
    collision_events: &broadcast::Sender<CollisionEventBatch>,
    simulation_time: &mut Duration,
) -> Result<(), PhysicsServiceError> {
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
                .ok_or(PhysicsServiceError::SimulationTimeOverflow)?;
            apply_queued_commands(simulation, &mut commands)?;
            simulation.step().await?;
            *simulation_time = next_simulation_time;

            let (mut snapshot, _) = PhysicsSnapshot::take_reusable(&mut reusable_snapshot);
            snapshot.rebuild(
                simulation.world().map_err(PhysicsServiceError::from)?,
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
                        .map_err(PhysicsServiceError::from)?
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
    simulation: &mut Simulation,
    commands: &mut mpsc::Receiver<CommandRequest>,
) -> Result<(), PhysicsServiceError> {
    let world = simulation.world_mut().map_err(PhysicsServiceError::from)?;
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

fn add_elapsed(accumulator: &mut Duration, elapsed: Duration) -> Result<(), PhysicsServiceError> {
    *accumulator = accumulator
        .checked_add(elapsed)
        .ok_or(PhysicsServiceError::SimulationTimeOverflow)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::num::NonZeroUsize;
    use std::time::Duration;

    use kapitsa_core::{
        mass::MassProperties, rigid_body::RigidBody, transform::Transform, world::PhysicsWorld,
    };
    use kapitsa_math::{Mat3, Vec3};
    use tokio::time::sleep;

    use crate::runtime::{SimulationConfig, SimulationState, WorkerConfig};

    use super::PhysicsService;

    fn dynamic_body() -> RigidBody {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass");
        RigidBody::new_dynamic(Transform::identity(), mass).expect("dynamic body")
    }

    fn worker_config() -> WorkerConfig {
        WorkerConfig {
            max_workers: NonZeroUsize::new(2),
            ..WorkerConfig::default()
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn service_start_stop_restart_and_client_commands_work() {
        let mut service = PhysicsService::new_with_worker_config(
            PhysicsWorld::new(),
            SimulationConfig {
                tick_rate: 100,
                ..SimulationConfig::default()
            },
            worker_config(),
        )
        .expect("service");
        let client = service.client();
        service.start().expect("start");
        assert_eq!(service.state(), SimulationState::Running);
        let id = client.add_body(dynamic_body()).await.expect("add body");
        client.apply_force(id, Vec3::X).await.expect("apply force");
        sleep(Duration::from_millis(35)).await;
        service.stop().await.expect("stop");
        assert_eq!(service.state(), SimulationState::Stopped);
        assert!(service.metrics().total_steps >= 1);
        assert!(service.world().expect("world restored").body(id).is_some());

        let previous_steps = service.metrics().total_steps;
        service.start().expect("restart");
        sleep(Duration::from_millis(25)).await;
        service.stop().await.expect("second stop");
        assert!(service.metrics().total_steps > previous_steps);
    }

    #[tokio::test(start_paused = true)]
    async fn service_manual_client_is_not_running_and_shutdown_returns_world() {
        let service = PhysicsService::new_with_worker_config(
            PhysicsWorld::new(),
            SimulationConfig::default(),
            worker_config(),
        )
        .expect("service");
        assert!(matches!(
            service
                .client()
                .add_body(RigidBody::new_static(Transform::identity()))
                .await,
            Err(crate::runtime::asynchronous::PhysicsClientError::NotRunning)
        ));
        let world = service.shutdown().await.expect("shutdown");
        assert_eq!(world.body_count(), 0);
    }

    #[tokio::test(start_paused = true)]
    async fn stop_waits_for_an_active_step_before_restoring_world() {
        let mut service = PhysicsService::new_with_worker_config(
            PhysicsWorld::new(),
            SimulationConfig {
                tick_rate: 100,
                ..SimulationConfig::default()
            },
            worker_config(),
        )
        .expect("service");
        service.start().expect("start");
        tokio::time::advance(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;
        service.stop().await.expect("stop");
        assert_eq!(service.state(), SimulationState::Stopped);
        assert!(service.world().is_ok());
    }
}
