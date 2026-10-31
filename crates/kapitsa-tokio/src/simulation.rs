use std::time::Duration;

use kapitsa_core::world::PhysicsWorld;
use tokio::{runtime::Handle, sync::watch, task::JoinHandle, time::Instant};

use crate::{
    PhysicsController, SchedulerConfig, SimulationError, SimulationMetrics, SimulationState,
};

struct SimulationTaskOutput {
    controller: PhysicsController,
    result: Result<(), SimulationError>,
}

/// Owns a physics world and advances it at a fixed, adaptive rate.
pub struct PhysicsSimulation {
    controller: Option<PhysicsController>,
    task: Option<JoinHandle<SimulationTaskOutput>>,
    stop: Option<watch::Sender<bool>>,
    metrics_tx: watch::Sender<SimulationMetrics>,
    metrics_rx: watch::Receiver<SimulationMetrics>,
}

impl PhysicsSimulation {
    pub fn new(world: PhysicsWorld) -> Self {
        Self::from_controller(PhysicsController::with_default_config(world))
    }

    pub fn with_config(
        world: PhysicsWorld,
        config: SchedulerConfig,
    ) -> Result<Self, SimulationError> {
        Ok(Self::from_controller(PhysicsController::new(
            world, config,
        )?))
    }

    fn from_controller(controller: PhysicsController) -> Self {
        let (metrics_tx, metrics_rx) = watch::channel(controller.metrics());
        Self {
            controller: Some(controller),
            task: None,
            stop: None,
            metrics_tx,
            metrics_rx,
        }
    }

    /// Starts the background simulation on the caller's Tokio runtime.
    pub fn start(&mut self) -> Result<(), SimulationError> {
        if let Some(task) = &self.task {
            return if task.is_finished() {
                Err(SimulationError::TaskRequiresStop)
            } else {
                Ok(())
            };
        }
        let runtime = Handle::try_current().map_err(|_| SimulationError::MissingRuntime)?;

        let Some(mut controller) = self.controller.take() else {
            return Err(SimulationError::TaskRequiresStop);
        };
        let (stop_tx, stop_rx) = watch::channel(false);
        controller.reset_actual_frequency();
        controller.set_simulation_state(SimulationState::Running);
        controller.set_simulation_time_debt(Duration::ZERO);
        self.metrics_tx.send_replace(controller.metrics());

        let metrics_tx = self.metrics_tx.clone();
        self.stop = Some(stop_tx);
        self.task = Some(runtime.spawn(async move {
            let result = run_simulation(&mut controller, stop_rx, &metrics_tx).await;
            controller.reset_actual_frequency();
            controller.set_simulation_state(if result.is_ok() {
                SimulationState::Stopped
            } else {
                SimulationState::Error
            });
            metrics_tx.send_replace(controller.metrics());
            SimulationTaskOutput { controller, result }
        }));
        Ok(())
    }

    /// Stops scheduling steps and waits for any step already in progress.
    pub async fn stop(&mut self) -> Result<(), SimulationError> {
        let Some(task) = &mut self.task else {
            return Ok(());
        };
        if let Some(stop) = &self.stop {
            stop.send_replace(true);
        }

        let joined = task.await;
        self.task = None;
        self.stop = None;
        match joined {
            Ok(output) => {
                self.controller = Some(output.controller);
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

    /// Returns the most recent snapshot without waiting for the simulation task.
    pub fn metrics(&self) -> SimulationMetrics {
        self.metrics_rx.borrow().clone()
    }

    /// Borrows the world while the simulation is stopped.
    pub fn world(&self) -> Result<&PhysicsWorld, SimulationError> {
        self.controller
            .as_ref()
            .ok_or(SimulationError::WorldUnavailable)?
            .world()
            .map_err(SimulationError::from)
    }

    /// Mutates the world while the simulation is stopped.
    pub fn world_mut(&mut self) -> Result<&mut PhysicsWorld, SimulationError> {
        self.controller
            .as_mut()
            .ok_or(SimulationError::WorldUnavailable)?
            .world_mut()
            .map_err(SimulationError::from)
    }
}

impl Drop for PhysicsSimulation {
    fn drop(&mut self) {
        if let Some(stop) = &self.stop {
            stop.send_replace(true);
        }
    }
}

fn stopping(stop: &watch::Receiver<bool>) -> bool {
    *stop.borrow() || stop.has_changed().is_err()
}

async fn run_simulation(
    controller: &mut PhysicsController,
    mut stop: watch::Receiver<bool>,
    metrics: &watch::Sender<SimulationMetrics>,
) -> Result<(), SimulationError> {
    let mut previous = Instant::now();
    let mut accumulator = Duration::ZERO;

    loop {
        let delay = controller.step_duration().saturating_sub(accumulator);
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

        run_catch_up_batch(controller, &stop, metrics, &mut accumulator, &mut previous).await?;

        if stopping(&stop) {
            break;
        }
        if accumulator >= controller.step_duration() {
            tokio::task::yield_now().await;
        }
    }

    Ok(())
}

async fn run_catch_up_batch(
    controller: &mut PhysicsController,
    stop: &watch::Receiver<bool>,
    metrics: &watch::Sender<SimulationMetrics>,
    accumulator: &mut Duration,
    previous: &mut Instant,
) -> Result<(), SimulationError> {
    for _ in 0..controller.config().max_catch_up_steps {
        let step_duration = controller.step_duration();
        if *accumulator < step_duration || stopping(stop) {
            break;
        }
        controller
            .step()
            .await
            .map_err(SimulationError::Scheduler)?;
        *accumulator -= step_duration;
        let completed_at = Instant::now();
        add_elapsed(accumulator, completed_at.duration_since(*previous))?;
        *previous = completed_at;
        controller.set_simulation_time_debt(accumulator.saturating_sub(controller.step_duration()));
        metrics.send_replace(controller.metrics());
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
    use std::{
        future::Future,
        task::{Context, Poll, Waker},
        time::Duration,
    };

    use kapitsa_core::{
        mass::MassProperties,
        rigid_body::RigidBody,
        transform::Transform,
        world::{BodyId, PhysicsWorld},
    };
    use kapitsa_math::{Mat3, Vec3};

    use super::{PhysicsSimulation, add_elapsed, run_catch_up_batch, run_simulation};
    use crate::{
        PhysicsController, SchedulerConfig, SchedulerError, SimulationError, SimulationState,
    };

    fn moving_world() -> (PhysicsWorld, BodyId) {
        let mut world = PhysicsWorld::new();
        let mut body = RigidBody::new_kinematic(Transform::identity());
        body.set_linear_velocity(Vec3::X).expect("finite velocity");
        let id = world.add_body(body).expect("body insertion");
        (world, id)
    }

    #[test]
    fn start_requires_an_existing_runtime() {
        let mut simulation = PhysicsSimulation::new(PhysicsWorld::new());
        assert!(matches!(
            simulation.start(),
            Err(SimulationError::MissingRuntime)
        ));
        assert_eq!(simulation.state(), SimulationState::Stopped);
    }

    #[test]
    fn elapsed_overflow_is_reported_without_changing_the_accumulator() {
        let mut accumulator = Duration::MAX;
        assert!(matches!(
            add_elapsed(&mut accumulator, Duration::from_nanos(1)),
            Err(SimulationError::SimulationTimeOverflow)
        ));
        assert_eq!(accumulator, Duration::MAX);
    }

    #[tokio::test(start_paused = true)]
    async fn catch_up_batch_is_bounded_and_retains_debt() {
        let config = SchedulerConfig {
            max_catch_up_steps: 2,
            adaptive: false,
            ..SchedulerConfig::default()
        };
        let mut controller =
            PhysicsController::new(PhysicsWorld::new(), config).expect("controller");
        let quantum = controller.step_duration();
        let (stop_tx, stop_rx) = tokio::sync::watch::channel(false);
        let (metrics_tx, _) = tokio::sync::watch::channel(controller.metrics());
        let mut accumulator = Duration::from_secs(1);
        let mut previous = tokio::time::Instant::now();
        run_catch_up_batch(
            &mut controller,
            &stop_rx,
            &metrics_tx,
            &mut accumulator,
            &mut previous,
        )
        .await
        .expect("batch");
        assert_eq!(controller.metrics().total_steps, 2);
        assert!(accumulator >= Duration::from_secs(1) - quantum * 2);
        assert!(controller.metrics().simulation_time_debt > Duration::ZERO);
        stop_tx.send_replace(true);
        run_catch_up_batch(
            &mut controller,
            &stop_rx,
            &metrics_tx,
            &mut accumulator,
            &mut previous,
        )
        .await
        .expect("stopped batch");
        assert_eq!(controller.metrics().total_steps, 2);
    }

    #[test]
    fn invalid_simulation_configuration_is_rejected() {
        for config in [
            SchedulerConfig {
                target_hz: 0,
                ..SchedulerConfig::default()
            },
            SchedulerConfig {
                max_workers: 0,
                ..SchedulerConfig::default()
            },
        ] {
            assert!(matches!(
                PhysicsSimulation::with_config(PhysicsWorld::new(), config),
                Err(SimulationError::Scheduler(SchedulerError::InvalidConfig(_)))
            ));
        }
    }

    #[tokio::test(start_paused = true)]
    async fn configured_simulation_uses_its_scheduler_settings() {
        let config = SchedulerConfig {
            target_hz: 120,
            min_hz: 40,
            max_workers: 2,
            max_catch_up_steps: 1,
            adaptive: false,
            ..SchedulerConfig::default()
        };
        let mut simulation = PhysicsSimulation::with_config(PhysicsWorld::new(), config.clone())
            .expect("valid configuration");
        assert_eq!(
            simulation.controller.as_ref().expect("controller").config(),
            &config
        );
        assert_eq!(simulation.metrics().target_hz, 120);
        simulation.start().expect("start");
        tokio::time::sleep(Duration::from_millis(40)).await;
        simulation.stop().await.expect("stop");
        assert!(simulation.metrics().total_steps > 0);
        assert_eq!(simulation.metrics().current_hz, 120);
        assert_eq!(simulation.metrics().active_workers, 1);
    }

    #[tokio::test(start_paused = true)]
    async fn dropping_simulation_signals_the_background_task() {
        let mut simulation = PhysicsSimulation::new(PhysicsWorld::new());
        simulation.start().expect("start");
        let task = simulation.task.take().expect("background task");
        drop(simulation);
        let output = task.await.expect("background task completes");
        assert!(output.result.is_ok());
        assert_eq!(output.controller.metrics().total_steps, 0);
    }

    #[test]
    fn dropping_with_a_queued_step_and_large_debt_finishes_that_step() {
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .start_paused(true)
            .max_blocking_threads(1)
            .build()
            .expect("runtime");
        runtime.block_on(async {
            let (entered_tx, entered_rx) = tokio::sync::oneshot::channel();
            let (release_tx, release_rx) = std::sync::mpsc::channel();
            let blocker = tokio::task::spawn_blocking(move || {
                let _ = entered_tx.send(());
                let _ = release_rx.recv();
            });
            entered_rx.await.expect("blocking thread occupied");
            let mut simulation = PhysicsSimulation::new(PhysicsWorld::new());
            simulation.start().expect("start");
            tokio::task::yield_now().await;
            tokio::time::advance(Duration::from_secs(1)).await;
            tokio::task::yield_now().await;
            let task = simulation.task.take().expect("background task");
            drop(simulation);
            release_tx.send(()).expect("release blocking thread");
            blocker.await.expect("blocker completes");
            let output = task.await.expect("background task completes");
            assert!(output.result.is_ok());
            assert_eq!(output.controller.metrics().total_steps, 1);
            assert_eq!(
                output
                    .controller
                    .world()
                    .expect("world restored")
                    .body_count(),
                0
            );
        });
    }

    #[tokio::test(start_paused = true)]
    async fn closed_stop_channel_stops_the_loop() {
        let mut controller = PhysicsController::with_default_config(PhysicsWorld::new());
        let (stop_tx, stop_rx) = tokio::sync::watch::channel(false);
        let (metrics_tx, _) = tokio::sync::watch::channel(controller.metrics());
        drop(stop_tx);
        run_simulation(&mut controller, stop_rx, &metrics_tx)
            .await
            .expect("closed channel stops loop");
        assert_eq!(controller.metrics().total_steps, 0);
    }

    #[tokio::test(start_paused = true)]
    async fn start_stop_restart_metrics_and_world_access_work() {
        let (world, id) = moving_world();
        let mut simulation = PhysicsSimulation::new(world);
        assert!(simulation.world_mut().is_ok());

        simulation.start().expect("start in runtime");
        simulation.start().expect("repeated start is idempotent");
        assert_eq!(simulation.state(), SimulationState::Running);
        assert!(matches!(
            simulation.world_mut(),
            Err(SimulationError::WorldUnavailable)
        ));

        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(simulation.metrics().total_steps > 0);
        simulation.stop().await.expect("stop after active step");
        assert_eq!(simulation.state(), SimulationState::Stopped);
        assert_eq!(simulation.metrics().actual_hz, 0.0);
        let steps_when_stopped = simulation.metrics().total_steps;
        assert!(steps_when_stopped > 0);
        assert!(simulation.world().is_ok());
        assert!(
            simulation
                .world()
                .expect("world restored")
                .body(id)
                .expect("body remains in the world")
                .state()
                .transform()
                .translation()
                .x
                > 0.0
        );
        simulation.stop().await.expect("repeated stop is safe");
        tokio::time::sleep(Duration::from_secs(1)).await;
        assert_eq!(simulation.metrics().total_steps, steps_when_stopped);

        tokio::time::advance(Duration::from_secs(2)).await;
        simulation.start().expect("restart");
        assert_eq!(simulation.metrics().actual_hz, 0.0);
        tokio::time::sleep(Duration::from_millis(5)).await;
        assert_eq!(simulation.metrics().total_steps, steps_when_stopped);
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(simulation.metrics().total_steps > steps_when_stopped);
        let mut stop = Box::pin(simulation.stop());
        let waker = Waker::noop();
        let mut context = Context::from_waker(waker);
        assert!(matches!(stop.as_mut().poll(&mut context), Poll::Pending));
        drop(stop);
        simulation
            .stop()
            .await
            .expect("cancelled stop can be completed later");
        let final_steps = simulation.metrics().total_steps;
        tokio::time::sleep(Duration::from_secs(1)).await;
        assert_eq!(simulation.metrics().total_steps, final_steps);
        assert_eq!(simulation.state(), SimulationState::Stopped);
    }

    #[tokio::test(start_paused = true)]
    async fn background_physics_error_is_reported_and_world_is_restored() {
        let mut world = PhysicsWorld::new();
        let tiny_mass = MassProperties::new(f64::from_bits(1), Vec3::ZERO, Mat3::IDENTITY)
            .expect("valid mass properties");
        let mut body =
            RigidBody::new_dynamic(Transform::identity(), tiny_mass).expect("valid dynamic body");
        body.apply_force(Vec3::X * f64::MAX).expect("finite force");
        world.add_body(body).expect("body insertion");
        let mut simulation = PhysicsSimulation::new(world);

        simulation.start().expect("start in runtime");
        tokio::time::sleep(Duration::from_millis(100)).await;

        assert_eq!(simulation.state(), SimulationState::Error);
        assert!(matches!(
            simulation.stop().await,
            Err(SimulationError::Scheduler(SchedulerError::PhysicsStep(_)))
        ));
        assert_eq!(simulation.world().expect("world restored").body_count(), 1);
    }
}
