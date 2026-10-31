use std::time::Duration;

use kapitsa_core::world::PhysicsWorld;
use tokio::{runtime::Handle, sync::watch, task::JoinHandle, time::Instant};

use crate::{PhysicsController, SimulationError, SimulationMetrics, SimulationState};

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
        let controller = PhysicsController::with_default_config(world);
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
        controller.set_simulation_state(SimulationState::Running);
        controller.set_simulation_time_debt(Duration::ZERO);
        self.metrics_tx.send_replace(controller.metrics());

        let metrics_tx = self.metrics_tx.clone();
        self.stop = Some(stop_tx);
        self.task = Some(runtime.spawn(async move {
            let result = run_simulation(&mut controller, stop_rx, &metrics_tx).await;
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

async fn run_simulation(
    controller: &mut PhysicsController,
    mut stop: watch::Receiver<bool>,
    metrics: &watch::Sender<SimulationMetrics>,
) -> Result<(), SimulationError> {
    let max_catch_up_steps = controller.config().max_catch_up_steps;
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
        if *stop.borrow() {
            break;
        }

        let now = Instant::now();
        accumulator = accumulator.saturating_add(now.duration_since(previous));
        previous = now;
        let mut steps_since_yield = 0;

        while accumulator >= controller.step_duration() && !*stop.borrow() {
            let step_duration = controller.step_duration();
            controller
                .step()
                .await
                .map_err(SimulationError::Scheduler)?;
            accumulator = accumulator.saturating_sub(step_duration);

            let completed_at = Instant::now();
            accumulator = accumulator.saturating_add(completed_at.duration_since(previous));
            previous = completed_at;
            controller
                .set_simulation_time_debt(accumulator.saturating_sub(controller.step_duration()));
            metrics.send_replace(controller.metrics());

            steps_since_yield += 1;
            if steps_since_yield >= max_catch_up_steps {
                tokio::task::yield_now().await;
                steps_since_yield = 0;
            }
        }

        if *stop.borrow() {
            break;
        }
    }

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

    use super::PhysicsSimulation;
    use crate::{SchedulerError, SimulationError, SimulationState};

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
