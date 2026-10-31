use std::{
    mem,
    time::{Duration, Instant},
};

use kapitsa_core::world::{PhysicsWorld, StepError};
use tokio::{runtime::Handle, task::JoinHandle};

use crate::{AdvanceStats, SchedulerConfig, SchedulerError, SchedulerMetrics, StepMeasurement};

type BlockingOutput = (PhysicsWorld, Result<(), StepError>, Duration);

enum WorldState {
    Available(PhysicsWorld),
    InFlight {
        task: JoinHandle<BlockingOutput>,
        wait_started: Instant,
        step_duration: Duration,
        step_budget: Duration,
    },
    Lost,
}

/// Owns a physics world and advances it using the caller's Tokio runtime.
pub struct PhysicsController {
    world: WorldState,
    config: SchedulerConfig,
    accumulator: Duration,
    current_hz: u32,
    active_workers: usize,
    metrics: SchedulerMetrics,
}

impl PhysicsController {
    pub fn new(world: PhysicsWorld, config: SchedulerConfig) -> Result<Self, SchedulerError> {
        config.validate()?;
        let current_hz = config.target_hz;
        let active_workers = 1;
        Ok(Self {
            world: WorldState::Available(world),
            config,
            accumulator: Duration::ZERO,
            current_hz,
            active_workers,
            metrics: SchedulerMetrics::new(current_hz, active_workers),
        })
    }

    /// Borrows the world between calls to [`Self::advance`].
    pub fn world(&self) -> Result<&PhysicsWorld, SchedulerError> {
        match &self.world {
            WorldState::Available(world) => Ok(world),
            WorldState::InFlight { .. } => Err(SchedulerError::WorldUnavailable),
            WorldState::Lost => Err(SchedulerError::WorldLost),
        }
    }

    /// Mutates the world between calls to [`Self::advance`].
    pub fn world_mut(&mut self) -> Result<&mut PhysicsWorld, SchedulerError> {
        match &mut self.world {
            WorldState::Available(world) => Ok(world),
            WorldState::InFlight { .. } => Err(SchedulerError::WorldUnavailable),
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

    pub fn simulation_time_debt(&self) -> Duration {
        self.accumulator
    }

    /// Adds elapsed wall time and runs a bounded number of fixed physics steps.
    /// Remaining time is retained as simulation-time debt for the next call.
    pub async fn advance(&mut self, elapsed: Duration) -> Result<AdvanceStats, SchedulerError> {
        Handle::try_current().map_err(|_| SchedulerError::MissingRuntime)?;
        self.accumulator = self
            .accumulator
            .checked_add(elapsed)
            .ok_or(SchedulerError::SimulationTimeOverflow)?;
        self.update_metrics_state();

        let mut steps_completed = 0;
        let mut budget_overruns = 0;
        let mut step_duration_total = Duration::ZERO;
        let mut task_wait_total = Duration::ZERO;

        if matches!(self.world, WorldState::InFlight { .. }) {
            let measurement = self.complete_pending_step().await?;
            self.add_measurement(
                measurement,
                &mut steps_completed,
                &mut budget_overruns,
                &mut step_duration_total,
                &mut task_wait_total,
            );
        }

        while steps_completed < self.config.max_catch_up_steps {
            let step_duration = self.step_duration();
            if self.accumulator < step_duration {
                break;
            }
            self.start_step(step_duration)?;
            let measurement = self.complete_pending_step().await?;
            self.add_measurement(
                measurement,
                &mut steps_completed,
                &mut budget_overruns,
                &mut step_duration_total,
                &mut task_wait_total,
            );
        }

        self.update_metrics_state();
        Ok(AdvanceStats::snapshot(
            steps_completed,
            budget_overruns,
            step_duration_total,
            task_wait_total,
            &self.metrics,
        ))
    }

    fn step_duration(&self) -> Duration {
        self.config.step_duration(self.current_hz)
    }

    fn start_step(&mut self, step_duration: Duration) -> Result<(), SchedulerError> {
        let mut world = match mem::replace(&mut self.world, WorldState::Lost) {
            WorldState::Available(world) => world,
            state @ WorldState::InFlight { .. } => {
                self.world = state;
                return Err(SchedulerError::WorldUnavailable);
            }
            state @ WorldState::Lost => {
                self.world = state;
                return Err(SchedulerError::WorldLost);
            }
        };

        let step_budget = step_duration.mul_f64(self.config.target_utilization);
        let wait_started = Instant::now();
        let task = tokio::task::spawn_blocking(move || {
            let step_started = Instant::now();
            let result = world.step(step_duration.as_secs_f64());
            let worker_duration = step_started.elapsed();
            (world, result, worker_duration)
        });
        self.world = WorldState::InFlight {
            task,
            wait_started,
            step_duration,
            step_budget,
        };
        Ok(())
    }

    async fn complete_pending_step(&mut self) -> Result<StepMeasurement, SchedulerError> {
        let completion = match &mut self.world {
            WorldState::InFlight { task, .. } => task.await,
            WorldState::Available(_) => return Err(SchedulerError::WorldUnavailable),
            WorldState::Lost => return Err(SchedulerError::WorldLost),
        };
        let (step_duration, step_budget, task_wait_duration) = match &self.world {
            WorldState::InFlight {
                wait_started,
                step_duration,
                step_budget,
                ..
            } => (*step_duration, *step_budget, wait_started.elapsed()),
            WorldState::Available(_) => return Err(SchedulerError::WorldUnavailable),
            WorldState::Lost => return Err(SchedulerError::WorldLost),
        };

        match completion {
            Ok((world, Ok(()), worker_duration)) => {
                self.world = WorldState::Available(world);
                self.accumulator = self
                    .accumulator
                    .checked_sub(step_duration)
                    .ok_or(SchedulerError::SimulationTimeOverflow)?;
                self.update_metrics_state();
                Ok(StepMeasurement {
                    step_duration: worker_duration,
                    task_wait_duration,
                    step_budget,
                })
            }
            Ok((world, Err(error), _)) => {
                self.world = WorldState::Available(world);
                self.update_metrics_state();
                Err(SchedulerError::from(error))
            }
            Err(error) => {
                self.world = WorldState::Lost;
                self.update_metrics_state();
                Err(SchedulerError::BlockingTaskFailed(error.to_string()))
            }
        }
    }

    fn add_measurement(
        &mut self,
        measurement: StepMeasurement,
        steps_completed: &mut usize,
        budget_overruns: &mut usize,
        step_duration_total: &mut Duration,
        task_wait_total: &mut Duration,
    ) {
        *steps_completed += 1;
        if self.metrics.record_step(measurement) {
            *budget_overruns += 1;
        }
        *step_duration_total = step_duration_total.saturating_add(measurement.step_duration);
        *task_wait_total = task_wait_total.saturating_add(measurement.task_wait_duration);
    }

    fn update_metrics_state(&mut self) {
        self.metrics
            .update_scheduler_state(self.current_hz, self.active_workers, self.accumulator);
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

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
            ..SchedulerConfig::default()
        }
    }

    #[test]
    fn advance_carries_partial_elapsed_time_and_steps_on_the_blocking_pool() {
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

        let first = runtime
            .block_on(controller.advance(Duration::from_millis(8)))
            .expect("partial advance");
        assert_eq!(first.steps_completed, 0);
        assert_eq!(first.simulation_time_debt, Duration::from_millis(8));
        assert_eq!(
            controller
                .world()
                .expect("world available")
                .body(id)
                .expect("body exists")
                .state()
                .transform()
                .translation(),
            Vec3::ZERO
        );

        let second = runtime
            .block_on(controller.advance(Duration::from_millis(9)))
            .expect("one fixed step");
        assert_eq!(second.steps_completed, 1);
        assert!(second.step_duration > Duration::ZERO);
        assert!(second.task_wait_duration >= second.step_duration);
        assert_eq!(controller.metrics().total_steps(), 1);
        let translation = controller
            .world()
            .expect("world available")
            .body(id)
            .expect("body exists")
            .state()
            .transform()
            .translation();
        assert!((translation.x - 1.0 / 60.0).abs() <= 1.0e-9);
    }

    #[test]
    fn catch_up_is_bounded_and_retains_simulation_time_debt() {
        let runtime = runtime();
        let mut controller =
            PhysicsController::new(moving_kinematic_world(), config(2)).expect("valid controller");

        let stats = runtime
            .block_on(controller.advance(Duration::from_secs(1)))
            .expect("bounded advance");

        assert_eq!(stats.steps_completed, 2);
        assert!(stats.simulation_time_debt >= Duration::from_millis(966));
        assert_eq!(controller.metrics().total_steps(), 2);
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
        let elapsed = Duration::from_secs_f64(1.0 / 60.0);

        assert!(matches!(
            runtime.block_on(controller.advance(elapsed)),
            Err(SchedulerError::PhysicsStep(_))
        ));
        assert!(
            controller
                .world()
                .expect("world restored")
                .body(id)
                .is_some()
        );
        assert!(controller.simulation_time_debt() >= elapsed);
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
    fn world_can_be_configured_between_advances() {
        let runtime = runtime();
        let mut controller =
            PhysicsController::new(PhysicsWorld::new(), config(4)).expect("valid controller");

        let id = controller
            .world_mut()
            .expect("world available")
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("body insertion");
        runtime
            .block_on(controller.advance(Duration::from_secs_f64(1.0 / 60.0)))
            .expect("valid advance");

        assert!(
            controller
                .world()
                .expect("world available")
                .body(id)
                .is_some()
        );
    }
}
