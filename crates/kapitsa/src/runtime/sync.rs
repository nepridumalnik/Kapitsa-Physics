use std::time::Duration;

use kapitsa_core::{
    execution::{ExecutionError, SequentialExecutor, StepExecutor},
    world::{PhysicsWorld, StepError},
};

use super::{
    config::{ConfigError, SimulationConfig},
    engine::StepEngine,
    metrics::{SimulationMetrics, StepStats},
};

/// Runs physics work in the caller's thread.
#[derive(Clone, Copy, Debug, Default)]
pub struct SyncExecutor;

impl StepExecutor for SyncExecutor {
    fn for_each_chunk_mut<T, F>(
        &self,
        values: &mut [T],
        chunk_size: usize,
        operation: F,
    ) -> Result<(), ExecutionError>
    where
        T: Send,
        F: Fn(usize, &mut [T]) + Send + Sync,
    {
        SequentialExecutor.for_each_chunk_mut(values, chunk_size, operation)
    }

    fn for_each_chunk_layout_mut<T, F>(
        &self,
        values: &mut [T],
        chunk_sizes: &[usize],
        operation: F,
    ) -> Result<(), ExecutionError>
    where
        T: Send,
        F: Fn(usize, &mut [T]) + Send + Sync,
    {
        SequentialExecutor.for_each_chunk_layout_mut(values, chunk_sizes, operation)
    }
}

/// Owns a world and advances it one fixed step at a time on the calling thread.
pub struct Simulation {
    engine: StepEngine<SyncExecutor>,
}

impl Simulation {
    pub fn new(world: PhysicsWorld, config: SimulationConfig) -> Result<Self, ConfigError> {
        Ok(Self {
            engine: StepEngine::new(world, config, SyncExecutor)?,
        })
    }

    /// Runs exactly one fixed physics step before returning.
    pub fn step(&mut self) -> Result<StepStats, StepError> {
        self.engine.step(Duration::ZERO)
    }

    pub fn world(&self) -> &PhysicsWorld {
        self.engine.world()
    }

    pub fn world_mut(&mut self) -> &mut PhysicsWorld {
        self.engine.world_mut()
    }

    pub fn config(&self) -> &SimulationConfig {
        self.engine.config()
    }

    pub fn metrics(&self) -> &SimulationMetrics {
        self.engine.metrics()
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use kapitsa_core::{rigid_body::RigidBody, transform::Transform, world::PhysicsWorld};

    use crate::runtime::{ConfigError, SimulationConfig};

    use super::Simulation;

    fn world_with_static_body() -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("static body should be valid");
        world
    }

    #[test]
    fn one_step_matches_physics_world_and_records_metrics() {
        let config = SimulationConfig::default();
        let mut expected = world_with_static_body();
        let mut simulation =
            Simulation::new(world_with_static_body(), config.clone()).expect("valid simulation");

        expected
            .step(config.fixed_step_seconds())
            .expect("world step should succeed");
        let stats = simulation.step().expect("simulation step should succeed");

        assert!(
            simulation
                .world()
                .bodies()
                .map(|(_, body)| body)
                .zip(expected.bodies())
                .all(|(body, (_, expected_body))| body == expected_body)
        );
        assert_eq!(stats.task_wait_duration, Duration::ZERO);
        assert_eq!(stats.active_workers, 1);
        assert_eq!(stats.step_budget, config.step_budget());
        assert_eq!(simulation.metrics().total_steps, 1);
        assert_eq!(simulation.metrics().active_workers, 1);
        assert_eq!(simulation.metrics().rayon_pool_size, None);
    }

    #[test]
    fn rejects_invalid_configuration_before_owning_the_world() {
        let mut config = SimulationConfig::default();
        config.tick_rate = 0;

        assert!(matches!(
            Simulation::new(PhysicsWorld::new(), config),
            Err(ConfigError::InvalidTickRate)
        ));
    }

    #[test]
    fn manual_step_does_not_start_a_background_runtime() {
        let mut simulation = Simulation::new(PhysicsWorld::new(), SimulationConfig::default())
            .expect("valid simulation");

        simulation.step().expect("step should succeed");

        assert_eq!(simulation.metrics().total_steps, 1);
        assert_eq!(
            simulation.metrics().state,
            crate::runtime::SimulationState::Stopped
        );
    }
}
