//! Simulation runtime configuration and execution APIs.

mod adaptive_step;
mod asynchronous;
mod config;
mod engine;
mod executor;
pub mod metrics;
#[cfg(test)]
mod modular_rocket_fixture;
mod planner;
#[cfg(test)]
mod sync;

#[cfg(all(test, feature = "bench-internals"))]
mod bench_counter_tests {
    use super::modular_rocket_fixture::ModularRocket;

    #[test]
    fn fixed_joint_baseline_reports_scalar_solver_work() {
        let mut rocket = ModularRocket::new(100);
        assert!(rocket.world.collider(rocket.surface_collider).is_some());
        assert_eq!(rocket.lower_parts.len() + rocket.upper_parts.len(), 100);
        assert!(!rocket.lower_tanks.is_empty() && !rocket.upper_tanks.is_empty());

        rocket
            .world
            .step(1.0 / 120.0)
            .expect("baseline physics step");
        let counters = rocket.world.physics_step_metrics().joints;

        assert_eq!(counters.prepared_rows, 99 * 6 * (3 + 8));
        assert_eq!(counters.scalar_rows_solved, 99 * 6 * (3 + 8));
        assert_eq!(counters.position_sweeps, 3);
        assert_eq!(counters.velocity_sweeps, 8);
        assert_eq!(counters.inverse_inertia_updates, 0);
        assert_eq!(counters.block_rows_solved, 0);
        assert_eq!(counters.velocity_warm_starts, 0);
        assert_eq!(counters.color_batches_processed, 0);
    }
}

pub use asynchronous::{
    BatchId, CollisionEventBatch, CollisionEventStreamError, CollisionEvents, JointBreakEventBatch,
    JointBreakEventStreamError, JointBreakEvents, PhysicsBatchError, PhysicsBatchResult,
    PhysicsBatchResultStreamError, PhysicsBatchResults, PhysicsClient, PhysicsClientError,
    PhysicsCommand, PhysicsCommandBuffer, PhysicsCommandResult, PhysicsSnapshot,
    PhysicsSubmitError, Simulation, SimulationError,
};
pub use config::{
    AdaptiveTimeStepConfig, ConfigError, SchedulerConfig, SimulationConfig, TimeStepPolicy,
};
pub use metrics::{SimulationMetrics, SimulationState, StepMeasurement, StepStats};
pub use planner::{WorkerConfig, WorkerConfigError};
