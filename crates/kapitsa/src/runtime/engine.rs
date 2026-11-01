use std::collections::VecDeque;
use std::time::{Duration, Instant};

use kapitsa_core::{
    collision::CollisionError,
    execution::StepExecutor,
    rigid_body::BodyType,
    world::{BodyStep, PhysicsWorld, StepError},
};

#[cfg(test)]
use super::config::ConfigError;
use super::{
    config::SimulationConfig,
    metrics::{SimulationMetrics, SimulationState, StepStats},
};

const METRICS_WINDOW: usize = 120;

pub(crate) struct StepEngine<E: StepExecutor> {
    world: PhysicsWorld,
    executor: E,
    config: SimulationConfig,
    active_workers: usize,
    rayon_pool_size: Option<usize>,
    bucket_buffers: Vec<Vec<BodyStep>>,
    recent_step_durations: VecDeque<Duration>,
    metrics: SimulationMetrics,
}

impl<E: StepExecutor> StepEngine<E> {
    #[cfg(test)]
    pub(crate) fn new(
        world: PhysicsWorld,
        config: SimulationConfig,
        executor: E,
    ) -> Result<Self, ConfigError> {
        config.validate()?;
        Ok(Self::new_validated(world, config, executor))
    }

    pub(crate) fn new_validated(
        mut world: PhysicsWorld,
        config: SimulationConfig,
        executor: E,
    ) -> Self {
        if let Some(gravity) = config.central_gravity {
            world.set_central_gravity(Some(gravity));
        }
        let step_budget = config.step_budget();
        let metrics = SimulationMetrics {
            step_budget,
            active_workers: 1,
            requested_time_multiplier: 1.0,
            ..SimulationMetrics::default()
        };
        Self {
            world,
            executor,
            config,
            active_workers: 1,
            rayon_pool_size: None,
            bucket_buffers: Vec::new(),
            recent_step_durations: VecDeque::with_capacity(METRICS_WINDOW),
            metrics,
        }
    }

    pub(crate) fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    pub(crate) fn world_mut(&mut self) -> &mut PhysicsWorld {
        &mut self.world
    }

    pub(crate) fn metrics(&self) -> &SimulationMetrics {
        &self.metrics
    }

    pub(crate) fn set_parallelism(&mut self, active_workers: usize, pool_size: Option<usize>) {
        self.active_workers = active_workers.max(1);
        self.rayon_pool_size = pool_size;
        self.metrics.active_workers = self.active_workers;
        self.metrics.rayon_pool_size = pool_size;
    }

    pub(crate) fn record_task_wait(&mut self, stats: &mut StepStats, duration: Duration) {
        stats.task_wait_duration = duration;
        self.metrics.task_wait_duration = duration;
    }

    pub(crate) fn set_budget_status(&mut self, warning: bool, overloaded: bool) {
        self.metrics.budget_warning = warning;
        self.metrics.overloaded = overloaded;
    }

    pub(crate) fn set_runtime_status(
        &mut self,
        state: SimulationState,
        simulation_time_debt: Duration,
    ) {
        self.metrics.state = state;
        self.metrics.simulation_time_debt = simulation_time_debt;
    }

    #[cfg(test)]
    pub(crate) fn into_world(self) -> PhysicsWorld {
        self.world
    }

    pub(crate) fn step(&mut self, task_wait_duration: Duration) -> Result<StepStats, StepError> {
        let started = Instant::now();
        let result = self.run_step();
        let step_duration = started.elapsed();
        if let Err(error) = result {
            self.metrics.state = SimulationState::Error;
            return Err(error);
        }

        if self.recent_step_durations.len() == METRICS_WINDOW {
            self.recent_step_durations.pop_front();
        }
        self.recent_step_durations.push_back(step_duration);

        let average_step_duration = average_duration(&self.recent_step_durations);
        let max_step_duration = self
            .recent_step_durations
            .iter()
            .copied()
            .max()
            .unwrap_or(Duration::ZERO);
        let step_budget = self.config.step_budget();
        let budget_usage = step_duration.as_secs_f64() / step_budget.as_secs_f64();
        let budget_warning = step_duration >= step_budget.mul_f64(0.75);
        let overloaded = step_duration >= step_budget.mul_f64(0.85);
        let deadline_missed = step_duration > step_budget;

        self.metrics.state = SimulationState::Stopped;
        self.metrics.last_step_duration = step_duration;
        self.metrics.task_wait_duration = task_wait_duration;
        self.metrics.average_step_duration = average_step_duration;
        self.metrics.max_step_duration = max_step_duration;
        self.metrics.step_budget = step_budget;
        self.metrics.budget_usage = budget_usage;
        self.metrics.budget_warning = budget_warning;
        self.metrics.overloaded = overloaded;
        self.metrics.active_workers = self.active_workers;
        self.metrics.rayon_pool_size = self.rayon_pool_size;
        self.metrics.total_steps = self.metrics.total_steps.saturating_add(1);
        self.metrics.deadline_misses = self
            .metrics
            .deadline_misses
            .saturating_add(u64::from(deadline_missed));

        Ok(StepStats {
            step_duration,
            task_wait_duration,
            step_budget,
            budget_usage,
            budget_warning,
            overloaded,
            deadline_missed,
            active_workers: self.active_workers,
            average_step_duration,
            max_step_duration,
        })
    }

    fn run_step(&mut self) -> Result<(), StepError> {
        let body_count = self.world.body_count();
        let worker_count = self.active_workers.min(body_count).max(1);
        if worker_count <= 1 {
            return self.world.step_with_executor(
                self.config.fixed_step_seconds(),
                &self.executor,
                worker_count,
            );
        }

        let mut active_bodies = 0;
        let threshold = body_count / 2;
        for (_, body) in self.world.bodies() {
            if body.physics().body_type() != BodyType::Static && !body.state().is_sleeping() {
                active_bodies += 1;
                if active_bodies > threshold {
                    break;
                }
            }
        }
        if active_bodies <= threshold {
            return self.world.step_with_executor(
                self.config.fixed_step_seconds(),
                &self.executor,
                worker_count,
            );
        }

        let max_buckets = worker_count.saturating_mul(self.config.scheduler.max_batches_per_worker);
        let size_limited_buckets = body_count
            .div_ceil(self.config.scheduler.min_batch_size)
            .max(worker_count);
        let bucket_count = size_limited_buckets.min(max_buckets);
        if self.bucket_buffers.len() < bucket_count {
            self.bucket_buffers
                .try_reserve(bucket_count - self.bucket_buffers.len())
                .map_err(|_| StepError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
            self.bucket_buffers.resize_with(bucket_count, Vec::new);
        }
        let buckets = &mut self.bucket_buffers[..bucket_count];
        let dt = self.config.fixed_step_seconds();
        self.world
            .step_with_shards(dt, buckets, &self.executor, worker_count)
    }
}

fn average_duration(durations: &VecDeque<Duration>) -> Duration {
    let total = durations
        .iter()
        .copied()
        .fold(Duration::ZERO, Duration::saturating_add);
    if durations.is_empty() {
        Duration::ZERO
    } else {
        total.div_f64(durations.len() as f64)
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "bench-internals")]
    use std::time::Instant;
    use std::{
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
        time::Duration,
    };

    use kapitsa_core::gravity::CentralGravity;
    use kapitsa_core::{
        execution::{ExecutionError, StepExecutor},
        mass::MassProperties,
        rigid_body::RigidBody,
        transform::Transform,
        world::{BodyId, ForceSourceId, PhysicsWorld},
    };
    use kapitsa_math::{Mat3, Quat, Vec3};
    use tokio::runtime::Handle;

    use super::StepEngine;
    use crate::runtime::{
        SimulationConfig,
        executor::{RayonExecutor, RayonPool},
    };

    struct CountingExecutor {
        inner: RayonExecutor,
        calls: Arc<AtomicUsize>,
    }

    impl StepExecutor for CountingExecutor {
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
            self.calls.fetch_add(1, Ordering::Relaxed);
            self.inner.for_each_chunk_mut(values, chunk_size, operation)
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
            self.calls.fetch_add(1, Ordering::Relaxed);
            self.inner
                .for_each_chunk_layout_mut(values, chunk_sizes, operation)
        }
    }

    fn world_with_force_sources() -> (PhysicsWorld, Vec<BodyId>) {
        let mut world = PhysicsWorld::new();
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
        let bodies = (0..4)
            .map(|index| {
                world
                    .add_body(
                        RigidBody::new_dynamic(
                            Transform::new(Vec3::X * (index as f64 * 4.0), Quat::IDENTITY)
                                .expect("transform"),
                            mass,
                        )
                        .expect("body"),
                    )
                    .expect("body id")
            })
            .collect::<Vec<_>>();
        world
            .add_force_source(bodies[0], Vec3::ZERO, Vec3::X * 1.0e16)
            .expect("first force source");
        world
            .add_force_source(bodies[0], Vec3::ZERO, Vec3::X * -1.0e16)
            .expect("second force source");
        world
            .add_force_source(bodies[0], Vec3::ZERO, Vec3::X)
            .expect("third force source");
        world
            .add_force_source(bodies[0], Vec3::Y, Vec3::X * 2.0)
            .expect("offset force source");
        world
            .add_force_source(bodies[1], Vec3::ZERO, Vec3::Y * 0.25)
            .expect("source on second body");
        (world, bodies)
    }

    fn world_with_one_force_source() -> (PhysicsWorld, Vec<BodyId>, ForceSourceId) {
        let mut world = PhysicsWorld::new();
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
        let bodies = (0..4)
            .map(|index| {
                world
                    .add_body(
                        RigidBody::new_dynamic(
                            Transform::new(Vec3::X * (index as f64 * 4.0), Quat::IDENTITY)
                                .expect("transform"),
                            mass,
                        )
                        .expect("body"),
                    )
                    .expect("body id")
            })
            .collect::<Vec<_>>();
        let source = world
            .add_force_source(bodies[0], Vec3::ZERO, Vec3::X * 2.0)
            .expect("force source");
        (world, bodies, source)
    }

    fn assert_vec3_bits_eq(actual: Vec3, expected: Vec3) {
        assert_eq!(actual.x.to_bits(), expected.x.to_bits());
        assert_eq!(actual.y.to_bits(), expected.y.to_bits());
        assert_eq!(actual.z.to_bits(), expected.z.to_bits());
    }

    fn assert_quat_bits_eq(actual: Quat, expected: Quat) {
        assert_eq!(actual.x.to_bits(), expected.x.to_bits());
        assert_eq!(actual.y.to_bits(), expected.y.to_bits());
        assert_eq!(actual.z.to_bits(), expected.z.to_bits());
        assert_eq!(actual.w.to_bits(), expected.w.to_bits());
    }

    fn assert_states_match(
        actual: &PhysicsWorld,
        actual_bodies: &[BodyId],
        expected: &PhysicsWorld,
        expected_bodies: &[BodyId],
    ) {
        for (actual_id, expected_id) in actual_bodies.iter().zip(expected_bodies) {
            let actual = actual.body(*actual_id).expect("actual body");
            let expected = expected.body(*expected_id).expect("expected body");
            assert_vec3_bits_eq(
                actual.state().transform().translation(),
                expected.state().transform().translation(),
            );
            assert_quat_bits_eq(
                actual.state().transform().rotation(),
                expected.state().transform().rotation(),
            );
            assert_vec3_bits_eq(
                actual.state().linear_velocity(),
                expected.state().linear_velocity(),
            );
            assert_vec3_bits_eq(
                actual.state().angular_velocity(),
                expected.state().angular_velocity(),
            );
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn sharded_step_matches_sequential_with_multiple_sources_and_torque() {
        let (mut sequential_world, sequential_bodies) = world_with_force_sources();
        let (sharded_world, sharded_bodies) = world_with_force_sources();
        let config = SimulationConfig {
            tick_rate: 10,
            ..SimulationConfig::default()
        };
        sequential_world
            .step(config.fixed_step_seconds())
            .expect("sequential step");

        let mut pool = RayonPool::new(&Handle::current(), 4).expect("Rayon pool");
        let executor_calls = Arc::new(AtomicUsize::new(0));
        let executor = CountingExecutor {
            inner: pool.executor().expect("Rayon executor"),
            calls: Arc::clone(&executor_calls),
        };
        let mut engine = StepEngine::new(sharded_world, config, executor).expect("step engine");
        engine.set_parallelism(4, Some(4));
        engine.step(Duration::ZERO).expect("sharded step");
        let sharded_world = engine.into_world();
        pool.shutdown().await.expect("Rayon pool shutdown");

        assert!(executor_calls.load(Ordering::Relaxed) > 0);
        assert_states_match(
            &sharded_world,
            &sharded_bodies,
            &sequential_world,
            &sequential_bodies,
        );
        assert_ne!(
            sharded_world
                .body(sharded_bodies[0])
                .expect("body")
                .state()
                .angular_velocity(),
            Vec3::ZERO
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn sharded_step_matches_sequential_with_gravity_and_force_sources() {
        let (mut sequential_world, sequential_bodies) = world_with_force_sources();
        let (sharded_world, sharded_bodies) = world_with_force_sources();
        let gravity =
            CentralGravity::new(Vec3::new(-100.0, 0.0, 0.0), 10_000.0).expect("gravity field");
        sequential_world.set_central_gravity(Some(gravity));
        let config = SimulationConfig {
            tick_rate: 10,
            central_gravity: Some(gravity),
            ..SimulationConfig::default()
        };
        sequential_world
            .step(config.fixed_step_seconds())
            .expect("sequential step");

        let mut pool = RayonPool::new(&Handle::current(), 4).expect("Rayon pool");
        let executor = pool.executor().expect("Rayon executor");
        let mut engine = StepEngine::new(sharded_world, config, executor).expect("step engine");
        engine.set_parallelism(4, Some(4));
        engine.step(Duration::ZERO).expect("sharded step");
        let sharded_world = engine.into_world();
        pool.shutdown().await.expect("Rayon pool shutdown");

        assert_states_match(
            &sharded_world,
            &sharded_bodies,
            &sequential_world,
            &sequential_bodies,
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn force_source_enable_and_active_body_path_switch_match_sequential_steps() {
        let (mut sequential_world, sequential_bodies, sequential_source) =
            world_with_one_force_source();
        let (sharded_world, sharded_bodies, sharded_source) = world_with_one_force_source();
        let config = SimulationConfig {
            tick_rate: 10,
            ..SimulationConfig::default()
        };
        let mut pool = RayonPool::new(&Handle::current(), 4).expect("Rayon pool");
        let executor = pool.executor().expect("Rayon executor");
        let mut engine =
            StepEngine::new(sharded_world, config.clone(), executor).expect("step engine");
        engine.set_parallelism(4, Some(4));

        sequential_world
            .set_force_source_enabled(sequential_source, false)
            .expect("disable sequential source");
        engine
            .world_mut()
            .set_force_source_enabled(sharded_source, false)
            .expect("disable sharded source");
        sequential_world
            .step(config.fixed_step_seconds())
            .expect("disabled sequential step");
        engine.step(Duration::ZERO).expect("disabled runtime step");
        assert_states_match(
            engine.world(),
            &sharded_bodies,
            &sequential_world,
            &sequential_bodies,
        );
        assert_eq!(
            engine
                .world()
                .body(sharded_bodies[0])
                .expect("body")
                .state()
                .linear_velocity(),
            Vec3::ZERO
        );

        sequential_world
            .set_force_source_enabled(sequential_source, true)
            .expect("enable sequential source");
        engine
            .world_mut()
            .set_force_source_enabled(sharded_source, true)
            .expect("enable sharded source");
        sequential_world
            .step(config.fixed_step_seconds())
            .expect("enabled sequential step");
        engine.step(Duration::ZERO).expect("enabled runtime step");
        assert_states_match(
            engine.world(),
            &sharded_bodies,
            &sequential_world,
            &sequential_bodies,
        );

        for id in &sequential_bodies[2..] {
            sequential_world
                .body_mut(*id)
                .expect("sequential body")
                .sleep()
                .expect("sleep body");
        }
        for id in &sharded_bodies[2..] {
            engine
                .world_mut()
                .body_mut(*id)
                .expect("sharded body")
                .sleep()
                .expect("sleep body");
        }
        sequential_world
            .step(config.fixed_step_seconds())
            .expect("ordinary runtime path");
        engine.step(Duration::ZERO).expect("ordinary runtime path");
        assert_states_match(
            engine.world(),
            &sharded_bodies,
            &sequential_world,
            &sequential_bodies,
        );

        sequential_world
            .body_mut(sequential_bodies[2])
            .expect("sequential body")
            .set_linear_velocity(Vec3::Y * 0.1)
            .expect("wake sequential body");
        engine
            .world_mut()
            .body_mut(sharded_bodies[2])
            .expect("sharded body")
            .set_linear_velocity(Vec3::Y * 0.1)
            .expect("wake sharded body");
        sequential_world
            .step(config.fixed_step_seconds())
            .expect("sharded runtime path after wake");
        engine
            .step(Duration::ZERO)
            .expect("sharded runtime path after wake");
        assert_states_match(
            engine.world(),
            &sharded_bodies,
            &sequential_world,
            &sequential_bodies,
        );

        let _world = engine.into_world();
        pool.shutdown().await.expect("Rayon pool shutdown");
    }

    #[cfg(feature = "bench-internals")]
    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    #[ignore = "Release sharded ForceSource scaling benchmark; run with bench-internals"]
    async fn benchmark_sharded_force_source_scaling() -> Result<(), Box<dyn std::error::Error>> {
        const SAMPLE_STEPS: usize = 32;
        const WARMUP_STEPS: usize = 16;
        let scenarios = [
            ("1 source / 1 body", 1, 1, false, false),
            ("0 sources / 4 body control", 4, 0, false, false),
            ("0 sources / 128 body control", 128, 0, false, false),
            ("128 sources / 128 bodies", 128, 128, false, false),
            ("0 sources / 1024 body control", 1_024, 0, false, false),
            ("1024 sources / 1024 bodies", 1_024, 1_024, false, false),
            ("0 sources / 8192 body control", 8_192, 0, false, false),
            ("8192 sources / 8192 bodies", 8_192, 8_192, false, false),
            ("0 sources / one-target control", 4, 0, false, false),
            ("8192 sources / one target", 4, 8_192, true, false),
            (
                "0 sources / 1024 body + gravity control",
                1_024,
                0,
                false,
                true,
            ),
            ("1024 sources / bodies + gravity", 1_024, 1_024, false, true),
        ];
        let mut pool = RayonPool::new(&Handle::current(), 4).expect("Rayon pool");
        println!(
            "Sharded persistent-force benchmark: 4 Rayon workers, 16 warm-up steps, 5 samples x {SAMPLE_STEPS} steps"
        );

        for (label, body_count, source_count, one_target, with_gravity) in scenarios {
            let mut world = PhysicsWorld::new();
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
            let bodies = (0..body_count)
                .map(|index| {
                    world
                        .add_body(
                            RigidBody::new_dynamic(
                                Transform::new(
                                    Vec3::new(index as f64 * 4.0, 100.0, 0.0),
                                    Quat::IDENTITY,
                                )
                                .expect("transform"),
                                mass,
                            )
                            .expect("body"),
                        )
                        .expect("body id")
                })
                .collect::<Vec<_>>();
            for source_index in 0..source_count {
                let body_index = if one_target {
                    0
                } else {
                    source_index % body_count
                };
                world.add_force_source(
                    bodies[body_index],
                    Vec3::Y * 0.01,
                    Vec3::new(1.0e-6, 2.0e-6, -1.0e-6),
                )?;
            }

            let config = SimulationConfig {
                tick_rate: 120,
                central_gravity: with_gravity
                    .then(|| CentralGravity::new(Vec3::ZERO, 1_000.0).expect("valid gravity")),
                ..SimulationConfig::default()
            };
            let executor = pool.executor().expect("Rayon executor");
            let mut engine = StepEngine::new(world, config, executor)?;
            engine.set_parallelism(4, Some(4));
            for _ in 0..WARMUP_STEPS {
                engine.step(Duration::ZERO)?;
            }

            let mut samples = Vec::with_capacity(5);
            for _ in 0..5 {
                let started = Instant::now();
                for _ in 0..SAMPLE_STEPS {
                    engine.step(Duration::ZERO)?;
                }
                samples.push(started.elapsed().as_secs_f64() * 1.0e6 / SAMPLE_STEPS as f64);
            }
            samples.sort_by(f64::total_cmp);

            crate::runtime::asynchronous::marshalling::allocation_tracking::start();
            for _ in 0..SAMPLE_STEPS {
                engine.step(Duration::ZERO)?;
            }
            let (allocations, bytes) =
                crate::runtime::asynchronous::marshalling::allocation_tracking::counts();
            let (_, _, peak_extra, baseline_live) =
                crate::runtime::asynchronous::marshalling::allocation_tracking::stop();
            std::hint::black_box(engine.metrics().total_steps);

            println!(
                "{label}: min/median/max={:.3}/{:.3}/{:.3} us/step, allocs/step={:.3}, requested_bytes/step={:.1}, live/peak-extra={baseline_live}/{peak_extra} B",
                samples[0],
                samples[2],
                samples[4],
                allocations as f64 / SAMPLE_STEPS as f64,
                bytes as f64 / SAMPLE_STEPS as f64,
            );
            drop(engine);
        }

        pool.shutdown().await?;
        Ok(())
    }
}
