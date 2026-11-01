use std::{
    collections::HashMap,
    future::Future,
    hint::black_box,
    io,
    num::NonZeroUsize,
    pin::Pin,
    sync::{
        Arc,
        atomic::{AtomicU64, AtomicUsize, Ordering},
    },
    task::{Context, Poll, Wake, Waker},
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::{BodyId, PhysicsWorld},
};
use kapitsa_math::{Mat3, Quat, Vec3};
use tokio::sync::{broadcast, oneshot, watch};

use super::{
    COLLISION_EVENT_BATCH_CAPACITY, COMMAND_QUEUE_CAPACITY, CollisionEventBatch, CommandBufferPool,
    CommandQueue, PHYSICS_BATCH_RESULT_CAPACITY, PhysicsClient, PhysicsSnapshot, Simulation,
    SimulationEvents, apply_queued_commands, publish_command_results, publish_runtime_metrics,
    wait_for_scaled_step,
};
use crate::runtime::{
    PhysicsBatchResults, PhysicsCommandBuffer, PhysicsSubmitError, SimulationConfig, WorkerConfig,
};

mod mission {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/examples/rocket_flight/mission.rs"
    ));
}

#[derive(Clone)]
struct FlightSample {
    elapsed: Duration,
    step_times: Vec<Duration>,
    simulation_seconds: f64,
    maximum_altitude: f64,
    maximum_speed: f64,
    collision_count: u64,
    reference: mission::FlightReference,
    starting_energy: f64,
}

fn timed_headless_flight() -> mission::MissionResult<FlightSample> {
    let mut flight = mission::HeadlessFlight::new()?;
    let mut step_times =
        Vec::with_capacity((mission::MAX_FLIGHT_SECONDS / mission::STEP_SECONDS) as usize);
    let started = Instant::now();
    while !flight.is_complete() {
        let step_started = Instant::now();
        black_box(flight.step_once()?);
        step_times.push(step_started.elapsed());
        black_box(flight.world.body_count());
    }
    let elapsed = started.elapsed();
    let reference = flight
        .ballistic_reference
        .ok_or_else(|| std::io::Error::other("missing ballistic reference point"))?;
    let starting_energy = flight
        .ballistic_energy_start
        .ok_or_else(|| std::io::Error::other("missing coast energy sample"))?;
    if flight.collision_count != 2 || flight.world.body_count() != 1 {
        return Err(std::io::Error::other("mission did not remove both impacted stages").into());
    }
    Ok(FlightSample {
        elapsed,
        simulation_seconds: step_times.len() as f64 * mission::STEP_SECONDS,
        step_times,
        maximum_altitude: flight.maximum_altitude,
        maximum_speed: flight.maximum_speed,
        collision_count: flight.collision_count,
        reference,
        starting_energy,
    })
}

#[test]
#[ignore = "Release headless flight benchmark; run with bench-internals"]
fn benchmark_rocket_flight_headless() -> Result<(), Box<dyn std::error::Error>> {
    let warmup = timed_headless_flight()?;
    let mut samples = Vec::with_capacity(5);
    for _ in 0..5 {
        samples.push(timed_headless_flight()?);
    }

    let mut step_times = samples
        .iter()
        .flat_map(|sample| sample.step_times.iter().copied())
        .collect::<Vec<_>>();
    step_times.sort_unstable();
    let mut mission_times = samples
        .iter()
        .map(|sample| sample.elapsed)
        .collect::<Vec<_>>();
    mission_times.sort_unstable();
    let first = &samples[0];
    for sample in &samples[1..] {
        assert_eq!(sample.collision_count, 2);
        assert_eq!(sample.reference, first.reference);
        assert_eq!(sample.starting_energy, first.starting_energy);
    }
    let energy_drift_percent = 100.0
        * (first.reference.specific_energy - first.starting_energy).abs()
        / first.starting_energy.abs();

    println!("Headless fixed-step flight: five independent Release runs after one warm-up");
    println!(
        "steps={} sim_s={:.3} mission_ms min/median/max={:.3}/{:.3}/{:.3} steps_per_s median={:.0}",
        first.step_times.len(),
        first.simulation_seconds,
        mission_times[0].as_secs_f64() * 1.0e3,
        mission_times[2].as_secs_f64() * 1.0e3,
        mission_times[4].as_secs_f64() * 1.0e3,
        first.simulation_seconds / mission_times[2].as_secs_f64(),
    );
    println!(
        "step_cpu_us p50/p95/p99={:.3}/{:.3}/{:.3}; sim_s_per_wall_s={:.1}",
        duration_quantile_us(&step_times, 0.50),
        duration_quantile_us(&step_times, 0.95),
        duration_quantile_us(&step_times, 0.99),
        first.simulation_seconds / mission_times[2].as_secs_f64(),
    );
    println!(
        "max_altitude_m={:.6} max_speed_m_s={:.6} ballistic_t={:.3}s position={:?} speed_m_s={:.6} energy_drift_percent={:.8} impacts={}",
        first.maximum_altitude,
        first.maximum_speed,
        first.reference.simulation_seconds,
        first.reference.position,
        first.reference.velocity.length(),
        energy_drift_percent,
        first.collision_count,
    );
    println!(
        "warmup_steps={} warmup_mission_ms={:.3}",
        warmup.step_times.len(),
        warmup.elapsed.as_secs_f64() * 1.0e3,
    );

    let mut allocation_flight = mission::HeadlessFlight::new()?;
    for _ in 0..mission::TICK_RATE {
        black_box(allocation_flight.step_once()?);
    }
    let mut allocations_per_step =
        Vec::with_capacity((mission::MAX_FLIGHT_SECONDS / mission::STEP_SECONDS) as usize);
    let mut bytes_per_step = Vec::with_capacity(allocations_per_step.capacity());
    let mut total_allocations = 0usize;
    let mut total_bytes = 0usize;
    crate::runtime::asynchronous::marshalling::allocation_tracking::start();
    while !allocation_flight.is_complete() {
        crate::runtime::asynchronous::marshalling::allocation_tracking::reset_counts();
        black_box(allocation_flight.step_once()?);
        let (allocations, bytes) =
            crate::runtime::asynchronous::marshalling::allocation_tracking::counts();
        allocations_per_step.push(allocations);
        bytes_per_step.push(bytes);
        total_allocations += allocations;
        total_bytes += bytes;
    }
    let (_, _, peak_extra_live_bytes, baseline_live_bytes) =
        crate::runtime::asynchronous::marshalling::allocation_tracking::stop();
    assert_eq!(allocation_flight.collision_count, 2);
    println!(
        "post-warm-step allocation sample: steps={} allocs_per_step_mean={:.3} max={} requested_bytes_per_step_mean={:.1} p95={} max_bytes_per_step={} setup_live_heap_bytes={} peak_extra_live_bytes={} peak_live_heap_bytes={}",
        allocations_per_step.len(),
        total_allocations as f64 / allocations_per_step.len() as f64,
        allocations_per_step.iter().copied().max().unwrap_or(0),
        total_bytes as f64 / bytes_per_step.len() as f64,
        usize_quantile(&bytes_per_step, 0.95),
        bytes_per_step.iter().copied().max().unwrap_or(0),
        baseline_live_bytes,
        peak_extra_live_bytes,
        baseline_live_bytes + peak_extra_live_bytes,
    );
    Ok(())
}

fn duration_quantile_us(values: &[Duration], quantile: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let index = ((sorted.len() - 1) as f64 * quantile).round() as usize;
    sorted[index].as_secs_f64() * 1.0e6
}

fn usize_quantile(values: &[usize], quantile: f64) -> usize {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    sorted[((sorted.len() - 1) as f64 * quantile).round() as usize]
}

const RUNTIME_SAMPLE: Duration = Duration::from_millis(100);
const RUNTIME_REPEATS: usize = 3;
const COMMAND_WRITERS: usize = 8;
const COMMAND_BATCHES_PER_WRITER: usize = 4;

#[derive(Default)]
struct EventCounts {
    collisions: AtomicU64,
    lost_batches: AtomicU64,
}

struct RuntimeReport {
    label: String,
    body_count: usize,
    dynamic_count: usize,
    collider_count: usize,
    source_count: usize,
    requested_warp: f64,
    steps: u64,
    simulated_seconds: f64,
    wall_seconds: f64,
    actual_warp: f64,
    cpu_samples: Vec<Duration>,
    allocation_calls: usize,
    requested_bytes: usize,
    peak_extra_live_bytes: usize,
    baseline_live_bytes: usize,
    deadlines_missed: u64,
    peak_time_debt: Duration,
    collisions: u64,
    lost_event_batches: u64,
    commands: CommandReport,
    settled: bool,
    allocation_steps: u64,
    active_workers: usize,
    rayon_pool_size: Option<usize>,
}

struct CommandReport {
    batches: usize,
    commands: usize,
    queue_full: usize,
    failures: usize,
    ack_latencies: Vec<Duration>,
}

fn benchmark_config() -> SimulationConfig {
    SimulationConfig {
        tick_rate: 120,
        max_catch_up_steps: 8,
        workers: WorkerConfig {
            max_workers: NonZeroUsize::new(4),
            scale_down_at: 0.000_001,
            ..WorkerConfig::default()
        },
        ..SimulationConfig::default()
    }
}

fn benchmark_body(index: usize, dynamic: bool) -> Result<RigidBody, Box<dyn std::error::Error>> {
    let x = index as f64 * 4.0;
    let transform = Transform::new(Vec3::new(x, 1_000.0, 0.0), Quat::IDENTITY)?;
    if dynamic {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        Ok(RigidBody::new_dynamic(transform, mass)?)
    } else {
        Ok(RigidBody::new_static(transform))
    }
}

fn populate_runtime_world(
    simulation: &mut Simulation,
    body_count: usize,
    dynamic_count: usize,
    collider_count: usize,
    source_count: usize,
) -> Result<Vec<BodyId>, Box<dyn std::error::Error>> {
    let runner = simulation
        .simulation
        .as_mut()
        .ok_or_else(|| io::Error::other("runtime did not retain its step runner"))?;
    let world = runner
        .world_mut()
        .map_err(|error| io::Error::other(error.to_string()))?;
    let mut bodies = Vec::with_capacity(body_count);
    for index in 0..body_count {
        bodies.push(world.add_body(benchmark_body(index, index < dynamic_count)?)?);
    }
    for body in bodies.iter().take(collider_count).copied() {
        let collider = Collider::new(Shape::Sphere(Sphere::new(0.35)?), Transform::identity());
        world.add_collider(body, collider)?;
    }
    if source_count > 0 {
        let body = *bodies
            .first()
            .ok_or_else(|| io::Error::other("force source fixture needs a body"))?;
        for _ in 0..source_count {
            world.add_force_source(body, Vec3::ZERO, Vec3::Y)?;
        }
    }
    Ok(bodies)
}

async fn runtime_fixture(
    label: &str,
    body_count: usize,
    dynamic_count: usize,
    collider_count: usize,
    source_count: usize,
    requested_warp: f64,
) -> Result<RuntimeReport, Box<dyn std::error::Error>> {
    let mut simulation = Simulation::new_from_config(benchmark_config())?;
    let bodies = populate_runtime_world(
        &mut simulation,
        body_count,
        dynamic_count,
        collider_count,
        source_count,
    )?;
    simulation
        .simulation
        .as_mut()
        .ok_or_else(|| io::Error::other("runtime did not retain its step runner"))?
        .set_active_workers(4)
        .map_err(|error| io::Error::other(error.to_string()))?;
    let client = simulation.client();
    let mut results = client.subscribe_batch_results();
    let event_counts = Arc::new(EventCounts::default());
    let mut collision_events = client.subscribe_collision_events();
    let listener_counts = Arc::clone(&event_counts);
    let collision_listener = tokio::spawn(async move {
        loop {
            match collision_events.next_batch().await {
                Ok(batch) => {
                    listener_counts
                        .collisions
                        .fetch_add(batch.events().len() as u64, Ordering::Relaxed);
                }
                Err(crate::runtime::CollisionEventStreamError::Lagged { missed_batches }) => {
                    listener_counts
                        .lost_batches
                        .fetch_add(missed_batches, Ordering::Relaxed);
                }
                Err(
                    crate::runtime::CollisionEventStreamError::NotRunning
                    | crate::runtime::CollisionEventStreamError::ChannelClosed,
                ) => break,
            }
        }
    });

    simulation.start()?;
    wait_for_steps(&simulation, 32).await?;
    simulation.set_time_multiplier(requested_warp)?;
    let initial_metrics = simulation.metrics();
    let mut cpu_samples = Vec::with_capacity(2_048);
    let mut sample_steps = 0u64;
    let mut sample_simulated_seconds = 0.0;
    let mut sample_wall_seconds = 0.0;
    let mut max_debt = Duration::ZERO;
    let mut latest_actual_warp = 0.0;
    let mut window = tokio::time::interval(Duration::from_millis(1));
    window.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    crate::runtime::asynchronous::marshalling::allocation_tracking::start();
    for _ in 0..RUNTIME_REPEATS {
        let before = simulation.metrics();
        let before_time = client.snapshot().simulation_time();
        let before_wall = Instant::now();
        let deadline = before_wall + RUNTIME_SAMPLE;
        let mut observed_step = before.total_steps;
        while Instant::now() < deadline {
            window.tick().await;
            let metrics = simulation.metrics();
            if metrics.total_steps != observed_step {
                cpu_samples.push(metrics.last_step_duration);
                observed_step = metrics.total_steps;
            }
            max_debt = max_debt.max(metrics.simulation_time_debt);
        }
        let elapsed = before_wall.elapsed();
        let after = simulation.metrics();
        let after_time = client.snapshot().simulation_time();
        sample_steps += after.total_steps.saturating_sub(before.total_steps);
        sample_simulated_seconds += after_time.saturating_sub(before_time).as_secs_f64();
        sample_wall_seconds += elapsed.as_secs_f64();
        max_debt = max_debt.max(after.simulation_time_debt);
        latest_actual_warp = after.actual_time_multiplier;
    }

    let source_velocity = client
        .snapshot()
        .body(bodies[0])
        .ok_or_else(|| io::Error::other("runtime snapshot lost a benchmark body"))?
        .linear_velocity();
    if !source_velocity.is_finite() || (source_count > 0 && source_velocity.y <= 0.0) {
        return Err(
            io::Error::other("persistent force source did not move its dynamic body").into(),
        );
    }

    let (allocation_calls, requested_bytes, peak_extra_live_bytes, baseline_live_bytes) =
        crate::runtime::asynchronous::marshalling::allocation_tracking::stop();
    let commands = command_burst(&client, &mut results, bodies[0]).await?;
    let end_metrics = simulation.metrics();
    let deadlines_missed = end_metrics
        .deadline_misses
        .saturating_sub(initial_metrics.deadline_misses);
    simulation.set_time_multiplier(0.0)?;
    let settled = wait_until_quiet(&simulation, Duration::from_millis(100)).await;
    simulation.stop().await?;
    collision_listener.await?;

    Ok(RuntimeReport {
        label: label.to_owned(),
        body_count,
        dynamic_count,
        collider_count,
        source_count,
        requested_warp,
        steps: sample_steps,
        simulated_seconds: sample_simulated_seconds,
        wall_seconds: sample_wall_seconds,
        actual_warp: latest_actual_warp,
        cpu_samples,
        allocation_calls,
        requested_bytes,
        peak_extra_live_bytes,
        baseline_live_bytes,
        deadlines_missed,
        peak_time_debt: max_debt,
        collisions: event_counts.collisions.load(Ordering::Relaxed),
        lost_event_batches: event_counts.lost_batches.load(Ordering::Relaxed),
        commands,
        settled,
        allocation_steps: sample_steps,
        active_workers: end_metrics.active_workers,
        rayon_pool_size: end_metrics.rayon_pool_size,
    })
}

async fn wait_for_steps(
    simulation: &Simulation,
    target_steps: u64,
) -> Result<(), Box<dyn std::error::Error>> {
    tokio::time::timeout(Duration::from_secs(10), async {
        while simulation.metrics().total_steps < target_steps {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    })
    .await
    .map_err(|_| io::Error::other("runtime warm-up did not complete"))?;
    Ok(())
}

async fn wait_until_quiet(simulation: &Simulation, limit: Duration) -> bool {
    let started = Instant::now();
    let mut previous_step = simulation.metrics().total_steps;
    let mut stable = 0;
    while started.elapsed() < limit {
        tokio::time::sleep(Duration::from_millis(2)).await;
        let metrics = simulation.metrics();
        if metrics.total_steps == previous_step && metrics.simulation_time_debt.is_zero() {
            stable += 1;
            if stable == 3 {
                return true;
            }
        } else {
            previous_step = metrics.total_steps;
            stable = 0;
        }
    }
    false
}

async fn command_burst(
    client: &PhysicsClient,
    results: &mut PhysicsBatchResults,
    body: BodyId,
) -> Result<CommandReport, Box<dyn std::error::Error>> {
    let mut writers = tokio::task::JoinSet::new();
    for _ in 0..COMMAND_WRITERS {
        let client = client.clone();
        writers.spawn(async move {
            let mut accepted = Vec::with_capacity(COMMAND_BATCHES_PER_WRITER);
            let mut queue_full = 0;
            for _ in 0..COMMAND_BATCHES_PER_WRITER {
                let mut buffer = client.commands();
                for _ in 0..PhysicsCommandBuffer::MAX_COMMANDS {
                    buffer.set_linear_velocity(body, Vec3::ZERO);
                }
                loop {
                    match client.try_submit(buffer) {
                        Ok(id) => {
                            accepted.push((id, Instant::now()));
                            break;
                        }
                        Err(PhysicsSubmitError::QueueFull(returned)) => {
                            queue_full += 1;
                            buffer = returned;
                            tokio::task::yield_now().await;
                        }
                        Err(error) => return Err(format!("batch submit failed: {error:?}")),
                    }
                }
            }
            Ok::<_, String>((accepted, queue_full))
        });
    }

    let mut pending = HashMap::with_capacity(COMMAND_WRITERS * COMMAND_BATCHES_PER_WRITER);
    let mut queue_full = 0;
    while let Some(joined) = writers.join_next().await {
        let (accepted, retries) = joined
            .map_err(|error| io::Error::other(error.to_string()))?
            .map_err(io::Error::other)?;
        queue_full += retries;
        for (id, submitted_at) in accepted {
            pending.insert(id.get(), submitted_at);
        }
    }

    let expected_batches = COMMAND_WRITERS * COMMAND_BATCHES_PER_WRITER;
    let mut ack_latencies = Vec::with_capacity(expected_batches);
    let mut failures = 0;
    while ack_latencies.len() + failures < expected_batches {
        let result = results.next_result().await?;
        if let Some(submitted_at) = pending.remove(&result.batch_id.get()) {
            if result.result.is_ok() {
                ack_latencies.push(submitted_at.elapsed());
            } else {
                failures += 1;
            }
        }
    }
    Ok(CommandReport {
        batches: expected_batches,
        commands: expected_batches * PhysicsCommandBuffer::MAX_COMMANDS,
        queue_full,
        failures,
        ack_latencies,
    })
}

#[test]
#[ignore = "Release Tokio/Rayon runtime stress benchmark; run with bench-internals"]
fn benchmark_rocket_flight_runtime_scaling() -> Result<(), Box<dyn std::error::Error>> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(4)
        .enable_all()
        .build()?;
    runtime.block_on(async {
        println!(
            "Runtime benchmark: Tokio multi-thread (4 workers), Rayon max_workers=4, 3 x 100 ms windows after 32-step warm-up; target={} arch={} debug_assertions={}",
            std::thread::available_parallelism().map_or(1, usize::from),
            std::env::consts::ARCH,
            cfg!(debug_assertions),
        );
        for bodies in [1, 100, 1_000, 10_000] {
            let report = runtime_fixture(
                "body scaling",
                bodies,
                bodies,
                bodies.min(100),
                0,
                1.0,
            )
            .await?;
            print_runtime_report(&report);
        }
        for warp in [0.1, 1.0, 10.0, 100.0, 1_000.0] {
            let report = runtime_fixture("time multiplier", 100, 100, 100, 0, warp).await?;
            print_runtime_report(&report);
        }
        for sources in [0, 1, 10] {
            let report = runtime_fixture("persistent sources", 1_000, 1, 100, sources, 1.0).await?;
            print_runtime_report(&report);
        }
        Ok::<(), Box<dyn std::error::Error>>(())
    })?;
    Ok(())
}

struct CountingWake {
    downstream: Waker,
    count: Arc<AtomicUsize>,
}

impl Wake for CountingWake {
    fn wake(self: Arc<Self>) {
        self.count.fetch_add(1, Ordering::Relaxed);
        self.downstream.wake_by_ref();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.count.fetch_add(1, Ordering::Relaxed);
        self.downstream.wake_by_ref();
    }
}

struct CountWakes<F> {
    future: Pin<Box<F>>,
    count: Arc<AtomicUsize>,
}

impl<F> CountWakes<F> {
    fn new(future: F, count: Arc<AtomicUsize>) -> Self {
        Self {
            future: Box::pin(future),
            count,
        }
    }
}

impl<F: Future> Future for CountWakes<F> {
    type Output = F::Output;

    fn poll(mut self: Pin<&mut Self>, context: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.as_mut().get_mut();
        let waker = Waker::from(Arc::new(CountingWake {
            downstream: context.waker().clone(),
            count: Arc::clone(&this.count),
        }));
        this.future.as_mut().poll(&mut Context::from_waker(&waker))
    }
}

fn scheduler_benchmark_world(
    body_count: usize,
) -> Result<PhysicsWorld, Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    for index in 0..body_count {
        let body = world.add_body(benchmark_body(index, true)?)?;
        world.add_collider(
            body,
            Collider::new(Shape::Sphere(Sphere::new(0.35)?), Transform::identity()),
        )?;
    }
    Ok(world)
}

fn duration_stats_us(samples: &[Duration]) -> (f64, f64, f64) {
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    let Some(min) = sorted.first() else {
        return (0.0, 0.0, 0.0);
    };
    let median = sorted[sorted.len() / 2];
    let max = sorted.last().copied().unwrap_or(*min);
    (
        min.as_secs_f64() * 1.0e6,
        median.as_secs_f64() * 1.0e6,
        max.as_secs_f64() * 1.0e6,
    )
}

#[test]
#[ignore = "Release Time Warp scheduler phase benchmark; run with bench-internals"]
fn benchmark_time_warp_scheduler_overhead() -> Result<(), Box<dyn std::error::Error>> {
    const BODY_COUNT: usize = 100;
    const WARMUP_STEPS: usize = 32;
    const SAMPLES: usize = 5;
    const STEPS_PER_SAMPLE: usize = 128;
    const SELECT_ITERATIONS: usize = 8;
    const TIMER_WAITS_PER_SAMPLE: usize = 5;

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(4)
        .enable_all()
        .build()?;
    runtime.block_on(async {
        println!(
            "Time Warp phases: Tokio workers=4, Rayon workers=4, bodies={BODY_COUNT}, warmup={WARMUP_STEPS}, samples={SAMPLES} x {STEPS_PER_SAMPLE}"
        );

        let mut ready_select_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..STEPS_PER_SAMPLE * 8 {
                tokio::select! {
                    _ = std::future::ready(()) => {}
                }
            }
            ready_select_samples.push(started.elapsed().div_f64((STEPS_PER_SAMPLE * 8) as f64));
        }
        let (min, median, max) = duration_stats_us(&ready_select_samples);
        println!("tokio::select! ready branch: {min:.3}/{median:.3}/{max:.3} us/iteration");

        let mut direct_world = scheduler_benchmark_world(BODY_COUNT)?;
        for _ in 0..WARMUP_STEPS {
            black_box(direct_world.step(1.0 / 120.0)?);
        }
        let mut direct_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..STEPS_PER_SAMPLE {
                black_box(direct_world.step(1.0 / 120.0)?);
            }
            direct_samples.push(started.elapsed().div_f64(STEPS_PER_SAMPLE as f64));
        }
        let (min, median, max) = duration_stats_us(&direct_samples);
        println!("PhysicsWorld::step (100 bodies): {min:.3}/{median:.3}/{max:.3} us/step");

        let mut simulation = Simulation::new_from_config(benchmark_config())?;
        populate_runtime_world(&mut simulation, BODY_COUNT, BODY_COUNT, BODY_COUNT, 0)?;
        let runner = simulation
            .simulation
            .as_mut()
            .ok_or_else(|| io::Error::other("missing StepRunner"))?;
        runner
            .set_active_workers(4)
            .map_err(|error| io::Error::other(error.to_string()))?;
        for _ in 0..WARMUP_STEPS {
            black_box(runner.step().await?);
        }

        let executor = runner
            .dispatch
            .as_ref()
            .cloned()
            .ok_or_else(|| io::Error::other("missing Rayon executor"))?;
        let mut dispatch_samples = Vec::with_capacity(SAMPLES * STEPS_PER_SAMPLE);
        for _ in 0..SAMPLES {
            for _ in 0..STEPS_PER_SAMPLE {
                let (sender, receiver) = oneshot::channel();
                let started = Instant::now();
                executor.spawn(move || {
                    let _ = sender.send(());
                })?;
                receiver.await?;
                dispatch_samples.push(started.elapsed());
            }
        }
        let (min, median, max) = duration_stats_us(&dispatch_samples);
        println!("Rayon spawn + oneshot round trip: {min:.3}/{median:.3}/{max:.3} us/task");

        let mut runner_wall_samples = Vec::with_capacity(SAMPLES);
        let mut engine_samples = Vec::with_capacity(SAMPLES * STEPS_PER_SAMPLE);
        let mut task_wait_samples = Vec::with_capacity(SAMPLES * STEPS_PER_SAMPLE);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..STEPS_PER_SAMPLE {
                let stats = black_box(runner.step().await?);
                engine_samples.push(stats.step_duration);
                task_wait_samples.push(stats.task_wait_duration);
            }
            runner_wall_samples.push(started.elapsed().div_f64(STEPS_PER_SAMPLE as f64));
        }
        let (min, median, max) = duration_stats_us(&runner_wall_samples);
        println!("StepRunner::step total: {min:.3}/{median:.3}/{max:.3} us/step");
        println!(
            "StepRunner engine CPU p50/p95/p99={:.3}/{:.3}/{:.3} us; dispatch+wait p50/p95/p99={:.3}/{:.3}/{:.3} us",
            duration_quantile_us(&engine_samples, 0.50),
            duration_quantile_us(&engine_samples, 0.95),
            duration_quantile_us(&engine_samples, 0.99),
            duration_quantile_us(&task_wait_samples, 0.50),
            duration_quantile_us(&task_wait_samples, 0.95),
            duration_quantile_us(&task_wait_samples, 0.99),
        );

        let (stop_tx, mut stop_rx) = watch::channel(false);
        let (warp_tx, mut warp_rx) = watch::channel(1_000.0);
        let _ = stop_rx.borrow_and_update();
        let _ = warp_rx.borrow_and_update();
        let mut select_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..SELECT_ITERATIONS {
                tokio::select! {
                    changed = stop_rx.changed() => {
                        return Err(io::Error::other(format!("unexpected stop change: {changed:?}")).into());
                    }
                    changed = warp_rx.changed() => {
                        return Err(io::Error::other(format!("unexpected warp change: {changed:?}")).into());
                    }
                    _ = wait_for_scaled_step(Duration::ZERO, 1_000.0) => {}
                }
            }
            select_samples.push(started.elapsed().div_f64(SELECT_ITERATIONS as f64));
        }
        drop((stop_tx, warp_tx));
        let (min, median, max) = duration_stats_us(&select_samples);
        println!("select! + zero-delay scaled timer: {min:.3}/{median:.3}/{max:.3} us/iteration");

        let mut yield_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..STEPS_PER_SAMPLE * 8 {
                tokio::task::yield_now().await;
            }
            yield_samples.push(started.elapsed().div_f64((STEPS_PER_SAMPLE * 8) as f64));
        }
        let (min, median, max) = duration_stats_us(&yield_samples);
        println!("yield_now: {min:.3}/{median:.3}/{max:.3} us/yield");

        for warp in [1.0, 10.0, 100.0, 1_000.0] {
            let mut wait_samples = Vec::with_capacity(SAMPLES * 32);
            for _ in 0..SAMPLES {
                let started = Instant::now();
                for _ in 0..TIMER_WAITS_PER_SAMPLE {
                    wait_for_scaled_step(Duration::from_millis(1), warp).await;
                }
                wait_samples.push(started.elapsed().div_f64(TIMER_WAITS_PER_SAMPLE as f64));
            }
            let (min, median, max) = duration_stats_us(&wait_samples);
            let wake_count = Arc::new(AtomicUsize::new(0));
            for _ in 0..TIMER_WAITS_PER_SAMPLE {
                CountWakes::new(
                    wait_for_scaled_step(Duration::from_millis(1), warp),
                    Arc::clone(&wake_count),
                )
                .await;
            }
            println!(
                "scaled timer remaining=1ms multiplier={warp:.0}x: {min:.3}/{median:.3}/{max:.3} us/wait; observed_wakes/wait={:.3}",
                wake_count.load(Ordering::Relaxed) as f64 / TIMER_WAITS_PER_SAMPLE as f64,
            );
        }

        let runner = simulation
            .simulation
            .as_mut()
            .ok_or_else(|| io::Error::other("missing StepRunner"))?;
        let world = runner
            .world()
            .map_err(|error| io::Error::other(error.to_string()))?;
        let mut snapshot = PhysicsSnapshot::from_world(world, 0, Duration::ZERO);
        let mut snapshot_samples = Vec::with_capacity(SAMPLES);
        for sample in 0..SAMPLES {
            let started = Instant::now();
            for step in 0..STEPS_PER_SAMPLE {
                snapshot.rebuild(world, (sample * STEPS_PER_SAMPLE + step) as u64, Duration::ZERO);
                black_box(snapshot.len());
            }
            snapshot_samples.push(started.elapsed().div_f64(STEPS_PER_SAMPLE as f64));
        }
        let (min, median, max) = duration_stats_us(&snapshot_samples);
        println!("snapshot rebuild (100 bodies): {min:.3}/{median:.3}/{max:.3} us/step");

        let snapshot = Arc::new(PhysicsSnapshot::from_world(world, 0, Duration::ZERO));
        let (snapshot_tx, snapshot_rx) = watch::channel(Arc::clone(&snapshot));
        let mut publish_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..STEPS_PER_SAMPLE {
                black_box(snapshot_tx.send_replace(Arc::clone(&snapshot)));
            }
            publish_samples.push(started.elapsed().div_f64(STEPS_PER_SAMPLE as f64));
        }
        drop(snapshot_rx);
        let (min, median, max) = duration_stats_us(&publish_samples);
        println!("watch snapshot publication: {min:.3}/{median:.3}/{max:.3} us/step");

        let mut reusable_snapshot = None;
        let (snapshot_tx, snapshot_rx) = watch::channel(snapshot);
        let mut full_snapshot_samples = Vec::with_capacity(SAMPLES);
        for sample in 0..SAMPLES {
            let started = Instant::now();
            for step in 0..STEPS_PER_SAMPLE {
                let sequence = (sample * STEPS_PER_SAMPLE + step + 1) as u64;
                let (mut next, _) = PhysicsSnapshot::take_reusable(&mut reusable_snapshot);
                next.rebuild(world, sequence, Duration::ZERO);
                let previous = snapshot_tx.send_replace(Arc::new(next));
                PhysicsSnapshot::recycle(previous, &mut reusable_snapshot);
            }
            full_snapshot_samples.push(started.elapsed().div_f64(STEPS_PER_SAMPLE as f64));
        }
        drop(snapshot_rx);
        let (min, median, max) = duration_stats_us(&full_snapshot_samples);
        println!("snapshot rebuild + Arc/watch publication: {min:.3}/{median:.3}/{max:.3} us/step");

        let (collision_tx, mut collision_rx) = broadcast::channel(COLLISION_EVENT_BATCH_CAPACITY);
        let mut event_samples = Vec::with_capacity(SAMPLES);
        for sample in 0..SAMPLES {
            let started = Instant::now();
            for step in 0..STEPS_PER_SAMPLE {
                let sequence = (sample * STEPS_PER_SAMPLE + step + 1) as u64;
                let batch = CollisionEventBatch::new(sequence, Vec::new());
                let _ = black_box(collision_tx.send(batch));
            }
            event_samples.push(started.elapsed().div_f64(STEPS_PER_SAMPLE as f64));
            while collision_rx.try_recv().is_ok() {}
        }
        let (min, median, max) = duration_stats_us(&event_samples);
        println!("empty collision-event batch + broadcast: {min:.3}/{median:.3}/{max:.3} us/step");

        let (metrics_tx, metrics_rx) = watch::channel(runner.metrics().clone());
        let mut metrics_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..STEPS_PER_SAMPLE {
                publish_runtime_metrics(runner, &metrics_tx, 1_000.0, 0.0);
            }
            metrics_samples.push(started.elapsed().div_f64(STEPS_PER_SAMPLE as f64));
        }
        drop(metrics_rx);
        let (min, median, max) = duration_stats_us(&metrics_samples);
        println!("metrics clone + watch publication: {min:.3}/{median:.3}/{max:.3} us/step");

        let (command_tx, command_rx) = tokio::sync::mpsc::channel(1);
        let mut queue = CommandQueue::new(command_rx, Arc::new(CommandBufferPool::default()));
        let events = SimulationEvents {
            collisions: collision_tx,
            batch_results: broadcast::channel(PHYSICS_BATCH_RESULT_CAPACITY).0,
        };
        let mut command_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..STEPS_PER_SAMPLE {
                apply_queued_commands(runner, &mut queue)?;
                publish_command_results(&mut queue, &events);
            }
            command_samples.push(started.elapsed().div_f64(STEPS_PER_SAMPLE as f64));
        }
        drop(command_tx);
        let (min, median, max) = duration_stats_us(&command_samples);
        println!("empty command poll + result drain: {min:.3}/{median:.3}/{max:.3} us/step");

        drop(simulation);
        for warp in [0.1, 1.0, 10.0, 100.0, 1_000.0] {
            let report = runtime_fixture("time warp scheduler", 100, 100, 100, 0, warp).await?;
            print_runtime_report(&report);
        }
        Ok::<(), Box<dyn std::error::Error>>(())
    })?;
    Ok(())
}

fn print_runtime_report(report: &RuntimeReport) {
    println!(
        "{} bodies={} dynamic={} colliders={} sources={} workers={}/{} requested={:.1}x steps={} sim_s={:.4} wall_s={:.4} measured_actual={:.2}x recent_metric={:.1}x steps_s={:.1}",
        report.label,
        report.body_count,
        report.dynamic_count,
        report.collider_count,
        report.source_count,
        report.active_workers,
        report.rayon_pool_size.unwrap_or(0),
        report.requested_warp,
        report.steps,
        report.simulated_seconds,
        report.wall_seconds,
        report.simulated_seconds / report.wall_seconds,
        report.actual_warp,
        report.steps as f64 / report.wall_seconds,
    );
    println!(
        "step_cpu_us observed={} p50/p95/p99={:.3}/{:.3}/{:.3}; allocs_per_step={:.3} requested_bytes_per_step={:.1} setup_live_heap_bytes={} peak_extra_live_bytes={} peak_live_heap_bytes={} deadlines={} peak_debt_ms={:.3} collisions={} lost_event_batches={} settled={}",
        report.cpu_samples.len(),
        duration_quantile_us(&report.cpu_samples, 0.50),
        duration_quantile_us(&report.cpu_samples, 0.95),
        duration_quantile_us(&report.cpu_samples, 0.99),
        report.allocation_calls as f64 / report.allocation_steps.max(1) as f64,
        report.requested_bytes as f64 / report.allocation_steps.max(1) as f64,
        report.baseline_live_bytes,
        report.peak_extra_live_bytes,
        report.baseline_live_bytes + report.peak_extra_live_bytes,
        report.deadlines_missed,
        report.peak_time_debt.as_secs_f64() * 1.0e3,
        report.collisions,
        report.lost_event_batches,
        report.settled,
    );
    println!(
        "commands batches={} commands={} QueueFull={} failed={} ack_us p50/p95/p99={:.1}/{:.1}/{:.1}",
        report.commands.batches,
        report.commands.commands,
        report.commands.queue_full,
        report.commands.failures,
        duration_quantile_us(&report.commands.ack_latencies, 0.50),
        duration_quantile_us(&report.commands.ack_latencies, 0.95),
        duration_quantile_us(&report.commands.ack_latencies, 0.99),
    );
}

#[test]
#[ignore = "Release bounded-queue saturation benchmark; run with bench-internals"]
fn benchmark_rocket_flight_runtime_queue_pressure() -> Result<(), Box<dyn std::error::Error>> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(4)
        .enable_all()
        .build()?;
    runtime.block_on(async {
        let mut simulation = Simulation::new_from_config(benchmark_config())?;
        let body = {
            let runner = simulation
                .simulation
                .as_mut()
                .ok_or_else(|| io::Error::other("missing step runner"))?;
            runner
                .world_mut()
                .map_err(|error| io::Error::other(error.to_string()))?
                .add_body(benchmark_body(0, true)?)?
        };
        simulation
            .simulation
            .as_mut()
            .ok_or_else(|| io::Error::other("runtime did not retain its step runner"))?
            .set_active_workers(4)
            .map_err(|error| io::Error::other(error.to_string()))?;
        let client = simulation.client();
        let mut results = client.subscribe_batch_results();
        simulation.set_time_multiplier(0.0)?;
        simulation.start()?;
        tokio::task::yield_now().await;

        crate::runtime::asynchronous::marshalling::allocation_tracking::start();
        let mut writers = tokio::task::JoinSet::new();
        for _ in 0..COMMAND_WRITERS {
            let client = client.clone();
            writers.spawn(async move {
                let mut accepted = Vec::with_capacity(COMMAND_QUEUE_CAPACITY / COMMAND_WRITERS);
                let mut queue_full = 0usize;
                loop {
                    let mut buffer = client.commands();
                    for _ in 0..PhysicsCommandBuffer::MAX_COMMANDS {
                        buffer.set_linear_velocity(body, Vec3::ZERO);
                    }
                    match client.try_submit(buffer) {
                        Ok(id) => {
                            accepted.push((id, Instant::now()));
                            tokio::task::yield_now().await;
                        }
                        Err(PhysicsSubmitError::QueueFull(buffer)) => {
                            queue_full += 1;
                            return Ok::<_, String>((accepted, queue_full, buffer));
                        }
                        Err(error) => return Err(format!("queue submission failed: {error:?}")),
                    }
                }
            });
        }

        let mut accepted = HashMap::with_capacity(COMMAND_QUEUE_CAPACITY + 1);
        let mut queue_full = 0usize;
        let mut overflow = None;
        let mut overflow_len = 0;
        let mut overflow_capacity = 0;
        while let Some(joined) = writers.join_next().await {
            let (ids, rejections, buffer) = joined
                .map_err(|error| io::Error::other(error.to_string()))?
                .map_err(io::Error::other)?;
            queue_full += rejections;
            for (id, submitted_at) in ids {
                accepted.insert(id.get(), submitted_at);
            }
            if overflow.is_none() {
                overflow_len = buffer.len();
                overflow_capacity = buffer.commands.capacity();
                overflow = Some(buffer);
            }
        }
        if accepted.len() != COMMAND_QUEUE_CAPACITY || overflow_len != PhysicsCommandBuffer::MAX_COMMANDS {
            return Err(io::Error::other("bounded queue did not reach its expected full state").into());
        }

        simulation.set_time_multiplier(1.0)?;
        let mut overflow = overflow.ok_or_else(|| io::Error::other("no QueueFull buffer was returned"))?;
        let (overflow_id, overflow_submitted_at) = loop {
            match client.try_submit(overflow) {
                Ok(id) => break (id, Instant::now()),
                Err(PhysicsSubmitError::QueueFull(returned)) => {
                    queue_full += 1;
                    overflow = returned;
                    tokio::time::sleep(Duration::from_millis(2)).await;
                    if overflow.len() != PhysicsCommandBuffer::MAX_COMMANDS {
                        return Err(io::Error::other("QueueFull did not retain the command buffer").into());
                    }
                }
                Err(error) => return Err(io::Error::other(format!("overflow submit failed: {error:?}")).into()),
            }
        };
        accepted.insert(overflow_id.get(), overflow_submitted_at);

        let completion_started = Instant::now();
        let mut latencies = Vec::with_capacity(COMMAND_QUEUE_CAPACITY + 1);
        let mut failures = 0usize;
        for _ in 0..COMMAND_QUEUE_CAPACITY + 1 {
            let result = tokio::time::timeout(Duration::from_secs(30), results.next_result())
                .await
                .map_err(|_| io::Error::other("queued command results timed out"))??;
            match accepted.remove(&result.batch_id.get()) {
                Some(submitted_at) if result.result.is_ok() => {
                    latencies.push(submitted_at.elapsed());
                }
                Some(_) => failures += 1,
                None => return Err(io::Error::other("received an unknown batch result").into()),
            }
        }
        simulation.stop().await?;
        let (
            allocation_calls,
            requested_bytes,
            peak_extra_live_bytes,
            baseline_live_bytes,
        ) =
            crate::runtime::asynchronous::marshalling::allocation_tracking::stop();
        println!(
            "queue pressure accepted={} full_rejections={} overflow_buffer_len/cap={}/{} commands={} wall_ms={:.3} ack_ms p50/p95/p99={:.3}/{:.3}/{:.3} failures={} allocations={} requested_bytes={} setup_live_heap_bytes={} peak_extra_live_bytes={} peak_live_heap_bytes={}",
            COMMAND_QUEUE_CAPACITY + 1,
            queue_full,
            overflow_len,
            overflow_capacity,
            (COMMAND_QUEUE_CAPACITY + 1) * PhysicsCommandBuffer::MAX_COMMANDS,
            completion_started.elapsed().as_secs_f64() * 1.0e3,
            duration_quantile_us(&latencies, 0.50) / 1_000.0,
            duration_quantile_us(&latencies, 0.95) / 1_000.0,
            duration_quantile_us(&latencies, 0.99) / 1_000.0,
            failures,
            allocation_calls,
            requested_bytes,
            baseline_live_bytes,
            peak_extra_live_bytes,
            baseline_live_bytes + peak_extra_live_bytes,
        );
        Ok::<(), Box<dyn std::error::Error>>(())
    })?;
    Ok(())
}
