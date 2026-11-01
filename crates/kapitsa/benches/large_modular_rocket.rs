use std::{
    alloc::{GlobalAlloc, Layout, System},
    error::Error,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    time::{Duration, Instant},
};

use kapitsa_core::{
    collision::CollisionStepMetrics,
    world::{BodyId, PhysicsWorld},
};
use kapitsa_math::Vec3;

#[path = "../src/runtime/modular_rocket_fixture.rs"]
mod modular_rocket_fixture;
#[path = "../../kapitsa-core/benches/support/scoped_executor.rs"]
mod scoped_executor;

use modular_rocket_fixture::ModularRocket;
use scoped_executor::ScopedThreadExecutor;

const SIZES: [usize; 4] = [100, 200, 500, 1000];
const WORKERS: [usize; 4] = [1, 2, 4, 8];
const WARMUPS: usize = 5;
const SAMPLES: usize = 60;
const EVENT_SAMPLES: usize = 30;
const DT: f64 = 1.0 / 120.0;
const SEPARATION_IMPULSE: Vec3 = Vec3::new(0.0, 2.0, 0.0);

static TRACK_ALLOCATIONS: AtomicBool = AtomicBool::new(false);
static ALLOCATION_COUNT: AtomicUsize = AtomicUsize::new(0);
static ALLOCATION_BYTES: AtomicUsize = AtomicUsize::new(0);

struct CountingAllocator;

// SAFETY: allocation operations delegate to the system allocator.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        record_allocation(pointer, layout.size());
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        record_allocation(pointer, layout.size());
        pointer
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let pointer = unsafe { System.realloc(pointer, layout, size) };
        record_allocation(pointer, size);
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) };
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

#[derive(Clone, Copy)]
struct TimedSample {
    elapsed: Duration,
    allocations: usize,
    requested_bytes: usize,
    metrics: CollisionStepMetrics,
}

#[derive(Clone, Copy)]
struct EventSample {
    action: Duration,
    step: TimedSample,
}

fn record_allocation(pointer: *mut u8, requested_bytes: usize) {
    if !pointer.is_null() && TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
        ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
        ALLOCATION_BYTES.fetch_add(requested_bytes, Ordering::Relaxed);
    }
}

fn begin_allocation_sample() {
    ALLOCATION_COUNT.store(0, Ordering::Relaxed);
    ALLOCATION_BYTES.store(0, Ordering::Relaxed);
    TRACK_ALLOCATIONS.store(true, Ordering::Relaxed);
}

fn end_allocation_sample() -> (usize, usize) {
    TRACK_ALLOCATIONS.store(false, Ordering::Relaxed);
    (
        ALLOCATION_COUNT.load(Ordering::Relaxed),
        ALLOCATION_BYTES.load(Ordering::Relaxed),
    )
}

fn main() -> Result<(), Box<dyn Error>> {
    println!(
        "Large modular rocket Release benchmark: dt={DT:.9}s warmups={WARMUPS} steady_samples={SAMPLES} event_samples={EVENT_SAMPLES} workers={WORKERS:?}"
    );
    for parts in SIZES {
        for workers in WORKERS {
            report_steady(parts, workers)?;
            report_start(parts, workers)?;
            report_separation(parts, workers)?;
        }
    }
    Ok(())
}

fn report_steady(parts: usize, workers: usize) -> Result<(), Box<dyn Error>> {
    let mut rocket = ModularRocket::new(parts);
    validate_fixture(&rocket, parts)?;
    warm(&mut rocket.world, workers, WARMUPS)?;
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        samples.push(measure_step(&mut rocket.world, workers)?);
    }
    validate_world(&rocket.world)?;
    report_samples("steady", parts, workers, &samples);
    Ok(())
}

fn report_start(parts: usize, workers: usize) -> Result<(), Box<dyn Error>> {
    let mut samples = Vec::with_capacity(EVENT_SAMPLES);
    for _ in 0..EVENT_SAMPLES {
        let mut rocket = ModularRocket::new(parts);
        validate_fixture(&rocket, parts)?;
        warm(&mut rocket.world, workers, WARMUPS)?;
        let started = Instant::now();
        rocket
            .world
            .set_force_source_enabled(rocket.lower_engine, true)?;
        let action = started.elapsed();
        let step = measure_step(&mut rocket.world, workers)?;
        validate_world(&rocket.world)?;
        samples.push(EventSample { action, step });
    }
    report_events("start", parts, workers, &samples);
    Ok(())
}

fn report_separation(parts: usize, workers: usize) -> Result<(), Box<dyn Error>> {
    let mut samples = Vec::with_capacity(EVENT_SAMPLES);
    for _ in 0..EVENT_SAMPLES {
        let mut rocket = ModularRocket::new(parts);
        validate_fixture(&rocket, parts)?;
        warm(&mut rocket.world, workers, WARMUPS)?;
        let started = Instant::now();
        rocket
            .world
            .remove_joint(rocket.separation_joint)
            .ok_or_else(|| std::io::Error::other("separation joint is missing"))?;
        rocket
            .world
            .set_force_source_enabled(rocket.lower_engine, false)?;
        rocket
            .world
            .set_force_source_enabled(rocket.upper_engine, true)?;
        apply_stage_impulse(&mut rocket.world, &rocket.lower_parts, -SEPARATION_IMPULSE)?;
        apply_stage_impulse(&mut rocket.world, &rocket.upper_parts, SEPARATION_IMPULSE)?;
        let action = started.elapsed();
        let step = measure_step(&mut rocket.world, workers)?;
        if rocket.world.joint_count() != parts - 2 {
            return Err(
                format!("separation left an unexpected joint count for {parts} parts").into(),
            );
        }
        validate_world(&rocket.world)?;
        samples.push(EventSample { action, step });
    }
    report_events("separation", parts, workers, &samples);
    Ok(())
}

fn warm(world: &mut PhysicsWorld, workers: usize, count: usize) -> Result<(), Box<dyn Error>> {
    for _ in 0..count {
        world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;
    }
    Ok(())
}

fn measure_step(world: &mut PhysicsWorld, workers: usize) -> Result<TimedSample, Box<dyn Error>> {
    begin_allocation_sample();
    let started = Instant::now();
    let result = world.step_with_executor(DT, &ScopedThreadExecutor, workers);
    let elapsed = started.elapsed();
    let (allocations, requested_bytes) = end_allocation_sample();
    result?;
    Ok(TimedSample {
        elapsed,
        allocations,
        requested_bytes,
        metrics: world.collision_step_metrics(),
    })
}

fn apply_stage_impulse(
    world: &mut PhysicsWorld,
    stage: &[BodyId],
    impulse: Vec3,
) -> Result<(), Box<dyn Error>> {
    let mass = stage.iter().try_fold(0.0, |total, &id| {
        let part_mass = world
            .body(id)
            .and_then(|body| body.physics().mass())
            .map(|properties| properties.mass())
            .ok_or_else(|| "stage body has no mass".to_owned())?;
        Ok::<_, String>(total + part_mass)
    })?;
    let delta_velocity = impulse / mass;
    for &id in stage {
        let velocity = world
            .body(id)
            .ok_or("stage body is missing")?
            .state()
            .linear_velocity();
        world
            .body_mut(id)
            .ok_or("stage body is missing")?
            .set_linear_velocity(velocity + delta_velocity)?;
    }
    Ok(())
}

fn validate_world(world: &PhysicsWorld) -> Result<(), Box<dyn Error>> {
    if world.bodies().any(|(_, body)| {
        let state = body.state();
        !state.transform().translation().is_finite()
            || !state.transform().rotation().is_normalized()
            || !state.linear_velocity().is_finite()
            || !state.angular_velocity().is_finite()
    }) {
        return Err("benchmark produced a non-finite body state".into());
    }
    Ok(())
}

fn validate_fixture(rocket: &ModularRocket, expected_parts: usize) -> Result<(), Box<dyn Error>> {
    if rocket.parts.len() != expected_parts
        || rocket.lower_parts.len() + rocket.upper_parts.len() != expected_parts
        || !rocket.lower_parts.contains(&rocket.lower_root)
        || !rocket.upper_parts.contains(&rocket.upper_root)
        || rocket.lower_tanks.is_empty()
        || rocket.upper_tanks.is_empty()
        || rocket.world.collider(rocket.surface_collider).is_none()
    {
        return Err(format!("invalid modular rocket fixture for {expected_parts} parts").into());
    }
    Ok(())
}

fn report_samples(phase: &str, parts: usize, workers: usize, samples: &[TimedSample]) {
    let mut ordered = samples.to_vec();
    ordered.sort_unstable_by_key(|sample| sample.elapsed);
    let min = ordered[0];
    let p50 = quantile(&ordered, 0.50);
    let p95 = quantile(&ordered, 0.95);
    let p99 = quantile(&ordered, 0.99);
    let max = ordered[ordered.len() - 1];
    let metrics = p50.metrics;
    let mean_wall = samples
        .iter()
        .map(|sample| sample.elapsed.as_secs_f64())
        .sum::<f64>()
        / samples.len() as f64;
    println!(
        "phase={phase} parts={parts} joints={} workers={workers} step_ms_min/p50/p95/p99/max={:.4}/{:.4}/{:.4}/{:.4}/{:.4} sim_x={:.2} allocs_p50/p95={}/{} bytes_p50/p95={}/{} retained_kib={} peak_scratch_kib={} growth_bytes={} joint_prepare_us={:.2} solver_prepare_us={:.2} solver_iterations_us={:.2} collision_us={:.2} island_build_us={:.2} islands={} largest_island_bodies/joints/contacts={}/{}/{} candidates/contacts={}/{}",
        parts - 1,
        milliseconds(min.elapsed),
        milliseconds(p50.elapsed),
        milliseconds(p95.elapsed),
        milliseconds(p99.elapsed),
        milliseconds(max.elapsed),
        DT / mean_wall,
        allocation_quantile(samples, 0.50, |sample| sample.allocations),
        allocation_quantile(samples, 0.95, |sample| sample.allocations),
        allocation_quantile(samples, 0.50, |sample| sample.requested_bytes),
        allocation_quantile(samples, 0.95, |sample| sample.requested_bytes),
        metrics.retained_scratch_bytes / 1024,
        metrics.peak_scratch_bytes / 1024,
        metrics.logical_capacity_growth_bytes,
        micros(metrics.joint_constraint_prepare),
        micros(metrics.solver_prepare),
        micros(metrics.solver_iterations),
        micros(collision_time(metrics)),
        micros(metrics.island_build),
        metrics.island_count,
        metrics.largest_island_body_count,
        metrics.largest_island_joint_count,
        metrics.largest_island_contact_count,
        metrics.candidate_count,
        metrics.contact_count,
    );
}

fn report_events(phase: &str, parts: usize, workers: usize, samples: &[EventSample]) {
    let mut steps = samples.iter().map(|sample| sample.step).collect::<Vec<_>>();
    steps.sort_unstable_by_key(|sample| sample.elapsed);
    let actions = samples
        .iter()
        .map(|sample| sample.action)
        .collect::<Vec<_>>();
    println!(
        "phase={phase} parts={parts} joints={} workers={workers} action_us_min/p50/p95/p99/max={:.2}/{:.2}/{:.2}/{:.2}/{:.2}",
        parts - 1,
        actions
            .iter()
            .copied()
            .min()
            .unwrap_or_default()
            .as_secs_f64()
            * 1.0e6,
        duration_quantile(&actions, 0.50),
        duration_quantile(&actions, 0.95),
        duration_quantile(&actions, 0.99),
        actions
            .iter()
            .copied()
            .max()
            .unwrap_or_default()
            .as_secs_f64()
            * 1.0e6,
    );
    report_samples(phase, parts, workers, &steps);
}

fn collision_time(metrics: CollisionStepMetrics) -> Duration {
    metrics.spatial_index + metrics.broad_phase + metrics.ccd + metrics.narrow_phase
}

fn quantile(samples: &[TimedSample], percentile: f64) -> TimedSample {
    let index = ((samples.len() - 1) as f64 * percentile).round() as usize;
    samples[index]
}

fn allocation_quantile(
    samples: &[TimedSample],
    percentile: f64,
    value: impl Fn(&TimedSample) -> usize,
) -> usize {
    let mut values = samples.iter().map(value).collect::<Vec<_>>();
    values.sort_unstable();
    let index = ((values.len() - 1) as f64 * percentile).round() as usize;
    values[index]
}

fn duration_quantile(values: &[Duration], percentile: f64) -> f64 {
    let mut values = values.to_vec();
    values.sort_unstable();
    let index = ((values.len() - 1) as f64 * percentile).round() as usize;
    values[index].as_secs_f64() * 1.0e6
}

fn milliseconds(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1.0e3
}

fn micros(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1.0e6
}
