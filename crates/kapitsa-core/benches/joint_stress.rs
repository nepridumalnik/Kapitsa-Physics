#[cfg(windows)]
use std::ffi::c_void;
use std::{
    alloc::{GlobalAlloc, Layout, System},
    error::Error,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    collision::CollisionStepMetrics,
    gravity::CentralGravity,
    joint::{FixedJoint, Joint, JointCommon, JointFrame},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

#[path = "support/scoped_executor.rs"]
mod scoped_executor;
use scoped_executor::ScopedThreadExecutor;

const SIZES: [usize; 3] = [144, 1_440, 14_400];
const WARMUPS: usize = 3;
const SAMPLES: usize = 10;
const DT: f64 = 1.0 / 120.0;
const WORKERS: [usize; 4] = [1, 2, 4, 8];

static TRACK_ALLOCATIONS: AtomicBool = AtomicBool::new(false);
static ALLOCATION_COUNT: AtomicUsize = AtomicUsize::new(0);
static ALLOCATION_BYTES: AtomicUsize = AtomicUsize::new(0);

struct CountingAllocator;

// SAFETY: all allocation operations delegate to the system allocator.
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

#[derive(Clone, Copy)]
struct StepSample {
    timing: TimedSample,
    metrics: CollisionStepMetrics,
}

#[derive(Clone, Copy)]
struct TimedSample {
    elapsed: Duration,
    cpu: Option<Duration>,
    allocations: usize,
    requested_bytes: usize,
}

#[cfg(windows)]
#[repr(C)]
#[derive(Default)]
struct FileTime {
    low: u32,
    high: u32,
}

#[cfg(windows)]
#[link(name = "kernel32")]
unsafe extern "system" {
    fn GetCurrentProcess() -> *mut c_void;
    fn GetProcessTimes(
        process: *mut c_void,
        creation: *mut FileTime,
        exit: *mut FileTime,
        kernel: *mut FileTime,
        user: *mut FileTime,
    ) -> i32;
}

#[cfg(windows)]
fn process_cpu_time() -> Option<Duration> {
    let mut creation = FileTime::default();
    let mut exit = FileTime::default();
    let mut kernel = FileTime::default();
    let mut user = FileTime::default();
    // SAFETY: GetCurrentProcess returns a pseudo-handle and all output pointers
    // point to initialized, writable FILETIME values.
    let succeeded = unsafe {
        GetProcessTimes(
            GetCurrentProcess(),
            &mut creation,
            &mut exit,
            &mut kernel,
            &mut user,
        )
    };
    if succeeded == 0 {
        return None;
    }
    let ticks = |time: FileTime| (u64::from(time.high) << 32) | u64::from(time.low);
    Some(Duration::from_nanos(
        ticks(kernel)
            .saturating_add(ticks(user))
            .saturating_mul(100),
    ))
}

#[cfg(not(windows))]
fn process_cpu_time() -> Option<Duration> {
    None
}

fn measure<T>(operation: impl FnOnce() -> T) -> (T, TimedSample) {
    let cpu_started = process_cpu_time();
    let started = Instant::now();
    begin_allocation_sample();
    let result = operation();
    let elapsed = started.elapsed();
    let (allocations, requested_bytes) = end_allocation_sample();
    let cpu = cpu_started
        .zip(process_cpu_time())
        .and_then(|(started, ended)| ended.checked_sub(started));
    (
        result,
        TimedSample {
            elapsed,
            cpu,
            allocations,
            requested_bytes,
        },
    )
}

fn percentile_index(count: usize, percentile: usize) -> usize {
    count
        .saturating_mul(percentile)
        .div_ceil(100)
        .saturating_sub(1)
        .min(count.saturating_sub(1))
}

fn cpu_summary(samples: impl Iterator<Item = Option<Duration>>, count: usize) -> String {
    let mut times = samples.collect::<Option<Vec<_>>>();
    let Some(times) = times.as_mut() else {
        return "n/a".to_owned();
    };
    if times.len() != count {
        return "n/a".to_owned();
    }
    times.sort_unstable();
    let p95 = percentile_index(times.len(), 95);
    format!(
        "{:.3}/{:.3}/{:.3}/{:.3}",
        milliseconds(times[0]),
        milliseconds(times[count / 2]),
        milliseconds(times[p95]),
        milliseconds(times[count - 1]),
    )
}

fn timing_summary(samples: &[TimedSample]) -> String {
    let mut sorted = samples.to_vec();
    sorted.sort_unstable_by_key(|sample| sample.elapsed);
    let p95 = percentile_index(sorted.len(), 95);
    format!(
        "wall_ms={}/{}/{}/{} cpu_ms={}",
        milliseconds(sorted[0].elapsed),
        milliseconds(sorted[sorted.len() / 2].elapsed),
        milliseconds(sorted[p95].elapsed),
        milliseconds(sorted[sorted.len() - 1].elapsed),
        cpu_summary(sorted.iter().map(|sample| sample.cpu), sorted.len()),
    )
}

fn allocation_percentile<T>(
    samples: &[T],
    value: impl Fn(&T) -> usize,
    percentile: usize,
) -> usize {
    let mut values = samples.iter().map(value).collect::<Vec<_>>();
    values.sort_unstable();
    values[percentile_index(values.len(), percentile)]
}

fn main() -> Result<(), Box<dyn Error>> {
    println!(
        "Joint stress scenes: warmups={WARMUPS}, samples={SAMPLES}, sizes={SIZES:?}, workers={WORKERS:?}"
    );
    report("fleet of 100 two-stage rockets", 100, rocket_fleet)?;
    for size in SIZES {
        report("fixed-joint chain", size, fixed_chain)?;
        report("branching fixed-joint assembly", size, branching_assembly)?;
        report("independent two-body assemblies", size, independent_pairs)?;
        report("dense vertical contact stack", size, dense_contact_stack)?;
        report_first_step("mass joint break", size, mass_break)?;
        report_mass_separation(size)?;
    }
    Ok(())
}

fn report(
    name: &str,
    size: usize,
    build: fn(usize) -> Result<PhysicsWorld, Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    for workers in WORKERS {
        let mut world = build(size)?;
        let initial_joint_errors = max_joint_frame_errors(&world)?;
        for _ in 0..WARMUPS {
            world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;
        }
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let (result, timing) =
                measure(|| world.step_with_executor(DT, &ScopedThreadExecutor, workers));
            result?;
            samples.push(StepSample {
                timing,
                metrics: world.collision_step_metrics(),
            });
        }
        let joint_errors = max_joint_frame_errors(&world)?;
        validate_bodies(&world)?;
        if expected_joints(name, size) > 0 && (joint_errors.0 > 0.002 || joint_errors.1 > 0.0002) {
            return Err(format!(
                "{name} size={size} workers={workers} failed joint tolerances: {joint_errors:?}"
            )
            .into());
        }
        if name == "dense vertical contact stack"
            && samples
                .iter()
                .any(|sample| sample.metrics.contact_count != size)
        {
            return Err(format!("{name} size={size} did not retain all {size} contacts").into());
        }
        samples.sort_unstable_by_key(|sample| sample.timing.elapsed);
        let min = &samples[0];
        let median = &samples[SAMPLES / 2];
        let p95 = &samples[percentile_index(SAMPLES, 95)];
        let max = &samples[SAMPLES - 1];
        println!(
            "{name:34} size={size:>6} workers={workers}: wall_ms={}/{}/{}/{} cpu_ms={} allocs_med/p95={}/{} requested_bytes_med/p95={}/{} solver_prepare_ms={:.3} solver_iterations_ms={:.3} narrow_ms={:.3} candidates={} contacts={} islands={} joints={} initial_joint_error_m/rad={:.6e}/{:.6e} final_joint_error_m/rad={:.6e}/{:.6e} retained_scratch_kib={} peak_scratch_kib={} logical_growth_bytes={}",
            milliseconds(min.timing.elapsed),
            milliseconds(median.timing.elapsed),
            milliseconds(p95.timing.elapsed),
            milliseconds(max.timing.elapsed),
            cpu_summary(
                samples.iter().map(|sample| sample.timing.cpu),
                samples.len()
            ),
            allocation_percentile(&samples, |sample| sample.timing.allocations, 50),
            allocation_percentile(&samples, |sample| sample.timing.allocations, 95),
            allocation_percentile(&samples, |sample| sample.timing.requested_bytes, 50),
            allocation_percentile(&samples, |sample| sample.timing.requested_bytes, 95),
            milliseconds(median.metrics.solver_prepare),
            milliseconds(median.metrics.solver_iterations),
            milliseconds(median.metrics.narrow_phase),
            median.metrics.candidate_count,
            median.metrics.contact_count,
            median.metrics.island_count,
            expected_joints(name, size),
            initial_joint_errors.0,
            initial_joint_errors.1,
            joint_errors.0,
            joint_errors.1,
            median.metrics.retained_scratch_bytes / 1024,
            median.metrics.peak_scratch_bytes / 1024,
            median.metrics.logical_capacity_growth_bytes,
        );
    }
    Ok(())
}

fn expected_joints(name: &str, size: usize) -> usize {
    match name {
        "fixed-joint chain" | "branching fixed-joint assembly" => size.saturating_sub(1),
        "independent two-body assemblies" => size / 2,
        "fleet of 100 two-stage rockets" => size,
        _ => 0,
    }
}

fn validate_bodies(world: &PhysicsWorld) -> Result<(), Box<dyn Error>> {
    if world.bodies().any(|(_, body)| {
        let state = body.state();
        !state.transform().translation().is_finite()
            || !state.transform().rotation().is_normalized()
            || !state.linear_velocity().is_finite()
            || !state.angular_velocity().is_finite()
    }) {
        return Err("benchmark produced an invalid body state".into());
    }
    Ok(())
}

fn report_first_step(
    name: &str,
    size: usize,
    build: fn(usize) -> Result<PhysicsWorld, Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    for workers in WORKERS {
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let mut world = build(size)?;
            // Breaking destroys the measured joints, so this is intentionally the first step.
            let (result, timing) =
                measure(|| world.step_with_executor(DT, &ScopedThreadExecutor, workers));
            result?;
            validate_bodies(&world)?;
            if world.joint_events().len() != size / 2 {
                return Err(format!(
                    "{name} size={size} broke {} of {} joints",
                    world.joint_events().len(),
                    size / 2
                )
                .into());
            }
            samples.push((
                timing,
                world.collision_step_metrics(),
                world.joint_events().len(),
            ));
        }
        samples.sort_unstable_by_key(|sample| sample.0.elapsed);
        let min = &samples[0];
        let median = &samples[SAMPLES / 2];
        let p95 = &samples[percentile_index(SAMPLES, 95)];
        let max = &samples[SAMPLES - 1];
        println!(
            "{name:34} elements={size:>6} workers={workers}: wall_ms={}/{}/{}/{} cpu_ms={} allocs_med/p95={}/{} requested_bytes_med/p95={}/{} islands={} break_events={} retained_scratch_kib={} peak_scratch_kib={} logical_growth_bytes={}",
            milliseconds(min.0.elapsed),
            milliseconds(median.0.elapsed),
            milliseconds(p95.0.elapsed),
            milliseconds(max.0.elapsed),
            cpu_summary(samples.iter().map(|sample| sample.0.cpu), samples.len()),
            allocation_percentile(&samples, |sample| sample.0.allocations, 50),
            allocation_percentile(&samples, |sample| sample.0.allocations, 95),
            allocation_percentile(&samples, |sample| sample.0.requested_bytes, 50),
            allocation_percentile(&samples, |sample| sample.0.requested_bytes, 95),
            median.1.island_count,
            median.2,
            median.1.retained_scratch_bytes / 1024,
            median.1.peak_scratch_bytes / 1024,
            median.1.logical_capacity_growth_bytes,
        );
    }
    Ok(())
}

fn report_mass_separation(count: usize) -> Result<(), Box<dyn Error>> {
    for workers in WORKERS {
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let (mut world, joints) = independent_pair_world(count)?;
            world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;

            let (_, remove_timing) = measure(|| {
                for joint in joints {
                    world.remove_joint(joint);
                }
            });
            let (result, detached_timing) =
                measure(|| world.step_with_executor(DT, &ScopedThreadExecutor, workers));
            result?;
            validate_bodies(&world)?;
            if world.joint_count() != 0 {
                return Err(format!("mass separation left joints attached at size={count}").into());
            }
            samples.push((
                remove_timing,
                detached_timing,
                world.collision_step_metrics(),
            ));
        }
        let mut remove_samples: Vec<_> = samples.iter().map(|sample| sample.0).collect();
        let mut detached_samples: Vec<_> = samples.iter().map(|sample| sample.1).collect();
        remove_samples.sort_unstable_by_key(|sample| sample.elapsed);
        detached_samples.sort_unstable_by_key(|sample| sample.elapsed);
        samples.sort_unstable_by_key(|sample| sample.0.elapsed + sample.1.elapsed);
        let median = &samples[SAMPLES / 2];
        println!(
            "mass stage separation               elements={count:>6} workers={workers}: remove={} detached_step={} remove_allocs/bytes_med/p95={}/{}/{} detached_allocs/bytes_med/p95={}/{}/{} retained_scratch_kib={} peak_scratch_kib={} logical_growth_bytes={}",
            timing_summary(&remove_samples),
            timing_summary(&detached_samples),
            allocation_percentile(&remove_samples, |sample| sample.allocations, 50),
            allocation_percentile(&remove_samples, |sample| sample.requested_bytes, 50),
            allocation_percentile(&remove_samples, |sample| sample.requested_bytes, 95),
            allocation_percentile(&detached_samples, |sample| sample.allocations, 50),
            allocation_percentile(&detached_samples, |sample| sample.requested_bytes, 50),
            allocation_percentile(&detached_samples, |sample| sample.requested_bytes, 95),
            median.2.retained_scratch_bytes / 1024,
            median.2.peak_scratch_bytes / 1024,
            median.2.logical_capacity_growth_bytes,
        );
    }
    Ok(())
}

fn fixed_chain(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mut bodies = Vec::with_capacity(count);
    for index in 0..count {
        bodies.push(world.add_body(dynamic_body(Vec3::X * index as f64)?)?);
    }
    let common = fixed_joint_frames();
    for pair in bodies.windows(2) {
        world.add_joint(pair[0], pair[1], Joint::Fixed(FixedJoint::new(common)))?;
    }
    Ok(world)
}

fn branching_assembly(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let hub = world.add_body(dynamic_body(Vec3::ZERO)?)?;
    let leaf_joint = Joint::Fixed(FixedJoint::new(JointCommon::new(
        JointFrame::new(Vec3::ZERO, Quat::IDENTITY),
        JointFrame::new(Vec3::ZERO, Quat::IDENTITY),
    )));
    for _ in 1..count {
        let leaf_rotation = Quat::from_axis_angle(Vec3::Z, 0.001).ok_or("valid leaf rotation")?;
        let leaf_transform = Transform::new(Vec3::X * 0.01, leaf_rotation)?;
        let leaf = world.add_body(dynamic_body_at(leaf_transform)?)?;
        world.add_joint(hub, leaf, leaf_joint)?;
    }
    Ok(world)
}

fn independent_pairs(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    independent_pair_world(count).map(|(world, _)| world)
}

fn independent_pair_world(
    count: usize,
) -> Result<(PhysicsWorld, Vec<kapitsa_core::world::JointId>), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let common = fixed_joint_frames();
    let mut joints = Vec::with_capacity(count / 2);
    for pair in 0..count / 2 {
        let x = pair as f64 * 3.0;
        let body_a = world.add_body(dynamic_body(Vec3::X * x)?)?;
        let rotation = Quat::from_axis_angle(Vec3::Z, 0.001).ok_or("valid pair rotation")?;
        let transform = Transform::new(Vec3::X * (x + 1.02), rotation)?;
        let body_b = world.add_body(dynamic_body_at(transform)?)?;
        joints.push(world.add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))?);
    }
    Ok((world, joints))
}

fn rocket_fleet(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let common = fixed_joint_frames();
    for rocket in 0..count {
        let x = rocket as f64 * 4.0;
        let body_a = world.add_body(dynamic_body(Vec3::X * x)?)?;
        let body_b = world.add_body(dynamic_body(Vec3::X * (x + 1.0))?)?;
        world.add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))?;
    }
    Ok(world)
}

fn dense_contact_stack(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    const PLANET_RADIUS: f64 = 100.0;

    let mut world = PhysicsWorld::new();
    world.set_central_gravity(Some(CentralGravity::new(
        Vec3::ZERO,
        9.806_65 * PLANET_RADIUS * PLANET_RADIUS,
    )?));
    let ground = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        ground,
        Collider::new(
            Shape::Sphere(Sphere::new(PLANET_RADIUS)?),
            Transform::identity(),
        ),
    )?;

    for index in 0..count {
        let altitude = PLANET_RADIUS + 0.45 + index as f64 * 0.9;
        let body = world.add_body(dynamic_body(Vec3::Y * altitude)?)?;
        world.add_collider(
            body,
            Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
        )?;
    }
    Ok(world)
}

fn mass_break(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mut common = fixed_joint_frames();
    common.break_force = Some(100.0);
    for pair in 0..count / 2 {
        let x = pair as f64 * 3.0;
        let body_a = world.add_body(dynamic_body(Vec3::X * x)?)?;
        let mut body_b = dynamic_body(Vec3::X * x)?;
        body_b.set_linear_velocity(Vec3::X * 10.0)?;
        let body_b = world.add_body(body_b)?;
        world.add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))?;
    }
    Ok(world)
}

fn dynamic_body(position: Vec3) -> Result<RigidBody, Box<dyn Error>> {
    dynamic_body_at(Transform::new(position, Quat::IDENTITY)?)
}

fn dynamic_body_at(transform: Transform) -> Result<RigidBody, Box<dyn Error>> {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    Ok(RigidBody::new_dynamic(transform, mass)?)
}

fn max_joint_frame_errors(world: &PhysicsWorld) -> Result<(f64, f64), Box<dyn Error>> {
    let mut max_anchor_error: f64 = 0.0;
    let mut max_frame_error: f64 = 0.0;
    for (_, body_a_id, body_b_id, joint) in world.joints() {
        let body_a = world.body(body_a_id).ok_or("joint body A is missing")?;
        let body_b = world.body(body_b_id).ok_or("joint body B is missing")?;
        let transform_a = body_a.state().transform();
        let transform_b = body_b.state().transform();
        let common = joint.common();
        let anchor_a = transform_a.transform_point(common.frame_a.local_anchor);
        let anchor_b = transform_b.transform_point(common.frame_b.local_anchor);
        max_anchor_error = max_anchor_error.max((anchor_b - anchor_a).length());

        let frame_a = transform_a.rotation() * common.frame_a.local_rotation;
        let frame_b = transform_b.rotation() * common.frame_b.local_rotation;
        let relative = frame_a.conjugate() * frame_b;
        let vector_length = Vec3::new(relative.x, relative.y, relative.z).length();
        max_frame_error = max_frame_error.max(2.0 * vector_length.atan2(relative.w.abs()));
    }
    Ok((max_anchor_error, max_frame_error))
}

fn fixed_joint_frames() -> JointCommon {
    JointCommon::new(
        JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
        JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
    )
}

fn milliseconds(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1.0e3
}
