use std::{error::Error, time::Instant};

use kapitsa_core::{
    collider::Collider,
    execution::SequentialExecutor,
    joint::{BallJoint, HingeJoint, Joint, JointCommon, JointFrame},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

#[path = "support/rayon_executor.rs"]
mod rayon_executor;
use rayon_executor::RayonExecutor;

const COUNTS: [usize; 5] = [128, 512, 1_000, 5_000, 10_000];
const WORKERS: [usize; 3] = [2, 4, 8];
const WARMUPS: usize = 8;
const SAMPLES: usize = 40;
const DT: f64 = 1.0 / 120.0;

fn main() -> Result<(), Box<dyn Error>> {
    println!("colored solver scaling: counts={COUNTS:?} workers={WORKERS:?}");
    for count in COUNTS {
        for mode in ["joints", "contacts", "mixed"] {
            for workers in WORKERS {
                run_case(count, mode, workers)?;
            }
            run_sequential(count, mode)?;
        }
    }
    Ok(())
}

fn run_case(count: usize, mode: &str, workers: usize) -> Result<(), Box<dyn Error>> {
    let mut world = fixture(count, mode)?;
    world.set_partial_coloring_for_bench(true);
    let executor = RayonExecutor::new(workers)?;
    for _ in 0..WARMUPS {
        world.step_with_executor(DT, &executor, workers)?;
    }
    let mut times = Vec::with_capacity(SAMPLES);
    let mut binding_times = Vec::with_capacity(SAMPLES);
    let mut executor_times = Vec::with_capacity(SAMPLES);
    let mut processing_times = Vec::with_capacity(SAMPLES);
    let mut outside_times = Vec::with_capacity(SAMPLES);
    let mut rebuild_steps = 0;
    let mut cache_hit_steps = 0;
    for sample in 0..SAMPLES {
        if sample % 5 == 0 {
            // Exercise the steady-state cache without rebuilding the world.
            world.step_with_executor(DT, &executor, workers)?;
        }
        let started = Instant::now();
        world.step_with_executor(DT, &executor, workers)?;
        times.push(started.elapsed().as_nanos());
        let metrics = world.collision_step_metrics();
        binding_times.push(metrics.colored_binding.as_nanos());
        executor_times.push(metrics.colored_executor_wall.as_nanos());
        processing_times.push(metrics.colored_processing_sum.as_nanos());
        outside_times.push(metrics.colored_outside_callbacks.as_nanos());
        if !metrics.colored_solver_selected
            || metrics.colored_groups_dispatched == 0
            || metrics.colored_parallel_batches == 0
        {
            return Err(format!(
                "invalid colored fixture mode={mode} constraints={count} workers={workers}: selected={} groups={} parallel_batches={} rejection={}",
                metrics.colored_solver_selected,
                metrics.colored_groups_dispatched,
                metrics.colored_parallel_batches,
                metrics.colored_schedule.rejection,
            ).into());
        }
        let joint_metrics = world.physics_step_metrics().joints;
        rebuild_steps += usize::from(joint_metrics.constraint_coloring_rebuilds > 0);
        cache_hit_steps += usize::from(joint_metrics.constraint_coloring_cache_hits > 0);
        let joints = world.joints().count();
        let contact_points = metrics.contact_count;
        let graph_constraints = joint_metrics.constraint_coloring_constraints;
        if sample == 0 {
            println!(
                "fixture mode={mode} requested={count} contact_points={contact_points} world_joints={joints} colored_graph_constraints={graph_constraints} topology=unverified",
            );
        }
        if sample == 0 {
            println!(
                "colored mode={mode} constraints={count} workers={workers} colors={} batches={} groups={} parallel_batches={} retained={} rebuilds={} cache_hits={} step_ms={:.3}",
                metrics.colored_schedule.colors,
                metrics.colored_schedule.batches,
                metrics.colored_groups_dispatched,
                metrics.colored_parallel_batches,
                metrics.retained_scratch_bytes,
                world
                    .physics_step_metrics()
                    .joints
                    .constraint_coloring_rebuilds,
                world
                    .physics_step_metrics()
                    .joints
                    .constraint_coloring_cache_hits,
                nanos_to_ms(times[0]),
            );
            println!(
                "timing mode={mode} workers={workers} binding_ms={:.3} processing_ms={:.3} dispatch_overhead_ms={:.3}",
                metrics.colored_binding.as_secs_f64() * 1_000.0,
                metrics.colored_constraint_processing.as_secs_f64() * 1_000.0,
                metrics.colored_dispatch_overhead.as_secs_f64() * 1_000.0,
            );
        }
    }
    times.sort_unstable();
    binding_times.sort_unstable();
    executor_times.sort_unstable();
    processing_times.sort_unstable();
    outside_times.sort_unstable();
    println!(
        "result mode={mode} requested={count} workers={workers} topology=unverified rebuild_steps={rebuild_steps} cache_hit_steps={cache_hit_steps} p50_ms={:.3} p95_ms={:.3}",
        nanos_to_ms(times[SAMPLES / 2]),
        nanos_to_ms(times[p95_index(SAMPLES)]),
    );
    println!(
        "timing_result mode={mode} workers={workers} binding_p50_p95_ms={:.3}/{:.3} executor_wall_p50_p95_ms={:.3}/{:.3} processing_sum_p50_p95_ms={:.3}/{:.3} outside_callbacks_p50_p95_ms={:.3}/{:.3}",
        nanos_to_ms(binding_times[SAMPLES / 2]),
        nanos_to_ms(binding_times[p95_index(SAMPLES)]),
        nanos_to_ms(executor_times[SAMPLES / 2]),
        nanos_to_ms(executor_times[p95_index(SAMPLES)]),
        nanos_to_ms(processing_times[SAMPLES / 2]),
        nanos_to_ms(processing_times[p95_index(SAMPLES)]),
        nanos_to_ms(outside_times[SAMPLES / 2]),
        nanos_to_ms(outside_times[p95_index(SAMPLES)]),
    );
    Ok(())
}

fn run_sequential(count: usize, mode: &str) -> Result<(), Box<dyn Error>> {
    let mut world = fixture(count, mode)?;
    for _ in 0..WARMUPS {
        world.step_with_executor(DT, &SequentialExecutor, 1)?;
    }
    let started = Instant::now();
    world.step_with_executor(DT, &SequentialExecutor, 1)?;
    println!(
        "sequential mode={mode} constraints={count} step_ms={:.3}",
        nanos_to_ms(started.elapsed().as_nanos())
    );
    Ok(())
}

fn fixture(count: usize, mode: &str) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let component_count = count / 2;
    let bodies: Vec<_> = (0..component_count * 2)
        .map(|i| {
            let body =
                RigidBody::new_dynamic(Transform::new(Vec3::X * i as f64, Quat::IDENTITY)?, mass)?;
            world.add_body(body).map_err(Into::into)
        })
        .collect::<Result<_, Box<dyn Error>>>()?;
    if mode != "contacts" {
        for i in 0..component_count {
            let common = JointCommon::new(
                JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
                JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
            );
            let joint = if i % 2 == 0 {
                Joint::Ball(BallJoint::new(common))
            } else {
                Joint::Hinge(HingeJoint::new(common))
            };
            world.add_joint(bodies[2 * i], bodies[2 * i + 1], joint)?;
            let next = (i + 1) % component_count;
            let bridge = JointCommon::new(
                JointFrame::new(Vec3::ZERO, Quat::IDENTITY),
                JointFrame::new(Vec3::ZERO, Quat::IDENTITY),
            );
            world.add_joint(
                bodies[2 * i + 1],
                bodies[2 * next],
                Joint::Ball(BallJoint::new(bridge)),
            )?;
        }
    }
    if mode == "contacts" || mode == "mixed" {
        let static_body = world.add_body(RigidBody::new_static(Transform::new(
            Vec3::new(0.0, -1.0, 0.0),
            Quat::IDENTITY,
        )?))?;
        world.add_collider(
            static_body,
            Collider::new(Shape::Sphere(Sphere::new(1.0)?), Transform::identity()),
        )?;
        for (i, body) in bodies.iter().enumerate() {
            world.add_collider(
                *body,
                Collider::new(
                    Shape::Sphere(Sphere::new(1.0)?),
                    Transform::new(
                        Vec3::new(0.0, -0.25 - (i % 3) as f64 * 0.1, 0.0),
                        Quat::IDENTITY,
                    )?,
                ),
            )?;
        }
    }
    Ok(world)
}

fn nanos_to_ms(ns: u128) -> f64 {
    ns as f64 / 1_000_000.0
}

fn p95_index(sample_count: usize) -> usize {
    ((sample_count * 95).div_ceil(100)).saturating_sub(1)
}
