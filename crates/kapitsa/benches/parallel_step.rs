use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

use kapitsa::execution::RayonPool;
use kapitsa_core::{
    collider::Collider,
    execution::StepExecutor,
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

const DT: f64 = 1.0 / 60.0;
const STEPS: usize = 12;
const SAMPLES: usize = 5;
const BODY_COUNTS: [usize; 3] = [256, 4_096, 20_000];

fn main() -> Result<(), Box<dyn Error>> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(1)
        .build()?;
    let pool = RayonPool::new(runtime.handle(), 8)?;
    println!(
        "Kapitsa core parallel-step benchmark; steps={STEPS}, warmups=1, samples={SAMPLES}, statistics=min/median/max"
    );
    for body_count in BODY_COUNTS {
        let mut samples_by_worker = Vec::with_capacity(4);
        for workers in [1, 2, 4, 8] {
            let elapsed = sample(|| {
                let mut world = make_world(body_count, false)?;
                let mut executor = pool.executor()?;
                executor.set_parallelism(workers);
                let started = Instant::now();
                for _ in 0..STEPS {
                    black_box(world.step_with_executor(DT, &executor, workers)?);
                }
                Ok(started.elapsed().div_f64(STEPS as f64))
            })?;
            samples_by_worker.push(elapsed);
        }
        println!("\n{body_count} dynamic bodies");
        let baseline = samples_by_worker[0].1;
        for (workers, (min, median, max)) in [1, 2, 4, 8]
            .into_iter()
            .zip(samples_by_worker.iter().copied())
        {
            println!(
                "workers={workers} step={:8.3}/{:8.3}/{:8.3} ms min/median/max speedup={:.2}x",
                millis(min),
                millis(median),
                millis(max),
                baseline.as_secs_f64() / median.as_secs_f64(),
            );
        }
    }

    let mut samples_by_worker = Vec::with_capacity(4);
    for workers in [1, 2, 4, 8] {
        let elapsed = sample(|| {
            let mut world = collision_world()?;
            let mut executor = pool.executor()?;
            executor.set_parallelism(workers);
            let started = Instant::now();
            for _ in 0..STEPS {
                black_box(world.step_with_executor(DT, &executor, workers)?);
            }
            Ok(started.elapsed().div_f64(STEPS as f64))
        })?;
        samples_by_worker.push(elapsed);
    }
    println!("\n4,096 dynamic bodies / 2,048 two-body contact islands");
    let baseline = samples_by_worker[0].1;
    for (workers, (min, median, max)) in [1, 2, 4, 8]
        .into_iter()
        .zip(samples_by_worker.iter().copied())
    {
        println!(
            "workers={workers} step={:8.3}/{:8.3}/{:8.3} ms min/median/max speedup={:.2}x",
            millis(min),
            millis(median),
            millis(max),
            baseline.as_secs_f64() / median.as_secs_f64(),
        );
    }
    println!(
        "\n20,000 dynamic bodies / central gravity / sharded prediction / production executor"
    );
    for workers in [1, 2, 4, 8] {
        let (min, median, max) = sample(|| {
            let mut world = make_world(20_000, false)?;
            world.set_central_gravity(Some(kapitsa_core::CentralGravity::new(
                Vec3::new(-100.0, 0.0, 0.0),
                10_000.0,
            )?));
            let mut executor = pool.executor()?;
            executor.set_parallelism(workers);
            let mut shards = vec![Vec::new(); workers * 4];
            let started = Instant::now();
            for _ in 0..STEPS {
                black_box(world.step_with_shards(DT, &mut shards, &executor, workers)?);
            }
            Ok(started.elapsed().div_f64(STEPS as f64))
        })?;
        println!(
            "workers={workers} step={:.3}/{:.3}/{:.3} ms min/median/max",
            millis(min),
            millis(median),
            millis(max)
        );
    }
    Ok(())
}

fn make_world(body_count: usize, sleeping_heavy: bool) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let sleeping_count = if sleeping_heavy {
        body_count * 80 / 100
    } else {
        0
    };
    for index in 0..body_count {
        let position = Vec3::new(index as f64 * 2.0, 0.0, 0.0);
        let mut body = RigidBody::new_dynamic(Transform::new(position, Quat::IDENTITY)?, mass)?;
        body.set_linear_velocity(Vec3::new(1.0, -0.5, 0.25))?;
        body.set_angular_velocity(Vec3::new(0.1, 0.2, -0.3))?;
        if index < sleeping_count {
            body.sleep()?;
        }
        world.add_body(body)?;
    }
    Ok(world)
}

fn collision_world() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    for pair in 0..2_048 {
        let x = pair as f64 * 3.0;
        for offset in [0.0, 0.9] {
            let body = world.add_body(RigidBody::new_dynamic(
                Transform::new(Vec3::new(x + offset, 0.0, 0.0), Quat::IDENTITY)?,
                mass,
            )?)?;
            world.add_collider(
                body,
                Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
            )?;
        }
    }
    Ok(world)
}

fn sample(
    run: impl Fn() -> Result<Duration, Box<dyn Error>>,
) -> Result<(Duration, Duration, Duration), Box<dyn Error>> {
    black_box(run()?);
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        samples.push(run()?);
    }
    samples.sort_unstable();
    let min = *samples.first().ok_or("missing benchmark sample")?;
    let median = samples[SAMPLES / 2];
    let max = *samples.last().ok_or("missing benchmark sample")?;
    Ok((min, black_box(median), max))
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1_000.0
}
