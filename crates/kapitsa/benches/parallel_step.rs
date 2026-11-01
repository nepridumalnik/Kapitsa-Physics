use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    execution::{ExecutionError, StepExecutor},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};
use rayon::{ThreadPool, ThreadPoolBuilder, prelude::*};

const DT: f64 = 1.0 / 60.0;
const STEPS: usize = 12;
const SAMPLES: usize = 5;
const BODY_COUNTS: [usize; 3] = [256, 4_096, 20_000];

struct RayonStepExecutor {
    pool: ThreadPool,
}

impl StepExecutor for RayonStepExecutor {
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
        if chunk_size == 0 {
            return Err(ExecutionError::InvalidChunkLayout);
        }
        self.pool.install(|| {
            values
                .par_chunks_mut(chunk_size)
                .enumerate()
                .for_each(|(index, chunk)| operation(index, chunk));
        });
        Ok(())
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
        validate_layout(values.len(), chunk_sizes)?;
        self.pool.scope(|scope| {
            let mut remaining = values;
            let mut start = 0;
            for &chunk_size in chunk_sizes {
                let (chunk, tail) = remaining.split_at_mut(chunk_size);
                let operation = &operation;
                scope.spawn(move |_| operation(start, chunk));
                start += chunk_size;
                remaining = tail;
            }
        });
        Ok(())
    }
}

fn validate_layout(length: usize, chunk_sizes: &[usize]) -> Result<(), ExecutionError> {
    let total = chunk_sizes.iter().try_fold(0_usize, |total, &size| {
        if size == 0 {
            None
        } else {
            total.checked_add(size)
        }
    });
    if total == Some(length) {
        Ok(())
    } else {
        Err(ExecutionError::InvalidChunkLayout)
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    println!(
        "Kapitsa core parallel-step benchmark; steps={STEPS}, samples={SAMPLES}, statistic=median"
    );
    for body_count in BODY_COUNTS {
        let mut medians = Vec::with_capacity(3);
        for workers in [1, 2, 4] {
            let elapsed = sample(|| {
                let mut world = make_world(body_count, false)?;
                let executor = executor(workers)?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    black_box(world.step_with_executor(DT, &executor, workers)?);
                }
                Ok(started.elapsed().div_f64(STEPS as f64))
            })?;
            medians.push(elapsed);
        }
        println!("\n{body_count} dynamic bodies");
        for (workers, elapsed) in [1, 2, 4].into_iter().zip(medians.iter().copied()) {
            println!(
                "workers={workers} median={:8.3} ms speedup={:.2}x",
                millis(elapsed),
                medians[0].as_secs_f64() / elapsed.as_secs_f64(),
            );
        }
    }

    let mut medians = Vec::with_capacity(3);
    for workers in [1, 2, 4] {
        let elapsed = sample(|| {
            let mut world = collision_world()?;
            let executor = executor(workers)?;
            let started = Instant::now();
            for _ in 0..STEPS {
                black_box(world.step_with_executor(DT, &executor, workers)?);
            }
            Ok(started.elapsed().div_f64(STEPS as f64))
        })?;
        medians.push(elapsed);
    }
    println!("\n4,096 dynamic bodies / 2,048 two-body contact islands");
    for (workers, elapsed) in [1, 2, 4].into_iter().zip(medians.iter().copied()) {
        println!(
            "workers={workers} median={:8.3} ms speedup={:.2}x",
            millis(elapsed),
            medians[0].as_secs_f64() / elapsed.as_secs_f64(),
        );
    }
    Ok(())
}

fn executor(workers: usize) -> Result<RayonStepExecutor, Box<dyn Error>> {
    Ok(RayonStepExecutor {
        pool: ThreadPoolBuilder::new().num_threads(workers).build()?,
    })
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

fn sample(run: impl Fn() -> Result<Duration, Box<dyn Error>>) -> Result<Duration, Box<dyn Error>> {
    black_box(run()?);
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        samples.push(run()?);
    }
    samples.sort_unstable();
    samples
        .get(SAMPLES / 2)
        .copied()
        .ok_or_else(|| "missing benchmark sample".into())
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1_000.0
}
