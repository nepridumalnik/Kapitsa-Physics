use std::{
    collections::VecDeque,
    error::Error,
    hint::black_box,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use kapitsa_core::{
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
    world::{BodyStep, PhysicsWorld, StepError},
};
use kapitsa_math::{Mat3, Vec3};
use kapitsa_tokio::{PhysicsController, SchedulerConfig};
use tokio::runtime::{Builder, Runtime};

const BODY_COUNTS: [(&str, usize); 3] = [("small", 256), ("medium", 4_096), ("large", 20_000)];
const WORKLOADS: [Workload; 3] = [
    Workload {
        name: "dynamic",
        static_percent: 0,
        sleeping_percent: 0,
    },
    Workload {
        name: "mixed",
        static_percent: 50,
        sleeping_percent: 25,
    },
    Workload {
        name: "sleeping-heavy",
        static_percent: 10,
        sleeping_percent: 80,
    },
];
const STEPS: usize = 12;
const SAMPLES: usize = 5;

#[derive(Clone, Copy)]
struct Workload {
    name: &'static str,
    static_percent: usize,
    sleeping_percent: usize,
}

fn main() -> Result<(), Box<dyn Error>> {
    let runtime = Builder::new_current_thread().build()?;
    println!("kapitsa integration benchmark; steps={STEPS}, samples={SAMPLES}, statistic=median");

    for (size, body_count) in BODY_COUNTS {
        for workload in WORKLOADS {
            let sequential = sample(SAMPLES, || {
                let mut world = make_world(body_count, workload)?;
                let dt = 1.0 / 60.0;
                let started = Instant::now();
                for _ in 0..STEPS {
                    world.step(dt)?;
                }
                Ok(started.elapsed())
            })?;
            println!("\n{size} world ({}): {body_count} bodies", workload.name);
            report("PhysicsWorld::step", sequential, sequential);

            let fast_path = sample(SAMPLES, || {
                let world = make_world(body_count, workload)?;
                let mut controller = PhysicsController::new(world, config(1))?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    runtime.block_on(controller.step())?;
                }
                Ok(started.elapsed())
            })?;
            report("Tokio fast path", fast_path, sequential);

            let static_shards = sample(SAMPLES, || {
                let mut world = make_world(body_count, workload)?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    world = runtime.block_on(static_sharded_step(world, 1.0 / 60.0, 2))?;
                }
                Ok(started.elapsed())
            })?;
            report("static shards x2", static_shards, sequential);

            let dynamic_buckets = sample(SAMPLES, || {
                let world = make_world(body_count, workload)?;
                let mut controller = PhysicsController::new(world, config(2))?;
                controller.set_active_workers(2)?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    runtime.block_on(controller.step())?;
                }
                Ok(started.elapsed())
            })?;
            report("dynamic buckets x2", dynamic_buckets, sequential);
        }
    }
    for workload in WORKLOADS {
        report_phases(&runtime, workload)?;
    }
    Ok(())
}

fn report_phases(runtime: &Runtime, workload: Workload) -> Result<(), Box<dyn Error>> {
    let mut samples = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    for _ in 0..SAMPLES {
        let mut world = make_world(20_000, workload)?;
        let mut buckets = (0..8).map(|_| Vec::new()).collect::<Vec<Vec<BodyStep>>>();
        let mut times = [Duration::ZERO; 4];
        for _ in 0..STEPS {
            let started = Instant::now();
            world.fill_step_shards(&mut buckets)?;
            times[0] += started.elapsed();

            let started = Instant::now();
            let queue = Arc::new(Mutex::new(
                buckets.into_iter().enumerate().collect::<VecDeque<_>>(),
            ));
            let mut tasks = Vec::new();
            for _ in 0..2 {
                let queue = Arc::clone(&queue);
                tasks.push(runtime.spawn_blocking(
                    move || -> Result<Vec<(usize, Vec<BodyStep>)>, StepError> {
                        let mut completed = Vec::new();
                        loop {
                            let next = queue
                                .lock()
                                .unwrap_or_else(std::sync::PoisonError::into_inner)
                                .pop_front();
                            let Some((index, mut bucket)) = next else {
                                break;
                            };
                            for body in &mut bucket {
                                body.integrate(1.0 / 60.0)?;
                            }
                            completed.push((index, bucket));
                        }
                        Ok(completed)
                    },
                ));
            }
            times[1] += started.elapsed();

            let started = Instant::now();
            let completed = runtime.block_on(async {
                let mut completed = Vec::new();
                for task in tasks {
                    completed.extend(task.await??);
                }
                Ok::<_, Box<dyn Error>>(completed)
            })?;
            let mut ordered = (0..8).map(|_| None).collect::<Vec<_>>();
            for (index, bucket) in completed {
                ordered[index] = Some(bucket);
            }
            buckets = ordered
                .into_iter()
                .map(|bucket| bucket.ok_or("missing bucket"))
                .collect::<Result<_, _>>()?;
            times[2] += started.elapsed();

            let started = Instant::now();
            world.commit_step_shards(&buckets)?;
            times[3] += started.elapsed();
        }
        for (sample, time) in samples.iter_mut().zip(times) {
            sample.push(time);
        }
    }
    for sample in &mut samples {
        sample.sort_unstable();
    }
    println!(
        "\n{} raw bucket phases, 20,000 bodies (ms / {STEPS} steps): prepare={:.2}, dispatch={:.2}, join+collect={:.2}, commit={:.2}",
        workload.name,
        samples[0][SAMPLES / 2].as_secs_f64() * 1000.0,
        samples[1][SAMPLES / 2].as_secs_f64() * 1000.0,
        samples[2][SAMPLES / 2].as_secs_f64() * 1000.0,
        samples[3][SAMPLES / 2].as_secs_f64() * 1000.0
    );
    Ok(())
}

fn config(workers: usize) -> SchedulerConfig {
    SchedulerConfig {
        max_workers: workers,
        max_catch_up_steps: 1,
        adaptive: false,
        ..SchedulerConfig::default()
    }
}

fn make_world(body_count: usize, workload: Workload) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let static_count = body_count * workload.static_percent / 100;
    let sleeping_count = body_count * workload.sleeping_percent / 100;
    for index in 0..body_count {
        if index < static_count {
            world.add_body(RigidBody::new_static(Transform::identity()))?;
            continue;
        }

        let scale = index as f64 * 0.001 + 1.0;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::new(scale, -scale, 0.5 * scale))?;
        body.set_angular_velocity(Vec3::new(0.1 * scale, 0.2, -0.3))?;
        body.apply_force(Vec3::new(0.25, -0.5 * scale, 0.75))?;
        body.apply_torque(Vec3::new(-0.1 * scale, 0.2, 0.05))?;
        if index < static_count + sleeping_count {
            body.sleep()?;
        }
        world.add_body(body)?;
    }
    Ok(world)
}

async fn static_sharded_step(
    world: PhysicsWorld,
    dt: f64,
    workers: usize,
) -> Result<PhysicsWorld, Box<dyn Error>> {
    let (world, mut shards, result) = tokio::task::spawn_blocking(move || {
        let mut shards = (0..workers).map(|_| Vec::new()).collect::<Vec<_>>();
        let result = world.fill_step_shards(&mut shards);
        (world, shards, result)
    })
    .await?;
    result?;

    let tasks = shards
        .drain(..)
        .map(|mut shard| {
            tokio::task::spawn_blocking(move || {
                let mut error = None;
                for body in &mut shard {
                    if let Err(step_error) = body.integrate(dt) {
                        error = Some(step_error);
                        break;
                    }
                }
                (shard, error)
            })
        })
        .collect::<Vec<_>>();
    let mut buffers = Vec::with_capacity(tasks.len());
    let mut integration_error = None;
    for task in tasks {
        let (shard, error) = task.await?;
        buffers.push(shard);
        if integration_error.is_none() {
            integration_error = error;
        }
    }
    if let Some(error) = integration_error {
        return Err(Box::new(error));
    }

    let (world, result) = tokio::task::spawn_blocking(move || {
        let mut world = world;
        let result = world.commit_step_shards(&buffers);
        (world, result)
    })
    .await?;
    result?;
    Ok(world)
}

fn sample(
    count: usize,
    mut run: impl FnMut() -> Result<Duration, Box<dyn Error>>,
) -> Result<Duration, Box<dyn Error>> {
    let mut timings = Vec::with_capacity(count);
    for _ in 0..count {
        timings.push(run()?);
    }
    timings.sort_unstable();
    Ok(black_box(timings[count / 2]))
}

fn report(label: &str, duration: Duration, sequential: Duration) {
    println!(
        "{label:20} median={:8.2} ms  speedup={:.2}x",
        duration.as_secs_f64() * 1_000.0,
        sequential.as_secs_f64() / duration.as_secs_f64()
    );
}
