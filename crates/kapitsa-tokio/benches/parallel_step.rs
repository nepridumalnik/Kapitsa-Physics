use std::{
    collections::VecDeque,
    error::Error,
    future::Future,
    hint::black_box,
    pin::Pin,
    sync::{Arc, Mutex},
    task::{Context, Poll, Waker},
    time::{Duration, Instant},
};

use kapitsa_core::{
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
    world::{BodyStep, PhysicsWorld, StepError},
};
use kapitsa_math::{Mat3, Vec3};
use kapitsa_tokio::{PhysicsClientError, PhysicsController, PhysicsSimulation, SchedulerConfig};
use tokio::{
    runtime::{Builder, Runtime},
    sync::{mpsc, watch},
};

#[allow(dead_code, unused_imports)]
mod marshalling {
    #[path = "../../src/marshalling/client.rs"]
    mod client;
    #[path = "../../src/marshalling/command.rs"]
    mod command;
    #[path = "../../src/marshalling/snapshot.rs"]
    mod snapshot;

    pub(crate) use client::PhysicsClient;
    pub(crate) use command::{PhysicsCommand, PhysicsCommandResult, apply_command};
    pub(crate) use snapshot::PhysicsSnapshot;
}

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
    let runtime = Builder::new_current_thread().enable_time().build()?;
    println!("kapitsa integration benchmark; steps={STEPS}, samples={SAMPLES}, statistic=median");
    if std::env::var_os("KAPITSA_MARSHALLING_ONLY").is_some() {
        return runtime.block_on(async {
            tokio::time::pause();
            report_marshalling().await
        });
    }

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
    runtime.block_on(async {
        tokio::time::pause();
        report_marshalling().await
    })?;
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

#[derive(Clone, Copy, Default)]
struct PhaseTimes {
    total: Duration,
    commands: Duration,
    physics: Duration,
    snapshot_build: Duration,
    publication: Duration,
}

#[derive(Clone, Copy, Default)]
struct SimulationTimes {
    wall: Duration,
    physics: Duration,
    steps: u64,
}

async fn report_marshalling() -> Result<(), Box<dyn Error>> {
    println!(
        "\nphysics marshalling (dynamic bodies, {STEPS} steps, {SAMPLES} samples, median; Tokio timer paused)"
    );
    for (size, body_count) in BODY_COUNTS {
        let mut controller_wall = Vec::with_capacity(SAMPLES);
        let mut no_command = Vec::with_capacity(SAMPLES);
        let mut with_commands = Vec::with_capacity(SAMPLES);
        let mut simulation_wall = Vec::with_capacity(SAMPLES);
        let mut simulation_commands_wall = Vec::with_capacity(SAMPLES);
        let mut snapshot_reads = Vec::with_capacity(SAMPLES);

        for _ in 0..SAMPLES {
            controller_wall.push(controller_sample(body_count).await?);
            no_command.push(pipeline_sample(body_count, 0).await?);
            with_commands.push(pipeline_sample(body_count, 100).await?);
            simulation_wall.push(simulation_sample(body_count, 0).await?);
            simulation_commands_wall.push(simulation_sample(body_count, 100).await?);
            snapshot_reads.push(snapshot_read_sample(body_count)?);
        }

        let baseline = median(&mut controller_wall);
        let plain = median_phase(&mut no_command);
        let commanded = median_phase(&mut with_commands);
        let actual_simulation = median_simulation(&mut simulation_wall);
        let actual_commands = median_simulation(&mut simulation_commands_wall);
        let reads = median(&mut snapshot_reads);
        println!(
            "\n{size}: {body_count} bodies | PhysicsController::step wall={:.2} ms | simulation API no commands={:.2} ms ({} steps, physics={:.2} ms), 100 ApplyForce/step={:.2} ms ({} steps, physics={:.2} ms)",
            millis(baseline),
            millis(actual_simulation.wall),
            actual_simulation.steps,
            millis(actual_simulation.physics),
            millis(actual_commands.wall),
            actual_commands.steps,
            millis(actual_commands.physics),
        );
        println!(
            "  isolated pipeline / {STEPS} steps: commands={:.3} ms, controller={:.2} ms, snapshot build={:.2} ms, watch publication={:.3} ms, total={:.2} ms",
            millis(commanded.commands),
            millis(commanded.physics),
            millis(commanded.snapshot_build),
            millis(commanded.publication),
            millis(commanded.total),
        );
        println!(
            "  no-command pipeline: controller={:.2} ms, snapshot build={:.2} ms, watch publication={:.3} ms, total={:.2} ms; 4 clients x 1,000 snapshot reads={:.3} ms",
            millis(plain.physics),
            millis(plain.snapshot_build),
            millis(plain.publication),
            millis(plain.total),
            millis(reads),
        );
    }
    Ok(())
}

async fn controller_sample(body_count: usize) -> Result<Duration, Box<dyn Error>> {
    let world = make_world(body_count, WORKLOADS[0])?;
    let mut controller = PhysicsController::new(world, config(1))?;
    let started = Instant::now();
    for _ in 0..STEPS {
        black_box(controller.step().await?);
    }
    Ok(started.elapsed())
}

async fn pipeline_sample(
    body_count: usize,
    commands_per_step: usize,
) -> Result<PhaseTimes, Box<dyn Error>> {
    let world = make_world(body_count, WORKLOADS[0])?;
    let ids = world.bodies().map(|(id, _)| id).collect::<Vec<_>>();
    let mut controller = PhysicsController::new(world, config(1))?;
    let initial = Arc::new(marshalling::PhysicsSnapshot::from_world(
        controller.world()?,
        0,
        Duration::ZERO,
    ));
    let (snapshot_tx, _snapshot_rx) = watch::channel(initial);
    let mut simulation_time = Duration::ZERO;
    let dt = Duration::from_secs_f64(1.0 / 60.0);
    let mut times = PhaseTimes::default();
    let started = Instant::now();

    for _ in 0..STEPS {
        let commands = ids
            .iter()
            .take(commands_per_step)
            .map(|id| marshalling::PhysicsCommand::ApplyForce {
                id: *id,
                force: Vec3::Y * 0.1,
            })
            .collect::<Vec<_>>();
        let phase = Instant::now();
        let world = controller.world_mut()?;
        for command in commands {
            black_box(marshalling::apply_command(world, command)?);
        }
        times.commands += phase.elapsed();

        let stats = controller.step().await?;
        times.physics += stats.step_duration;
        simulation_time = simulation_time
            .checked_add(dt)
            .ok_or("benchmark simulation time overflow")?;

        let phase = Instant::now();
        let snapshot = marshalling::PhysicsSnapshot::from_world(
            controller.world()?,
            controller.metrics().total_steps,
            simulation_time,
        );
        times.snapshot_build += phase.elapsed();

        let phase = Instant::now();
        snapshot_tx.send_replace(Arc::new(snapshot));
        times.publication += phase.elapsed();
    }
    times.total = started.elapsed();
    Ok(times)
}

async fn simulation_sample(
    body_count: usize,
    commands_per_step: usize,
) -> Result<SimulationTimes, Box<dyn Error>> {
    let world = make_world(body_count, WORKLOADS[0])?;
    let command_ids = world
        .bodies()
        .take(commands_per_step)
        .map(|(id, _)| id)
        .collect::<Vec<_>>();
    let mut simulation = PhysicsSimulation::with_config(
        world,
        SchedulerConfig {
            max_workers: 1,
            max_catch_up_steps: 1,
            adaptive: false,
            ..SchedulerConfig::default()
        },
    )?;
    let client = simulation.client();
    let mut reader = client.clone();
    simulation.start()?;
    tokio::task::yield_now().await;
    let start_snapshot =
        tokio::time::timeout(Duration::from_secs(5), reader.next_snapshot()).await??;
    tokio::task::yield_now().await;

    let started = Instant::now();
    let mut expected_step = start_snapshot.step().saturating_add(1);
    let mut physics = Duration::ZERO;
    for _ in 0..STEPS {
        let mut submissions = Vec::with_capacity(command_ids.len());
        for id in &command_ids {
            submissions.push(Box::pin(client.apply_force(*id, Vec3::Y * 0.1)));
        }
        for submission in &mut submissions {
            if !matches!(poll_once(submission.as_mut()), Poll::Pending) {
                return Err("physics command completed before a step".into());
            }
        }

        let snapshot =
            tokio::time::timeout(Duration::from_secs(5), reader.next_snapshot()).await??;
        if snapshot.step() != expected_step {
            return Err("simulation did not advance exactly one benchmark step".into());
        }
        expected_step = snapshot.step().saturating_add(1);
        for submission in submissions {
            submission.await?;
        }
        let metrics = simulation.metrics();
        if metrics.total_steps != snapshot.step() {
            return Err("command batch was not applied in one physical step".into());
        }
        physics += metrics.step_duration;
        tokio::task::yield_now().await;
    }
    let elapsed = started.elapsed();
    simulation.stop().await?;
    Ok(SimulationTimes {
        wall: elapsed,
        physics,
        steps: simulation.metrics().total_steps,
    })
}

fn snapshot_read_sample(body_count: usize) -> Result<Duration, Box<dyn Error>> {
    let world = make_world(body_count, WORKLOADS[0])?;
    let first_id = world.bodies().next().ok_or("empty benchmark world")?.0;
    let initial = Arc::new(marshalling::PhysicsSnapshot::from_world(
        &world,
        0,
        Duration::ZERO,
    ));
    let (command_tx, _command_rx) = mpsc::channel(1);
    let (_command_state_tx, command_state_rx) = watch::channel(Some(command_tx));
    let (_snapshot_tx, snapshot_rx) = watch::channel(initial);
    let clients = (0..4)
        .map(|_| marshalling::PhysicsClient::new(command_state_rx.clone(), snapshot_rx.clone()))
        .collect::<Vec<_>>();

    let started = Instant::now();
    for _ in 0..1_000 {
        for client in &clients {
            let snapshot = client.snapshot();
            black_box(snapshot.body(first_id));
        }
    }
    Ok(started.elapsed())
}

fn poll_once<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
    let waker = Waker::noop();
    future.poll(&mut Context::from_waker(waker))
}

fn median(samples: &mut [Duration]) -> Duration {
    samples.sort_unstable();
    samples[samples.len() / 2]
}

fn median_phase(samples: &mut [PhaseTimes]) -> PhaseTimes {
    samples.sort_unstable_by_key(|sample| sample.total);
    samples[samples.len() / 2]
}

fn median_simulation(samples: &mut [SimulationTimes]) -> SimulationTimes {
    samples.sort_unstable_by_key(|sample| sample.wall);
    samples[samples.len() / 2]
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
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
