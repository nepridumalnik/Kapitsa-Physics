use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

use kapitsa_core::{
    mass::MassProperties, rigid_body::RigidBody, transform::Transform, world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Vec3};
use kapitsa_tokio::{PhysicsController, SchedulerConfig};
use tokio::runtime::Builder;

const BODY_COUNT: usize = 20_000;
const STEPS: usize = 24;
const SAMPLES: usize = 5;

fn main() -> Result<(), Box<dyn Error>> {
    let runtime = Builder::new_current_thread().build()?;
    let mut config = SchedulerConfig {
        max_workers: 4,
        max_catch_up_steps: 1,
        adaptive: false,
        ..SchedulerConfig::default()
    };
    let quantum = Duration::from_secs_f64(1.0 / f64::from(config.target_hz));

    println!("kapitsa integration benchmark");
    println!("bodies={BODY_COUNT}, steps={STEPS}, samples={SAMPLES}, statistic=median");

    let sequential = sample(SAMPLES, || {
        let mut world = make_world(BODY_COUNT)?;
        let started = Instant::now();
        for _ in 0..STEPS {
            world.step(quantum.as_secs_f64())?;
        }
        Ok(started.elapsed())
    })?;
    report("sequential", sequential, sequential);

    for workers in [1, 2, 4] {
        config.max_workers = workers;
        let parallel = sample(SAMPLES, || {
            let world = make_world(BODY_COUNT)?;
            let mut controller = PhysicsController::new(world, config.clone())?;
            controller.set_active_workers(workers)?;
            let started = Instant::now();
            for _ in 0..STEPS {
                runtime.block_on(controller.step())?;
            }
            Ok(started.elapsed())
        })?;
        report(&format!("Tokio {workers} worker(s)"), parallel, sequential);
    }
    Ok(())
}

fn make_world(body_count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    for index in 0..body_count {
        let scale = index as f64 * 0.001 + 1.0;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::new(scale, -scale, 0.5 * scale))?;
        body.set_angular_velocity(Vec3::new(0.1 * scale, 0.2, -0.3))?;
        body.apply_force(Vec3::new(0.25, -0.5 * scale, 0.75))?;
        body.apply_torque(Vec3::new(-0.1 * scale, 0.2, 0.05))?;
        world.add_body(body)?;
    }
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
