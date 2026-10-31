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

const BODY_COUNTS: [(&str, usize); 3] = [("small", 256), ("medium", 4_096), ("large", 20_000)];
const STEPS: usize = 12;
const SAMPLES: usize = 5;

fn main() -> Result<(), Box<dyn Error>> {
    let runtime = Builder::new_current_thread().build()?;
    println!("kapitsa integration benchmark; steps={STEPS}, samples={SAMPLES}, statistic=median");

    for (size, body_count) in BODY_COUNTS {
        let sequential = sample(SAMPLES, || {
            let mut world = make_world(body_count)?;
            let quantum = Duration::from_secs_f64(1.0 / 60.0).as_secs_f64();
            let started = Instant::now();
            for _ in 0..STEPS {
                world.step(quantum)?;
            }
            Ok(started.elapsed())
        })?;
        println!("\n{size} world: {body_count} bodies");
        report("PhysicsWorld::step", sequential, sequential);

        for workers in [1, 2] {
            let config = SchedulerConfig {
                max_workers: workers,
                max_catch_up_steps: 1,
                adaptive: false,
                ..SchedulerConfig::default()
            };
            let scheduled = sample(SAMPLES, || {
                let world = make_world(body_count)?;
                let mut controller = PhysicsController::new(world, config.clone())?;
                controller.set_active_workers(workers)?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    runtime.block_on(controller.step())?;
                }
                Ok(started.elapsed())
            })?;
            report(&format!("Tokio {workers} worker(s)"), scheduled, sequential);
        }
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
