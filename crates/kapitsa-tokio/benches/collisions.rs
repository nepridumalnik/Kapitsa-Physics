use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Plane, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};
use kapitsa_tokio::{PhysicsController, SchedulerConfig};
use tokio::runtime::Builder;

const SAMPLES: usize = 5;
const STEPS: usize = 8;
const DT: f64 = 1.0 / 60.0;

fn main() -> Result<(), Box<dyn Error>> {
    let runtime = Builder::new_current_thread().enable_time().build()?;
    println!(
        "Kapitsa Tokio collision benchmark: samples={SAMPLES}, steps={STEPS}, statistic=median"
    );

    for body_count in [256, 4_096, 20_000] {
        let sequential = sample(|| {
            let mut world = collision_world(body_count)?;
            let started = Instant::now();
            for _ in 0..STEPS {
                black_box(world.step(DT)?);
            }
            Ok(started.elapsed())
        })?;
        println!("\n{body_count} bodies, localized collision workload");
        report("PhysicsWorld::step", sequential, sequential);

        for workers in [1, 2, 4] {
            let elapsed = sample(|| {
                let world = collision_world(body_count)?;
                let mut controller = PhysicsController::new(world, config(workers))?;
                controller.set_active_workers(workers)?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    black_box(runtime.block_on(controller.step())?);
                }
                Ok(started.elapsed())
            })?;
            report(
                &format!("PhysicsController x{workers}"),
                elapsed,
                sequential,
            );
        }
    }
    Ok(())
}

fn collision_world(body_count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let plane = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        plane,
        Collider::new(
            Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0)?),
            Transform::identity(),
        ),
    )?;

    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let contact_count = (body_count / 8).min(128);
    for index in 0..body_count {
        let (position, velocity) = if index < contact_count {
            let x = (index % 32) as f64 * 2.0;
            let z = (index / 32) as f64 * 2.0;
            (Vec3::new(x, 0.5, z), Vec3::ZERO)
        } else {
            let index = index - contact_count;
            let x = (index % 200) as f64 * 2.0;
            let z = (index / 200) as f64 * 2.0;
            (Vec3::new(x, 5.0, z), Vec3::new(0.0, -0.1, 0.0))
        };
        let mut body = RigidBody::new_dynamic(Transform::new(position, Quat::IDENTITY)?, mass)?;
        body.set_linear_velocity(velocity)?;
        let id = world.add_body(body)?;
        world.add_collider(
            id,
            Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
        )?;
    }
    Ok(world)
}

fn config(workers: usize) -> SchedulerConfig {
    SchedulerConfig {
        max_workers: workers,
        max_catch_up_steps: 1,
        adaptive: false,
        ..SchedulerConfig::default()
    }
}

fn sample(run: impl Fn() -> Result<Duration, Box<dyn Error>>) -> Result<Duration, Box<dyn Error>> {
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        samples.push(run()?);
    }
    samples.sort_unstable();
    Ok(black_box(samples[SAMPLES / 2]))
}

fn report(label: &str, duration: Duration, sequential: Duration) {
    println!(
        "{label:24} median={:8.2} ms / {STEPS} steps  speedup={:.2}x",
        duration.as_secs_f64() * 1_000.0,
        sequential.as_secs_f64() / duration.as_secs_f64()
    );
}
