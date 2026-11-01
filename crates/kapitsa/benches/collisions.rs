use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    mass::MassProperties,
    rigid_body::{CcdMode, RigidBody},
    shape::{Cuboid, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

const SAMPLES: usize = 5;
const WARMUP_RUNS: usize = 1;
const STEPS: usize = 8;
const DT: f64 = 1.0 / 60.0;

fn main() -> Result<(), Box<dyn Error>> {
    println!(
        "Kapitsa collision baseline: warmups={WARMUP_RUNS}, samples={SAMPLES}, steps={STEPS}, statistics=min/median/max"
    );
    for body_count in [256, 4_096, 20_000] {
        let elapsed = sample(|| {
            let mut world = collision_world(body_count)?;
            let started = Instant::now();
            for _ in 0..STEPS {
                black_box(world.step(DT)?);
            }
            Ok(started.elapsed())
        })?;
        report(&format!("{body_count} body step"), elapsed, elapsed);
    }

    println!("\nIndependent GJK CCD islands (one step/sample)");
    for body_count in [256, 1_024] {
        let elapsed = sample(|| {
            let mut world = gjk_ccd_world(body_count)?;
            let started = Instant::now();
            black_box(world.step(DT)?);
            Ok(started.elapsed())
        })?;
        println!("{body_count} bodies, median={:.3} ms", millis(elapsed.1));
    }
    Ok(())
}

fn gjk_ccd_world(body_count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let wall = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let columns = (body_count as f64).sqrt().ceil() as usize;
    let side = columns as f64 * 8.0 + 4.0;
    world.add_collider(
        wall,
        Collider::new(
            Shape::Cuboid(Cuboid::new(0.1, side, side)?),
            Transform::identity(),
        ),
    )?;
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    for index in 0..body_count {
        let y = (index % columns) as f64 * 4.0;
        let z = (index / columns) as f64 * 4.0;
        let mut body =
            RigidBody::new_dynamic(Transform::new(Vec3::new(-5.0, y, z), Quat::IDENTITY)?, mass)?;
        body.set_linear_velocity(Vec3::new(1_000.0, 0.0, 0.0))?;
        body.set_ccd_mode(CcdMode::Continuous);
        let id = world.add_body(body)?;
        world.add_collider(
            id,
            Collider::new(
                Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0)?),
                Transform::identity(),
            ),
        )?;
    }
    Ok(world)
}

fn collision_world(body_count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let pair_count = (body_count / 16).min(128);
    for index in 0..body_count {
        let (position, velocity) = if index < pair_count * 2 {
            let pair = index / 2;
            let offset = if index % 2 == 0 { 0.0 } else { 0.9 };
            (Vec3::new(pair as f64 * 3.0 + offset, 0.0, 0.0), Vec3::ZERO)
        } else {
            let index = index - pair_count * 2;
            let x = (index % 200) as f64 * 2.0;
            let z = (index / 200) as f64 * 2.0;
            (Vec3::new(x, 10.0, z), Vec3::ZERO)
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

fn sample(
    run: impl Fn() -> Result<Duration, Box<dyn Error>>,
) -> Result<(Duration, Duration, Duration), Box<dyn Error>> {
    for _ in 0..WARMUP_RUNS {
        black_box(run()?);
    }
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

fn report(
    label: &str,
    (min, median, max): (Duration, Duration, Duration),
    sequential: (Duration, Duration, Duration),
) {
    println!(
        "{label:24} {:8.2}/{:8.2}/{:8.2} ms min/median/max / {STEPS} steps  speedup={:.2}x",
        min.as_secs_f64() * 1_000.0,
        median.as_secs_f64() * 1_000.0,
        max.as_secs_f64() * 1_000.0,
        sequential.1.as_secs_f64() / median.as_secs_f64()
    );
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1_000.0
}
