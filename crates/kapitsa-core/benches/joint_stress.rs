use std::{
    error::Error,
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
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

const SIZES: [usize; 5] = [144, 576, 1_440, 7_200, 14_400];
const WARMUPS: usize = 1;
const SAMPLES: usize = 3;
const DT: f64 = 1.0 / 120.0;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Joint stress scenes: warmups={WARMUPS}, samples={SAMPLES}, workers=1,4");
    for size in SIZES {
        report("one body / colliders", size, colliders_on_one_body)?;
        report("fixed-joint chain", size, fixed_chain)?;
        report("branching fixed-joint assembly", size, branching_assembly)?;
        report("independent two-body assemblies", size, independent_pairs)?;
    }
    Ok(())
}

fn report(
    name: &str,
    size: usize,
    build: fn(usize) -> Result<PhysicsWorld, Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    for workers in [1, 4] {
        let mut samples = Vec::with_capacity(SAMPLES);
        for run in 0..WARMUPS + SAMPLES {
            let mut world = build(size)?;
            let started = Instant::now();
            world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;
            let elapsed = started.elapsed();
            if run >= WARMUPS {
                samples.push((elapsed, world.collision_step_metrics()));
            }
        }
        samples.sort_unstable_by_key(|sample| sample.0);
        let min = &samples[0];
        let median = &samples[SAMPLES / 2];
        let max = &samples[SAMPLES - 1];
        println!(
            "{name:34} size={size:>6} workers={workers}: step_ms={:.3}/{:.3}/{:.3} solver_prepare_ms={:.3} solver_iterations_ms={:.3} narrow_ms={:.3} islands={} joints={} retained_scratch_kib={}",
            milliseconds(min.0),
            milliseconds(median.0),
            milliseconds(max.0),
            milliseconds(median.1.solver_prepare),
            milliseconds(median.1.solver_iterations),
            milliseconds(median.1.narrow_phase),
            median.1.island_count,
            expected_joints(name, size),
            median.1.retained_scratch_bytes / 1024,
        );
    }
    Ok(())
}

fn expected_joints(name: &str, size: usize) -> usize {
    match name {
        "fixed-joint chain" | "branching fixed-joint assembly" => size.saturating_sub(1),
        "independent two-body assemblies" => size / 2,
        _ => 0,
    }
}

fn colliders_on_one_body(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(dynamic_body(Vec3::ZERO)?)?;
    for _ in 0..count {
        world.add_collider(
            body,
            Collider::new(Shape::Sphere(Sphere::new(0.25)?), Transform::identity()),
        )?;
    }
    Ok(world)
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
        let leaf = world.add_body(dynamic_body(Vec3::ZERO)?)?;
        world.add_joint(hub, leaf, leaf_joint)?;
    }
    Ok(world)
}

fn independent_pairs(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let common = fixed_joint_frames();
    for pair in 0..count / 2 {
        let x = pair as f64 * 3.0;
        let body_a = world.add_body(dynamic_body(Vec3::X * x)?)?;
        let body_b = world.add_body(dynamic_body(Vec3::X * (x + 1.0))?)?;
        world.add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))?;
    }
    Ok(world)
}

fn dynamic_body(position: Vec3) -> Result<RigidBody, Box<dyn Error>> {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    Ok(RigidBody::new_dynamic(
        Transform::new(position, Quat::IDENTITY)?,
        mass,
    )?)
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
