use std::{
    error::Error,
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    gravity::CentralGravity,
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
const WORKERS: [usize; 4] = [1, 2, 4, 8];

fn main() -> Result<(), Box<dyn Error>> {
    println!("Joint stress scenes: warmups={WARMUPS}, samples={SAMPLES}, workers={WORKERS:?}");
    report("fleet of 100 two-stage rockets", 100, rocket_fleet)?;
    for size in SIZES {
        report("one body / colliders", size, colliders_on_one_body)?;
        report("fixed-joint chain", size, fixed_chain)?;
        report("branching fixed-joint assembly", size, branching_assembly)?;
        report("independent two-body assemblies", size, independent_pairs)?;
        report("independent debris with colliders", size, debris_cloud)?;
        report("dense vertical contact stack", size, dense_contact_stack)?;
        report_first_step("mass joint break", size, mass_break)?;
        report_mass_separation(size)?;
    }
    Ok(())
}

fn report(
    name: &str,
    size: usize,
    build: fn(usize) -> Result<PhysicsWorld, Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    for workers in WORKERS {
        let mut world = build(size)?;
        let initial_joint_errors = max_joint_frame_errors(&world)?;
        for _ in 0..WARMUPS {
            world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;
        }
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;
            let elapsed = started.elapsed();
            samples.push((elapsed, world.collision_step_metrics()));
        }
        let joint_errors = max_joint_frame_errors(&world)?;
        samples.sort_unstable_by_key(|sample| sample.0);
        let min = &samples[0];
        let median = &samples[SAMPLES / 2];
        let max = &samples[SAMPLES - 1];
        println!(
            "{name:34} size={size:>6} workers={workers}: step_ms={:.3}/{:.3}/{:.3} solver_prepare_ms={:.3} solver_iterations_ms={:.3} narrow_ms={:.3} candidates={} contacts={} islands={} joints={} initial_joint_error_m/rad={:.6e}/{:.6e} final_joint_error_m/rad={:.6e}/{:.6e} retained_scratch_kib={} peak_scratch_kib={} logical_growth_bytes={}",
            milliseconds(min.0),
            milliseconds(median.0),
            milliseconds(max.0),
            milliseconds(median.1.solver_prepare),
            milliseconds(median.1.solver_iterations),
            milliseconds(median.1.narrow_phase),
            median.1.candidate_count,
            median.1.contact_count,
            median.1.island_count,
            expected_joints(name, size),
            initial_joint_errors.0,
            initial_joint_errors.1,
            joint_errors.0,
            joint_errors.1,
            median.1.retained_scratch_bytes / 1024,
            median.1.peak_scratch_bytes / 1024,
            median.1.logical_capacity_growth_bytes,
        );
    }
    Ok(())
}

fn expected_joints(name: &str, size: usize) -> usize {
    match name {
        "fixed-joint chain" | "branching fixed-joint assembly" => size.saturating_sub(1),
        "independent two-body assemblies" => size / 2,
        "fleet of 100 two-stage rockets" => size,
        _ => 0,
    }
}

fn report_first_step(
    name: &str,
    size: usize,
    build: fn(usize) -> Result<PhysicsWorld, Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    for workers in WORKERS {
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let mut world = build(size)?;
            // Breaking destroys the measured joints, so this is intentionally the first step.
            let started = Instant::now();
            world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;
            let elapsed = started.elapsed();
            samples.push((
                elapsed,
                world.collision_step_metrics(),
                world.joint_events().len(),
            ));
        }
        samples.sort_unstable_by_key(|sample| sample.0);
        let min = &samples[0];
        let median = &samples[SAMPLES / 2];
        let max = &samples[SAMPLES - 1];
        println!(
            "{name:34} elements={size:>6} workers={workers}: first_step_ms={:.3}/{:.3}/{:.3} islands={} break_events={} retained_scratch_kib={} peak_scratch_kib={} logical_growth_bytes={}",
            milliseconds(min.0),
            milliseconds(median.0),
            milliseconds(max.0),
            median.1.island_count,
            median.2,
            median.1.retained_scratch_bytes / 1024,
            median.1.peak_scratch_bytes / 1024,
            median.1.logical_capacity_growth_bytes,
        );
    }
    Ok(())
}

fn report_mass_separation(count: usize) -> Result<(), Box<dyn Error>> {
    for workers in WORKERS {
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let (mut world, joints) = independent_pair_world(count)?;
            world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;

            let remove_started = Instant::now();
            for joint in joints {
                world.remove_joint(joint);
            }
            let remove_time = remove_started.elapsed();

            let step_started = Instant::now();
            world.step_with_executor(DT, &ScopedThreadExecutor, workers)?;
            let step_time = step_started.elapsed();
            samples.push((remove_time, step_time, world.collision_step_metrics()));
        }
        let mut remove_times: Vec<_> = samples.iter().map(|sample| sample.0).collect();
        let mut step_times: Vec<_> = samples.iter().map(|sample| sample.1).collect();
        remove_times.sort_unstable();
        step_times.sort_unstable();
        samples.sort_unstable_by_key(|sample| sample.0 + sample.1);
        let median = &samples[SAMPLES / 2];
        println!(
            "mass stage separation               elements={count:>6} workers={workers}: remove_ms={:.3}/{:.3}/{:.3} detached_step_ms={:.3}/{:.3}/{:.3} retained_scratch_kib={} peak_scratch_kib={} logical_growth_bytes={}",
            milliseconds(remove_times[0]),
            milliseconds(remove_times[SAMPLES / 2]),
            milliseconds(remove_times[SAMPLES - 1]),
            milliseconds(step_times[0]),
            milliseconds(step_times[SAMPLES / 2]),
            milliseconds(step_times[SAMPLES - 1]),
            median.2.retained_scratch_bytes / 1024,
            median.2.peak_scratch_bytes / 1024,
            median.2.logical_capacity_growth_bytes,
        );
    }
    Ok(())
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
        bodies.push(world.add_body(dynamic_body(Vec3::X * (index as f64 * 1.01))?)?);
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
        let leaf_rotation = Quat::from_axis_angle(Vec3::Z, 0.001).ok_or("valid leaf rotation")?;
        let leaf_transform = Transform::new(Vec3::X * 0.01, leaf_rotation)?;
        let leaf = world.add_body(dynamic_body_at(leaf_transform)?)?;
        world.add_joint(hub, leaf, leaf_joint)?;
    }
    Ok(world)
}

fn independent_pairs(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    independent_pair_world(count).map(|(world, _)| world)
}

fn independent_pair_world(
    count: usize,
) -> Result<(PhysicsWorld, Vec<kapitsa_core::world::JointId>), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let common = fixed_joint_frames();
    let mut joints = Vec::with_capacity(count / 2);
    for pair in 0..count / 2 {
        let x = pair as f64 * 3.0;
        let body_a = world.add_body(dynamic_body(Vec3::X * x)?)?;
        let rotation = Quat::from_axis_angle(Vec3::Z, 0.001).ok_or("valid pair rotation")?;
        let transform = Transform::new(Vec3::X * (x + 1.02), rotation)?;
        let body_b = world.add_body(dynamic_body_at(transform)?)?;
        joints.push(world.add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))?);
    }
    Ok((world, joints))
}

fn rocket_fleet(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let common = fixed_joint_frames();
    for rocket in 0..count {
        let x = rocket as f64 * 4.0;
        let body_a = world.add_body(dynamic_body(Vec3::X * x)?)?;
        let body_b = world.add_body(dynamic_body(Vec3::X * (x + 1.0))?)?;
        world.add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))?;
    }
    Ok(world)
}

fn debris_cloud(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    for index in 0..count {
        let body = world.add_body(dynamic_body(Vec3::X * (index as f64 * 2.0))?)?;
        world.add_collider(
            body,
            Collider::new(Shape::Sphere(Sphere::new(0.25)?), Transform::identity()),
        )?;
    }
    Ok(world)
}

fn dense_contact_stack(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    const PLANET_RADIUS: f64 = 100.0;

    let mut world = PhysicsWorld::new();
    world.set_central_gravity(Some(CentralGravity::new(
        Vec3::ZERO,
        9.806_65 * PLANET_RADIUS * PLANET_RADIUS,
    )?));
    let ground = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        ground,
        Collider::new(
            Shape::Sphere(Sphere::new(PLANET_RADIUS)?),
            Transform::identity(),
        ),
    )?;

    for index in 0..count {
        let altitude = PLANET_RADIUS + 0.45 + index as f64 * 0.9;
        let body = world.add_body(dynamic_body(Vec3::Y * altitude)?)?;
        world.add_collider(
            body,
            Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
        )?;
    }
    Ok(world)
}

fn mass_break(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mut common = fixed_joint_frames();
    common.break_force = Some(100.0);
    for pair in 0..count / 2 {
        let x = pair as f64 * 3.0;
        let body_a = world.add_body(dynamic_body(Vec3::X * x)?)?;
        let mut body_b = dynamic_body(Vec3::X * x)?;
        body_b.set_linear_velocity(Vec3::X * 10.0)?;
        let body_b = world.add_body(body_b)?;
        world.add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))?;
    }
    Ok(world)
}

fn dynamic_body(position: Vec3) -> Result<RigidBody, Box<dyn Error>> {
    dynamic_body_at(Transform::new(position, Quat::IDENTITY)?)
}

fn dynamic_body_at(transform: Transform) -> Result<RigidBody, Box<dyn Error>> {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    Ok(RigidBody::new_dynamic(transform, mass)?)
}

fn max_joint_frame_errors(world: &PhysicsWorld) -> Result<(f64, f64), Box<dyn Error>> {
    let mut max_anchor_error: f64 = 0.0;
    let mut max_frame_error: f64 = 0.0;
    for (_, body_a_id, body_b_id, joint) in world.joints() {
        let body_a = world.body(body_a_id).ok_or("joint body A is missing")?;
        let body_b = world.body(body_b_id).ok_or("joint body B is missing")?;
        let transform_a = body_a.state().transform();
        let transform_b = body_b.state().transform();
        let common = joint.common();
        let anchor_a = transform_a.transform_point(common.frame_a.local_anchor);
        let anchor_b = transform_b.transform_point(common.frame_b.local_anchor);
        max_anchor_error = max_anchor_error.max((anchor_b - anchor_a).length());

        let frame_a = transform_a.rotation() * common.frame_a.local_rotation;
        let frame_b = transform_b.rotation() * common.frame_b.local_rotation;
        let relative = frame_a.conjugate() * frame_b;
        let vector_length = Vec3::new(relative.x, relative.y, relative.z).length();
        max_frame_error = max_frame_error.max(2.0 * vector_length.atan2(relative.w.abs()));
    }
    Ok((max_anchor_error, max_frame_error))
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
