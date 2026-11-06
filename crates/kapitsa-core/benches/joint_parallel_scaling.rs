use std::{error::Error, time::Instant};

use kapitsa_core::{
    execution::SequentialExecutor,
    joint::{FixedJoint, Joint, JointCommon, JointFrame},
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

#[path = "support/rayon_executor.rs"]
mod rayon_executor;

use rayon_executor::RayonExecutor;

const BODY_COUNTS: [usize; 4] = [100, 200, 500, 1_000];
const WORKER_COUNTS: [usize; 4] = [1, 2, 4, 8];
const WARMUPS: usize = 3;
const SAMPLES: usize = 20;
const DT: f64 = 1.0 / 120.0;
const PCG_ITERATIONS: u32 = 5;

#[derive(Clone, Copy)]
enum SolverMode {
    Scalar,
    Pcg,
}

impl SolverMode {
    fn name(self) -> &'static str {
        match self {
            Self::Scalar => "scalar",
            Self::Pcg => "pcg_ic0",
        }
    }
}

enum Executor {
    Sequential,
    Rayon(RayonExecutor),
}

impl Executor {
    fn run(&self, world: &mut PhysicsWorld, workers: usize) -> Result<(), Box<dyn Error>> {
        match self {
            Self::Sequential => world.step_with_executor(DT, &SequentialExecutor, 1)?,
            Self::Rayon(executor) => world.step_with_executor(DT, executor, workers)?,
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct Sample {
    elapsed_ns: u128,
    solver_prepare_ns: u128,
    row_prepare_ns: u128,
    velocity_sweeps: u64,
    position_sweeps: u64,
    velocity_pcg_iterations: u64,
    position_pcg_iterations: u64,
    velocity_pcg_caps: u64,
    position_pcg_caps: u64,
    velocity_residual_linear: f64,
    velocity_residual_angular: f64,
    position_residual_linear: f64,
    position_residual_angular: f64,
    measured_jv_linear: f64,
    measured_jv_angular: f64,
    colors: u32,
    color_batches: u64,
}

fn main() -> Result<(), Box<dyn Error>> {
    println!(
        "joint scaling: body_counts={BODY_COUNTS:?} rayon_workers={WORKER_COUNTS:?} warmups={WARMUPS} samples={SAMPLES} pcg_caps={PCG_ITERATIONS}/{PCG_ITERATIONS}"
    );
    for body_count in BODY_COUNTS {
        for mode in [SolverMode::Scalar, SolverMode::Pcg] {
            run_case(body_count, mode, 1, Executor::Sequential, "no_pool")?;
            for workers in WORKER_COUNTS {
                run_case(
                    body_count,
                    mode,
                    workers,
                    Executor::Rayon(RayonExecutor::new(workers)?),
                    "rayon",
                )?;
            }
        }
    }
    Ok(())
}

fn run_case(
    body_count: usize,
    mode: SolverMode,
    workers: usize,
    executor: Executor,
    executor_name: &str,
) -> Result<(), Box<dyn Error>> {
    let (mut world, terminal_body) = fixed_chain(body_count)?;
    if matches!(mode, SolverMode::Pcg) {
        world.set_joint_global_velocity_pcg_for_bench(PCG_ITERATIONS, true);
        world.set_joint_global_position_pcg_for_bench(PCG_ITERATIONS, true);
    }

    for _ in 0..WARMUPS {
        perturb_terminal_body(&mut world, terminal_body)?;
        executor.run(&mut world, workers)?;
    }

    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        perturb_terminal_body(&mut world, terminal_body)?;
        let started = Instant::now();
        executor.run(&mut world, workers)?;
        let elapsed_ns = started.elapsed().as_nanos();
        let metrics = world.physics_step_metrics();
        let joints = metrics.joints;
        let (measured_jv_linear, measured_jv_angular) = joint_velocity_residuals(&world)?;
        samples.push(Sample {
            elapsed_ns,
            solver_prepare_ns: metrics.collision.solver_prepare.as_nanos(),
            row_prepare_ns: joints.joint_row_preparation_time.as_nanos(),
            velocity_sweeps: joints.velocity_sweeps,
            position_sweeps: joints.position_sweeps,
            velocity_pcg_iterations: joints.velocity_pcg_iterations,
            position_pcg_iterations: joints.position_pcg_iterations,
            velocity_pcg_caps: joints.velocity_pcg_iteration_limit_calls,
            position_pcg_caps: joints.position_pcg_iteration_limit_calls,
            velocity_residual_linear: joints.velocity_pcg_linear_residual_mps,
            velocity_residual_angular: joints.velocity_pcg_angular_residual_radps,
            position_residual_linear: joints.position_pcg_linear_residual_m,
            position_residual_angular: joints.position_pcg_angular_residual_rad,
            measured_jv_linear,
            measured_jv_angular,
            colors: joints.max_colors_per_island,
            color_batches: joints.color_batches_processed,
        });
    }

    let final_geometry = joint_geometry_errors(&world)?;
    validate_bodies(&world)?;
    samples.sort_unstable_by_key(|sample| sample.elapsed_ns);
    let median = samples[SAMPLES / 2];
    let p95 = samples[percentile_index(SAMPLES, 95)];
    let max_velocity_residual = samples.iter().fold((0.0_f64, 0.0_f64), |max, sample| {
        (
            max.0.max(sample.velocity_residual_linear),
            max.1.max(sample.velocity_residual_angular),
        )
    });
    let max_position_residual = samples.iter().fold((0.0_f64, 0.0_f64), |max, sample| {
        (
            max.0.max(sample.position_residual_linear),
            max.1.max(sample.position_residual_angular),
        )
    });

    println!(
        "fixed_chain bodies={body_count:>4} joints={:>4} solver={} executor={} workers={workers}: step_p50/p95_ms={:.3}/{:.3} prepare_p50_ms={:.3} row_prepare_p50_ms={:.3} sweeps_v/p={}/{} pcg_iters_v/p={}/{} pcg_caps_v/p={}/{} pcg_residual_max_v_mps/radps={:.3e}/{:.3e} pcg_residual_max_p_m/rad={:.3e}/{:.3e} measured_Jv_max_mps/radps={:.3e}/{:.3e} colors={} color_batches={} final_geometry_anchor_m/frame_rad={:.3e}/{:.3e}",
        body_count.saturating_sub(1),
        mode.name(),
        executor_name,
        nanos_to_ms(median.elapsed_ns),
        nanos_to_ms(p95.elapsed_ns),
        nanos_to_ms(median.solver_prepare_ns),
        nanos_to_ms(median.row_prepare_ns),
        median.velocity_sweeps,
        median.position_sweeps,
        median.velocity_pcg_iterations,
        median.position_pcg_iterations,
        median.velocity_pcg_caps,
        median.position_pcg_caps,
        max_velocity_residual.0,
        max_velocity_residual.1,
        max_position_residual.0,
        max_position_residual.1,
        samples
            .iter()
            .fold(0.0_f64, |max, sample| max.max(sample.measured_jv_linear)),
        samples
            .iter()
            .fold(0.0_f64, |max, sample| max.max(sample.measured_jv_angular)),
        median.colors,
        median.color_batches,
        final_geometry.0,
        final_geometry.1,
    );
    Ok(())
}

fn fixed_chain(
    body_count: usize,
) -> Result<(PhysicsWorld, kapitsa_core::world::BodyId), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let mut bodies = Vec::with_capacity(body_count);
    for index in 0..body_count {
        let transform = Transform::new(Vec3::X * index as f64, Quat::IDENTITY)?;
        let mut body = RigidBody::new_dynamic(transform, mass)?;
        if index + 1 == body_count {
            body.set_linear_velocity(Vec3::Y * 0.1)?;
            body.set_angular_velocity(Vec3::Z * 0.01)?;
        }
        bodies.push(world.add_body(body)?);
    }
    let common = JointCommon::new(
        JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
        JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
    );
    for pair in bodies.windows(2) {
        world.add_joint(pair[0], pair[1], Joint::Fixed(FixedJoint::new(common)))?;
    }
    let terminal_body = *bodies.last().ok_or("chain has no bodies")?;
    Ok((world, terminal_body))
}

fn perturb_terminal_body(
    world: &mut PhysicsWorld,
    terminal_body: kapitsa_core::world::BodyId,
) -> Result<(), Box<dyn Error>> {
    let body = world
        .body_mut(terminal_body)
        .ok_or("terminal body missing")?;
    body.set_linear_velocity(Vec3::Y * 0.1)?;
    body.set_angular_velocity(Vec3::Z * 0.01)?;
    Ok(())
}

fn joint_geometry_errors(world: &PhysicsWorld) -> Result<(f64, f64), Box<dyn Error>> {
    let mut max_anchor = 0.0_f64;
    let mut max_frame = 0.0_f64;
    for (_, body_a_id, body_b_id, joint) in world.joints() {
        let body_a = world.body(body_a_id).ok_or("joint body A missing")?;
        let body_b = world.body(body_b_id).ok_or("joint body B missing")?;
        let transform_a = body_a.state().transform();
        let transform_b = body_b.state().transform();
        let common = joint.common();
        let anchor_a = transform_a.transform_point(common.frame_a.local_anchor);
        let anchor_b = transform_b.transform_point(common.frame_b.local_anchor);
        max_anchor = max_anchor.max((anchor_b - anchor_a).length());
        let frame_a = transform_a.rotation() * common.frame_a.local_rotation;
        let frame_b = transform_b.rotation() * common.frame_b.local_rotation;
        let relative = frame_a.conjugate() * frame_b;
        let vector_length = Vec3::new(relative.x, relative.y, relative.z).length();
        max_frame = max_frame.max(2.0 * vector_length.atan2(relative.w.abs()));
    }
    Ok((max_anchor, max_frame))
}

fn joint_velocity_residuals(world: &PhysicsWorld) -> Result<(f64, f64), Box<dyn Error>> {
    let mut max_linear = 0.0_f64;
    let mut max_angular = 0.0_f64;
    for (_, body_a_id, body_b_id, joint) in world.joints() {
        let body_a = world.body(body_a_id).ok_or("joint body A missing")?;
        let body_b = world.body(body_b_id).ok_or("joint body B missing")?;
        let state_a = body_a.state();
        let state_b = body_b.state();
        let transform_a = state_a.transform();
        let transform_b = state_b.transform();
        let common = joint.common();
        let anchor_a = transform_a.transform_point(common.frame_a.local_anchor);
        let anchor_b = transform_b.transform_point(common.frame_b.local_anchor);
        let velocity_a = state_a.linear_velocity()
            + state_a
                .angular_velocity()
                .cross(anchor_a - transform_a.translation());
        let velocity_b = state_b.linear_velocity()
            + state_b
                .angular_velocity()
                .cross(anchor_b - transform_b.translation());
        max_linear = max_linear.max((velocity_b - velocity_a).length());
        max_angular =
            max_angular.max((state_b.angular_velocity() - state_a.angular_velocity()).length());
    }
    Ok((max_linear, max_angular))
}

fn validate_bodies(world: &PhysicsWorld) -> Result<(), Box<dyn Error>> {
    if world.bodies().any(|(_, body)| {
        let state = body.state();
        !state.transform().translation().is_finite()
            || !state.transform().rotation().is_normalized()
            || !state.linear_velocity().is_finite()
            || !state.angular_velocity().is_finite()
    }) {
        return Err("benchmark produced an invalid body state".into());
    }
    Ok(())
}

fn percentile_index(count: usize, percentile: usize) -> usize {
    count
        .saturating_mul(percentile)
        .div_ceil(100)
        .saturating_sub(1)
        .min(count.saturating_sub(1))
}

fn nanos_to_ms(nanoseconds: u128) -> f64 {
    nanoseconds as f64 / 1_000_000.0
}
