use super::{FixedJoint, Joint, JointCommon, JointFrame};
#[cfg(feature = "bench-internals")]
use crate::execution::{ExecutionError, StepExecutor};
use crate::{
    collision::CollisionConfig, mass::MassProperties, rigid_body::RigidBody, transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};
#[cfg(feature = "bench-internals")]
use std::time::Instant;

const DT: f64 = 1.0 / 120.0;

#[cfg(feature = "bench-internals")]
struct ScopedPcgTestExecutor;

#[cfg(feature = "bench-internals")]
impl StepExecutor for ScopedPcgTestExecutor {
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
        std::thread::scope(|scope| {
            for (index, chunk) in values.chunks_mut(chunk_size).enumerate() {
                let operation = &operation;
                scope.spawn(move || operation(index, chunk));
            }
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
        let total = chunk_sizes.iter().try_fold(0_usize, |total, size| {
            (*size > 0).then(|| total.checked_add(*size)).flatten()
        });
        if total != Some(values.len()) {
            return Err(ExecutionError::InvalidChunkLayout);
        }
        std::thread::scope(|scope| {
            let mut remaining = values;
            let mut start = 0;
            for &chunk_size in chunk_sizes {
                let (chunk, tail) = remaining.split_at_mut(chunk_size);
                let operation = &operation;
                scope.spawn(move || operation(start, chunk));
                start += chunk_size;
                remaining = tail;
            }
        });
        Ok(())
    }
}

#[test]
fn fixed_hub_with_144_bodies_converges_from_pose_perturbations() {
    let mut world = fixed_hub_world(144);
    let initial = max_joint_errors(&world);
    let mut peak_energy = 0.0_f64;
    for _ in 0..20 {
        world.step(DT).unwrap();
        peak_energy = peak_energy.max(kinetic_energy(&world));
        assert_finite_body_states(&world);
    }
    let final_errors = max_joint_errors(&world);

    assert!(initial.0 > 0.009 && initial.1 > 0.0009);
    assert!(final_errors.0 < 0.002, "anchor error: {final_errors:?}");
    assert!(final_errors.1 < 0.0002, "frame error: {final_errors:?}");
    assert!(peak_energy < 100.0, "kinetic energy grew to {peak_energy}");
}

#[test]
fn fixed_hub_with_14400_bodies_survives_the_original_stress_step() {
    let mut world = fixed_hub_world(14_400);
    world.step(DT).unwrap();
    assert_finite_body_states(&world);
    let errors = max_joint_errors(&world);
    assert!(errors.0 < 0.002, "anchor error: {errors:?}");
    assert!(errors.1 < 0.0002, "frame error: {errors:?}");
    assert!(kinetic_energy(&world) < 100.0);
}

fn fixed_hub_world(count: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let hub = world
        .add_body(dynamic_body(Vec3::ZERO, Quat::IDENTITY))
        .unwrap();
    let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    let leaf_rotation = Quat::from_axis_angle(Vec3::Z, 0.001).unwrap();

    for _ in 1..count {
        let leaf = world
            .add_body(dynamic_body(Vec3::X * 0.01, leaf_rotation))
            .unwrap();
        world
            .add_joint(hub, leaf, Joint::Fixed(FixedJoint::new(common)))
            .unwrap();
    }
    world
}

#[test]
fn fixed_chain_and_branching_tree_converge_over_successive_steps() {
    let mut chain = PhysicsWorld::new();
    chain
        .set_collision_config(CollisionConfig {
            position_iterations: 12,
            ..CollisionConfig::default()
        })
        .unwrap();
    let chain_bodies: Vec<_> = (0..16)
        .map(|index| {
            let angle = if index % 2 == 0 { 0.001 } else { -0.001 };
            let rotation = Quat::from_axis_angle(Vec3::Z, angle).unwrap();
            chain
                .add_body(dynamic_body(Vec3::X * (index as f64 * 1.01), rotation))
                .unwrap()
        })
        .collect();
    let chain_joint = Joint::Fixed(FixedJoint::new(JointCommon::new(
        JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
        JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
    )));
    for pair in chain_bodies.windows(2) {
        chain.add_joint(pair[0], pair[1], chain_joint).unwrap();
    }

    let mut tree = PhysicsWorld::new();
    let tree_bodies: Vec<_> = (0..63)
        .map(|index| {
            let position = Vec3::new((index % 3) as f64 * 0.005, 0.0, 0.0);
            let angle = (index % 2) as f64 * 0.001;
            let rotation = Quat::from_axis_angle(Vec3::Z, angle).unwrap();
            tree.add_body(dynamic_body(position, rotation)).unwrap()
        })
        .collect();
    let tree_joint = Joint::Fixed(FixedJoint::new(JointCommon::new(
        JointFrame::IDENTITY,
        JointFrame::IDENTITY,
    )));
    for child in 1..tree_bodies.len() {
        tree.add_joint(tree_bodies[(child - 1) / 2], tree_bodies[child], tree_joint)
            .unwrap();
    }

    for _ in 0..20 {
        chain.step(DT).unwrap();
        tree.step(DT).unwrap();
        assert_finite_body_states(&chain);
        assert_finite_body_states(&tree);
    }
    let chain_errors = max_joint_errors(&chain);
    let tree_errors = max_joint_errors(&tree);
    assert!(chain_errors.0 < 0.005, "chain errors: {chain_errors:?}");
    assert!(chain_errors.1 < 0.005, "chain errors: {chain_errors:?}");
    assert!(tree_errors.0 < 0.002, "tree errors: {tree_errors:?}");
    assert!(tree_errors.1 < 0.002, "tree errors: {tree_errors:?}");
    assert!(kinetic_energy(&chain) < 100.0);
    assert!(kinetic_energy(&tree) < 100.0);
}

#[test]
fn fixed_joint_stability_is_symmetric_for_body_order_and_static_connections() {
    let mut forward = two_body_world(false, false);
    let mut reversed = two_body_world(true, false);
    let mut dynamic_static = two_body_world(false, true);

    for _ in 0..20 {
        forward.step(DT).unwrap();
        reversed.step(DT).unwrap();
        dynamic_static.step(DT).unwrap();
        assert_finite_body_states(&forward);
        assert_finite_body_states(&reversed);
        assert_finite_body_states(&dynamic_static);
    }

    for (name, world) in [
        ("dynamic-dynamic", &forward),
        ("reversed", &reversed),
        ("dynamic-static", &dynamic_static),
    ] {
        let errors = max_joint_errors(world);
        assert!(errors.0 < 0.002, "{name} anchor errors: {errors:?}");
        assert!(errors.1 < 0.0002, "{name} frame errors: {errors:?}");
        assert!(kinetic_energy(world) < 10.0, "{name} kinetic energy");
    }
}

#[test]
fn fixed_joint_handles_mass_ratios_and_off_center_thrust() {
    for ratio in [1.0, 100.0, 10_000.0] {
        let mut world = PhysicsWorld::new();
        let inertia_a = Mat3::from_diagonal(Vec3::new(0.4, 0.6, 0.9));
        let inertia_b = Mat3::from_diagonal(Vec3::new(0.2, 0.35, 0.5) * ratio);
        let body_a = world
            .add_body(dynamic_body_with_properties(
                Vec3::ZERO,
                MassProperties::new(1.0, Vec3::ZERO, inertia_a).unwrap(),
            ))
            .unwrap();
        let body_b = world
            .add_body(dynamic_body_with_properties(
                Vec3::X,
                MassProperties::new(ratio, Vec3::ZERO, inertia_b).unwrap(),
            ))
            .unwrap();
        let common = JointCommon::new(
            JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
            JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
        );
        world
            .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
            .unwrap();

        let force = Vec3::X * 100.0;
        let local_point = Vec3::Y * 0.25;
        let mut peak_energy = 0.0_f64;
        for _ in 0..20 {
            let point = world
                .body(body_a)
                .unwrap()
                .state()
                .transform()
                .transform_point(local_point);
            world
                .body_mut(body_a)
                .unwrap()
                .apply_force_at_point(force, point)
                .unwrap();
            world.step(DT).unwrap();
            peak_energy = peak_energy.max(kinetic_energy(&world));
            assert_finite_body_states(&world);
        }

        let errors = max_joint_errors(&world);
        assert!(errors.0 < 0.002, "mass ratio 1:{ratio}, anchors {errors:?}");
        assert!(errors.1 < 0.0002, "mass ratio 1:{ratio}, frames {errors:?}");
        assert!(
            peak_energy < 2_000.0,
            "mass ratio 1:{ratio}, energy {peak_energy}"
        );

        let momentum = [body_a, body_b]
            .into_iter()
            .map(|id| {
                let body = world.body(id).unwrap();
                body.physics().mass().unwrap().mass() * body.state().linear_velocity()
            })
            .fold(Vec3::ZERO, |sum, value| sum + value);
        let expected_momentum = force * (DT * 20.0);
        assert!(
            (momentum - expected_momentum).length() < 1.0e-6,
            "mass ratio 1:{ratio}, momentum {momentum:?} != {expected_momentum:?}"
        );
    }
}

#[test]
fn fixed_joint_chain_and_branch_sizes_report_residual_by_step() {
    fn perturbed_body(index: usize, position: Vec3) -> RigidBody {
        let angle = if index.is_multiple_of(2) {
            0.001
        } else {
            -0.001
        };
        let rotation = Quat::from_axis_angle(Vec3::new(0.2, 0.7, -0.3), angle).unwrap();
        let mass = 0.8 + (index % 7) as f64 * 0.17;
        let center = Vec3::new(
            (index % 3) as f64 * 0.002,
            -((index % 5) as f64) * 0.001,
            (index % 2) as f64 * 0.0015,
        );
        let inertia_scale = 0.7 + (index % 5) as f64 * 0.13;
        let inertia = Mat3::from_diagonal(Vec3::new(
            inertia_scale * 0.6,
            inertia_scale * 0.8,
            inertia_scale,
        ));
        RigidBody::new_dynamic(
            Transform::new(position, rotation).unwrap(),
            MassProperties::new(mass, center, inertia).unwrap(),
        )
        .unwrap()
    }

    fn report(mut world: PhysicsWorld, name: &str) {
        let initial = max_joint_errors(&world);
        let mut by_step = Vec::with_capacity(11);
        by_step.push(initial);
        for _ in 0..10 {
            world.step(DT).unwrap();
            assert_finite_body_states(&world);
            by_step.push(max_joint_errors(&world));
        }
        println!(
            "{name} joints={} initial={initial:?} after1={:?} after2={:?} after10={:?}",
            world.joint_count(),
            by_step[1],
            by_step[2],
            by_step[10],
        );
        let final_error = by_step[10];
        assert!(
            final_error.0 < initial.0,
            "{name} anchor error did not fall"
        );
        assert!(final_error.1 < initial.1, "{name} frame error did not fall");
    }

    for count in [2, 10, 50, 100] {
        let mut world = PhysicsWorld::new();
        let bodies = (0..count)
            .map(|index| {
                let position = Vec3::X * index as f64
                    + Vec3::new(0.0, (index % 3) as f64 * 0.001, (index % 2) as f64 * 0.001);
                world.add_body(perturbed_body(index, position)).unwrap()
            })
            .collect::<Vec<_>>();
        let common = JointCommon::new(
            JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
            JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
        );
        let joint = Joint::Fixed(FixedJoint::new(common));
        for pair in bodies.windows(2) {
            world.add_joint(pair[0], pair[1], joint).unwrap();
        }
        report(world, &format!("chain_{count}"));
    }

    let mut world = PhysicsWorld::new();
    let mut positions = vec![Vec3::ZERO; 100];
    let mut bodies = Vec::with_capacity(positions.len());
    for index in 0..positions.len() {
        if index > 0 {
            let parent = (index - 1) / 2;
            let side = if index.is_multiple_of(2) { 1.0 } else { -1.0 };
            positions[index] =
                positions[parent] + Vec3::new(side * 0.003, 0.004, (index % 3) as f64 * 0.001);
        }
        bodies.push(
            world
                .add_body(perturbed_body(index, positions[index]))
                .unwrap(),
        );
    }
    let joint = Joint::Fixed(FixedJoint::new(JointCommon::new(
        JointFrame::IDENTITY,
        JointFrame::IDENTITY,
    )));
    for child in 1..bodies.len() {
        world
            .add_joint(bodies[(child - 1) / 2], bodies[child], joint)
            .unwrap();
    }
    report(world, "branch_100");
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Release comparison of post-integration Jv error and velocity projection work"]
fn diagnose_free_asymmetric_fixed_joint_velocity_projection_work() {
    for count in [2, 100] {
        for dt in [DT, DT * 0.5] {
            for (label, pcg_iterations) in [
                ("scalar", None),
                ("global_pcg", Some(64)),
                ("global_block_jacobi_pcg", Some(64)),
            ] {
                let mut world = PhysicsWorld::new();
                let mut bodies = Vec::with_capacity(count);
                let mut centers = Vec::with_capacity(count);
                for index in 0..count {
                    let scale = 0.7 + (index % 5) as f64 * 0.13;
                    let mass = MassProperties::new(
                        0.8 + (index % 7) as f64 * 0.17,
                        Vec3::new(
                            (index % 3) as f64 * 0.02,
                            -((index % 5) as f64) * 0.015,
                            (index % 2) as f64 * 0.01,
                        ),
                        Mat3::from_diagonal(Vec3::new(scale * 0.6, scale * 0.8, scale * 1.1)),
                    )
                    .unwrap();
                    let body = RigidBody::new_dynamic(
                        Transform::new(Vec3::X * index as f64, Quat::IDENTITY).unwrap(),
                        mass,
                    )
                    .unwrap();
                    centers.push(body.world_center_of_mass().unwrap().unwrap());
                    bodies.push(world.add_body(body).unwrap());
                }
                let common = Joint::Fixed(FixedJoint::new(JointCommon::new(
                    JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
                    JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
                )));
                for pair in bodies.windows(2) {
                    world.add_joint(pair[0], pair[1], common).unwrap();
                }
                let initial_angular_velocity = Vec3::new(0.15, -0.12, 0.2);
                let base_linear_velocity = Vec3::new(0.4, -0.2, 0.1);
                for (index, id) in bodies.iter().copied().enumerate() {
                    let velocity =
                        base_linear_velocity + initial_angular_velocity.cross(centers[index]);
                    let body = world.body_mut(id).unwrap();
                    body.set_linear_velocity(velocity).unwrap();
                    body.set_angular_velocity(initial_angular_velocity).unwrap();
                }
                if let Some(iterations) = pcg_iterations {
                    if label == "global_block_jacobi_pcg" {
                        world.set_joint_global_velocity_block_pcg_for_bench(iterations);
                    } else {
                        world.set_joint_global_velocity_pcg_for_bench(iterations, true);
                    }
                }

                let initial_energy = physical_kinetic_energy(&world);
                let steps = (1.5 / dt).round() as usize;
                let mut integration_delta = crate::joint::diagnostics::EnergyComponents::default();
                let mut velocity_delta = crate::joint::diagnostics::EnergyComponents::default();
                let mut position_delta = crate::joint::diagnostics::EnergyComponents::default();
                let mut impulse_work = 0.0;
                let mut quadratic_work = 0.0;
                let mut first_metrics = None;
                let mut last_metrics = None;
                for _ in 0..steps {
                    world.step(dt).unwrap();
                    let metrics = world.physics_step_metrics().joints;
                    integration_delta.add_assign(
                        metrics
                            .energy_after_integration
                            .delta(metrics.energy_before_integration),
                    );
                    velocity_delta.add_assign(
                        metrics
                            .energy_after_velocity_solver
                            .delta(metrics.energy_after_integration),
                    );
                    position_delta.add_assign(
                        metrics
                            .energy_after_position_solver
                            .delta(metrics.energy_after_velocity_solver),
                    );
                    impulse_work += metrics.velocity_projection_impulse_work_j;
                    quadratic_work += metrics.velocity_projection_quadratic_work_j;
                    first_metrics.get_or_insert(metrics);
                    last_metrics = Some(metrics);
                }
                let first = first_metrics.unwrap();
                let last = last_metrics.unwrap();
                let measured_velocity_delta = velocity_delta.translational_j
                    + velocity_delta.rotational_j
                    + velocity_delta.potential_j;
                let predicted_velocity_delta = impulse_work + quadratic_work;
                let final_energy = physical_kinetic_energy(&world);
                println!(
                    "free_fixed_graph={count} solver={label} dt={dt:.9e}s steps={steps} kinetic_drift={:+.9e}% post_integration_Jv={:.9e}m/s/{:.9e}rad/s post_velocity_solver_Jv={:.9e}m/s/{:.9e}rad/s post_position_solver_Jv={:.9e}m/s/{:.9e}rad/s energy_delta(integration/velocity/position)={:+.9e}/{:+.9e}/{:+.9e}J impulse_work={impulse_work:+.9e}J quadratic_work={quadratic_work:+.9e}J work_prediction={predicted_velocity_delta:+.9e}J closure_error={:+.9e}J",
                    (final_energy - initial_energy) / initial_energy * 100.0,
                    first.initial_velocity_linear_residual_mps,
                    first.initial_velocity_angular_residual_radps,
                    last.post_velocity_linear_residual_mps,
                    last.post_velocity_angular_residual_radps,
                    last.post_position_linear_residual_mps,
                    last.post_position_angular_residual_radps,
                    integration_delta.translational_j
                        + integration_delta.rotational_j
                        + integration_delta.potential_j,
                    measured_velocity_delta,
                    position_delta.translational_j
                        + position_delta.rotational_j
                        + position_delta.potential_j,
                    predicted_velocity_delta - measured_velocity_delta,
                );
                assert!((predicted_velocity_delta - measured_velocity_delta).abs() < 1.0e-8);
            }
        }
    }
}

#[cfg(feature = "bench-internals")]
fn physical_kinetic_energy(world: &PhysicsWorld) -> f64 {
    world
        .bodies()
        .map(|(_, body)| {
            let Some(mass) = body.physics().mass() else {
                return 0.0;
            };
            let state = body.state();
            let rotation = state.transform().rotation();
            let local_angular_velocity = rotation
                .conjugate()
                .rotate_vector(state.angular_velocity())
                .unwrap();
            0.5 * mass.mass() * state.linear_velocity().length_squared()
                + 0.5 * local_angular_velocity.dot(mass.inertia() * local_angular_velocity)
        })
        .sum()
}

#[cfg(feature = "bench-internals")]
#[test]
fn parallel_velocity_pcg_matches_single_worker_on_a_large_joint_island() {
    let mut single_worker = loaded_fixed_joint_graph(600, false, 8, false, false);
    let mut multiple_workers = loaded_fixed_joint_graph(600, false, 8, false, false);
    let mut repeated_parallel = loaded_fixed_joint_graph(600, false, 8, false, false);
    single_worker.set_joint_global_velocity_pcg_for_bench(64, true);
    multiple_workers.set_joint_global_velocity_pcg_for_bench(64, true);
    repeated_parallel.set_joint_global_velocity_pcg_for_bench(64, true);
    for world in [
        &mut single_worker,
        &mut multiple_workers,
        &mut repeated_parallel,
    ] {
        let root = world.bodies().next().unwrap().0;
        let point = world
            .body(root)
            .unwrap()
            .state()
            .transform()
            .transform_point(Vec3::Y * 0.25);
        world
            .body_mut(root)
            .unwrap()
            .apply_force_at_point(Vec3::Y * 30_000.0, point)
            .unwrap();
    }

    single_worker.step(DT).unwrap();
    multiple_workers
        .step_with_executor(DT, &ScopedPcgTestExecutor, 4)
        .unwrap();
    repeated_parallel
        .step_with_executor(DT, &ScopedPcgTestExecutor, 4)
        .unwrap();
    single_worker.step(DT).unwrap();
    multiple_workers
        .step_with_executor(DT, &ScopedPcgTestExecutor, 4)
        .unwrap();
    repeated_parallel
        .step_with_executor(DT, &ScopedPcgTestExecutor, 4)
        .unwrap();

    for (((_, single), (_, parallel)), (_, repeated)) in single_worker
        .bodies()
        .zip(multiple_workers.bodies())
        .zip(repeated_parallel.bodies())
    {
        assert_eq!(parallel.state(), repeated.state());
        let single_state = single.state();
        let parallel_state = parallel.state();
        let single_transform = single_state.transform();
        let parallel_transform = parallel_state.transform();
        assert!(
            (single_transform.translation() - parallel_transform.translation()).length() < 1.0e-9
        );
        assert!(
            (single_state.linear_velocity() - parallel_state.linear_velocity()).length() < 1.0e-9
        );
        assert!(
            (single_state.angular_velocity() - parallel_state.angular_velocity()).length() < 1.0e-9
        );
        let single_rotation = single_transform.rotation();
        let parallel_rotation = parallel_transform.rotation();
        let rotation_delta = |sign: f64| {
            Vec3::new(
                single_rotation.x - sign * parallel_rotation.x,
                single_rotation.y - sign * parallel_rotation.y,
                single_rotation.z - sign * parallel_rotation.z,
            )
            .length()
            .hypot(single_rotation.w - sign * parallel_rotation.w)
        };
        assert!(rotation_delta(1.0).min(rotation_delta(-1.0)) < 1.0e-9);
    }

    let metrics = multiple_workers.physics_step_metrics().joints;
    assert!(metrics.velocity_pcg_iterations > 0);
    assert!(metrics.velocity_pcg_converged_calls > 0);
    assert_eq!(metrics.velocity_pcg_iteration_limit_calls, 0);
    assert!(metrics.velocity_pcg_system_prepare_time > std::time::Duration::ZERO);
    assert!(metrics.velocity_pcg_matrix_vector_time > std::time::Duration::ZERO);
    assert!(metrics.velocity_pcg_preconditioner_time > std::time::Duration::ZERO);
    assert!(metrics.velocity_pcg_reduction_time > std::time::Duration::ZERO);
    assert!(metrics.constraint_coloring_prepare_time > std::time::Duration::ZERO);
    assert!(metrics.constraint_coloring_constraints >= 599);
    assert!(metrics.max_colors_per_island >= 2);
    assert_eq!(metrics.constraint_coloring_rebuilds, 0);
    assert!(metrics.constraint_coloring_cache_hits > 0);
    assert!(metrics.post_velocity_linear_residual_mps < 1.0e-4);
    assert!(metrics.post_velocity_angular_residual_radps < 1.0e-4);
}

#[cfg(feature = "bench-internals")]
#[test]
fn conflict_free_colored_solver_is_deterministic_and_reuses_batches() {
    let mut single_worker = loaded_fixed_joint_graph(200, true, 8, false, false);
    let mut multiple_workers = loaded_fixed_joint_graph(200, true, 8, false, false);
    let mut repeated_parallel = loaded_fixed_joint_graph(200, true, 8, false, false);
    for world in [
        &mut single_worker,
        &mut multiple_workers,
        &mut repeated_parallel,
    ] {
        let root = world.bodies().next().unwrap().0;
        let point = world
            .body(root)
            .unwrap()
            .state()
            .transform()
            .transform_point(Vec3::Y * 0.25);
        world
            .body_mut(root)
            .unwrap()
            .apply_force_at_point(Vec3::Y * 30_000.0, point)
            .unwrap();
    }

    single_worker.step(DT).unwrap();
    multiple_workers
        .step_with_executor(DT, &ScopedPcgTestExecutor, 4)
        .unwrap();
    repeated_parallel
        .step_with_executor(DT, &ScopedPcgTestExecutor, 4)
        .unwrap();
    for (((_, single), (_, parallel)), (_, repeated)) in single_worker
        .bodies()
        .zip(multiple_workers.bodies())
        .zip(repeated_parallel.bodies())
    {
        assert_eq!(parallel.state(), repeated.state());
        assert!(single.state().linear_velocity().is_finite());
        assert!(parallel.state().linear_velocity().is_finite());
    }
    let first_step_metrics = multiple_workers.physics_step_metrics().joints;
    assert!(first_step_metrics.constraint_coloring_rebuilds > 0);
    assert!(first_step_metrics.color_batches_processed > 0);
    assert_eq!(first_step_metrics.max_colors_per_island, 2);
    assert!(first_step_metrics.constraint_coloring_constraints >= 199);

    single_worker.step(DT).unwrap();
    multiple_workers
        .step_with_executor(DT, &ScopedPcgTestExecutor, 4)
        .unwrap();
    repeated_parallel
        .step_with_executor(DT, &ScopedPcgTestExecutor, 4)
        .unwrap();
    for ((_, parallel), (_, repeated)) in multiple_workers.bodies().zip(repeated_parallel.bodies())
    {
        assert_eq!(parallel.state(), repeated.state());
    }
    let cached_metrics = multiple_workers.physics_step_metrics().joints;
    assert_eq!(cached_metrics.constraint_coloring_rebuilds, 0);
    assert!(cached_metrics.constraint_coloring_cache_hits > 0);
    assert_finite_body_states(&multiple_workers);
    let scalar_errors = max_joint_errors(&single_worker);
    let colored_errors = max_joint_errors(&multiple_workers);
    assert!((scalar_errors.0 - colored_errors.0).abs() < 1.0e-12);
    assert!((scalar_errors.1 - colored_errors.1).abs() < 1.0e-12);
    println!(
        "branch=200 scalar_anchor/frame={:.6e}/{:.6e} colored_anchor/frame={:.6e}/{:.6e} colors={} batches={} cache_hits={}",
        scalar_errors.0,
        scalar_errors.1,
        colored_errors.0,
        colored_errors.1,
        cached_metrics.max_colors_per_island,
        cached_metrics.color_batches_processed,
        cached_metrics.constraint_coloring_cache_hits,
    );
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Release comparison of scalar and conflict-free colored large-island passes"]
fn diagnose_colored_constraint_batch_scaling() {
    fn percentile_micros(samples: &mut [u128], percentile: usize) -> u128 {
        samples.sort_unstable();
        let index = samples
            .len()
            .saturating_mul(percentile)
            .div_ceil(100)
            .max(1)
            - 1;
        samples[index.min(samples.len() - 1)]
    }

    for (topology, branching) in [("chain", false), ("branch", true)] {
        for body_count in [200, 500, 1000] {
            for worker_count in [1, 2, 4, 8] {
                let mut world = loaded_fixed_joint_graph(body_count, branching, 8, false, false);
                let root = world.bodies().next().unwrap().0;
                let point = world
                    .body(root)
                    .unwrap()
                    .state()
                    .transform()
                    .transform_point(Vec3::Y * 0.25);
                let mut elapsed_samples = Vec::with_capacity(5);
                for sample in 0..6 {
                    world
                        .body_mut(root)
                        .unwrap()
                        .apply_force_at_point(Vec3::Y * 30_000.0, point)
                        .unwrap();
                    let started = Instant::now();
                    if worker_count == 1 {
                        world.step(DT).unwrap();
                    } else {
                        world
                            .step_with_executor(DT, &ScopedPcgTestExecutor, worker_count)
                            .unwrap();
                    }
                    if sample > 0 {
                        elapsed_samples.push(started.elapsed().as_micros());
                    }
                }
                let p50 = percentile_micros(&mut elapsed_samples, 50);
                let p95 = percentile_micros(&mut elapsed_samples, 95);
                let metrics = world.physics_step_metrics().joints;
                let errors = max_joint_errors(&world);
                println!(
                    "topology={topology} bodies={body_count} workers={worker_count} step_p50/p95_us={p50}/{p95} anchor/frame_error={:.6e}/{:.6e} colors={} batches={} color_prepare_us={} rebuild/cache={}/{}",
                    errors.0,
                    errors.1,
                    metrics.max_colors_per_island,
                    metrics.color_batches_processed,
                    metrics.constraint_coloring_prepare_time.as_micros(),
                    metrics.constraint_coloring_rebuilds,
                    metrics.constraint_coloring_cache_hits,
                );
            }
        }
    }
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Release convergence sweep for colored large-island iteration counts"]
fn diagnose_colored_large_island_iteration_requirements() {
    for (topology, branching) in [("chain", false), ("branch", true)] {
        for (worker_count, velocity_iterations, position_iterations) in [
            (1, 8, 3),
            (4, 8, 3),
            (4, 16, 3),
            (4, 32, 3),
            (4, 64, 3),
            (4, 16, 8),
            (4, 32, 8),
        ] {
            let mut world =
                loaded_fixed_joint_graph(200, branching, velocity_iterations, false, false);
            world
                .set_collision_config(CollisionConfig {
                    position_iterations,
                    ..world.collision_config()
                })
                .unwrap();
            let root = world.bodies().next().unwrap().0;
            let point = world
                .body(root)
                .unwrap()
                .state()
                .transform()
                .transform_point(Vec3::Y * 0.25);
            for _ in 0..6 {
                world
                    .body_mut(root)
                    .unwrap()
                    .apply_force_at_point(Vec3::Y * 30_000.0, point)
                    .unwrap();
                if worker_count == 1 {
                    world.step(DT).unwrap();
                } else {
                    world
                        .step_with_executor(DT, &ScopedPcgTestExecutor, worker_count)
                        .unwrap();
                }
            }
            let errors = max_joint_errors(&world);
            let metrics = world.physics_step_metrics().joints;
            println!(
                "topology={topology} bodies=200 workers={worker_count} velocity/position={velocity_iterations}/{position_iterations} anchor/frame={:.6e}/{:.6e} batches={} residual={:.6e}m/s/{:.6e}rad/s",
                errors.0,
                errors.1,
                metrics.color_batches_processed,
                metrics.post_velocity_linear_residual_mps,
                metrics.post_velocity_angular_residual_radps,
            );
        }
    }
}

#[cfg(feature = "bench-internals")]
#[test]
fn velocity_pcg_reports_iteration_limit_without_claiming_convergence() {
    let mut world = loaded_fixed_joint_graph(100, false, 8, false, false);
    world.set_joint_global_velocity_pcg_for_bench(1, false);
    let root = world.bodies().next().unwrap().0;
    let point = world
        .body(root)
        .unwrap()
        .state()
        .transform()
        .transform_point(Vec3::Y * 0.25);
    world
        .body_mut(root)
        .unwrap()
        .apply_force_at_point(Vec3::Y * 30_000.0, point)
        .unwrap();

    world.step(DT).unwrap();

    let metrics = world.physics_step_metrics().joints;
    assert!(metrics.velocity_pcg_iteration_limit_calls > 0);
    assert!(metrics.velocity_pcg_linear_residual_mps > 1.0e-4);
}

#[cfg(feature = "bench-internals")]
#[test]
fn position_pcg_reports_iteration_limit_without_claiming_convergence() {
    let mut world = loaded_fixed_joint_graph(100, false, 8, false, false);
    world.set_joint_global_position_pcg_for_bench(1, false);
    let end = world.bodies().last().unwrap().0;
    world
        .body_mut(end)
        .unwrap()
        .set_linear_velocity(Vec3::Y * 30.0)
        .unwrap();

    world.step(DT).unwrap();

    let metrics = world.physics_step_metrics().joints;
    assert!(metrics.position_pcg_iteration_limit_calls > 0);
    assert!(metrics.position_pcg_linear_residual_m > 1.0e-5);
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Release comparison of parallel matrix-free velocity PCG across island sizes and workers"]
fn diagnose_parallel_velocity_pcg_scaling() {
    fn percentile_micros(samples: &mut [u128], percentile: usize) -> u128 {
        samples.sort_unstable();
        let index = samples
            .len()
            .saturating_mul(percentile)
            .div_ceil(100)
            .max(1)
            - 1;
        samples[index.min(samples.len() - 1)]
    }

    for body_count in [100, 200, 500, 1000] {
        for worker_count in [1, 2, 4, 8] {
            let mut world = loaded_fixed_joint_graph(body_count, false, 8, false, false);
            world.set_joint_global_velocity_pcg_for_bench(64, true);
            let root = world.bodies().next().unwrap().0;
            let point = world
                .body(root)
                .unwrap()
                .state()
                .transform()
                .transform_point(Vec3::Y * 0.25);
            let mut elapsed_samples = Vec::with_capacity(7);
            let mut solver_samples = Vec::with_capacity(7);
            let mut coloring_samples = Vec::with_capacity(7);
            let mut system_prepare_samples = Vec::with_capacity(7);
            let mut matrix_vector_samples = Vec::with_capacity(7);
            let mut preconditioner_samples = Vec::with_capacity(7);
            let mut reduction_samples = Vec::with_capacity(7);
            let mut cold_coloring_us = 0;
            for sample in 0..8 {
                world
                    .body_mut(root)
                    .unwrap()
                    .apply_force_at_point(Vec3::Y * 30_000.0, point)
                    .unwrap();
                let started = Instant::now();
                if worker_count == 1 {
                    world.step(DT).unwrap();
                } else {
                    world
                        .step_with_executor(DT, &ScopedPcgTestExecutor, worker_count)
                        .unwrap();
                }
                let elapsed = started.elapsed();
                let coloring_us = world
                    .physics_step_metrics()
                    .joints
                    .constraint_coloring_prepare_time
                    .as_micros();
                if sample == 0 {
                    cold_coloring_us = coloring_us;
                    continue;
                }
                let metrics = world.physics_step_metrics();
                elapsed_samples.push(elapsed.as_micros());
                solver_samples.push(metrics.collision.solver_iterations.as_micros());
                coloring_samples.push(coloring_us);
                system_prepare_samples
                    .push(metrics.joints.velocity_pcg_system_prepare_time.as_micros());
                matrix_vector_samples
                    .push(metrics.joints.velocity_pcg_matrix_vector_time.as_micros());
                preconditioner_samples
                    .push(metrics.joints.velocity_pcg_preconditioner_time.as_micros());
                reduction_samples.push(metrics.joints.velocity_pcg_reduction_time.as_micros());
            }
            let elapsed_p50 = percentile_micros(&mut elapsed_samples, 50);
            let elapsed_p95 = percentile_micros(&mut elapsed_samples, 95);
            let solver_p50 = percentile_micros(&mut solver_samples, 50);
            let solver_p95 = percentile_micros(&mut solver_samples, 95);
            let coloring_p50 = percentile_micros(&mut coloring_samples, 50);
            let system_prepare_p50 = percentile_micros(&mut system_prepare_samples, 50);
            let matrix_vector_p50 = percentile_micros(&mut matrix_vector_samples, 50);
            let preconditioner_p50 = percentile_micros(&mut preconditioner_samples, 50);
            let reduction_p50 = percentile_micros(&mut reduction_samples, 50);
            let pcg_metrics = world.physics_step_metrics().joints;
            println!(
                "graph=chain_{body_count} workers={worker_count} steps={} step_p50/p95_us={elapsed_p50}/{elapsed_p95} solver_p50/p95_us={solver_p50}/{solver_p95} coloring_cold/steady_p50_us={cold_coloring_us}/{coloring_p50} colors={} graph_rebuilds/cache_hits={}/{} pcg_prepare/matrix_vector/preconditioner/reduction_p50_us={system_prepare_p50}/{matrix_vector_p50}/{preconditioner_p50}/{reduction_p50} converged/iteration_limit={}/{} residual={:.6e}m/s/{:.6e}rad/s",
                elapsed_samples.len(),
                pcg_metrics.max_colors_per_island,
                pcg_metrics.constraint_coloring_rebuilds,
                pcg_metrics.constraint_coloring_cache_hits,
                pcg_metrics.velocity_pcg_converged_calls,
                pcg_metrics.velocity_pcg_iteration_limit_calls,
                pcg_metrics.post_velocity_linear_residual_mps,
                pcg_metrics.post_velocity_angular_residual_radps,
            );
        }
    }
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "diagnostic of loaded velocity convergence on joint graphs without contacts"]
fn diagnose_loaded_fixed_joint_graph_velocity_convergence() {
    for (name, count, branching) in [
        ("chain_2", 2, false),
        ("chain_10", 10, false),
        ("chain_50", 50, false),
        ("chain_100", 100, false),
        ("branch_100", 100, true),
    ] {
        for velocity_iterations in [1, 2, 4, 8, 16] {
            let mut world =
                loaded_fixed_joint_graph(count, branching, velocity_iterations, false, false);
            let first_body = world.bodies().next().unwrap().0;
            let force = world
                .bodies()
                .filter_map(|(_, body)| body.physics().mass())
                .map(|mass| mass.mass())
                .sum::<f64>()
                * 9.81
                * 1.3;
            let point = world
                .body(first_body)
                .unwrap()
                .state()
                .transform()
                .transform_point(Vec3::Y * 0.25);
            world
                .body_mut(first_body)
                .unwrap()
                .apply_force_at_point(Vec3::Y * force, point)
                .unwrap();

            let started = Instant::now();
            world.step(DT).unwrap();
            let first_elapsed = started.elapsed();
            let first = world.physics_step_metrics().joints;
            world.step(DT).unwrap();
            let second_elapsed = started.elapsed().saturating_sub(first_elapsed);
            let second = world.physics_step_metrics().joints;
            println!(
                "graph={name} joints={} velocity_iterations={} elapsed_us={}/{} step1_geometry_pre/post={:.6e}/{:.6e}m step2_geometry_pre/post={:.6e}/{:.6e}m velocity_linear_step1_first/final={:.6e}/{:.6e}m/s step2_final={:.6e}m/s velocity_angular_step1_first/final={:.6e}/{:.6e}rad/s step2_final={:.6e}rad/s",
                world.joint_count(),
                first.velocity_sweeps,
                first_elapsed.as_micros(),
                second_elapsed.as_micros(),
                first.pre_velocity_anchor_error_m,
                first.post_joint_anchor_error_m,
                second.pre_velocity_anchor_error_m,
                second.post_joint_anchor_error_m,
                first.first_velocity_linear_residual_mps,
                first.post_velocity_linear_residual_mps,
                second.post_velocity_linear_residual_mps,
                first.first_velocity_angular_residual_radps,
                first.post_velocity_angular_residual_radps,
                second.post_velocity_angular_residual_radps,
            );
        }
    }

    for (name, branching) in [("chain_100", false), ("branch_100", true)] {
        for reverse_joint_order in [false, true] {
            for velocity_iterations in [8, 16] {
                let mut world = loaded_fixed_joint_graph(
                    100,
                    branching,
                    velocity_iterations,
                    reverse_joint_order,
                    false,
                );
                let first_body = world.bodies().next().unwrap().0;
                let force = world
                    .bodies()
                    .filter_map(|(_, body)| body.physics().mass())
                    .map(|mass| mass.mass())
                    .sum::<f64>()
                    * 9.81
                    * 1.3;
                let point = world
                    .body(first_body)
                    .unwrap()
                    .state()
                    .transform()
                    .transform_point(Vec3::Y * 0.25);
                world
                    .body_mut(first_body)
                    .unwrap()
                    .apply_force_at_point(Vec3::Y * force, point)
                    .unwrap();
                world.step(DT).unwrap();
                let metrics = world.physics_step_metrics().joints;
                println!(
                    "graph={name} joint_order={} velocity_iterations={} linear_residual={:.6e}m/s angular_residual={:.6e}rad/s",
                    if reverse_joint_order {
                        "reverse"
                    } else {
                        "forward"
                    },
                    metrics.velocity_sweeps,
                    metrics.post_velocity_linear_residual_mps,
                    metrics.post_velocity_angular_residual_radps,
                );
            }
        }
    }

    for (name, branching) in [("chain_100", false), ("branch_100", true)] {
        for velocity_iterations in [8, 16] {
            let mut world =
                loaded_fixed_joint_graph(100, branching, velocity_iterations, false, true);
            let first_body = world.bodies().next().unwrap().0;
            let force = world
                .bodies()
                .filter_map(|(_, body)| body.physics().mass())
                .map(|mass| mass.mass())
                .sum::<f64>()
                * 9.81
                * 1.3;
            let point = world
                .body(first_body)
                .unwrap()
                .state()
                .transform()
                .transform_point(Vec3::Y * 0.25);
            world
                .body_mut(first_body)
                .unwrap()
                .apply_force_at_point(Vec3::Y * force, point)
                .unwrap();
            world.step(DT).unwrap();
            let first = world.physics_step_metrics().joints;
            world.step(DT).unwrap();
            let second = world.physics_step_metrics().joints;
            println!(
                "graph={name} joint_order=alternating velocity_iterations={} linear_residual={:.6e}m/s angular_residual={:.6e}rad/s next_predicted_anchor={:.6e}m",
                first.velocity_sweeps,
                first.post_velocity_linear_residual_mps,
                first.post_velocity_angular_residual_radps,
                second.pre_velocity_anchor_error_m,
            );
        }
    }

    for (name, branching) in [("chain_100", false), ("branch_100", true)] {
        for max_velocity_iterations in [8, 16, 32, 64, 128] {
            let mut world = loaded_fixed_joint_graph(100, branching, 8, false, false);
            world.set_joint_adaptive_velocity_max_for_bench(max_velocity_iterations);
            let first_body = world.bodies().next().unwrap().0;
            let force = world
                .bodies()
                .filter_map(|(_, body)| body.physics().mass())
                .map(|mass| mass.mass())
                .sum::<f64>()
                * 9.81
                * 1.3;
            let point = world
                .body(first_body)
                .unwrap()
                .state()
                .transform()
                .transform_point(Vec3::Y * 0.25);
            world
                .body_mut(first_body)
                .unwrap()
                .apply_force_at_point(Vec3::Y * force, point)
                .unwrap();

            let started = Instant::now();
            world.step(DT).unwrap();
            let metrics = world.physics_step_metrics().joints;
            println!(
                "graph={name} adaptive_velocity_max={} actual_sweeps={} elapsed_us={} residual={:.6e}m/s/{:.6e}rad/s geometry_post={:.6e}m/{:.6e}rad",
                max_velocity_iterations,
                metrics.velocity_sweeps,
                started.elapsed().as_micros(),
                metrics.post_velocity_linear_residual_mps,
                metrics.post_velocity_angular_residual_radps,
                metrics.post_joint_anchor_error_m,
                metrics.post_joint_frame_error_rad,
            );
        }
    }

    for (name, branching) in [("chain_100", false), ("branch_100", true)] {
        for relaxation in [1.0, 1.3, 1.6, 1.8, 1.9, 1.95] {
            let mut world = loaded_fixed_joint_graph(100, branching, 8, false, false);
            world.set_joint_velocity_relaxation_for_bench(relaxation);
            let first_body = world.bodies().next().unwrap().0;
            let force = world
                .bodies()
                .filter_map(|(_, body)| body.physics().mass())
                .map(|mass| mass.mass())
                .sum::<f64>()
                * 9.81
                * 1.3;
            let point = world
                .body(first_body)
                .unwrap()
                .state()
                .transform()
                .transform_point(Vec3::Y * 0.25);
            world
                .body_mut(first_body)
                .unwrap()
                .apply_force_at_point(Vec3::Y * force, point)
                .unwrap();
            let started = Instant::now();
            world.step(DT).unwrap();
            let metrics = world.physics_step_metrics().joints;
            world.step(DT).unwrap();
            let next = world.physics_step_metrics().joints;
            println!(
                "graph={name} solver=relaxed_gs relaxation={relaxation:.2} elapsed_us={} residual={:.6e}m/s/{:.6e}rad/s next_predicted_anchor={:.6e}m",
                started.elapsed().as_micros(),
                metrics.post_velocity_linear_residual_mps,
                metrics.post_velocity_angular_residual_radps,
                next.pre_velocity_anchor_error_m,
            );
        }
    }

    for (name, branching) in [("chain_100", false), ("branch_100", true)] {
        for preconditioner in ["diagonal", "ic0", "block_jacobi"] {
            for max_pcg_iterations in [64, 128, 256] {
                let mut world = loaded_fixed_joint_graph(100, branching, 8, false, false);
                match preconditioner {
                    "diagonal" => {
                        world.set_joint_global_velocity_pcg_for_bench(max_pcg_iterations, false)
                    }
                    "ic0" => {
                        world.set_joint_global_velocity_pcg_for_bench(max_pcg_iterations, true)
                    }
                    "block_jacobi" => {
                        world.set_joint_global_velocity_block_pcg_for_bench(max_pcg_iterations)
                    }
                    _ => unreachable!(),
                }
                let first_body = world.bodies().next().unwrap().0;
                let force = world
                    .bodies()
                    .filter_map(|(_, body)| body.physics().mass())
                    .map(|mass| mass.mass())
                    .sum::<f64>()
                    * 9.81
                    * 1.3;
                let point = world
                    .body(first_body)
                    .unwrap()
                    .state()
                    .transform()
                    .transform_point(Vec3::Y * 0.25);
                world
                    .body_mut(first_body)
                    .unwrap()
                    .apply_force_at_point(Vec3::Y * force, point)
                    .unwrap();

                let started = Instant::now();
                world.step(DT).unwrap();
                let first = world.physics_step_metrics().joints;
                world.step(DT).unwrap();
                let second = world.physics_step_metrics().joints;
                println!(
                    "graph={name} solver=global_pcg preconditioner={preconditioner} max_iterations={max_pcg_iterations} actual_iterations={} converged/iteration_limit={}/{} pcg_residual={:.6e}m/s/{:.6e}rad/s elapsed_us={} velocity_residual={:.6e}m/s/{:.6e}rad/s next_predicted_anchor={:.6e}m",
                    first.velocity_pcg_iterations,
                    first.velocity_pcg_converged_calls,
                    first.velocity_pcg_iteration_limit_calls,
                    first.velocity_pcg_linear_residual_mps,
                    first.velocity_pcg_angular_residual_radps,
                    started.elapsed().as_micros(),
                    first.post_velocity_linear_residual_mps,
                    first.post_velocity_angular_residual_radps,
                    second.pre_velocity_anchor_error_m,
                );
            }
        }
    }
}

#[cfg(feature = "bench-internals")]
fn loaded_fixed_joint_graph(
    count: usize,
    branching: bool,
    velocity_iterations: u32,
    reverse_joint_order: bool,
    alternate_velocity_sweeps: bool,
) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let mut bodies = Vec::with_capacity(count);
    let mut positions = Vec::with_capacity(count);
    let mut edges = Vec::with_capacity(count.saturating_sub(1));
    for index in 0..count {
        let parent = if branching && index > 0 {
            Some((index - 1) / 2)
        } else if index > 0 {
            Some(index - 1)
        } else {
            None
        };
        let position = parent.map_or(Vec3::ZERO, |parent| positions[parent] + Vec3::X);
        positions.push(position);

        let mass = 0.8 + (index % 7) as f64 * 0.17;
        let center = Vec3::new(
            (index % 3) as f64 * 0.002,
            -((index % 5) as f64) * 0.001,
            (index % 2) as f64 * 0.0015,
        );
        let scale = 0.7 + (index % 5) as f64 * 0.13;
        let inertia = Mat3::from_diagonal(Vec3::new(scale * 0.6, scale * 0.8, scale));
        let body = RigidBody::new_dynamic(
            Transform::new(position, Quat::IDENTITY).unwrap(),
            MassProperties::new(mass, center, inertia).unwrap(),
        )
        .unwrap();
        bodies.push(world.add_body(body).unwrap());

        if let Some(parent) = parent {
            edges.push((parent, index));
        }
    }
    if reverse_joint_order {
        edges.reverse();
    }
    for (parent, child) in edges {
        world
            .add_joint(
                bodies[parent],
                bodies[child],
                Joint::Fixed(FixedJoint::new(JointCommon::new(
                    JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
                    JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
                ))),
            )
            .unwrap();
    }
    world
        .set_collision_config(CollisionConfig {
            velocity_iterations,
            ..CollisionConfig::default()
        })
        .unwrap();
    world.set_joint_alternating_velocity_sweeps_for_bench(alternate_velocity_sweeps);
    world
}

fn two_body_world(reverse: bool, static_a: bool) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let rotation = Quat::from_axis_angle(Vec3::Z, 0.001).unwrap();
    let first = world
        .add_body(if static_a {
            RigidBody::new_static(Transform::identity())
        } else {
            dynamic_body(Vec3::ZERO, Quat::IDENTITY)
        })
        .unwrap();
    let second = world
        .add_body(dynamic_body(Vec3::X * 0.01, rotation))
        .unwrap();
    let joint = Joint::Fixed(FixedJoint::new(JointCommon::new(
        JointFrame::IDENTITY,
        JointFrame::IDENTITY,
    )));
    if reverse {
        world.add_joint(second, first, joint).unwrap();
    } else {
        world.add_joint(first, second, joint).unwrap();
    }
    world
}

fn dynamic_body(position: Vec3, rotation: Quat) -> RigidBody {
    let transform = Transform::new(position, rotation).unwrap();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    RigidBody::new_dynamic(transform, mass).unwrap()
}

fn dynamic_body_with_properties(position: Vec3, mass: MassProperties) -> RigidBody {
    let transform = Transform::new(position, Quat::IDENTITY).unwrap();
    RigidBody::new_dynamic(transform, mass).unwrap()
}

fn max_joint_errors(world: &PhysicsWorld) -> (f64, f64) {
    let mut anchor_error: f64 = 0.0;
    let mut rotation_error: f64 = 0.0;
    for (_, body_a_id, body_b_id, joint) in world.joints() {
        let body_a = world.body(body_a_id).unwrap();
        let body_b = world.body(body_b_id).unwrap();
        let common = joint.common();
        let transform_a = body_a.state().transform();
        let transform_b = body_b.state().transform();
        let anchor_a = transform_a.transform_point(common.frame_a.local_anchor);
        let anchor_b = transform_b.transform_point(common.frame_b.local_anchor);
        anchor_error = anchor_error.max((anchor_b - anchor_a).length());
        let frame_a = transform_a.rotation() * common.frame_a.local_rotation;
        let frame_b = transform_b.rotation() * common.frame_b.local_rotation;
        let relative = (frame_a.conjugate() * frame_b).try_normalize().unwrap();
        let vector_length = Vec3::new(relative.x, relative.y, relative.z).length();
        rotation_error = rotation_error.max(2.0 * vector_length.atan2(relative.w.abs()));
    }
    (anchor_error, rotation_error)
}

fn assert_finite_body_states(world: &PhysicsWorld) {
    for (_, body) in world.bodies() {
        let state = body.state();
        assert!(state.transform().translation().is_finite());
        assert!(state.transform().rotation().is_normalized());
        assert!(state.linear_velocity().is_finite());
        assert!(state.angular_velocity().is_finite());
    }
}

fn kinetic_energy(world: &PhysicsWorld) -> f64 {
    world
        .bodies()
        .map(|(_, body)| {
            let Some(mass) = body.physics().mass() else {
                return 0.0;
            };
            let state = body.state();
            0.5 * mass.mass() * state.linear_velocity().length_squared()
                + 0.5 * state.angular_velocity().length_squared()
        })
        .sum()
}
