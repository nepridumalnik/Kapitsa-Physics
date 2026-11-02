use super::{FixedJoint, Joint, JointCommon, JointFrame};
use crate::{
    collision::CollisionConfig, mass::MassProperties, rigid_body::RigidBody, transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};
#[cfg(feature = "bench-internals")]
use std::time::Instant;

const DT: f64 = 1.0 / 120.0;

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
            let mut world = loaded_fixed_joint_graph(count, branching, velocity_iterations, false);
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
}

#[cfg(feature = "bench-internals")]
fn loaded_fixed_joint_graph(
    count: usize,
    branching: bool,
    velocity_iterations: u32,
    reverse_joint_order: bool,
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
