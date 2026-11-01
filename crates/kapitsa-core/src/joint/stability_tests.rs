use super::{FixedJoint, Joint, JointCommon, JointFrame};
use crate::{
    collision::CollisionConfig, mass::MassProperties, rigid_body::RigidBody, transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

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
