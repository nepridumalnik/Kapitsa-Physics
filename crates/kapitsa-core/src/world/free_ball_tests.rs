use kapitsa_math::{Mat3, Quat, Vec3};

use crate::{
    collision::CollisionError,
    execution::SequentialExecutor,
    gravity::CentralGravity,
    joint::{
        BallJoint, D6Joint, FixedJoint, HingeJoint, Joint, JointCommon, JointFrame, JointLimit,
        SliderJoint,
    },
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
    world::{BodyId, JointId, PhysicsWorld, StepError},
};

fn pair_world() -> (PhysicsWorld, [BodyId; 2], JointId) {
    let mut world = PhysicsWorld::new();
    let omega = Vec3::new(0.05, 0.1, 0.02);
    let mut add = |origin: Vec3, mass, center| {
        let mass = MassProperties::new(
            mass,
            center,
            Mat3::from_rows([[0.6, 0.08, -0.03], [0.08, 0.8, 0.04], [-0.03, 0.04, 1.0]]),
        )
        .unwrap();
        let mut body =
            RigidBody::new_dynamic(Transform::new(origin, Quat::IDENTITY).unwrap(), mass).unwrap();
        body.set_angular_velocity(omega).unwrap();
        body.set_linear_velocity(Vec3::new(0.2, -0.1, 0.05) + omega.cross(origin + center))
            .unwrap();
        world.add_body(body).unwrap()
    };
    let ids = [
        add(Vec3::ZERO, 0.8, Vec3::new(0.01, -0.02, 0.03)),
        add(Vec3::X, 1.3, Vec3::new(-0.03, 0.01, 0.02)),
    ];
    // Opposite joint/dense ordering also exercises ordered prediction commit.
    let joint = world
        .add_joint(
            ids[1],
            ids[0],
            Joint::Ball(BallJoint::new(JointCommon::new(
                JointFrame::new(-Vec3::X * 0.5, Quat::IDENTITY),
                JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
            ))),
        )
        .unwrap();
    (world, ids, joint)
}

fn step_route(world: &mut PhysicsWorld, route: usize, dt: f64) -> Result<(), StepError> {
    match route {
        0 => world.step(dt),
        1 => world.step_with_executor(dt, &super::ScopedThreadExecutor, 4),
        2 => world.step_with_shards(dt, &mut [Vec::new(), Vec::new()], &SequentialExecutor, 2),
        3 => {
            let mut shards = [Vec::new(), Vec::new()];
            world.fill_step_shards(&mut shards)?;
            for step in shards.iter_mut().flatten() {
                step.integrate(dt)?;
            }
            world.commit_step_shards(&shards)
        }
        _ => unreachable!(),
    }
}

#[test]
fn free_ball_pair_all_step_routes_are_deterministic() {
    for frequency in [60, 120, 240] {
        let mut worlds = [
            pair_world(),
            pair_world(),
            pair_world(),
            pair_world(),
            pair_world(),
        ];
        for _ in 0..5 * frequency {
            for (route, (world, _, _)) in worlds.iter_mut().enumerate() {
                step_route(world, route % 4, 1.0 / f64::from(frequency)).unwrap();
            }
            for (world, ids, _) in &worlds[1..] {
                for (&id, &reference) in ids.iter().zip(&worlds[0].1) {
                    assert_eq!(world.body(id), worlds[0].0.body(reference));
                }
            }
        }
    }
}

#[test]
fn free_ball_pair_numerical_failures_are_atomic() {
    for overflow in [false, true] {
        for route in 0..4 {
            let (mut world, ids, joint) = pair_world();
            world.step(1.0 / 120.0).unwrap();
            for id in ids {
                let body = world.body_mut(id).unwrap();
                if overflow {
                    body.set_linear_velocity(Vec3::X * 1.0e300).unwrap();
                } else {
                    body.set_angular_velocity(Vec3::new(10.0, 15.0, 20.0))
                        .unwrap();
                }
            }
            let original = ids.map(|id| *world.body(id).unwrap());
            let original_joint = *world.joint(joint).unwrap();
            let dt = if overflow { 1.0e100 } else { 1.0 };
            let error = step_route(&mut world, route, dt).unwrap_err();
            if !overflow {
                assert_eq!(error, StepError::Collision(CollisionError::NonConvergent));
            }
            assert_eq!(ids.map(|id| *world.body(id).unwrap()), original);
            assert_eq!(*world.joint(joint).unwrap(), original_joint);
            assert!(world.joint_events().is_empty());
            assert!(world.collision_events().is_empty());
            assert!(world.collision_system.context.predicted_states.is_empty());
        }
    }
}

#[test]
fn free_ball_pair_rejects_unsupported_configurations_without_mutation() {
    for case in (0..7).chain(13..22) {
        let (mut world, ids, joint) = pair_world();
        let Joint::Ball(mut ball) = *world.joint(joint).unwrap() else {
            unreachable!()
        };
        match case {
            0 => ball.common.enabled = false,
            1 => ball.common.compliance = 1.0e-4,
            2 => ball.common.damping = 0.1,
            3 => ball.common.break_force = Some(1.0),
            4 => ball.common.break_torque = Some(1.0),
            5 => ball.swing_limit = Some(0.5),
            6 => ball.twist_limit = Some(JointLimit::new(-0.5, 0.5)),
            _ => {}
        }
        let replacement = match case {
            7 => Joint::Fixed(FixedJoint::new(ball.common)),
            8 => Joint::Hinge(HingeJoint::new(ball.common)),
            9 => Joint::Slider(SliderJoint::new(ball.common)),
            10 => Joint::D6(D6Joint::new(ball.common)),
            _ => Joint::Ball(ball),
        };
        world.update_joint(joint, replacement).unwrap();
        match case {
            11 => {
                world.add_body(super::dynamic_body()).unwrap();
            }
            12 => {
                world.add_joint(ids[0], ids[1], replacement).unwrap();
            }
            13 => {
                world.add_collider(ids[0], super::collider(0.1)).unwrap();
            }
            14 => world
                .body_mut(ids[0])
                .unwrap()
                .apply_force(Vec3::Y)
                .unwrap(),
            15 => world
                .body_mut(ids[0])
                .unwrap()
                .apply_torque(Vec3::Y)
                .unwrap(),
            16 => {
                world.set_central_gravity(Some(CentralGravity::new(Vec3::Y * 10.0, 1.0).unwrap()))
            }
            17 => {
                world.add_force_source(ids[0], Vec3::ZERO, Vec3::Y).unwrap();
            }
            18 => world.body_mut(ids[0]).unwrap().sleep().unwrap(),
            19 => *world.body_mut(ids[0]).unwrap() = RigidBody::new_static(Transform::identity()),
            20 => {
                *world.body_mut(ids[0]).unwrap() = RigidBody::new_kinematic(Transform::identity())
            }
            21 => {
                world.remove_joint(joint).unwrap();
            }
            _ => {}
        }
        let original: Vec<_> = world.bodies().map(|(_, body)| *body).collect();

        assert_eq!(
            world.bodies().map(|(_, body)| *body).collect::<Vec<_>>(),
            original
        );
        world.step(1.0 / 120.0).unwrap();
        assert_eq!(
            world.collision_system.constrained_island_count(),
            0,
            "case {case}"
        );
    }
}

#[test]
fn free_ball_pair_rejects_stale_shards_before_any_commit() {
    let (mut world, ids, _) = pair_world();
    let mut shards = [Vec::new(), Vec::new()];
    world.fill_step_shards(&mut shards).unwrap();
    for step in shards.iter_mut().flatten() {
        step.integrate(1.0 / 120.0).unwrap();
    }
    world
        .body_mut(ids[0])
        .unwrap()
        .set_linear_velocity(Vec3::Y)
        .unwrap();
    let original = ids.map(|id| *world.body(id).unwrap());
    assert_eq!(
        world.commit_step_shards(&shards),
        Err(StepError::InvalidStepResults)
    );
    assert_eq!(ids.map(|id| *world.body(id).unwrap()), original);
}

#[test]
fn free_ball_pair_preserves_pending_collision_events_across_failure_and_commit() {
    use crate::{collider::CollisionMode, collision::CollisionEventKind};

    for route in 0..4 {
        let (mut world, ids, joint) = pair_world();
        let Joint::Ball(mut ball) = *world.joint(joint).unwrap() else {
            unreachable!()
        };
        ball.common.collide_connected = true;
        world.update_joint(joint, Joint::Ball(ball)).unwrap();
        let colliders = ids.map(|id| {
            let mut sensor = super::collider(1.0);
            sensor.set_mode(CollisionMode::Sensor);
            world.add_collider(id, sensor).unwrap()
        });
        world.step(1.0 / 120.0).unwrap();
        assert_eq!(world.collision_events().len(), 1);
        assert_eq!(world.collision_events()[0].kind, CollisionEventKind::Begin);
        let events = world.collision_events().to_vec();
        for id in colliders {
            world.remove_collider(id).unwrap();
        }
        let original = ids.map(|id| *world.body(id).unwrap());
        for id in ids {
            world
                .body_mut(id)
                .unwrap()
                .set_angular_velocity(Vec3::new(10.0, 15.0, 20.0))
                .unwrap();
        }
        assert_eq!(
            step_route(&mut world, route, 1.0),
            Err(StepError::Collision(CollisionError::NonConvergent))
        );
        assert_eq!(world.collision_events(), events);
        for (id, body) in ids.into_iter().zip(original) {
            *world.body_mut(id).unwrap() = body;
        }
        step_route(&mut world, route, 1.0 / 120.0).unwrap();
        assert_eq!(world.collision_events().len(), 1);
        assert_eq!(world.collision_events()[0].kind, CollisionEventKind::End);
        step_route(&mut world, route, 1.0 / 120.0).unwrap();
        assert!(world.collision_events().is_empty());
    }
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Run alone to count process-wide allocations after warming the Ball pair"]
fn free_ball_pair_warm_steps_do_not_allocate() {
    let (mut world, _, _) = pair_world();
    for _ in 0..32 {
        world.step(1.0 / 120.0).unwrap();
    }
    let (allocations, bytes) = super::tracked_world_allocations(|| {
        for _ in 0..600 {
            world.step(1.0 / 120.0).unwrap();
        }
    });
    println!("free Ball pair: 600 warmed steps, {allocations} allocations, {bytes} bytes");
    assert_eq!((allocations, bytes), (0, 0));
}
