use kapitsa_core::{
    collider::{Collider, CollisionMode},
    collision::{CollisionEventKind, CollisionPair},
    execution::{ExecutionError, SequentialExecutor, StepExecutor},
    joint::{FixedJoint, Joint, JointCommon, JointFrame, SpringJoint},
    mass::MassProperties,
    material::PhysicsMaterial,
    rigid_body::{CcdMode, RigidBody},
    shape::{Cuboid, Plane, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};
use std::collections::HashSet;

struct ScopedExecutor;

impl StepExecutor for ScopedExecutor {
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
        let total = chunk_sizes
            .iter()
            .try_fold(0_usize, |total, size| total.checked_add(*size));
        if chunk_sizes.contains(&0) || total != Some(values.len()) {
            return Err(ExecutionError::InvalidChunkLayout);
        }
        std::thread::scope(|scope| {
            let mut remaining = values;
            let mut start = 0;
            for size in chunk_sizes.iter().copied() {
                let (chunk, tail) = remaining.split_at_mut(size);
                let operation = &operation;
                scope.spawn(move || operation(start, chunk));
                start += size;
                remaining = tail;
            }
        });
        Ok(())
    }
}

fn transform(position: Vec3) -> Transform {
    Transform::new(position, Quat::IDENTITY).expect("valid transform")
}

fn dynamic(position: Vec3, velocity: Vec3) -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let mut body = RigidBody::new_dynamic(transform(position), mass).expect("dynamic body");
    body.set_linear_velocity(velocity).expect("linear velocity");
    body
}

fn add_plane(world: &mut PhysicsWorld, mode: CollisionMode) -> kapitsa_core::world::ColliderId {
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("static body");
    let mut collider = Collider::new(
        Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).expect("plane")),
        Transform::identity(),
    );
    collider.set_mode(mode);
    world.add_collider(body, collider).expect("plane collider")
}

fn add_sphere(
    world: &mut PhysicsWorld,
    body: kapitsa_core::world::BodyId,
) -> kapitsa_core::world::ColliderId {
    world
        .add_collider(
            body,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                Transform::identity(),
            ),
        )
        .expect("sphere collider")
}

fn material(
    world: &mut PhysicsWorld,
    friction: f64,
    restitution: f64,
) -> kapitsa_core::material::MaterialId {
    world
        .add_material(PhysicsMaterial::new(friction, friction, restitution, 1.0).expect("material"))
        .expect("world material")
}

#[test]
fn sphere_plane_impact_changes_motion_and_publishes_lifecycle() {
    let mut world = PhysicsWorld::new();
    let plane = add_plane(&mut world, CollisionMode::Solid);
    let body = world
        .add_body(dynamic(
            Vec3::new(0.0, 0.6, 0.0),
            Vec3::new(0.0, -10.0, 0.0),
        ))
        .expect("dynamic body");
    let sphere = add_sphere(&mut world, body);
    let static_body = world.collider_body(plane).expect("plane body");
    let static_before = *world.body(static_body).expect("static state").state();

    world.step(0.02).expect("collision step");

    assert!(world.body(body).unwrap().state().linear_velocity().y > 0.0);
    assert!(
        world
            .body(body)
            .unwrap()
            .state()
            .transform()
            .translation()
            .y
            >= 0.4
    );
    assert_eq!(*world.body(static_body).unwrap().state(), static_before);
    let pair = CollisionPair::new(plane, sphere).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Begin
            && event.pair == pair
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse > 0.0)
    }));
    assert!(
        world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Impact && event.pair == pair })
    );
}

#[test]
fn dynamic_dynamic_collision_conserves_linear_momentum_without_restitution() {
    let mut world = PhysicsWorld::new();
    let material = material(&mut world, 0.0, 0.0);
    let left = world
        .add_body(dynamic(Vec3::new(-0.45, 0.0, 0.0), Vec3::X))
        .expect("left body");
    let left_collider = add_sphere(&mut world, left);
    world
        .set_collider_material(left_collider, Some(material))
        .expect("left material");
    let right = world
        .add_body(dynamic(Vec3::new(0.45, 0.0, 0.0), -Vec3::X))
        .expect("right body");
    let right_collider = add_sphere(&mut world, right);
    world
        .set_collider_material(right_collider, Some(material))
        .expect("right material");
    let before = world.body(left).unwrap().state().linear_velocity()
        + world.body(right).unwrap().state().linear_velocity();

    world.step(1.0 / 60.0).expect("collision step");

    let after = world.body(left).unwrap().state().linear_velocity()
        + world.body(right).unwrap().state().linear_velocity();
    assert!((after - before).length() <= 1.0e-8);
    assert!(world.body(left).unwrap().state().linear_velocity().x < 1.0);
    assert!(world.body(right).unwrap().state().linear_velocity().x > -1.0);
}

#[test]
fn material_restitution_and_friction_affect_world_contacts() {
    let mut world = PhysicsWorld::new();
    let material = material(&mut world, 1.0, 0.8);
    let plane = add_plane(&mut world, CollisionMode::Solid);

    let body = world
        .add_body(dynamic(
            Vec3::new(0.0, 0.49, 0.0),
            Vec3::new(1.0, -1.0, 0.0),
        ))
        .unwrap();
    let sphere = add_sphere(&mut world, body);
    world.set_collider_material(sphere, Some(material)).unwrap();
    world.set_collider_material(plane, Some(material)).unwrap();

    world.step(0.01).unwrap();

    let state = world.body(body).unwrap().state();
    assert!(state.linear_velocity().y > 0.0);
    assert!(state.linear_velocity().x < 1.0);
    assert!(state.angular_velocity().z.abs() > 0.0);
}

#[test]
fn moving_kinematic_body_keeps_its_trajectory_and_wakes_sleeping_dynamic_body() {
    let mut world = PhysicsWorld::new();
    let sleeping = world
        .add_body(dynamic(Vec3::ZERO, Vec3::ZERO))
        .expect("sleeping body");
    world.body_mut(sleeping).unwrap().sleep().unwrap();
    add_sphere(&mut world, sleeping);

    let mut kinematic = RigidBody::new_kinematic(transform(Vec3::new(1.01, 0.0, 0.0)));
    kinematic.set_linear_velocity(-Vec3::X).unwrap();
    let kinematic = world.add_body(kinematic).unwrap();
    add_sphere(&mut world, kinematic);

    world.step(0.02).expect("collision step");

    let kinematic_state = world.body(kinematic).unwrap().state();
    assert!((kinematic_state.transform().translation().x - 0.99).abs() < 1.0e-12);
    assert_eq!(kinematic_state.linear_velocity(), -Vec3::X);
    assert!(!world.body(sleeping).unwrap().state().is_sleeping());
}

#[test]
fn ten_box_stack_remains_bounded_without_gravity() {
    let mut world = PhysicsWorld::new();
    add_plane(&mut world, CollisionMode::Solid);
    let mut bodies = Vec::new();
    for index in 0..10 {
        let y = 0.49 + index as f64 * 0.98;
        let body = world
            .add_body(dynamic(Vec3::new(0.0, y, 0.0), Vec3::ZERO))
            .unwrap();
        world
            .add_collider(
                body,
                Collider::new(
                    Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()),
                    Transform::identity(),
                ),
            )
            .unwrap();
        bodies.push(body);
    }
    let initial = bodies
        .iter()
        .map(|id| world.body(*id).unwrap().state().transform().translation().y)
        .collect::<Vec<_>>();

    for _ in 0..120 {
        world.step(1.0 / 60.0).unwrap();
    }

    for (id, start_y) in bodies.into_iter().zip(initial) {
        let state = world.body(id).unwrap().state();
        let y = state.transform().translation().y;
        assert!(y.is_finite() && y > 0.35);
        assert!((y - start_y).abs() < 0.5);
        assert!(state.linear_velocity().is_finite());
        assert!(state.angular_velocity().is_finite());
    }
}

#[test]
fn thousand_body_contact_chain_steps_with_finite_results() {
    let mut world = PhysicsWorld::new();
    for index in 0..1_000 {
        let body = world
            .add_body(dynamic(Vec3::new(index as f64 * 0.9, 0.0, 0.0), Vec3::ZERO))
            .unwrap();
        world
            .add_collider(
                body,
                Collider::new(
                    Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()),
                    Transform::identity(),
                ),
            )
            .unwrap();
    }

    world.step(1.0 / 60.0).expect("large contact step");

    assert_eq!(world.body_count(), 1_000);
    let collider_ids = world
        .colliders()
        .map(|(collider, _, _)| collider)
        .collect::<Vec<_>>();
    assert_eq!(collider_ids.len(), 1_000);
    let expected_pairs = collider_ids
        .windows(2)
        .map(|window| CollisionPair::new(window[0], window[1]).unwrap())
        .collect::<HashSet<_>>();
    let actual_pairs = world
        .collision_events()
        .iter()
        .filter(|event| event.kind == CollisionEventKind::Begin)
        .map(|event| event.pair)
        .collect::<HashSet<_>>();
    assert_eq!(actual_pairs, expected_pairs);
    for (body, _) in world.bodies() {
        let state = world.body(body).unwrap().state();
        assert!(state.transform().translation().is_finite());
        assert!(state.linear_velocity().is_finite());
        assert!(state.angular_velocity().is_finite());
    }
}

#[test]
fn continuous_sphere_does_not_tunnel_through_a_thin_wall() {
    let mut world = PhysicsWorld::new();
    let mut body = dynamic(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(1_000.0, 0.0, 0.0));
    body.set_ccd_mode(CcdMode::Continuous);
    let body = world.add_body(body).unwrap();
    add_sphere(&mut world, body);
    let wall = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    world
        .add_collider(
            wall,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 5.0, 5.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(0.01).unwrap();

    assert!(world.body(body).unwrap().state().linear_velocity().x < 0.0);
}

#[test]
fn sensor_reports_overlap_without_changing_body_motion() {
    let mut world = PhysicsWorld::new();
    let sensor = add_plane(&mut world, CollisionMode::Sensor);
    let body = world
        .add_body(dynamic(Vec3::new(0.0, 0.4, 0.0), Vec3::new(0.0, -2.0, 0.0)))
        .unwrap();
    let sphere = add_sphere(&mut world, body);

    world.step(0.01).unwrap();

    let state = world.body(body).unwrap().state();
    assert_eq!(state.linear_velocity().y, -2.0);
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Begin
            && event.pair == CollisionPair::new(sensor, sphere).unwrap()
            && event.sensor
    }));
    assert!(
        !world
            .collision_events()
            .iter()
            .any(|event| event.kind == CollisionEventKind::Impact)
    );
}

#[test]
fn compound_body_keeps_contacts_for_each_distinct_collider() {
    let mut world = PhysicsWorld::new();
    let plane = add_plane(&mut world, CollisionMode::Solid);
    let body = world
        .add_body(dynamic(
            Vec3::new(0.0, 0.45, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
        ))
        .expect("compound body");
    let collider = |x| {
        Collider::new(
            Shape::Sphere(Sphere::new(0.5).expect("sphere")),
            transform(Vec3::new(x, 0.0, 0.0)),
        )
    };
    let left = world.add_collider(body, collider(-0.75)).unwrap();
    let right = world.add_collider(body, collider(0.75)).unwrap();

    world.step(0.01).expect("collision step");

    let expected = [
        CollisionPair::new(plane, left).unwrap(),
        CollisionPair::new(plane, right).unwrap(),
    ]
    .into_iter()
    .collect::<HashSet<_>>();
    let actual = world
        .collision_events()
        .iter()
        .filter(|event| event.kind == CollisionEventKind::Begin)
        .map(|event| event.pair)
        .collect::<HashSet<_>>();
    assert_eq!(actual, expected);
}

#[test]
fn joint_only_world_solves_connected_bodies_without_contacts() {
    let mut world = PhysicsWorld::new();
    let body_a = world.add_body(dynamic(Vec3::ZERO, Vec3::ZERO)).unwrap();
    let body_b = world
        .add_body(dynamic(Vec3::new(2.0, 0.0, 0.0), Vec3::ZERO))
        .unwrap();
    let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    world
        .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
        .unwrap();

    world.step(0.01).expect("joint-only step");

    let position_a = world
        .body(body_a)
        .unwrap()
        .state()
        .transform()
        .translation();
    let position_b = world
        .body(body_b)
        .unwrap()
        .state()
        .transform()
        .translation();
    assert!((position_b - position_a).length() < 1.0e-10);
}

#[test]
fn unconstrained_body_preserves_kinetic_energy_without_external_effects() {
    let mut world = PhysicsWorld::new();
    let mut body = dynamic(Vec3::ZERO, Vec3::new(3.0, -2.0, 1.0));
    body.set_angular_velocity(Vec3::new(0.5, -1.0, 2.0))
        .expect("angular velocity");
    let body_id = world.add_body(body).expect("free body");

    let initial = *world.body(body_id).expect("body").state();
    let initial_energy = 0.5
        * (initial.linear_velocity().length_squared()
            + initial.angular_velocity().length_squared());
    for _ in 0..120 {
        world.step(1.0 / 120.0).expect("free-body step");
    }

    let final_state = world.body(body_id).expect("body").state();
    let final_energy = 0.5
        * (final_state.linear_velocity().length_squared()
            + final_state.angular_velocity().length_squared());
    assert!((final_energy - initial_energy).abs() < 1.0e-12);
    assert_eq!(final_state.linear_velocity(), initial.linear_velocity());
    assert_eq!(final_state.angular_velocity(), initial.angular_velocity());
}

#[test]
fn fixed_joint_transmits_momentum_and_preserves_relative_transform() {
    let mut world = PhysicsWorld::new();
    let mut moving_body = dynamic(Vec3::ZERO, Vec3::X);
    moving_body
        .set_angular_velocity(Vec3::Z)
        .expect("angular velocity");
    let body_a = world.add_body(moving_body).expect("moving body");
    let body_b = world
        .add_body(dynamic(Vec3::ZERO, Vec3::ZERO))
        .expect("stationary body");
    let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    world
        .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
        .expect("fixed joint");

    for _ in 0..120 {
        world.step(1.0 / 120.0).expect("fixed-joint step");
    }

    let state_a = world.body(body_a).expect("body A").state();
    let state_b = world.body(body_b).expect("body B").state();
    let relative_position = state_b.transform().translation() - state_a.transform().translation();
    assert!(relative_position.length() < 1.0e-9);
    assert!((state_b.linear_velocity() - state_a.linear_velocity()).length() < 1.0e-9);
    assert!((state_a.linear_velocity().x + state_b.linear_velocity().x - 1.0).abs() < 1.0e-9);
    assert!((state_b.angular_velocity() - state_a.angular_velocity()).length() < 1.0e-9);
    assert!((state_a.angular_velocity().z + state_b.angular_velocity().z - 1.0).abs() < 1.0e-9);
}

#[test]
fn long_fixed_joint_chain_stays_finite_and_reproduces_identical_runs() {
    fn make_chain() -> (PhysicsWorld, Vec<kapitsa_core::world::BodyId>) {
        let mut world = PhysicsWorld::new();
        let bodies = (0..24)
            .map(|index| {
                let velocity = if index == 23 {
                    Vec3::Y * 0.1
                } else {
                    Vec3::ZERO
                };
                world
                    .add_body(dynamic(Vec3::X * index as f64, velocity))
                    .expect("chain body")
            })
            .collect::<Vec<_>>();
        let common = JointCommon::new(
            JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
            JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
        );
        for pair in bodies.windows(2) {
            world
                .add_joint(pair[0], pair[1], Joint::Fixed(FixedJoint::new(common)))
                .expect("chain joint");
        }
        (world, bodies)
    }

    let (mut first, first_bodies) = make_chain();
    let (mut second, second_bodies) = make_chain();
    for _ in 0..60 {
        first.step(0.01).expect("first chain step");
        second.step(0.01).expect("second chain step");
        for (first_id, second_id) in first_bodies.iter().zip(&second_bodies) {
            assert_eq!(
                first.body(*first_id).expect("first body").state(),
                second.body(*second_id).expect("second body").state()
            );
        }
    }

    let positions = first_bodies
        .iter()
        .map(|id| {
            let state = first.body(*id).expect("chain body").state();
            assert!(state.transform().translation().is_finite());
            assert!(state.linear_velocity().is_finite());
            assert!(state.angular_velocity().is_finite());
            state.transform().translation()
        })
        .collect::<Vec<_>>();
    for pair in positions.windows(2) {
        let distance = (pair[1] - pair[0]).length();
        assert!((distance - 1.0).abs() < 0.1, "link distance={distance}");
    }
}

#[test]
fn spring_oscillates_and_damping_reduces_motion() {
    fn simulate(damping: f64) -> (f64, f64, f64, f64, f64) {
        let mut world = PhysicsWorld::new();
        let anchor = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("spring anchor");
        let moving = world
            .add_body(dynamic(Vec3::X * 2.0, Vec3::ZERO))
            .expect("spring body");
        let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        world
            .add_joint(
                anchor,
                moving,
                Joint::Spring(SpringJoint::new(common, 1.0, 400.0, damping)),
            )
            .expect("spring joint");

        let mut minimum_distance = f64::INFINITY;
        let mut maximum_distance = 0.0_f64;
        let mut peak_speed = 0.0_f64;
        for _ in 0..1200 {
            world.step(0.005).expect("spring step");
            let state = world.body(moving).expect("spring body").state();
            let distance = state.transform().translation().length();
            minimum_distance = minimum_distance.min(distance);
            maximum_distance = maximum_distance.max(distance);
            peak_speed = peak_speed.max(state.linear_velocity().length());
        }
        let final_state = world.body(moving).expect("spring body").state();
        (
            minimum_distance,
            maximum_distance,
            final_state.transform().translation().length(),
            final_state.linear_velocity().length(),
            peak_speed,
        )
    }

    let undamped = simulate(0.0);
    assert!(undamped.0 < 0.9, "minimum distance={}", undamped.0);
    assert!(undamped.1 > 1.1, "maximum distance={}", undamped.1);

    let damped = simulate(40.0);
    assert!(damped.2 < 1.1, "final distance={}", damped.2);
    assert!(
        damped.3 < undamped.4 * 0.1,
        "damped final speed={} undamped peak={}",
        damped.3,
        undamped.4
    );
}

#[test]
fn joint_break_force_uses_solver_impulse_divided_by_step_dt() {
    let mut world = PhysicsWorld::new();
    let body_a = world.add_body(dynamic(Vec3::ZERO, Vec3::ZERO)).unwrap();
    let body_b = world.add_body(dynamic(Vec3::ZERO, Vec3::X * 10.0)).unwrap();
    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.break_force = Some(100.0);
    let joint = world
        .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
        .unwrap();

    world.step(0.01).expect("breaking step");

    assert_eq!(world.joint_count(), 0);
    assert_eq!(world.joint_events().len(), 1);
    let event = world.joint_events()[0];
    assert_eq!(event.joint(), joint);
    let impulse = world
        .body(body_a)
        .unwrap()
        .state()
        .linear_velocity()
        .length();
    assert!((event.force() - impulse / 0.01).abs() < 1.0e-8);
    assert!(event.torque() < 1.0e-8);

    world.step(0.01).expect("step after break");
    assert!(world.joint_events().is_empty());
}

#[test]
fn joint_break_torque_uses_angular_solver_impulse_divided_by_step_dt() {
    let mut world = PhysicsWorld::new();
    let body_a = dynamic(Vec3::ZERO, Vec3::ZERO);
    let mut body_b = dynamic(Vec3::ZERO, Vec3::ZERO);
    body_b
        .set_angular_velocity(Vec3::Z * 10.0)
        .expect("angular velocity");
    let body_a = world.add_body(body_a).unwrap();
    let body_b = world.add_body(body_b).unwrap();
    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.break_torque = Some(100.0);
    let joint = world
        .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
        .unwrap();

    world.step(0.01).expect("breaking step");

    assert_eq!(world.joint_count(), 0);
    assert_eq!(world.joint_events().len(), 1);
    let event = world.joint_events()[0];
    assert_eq!(event.joint(), joint);
    assert!(event.force() < 1.0e-8);
    let angular_impulse = world
        .body(body_a)
        .unwrap()
        .state()
        .angular_velocity()
        .length();
    assert!((event.torque() - angular_impulse / 0.01).abs() < 1.0e-8);
}

#[test]
fn disabled_joint_does_not_break_and_can_be_reenabled_before_breaking() {
    let mut world = PhysicsWorld::new();
    let body_a = world.add_body(dynamic(Vec3::ZERO, Vec3::ZERO)).unwrap();
    let body_b = world.add_body(dynamic(Vec3::ZERO, Vec3::X * 10.0)).unwrap();
    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.break_force = Some(100.0);
    common.enabled = false;
    let joint = world
        .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
        .unwrap();

    world.step(0.01).expect("disabled joint step");
    assert_eq!(world.joint_count(), 1);
    assert!(world.joint_events().is_empty());

    world.set_joint_enabled(joint, true).expect("enable joint");
    world.step(0.01).expect("enabled joint step");
    assert_eq!(world.joint_count(), 0);
    assert_eq!(world.joint_events().len(), 1);
    assert!(world.set_joint_enabled(joint, true).is_err());
}

#[test]
fn contact_and_joint_constraints_solve_together_and_respect_collision_suppression() {
    let mut world = PhysicsWorld::new();
    let body_a = world.add_body(dynamic(Vec3::ZERO, Vec3::ZERO)).unwrap();
    let body_b = world
        .add_body(dynamic(Vec3::new(2.0, 0.0, 0.0), Vec3::ZERO))
        .unwrap();
    let body_c = world
        .add_body(dynamic(Vec3::new(2.8, 0.0, 0.0), Vec3::ZERO))
        .unwrap();
    let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    world
        .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
        .unwrap();
    add_sphere(&mut world, body_b);
    add_sphere(&mut world, body_c);

    world.step(0.01).expect("combined contact and joint step");

    let position_a = world
        .body(body_a)
        .unwrap()
        .state()
        .transform()
        .translation();
    let position_b = world
        .body(body_b)
        .unwrap()
        .state()
        .transform()
        .translation();
    let position_c = world
        .body(body_c)
        .unwrap()
        .state()
        .transform()
        .translation();
    assert!((position_b - position_a).length() < 1.0e-10);
    assert!(position_c.x > 2.8);

    let mut suppressed_world = PhysicsWorld::new();
    let first = suppressed_world
        .add_body(dynamic(Vec3::ZERO, Vec3::ZERO))
        .unwrap();
    let second = suppressed_world
        .add_body(dynamic(Vec3::new(0.5, 0.0, 0.0), Vec3::ZERO))
        .unwrap();
    let first_collider = add_sphere(&mut suppressed_world, first);
    let second_collider = add_sphere(&mut suppressed_world, second);
    let joint_id = suppressed_world
        .add_joint(first, second, Joint::Fixed(FixedJoint::new(common)))
        .unwrap();

    suppressed_world
        .step(0.01)
        .expect("suppressed joint contact");
    assert!(suppressed_world.collision_events().is_empty());

    let mut collide_common = common;
    collide_common.collide_connected = true;
    suppressed_world
        .update_joint(joint_id, Joint::Fixed(FixedJoint::new(collide_common)))
        .unwrap();
    suppressed_world.step(0.01).expect("enabled joint contact");
    let pair = CollisionPair::new(first_collider, second_collider).unwrap();
    assert!(
        suppressed_world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Begin && event.pair == pair })
    );
}

#[test]
fn joint_solver_error_rolls_back_every_body_state() {
    let mut world = PhysicsWorld::new();
    let body_a = world
        .add_body(dynamic(Vec3::new(-f64::MAX, 0.0, 0.0), Vec3::ZERO))
        .unwrap();
    let body_b = world
        .add_body(dynamic(Vec3::new(f64::MAX, 0.0, 0.0), Vec3::ZERO))
        .unwrap();
    let before_a = *world.body(body_a).unwrap().state();
    let before_b = *world.body(body_b).unwrap().state();
    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.break_force = Some(1.0);
    let joint = world
        .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
        .unwrap();

    assert!(world.step(0.01).is_err());
    assert_eq!(*world.body(body_a).unwrap().state(), before_a);
    assert_eq!(*world.body(body_b).unwrap().state(), before_b);
    assert!(world.joint(joint).is_some());
    assert!(world.joint_events().is_empty());
}

#[test]
fn independent_joint_islands_match_sequential_and_parallel_execution() {
    let create_world = || {
        let mut world = PhysicsWorld::new();
        let bodies = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 13.0, 0.0),
        ]
        .map(|position| world.add_body(dynamic(position, Vec3::ZERO)).unwrap());
        let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        world
            .add_joint(bodies[0], bodies[1], Joint::Fixed(FixedJoint::new(common)))
            .unwrap();
        world
            .add_joint(bodies[2], bodies[3], Joint::Fixed(FixedJoint::new(common)))
            .unwrap();
        (world, bodies)
    };
    let (mut sequential, sequential_bodies) = create_world();
    let (mut parallel, parallel_bodies) = create_world();

    sequential
        .step_with_executor(0.01, &SequentialExecutor, 1)
        .expect("sequential joint islands");
    parallel
        .step_with_executor(0.01, &ScopedExecutor, 2)
        .expect("parallel joint islands");

    for (sequential_id, parallel_id) in sequential_bodies.into_iter().zip(parallel_bodies) {
        assert_eq!(
            sequential.body(sequential_id).unwrap().state(),
            parallel.body(parallel_id).unwrap().state()
        );
    }
}
