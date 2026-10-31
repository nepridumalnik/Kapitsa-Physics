use kapitsa_core::{
    collider::{Collider, CollisionMode},
    collision::{CollisionEventKind, CollisionPair},
    mass::MassProperties,
    material::PhysicsMaterial,
    rigid_body::{CcdMode, RigidBody},
    shape::{Cuboid, Plane, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};
use std::collections::HashSet;

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
