use crate::{
    collider::{Collider, CollisionMode},
    collision::{CollisionConfig, CollisionError, CollisionEventKind, CollisionPair},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Plane, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

fn dynamic_body(position: Vec3, velocity: Vec3) -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut body =
        RigidBody::new_dynamic(Transform::new(position, Quat::IDENTITY).unwrap(), mass).unwrap();
    body.set_linear_velocity(velocity).unwrap();
    body
}

fn add_plane(world: &mut PhysicsWorld, mode: CollisionMode) -> crate::world::ColliderId {
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let mut collider = Collider::new(
        Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).unwrap()),
        Transform::identity(),
    );
    collider.set_mode(mode);
    world.add_collider(body, collider).unwrap()
}

#[test]
fn default_config_matches_documented_values() {
    assert_eq!(
        CollisionConfig::default(),
        CollisionConfig {
            velocity_iterations: 8,
            position_iterations: 3,
            contact_offset: 0.01,
            penetration_slop: 0.001,
            restitution_threshold: 1.0,
            max_ccd_impacts: 8,
        }
    );
}

#[test]
fn config_rejects_invalid_values() {
    let defaults = CollisionConfig::default();
    assert_eq!(defaults.validate(), Ok(()));
    assert_eq!(
        CollisionConfig {
            velocity_iterations: 0,
            ..defaults
        }
        .validate(),
        Err(CollisionError::InvalidConfig)
    );
    assert_eq!(
        CollisionConfig {
            penetration_slop: f64::NAN,
            ..defaults
        }
        .validate(),
        Err(CollisionError::InvalidConfig)
    );
    assert_eq!(
        CollisionConfig {
            penetration_slop: defaults.contact_offset + 1.0,
            ..defaults
        }
        .validate(),
        Err(CollisionError::InvalidConfig)
    );
}

#[test]
fn world_keeps_previous_config_when_update_is_invalid() {
    let mut world = PhysicsWorld::new();
    let original = world.collision_config();
    let invalid = CollisionConfig {
        max_ccd_impacts: 0,
        ..original
    };
    assert_eq!(
        world.set_collision_config(invalid),
        Err(CollisionError::InvalidConfig)
    );
    assert_eq!(world.collision_config(), original);
    assert!(world.collision_events().is_empty());
}

#[test]
fn pair_constructor_is_canonical_and_rejects_self_pairs() {
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(crate::rigid_body::RigidBody::new_static(
            crate::transform::Transform::identity(),
        ))
        .unwrap();
    let first = world
        .add_collider(
            body,
            crate::collider::Collider::new(
                crate::shape::Shape::Sphere(crate::shape::Sphere::new(1.0).unwrap()),
                crate::transform::Transform::identity(),
            ),
        )
        .unwrap();
    let second = world
        .add_collider(
            body,
            crate::collider::Collider::new(
                crate::shape::Shape::Sphere(crate::shape::Sphere::new(1.0).unwrap()),
                crate::transform::Transform::identity(),
            ),
        )
        .unwrap();

    let forward = CollisionPair::new(first, second).unwrap();
    let reverse = CollisionPair::new(second, first).unwrap();
    assert_eq!(forward, reverse);
    assert!(forward.a < forward.b);
    assert_eq!(CollisionPair::new(first, first), None);
}

#[test]
fn world_step_resolves_solid_contacts_and_publishes_lifecycle_events() {
    let mut world = PhysicsWorld::new();
    let plane = add_plane(&mut world, CollisionMode::Solid);
    let ball = world
        .add_body(dynamic_body(
            Vec3::new(0.0, 0.6, 0.0),
            Vec3::new(0.0, -10.0, 0.0),
        ))
        .unwrap();
    let sphere = world
        .add_collider(
            ball,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(0.02).unwrap();

    assert!(world.body(ball).unwrap().state().linear_velocity().y > 0.0);
    assert!(
        world
            .body(ball)
            .unwrap()
            .state()
            .transform()
            .translation()
            .y
            >= 0.4
    );
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
            .any(|event| event.kind == CollisionEventKind::Impact && event.pair == pair)
    );

    world.step(0.01).unwrap();
    assert!(
        world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Persist && event.pair == pair })
    );

    world
        .body_mut(ball)
        .unwrap()
        .set_transform(Transform::new(Vec3::new(0.0, 10.0, 0.0), Quat::IDENTITY).unwrap())
        .unwrap();
    world.step(0.01).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::End && event.pair == pair && event.contact.is_none()
    }));
}

#[test]
fn world_step_reports_sensor_overlap_without_changing_motion() {
    let mut world = PhysicsWorld::new();
    let plane = add_plane(&mut world, CollisionMode::Sensor);
    let ball = world
        .add_body(dynamic_body(
            Vec3::new(0.0, 0.6, 0.0),
            Vec3::new(0.0, -10.0, 0.0),
        ))
        .unwrap();
    let sphere = world
        .add_collider(
            ball,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(0.02).unwrap();

    assert_eq!(
        world.body(ball).unwrap().state().linear_velocity(),
        Vec3::new(0.0, -10.0, 0.0)
    );
    assert!(
        (world
            .body(ball)
            .unwrap()
            .state()
            .transform()
            .translation()
            .y
            - 0.4)
            .abs()
            < 1.0e-12
    );
    let pair = CollisionPair::new(plane, sphere).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Begin
            && event.pair == pair
            && event.sensor
            && event.contact.is_none()
    }));
    assert!(
        !world
            .collision_events()
            .iter()
            .any(|event| event.kind == CollisionEventKind::Impact && event.pair == pair)
    );
}

#[test]
fn sharded_world_step_matches_sequential_collision_results() {
    let make_world = || {
        let mut world = PhysicsWorld::new();
        add_plane(&mut world, CollisionMode::Solid);
        for x in [-2.0, 2.0] {
            let ball = world
                .add_body(dynamic_body(
                    Vec3::new(x, 0.6, 0.0),
                    Vec3::new(0.0, -10.0, 0.0),
                ))
                .unwrap();
            world
                .add_collider(
                    ball,
                    Collider::new(
                        Shape::Sphere(Sphere::new(0.5).unwrap()),
                        Transform::identity(),
                    ),
                )
                .unwrap();
        }
        world
    };

    let mut sequential = make_world();
    let mut sharded = make_world();
    sequential.step(0.02).unwrap();

    let mut shards = vec![Vec::new(), Vec::new()];
    sharded.fill_step_shards(&mut shards).unwrap();
    for shard in &mut shards {
        for body_step in shard {
            body_step.integrate(0.02).unwrap();
        }
    }
    sharded.commit_step_shards(&shards).unwrap();

    for body_index in [1, 2] {
        let sequential_body = sequential
            .body(sequential.bodies().nth(body_index).unwrap().0)
            .unwrap();
        let sharded_body = sharded
            .body(sharded.bodies().nth(body_index).unwrap().0)
            .unwrap();
        assert_eq!(
            sequential_body.state().transform(),
            sharded_body.state().transform()
        );
        assert_eq!(
            sequential_body.state().linear_velocity(),
            sharded_body.state().linear_velocity()
        );
    }
    assert_eq!(
        sequential.collision_events().len(),
        sharded.collision_events().len()
    );
    for (sequential, sharded) in sequential
        .collision_events()
        .iter()
        .zip(sharded.collision_events())
    {
        assert_eq!(sequential.kind, sharded.kind);
        assert_eq!(sequential.sensor, sharded.sensor);
        assert_eq!(sequential.contact, sharded.contact);
    }
}
