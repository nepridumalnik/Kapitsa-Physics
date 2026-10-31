use crate::{
    collider::{Collider, CollisionMode},
    collision::{CollisionConfig, CollisionError, CollisionEventKind, CollisionPair},
    mass::MassProperties,
    rigid_body::{CcdMode, RigidBody},
    shape::{
        BoneInfluence, Capsule, Cuboid, HeightField, LevelSet, Plane, Shape, SkinnedTriangleMesh,
        Sphere, TriangleMesh,
    },
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Affine3, Mat3, Quat, Vec2, Vec3};

fn dynamic_body(position: Vec3, velocity: Vec3) -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut body =
        RigidBody::new_dynamic(Transform::new(position, Quat::IDENTITY).unwrap(), mass).unwrap();
    body.set_linear_velocity(velocity).unwrap();
    body
}

fn add_plane(world: &mut PhysicsWorld, mode: CollisionMode) -> crate::world::ColliderId {
    add_plane_at(world, Vec3::Y, 0.0, mode)
}

fn add_plane_at(
    world: &mut PhysicsWorld,
    normal: Vec3,
    distance: f64,
    mode: CollisionMode,
) -> crate::world::ColliderId {
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let mut collider = Collider::new(
        Shape::Plane(Plane::from_vec(normal, distance).unwrap()),
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
fn world_step_resolves_contacts_against_triangle_mesh() {
    let mut world = PhysicsWorld::new();
    let mesh_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let mesh = TriangleMesh::new(
        vec![
            Vec3::new(-5.0, 0.0, -5.0),
            Vec3::new(5.0, 0.0, -5.0),
            Vec3::new(5.0, 0.0, 5.0),
            Vec3::new(-5.0, 0.0, 5.0),
        ],
        vec![[0, 1, 2], [0, 2, 3]],
    )
    .unwrap();
    let mesh_collider = world
        .add_collider(
            mesh_body,
            Collider::new(Shape::TriangleMesh(mesh), Transform::identity()),
        )
        .unwrap();
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
    let pair = CollisionPair::new(mesh_collider, sphere).unwrap();
    assert!(
        world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Begin && event.pair == pair })
    );
}

#[test]
fn world_step_resolves_contacts_against_height_field() {
    let mut world = PhysicsWorld::new();
    let field_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let field = HeightField::from_vec(2, 2, Vec2::new(10.0, 10.0), vec![0.0; 4]).unwrap();
    let rotation = Quat::from_axis_angle(Vec3::X, std::f64::consts::FRAC_PI_2).unwrap();
    let field_collider = world
        .add_collider(
            field_body,
            Collider::new(
                Shape::HeightField(field),
                Transform::new(Vec3::ZERO, rotation).unwrap(),
            ),
        )
        .unwrap();
    let ball = world
        .add_body(dynamic_body(
            Vec3::new(5.0, 0.6, 5.0),
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
    let pair = CollisionPair::new(field_collider, sphere).unwrap();
    assert!(
        world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Begin && event.pair == pair })
    );
}

#[test]
fn world_step_resolves_contacts_against_level_set() {
    let mut world = PhysicsWorld::new();
    let field_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let mut values = Vec::new();
    for _ in 0..2 {
        values.extend([-1.0, -1.0, 0.0, 0.0, 1.0, 1.0]);
    }
    let field =
        LevelSet::from_vec(Vec3::new(-1.0, -1.0, -1.0), [2, 3, 2], Vec3::ONE, values).unwrap();
    let field_collider = world
        .add_collider(
            field_body,
            Collider::new(Shape::LevelSet(field), Transform::identity()),
        )
        .unwrap();
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
    let pair = CollisionPair::new(field_collider, sphere).unwrap();
    assert!(
        world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Begin && event.pair == pair })
    );
}

#[test]
fn world_step_uses_the_current_skinned_mesh_pose() {
    let mut world = PhysicsWorld::new();
    let mesh_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let vertices = vec![
        Vec3::new(-5.0, 2.0, -5.0),
        Vec3::new(5.0, 2.0, -5.0),
        Vec3::new(0.0, 2.0, 5.0),
    ];
    let mesh = TriangleMesh::new(vertices, vec![[0, 1, 2]]).unwrap();
    let influences = (0..3)
        .map(|_| vec![BoneInfluence::new(0, 1.0).unwrap()])
        .collect();
    let skinned = SkinnedTriangleMesh::new(
        mesh,
        vec![String::from("root")],
        influences,
        Affine3::IDENTITY,
        vec![Affine3::IDENTITY],
    )
    .unwrap();
    let mesh_collider = world
        .add_collider(
            mesh_body,
            Collider::new(Shape::SkinnedTriangleMesh(skinned), Transform::identity()),
        )
        .unwrap();
    world
        .set_skin_pose(
            mesh_collider,
            &[Affine3::from_translation(Vec3::new(0.0, -2.0, 0.0))],
        )
        .unwrap();
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
    let pair = CollisionPair::new(mesh_collider, sphere).unwrap();
    assert!(
        world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Begin && event.pair == pair })
    );
}

#[test]
fn continuous_body_does_not_tunnel_through_a_plane() {
    let mut world = PhysicsWorld::new();
    let plane = add_plane(&mut world, CollisionMode::Solid);
    let mut body = dynamic_body(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1000.0, 0.0));
    body.set_ccd_mode(CcdMode::Continuous);
    let ball = world.add_body(body).unwrap();
    let sphere = world
        .add_collider(
            ball,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(0.01).unwrap();

    assert!(
        world
            .body(ball)
            .unwrap()
            .state()
            .transform()
            .translation()
            .y
            > 0.5
    );
    assert!(world.body(ball).unwrap().state().linear_velocity().y > 0.0);
    let pair = CollisionPair::new(plane, sphere).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Impact
            && event.pair == pair
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse > 0.0)
    }));
}

#[test]
fn continuous_sphere_hits_a_thin_cuboid() {
    let mut world = PhysicsWorld::new();
    let wall = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let wall_collider = world
        .add_collider(
            wall,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let mut body = dynamic_body(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(1000.0, 0.0, 0.0));
    body.set_ccd_mode(CcdMode::Continuous);
    let ball = world.add_body(body).unwrap();
    let sphere = world
        .add_collider(
            ball,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(0.01).unwrap();

    assert!(
        world
            .body(ball)
            .unwrap()
            .state()
            .transform()
            .translation()
            .x
            < 0.0
    );
    assert!(world.body(ball).unwrap().state().linear_velocity().x < 0.0);
    let pair = CollisionPair::new(wall_collider, sphere).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Impact
            && event.pair == pair
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse > 0.0)
    }));
}

#[test]
fn continuous_capsule_does_not_tunnel_through_a_plane() {
    let mut world = PhysicsWorld::new();
    let plane = add_plane(&mut world, CollisionMode::Solid);
    let mut body = dynamic_body(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1000.0, 0.0));
    body.set_ccd_mode(CcdMode::Continuous);
    let capsule_body = world.add_body(body).unwrap();
    let capsule = world
        .add_collider(
            capsule_body,
            Collider::new(
                Shape::Capsule(
                    Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5).unwrap(),
                ),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(0.01).unwrap();

    assert!(
        world
            .body(capsule_body)
            .unwrap()
            .state()
            .transform()
            .translation()
            .y
            > 1.5
    );
    assert!(
        world
            .body(capsule_body)
            .unwrap()
            .state()
            .linear_velocity()
            .y
            > 0.0
    );
    let pair = CollisionPair::new(plane, capsule).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Impact
            && event.pair == pair
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse > 0.0)
    }));
}

#[test]
fn continuous_body_resolves_relative_motion_against_a_discrete_dynamic_body() {
    let mut world = PhysicsWorld::new();
    let mut left_body = dynamic_body(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(1000.0, 0.0, 0.0));
    left_body.set_ccd_mode(CcdMode::Continuous);
    let left = world.add_body(left_body).unwrap();
    let right = world
        .add_body(dynamic_body(
            Vec3::new(5.0, 0.0, 0.0),
            Vec3::new(-1000.0, 0.0, 0.0),
        ))
        .unwrap();
    let shape = || {
        Collider::new(
            Shape::Sphere(Sphere::new(0.5).unwrap()),
            Transform::identity(),
        )
    };
    let left_collider = world.add_collider(left, shape()).unwrap();
    let right_collider = world.add_collider(right, shape()).unwrap();

    world.step(0.01).unwrap();

    let left_state = world.body(left).unwrap().state();
    let right_state = world.body(right).unwrap().state();
    assert!(left_state.transform().translation().x < right_state.transform().translation().x);
    assert!(left_state.linear_velocity().x < 0.0);
    assert!(right_state.linear_velocity().x > 0.0);
    assert!(left_state.linear_velocity().x.abs() < 1000.0);
    assert!(right_state.linear_velocity().x.abs() < 1000.0);
    let pair = CollisionPair::new(left_collider, right_collider).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Impact
            && event.pair == pair
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse > 0.0)
    }));
}

#[test]
fn continuous_sensor_reports_a_transient_crossing_without_response() {
    let mut world = PhysicsWorld::new();
    let plane = add_plane(&mut world, CollisionMode::Sensor);
    let mut body = dynamic_body(Vec3::new(0.0, 5.0, 0.0), Vec3::new(0.0, -1000.0, 0.0));
    body.set_ccd_mode(CcdMode::Continuous);
    let ball = world.add_body(body).unwrap();
    let sphere = world
        .add_collider(
            ball,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(0.01).unwrap();

    assert!(
        world
            .body(ball)
            .unwrap()
            .state()
            .transform()
            .translation()
            .y
            < 0.0
    );
    assert_eq!(
        world.body(ball).unwrap().state().linear_velocity().y,
        -1000.0
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
fn ccd_impact_limit_failure_preserves_the_world_state_and_events() {
    let mut world = PhysicsWorld::new();
    let floor = add_plane_at(&mut world, Vec3::Y, -1.0, CollisionMode::Solid);
    let ceiling = add_plane_at(&mut world, -Vec3::Y, -1.0, CollisionMode::Solid);
    let mut body = dynamic_body(Vec3::ZERO, Vec3::new(0.0, 3000.0, 0.0));
    body.set_ccd_mode(CcdMode::Continuous);
    let ball = world.add_body(body).unwrap();
    world
        .add_collider(
            ball,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let config = CollisionConfig {
        max_ccd_impacts: 1,
        ..world.collision_config()
    };
    world.set_collision_config(config).unwrap();
    let before_state = *world.body(ball).unwrap().state();
    let before_events = world.collision_events().to_vec();

    assert_eq!(
        world.step(0.01),
        Err(crate::world::StepError::Collision(
            CollisionError::CcdLimitExceeded
        ))
    );

    assert_eq!(*world.body(ball).unwrap().state(), before_state);
    assert_eq!(world.collision_events(), before_events);
    assert!(floor < ceiling);
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
