use crate::{
    collider::{Collider, CollisionMode},
    collision::{
        CcdLimitPolicy, CollisionConfig, CollisionError, CollisionEventKind, CollisionPair,
    },
    mass::MassProperties,
    material::PhysicsMaterial,
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
            ccd_limit_policy: CcdLimitPolicy::ClipIsland,
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
fn world_step_resolves_overlapping_dynamic_spheres() {
    let mut world = PhysicsWorld::new();
    let make_collider = || {
        Collider::new(
            Shape::Sphere(Sphere::new(0.5).unwrap()),
            Transform::identity(),
        )
    };
    let left = world
        .add_body(dynamic_body(
            Vec3::new(-0.45, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        ))
        .unwrap();
    let left_collider = world.add_collider(left, make_collider()).unwrap();
    let right = world
        .add_body(dynamic_body(
            Vec3::new(0.45, 0.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
        ))
        .unwrap();
    let right_collider = world.add_collider(right, make_collider()).unwrap();

    world.step(1.0 / 60.0).unwrap();

    let pair = CollisionPair::new(left_collider, right_collider).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Impact
            && event.pair == pair
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse > 0.0)
    }));
    assert!(world.body(left).unwrap().state().linear_velocity().x < 1.0);
    assert!(world.body(right).unwrap().state().linear_velocity().x > -1.0);
}

#[test]
fn continuous_dynamic_spheres_resolve_relative_toi() {
    let mut world = PhysicsWorld::new();
    let make_collider = || {
        Collider::new(
            Shape::Sphere(Sphere::new(0.5).unwrap()),
            Transform::identity(),
        )
    };
    let mut left_body = dynamic_body(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(1000.0, 0.0, 0.0));
    left_body.set_ccd_mode(CcdMode::Continuous);
    let left = world.add_body(left_body).unwrap();
    let left_collider = world.add_collider(left, make_collider()).unwrap();
    let mut right_body = dynamic_body(Vec3::new(5.0, 0.0, 0.0), Vec3::new(-1000.0, 0.0, 0.0));
    right_body.set_ccd_mode(CcdMode::Continuous);
    let right = world.add_body(right_body).unwrap();
    let right_collider = world.add_collider(right, make_collider()).unwrap();

    world.step(0.01).unwrap();

    assert!(world.body(left).unwrap().state().linear_velocity().x < 0.0);
    assert!(world.body(right).unwrap().state().linear_velocity().x > 0.0);
    let pair = CollisionPair::new(left_collider, right_collider).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Impact
            && event.pair == pair
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse > 0.0)
    }));
}

#[cfg(feature = "bench-internals")]
#[test]
fn independent_ccd_islands_evaluate_only_their_own_candidates() {
    let mut world = PhysicsWorld::new();
    let wall = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    world
        .add_collider(
            wall,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 20.0, 20.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    for y in [-2.0, 2.0] {
        let mut body = dynamic_body(Vec3::new(-5.0, y, 0.0), Vec3::new(1000.0, 0.0, 0.0));
        body.set_ccd_mode(CcdMode::Continuous);
        let body = world.add_body(body).unwrap();
        world
            .add_collider(
                body,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.5).unwrap()),
                    Transform::identity(),
                ),
            )
            .unwrap();
    }

    world.step(1.0 / 60.0).unwrap();
    let metrics = world.collision_step_metrics();
    assert_eq!(metrics.ccd_island_count, 2);
    assert_eq!(metrics.ccd_initial_candidate_count, 2);
    assert_eq!(metrics.ccd_toi_call_count, 2);
    assert_eq!(metrics.ccd_candidate_refresh_count, 0);
    assert_eq!(metrics.ccd_impact_count, 2);
}

#[cfg(feature = "bench-internals")]
#[test]
fn ccd_resweep_only_visits_live_colliders_attached_to_the_changed_body() {
    for collider_count in [1, 10, 100, 1_000] {
        let mut world = PhysicsWorld::new();
        let wall_body = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        world
            .add_collider(
                wall_body,
                Collider::new(
                    Shape::Cuboid(Cuboid::new(0.1, 2.0, 2.0).unwrap()),
                    Transform::identity(),
                ),
            )
            .unwrap();

        let mut moving = dynamic_body(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(1_000.0, 0.0, 0.0));
        moving.set_ccd_mode(CcdMode::Continuous);
        let moving_body = world.add_body(moving).unwrap();
        let mut component_ids = Vec::with_capacity(collider_count);
        for index in 0..collider_count {
            let local_y = index as f64 * 4.0;
            component_ids.push(
                world
                    .add_collider(
                        moving_body,
                        Collider::new(
                            Shape::Sphere(Sphere::new(0.5).unwrap()),
                            Transform::new(Vec3::new(0.0, local_y, 0.0), Quat::IDENTITY).unwrap(),
                        ),
                    )
                    .unwrap(),
            );
        }
        let replaced_local_y = (collider_count - 1) as f64 * 4.0;
        assert!(
            world
                .remove_collider(*component_ids.last().unwrap())
                .is_some()
        );
        world
            .add_collider(
                moving_body,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.5).unwrap()),
                    Transform::new(Vec3::new(0.0, replaced_local_y, 0.0), Quat::IDENTITY).unwrap(),
                ),
            )
            .unwrap();

        let unrelated_body = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        for index in 0..64 {
            world
                .add_collider(
                    unrelated_body,
                    Collider::new(
                        Shape::Sphere(Sphere::new(0.5).unwrap()),
                        Transform::new(
                            Vec3::new(1_000.0 + index as f64 * 3.0, 2_000.0, 0.0),
                            Quat::IDENTITY,
                        )
                        .unwrap(),
                    ),
                )
                .unwrap();
        }

        world.step(1.0 / 60.0).unwrap();

        let metrics = world.collision_step_metrics();
        assert_eq!(
            metrics.ccd_impact_count, 1,
            "collider count {collider_count}"
        );
        assert_eq!(
            metrics.ccd_resweep_count, 1,
            "collider count {collider_count}"
        );
        assert_eq!(
            metrics.ccd_resweep_colliders_visited, collider_count,
            "collider count {collider_count}"
        );
        assert_eq!(
            metrics.ccd_resweep_proxies_updated, collider_count,
            "collider count {collider_count}"
        );
        assert_eq!(
            world.colliders_of(moving_body).unwrap().count(),
            collider_count,
            "collider count {collider_count}"
        );
    }
}

#[test]
fn expensive_independent_ccd_islands_match_across_worker_counts() {
    let make_world = || {
        let mut world = PhysicsWorld::new();
        let wall = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let columns = 16;
        let wall_side = columns as f64 * 8.0 + 4.0;
        world
            .add_collider(
                wall,
                Collider::new(
                    Shape::Cuboid(Cuboid::new(0.1, wall_side, wall_side).unwrap()),
                    Transform::identity(),
                ),
            )
            .unwrap();
        for index in 0..columns * columns {
            let y = (index % columns) as f64 * 4.0;
            let z = (index / columns) as f64 * 4.0;
            let mut body = dynamic_body(Vec3::new(-5.0, y, z), Vec3::new(1000.0, 0.0, 0.0));
            body.set_ccd_mode(CcdMode::Continuous);
            let body = world.add_body(body).unwrap();
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
        world
    };
    let dt = 1.0 / 60.0;
    let mut expected = make_world();
    expected.step(dt).unwrap();

    for worker_count in [1, 2, 4] {
        let mut actual = make_world();
        let mut shards = (0..worker_count).map(|_| Vec::new()).collect::<Vec<_>>();
        actual.fill_step_shards(&mut shards).unwrap();
        for shard in &mut shards {
            for body_step in shard {
                body_step.integrate(dt).unwrap();
            }
        }
        actual
            .commit_step_shards_with_workers(&shards, worker_count)
            .unwrap();

        for ((_, actual_body), (_, expected_body)) in actual.bodies().zip(expected.bodies()) {
            assert_eq!(actual_body.state(), expected_body.state());
        }
        assert_eq!(
            actual.collision_events().len(),
            expected.collision_events().len()
        );
        for (actual, expected) in actual
            .collision_events()
            .iter()
            .zip(expected.collision_events())
        {
            assert_eq!(actual.kind, expected.kind);
            assert_eq!(actual.sensor, expected.sensor);
            assert_eq!(actual.contact, expected.contact);
        }
    }
}

#[test]
fn continuous_rebound_refreshes_candidates_for_the_remaining_step() {
    let mut world = PhysicsWorld::new();
    let material = world
        .add_material(PhysicsMaterial::new(0.0, 0.0, 1.0, 1.0).unwrap())
        .unwrap();

    let wall_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let wall = world
        .add_collider(
            wall_body,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    world.set_collider_material(wall, Some(material)).unwrap();

    let target_body = world
        .add_body(RigidBody::new_static(
            Transform::new(Vec3::new(15.0, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        ))
        .unwrap();
    let target = world
        .add_collider(
            target_body,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    world.set_collider_material(target, Some(material)).unwrap();

    let mut body = dynamic_body(Vec3::new(5.0, 0.0, 0.0), Vec3::new(-2000.0, 0.0, 0.0));
    body.set_ccd_mode(CcdMode::Continuous);
    let dynamic = world.add_body(body).unwrap();
    let sphere = world
        .add_collider(
            dynamic,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    world.set_collider_material(sphere, Some(material)).unwrap();

    world.step(0.01).unwrap();

    let wall_pair = CollisionPair::new(wall, sphere).unwrap();
    let target_pair = CollisionPair::new(target, sphere).unwrap();
    for pair in [wall_pair, target_pair] {
        assert!(world.collision_events().iter().any(|event| {
            event.kind == CollisionEventKind::Impact
                && event.pair == pair
                && event
                    .contact
                    .is_some_and(|contact| contact.normal_impulse > 0.0)
        }));
    }
    assert_eq!(
        world
            .collision_events()
            .iter()
            .filter(|event| event.kind == CollisionEventKind::Impact)
            .count(),
        2
    );
    let state = world.body(dynamic).unwrap().state();
    assert!(state.transform().translation().x < 15.0);
    assert!(state.linear_velocity().x < 0.0);
    #[cfg(feature = "bench-internals")]
    {
        let metrics = world.collision_step_metrics();
        assert_eq!(metrics.ccd_initial_candidate_count, 1);
        assert_eq!(metrics.ccd_final_candidate_count, 2);
        assert_eq!(metrics.ccd_new_candidate_count, 1);
        assert_eq!(metrics.ccd_global_fallback_count, 0);
    }
}

#[cfg(feature = "bench-internals")]
#[test]
fn continuous_rebound_reschedules_a_new_cross_island_pair() {
    let mut world = PhysicsWorld::new();
    let material = world
        .add_material(PhysicsMaterial::new(0.0, 0.0, 1.0, 1.0).unwrap())
        .unwrap();

    let wall_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let wall = world
        .add_collider(
            wall_body,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    let mut first = dynamic_body(Vec3::new(5.0, 0.0, 0.0), Vec3::new(-2000.0, 0.0, 0.0));
    first.set_ccd_mode(CcdMode::Continuous);
    let first = world.add_body(first).unwrap();
    let first_collider = world
        .add_collider(
            first,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    let mut second = dynamic_body(Vec3::new(13.0, 0.0, 0.0), Vec3::ZERO);
    second.set_ccd_mode(CcdMode::Continuous);
    let second = world.add_body(second).unwrap();
    let second_collider = world
        .add_collider(
            second,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    for collider in [wall, first_collider, second_collider] {
        world
            .set_collider_material(collider, Some(material))
            .unwrap();
    }

    world.step(0.01).unwrap();

    let wall_pair = CollisionPair::new(wall, first_collider).unwrap();
    let dynamic_pair = CollisionPair::new(first_collider, second_collider).unwrap();
    for pair in [wall_pair, dynamic_pair] {
        assert!(world.collision_events().iter().any(|event| {
            event.kind == CollisionEventKind::Impact
                && event.pair == pair
                && event
                    .contact
                    .is_some_and(|contact| contact.normal_impulse > 0.0)
        }));
    }

    let metrics = world.collision_step_metrics();
    assert_eq!(metrics.ccd_initial_candidate_count, 1);
    assert_eq!(metrics.ccd_final_candidate_count, 2);
    assert_eq!(metrics.ccd_new_candidate_count, 1);
    assert_eq!(metrics.ccd_global_fallback_count, 1);
    assert_eq!(metrics.ccd_impact_count, 2);
    assert_eq!(metrics.ccd_island_count, 1);
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
fn collision_mode_changes_restart_the_pair_lifecycle_and_failed_steps_preserve_events() {
    let mut world = PhysicsWorld::new();
    let plane = add_plane(&mut world, CollisionMode::Solid);
    let ball = world
        .add_body(dynamic_body(Vec3::new(0.0, 0.4, 0.0), Vec3::ZERO))
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
    let pair = CollisionPair::new(plane, sphere).unwrap();

    world.step(1.0 / 60.0).unwrap();
    assert_eq!(world.collision_events().len(), 1);
    assert_eq!(world.collision_events()[0].kind, CollisionEventKind::Begin);
    assert!(!world.collision_events()[0].sensor);
    let committed_events = world.collision_events().to_vec();

    assert_eq!(
        world.step(0.0),
        Err(crate::world::StepError::InvalidDeltaTime)
    );
    assert_eq!(world.collision_events(), committed_events);

    world.step(1.0 / 60.0).unwrap();
    assert_eq!(world.collision_events().len(), 1);
    assert_eq!(
        world.collision_events()[0].kind,
        CollisionEventKind::Persist
    );
    assert!(!world.collision_events()[0].sensor);

    world
        .collider_mut(plane)
        .unwrap()
        .set_mode(CollisionMode::Sensor);
    let body = world.body_mut(ball).unwrap();
    body.set_transform(Transform::new(Vec3::new(0.0, 0.4, 0.0), Quat::IDENTITY).unwrap())
        .unwrap();
    body.set_linear_velocity(Vec3::ZERO).unwrap();
    world.step(1.0 / 60.0).unwrap();
    let events = world
        .collision_events()
        .iter()
        .filter(|event| event.pair == pair)
        .collect::<Vec<_>>();
    assert_eq!(events.len(), 2);
    assert_eq!(events[0].kind, CollisionEventKind::End);
    assert!(!events[0].sensor);
    assert_eq!(events[1].kind, CollisionEventKind::Begin);
    assert!(events[1].sensor);

    world
        .collider_mut(plane)
        .unwrap()
        .set_mode(CollisionMode::Solid);
    let body = world.body_mut(ball).unwrap();
    body.set_transform(Transform::new(Vec3::new(0.0, 0.4, 0.0), Quat::IDENTITY).unwrap())
        .unwrap();
    body.set_linear_velocity(Vec3::ZERO).unwrap();
    world.step(1.0 / 60.0).unwrap();
    let events = world
        .collision_events()
        .iter()
        .filter(|event| event.pair == pair)
        .collect::<Vec<_>>();
    assert_eq!(events.len(), 2);
    assert_eq!(events[0].kind, CollisionEventKind::End);
    assert!(events[0].sensor);
    assert_eq!(events[1].kind, CollisionEventKind::Begin);
    assert!(!events[1].sensor);
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
fn continuous_pure_rotation_detects_a_full_turn_between_equal_endpoint_poses() {
    let mut world = PhysicsWorld::new();
    let target_body = world
        .add_body(RigidBody::new_static(
            Transform::new(Vec3::new(0.0, 1.5, 0.0), Quat::IDENTITY).unwrap(),
        ))
        .unwrap();
    let target = Collider::new(
        Shape::Cuboid(Cuboid::new(4.0, 0.1, 4.0).unwrap()),
        Transform::identity(),
    );
    let target = world.add_collider(target_body, target).unwrap();

    let mut rod = dynamic_body(Vec3::ZERO, Vec3::ZERO);
    rod.set_angular_velocity(Vec3::Z * std::f64::consts::TAU)
        .unwrap();
    rod.set_ccd_mode(CcdMode::Continuous);
    let rod_body = world.add_body(rod).unwrap();
    let rod_collider = world
        .add_collider(
            rod_body,
            Collider::new(
                Shape::Cuboid(Cuboid::new(4.0, 0.2, 0.2).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(1.0).unwrap();

    let pair = CollisionPair::new(target, rod_collider).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Impact && event.pair == pair && !event.sensor
    }));
    assert!(
        world
            .body(rod_body)
            .unwrap()
            .state()
            .angular_velocity()
            .length()
            < std::f64::consts::TAU
    );
}

#[test]
fn continuous_rotation_with_offset_center_of_mass_sweeps_the_collider_arc() {
    let mut world = PhysicsWorld::new();
    let arc_radius = 5.0;
    let target_center = Vec3::new(
        arc_radius * (1.0 - std::f64::consts::FRAC_1_SQRT_2),
        -arc_radius * std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    );
    let target_body = world
        .add_body(RigidBody::new_static(
            Transform::new(target_center, Quat::IDENTITY).unwrap(),
        ))
        .unwrap();
    let target = world
        .add_collider(
            target_body,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.5, 0.5, 0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    let mass = MassProperties::new(1.0, Vec3::X * arc_radius, Mat3::IDENTITY).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_angular_velocity(Vec3::Z * std::f64::consts::FRAC_PI_2)
        .unwrap();
    body.set_ccd_mode(CcdMode::Continuous);
    let body = world.add_body(body).unwrap();
    let collider = world
        .add_collider(
            body,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(1.0).unwrap();

    let pair = CollisionPair::new(target, collider).unwrap();
    assert!(
        world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Impact && event.pair == pair })
    );
    let state = world.body(body).unwrap().state();
    assert!(state.transform().translation().is_finite());
    assert!(state.angular_velocity().is_finite());
}

#[test]
fn continuous_dynamic_body_responds_to_a_moving_kinematic_cuboid() {
    let mut world = PhysicsWorld::new();
    let mut kinematic =
        RigidBody::new_kinematic(Transform::new(Vec3::new(5.0, 0.0, 0.0), Quat::IDENTITY).unwrap());
    kinematic
        .set_linear_velocity(Vec3::new(-1000.0, 0.0, 0.0))
        .unwrap();
    let kinematic = world.add_body(kinematic).unwrap();
    let wall = world
        .add_collider(
            kinematic,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 2.0, 2.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    let mut dynamic = dynamic_body(Vec3::ZERO, Vec3::ZERO);
    dynamic.set_ccd_mode(CcdMode::Continuous);
    let dynamic = world.add_body(dynamic).unwrap();
    let sphere = world
        .add_collider(
            dynamic,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    world.step(0.01).unwrap();

    let pair = CollisionPair::new(wall, sphere).unwrap();
    assert!(
        world
            .collision_events()
            .iter()
            .any(|event| { event.kind == CollisionEventKind::Impact && event.pair == pair })
    );
    assert_eq!(
        world
            .body(kinematic)
            .unwrap()
            .state()
            .transform()
            .translation()
            .x,
        -5.0
    );
    let state = world.body(dynamic).unwrap().state();
    assert!(state.linear_velocity().x < 0.0);
    assert!(state.transform().translation().is_finite());
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
    let pair_events = world
        .collision_events()
        .iter()
        .filter(|event| event.pair == pair)
        .collect::<Vec<_>>();
    assert_eq!(pair_events.len(), 2);
    assert_eq!(pair_events[0].kind, CollisionEventKind::Begin);
    assert_eq!(pair_events[1].kind, CollisionEventKind::End);
    assert!(
        pair_events
            .iter()
            .all(|event| event.sensor && event.contact.is_none())
    );
    assert!(
        !world
            .collision_events()
            .iter()
            .any(|event| event.kind == CollisionEventKind::Impact && event.pair == pair)
    );

    world
        .body_mut(ball)
        .unwrap()
        .set_linear_velocity(Vec3::ZERO)
        .unwrap();
    world.step(0.01).unwrap();
    assert!(world.collision_events().is_empty());
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
        ccd_limit_policy: CcdLimitPolicy::AbortStep,
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
fn ccd_impact_limit_clips_the_connected_island_at_the_contact() {
    let mut world = PhysicsWorld::new();
    add_plane_at(&mut world, Vec3::Y, -1.0, CollisionMode::Solid);
    add_plane_at(&mut world, -Vec3::Y, -1.0, CollisionMode::Solid);
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
    world
        .set_collision_config(CollisionConfig {
            max_ccd_impacts: 1,
            ..world.collision_config()
        })
        .unwrap();

    world.step(0.01).unwrap();

    let state = world.body(ball).unwrap().state();
    assert!(state.transform().translation().y >= -0.500_001);
    assert!(state.transform().translation().y <= -0.499_999);
    assert_eq!(state.linear_velocity(), Vec3::ZERO);
    assert_eq!(state.angular_velocity(), Vec3::ZERO);
    assert!(world.collision_events().iter().any(|event| {
        event.kind == CollisionEventKind::Begin
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse == 0.0)
    }));
    #[cfg(feature = "bench-internals")]
    assert_eq!(world.collision_step_metrics().ccd_clipped_island_count, 1);
}

#[cfg(feature = "bench-internals")]
#[test]
fn ccd_budget_clips_only_the_contacting_body_in_a_hundred_body_island() {
    let mut world = PhysicsWorld::new();
    add_plane_at(&mut world, Vec3::Y, -1.0, CollisionMode::Solid);
    add_plane_at(&mut world, -Vec3::Y, -1.0, CollisionMode::Solid);
    let mut bodies = Vec::with_capacity(100);
    for index in 0..100 {
        let velocity = if index == 0 {
            Vec3::new(0.0, 3_000.0, 0.0)
        } else if index == 99 {
            Vec3::new(10.0, 0.0, 0.0)
        } else {
            Vec3::ZERO
        };
        let mut body = dynamic_body(Vec3::ZERO, velocity);
        body.set_ccd_mode(CcdMode::Continuous);
        let body_id = world.add_body(body).unwrap();
        let mut sensor = Collider::new(
            Shape::Sphere(Sphere::new(0.5).unwrap()),
            Transform::identity(),
        );
        sensor.set_mode(CollisionMode::Sensor);
        world.add_collider(body_id, sensor).unwrap();
        bodies.push(body_id);
    }
    world
        .add_collider(
            bodies[0],
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    world
        .set_collision_config(CollisionConfig {
            max_ccd_impacts: 1,
            ..world.collision_config()
        })
        .unwrap();

    world.step(0.01).unwrap();

    let clipped = world.body(bodies[0]).unwrap().state();
    assert_eq!(clipped.linear_velocity(), Vec3::ZERO);
    assert_eq!(clipped.angular_velocity(), Vec3::ZERO);
    assert!(clipped.transform().translation().y < 0.0);
    let continuing = world.body(bodies[99]).unwrap().state();
    assert_eq!(continuing.linear_velocity(), Vec3::new(10.0, 0.0, 0.0));
    assert_eq!(continuing.transform().translation().x, 0.1);

    let metrics = world.collision_step_metrics();
    assert_eq!(metrics.ccd_island_count, 1);
    assert_eq!(metrics.ccd_clipped_island_count, 1);
    assert_eq!(metrics.ccd_clipped_body_count, 1);
}

#[cfg(feature = "bench-internals")]
#[test]
fn two_ccd_bodies_with_exhausted_budgets_clip_at_their_shared_contact() {
    let mut world = PhysicsWorld::new();
    let material = world
        .add_material(PhysicsMaterial::new(0.0, 0.0, 1.0, 1.0).unwrap())
        .unwrap();
    let left_wall_body = world
        .add_body(RigidBody::new_static(
            Transform::new(Vec3::new(-8.0, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        ))
        .unwrap();
    let left_wall = world
        .add_collider(
            left_wall_body,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let right_wall_body = world
        .add_body(RigidBody::new_static(
            Transform::new(Vec3::new(8.0, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        ))
        .unwrap();
    let right_wall = world
        .add_collider(
            right_wall_body,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();

    let mut left_body = dynamic_body(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(-1_000.0, 0.0, 0.0));
    left_body.set_ccd_mode(CcdMode::Continuous);
    let left = world.add_body(left_body).unwrap();
    let left_collider = world
        .add_collider(
            left,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let mut right_body = dynamic_body(Vec3::new(5.0, 0.0, 0.0), Vec3::new(1_000.0, 0.0, 0.0));
    right_body.set_ccd_mode(CcdMode::Continuous);
    let right = world.add_body(right_body).unwrap();
    let right_collider = world
        .add_collider(
            right,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    for collider in [left_wall, right_wall, left_collider, right_collider] {
        world
            .set_collider_material(collider, Some(material))
            .unwrap();
    }
    world
        .set_collision_config(CollisionConfig {
            max_ccd_impacts: 1,
            ..world.collision_config()
        })
        .unwrap();

    world.step(0.02).unwrap();

    for body in [left, right] {
        let state = world.body(body).unwrap().state();
        assert_eq!(state.linear_velocity(), Vec3::ZERO);
        assert_eq!(state.angular_velocity(), Vec3::ZERO);
    }
    let pair = CollisionPair::new(left_collider, right_collider).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.pair == pair
            && event.kind == CollisionEventKind::Begin
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse == 0.0)
    }));
    assert_eq!(world.collision_step_metrics().ccd_clipped_body_count, 2);
}

#[cfg(feature = "bench-internals")]
#[test]
fn ccd_budget_clip_keeps_moving_kinematic_trajectories() {
    let mut world = PhysicsWorld::new();
    let mut kinematic_bodies = Vec::new();
    let mut kinematic_colliders = Vec::new();
    for (x, velocity) in [(-3.0, Vec3::X * 1_000.0), (3.0, -Vec3::X * 1_000.0)] {
        let mut body = RigidBody::new_kinematic(
            Transform::new(Vec3::new(x, 0.0, 0.0), Quat::IDENTITY).unwrap(),
        );
        body.set_linear_velocity(velocity).unwrap();
        let body_id = world.add_body(body).unwrap();
        let collider_id = world
            .add_collider(
                body_id,
                Collider::new(
                    Shape::Cuboid(Cuboid::new(0.1, 2.0, 2.0).unwrap()),
                    Transform::identity(),
                ),
            )
            .unwrap();
        kinematic_bodies.push(body_id);
        kinematic_colliders.push(collider_id);
    }
    let mut dynamic = dynamic_body(Vec3::ZERO, Vec3::ZERO);
    dynamic.set_ccd_mode(CcdMode::Continuous);
    let dynamic_body = world.add_body(dynamic).unwrap();
    let dynamic_collider = world
        .add_collider(
            dynamic_body,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    world
        .set_collision_config(CollisionConfig {
            max_ccd_impacts: 1,
            ..world.collision_config()
        })
        .unwrap();

    world.step(0.01).unwrap();

    for (body, expected_x) in kinematic_bodies.into_iter().zip([7.0, -7.0]) {
        assert_eq!(
            world
                .body(body)
                .unwrap()
                .state()
                .transform()
                .translation()
                .x,
            expected_x
        );
    }
    let dynamic = world.body(dynamic_body).unwrap().state();
    assert_eq!(dynamic.linear_velocity(), Vec3::ZERO);
    assert!(dynamic.transform().translation().is_finite());
    assert!(kinematic_colliders.into_iter().any(|kinematic| {
        let pair = CollisionPair::new(kinematic, dynamic_collider).unwrap();
        world.collision_events().iter().any(|event| {
            event.pair == pair
                && event.kind == CollisionEventKind::Begin
                && event
                    .contact
                    .is_some_and(|contact| contact.normal_impulse == 0.0)
        })
    }));
    assert_eq!(world.collision_step_metrics().ccd_clipped_body_count, 1);
}

#[test]
fn ccd_budget_clips_a_newly_connected_dynamic_pair() {
    let mut world = PhysicsWorld::new();
    let material = world
        .add_material(PhysicsMaterial::new(0.0, 0.0, 1.0, 1.0).unwrap())
        .unwrap();
    let wall_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let wall = world
        .add_collider(
            wall_body,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let mut first_body = dynamic_body(Vec3::new(5.0, 0.0, 0.0), Vec3::new(-2000.0, 0.0, 0.0));
    first_body.set_ccd_mode(CcdMode::Continuous);
    let first = world.add_body(first_body).unwrap();
    let first_collider = world
        .add_collider(
            first,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let mut second_body = dynamic_body(Vec3::new(13.0, 0.0, 0.0), Vec3::ZERO);
    second_body.set_ccd_mode(CcdMode::Continuous);
    let second = world.add_body(second_body).unwrap();
    let second_collider = world
        .add_collider(
            second,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    for collider in [wall, first_collider, second_collider] {
        world
            .set_collider_material(collider, Some(material))
            .unwrap();
    }
    world
        .set_collision_config(CollisionConfig {
            max_ccd_impacts: 1,
            ..world.collision_config()
        })
        .unwrap();

    world.step(0.01).unwrap();

    for body in [first, second] {
        let state = world.body(body).unwrap().state();
        assert_eq!(state.linear_velocity(), Vec3::ZERO);
        assert_eq!(state.angular_velocity(), Vec3::ZERO);
    }
    let pair = CollisionPair::new(first_collider, second_collider).unwrap();
    assert!(world.collision_events().iter().any(|event| {
        event.pair == pair
            && event.kind == CollisionEventKind::Begin
            && event
                .contact
                .is_some_and(|contact| contact.normal_impulse == 0.0)
    }));
    #[cfg(feature = "bench-internals")]
    assert_eq!(world.collision_step_metrics().ccd_clipped_island_count, 1);
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
