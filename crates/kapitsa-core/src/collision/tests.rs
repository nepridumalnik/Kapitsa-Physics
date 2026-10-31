use crate::{
    collision::{CollisionConfig, CollisionError, CollisionPair},
    world::PhysicsWorld,
};

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
