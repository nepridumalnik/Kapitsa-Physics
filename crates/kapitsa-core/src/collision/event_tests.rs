use crate::{
    collider::Collider,
    collision::{
        CollisionEventKind, CollisionInteraction, CollisionLifecycle, CollisionPair, ContactInfo,
    },
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::Vec3;

fn pairs() -> [CollisionPair; 2] {
    let mut world = PhysicsWorld::new();
    let mut colliders = Vec::new();
    for _ in 0..4 {
        let body = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        colliders.push(
            world
                .add_collider(
                    body,
                    Collider::new(
                        Shape::Sphere(Sphere::new(1.0).unwrap()),
                        Transform::identity(),
                    ),
                )
                .unwrap(),
        );
    }
    [
        CollisionPair::new(colliders[0], colliders[1]).unwrap(),
        CollisionPair::new(colliders[2], colliders[3]).unwrap(),
    ]
}

fn contact(impulse: f64) -> ContactInfo {
    ContactInfo {
        point: Vec3::ZERO,
        normal: Vec3::X,
        penetration: 0.25,
        normal_impulse: impulse,
    }
}

fn interaction(
    pair: CollisionPair,
    sensor: bool,
    contact: Option<ContactInfo>,
    impact: bool,
) -> CollisionInteraction {
    CollisionInteraction {
        pair,
        sensor,
        contact,
        impact,
    }
}

#[test]
fn lifecycle_emits_sorted_begin_persist_end_and_impacts() {
    let [solid_pair, sensor_pair] = pairs();
    let mut lifecycle = CollisionLifecycle::new();
    let mut current = [
        interaction(solid_pair, false, Some(contact(2.0)), true),
        interaction(sensor_pair, true, None, false),
    ];
    let mut events = Vec::new();
    lifecycle.stage(&mut current, &mut events).unwrap();
    assert_eq!(
        events
            .iter()
            .map(|event| (event.kind, event.pair))
            .collect::<Vec<_>>(),
        vec![
            (CollisionEventKind::Begin, solid_pair),
            (CollisionEventKind::Begin, sensor_pair),
            (CollisionEventKind::Impact, solid_pair),
        ]
    );
    assert_eq!(events[1].contact, None);
    assert!(events[1].sensor);
    lifecycle.commit();

    let mut current = [interaction(solid_pair, false, Some(contact(0.0)), false)];
    lifecycle.stage(&mut current, &mut events).unwrap();
    assert_eq!(events.len(), 2);
    assert_eq!(events[0].kind, CollisionEventKind::Persist);
    assert_eq!(events[0].pair, solid_pair);
    assert_eq!(events[1].kind, CollisionEventKind::End);
    assert_eq!(events[1].pair, sensor_pair);
    assert!(events[1].sensor);
    assert_eq!(events[1].contact, None);
    lifecycle.commit();

    lifecycle.stage(&mut [], &mut events).unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].kind, CollisionEventKind::End);
    assert_eq!(events[0].pair, solid_pair);
    assert_eq!(events[0].contact, None);
}

#[test]
fn aborting_event_staging_preserves_the_committed_pair_state() {
    let [pair, _] = pairs();
    let mut lifecycle = CollisionLifecycle::new();
    let mut events = Vec::new();
    let mut current = [interaction(pair, false, Some(contact(0.0)), false)];
    lifecycle.stage(&mut current, &mut events).unwrap();
    lifecycle.commit();

    lifecycle.stage(&mut [], &mut events).unwrap();
    lifecycle.abort();
    let mut current = [interaction(pair, false, Some(contact(0.0)), false)];
    lifecycle.stage(&mut current, &mut events).unwrap();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].kind, CollisionEventKind::Persist);
}

#[test]
fn lifecycle_rejects_duplicate_pairs_and_sensor_impacts() {
    let [pair, _] = pairs();
    let mut lifecycle = CollisionLifecycle::new();
    let mut events = Vec::new();
    let mut duplicates = [
        interaction(pair, false, Some(contact(0.0)), false),
        interaction(pair, false, Some(contact(0.0)), false),
    ];
    assert_eq!(
        lifecycle.stage(&mut duplicates, &mut events),
        Err(crate::collision::CollisionError::InvalidContact)
    );
    let mut sensor_impact = [interaction(pair, true, Some(contact(1.0)), true)];
    assert_eq!(
        lifecycle.stage(&mut sensor_impact, &mut events),
        Err(crate::collision::CollisionError::InvalidContact)
    );
}
