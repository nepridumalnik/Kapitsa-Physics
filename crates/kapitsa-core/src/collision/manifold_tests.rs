use crate::{
    collider::Collider,
    collision::{
        CollisionPair,
        manifold::{ContactManifold, ContactPoint, combine_feature_ids},
    },
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Quat, Vec3};

fn pair() -> CollisionPair {
    let mut world = PhysicsWorld::new();
    let body_a = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body a");
    let body_b = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body b");
    let collider = |world: &mut PhysicsWorld, body| {
        world
            .add_collider(
                body,
                Collider::new(
                    Shape::Sphere(Sphere::new(1.0).expect("sphere")),
                    Transform::identity(),
                ),
            )
            .expect("collider")
    };
    CollisionPair::new(collider(&mut world, body_a), collider(&mut world, body_b)).expect("pair")
}

fn contact(feature_id: u64, penetration: f64, position: Vec3) -> ContactPoint {
    ContactPoint {
        local_a: position,
        local_b: position,
        normal: Vec3::X,
        penetration,
        feature_id,
        normal_impulse: 0.0,
        normal_impulse_delta: 0.0,
        tangent_impulse: [0.0; 2],
    }
}

#[test]
fn manifold_reduction_keeps_the_contact_patch_corners() {
    let mut manifold = ContactManifold::new(pair());
    for (feature, position) in [
        (1, Vec3::new(-1.0, -1.0, 0.0)),
        (2, Vec3::new(1.0, -1.0, 0.0)),
        (3, Vec3::new(1.0, 1.0, 0.0)),
        (4, Vec3::new(-1.0, 1.0, 0.0)),
    ] {
        manifold
            .insert(contact(feature, 0.1, position), position)
            .expect("insert patch point");
    }
    manifold
        .insert(contact(5, 0.1, Vec3::ZERO), Vec3::ZERO)
        .expect("reduce patch");

    assert_eq!(manifold.len(), 4);
    let features: Vec<_> = manifold.points().map(|point| point.feature_id).collect();
    assert!(!features.contains(&5));
    assert_eq!(features.len(), 4);
}

#[test]
fn manifold_reduction_preserves_the_deepest_point() {
    let mut manifold = ContactManifold::new(pair());
    for (feature, position) in [
        (1, Vec3::new(-1.0, -1.0, 0.0)),
        (2, Vec3::new(1.0, -1.0, 0.0)),
        (3, Vec3::new(1.0, 1.0, 0.0)),
        (4, Vec3::new(-1.0, 1.0, 0.0)),
    ] {
        manifold
            .insert(contact(feature, 0.1, position), position)
            .expect("insert patch point");
    }
    manifold
        .insert(contact(5, 0.5, Vec3::ZERO), Vec3::ZERO)
        .expect("reduce patch");

    assert!(manifold.points().any(|point| point.feature_id == 5));
}

#[test]
fn regenerated_features_keep_cached_impulses() {
    let mut manifold = ContactManifold::new(pair());
    let mut previous = contact(7, 0.2, Vec3::ZERO);
    previous.normal_impulse = 3.0;
    previous.tangent_impulse = [0.25, -0.5];
    manifold
        .insert(previous, Vec3::ZERO)
        .expect("insert original");
    manifold
        .insert(contact(7, 0.3, Vec3::X), Vec3::X)
        .expect("update feature");

    let updated = manifold.points().next().expect("updated contact");
    assert_eq!(updated.penetration, 0.3);
    assert_eq!(updated.normal_impulse, 3.0);
    assert_eq!(updated.tangent_impulse, [0.25, -0.5]);
}

#[test]
fn world_contacts_are_stored_in_body_local_coordinates() {
    let body_a = Transform::new(
        Vec3::new(3.0, -2.0, 1.0),
        Quat::from_axis_angle(Vec3::Z, 0.5).expect("rotation"),
    )
    .expect("transform a");
    let body_b = Transform::new(Vec3::new(-4.0, 2.0, 0.0), Quat::IDENTITY).expect("transform b");
    let local_a = Vec3::new(0.5, 0.25, -0.75);
    let local_b = Vec3::new(-0.5, 0.0, 1.25);
    let point_a = body_a.transform_point(local_a);
    let point_b = body_b.transform_point(local_b);
    let point = ContactPoint::from_world(
        &body_a,
        &body_b,
        point_a,
        point_b,
        Vec3::X,
        0.1,
        combine_feature_ids(4, 9),
    )
    .expect("contact");

    assert!((point.local_a - local_a).length() < 1.0e-12);
    assert!((point.local_b - local_b).length() < 1.0e-12);
    assert_eq!(point.normal, Vec3::X);
    assert_ne!(combine_feature_ids(4, 9), combine_feature_ids(9, 4));
}
