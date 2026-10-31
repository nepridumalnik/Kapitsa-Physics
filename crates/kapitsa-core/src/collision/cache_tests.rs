use crate::{
    collider::Collider,
    collision::{
        CollisionPair,
        cache::ContactCache,
        manifold::{ContactManifold, ContactPoint},
    },
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::Vec3;

fn pair() -> CollisionPair {
    let mut world = PhysicsWorld::new();
    let body_a = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let body_b = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let collider_a = world
        .add_collider(
            body_a,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let collider_b = world
        .add_collider(
            body_b,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    CollisionPair::new(collider_a, collider_b).unwrap()
}

fn manifold(pair: CollisionPair, normal: Vec3, feature: u64, impulse: f64) -> ContactManifold {
    let contact = ContactPoint::from_world(
        &Transform::identity(),
        &Transform::identity(),
        Vec3::ZERO,
        Vec3::ZERO,
        normal,
        0.0,
        feature,
    )
    .unwrap();
    let mut manifold = ContactManifold::new(pair);
    let mut contact = contact;
    contact.normal_impulse = impulse;
    contact.tangent_impulse = [0.25, -0.5];
    manifold.insert(contact, Vec3::ZERO).unwrap();
    manifold
}

#[test]
fn cached_impulses_restore_for_matching_pair_feature_and_normal() {
    let pair = pair();
    let mut cache = ContactCache::new();
    cache.stage(&[manifold(pair, Vec3::Y, 7, 2.0)]).unwrap();
    cache.commit();
    let mut regenerated = manifold(pair, Vec3::Y, 7, 0.0);
    cache.warm_start(std::slice::from_mut(&mut regenerated));
    let point = regenerated.points().next().unwrap();
    assert_eq!(point.normal_impulse, 2.0);
    assert_eq!(point.tangent_impulse, [0.25, -0.5]);
}

#[test]
fn cache_rejects_changed_normals_and_drops_absent_features_on_commit() {
    let pair = pair();
    let mut cache = ContactCache::new();
    cache.stage(&[manifold(pair, Vec3::Y, 7, 2.0)]).unwrap();
    cache.commit();

    let mut changed = manifold(pair, Vec3::X, 7, 0.0);
    cache.warm_start(std::slice::from_mut(&mut changed));
    assert_eq!(changed.points().next().unwrap().normal_impulse, 0.0);

    cache.stage(&[]).unwrap();
    cache.commit();
    assert_eq!(cache.len(), 0);
}

#[test]
fn staged_cache_is_transactional_and_collider_invalidation_is_exact() {
    let first_pair = pair();
    let second_pair = pair();
    let mut cache = ContactCache::new();
    cache
        .stage(&[
            manifold(first_pair, Vec3::Y, 1, 1.0),
            manifold(second_pair, Vec3::Y, 2, 2.0),
        ])
        .unwrap();
    cache.commit();
    cache.stage(&[]).unwrap();
    cache.abort();
    assert_eq!(cache.len(), 2);

    cache.invalidate_collider(first_pair.a);
    assert_eq!(cache.len(), 1);
    cache.invalidate_collider(second_pair.b);
    assert_eq!(cache.len(), 0);
}
