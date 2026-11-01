use crate::{
    collider::Collider,
    collision::{
        CollisionPair,
        cache::ContactCache,
        manifold::{ContactManifold, ContactPoint},
        solver::{SolverManifold, SolverMaterial, tangent_basis},
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
    contact.tangent_impulse = if impulse > 0.0 {
        [0.25, -0.5]
    } else {
        [0.0; 2]
    };
    manifold.insert(contact, Vec3::ZERO).unwrap();
    manifold
}

fn solver_manifold(
    manifold: ContactManifold,
    static_friction: f64,
    dynamic_friction: f64,
) -> SolverManifold {
    SolverManifold {
        body_a: 0,
        body_b: 1,
        manifold,
        material: SolverMaterial {
            restitution: 0.0,
            static_friction,
            dynamic_friction,
        },
    }
}

#[test]
fn cached_impulses_restore_for_matching_pair_feature_and_normal() {
    let pair = pair();
    let mut cache = ContactCache::new();
    cache
        .stage(&[manifold(pair, Vec3::Y, 7, 2.0)], 1.0)
        .unwrap();
    cache.commit();
    let mut regenerated = solver_manifold(manifold(pair, Vec3::Y, 7, 0.0), 0.5, 0.5);
    cache
        .warm_start_solver(std::slice::from_mut(&mut regenerated), 1.0)
        .unwrap();
    let point = regenerated.manifold.points().first().unwrap();
    assert_eq!(point.normal_impulse, 2.0);
    assert_eq!(point.tangent_impulse, [0.25, -0.5]);
}

#[test]
fn warm_start_impulses_scale_with_timestep_and_staged_dt_is_transactional() {
    let pair = pair();
    let mut cache = ContactCache::new();
    cache
        .stage(&[manifold(pair, Vec3::Y, 7, 2.0)], 0.01)
        .unwrap();
    cache.commit();

    let mut regenerated = solver_manifold(manifold(pair, Vec3::Y, 7, 0.0), 0.5, 0.5);
    cache
        .warm_start_solver(std::slice::from_mut(&mut regenerated), 0.02)
        .unwrap();
    let contact = regenerated.manifold.points().first().unwrap();
    assert_eq!(contact.normal_impulse, 4.0);
    assert_eq!(contact.tangent_impulse, [0.5, -1.0]);

    cache.stage(&[], 0.02).unwrap();
    cache.abort();
    let mut retry = solver_manifold(manifold(pair, Vec3::Y, 7, 0.0), 0.5, 0.5);
    cache
        .warm_start_solver(std::slice::from_mut(&mut retry), 0.01)
        .unwrap();
    assert_eq!(retry.manifold.points().first().unwrap().normal_impulse, 2.0);
}

#[test]
fn cache_rejects_changed_normals_and_drops_absent_features_on_commit() {
    let pair = pair();
    let mut cache = ContactCache::new();
    cache
        .stage(&[manifold(pair, Vec3::Y, 7, 2.0)], 1.0)
        .unwrap();
    cache.commit();

    let mut changed = solver_manifold(manifold(pair, Vec3::X, 7, 0.0), 0.5, 0.5);
    cache
        .warm_start_solver(std::slice::from_mut(&mut changed), 1.0)
        .unwrap();
    let changed_contact = changed.manifold.points().first().unwrap();
    assert_eq!(changed_contact.normal_impulse, 0.0);
    assert_eq!(changed_contact.tangent_impulse, [0.0; 2]);

    cache.stage(&[], 1.0).unwrap();
    cache.commit();
    assert_eq!(cache.len(), 0);
}

#[test]
fn warm_start_reprojects_world_tangent_impulse_as_contact_normal_rotates() {
    let pair = pair();
    let previous = manifold(pair, Vec3::Y, 7, 2.0);
    let (old_a, old_b) = tangent_basis(Vec3::Y).unwrap();
    let old_contact = previous.points().first().unwrap();
    let tangent_world =
        old_a * old_contact.tangent_impulse[0] + old_b * old_contact.tangent_impulse[1];
    let mut cache = ContactCache::new();
    cache.stage(&[previous], 1.0).unwrap();
    cache.commit();

    let normal = kapitsa_math::Quat::from_axis_angle(Vec3::Z, 0.1)
        .unwrap()
        .rotate_vector(Vec3::Y)
        .unwrap();
    let regenerated = manifold(pair, normal, 7, 0.0);
    let mut regenerated = solver_manifold(regenerated, 1.0, 1.0);
    cache
        .warm_start_solver(std::slice::from_mut(&mut regenerated), 1.0)
        .unwrap();

    let contact = regenerated.manifold.points().first().unwrap();
    let (new_a, new_b) = tangent_basis(normal).unwrap();
    let tangent_after = new_a * contact.tangent_impulse[0] + new_b * contact.tangent_impulse[1];
    let expected = tangent_world - normal * tangent_world.dot(normal);
    assert!((tangent_after - expected).length() < 1.0e-10);
    assert!(tangent_after.length() <= tangent_world.length() + 1.0e-12);
}

#[test]
fn warm_start_drops_changed_features_and_clamps_to_the_new_friction_cone() {
    let pair = pair();
    let mut cache = ContactCache::new();
    cache
        .stage(&[manifold(pair, Vec3::Y, 7, 2.0)], 1.0)
        .unwrap();
    cache.commit();

    let mut changed_feature = solver_manifold(manifold(pair, Vec3::Y, 8, 0.0), 0.1, 0.1);
    cache
        .warm_start_solver(std::slice::from_mut(&mut changed_feature), 1.0)
        .unwrap();
    let changed = changed_feature.manifold.points().first().unwrap();
    assert_eq!(changed.normal_impulse, 0.0);
    assert_eq!(changed.tangent_impulse, [0.0; 2]);

    let mut same_feature = solver_manifold(manifold(pair, Vec3::Y, 7, 0.0), 0.1, 0.1);
    cache
        .warm_start_solver(std::slice::from_mut(&mut same_feature), 1.0)
        .unwrap();
    let contact = same_feature.manifold.points().first().unwrap();
    assert_eq!(contact.normal_impulse, 2.0);
    assert!((contact.tangent_impulse[0].hypot(contact.tangent_impulse[1]) - 0.2).abs() < 1.0e-12);
}

#[test]
fn staged_cache_is_transactional_and_collider_invalidation_is_exact() {
    let first_pair = pair();
    let second_pair = pair();
    let mut cache = ContactCache::new();
    cache
        .stage(
            &[
                manifold(first_pair, Vec3::Y, 1, 1.0),
                manifold(second_pair, Vec3::Y, 2, 2.0),
            ],
            1.0,
        )
        .unwrap();
    cache.commit();
    cache.stage(&[], 1.0).unwrap();
    cache.abort();
    assert_eq!(cache.len(), 2);

    cache.invalidate_collider(first_pair.a);
    assert_eq!(cache.len(), 1);
    cache.invalidate_collider(second_pair.b);
    assert_eq!(cache.len(), 0);
}
