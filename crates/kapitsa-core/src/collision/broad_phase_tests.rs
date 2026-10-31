use crate::{
    aabb::Aabb,
    collider::{CollisionFilter, CollisionMode},
    collision::{CollisionPair, broad_phase::BroadPhase, spatial_index::SpatialIndex},
    rigid_body::BodyType,
    world::{BodyId, ColliderId, PhysicsWorld},
};
use kapitsa_math::Vec3;

fn bounds(min_x: f64, max_x: f64) -> Aabb {
    Aabb::new(Vec3::new(min_x, -1.0, -1.0), Vec3::new(max_x, 1.0, 1.0)).expect("bounds")
}

fn identities(count: usize) -> (Vec<BodyId>, Vec<ColliderId>) {
    let mut world = PhysicsWorld::new();
    let bodies: Vec<_> = (0..count)
        .map(|_| {
            world
                .add_body(crate::rigid_body::RigidBody::new_static(
                    crate::transform::Transform::identity(),
                ))
                .expect("body")
        })
        .collect();
    let colliders: Vec<_> = bodies
        .iter()
        .map(|body| {
            world
                .add_collider(
                    *body,
                    crate::collider::Collider::new(
                        crate::shape::Shape::Sphere(
                            crate::shape::Sphere::new(1.0).expect("sphere"),
                        ),
                        crate::transform::Transform::identity(),
                    ),
                )
                .expect("collider")
        })
        .collect();
    (bodies, colliders)
}

fn proxy(
    body: BodyId,
    collider: ColliderId,
    bounds: Option<Aabb>,
    body_type: BodyType,
    mode: CollisionMode,
    filter: CollisionFilter,
    convex: bool,
) -> super::CollisionProxy {
    super::CollisionProxy {
        collider,
        body,
        bounds,
        body_type,
        mode,
        filter,
        convex,
    }
}

#[test]
fn creates_one_canonical_pair_and_applies_filter_and_body_rules() {
    let (bodies, colliders) = identities(4);
    let proxies = [
        proxy(
            bodies[0],
            colliders[0],
            Some(bounds(0.0, 1.0)),
            BodyType::Dynamic,
            CollisionMode::Solid,
            CollisionFilter::default(),
            true,
        ),
        proxy(
            bodies[1],
            colliders[1],
            Some(bounds(0.5, 1.5)),
            BodyType::Static,
            CollisionMode::Solid,
            CollisionFilter::default(),
            true,
        ),
        proxy(
            bodies[2],
            colliders[2],
            Some(bounds(0.0, 1.0)),
            BodyType::Dynamic,
            CollisionMode::Solid,
            CollisionFilter::new(0, 0),
            true,
        ),
        proxy(
            bodies[3],
            colliders[3],
            Some(bounds(0.25, 0.75)),
            BodyType::Dynamic,
            CollisionMode::Solid,
            CollisionFilter::default(),
            true,
        ),
    ];
    let mut index = SpatialIndex::new();
    let bounds: Vec<_> = proxies
        .iter()
        .filter_map(|proxy| proxy.bounds.map(|bounds| (proxy.collider, bounds)))
        .collect();
    index.synchronize(&bounds).expect("tree sync");
    let mut broad_phase = BroadPhase::new();
    let pairs = broad_phase
        .generate(Some(&mut index), &proxies)
        .expect("candidate pairs");
    assert_eq!(pairs.len(), 3);
    assert!(pairs.contains(&CollisionPair::new(colliders[0], colliders[1]).unwrap()));
    assert!(pairs.contains(&CollisionPair::new(colliders[0], colliders[3]).unwrap()));
    assert!(pairs.contains(&CollisionPair::new(colliders[1], colliders[3]).unwrap()));
}

#[test]
fn sensor_against_infinite_surface_is_reported_but_static_solid_is_not() {
    let (bodies, colliders) = identities(2);
    let finite = bounds(-1.0, 1.0);
    let mut index = SpatialIndex::new();
    index
        .insert(colliders[0], finite)
        .expect("insert finite collider");
    let plane = proxy(
        bodies[1],
        colliders[1],
        None,
        BodyType::Static,
        CollisionMode::Sensor,
        CollisionFilter::default(),
        false,
    );
    let dynamic = proxy(
        bodies[0],
        colliders[0],
        Some(finite),
        BodyType::Dynamic,
        CollisionMode::Solid,
        CollisionFilter::default(),
        true,
    );
    let mut broad_phase = BroadPhase::new();
    assert_eq!(
        broad_phase.generate(Some(&mut index), &[plane, dynamic]),
        Ok(&[CollisionPair::new(colliders[0], colliders[1]).unwrap()][..])
    );

    let static_solid = proxy(
        bodies[1],
        colliders[1],
        None,
        BodyType::Static,
        CollisionMode::Solid,
        CollisionFilter::default(),
        false,
    );
    let static_finite = proxy(
        bodies[0],
        colliders[0],
        Some(finite),
        BodyType::Static,
        CollisionMode::Solid,
        CollisionFilter::default(),
        true,
    );
    assert!(
        broad_phase
            .generate(Some(&mut index), &[static_solid, static_finite])
            .expect("static pair")
            .is_empty()
    );
}

#[test]
fn sensors_do_not_track_surface_to_surface_pairs() {
    let (bodies, colliders) = identities(2);
    let proxies = [
        proxy(
            bodies[0],
            colliders[0],
            None,
            BodyType::Static,
            CollisionMode::Sensor,
            CollisionFilter::default(),
            false,
        ),
        proxy(
            bodies[1],
            colliders[1],
            None,
            BodyType::Kinematic,
            CollisionMode::Sensor,
            CollisionFilter::default(),
            false,
        ),
    ];
    let mut broad_phase = BroadPhase::new();
    assert!(
        broad_phase
            .generate(None, &proxies)
            .expect("surface sensors")
            .is_empty()
    );
}
