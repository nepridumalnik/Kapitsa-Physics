use kapitsa_core::{
    collider::{Collider, CollisionFilter, CollisionMode},
    collision::{QueryError, QueryFilter, Ray},
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Quat, Vec3};

fn transform(position: Vec3) -> Transform {
    Transform::new(position, Quat::IDENTITY).expect("valid transform")
}

fn add_sphere(
    world: &mut PhysicsWorld,
    position: Vec3,
    mode: CollisionMode,
    filter: CollisionFilter,
) -> kapitsa_core::world::ColliderId {
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("static body");
    let mut collider = Collider::new(
        Shape::Sphere(Sphere::new(0.5).expect("sphere")),
        transform(position),
    );
    collider.set_mode(mode);
    collider.set_filter(filter);
    world.add_collider(body, collider).expect("collider")
}

#[test]
fn raycast_returns_nearest_hit_and_applies_layer_and_sensor_filters() {
    let mut world = PhysicsWorld::new();
    let near = add_sphere(
        &mut world,
        Vec3::new(2.0, 0.0, 0.0),
        CollisionMode::Solid,
        CollisionFilter::default(),
    );
    let far = add_sphere(
        &mut world,
        Vec3::new(4.0, 0.0, 0.0),
        CollisionMode::Sensor,
        CollisionFilter::new(2, 2),
    );
    let ray = Ray::new(Vec3::ZERO, Vec3::X, 10.0).expect("ray");

    let hit = world
        .raycast(ray, QueryFilter::default())
        .expect("raycast")
        .expect("solid hit");
    assert_eq!(hit.collider, near);
    assert!((hit.distance - 1.5).abs() < 1.0e-10);

    let hit = world
        .raycast(
            ray,
            QueryFilter {
                layers: 2,
                mask: 2,
                include_sensors: true,
                ..QueryFilter::default()
            },
        )
        .expect("filtered raycast")
        .expect("sensor hit");
    assert_eq!(hit.collider, far);
    assert!((hit.distance - 3.5).abs() < 1.0e-10);
}

#[test]
fn sweep_returns_first_impact_and_overlap_lists_all_matching_colliders() {
    let mut world = PhysicsWorld::new();
    let first = add_sphere(
        &mut world,
        Vec3::new(2.0, 0.0, 0.0),
        CollisionMode::Solid,
        CollisionFilter::default(),
    );
    add_sphere(
        &mut world,
        Vec3::new(4.0, 0.0, 0.0),
        CollisionMode::Solid,
        CollisionFilter::default(),
    );
    let shape = Shape::Sphere(Sphere::new(0.5).expect("query sphere"));
    let hit = world
        .sweep(
            &shape,
            transform(Vec3::new(-3.0, 0.0, 0.0)),
            Vec3::new(10.0, 0.0, 0.0),
            QueryFilter::default(),
        )
        .expect("sweep")
        .expect("sweep hit");
    assert_eq!(hit.collider, first);
    assert!((hit.toi - 0.4).abs() < 1.0e-9);

    let mut overlaps = Vec::new();
    world
        .overlaps(
            &shape,
            transform(Vec3::new(2.0, 0.0, 0.0)),
            QueryFilter::default(),
            &mut overlaps,
        )
        .expect("overlaps");
    assert_eq!(overlaps.len(), 1);
    assert_eq!(overlaps[0].collider, first);

    for _ in 0..2 {
        assert_eq!(
            world
                .raycast(
                    Ray::new(Vec3::ZERO, Vec3::X, 10.0).expect("ray"),
                    QueryFilter::default(),
                )
                .expect("reused raycast")
                .expect("reused ray hit")
                .collider,
            first
        );
        assert_eq!(
            world
                .sweep(
                    &shape,
                    transform(Vec3::new(-3.0, 0.0, 0.0)),
                    Vec3::new(10.0, 0.0, 0.0),
                    QueryFilter::default(),
                )
                .expect("reused sweep")
                .expect("reused sweep hit")
                .collider,
            first
        );
        let mut reused_overlaps = Vec::new();
        world
            .overlaps(
                &shape,
                transform(Vec3::new(2.0, 0.0, 0.0)),
                QueryFilter::default(),
                &mut reused_overlaps,
            )
            .expect("reused overlaps");
        assert_eq!(reused_overlaps.len(), 1);
        assert_eq!(reused_overlaps[0].collider, first);
    }
}

#[test]
fn invalid_ray_and_non_convex_query_shape_return_explicit_errors() {
    assert_eq!(
        Ray::new(Vec3::ZERO, Vec3::ZERO, 1.0),
        Err(QueryError::InvalidInput)
    );
    assert_eq!(
        Ray::new(Vec3::ZERO, Vec3::X, f64::INFINITY),
        Err(QueryError::InvalidInput)
    );

    let mut world = PhysicsWorld::new();
    let plane_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("static body");
    world
        .add_collider(
            plane_body,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).expect("sphere")),
                Transform::identity(),
            ),
        )
        .expect("collider");
    let invalid_query =
        Shape::Plane(kapitsa_core::shape::Plane::from_vec(Vec3::Y, 0.0).expect("plane"));
    assert_eq!(
        world.sweep(
            &invalid_query,
            Transform::identity(),
            Vec3::X,
            QueryFilter::default()
        ),
        Err(QueryError::UnsupportedQueryShape)
    );
}
