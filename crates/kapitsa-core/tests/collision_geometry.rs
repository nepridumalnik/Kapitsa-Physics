use kapitsa_core::{
    collider::Collider,
    collision::{OverlapHit, QueryFilter},
    rigid_body::RigidBody,
    shape::{Cuboid, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Quat, Vec3};

fn transform(position: Vec3, rotation: Quat) -> Transform {
    Transform::new(position, rotation).expect("valid transform")
}

fn add_static_shape(
    world: &mut PhysicsWorld,
    shape: Shape,
    body_transform: Transform,
    local_transform: Transform,
) -> kapitsa_core::world::ColliderId {
    let body = world
        .add_body(RigidBody::new_static(body_transform))
        .expect("static body");
    world
        .add_collider(body, Collider::new(shape, local_transform))
        .expect("collider")
}

fn overlaps(world: &PhysicsWorld, shape: &Shape, at: Transform) -> Vec<OverlapHit> {
    let mut hits = Vec::new();
    world
        .overlaps(shape, at, QueryFilter::default(), &mut hits)
        .expect("overlap query");
    hits
}

#[test]
fn sphere_contacts_cover_separation_touching_partial_overlap_and_embedding() {
    let mut world = PhysicsWorld::new();
    let target = add_static_shape(
        &mut world,
        Shape::Sphere(Sphere::new(1.0).expect("target sphere")),
        Transform::identity(),
        Transform::identity(),
    );
    let query = Shape::Sphere(Sphere::new(0.5).expect("query sphere"));

    assert!(
        overlaps(
            &world,
            &query,
            transform(Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY)
        )
        .is_empty()
    );
    assert_eq!(
        overlaps(
            &world,
            &query,
            transform(Vec3::new(1.5, 0.0, 0.0), Quat::IDENTITY)
        )[0]
        .collider,
        target
    );
    assert_eq!(
        overlaps(
            &world,
            &query,
            transform(Vec3::new(1.25, 0.0, 0.0), Quat::IDENTITY)
        )[0]
        .collider,
        target
    );
    assert_eq!(
        overlaps(&world, &query, Transform::identity())[0].collider,
        target
    );
}

#[test]
fn rotated_body_and_local_collider_transforms_compose_for_queries() {
    let mut world = PhysicsWorld::new();
    let body_rotation = Quat::from_axis_angle(Vec3::Z, 0.2).expect("body rotation");
    let local_rotation = Quat::from_axis_angle(Vec3::Z, 0.5).expect("collider rotation");
    let body_transform = transform(Vec3::new(1.0, 2.0, 0.0), body_rotation);
    let local_transform = transform(Vec3::new(2.0, 0.0, 0.0), local_rotation);
    let collider = add_static_shape(
        &mut world,
        Shape::Cuboid(Cuboid::new(2.0, 1.0, 1.0).expect("cuboid")),
        body_transform,
        local_transform,
    );
    let world_transform = body_transform
        .compose(&local_transform)
        .expect("composed transform");
    let surface_point = world_transform.transform_point(Vec3::new(1.02, 0.0, 0.0));
    let query = Shape::Sphere(Sphere::new(0.05).expect("query sphere"));

    assert_eq!(
        overlaps(&world, &query, transform(surface_point, Quat::IDENTITY))[0].collider,
        collider
    );
    assert!(
        overlaps(
            &world,
            &query,
            transform(Vec3::new(1.0, 2.0, 0.0), Quat::IDENTITY)
        )
        .is_empty()
    );
}

#[test]
fn collision_queries_remain_finite_across_different_geometry_scales() {
    let mut world = PhysicsWorld::new();
    let large = add_static_shape(
        &mut world,
        Shape::Sphere(Sphere::new(1.0e6).expect("large sphere")),
        Transform::identity(),
        Transform::identity(),
    );
    let small_query = Shape::Sphere(Sphere::new(1.0e-2).expect("small sphere"));
    let hit = overlaps(
        &world,
        &small_query,
        transform(Vec3::new(1.0e6 - 5.0e-3, 0.0, 0.0), Quat::IDENTITY),
    );
    assert_eq!(hit.len(), 1);
    assert_eq!(hit[0].collider, large);
    assert!(
        overlaps(
            &world,
            &small_query,
            transform(Vec3::new(1.0e6 + 1.0, 0.0, 0.0), Quat::IDENTITY)
        )
        .is_empty()
    );
}

#[test]
fn collision_queries_remain_finite_for_small_f64_geometry() {
    let mut world = PhysicsWorld::new();
    let target = add_static_shape(
        &mut world,
        Shape::Sphere(Sphere::new(1.0e-7).expect("small target sphere")),
        Transform::identity(),
        Transform::identity(),
    );
    let query = Shape::Sphere(Sphere::new(5.0e-8).expect("small query sphere"));
    let hit = overlaps(
        &world,
        &query,
        transform(Vec3::new(1.4e-7, 0.0, 0.0), Quat::IDENTITY),
    );
    assert_eq!(hit.len(), 1);
    assert_eq!(hit[0].collider, target);
    assert!(
        overlaps(
            &world,
            &query,
            transform(Vec3::new(2.0e-7, 0.0, 0.0), Quat::IDENTITY)
        )
        .is_empty()
    );
}
