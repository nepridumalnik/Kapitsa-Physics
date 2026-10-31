use crate::{
    collider::{Collider, CollisionMode},
    collision::{CollisionError, CollisionPair, narrow_phase::NarrowPhase},
    rigid_body::RigidBody,
    shape::{Plane, Shape, Sphere, Triangle},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::Vec3;

#[path = "geometry_matrix_tests.rs"]
mod geometry_matrix;

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

fn translated(y: f64) -> Transform {
    Transform::new(Vec3::new(0.0, y, 0.0), kapitsa_math::Quat::IDENTITY).unwrap()
}

fn at(x: f64, y: f64, z: f64) -> Transform {
    Transform::new(Vec3::new(x, y, z), kapitsa_math::Quat::IDENTITY).unwrap()
}

#[test]
fn plane_contacts_front_side_and_rejects_back_side() {
    let mut narrow = NarrowPhase::new();
    let pair = pair();
    let plane = Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).unwrap());
    let sphere = Shape::Sphere(Sphere::new(0.5).unwrap());
    let front = narrow
        .detect(
            pair,
            &plane,
            &Transform::identity(),
            &Transform::identity(),
            CollisionMode::Solid,
            &sphere,
            &translated(0.25),
            &translated(0.25),
            CollisionMode::Solid,
            0.01,
        )
        .unwrap()
        .unwrap();
    let contact = front.points().first().unwrap();
    assert_eq!(front.len(), 1);
    assert!((contact.penetration - 0.25).abs() < 1.0e-12);
    assert_eq!(contact.normal, Vec3::Y);

    assert!(
        narrow
            .detect(
                pair,
                &plane,
                &Transform::identity(),
                &Transform::identity(),
                CollisionMode::Solid,
                &sphere,
                &translated(-0.25),
                &translated(-0.25),
                CollisionMode::Solid,
                0.01,
            )
            .unwrap()
            .is_none()
    );
}

#[test]
fn plane_sensor_is_two_sided_and_contact_normal_follows_pair_order() {
    let mut narrow = NarrowPhase::new();
    let pair = pair();
    let plane = Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).unwrap());
    let sphere = Shape::Sphere(Sphere::new(0.5).unwrap());
    let manifold = narrow
        .detect(
            pair,
            &sphere,
            &translated(-0.25),
            &translated(-0.25),
            CollisionMode::Sensor,
            &plane,
            &Transform::identity(),
            &Transform::identity(),
            CollisionMode::Solid,
            0.01,
        )
        .unwrap()
        .unwrap();
    assert_eq!(manifold.points().first().unwrap().normal, Vec3::Y);
}

#[test]
fn triangle_contacts_are_two_sided_and_use_exact_triangle_geometry() {
    let mut narrow = NarrowPhase::new();
    let pair = pair();
    let triangle = Shape::Triangle(
        Triangle::new(
            Vec3::new(-1.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
    );
    let sphere = Shape::Sphere(Sphere::new(0.5).unwrap());
    let manifold = narrow
        .detect(
            pair,
            &triangle,
            &Transform::identity(),
            &Transform::identity(),
            CollisionMode::Solid,
            &sphere,
            &translated(-0.25),
            &translated(-0.25),
            CollisionMode::Solid,
            0.01,
        )
        .unwrap()
        .unwrap();
    let contact = manifold.points().first().unwrap();
    assert!((contact.penetration - 0.25).abs() < 1.0e-12);
    assert_eq!(contact.normal, -Vec3::Y);
}

#[test]
fn triangle_rejects_sphere_that_only_overlaps_its_aabb() {
    let mut narrow = NarrowPhase::new();
    let pair = pair();
    let triangle = Shape::Triangle(
        Triangle::new(
            Vec3::new(-1.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, -1.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap(),
    );
    let sphere = Shape::Sphere(Sphere::new(0.05).unwrap());
    let transform = at(0.7, 0.0, 0.0);
    assert!(
        narrow
            .detect(
                pair,
                &triangle,
                &Transform::identity(),
                &Transform::identity(),
                CollisionMode::Solid,
                &sphere,
                &transform,
                &transform,
                CollisionMode::Solid,
                0.01,
            )
            .unwrap()
            .is_none()
    );
}

#[test]
fn unsupported_surface_pairs_return_an_error() {
    let mut narrow = NarrowPhase::new();
    let pair = pair();
    let plane = Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).unwrap());
    let triangle = Shape::Triangle(Triangle::new(Vec3::ZERO, Vec3::X, Vec3::Y).unwrap());
    assert_eq!(
        narrow.detect(
            pair,
            &plane,
            &Transform::identity(),
            &Transform::identity(),
            CollisionMode::Solid,
            &triangle,
            &Transform::identity(),
            &Transform::identity(),
            CollisionMode::Solid,
            0.01,
        ),
        Err(CollisionError::UnsupportedGeometryPair)
    );
}
