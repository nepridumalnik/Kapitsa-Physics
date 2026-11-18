//! Host-driven synchronous stepping without a Tokio runtime or the async service.

use kapitsa::math::Vec3;
use kapitsa::{
    Collider, MassProperties, PhysicsWorld, RigidBody, Shape, Transform, UniformGravity,
    shape::{Plane, Sphere},
};

#[test]
fn synchronous_world_steps_and_exposes_state_without_a_runtime()
-> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    world.add_gravity(UniformGravity::new(Vec3::new(0.0, -9.81, 0.0))?)?;

    let floor = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let mut floor_collider = Collider::new(
        Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0)?),
        Transform::identity(),
    );
    floor_collider.set_user_data(0x00f1_0000);
    let floor_collider = world.add_collider(floor, floor_collider)?;

    let mass = MassProperties::new(1.0, Vec3::ZERO, kapitsa::math::Mat3::IDENTITY)?;
    let mut ball = RigidBody::new_dynamic(
        Transform::new(Vec3::new(0.0, 2.0, 0.0), kapitsa::math::Quat::IDENTITY)?,
        mass,
    )?;
    ball.set_user_data(0x00ba_1100);
    let ball = world.add_body(ball)?;
    let ball_collider = world.add_collider(
        ball,
        Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
    )?;

    assert_eq!(
        world.body(ball).map(RigidBody::user_data),
        Some(0x00ba_1100)
    );
    assert_eq!(
        world.collider(floor_collider).map(Collider::user_data),
        Some(0x00f1_0000)
    );

    world.step(1.0 / 60.0)?;
    world.step(1.0 / 60.0)?;

    let mut commands = world.commands();
    commands.set_linear_velocity(ball, Vec3::new(1.0, 0.0, 0.0));
    world.apply(&commands)?;
    world.simulate(1.0 / 60.0);
    let results = world.fetch_results()?;
    assert_eq!(results.diagnostics(), world.step_diagnostics());

    let state = world.body_state(ball).ok_or("dynamic body is missing")?;
    let transform = world
        .body_transform(ball)
        .ok_or("dynamic body is missing")?;
    let velocity = world
        .body_linear_velocity(ball)
        .ok_or("dynamic body is missing")?;
    let angular_velocity = world
        .body_angular_velocity(ball)
        .ok_or("dynamic body is missing")?;

    assert_eq!(transform, state.transform());
    assert_eq!(velocity, state.linear_velocity());
    assert_eq!(angular_velocity, state.angular_velocity());
    assert!(transform.translation().y < 2.0);
    assert!(velocity.y < 0.0);
    assert!(world.body(ball).map(RigidBody::user_data) == Some(0x00ba_1100));
    assert!(world.collider(ball_collider).is_some());
    assert!(world.collision_events().is_empty());
    assert!(world.contact_impulses().is_empty());

    world
        .body_mut(ball)
        .ok_or("dynamic body is missing")?
        .set_user_data(7);
    assert_eq!(world.body(ball).map(RigidBody::user_data), Some(7));
    world
        .collider_mut(floor_collider)
        .ok_or("floor collider is missing")?
        .set_user_data(9);
    assert_eq!(
        world.collider(floor_collider).map(Collider::user_data),
        Some(9)
    );

    Ok(())
}
