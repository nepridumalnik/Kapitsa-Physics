use std::time::Duration;

use kapitsa_core::{
    collider::Collider,
    collision::{CollisionEventKind, QueryFilter, Ray},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Plane, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};
use kapitsa_tokio::{PhysicsSimulation, SchedulerConfig};

fn transform(position: Vec3) -> Transform {
    Transform::new(position, Quat::IDENTITY).expect("valid transform")
}

fn dynamic(position: Vec3) -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    RigidBody::new_dynamic(transform(position), mass).expect("dynamic body")
}

fn config() -> SchedulerConfig {
    SchedulerConfig {
        target_hz: 60,
        min_hz: 60,
        max_workers: 1,
        adaptive: false,
        ..SchedulerConfig::default()
    }
}

fn collision_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let plane = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("plane body");
    world
        .add_collider(
            plane,
            Collider::new(
                Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).expect("plane")),
                Transform::identity(),
            ),
        )
        .expect("plane collider");
    let mut ball = dynamic(Vec3::new(0.0, 0.49, 0.0));
    ball.set_linear_velocity(Vec3::new(0.0, -1.0, 0.0))
        .expect("ball velocity");
    let ball = world.add_body(ball).expect("ball body");
    world
        .add_collider(
            ball,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                Transform::identity(),
            ),
        )
        .expect("ball collider");
    world
}

#[tokio::test(start_paused = true)]
async fn collision_batch_and_snapshot_identify_the_same_committed_step() {
    let mut simulation =
        PhysicsSimulation::with_config(collision_world(), config()).expect("simulation");
    let client = simulation.client();
    let mut events = client.subscribe_collision_events();
    simulation.start().expect("start");
    tokio::task::yield_now().await;
    tokio::time::advance(Duration::from_millis(17)).await;
    tokio::task::yield_now().await;
    tokio::task::yield_now().await;

    let batch = tokio::time::timeout(Duration::from_secs(1), events.next_batch())
        .await
        .expect("event batch arrives")
        .expect("event stream remains open");
    let snapshot = client.snapshot();
    assert_eq!(batch.step(), snapshot.step());
    assert!(
        batch
            .events()
            .iter()
            .any(|event| event.kind == CollisionEventKind::Begin)
    );
    assert!(
        batch
            .events()
            .iter()
            .any(|event| event.kind == CollisionEventKind::Impact)
    );
    simulation.stop().await.expect("stop");
}

#[tokio::test]
async fn client_commands_and_queries_work_while_simulation_is_running() {
    let mut world = PhysicsWorld::new();
    let target = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("static body");
    world
        .add_collider(
            target,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                transform(Vec3::new(5.0, 0.0, 0.0)),
            ),
        )
        .expect("static collider");
    let mut simulation = PhysicsSimulation::with_config(world, config()).expect("simulation");
    let client = simulation.client();
    simulation.start().expect("start");

    let ray_hit = tokio::time::timeout(
        Duration::from_secs(2),
        client.raycast(
            Ray::new(Vec3::ZERO, Vec3::X, 10.0).expect("ray"),
            QueryFilter::default(),
        ),
    )
    .await
    .expect("ray query is processed")
    .expect("ray query succeeds")
    .expect("ray hits the static body");
    assert_eq!(ray_hit.body, target);

    let body = tokio::time::timeout(Duration::from_secs(2), client.add_body(dynamic(Vec3::ZERO)))
        .await
        .expect("body command is processed")
        .expect("body command succeeds");
    client
        .set_linear_velocity(body, Vec3::X)
        .await
        .expect("velocity command succeeds");
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let snapshot = client.snapshot();
            if snapshot
                .body(body)
                .is_some_and(|state| state.linear_velocity() == Vec3::X)
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    })
    .await
    .expect("running snapshot reflects the command");

    simulation.stop().await.expect("stop");
}
