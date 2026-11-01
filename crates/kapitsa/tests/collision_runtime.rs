use std::time::Duration;

use kapitsa::runtime::{
    CollisionEventBatch, PhysicsClient, Simulation, SimulationConfig, WorkerConfig,
};
use kapitsa_core::{
    collider::Collider,
    collision::{CollisionEventKind, QueryFilter, Ray},
    mass::MassProperties,
    rigid_body::{CcdMode, RigidBody},
    shape::{Cuboid, Shape, Sphere},
    transform::Transform,
    world::{BodyId, ColliderId},
};
use kapitsa_math::{Mat3, Quat, Vec3};

fn transform(position: Vec3) -> Transform {
    Transform::new(position, Quat::IDENTITY).expect("valid transform")
}

fn dynamic(position: Vec3) -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    RigidBody::new_dynamic(transform(position), mass).expect("dynamic body")
}

fn config(tick_rate: u32) -> SimulationConfig {
    SimulationConfig {
        tick_rate,
        max_catch_up_steps: 1,
        workers: WorkerConfig {
            max_workers: std::num::NonZeroUsize::new(2),
            ..WorkerConfig::default()
        },
        ..SimulationConfig::default()
    }
}

async fn next_batch_with(
    events: &mut kapitsa::runtime::CollisionEvents,
    kind: CollisionEventKind,
) -> CollisionEventBatch {
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let batch = events
                .next_batch()
                .await
                .expect("event stream remains open");
            if batch.events().iter().any(|event| event.kind == kind) {
                break batch;
            }
        }
    })
    .await
    .expect("collision event arrives")
}

async fn add_sphere(
    client: &PhysicsClient,
    position: Vec3,
    dynamic_body: bool,
) -> (BodyId, ColliderId) {
    let body = if dynamic_body {
        dynamic(position)
    } else {
        RigidBody::new_static(transform(position))
    };
    let body_id = client.add_body(body).await.expect("body added");
    let collider_id = client
        .add_collider(
            body_id,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                Transform::identity(),
            ),
        )
        .await
        .expect("collider added");
    (body_id, collider_id)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn snapshots_and_collision_batches_report_the_same_step() {
    let mut simulation = Simulation::new_from_config(config(120)).expect("simulation");
    let client = simulation.client();
    let mut events = client.subscribe_collision_events();
    simulation.start().expect("start");

    add_sphere(&client, Vec3::ZERO, false).await;
    let (dynamic_id, _) = add_sphere(&client, Vec3::new(0.9, 0.0, 0.0), true).await;
    let batch = next_batch_with(&mut events, CollisionEventKind::Begin).await;
    assert!(batch.step() > 0);
    let snapshot = client.snapshot();
    assert_eq!(batch.step(), snapshot.step());
    assert!(snapshot.body(dynamic_id).is_some());

    simulation.stop().await.expect("stop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn client_queries_and_body_commands_work_while_running() {
    let mut simulation = Simulation::new_from_config(config(120)).expect("simulation");
    let client = simulation.client();
    simulation.start().expect("start");

    let target = client
        .add_body(RigidBody::new_static(Transform::identity()))
        .await
        .expect("static body");
    client
        .add_collider(
            target,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                transform(Vec3::new(5.0, 0.0, 0.0)),
            ),
        )
        .await
        .expect("static collider");

    let hit = client
        .raycast(
            Ray::new(Vec3::ZERO, Vec3::X, 10.0).expect("ray"),
            QueryFilter::default(),
        )
        .await
        .expect("ray query succeeds")
        .expect("ray hits the static body");
    assert_eq!(hit.body, target);

    let body = client
        .add_body(dynamic(Vec3::ZERO))
        .await
        .expect("dynamic body");
    client
        .set_linear_velocity(body, Vec3::X)
        .await
        .expect("velocity command");
    assert!(client.snapshot().body(body).is_some());
    simulation.stop().await.expect("stop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn continuous_body_does_not_tunnel_through_a_thin_wall() {
    let mut simulation = Simulation::new_from_config(config(100)).expect("simulation");
    let client = simulation.client();
    let mut events = client.subscribe_collision_events();
    simulation.start().expect("start");

    let wall = client
        .add_body(RigidBody::new_static(Transform::identity()))
        .await
        .expect("wall body");
    client
        .add_collider(
            wall,
            Collider::new(
                Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0).expect("wall")),
                Transform::identity(),
            ),
        )
        .await
        .expect("wall collider");

    let body = client
        .add_body(dynamic(Vec3::new(5.0, 0.0, 0.0)))
        .await
        .expect("moving body");
    client
        .add_collider(
            body,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                Transform::identity(),
            ),
        )
        .await
        .expect("moving collider");
    client
        .set_ccd_mode(body, CcdMode::Continuous)
        .await
        .expect("enable CCD");
    client
        .set_linear_velocity(body, Vec3::new(-2000.0, 0.0, 0.0))
        .await
        .expect("set velocity");

    let batch = next_batch_with(&mut events, CollisionEventKind::Impact).await;
    let snapshot = client.snapshot();
    let state = snapshot.body(body).expect("moving body state");
    assert!(state.transform().translation().x > 0.0);
    assert!(state.linear_velocity().x > -2000.0);
    assert!(batch.step() <= snapshot.step());
    simulation.stop().await.expect("stop");
}
