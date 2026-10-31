use std::time::Duration;

use kapitsa_core::{
    collider::Collider,
    collision::{CollisionEvent, CollisionEventKind, QueryFilter, Ray},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Plane, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};
use kapitsa_tokio::{PhysicsController, PhysicsSimulation, SchedulerConfig};

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
    for x in [-2.0, 2.0] {
        let mut ball = dynamic(Vec3::new(x, 0.49, 0.0));
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
    }
    world
}

fn assert_worlds_near(actual: &PhysicsWorld, expected: &PhysicsWorld) {
    let actual_bodies = actual.bodies().collect::<Vec<_>>();
    let expected_bodies = expected.bodies().collect::<Vec<_>>();
    assert_eq!(actual_bodies.len(), expected_bodies.len());
    for ((actual_id, actual_body), (expected_id, expected_body)) in
        actual_bodies.into_iter().zip(expected_bodies)
    {
        assert_eq!(actual_id.index(), expected_id.index());
        let actual = actual_body.state();
        let expected = expected_body.state();
        assert!(
            (actual.transform().translation() - expected.transform().translation()).length()
                < 1.0e-8
        );
        assert!((actual.linear_velocity() - expected.linear_velocity()).length() < 1.0e-8);
        assert!((actual.angular_velocity() - expected.angular_velocity()).length() < 1.0e-8);
        assert_eq!(actual.is_sleeping(), expected.is_sleeping());
    }

    let actual_events = actual.collision_events();
    let expected_events = expected.collision_events();
    assert_eq!(actual_events.len(), expected_events.len());
    for (actual_event, expected_event) in actual_events.iter().zip(expected_events) {
        assert_event_near(actual_event, expected_event, actual, expected);
    }
}

fn assert_event_near(
    actual: &CollisionEvent,
    expected: &CollisionEvent,
    actual_world: &PhysicsWorld,
    expected_world: &PhysicsWorld,
) {
    assert_eq!(actual.kind, expected.kind);
    assert_eq!(actual.sensor, expected.sensor);
    let actual_a = actual_world
        .collider_body(actual.pair.a)
        .expect("actual pair collider A");
    let actual_b = actual_world
        .collider_body(actual.pair.b)
        .expect("actual pair collider B");
    let expected_a = expected_world
        .collider_body(expected.pair.a)
        .expect("expected pair collider A");
    let expected_b = expected_world
        .collider_body(expected.pair.b)
        .expect("expected pair collider B");
    assert_eq!(actual_a.index(), expected_a.index());
    assert_eq!(actual_b.index(), expected_b.index());
    match (actual.contact, expected.contact) {
        (Some(actual), Some(expected)) => {
            assert!((actual.point - expected.point).length() < 1.0e-8);
            assert!((actual.normal - expected.normal).length() < 1.0e-8);
            assert!((actual.penetration - expected.penetration).abs() < 1.0e-8);
            assert!((actual.normal_impulse - expected.normal_impulse).abs() < 1.0e-8);
        }
        (None, None) => {}
        _ => panic!("collision event contact data differs"),
    }
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

#[tokio::test(start_paused = true)]
async fn collision_results_match_world_and_controller_worker_counts() {
    let step = config().target_hz;
    let dt = 1.0 / f64::from(step);
    let mut expected = collision_world();
    expected.step(dt).expect("sequential step");

    for workers in [1, 2, 4] {
        let mut worker_config = config();
        worker_config.max_workers = workers;
        let mut controller =
            PhysicsController::new(collision_world(), worker_config).expect("controller");
        controller
            .set_active_workers(workers)
            .expect("worker count");
        let stats = controller.step().await.expect("controller step");
        assert_eq!(stats.active_workers, workers);
        assert_worlds_near(controller.world().expect("controller world"), &expected);
    }

    let mut simulation =
        PhysicsSimulation::with_config(collision_world(), config()).expect("simulation");
    simulation.start().expect("start");
    tokio::task::yield_now().await;
    tokio::time::advance(Duration::from_millis(17)).await;
    tokio::task::yield_now().await;
    tokio::task::yield_now().await;
    simulation.stop().await.expect("stop");
    assert_worlds_near(simulation.world().expect("stopped world"), &expected);
}
