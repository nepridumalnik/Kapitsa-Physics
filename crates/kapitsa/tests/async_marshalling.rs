use std::{
    collections::HashSet,
    future::Future,
    num::NonZeroUsize,
    pin::Pin,
    task::{Context, Poll, Waker},
    time::Duration,
};

use kapitsa::math::{Mat3, Vec3};
use kapitsa::{
    MassProperties, PhysicsClient, PhysicsClientError, PhysicsSnapshot, RigidBody, Simulation,
    SimulationConfig, Transform, WorkerConfig,
};
use tokio::task::JoinSet;

fn simulation() -> Simulation {
    Simulation::new_from_config(SimulationConfig {
        tick_rate: 10,
        max_catch_up_steps: 2,
        workers: WorkerConfig {
            max_workers: NonZeroUsize::new(1),
            ..WorkerConfig::default()
        },
        ..SimulationConfig::default()
    })
    .expect("valid simulation configuration")
}

fn dynamic_body() -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass");
    RigidBody::new_dynamic(Transform::identity(), mass).expect("valid body")
}

fn poll_once<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
    let waker = Waker::noop();
    future.poll(&mut Context::from_waker(waker))
}

async fn next_snapshot(client: &mut PhysicsClient) -> std::sync::Arc<PhysicsSnapshot> {
    tokio::time::timeout(Duration::from_secs(5), client.next_snapshot())
        .await
        .expect("snapshot publication timeout")
        .expect("simulation still running")
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn commands_steps_snapshots_remove_and_recreate_bodies() {
    let mut simulation = simulation();
    let mut client = simulation.client();
    assert!(matches!(
        client.add_body(dynamic_body()).await,
        Err(PhysicsClientError::NotRunning)
    ));

    simulation.start().expect("start");
    assert!(next_snapshot(&mut client).await.step() > 0);

    let id = tokio::time::timeout(Duration::from_secs(5), client.add_body(dynamic_body()))
        .await
        .expect("add body timeout")
        .expect("add body");
    let created = next_snapshot(&mut client).await;
    assert!(created.body(id).is_some());
    assert_eq!(created.len(), 1);

    client
        .apply_force(id, Vec3::Y * 10.0)
        .await
        .expect("apply force");
    let accelerated = next_snapshot(&mut client).await;
    assert!(accelerated.body(id).expect("body").linear_velocity().y > 0.0);
    let retained = accelerated.clone();

    client.remove_body(id).await.expect("remove body");
    let removed = next_snapshot(&mut client).await;
    assert!(removed.body(id).is_none());
    assert!(retained.body(id).is_some());

    let replacement = client
        .add_body(dynamic_body())
        .await
        .expect("recreate body");
    assert_eq!(replacement.index(), id.index());
    assert_ne!(replacement.generation(), id.generation());
    let recreated = next_snapshot(&mut client).await;
    assert!(recreated.body(id).is_none());
    assert!(recreated.body(replacement).is_some());

    simulation.stop().await.expect("stop");
    assert_eq!(client.snapshot().len(), 1);
    assert!(matches!(
        client.apply_force(replacement, Vec3::Y).await,
        Err(PhysicsClientError::NotRunning)
    ));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn cloned_clients_do_not_lose_concurrent_body_commands() {
    let mut simulation = simulation();
    let mut reader = simulation.client();
    simulation.start().expect("start");
    assert!(next_snapshot(&mut reader).await.step() > 0);

    let mut clients = JoinSet::new();
    for _ in 0..100 {
        let client = reader.clone();
        clients.spawn(async move {
            client
                .add_body(RigidBody::new_static(Transform::identity()))
                .await
        });
    }

    let ids = tokio::time::timeout(Duration::from_secs(20), async {
        let mut ids = Vec::new();
        while let Some(result) = clients.join_next().await {
            ids.push(result.expect("client task").expect("body command"));
        }
        ids
    })
    .await
    .expect("concurrent command timeout");
    assert_eq!(ids.len(), 100);
    assert_eq!(ids.iter().copied().collect::<HashSet<_>>().len(), 100);
    let snapshot = next_snapshot(&mut reader).await;
    assert_eq!(snapshot.len(), 100);
    assert!(ids.iter().all(|id| snapshot.body(*id).is_some()));

    simulation.stop().await.expect("stop");
    assert_eq!(reader.snapshot().len(), 100);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn queued_cancellation_applies_and_stop_discards_other_session_requests() {
    let mut simulation = simulation();
    let mut client = simulation.client();
    simulation.start().expect("start");
    assert!(next_snapshot(&mut client).await.step() > 0);

    let mut cancelled = Box::pin(client.add_body(RigidBody::new_static(Transform::identity())));
    assert!(matches!(poll_once(cancelled.as_mut()), Poll::Pending));
    drop(cancelled);
    assert_eq!(next_snapshot(&mut client).await.len(), 1);

    let prior_snapshot = client.snapshot();
    let mut pending = Box::pin(client.add_body(RigidBody::new_static(Transform::identity())));
    assert!(matches!(poll_once(pending.as_mut()), Poll::Pending));
    simulation.stop().await.expect("stop before next step");
    assert!(matches!(pending.await, Err(PhysicsClientError::NotRunning)));
    assert!(matches!(
        client.next_snapshot().await,
        Err(PhysicsClientError::NotRunning)
    ));
    assert_eq!(client.snapshot().len(), 1);

    simulation.start().expect("restart with fresh queue");
    let restarted = next_snapshot(&mut client).await;
    assert!(restarted.step() > prior_snapshot.step());
    assert!(restarted.simulation_time() > prior_snapshot.simulation_time());
    assert_eq!(restarted.len(), 1);
    simulation.stop().await.expect("second stop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn body_ids_remain_valid_across_simulation_restart() {
    let mut simulation = simulation();
    let client = simulation.client();

    simulation.start().expect("start");
    let id = client
        .add_body(RigidBody::new_static(Transform::identity()))
        .await
        .expect("body insertion");
    simulation.stop().await.expect("stop");
    assert!(client.snapshot().body(id).is_some());

    simulation.start().expect("restart");
    assert!(client.snapshot().body(id).is_some());
    simulation.stop().await.expect("second stop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn held_snapshots_stay_immutable_while_clients_receive_new_publications() {
    let mut simulation = simulation();
    let mut first_reader = simulation.client();
    let mut second_reader = simulation.client();
    simulation.start().expect("start");
    let mut body = RigidBody::new_kinematic(Transform::identity());
    body.set_linear_velocity(Vec3::X).expect("finite velocity");
    let id = first_reader.add_body(body).await.expect("body insertion");

    let first_task = tokio::spawn(async move { first_reader.next_snapshot().await });
    let second_task = tokio::spawn(async move { second_reader.next_snapshot().await });
    tokio::task::yield_now().await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    let first = first_task
        .await
        .expect("first reader task")
        .expect("snapshot");
    let second = second_task
        .await
        .expect("second reader task")
        .expect("snapshot");
    assert_eq!(first.step(), second.step());
    let held = first.clone();
    let held_position = held.body(id).expect("held body").transform().translation();

    for _ in 0..3 {
        let mut reader = simulation.client();
        let task = tokio::spawn(async move { reader.next_snapshot().await });
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        let current = task.await.expect("reader task").expect("new snapshot");
        assert!(current.step() > held.step());
        assert!(
            current
                .body(id)
                .expect("current body")
                .transform()
                .translation()
                .x
                > held_position.x
        );
    }

    assert_eq!(
        held.body(id)
            .expect("held body remains immutable")
            .transform()
            .translation(),
        held_position
    );
    assert!(simulation.metrics().total_steps >= 4);
    simulation.stop().await.expect("stop");
}
