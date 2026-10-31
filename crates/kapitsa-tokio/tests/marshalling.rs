use std::{
    collections::HashSet,
    future::Future,
    pin::Pin,
    task::{Context, Poll, Waker},
    time::Duration,
};

use kapitsa_core::{
    mass::MassProperties, rigid_body::RigidBody, transform::Transform, world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Vec3};
use kapitsa_tokio::{
    PhysicsClientError, PhysicsSimulation, SchedulerConfig,
    marshalling::{PhysicsClient, PhysicsSnapshot},
};
use tokio::task::JoinSet;

fn simulation(world: PhysicsWorld) -> PhysicsSimulation {
    PhysicsSimulation::with_config(
        world,
        SchedulerConfig {
            target_hz: 10,
            min_hz: 10,
            max_workers: 1,
            max_catch_up_steps: 2,
            adaptive: false,
            ..SchedulerConfig::default()
        },
    )
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

#[tokio::test(start_paused = true)]
async fn commands_steps_snapshots_remove_and_recreate_bodies() {
    let mut simulation = simulation(PhysicsWorld::new());
    let mut client = simulation.client();
    assert!(matches!(
        client.add_body(dynamic_body()).await,
        Err(PhysicsClientError::NotRunning)
    ));

    simulation.start().expect("start");
    assert_eq!(next_snapshot(&mut client).await.step(), 0);

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
    assert_eq!(simulation.world().expect("world returned").body_count(), 1);
    assert!(matches!(
        client.apply_force(replacement, Vec3::Y).await,
        Err(PhysicsClientError::NotRunning)
    ));
}

#[tokio::test(start_paused = true)]
async fn cloned_clients_do_not_lose_concurrent_body_commands() {
    let mut simulation = simulation(PhysicsWorld::new());
    let mut reader = simulation.client();
    simulation.start().expect("start");
    assert_eq!(next_snapshot(&mut reader).await.step(), 0);

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
    assert_eq!(
        simulation.world().expect("world returned").body_count(),
        100
    );
}

#[tokio::test(start_paused = true)]
async fn queued_cancellation_applies_and_stop_discards_other_session_requests() {
    let mut simulation = simulation(PhysicsWorld::new());
    let mut client = simulation.client();
    simulation.start().expect("start");
    assert_eq!(next_snapshot(&mut client).await.step(), 0);

    let mut cancelled = Box::pin(client.add_body(RigidBody::new_static(Transform::identity())));
    assert!(matches!(poll_once(cancelled.as_mut()), Poll::Pending));
    drop(cancelled);
    assert_eq!(next_snapshot(&mut client).await.len(), 1);

    let prior_snapshot = client.snapshot();
    let mut pending = Box::pin(client.add_body(RigidBody::new_static(Transform::identity())));
    assert!(matches!(poll_once(pending.as_mut()), Poll::Pending));
    simulation.stop().await.expect("stop before next step");
    assert!(matches!(
        pending.await,
        Err(PhysicsClientError::ChannelClosed)
    ));
    assert!(matches!(
        client.next_snapshot().await,
        Err(PhysicsClientError::NotRunning)
    ));
    assert_eq!(simulation.world().expect("world returned").body_count(), 1);

    simulation.start().expect("restart with fresh queue");
    let restarted = next_snapshot(&mut client).await;
    assert_eq!(restarted.step(), prior_snapshot.step());
    assert_eq!(
        restarted.simulation_time(),
        prior_snapshot.simulation_time()
    );
    assert_eq!(restarted.len(), 1);
    simulation.stop().await.expect("second stop");
}
