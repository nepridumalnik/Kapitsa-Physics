use std::{
    collections::HashSet,
    future::Future,
    num::NonZeroUsize,
    pin::Pin,
    task::{Context, Poll, Waker},
    time::Duration,
};

#[cfg(feature = "bench-internals")]
use std::time::Instant;

#[cfg(feature = "bench-internals")]
use std::{
    alloc::{GlobalAlloc, Layout, System},
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use kapitsa::math::{Mat3, Vec3};
use kapitsa::query::{QueryFilter, Ray};
use kapitsa::shape::Sphere;
use kapitsa::{
    Collider, MassProperties, PhysicsClient, PhysicsClientError, PhysicsSnapshot,
    PhysicsSubmitError, RigidBody, Shape, Simulation, SimulationConfig, Transform, WorkerConfig,
};
use tokio::task::JoinSet;

#[cfg(feature = "bench-internals")]
static TRACK_ALLOCATIONS: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "bench-internals")]
static ALLOCATION_COUNT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "bench-internals")]
static ALLOCATION_BYTES: AtomicUsize = AtomicUsize::new(0);

#[cfg(feature = "bench-internals")]
struct TrackingAllocator;

#[cfg(feature = "bench-internals")]
// SAFETY: all allocator operations delegate to the system allocator.
unsafe impl GlobalAlloc for TrackingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() && TRACK_ALLOCATIONS.load(std::sync::atomic::Ordering::Relaxed) {
            ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            ALLOCATION_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc_zeroed(layout) };
        if !pointer.is_null() && TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
            ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            ALLOCATION_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let pointer = unsafe { System.realloc(pointer, layout, size) };
        if !pointer.is_null() && TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
            ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            ALLOCATION_BYTES.fetch_add(size, Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) };
    }
}

#[cfg(feature = "bench-internals")]
#[global_allocator]
static TEST_ALLOCATOR: TrackingAllocator = TrackingAllocator;

#[cfg(feature = "bench-internals")]
struct AllocationTracking;

#[cfg(feature = "bench-internals")]
impl AllocationTracking {
    fn start() -> Self {
        ALLOCATION_COUNT.store(0, Ordering::Relaxed);
        ALLOCATION_BYTES.store(0, Ordering::Relaxed);
        TRACK_ALLOCATIONS.store(true, Ordering::Relaxed);
        Self
    }
}

#[cfg(feature = "bench-internals")]
impl Drop for AllocationTracking {
    fn drop(&mut self) {
        TRACK_ALLOCATIONS.store(false, Ordering::Relaxed);
    }
}

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
    assert!(client.snapshot().body(id).is_some());
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
    assert!(client.snapshot().body(id).is_none());
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
async fn command_buffers_are_ordered_atomic_and_report_step_results() {
    let mut simulation = simulation();
    let mut client = simulation.client();
    let mut results = client.subscribe_batch_results();
    simulation.start().expect("start");

    let dynamic = client.add_body(dynamic_body()).await.expect("dynamic body");
    let fixed = client
        .add_body(RigidBody::new_static(Transform::identity()))
        .await
        .expect("static body");
    let mut commands = client.commands();
    commands.set_mass(
        dynamic,
        MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass"),
    );
    commands.apply_impulse(dynamic, Vec3::X * 10.0);
    let accepted_id = client.try_submit(commands).expect("nonblocking submit");
    let applied = tokio::time::timeout(Duration::from_secs(5), results.next_result())
        .await
        .expect("batch result timeout")
        .expect("batch result");
    assert_eq!(applied.batch_id, accepted_id);
    assert!(applied.step > 0);
    assert_eq!(applied.result, Ok(()));

    let snapshot = next_snapshot(&mut client).await;
    assert_eq!(
        snapshot
            .body(dynamic)
            .expect("dynamic body")
            .linear_velocity(),
        Vec3::X * 5.0
    );

    let mut invalid = client.commands();
    invalid.set_linear_velocity(dynamic, Vec3::ZERO);
    invalid.set_mass(
        fixed,
        MassProperties::new(3.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass"),
    );
    let rejected_id = client.try_submit(invalid).expect("queued for validation");
    let rejected = tokio::time::timeout(Duration::from_secs(5), results.next_result())
        .await
        .expect("batch result timeout")
        .expect("batch result");
    assert_eq!(rejected.batch_id, rejected_id);
    assert_eq!(
        rejected.result,
        Err(kapitsa::PhysicsBatchError {
            command_index: Some(1),
            error: PhysicsClientError::Body(kapitsa::rigid_body::BodyError::BodyMustBeDynamic),
        })
    );

    let unchanged = next_snapshot(&mut client).await;
    assert_eq!(
        unchanged
            .body(dynamic)
            .expect("dynamic body")
            .linear_velocity(),
        Vec3::X * 5.0
    );
    simulation.stop().await.expect("stop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn single_commands_and_batches_keep_fifo_order_and_snapshot_completion() {
    let mut simulation = simulation();
    let client = simulation.client();
    let mut results = client.subscribe_batch_results();
    simulation.start().expect("start");
    let body = client.add_body(dynamic_body()).await.expect("body");

    let mut first = Box::pin(client.set_linear_velocity(body, Vec3::X));
    assert!(matches!(poll_once(first.as_mut()), Poll::Pending));
    let mut batch = client.commands();
    batch.set_linear_velocity(body, Vec3::Y);
    let batch_id = client.try_submit(batch).expect("batch accepted");
    let mut last = Box::pin(client.set_linear_velocity(body, Vec3::Z));
    assert!(matches!(poll_once(last.as_mut()), Poll::Pending));

    let (first_result, batch_result, last_result) =
        tokio::time::timeout(Duration::from_secs(5), async {
            tokio::join!(first.as_mut(), results.next_result(), last.as_mut())
        })
        .await
        .expect("all command completions arrive");
    first_result.expect("first single command");
    last_result.expect("last single command");
    let batch_result = batch_result.expect("batch result");
    assert_eq!(batch_result.batch_id, batch_id);
    assert_eq!(batch_result.result, Ok(()));
    let snapshot = client.snapshot();
    assert!(snapshot.step() >= batch_result.step);
    assert_eq!(
        snapshot
            .body(body)
            .expect("body snapshot")
            .linear_velocity(),
        Vec3::Z
    );
    simulation.stop().await.expect("stop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn spatial_query_observes_fifo_state_before_integration() {
    let mut simulation = simulation();
    let client = simulation.client();
    simulation.start().expect("start");
    let body = client.add_body(dynamic_body()).await.expect("body");
    client
        .add_collider(
            body,
            Collider::new(
                Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                Transform::identity(),
            ),
        )
        .await
        .expect("collider");

    let mut velocity = Box::pin(client.set_linear_velocity(body, Vec3::X * 100.0));
    assert!(matches!(poll_once(velocity.as_mut()), Poll::Pending));
    let ray = Ray::new(Vec3::new(0.0, 0.0, -2.0), Vec3::Z, 5.0).expect("ray");
    let mut query = Box::pin(client.raycast(ray, QueryFilter::default()));
    assert!(matches!(poll_once(query.as_mut()), Poll::Pending));

    let (velocity_result, hit) = tokio::time::timeout(Duration::from_secs(5), async {
        tokio::join!(velocity.as_mut(), query.as_mut())
    })
    .await
    .expect("mutation and query complete");
    velocity_result.expect("velocity command");
    let hit = hit.expect("query succeeds").expect("pre-step sphere hit");
    assert!((hit.distance - 1.5).abs() < 1.0e-10);
    assert!(
        client
            .snapshot()
            .body(body)
            .expect("published body")
            .transform()
            .translation()
            .x
            > 5.0
    );
    simulation.stop().await.expect("stop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn many_short_command_batches_are_processed_in_fifo_order() {
    const BATCH_COUNT: usize = 2_048;

    let mut simulation = simulation();
    let mut client = simulation.client();
    simulation.start().expect("start");
    let body = client.add_body(dynamic_body()).await.expect("body");

    for index in 1..=BATCH_COUNT {
        let expected = Vec3::X * index as f64;
        let mut commands = client.commands();
        commands.set_linear_velocity(body, expected);

        loop {
            match client.try_submit(commands) {
                Ok(_) => break,
                Err(PhysicsSubmitError::QueueFull(buffer)) => {
                    commands = buffer;
                    tokio::task::yield_now().await;
                }
                Err(error) => panic!("command batch submission failed: {error}"),
            }
        }
    }

    tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            if client
                .snapshot()
                .body(body)
                .is_some_and(|state| state.linear_velocity() == Vec3::X * BATCH_COUNT as f64)
            {
                break;
            }
            client
                .next_snapshot()
                .await
                .expect("simulation remains running");
        }
    })
    .await
    .expect("all short batches are applied");

    assert_eq!(
        client
            .snapshot()
            .body(body)
            .expect("body snapshot")
            .linear_velocity(),
        Vec3::X * BATCH_COUNT as f64
    );
    simulation.stop().await.expect("stop");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn stop_publishes_results_for_accepted_but_unapplied_batches() {
    let mut simulation = Simulation::new_from_config(SimulationConfig {
        tick_rate: 2,
        ..SimulationConfig::default()
    })
    .expect("simulation");
    let client = simulation.client();
    let mut results = client.subscribe_batch_results();
    simulation.start().expect("start");

    let body = client.add_body(dynamic_body()).await.expect("body");
    let mut commands = client.commands();
    commands.apply_impulse(body, Vec3::X);
    let batch_id = client.try_submit(commands).expect("accepted batch");
    simulation.stop().await.expect("stop");

    let result = results.next_result().await.expect("stopped batch result");
    assert_eq!(result.batch_id, batch_id);
    assert_eq!(
        result.result,
        Err(kapitsa::PhysicsBatchError {
            command_index: None,
            error: PhysicsClientError::NotRunning,
        })
    );
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

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn cloned_clients_submit_command_buffers_concurrently() {
    const CLIENTS: usize = 24;

    let mut simulation = simulation();
    let client = simulation.client();
    simulation.start().expect("start");
    let body = client.add_body(dynamic_body()).await.expect("body");
    let mut results = client.subscribe_batch_results();

    let mut clients = JoinSet::new();
    for index in 0..CLIENTS {
        let client = client.clone();
        clients.spawn(async move {
            let mut commands = client.commands();
            commands.set_linear_velocity(body, Vec3::new(index as f64 + 1.0, 0.0, 0.0));
            client.try_submit(commands).expect("submit command buffer")
        });
    }

    let mut submitted = HashSet::new();
    while let Some(result) = clients.join_next().await {
        submitted.insert(result.expect("client task"));
    }
    assert_eq!(submitted.len(), CLIENTS);

    let mut applied = HashSet::new();
    for _ in 0..CLIENTS {
        let result = tokio::time::timeout(Duration::from_secs(5), results.next_result())
            .await
            .expect("batch result timeout")
            .expect("batch result");
        assert!(result.result.is_ok());
        applied.insert(result.batch_id);
    }
    assert_eq!(applied, submitted);
    simulation.stop().await.expect("stop");
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
async fn cancelling_stop_wait_can_be_joined_and_restarted() {
    let mut simulation = simulation();
    let mut client = simulation.client();
    simulation.start().expect("start");
    assert!(next_snapshot(&mut client).await.step() > 0);

    let mut stop = Box::pin(simulation.stop());
    let _ = poll_once(stop.as_mut());
    drop(stop);
    simulation.stop().await.expect("finish cancelled stop");

    let previous_step = simulation.metrics().total_steps;
    simulation.start().expect("restart after cancelled stop");
    assert!(next_snapshot(&mut client).await.step() > previous_step);
    simulation.stop().await.expect("stop restarted simulation");
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

#[cfg(feature = "bench-internals")]
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "Release end-to-end command completion latency and allocation benchmark"]
async fn benchmark_runtime_completion_latency() {
    const WARMUP: usize = 20;
    const CYCLES_PER_SAMPLE: usize = 20;
    const SAMPLE_COUNT: usize = 5;

    let mut simulation = Simulation::new_from_config(SimulationConfig {
        tick_rate: 1_000,
        max_catch_up_steps: 2,
        workers: WorkerConfig {
            max_workers: NonZeroUsize::new(1),
            ..WorkerConfig::default()
        },
        ..SimulationConfig::default()
    })
    .expect("valid simulation configuration");
    let client = simulation.client();
    let mut results = client.subscribe_batch_results();
    simulation.start().expect("start");
    let body = client.add_body(dynamic_body()).await.expect("body");

    for index in 0..WARMUP {
        client
            .set_linear_velocity(body, Vec3::X * index as f64)
            .await
            .expect("warm single command");
    }
    for index in 0..WARMUP {
        let mut buffer = client.commands();
        for command in 0..128 {
            buffer.set_linear_velocity(body, Vec3::new((index * 128 + command) as f64, 1.0, 0.0));
        }
        let id = client.try_submit(buffer).expect("warm batch submit");
        let result = results.next_result().await.expect("warm batch result");
        assert_eq!(result.batch_id, id);
        assert!(result.result.is_ok());
    }

    let mut single_ack_means = Vec::with_capacity(SAMPLE_COUNT);
    let mut single_submit_means = Vec::with_capacity(SAMPLE_COUNT);
    let mut single_throughputs = Vec::with_capacity(SAMPLE_COUNT);
    for sample in 0..SAMPLE_COUNT {
        let sample_started = Instant::now();
        let mut ack_total = 0.0;
        let mut submit_total = 0.0;
        for cycle in 0..CYCLES_PER_SAMPLE {
            let mut request = std::pin::pin!(client.set_linear_velocity(
                body,
                Vec3::new((sample * CYCLES_PER_SAMPLE + cycle) as f64, 0.0, 0.0),
            ));
            let started = Instant::now();
            let submit_started = Instant::now();
            let first_poll = poll_once(request.as_mut());
            submit_total += submit_started.elapsed().as_nanos() as f64;
            match first_poll {
                Poll::Ready(result) => result.expect("single command"),
                Poll::Pending => request.as_mut().await.expect("single command"),
            }
            ack_total += started.elapsed().as_secs_f64() * 1_000_000.0;
        }
        single_ack_means.push(ack_total / CYCLES_PER_SAMPLE as f64);
        single_submit_means.push(submit_total / CYCLES_PER_SAMPLE as f64);
        single_throughputs.push(CYCLES_PER_SAMPLE as f64 / sample_started.elapsed().as_secs_f64());
    }

    let mut batch_ack_means = Vec::with_capacity(SAMPLE_COUNT);
    let mut batch_submit_means = Vec::with_capacity(SAMPLE_COUNT);
    let mut batch_throughputs = Vec::with_capacity(SAMPLE_COUNT);
    for sample in 0..SAMPLE_COUNT {
        let sample_started = Instant::now();
        let mut ack_total = 0.0;
        let mut submit_total = 0.0;
        for cycle in 0..CYCLES_PER_SAMPLE {
            let mut buffer = client.commands();
            for command in 0..128 {
                buffer.set_linear_velocity(
                    body,
                    Vec3::new(
                        (sample * CYCLES_PER_SAMPLE + cycle * 128 + command) as f64,
                        1.0,
                        0.0,
                    ),
                );
            }
            let started = Instant::now();
            let submit_started = Instant::now();
            let id = client.try_submit(buffer).expect("batch submit");
            submit_total += submit_started.elapsed().as_nanos() as f64;
            let result = results.next_result().await.expect("batch result");
            ack_total += started.elapsed().as_secs_f64() * 1_000_000.0;
            assert_eq!(result.batch_id, id);
            assert!(result.result.is_ok());
            assert!(client.snapshot().step() >= result.step);
        }
        batch_ack_means.push(ack_total / CYCLES_PER_SAMPLE as f64);
        batch_submit_means.push(submit_total / CYCLES_PER_SAMPLE as f64);
        batch_throughputs.push(CYCLES_PER_SAMPLE as f64 / sample_started.elapsed().as_secs_f64());
    }

    let tracking = AllocationTracking::start();
    for cycle in 0..CYCLES_PER_SAMPLE {
        client
            .set_linear_velocity(body, Vec3::X * cycle as f64)
            .await
            .expect("tracked single command");
    }
    drop(tracking);
    let single_allocations = ALLOCATION_COUNT.load(Ordering::Relaxed);
    let single_bytes = ALLOCATION_BYTES.load(Ordering::Relaxed);

    let tracking = AllocationTracking::start();
    for cycle in 0..CYCLES_PER_SAMPLE {
        let mut buffer = client.commands();
        for command in 0..128 {
            buffer.set_linear_velocity(body, Vec3::new((cycle * 128 + command) as f64, 1.0, 0.0));
        }
        let id = client.try_submit(buffer).expect("tracked batch submit");
        let result = results.next_result().await.expect("tracked batch result");
        assert_eq!(result.batch_id, id);
        assert!(result.result.is_ok());
    }
    drop(tracking);
    let batch_allocations = ALLOCATION_COUNT.load(Ordering::Relaxed);
    let batch_bytes = ALLOCATION_BYTES.load(Ordering::Relaxed);

    println!(
        "mode,commands_per_request,samples,min_mean_ack_us,median_mean_ack_us,max_mean_ack_us,median_submit_cpu_ns,median_requests_per_second,median_commands_per_second,allocations_per_request,requested_bytes_per_request"
    );
    for (mode, commands, ack, submit, throughput, allocations, bytes) in [
        (
            "single",
            1,
            single_ack_means,
            single_submit_means,
            single_throughputs,
            single_allocations,
            single_bytes,
        ),
        (
            "batch",
            128,
            batch_ack_means,
            batch_submit_means,
            batch_throughputs,
            batch_allocations,
            batch_bytes,
        ),
    ] {
        let (min_ack, median_ack, max_ack) = min_median_max(ack);
        let (_, median_submit, _) = min_median_max(submit);
        let (_, median_throughput, _) = min_median_max(throughput);
        println!(
            "{mode},{commands},{SAMPLE_COUNT},{min_ack:.3},{median_ack:.3},{max_ack:.3},{median_submit:.1},{median_throughput:.0},{:.0},{:.3},{:.1}",
            median_throughput * commands as f64,
            allocations as f64 / CYCLES_PER_SAMPLE as f64,
            bytes as f64 / CYCLES_PER_SAMPLE as f64,
        );
    }
    simulation.stop().await.expect("stop");
}

#[cfg(feature = "bench-internals")]
fn min_median_max(mut values: Vec<f64>) -> (f64, f64, f64) {
    values.sort_by(f64::total_cmp);
    (
        values[0],
        values[values.len() / 2],
        values[values.len() - 1],
    )
}
