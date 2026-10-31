# kapitsa-tokio

`PhysicsSimulation` owns a world and advances it in the background at a fixed,
adaptive rate. It uses the application's Tokio runtime and never creates a
runtime of its own.

```rust
use kapitsa_core::world::PhysicsWorld;
use kapitsa_tokio::{PhysicsSimulation, SimulationError};

async fn run(world: PhysicsWorld) -> Result<(), SimulationError> {
    let mut simulation = PhysicsSimulation::new(world);
    simulation.start()?;

    let metrics = simulation.metrics();
    println!("{} steps at {:.1} Hz", metrics.total_steps, metrics.actual_hz);

    simulation.stop().await?;
    Ok(())
}
```

`start()` requires an active Tokio runtime. Repeated starts while running and
repeated stops are safe. After stopping, `world()` and `world_mut()` provide
access to the world. They return `WorldUnavailable` while the simulation owns
it. A stop future can be cancelled and called again later; an already-started
physics step is allowed to finish before stopping completes.
Dropping the simulation requests a best-effort stop without waiting. Call
`stop().await` to recover the world and wait for completion; if the application
destroys its Tokio runtime first, detached work cannot be guaranteed to finish.

Use `PhysicsSimulation::with_config(world, config)?` to supply a validated
`SchedulerConfig`. The default constructor remains available without configuration.
`max_catch_up_steps` limits each scheduling batch; excess accumulated time is
retained and the task yields before the next batch. Accumulator overflow is
reported as `SimulationError::SimulationTimeOverflow`.

`metrics()` returns the latest ready snapshot without waiting. `actual_hz` is
computed from recent completion timestamps. `step_budget` accounts for target
utilization; `deadline` is the full physics interval used internally for step
measurements. `budget_overruns` and `deadline_misses` count those limits
separately. `actual_hz` is zero while stopped and until two steps complete in a
new run. Cumulative counters survive restarts. `simulation_time_debt` is the
accumulated time exceeding the next step interval after a completed step; a
normal remainder below one interval is retained internally but reported as zero.

## Physics client and snapshots

`PhysicsSimulation::client()` returns a cloneable, `Send + Sync` `PhysicsClient`
that can be kept before, during, and after a run. During a run, the simulation
task remains the only writer to `PhysicsWorld`. Client commands travel through
a bounded queue and are applied FIFO at the next scheduled step boundary,
before integration. They do not trigger an off-schedule step. At most 1,024
commands are applied per step; a full queue makes senders wait for capacity.

Each command completes after its world operation has been applied. Add
operations return the new world-scoped ID; other successful operations return
unit. `PhysicsClientError::World` and `PhysicsClientError::Body` preserve
operation failures such as stale IDs or invalid body operations. Assigning a
collider material keeps the core API's existing behavior and does not validate
that the material exists.

`snapshot()` clones an `Arc` to the latest immutable `PhysicsSnapshot` without
waiting. `next_snapshot().await` waits for a later publication, and a slow
reader may skip intermediate snapshots. Each call starts from the latest
published version and waits for a publication after the call begins, even if
the receiver had an unread version. Snapshots contain copied `BodyState`
values, not references to the world; old `Arc`s remain valid. A snapshot can
lag a command that just completed because publication happens only after the
next successful physics step. Initial snapshots describe the current world
and step count, including edits made through `world_mut()` while stopped.

Snapshots store live bodies densely and use a hash index keyed by the full
world-scoped body ID. The simulation task recycles a prior snapshot's buffers
when its `Arc` has no other owners. A reader may retain old frames; publication
does not wait for those readers and never changes a published snapshot. In that
case the task drops that buffer when possible and builds into another one.

```rust
use kapitsa_core::{
    rigid_body::RigidBody,
    world::PhysicsWorld,
};
use kapitsa_math::Vec3;
use kapitsa_tokio::PhysicsSimulation;

async fn run(
    world: PhysicsWorld,
    rocket: RigidBody,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut simulation = PhysicsSimulation::new(world);
    let physics = simulation.client();

    simulation.start()?;
    let rocket_id = physics.add_body(rocket).await?;
    physics.apply_force(rocket_id, Vec3::new(0.0, 1000.0, 0.0)).await?;

    let snapshot = physics.snapshot();
    if let Some(body) = snapshot.body(rocket_id) {
        println!("{:?}", body.transform());
    }

    simulation.stop().await?;
    Ok(())
}
```

Cancelling a command future before its bounded send succeeds prevents the
command from entering the queue. Cancelling after enqueueing only stops waiting
for its reply; the simulation still applies the command in FIFO order. Stopping
unpublishes the current sender, waits for an active step, and drops any
unapplied requests. Each restart creates a new queue, so requests from an older
run cannot carry forward. Call `stop().await` to recover the world; dropping
the simulation only requests a best-effort stop as described above.

## Manual stepping

`PhysicsController` is the lower-level API when the application should decide
when to run each step. Every `step()` call performs exactly one fixed step; it
does not accept elapsed time or run catch-up steps. The one-worker path calls
`PhysicsWorld::step(dt)` on Tokio's blocking pool. With multiple workers, the
controller schedules bounded buckets and atomically commits their results.
When at least half the bodies are static or sleeping, the controller instead
uses the atomic sequential step within a blocking task. This avoids full-body
bucket snapshots for those workloads; the chosen worker count remains the
scheduler's configured concurrency.

`PhysicsWorld::step(dt)` remains available for synchronous use without Tokio.

## Adaptive policy

The default simulation targets 60 Hz and uses 75% of each fixed interval as its
working budget. The overload threshold is that budget multiplied by
`1 + overload_tolerance`. The regulator updates its bounded integral as
`clamp(error + min(max(ratio - 1, 0), 1) - max(1 - ratio, 0), 0, 4)`, where
`ratio = duration / threshold`.
It changes settings only after at least three consecutive over-threshold steps
and an integral of at least `2.2`.

Worker increases double the current count up to `max_workers`. Each increase is
measured over 12 completed steps and retained only when average full-step time
improves by at least 5%; otherwise the prior count is restored. An ineffective
increase is not retried until the integral clears below `0.4` or measured load
rises by at least 25%. Changes have a 12-step stabilization period.

After parallelism is exhausted or rejected, frequency can decrease through
`60, 50, 40, 30 Hz`, filtered by the configured target and minimum. Recovery
requires 24 consecutive steps below 55% of the working budget; frequency rises
one level at a time before worker count is reduced. Setting `adaptive` to
`false` keeps the configured frequency and selected worker count fixed.

The manual controller does not track wall-clock time. Its future may be
cancelled safely; the next `step()` call waits for the in-flight step and
reports its result. The background simulation owns wall-clock accumulation and
does not count time spent stopped as simulation debt.
