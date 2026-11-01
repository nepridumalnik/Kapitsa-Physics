# Simulation runtimes

The `kapitsa` crate always provides synchronous stepping. Tokio and Rayon are
available only with the optional `async` feature:

```toml
kapitsa = { version = "0.1", features = ["async"] }
```

## Manual stepping

Both runtime APIs advance exactly one fixed step per call. Manual stepping does
not start a timer or perform catch-up work.

```rust,ignore
use kapitsa::runtime::{SimulationConfig, sync::Simulation};

let mut simulation = Simulation::new(world, SimulationConfig::default())?;
let stats = simulation.step()?;
```

The async equivalent is `runtime::asynchronous::Simulation`; its `step().await`
runs CPU work on a reusable Rayon pool whose workers use the host Tokio
blocking pool. Cancelled step futures leave the in-flight step owned by the
simulation; the next `step().await` collects that result.

Async simulations are constructed inside a Tokio runtime with `new`, or with
an explicit host `Handle` using `new_on`. Call `shutdown().await` to wait for
worker exit and recover the world. `new_with_executor` allows multiple
simulations to share a host-owned executor.

`kapitsa-core` does not depend on Tokio or Rayon. It accepts a statically
dispatched `StepExecutor`; CCD trajectory samples are stored as core data, so
the collision path does not call a boxed or trait-object callback.

`WorkerConfig` limits the fixed pool and controls adaptive active parallelism.
The pool is created once; low or high load changes how many tasks a step may
use, not the number of OS workers. `SimulationMetrics` reports step duration,
budget use, active workers, pool size, deadline misses, and accumulated service
time debt.

## Clock-driven service

Use `asynchronous::PhysicsService` when the application needs automatic
fixed-rate stepping, catch-up, commands, snapshots, and collision events. This
is a separate API from manual `Simulation::step()`:

```rust,ignore
use kapitsa::runtime::{SimulationConfig, asynchronous::PhysicsService};

let mut service = PhysicsService::new(world, SimulationConfig::default())?;
let client = service.client();
service.start()?;

// Client operations are queued FIFO and applied at the next step boundary.
let body_id = client.add_body(body).await?;
let snapshot = client.snapshot();

service.stop().await?;
let world = service.shutdown().await?;
```

`start()` and `stop().await` can be repeated; stopping waits for an active step
to finish and leaves the worker pool available for restart. `shutdown()` stops
the service, waits for Rayon workers to exit, and returns the world. Dropping a
running service signals a best-effort stop; use `shutdown()` when orderly
worker teardown is required.

The command queue holds at most 1,024 requests and applies at most 1,024 at a
step boundary. `PhysicsClient` supports body/collider edits, forces, impulses,
and spatial queries. `PhysicsSnapshot` is an immutable view of body states;
readers can skip intermediate snapshots. Collision event batches are bounded
and identify their committed step. Lagging readers receive an explicit
`CollisionEventStreamError::Lagged` result.

Catch-up is only part of `PhysicsService`. `SimulationConfig::max_catch_up_steps`
limits each service batch; remaining elapsed time is processed in later
batches. The service reports the resulting simulation-time debt in metrics.

The current CCD and runtime benchmark results are in
[`benchmarks/collisions.md`](../benchmarks/collisions.md) and the
[`CCD optimization report`](reviews/ccd-final-optimization-report.md).
