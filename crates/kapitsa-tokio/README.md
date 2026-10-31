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

`metrics()` returns the latest ready snapshot without waiting. `actual_hz` is
computed from recent completion timestamps. `step_budget` accounts for target
utilization; `deadline` is the full physics interval used internally for step
measurements. `budget_overruns` and `deadline_misses` count those limits
separately. `simulation_time_debt` reports whole-step lag and excludes the
normal fractional remainder retained by the scheduler.

## Manual stepping

`PhysicsController` is the lower-level API when the application should decide
when to run each step. Every `step()` call performs exactly one fixed step; it
does not accept elapsed time or run catch-up steps. The one-worker path calls
`PhysicsWorld::step(dt)` on Tokio's blocking pool. With multiple workers, the
controller schedules bounded buckets and atomically commits their results.

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
