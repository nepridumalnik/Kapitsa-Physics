# Simulation runtime

`kapitsa` exposes one clock-driven `Simulation`. The host application provides
the Tokio runtime; Kapitsa creates its worker pool internally. Construction
outside Tokio returns `SimulationError::MissingRuntime`.

```rust,no_run
use std::error::Error;

use kapitsa::math::{Mat3, Vec3};
use kapitsa::{
    MassProperties, RigidBody, Simulation, SimulationConfig, Transform,
};

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn Error>> {
    let mut simulation = Simulation::new_from_config(SimulationConfig::default())?;
    let physics = simulation.client();
    let mut results = physics.subscribe_batch_results();

    simulation.start()?;

    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let body = physics
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)
        .await?;
    let source = physics
        .add_force_source(body, Vec3::ZERO, Vec3::X * 10.0)
        .await?;

    let mut commands = physics.commands();
    commands.set_force_source(source, Vec3::ZERO, Vec3::Y * 10.0);
    let batch_id = physics.try_submit(commands)?;
    let result = results.next_result().await?;
    if result.batch_id != batch_id {
        return Err("received an unexpected command batch result".into());
    }
    result.result.map_err(|error| format!("batch rejected: {error:?}"))?;

    if let Some(state) = physics.snapshot().body(body) {
        println!("body velocity: {:?}", state.linear_velocity());
    }

    physics.remove_force_source(source).await?;
    simulation.stop().await?;
    Ok(())
}
```

Use `Simulation::new()` for the default `SimulationConfig`, or
`Simulation::new_from_config(config)` to set the fixed tick rate, catch-up
limit, scheduler, and worker settings. `start()` returns immediately.
`stop().await` stops scheduling steps, waits for active physics work, and
preserves the world so the same simulation can be started again.

`PhysicsClient` is cloneable and thread-safe. Use awaited methods for structural
operations such as adding or removing bodies, colliders, and force sources.
Successful world-changing calls complete only after their physics step succeeds
and the corresponding snapshot has been published. Thus, after `add_body`
returns, `snapshot().body(id)` is present; after `remove_body` returns, it is
absent. `next_snapshot()` can still skip later intermediate publications.

Use `PhysicsCommandBuffer` and `try_submit` for ordered changes during a
synchronous game tick. `try_submit()` means only that the complete buffer was
accepted into the bounded queue. A successful `PhysicsBatchResult` means the
batch was applied atomically, its step completed, and that step's snapshot was
published. The result's `step` identifies that publication; the latest snapshot
may have a greater step because snapshots can skip intermediate steps.

Command validation failures preserve the original error and command index, and
the batch makes no partial changes. `PhysicsClientError::StepFailedAfterApply`
means the command or batch did change the world, but the following physical step
failed; it includes the failed attempt number and `StepError` cause. Those
changes are not automatically rolled back, and no snapshot for the failed step
is published. `NotRunning` means the request was not applied before stopping.

Raycast, sweep, and overlap requests run at their FIFO position before the
following integration. Their result is computed once from the world at that
point, then delivered when that step attempt finishes. A query result is a read
of that earlier state, not a claim about the published post-step snapshot; a
query error remains distinct from a later integration error.

`SimulationConfig::central_gravity` optionally enables a point-mass field with
`CentralGravity::new(center, μ)`, where `μ = GM` is in m³/s². The field acts on
dynamic bodies at their world-space center of mass; static and kinematic bodies
are unaffected. The physical step stays fixed at `1 / tick_rate`. A body at the
field center, non-finite input, or an unrepresentable acceleration fails the
step atomically with a typed gravity cause. `PhysicsWorld` also exposes
`set_central_gravity` for synchronous world stepping.

`apply_impulse` changes velocity immediately at the next command boundary;
`apply_force` adds a one-step force that is cleared after a successful step. A
persistent `ForceSource` applies its body-local force at its body-local point on
every step until it is updated, disabled, or removed. Deleting its body removes
its force sources too.

Snapshots are immutable and may skip intermediate steps. Collision events are
published in bounded batches and report their committed step. Lagging event
subscribers receive an explicit `CollisionEventStreamError::Lagged` result.

For collision pipeline and worker measurements, see
[`benchmarks/collisions.md`](../benchmarks/collisions.md). The historical CCD
optimization details remain in the
[`CCD optimization report`](reviews/ccd-final-optimization-report.md).
