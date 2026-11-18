# kapitsa

`kapitsa` is the public API of the Kapitsa physics engine for space simulators and
games. `PhysicsWorld` is the synchronous host-driven primitive: create bodies and
colliders, step the world, and read committed state directly. The optional `service`
feature adds the asynchronous simulation runtime that advances the world on a
scheduler and exposes it through a physics client and published snapshots.

## Features

- `default` — synchronous `PhysicsWorld` API only; no Tokio or Rayon dependency.
- `service` — the clock-driven `Simulation` service, `PhysicsClient`, snapshots, and
  command submission. Implies `parallel`.
- `parallel` — Rayon-backed step execution for host-driven synchronous stepping.
  Without it, steps use the sequential executor.

## Role in the workspace

- `kapitsa-math` — `f64` vector, matrix, and quaternion primitives.
- `kapitsa-core` — synchronous physics world and collision state.
- `kapitsa` — this crate; the public re-exports of the lower-level types plus the
  optional async service.

## Quick start

The service example requires a Tokio runtime and the `service` feature:

```rust
use kapitsa::{RigidBody, Simulation, Transform};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut simulation = Simulation::builder()
        .frequency(120)
        .max_workers(4)
        .spawn_on_tokio(&tokio::runtime::Handle::current())?;

    let client = simulation.client();
    let ground = client.add_body(RigidBody::new_static(Transform::identity())).await?;
    let snapshot = client.snapshot();
    assert!(snapshot.body(ground).is_some());

    simulation.pause()?;
    // Client commands continue to execute while the simulation is paused.
    simulation.resume()?;
    simulation.stop().await?;
    Ok(())
}
```

For host-driven synchronous stepping, use `PhysicsWorld` directly:

```rust
use kapitsa::{PhysicsWorld, RigidBody, Transform};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.step(1.0 / 60.0)?;
    assert!(world.body(body).is_some());
    Ok(())
}
```

See the [repository README](https://github.com/nepridumalnik/Kapitsa-Physics#readme)
for the full guide: bodies and state, colliders and shapes, streaming mesh chunks,
joints and clusters, materials and gravity, queries and events, and aerodynamics.

## Documentation

- [Collision system](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/collisions.md)
- [Joints](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/joints.md)
- [Chunks](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/chunks.md)
- [Aerodynamics](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/aerodynamics.md)
- [API documentation](https://docs.rs/kapitsa)

## License

MIT. See [LICENSE.md](LICENSE.md).