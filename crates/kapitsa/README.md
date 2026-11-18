# kapitsa

`kapitsa` is the public API of the Kapitsa physics engine for space simulators and
games. It wraps the synchronous `kapitsa-core` world in an asynchronous runtime:
simulations advance on a dedicated scheduler with a Rayon compute pool, while clients
submit body, collider, joint, material, gravity, and chunk commands through a bounded
queue and read published snapshots.

## Role in the workspace

- `kapitsa-math` — `f64` vector, matrix, and quaternion primitives.
- `kapitsa-core` — synchronous physics world and collision state.
- `kapitsa` — this crate; simulation builder, physics client, snapshots, events, and
  the public re-exports of the lower-level types.

## Quick start

The example requires a Tokio runtime:

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