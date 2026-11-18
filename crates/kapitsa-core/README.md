# kapitsa-core

`kapitsa-core` is the synchronous rigid-body physics core of the Kapitsa engine.
It owns the physics world and steps it explicitly; scheduling, Tokio integration, and
client/server marshalling live in the `kapitsa` crate.

The crate models static, kinematic, and dynamic bodies with `f64` positions,
velocities, and mass properties. It provides convex and surface shapes, colliders with
filtering and materials, discrete and continuous collision detection, contact
resolution, joints, weld-based clusters, spatial queries, collision events, uniform and
point-source gravity, and aerodynamic load evaluation. The `command` module owns the
Tokio-free command buffer and the atomic apply path used by both host-driven loops and
the async service. There is no implicit gravity: apply forces or set initial velocities
when building a scene.

```rust
use kapitsa_core::{
    collider::Collider,
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Plane, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Vec3};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let floor = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        floor,
        Collider::new(
            Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0)?),
            Transform::identity(),
        ),
    )?;

    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let ball = world.add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)?;
    world.add_collider(
        ball,
        Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
    )?;

    world.step(1.0 / 60.0)?;
    Ok(())
}
```

## Role in the workspace

- `kapitsa-math` — vector, matrix, and quaternion primitives used by this crate.
- `kapitsa-core` — this crate; synchronous physics and collision state.
- `kapitsa` — asynchronous simulation runtime and the public engine API.

## Documentation

The repository documentation describes the collision model, joints, chunks, and
aerodynamics in more detail:

- [Collision system](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/collisions.md)
- [Joints](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/joints.md)
- [Chunks](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/chunks.md)
- [Aerodynamics](https://github.com/nepridumalnik/Kapitsa-Physics/blob/main/docs/aerodynamics.md)
- [Repository README](https://github.com/nepridumalnik/Kapitsa-Physics#readme)

## License

MIT. See [LICENSE.md](LICENSE.md).