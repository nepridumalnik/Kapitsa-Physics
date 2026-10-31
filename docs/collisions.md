# Collision system

`kapitsa-core` owns collision state as part of `PhysicsWorld`. Add bodies and
colliders to the world, then advance the complete simulation with
`PhysicsWorld::step(dt)`. The step predicts motion, updates the spatial index,
generates filtered candidate pairs, builds contacts, resolves CCD and discrete
contacts, stages lifecycle events, validates results, and commits once. A
collision error leaves body states, accumulated forces, cached contacts, and
published events at their previous committed values.

There is no implicit gravity. Apply forces or set initial velocities when
building a falling or moving scene. Positions, sizes, mass properties, forces,
and velocities use consistent application-defined units; calculations use
`f64`.

## Quick start

This example adds a solid plane, a continuous dynamic sphere, a material, then
steps and queries the world. Errors are returned to the caller.

```rust,no_run
use std::error::Error;

use kapitsa_core::{
    collider::{Collider, CollisionMode},
    collision::{CollisionEventKind, QueryFilter, Ray},
    mass::MassProperties,
    material::PhysicsMaterial,
    rigid_body::{CcdMode, RigidBody},
    shape::{Plane, Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

fn simulate() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let floor = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let mut floor_collider = Collider::new(
        Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0)?),
        Transform::identity(),
    );
    floor_collider.set_mode(CollisionMode::Solid);
    world.add_collider(floor, floor_collider)?;

    let material = world.add_material(PhysicsMaterial::new(0.6, 0.8, 0.2, 1.0)?)?;
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let mut ball = RigidBody::new_dynamic(
        Transform::new(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY)?,
        mass,
    )?;
    ball.set_linear_velocity(Vec3::new(0.0, -20.0, 0.0))?;
    ball.set_ccd_mode(CcdMode::Continuous);
    let ball_id = world.add_body(ball)?;
    let ball_collider = world.add_collider(
        ball_id,
        Collider::new(
            Shape::Sphere(Sphere::new(0.5)?),
            Transform::identity(),
        ),
    )?;
    world.set_collider_material(ball_collider, Some(material))?;

    world.step(1.0 / 60.0)?;
    for event in world.collision_events() {
        if event.kind == CollisionEventKind::Impact {
            println!("impact: {:?}", event.contact);
        }
    }

    let ray = Ray::new(Vec3::new(0.0, 5.0, 0.0), -Vec3::Y, 10.0)?;
    if let Some(hit) = world.raycast(ray, QueryFilter::default())? {
        println!("hit {:?} at {:?}", hit.collider, hit.position);
    }
    Ok(())
}
```

`PhysicsWorld::collision_events()` contains the events from the latest
successful step. `Raycast` returns the nearest hit that passes its query filter.

## Shape support

Dynamic bodies can use convex geometry only. The supported pair set is the
convex-convex and convex-surface cross product below; reversing A and B is also
supported, with contact orientation reversed. This covers 28 unordered convex
pair types (including same-type pairs) and 42 convex-to-surface pair types.

| Category | `Shape` variants |
|---|---|
| Convex | `Sphere`, `Cuboid`, `Capsule`, `Cylinder`, `TaperedCapsule`, `TaperedCylinder`, `Convex` |
| Surface | `Plane`, `Triangle`, `TriangleMesh`, `HeightField`, `LevelSet`, `SkinnedTriangleMesh` |

Convex shapes collide with every convex shape and every surface shape in the
table. Surface-to-surface pairs are unsupported. A non-convex collider on a
dynamic body is rejected when it is added. Static and kinematic bodies may own
surface colliders.

The narrow phase uses support mapping with GJK distance/intersection and EPA
penetration for convex pairs, plane contact generation, triangle contacts,
mesh BVH traversal, heightfield cell traversal, and interpolated level-set
sampling. Skinned meshes use the current deformed pose and a refitted BVH.
Contacts are reduced into persistent manifolds with feature IDs; contact
impulses are cached for warm starting. Dynamic bodies connected by contacts are
grouped into deterministically ordered islands.

## Bodies, colliders, and materials

`RigidBody` can be Static, Kinematic, or Dynamic. A collider has a local
transform, a solid or sensor mode, a symmetric layer/mask filter, and an
optional material ID. Multiple colliders can belong to one body. A sensor
reports overlap lifecycle events but does not generate solver contacts or
impact impulses.

Use `PhysicsWorld::add_material` and `set_collider_material` to assign a
world-owned material. The default material is friction `0.5`, static friction
`0.0`, restitution `0.1`, and density `1.0`. Pair friction coefficients are
combined by geometric mean; pair restitution is the larger value. Density is
stored on the material but does not create mass properties automatically;
dynamic bodies receive explicit `MassProperties`. Removing a material that is
still assigned to a collider returns `WorldError::MaterialInUse`.

`CollisionFilter::new(layers, mask)` requires mutual permission:

```text
left.layers & right.mask != 0 && right.layers & left.mask != 0
```

Query filters have their own `layers` and `mask` with the same interaction
rule. `QueryFilter::default()` queries layer 0, accepts all collider masks,
excludes no body, and omits sensors.

## Solver and configuration

`PhysicsWorld::collision_config()` returns the current `CollisionConfig`;
`set_collision_config` validates a replacement. Defaults are 8 velocity
iterations, 3 position iterations, `0.01` contact offset, `0.001` penetration
slop, `1.0` restitution threshold, and 8 CCD impacts per continuous body.
Iteration counts and the impact limit must be positive; distances and threshold
must be finite and nonnegative; penetration slop cannot exceed contact offset.

The solver applies normal impulses, friction, restitution, warm starting, and
bounded positional correction. Finite iterations and slop permit small
residual penetration; exact zero penetration is not guaranteed. Bodies can be
put to sleep explicitly and are woken by relevant state changes and contacts.

Set `RigidBody::set_ccd_mode(CcdMode::Continuous)` on bodies that need
continuous collision detection. Swept broadphase bounds feed time-of-impact
queries for the supported convex-convex and convex-surface pairings. The
continuous path handles moving and rotating transforms over the step and
resolves impacts in chronological order. Exceeding `max_ccd_impacts` returns
`CollisionError::CcdLimitExceeded` without committing the step.

## Queries

- `PhysicsWorld::raycast` returns the closest `RayHit` or `None`.
- `PhysicsWorld::sweep` moves a convex query shape by a displacement and
  returns its earliest `ShapeHit` or `None`.
- `PhysicsWorld::overlaps` writes sorted `OverlapHit` values into a caller
  buffer; it replaces the buffer only after a successful query.

All queries read the current committed transforms, honor collision filters,
and return `QueryError` for invalid input, unsupported shapes, arithmetic
failures, or non-convergent geometry. Sweep and overlap query shapes must be
convex. Query traversal currently checks colliders directly rather than using
the step broadphase index; see the measured query workload in
`benchmarks/collisions.md`.

## Events and errors

Events are sorted by event kind and canonical collider pair. A newly active
pair emits `Begin`; an ongoing pair emits `Persist`; a separated pair emits
`End`. A solid contact with a positive new impulse can also emit `Impact`.
Sensors carry no contact data and do not emit `Impact`. Event changes are
staged with contacts and become visible only after a successful commit.

Step failures are returned as `StepError`, including `StepError::Collision`
with a `CollisionError` such as `InvalidMaterial`, `UnsupportedGeometryPair`,
`NonConvergent`, or `CcdLimitExceeded`. Query failures use the separate
`QueryError` type. Errors are not converted to empty hits or ignored contacts.

## Tokio integration

Use `PhysicsController` for explicit one-step-at-a-time control, or
`PhysicsSimulation` for a background fixed-step loop on the application's
Tokio runtime. The simulation task owns the world while running; do not share a
world behind a mutex. Its `PhysicsClient` applies typed commands through one
bounded FIFO queue at step boundaries. The queue holds up to 1,024 requests,
and no more than 1,024 commands are applied in one step.

Snapshots are immutable, nonblocking views and can skip intermediate steps.
Collision events use a separate bounded stream of 64 step batches. Match the
two channels by their shared `step` number; a slow event reader receives an
explicit `Lagged { missed_batches }` error. Commands and query requests are
serialized through the same FIFO path.

```rust,ignore
use std::error::Error;

use kapitsa_core::world::PhysicsWorld;
use kapitsa_tokio::PhysicsSimulation;

async fn run(world: PhysicsWorld) -> Result<(), Box<dyn Error>> {
    let mut simulation = PhysicsSimulation::new(world);
    let client = simulation.client();
    let mut events = client.subscribe_collision_events();
    simulation.start()?; // call from within a Tokio runtime

    let batch = events.next_batch().await?;
    let snapshot = client.snapshot();
    println!(
        "events for step {}; latest snapshot {}",
        batch.step(),
        snapshot.step()
    );

    simulation.stop().await?;
    Ok(())
}
```

See `crates/kapitsa-tokio/README.md` for command, snapshot, cancellation,
scheduling, and shutdown behavior. The measured collision pipeline and worker
comparisons are in `benchmarks/collisions.md`.

## Scope

This collision system does not add gravity, joints, or object destruction.
Those remain separate physics systems. Numerical algorithms can return
`NonConvergent` for geometry they cannot resolve within their iteration
bounds; such errors abort the entire step.
