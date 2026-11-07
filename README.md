# Kapitsa Physics

Kapitsa Physics is a physics engine written in Rust for space simulators, including orbital motion, rockets, multipart spacecraft, and collisions. It simulates rigid bodies, joints, contacts, and central gravity. Its public API is provided by the `kapitsa` crate.

## Integration

For a project within the workspace:

```toml
[dependencies]
kapitsa = { path = "../kapitsa" }
tokio = { version = "1", features = ["macros", "rt-multi-thread"] }
```

Set the path to `crates/kapitsa` according to your project's location. The example below requires a Tokio runtime.

## Creating and Controlling a Simulation

```rust
use kapitsa::{Simulation, RigidBody, Transform};

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

`frequency` sets the base frequency in Hz; the default is 60. With a fixed time step, each step advances simulation time by `1 / frequency` seconds. The valid range is 1 to 1,000,000,000 Hz.

`max_workers` optionally limits the number of computation workers; zero returns an error. By default, the available system parallelism is used. The scheduler automatically adjusts the number of active workers within the pool based on the measured step duration. The pool is not resized on every load change.

`time_step_policy` is optional and defaults to `TimeStepPolicy::Fixed`. To use adaptive time stepping, specify `.time_step_policy(TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default()))`.

`pause` stops physics steps while preserving the current time multiplier; `resume` restores it. While paused, commands are applied and updated state is published without advancing simulation time. `stop().await` stops the service. The client can be cloned and shared among multiple tasks.

`spawn_on_tokio` uses Tokio for coordination and an engine-created Rayon pool for computation. You can also supply an existing Rayon pool:

```rust
let pool = std::sync::Arc::new(rayon::ThreadPoolBuilder::new().num_threads(4).build()?);
let simulation = Simulation::builder().frequency(120).spawn_on_rayon(pool)?;
```

With this approach, the calling code does not need a Tokio runtime: the engine uses a dedicated thread for time management. Multiple simulations can share one pool, with each simulation respecting its own worker limit. Add `rayon` to the application's Cargo.toml dependencies.

## Time and Performance

`simulation.set_time_multiplier(x)` sets the rate of simulation time relative to real time: `1.0` is normal speed, `0.0` pauses the simulation, and values greater than one accelerate it. Negative and non-finite values are invalid. Actual speed depends on the workload.

Under heavy load, the engine reduces the rate at which simulation time advances based on the measured step duration, down to 30% of the requested speed. With fixed time stepping, `dt` remains unchanged. If steps still cannot keep up, lag accumulates. In adaptive mode, the selected policy determines `dt`; measurements and speed may fluctuate as the workload changes.

Advanced configuration (`Simulation::new_from_config`) is available when the builder's options are insufficient:

| `SimulationConfig` field | Default | Description |
| --- | --- | --- |
| `tick_rate` | `60` | Base frequency, Hz |
| `max_catch_up_steps` | `4` | Maximum number of consecutive steps per scheduler iteration |
| `time_step_policy` | `Fixed` | Fixed time step or `Adaptive(AdaptiveTimeStepConfig)` |
| `scheduler.min_batch_size` | `128` | Minimum computation batch size |
| `scheduler.max_batches_per_worker` | `4` | Maximum number of batches per worker |
| `workers.max_workers` | `None` | Pool limit; `None` uses available system parallelism |
| `workers.scale_up_at` | `0.70` | Fraction of the step budget at which to try increasing the worker count |
| `workers.scale_down_at` | `0.30` | Threshold for reducing the worker count |
| `workers.smoothing` | `0.20` | Weight of a new step-duration measurement in smoothing |
| `central_gravity` | `None` | Single central gravity source retained for compatibility with the previous API |

In `AdaptiveTimeStepConfig`, the defaults are `minimum_step = 10 µs`, `maximum_step = 500 ms`, and `relative_error_tolerance = 0.01`. These settings do not guarantee a particular error bound for every scene.

`simulation.metrics()` reports the most recent and average step durations, step budget, active workers, missed deadlines, accumulated lag, time rates, and `load_speed_factor`—the slowdown caused by load. Internal numeric runtime limits are defined in `crates/kapitsa/src/runtime/constants.rs`.

## Bodies and State

`RigidBody::new_static` creates a static body, `new_kinematic` creates a kinematic body, and `new_dynamic` creates a dynamic body with `MassProperties`. `Transform` specifies position and orientation, while `Vec3` represents linear quantities. Client-side changes use a `BodyId`, assigned by the engine when a body is added. IDs are specific to a simulation.

Example: creating a dynamic body with a spherical collider:

```rust
use kapitsa::{Collider, MassProperties, RigidBody, Shape, Transform};
use kapitsa::math::{Mat3, Vec3};
use kapitsa::shape::Sphere;

let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY)?;
let body = RigidBody::new_dynamic(Transform::identity(), mass)?;
let id = client.add_body(body).await?;
let collider = Collider::new(Shape::Sphere(Sphere::new(1.0)?), Transform::identity());
let collider_id = client.add_collider(id, collider).await?;
client.apply_force(id, Vec3::new(0.0, 1000.0, 0.0)).await?;
```

The mass and inertia tensor are illustrative; use values appropriate for the object. The client can change position, velocities, mass, and CCD mode; apply forces, torques, and impulses; and put a body to sleep or wake it up. For a persistent force, use `add_force_source` and `remove_force_source`: the application point and force are specified in the body's local coordinates.

`client.snapshot()` returns the most recently published snapshot without waiting. `snapshot.body(id)` reads a specific body's state, and `snapshot.bodies()` iterates over all bodies. `next_snapshot().await` waits for a new publication, but intermediate snapshots may be skipped. Each publication contains a step number and simulation time. While paused, state can change without changing the step number or simulation time.

## Colliders and Shapes

A `Collider` is attached to a body using `client.add_collider(body_id, collider)`. It contains a shape and a local `Transform`. `CollisionMode::Solid` generates physical contacts; `Sensor` reports overlaps without a physical response. `CollisionFilter` controls collision filtering, and materials define contact properties. Only convex shapes are allowed for dynamic bodies.

Available `Shape` variants:

| Type | Purpose |
| --- | --- |
| `Sphere`, `Cuboid` | Sphere and rectangular box |
| `Capsule`, `Cylinder` | Capsule and cylinder between two points |
| `TaperedCapsule`, `TaperedCylinder` | Similar shapes with different radii at their ends |
| `Convex` | Convex polyhedron |
| `Plane`, `Triangle` | Plane and triangle |
| `TriangleMesh` | Triangle mesh |
| `HeightField` | Surface defined by a height grid |
| `LevelSet` | Geometry defined by a distance field |
| `SkinnedTriangleMesh` | Deformable triangle mesh with bones |

Support for specific shape and operation combinations depends on the collision module's implementation; handle query errors and unsupported geometry. See [docs/collisions.md](docs/collisions.md) for more details.

## Joints and Composite Objects

A `Joint` connects two bodies. Shared settings are stored in `JointCommon`, and local frames in `JointFrame`. Available variants:

| Type | Behavior |
| --- | --- |
| `Fixed` | Maintains relative position and orientation |
| `Hinge` | Rotation around an axis |
| `Slider` | Translation along an axis |
| `Ball` | Joint allowing rotation in multiple directions |
| `Cylindrical` | Translation and rotation along the same axis |
| `Planar` | Movement in a plane and rotation around its normal |
| `Distance` | Constrains the distance between two points |
| `Spring` | Spring connection |
| `ConeTwist` | Joint with swing and twist limits |
| `D6` | Independent configuration of six degrees of freedom |

The client can add, update, disable, and remove joints. `JointLimit`, `JointDrive`, and `JointMotion` configure limits and drives where supported. Joint breakage is published as an event.

A cluster combines parts into a single rigid body. Create a dynamic body as usual, then enable clustering and add a part:

```rust
use kapitsa::cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength};
use kapitsa::joint::JointFrame;
use kapitsa::math::{Mat3, Vec3};
use kapitsa::{MassProperties, RigidBody, Transform};

let part_mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
let body_id = client.add_body(RigidBody::new_dynamic(Transform::identity(), part_mass)?).await?;
let root = client.enable_cluster(body_id).await?;
let weld = WeldDesc {
    frame_a: JointFrame::IDENTITY,
    frame_b: JointFrame::IDENTITY,
    strength: WeldStrength::Unbreakable,
    stiffness: WeldStiffness { linear: 1.0, angular: 1.0 },
};
let part = client.add_cluster_part(root, ClusterPartDesc {
    local_transform: Transform::identity(),
    mass: part_mass,
    colliders: Vec::new(),
    linear_velocity: Vec3::ZERO,
    angular_velocity: Vec3::ZERO,
}, weld).await?;

let cluster = client.cluster(body_id).await?.expect("cluster exists");
let weld_id = cluster.welds()[0];
let bodies = client.break_weld(weld_id).await?;
let detached_body = client.cluster_part(part).await?.expect("part exists").body();
```

In this example, the parts' initial velocities and local coordinate frames coincide. For other positions and velocities, provide consistent frames and a consistent velocity field. `break_weld` and `detach_part` return the body IDs of the resulting components. After separation, the snapshot already contains their states. `cluster`, `cluster_part`, and `weld` return data as of the time the request is processed; they can be used to obtain current `PartId`, `WeldId`, `BodyId`, and collider information.

`add_part_collider` and `remove_part_collider` manage a part's colliders. `destroy_cluster_part` removes a part when the topology allows it; the final part in a cluster cannot be removed through this method. `merge_clusters` attaches a second cluster to the first. Change a part's mass with `set_part_mass` using `MassChangeMode::KeepVelocityField`, or with `set_part_mass_with_transfer` using an explicit `MassTransfer`. Use `add_part_force_source` to apply a force in a part's local coordinates. To connect a body or part to another target, use `add_joint_targets` and `JointTarget`; update such a joint with `update_joint_targets`.

Strength limits and stiffness in `WeldDesc` are currently stored and validated, but automatic breaking of welded connections under load is not implemented. An explicit `break_weld` separates the parts regardless of these limits.

## Materials and Gravity

`PhysicsMaterial` describes dynamic and static friction, restitution, and density; a collider references a material through `MaterialId`. The example uses the `collider_id` of the collider created above:

```rust
use kapitsa::PhysicsMaterial;

let material = PhysicsMaterial::new(0.4, 0.5, 0.1, 1000.0)?;
let material_id = client.add_material(material).await?;
client.set_collider_material(collider_id, Some(material_id)).await?;
let current = client.material(material_id).await?;
client.update_material(material_id, PhysicsMaterial::new(0.2, 0.3, 0.0, 1000.0)?).await?;
client.set_collider_material(collider_id, None).await?;
client.remove_material(material_id).await?;
```

A material assigned to a collider cannot be removed: clear all assignments first. Updating a material invalidates cached contacts for colliders using it. `CollisionConfig` controls contact processing and CCD settings and can be updated through the client.

Gravity sources are added through the client and identified by `GravityId`. Multiple sources act simultaneously, and their accelerations are summed:

```rust
use kapitsa::{UniformGravity, math::Vec3};

let gravity = client.add_gravity(UniformGravity::new(Vec3::new(0.0, -9.81, 0.0))?).await?;
client.update_gravity(gravity, UniformGravity::new(Vec3::new(0.0, -3.71, 0.0))?).await?;
client.remove_gravity(gravity).await?;
```

`UniformGravity` provides constant acceleration throughout space. `CentralGravity` is a point source with a center and parameter `μ = GM`, acting at any distance. `SphericalGravity` uses the same point-source model within a specified radius; acceleration is zero outside it. At the center of a nonzero point source, acceleration is undefined and the step returns an error. This is not a model of distributed mass inside a planet. For backward compatibility, `SimulationConfig::central_gravity` also supports a single central source, whose contribution is added to the configured gravity fields.

## Queries, Events, and Batches

`client.raycast`, `sweep`, and `overlaps` perform spatial queries. `QueryFilter` restricts the candidates. State queries and spatial queries are processed at their position in the queue before integration of the corresponding step. Creating and updating materials and clusters also works while paused.

`subscribe_collision_events()` and `subscribe_joint_break_events()` create subscriptions. The streams are bounded: a slow consumer receives a `Lagged` error with the number of skipped batches.

`client.commands()` creates a command buffer; `try_submit` enqueues it without waiting and returns a batch ID. The queue is bounded, so when full, it returns the batch with an error, allowing it to be resubmitted. Results are available through `subscribe_batch_results`. The buffer supports the body, force, and joint operations listed in `PhysicsCommandBuffer`; newer material and cluster operations are executed as separate commands. Regular client methods wait for confirmation that changes have been applied and state has been published.

If a command has already changed the world but the subsequent physics step fails, the client receives `StepFailedAfterApply`: the command is not rolled back, and the snapshot remains at the last successful step. If an object was created in this situation, its new ID is not returned. Results of read-only queries performed before the failed step remain valid.

## Aerodynamics and Hydrodynamics

The `aerodynamics` and `hydrodynamics` modules currently provide extension points without an active physics model. For now, control spacecraft using explicit forces and torques.

## Verification

```text
cargo test --workspace
cargo clippy -p kapitsa --lib -- -D warnings
```
