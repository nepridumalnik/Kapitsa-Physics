# Rigid cluster contract

Status: design contract; the cluster API is not yet implemented.

## Ownership and identity

A connected, absolutely rigid assembly is represented by one physics body. Its
cluster metadata contains a root part and the IDs of its parts and welds; parts
have no independent dynamic state. Each part belongs to exactly one cluster and
owns zero or more colliders. Each collider on a clustered body belongs to one
part. A part's positive mass properties are independent of its collider count.
Welds join distinct parts, form a connected graph, and never create ordinary
joint solver rows. Parallel welds and cycles are permitted.

`BodyId`, `PartId`, `WeldId`, `ColliderId`, and `JointId` have independent
world-scoped generational identity. A cluster is identified by its body; there
is no separate cluster ID. Parts, surviving welds, colliders, joints, and force
sources retain their IDs across ownership changes.

## Structural lifecycle

Structural edits occur between completed physics steps. Preparation validates
IDs, geometry, graph connectivity, arithmetic, motion compatibility and storage
capacity, and computes all resulting state before mutation. An unsuccessful
edit must leave topology, ownership, physics state, caches and events unchanged.
Commit must not introduce a new fallible operation unless rollback is complete.
Events are published deterministically only after successful commit.

Enabling a cluster makes the existing body the owner of an identity-transform
root part and assigns all its colliders to that part. For a dynamic body the
root mass must match its current mass properties. Adding a part simultaneously
adds a weld to an existing part; an intermediate disconnected cluster is not
observable. Weld frames are part-local and must coincide in world space within a
documented tolerance; welding must not teleport geometry.

A merge retains the body of the first endpoint and retires the other body.
World-space part and collider poses remain unchanged. Dynamic merge conserves
linear and angular momentum and transfers accumulated force and torque about the
new center of mass. Static bodies may merge with static bodies. Kinematic bodies
may merge only when their rigid velocity fields agree. Dynamic/static and
dynamic/kinematic merges require an explicit motion transition first.

Breaking a weld removes only that edge. If the graph separates, the component
containing the root retains the original body; other components receive bodies
in deterministic stable-ID order. Each fragment retains the parent rigid velocity
field: its center velocity is `V + omega × (C_fragment - C_parent)`, and its
angular velocity is `omega`. Detaching a part removes its incident welds but
preserves the part and its colliders. Destroying a part removes its mass and
colliders and is not an impulse-conserving separation. If the root is destroyed,
the remaining root is chosen deterministically by minimum stable part ID.

Mass properties are aggregated from transformed part mass properties, not
collider geometry, only when the composition or part masses change. Static and
kinematic clusters retain these properties for later transition to dynamic.

`KeepVelocityField` preserves world-space rigid velocity and geometry when the
COM changes: `V_new = V_old + omega × (C_new - C_old)` and `omega_new = omega_old`.
`ConserveRemainingMomentum` requires an explicit matter-transfer model with
signed momentum changes about a fixed world point `O`; it is rejected when these
inputs are absent. Given old momentum `P_old` and angular momentum `H_old` about
`O`, use `P_new = P_old + ΔP` and `H_new = H_old + ΔH`, including any external
impulses belonging to the same operation. Then `V_new = P_new / M_new` and
`omega_new = I_new_world⁻¹ (H_new - (C_new - O) × P_new)`. Positive transfer
enters the body and negative transfer leaves it. Preserving the old total body
momentum is not a substitute for this model.

Fuel mass editing and nozzle thrust are separate operations: calculate thrust
from mass flow and effective exhaust velocity and apply it at the nozzle, without
double-counting the reactive impulse. Stage separation uses Split, followed by
optional equal and opposite separation impulses.

## External objects and collision lifecycle

Joint endpoints and force sources may target a body or a part. Legacy body
bindings are preserved, and enabling a cluster anchors existing body bindings
to its root. Part-local targets follow their part through topology changes.
A joint whose endpoints resolve to the same body is suspended without changing
its user-enabled flag or ID; it becomes active again if its endpoints separate.
Force sources retain their identity, enabled flag, world application point and
world force when their owner changes.

Colliders are rebound without pool removal or ID replacement. Structural edits
invalidate the affected contact and joint warm starts, prepared solver data,
spatial proxies, CCD trajectories and island mappings. Internal collider pairs
are not contacts; separated fragments immediately use ordinary collision
filtering. Contact event semantics remain separate from topology events, and
invalidated active pairs produce their normal end lifecycle on the next
successful update.

Direct mutable access, collider operations and body removal must not bypass
cluster ownership or managed mass invariants. World-level validated operations
are required for motion and force changes on clustered bodies.

## Fracture and runtime

Manual weld removal is independent of automatic fracture. Automatic fracture is
disabled by default and, when explicitly enabled, uses the configured elastic
graph model. Cyclic load sharing depends on weld stiffness and is an estimate,
not a FEM deformation model. A failed or non-convergent solve cannot cause
fracture. Selected welds are removed together at a permitted step boundary.
Contact impulses must include their actual vector components and application
points; XPBD position multipliers are not automatically physical impulses.

Runtime topology commands execute in existing FIFO order at step boundaries.
Each structural command is atomic; no stronger batch atomicity is implied.
Explicit topology queries return owned snapshots with stable IDs, without
copying every cluster into every ordinary physics snapshot.

## Existing API dependency inventory

This inventory describes the baseline, not implemented cluster behavior.

| API | Current behavior and migration concern |
| --- | --- |
| `PhysicsWorld::body_mut` (`crates/kapitsa-core/src/world.rs`) | Returns `&mut RigidBody`, allowing replacement as well as mutation. Marks query and collision spatial data dirty and invalidates attached joint warm starts when that experimental cache is enabled. Cluster bodies cannot safely expose this reference. |
| `PhysicsWorld::bodies_mut` (`crates/kapitsa-core/src/world.rs`) | Returns mutable bodies in unstable dense order and marks spatial data dirty; unlike `body_mut`, it does not invalidate joint warm starts. Its behavior must explicitly change for managed bodies. |
| `add_collider` / `remove_collider` (`crates/kapitsa-core/src/world.rs`) | Colliders belong to bodies, not parts. Addition validates material and dynamic geometry; removal unlinks and invalidates contacts. Neither establishes part ownership, so cluster-aware operations are required. `collider_mut` also exposes arbitrary collider mutation. |
| `remove_body` (`crates/kapitsa-core/src/world.rs`) | Removes attached colliders, joints and force sources. Cluster cleanup must preserve this documented removal intent without leaving part or weld records. |
| `add_joint` (`crates/kapitsa-core/src/world.rs`) | Requires distinct live bodies and stores body endpoints. Part targets and suspension require changes to preparation and lifecycle while preserving the legacy entry point. |
| Async commands and batch (`crates/kapitsa/src/runtime/asynchronous/marshalling/command.rs`, `client.rs`) | Commands delegate to core; body mutations and batch copyback use mutable body access. Both paths need migration. Existing `StepFailedAfterApply` means an applied change can outlive a failed step without a published snapshot; it is not a rollback guarantee. |
| Rocket fixtures and consumers (`crates/kapitsa/src/runtime/modular_rocket_fixture.rs`, `examples/rocket-flight-viewer/src/flight.rs`) | Existing assemblies use separate bodies and joints; their fixed-joint baseline and existing behaviors must remain available. |
