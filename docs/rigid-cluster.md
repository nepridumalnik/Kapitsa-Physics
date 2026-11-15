# Rigid cluster contract

Status: design contract; core lifecycle, targets, merge, and weld splitting are implemented.

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

Manual weld removal is implemented; automatic weld fracture and an elastic graph
load solver are **not implemented**. `WeldStrength::Breakable` stores validated
thresholds but does not currently trigger fracture. Diagnostic snapshots retain
some part-owned aerodynamic wrenches, but they are not a complete load ledger
and are not used for weld evaluation. Do not interpret the net cluster wrench
as the force on any weld.

The simple `AerodynamicSurface` path applies a body-level timestep-dependent
drag limit and a coupled lift correction after summing surfaces. The raw force
from a particular surface is therefore not necessarily its applied force.
`PhysicsWorld::simple_surface_corrections` publishes the raw and applied
body-level drag and lift wrenches after a successful full step. Their difference
is not attributed to any part. `raw_simple_surface_wrenches` records the raw
drag and lift of each body- or part-targeted surface before that correction;
they are not generally the applied loads. `check_part_attribution` accepts a
complete part-only snapshot only when raw and applied aggregate wrenches match
exactly; it rejects an unallocated correction rather than ignoring it. A
missing aggregate, including cancelling loads, must likewise not be assumed
attributable. A future weld ledger must adopt and document a
physically justified allocation policy before using these corrected loads; it
must not silently use uncorrected per-surface values as weld loads.

For separate rigid bodies joined by a breakable `Joint`, the existing velocity
constraint solver evaluates its reaction from accumulated impulses, including
the effects of applied aerodynamic forces, and compares force and torque at the
joint anchor against their thresholds. This is a discrete solver reaction, not
a stress or material failure model. A differential-drag regression covers the
equal-mass fixed-joint case.

A future weld-fracture implementation must account for external loads per part
and the inertial wrench of each component, including moments about a common
point. A tree edge can be assessed from the balance of either component cut by
that edge; for cycles, load sharing is statically indeterminate without an
explicit mechanical model. Contact impulses must include their actual vector
components and application points; XPBD position multipliers are not
automatically physical impulses. Any eventual fracture policy must specify
solver convergence and failure handling before removing welds.

The velocity contact solver applies the normal and both tangent components at
separate body-side contact positions: its pair impulse acts negatively on A
and positively on B. Warm-start impulses are applied before iterative updates;
the final accumulated contact components include that warm start. The positive
normal-update sum used for impact detection is not a net impulse. Contact
events instead expose a representative point and normal-impulse summary, not a
complete vector impulse ledger. CCD has separate contact solves, potentially
more than one per step, while the position solver applies separate corrections.
Consequently a future transactional ledger must cover every applicable solve
and preserve the relevant application positions; neither events nor the last
manifold alone are sufficient to claim a complete weld balance.
`PhysicsWorld::contact_impulses_ccd_clipped()` reports whether the last
successful full step clipped a dynamic body. CCD clipping sets velocities to
zero without recording a physical impulse, so such a snapshot cannot support a
complete load balance even if `contact_impulses_omitted_points()` is zero. A
false clipping flag does not establish completeness of other solver paths.

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
# Automatic weld failure (restricted)

`PhysicsWorld::set_automatic_weld_failure` opts full steps into instantaneous
tree-weld strength checks; it is disabled by default. Specify absolute force
and moment equilibrium tolerances in N and N·m. The force and torque limits
compare magnitudes at the child-side weld anchor; equality does not break a
weld. A step with a breakable cluster fails before commit when its loads cannot
be certified. Currently certification requires a dynamic cluster with no
accumulated body forces, no aerodynamic loads, and only part-targeted persistent
force sources, part-owned `HullSection` and `WingSurface` loads, and gravity sampled uniformly at the cluster center of mass, with
no colliders on the cluster and no joints or chunks in the world. Gravity
gradients across the cluster are not modeled. Colliders on other bodies are permitted. Thus
contact and CCD forces are not supported, not assumed absent from a diagnostic
log. Low-level shard commits do not perform automatic weld failure.

The separate diagnostic `part_hull_weld_reactions` also accepts part-owned
`HullSection` loads from the same successful step. It rejects wings, simple
surfaces, primary and geometry-based aerodynamics, and the other unsupported
paths above. `part_aerodynamic_weld_reactions` also accepts part-owned wings;
automatic failure uses the corresponding pending hull and wing snapshots.

Part-targeted `AerodynamicSurface` loads are not yet certified for weld
failure. A safe extension must use the same step's raw per-surface records and
verify that the applied body-level drag and lift exactly equal their raw sums;
it must reject body-targeted surfaces and any unattributed correction. An
absent `SimpleSurfaceCorrection` is not a certificate of zero individual loads:
the aerodynamic evaluator returns before recording a correction when the
aggregate force and moment vanish, including when individual loads cancel.
Missing records and the zero-density path therefore require explicit handling,
not an assumption that every part's load is zero.

The reaction is evaluated at the start of the timestep, while the split uses
the predicted end-of-step rigid velocity field. This is a discrete failure
policy, not an impact, fracture propagation, or within-step failure-time model.

The contact snapshot records velocity-solver impulses, including warm-start and
friction, but a discrete impulse is not an instantaneous force at the start of
the step. CCD impacts can occur at different times and `ClipIsland` may alter
velocity without a physical impulse. Contact-enabled weld failure therefore
also needs a specified impulse-to-strength policy and a complete account of all
accepted solver and CCD paths; zero omitted points alone is not sufficient.
