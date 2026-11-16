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

Manual weld removal and restricted automatic tree-weld fracture are implemented;
see the policy below. An elastic graph load solver is not implemented.
Diagnostic snapshots retain some part-owned aerodynamic wrenches, but they are
not a complete load ledger for arbitrary configurations. Do not interpret the
net cluster wrench as the force on any weld.

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
weld. Under `Strict`, a step with a breakable cluster fails before commit when
its loads cannot be certified. Under `Realtime`, an incomplete load ledger skips
automatic failure for that cluster and records it in `StepDiagnostics` after
commit; `weld_collision_load_clusters` identifies the subset affected by contact,
CCD clipping or unresolved collision ownership. No reaction is inferred.
Other balance, topology, numerical and storage
errors still fail the step. Currently certification requires a dynamic cluster with no
accumulated body forces, only part-targeted persistent force sources,
certified part-owned aerodynamic loads, and gravity sampled uniformly at the cluster center of mass, with
no joints or chunks in the world. Gravity gradients across the cluster are not
modeled. A cluster may have colliders; contact-free clusters can be checked,
and one narrowly supported discrete contact can be checked using the separate
impulse criterion described below. Contacts on unrelated bodies do not
invalidate a contact-free cluster. If manifold ownership cannot be resolved,
all cluster certificates are rejected. Absence of contact is not inferred from
an empty impulse log. CCD forces remain unsupported.
Low-level shard commits do not perform automatic weld failure.

The separate diagnostic `part_hull_weld_reactions` also accepts part-owned
`HullSection` loads from the same successful step. It rejects wings, simple
surfaces, primary and geometry-based aerodynamics, and the other unsupported
paths above. `part_aerodynamic_weld_reactions` also accepts part-owned wings;
automatic failure uses the corresponding pending hull and wing snapshots.

Part-targeted `AerodynamicSurface` loads are certified only when all configured
surfaces have raw records from the same step, their sums match the recorded
aggregate, and applied drag and lift exactly equal their raw values. Body-targeted
surfaces and unattributed corrections are rejected. The
evaluator now records an explicit zero correction when the aggregate force and
moment vanish, including when individual loads cancel. An absent correction is
still not a certificate of zero individual loads: missing surface records in
an atmosphere cause an explicit incomplete-load error. Surfaces sampled at
zero local density explicitly record zero force and moment after their target
and position are validated. The explicit global
`Atmosphere::Vacuum` configuration certifies zero aerodynamic loads without
surface records, including part-owned aerodynamic geometry; this does not apply
to locally zero-density samples. Collider and other load-path restrictions
remain in force even in vacuum.

For part-owned Mesh geometry in air, the full-step pressure workspace must be
staged from the same patches and flow as the applied impact-pressure wrench;
its patch-order total must match that applied wrench exactly. Every pressure
owner must be a part in the certified tree. The owner wrenches are evaluated
about the assembly COM and incorporated into the pending weld balance. Missing
staging is rejected, including when COM density is zero. Part-owned voxel wave
loads use the existing part hull wrench snapshot; mesh wave-source geometry is
geometry-only. Joints and unsupported contact paths remain excluded as
described above.
This certifies attribution of the implemented Mach >= 5 impact approximation,
not experimental accuracy of the pressure model.

The reaction is evaluated at the start of the timestep, while the split uses
the predicted end-of-step rigid velocity field. This is a discrete failure
policy, not an impact, fracture propagation, or within-step failure-time model.

The contact snapshot records velocity-solver impulses, including warm-start and
friction, but a discrete impulse is not an instantaneous force at the start of
the step. CCD impacts can occur at different times and `ClipIsland` may alter
velocity without a physical impulse. Zero omitted points alone is not sufficient
to certify the full impulse ledger.
Each `ContactImpulse` additionally carries optional body transforms from its
velocity solve, before positional correction. These are diagnostic witnesses,
not a certificate of simultaneous impacts or complete velocity changes.
For a two-body discrete solver island, possibly with multiple manifolds, without joints, CCD solves or clipping,
`isolated_velocity_states` also records before/after states when position
projection did not change either pose. The record is committed with the contact
snapshot and retained on failure. It does not by itself certify all physical
load paths; CCD events are deliberately excluded.

`cluster::impulse::tree_impulse_reactions` provides a separate fixed-pose,
simultaneous impulse-balance primitive. The caller supplies a complete per-part
external ledger and the corresponding changes of assembly COM and angular
velocities; it checks total momentum balance and returns parent-on-child impulse
reactions. Its tolerances are in N·s and N·m·s, not N or N·m. It does not
certify contact snapshots or process sequential CCD impacts. `ImpulseStrength` supplies separate
linear and angular impulse limits; angular reactions must first be shifted to
the weld anchor with `ImpulseReaction::at_anchor`. It does not derive impulse
limits from force limits.
`PhysicsWorld::set_weld_impulse_strength` stores optional limits on breakable
welds, preserving them through cluster topology changes.
`set_automatic_weld_impulse_tolerances` separately stores absolute momentum
balance tolerances in N·s and N·m·s; these are not force-balance tolerances.
With automatic weld failure enabled, a configured tolerance permits checks for
one discrete isolated two-body island with recorded owners for all manifold points and solver
states, no omitted impulses, CCD solves or clipping, and no pose change during
position projection. The force ledger must also be certified. The solver's
momentum change is checked against the sum of attributed impulses and moments at the contact pose;
force and impulse candidates are committed together. An unset per-weld impulse
limit disables that weld's impulse criterion. Unsupported cases retain strict
refusal or realtime diagnostic skipping; sequential CCD fracture is unsupported.

### CCD fracture certification gap

A possible first supported CCD case is one complete, isolated two-body impact
with no joints, clipping, or other impulses on the assembly during the step.
This is not implemented. CCD contact impulses carry a diagnostic witness with
the event time and the solver's before/after body states. The sequential path can
solve and resweep further impacts; the parallel path commits its own island
results. A replay can discard earlier CCD solves. Consequently, a single entry
in the final journal alone does not certify that it is the only relevant event.

Before enabling fracture for that case, the implementation must establish
complete ownership and absence of other
velocity changes, and verify the linear and angular momentum balance at the
impact pose. The implementation must also define and test how a split at impact
affects the remainder of the timestep; applying a split only at the end is not
equivalent to resolving the post-impact motion of separate parts. Until then,
CCD remains a strict refusal or a diagnosed realtime skip, not an inferred
discrete event.
