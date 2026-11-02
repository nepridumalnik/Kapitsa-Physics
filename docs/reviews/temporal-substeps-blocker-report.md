# Stage 10: temporal substeps blocker report

- Date: 2026-11-02
- Branch: `feat/collisions`
- Inspected baseline: `c25d127`

## Decision

Temporal substeps remain **bench-only research**; production integration is
blocked by the current world-step transaction. No `JointTimeIntegration` option
or partial collision-loop implementation was added. No TGS performance result
is claimed because there is no TGS implementation to measure.

## Current step contract

`PhysicsWorld::step` prepares force-source and gravity updates once, then
`step_prepared` opens one `CollisionSystem` transaction, predicts every body for
the full `dt`, builds swept collision candidates, processes CCD, solves discrete
contacts and joints, and commits predicted body states, joint breaks, caches,
and lifecycle events. Any error before the commit aborts staged collision data
and rolls prepared gravity and force-source updates back
(`world.rs:975-1139`).

The joint solver receives this full-step `dt` from `solve_contacts`
(`collision/system.rs:945-953,1113`). Collision manifolds are generated once
from the post-CCD predictions and passed into that solve
(`world.rs:4245-4539`). Joint anchors can be refreshed while solver rows are
prepared, but this does not move bodies forward in time or regenerate contact
manifolds between temporal samples.

## Blocking invariants

### Forces and gravity

`predict_dynamic` applies the stored force and torque using the supplied `dt`
(`rigid_body/integrator.rs:160`). For a constant force, N integrations of
`h = dt / N` produce the expected total velocity change because the increments
sum to `F * dt / m`; applying the full `dt` on every microstep would multiply
that change by N.

That restricted result does not prove the general force contract. Central
gravity is computed from each body's current center and prepared once before
the world step (`world.rs:1017,1297-1381`). Persistent force-source local force
and point are transformed using the body's current pose once before prediction
(`world.rs:1166-1278`). An internal temporal loop would need to recompute these
pose-dependent wrenches from each staged microstep state while still applying
each force for exactly `h`. The current transaction has no such per-microstep
force preparation path.

### CCD time and impact budget

`StepContext::clear` clears CCD states, elapsed times, per-body impact counts,
and processed time (`collision/step.rs:180-216`). CCD initializes those values
for one `dt` and tracks TOI replay and the configured `max_ccd_impacts` within
that interval (`world.rs:1711-1803,2175-2351`). Running N ordinary step
transactions of length `h` would reset elapsed time and the impact budget N
times. Keeping one transaction would instead require the CCD candidate refresh,
replay, clipping, and TOI ordering to share a global elapsed-time origin across
all microsteps. The existing full-step replay pipeline does not expose that
contract.

### Contact and joint caches, events, and rollback

Contact warm starting reads the committed `current` cache. `ContactCache::stage`
clears and rebuilds `next`, and `commit` swaps the two maps
(`collision/cache.rs:44-45,132-207`). Joint warm starting follows the same
current/next transaction pattern (`joint/warm_start.rs:111-199`). Therefore,
committing each microstep makes intermediate cache state visible and prevents a
later failure from rolling the full outer step back; delaying commit means the
next microstep cannot warm-start from the preceding microstep's staged result
with the current API.

Collision lifecycle staging also compares against the previously committed
active pairs and replaces the staged event batch (`collision/event.rs:62-90,
222-229`). Per-microstep staging without a separate event accumulator can lose
transient Begin/End transitions; per-microstep commits expose those transitions
before the outer step succeeds. Joint breaks and predicted body states are
committed at the end of `step_prepared` (`world.rs:1137-1139`), so repeating
that routine cannot preserve the existing all-or-nothing step behavior if a
later microstep fails.

## Work required before production integration

The world needs an outer transaction that owns the entire requested `dt` while
microsteps operate on staged body states. That transaction must:

1. Recompute position-dependent gravity and persistent force-source wrenches
   from each staged pose, applying each over exactly `h`.
2. Rebuild broadphase/narrowphase data and run CCD for each interval while
   carrying global elapsed time and the full-step impact budget through replay.
3. Let the next microstep consume staged contact and joint impulses without
   publishing those caches before the outer commit.
4. Accumulate collision lifecycle transitions across microsteps and defer body,
   joint-break, cache, and event commits until every microstep succeeds.
5. Preserve the same behavior for the executor and shard-based step entry
   points, or define and test an explicit supported boundary for them.

The required force-once and angular-connector continuity tests should be added
with that staged path, followed by contact-cache, CCD-budget, and late-failure
rollback tests. Until those contracts exist, enabling `TemporalSubsteps` in the
production collision loop would be a partial feature with different cache,
event, CCD-budget, and failure semantics.

## Stage result

Production gate: **not passed**. The phase stays **bench-only**; no benchmark
comparison for temporal substeps was run. Do not proceed to Stage 11 until the
transaction and CCD contracts above are resolved and the Stage 10 gate is
revisited.
