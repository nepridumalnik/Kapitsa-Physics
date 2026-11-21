# Aero async: off-step geometry preparation

Status: **planned, not started**. Branch: `feat/aero_async`, created from the
updated `master` after `aero-shapes` merges. The spec deltas and unchecked
tasks record the agreed acceptance criteria; they are refined when the change
starts.

## Summary

Move aerodynamic shape cooking off the physics step onto worker threads. A
vessel construction change schedules preparation instead of voxelizing
synchronously inside the step, with explicit, tested behavior while a shape is
pending.

## Scope

- Cook `CookedShape` on worker threads, outside the physics step.
- A preparation task queue with cancellation or rejection of stale results
  when geometry or topology changes again before a cook finishes.
- Explicit shape states `Ready`, `Cooking`, `Failed`, with completion
  notification.
- `PendingAero` policies selecting step behavior while a shape is not ready:
  - `KeepPrevious`: keep the last good shape for edits that do not change
    topology;
  - `ZeroForce`: contribute no aerodynamic load after a topology change until
    the new shape installs;
  - `BlockStep`: block the step until ready, for offline/non-realtime use.
- `PhysicsClient` integration: submit geometry/topology changes, observe
  preparation state and receive completion or failure.
- Atomic publication of a cooked shape; a failed cook preserves the last
  successful shape and reports the cause.
- Tests for concurrent edits, atomic publication, cancellation and error
  handling.

## Dependencies

- `aero-shapes` (shared immutable `CookedShape`, registry, bindings and
  revision-checked install).
- The existing runtime command queue and executor infrastructure; reuse it
  rather than introducing a separate scheduling path.

## Acceptance criteria

- A vessel construction change never triggers synchronous voxelization inside
  the physics step.
- While `Cooking`, step behavior matches the selected `PendingAero` policy and
  is tested in ordinary and sharded paths.
- After a topology change, a stale shape is never used; `KeepPrevious` applies
  only to topology-preserving edits.
- Install and abort are atomic; a failed cook keeps the last successful shape
  and preserves the error cause; motion state is unchanged on rejection.
- Concurrent geometry edits and cancellation are tested; stale results are
  rejected by identity/revision, not applied.
- Determinism of the committed state is preserved and tested; snapshot
  interpolation remains a render concern, not an aerodynamics behavior.
- No steady-state per-step allocation for state checks; docs and public API
  snapshots are updated.

## Non-goals

- new physical models, pressure closures or atmosphere profiles;
- runtime control-surface deflection (`aero-controls`);
- network replication or multi-client synchronization;
- claiming independent physical validation.
