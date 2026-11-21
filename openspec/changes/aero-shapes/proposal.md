# Aero shapes: shared prepared vessel geometry

Status: **planned, not started**. Branch: `feat/aero_shapes`, created from the
updated `master` after `feat/aero_rest` merges. The spec deltas and unchecked
tasks record the agreed acceptance criteria; they are refined when the change
starts.

## Summary

Separate the immutable prepared aerodynamic shape of a vessel from the physics
bodies that use it. Several bodies can share one cooked shape without repeated
voxelization, while per-part load attribution and assembly invalidation keep
their current guarantees.

## Scope

- `CookedShape`: an immutable prepared vessel shape, independent of physics
  bodies. Owners are identified by owner slots, not `PartId`/`BodyId`.
- `ShapeId` and a shape registry supporting shared use of one prepared shape
  by several bodies.
- `ShapeBinding`: binds a shape to a body and maps `OwnerSlot -> PartId` with
  the per-part transforms, so the same shape serves different assemblies.
- Reuse the existing `PreparedGeometry`, `GeometryPreparation` and caching
  machinery; generalize the cache key from `BodyId` to an opaque shape identity
  instead of building a parallel cache.
- Atomic shape installation and replacement with identity and revision checks;
  stale or foreign results are rejected without partial application.
- Correct invalidation when part geometry or assembly topology changes; a
  shape edit on one vessel must not invalidate an unrelated vessel sharing the
  registry.
- Per-part load attribution (pressure, wave, wing shielding) preserved through
  merge/split and root transfer, without double counting.

## Dependencies

- Merged `feat/aero_rest` (stable geometry, visibility, pressure and assembly
  integration).
- No dependency on `aero-async` or `aero-controls`; synchronous preparation
  remains available. Async cooking is the next change, not this one.

## Acceptance criteria

- Two bodies with different `OwnerSlot -> PartId` mappings use one cooked
  shape; voxelization runs once, and both report correct per-part wrenches.
- Editing one body's geometry does not invalidate another body's shared shape.
- Stale revisions and foreign shape identities are rejected atomically; a
  failed install preserves the previous shape and motion state.
- Installed shapes are never re-voxelized inside the physics step; preparation
  happens at explicit cook/install boundaries.
- Merge/split and root transfer preserve load ownership; ordinary and sharded
  step paths agree.
- No per-body heap-resident shape duplicate; steady-state steps perform no
  shape-related allocation.
- Regression tests cover sharing, invalidation, stale tickets, topology
  changes and rollback; docs and public API snapshots are updated.

## Non-goals

- asynchronous cooking, task queues or `PendingAero` policies (`aero-async`);
- runtime control-surface deflection (`aero-controls`);
- new physical models, pressure closures or atmosphere profiles;
- per-module chunk caching (deferred until profiling justifies it);
- claiming independent physical validation.
