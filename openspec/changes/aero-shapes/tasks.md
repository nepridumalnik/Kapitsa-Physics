# Tasks

- [x] Introduce `CookedShape` as an immutable prepared vessel shape with owner
  slots, independent of `PartId`/`BodyId`.
- [x] Add `ShapeId` and a shape registry supporting shared use by several
  bodies.
- [x] Add `ShapeBinding` mapping `OwnerSlot -> PartId` with per-part transforms.
- [x] Reuse `PreparedGeometry`/`GeometryPreparation`: the registry owns cooked
  geometry directly and bound evaluation consumes it without a per-body union.
  Generalizing the `BodyId` cache key is deferred to `aero-async`.
- [x] Implement atomic install/replace with identity and revision checks and
  stale/foreign rejection (`replace_shape`, `ShapeRegistry::replace`).
- [x] Invalidate correctly on part-geometry and assembly-topology changes
  without invalidating unrelated vessels sharing the registry (lazy
  allocation-free binding validation with body-local fallback).
- [x] Preserve per-part load attribution through merge/split and root transfer.
- [x] Fix the audited leak: `destroy_cluster_part` orphans
  `part_aerodynamic_geometry` entries until `remove_body`; clean them up on
  part destruction with generation-guarded lookup.
- [x] Add regression tests: sharing, invalidation, stale revisions, topology
  changes, rollback, ordinary/sharded parity, allocation comparison.
- [x] Update `docs/aerodynamics.md`, the acceptance review and public API
  snapshots; record benchmark evidence for the shared-shape path.