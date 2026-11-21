# Tasks

- [ ] Introduce `CookedShape` as an immutable prepared vessel shape with owner
  slots, independent of `PartId`/`BodyId`.
- [ ] Add `ShapeId` and a shape registry supporting shared use by several
  bodies.
- [ ] Add `ShapeBinding` mapping `OwnerSlot -> PartId` with per-part transforms.
- [ ] Generalize the existing cache key from `BodyId` to an opaque shape
  identity, reusing `PreparedGeometry`/`GeometryPreparation`/publication.
- [ ] Implement atomic install/replace with identity and revision checks and
  stale/foreign rejection.
- [ ] Invalidate correctly on part-geometry and assembly-topology changes
  without invalidating unrelated vessels sharing the registry.
- [ ] Preserve per-part load attribution through merge/split and root transfer.
- [ ] Fix the audited leak: `destroy_cluster_part` orphans
  `part_aerodynamic_geometry` entries until `remove_body`; clean them up on
  part destruction with generation-guarded lookup.
- [ ] Add regression tests: sharing, invalidation, stale tickets, topology
  changes, rollback, ordinary/sharded parity, no per-step allocation.
- [ ] Update `docs/aerodynamics.md`, the acceptance review and public API
  snapshots; record benchmark evidence for the shared-shape path.
