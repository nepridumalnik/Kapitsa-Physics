# Aero shapes: shared prepared vessel geometry

Status: **complete** on `feat/aero_shapes`, created from the updated `master`
after the `feat/aero_rest` merge. The spec deltas and tasks record the agreed
acceptance criteria; the amendments below record where implementation details
differ from the initial plan.

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
  machinery; the registry owns cooked geometry directly instead of deepening
  the `BodyId`-keyed cache. Cache-key generalization is deferred to
  `aero-async`.
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
  body-local shape preparation or voxel-storage allocation. Temporary
  instance/patch buffers and the wing-visibility cache are unchanged from the
  body-local path.
- Regression tests cover sharing, invalidation, stale tickets, topology
  changes and rollback; docs and public API snapshots are updated.

## Design

Recorded from the integration audit at the branch start. The audit mapped
every entry point of `BodyId`/`PartId` into geometry preparation, caching,
owner attribution and load dispatch.

Audit facts that shape the design:

- Production cache keys are the constant `{body, geometry_version: 0,
  config_version: 1}`; invalidation happens by dropping the cache owner and
  by `Arc<()>` incarnation tickets, not by version bumps. The
  `request -> prepare -> publish` ticket path is already async-capable and is
  reused, not rebuilt.
- `GeometryOwner::Part(PartId)` is embedded in prepared occupancy, surface
  triangles, patches and `OwnerPressureWorkspace`. `PartHullWrench` /
  `PartWingWrench`, assembly wave ownership and wing visibility keys carry
  `PartId` directly from cluster parts and are independent of prepared-geometry
  owners; only mesh-pressure attribution needs a slot translation layer.
- Patch geometry is re-derived from instances at evaluation time; only
  occupancy, sections and volume are baked at preparation. A shared shape can
  therefore serve different bindings without re-voxelization.
- `GeometryOwner` is facade-public and `Ord`-sensitive: overlap ties resolve to
  the lowest owner and canonical patch sorting is owner-first, currently
  ordered by `PartId` `(world, index, generation)`.
- `MaterialStorage`/`MaterialId` (storage-scoped identity over `DensePool`
  with generation checks) is the closest existing registry analogue.
- Existing defect found: `destroy_cluster_part` orphans
  `part_aerodynamic_geometry` entries until `remove_body`. The shape lifecycle
  must not repeat this; the leak is fixed as a focused separate commit.

Decisions:

1. **Slot-based ownership, additive.** `GeometryOwner` gains
   `Slot(OwnerSlot)` with `OwnerSlot` a small `Copy` index newtype; the
   existing `Body` and `Part(PartId)` variants are preserved. Cooked shared
   shapes bake `Slot` owners so prepared geometry is vessel-independent;
   body-bound evaluation keeps `Part` owners after the binding resolves slots
   to parts, leaving `OwnerPressureWorkspace`, weld-reaction dispatch and
   public load accessors unchanged. This avoids a breaking public-API change
   and keeps published weld loads resolvable after assembly-cache drops.
   Overlap masks, sections and volume are owner-independent, and slot order
   assigned in sorted `PartId` order reproduces the current tie-break and
   canonical patch ordering bit-for-bit. `Part` and `Slot` owners are never
   mixed within one preparation; assembly pressure evaluation rejects
   unresolved slot owners explicitly.
2. **Registry.** `ShapeRegistry` mirrors `MaterialStorage`: storage-scoped
   `ShapeId { registry, DenseId }` over a `DensePool` of cooked shapes, with
   generation-guarded lookups and explicit removal. A `CookedShape` holds the
   immutable `PreparedGeometry`, its `GeometryConfig`, a revision, and the
   `Arc`-shared source instances (mesh, cook-frame transform, owner slot)
   needed to re-derive patches at evaluation.
3. **Cook frame is fixed.** Source transforms are baked into occupancy at cook
   time. A `ShapeBinding` maps `OwnerSlot -> PartId` and is validated against
   the bound parts' existence and cook-frame transforms; a binding whose
   transforms differ is rejected and requires a re-cook. No silent re-framing
   of baked occupancy.
4. **Registry-owned geometry, no cache-key generalization.** A `CookedShape`
   owns its `PreparedGeometry` directly, and bound evaluation feeds that
   geometry to pressure and wing shielding instead of calling
   `PreparedGeometryCache::prepare_instances`. The `PreparedGeometryCache`
   keeps its `BodyId` key for body-local and wave preparation. Generalizing
   the cache key is deferred to `aero-async`, where the worker boundary needs
   it; this change neither generalizes the key nor adds a parallel cache.
5. **Binding resolution at evaluation.** The binding resolves slots to
   `PartId` when constructing evaluation instances, so weld-reaction dispatch,
   `OwnerPressureWorkspace` and public load accessors keep operating on part
   owners unchanged. `PartHullWrench`/`PartWingWrench` channels are unchanged.
6. **Determinism.** Slot order is fixed by the cooked shape; bindings never
   alter cooked data. Ordinary and sharded paths must agree, and a geometry
   edit on one vessel must not invalidate another vessel's shared shape.
7. **Lazy binding invalidation.** A `ShapeBinding` stores `{shape, revision,
   slot-ordered parts}`. Every evaluation re-validates revision, preparation
   configuration, cavity policy, cluster membership, part meshes and part
   transforms without allocating; a mismatch makes the mutable paths drop the
   binding and fall back to body-local preparation, which computes the same
   physics without sharing. `replace_shape` advances the revision, so previous
   bindings fail verification. Pruning runs before the ordinary step loop; the
   sharded path validates without mutating world state.
8. **Bindings require a shared mesh allocation.** Mesh checks use
   `Arc::ptr_eq` plus the part transform, so the reused occupancy is
   unambiguous. `set_part_aerodynamic_mesh_shared` installs one
   `Arc<TriangleMesh>` on several parts; the owning
   `set_part_aerodynamic_mesh` still creates a private allocation per part, and
   structurally equal but distinct meshes are rejected with `InvalidBinding`.
   Collider-tessellated and wave sources are rejected with `UnsupportedSource`.
9. **Scope of sharing.** Sharing covers the pressure union and wing shielding,
   which consume one occupancy. The wave union stays body-local because it is
   expressed in the wave part's local frame with the model's own geometry
   configuration. Bound vessels therefore contain mesh parts only. Topology
   changes either keep a binding valid (a split leaving exactly the bound
   parts) or stale it (membership changed), in which case evaluation falls back
   until the body is re-bound.

## Non-goals

- asynchronous cooking, task queues or `PendingAero` policies (`aero-async`);
- runtime control-surface deflection (`aero-controls`);
- new physical models, pressure closures or atmosphere profiles;
- per-module chunk caching (deferred until profiling justifies it);
- claiming independent physical validation.
