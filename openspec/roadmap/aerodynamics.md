# Aerodynamics roadmap

Planning record agreed after the `feat/aero_rest` stabilization. It sequences
the post-merge aerodynamics work into separate, logically complete changes.
Each planned change carries a proposal, requirement-level spec deltas and an
unchecked task list; the deltas and tasks are refined when the change starts.
Unchecked tasks and `ADDED` deltas denote planned work, not implemented or
validated behavior.

## Order and dependencies

```text
feat/aero_rest (stabilization, merge first)
        |
        +--> feat/aero_shapes  (aero-shapes)
        |           |
        |           +--> feat/aero_async  (aero-async)
        |
        +--> feat/aero_controls  (aero-controls, independent of shapes/async)
```

- `aero-shapes` depends on the merged `feat/aero_rest`.
- `aero-async` depends on `aero-shapes`.
- `aero-controls` depends only on the updated `master`; it may be developed in
  parallel with `aero-shapes` and `aero-async`.
- Every branch is created from the updated `master` and merged as one
  logically complete PR. No mixing of the geometry-lifecycle refactor with new
  physics models.

## Planned changes

| Change | Branch | Result |
| --- | --- | --- |
| [`aero-shapes`](../changes/aero-shapes/proposal.md) | `feat/aero_shapes` | Several bodies share one prepared vessel shape without repeated voxelization. |
| [`aero-async`](../changes/aero-async/proposal.md) | `feat/aero_async` | Vessel construction changes never voxelize synchronously inside the physics step. |
| [`aero-controls`](../changes/aero-controls/proposal.md) | `feat/aero_controls` | Runtime-controllable aerodynamic surfaces for stabilization and attitude control. |

All three develop infrastructure and control. **None adds a new physical
model.**

## Shared constraints

- Realism target is plausible dynamics, not experimental validation; no
  heating or ablation. No network replication is planned.
- Minimize heap-resident objects and allocations: no per-body/per-part shape
  duplicates, no steady-state per-step allocation on the aero hot path.
- Reuse existing infrastructure (`PreparedGeometry`, `GeometryPreparation`,
  publication/stale-ticket protection, part wrench snapshots, full-angle
  polars) instead of reimplementing it.
- Prepared shapes are immutable shared resources; bindings are vessel-specific.
  Owner slots, not `PartId`/`BodyId`, identify shape owners.
- Out-of-domain input returns a typed error with its cause preserved and an
  atomic step rollback; no silent clamping, blending or model fallback.
- Ordinary and sharded step paths must agree; determinism is a property of the
  committed state, not of the executor alone.
- Each change updates `docs/aerodynamics.md`, the acceptance review and the
  public API snapshots, and records focused regression tests plus benchmark
  evidence where performance is claimed.

## Beyond these changes

Future physical-model development (automatic pressure-model selection, layered
atmosphere, rarefaction extensions) is separate work. An `Auto` pressure model
requires a documented applicability table, trajectory sensitivity tests at the
model seams and independent data before it is considered; smooth blending alone
is not physical correctness. Per-module chunk caching stays deferred until
profiling shows it is warranted: a single part change can alter whole-vessel
exposure and sections.
