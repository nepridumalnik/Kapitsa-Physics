# Tasks

- [x] Make `WingControl` deflection mutable at runtime without recreating
  `WingSurface`.
- [x] Add `PhysicsClient` commands for individual rudders and flaps.
- [x] Enforce deflection-angle limits and validate parameters; reject invalid,
  non-finite, out-of-range and stale/foreign identities without mutation.
- [x] Apply batched commands atomically.
- [x] Compute correct force/moment from deflected surfaces about the body COM
  using existing full-angle polars, including reverse flow.
- [x] Expose per-wing load diagnostics consistent with applied loads.
- [x] Add tests: Mach 0.5/2/8 signs, neutral restore, reverse flow, allocation-free
  control batches, no visibility invalidation, ordinary/sharded parity.
- [x] Update `docs/aerodynamics.md`, the acceptance review and public API
  snapshots.
