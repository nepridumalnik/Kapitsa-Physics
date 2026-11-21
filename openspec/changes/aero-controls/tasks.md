# Tasks

- [ ] Make `WingControl` deflection mutable at runtime without recreating
  `WingSurface`.
- [ ] Add `PhysicsClient` commands for individual rudders and flaps.
- [ ] Enforce deflection-angle limits and validate parameters; reject invalid,
  non-finite, out-of-range and stale/foreign identities without mutation.
- [ ] Apply batched commands atomically.
- [ ] Compute correct force/moment from deflected surfaces about the body COM
  using existing full-angle polars, including reverse flow.
- [ ] Expose per-wing load diagnostics consistent with applied loads.
- [ ] Add tests: Mach 0.5/2/8 signs, neutral restore, reverse flow, no
  per-step allocation, no visibility invalidation, ordinary/sharded parity.
- [ ] Update `docs/aerodynamics.md`, the acceptance review and public API
  snapshots.
