# Aero controls: runtime-controllable aerodynamic surfaces

Status: **implemented on `feat/aero_controls`**. This change is independent of
`aero-shapes` and `aero-async`.

## Summary

Let a vessel be controlled in the atmosphere by deflecting aerodynamic surfaces
at runtime, without recreating wing surfaces. This enables aerodynamic
stabilization and attitude control for descent, tumble recovery and landing.

## Scope

- Runtime deflection of the existing `WingControl` without recreating
  `WingSurface`; deflection becomes mutable state rather than a
  construction-only parameter.
- `PhysicsClient` commands to drive individual rudders and flaps.
- Deflection-angle limits, parameter validation and atomic command application;
  invalid, non-finite or out-of-range deflection is rejected without mutation.
- Use the existing full-angle polars so reverse flow and tumbling remain
  supported.
- Correct force and moment from deflected surfaces, summed about the body COM
  with the current world-space conventions.
- Load diagnostics for automatic stabilization systems, exposing per-surface
  wrenches consistent with the applied loads.

## Dependencies

- Merged `feat/aero_rest` (stable wing models, full-angle polars, part wing
  wrenches and assembly integration).
- No dependency on `aero-shapes` or `aero-async`; may be developed in parallel.

## Acceptance criteria

- Deflection changes take effect at runtime without recreating the wing
  surface or invalidating visibility preparation.
- Force and moment signs and magnitudes are checked at representative Mach
  numbers (e.g. 0.5, 2, 8) across the supported polar domains.
- Returning a surface to neutral restores the baseline loads.
- Reverse flow is handled through the full-angle polar in both world step
  paths.
- A deflection change performs no steady-state per-step allocation and does not
  rebuild geometry.
- Stale or foreign part/wing identities are rejected without mutation; batched
  commands apply atomically.
- Diagnostics expose per-wing wrenches consistent with the applied loads.
- Ordinary and sharded step paths agree; docs and public API snapshots are
  updated.

## Non-goals

- new physical models, pressure closures or atmosphere profiles;
- shared prepared shapes (`aero-shapes`) or asynchronous cooking
  (`aero-async`);
- a full autopilot or guidance controller (only the surface actuation and load
  diagnostics it would consume);
- heating, ablation or network replication;
- claiming independent physical validation.
