# Harden surface visibility

## Summary

Document the surface-visibility hardening on `feat/aero_rest` since
`ccb5487`.

## Scope

This change covers:

- preserving representable shadow intersections when finite floating-point
  distance or coordinate differences would overflow during intersection
  arithmetic;
- checking that each source triangle's clipped pieces conserve its local
  barycentric area, with a heuristic precision allowance based on work for
  that source partition;
- reusing clipping polygon lists and the vertex-distance scratch buffer within
  a source partition;
- reusing the triangle count obtained during visibility preflight;
- carrying the remaining hard surface-work budget into grid operations and
  subsequent charged operations, subject to the existing hard cap; and
- documenting that work units are heuristic accounting weights, not an exact
  byte or instruction budget.

## Non-goals

- introducing a new visibility or physical model;
- claiming exact computational geometry or exact numerical error bounds;
- claiming an exact allocator-byte budget; and
- reporting final validation status.
