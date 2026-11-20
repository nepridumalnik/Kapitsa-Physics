# Harden surface visibility

## Summary

Document the surface-visibility hardening on `feat/aero_rest` since
`ccb5487`, including committed change `07c7285` for independent per-axis
predicate scaling in generic surface geometry and `7d6e5a6` for stable,
linear-work shadow-grid bucket construction without sorting entries, plus
`8cf7271` for measured clipping split reservations.

## Scope

This change covers:

- scaling predicate coordinates independently by a power of two per axis,
  preserving the same positive determinant factor for orientation comparisons
  within a frame, including comparisons used as affine distances;
- recovering barycentric area by reversing the two in-plane scales, while
  retaining exponent-span validation within each axis;
- recording the two synthetic regressions in `07c7285`: barycentric area
  preservation across axis scales and shadow coverage of a thin source;
- dropping the axial coordinate only from projected edge tests for exactly axial
  flow, while retaining three-dimensional depth signs for close blockers;
- preserving representable shadow intersections when finite floating-point
  distance or coordinate differences would overflow during intersection
  arithmetic;
- checking that each source triangle's clipped pieces conserve its local
  barycentric area, with a heuristic precision allowance based on work for
  that source partition;
- reusing clipping polygon lists and the vertex-distance scratch buffer within
  a source partition;
- charging `1 + 16*n` before scratch growth or distance evaluation, then making
  one exact reservation request per split side from retained vertices and
  strict crossings, including the closing edge;
- retaining boundary vertices in both half-spaces, bounding logical requests
  by `4*n` output vertex slots and `n` distance slots, and moving an unsplit
  polygon without a new vertex buffer; these are not allocator-capacity bounds;
- reusing the triangle count obtained during visibility preflight;
- preflighting grid entry counts, constructing per-cell counts and prefix
  offsets, and populating a single entry-sized final index in input triangle
  order, preserving canonical order within each bucket without a temporary
  entry-pair array or grid-entry sorting;
- recording linear grid construction work in source triangles, cells and
  triangle-to-cell entries, including the exact synthetic grid layout and
  budget boundary regression in `7d6e5a6`;
- carrying the remaining hard surface-work budget into grid operations and
  subsequent charged operations, subject to the existing hard cap; and
- documenting that work units are heuristic accounting weights, not an exact
  byte or instruction budget.

## Non-goals

- introducing a new visibility or physical model;
- claiming exact computational geometry or exact numerical error bounds;
- claiming an exact allocator-byte budget; and
- reporting final validation status.
