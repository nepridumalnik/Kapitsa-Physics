# Harden surface visibility

## Summary

Document the surface-visibility hardening on `feat/aero_rest` since
`ccb5487`, including committed change `07c7285` for independent per-axis
predicate scaling in generic surface geometry and `7d6e5a6` for stable,
linear-work shadow-grid bucket construction without sorting entries, plus
`8cf7271` for measured clipping split reservations and `0337ef5` for bounded
exact inclined depth fallback, `aec8f8d` for absolute-scale triangle-area
fallback, `d5624ae` for exact projection from original vertex coordinates, and
`b449468` for early exact projection when subtraction erases a nonzero operand,
and `e3cb8b5` for selective exact rechecks of zero projected determinants.

## Scope

This change covers:

- scaling predicate coordinates independently by a power of two per axis,
  preserving the same positive determinant factor for orientation comparisons
  within a frame, including comparisons used as affine distances;
- recovering barycentric area by reversing the two in-plane scales, with exact
  absolute-scale fallback for windward and partition triangles when the 2D
  frame exceeds its span; precharging the regular surface-work budget,
  preserving representable subnormal fractions and returning `PrecisionLoss`
  when a nonzero fraction rounds to zero;
- recording the two synthetic regressions in `07c7285`: barycentric area
  preservation across axis scales and shadow coverage of a thin source;
- evaluating coordinate-aligned blocker depth by finite signed axial separation,
  using the general depth predicate (3D frame, then exact fallback) when that
  separation overflows;
- direction, depth and projected-edge predicates fall back to exact
  orientations when their frames exceed span or direction scaling underflows;
  direction and edge references use sign-only orientation, with edge
  coefficients scaled separately;
- expanding exact projection as `det(end-start, point-start, direction)` from
  original finite endpoints and the original ray direction, before subtraction;
  overflowing intermediate edges or offsets select exact projection;
- selecting precharged exact projection before direction or projected-edge
  frame evaluation when a component difference `a-b` equals `a` with nonzero
  `b`, or equals `-b` with nonzero `a`; other cases retain the rounded
  floating-point edge and offset fast path, subject to the zero recheck below;
- rechecking a zero fast direction or edge-reference determinant with precharged
  exact projection only when a nonzero determinant product can contain an
  inexact subtraction, detected by an error-free subtraction residual; this
  occurs before tangent classification or degenerate-reference rejection;
  an exact edge reference also selects exact edge coefficients;
- retaining the limitation that this zero check does not bound the error of
  nonzero fast determinants;
- recording `e3cb8b5` synthetic regression
  `partially_rounded_projection_does_not_hide_thin_blocker`: neighboring
  coordinates `x = 0.1` and `x.next_up()` produce a zero rounded edge cross
  product despite passing endpoint-retention checks, but the thin blocker
  still casts a covering shadow; this is not test execution evidence or
  independent physical validation;
- rescaling the projection direction by a power of two to preserve component
  ratios, retaining exact fallback for unsafe scaling;
- recording `b449468` synthetic regression
  `rounded_edge_difference_does_not_make_blocker_tangent`: a thin non-tangent
  blocker whose rounded edge cross product is zero still constructs a shadow
  covering source-affine point `(0.25, 0.25, 0)`, without claiming execution or
  independent physical validation;
- recording `d5624ae` synthetic regressions for displacement erased by rounded
  subtraction, overflowing endpoint differences and overflowing axial depth,
  without claiming test execution or independent physical validation;
- precharging fixed, allocation-free 99-word orientation accumulations;
  scaling coefficients and rounding ties to even, with `PrecisionLoss` if a
  nonzero coefficient rounds to zero;
- recording synthetic cancellation, subnormal rounding and precharge budget
  rejection scenarios, without claiming execution or physical validation;
- excluding exact integer depth work from the area consistency tolerance;
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
- recording the end-to-end wide projected-edge boundary at exactly 180,958
  units: twelve exact determinants at 15,048 units each plus 382 ordinary
  units; the minimum successful budget and the hard cap produce identical
  complete output, while one unit less returns `WorkBudgetExceeded`;
- carrying the remaining hard surface-work budget into grid operations and
  subsequent charged operations, subject to the existing hard cap; and
- completing the logical per-preparation work and storage audit, distinguishing
  logical element counts from allocator overhead, caller input, existing
  caches and elapsed time; rounded numerical fast paths remain rounded and
  are not claimed resolved; and
- documenting that work units are heuristic accounting weights, not an exact
  byte or instruction budget.

## Non-goals

- introducing a new visibility or physical model;
- claiming exact computational geometry or exact numerical error bounds;
- claiming an exact allocator-byte budget; and
- reporting final validation status.
