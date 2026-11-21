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
`e3cb8b5` for selective exact rechecks of zero projected determinants, and
`3144d7c` for selective exact affine coefficients and amortized carry charges,
`8607c83` for precharged exact evaluation of uncertain affine distances, and
`efedd1d` for reversible power-of-two normalization that preserves stored
shadow boundaries and windward values, and `c603ae5` for fused multiply-add
crossing interpolation with finite coordinate differences, and `a633369` for
fused residual correction of crossing fractions near binary rounding boundaries,
and `3ef81e7` for reciprocal residual correction in the larger endpoint-order
branch and its rational-reference regression, and `b3ab97b` for shared
power-of-two scaling of two subnormal crossing distances before fused ratio
residual correction and its exact rational-reference regression, plus
 `791d0e4` for exponent-based power-of-two normalization of the FMA residual
 operands, retaining the smaller magnitude when scaling down would erase it,
 and its exact rational-reference regression, plus `b1a963c` for a wide-range
 rational-reference regression recording an observed one-ULP difference,
 not a universal rounding bound.

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
- screening zero and nonzero fast direction or edge-reference determinants
  inside `32*epsilon` times the absolute triple-product sum; an error-free
  subtraction residual in a participating product selects charged exact
  orientation before tangent classification or degenerate-reference rejection;
  this does not provide a global numeric error bound;
- in an accepted frame with an accepted fast edge reference, screening each
  affine coefficient and precharging exact recomputation only for uncertain
  determinants; rounding each exact result on the same binary scale as its
  fast neighbors before normalization; without an accepted frame or reference,
  computing all three coefficients together;
- recording `projected_coefficient_uses_the_requested_binary_scale`: points
  `[0, X, Y*0.25]` and direction `Z` yield `2^(exponent-2)` at requested
  exponents `-1000`, `0` and `1000`; this is a synthetic assertion, not test
  execution evidence or independent physical validation;
- recording the opposite-sign nonzero-determinant regression, where the rounded
  fast sign differs from the exact projected sign and exact orientation is
  selected;
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
- bounding carry writes amortized across each determinant: each payload has
  one initial and at most one terminating write; intermediate carries clear
  all-ones limbs, which only those writes can create, allowing at most two
  intermediate writes per payload amortized; at most six payload words per
  term give `18*6*4 + 8*99 = 1,224` projected and
  `24*6*4 + 8*99 = 1,368` other determinant units, including fixed scans and
  initialization;
- recording synthetic cancellation, subnormal rounding and precharge budget
  rejection scenarios, without claiming execution or physical validation;
- retaining compensated affine distance evaluation when accepted; cancellation
  comparable to correction rounding, possible product low-bit underflow or
  intermediate overflow selects exact evaluation in clipping and coverage;
- precharging `5*6*4 + 8*99 = 912` regular surface-work units before accumulating
  `a + b*x - a*x + c*y - a*y` exactly from stored binary64 coefficients
  `[a,b,c]` and point `(x,y)` in fixed, allocation-free 99-word accumulators;
  rounding once, ties to even, at the original scale; preserving representable
  subnormals, returning `PrecisionLoss` for a nonzero result rounding to zero
  and `ArithmeticOverflow` for non-finite inputs or unrepresentable overflow;
- recording `8607c83` synthetic regressions for coefficient-correction
  underflow recovery, intermediate-overflow recovery and rejection, smallest
  subnormal sum recovery with exactly 912 charged units, and rejected precharge,
  without claiming test execution or independent physical validation;
- limiting exact affine evaluation to stored coefficients and points, not
  unrounded geometry; constructed intersection coordinates still round to
  binary64 and accepted fast paths have no global numeric error bound;
- excluding all charged exact evaluation work from the area consistency
  tolerance, including affine distance work;
- normalizing shadow depth and projected-edge coefficients and windward
  clipping values by a common positive power of two instead of their maximum
  absolute value; selecting the power from that maximum's binary64 exponent,
  with `f64::MIN_POSITIVE` for a subnormal maximum, and leaving all-zero
  triples unchanged;
- requiring exact reversibility for every normalized component: original
  `a == scaled * power` with no tolerance; returning `PrecisionLoss` on any
  mismatch, including nonzero-to-zero loss or rounded subnormal scaling,
  rather than shifting the stored affine half-space boundary;
- recording `efedd1d` synthetic regression
  `binary_normalization_preserves_exact_boundary`: `[3,-1,0]` normalizes to
  `[1.5,-0.5,0]` and its affine distance at `X*0.75` is exactly zero; this
  preserves the boundary of the stored coefficients, not unrounded geometry,
  and is not test execution evidence or independent physical validation;
- preserving representable shadow intersections when finite floating-point
  distance or coordinate differences would overflow during intersection
  arithmetic; crossing fractions use the smaller-to-larger absolute distance
  ratio, then reconstruct the endpoint-order result without forming an
  overflowing distance sum;
- applying `a633369` fused residual correction to the smaller fraction: with
  `r = small/large`, `d = 1 + r` and `f = r/d`, compute
  `e = r - (d - 1)`, `q = (-r).mul_add(L, S)/L` and
  `s = (-f).mul_add(d, r)`, then use `f + ((s + q) - f*e)/d`;
  these bounded correction quantities reduce ratio, sum and division rounding
  error without forming an overflowing distance sum or a separately rounded,
  potentially underflowing residual product; `S` and `L` are the residual
  operands selected by the shared scaling below;
- applying `b3ab97b` and `791d0e4` shared exact binary scaling before the FMA
  ratio residual: choose a common power-of-two shift from the binary64
  exponent of `large`, scaling toward the normal range; apply it to both
  residual operands when the shift is upward, and when the shift is downward
  retain `(S,L) = (small,large)` if scaling the smaller magnitude would erase
  information needed by the FMA. The common scale preserves the mathematical
  ratio and keeps the residual operands representable before division by `L`;
  the initial `r`, `d` and `f` remain computed from the unscaled ratio, and the
  resulting `q` is shared by the smaller and larger endpoint-order corrections;
- recording `b3ab97b` synthetic regression
  `crossing_fraction_corrects_two_subnormal_distances`: with
  `small = f64::from_bits(0x5d)` and `large = f64::from_bits(0x3c_f738)`,
  both `(small, -large)` and `(-small, large)` yield
  `f64::from_bits(0x3ef8_6816_e38d_5625)`, the nearest binary64 to the exact
  rational `small/(small+large) = 0x5d/(0x5d + 0x3c_f738)` formed from
  those stored inputs; the common subnormal unit cancels exactly; this is not
  test execution evidence or independent physical validation;
- recording `791d0e4` synthetic regression
  `crossing_fraction_corrects_normal_ratio_near_underflow`: with
  `small = f64::from_bits(0x2a38_b9be_9e64_a7b4)` and
  `large = f64::from_bits(0x6a04_e05d_a5d8_626a)`, the crossing fraction for
  `(small, -large)` equals `f64::from_bits(0x0022_f335_6d15_4415)`, the
  nearest binary64 to the exact rational formed from those stored inputs;
  this is a synthetic assertion only, not test execution evidence or
  independent physical validation;
- recording `b1a963c` synthetic regression
  `crossing_fraction_wide_range_stays_within_one_ulp`: with
  `small = f64::from_bits(0x3f9b_7e1e_cdef_1832)` and
  `large = f64::from_bits(0x7f49_8322_6928_7812)`, evaluate
  `crossing_fraction(small, -large)` against
  `f64::from_bits(0x0041_3df4_c5af_7ddf)`, the nearest binary64 to exact
  rational `small/(small+large)` formed from those stored inputs; the test
  asserts `actual.to_bits().abs_diff(expected.to_bits()) <= 1`, while the
  accompanying documentation records an observed one-ULP difference for this
  widely separated normal-distance pair. This does not establish a universal
  one-ULP bound or correctly rounded fractions and is not test execution
  evidence or independent physical validation;
- applying `3ef81e7` reciprocal residual correction in the larger endpoint-order
  branch: with the same `r`, `d`, `e` and `q`, compute `u = 1/d` and
  `v = (-u).mul_add(d, 1.0)`, then use `u + (v - u*(e + q))/d` instead
  of the uncorrected reciprocal; this accounts for reciprocal, denominator
  and ratio rounding without forming the distance sum;
- recording `3ef81e7` synthetic regression
  `crossing_fraction_corrects_large_endpoint_reciprocal`: with
  `a = f64::from_bits(0x4b18_5f64_cee3_79c6)` and
  `b = f64::from_bits(0x4ae1_f9d1_568c_8566)`, both signed-distance pairs
  `(a, -b)` and `(-a, b)` equal
  `f64::from_bits(0x3fed_4c81_d7e1_b240)`, the nearest binary64 to the exact
  rational `a/(a+b)`; this reference concerns the stored binary64 inputs,
  not exact geometry, test execution evidence or independent physical validation;
- recording `a633369` synthetic regression
  `crossing_fraction_corrects_ratio_at_rounding_boundary`: for
  `small = f64::from_bits(1)` and `large = f64::MIN_POSITIVE * 2.0`,
  `crossing_fraction(small, -large)` equals `2.0_f64.powi(-53).next_down()`
  rather than `2^-53` from the uncorrected fraction; this is not test execution
  evidence or independent physical validation;
- using `difference.mul_add(fraction, near)` for each finite coordinate
  difference `far - near`, rounding the product and addition together so a
  subnormal product residual can contribute before final binary64 rounding;
  retaining weighted endpoint interpolation when the difference overflows;
- recording `c603ae5` synthetic regression
  `crossing_interpolation_retains_subnormal_product_residual`: endpoints
  `X*m` and `-X*m`, where `m = f64::from_bits(1)`, with signed distances
  `1.0` and `-3.0`, give fraction `0.25` and round the resulting `m/2` to
  zero, identically after reversing endpoints and distances; separate product
  rounding would lose `-m/2` before addition and leave `m`; this is not test
  execution evidence or independent physical validation;
- recording the subnormal crossing-fraction regression for distances in the
  `1:2` and `2:1` ratios: the smaller-to-larger ratio remains representable,
  producing `1/3` or `2/3` in the corresponding endpoint order; this is not
  test execution evidence or independent physical validation;
- retaining binary64 rounding of coordinate differences, crossing fractions
  and constructed intersections, including possible fraction rounding to one;
  the ratio, `1 + ratio`, residual correction, reciprocal and final fraction
  remain binary64 operations despite exact shared subnormal scaling, with no
  universal correct-rounding guarantee;
  a smaller fraction rounding to zero still returns `PrecisionLoss` and
  non-finite constructed points still return `ArithmeticOverflow`; fused
  multiply-add does not make geometry exact or provide a global numeric error
  bound;
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
- recording the end-to-end wide projected-edge boundary at exactly 15,070
  units: twelve exact determinants at 1,224 units each plus 382 ordinary
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
