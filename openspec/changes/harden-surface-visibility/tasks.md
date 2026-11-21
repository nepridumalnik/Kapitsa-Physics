# Tasks

- [x] Record the branch delta since `ccb5487`, including finite-difference
  overflow handling for crossing fractions and points.
- [x] Record `07c7285` independent power-of-two scaling per coordinate axis,
  the shared positive determinant factor within each orientation frame, and
  reversal of the two in-plane scales for barycentric area.
- [x] Record exact fallback for direction, depth and projected-edge frame-span
  rejection or direction-scale underflow; charge before exact work, use sign
  only for direction and edge references, and scale three edge coefficients
  separately; `d5624ae` removes rounded differences from exact projection only,
  retaining rounded fast-path offsets and zero-coefficient limits.
- [x] Document the two new synthetic regressions,
  `triangle_fraction_preserves_area_across_axis_scales` and
  `shadow_covers_thin_source_across_axis_scales`, without claiming execution
  or independent physical validation.
- [x] Record source-local area consistency checking and its heuristic,
  non-proven precision allowance.
- [x] Record clipping scratch reuse and the propagation of remaining hard
  work budget.
- [x] Record `d4dbf07` projected-edge axial-coordinate omission for exactly
  axial flow and its `1e-100` depth-separation regression across both signs
  of all three axes.
- [x] Record `9ac852c` signed axial separation for coordinate-aligned blocker
  depth and the wide within-axis-span regression; `0337ef5` supersedes the
  inclined depth frame's span rejection with exact fallback.
- [x] Record that work accounting does not provide an exact byte budget or
  exact instruction/elapsed-time bound.
- [x] Record `7d6e5a6` stable grid bucket construction from preflight entry
  counts, per-cell counts and prefix offsets, with input-order population of
  a single entry-sized index instead of sorting temporary entry pairs.
- [x] Record linear grid construction work: four source-triangle scans plus
  three units per entry, three per cell and one extra prefix offset.
- [x] Document `triangle_grid_scatter_preserves_bucket_membership_and_order`:
  exact synthetic dimensions, offsets and index contents, success at 67 work
  units and rejection at 66; record the updated 117-unit boundary in
  `unshadowed_patch_replacement_has_budget_boundary`, without claiming test
  execution or independent physical validation.
- [x] Record `8cf7271`: charge `1 + 16*n` before scratch growth and distance
  evaluation; make one exact reservation request per split side from measured
  counts.
- [x] Record inclusive half-space boundaries, strict opposite-sign crossings
  including the closing edge, logical bounds of `4*n` output vertex slots and
  `n` distance slots, and no new vertex buffer for an unsplit polygon.
- [x] Distinguish logical reservation requests from allocator capacity and
  bytes, without claiming test execution or closing the remaining work audit.
- [x] Record `0337ef5` and `cdc3e66`: exact signed binary64 products in fixed
  99-word accumulators, ties-to-even conversion, and separate edge-coefficient
  scaling from sign-only references.
- [x] Record precharge before fallback determinants: three other determinants
  cost `3 * 1,368`, while each projected determinant costs `1,224` under
  `3144d7c`; exclude exact depth work from the area consistency tolerance.
- [x] Document synthetic cancellation, smallest-subnormal preservation,
  underflow-to-zero `PrecisionLoss` and precharge `WorkBudgetExceeded`
  scenarios without claiming test execution or physical validation.
- [x] Keep the retrospective change limited to
  `openspec/changes/harden-surface-visibility/`.
- [x] Record `aec8f8d`: exact absolute-scale area fallback for windward and
  partition triangles beyond the 2D frame span, precharged against the regular
  surface-work budget; preserve representable subnormals or return
  `PrecisionLoss` for nonzero fractions rounding to zero.
- [x] Document `wide_triangle_area_preserves_absolute_scale_and_precharges_work`:
  smallest-subnormal area in either winding, exact work charge, underflow
  rejection and rejected precharge, without claiming execution or physical
  validation; remove the obsolete triangle-area frame-span limitation.
- [x] Record `d5624ae`: expand exact projected determinants from original
  endpoints and ray direction before subtraction; overflowing intermediate
  edges or offsets select fallback, while fast-path edges and offsets stay
  rounded. Non-finite original vertices or direction remain errors.
- [x] Record general depth evaluation (3D frame, then exact fallback) when a
  signed axial separation overflows, rather than rejecting the separation.
- [x] Document `projected_determinants_preserve_displacement_lost_by_subtraction`,
  `projected_determinants_accept_overflowing_endpoint_difference` and
  `axial_depth_accepts_overflowing_separation` as synthetic regressions only,
  without claiming execution or independent physical validation.
- [x] Record `b449468`: before direction or projected-edge frame evaluation,
  precharge exact projection if a component difference `a-b` equals `a` with
  nonzero `b`, or `-b` with nonzero `a`; keep rounded fast-path edges and
  offsets for other cases and use power-of-two direction rescaling.
- [x] Document `rounded_edge_difference_does_not_make_blocker_tangent`: a thin
  non-tangent blocker with a zero rounded edge cross product still casts a
  covering shadow, without claiming execution or independent physical validation.
- [x] Record the completed logical per-preparation work and storage audit,
  distinguishing logical element counts from allocator overhead, caller input,
  existing caches and elapsed time; retain the rounded numerical fast path
  without claiming its rounding is resolved.
- [x] Record the end-to-end wide projected-edge boundary: 15,070 units,
  comprising twelve exact determinants at 1,224 units each and 382 ordinary
  units; the minimum successful budget and hard cap return identical complete
  output, while one unit less returns `WorkBudgetExceeded`.
- [x] Record `b1ed96a`: screen zero and nonzero fast direction or
  edge-reference determinants inside `32*epsilon` times the absolute
  triple-product sum; inspect participating subtraction residuals and charge
  exact orientation when needed before classification.
- [x] Document the opposite-sign nonzero-determinant regression, without
  claiming affine-coefficient error is bounded.
- [x] Document `partially_rounded_projection_does_not_hide_thin_blocker` with
  neighboring `0.1` coordinates, retained endpoints, zero rounded edge cross
  product and successful shadow coverage, without claiming execution or
  independent physical validation.
- [x] Record `3144d7c`: screen affine coefficients in an accepted frame with
  an accepted fast reference; precharge and recompute only uncertain
  determinants on the same binary scale before normalization. Without an
  accepted frame or reference, compute all three coefficients together.
- [x] Record amortized carry bounds: initial and terminating writes alone can
  create all-ones limbs; intermediate carries clear them, bounding intermediate
  writes by two per payload amortized. Include six payload words per term,
  scans and initialization: projected charge `18*6*4 + 8*99 = 1,224`, other
  charge `24*6*4 + 8*99 = 1,368`.
- [x] Document `projected_coefficient_uses_the_requested_binary_scale` at
  exponents `-1000`, `0` and `1000`, without claiming execution or independent
  physical validation; record budget rejection at 15,069 units.
- [x] Retain rounded fast-path limitations without claiming a global numeric
  error bound.
- [x] Record `8607c83`: cancellation comparable to correction rounding, possible
  product low-bit underflow or intermediate overflow selects exact affine
  distance evaluation from stored coefficients and points in clipping and
  coverage; retain the accepted compensated fast path.
- [x] Record precharge of `5*6*4 + 8*99 = 912` regular surface-work units before
  exact accumulation of `a + b*x - a*x + c*y - a*y`; rejected precharge returns
  `WorkBudgetExceeded` before exact work. Exclude exact affine work from the
  source-partition roundoff allowance.
- [x] Record one ties-to-even rounding at the original scale, representable
  subnormal preservation, nonzero-to-zero `PrecisionLoss` and non-finite input
  or unrepresentable-overflow `ArithmeticOverflow`; do not claim exact
  intersection coordinates or unrounded geometry.
- [x] Document `affine_distance_recovers_after_coefficient_correction_underflow`,
  `shadow_coverage_recovers_from_intermediate_overflow` and
  `affine_distance_precharges_exact_recovery_of_subnormal_sum`, including the
  912-unit charge and rejected precharge, as synthetic assertions only, without
  claiming test execution or independent physical validation.
- [x] Record `efedd1d`: normalize shadow depth and projected-edge coefficients
  and windward clipping values by a common positive power of two selected from
  the maximum absolute value's exponent; use `f64::MIN_POSITIVE` for a
  subnormal maximum and leave all-zero triples unchanged.
- [x] Record exact componentwise reversibility `a == scaled * power`, without
  a tolerance; any mismatch returns `PrecisionLoss`, including rounding that
  does not erase a nonzero value. Limit boundary preservation to stored inputs.
- [x] Document `binary_normalization_preserves_exact_boundary`: `[3,-1,0]`
  becomes `[1.5,-0.5,0]` with exactly zero affine distance at `X*0.75`, as a
  synthetic assertion only, without claiming execution or independent physical
  validation.
- [x] Record `c603ae5`: use `difference.mul_add(fraction, near)` for finite
  coordinate differences, retaining subnormal product contributions through
  addition; keep weighted endpoint interpolation for overflowing differences.
- [x] Document `crossing_interpolation_retains_subnormal_product_residual`:
  endpoints `X*m` and `-X*m`, `m = f64::from_bits(1)`, and signed distances
  `1.0`, `-3.0` yield fraction `0.25` and ties-to-even zero from `m/2`, in
  either endpoint order; separate product rounding would leave `m`. Record
  synthetic assertions only, not test execution or independent physical validation.
- [x] Record smaller-to-larger absolute-distance ratios for crossing fractions,
  including the subnormal `1:2` and `2:1` regression (`1/3` and `2/3` in
  endpoint order); retain the underflow error as `PrecisionLoss`.
- [x] Retain rounded binary64 differences, ratios, `1 + ratio`, reciprocals,
  fractions and intersection coordinates, possible fraction rounding to one,
  and non-finite-point `ArithmeticOverflow`; do not claim exact geometry or a
  global numeric error bound from fused multiply-add.
- [x] Record `a633369`: correct the smaller crossing fraction using fused ratio
  and division residuals and the rounded denominator's error, with bounded
  correction quantities and no overflowing distance sum or separately rounded
  residual product; the larger endpoint-order branch is subsequently corrected
  by `3ef81e7` below.
- [x] Document `crossing_fraction_corrects_ratio_at_rounding_boundary`: smallest
  subnormal `f64::from_bits(1)` and opposite distance
  `-(f64::MIN_POSITIVE * 2.0)` yield `2.0_f64.powi(-53).next_down()`, not the
  uncorrected `2^-53`; record a synthetic assertion only, not test execution
  evidence or independent physical validation.
- [x] Retain binary64 rounding of the residual correction, no universal
  correct-rounding guarantee, possible fraction rounding to one, zero smaller
  fraction `PrecisionLoss` and non-finite-point `ArithmeticOverflow`; do not
  claim exact geometry or a global numeric error bound.
- [x] Record `3ef81e7`: for the larger endpoint-order branch, use
  `u = 1/d`, `v = (-u).mul_add(d, 1.0)` and
  `u + (v - u*(e + q))/d`, with `r = small/large`, `d = 1 + r`,
  `e = r - (d - 1)` and `q = (-r).mul_add(large, small)/large`;
  correct reciprocal, denominator and ratio rounding without forming the
  distance sum, while retaining the smaller-fraction underflow check.
- [x] Document `crossing_fraction_corrects_large_endpoint_reciprocal`:
  `a = f64::from_bits(0x4b18_5f64_cee3_79c6)` and
  `b = f64::from_bits(0x4ae1_f9d1_568c_8566)` give
  `f64::from_bits(0x3fed_4c81_d7e1_b240)` for both `(a, -b)` and `(-a, b)`,
  using the nearest binary64 to exact rational `a/(a+b)` as reference for
  stored inputs; record a synthetic assertion only, not execution evidence
  or independent physical validation, and retain all binary64 limitations.
