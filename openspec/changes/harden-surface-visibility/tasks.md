# Tasks

- [x] Record the branch delta since `ccb5487`, including finite-difference
  overflow handling for crossing fractions and points.
- [x] Record `07c7285` independent power-of-two scaling per coordinate axis,
  the shared positive determinant factor within each orientation frame, and
  reversal of the two in-plane scales for barycentric area.
- [x] Record exact fallback for direction, depth and projected-edge frame-span
  rejection or direction-scale underflow; charge before exact work, use sign
  only for direction and edge references, and scale three edge coefficients
  separately while retaining rounded-input and zero-coefficient limits.
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
- [x] Record the `3 * (24 * 6 + 8) * 99` precharge before any fallback
  determinant; exclude exact depth work from the area consistency tolerance.
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
- [ ] Audit and complete the remaining hard work budget coverage for every
  intermediate operation before treating the preparation path as production-
  complete.
