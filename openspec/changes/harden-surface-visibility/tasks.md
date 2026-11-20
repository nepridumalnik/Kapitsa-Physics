# Tasks

- [x] Record the branch delta since `ccb5487`, including finite-difference
  overflow handling for crossing fractions and points.
- [x] Record `07c7285` independent power-of-two scaling per coordinate axis,
  the shared positive determinant factor within each orientation frame, and
  reversal of the two in-plane scales for barycentric area.
- [x] Record that exponent-span validation still applies within each axis;
  differences between axis scales alone do not reject thin geometry.
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
- [x] Keep the retrospective change limited to
  `openspec/changes/harden-surface-visibility/`.
- [ ] Audit and complete the remaining hard work budget coverage for every
  intermediate operation before treating the preparation path as production-
  complete.
