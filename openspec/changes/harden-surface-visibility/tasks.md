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
- [x] Record that work accounting does not provide an exact byte budget or
  exact instruction/elapsed-time bound.
- [x] Keep the retrospective change limited to
  `openspec/changes/harden-surface-visibility/`.
- [ ] Audit and complete the remaining hard work budget coverage for every
  intermediate operation before treating the preparation path as production-
  complete.
