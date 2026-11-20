# Tasks

- [x] Record the branch delta since `ccb5487`, including finite-difference
  overflow handling for crossing fractions and points.
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
