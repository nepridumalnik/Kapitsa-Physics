# Tasks

- [x] Move `CookedShape` cooking onto worker threads outside the physics step
  for managed explicit-mesh assemblies.
- [x] Add a preparation task queue with cancellation/rejection of stale
  results.
- [x] Expose `Ready`/`Cooking`/`Failed` states with completion notification.
- [ ] Implement `PendingAero` policies `KeepPrevious`, `ZeroForce`, `BlockStep`
  with topology-aware behavior.
- [x] Integrate with `PhysicsClient` for change submission, state observation
  and completion/failure delivery.
- [ ] Make install/abort atomic; preserve the last successful shape and error
  cause on failure.
- [ ] Add tests for concurrent edits, atomic publication, cancellation, error
  handling and ordinary/sharded determinism.
- [x] Update `docs/aerodynamics.md`, the acceptance review and public API
  snapshots; record a post-edit step comparison and separate regression
  evidence that managed explicit-mesh steps do not cook synchronously.
