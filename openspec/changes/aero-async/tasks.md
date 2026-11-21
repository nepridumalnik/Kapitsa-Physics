# Tasks

- [ ] Move `CookedShape` cooking onto worker threads outside the physics step.
- [ ] Add a preparation task queue with cancellation/rejection of stale
  results.
- [ ] Expose `Ready`/`Cooking`/`Failed` states with completion notification.
- [ ] Implement `PendingAero` policies `KeepPrevious`, `ZeroForce`, `BlockStep`
  with topology-aware behavior.
- [ ] Integrate with `PhysicsClient` for change submission, state observation
  and completion/failure delivery.
- [ ] Make install/abort atomic; preserve the last successful shape and error
  cause on failure.
- [ ] Add tests for concurrent edits, atomic publication, cancellation, error
  handling and ordinary/sharded determinism.
- [ ] Update `docs/aerodynamics.md`, the acceptance review and public API
  snapshots; record benchmark evidence that steps no longer voxelize.
