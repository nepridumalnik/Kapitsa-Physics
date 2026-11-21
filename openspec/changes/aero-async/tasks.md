# Tasks

- [x] Move `CookedShape` cooking onto worker threads outside the physics step
  for managed explicit-mesh assemblies.
- [x] Add a preparation task queue with cancellation/rejection of stale
  results.
- [x] Expose `Ready`/`Cooking`/`Failed` states with completion notification.
- [x] Implement `PendingAero` policies `KeepPrevious`, `ZeroForce`, `BlockStep`
  for managed explicit-mesh assemblies. Management follows the surviving body
  ID; a newly detached body must opt in separately.
- [x] Integrate with `PhysicsClient` for change submission, state observation
  and completion/failure delivery.
- [x] Make install/abort atomic; preserve the last successful shape and error
  cause on failure.
- [x] Add tests for competing cook requests, atomic publication, cancellation,
  error handling and ordinary/sharded parity at the same publication boundary.
  Covered: supersession before publication yields identical motion and pressure
  in both step paths; cancelling an already delivered result preserves the live
  binding and motion. This does not establish parity for arbitrary arrival
  times under nonblocking policies.
- [x] Update `docs/aerodynamics.md`, the acceptance review and public API
  snapshots; record a post-edit step comparison and separate regression
  evidence that managed explicit-mesh steps do not cook synchronously.
