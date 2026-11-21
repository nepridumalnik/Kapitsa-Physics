# Tasks

## 1. Blocking updates and results

- [x] 1.1 Expose the committed result view without consuming staged results; test compatibility with `simulate` and `fetch_results`.
- [x] 1.2 Add the sequential `PhysicsSystem` builder and blocking `update`; run a no-Tokio example and test variable durations, committed events and rollback after validation and physics failures.

## 2. Parallel execution

- [ ] 2.1 Add private-pool and shared-pool builder modes using `RayonExecutor`; test invalid configuration, worker participation, independent worlds, pool reuse and release.
- [x] 2.2 Compare sequential and parallel body states, events, impulses and diagnostics in a deterministic contact fixture; run the parallel-only and service test configurations.

## 3. Public integration

- [x] 3.1 Document feature flags, pool ownership and result lifetimes in `docs/runtime.md` and provide runnable sequential and parallel examples.
- [ ] 3.2 Review release-version policy, regenerate and check the public API baseline, and verify the async service remains compatible.
