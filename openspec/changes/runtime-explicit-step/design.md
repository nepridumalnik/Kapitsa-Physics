# Design

## Context

`PhysicsWorld::step_with_executor` is blocking and already implements transactional stepping. `RayonExecutor::from_pool` is available with `parallel`, independently of the Tokio-backed `RayonPool`. The test-only `runtime/sync.rs` uses fixed-rate runtime configuration and is not the public implementation.

## Goals / Non-Goals

Provide a world-owning `PhysicsSystem` for explicit host updates without pulling in Tokio. Keep automatic scheduling, time accumulation, host-level substeps, snapshots and rendering out of the interface. Preserve the existing solver's internal algorithms and the autonomous service.

## Decisions

- Build from a supplied `PhysicsWorld`. Expose direct world access for setup and mutation between calls. The default uses `SequentialExecutor` and allocates no worker pool.
- Under `parallel`, support a positive worker count for a privately constructed Rayon pool and an `Arc<rayon::ThreadPool>` for sharing the application's pool. Reuse `RayonExecutor` with a separate executor budget per system. Propagate pool construction errors with their causes. Drop owned references normally; do not shut down an app-owned pool.
- `update(dt: f64) -> Result<StepResults<'_>, Error>` executes one core step with `dt` in seconds. Use the core's validation and rollback. On success return a borrowed view of committed results without touching the staged `simulate` outcome or copying event buffers. Add a read-only committed-results accessor as needed; keep `fetch_results()` semantics intact.
- Keep the async API and feature gates unchanged. Verify the public API baseline and check the workspace's release policy before deciding whether this additive change requires a manifest version bump.

## Risks / Trade-offs

Shared pool contention can increase latency but must not affect deterministic committed outputs. An owned pool consumes threads per instance; sequential execution remains available when that cost is unwarranted. Borrowed results must be read before mutating the system again.
