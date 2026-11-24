# Tasks

All tasks are unchecked intentionally. Implement in order; each numbered group is an atomic reviewable milestone. Before code changes read AGENTS.md and design.md. Do not edit unrelated OpenSpec changes or archive this change while acceptance remains unmet.

## 0. Reproduce and map contracts (P0)

- [x] 0.1 Record baseline SHA, feature flags and relevant tests. Run existing focused service/adaptive/core transaction tests. Write a short error-path matrix in the change notes with actual typed error variants, whether rollback is proven, retry decision and corresponding test.
- [x] 0.2 Add failing deterministic regression for recoverable numerical rejection terminating the existing service, below-preferred-minimum hold, overload debt, and stale dt after a queued impulse or geometry/force mutation. Use test-only seams where deterministic injection is needed. Record baseline failures; do not change assertions to make baseline pass.

## 1. Core transaction and error classification (P0)

- [x] 1.1 Audit step_with_executor/step_with_shards, force preparation, collision abort, joint/cluster staging and commit; identify all mutation and publication boundaries. Add failure-after-preparation and collision/precommit tests against a reference successful smaller-step world.
- [x] 1.2 Repair only demonstrated rollback holes. Verify body state, sleeping, one-shot/persistent forces, contact warm-start, event/impulse buffers, topology, gravity/aero state and committed diagnostics. Include sequential and parallel execution; reuse scratch, no full-world clone per attempt.
- [x] 1.3 Implement narrow typed classification and pure bounded retry decision; distinguish dt-sensitive, corrective-input-required, transient worker failure with intact world, and fatal ownership/rollback failure. Test unknown error, invalid dt, strict policy, floor, overflow and exhausted retry budget. No string matching.

## 2. Opt-in adaptive policy and progress (P0)

- [x] 2.1 Add validated RecoveryConfig and StepRecoveryPolicy to config/builders/public exports; preserve FailFast defaults and reject unsupported combinations. Test invalid configuration before start.
- [x] 2.2 Refactor choose_step to expose the physical bound; keep existing legacy tests. Resilient adaptive mode may use a positive safe dt below preferred minimum to hard floor. Test safe smaller dt, floor violation, sub-floor horizon, duration quantization and overflow.
- [x] 2.3 Select/revalidate dt after FIFO mutations; add test that a newly applied impulse, force and collider/joint change influences that same attempt. Test horizon and exact committed time.
- [x] 2.4 Integrate retry context into service and StepRunner; halve dt with strict reduction, obey fresh bounds, keep retry count across scheduler turns, commit only actual successful dt, grow retry ceiling conservatively after success. A transient rejected step must not end service. Verify reference trajectory and event sequence; include a real high-speed contact and constrained joint fixture.
- [x] 2.5 On exhausted retry/floor enter explicit Blocked, keep service alive, do not retry at unchanged inputs or accrue new demand. Corrective mutation must resume without restart; distinguish fatal world loss and intentional pause. Test bounded worker retry only for proven intact world.

## 3. Commands, events and snapshots (P0)

- [ ] 3.1 Preserve command application exactly once across attempts; add tests for one-shot force, impulse, persistent force, structural addition/removal, invalid atomic batch and FIFO query. Verify against successful dt reference.
- [ ] 3.2 Defer step success results until matching committed snapshot; settle every accepted request once on Blocked/Fatal/stop with correct applied-vs-rejected semantics and original cause. Bound deferred results and ensure corrective commands cannot starve.
- [ ] 3.3 Service pause/horizon/Blocked command-only publications at unchanged physical time; add publication revision if required. Test acknowledgement after publication, distinct revision, no fictitious step/event, and recovery by corrective command.
- [ ] 3.4 Verify immutable snapshot reads while a worker is gated, monotonic committed step/time, snapshot-before-events, explicit event lag, and no failed-attempt collision/joint-break events. Verify stale snapshot readers do not block producer or grow an internal unbounded queue.

## 4. Load shedding and asynchronous lifecycle (P0)

- [ ] 4.1 In resilient mode remove the nonzero admission floor; bound accumulator by configured simulated-time debt and horizon, expose discarded demand and handle huge finite warp safely. Preserve successful physics under heavy load.
- [ ] 4.2 Count attempts against per-turn work limit, yield fairly, prioritize stop/pause/horizon and bound command work/backlog. Test with paused Tokio clock, gated executor, sustained overload and saturated command queue. No busy spin in Blocked.
- [ ] 4.3 Expose typed active recovery state, historical failure/attempt diagnostics, actual committed dt/time, load and debt metrics without conflating overload with physics failure. Test metrics across success, retry, block, recovery and fatal paths.
- [ ] 4.4 Verify dedicated Tokio and owned/shared bounded Rayon pool lifecycle; add additive spawn entry only if required to compose both existing APIs. Ensure last snapshot readable during in-flight work, stop waits only for active attempt and resolves requests, restart preserves world if valid, and dropping simulation does not shut down caller-owned pools. Test both pool modes and cancellation; no claim of forcibly interrupting a permanently hung thread.

## 5. Host contract and documentation (P0)

- [ ] 5.1 Add runnable dedicated-runtime example using mock nonblocking host loop, try_submit, persistent loads, committed snapshots and clean async shutdown. Show zero-horizon setup, acknowledged environment boundary and horizon extension. No Delta-V code.
- [ ] 5.2 Update docs/runtime.md, crate README/public docs and relevant error/metric comments. Document compatibility, typed recovery matrix, bounds, exactly-once semantics, below-minimum stepping, overload vs numerical failure, host time ownership, pause/horizon and explicit impossibility of progressing invalid/unrecoverable worlds.
- [ ] 5.3 Review public API snapshot scripts and release/version policy; regenerate/check baseline following repository instructions. Verify no Tokio dependency in default and parallel-only configurations.

## 6. Verification and performance (P0)

- [ ] 6.1 Run cargo fmt --all -- --check; cargo clippy --workspace --all-targets --all-features -- -D warnings; cargo test --workspace; cargo test --workspace --all-features.
- [ ] 6.2 Run cargo test -p kapitsa --no-default-features; cargo test -p kapitsa --no-default-features --features parallel; cargo test -p kapitsa --no-default-features --features service; run the new example with service. Test legacy exact one-step update and fail-fast behavior without weakening assertions.
- [ ] 6.3 Run relevant release tests and paired release benchmarks on baseline and implementation with identical inputs, committed time and environment (warm up; >= 5 repetitions). Report median/p95 step wall time, snapshot read latency, achieved warp, committed simulated seconds, retry counts, maximum debt, queue bounds, allocations or justified proxy; compare ordinary no-fault overhead and high-speed/contact/overload cases. Wall-time timing is diagnostic, not a deterministic unit assertion.
- [ ] 6.4 Run openspec validate adaptive-async-runtime-resilience --strict and openspec validate --all --strict (or installed equivalent). Record commands, outcomes, commits, benchmark raw data, remaining limitations and any pre-existing unrelated failures. Review every spec scenario against a named test or reproducible check. Do not mark unverified tasks complete or archive automatically.
