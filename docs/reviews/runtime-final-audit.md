# Runtime lifecycle and command pipeline audit

Audit baseline: `15a0559596deda4292e5037e10c5e75c2484714b` on `feat/collisions`.
The review covered the Tokio service, command marshalling, result/event streams,
snapshot reuse, command-buffer pooling, and the fixed-step engine.

## Summary

One P1 physics correctness defect was found and fixed: the parallel shard path
now prepares persistent force sources and rolls their staged changes back with
gravity if a step fails. The runtime lifecycle and command semantics otherwise
have no confirmed P0 defect. Tests cover the key ordering, atomicity,
cancellation, restart, failure, and bounded-pool contracts. The P2/P3 items
below are operational limits or observability gaps.

## Findings

| Priority | Finding | Evidence and impact | Recommendation |
|---|---|---|---|
| Fixed P1 | The parallel shard path skipped persistent force-source preparation. | `StepEngine::run_step` selected the shard path when multiple workers were active and more than half of the bodies were active. It omitted enabled persistent sources from shard copies. A runtime regression test reproduced this with four active bodies and an actual four-worker Rayon executor. | `StepEngine` now calls `PhysicsWorld::step_with_shards`, which stages force sources before shard fill and rolls force and gravity changes back on errors. Sequential and sharded steps match bit-for-bit for body position, orientation, and linear/angular velocity. |
| P2 | Event streams are bounded and intentionally lossy for slow subscribers. | Collision events have capacity 64 and batch results capacity 256. A receiver that falls behind receives a typed `Lagged` error and must decide whether to resynchronize. Accepted batches still execute; the stream is not a durable result log. | Drain result streams promptly, handle `Lagged`, and correlate important work through a separately retained application record if it must survive subscriber lag. |
| P2 | Snapshot reuse depends on readers releasing old snapshots. | The runtime recycles an `Arc<PhysicsSnapshot>` only when it has no external readers. A client retaining snapshots can prevent reuse and cause later snapshot buffers to be allocated. This is the cost of immutable snapshots and does not change their contents. | Keep only the snapshots needed by the application; use snapshot iteration/copying for long-term records. |
| P2 | Some non-physics step failures have a generic client-facing cause. | `StepFailedAfterApply` preserves a concrete `StepError` for physics failures. Internal runner failures such as worker-pool or world availability errors are converted to `StepError::Collision(CollisionError::WorkerFailed)` before being attached to the result, so their lower-level source is not available from that public result. | If these infrastructure failures become actionable in applications, add a compatible diagnostic channel rather than widening physics error semantics. |
| P3 | Runtime metrics are aggregate, not a full latency/allocation profile. | `SimulationMetrics` reports the latest, average, and maximum step time over a rolling window, deadline misses, worker counts, and time debt. It does not report p95/p99, command queue latency, allocation counts, or memory high-water marks. | Use the dedicated Release benchmarks for those measurements; add runtime telemetry only when a consumer needs it. |
| P3 | `Simulation::drop` requests shutdown but cannot await it. | Drop closes command submission and signals the task; orderly completion is observable only through `stop().await`. Dropping the Tokio runtime can cancel outstanding tasks. | Call `stop().await` when the host needs deterministic shutdown and access to the final simulation result. |

## Confirmed contracts

- A submitted buffer is limited to 128 commands. The limit is a public
  compatibility contract and bounds one atomic unit of work. The service admits
  at most 1,024 commands per physics step, so up to eight full batches can be
  applied in one step. The number 128 is not demonstrated to be a universal
  performance knee; changing it should follow separate workload measurements.
- The command channel holds at most 1,024 requests. Async `submit` waits for
  capacity; `try_submit` returns `QueueFull` with the original buffer intact.
- Each batch validates/stages all commands before committing. A failed command
  reports its index and leaves that batch's world changes uncommitted. Separate
  requests are not a single transaction and retain FIFO order.
- Batch and successful single-mutation acknowledgments follow successful step
  completion and snapshot publication. A physics error after command apply is
  reported as `StepFailedAfterApply`; the command-side world changes are not
  described as rolled back. The failed step does not publish a snapshot.
- Spatial queries execute once at their FIFO position before integration. Their
  results describe that earlier world state, even when the following step
  fails.
- Cancelling a client future before enqueue prevents submission. After enqueue,
  cancellation drops only the waiter; the request remains queued and is
  processed. Cancelling `stop().await` does not cancel the spawned simulation
  task, and a later `stop().await` can join it.
- The command-buffer pool retains at most 16 buffers and at most 256 KiB by its
  capacity accounting. Pool access uses `try_lock`; contention falls back to an
  empty buffer or drops the recyclable buffer rather than blocking the command
  path.
- The per-step result vector is allocated with capacity for 1,024 records and
  drained after each step. Batch staging vectors and indices are runtime-owned
  and reused. Snapshot reuse is conditional on `Arc` ownership as noted above.

## Existing test coverage

| Contract | Tests |
|---|---|
| Start, stop, restart, and command use | `simulation_start_stop_restart_and_client_commands_work`; `stopped_queue_batches_are_recycled_and_available_after_restart` |
| FIFO, batch atomicity, and snapshot-before-result | `batch_results_follow_fifo_and_wait_for_snapshot_publication`; `many_short_command_batches_are_processed_in_fifo_order`; `failed_body_batch_does_not_commit_earlier_body_changes`; `failed_force_source_batch_does_not_commit_source_or_body_changes` |
| Step failure after apply and cause preservation | `applied_batch_reports_step_failure_without_rolling_back_commands`; `single_mutations_wait_for_snapshot_and_report_failed_step` |
| Stop and cancellation | `cancelling_stop_during_step_preserves_applied_batch_completion`; `cancelling_before_send_does_not_enqueue_a_command`; `cancelling_after_send_does_not_remove_the_queued_command`; `stop_does_not_hide_a_result_published_at_the_same_time` |
| Queue backpressure and retry | `full_queue_applies_backpressure_until_capacity_is_available`; `nonblocking_submit_preserves_a_full_queue_buffer_for_retry` |
| Pool bound, reuse, and nonblocking contention | `pool_bounds_buffer_count_and_retained_capacity_bytes`; `recycle_clears_commands_and_preserves_capacity`; `lock_contention_uses_nonblocking_fallbacks` |
| Lag signaling, restart subscriptions, and snapshot readers | `lagged_batches_report_the_exact_loss_and_receiver_can_continue`; `subscription_reports_stop_and_resumes_after_restart`; `arc_recycling_skips_readers_and_reuses_released_capacity` |

| Persistent force sources in sharded steps | `sharded_step_matches_sequential_with_multiple_sources_and_torque`; `sharded_step_matches_sequential_with_gravity_and_force_sources`; `force_source_enable_and_active_body_path_switch_match_sequential_steps`; seven sharded failure/rollback/retry tests in `world_tests`; `cancelling_sharded_force_source_step_keeps_the_world_and_in_flight_result` |

## P1 fix validation and performance

The fix was first validated by a failing-before/fixed-after regression: the
four-body runtime shard path dropped a persistent source's `0.02` position
change at `dt = 0.1`; the sequential path produced it. After the fix, the
runtime path uses the actual Rayon executor and matches a sequential world
bit-for-bit. Coverage also includes multiple sources with order-sensitive force
accumulation, off-center torque, central gravity, source enable/disable,
switching between ordinary and sharded runtime paths, cancellation of a client
waiter during a sharded step, and rollback/retry after force-source, gravity,
fill, integration, executor,
collision preparation, and collision processing failures. Existing user
forces survive rollback.

Release benchmark environment: AMD Ryzen 9 7900X3D (12 cores / 24 logical
processors), Windows x86_64 MSVC, Rust 1.98.0, optimized `--release` profile.
The runtime harness uses four Rayon workers, 16 warm-up steps, five samples of
32 steps, and a separate warmed allocation-counting pass. Run with:

```powershell
cargo test -p kapitsa --release --lib --features bench-internals benchmark_sharded_force_source_scaling -- --ignored --nocapture --test-threads=1
cargo test -p kapitsa-core --release --lib --features bench-internals benchmark_force_source_update_scaling -- --ignored --nocapture --test-threads=1
cargo bench -p kapitsa --bench parallel_step
cargo bench -p kapitsa --bench collisions
```

Sharded runtime measurements (microseconds per step; min / median / max):

| Scenario | Time (µs) | Allocations / step | Requested bytes / step |
|---|---:|---:|---:|
| 1 source / 1 body | 0.650 / 0.703 / 0.791 | 0.000 | 0.0 |
| 0 sources / 128 bodies (control) | 52.481 / 55.466 / 62.484 | 4.000 | 160.0 |
| 128 sources / 128 bodies | 82.447 / 84.531 / 90.406 | 4.000 | 160.0 |
| 0 sources / 1,024 bodies (control) | 246.316 / 273.256 / 293.109 | 8.031 | 367.5 |
| 1,024 sources / 1,024 bodies | 536.906 / 551.106 / 585.475 | 8.000 | 320.0 |
| 0 sources / 8,192 bodies (control) | 1,862.763 / 1,989.525 / 2,005.638 | 16.000 | 640.0 |
| 8,192 sources / 8,192 bodies | 4,243.600 / 4,450.181 / 4,592.656 | 16.031 | 687.5 |
| 0 sources / one target (control) | 3.709 / 4.172 / 8.150 | 4.031 | 207.5 |
| 8,192 sources / one target | 1,190.703 / 1,199.903 / 1,225.150 | 4.000 | 160.0 |
| 0 sources / 1,024 bodies + gravity (control) | 489.225 / 505.984 / 519.337 | 8.031 | 367.5 |
| 1,024 sources / bodies + gravity | 770.519 / 856.400 / 903.816 | 8.031 | 367.5 |

Against no-source controls, adding sources costs about 29.1, 277.9, and
2,460.7 µs at 128, 1,024, and 8,192 distinct bodies. This tracks the number
of affected bodies and does not show a bodies-times-sources scan. By
comparison, 8,192 updates to one target add about 1,195.7 µs, so repeated
lookup of the same body is substantially cheaper than touching 8,192 bodies.
Adding 1,024 sources to the gravity workload adds about 350.4 µs. The runtime
allocation counts remain flat versus controls (at most one extra allocation
over the 32-step tracked sample); the counts grow with shard/task count, not
per source. The dedicated core benchmark records zero warmed allocations in
both source preparation and full stepping.

The dedicated core source benchmark at 8,192 sources measured preparation
medians of 808.467 µs for one body, 1,272.867 µs for distinct bodies, and
872.400 µs for mixed distribution; full-step medians were 1,127.900,
4,047.900, and 1,499.633 µs. All warmed preparation/full-step allocation
counts were zero. Source-preparation medians at 8,192 sources were also
compared to the immediately preceding run:

| Source distribution | Before P1 (µs) | After P1 (µs) |
|---|---:|---:|
| One body | 838.417 | 808.467 |
| Distinct bodies | 1,290.267 | 1,272.867 |
| Mixed | 887.880 | 872.400 |

The `parallel_step` four-worker medians before / after P1 were 0.089 / 0.086
ms (256 bodies), 1.416 / 1.355 ms (4,096 bodies), 6.888 / 6.674 ms (20,000
bodies), and 17.210 / 14.863 ms (4,096 bodies / 2,048 contact islands). The
collision benchmark medians before / after were 3.08 / 2.96, 43.58 / 45.04,
and 189.14 / 191.20 ms per eight-step sample for 256, 4,096, and 20,000
bodies. GJK island medians were 4.263 / 4.051 and 17.736 / 17.123 ms for
256 and 1,024 bodies. These runs show no broad hot-path regression; the small
collision-benchmark differences are within the observed sample ranges.
