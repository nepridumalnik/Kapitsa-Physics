# Runtime lifecycle and command pipeline audit

Audit baseline: `15a0559596deda4292e5037e10c5e75c2484714b` on `feat/collisions`.
The review covered the Tokio service, command marshalling, result/event streams,
snapshot reuse, command-buffer pooling, and the fixed-step engine.

## Summary

No confirmed P0 or P1 correctness defect was found in the reviewed lifecycle or
command paths. Existing tests cover the key ordering, atomicity, cancellation,
restart, failure, and bounded-pool contracts. The following P2/P3 items are
operational limits or observability gaps; they do not require changing runtime
semantics for this release.

## Findings

| Priority | Finding | Evidence and impact | Recommendation |
|---|---|---|---|
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

The audit found no small, isolated correctness defect that warranted a runtime
change. Later stages in this task add gravity, scheduler time scaling, and
flight workloads; they are not part of this audit's scope.
