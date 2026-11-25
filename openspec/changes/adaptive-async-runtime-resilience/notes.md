# Baseline and evidence

Baseline implementation: `826832c3568d6eb2856f1b91a5a85de3b6ebe22b`; the working
starting point `f603ae4cb2e77472b4243d88771795d573cb9f39` adds only these change
artifacts. `kapitsa` has no default features; `service` enables Tokio and `parallel`,
and `parallel` enables Rayon. Tests below were run on the starting point.

| Command | Result |
| --- | --- |
| `cargo test -p kapitsa --features service runtime::asynchronous::service` | 36 passed |
| `cargo test -p kapitsa --features service runtime::adaptive_step` | 13 passed, 1 ignored benchmark |
| `cargo test -p kapitsa-core failed_step` | 2 passed |
| `cargo test -p kapitsa-core parallel_ccd_abort_does_not_publish_completed_islands` | 1 passed |
| `cargo test -p kapitsa-core sharded_` | 66 passed |
| `cargo test -p kapitsa-core physics_step_metrics_are_empty_after_a_failed_step --features bench-internals` | 1 passed |

The four explicitly ignored `resilient_` baseline probes are run by
`cargo test -p kapitsa --features service resilient_ -- --ignored --nocapture`.
On the baseline all four fail: the numerical rejection terminates the service
(the same initial state succeeds at half the duration), the safe approximately
866 µs bound returns `Limited`, admission retains 10 s, and a queued velocity commits 1 ms
when the post-command bound is approximately 866 µs. The original probe used
500 µs incorrectly: the existing estimator uses the radius of the sphere's
bounding box, `sqrt(3) * 0.5 m`, not the sphere radius. The corrected threshold
preserves the estimator contract and still detects the original 1 ms violation.
They initially exercise legacy machinery
because the opt-in policy does not exist; as each policy component is implemented,
wire its corresponding probe to that policy and remove its ignore annotation.
The legacy tests and their assertions must continue to pass separately.

## Transaction audit in progress

`with_force_source_updates` stages published diagnostics and restores prepared
gravity, aerodynamic and persistent-force motion on an error. `step_prepared`
and `commit_step_shards_inner` abort collision staging on the documented error
paths; abort discards staged caches, lifecycle, joint impulses and context, and
marks the spatial index dirty. Body integration, joint removal, collision event
publication and weld-split finalization occur after precommit validation. Weld
split bodies are inserted during the last staging operation, so panic and
allocation-failure paths require separate integrity evidence before retrying.
In particular, a panic after successful weld-split body insertion cannot yet be
certified as rolled back. The asynchronous runner currently turns caught panics
into `CollisionError::WorkerFailed`, the same typed cause used for a reported
executor error, so the two must not be considered equally safe for retry.

Two reference-world tests now pass for a rejected prediction after force
preparation (ordinary and sharded) and a sharded collision-worker failure before
commit. They verify motion and event/impulse counts after a smaller successful
step. Follow-up tests cover sleeping state, preservation of already-published
contact events and impulses, and successful retry with aerodynamic, gravity,
one-shot and persistent forces. Component tests cover contact/joint warm-start
abort, and a two-cluster weld split test covers topology after allocation failure.
These do not establish every state component required by task 1.2.

An additional test, `failed_step_does_not_publish_a_ready_aerodynamic_shape`,
demonstrated an atomicity issue on the baseline: `finish_pending_shapes` installed
and acknowledged a completed shape during aerodynamic preparation, before a
subsequent gravity failure. The failed step left a new shape binding in the
world.

The agreed design is implemented: shape registry insertions receive an
insertion checkpoint, staged bindings commit only after integration succeeds,
and failed ordinary/sharded attempts restore the prior binding and dense-pool
free-list/generation state. Completion is delivered only on commit. The
regression covers both an absent previous shape and an existing previous shape.
The tested ordinary error paths preserve the listed state components. Panic
paths do not have a complete integrity proof and must be classified as fatal,
not retried as ordinary worker failures.

Core sends shape-completion notifications after core commit, before the service
publishes its new snapshot. The service exposes this completion channel to
clients, so task 3.2 must also address ordering against service snapshot
publication. The core-only change does not establish that host-facing guarantee.

Focused release allocation checks: `shared_shape_warm_steps_add_no_allocations`
reported 4,800 allocations / 1,168,384 bytes for 16 warm steps with four bodies,
both before (`f603ae4`) and after this change; its body-local and shared-shape
measurements were equal in both runs. `pending_shape_status_and_collection_do_not_allocate`
reported zero allocations across 128 pending polls on the candidate. These are
focused allocation probes, not the paired runtime performance gate in task 6.3.

Resilient service selection now applies queued FIFO mutations before choosing
the attempt. Tests cover velocity, impulse, one-shot force and spring-joint
changes, plus a 5 µs absolute horizon that commits exactly one 5 µs step.
Legacy service tests remain green (40 passed, 2 ignored in the focused service
module).

Validation of the in-progress transaction work: `cargo test -p kapitsa-core --lib
--quiet` passed 1,384 tests with 3 ignored before the final test-only sleeping
state extension; the extension passed its focused test. The targeted shape and
generation tests, warm-start cache tests, and multi-cluster weld rollback test
also passed. `cargo clippy -p kapitsa-core --all-targets --all-features -- -D
warnings` and strict validation of this change passed before the last test-only
edits. Full workspace checks and benchmarks remain outstanding.

`physics_step_metrics()` under `bench-internals` is an attempt diagnostic, not
the last committed diagnostic: the existing `physics_step_metrics_are_empty_after_a_failed_step`
test explicitly requires it to be empty after a failure. The regular
`step_diagnostics` uses pending and published storage and swaps only on success;
`step_diagnostics_publish_only_on_success` checks that contract. Preserve both
meanings rather than treating an empty failed-attempt metric as a committed step.

The existing service chooses its adaptive duration before applying queued commands,
returns on any step error, retains unbounded demand, and enforces a 0.30 load floor.
The legacy tests intentionally cover these behaviours; opt-in tests must not change them.

## Initial error-path matrix

These are conservative decisions for the audit, not claims that a general retry is
already safe. All errors currently terminate the service. Comprehensive rollback and
equivalent-reference tests remain tasks 1.1–1.2.

| Typed cause | Baseline rollback evidence | Proposed decision pending audit | Existing test |
| --- | --- | --- | --- |
| `StepError::InvalidDeltaTime` | rejected before preparation | reject invalid request; no retry | adaptive configuration tests |
| `StepError::BodyIntegration` / `IntegrationError::ArithmeticOverflow` | sharded fixture restores bodies and force preparation for one case | corrective input; dt sensitivity not established generally | `sharded_integration_error_rolls_back_force_sources_and_can_retry` |
| `StepError::Collision` / `CollisionError::CcdLimitExceeded` | parallel fixture preserves body states and previously committed events/impulses; not a complete world proof | candidate for bounded smaller-step retry only after full audit | `parallel_ccd_abort_does_not_publish_completed_islands` |
| `StepError::Collision` / `CollisionError::NonConvergent` | collision abort exists; complete rollback and dt sensitivity not proven | no automatic retry until demonstrated | no dedicated retry reference test yet |
| `StepError::Collision` / `CollisionError::WorkerFailed` | selected sharded executor failures restore body motion and prepared force sources | no automatic retry until ownership and complete rollback are proven | `sharded_executor_error_rolls_back_force_sources_and_can_retry`, `sharded_collision_solver_error_rolls_back_force_sources_and_can_retry` |
| `StepError::Gravity`, `Aerodynamics`, `ForceSource`, `WeldBalance`, `WeldSplit` | individual preparation rollback exists; not a general proof for all variants | corrective input or fatal if integrity is uncertain; no blanket retry | `failed_step_restores_body_after_aerodynamics_and_gravity_preparation` |
| unknown cause, lost worker/world, `InvalidStepResults` | integrity not established | fatal if ownership or rollback cannot be established; never silently retry | `failed_step_reports_runner_cause_instead_of_collision_failure` |

The service-independent retry decision is now wired into the in-progress task 2.4
service path. A real strict-policy CCD fixture demonstrates that a
rejected 10 ms step can commit at 5 ms without changing the CCD policy. Unknown
causes and unverified worker failures are fatal; an intact-worker retry decision
requires explicit integrity evidence and remains bounded. `NonConvergent` and
arithmetic overflow are not classified as universally dt-sensitive.

Task 2.4 carries a reduced retry duration across turns and counts attempts against
the turn limit. An injected rejection verifies a single impulse and a 5 ms commit;
a separate high-speed contact and spring-joint fixture compares committed state
and collision event content with a reference world. Failure exhaustion and
corrective-command behaviour are being implemented in task 2.5. Exhaustion and
unsafe physical bounds now produce a typed blocked status; tests cover no time
accumulation, query service, and corrective resumption. Unverified worker failure
is fatal. A bounded retry for a worker whose intact world is proven has not yet
been integrated or tested; 2.5 remains open.

The executor audit found that `CollisionError::WorkerFailed` represents both an
executor-returned error and a caught panic in multiple core paths. The runtime
cannot infer an intact world from that variant, so it reports fatal and refuses
restart; a typed provenance boundary and rollback evidence are still needed for
a safe transient-worker retry. Stop during a rejected attempt is covered by
`stopping_after_rejection_resolves_applied_command_once`.

The bounded worker retry handler is compiled in production and exercised by two
paused-clock tests that inject failure before dispatch moves the engine. Current
Rayon dispatch exposes no verified transient failure, so production does not yet
assert that a worker failure is intact; ambiguous worker errors remain fatal.
The service suite passed 50 tests with 2 ignored. The paused-clock test
`paused_during_worker_backoff_accepts_commands_without_advancing` also checks
that a pause permits command publication before a long backoff expires.

Task 3.1: `retry_applies_impulse_and_one_shot_force_once_with_persistent_load`
compares the committed state with a reference world, including an invalid atomic
batch. `retry_preserves_structural_commands_and_fifo_queries` checks addition,
removal and query ordering across a rejected attempt and compares the resulting
body with a successful-step reference.

Task 3.2: deferred outcomes are capped across command turns, and
service exit now settles already-applied requests even if no attempt started.
`deferred_results_are_bounded_across_command_turns` and
`stop_settles_applied_and_rejected_requests_before_any_attempt` cover these
paths. `saturated_blocked_queue_reaches_corrective_command` verifies that a
corrective request behind a full queue is applied and acknowledged. Existing
failure and publication tests cover typed step causes and snapshot ordering.

Task 3.3 adds a checked monotonic snapshot publication revision, including
command-only publications and restart. The pause/horizon regression checks
acknowledgement ordering, unchanged time/step and absence of collision and
joint-break events; the blocked recovery regression checks the revision before
resumption. Revision exhaustion is explicitly rejected before publication.

Task 3.4: `retained_snapshot_and_lagging_events_do_not_block_stepping` gates a
worker, retains its prior snapshot through more than the event-channel capacity,
checks monotonic observations, snapshot-before-event and explicit lag. Rejected
service attempts emit neither event stream; core tests
`rejected_collision_work_then_smaller_step_matches_reference` and
`aborted_precommit_discards_staged_joint_break` cover staged collision and joint
break rollback respectively. Event channels are bounded and stale snapshots are
owned by readers, not kept in a producer queue.

Task 4.1 bounds admitted time by recovery debt and the remaining horizon,
records discarded demand with explicit saturation, removes the resilient load
floor, and caps unrepresentable adaptive targets. Tests cover huge finite warp,
the lower admission factor, demand limits and a gated overloaded successful
step compared with a reference world. Paired release measurements remain in 6.3.

Task 4.2: attempt slots include retries; command-only paths yield between
bounded batches and check stop before draining another batch. Tests cover stop
with a saturated blocked queue, pause/horizon changes during a gated attempt,
and repeated overload with bounded debt and reference physics. Existing blocked
and retry-budget regressions exercise idle waiting and retries across turns.

Task 4.3 publishes active retry details, selected and attempted durations,
attempt/retry/rejection counts, consecutive failures, the last typed failure,
and committed simulation time. Historical diagnostics survive a successful
retry and stop; tests cover retry, blocked, recovered and fatal states alongside
the overload checks. The latest selected safe duration is distinct from the
actual retry duration; it is not presented as the raw physical estimator bound.

Task 4.4 adds `spawn_on_tokio_and_rayon` to compose a host runtime with a
caller-owned bounded pool. The gated lifecycle test covers owned and shared
pools, cancellation of stop, accepted-request resolution, retained snapshots,
restart and caller pool reuse. Existing `spawn_on_rayon` coverage checks its
dedicated runtime and independent shared-pool lifetimes; no thread termination
is claimed for permanently hung work.

Task 5.1: `cargo run -p kapitsa --features service --example resilient_host`
completed and reported 200 ms committed time. The example uses an owned clock
runtime, a host-owned Rayon pool, bounded submission, persistent force, zero
horizon setup, acknowledged environment update and a later horizon extension.

Task 5.2 updates the runtime guide, both README entry points and relevant
comments with the recovery matrix, resource limits, publication and host-time
contracts. The `kapitsa` service doctests passed and the example ran again.

Task 5.3: regenerated both `scripts/public_api_snapshot.py` and the nightly
rustdoc JSON snapshot; the source snapshot `--check` passed. Cargo dependency
trees show no Tokio in default or parallel-only builds. The workspace declares
version 0.7.0; no separate release/version policy was found in the reviewed CI,
README or manifests, and no release or version bump is part of this change.
Adding public `SimulationMetrics` fields affects downstream struct literals and
should be considered in release compatibility review. The rustdoc path snapshot
also picks up existing `PhysicsSystem` paths missing from the old baseline.

Verification: the first `cargo test --workspace` failed in
`part_aerodynamic_geometry_commands_apply_loads_on_both_executors` with a full
shape-worker queue. The same isolated test passed three times and failed on its
fourth run at baseline `f603ae4` with the identical error. Canceled requests
can still occupy the bounded worker queue until its worker drains them; this
pre-existing scheduling-sensitive test failure is retained here rather than
changing its assertions. The full command must still be rerun and recorded.

The rerun of `cargo test --workspace` passed. The first all-features build ran
out of tmpfs quota; after cleaning reproducible debug artifacts it compiled but
`physical_mesh_continuous_landing_regression` failed at frame 441 with collision
nonconvergence. The isolated test fails identically at baseline `f603ae4`, with
the same reported body state. This is a pre-existing deterministic failure;
remaining all-features tests are being run with `--no-fail-fast`.

The completed `--all-features --no-fail-fast` run failed two targets: the
baseline-identical landing nonconvergence and the baseline-reproduced
shape-worker queue race. Core all-features library tests passed 1442 with 20
ignored; the other executed targets completed. Task 6.1 remains unchecked.

Task 6.2: all three requested `kapitsa --no-default-features` test commands
passed (plain, `parallel`, `service`); the resilient host example ran to 200 ms.
The suites include `explicit_step` and legacy fail-fast step-failure tests.
Plain and parallel builds initially warned about a service-only helper; it was
feature-gated and both corresponding all-target clippy checks now pass with
warnings denied.

Task 6.3 partial evidence (not complete): release service tests passed 66,
2 ignored. On AMD Ryzen 7 255, rustc 1.99.1, the existing scheduler benchmark
was warmed up and run five times per revision using 100 bodies and four Tokio /
Rayon workers. Raw output is in `benchmark-baseline.txt` and
`benchmark-candidate.txt`. At requested 1x both committed 0.3000 simulated
seconds per sample, achieving 0.99–1.00x. Median of the five reported runner
medians was 74.474 µs baseline and 72.875 µs candidate; median of reported CPU
p95 values was 72.864 and 73.806 µs. These are diagnostic and noisy; the
benchmark uses the legacy policy, measures publication rather than snapshot
read latency, and its overload runs do not commit equal simulated time. It
does not establish resilient high-speed/contact/overload performance, retry
counts, or the required complete paired report, so 6.3 remains unchecked.

Additional contact/CCD measurements: `cargo bench -p kapitsa --bench collisions`
was run on `f603ae4` and `1fbd76c`, using the same release build, one warmup and
five samples, eight 1/60 s contact steps or one 1/60 s CCD step per sample.
The raw command outputs are in `benchmark-collision-baseline.txt` and
`benchmark-collision-candidate.txt`. Median eight-step times for 256 / 4096 /
20000 bodies were 3.69 / 58.25 / 259.87 ms baseline and 3.66 / 59.21 /
260.33 ms candidate. The 256 / 1024-body CCD medians were 5.839 / 24.337 ms
and 5.783 / 24.066 ms respectively. The existing benchmark prints aggregate
min/median/max, and only a median for CCD; it does not retain raw individual
samples or measure the service, so it does not satisfy task 6.3 by itself.
Snapshot diagnostics were added subsequently in `779896e`; these core-only
measurements do not assess that additional snapshot copy.

Snapshot-read probe: `benchmark-snapshot.rs` uses a paused single-body service,
10,000 warmup reads and five sets of 100,000 individually timed snapshot/body
lookups. The same standalone release harness with `kapitsa` path pointed to
`f603ae4` and `fc03f29` produced median 30 ns in every set; p95 was 30 ns for
all baseline sets and 40/30/30/30/31 ns for candidate. Command output is in
`benchmark-snapshot-baseline.txt` and `benchmark-snapshot-candidate.txt`.
Timer resolution and the inactive producer limit interpretation; this is not an
in-flight contention measurement. The probe depends on `kapitsa` with feature
`service` and Tokio 1 with `rt`, `time` and `sync`, and is run with
`cargo run --release --offline` from a separate Cargo project.

After snapshot diagnostics were added, `cargo fmt --all -- --check`,
`cargo clippy --workspace --all-targets --all-features -- -D warnings`, all 15
focused snapshot tests, and the host example passed. Clippy's first rebuild ran
out of tmpfs quota; removing reproducible baseline build outputs allowed the
unchanged command to pass. `openspec validate --all --strict` passed all seven
changes. The complete workspace test results above predate `779896e`.

Scenario audit against the delta specification (test names are reproducible via
`cargo test -p kapitsa --features service NAME` unless noted):

| Delta scenario | Evidence |
| --- | --- |
| Existing callers retain semantics | `explicit_step` integration suite; `applied_batch_reports_step_failure_without_rolling_back_commands` |
| Invalid resilience configuration | `resilient_policy_requires_valid_adaptive_resource_bounds`; `invalid_recovery_policy_is_rejected_before_spawn` |
| A queued mutation changes physical bounds | `resilient_step_uses_queued_impulse_force_and_joint_bounds` |
| A safe step is below the preferred minimum | `resilient_step_progresses_below_the_preferred_minimum` |
| No representable safe step | `resilient_step_observes_floor_short_demand_and_duration_rounding`; `unsafe_floor_blocks_until_motion_is_corrected` |
| A short horizon remainder | `resilient_short_horizon_commits_exactly_its_remainder`; `pause_and_horizon_preempt_catchup_after_inflight_step` |
| Transient numerical rejection | `resilient_retry_commits_only_the_successful_half_step`; `retry_matches_reference_contact_and_joint_step` |
| Persistent failure | `exhausted_retries_block_without_spinning_and_corrective_command_resumes` |
| Unknown or unrecoverable fault | `only_proven_causes_are_retryable`; `unverified_worker_failure_is_fatal_instead_of_retried` |
| Executor failure | `intact_dispatch_failure_uses_bounded_backoff_before_retry`; `intact_dispatch_failure_stops_after_retry_budget`; `unverified_worker_failure_is_fatal_instead_of_retried` |
| Failure after preparation or collision work | `kapitsa-core` tests `rejected_prediction_then_smaller_step_matches_reference_after_force_preparation` and `rejected_collision_work_then_smaller_step_matches_reference` |
| A ready aerodynamic shape precedes a failed step | `kapitsa-core` test `failed_step_does_not_publish_a_ready_aerodynamic_shape` |
| Impulse and force survive retry correctly | `retry_applies_impulse_and_one_shot_force_once_with_persistent_load` |
| A command was applied but no step can commit | `stopping_after_rejection_resolves_applied_command_once`; `exhausted_retries_block_without_spinning_and_corrective_command_resumes`; `single_mutations_wait_for_snapshot_and_report_failed_step` |
| A worker is gated | `retained_snapshot_and_lagging_events_do_not_block_stepping` |
| Commands while blocked or horizon-limited | `pause_and_horizon_publish_distinct_revisions_without_step_events`; `unsafe_floor_blocks_until_motion_is_corrected` |
| Slow subscriber | `lagged_batches_report_the_exact_loss_and_receiver_can_continue`; `retained_snapshot_and_lagging_events_do_not_block_stepping` |
| Sustained overload | `sustained_overload_stays_bounded_and_advances_real_steps` |
| Pause and resume | `pause_and_horizon_preempt_catchup_after_inflight_step`; `unsafe_floor_blocks_until_motion_is_corrected` |
| Host uses a dedicated runtime | `owned_and_shared_pools_preserve_world_on_gated_stop_and_restart`; `resilient_host` example |
| Corrective command resumes progress | `exhausted_retries_block_without_spinning_and_corrective_command_resumes` |
| Host sets up environment at a boundary | `resilient_host` example; see limitation below |

Audit: the host example now installs nonzero gravity at the acknowledged horizon
and verifies its effect on subsequent committed velocity. A snapshot previously
lacked its committed diagnostics; `779896e` adds
them and `snapshot_keeps_committed_diagnostics_after_world_advances` verifies
the snapshot retains the prior step's diagnostics after the world advances.
Task 6.4 remains open while remaining verification is resolved.

The equal-committed-time `service-benchmark` compares a baseline adaptive
service and resilient service with 50 ms warmup, 200 ms measured horizon and
five repetitions. Raw baseline and initial candidate output are in
`benchmark-service-baseline.txt` and `benchmark-service-before.txt`. In an
ordinary world the initial resilient service performed 25,268–26,513 steps,
versus 79–92 at baseline: it repeatedly consumed tiny newly admitted demand.
The scheduler now waits for the preferred minimum when the physical bound and
horizon permit; smaller physical bounds and short horizon remainders remain
eligible. `resilient_scheduler_waits_for_preferred_demand_when_physics_allows_it`
checks this policy. Its five ordinary follow-up samples took 88–103 steps, in
`benchmark-service-after.txt`. The follow-up contact run completed three samples
then failed to complete, so it was interrupted; no complete performance claim
is made. The harness samples step durations through the metrics watch channel,
which can skip steps, and its nominal overload scene does not demonstrate
sustained CPU overload. Allocation and queue-pressure measurements remain open.

Follow-up benchmark instrumentation put five-second timeouts around setup,
boundary commands and stop. One run timed out during the contact warmup or
following boundary command (`benchmark-service-timeout.txt`); the exact
operation was not distinguished in that run. A subsequent run with an extra
boundary marker and eight consecutive repeated runs completed
(`benchmark-service-completed.txt`, `benchmark-service-repeat.txt`). The
intermittent delay is not diagnosed or asserted fixed. Output logs retain it.

The service benchmark now includes 128 bodies in contact pairs at requested
1000x, one second of equal committed time, and five measured repetitions after
warmup. Raw paired outputs are `benchmark-service-loaded-baseline.txt` and
`benchmark-service-loaded-candidate.txt`. Median achieved warp was 7.840x /
7.764x; median sampled step wall time was 1.090 / 1.091 ms, and the median of
per-run sampled p95 was 1.169 / 1.255 ms (baseline / resilient). Maximum sampled
debt medians were 2.503 s / 0.980 s; candidate discarded approximately 123–128
simulated seconds and had zero retries in each sample. Both committed exactly
one second in 100 steps. Snapshot read medians were 130 / 120 ns and median p95
370 / 351 ns. Samples use step duration plus task-wait duration as the wall-time
proxy, polled approximately every 100 µs; some steps are missed. The measured
ordinary and contact cases also commit equal time, but differing adaptive step
sequences and scheduler noise prevent interpreting them as physical-equivalence
or precise overhead measurements. Existing allocation probes and bounded queue
tests are noted above. Full 6.3 remains open pending a retrying fault workload,
allocation/queue evidence under that workload, and interpretation of the observed
timing spread.

`cargo test -p kapitsa --release --features service,bench-internals --lib
benchmark_resilient_rejected_step -- --ignored --nocapture --test-threads=1`
ran one warmup and five measured injected-rejection episodes, each committing
5 ms with two successful 2.5 ms steps matching a reference, three attempts,
one retry and one rejection. Wall times were 11.166–11.261 ms including pacing;
process-wide tracked allocations were 22–23 calls and 281,331–282,851 bytes.
These include runtime and measurement activity, not just physics. Raw output is
`benchmark-retry.txt`; this test is candidate-only because the baseline exits
on the injected rejection. Queue limits are 1024 requests and 1024 deferred
results, with the saturated blocked-queue test above. The existing ignored
`benchmark_rocket_flight_runtime_queue_pressure` did not finish filling its
queue while the service consumed paused commands; it was interrupted after
about one minute (`benchmark-queue-interrupted.txt`). No completed allocation or
latency claim is made for that queue benchmark.

Queue benchmark follow-up: the fixture now gates an in-flight step before
filling the queue and consumes the first result before resubmitting the retained
overflow buffer. The same fixture was run five times against baseline `f603ae4`
and the candidate in default mode, then five times with resilient adaptive
mode. Raw logs are `benchmark-queue-baseline.txt`, `benchmark-queue-candidate.txt`
and `benchmark-queue-resilient.txt`. Each run admitted 1025 batches (131200
commands), observed eight full rejections, retained the 128-command overflow
buffer, and reported zero failed commands. Baseline/default candidate/resilient
median process-wide allocation calls were 6658/6658/6661; peak extra tracked
heap was approximately 108.5–108.7 MB in all modes. Median per-run p95 command
acknowledgement latencies were 1005.894/1005.718/11.768 ms. The first two use
fixed steps, while resilient uses adaptive steps, so these acknowledgement
times are workload descriptions rather than a like-for-like speedup claim.
The allocation tracker is process-wide and includes benchmark setup and queues.

After the scheduler fix, `cargo test --workspace --all-features --no-fail-fast
--quiet` again completed with the same two baseline-reproduced failing targets;
the core library passed 1442 tests with 20 ignored. The focused adaptive suite
passed 16 tests with one ignored, the service suite passed 66 with two ignored,
and focused all-target/all-feature clippy passed. The test for the physical
bound below the preferred minimum was also extended to use the service's new
scheduler selection path and passed.

`cargo test --workspace --quiet` passed after the scheduler change; the core
library reported 1386 passed and 3 ignored. The all-features failures remain
as recorded above, so task 6.1 is not marked complete.

The current Rayon executor exposes `InvalidChunkLayout`, a configuration or
invariant error, rather than a verified transient worker fault. The resumed
blocked-state test now also checks that an identical velocity update leaves the
service blocked; unchanged angular-velocity updates are handled likewise. Other
command types still need a complete no-op/relevant-mutation audit before task
2.5 can be accepted.
Body, joint and force-source batches now compare final state with their starting
state, so setting a value and restoring it does not unblock the service. Single
body, joint, force-source, atmosphere, central-gravity and collision-config
commands also compare their state, as do material, gravity-field and common
collider configuration updates. This bounded
temporary collection is used only while blocked; ordinary step processing does
not allocate it. The focused service suite passed 46 tests (2 ignored), and
all-target/all-feature clippy passed after updating the benchmark call site.

The focused service suite timed out twice in
`builder_installs_atmosphere_on_both_executors`; a subsequent isolated run and
the baseline passed. Its loop checked `snapshot()` and then called
`next_snapshot()`, which starts waiting from the latest publication, leaving a
race if the final horizon publication arrives between those calls. The test now
waits on the same horizon condition under a single timeout without changing its
state, time, or step assertions; the focused service suite passed afterward.
