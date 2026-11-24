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

The current Rayon executor exposes `InvalidChunkLayout`, a configuration or
invariant error, rather than a verified transient worker fault. The resumed
blocked-state test now also checks that an identical velocity update leaves the
service blocked; unchanged angular-velocity updates are handled likewise. Other
command types still need a complete no-op/relevant-mutation audit before task
2.5 can be accepted.
Velocity-only batches now compare final body motion with their starting state,
so setting a value and restoring it does not unblock the service. This bounded
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
