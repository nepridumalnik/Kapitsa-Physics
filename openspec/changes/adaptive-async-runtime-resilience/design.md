# Design

## 1. Baseline and exact code locations

Use commit 826832c3568d6eb2856f1b91a5a85de3b6ebe22b as the reference; verify symbols in the current working tree rather than relying on line numbers.

| File | Current behavior / change point |
| --- | --- |
| crates/kapitsa/src/runtime/asynchronous/service.rs | run loop near 897–1045: selects dt before apply_queued_commands; failed step calls publish_failed_command_results and returns Err; start task maps Err to SimulationState::Error. Inspect choose_runtime_step, load_speed_for_step, simulation_debt, wait_for_scheduler_turn and paused/horizon paths. |
| crates/kapitsa/src/runtime/adaptive_step.rs | choose_step returns Wait, Limited, or Step; motion/acceleration/contact/joint limits; minimum_step defaults to 10 µs. No numerical retry feedback. |
| crates/kapitsa/src/runtime/engine.rs | StepEngine::step_with_dt marks metrics Error on failed run_step; success counters only on success. |
| crates/kapitsa/src/runtime/asynchronous.rs | StepRunner owns the Available/InFlight/Lost state, dispatches work through Rayon and retrieves the world after completion. Audit cancellation and worker failure. |
| crates/kapitsa/src/runtime/config.rs, constants.rs, metrics.rs, planner.rs | TimeStepPolicy, max_catch_up_steps, worker adaptation, MIN_LOAD_SPEED = 0.30, bounded command/event capacities. |
| crates/kapitsa/src/runtime/asynchronous/marshalling/{client,command,events,snapshot}.rs | bounded submissions, deferred acknowledgements, immutable snapshots, lag reporting. |
| crates/kapitsa-core/src/world.rs | step_with_executor, step_with_shards, commit_step_shards_inner, collision abort and rollback helpers. Review all failure exits and stage/commit boundaries. |
| crates/kapitsa/src/runtime/physics_system.rs | update(dt) is exactly one blocking step; preserve this API and the no-Tokio build. |
| docs/runtime.md | documents step failures after command application, one-step forces, horizons and adaptive limits; revise only for opt-in mode. |

Existing tests in service.rs include failed-step command results, overloaded_step_keeps_dt_and_reports_speed_floor, adaptive_time_warp_scales_physics_step_and_reports_unattainable_warp and command fairness. Keep their default-policy intent intact.

## 2. Ownership and execution

One service owns exactly one mutable PhysicsWorld. The host supplies a Tokio Handle for orchestration and either uses Kapitsa's existing Rayon pool construction or supplies an Arc<rayon::ThreadPool> using the existing spawn_on_rayon path. Verify actual ownership and lifecycle of both existing spawn methods. If a documented dedicated Tokio + explicitly supplied Rayon pool cannot be composed with these methods, add the smallest additive builder entry point accepting both. Do not create a new Tokio runtime per step, use the global Rayon pool, or introduce a second simulation service.

The host reads immutable Arc snapshots without waiting for a physics step. All world mutation happens on the simulation owner at safe boundaries. No long CPU work on Tokio scheduler threads and no await while holding a lock required by snapshot readers. Work already running cannot be forcibly cancelled safely; stop finishes the in-flight attempt, schedules no further attempts, resolves queued/deferred requests and returns the preserved world when possible. Never claim a timeout can kill a hung Rust/Rayon thread. No process isolation or arbitrary thread termination in scope.

## 3. Policy and state machine

Add a public opt-in policy (suggested StepRecoveryPolicy::FailFast / Resilient(RecoveryConfig)) via SimulationConfig and SimulationBuilder, following existing builder conventions. Default FailFast retains current behavior for both fixed and adaptive modes. Resilient initially requires TimeStepPolicy::Adaptive; reject unsupported policy combinations at spawn/config validation, before any world mutation. The synchronous PhysicsSystem::update and core step remain exactly one attempt. Factor the numerical retry decision into a small service-independent pure module; do not change those public contracts or make synchronous calls silently subdivide.

RecoveryConfig: positive finite/representable minimum recovery step (hard floor, default 1 ns), max retries per failed interval (default 8), max simulation debt (default 1 simulated second; also require at least one base step), and a bounded retry backoff for a verified transient worker fault (default 10 ms). These are initial resource-policy defaults, not physical accuracy constants. Validate relation hard floor <= adaptive minimum <= adaptive maximum; document limits and allow configuration. Avoid duration overflow and zero-progress rounding.

Keep existing SimulationState compatibility where practical: expose a separate typed recovery status rather than silently reusing Error or introducing an exhaustive-enum break. Suggested statuses: Healthy, Throttled, Retrying, Blocked(reason), Fatal(reason). Publish requested and achieved speed, current safe dt, attempted dt, retry count, consecutive rejections, last typed error with source, last committed step/time, debt and discarded demand. Distinguish intentional pause/horizon from numerical block. Retain latest fault until explicitly superseded; a success clears active fault, not historical counts. Failed attempts never increment committed steps. Use a separate monotonically increasing attempt identifier if needed; preserve existing result.step semantics.

State transitions:
- Healthy -> Throttled: CPU pressure; choose safe dt and lower admitted wall-time demand.
- Healthy/Throttled -> Retrying: typed dt-sensitive failure after certified rollback.
- Retrying -> Healthy/Throttled: successful smaller step; commit actual dt and reset consecutive failures.
- Retrying -> Blocked: retry/floor limit, unsupported regime or recoverable world condition; keep service and last committed state alive.
- Blocked -> Healthy/Retrying: relevant successful world mutation, a changed policy/bound, or a bounded transient-worker retry; recompute all bounds. Do not spin at identical inputs.
- Any -> Fatal only for lost/untrustworthy world, failed rollback or other proven nonrecoverable runtime invariant. Keep previous snapshot readable; expose cause, close channels consistently and settle accepted requests.

A valid positive step can be slow. A smaller dt does not fix invalid mass, singular gravity or unsupported aerodynamics. Never repair such errors by zeroing forces, changing mass, clamping velocities, disabling constraints or ignoring a contact. Do not auto-delete bodies. Island quarantine/checkpoint restoration is out of scope unless a separate change establishes correct isolation including joints, contacts, clusters and external loads.

## 4. Step selection and recovery algorithm

Reuse choose_step; do not introduce a second physical integrator. Refactor its lower-bound handling so the estimator exposes the physical bound separately from the preferred minimum. Resilient mode can choose below the preferred minimum down to the configured hard floor. The floor is not permission to round a smaller physical bound upwards. If even the hard floor is unsafe or no positive Duration can represent the interval, enter Blocked with a typed reason.

The horizon is an absolute simulation-time cap and must never be exceeded by a new attempt. A positive remainder smaller than the preferred minimum may be consumed in resilient adaptive mode if representable, >= hard floor, and physically safe. Otherwise expose the remainder and wait; do not round up or fabricate a final commit. Preserve existing fixed/legacy behavior. In-flight work is not retroactively shortened by a horizon change (document acknowledgement boundary).

At each scheduling turn:
1. Observe lifecycle controls, horizon, requested multiplier. Process a bounded FIFO set of commands when the world is available. Validate atomic batches and retain their outcomes; rejected commands do not poison the world. Queries run once at their FIFO position.
2. Recompute physical bounds **after** accepted mutations (forces, impulses, mass, gravity, colliders, joints, etc.), not from the previous snapshot. Use current authoritative world. Candidate <= admitted debt, horizon remainder, configured maximum and physical estimates. Preserve existing estimator semantics unless an identified defect requires a targeted correction. Estimates are not a global accuracy proof.
3. If no step fits, classify intentional wait vs physical limitation. Keep control commands serviceable and avoid a busy loop.
4. Apply each command once; execute candidate in core. All step attempts start from the same post-command boundary until a commit or corrective mutation. Do not reapply queued impulses, forces or structural changes on retries.
5. On success, advance authoritative time by actual successful dt, subtract only that dt from accumulator, publish snapshot then committed events/results as described below. A shorter success does not consume the failed interval; the remainder stays admitted demand.
6. On a classified dt-sensitive failure, require proven rollback. Reduce next dt by half (floor toward zero), cap by fresh physical/horizon bounds, require strict reduction. Try at most max_retries additional attempts; attempts count toward the per-turn work budget. Preserve retry context across scheduling turns (do not reset the budget just because the loop yielded). Do not retry unknown errors by default.
7. On persistent failure enter Blocked, settle the applied-command results according to section 6, suspend accrual of additional demand and wait for a meaningful recovery trigger. No retry loop at unchanged inputs. A verified transient executor fault can retry with the existing executor or sequential fallback only when ownership and rollback are proven; bound retries and backoff. If ownership is lost, Fatal.
8. After success allow the retry ceiling to grow gradually (e.g. at most 2x successful dt on the next candidate); still obey all estimates. Avoid immediately retrying the same known failing large interval indefinitely. No wall-time dependence in numerical retry selection.

Classify concrete typed errors using their source/downcast chain, not Display strings or broad ErrorKind. Inspect StepError and nested CollisionError, IntegrationError, gravity/aerodynamics/joint errors. Start with the narrow proven dt-sensitive cases (e.g. CCD nonconvergence where rollback is certified); for each actual variant record rationale and a deterministic test. Arithmetic overflow is retryable only where it is demonstrably caused by dt. Invalid input/configuration, missing handles, invalid shapes, singular fields and unsupported models require corrective input or explicit failure. Existing ErrorPolicy::Strict and CcdLimitPolicy semantics remain intact: no silent approximate model or CCD clipping beyond explicitly configured policy.

## 5. CPU overload and debt

Separate physical dt from wall-clock pacing. Reuse AdaptiveScheduler and existing worker observations. A slow step is successful if the core committed it; it is not a numerical error. The physical dt may grow only within the estimator's admissible bound; otherwise lower achieved warp. No guaranteed minimum positive wall-clock speed. The old MIN_LOAD_SPEED floor must not constrain resilient demand admission; report the actual admission/load factor and its semantics.

Use bounded saturating/checked accumulator admission, capped by max_simulation_debt and horizon. Count discarded requested demand separately (simulated seconds not admitted); do not deduct it from already committed time. Pause, horizon hold and Blocked add no new demand. Preserve the already admitted remainder within the cap, clamp it on horizon reduction and report any discarded demand. Requested warp is never silently overwritten. Avoid accumulator overflow even for huge finite multipliers and large wall elapsed durations; reject unrepresentable configuration or safely saturate admission and diagnostics with an explicit saturation indication.

Per service turn bound total attempts (successes + retries), commands and deferred results; yield between turns, check stop/pause/horizon, and service commands fairly. Wall-clock measurements may affect admission and worker count only, not the retry decision or physics model. A slow consumer may lag bounded event queues but cannot stall the solver; surface existing lag diagnostics.

## 6. Transaction boundaries and publication

Define two distinct boundaries: (A) atomic command application (existing contract), (B) atomic core integration attempt. Do not roll back a previously acknowledged command merely because a later step fails. At B, verify rollback of body pose/velocity/sleep/forces, persistent force-source and gravity/aerodynamic state, joint and cluster topology, contact caches/warm starts, spatial proxies, events, impulses, diagnostics and staged results. Scratch may change if it cannot affect subsequent results. Test failures after force preparation, during prediction/collision and before final commit. Prefer existing staging and reusable scratch; no full-world clone each step. If a path cannot restore a trustworthy world, classify it Fatal; do not retry it.

On recoverable attempt failure, do not publish a new step snapshot, events or success acknowledgement. Keep accepted command outcomes until the next successful step, or settle them once on Blocked/Fatal/stop. Never replay commands. Existing StepFailedAfterApply / failed_step() semantics identify a command that mutated the world but whose following step did not commit; do not turn this into a rejection or instruct callers to resubmit. Preserve original typed cause. A failed atomic batch remains rejected, not applied. One-shot force must survive a failed attempt exactly once until a successful step consumes it; a persistent ForceSource applies every committed interval; an impulse applies exactly once at command application. Structural IDs remain valid if application succeeded.

Bound pending/deferred results independently from mpsc queue; when full, settle eligible outcomes or apply backpressure without starving stop or corrective commands. In Blocked, intentional pause and horizon hold, continue servicing commands and queries via the existing no-motion path: publish a coherent command-only snapshot at the same physical time/step and acknowledge successful corrective mutations; distinguish such publication from an integration commit (add a monotonic publication revision if required). No new collision/joint-break events for a command-only publication unless existing command semantics explicitly require and identify them. Preserve old behavior in FailFast. A query observes its FIFO command-boundary state, not a future snapshot; deliver it once, even if a later integration fails. Document all result states and cancellation behavior.

Snapshot time is authoritative and monotonic, step is the number of successful integrations, publication revision distinguishes same-time changes. Maintain existing immutable Arc snapshot and event ordering: snapshot before successful step acknowledgements/events. Avoid unbounded per-subscriber state. Consumers that require every physical event must handle explicit lag, not infer events from snapshot count.

## 7. Host integration contract

Provide a runnable example with dedicated Tokio runtime and explicit CPU pool budget, bounded nonblocking command submission from a mock game loop, snapshot reads without awaiting physics, and clean shutdown outside the render loop. Use existing APIs and the new policy. Show persistent loads rather than assuming one-shot forces last a host frame. Demonstrate absolute time_horizon coordination, command-only setup at zero horizon, safe extension after environment changes and in-flight horizon semantics. Document that host synchronization requiring exact future boundaries must install the horizon before advancement, wait for acknowledged publications on a non-render task, then apply environment/structural changes before extending. Do not introduce application-specific hooks, terrain or rendering logic.

The eventual host must not block on physics results and must use committed simulation time, not wall time, for gameplay accounting. Kapitsa does not guarantee real-time frame deadlines under arbitrary OS starvation; it guarantees no direct dependence of snapshot reads on step completion. External geometry availability and certified fallback belong to the host, not this change.

## 8. Compatibility, scope and evidence

No unrelated solver rewrite, new third-party dependency, whole-world checkpoint system, general island splitting, or application changes. Preserve core error sources, feature gates and existing synchronous exact-step behavior. Keep default service behavior and test it. Prefer small cohesive functions over expanding the service loop further. Follow AGENTS.md: no new unwrap/expect; explicit checked arithmetic; no new panic paths; independent atomic commits; no push/merge unless requested.

Add deterministic test-only failure injection at the smallest existing step/worker seam, not a production switch. Use gates, manual Tokio clock and channels rather than flaky sleeps. Compare retry results with a reference world executing the same successful dt sequence and commands, including contacts, impulses and event order; document tolerance rationale where exact equality is not expected. Run real collision and joint fixtures as well as injected failure paths. Measure before/after release workloads, using equal committed simulated time and inputs, report overhead, progress, debt, snapshot read latency and allocations; no invented numerical thresholds or claims of physical validation. If a criterion cannot be met, leave it open and report evidence; do not weaken acceptance criteria to archive.
