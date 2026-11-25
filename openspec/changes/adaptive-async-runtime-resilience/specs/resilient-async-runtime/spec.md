# Spec Delta

## Purpose

An application can run autonomous Kapitsa physics on a separate runtime without waiting for integration on its own thread, and the service advances physically valid time through bounded load shedding and numerical recovery.

## ADDED Requirements

### Requirement: Explicit opt-in and compatibility
The service SHALL expose a validated resilient adaptive recovery policy. Existing fixed/adaptive fail-fast defaults and the one-attempt synchronous PhysicsSystem update contract SHALL remain unchanged. The synchronous feature set SHALL NOT require Tokio.

#### Scenario: Existing callers retain semantics
- **WHEN** a caller uses default service configuration or the synchronous explicit-step API
- **THEN** it retains the documented failure and step-count semantics without implicit retries

#### Scenario: Invalid resilience configuration
- **WHEN** a resilient configuration has zero retries, invalid step floor, incompatible stepping policy or invalid debt bound
- **THEN** construction SHALL reject it before starting or mutating a world

### Requirement: Safe post-command adaptive step selection
The resilient scheduler SHALL choose each attempt from the current authoritative post-command world, bounded by physical estimates, admitted demand, maximum step and absolute time horizon. It SHALL permit safe positive representable steps below the preferred adaptive minimum to a validated hard floor, but SHALL NOT enlarge an unsafe step to that floor.

#### Scenario: A queued mutation changes physical bounds
- **WHEN** an impulse or another accepted mutation changes motion or contact constraints
- **THEN** the immediately following integration attempt SHALL use recomputed bounds including that mutation

#### Scenario: A safe step is below the preferred minimum
- **WHEN** the physical bound is below the preferred minimum but at or above the hard floor
- **THEN** the resilient service SHALL integrate a positive step no greater than that bound when demand and horizon allow

#### Scenario: No representable safe step
- **WHEN** the physical bound is smaller than the hard floor or no positive representable safe duration fits
- **THEN** no invalid step or fictitious time SHALL commit
- **AND** a typed limitation SHALL be reported while the service remains responsive

#### Scenario: A short horizon remainder
- **WHEN** a newly scheduled adaptive attempt would exceed the absolute horizon
- **THEN** its duration SHALL fit the remaining horizon or it SHALL wait
- **AND** a horizon change SHALL NOT retroactively invalidate an in-flight interval

### Requirement: Bounded typed recovery
The resilient service SHALL classify errors by typed cause, retry only failures with a trustworthy rollback and a justified recovery strategy, and bound total attempts per scheduling turn and retry episode. A dt-sensitive failure SHALL reduce the attempted duration strictly; unchanged failed inputs SHALL NOT be retried indefinitely.

#### Scenario: Transient numerical rejection
- **WHEN** a classified dt-sensitive attempt fails and rollback succeeds
- **THEN** a smaller physically admissible attempt SHALL be tried within the recovery budget
- **AND** a later successful attempt SHALL advance only its actual duration without restarting the service

#### Scenario: Persistent failure
- **WHEN** the retry budget or safe step floor is exhausted
- **THEN** the service SHALL enter a diagnosable blocked condition without spinning or accruing new demand
- **AND** relevant corrective input SHALL be accepted and permit fresh evaluation without recreating the world

#### Scenario: Unknown or unrecoverable fault
- **WHEN** a typed cause has no proven numerical recovery or the world cannot be trusted
- **THEN** the service SHALL NOT conceal the cause or retry unsafely
- **AND** it SHALL distinguish a corrective-input block from a fatal loss of world ownership or rollback integrity

#### Scenario: Executor failure
- **WHEN** a worker fails and the world is proven intact
- **THEN** any recovery SHALL be bounded and preserve the configured physical policy
- **AND** if ownership or rollback is not proven, the service SHALL report a fatal fault rather than continue with uncertain state

### Requirement: Atomic integration attempts
A rejected integration SHALL preserve all previously committed physical state and the post-command boundary state, including one-shot loads awaiting consumption, and SHALL NOT publish attempted collision or joint events, contact impulses or committed diagnostics. Recovery SHALL preserve core strictness and configured CCD policies.

#### Scenario: Failure after preparation or collision work
- **WHEN** an attempt fails after applying prepared loads or staging collision/contact state
- **THEN** subsequent successful execution SHALL match an equivalent reference world with the same accepted commands and successful dt sequence
- **AND** no failed interval SHALL count as committed time or a committed step

#### Scenario: A ready aerodynamic shape precedes a failed step
- **WHEN** a pending cook completes during step preparation and the integration subsequently fails
- **THEN** the previous shape binding and registry state SHALL remain visible
- **AND** the completion notification SHALL be deferred until a successful publication
- **AND** a later successful step SHALL use and publish the prepared shape exactly once

### Requirement: Exactly-once command and result semantics
Accepted commands SHALL be applied at most once across retries. Rejected atomic batches SHALL have no partial mutation. Results SHALL distinguish applied commands awaiting an integration commit from rejected commands, and every accepted request SHALL receive one terminal outcome or existing explicit bounded-stream lag indication. Query results SHALL retain their FIFO observation semantics.

#### Scenario: Impulse and force survive retry correctly
- **WHEN** an applied impulse, one-shot force and persistent force precede a rejected attempt and a successful retry
- **THEN** the impulse SHALL NOT be replayed, the one-shot force SHALL be consumed once on success, and the persistent force SHALL act over each committed interval

#### Scenario: A command was applied but no step can commit
- **WHEN** the runtime blocks, fails fatally or stops after a successful command mutation but before a successful integration
- **THEN** its outcome SHALL accurately identify applied-without-step versus rejection and preserve the original typed cause where applicable
- **AND** the runtime SHALL NOT request replay of a mutation that already occurred

### Requirement: Committed snapshots and responsive control plane
Immutable snapshots SHALL be readable without waiting for an in-flight integration. Committed integration publications SHALL contain coherent time, state, step and diagnostics. A command-only publication SHALL be distinguishable without inventing physical time or a step. Snapshot publication SHALL precede corresponding successful acknowledgements and committed event batches.

#### Scenario: A worker is gated
- **WHEN** a physical attempt has not completed
- **THEN** a consumer SHALL be able to read the last committed snapshot and submit a bounded nonblocking request without waiting for the worker

#### Scenario: Commands while blocked or horizon-limited
- **WHEN** motion cannot advance or is intentionally paused
- **THEN** commands and queries SHALL remain serviceable at available world boundaries
- **AND** successful command-only publications SHALL retain the same committed time/step and expose a distinct revision

#### Scenario: Slow subscriber
- **WHEN** a consumer cannot keep up with bounded event or result streams
- **THEN** it SHALL receive explicit lag diagnostics without blocking integration or causing an unbounded internal backlog

### Requirement: Bounded overload pacing
CPU budget exhaustion SHALL be a scheduling limitation, not a physics error. The resilient runtime SHALL bound admitted simulation debt and deferred work, expose discarded demand and actual achieved warp, and admit less demand as necessary without a nonzero minimum speed guarantee. It SHALL NOT skip an admitted physical interval and count it as simulated.

#### Scenario: Sustained overload
- **WHEN** supported physics is slower than requested wall-clock scaling for a sustained period
- **THEN** positive safe committed steps SHALL continue when resources allow
- **AND** debt and queue growth SHALL remain bounded, achieved speed SHALL reflect committed time, and discarded demand SHALL be observable

#### Scenario: Pause and resume
- **WHEN** the host pauses, reaches a horizon or enters a blocked recovery state
- **THEN** new wall-clock demand SHALL NOT accumulate during the hold
- **AND** on resume only bounded admitted demand SHALL be processed

### Requirement: Explicit runtime ownership and lifecycle
The autonomous service SHALL support separate host-provided Tokio orchestration and bounded Rayon computation through documented owned/shared pool configuration. CPU integration SHALL NOT run on the host render thread. Stop SHALL prevent new attempts, finish or safely settle active work, and resolve pending requests; caller-owned pools SHALL remain usable after service shutdown.

#### Scenario: Host uses a dedicated runtime
- **WHEN** a host schedules physics on a dedicated Tokio runtime with a bounded Rayon pool
- **THEN** it SHALL be able to submit commands and read snapshots independently of step completion
- **AND** shutdown SHALL preserve valid world state and shall not destroy a pool still owned by the caller

### Requirement: Observable recovery and host coordination
The service SHALL report typed recovery status, original causes, attempts/retries, committed time and step, safe/attempted dt, requested/achieved speed, debt and discarded demand. It SHALL document host coordination through absolute horizons and acknowledged boundary commands without application-specific dependencies.

#### Scenario: Corrective command resumes progress
- **WHEN** a blocked service receives a valid corrective mutation and safe advancement becomes possible
- **THEN** committed time SHALL resume increasing and the active fault SHALL clear while historical failure diagnostics remain available

#### Scenario: Host sets up environment at a boundary
- **WHEN** a host starts at a zero horizon, applies environment changes and extends the horizon after their acknowledged publication
- **THEN** advancement SHALL use those changes and report the actual integrated time to the host
