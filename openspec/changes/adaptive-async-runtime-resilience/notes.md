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
(the same initial state succeeds at half the duration), the safe 500 µs bound
returns `Limited`, admission retains 10 s, and a queued velocity commits 1 ms
when the post-command bound is 500 µs. They currently exercise legacy machinery
because the opt-in policy does not exist; as each policy component is implemented,
wire its corresponding probe to that policy and remove its ignore annotation.
The legacy tests and their assertions must continue to pass separately.

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
