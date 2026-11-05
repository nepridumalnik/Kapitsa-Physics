# Non-fixed joint solver final review

Date: 2026-11-05

## Outcome

The non-fixed joint solver work is complete through stages 6–8. The rotating
frame Jacobian defect was corrected without changing the FixedJoint
condensation path, CCD implementation, iteration budgets, damping, or
compliance defaults.

## Stage 6 — stability

Commit: `a79a321 fix(joint): stabilize rotating-axis non-fixed constraints`

Frame-derived linear axes now carry an explicit axis anchor. The velocity
Jacobian includes the derivative caused by body A rotating the axis, while the
position projection retains its frozen-axis Jacobian to preserve projection
convergence. `ConstraintRow` stores the separate position angular term.

Added coverage includes a finite-difference rotating-axis regression and graph
stability scenarios for hinge, slider, cylindrical, planar, D6, ball, driven,
closed-loop, mixed, chain, and large diagnostic graphs.

Observed regression values:

| Scenario | Result |
|---|---:|
| Rotating slider FD residual | below `1e-9` |
| Slider chain position error | `4.496e-4 m` |
| Slider chain velocity residual | `2.271e-2 m/s` |
| Slider chain angular residual | `7.342e-5 rad/s` |
| Free ball tree linear momentum drift | below `1e-9` |

The free-floating articulated tree does not assert exact angular momentum or
kinetic energy conservation: the existing position-projection architecture is
not an exactly angular-momentum-preserving or energy-preserving operation.
Measured angular-momentum drift was `1.5366e-3`; kinetic energy changed by
approximately `69.7%` in that diagnostic. These remain reported diagnostics,
not hidden failures.

## Stage 7 — collision and CCD integration

Commit: `b256978 test(collision): verify non-fixed joint CCD integration`

No collision implementation change was needed. The collision suite passed:

* 171 core collision/CCD tests;
* continuous sphere versus thin wall integration test;
* worker-count CCD equivalence and rebound/resweep coverage;
* expected CCD collision response remained intact.

The stage checkpoint is intentionally empty because the required behavior was
already present and was verified without touching unrelated collision code.

## Stage 8 — final verification

Workspace tests passed:

* `cargo fmt --all -- --check`
* `cargo test --workspace`
* 721 unit/integration tests, 2 pre-existing ignored tests, and documentation
  tests completed successfully.

The requested strict command
`cargo clippy --workspace --all-targets --all-features -- -D warnings` was run
and did not pass. It reported 20 warnings-as-errors, including pre-existing
style issues in `world/cluster.rs`, collision test helper signatures, and
solver tests, plus new nested-condition diagnostics in the stability metrics.
No broad `allow` was added to conceal these findings. This limitation is
explicitly retained rather than claiming a clean Clippy result.

## Git state

All requested work is committed locally and was not pushed. The final stage
report is committed separately after this review. No dependencies were added,
and no destructive repository operation was performed.
