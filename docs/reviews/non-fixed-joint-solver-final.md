# Follow-up: position-projection invariants (2026-11-06)

Both follow-up tasks remain open and separate. Large-graph convergence has not
been changed during this invariant investigation. The previous checkpoint report
below is historical; its tables are not measurements of the new spin fix.

Commit `bb8b80b` fixes a reproduced orientation-projection defect: the code changed
world inertia while retaining angular velocity. Projection now carries the body's
world spin momentum into the new inertia. Reference and cached paths share the
update; the cached path rotates its existing inverse inertia without another
factorization. COM translation, linear velocity, constraint tolerances, iteration
counts, damping, compliance, and the public `set_transform` behavior are unchanged.
No world-level momentum or energy correction was added.

The new anisotropic-body regression failed before the fix with spin drift
`9.401422370e-3 kg m^2/s`; it now requires drift below `1e-13`. Another regression
isolates pure-translation orbital drift without inventing spin to compensate it.
The pair stage test now verifies that all remaining position-projection drift is
explained by `sum(delta_COM cross linear_momentum)` to `1e-12`.

| Measurement | Before spin fix | After spin fix |
|---|---:|---:|
| Consistent pair, projection dL on first step | 4.8198e-9 | 3.7927e-9 |
| Consistent pair, total dL after 300 steps | 1.3680e-6 | 1.0807e-6 |
| Inconsistent pair, projection dL on first step | 8.7704e-5 | 7.5822e-5 |
| Consistent tree 31, total dL after 300 steps at dt=1/60 | 3.6467e-3 | 3.6172e-3 |
| Same tree, relative energy change | -3.5231e-4 | -3.5036e-4 |

The orbital source remains unresolved. A trial coupling position multipliers to
velocity impulses conserved pair angular momentum to integrator accuracy but
regressed the existing Hinge energy test (`6.1091 -> 8.6907 J`), FixedJoint
trajectory, and timestep-convergence check. That experiment was removed; no test
threshold was relaxed to accept it. A joint configuration correction cannot simply
be treated as an additional velocity impulse in the present solver schedule.

Verification of the retained code:

- PASS: 69 rigid-body tests, including unchanged CCD final velocity `1.3 m/s`.
- PASS: 80 joint solver tests; 2 existing ignored.
- PASS: consistent free-system energy bounds and fixed-duration dt convergence.
- PASS: pair stage regression, including the new orbital-source assertion.
- PASS: strict workspace/all-targets/all-features Clippy and formatting.
- Core all-feature `--no-fail-fast`: 767 passed, 7 existing stress failures,
  12 existing ignored; all 28 collision integration/query/geometry tests and
  2 doctests passed. This run includes both new body-level regressions.
- Full workspace tests were not repeated for this partial invariant stage.

**Acceptance is not reached.** The orientation component is fixed, the orbital
component still drifts, and the independent large-graph task has not started.

---

# Non-fixed joint validation ? incomplete

Date: 2026-11-06. Source checkpoint: `67053dc` on `feat/collisions`.
The final HEAD is the report commit containing this file (`git rev-parse HEAD`).
No push was performed.

## Confirmed changes

- All kinetic-energy diagnostics now use the same COM translational energy and
  local anisotropic inertia expression, in joules.
- Removed the environment-gated successful no-op and temporary print-only probes.
  Ten independent mandatory tests exercise all five graph types at 100/1000 bodies;
  a failure in one graph no longer prevents the other graphs from running.
- Added exact Ball anchors under rotated body frames and a rigid velocity field
  computed from world COMs. Initial position/velocity residuals must be below
  `1e-14` before any solver step. No warm-up projection is used.
- Independent-velocity initial states are tested separately: forbidden velocity
  decreases, linear momentum is conserved, energy decreases and cannot grow.
- Fixed a reproduced Ball velocity-impulse torque leak: equal/opposite impulses
  now act at the same world point. Position gradients retain their original
  anchor geometry. The stage regression failed before this change and passes
  after it. This does **not** fix the dominant position-projection drift.
- Strict Clippy diagnostics are resolved without new lint allowances. The rocket
  stability test module moved out of the fixture shared with a harness-free bench;
  it is still compiled and executed by library tests.

## Energy and momentum

The old independent-velocity tree starts outside the velocity constraint manifold,
so the initial constraint projection removes incompatible relative motion. Its
roughly 69.7% energy loss is not a measurement of conservative integration error.
The old fixture also has small initial anchor errors. With the present velocity
fix, that fixture changes from `2.854678111 J` to `0.8641052552 J` over 300 steps;
its remaining angular-momentum drift is `2.925865912e-3 kg m^2/s`.

Isolated two-body stages (dt=1/120, first inconsistent step):

| Stage | Absolute angular-momentum change before | After velocity fix |
|---|---:|---:|
| Integration | 3.311e-13 | 3.311e-13 |
| Position projection | 8.770e-5 | 8.770e-5 |
| Velocity solve | 7.187e-7 | 6.360e-17 |

Position projection is the dominant confirmed source: it changes COMs and
orientations while preserving velocities, thereby changing orbital and anisotropic
spin angular momentum. The free-body integrator error is much smaller. No global
momentum restoration, energy rescaling, iteration increase, damping change, or
compliance change was introduced. A conservative position-projection correction
remains unimplemented; exact articulated angular-momentum conservation is **not**
claimed.

Consistent systems, same five-second duration (300/600/1200 steps). Diagnostics also
record the first and 300th steps. P/L include COM orbital momentum, not only spin.
Energy error must remain below 0.1% and decrease by at least 40% at each halving of
dt for this deterministic fixture. This is a bounded-error and convergence check,
not a claim of exact energy conservation.

| Bodies | dt | E initial J | E final J | Relative dE | Absolute dP | Absolute dL | Relative dL | Max anchor m | Velocity residual m/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 0.016667 | 2.446544432e-2 | 2.446544432e-2 | 1.410165e-10 | 0.000000e0 | 2.226105e-10 | 3.623686e-9 | 0.000000e0 | 0.000000e0 |
| 1 | 0.008333 | 2.446544432e-2 | 2.446544432e-2 | 3.525413e-11 | 0.000000e0 | 5.560700e-11 | 9.051784e-10 | 0.000000e0 | 0.000000e0 |
| 1 | 0.004167 | 2.446544432e-2 | 2.446544432e-2 | 8.813781e-12 | 0.000000e0 | 1.389610e-11 | 2.262026e-10 | 0.000000e0 | 0.000000e0 |
| 2 | 0.016667 | 5.399634695e-2 | 5.399579487e-2 | -1.022436e-5 | 6.168985e-16 | 5.135800e-6 | 2.167315e-5 | 4.637178e-9 | 6.357579e-8 |
| 2 | 0.008333 | 5.399634695e-2 | 5.399607089e-2 | -5.112599e-6 | 2.185476e-16 | 2.568172e-6 | 1.083772e-5 | 1.159285e-9 | 1.589252e-8 |
| 2 | 0.004167 | 5.399634695e-2 | 5.399620891e-2 | -2.556405e-6 | 5.861620e-16 | 1.284154e-6 | 5.419149e-6 | 2.898203e-10 | 3.972952e-9 |
| 10 | 0.016667 | 3.451085846e-1 | 3.450440731e-1 | -1.869310e-4 | 2.673771e-15 | 5.868601e-4 | 1.763620e-4 | 7.243316e-6 | 1.008885e-4 |
| 10 | 0.008333 | 3.451085846e-1 | 3.450762887e-1 | -9.358165e-5 | 2.256240e-15 | 2.944529e-4 | 8.848837e-5 | 1.814235e-6 | 5.046930e-5 |
| 10 | 0.004167 | 3.451085846e-1 | 3.450924266e-1 | -4.681988e-5 | 3.512588e-15 | 1.474828e-4 | 4.432122e-5 | 4.539811e-7 | 2.524093e-5 |
| 31 | 0.016667 | 1.151448822e0 | 1.151043129e0 | -3.523322e-4 | 4.950151e-15 | 3.646776e-3 | 2.311088e-4 | 4.108018e-5 | 2.896326e-4 |
| 31 | 0.008333 | 1.151448822e0 | 1.151245330e0 | -1.767267e-4 | 6.665038e-15 | 1.839142e-3 | 1.165528e-4 | 1.034304e-5 | 1.450545e-4 |
| 31 | 0.004167 | 1.151448822e0 | 1.151346915e0 | -8.850359e-5 | 1.251355e-14 | 9.235379e-4 | 5.852779e-5 | 2.594874e-6 | 7.258685e-5 |

For the 31-body consistent fixture, initial vectors are
`P=(6.672600642460148,-2.681917700466424,-1.2924131038182551)` and
`L=(2.5145953989186545,8.941927683135344,-12.755806882036847)`.
At dt=1/240 after five seconds,
`P=(6.672600642460138,-2.681917700466418,-1.292413103818252)` and
`L=(2.5140343948871515,8.94120857047488,-12.75595204983383)`.
The dt-halving trend is approximately first order for articulated energy error;
the measured free-body energy error decreases approximately quadratically.

## Mandatory large graphs

Command: `cargo test -p kapitsa-core --release articulated_graph_ -- --nocapture`.
All ten scenarios executed; three passed, seven failed. Existing budgets remain
5e-3 m, 5e-3 rad, 5e-2 m/s, 5e-3 rad/s. All measured states remained finite.

| Graph | Bodies | Max anchor m | Linear residual m/s | Result |
|---|---:|---:|---:|---|
| Hinge | 100 | 1.477e-2 | 8.899e-3 | FAIL |
| Hinge | 1000 | 1.525e-2 | 9.075e-3 | FAIL |
| Slider | 100 | 1.047e-2 | 1.039e-1 | FAIL |
| Slider | 1000 | 1.047e-2 | 1.039e-1 | FAIL |
| Ball | 100 | 1.005e-3 | 8.391e-3 | PASS |
| Ball | 1000 | 1.254e-3 | 5.339e-4 | PASS |
| Mixed | 100 | 1.069e-2 | 7.753e-3 | FAIL |
| Mixed | 1000 | 1.092e-2 | 7.821e-3 | FAIL |
| Closed loop | 100 | 1.375e-3 | 1.500e-3 | PASS |
| Closed loop | 1000 | 6.089e-3 | 1.731e-3 | FAIL |

These formerly gated failures are now visible in ordinary workspace testing.
Their solver convergence has not been repaired.

## Verification

- PASS: formatting, workspace all-feature check, strict workspace/all-targets/
  all-features Clippy, and `git diff --check`.
- PASS: 15 targeted Ball unit tests and the matching Ball integration regression.
- PASS: 28 collision geometry/integration/query tests, plus the unchanged resumed
  CCD force regression with final velocity `1.3 m/s`.
- PASS: consistent dt-convergence and per-stage momentum tests; repeated targeted
  runs reproduce the same measurements. Large-graph release/debug results agree.
- FAIL: `cargo test --workspace`: core has 718 passed, 7 failed, 2 existing ignored.
- FAIL: `cargo test --workspace --all-features`: core has 765 passed, 7 failed,
  12 existing ignored. Both required commands stop at the failing core target.
- Final `cargo test --workspace --all-features --no-fail-fast`: 968 unit/integration
  tests and 2 doctests passed; 7 stress tests failed; 46 pre-existing ignored.
  Every remaining target passed. No GUI/manual validation was performed.
- All non-fixed joint type regressions in the executed core binaries passed;
  only the seven named stress scenarios failed. No new ignored tests or hidden
  environment skips were added. Collision pipeline code was not changed.

## Local commits

- `01831fd`: correct diagnostics and remove environment gates/probes.
- `f556ce5`: consistent initial states and separate inelastic projection test.
- `0906fa3`: isolate momentum drift by integration/projection/velocity stages.
- `f1c4d7a`: resolve Clippy diagnostics and separate shared fixture from unit tests.
- `82cd4d2`: conserve Ball velocity-impulse angular momentum.
- `67053dc`: independent mandatory stress tests and fixed-duration dt convergence.

Production-ready acceptance is **not achieved**: conservative position projection
and seven large-graph convergence failures remain unresolved.
