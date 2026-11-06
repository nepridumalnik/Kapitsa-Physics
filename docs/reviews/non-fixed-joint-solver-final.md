# Numerical correctness: convergence and orbital momentum

Date: 2026-11-06. Branch: `feat/collisions`. Starting HEAD: `253b161`.

**Status: partially resolved.** All ten requested articulated stress cases pass with their existing criteria. Production orbital angular-momentum drift remains; the test-only constrained integration reference below is not an engine fix. No push was performed.

## Convergence diagnosis and changes

The seven failures share slow propagation of coupled constraints through the ordered Gauss–Seidel sweep. The maximum Hinge and Slider errors are similar at 100 and 1000 bodies because the worst residual is repeatedly recreated by neighboring joints. On the Hinge-100 snapshot, solving the worst joint alone reduced its position error from 0.014768 m to 9.47e-6 m; during another ordered sweep the largest error immediately after processing a joint was 1.106e-4 m, but later joint updates raised the final maximum to 1.475e-2 m. Slider-100 showed the same effect: 0.010468 m to 2.247e-4 m in isolation, while the ordered sweep ended at 1.047e-2 m. The measured Slider velocity residual 0.103872 m/s fell to 0.001429 m/s when that joint was isolated. Mixed graphs and closed loops inherit the coupled residual rather than requiring a separate joint-type correction.

A coupled hard-constraint velocity solve now factors the current row system for eligible contact-free paths and cycles. It uses the existing mass and Jacobian data, validates the applied residual and rejects energy increases for stationary boundaries, and restores state before falling back when it cannot safely solve. The solver does not increase the configured pass count. Branching graphs, contacts, compliant or damped constraints, motors, unilateral rows, warm-started cases, FixedBlock3, coloring and PCG retain their existing paths. Thus the stress results below describe the tested default path, not a claim that all alternative solver modes now have the same large-graph convergence.

A separate Spring defect was found: zero force at the rest length was mistaken for a hard constraint, and damping could be applied once per solver pass. An explicit internal force-driven flag preserves the spring law even when its instantaneous force is zero. The existing damping test expectation was corrected to the implicit one-step result, `10/(1 + 0.01*100*2) = 10/3`, with a 1e-10 numerical tolerance; its previous expectation represented repeated application of damping. This is a corrected physical expectation, not an unchanged test assertion.

## Stress results

Maximum position error in metres, unchanged 5e-3 m budget:

| Graph | Bodies | Before | After |
|---|---:|---:|---:|
| Hinge chain | 100 | 0.014768 | 0.0001050 |
| Hinge chain | 1000 | 0.015248 | 0.0001019 |
| Slider chain | 100 | 0.010468 | 0.0001956 |
| Slider chain | 1000 | 0.010474 | 0.0002000 |
| Mixed | 100 | 0.010693 | 0.00008632 |
| Mixed | 1000 | 0.010920 | 0.00008402 |
| Closed loop | 1000 | 0.006089 | 0.00003955 |
| Ball tree | 100 | passed | 0.001005 |
| Ball tree | 1000 | passed | 0.001254 |
| Closed loop | 100 | 0.001375 | 0.00002494 |

Slider maximum velocity residual decreased from approximately 0.1039 m/s to 8.594e-6 m/s for 100 bodies and 8.562e-6 m/s for 1000 bodies, below the unchanged 0.05 m/s budget.

A test-only exact coupled position solve reduced the Hinge-100 snapshot error from 0.014768 m to 0.005959 m but changed angular momentum by 3.576 kg m²/s, explained by the orbital displacement term. It was not adopted. An exact coupled velocity reference on the original Slider snapshot reduced the residual from 0.103872 to 6.28e-14 m/s, with ΔL=3.40e-14 and ΔE=-0.3246 J; the loss is consistent with projecting incompatible velocities.

## Momentum and timestep convergence

Production still predicts freely, solves velocity constraints, corrects positions and commits. Position projection changes COM positions without changing their linear momenta, yielding `ΔL = Σ(ΔCOM × p)` in addition to roundoff. For the original compatible 31-body Ball tree over five seconds, the before and current production measurements are unchanged:

| dt | ΔP | absolute ΔL | relative ΔE |
|---|---:|---:|---:|
| 1/60 | 1.8310e-15 | 3.6172e-3 | -3.5036e-4 |
| 1/120 | 1.1270e-14 | 1.8242e-3 | -1.7572e-4 |
| 1/240 | 6.6168e-15 | 9.1600e-4 | -8.7997e-5 |

Both errors decrease approximately linearly with dt. This is a remaining production defect, not evidence of its correction.

A separate test-only RATTLE-like reference solves a start kick, constrained drift and independent endpoint kick, including world spin momentum and anisotropic rotational inertia. The compatible two-body Ball case over five seconds shows production ΔL of 2.076090e-5, 1.038104e-5 and 5.190666e-6 at dt 1/60, 1/120 and 1/240; reference ΔL is respectively 4.221e-16, 8.092e-17 and 3.047e-16. Production relative ΔE is -3.522211e-5, -1.761236e-5 and -8.806508e-6, versus reference -3.800e-10, -9.501e-11 and -2.375e-11. An incompatible initial velocity loses 8.391871% energy in its initial reference projection while conserving momentum to about 1e-15.

For compatible Ball trees over five seconds, the baseline test reports:

| Bodies | dt | Production ?P | Production ?L | Production relative ?E | Maximum anchor error | Maximum anchor velocity | Reference ?L | Reference relative ?E |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 10 | 1/60 | 2.987305e-15 | 5.751367e-4 | -1.839728e-4 | 5.240113e-6 | 8.136579e-5 | 9.930137e-16 | 1.510479e-9 |
| 10 | 1/120 | 5.885491e-15 | 2.885557e-4 | -9.209237e-5 | 1.312857e-6 | 4.075483e-5 | 2.106500e-15 | 3.776172e-10 |
| 10 | 1/240 | 5.503168e-15 | 1.445253e-4 | -4.607269e-5 | 3.285698e-7 | 2.039579e-5 | 8.393756e-15 | 9.440489e-11 |
| 31 | 1/60 | 5.604152e-15 | 3.617215e-3 | -3.503563e-4 | 2.940516e-5 | 2.509129e-4 | 3.552714e-15 | 4.208043e-10 |
| 31 | 1/120 | 5.777428e-15 | 1.824162e-3 | -1.757225e-4 | 7.403942e-6 | 1.256053e-4 | 6.541843e-15 | 1.051946e-10 |
| 31 | 1/240 | 9.900301e-15 | 9.159976e-4 | -8.799740e-5 | 1.857595e-6 | 6.284487e-5 | 1.005840e-14 | 2.630020e-11 |

Reference energy error decreases approximately fourfold per halving of dt. The reference uses dense finite-difference Newton solves and covers free bilateral Ball systems only; it does not validate production forces, contacts, CCD, motors, compliance or limits.

A safe production integration requires a common segment-level treatment of start state, constrained trajectories, candidate bounds and TOI, remaining-segment forces, rollback and constraint reactions. Solver modes, compliance and unilateral/contact constraints must share consistent residuals. The existing position-multiplier-to-velocity conversion was not repeated. No general production integration change is claimed.

## Verification

- `cargo fmt --all -- --check`, `cargo check --workspace --all-features`, and `cargo clippy --workspace --all-targets --all-features -- -D warnings`: passed.
- `cargo test --workspace --all-features --no-fail-fast`: 995 passed, 0 failed, 46 previously ignored, including doctests. The existing ignored tests were not disabled by this work.
- `cargo test -p kapitsa-core --release articulated_graph_ -- --nocapture`: 11 passed, including all ten requested cases, 0 failed.
- Workspace and focused tests cover FixedJoint, Ball, Hinge, Slider, D6, spring, collision and CCD, including remaining CCD-segment force handling; conservation and timestep comparisons above distinguish dissipative and conservative cases. Cached/reference parity and direct-solver fallback are tested. These regressions do not establish orbital conservation in any production solver mode.
- The existing ignored warm-step allocation check was run explicitly: scalar and block paths both reported zero allocations in its measured warm step.
- `git diff --check`: passed.

## Local commits

- `d124585` isolate articulated convergence defects
- `eccfe26` localize Slider velocity residual
- `47dab52` measure coupled position projection conservation tradeoff
- `7c762ef` compare coupled velocity and position solves
- `36c362f` solve eligible coupled hard-constraint velocities
- `e59e9f5` preserve Spring force law at zero displacement
- `a7ecfd0` validate test-only constrained Ball pair reference
- `d659736` validate test-only constrained Ball tree reference
- The documentation commit containing this report is the final HEAD of this work.
