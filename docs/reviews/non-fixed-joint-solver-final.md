# Orbital angular-momentum drift ? architectural stop

Date: 2026-11-06. Branch: `feat/collisions`. Starting HEAD: `d3bc1f6`.
**The orbital defect is reproduced, not fixed.** This uses the request's explicit
stop condition for a required integration redesign; no experimental solver is retained.

## 1. Established cause

Production executes free prediction ? velocity solve ? position solve ? commit.
Scalar/cached, adaptive, colored/partial-colored and PCG paths retain that order.
FixedBlock3 refreshes Anchor ? Frame ? Anchor but still corrects coordinates last;
Ball falls back to scalar rows. Reduced-fixed-tree is a different aggregate model,
restricted to fixed trees. The standalone joint test helper uses the reverse
projection order. The old pair stage test followed that helper; its replacement
checks the actual production order and matches `world.step` metrics.

After the spin fix, projection retains linear and world spin momentum, so
`delta_L = sum(delta_COM cross p)`. Reordering two endpoint projection phases does
not remove that term: a momentum-preserving velocity solve cannot cancel it at
fixed poses. The new regression reproduces it from exact initial anchors, both
compatible and incompatible velocities, unequal masses, offset COMs and rotated
anisotropic inertia; reference/cached results agree to roundoff.

An algebraic COM-only Ball counterexample has unit masses, COMs ?0.5X, velocities
?Y, zero spin, P=0, L=-Z and E=1 J. Position projection makes COMs coincide while
retaining velocities/spin: L=0, dP=dE=0, |dL|=1. At coincident COMs, P=0 and zero
spin imply L=0, so this pose-only map cannot also preserve the input L. These are
deliberately inconsistent coordinates, not conservative initial-value dynamics;
the separate compatible-pair test establishes the defect for valid initial states.

## 2. Required discrete model and justification

Choose segment-level constrained integration: retain `(q_n,p_n,h_n,dt_segment)`,
solve a constraint half-kick with the drift to `C(q_next)=0`, then an endpoint
velocity constraint with a **separate** multiplier (RATTLE/variational split).
For translation, `delta_COM=dt*p_half/m` implies `delta_COM cross p_half=0`.
Rotational drift must carry world spin momentum using a consistent discrete
rigid-body model. At each kick, equal/opposite forces at one world anchor and
opposite couples give `sum(COM cross delta_p + delta_h)=0`. No compensating torque
is needed. Finite residuals, rotational energy and convergence order still require
implementation validation; exact energy conservation is not claimed.

Required architecture: move this integration boundary above `SolverWorkspace`
into world/CCD segment prediction. Current projection sees endpoint bodies after
collision candidates and free-prediction TOI trajectories already exist. The
constrained predictor must supply both sampled trajectories and conservative
candidate bounds; changed active contacts require bounds/TOI revalidation before
commit. Keep remaining-segment force duration and transactional rollback intact.

Scalar, FixedBlock3, coloring and PCG must solve the same discrete residual.
`position_angular_a` can remain a descent/preconditioner direction, not a physical
kick Jacobian. Compliance needs its constitutive multiplier equation; unilateral
limits/contact need stage-consistent complementarity; damping/drives need work
terms. Preserve XPBD multiplier units, reaction accounting and mass-cache
invalidation without double application. This replaces the integration contract
across prediction, constraints and CCD, beyond a local Ball projection patch.
It is not implemented in this change.

## 3. Solver changes

None. Production Rust is unchanged from `d3bc1f6`. Only regression tests changed:
correct production phase ordering, actual anchor/velocity measurements after each
stage, matched initial geometry for the two velocity cases, reference/cached
parity, comparison with committed production stepping, and the COM-only proof.
These are explicit defect-reproduction tests, not conservation acceptance tests.
No hidden skip, iteration/tolerance change, dependency, or hot-path allocation
was introduced. The rejected multiplier/dt experiment was not repeated.

## 4. Measurements

One production step, dt=1/120. dP/dL are absolute norms, dE is signed joules.
The first state has position error <1e-14 and velocity residual <1e-14 before stepping.
The second has the same exact initial anchors and independent velocities.
Values are unchanged before/after this test-only change; there is no fixed result.

| Case / stage | dP | dL | dE | Anchor error m | Velocity residual m/s |
|---|---:|---:|---:|---:|---:|
| Compatible / integration | 0 | 2.3301e-13 | 3.6568e-15 | 2.3481e-7 | 5.6352e-5 |
| Compatible / velocity | 6.2063e-17 | 2.3300e-13 | -4.7348e-10 | 2.3481e-7 | 1.3836e-17 |
| Compatible / position | 6.2063e-17 | 3.7926e-9 | -4.2419e-10 | 3.3512e-17 | 1.5881e-8 |
| Incompatible / integration | 0 | 3.3113e-13 | -6.0230e-15 | 2.4106e-3 | 2.8926e-1 |
| Incompatible / velocity | 1.1189e-16 | 3.3114e-13 | -1.2338e-2 | 2.4106e-3 | 1.0250e-17 |
| Incompatible / position | 1.1189e-16 | 5.6199e-5 | -1.2337e-2 | 1.0675e-14 | 1.4152e-4 |

Compatible trees over the same five seconds (300/600/1200 steps):

| Bodies | dt | dP | dL | Relative dE |
|---:|---:|---:|---:|---:|
| 10 | 1/60 | 1.5263e-15 | 5.7514e-4 | -1.8397e-4 |
| 10 | 1/120 | 1.7936e-15 | 2.8856e-4 | -9.2092e-5 |
| 10 | 1/240 | 4.0003e-15 | 1.4453e-4 | -4.6073e-5 |
| 31 | 1/60 | 1.8310e-15 | 3.6172e-3 | -3.5036e-4 |
| 31 | 1/120 | 1.1270e-14 | 1.8242e-3 | -1.7572e-4 |
| 31 | 1/240 | 6.6168e-15 | 9.1600e-4 | -8.7997e-5 |

Both energy error and angular-momentum drift are approximately first order here;
this is a measured baseline, not exact conservation or an improvement.

## 5. FixedJoint / Hinge / CCD

PASS in the full all-feature run: Hinge energy, FixedJoint order/coupling and
scalar/in-place Block3 regressions; all joint-type tests; all 28 collision
geometry/integration/query tests; unchanged resumed-CCD force test (`1.3 m/s`).
Compatible free-system dt-convergence and finite-state assertions passed.

## 6. Clippy and tests

PASS: `cargo fmt --all -- --check`, `cargo check --workspace --all-features`,
`cargo clippy --workspace --all-targets --all-features -- -D warnings`,
`git diff --check`.

`cargo test --workspace --all-features --no-fail-fast`: 972 unit/integration tests
and 2 doctests passed; 7 known stress failures; 46 pre-existing ignored. Core alone:
769 passed, 7 failed, 12 ignored. All other targets passed. No new failures.
The seven failures remain Hinge/Slider/Mixed at 100 and 1000 bodies and closed loop
at 1000 bodies. Their tests and budgets were not changed.

## 7. Remaining limitations

Orbital drift remains. No production implementation or acceptance claim is made
for the proposed constrained integrator. The seven known stress-convergence failures
are a separate baseline and have not been modified. No GUI/manual checks were run.

## 8. Local commits and HEAD

- `3866447 test(physics): reproduce orbital momentum drift in projection`.
- The subsequent documentation commit contains this architectural decision and
  verified results; its hash is the final HEAD (`git rev-parse HEAD`).

No push was performed.
