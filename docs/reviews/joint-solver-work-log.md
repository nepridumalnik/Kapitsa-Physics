# Joint solver work log

## 2026-11-02 — baseline and projection comparison

- `6294fdf` records the local scalar-solver baseline at the original remote HEAD. Workspace fmt/check/Clippy passed; workspace tests passed (775 passed, 4 ignored). The strict Release rocket test failed the original error limits for all sizes.
- The strict fixture completed all 180 steps at 100, 200, 500, and 1,000 parts. It exercised gravity, pad contact, offset thrust, fuel mass changes, and stage separation. Separation momentum error stayed at or below `2.629e-13`; no step failed and no joint broke.
- A Release comparison with identical fixture, 3 position sweeps, and 8 velocity sweeps changed only scalar vs `FixedBlock3`. Both completed at all sizes, but neither met strict limits. Block3 lowered maximum and RMS frame error for 200/500/1,000 parts; both frame errors increased at 100 parts. It did not improve maximum anchor error at any size and increased coast energy drift at 500 parts. Keep Block3 experimental until geometry and pipeline correctness are addressed.

| Parts | Projection | Anchor max / RMS, m | Frame max / RMS, rad | Coast energy drift |
|---:|---|---:|---:|---:|
| 100 | Scalar | 0.1538798 / 0.01302640 | 1.333782 / 0.08493318 | 4.1225% |
| 100 | FixedBlock3 | 0.2213087 / 0.01416238 | 1.442233 / 0.08996116 | 3.2120% |
| 200 | Scalar | 0.2846846 / 0.02980872 | 1.565341 / 0.1226705 | 69.9332% |
| 200 | FixedBlock3 | 0.2883405 / 0.02782598 | 1.201834 / 0.09864404 | 60.8098% |
| 500 | Scalar | 0.6644705 / 0.06259963 | 3.141284 / 0.3265347 | 59.3510% |
| 500 | FixedBlock3 | 0.7414070 / 0.06269059 | 3.140986 / 0.3017271 | 62.0578% |
| 1,000 | Scalar | 2.814815 / 0.1660589 | 3.141479 / 0.6627714 | 44.4683% |
| 1,000 | FixedBlock3 | 2.866444 / 0.1481344 | 3.141102 / 0.4548406 | 44.1231% |

The near-π relative-frame errors and the known small-angle angular Jacobian are the next concrete mathematical check. Add a finite-difference test for the FixedJoint rotation-log derivative, correct the Jacobian if confirmed, then repeat this exact rocket test before changing production defaults.

## 2026-11-02 — angular Jacobian and singular twist regression

- Added the finite-angle transpose inverse SO(3) left-Jacobian to FixedJoint angular rows. Central finite differences pass for all three error coordinates and world axes at `1e-5`, `0.7`, `2.8`, and `π - 1e-5` radians. A separate finite-difference test covers fixed connector-point Jacobians with nonzero local centers of mass and nontrivial local frames.
- `twist_angle` now represents the undefined 180-degree swing coordinate as `None`; constraints that specifically require that twist coordinate are skipped until it is defined. It does not substitute an arbitrary angle or abort the step. Preparation tests cover Ball, Hinge, Cylindrical, Planar, and ConeTwist; a world integration test confirms a BallJoint step remains finite.
- Validation: `fmt --check`, `check -p kapitsa-core`, warning-free `clippy -p kapitsa-core`, all 581 core unit tests, 28 integration/query/geometry tests, 2 doctests, and `git diff --check` passed.
- The strict Release rocket test still fails at every size. The finite-angle Jacobian did not fix the drift; this change is not evidence for the rocket root cause. The report-only and strict runs completed all 180 ticks, repeated deterministically, retained fuel and separation invariants, and had no step failure or joint break.

| Parts | Anchor max / RMS, m | Frame max / RMS, rad | Coast energy drift | Elapsed |
|---:|---:|---:|---:|---:|
| 100 | 0.1559734 / 0.01254084 | 1.358748 / 0.08467397 | 4.1414% | 852 ms |
| 200 | 0.2910659 / 0.02982499 | 1.713914 / 0.1226009 | 70.8814% | 1,724 ms |
| 500 | 0.7676298 / 0.06260248 | 3.139344 / 0.2949535 | 58.3762% | 4,408 ms |
| 1,000 | 3.023810 / 0.1625110 | 3.141562 / 0.6276528 | 45.5216% | 8,836 ms |

These values remain far above the unchanged strict geometry limits. Contacts occurred at every size; the stage-separation momentum error remained at or below `1.990e-13`; fuel masses reached the expected values; and every run recorded zero joint breaks. The math slice is verified, but the cause of the instability remains unproven.

## 2026-11-02 — correction-order and load-source isolation

- Added controlled FixedJoint order measurements at 5°, 15°, 45°, and 90° with two dynamic bodies, unequal masses/inertias, offset centers of mass, and nonzero connector arms. No contacts or external forces are present. Linear-first (A) can increase the frame residual before the stale angular rows run; angular-first (B) can recreate a large frame residual when the refreshed linear rows run. The extra refreshed linear pass (C) gives the smallest one-sweep anchor error, but at 90° it leaves a larger frame residual than A/B. This is a coupled nonlinear constraint; no fixed one-sweep ordering dominates every residual.
- The actual force-free `PhysicsWorld` solver, with its configured repeated position sweeps, reduced all four initial cases below `7.61e-4 m / 6.58e-3 rad` after one step and below `1.56e-5 m / 5.33e-6 rad` after two. The one-joint ordering issue alone does not explain the rocket failure.
- Added chain/branch measurements with varied mass, inertia, and local COM. At step 10, chains of 2, 10, 50, and 100 bodies reached max anchor/frame errors of `(0, 2.39e-20)`, `(1.00e-5 m, 2.74e-5 rad)`, `(9.56e-6 m, 1.72e-5 rad)`, and `(8.48e-6 m, 1.97e-5 rad)`. A 100-body binary branch initially increased anchor error from `5.39e-3` to `5.87e-3 m`; after 10 steps it was `2.79e-3 m`, with frame error `3.94e-5 rad`. This confirms slower, non-monotone anchor convergence for the branch.
- Source isolation on the 100-part fixture: no loads stayed at roundoff; gravity without contact stayed below `3.93e-8 m / 5.97e-8 rad`; gravity plus pad contact reached `0.293 m / 2.187 rad`; thrust without gravity/contact reached `0.103 m / 0.907 rad`. Centering the same thrust barely changed the errors (`0.103 m / 0.984 rad`), so its offset moment is not the main trigger. Both contact and thrust load cases are sufficient to expose the weakness.
- The existing adaptive test path (1–8 position and velocity iterations) reduced geometry errors for contact (`0.293/2.187` to `0.134/1.259`) and thrust (`0.103/0.907` to `0.0604/0.567`) but roughly doubled elapsed time and increased coast energy drift (`46.7%` to `69.0%`, `68.1%` to `71.4%`). It remains experimental and is not a production fix.
- These experiments establish load sensitivity and slow branched-graph convergence, but do not prove whether contact/joint ordering, per-sweep Jacobian refresh, or velocity/position coupling is the first rocket failure. Continue with residual tracing before changing production order or iteration defaults.

## 2026-11-02 — position projection order A/B/C

Variant A is the existing scalar Linear → Angular sweep. Variant B solves Angular, refreshes connector geometry and rows, then solves Linear. Variant C solves Linear, refreshes and solves Angular, refreshes again, then solves Linear. All variants retain multipliers by row key and apply only the newly computed multiplier delta.

| Parts | Order | Anchor max, m | Frame max / RMS, rad | Coast energy drift | Elapsed, ms |
|---:|---|---:|---:|---:|---:|
| 100 | A | 0.1559734 | 1.358748 / 0.08467397 | 4.1414% | 852 |
| 100 | B | 0.1387972 | 1.389727 / 0.08279818 | 4.0002% | 911 |
| 100 | C | 0.1323372 | 1.581435 / 0.09111354 | 3.6841% | 1,098 |
| 200 | A | 0.2910659 | 1.713914 / 0.1226009 | 70.8814% | 1,724 |
| 200 | B | 0.2852765 | 1.649234 / 0.1132870 | 69.1131% | 1,828 |
| 200 | C | 0.2896826 | 1.491072 / 0.1129851 | 68.7790% | 2,259 |
| 500 | A | 0.7676298 | 3.139344 / 0.2949535 | 58.3762% | 4,408 |
| 500 | B | 0.8195813 | 2.614975 / 0.2813381 | 63.2394% | 4,638 |
| 500 | C | 0.7424552 | 3.094329 / 0.2548137 | 66.1312% | 5,700 |
| 1,000 | A | 3.023810 | 3.141562 / 0.6276528 | 45.5216% | 8,836 |
| 1,000 | B | 2.626698 | 3.141119 / 0.5419593 | 42.0811% | 9,293 |
| 1,000 | C | 2.576289 | 3.141282 / 0.4688818 | 46.5546% | 11,576 |

All three configurations fail the original strict limits. B is about 5–7% slower than A and trades anchor error against frame error. C improves maximum anchor error at all sizes and frame RMS at 200–1,000 parts, but costs about 26% more elapsed time, worsens the 100-part frame maximum, and raises 500/1,000-part coast energy drift relative to A. The actual strict C run failed at all sizes. Its first anchor/frame limit crossing ticks were `15/47`, `14/32`, `12/22`, and `12/19` for 100, 200, 500, and 1,000 parts. The larger cases cross the anchor limit on the first booster-thrust step (tick 12). The test does not identify a sequence as a complete fix.

The production path currently uses C because it refreshes the offset-anchor geometry after angular correction and reduces the measured anchor maximum in all four rocket runs. The strict Release run on this exact implementation failed at all four sizes; it completed every tick with no step failure and reproduced the report-only metrics. The full workspace suite passed (781 passed, 4 ignored), as did workspace check, strict Clippy, formatting, and diff checks. This is a partial mathematical correction only; the stability defect remains, the extra pass has a measurable cost, and no overall win is claimed. Continue isolating the first loaded-step residual before considering solver optimization or Block3 promotion.

## 2026-11-02 — loaded-step stage residuals and sweep response

- Added `bench-internals` snapshots of raw geometric residuals before and after velocity solving, after contact position correction, and after joint position projection. FixedJoint's three world-axis anchor rows and three rotation-log rows are combined as Euclidean norms. The trace also records the maximum velocity-row residual after velocity and position phases. This instrumentation is absent from the normal build and does not change solver behavior.
- In the 100-part full fixture, booster ignition at tick 12 raises anchor error from `0.01608835 m` on tick 11 to `0.03038108 m` before velocity constraints. It remains exactly `0.03038108 m` after velocity solving, falls to `0.01998944 m` after contact correction, and to `0.01944706 m` after joint projection. The strict anchor threshold is crossed at tick 15, but the pose error first jumps at force integration, before the velocity solver.
- On tick 12, the maximum linear velocity-row residual is `0.5146800 m/s` after velocity solving and `0.5146831 m/s` after position projection; angular residual is `0.2467740 rad/s` at both points. Through ticks 12–16, position projection changes the maximum velocity residual by only a few `1e-5 m/s` or less. This rejects velocity changes caused by position projection as the source of the observed growth.
- Load isolation remains consistent: no-load and gravity-only runs stay near machine precision; thrust without contact crosses the anchor limit at tick 20; gravity with contact crosses at tick 19. Centering the engine force changes the maxima only slightly, so thrust moment is not the main trigger. The first strict crossing also precedes tank mass updates at tick 24 and stage separation at tick 54.

The sweep-count comparison below is diagnostic only. It uses the same 100-part, 180-tick fixture and scalar solver:

| Load mode | Sweeps (velocity / position) | Anchor max, m | First limit tick | Velocity row residual at crossing, m/s | Elapsed, ms | Coast energy drift |
|---|---:|---:|---:|---:|---:|---:|
| Thrust, no gravity/contact | 8 / 3 | 0.09826316 | 20 | 0.6828425 | 1,323 | 68.8932% |
| Thrust, no gravity/contact | 16 / 3 | 0.05796826 | 25 | 0.3014304 | 1,954 | 73.0034% |
| Thrust, no gravity/contact | 8 / 6 | 0.07638819 | 28 | 0.7535137 | 2,043 | 70.7027% |
| Thrust, no gravity/contact | 16 / 6 | 0.03661618 | 77 | 0.2904382 | 2,688 | 73.6789% |
| Gravity with contact | 8 / 3 | 0.2081661 | 19 | 0.5208623 | 1,389 | 42.9573% |
| Gravity with contact | 16 / 3 | 0.1065852 | 24 | 0.3264612 | 2,031 | 20.0153% |
| Gravity with contact | 8 / 6 | 0.1462785 | 21 | 0.5407806 | 2,097 | 21.5286% |
| Gravity with contact | 16 / 6 | 0.1147427 | 28 | 0.3319039 | 2,775 | 12.9566% |

More velocity sweeps lower the velocity residual and delay geometry failure. More position sweeps also help, though less consistently. The 16 / 6 thrust run still exceeds the anchor, frame (`0.407537 rad`), and coast-energy (`73.6789%`) limits at roughly twice the default runtime. The data supports load-dependent under-convergence in the connected solver, but does not yet establish whether row ordering, island topology, or another velocity-stage defect is the underlying cause. Do not promote higher fixed iteration counts as a fix; continue tracing convergence by sweep and retain the strict failure.

- Validation for this diagnostic checkpoint: `cargo +1.98.0 fmt --all -- --check`, `cargo +1.98.0 check --workspace --all-features --locked`, and strict workspace Clippy passed. `cargo +1.98.0 test --workspace --locked` passed with 781 tests and 4 ignored; the focused geometric residual test and both ignored Release diagnostics also passed.
- The strict Release rocket fixture was rerun with `KAPITSA_ROCKET_REPORT_ONLY` unset and failed as required by its real thresholds. All scenarios completed without step failure or joint breaks, but numerical stability failed at 100/200/500/1,000 parts. Current C metrics were respectively anchor max/RMS `0.1323372/0.01108183`, `0.2896826/0.02798799`, `0.7424552/0.06016442`, and `2.576289/0.1463842` m; max frame error `1.581435`, `1.491072`, `3.094329`, and `3.141282` rad; coast drift `3.6841%`, `68.7790%`, `66.1312%`, and `46.5546%`. The failure is preserved; the diagnostic work is not a correctness pass.

## 2026-11-02 — per-sweep residual response and reverse-order check

- Extended the benchmark trace to retain the first and final velocity-sweep residuals, first and final joint-position anchor residuals, and the velocity residual after contact solving but before joint velocity solving. This distinguishes whether the contact phase reintroduces error before the joint solver sees it.
- At the first anchor-limit tick with thrust and no contact, default 8/3 sweeps reduce the linear velocity row residual from `0.9840819 m/s` after the first joint velocity sweep to `0.6828425 m/s` after all eight. The first position sweep leaves `0.03509969 m` anchor error; all three leave `0.03196534 m`. At 16/3 these fall from `0.9354011` to `0.3014304 m/s`, and from `0.03251405` to `0.03026537 m` respectively, but the complete run still fails.
- With gravity and contact at 8/3, the residual is `0.5133333 m/s` after the first complete velocity iteration, `0.5322764 m/s` after contact velocity solving in the last iteration, and `0.5208623 m/s` after the final joint velocity solve. The first/final position anchor residuals are `0.03351901/0.03245108 m`. At 16/3, velocity residual decreases from `0.3422268` to `0.3264612 m/s`; the contact phase ends at `0.3305984 m/s`. This is evidence of contact/joint interaction in the contact case, while thrust alone still fails without contacts.

Reversing the rocket's dense joint storage order provides a controlled solver ordering comparison without changing physics or solver code. It also updates the diagnostic stage-joint ID and verifies the stored body-pair order really is reversed:

| Mode | Order | Anchor max / RMS, m | Frame max, rad | First anchor limit tick | Coast energy drift |
|---|---|---:|---:|---:|---:|
| Full | Forward | 0.1323372 / 0.01108183 | 1.581435 | 15 | 3.6841% |
| Full | Reverse | 0.2783684 / 0.04333044 | 1.446800 | 16 | 5.7104% |
| Thrust, no gravity/contact | Forward | 0.09826316 / 0.01210467 | 1.026284 | 20 | 68.8932% |
| Thrust, no gravity/contact | Reverse | 0.1334365 / 0.02090281 | 0.993433 | 21 | 65.6509% |
| Gravity with contact | Forward | 0.2081661 / 0.02127407 | 2.206484 | 19 | 42.9573% |
| Gravity with contact | Reverse | 0.3175803 / 0.03548104 | 1.624990 | 20 | 48.2826% |

Reverse order slightly improves frame maxima but materially worsens anchor error in all three modes. A global reverse sweep is rejected as a fix; topology-aware ordering or symmetric sweeps still require separate measurement. This supports that graph/order sensitivity exists, but does not yet identify a single defective Jacobian or mass/COM calculation. Keep the remaining cause open.

## 2026-11-02 — loaded graph velocity convergence

Added an ignored Release diagnostic with no colliders or gravity. It applies a one-step thrust-like force at an offset point on the root of fixed-joint chains of 2, 10, 50, and 100 bodies, plus a 100-body binary branch. Bodies use varied mass, inertia, and local COM. The following rows show final velocity residual after the configured sweeps and the next step's predicted anchor error before position projection:

| Graph | Velocity sweeps | Linear residual, m/s | Angular residual, rad/s | Next-step predicted anchor error, m |
|---|---:|---:|---:|---:|
| Chain 10 | 8 | 0.033454 | 0.046610 | 0.000837 |
| Chain 50 | 8 | 0.179980 | 0.250763 | 0.004505 |
| Chain 100 | 8 | 0.360431 | 0.502182 | 0.009046 |
| Branch 100 | 8 | 0.287878 | 0.231644 | 0.009498 |
| Chain 10 | 16 | 0.018329 | 0.037358 | 0.000695 |
| Chain 50 | 16 | 0.098784 | 0.202327 | 0.003743 |
| Chain 100 | 16 | 0.197826 | 0.405184 | 0.007510 |
| Branch 100 | 16 | 0.070497 | 0.070647 | 0.007543 |

The 100-body chain's first post-thrust velocity sweep leaves `2.319957 m/s` linear and `1.420510 rad/s` angular residual; 8 sweeps reduce these to `0.360431/0.502182`, and 16 to `0.197826/0.405184`. A second load-free step still produces a predicted anchor error before projection, so the residual is carried into the next integration. Small graphs converge rapidly (the 2-body chain reaches roundoff by 8 sweeps), while residual grows with graph size. This is direct evidence of loaded Gauss–Seidel under-convergence in long connected graphs; it is not caused by contact because the diagnostic has no colliders, and unequal masses/inertias and nonzero COM are part of the setup.

Joint-storage order remains topology-dependent. At 100 bodies and 8 sweeps, reversing the chain increases linear/angular residual from `0.360431/0.502182` to `0.437700/0.613877`; reversing the branch improves linear but worsens angular (`0.287878/0.231644` to `0.138199/0.384722`). At 16 sweeps, reverse order worsens both residuals for both graphs. A global reverse traversal is not a general fix. The remaining production work should target bounded residual-based convergence and a measured graph-aware sweep policy; do not promote larger fixed iteration counts alone.

A bench-only alternating forward/reverse velocity sweep was measured against forward order. For the 100-body chain, 8 sweeps lower linear residual and next-step predicted anchor error (`0.360431` to `0.192529 m/s`, `0.009046` to `0.006347 m`) but increase angular residual (`0.502182` to `0.639143 rad/s`). At 16 sweeps the chain again trades a small linear improvement for worse angular residual (`0.197826/0.405184` to `0.167667/0.475874`). The 100-body branch worsens in both residuals at 16 sweeps (`0.070497/0.070647` to `0.084973/0.166048`). Alternating direction is therefore not promoted as a general convergence fix; it remains a measured diagnostic option only.

- Validation of the load/order diagnostic: Release ignored diagnostic passed, full workspace tests passed on rerun, format and strict workspace Clippy passed. The strict Release rocket test was rerun with report-only unset and reproduced the same failure metrics at all four sizes; the benchmark hook is disabled by default and the production trajectory is unchanged.

## 2026-11-02 — bounded residual iteration limit

The existing benchmark-only adaptive path was extended to test velocity iteration caps of 8, 16, 32, 64, and 128 at a fixed residual tolerance of `1e-4 m/s / 1e-4 rad/s`. It hit every configured cap on both a loaded 100-body chain and branch; none converged to the requested residual. At 128 sweeps, the chain remained at `0.05636 m/s / 0.10425 rad/s` and the branch at `0.008379 m/s / 0.02968 rad/s`. The one-step measurements took about `70 ms` for each 100-body graph, versus `14–15 ms` at the 8-sweep adaptive cap. Position projection reduced the end-of-step geometry to `0.00286 m / 0.00424 rad` (chain) and `0.00230 m / 0.00191 rad` (branch), but did not make the velocity residual converge.

This rejects simply raising the adaptive cap as a production solution: the existing loop silently exhausts its maximum without exposing that fact, and reaching even a loose local velocity residual is expensive. The next solver experiment needs to improve the linear solve or propagation per sweep, then retain a bounded cap with an explicit non-convergence result.

## 2026-11-03 — energy balance and velocity projection audit

- Replaced the explicit angular-velocity Euler update in `predict_dynamic()` with a midpoint update that evaluates gyroscopic acceleration at the midpoint orientation and angular velocity. A free asymmetric body with inertia `(2, 3, 4)`, initial `omega=(1,2,3)`, and ten simulated seconds previously drifted `+7.348426%` at `dt=1/120` and `+3.427050%` at `dt=1/240`. The midpoint update now measures `−4.313487e−4%` and `−1.077910e−4%`, with relative world angular-momentum errors `3.497e−5` and `8.365e−6`. The active Release regression passes at both steps.
- Corrected the rocket coast-energy potential reference from the arbitrary surface zero to physical `U=−mu*m/r` with zero at infinity. The metric normalizes signed drift by the initial energy scale `|T|+|R|+|U|`, so it is independent of a near-zero total and no longer uses an arbitrary potential offset. For the 500-part gravity-only, contact-free, thrust-free run, the former surface-relative metric reported `1.351126%`; the corrected metric reports `4.758411e−7%`. Across the strict scalar rocket's sizes it reports `1.50e−6%`, `1.56e−5%`, `8.52e−6%`, and `6.67e−5%`. This corrects the measurement; it does not fix the separate geometric Joint failure.
- In the no-load 100-part rigid rocket assembly over 180 steps, summed integration energy change was `−4.02e−10 J`, scalar velocity-solver change `−1.51153e−2 J`, and position-solver change `−5.91e−5 J`; total kinetic drift was `0.075847%`. With 64-iteration global velocity/position PCG, drift was `0.004841%`, summed velocity PCG change `−1.07777e−3 J`, and position PCG change `−1.87300e−5 J`. The scalar assembly does not pass the provisional `0.01%` diagnostic limit; the PCG diagnostic does.
- Added signed work accounting for the complete velocity correction. In an isolated two-body FixedJoint, measured `Delta K` matched `v·Delta p + 0.5 Delta v·Delta p + omega·Delta L + 0.5 Delta omega·Delta L` within `1.5e−16 J`. Halving `dt` halved the initial linear `|Jv|` and the integrated projection loss over equal simulated time. That projection loss is consistent with removing velocity that violates the updated constraint; it is not itself treated as an error.
- On an isolated 100-body chain at `dt=1/120`, initial post-integration max residual was `4.77e−4 m/s`. Scalar GS ended at `7.36e−2 m/s` and removed `120.23 J`; PCG ended at `3.37e−15 m/s` and removed `1.977 J`. At half step, PCG loss fell to `0.989 J`, while scalar ended at `4.77e−2 m/s` and removed `122.15 J`. This supports large-island scalar under-convergence as excess numerical dissipation; the PCG change matches its applied correction work. On one joint both methods converged to the same residual and loss, so the excess is graph-scale dependent.
- Position projection altered kinetic energy much less than the velocity phase in these fixtures. Gravity-only rocket integration showed signed accumulated changes `Delta T=+47,121.57 J`, `Delta U=−47,383.35 J`, `Delta E=−261.79 J` over 180 steps; velocity solving changed energy by `−1.50e−9 J` and position solving by `−5.72e−5 J`. The collision-free full-scene coast metric stops before the first upper-stage surface contact and records that contact separately.
- The strict Release scalar rocket test still fails geometry at 100/200/500/1,000 parts, with anchor maxima `0.1334/0.2907/0.7411/2.5072 m` and frame maxima `1.5689/1.5450/2.7995/3.1412 rad`; no step failed, all 180 ticks completed, and separation momentum error stayed below `6.4e−14`. A preliminary cap-64 PCG report completed all sizes with geometry below strict limits, but the filter accidentally selected four similarly named diagnostics that ran concurrently; timings are invalid and this was not the strict test. The controlled sequential cap-1 sweep is in progress and already fails anchor limits at 100/200/500 parts. Minimum-cap selection, a sequential strict PCG run, and fair performance comparison remain open.
- PCG position iteration count remained zero in cap-64 rocket measurements. Velocity PCG is the mechanism improving loaded graph convergence; no position-PCG benefit is established. Do not promote either path to production based on these diagnostics alone.
- Validation so far: focused Release free-spin test passed; Release free-assembly PCG check passed; isolated no-contact energy diagnostic passed; isolated two-vs-100-body projection-work diagnostic passed; `cargo fmt` and `cargo check -p kapitsa --features bench-internals` passed. The strict scalar Release fixture failed as expected on geometry. Workspace tests and Clippy have not yet been rerun for this checkpoint.

## 2026-11-03 — velocity PCG reuse and strict rocket follow-up

- The velocity PCG previously rebuilt the same joint rows, masses, diagonal, and IC(0) factorization on each velocity pass. The Jacobian and effective-mass matrix are fixed during the velocity phase; only the velocity residual changes. The solver now prepares those rows and matrix once per step and reuses PCG vectors and nested IC(0) storage across steps. Position PCG continues to rebuild its system after each position correction because those Jacobians can change with pose.
- A 100-body no-contact chain allocation diagnostic compared 32 warmed world steps for scalar and PCG. Each measured `1 allocation / 19.2 KB per step`; the PCG path added no allocation over the same scalar world path. The shared world-step allocation remains, so this is an incremental comparison rather than a claim of zero total allocations.
- After reuse, the loaded chain/branch Release diagnostic passed, and the 100-body energy-stage fixture still closed the velocity-projection work identity within `4.8e−15 J` with cap-64 PCG.
- The strict Release rocket test with PCG caps `64/64` passed numerical geometry and coast-energy checks at 100, 200, and 500 parts. At 1,000 parts geometry was also within strict limits (max anchor `1.012952e−4 m`, max frame `3.014574e−5 rad`) and signed coast drift was `−4.12e−7%`, but the fixture's existing launch-completion condition failed: upper-root max altitude was `1.524 m` versus the required `>1.6 m`. All 180 ticks completed and there was no step failure. A single-run 16/16 diagnostic had the same 1,000-part altitude and similarly small geometry residuals, indicating this flight-height condition is insensitive to PCG cap once the joints are well solved. Do not lower the flight criterion or deliberately under-converge constraints to fit it.
- Full-rocket elapsed time on the same Release fixture was `1.479 / 3.057 / 7.720 / 15.672 s` for scalar and `1.934 / 5.417 / 23.742 / 75.521 s` for PCG 64/64 at 100/200/500/1,000 parts. This is a 1.31x / 1.77x / 3.07x / 4.82x slowdown. These are scenario timings, not steady-state solver microbenchmarks; PCG currently improves constraint residuals but is not a performance win.
- The 64/64 strict run is not an overall PASS because the 1,000-part scenario remains incomplete by its existing altitude criterion. A physical investigation of the pad-contact/thrust interaction remains open; changing the metric or selecting a knowingly under-converged cap is not justified by these results.
