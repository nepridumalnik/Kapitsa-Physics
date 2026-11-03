# Joint Solver energy and parallel results

Date: 2026-11-03

Branch: `audit/physics-remediation`

Toolchain: Rust 1.98.0, Windows x86_64 MSVC, Ryzen 9 7900X3D (12 cores / 24 logical processors)

## Result

The large coast-energy failure was primarily a metric error: the rocket's gravitational potential used an arbitrary surface-relative zero, so its percent drift depended on that offset. The metric now uses `U = -μm/r` (zero at infinity) and normalizes signed drift by the initial `|T| + |R| + |U|` scale. The full-flight coast sample starts after upper-stage cutoff and stops at the first upper-stage surface contact, so impact dissipation is not labeled as coast drift. A separate integrator defect was proven for torque-free asymmetric rotation: explicit Euler gyroscopic acceleration caused large energy drift; a midpoint update reduces it by orders of magnitude.

For a constrained assembly, most of the remaining energy change occurs during velocity projection, after independent body predictions leave nonzero `Jv`. The measured kinetic-energy change matches the work of the applied correction impulses to roundoff. Reducing `dt` reduces the single-joint mismatch and projection loss. On a large chain, Scalar Gauss–Seidel additionally stops with a large residual and removes much more energy than the converged PCG projection. Position correction changes the measured `Jv` only slightly in these fixtures.

The strict 100/200/500/1,000-part rocket fixture passes with benchmark-only velocity/position PCG caps `5/5` and 16 outer velocity sweeps. Caps 1–4 fail at 1,000 parts. The production scalar configuration still fails the strict geometric limits, while the passing PCG configuration is substantially slower and many local PCG calls hit their cap. PCG remains experimental; no solver defaults or acceptance tolerances were changed.

## Energy accounting and diagnosis

All reported changes use the signed convention `ΔE = E_after − E_before`; positive values are gains, negative values are losses. `T`, `R`, and `U` are translational, rotational, and gravitational potential energy. Stage diagnostics sample after prediction/integration, after velocity solving (including each PCG call), and after position solving (including each PCG call).

### Free asymmetric rotor

The free-body test uses anisotropic inertia, no external force or torque, and ten seconds of simulation. The midpoint gyroscopic update is second-order in the tested timestep range:

| `dt` | Energy change | Relative energy drift | Relative angular-momentum drift |
|---:|---:|---:|---:|
| `1/120 s` | `−1.078371869e−4 J` | `−4.313487477e−4%` | `3.496610587e−5` |
| `1/240 s` | `−2.694775037e−5 J` | `−1.077910015e−4%` | `8.364597981e−6` |

Before the midpoint change, the same fixture drifted `+7.348426%` and `+3.427050%`, respectively. This was a demonstrated integrator error and is fixed without changing tolerances.

### Velocity projection and `Jv`

At fixed poses and inertia, the correction-work identity is

`ΔK = Σ(v·Δp + ½Δv·Δp + ω·ΔL + ½Δω·ΔL)`.

The first-order term may be negative when the solver removes constraint-violating relative velocity. The quadratic term is positive. The measured sum matches the velocity-stage energy change in the isolated fixtures:

| Graph / solver | `dt` | Post-integration max `|Jv|` | Post-velocity max residual | Velocity `ΔK` over 1.5 s | Work-identity closure |
|---|---:|---:|---:|---:|---:|
| 2-body FixedJoint / scalar | `1/120` | `4.5551e−4 m/s` | `3.47e−17 m/s` | `−8.2531e−6 J` | `5.38e−17 J` |
| 2-body FixedJoint / scalar | `1/240` | `2.2780e−4 m/s` | `1.39e−17 m/s` | `−4.1268e−6 J` | `3.36e−17 J` |
| 100-body chain / scalar | `1/120` | `4.7690e−4 m/s` | `7.3648e−2 m/s`, `1.976e−3 rad/s` | `−1.2023353e2 J` | `1.85e−13 J` |
| 100-body chain / IC(0) PCG 64 | `1/120` | `4.7690e−4 m/s` | `3.37e−15 m/s`, `1.53e−16 rad/s` | `−1.9767731 J` | `3.63e−13 J` |
| 100-body chain / IC(0) PCG 64 | `1/240` | `2.3850e−4 m/s` | `3.33e−15 m/s`, `8.33e−17 rad/s` | `−9.8888785e−1 J` | `9.93e−13 J` |

On one FixedJoint, scalar and PCG reach the same projected state and lose the same energy. Halving `dt` halves the initial linear `|Jv|` and the total projection loss over equal simulated time. On the chain, scalar under-convergence accounts for the excess loss: its final residual remains large, while IC(0) PCG drives it to roundoff. The projection identity establishes that the solver applies the measured correction; the timestep and graph-size comparison establishes where the incompatible velocity originates and how scalar under-convergence amplifies it. This is numerical dissipation from the split predict/project update, not a reason to add energy inside the solver.

### Position correction and velocity consistency

The diagnostics also measure the velocity residual again after position projection. In the 2-body test, residual after position correction is `1.70e−8 m/s` at `dt=1/120` and `4.26e−9 m/s` at `dt=1/240`. In the 100-body IC(0) chain it is `1.57e−7 m/s` after velocity solving at roundoff; position projection does not recreate the large scalar residual. In the 100-part free rocket assembly, the scalar residual changes from `2.795514e−3 / 3.146019e−3` to `2.795645e−3 m/s / 3.146019e−3 rad/s`; PCG changes from `3.801162e−5 / 8.639748e−5` to `3.781971e−5 m/s / 8.639754e−5 rad/s`.

Position-stage energy change is also smaller than the velocity projection change in the assembly: scalar `−5.9066e−5 J` and PCG `−1.8730e−5 J` over 180 ticks, versus velocity changes `−1.5115e−2 J` and `−1.0778e−3 J`. The position solver changes poses without applying physical velocity impulses; the measured post-position residual does not identify it as the source of the large drift.

### Free rigid assembly and gravity-only rocket

The 100-part rigid rocket has no colliders, gravity, or thrust. Over 180 ticks, integration changes energy by only `−4.02e−10 J` (scalar) / `−4.08e−10 J` (PCG). The signed stage changes are:

| Solver | Velocity stage `ΔT / ΔR / ΔU`, J | Position stage `ΔT / ΔR / ΔU`, J | Total kinetic drift |
|---|---:|---:|---:|
| Scalar | `−1.6452e−2 / +1.3362e−3 / 0` | `0 / −5.9066e−5 / 0` | `0.075847%` |
| IC(0) PCG 64/64 | `−5.9563e−3 / +4.8786e−3 / 0` | `0 / −1.8730e−5 / 0` | `0.004841%` |

The velocity-work closure is `3.48e−15 J` for scalar and `−4.79e−15 J` for PCG. A second test compares this free assembly to a single body with the equivalent combined mass properties. With PCG 64/64, after 180 ticks their relative energy difference is `9.7898e−5`, relative angular-momentum difference is `8.3736e−5`, and center-of-mass difference is `1.17e−14 m`. This is a regression/reference check, not an energy-preserving rigid-cluster implementation.

For a 500-part rocket under gravity only, with contacts and thrust disabled, there are zero contact events and no step failure. Across 180 ticks, integration changes `T` by `+4.7121567e4 J`, `U` by `−4.7383354e4 J`, and total energy by `−2.6178704e2 J`; velocity correction changes total energy by `−1.50e−9 J`, and position correction by `−5.72e−5 J`. The corrected coast metric reports `+4.7584070e−7%` drift. This separates gravity integration error from the constrained-velocity projection.

The earlier large full-flight PCG coast percentage is also metric-sensitive. Re-running the 500-part full fixture with IC(0) PCG 64/64 on the final code gives a physical coast drift of `4.598934067e−7%` through tick 176; sampling stops before the first upper-stage surface contact at tick 177. Evaluating that exact same trajectory with the former surface-relative potential and its `|E_initial|` denominator gives `88.94944039%`. This large discrepancy comes from the arbitrary potential offset and near-cancellation in that denominator, not from a second simulation or a change in solver state. The no-load 100-part assembly is a different quantity: kinetic-only drift with no gravity, contacts, thrust, or stage events. Current IC(0), diagonal, and block-Jacobi PCG runs report `0.004841%`, `0.004958%`, and `0.004907%` respectively. The previously mentioned approximately 6% experimental-PCG result is not reproducible from the saved current fixtures, and its exact prior configuration is not present in the repository; it should not be compared directly with the full-flight coast metric.

## Strict rocket iteration screen

The screen holds the strict fixture, 16 outer velocity sweeps, and three position sweeps constant. No strict thresholds were relaxed.

| PCG cap V/P | Anchor max, m | Frame max, rad | Max altitude, m | Result at 1,000 parts |
|---:|---:|---:|---:|---|
| `1/1` | `0.599928` | `0.089283` | `1.249` | `Collision(InvalidContact)` at tick 125; fail |
| `2/2` | `0.235893` | `0.084935` | `1.249` | Geometry and altitude fail |
| `3/3` | `0.067090` | `0.032343` | `1.452` | Anchor and altitude fail |
| `4/4` | `0.028269` | `0.010184` | `1.573` | Altitude fails (`>1.6 m` required) |
| `5/5` | `0.026590` | `0.010279` | `1.613` | Pass |

Caps 1–3 were rerun after the rotational CCD change. The cap-4 screen was also run on the final CCD version. The strict 5/5 test passes at all four sizes, completing 180 ticks and reproducing deterministic repeats. It retains the existing geometry, coast-energy, altitude, contact, separation, and completion criteria. Selected results:

| Parts | Run ms | Anchor max / RMS, m | Frame max / RMS, rad | Coast drift | Max altitude, m | PCG converged / cap hits |
|---:|---:|---:|---:|---:|---:|---:|
| 100 | 2,283 | `0.000347 / 0.000034` | `0.000691 / 0.000035` | `−4.78e−7%` | 2.310 | `2,744 / 2,152` |
| 200 | 6,135 | `0.001417 / 0.000150` | `0.001380 / 0.000178` | `−4.76e−7%` | 2.192 | `2,315 / 2,581` |
| 500 | 29,114 | `0.009504 / 0.001406` | `0.007923 / 0.001031` | `−1.77e−5%` | 1.834 | `1,513 / 3,271` |
| 1,000 | 81,353 | `0.026590 / 0.001923` | `0.010279 / 0.001075` | `−4.28e−7%` | 1.613 | `2,306 / 2,590` |

Many local PCG calls still hit the cap; this fixture pass does not imply every solve reached its internal residual tolerance. With the production scalar path and unchanged defaults, the same strict rocket test fails the geometric thresholds at every size: maximum anchor error is `0.1334 / 0.2907 / 0.7411 / 2.5072 m` for 100/200/500/1,000 parts. PCG is therefore not enabled by default. The 5/5 strict configuration is about `2.7x / 3.5x / 6.4x / 9.0x` slower than the original revision's strict-run timings, whose trajectory also fails the original correctness limits.

### Global PCG scratch reuse and minimum outer sweeps

Position PCG now keeps its vectors, row terms, IC(0) factor buffers, and body impulses in the joint workspace, and resolves mass data through the validated per-body cache. Repeated position solves at a stable island size reuse those capacities. Position PCG reports converged calls separately from calls that reached the iteration cap; the focused limit test verifies that a capped solve is not counted as converged. The strict fixture repeats every size and compares all final body states exactly.

The strict 1,000-body screen used three contact/position sweeps and IC(0) for position PCG. With velocity and position caps fixed at 5, all 16 integer external velocity sweep counts were tested. Counts 1 through 15 fail an unchanged strict criterion; 16 is the minimum passing value. The full screen is:

| External velocity sweeps | Run ms | Anchor max, m | Frame max, rad | Max altitude, m | Result |
|---:|---:|---:|---:|---:|---|
| 1 | 94,305 | `0.247406` | `0.228963` | 1.249 | Fail |
| 2 | 66,090 | `0.092469` | `0.031892` | 1.249 | Fail |
| 3 | 66,608 | `0.045015` | `0.010554` | 1.259 | Fail |
| 4 | 68,076 | `0.042868` | `0.014510` | 1.306 | Fail |
| 5 | 66,542 | `0.032450` | `0.011077` | 1.351 | Fail |
| 6 | 67,685 | `0.035791` | `0.010236` | 1.388 | Fail |
| 7 | 68,642 | `0.033007` | `0.010487` | 1.419 | Fail |
| 8 | 71,340 | `0.033844` | `0.010363` | 1.451 | Fail |
| 9 | 70,221 | `0.032167` | `0.011141` | 1.477 | Fail |
| 10 | 71,000 | `0.029973` | `0.011501` | 1.502 | Fail: altitude |
| 11 | 72,337 | `0.029105` | `0.011167` | 1.522 | Fail: altitude |
| 12 | 75,126 | `0.027486` | `0.010333` | 1.545 | Fail: altitude |
| 13 | 75,461 | `0.026793` | `0.010837` | 1.562 | Fail: altitude |
| 14 | 76,903 | `0.026770` | `0.010944` | 1.582 | Fail: altitude |
| 15 | 78,224 | `0.026486` | `0.010856` | 1.598 | Fail: altitude (`>1.6 m` required) |
| 16 | 79,388 | `0.026590` | `0.010279` | 1.613 | Pass |

Raising the velocity cap to 16 does not remove the need for the outer contact passes: 8 and 12 external sweeps still fail altitude at `1.523 m` and `1.585 m`. At 16 sweeps and cap 5, the 1,000-body run reports velocity `2,306 converged / 2,590 capped` calls and position `437 / 481`; the physical fixture passes even though many local linear solves reach their caps.

The position-PCG timing counters over the 180-tick 1,000-body run were `38,235 ms` system preparation (including IC(0) construction), `172 ms` matrix-vector products, `1,841 ms` preconditioner applications, and `13 ms` reductions. IC(0) setup dominates. The current strict run took `77,845 ms`, versus `81,353 ms` recorded before the Scalar and mass-cache optimizations; this `4.3%` change is not an isolated PCG A/B measurement. The saved strict results are:

| Parts | Run ms | Anchor max / RMS, m | Frame max / RMS, rad | Coast drift | Max altitude, m | Velocity conv/cap | Position conv/cap |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 1,967 | `0.000347 / 0.000034` | `0.000691 / 0.000035` | `−4.78e−7%` | 2.310 | `2,744 / 2,152` | `456 / 462` |
| 200 | 5,537 | `0.001417 / 0.000150` | `0.001380 / 0.000178` | `−4.76e−7%` | 2.192 | `2,315 / 2,581` | `406 / 512` |
| 500 | 27,839 | `0.009504 / 0.001406` | `0.007923 / 0.001031` | `−1.77e−5%` | 1.834 | `1,513 / 3,271` | `260 / 637` |
| 1,000 | 77,845 | `0.026590 / 0.001923` | `0.010279 / 0.001075` | `−4.28e−7%` | 1.613 | `2,306 / 2,590` | `437 / 481` |

Alternative position preconditioners were screened at 1,000 bodies, 16 external sweeps, and cap 5. Diagonal PCG took `43,364 ms` but ended at `0.216 m / 0.782 rad`; one-joint block Jacobi took `42,559 ms` and ended at `0.121 m / 0.400 rad`. A two-joint block test took `42,108 ms` and ended at `0.139 m / 0.361 rad`; increasing its position cap to 16 reduced the anchor error to `0.0480 m` but frame error remained `0.208 rad`. These faster variants fail strict geometry and are not retained as selectable solver paths. IC(0) remains the only screened position preconditioner that passes.

## Release performance and parallelism

The large modular rocket benchmark uses 5 warm-up and 60 steady samples at `dt=1/120 s`, on the same local machine and toolchain as the original baseline. The table below preserves the measurements from before the Scalar FixedJoint optimization:

| Parts | Original 1 worker | Current 1 worker | Current 8 workers |
|---:|---:|---:|---:|
| 100 | `5.582 / 5.633 ms` | `8.289 / 8.377 ms` | `8.233 / 8.662 ms` |
| 200 | `11.293 / 12.089 ms` | `16.464 / 16.627 ms` | `16.545 / 17.381 ms` |
| 500 | `28.416 / 30.852 ms` | `41.731 / 42.251 ms` | `42.343 / 44.163 ms` |
| 1,000 | `56.540 / 57.065 ms` | `83.349 / 84.523 ms` | `85.739 / 87.599 ms` |

Those pre-optimization single-worker p50 values were `46–49%` slower than the original baseline. Eight scoped workers did not speed up this connected rocket workload; results were within noise at 100 parts and slower at larger sizes. A separate loaded binary-branch test measured `16%` improvement at 500 bodies with two workers and `23%` at 1,000 bodies with four workers. A connected chain remained sequential, and eight workers did not improve on four.

### Scalar FixedJoint incremental preparation

To isolate stage 1, the old `c936f44` implementation and the optimized implementation were benchmarked in Release with identical `bench-internals` row-preparation timers, warm-up, sample count, fixture, one worker, and the mass cache disabled in both runs. The detached baseline run completed all four one-worker sizes; later worker-count runs were stopped after capturing the four one-worker comparisons. Times are p50/p95. Each prepared FixedJoint row count includes all constraints in one world step; scalar solved-row counts are unchanged.

| Parts | Full step before, ms | Full step after, ms | Row preparation before, ms | Row preparation after, ms | Prepared rows before → after | Solved rows |
|---:|---:|---:|---:|---:|---:|---:|
| 100 | `8.418 / 8.540` | `7.926 / 8.573` | `1.346 / 1.372` | `1.044 / 1.120` | `10,098 → 8,316` | `7,425` |
| 200 | `17.111 / 18.024` | `15.871 / 16.484` | `2.720 / 2.803` | `2.090 / 2.150` | `20,298 → 16,716` | `14,925` |
| 500 | `43.634 / 46.913` | `39.642 / 40.804` | `6.953 / 7.408` | `5.225 / 5.381` | `50,898 → 41,916` | `37,425` |
| 1,000 | `86.931 / 89.420` | `79.374 / 81.198` | `13.828 / 14.118` | `10.393 / 10.655` | `101,898 → 83,916` | `74,925` |

The number of prepared rows fell by `17.65%`, while the number of solved scalar rows and projection ordering stayed the same. Row-preparation p50 fell by `22–25%`; complete-step p50 fell by `5.8–9.2%` (`0.49 / 1.24 / 3.99 / 7.56 ms`). At 100 parts the p95 step time is within run noise; larger sizes improve by `1.54 / 6.11 / 8.22 ms` at p95. The scalar strict-rocket geometric failure remains unchanged, so this optimization improves cost without claiming solver convergence or fixing that correctness limit.

### Effective-mass cache and FixedBlock3

The per-body `SolverMassSlot` cache is now the production default. It retains the inverse local inertia across position changes, refreshes world inverse inertia when orientation changes, and recomputes the world center when the transform changes. `MassProperties` and the full transform are compared on resolve, so an inertia-only or mass-only edit invalidates the cache. The solver also invalidates pose-derived values after an applied position correction. The retained workspace grows by `39 / 77 / 192 / 383 KiB` at 100/200/500/1,000 bodies; steady samples still allocate `0/0` times/bytes after warm-up.

| Parts | Scalar without cache p50/p95, ms | Scalar with cache p50/p95, ms | Cached FixedBlock3 p50/p95, ms |
|---:|---:|---:|---:|
| 100 | `7.810 / 8.024` | `6.455 / 6.532` | `5.426 / 5.464` |
| 200 | `15.784 / 17.863` | `13.004 / 13.199` | `10.869 / 11.053` |
| 500 | `39.588 / 41.106` | `32.579 / 33.207` | `27.308 / 27.568` |
| 1,000 | `79.664 / 83.074` | `66.155 / 69.001` | `55.275 / 56.218` |

The cache reduces full-step p50 by `17%` and p95 by `17–26%`. The prepared 3×3 FixedBlock3 path adds another `16%` over cached Scalar and uses the existing symmetric effective-mass block solve. In the strict rocket comparison it is about `17–20%` faster, but does not meet geometry limits: its maximum anchor errors are `0.1108 / 0.2859 / 0.7638 / 2.4983 m`, and maximum frame errors are `1.7293 / 1.4835 / 2.8749 / 3.1405 rad`. It remains experimental; the measured speedup does not justify changing the default projection while the strict fixture fails. Cached Scalar reproduced the uncached strict-rocket residuals at all sizes. Warm-start remains disabled because no end-to-end gain has been established.

### Whole-island constraint coloring

The experimental parallel solver now greedily colors the complete island constraint graph, rather than exposing parallelism only across dependency levels. Contacts and joints use the same dynamic-body conflict graph. A color group writes directly to disjoint dynamic `SolverBody` entries; static and kinematic entries are read-only. The solver validates dynamic-body ownership before entering the shared parallel path, joins each group before advancing, and keeps deterministic source order within each color. Small groups remain sequential. This removes per-task body copies and ordered body writeback. A failing executor test confirms that a partially processed direct-write group is rolled back to the original body state.

The 200-body mixed contact/joint fixture builds two colors and is repeatable across parallel runs. It also shows the numerical tradeoff from changing the Gauss–Seidel order: after two steps the scalar and colored maximum anchor errors are `0.10930 m` and `0.10793 m`, while maximum frame errors are `0.13557 rad` and `0.268998 rad`. In the six-step chain/branch diagnostic, the colored path required many more outer sweeps and still did not match scalar geometry; for example, the chain at `32` velocity and `8` position sweeps remains at `0.21994 m / 0.45701 rad`, versus `0.48699 m / 0.40625 rad` for scalar at `8 / 3`. The global-color route is therefore compiled only with `bench-internals` and is not selected in production. This verifies memory safety, rollback, and repeatability, but does not establish a production-quality convergence or scaling win.

Constraint-color graph construction measured `6–38 μs` cold and `1–3 μs` for a warm topology check in 200–1,000-body cases. The benchmark uses a scoped-thread executor, so task launch and synchronization are included in the end-to-end result; it does not establish persistent-pool utilization. The PCG chain benchmark also did not scale with worker count. Reusing diagnostic velocity-snapshot scratch removes steady-state diagnostic allocations (`0/0` p50/p95) at the cost of retaining 48 bytes per body; this scratch is compiled only with `bench-internals`.

## Classification and remaining limits

- **Proven integrator defect:** explicit Euler gyroscopic acceleration on a free asymmetric body. Fixed by the midpoint gyroscopic update; energy and angular-momentum errors now decrease with timestep.
- **Proven metric defect:** arbitrary gravitational-potential reference corrupted coast percent drift. Fixed with the physical zero-at-infinity potential and signed scale normalization.
- **Physically admissible split-solver dissipation:** post-integration `Jv` is projected out; the correction-work identity closes to roundoff. Halving `dt` reduces the one-joint mismatch and loss. Do not inject compensating energy into Velocity PCG.
- **Numerical solver defect:** scalar velocity sweeps under-converge on a long connected chain, leaving a large `Jv` residual and causing excess projection loss. IC(0) PCG removes that excess in the diagnostic, but its strict rocket pass is expensive and has frequent capped solves.
- **Position correction:** measured position-stage energy changes are much smaller; post-position residuals stay close to their post-velocity values in the tested fixtures. A fully coupled constrained midpoint integrator is not implemented, so the independent predict/project split still incurs timestep-dependent projection dissipation.
- **Production status:** scalar strict rocket geometry remains a failure. No new solver path is selected as a default. Performance, contacts, and broader load cases still require optimization and validation before production integration.

## Commands used

```powershell
cargo +1.98.0 test --release -p kapitsa-core --lib rigid_body::integrator::tests::free_asymmetric_rotation_preserves_energy_and_angular_momentum --locked -- --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa-core --features bench-internals --lib joint::stability_tests::diagnose_free_asymmetric_fixed_joint_velocity_projection_work --locked -- --ignored --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa --features bench-internals --lib runtime::modular_rocket_fixture::stability_tests::global_pcg_free_rigid_rocket_assembly_limits_kinetic_energy_drift --locked -- --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa --features bench-internals --lib runtime::modular_rocket_fixture::stability_tests::free_rigid_rocket_assembly_tracks_its_equivalent_single_body --locked -- --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa --features bench-internals --lib runtime::modular_rocket_fixture::stability_tests::diagnose_rocket_unforced_contact_free_energy_stages --locked -- --ignored --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa --features bench-internals --lib runtime::modular_rocket_fixture::stability_tests::diagnose_rocket_global_constraint_pcg_full_energy_trace --locked -- --ignored --exact --nocapture --test-threads=1
cargo +1.98.0 bench -p kapitsa --features bench-internals --bench large_modular_rocket --locked
```

## Final validation

| Check | Result |
|---|---|
| `cargo +1.98.0 fmt --all -- --check` | PASS |
| `cargo +1.98.0 check --workspace --all-features --locked` | PASS |
| `cargo +1.98.0 clippy --workspace --all-features --locked -- -D warnings` | PASS |
| `cargo +1.98.0 test --workspace --locked --no-fail-fast -- --test-threads=1` | PASS; all executed suites and doctests passed, 4 ignored |
| Focused Release energy and projection diagnostics | PASS |
| Same-run full-rocket physical vs legacy surface-zero coast metric | PASS; `4.598934067e−7%` vs `88.94944039%` |
| Strict rocket, default scalar path | FAIL on existing geometric criteria |
| Strict rocket, benchmark-only PCG 5/5, 16 outer velocity sweeps | PASS at 100/200/500/1,000 parts |
