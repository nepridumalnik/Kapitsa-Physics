# Joint Solver energy and parallel results

Date: 2026-11-03

Branch: `audit/physics-remediation`

Toolchain: Rust 1.98.0, Windows x86_64 MSVC, Ryzen 9 7900X3D (12 cores / 24 logical processors)

## Result

The large coast-energy failure was primarily a metric error: the rocket's gravitational potential used an arbitrary surface-relative zero, so its percent drift depended on that offset. The metric now uses `U = -μm/r` (zero at infinity) and normalizes signed drift by the initial `|T| + |R| + |U|` scale. The full-flight coast sample starts after upper-stage cutoff and stops at the first upper-stage surface contact, so impact dissipation is not labeled as coast drift. A separate integrator defect was proven for torque-free asymmetric rotation: explicit Euler gyroscopic acceleration caused large energy drift; a midpoint update reduces it by orders of magnitude.

For a constrained assembly, most of the remaining energy change occurs during velocity projection, after independent body predictions leave nonzero `Jv`. The measured kinetic-energy change matches the work of the applied correction impulses to roundoff. Reducing `dt` reduces the single-joint mismatch and projection loss. On a large chain, Scalar Gauss–Seidel additionally stops with a large residual and removes much more energy than the converged PCG projection. Position correction changes the measured `Jv` only slightly in these fixtures.

The strict 100/200/500/1,000-part rocket fixture passes with benchmark-only velocity/position PCG caps `5/5` and 16 outer velocity sweeps. Caps 1–4 fail at 1,000 parts. The production scalar configuration still fails the strict geometric limits, while the passing PCG configuration is substantially slower and many local PCG calls hit their cap. PCG remains experimental; no solver defaults or acceptance tolerances were changed.

The Scalar full-step regression was traced to repeated six-row velocity preparation across sweeps. Reusing the prepared rows cuts p50 by `39–51%` on the 100–1,000-body chain benchmark while preserving its final `Jv` and geometry. A per-row inverse effective-mass cache is also reused while both body mass slots remain valid; mass, inertia, and transform changes invalidate it. The production Scalar full rocket path now measures `4.57 / 9.14 / 22.95 / 46.52 ms` p50 at 100/200/500/1,000 parts in the no-pool control.

The worker scaling requirement did not produce a safe production parallel win. Persistent Rayon measurements show that the experimental whole-island color path slows the optimized Scalar chain, and the rocket's 1,000-part colored cases double row preparation and are slower than the sequential control. PCG remains accurate but gains no speed from additional workers; IC(0) setup still dominates the strict rocket run. Both paths remain experimental where the measurements do not justify enabling them.

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
| 100 | 1,980 | `0.000347 / 0.000034` | `0.000691 / 0.000035` | `−4.78e−7%` | 2.310 | `2,744 / 2,152` |
| 200 | 5,473 | `0.001417 / 0.000150` | `0.001380 / 0.000178` | `−4.76e−7%` | 2.192 | `2,315 / 2,581` |
| 500 | 28,484 | `0.009504 / 0.001406` | `0.007923 / 0.001031` | `−1.77e−5%` | 1.834 | `1,513 / 3,271` |
| 1,000 | 78,702 | `0.026590 / 0.001923` | `0.010279 / 0.001075` | `−4.28e−7%` | 1.613 | `2,306 / 2,590` |

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

An instrumented 180-tick 1,000-body position-PCG run attributed `38,235 ms` to system preparation (including IC(0) construction), `172 ms` to matrix-vector products, `1,841 ms` to preconditioner applications, and `13 ms` to reductions. IC(0) setup dominates. The final strict run at 1,000 bodies took `78,702 ms`; this is not a PCG-only A/B measurement.

The final strict run's convergence and geometry results are the same as the table above. A same-step IC(0) reuse trial kept the previous position factor across outer position passes while rebuilding the current masses, diagonal, residual, and matrix-vector operator. It cut the 1,000-body strict run from `78,333` to `53,336 ms` (`31.9%`), and the deterministic fixture still passed its broad limits, but maximum frame error rose from `0.010279` to `0.081698 rad` and RMS from `0.001075` to `0.003627 rad`. At 500 bodies, maximum frame error rose from `0.007923` to `0.011054 rad`. The factor is mathematically positive definite, but its stale approximation materially degraded convergence at the fixed cap, so the experiment was reverted and is not included in the solver.

Alternative position preconditioners were screened at 1,000 bodies, 16 external sweeps, and cap 5. Diagonal PCG took `43,364 ms` but ended at `0.216 m / 0.782 rad`; one-joint block Jacobi took `42,559 ms` and ended at `0.121 m / 0.400 rad`. A two-joint block test took `42,108 ms` and ended at `0.139 m / 0.361 rad`; increasing its position cap to 16 reduced the anchor error to `0.0480 m` but frame error remained `0.208 rad`. These faster variants fail strict geometry and are not retained as selectable solver paths. IC(0) remains the only screened position preconditioner that passes.

## Release performance and parallelism

The large modular rocket benchmark uses 5 warm-up and 60 steady samples at `dt=1/120 s`. It now defaults to a persistent Rayon pool, built outside the timed samples and reused for every size at that worker count. `--no-pool` runs a separate `SequentialExecutor` control at one worker. The historical baseline was recorded before the executor change.

The old comparison showed the regression that motivated this follow-up. Final full-step p50/p95 values are below; the historical original column used the earlier benchmark executor, so the chain A/B in the next section is the clean comparison for solver changes.

| Parts | Historical original, 1 worker | Pre-fix current, 1 worker | Final no pool | Final Rayon 1 | Final Rayon 2 | Final Rayon 4 | Final Rayon 8 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | `5.582 / 5.633` | `8.289 / 8.377` | `4.570 / 4.650` | `4.611 / 4.684` | `4.613 / 4.742` | `4.592 / 4.692` | `4.664 / 5.933` |
| 200 | `11.293 / 12.089` | `16.464 / 16.627` | `9.137 / 9.261` | `9.194 / 9.540` | `9.262 / 9.480` | `9.350 / 10.722` | `9.365 / 9.609` |
| 500 | `28.416 / 30.852` | `41.731 / 42.251` | `22.949 / 23.256` | `23.037 / 24.152` | `23.269 / 24.179` | `23.449 / 25.564` | `23.452 / 25.982` |
| 1,000 | `56.540 / 57.065` | `83.349 / 84.523` | `46.524 / 47.675` | `46.457 / 47.996` | `65.179 / 69.296` | `54.572 / 57.740` | `54.812 / 62.775` |

Times are milliseconds. At 100–500 parts, worker counts change little. At 1,000 parts, 2/4/8 workers trigger 803 color batches and prepare `83,916` rows, twice the `41,958` rows in the uncolored Scalar path. Those colored full steps are slower than no pool. The historical original-to-final table also changes executor; the paired chain measurements below attribute the row-reuse gain without that confound.

### Scalar FixedJoint incremental preparation

To isolate stage 1, the old `c936f44` implementation and the optimized implementation were benchmarked in Release with identical `bench-internals` row-preparation timers, warm-up, sample count, fixture, one worker, and the mass cache disabled in both runs. The detached baseline run completed all four one-worker sizes; later worker-count runs were stopped after capturing the four one-worker comparisons. Times are p50/p95. Each prepared FixedJoint row count includes all constraints in one world step; scalar solved-row counts are unchanged.

| Parts | Full step before, ms | Full step after, ms | Row preparation before, ms | Row preparation after, ms | Prepared rows before → after | Solved rows |
|---:|---:|---:|---:|---:|---:|---:|
| 100 | `8.418 / 8.540` | `7.926 / 8.573` | `1.346 / 1.372` | `1.044 / 1.120` | `10,098 → 8,316` | `7,425` |
| 200 | `17.111 / 18.024` | `15.871 / 16.484` | `2.720 / 2.803` | `2.090 / 2.150` | `20,298 → 16,716` | `14,925` |
| 500 | `43.634 / 46.913` | `39.642 / 40.804` | `6.953 / 7.408` | `5.225 / 5.381` | `50,898 → 41,916` | `37,425` |
| 1,000 | `86.931 / 89.420` | `79.374 / 81.198` | `13.828 / 14.118` | `10.393 / 10.655` | `101,898 → 83,916` | `74,925` |

The first implementation reduced prepared rows by `17.65%` while preserving solved rows and projection order. The follow-up reuses the six-row blocks across all Scalar velocity sweeps when warm-start, alternating sweep order, and PCG are disabled. It also measures the current residual directly from those prepared rows, so diagnostics do not rebuild them. Position projections still prepare rows after each pose update and retain their original ordered sequence.

The following paired persistent-benchmark run compares the earlier path with the final row-reuse path using the same no-pool executor and fixture. Row preparation is p50; full-step values are p50/p95.

| Bodies | Full step before, ms | Full step after, ms | Row preparation before → after, ms | Prepared rows after | p50 saved, ms |
|---:|---:|---:|---:|---:|---:|
| 100 | `4.008 / 4.127` | `2.426 / 2.549` | `0.738 → 0.389` | `4,158` | `1.582` |
| 200 | `6.659 / 7.382` | `3.698 / 4.391` | `1.217 → 0.578` | `8,358` | `2.961` |
| 500 | `15.216 / 15.612` | `7.457 / 8.207` | `2.617 → 1.111` | `20,958` | `7.759` |
| 1,000 | `28.798 / 31.098` | `14.178 / 14.788` | `5.214 → 2.034` | `41,958` | `14.620` |

The p50 reduction is `39.5 / 44.5 / 51.0 / 50.8%`. Scalar still runs eight velocity and three position sweeps. Its measured `Jv` (`2.003e−3 m/s / 3.652e−3 rad/s`) and final geometry (`1.667e−4 m / 3.196e−4 rad`) are unchanged from the earlier run. The strict rocket's Scalar geometric failure therefore remains; the optimization reduces cost without claiming convergence.

### Effective-mass cache and FixedBlock3

`SolverMassSlot` retains inverse local inertia across position changes, refreshes world inverse inertia when orientation changes, and recomputes the world center when the transform changes. It compares mass properties and the full transform, so mass-only, inertia-only, and pose edits invalidate it. The per-row scalar inverse effective mass is reused only when both body slots report a cache hit; otherwise it is recomputed from the refreshed 3×3 inverse-mass/inertia data. Position changes rebuild the rows before the next velocity solve. A focused anisotropic-inertia test rotates one body, observes the cache refresh, and compares the refreshed value with a fresh calculation. The retained workspace grows by `39 / 77 / 192 / 383 KiB` at 100/200/500/1,000 bodies; steady samples still allocate `0/0` times/bytes after warm-up.

Final no-pool p50/p95 comparisons include both per-body mass caching and row effective-mass reuse. `FixedBlock3` uses the prepared symmetric 3×3 block solve, but its earlier speed advantage was measured against Scalar before the row-reuse optimization and no longer holds.

| Parts | Scalar cache off, ms | Scalar cache on, ms | Saved p50 | FixedBlock3, ms |
|---:|---:|---:|---:|---:|
| 100 | `6.014 / 6.350` | `4.570 / 4.650` | `1.443 ms (24.0%)` | `5.308 / 5.432` |
| 200 | `12.137 / 12.447` | `9.137 / 9.261` | `2.999 ms (24.7%)` | `10.642 / 10.937` |
| 500 | `30.588 / 34.393` | `22.949 / 23.256` | `7.639 ms (25.0%)` | `26.798 / 27.400` |
| 1,000 | `61.555 / 65.107` | `46.524 / 47.675` | `15.031 ms (24.4%)` | `54.168 / 55.953` |

The scalar effective-mass cache now wins end-to-end. `FixedBlock3` is `16–17%` slower than this optimized Scalar path and remains experimental. It also fails the strict geometry limits: its maximum anchor errors are `0.1108 / 0.2859 / 0.7638 / 2.4983 m`, and maximum frame errors are `1.7293 / 1.4835 / 2.8749 / 3.1405 rad`. Warm-start remains disabled because no end-to-end gain has been established.

### Whole-island constraint coloring

The experimental parallel solver now greedily colors the complete island constraint graph, rather than exposing parallelism only across dependency levels. Contacts and joints use the same dynamic-body conflict graph. A color group writes directly to disjoint dynamic `SolverBody` entries; static and kinematic entries are read-only. The solver validates dynamic-body ownership before entering the shared parallel path, joins each group before advancing, and keeps deterministic source order within each color. Small groups remain sequential. This removes per-task body copies and ordered body writeback. A failing executor test confirms that a partially processed direct-write group is rolled back to the original body state.

The 200-body mixed contact/joint fixture builds two colors and is repeatable across parallel runs. It also shows the numerical tradeoff from changing the Gauss–Seidel order: after two steps the scalar and colored maximum anchor errors are `0.10930 m` and `0.10793 m`, while maximum frame errors are `0.13557 rad` and `0.268998 rad`. In the six-step chain/branch diagnostic, the colored path required many more outer sweeps and still did not match scalar geometry; for example, the chain at `32` velocity and `8` position sweeps remains at `0.21994 m / 0.45701 rad`, versus `0.48699 m / 0.40625 rad` for scalar at `8 / 3`. The global-color route is therefore compiled only with `bench-internals` and is not selected in production. This verifies memory safety, rollback, and repeatability, but does not establish a production-quality convergence or scaling win.

Constraint-color graph construction measured `6–38 μs` cold and `1–3 μs` for a warm topology check in 200–1,000-body cases. The earlier scoped-thread measurements included task launch and synchronization but did not establish persistent-pool utilization; the dedicated persistent-pool comparison follows below. Reusing diagnostic velocity-snapshot scratch removes steady-state diagnostic allocations (`0/0` p50/p95) at the cost of retaining 48 bytes per body; this scratch is compiled only with `bench-internals`.

### Persistent Rayon scaling benchmark

`joint_parallel_scaling` uses a persistent Rayon pool per worker configuration and a separate `SequentialExecutor` control with no pool. It measures 20 timed steps after three warm-ups on 100/200/500/1,000-body disturbed fixed-joint chains at `dt=1/120 s`. Each step starts with the same terminal velocity kick. Scalar uses eight velocity and three position sweeps; IC(0) PCG uses a `5/5` cap. The full rocket benchmark uses the same shared Rayon executor, with the no-pool control run separately. Pool construction is outside timed samples. Times below are p50/p95 milliseconds.

| Bodies | Scalar no pool | Scalar Rayon 1 | Scalar Rayon 2 | Scalar Rayon 4 | Scalar Rayon 8 |
|---:|---:|---:|---:|---:|---:|
| 100 | `2.426 / 2.549` | `2.445 / 2.550` | `2.437 / 2.529` | `2.416 / 2.582` | `2.420 / 2.563` |
| 200 | `3.698 / 4.391` | `3.787 / 4.383` | `6.363 / 6.610` | `5.362 / 5.675` | `6.195 / 6.485` |
| 500 | `7.457 / 8.207` | `7.483 / 8.176` | `13.798 / 15.024` | `11.912 / 13.136` | `12.728 / 13.748` |
| 1,000 | `14.178 / 14.788` | `13.897 / 14.517` | `28.513 / 30.168` | `24.975 / 27.093` | `22.052 / 23.338` |

| Bodies | PCG no pool | PCG Rayon 1 | PCG Rayon 2 | PCG Rayon 4 | PCG Rayon 8 |
|---:|---:|---:|---:|---:|---:|
| 100 | `2.578 / 2.613` | `2.571 / 2.616` | `2.770 / 2.924` | `2.970 / 3.118` | `3.538 / 3.939` |
| 200 | `5.185 / 5.255` | `5.167 / 5.204` | `5.523 / 5.875` | `5.552 / 5.869` | `7.140 / 7.729` |
| 500 | `13.409 / 13.503` | `13.317 / 14.490` | `14.124 / 18.887` | `14.549 / 16.853` | `14.859 / 19.672` |
| 1,000 | `27.594 / 28.710` | `27.923 / 29.313` | `29.892 / 31.118` | `28.922 / 29.885` | `30.649 / 35.315` |

The no-pool Scalar/PCG solver-prepare p50 values are `0.004/0.004`, `0.008/0.008`, `0.021/0.022`, and `0.048/0.048 ms`; joint-row preparation is `0.389/0.197`, `0.578/0.404`, `1.111/1.007`, and `2.034/2.138 ms`. PCG reaches convergence in one velocity and one position iteration at every size, with zero capped calls. Its maximum linear/angular velocity residual grows from `3.36e−14 / 3.92e−15` at 100 bodies to `1.96e−10 / 2.10e−11` at 1,000; final anchor/frame errors are at most `1.73e−9 m / 1.59e−14 rad`. Scalar no-pool ends at `2.003e−3 m/s / 3.652e−3 rad/s` measured `Jv`, `1.667e−4 m` maximum anchor error, and `3.196e−4 rad` maximum frame error.

Coloring was not selected for the 100-body cases; its threshold is reached on this chain at 200 bodies.

The persistent pool does not improve PCG. At 1,000 bodies, its best p50 is no pool (`27.594 ms`); eight workers cost `30.649 ms`. The colored Scalar path also loses to the optimized sequential path at every size from 200 bodies onward: at 1,000 bodies, even eight workers cost `22.052 ms` vs `14.178 ms` no pool, while p95 is `23.338` vs `14.788 ms`. The final colored chain frame error is `6.17e−4 rad`, compared with `3.20e−4 rad` sequentially. Coloring removes per-task body copies and serial writeback, and direct-write rollback/determinism tests pass, but its row preparation, synchronization, and changed projection order eliminate the speedup on these connected fixtures. It remains bench-only.

The measured cost contributions are listed separately; the effective-mass comparison is the current no-pool cache-on/off A/B. FixedBlock3's negative saving means it is slower than final Scalar.

| Bodies | Row reuse: preparation saved | Row reuse: full step saved | Mass/effective cache: full step saved | FixedBlock3 delta vs Scalar |
|---:|---:|---:|---:|---:|
| 100 | `0.349 ms` | `1.582 ms` | `1.443 ms` | `−0.738 ms` |
| 200 | `0.639 ms` | `2.961 ms` | `2.999 ms` | `−1.505 ms` |
| 500 | `1.506 ms` | `7.759 ms` | `7.639 ms` | `−3.848 ms` |
| 1,000 | `3.180 ms` | `14.620 ms` | `15.031 ms` | `−7.644 ms` |

No separate millisecond win is assigned to coloring: executor, preparation, and ordering costs change together. Rayon pool construction is excluded from timed samples; dispatch and synchronization remain included.

## Classification and remaining limits

- **Proven integrator defect:** explicit Euler gyroscopic acceleration on a free asymmetric body. Fixed by the midpoint gyroscopic update; energy and angular-momentum errors now decrease with timestep.
- **Proven metric defect:** arbitrary gravitational-potential reference corrupted coast percent drift. Fixed with the physical zero-at-infinity potential and signed scale normalization.
- **Physically admissible split-solver dissipation:** post-integration `Jv` is projected out; the correction-work identity closes to roundoff. Halving `dt` reduces the one-joint mismatch and loss. Do not inject compensating energy into Velocity PCG.
- **Numerical solver defect:** scalar velocity sweeps under-converge on a long connected chain, leaving a large `Jv` residual and causing excess projection loss. IC(0) PCG removes that excess in the diagnostic, but its strict rocket pass is expensive and has frequent capped solves.
- **Position correction:** measured position-stage energy changes are much smaller; post-position residuals stay close to their post-velocity values in the tested fixtures. A fully coupled constrained midpoint integrator is not implemented, so the independent predict/project split still incurs timestep-dependent projection dissipation.
- **Production status:** scalar strict rocket geometry remains a failure, despite the measured performance gain. Warm-start, FixedBlock3 projection, whole-island coloring, and PCG remain experimental; none is selected as the default projection. The new row and mass caches preserve the tested Scalar trajectory and are enabled with the production Scalar path.

## Commands used

```powershell
cargo +1.98.0 test --release -p kapitsa-core --lib rigid_body::integrator::tests::free_asymmetric_rotation_preserves_energy_and_angular_momentum --locked -- --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa-core --features bench-internals --lib joint::stability_tests::diagnose_free_asymmetric_fixed_joint_velocity_projection_work --locked -- --ignored --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa --features bench-internals --lib runtime::modular_rocket_fixture::stability_tests::global_pcg_free_rigid_rocket_assembly_limits_kinetic_energy_drift --locked -- --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa --features bench-internals --lib runtime::modular_rocket_fixture::stability_tests::free_rigid_rocket_assembly_tracks_its_equivalent_single_body --locked -- --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa --features bench-internals --lib runtime::modular_rocket_fixture::stability_tests::diagnose_rocket_unforced_contact_free_energy_stages --locked -- --ignored --exact --nocapture --test-threads=1
cargo +1.98.0 test --release -p kapitsa --features bench-internals --lib runtime::modular_rocket_fixture::stability_tests::diagnose_rocket_global_constraint_pcg_full_energy_trace --locked -- --ignored --exact --nocapture --test-threads=1
cargo +1.98.0 bench -p kapitsa --features bench-internals --bench large_modular_rocket --locked
cargo +1.98.0 bench -p kapitsa --features bench-internals --bench large_modular_rocket --locked -- --steady-only
cargo +1.98.0 bench -p kapitsa --features bench-internals --bench large_modular_rocket --locked -- --no-pool --steady-only
cargo +1.98.0 bench -p kapitsa --features bench-internals --bench large_modular_rocket --locked -- --no-pool --no-mass-cache --steady-only
cargo +1.98.0 bench -p kapitsa --features bench-internals --bench large_modular_rocket --locked -- --no-pool --block3 --steady-only
cargo +1.98.0 bench -p kapitsa-core --features bench-internals --bench joint_parallel_scaling --locked
cargo +1.98.0 test --workspace --locked --no-fail-fast -- --test-threads=1
cargo +1.98.0 test --workspace --all-features --locked --no-fail-fast -- --test-threads=1
cargo +1.98.0 check --workspace --all-features --locked
cargo +1.98.0 clippy --workspace --all-features --lib --bins --locked -- -D warnings
cargo +1.98.0 clippy -p kapitsa-core --features bench-internals --bench joint_parallel_scaling --locked -- -D warnings
cargo +1.98.0 clippy -p kapitsa --features bench-internals --bench large_modular_rocket --locked -- -D warnings -A dead_code -A unused_imports
```

## Final validation

| Check | Result |
|---|---|
| `cargo +1.98.0 fmt --all -- --check` | PASS |
| `cargo +1.98.0 check --workspace --all-features --locked` | PASS |
| `cargo +1.98.0 clippy --workspace --all-features --lib --bins --locked -- -D warnings` | PASS |
| New `joint_parallel_scaling` bench Clippy | PASS |
| `large_modular_rocket` bench compile and targeted Clippy | PASS; 26 pre-existing warnings in its included test module are allowed for this target-specific check |
| `cargo +1.98.0 clippy --workspace --all-features --all-targets --locked -- -D warnings` | FAIL: the existing `large_modular_rocket` benchmark compiles a test module with unused imports/dead code under `-D warnings`; 26 warnings, outside this change |
| `cargo +1.98.0 test --workspace --locked --no-fail-fast -- --test-threads=1` | PASS; all executed suites and doctests passed, 4 ignored |
| `cargo +1.98.0 test --workspace --all-features --locked --no-fail-fast -- --test-threads=1` | PASS; all executed suites and doctests passed |
| Stage 4 colored solver and graph tests | PASS; rollback, repeated parallel state, contacts plus joints; 630 core unit tests passed under all features |
| Persistent Rayon chain benchmark | PASS; 40 solver/size/executor cases, 20 measured samples each; includes Scalar, PCG, p50/p95, preparation, iterations, residuals, and geometry |
| Persistent Rayon full rocket benchmark | PASS; 16 worker/size cases plus 4 no-pool controls; 5 warm-ups and 60 samples per case |
| Current Scalar mass-cache A/B | PASS; no-pool cache-on/off at all four sizes; steady samples allocate `0/0` times/bytes |
| Current FixedBlock3 A/B | PASS; slower than optimized Scalar by `0.74 / 1.50 / 3.85 / 7.64 ms` p50, so remains experimental |
| Focused Release energy and projection diagnostics | PASS |
| Same-run full-rocket physical vs legacy surface-zero coast metric | PASS; `4.598934067e−7%` vs `88.94944039%` |
| Strict rocket, default scalar path | FAIL on existing geometric criteria |
| Strict rocket, benchmark-only PCG 5/5, 16 outer velocity sweeps | PASS at 100/200/500/1,000 parts; 180 ticks per run and deterministic repeats |
| Same-step position IC(0) reuse experiment | REJECTED: 31.9% faster at 1,000 parts, but maximum frame error increased 7.9x; implementation reverted |
| Free rigid rocket energy invariant | PASS; Scalar `0.075847%`, IC(0) PCG `0.004841%`, correction-work closure `−4.79e−15 J` |
