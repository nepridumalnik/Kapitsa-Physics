# Large modular rocket: performance observation

Run on 2026-11-02, AMD EPYC 9V74, 5 available logical CPUs, Linux x86_64, Rust 1.98.0. Command: `cargo +1.98.0 bench -p kapitsa --features bench-internals --bench large_modular_rocket --locked --offline`. The run was launched at `cf28111`, before subsequent refactorings and runtime optimizations; event documentation was edited while the binary was compiling. It used the corrected pad-start fixture. 60 steady samples followed 5 warmups; step size 1/120 s. It is a single run, not a before/after experiment.

Other tests and builds ran concurrently. One sample in the 500-body, 2-worker group was suspended while tests ran, yielding an invalid 12.97-second maximum and affecting its mean and potentially tail quantiles. The reported numbers should be treated as approximate observations, not controlled performance comparisons. Eight workers also exceed the five CPUs available. Baseline and final changes were not compared; the speed of snapshot and event changes was not measured.

| Parts | Workers | Step p50, ms | Step p95, ms | Solver iteration p50, ms | Allocations p50/p95 |
|---:|---:|---:|---:|---:|---:|
| 100 | 1 | 12.1807 | 13.3541 | 12.024 | 0/0 |
| 100 | 2 | 11.9417 | 14.3173 | 11.699 | 0/0 |
| 100 | 4 | 11.7491 | 12.5264 | 11.584 | 0/0 |
| 100 | 8 | 12.3099 | 13.9334 | 12.128 | 0/0 |
| 200 | 1 | 24.0850 | 26.3361 | 23.774 | 0/0 |
| 200 | 2 | 24.1311 | 29.9548 | 23.741 | 0/0 |
| 200 | 4 | 27.1564 | 34.6052 | 26.741 | 0/0 |
| 200 | 8 | 25.7023 | 33.5980 | 25.389 | 0/0 |
| 500 | 1 | 64.8374 | 81.9361 | 64.123 | 0/0 |
| 500 | 2 | 60.3797 | 76.8205 | 59.684 | 0/0 |
| 500 | 4 | 59.0660 | 72.7315 | 58.406 | 0/0 |
| 500 | 8 | 59.9173 | 64.2863 | 59.239 | 0/0 |
| 1000 | 1 | 124.8703 | 148.1424 | 123.579 | 0/0 |
| 1000 | 2 | 120.8739 | 140.6183 | 119.594 | 0/0 |
| 1000 | 4 | 124.7779 | 141.3189 | 123.267 | 0/0 |
| 1000 | 8 | 124.8306 | 160.4261 | 123.492 | 0/0 |

The 8.33 ms nominal step budget is exceeded at every measured size. One-worker median solver iteration time is dominant relative to other reported phases (see table). One large island remains predominantly sequential. Steady-state samples reported zero allocator calls at p50 and p95; this does not mean the whole application is allocation-free. Separation creates new islands and does allocate. The timing does not establish physical correctness: the separately run strict rocket scenario failed its joint-error limits.

## Controlled comparison update (2026-11-03)

The earlier EPYC run above is a historical, contended observation. The following controlled Release run was taken on the same Windows/Ryzen 9 7900X3D machine and Rust 1.98.0 as [`joint-solver-baseline-local.md`](joint-solver-baseline-local.md), with no concurrent heavy workload. It used 5 warm-ups and 60 steady samples at `dt=1/120 s`. The old baseline is revision `1e083a51`; current results are from the level-ordered/coloring solver checkpoint and include the current production scalar settings.

| Parts | Original 1 worker p50/p95, ms | Current 1 worker p50/p95, ms | Current 8 workers p50/p95, ms |
|---:|---:|---:|---:|
| 100 | 5.582 / 5.633 | 8.289 / 8.377 | 8.233 / 8.662 |
| 200 | 11.293 / 12.089 | 16.464 / 16.627 | 16.545 / 17.381 |
| 500 | 28.416 / 30.852 | 41.731 / 42.251 | 42.343 / 44.163 |
| 1,000 | 56.540 / 57.065 | 83.349 / 84.523 | 85.739 / 87.599 |

Current one-worker p50 is 46–49% slower than the original baseline. Eight workers do not improve the connected rocket benchmark. The coloring path measured 6–38 μs cold graph preparation and 1–3 μs warm topology checks on 200–1,000-body cases. Those preparation costs are small relative to full-step cost, but scoped-thread launch/synchronization eliminates the observed end-to-end chain speedup. The separate branch fixture improves 16% at 500 bodies with two workers and 23% at 1,000 bodies with four; that does not transfer to the chain-shaped rocket.

The strict rocket is a different workload from the steady benchmark. Its minimum tested passing configuration uses benchmark-only IC(0) Global PCG caps 5/5 and 16 outer velocity sweeps:

| Parts | Scalar strict run, ms | PCG 5/5 strict run, ms | PCG / scalar |
|---:|---:|---:|---:|
| 100 | 1,466 | 2,283 | 1.56x |
| 200 | 3,008 | 6,135 | 2.04x |
| 500 | 7,451 | 29,114 | 3.91x |
| 1,000 | 15,076 | 81,353 | 5.39x |

PCG 5/5 passes the unchanged strict geometry, coast-energy, altitude, and completion limits at all four sizes. The scalar path fails strict geometry at every size. PCG local iteration limits remain frequent; therefore the strict pass is not evidence that all local systems converged. Since it increases run time substantially and does not meet the performance objective, PCG remains benchmark-only. The 48-byte-per-body reusable energy-diagnostic snapshot is compiled only under `bench-internals`; the current warmed benchmark reports zero diagnostic allocations, which is not a production allocation claim.
