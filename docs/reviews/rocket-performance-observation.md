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
