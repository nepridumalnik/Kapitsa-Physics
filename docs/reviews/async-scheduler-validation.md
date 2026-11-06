# Adaptive execution validation - 2026-11-06

Branch: `feat/async_scheduler`, baseline: `5b74893`.
Windows, 24 available logical processors, rustc 1.98.0.
No physics tolerances, solver iteration counts or solver selection rules changed.
No benchmark environment overrides were set. Benchmark processes ran sequentially.

## Checks

| Check | Result |
| --- | --- |
| Release workspace, all features, serial tests | 1020 passed, 0 failed, 50 explicitly ignored |
| Added gravity error/rollback/retry test, Release | 1 passed (added after the full run) |
| Cargo bench workspace, all features, all bench targets | All 9 binaries completed, exit 0 |
| Ignored tests matching `benchmark_`, Release, all features | All 12 passed |
| Ignored constrained parallel scaling test | Passed |
| Clippy workspace/all targets/all features, `-D warnings` | Passed |
| `cargo fmt --all -- --check`, `git diff --check` | Passed |

The full suite includes articulated 100/1000-body stress tests at their existing
limits. New tests cover nested concurrency limits 1/2/3/4, actual overlapping
branches, borrowed results, ordered chunk indices, panic permit recovery,
worker doubling/capping/halving, and bitwise state equality over 120 gravity and
force-source steps while changing worker counts. Gravity failures preserve all
original bodies and allow a retry on both sequential and sharded paths.
Remaining ignored exploratory solver diagnostics were not enabled as a group.
GUI/manual verification was not needed or performed.

## Production executor measurements

`cargo bench -p kapitsa --bench parallel_step`, without diagnostic features.
Median ms/step, five samples after one warm-up; fixed eight-thread pool, active
worker limits below. These are end-to-end scene costs, not isolated task timings.

| Scene | 1 worker | 2 workers | 4 workers | 8 workers | 1 to 8 speedup |
| --- | ---: | ---: | ---: | ---: | ---: |
| 4096 bodies / 2048 contact islands | 37.574 | 27.114 | 18.902 | 14.841 | 2.53x |
| 20000 bodies / gravity / sharded prediction | 18.314 | 15.655 | 14.139 | 11.076 | 1.65x |

The gravity scene still exceeds the example 8.33 ms budget at eight workers.
The controller adapts concurrency; it cannot guarantee a deadline for every scene.
The direct non-sharded free-body benchmark remains mostly sequential by design;
the runtime uses the existing sharded path for predominantly active bodies.

For the existing constrained-solver benchmark (with diagnostic features), eight
independent 100-body islands improve from 8.084 to 2.694 ms (3.00x). One connected
1000-body island remains about 11.5 ms regardless of worker count. The scheduler
does not replace the solver to accelerate this sequential work.

An initial baseline parallel-step log is retained, but it used the previous
benchmark-only executor; subsequent absolute timings also vary with host load.
The table reports within-run scaling of the production executor, not a claim
that every scene is faster than the baseline commit. Diagnostic-feature timings
are recorded separately and should not be compared directly with the table.
Existing large-island solver scheduling can produce different fingerprints at
different worker counts; bitwise equality is asserted for the independent paths
covered by tests, not claimed for every connected contact graph.

## Commands and logs

```text
cargo test --workspace --all-features --release --no-fail-fast -- --test-threads=1
cargo test -p kapitsa --release --all-features gravity_task_failure_rolls_back_sources_and_allows_retry -- --test-threads=1
cargo bench --workspace --all-features --benches --no-fail-fast
cargo test --workspace --all-features --release benchmark_ --no-fail-fast -- --ignored --test-threads=1 --nocapture
cargo test -p kapitsa-core --release --all-features constrained_parallel_scaling_and_single_island_cost -- --ignored --test-threads=1 --nocapture
cargo bench -p kapitsa --bench parallel_step
cargo clippy --workspace --all-features --all-targets -- -D warnings
```

Local raw logs: `target/scheduler-validation/` (`workspace-tests.log`,
`gravity-rollback.log`, `all-benches.log`, `ignored-benchmarks.log`,
`constrained-scaling.log`, `parallel-before.log`, `parallel-after.log`,
`clippy-final.log`). The original full test command was redirected through
Windows PowerShell, whose native stderr handling returned a shell status of 1
for build diagnostics despite every test target reporting success; the later
commands explicitly propagate Cargo's exit code.
