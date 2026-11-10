# Colored Solver Performance Audit

## Result

The safe colored solver remains numerically safe and lock-free, and no new
`unsafe` code was introduced. The measured dominant cost is constraint
processing/solver iterations, not safe body binding. The largest contacts-only
case is also limited by aggregate callback CPU time and does not scale
linearly from four to eight workers.

An experiment combined colored phase metadata scans into one pass. It was
reverted because a reproducible full-step improvement was not established.
No new solver optimization is retained from this experiment.

Parity with the historical unsafe implementation is not established for all
fixtures. The current evidence is sufficient to reject the hypothesis that
safe binding is the primary cause of the contacts-only full-step cost.

## Benchmark table

Measurements are release benchmark runs on the same fixture. Values are
full-step p50/p95 in milliseconds. Historical unsafe and safe-baseline values
are retained from the earlier audit run and are not re-instrumented here.

| Scenario | Workers | Unsafe p50/p95 | Safe baseline | Before | After | Improvement |
|---|---:|---:|---:|---:|---:|---:|
| Joints-only 10000 | 2 | N/A | N/A | 117.540/124.184 | N/A | not measured after |
| Joints-only 10000 | 4 | N/A | N/A | 91.823/96.013 | N/A | not measured after |
| Joints-only 10000 | 8 | historical safe ~3.6–5.3% over unsafe | N/A | 80.451/90.301 | N/A | not measured after |
| Contacts-only 10000 | 2 | N/A | N/A | 525.435/575.206 | N/A | not measured after |
| Contacts-only 10000 | 4 | N/A | N/A | 465.245/537.318 | N/A | not measured after |
| Contacts-only 10000 | 8 | N/A | N/A | 463.767/522.324 | N/A | not measured after |
| Mixed 10000 | 2 | N/A | N/A | 419.171/454.126 | N/A | not measured after |
| Mixed 10000 | 4 | N/A | N/A | 359.544/445.859 | N/A | not measured after |
| Mixed 10000 | 8 | N/A | N/A | 348.860/397.984 | N/A | not measured after |

The benchmark output also covers 128, 512, 1000, and 5000 constraints, all
three workload types, workers 2/4/8, and scalar worker-count-one samples.
The metadata optimization experiment was reverted because its recorded run
was not a controlled before/after comparison.

## Timing breakdown

Detailed current-safe timing for 10000 constraints and eight workers:

| Scenario | Full step p50/p95 | Binding | Solver iterations | Callback processing sum | Rebuild/cache |
|---|---:|---:|---:|---:|---|
| Joints-only | 100.793/110.737 | 3.573/4.439 | 89.733/100.078 | 136.462/147.990 | 0/40 |
| Contacts-only | 345.640/435.228 | 4.631/5.723 | 59.023/63.566 | 267.082/270.120 | 1/39 |
| Mixed | 269.879/293.024 | 5.330/6.517 | 113.051/122.917 | 261.349/284.896 | 40/0 |

`processing_sum` is aggregate CPU duration across callbacks and must not be
added to wall-clock timings. `solver_iterations` is nested inside the solver
interval. Spatial, broad-phase, narrow-phase, CCD, island-build, prepare, and
writeback metrics are reported independently for the same reason.

For contacts-only 10000/8, spatial/broad/narrow p50 values were approximately
7.889/5.251/6.546 ms, while solver iterations were 59.023 ms. Binding was
approximately 4.631 ms. Retained and peak scratch were both about 153 MB and
capacity growth was zero. This rules out binding as the dominant full-step
cost and shows stable steady-state scratch capacity.

## Instrumentation

`bench-detailed-timing = ["bench-internals"]` gates per-chunk timers and
callback atomics. Ordinary `bench-internals` omits those diagnostics and
prints `detailed=N/A`; it does not report disabled timings as zero.

The detailed contacts-only 10000/8 run was about 345.640 ms p50 versus the
ordinary run about 463.767 ms p50 in separate sequential executions. This
difference is too large to treat as a clean instrumentation-overhead estimate:
the runs show substantial machine/fixture variance and must be repeated under
tighter environmental control before drawing a percentage conclusion.

## Implemented changes

| Commit | Function | Change | Complexity | Effect |
|---|---|---|---|---|
| `1fc2ff6` | colored diagnostics | Gate detailed timers and atomics behind feature | same algorithm, less normal-mode instrumentation | normal mode no longer pays detailed timing cost |
| `82ec5ba` | benchmark reporting | Print `N/A` when constraint timing is disabled | O(1) output change | removes misleading zero metrics |
| `80e643d` | scalar benchmark | Use 40 samples and p50/p95 | one sampled loop instead of one step | comparable scalar data |
| `9782fcd` | `execute_colored_phase` | Revert unconfirmed metadata-scan optimization | baseline restored | no claim made without controlled evidence |

## Rejected or unresolved hypotheses

- **Safe binding as the main contacts-only hotspot:** rejected by measured
  binding time of roughly 4–5 ms versus hundreds of milliseconds full step.
- **Capacity growth as the steady-state cause:** not supported; contacts-only
  10000 reported zero logical capacity growth and stable scratch capacity.
- **Allocator instrumentation in production:** rejected because accurate
  attribution would require unsafe global allocator machinery.
- **Historical internal phase comparison:** unavailable because the unsafe and
  safe-baseline worktrees predate the current binding and coloring APIs.
- **Metadata scan optimization as proven win:** not yet proven independently;
  the before/after runs were affected by large run-to-run variance.

## Checks

Passed during this cycle:

```text
cargo fmt --all
cargo check -p kapitsa-core --bench colored_solver_scaling --features bench-internals
cargo check -p kapitsa-core --bench colored_solver_scaling --features bench-detailed-timing
cargo test -p kapitsa-core --lib collision::tests --features bench-internals
git diff --check
```

The targeted collision suite passed 35 tests. A longer full library test run
exceeded the command timeout and reported one unrelated/diagnostic parallel
velocity-scaling failure before continuing with long-running tests; it needs a
dedicated rerun before claiming full workspace validation.

## Remaining work

1. Repeat the ordinary before/after benchmark with CPU affinity and alternating
   runs to establish whether `c71d05d` is a reproducible win.
2. Rerun the full workspace test and strict clippy checks.
3. Repeat historical unsafe/safe-baseline full-step comparisons under the same
   release settings.
4. Profile contacts-only callback processing externally, if a profiler is
   available, to distinguish memory bandwidth, cache behavior, and executor
   scheduling from solver arithmetic.
5. Investigate why mixed fixtures rebuild coloring every step; this is a
   topology/fixture issue rather than a safe-binding issue.
