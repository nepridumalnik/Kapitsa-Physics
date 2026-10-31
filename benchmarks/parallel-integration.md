# Parallel Integration Benchmarks

Measured with `cargo bench -p kapitsa-tokio --bench parallel_step` on
2026-10-31. Each value is the median of five runs of 12 steps; world setup is
excluded from timing. The benchmark compares synchronous stepping, the Tokio
one-worker fast path, the previous static sharding layout, and dynamic buckets
with two workers.

## All-Dynamic Worlds

| Bodies | Synchronous | Tokio fast path | Static shards x2 | Dynamic buckets x2 |
|---:|---:|---:|---:|---:|
| 256 | 0.95 ms | 1.09 ms | 1.39 ms | 0.95 ms |
| 4,096 | 15.47 ms | 15.66 ms | 15.23 ms | 10.27 ms |
| 20,000 | 75.54 ms | 75.33 ms | 78.12 ms | 48.28 ms |

## Mixed 20,000-Body Worlds

| Mix | Synchronous | Tokio fast path | Static shards x2 | Dynamic buckets x2 |
|---|---:|---:|---:|---:|
| 50% static, 25% sleeping, 25% dynamic | 21.68 ms | 22.89 ms | 58.79 ms | 22.30 ms |
| 10% static, 80% sleeping, 10% dynamic | 10.75 ms | 11.07 ms | 46.83 ms | 11.53 ms |

The two-worker controller selects the sequential blocking path when at least
half the bodies are inactive. In the preceding run, before that selection, its
20,000-body mixed and sleeping-heavy medians were 23.68 ms and 21.25 ms.
The adaptive controller starts with one worker and keeps worker increases only
when measured full-step time improves by at least 5%.

## Raw Bucket Phase Timings

These timings exercise the bucket pipeline directly, including for workloads
that now choose the sequential path. Values are median milliseconds per 12 steps.
Dispatch overlaps worker execution; join and collection therefore measure the
remaining wall-clock wait and result ordering, not summed worker CPU time.

| 20,000-body mix | Prepare snapshots | Dispatch | Join and collect | Commit |
|---|---:|---:|---:|---:|
| Dynamic | 5.01 | 0.06 | 39.83 | 4.02 |
| Mixed | 4.94 | 0.06 | 11.70 | 3.37 |
| Sleeping-heavy | 5.07 | 0.06 | 9.85 | 3.65 |

These measurements are specific to this machine, runtime, and workload; use the
benchmark for comparisons rather than treating the timings as guarantees.
