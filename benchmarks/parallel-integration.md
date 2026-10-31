# Parallel Integration Benchmarks

Measured with `cargo bench -p kapitsa-tokio --bench parallel_step` on
2026-10-31. Each value is the median of five runs of 12 steps; world setup is
excluded from timing. The benchmark compares synchronous stepping, the Tokio
one-worker fast path, the previous static sharding layout, and dynamic buckets
with two workers.

## All-Dynamic Worlds

| Bodies | Synchronous | Tokio fast path | Static shards x2 | Dynamic buckets x2 |
|---:|---:|---:|---:|---:|
| 256 | 1.03 ms | 1.30 ms | 1.75 ms | 1.44 ms |
| 4,096 | 16.81 ms | 18.73 ms | 17.96 ms | 13.04 ms |
| 20,000 | 84.99 ms | 85.63 ms | 91.54 ms | 60.74 ms |

## Mixed 20,000-Body Worlds

| Mix | Synchronous | Tokio fast path | Static shards x2 | Dynamic buckets x2 |
|---|---:|---:|---:|---:|
| 50% static, 25% sleeping, 25% dynamic | 24.05 ms | 27.61 ms | 67.35 ms | 32.96 ms |
| 10% static, 80% sleeping, 10% dynamic | 11.87 ms | 12.26 ms | 60.07 ms | 25.04 ms |

Dynamic buckets beat static sharding in each measured profile. They are faster
than synchronous stepping for the larger all-dynamic worlds, but not for the
small world or the mixed and sleeping-heavy cases, where the one-worker path is
cheaper. The adaptive controller starts with one worker and only keeps worker
increases when measured full-step time improves by at least 5%.

These measurements are specific to this machine, runtime, and workload; use the
benchmark for comparisons rather than treating the timings as guarantees.
