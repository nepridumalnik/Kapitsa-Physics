# Parallel Integration Benchmarks

Measured with `cargo bench -p kapitsa-tokio --bench parallel_step` on
2026-10-31. Each value is the median of five runs of 12 steps; world setup is
excluded from timing. The benchmark compares synchronous stepping, the Tokio
one-worker fast path, the previous static sharding layout, and dynamic buckets
with two workers.

## All-Dynamic Worlds

| Bodies | Synchronous | Tokio fast path | Static shards x2 | Dynamic buckets x2 |
|---:|---:|---:|---:|---:|
| 256 | 1.01 ms | 1.29 ms | 1.88 ms | 1.51 ms |
| 4,096 | 17.03 ms | 17.23 ms | 17.57 ms | 12.46 ms |
| 20,000 | 81.40 ms | 82.02 ms | 89.15 ms | 62.76 ms |

## Mixed 20,000-Body Worlds

| Mix | Synchronous | Tokio fast path | Static shards x2 | Dynamic buckets x2 |
|---|---:|---:|---:|---:|
| 50% static, 25% sleeping, 25% dynamic | 23.28 ms | 26.99 ms | 73.64 ms | 33.94 ms |
| 10% static, 80% sleeping, 10% dynamic | 11.86 ms | 14.46 ms | 61.86 ms | 26.56 ms |

Dynamic buckets beat static sharding in each measured profile. They are faster
than synchronous stepping for the larger all-dynamic worlds, but not for the
small world or the mixed and sleeping-heavy cases, where the one-worker path is
cheaper. The adaptive controller starts with one worker and only keeps worker
increases when measured full-step time improves by at least 5%.

These measurements are specific to this machine, runtime, and workload; use the
benchmark for comparisons rather than treating the timings as guarantees.
