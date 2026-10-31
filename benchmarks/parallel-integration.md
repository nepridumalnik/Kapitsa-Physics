# Parallel Integration Benchmarks

Measured with `cargo bench -p kapitsa-tokio --bench parallel_step` on
2026-10-31. Each value is the median of five runs of 12 steps; world setup is
excluded from timing. The benchmark compares synchronous stepping, the Tokio
one-worker fast path, the previous static sharding layout, and dynamic buckets
with two workers.

## All-Dynamic Worlds

| Bodies | Synchronous | Tokio fast path | Static shards x2 | Dynamic buckets x2 |
|---:|---:|---:|---:|---:|
| 256 | 1.26 ms | 1.49 ms | 2.05 ms | 1.59 ms |
| 4,096 | 16.92 ms | 18.02 ms | 16.36 ms | 13.47 ms |
| 20,000 | 82.49 ms | 83.25 ms | 92.61 ms | 66.04 ms |

## Mixed 20,000-Body Worlds

| Mix | Synchronous | Tokio fast path | Static shards x2 | Dynamic buckets x2 |
|---|---:|---:|---:|---:|
| 50% static, 25% sleeping, 25% dynamic | 23.59 ms | 26.94 ms | 73.85 ms | 26.30 ms |
| 10% static, 80% sleeping, 10% dynamic | 12.05 ms | 14.13 ms | 66.37 ms | 14.38 ms |

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
| Dynamic | 9.71 | 0.11 | 50.08 | 9.17 |
| Mixed | 11.13 | 0.13 | 16.16 | 9.81 |
| Sleeping-heavy | 9.86 | 0.12 | 12.49 | 7.81 |

## Physics Marshalling

Measured with the same release command. These rows use dynamic worlds, 12
steps, and medians of five samples. The simulation timer is paused and advanced
by Tokio, so the API totals contain command/step/snapshot work without real
60 Hz waiting. Each simulation API sample completed exactly 12 successful
steps. Isolated phase rows call the same command handler and snapshot builder
used by the simulation; command timing excludes queue and oneshot overhead.

| Bodies | Controller wall | Simulation, no commands | Simulation, 100 `ApplyForce` per step |
|---:|---:|---:|---:|
| 256 | 1.23 ms | 1.36 ms | 1.79 ms |
| 4,096 | 17.00 ms | 18.29 ms | 19.56 ms |
| 20,000 | 83.03 ms | 95.94 ms | 95.45 ms |

The isolated pipeline phases below are median milliseconds per 12 steps.
Snapshot reads are 4 clients x 1,000 calls against one immutable snapshot.

| Bodies | Commands | Controller | Snapshot build | Watch publication | Pipeline total | 4-client snapshot reads |
|---:|---:|---:|---:|---:|---:|---:|
| 256, with 100 commands | 0.020 | 1.08 | 0.06 | 0.004 | 1.25 | 0.028 |
| 256, no commands | — | 1.14 | 0.09 | 0.006 | 1.33 | 0.028 |
| 4,096, with 100 commands | 0.033 | 17.23 | 1.35 | 0.305 | 19.13 | 0.028 |
| 4,096, no commands | — | 16.99 | 0.87 | 0.009 | 18.01 | 0.028 |
| 20,000, with 100 commands | 0.132 | 82.22 | 9.06 | 0.782 | 92.36 | 0.028 |
| 20,000, no commands | — | 82.12 | 9.36 | 0.912 | 92.58 | 0.028 |

At 20,000 bodies, building the immutable snapshot takes about 9 ms per 12
steps, roughly 10% of the measured pipeline time. Watch publication and
multi-client reads are each below 1 ms for the reported batch. No optimization
was made from this single workload measurement; snapshot rebuilding is the
main measurable marshalling cost at this size.

These measurements are specific to this machine, runtime, and workload; use the
benchmark for comparisons rather than treating the timings as guarantees.
