# Parallel Integration Benchmarks

> Historical measurements from before the unified `kapitsa::Simulation` API.
> The worker comparison still exercises the engine implementation; the public
> integration example is in [`docs/runtime.md`](../docs/runtime.md).

Measured with `cargo bench -p kapitsa --bench parallel_step` on
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

## Physics Marshalling: Previous Baseline

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
was made from this baseline measurement; snapshots used slot-indexed storage
and rebuilt fresh buffers on every step.

## Compact Snapshots and Buffer Recycling

Measured after compact indexed storage and `Arc::try_unwrap` recycling were
added. Each row is the median of five samples of 12 successful controller
steps. The snapshot scenarios also look up every live body once per step;
"full" includes the controller step, snapshot build, those lookups, and watch
publication. The separate API table below measures the simulation API without
that extra all-body lookup loop. The timer is paused in simulation API samples.

### A. Dense snapshots built fresh

| Bodies | Build | Lookup all bodies | Publish | Full step + marshalling | Compact payload | Previous slot-array estimate |
|---:|---:|---:|---:|---:|---:|---:|
| 256 | 0.10 ms | 0.04 ms | 0.002 ms | 1.32 ms | 42.5 KiB | 30.0 KiB |
| 4,096 | 4.08 ms | 0.96 ms | 0.007 ms | 22.33 ms | 680.0 KiB | 480.0 KiB |
| 20,000 | 15.85 ms | 4.39 ms | 0.015 ms | 97.86 ms | 3,172.0 KiB | 2,343.8 KiB |

### B. Sparse slot history

The world starts with 20,000 bodies, then removes 19,900 and keeps the 100
bodies with the highest slot indices.

| Slots in history | Live bodies | Build | Lookup | Publish | Full step + marshalling | Compact payload | Previous slot-array estimate | Reused / reclaimed buffers | Capacity growths after reuse |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20,000 | 100 | 0.04 ms | 0.02 ms | 0.003 ms | 0.66 ms | 15.1 KiB | 2,343.8 KiB | 11 / 12 | 0 |

### C and D. Retained readers and normal publication

Scenario C retains three previous `Arc<PhysicsSnapshot>` values across
publication. Scenario D drops old frames and allows the task to reuse their
buffers.

| Scenario | Bodies | Build | Lookup all bodies | Publish | Full step + marshalling | Compact payload | Reused / reclaimed buffers | Capacity growths after reuse |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| C: retain old Arcs | 256 | 0.15 ms | 0.05 ms | 0.005 ms | 1.39 ms | 42.5 KiB | 0 / 0 | 0 |
| C: retain old Arcs | 4,096 | 2.89 ms | 0.79 ms | 0.208 ms | 19.98 ms | 680.0 KiB | 0 / 0 | 0 |
| C: retain old Arcs | 20,000 | 17.69 ms | 5.25 ms | 1.038 ms | 101.42 ms | 3,172.0 KiB | 0 / 0 | 0 |
| D: normal reuse | 256 | 0.09 ms | 0.04 ms | 0.002 ms | 1.24 ms | 42.5 KiB | 11 / 12 | 0 |
| D: normal reuse | 4,096 | 1.59 ms | 0.81 ms | 0.007 ms | 18.51 ms | 680.0 KiB | 11 / 12 | 0 |
| D: normal reuse | 20,000 | 10.39 ms | 4.47 ms | 0.015 ms | 91.19 ms | 3,172.0 KiB | 11 / 12 | 0 |

In normal publication the first frame seeds the reusable buffer; 11 of the 12
frames reused a prior buffer, and all 12 outgoing frames were successfully
reclaimed. Capacity did not grow after reuse began. Retaining old frames did
not block publication; `Arc::try_unwrap` simply failed and that frame was not
recycled.

The memory values estimate container payload from `Vec` and `HashMap`
capacities; they exclude allocator metadata and hash-table control bytes. The
previous-layout estimate is a slot-indexed `Vec<Option<Entry>>` sized to the
maximum slot. Dense snapshots use about 35–42% more estimated payload because
the full-ID hash index adds storage. For the sparse history, compact storage
uses 15.1 KiB instead of an estimated 2,343.8 KiB because it stores only live
bodies.

The simulation API timings after recycling were:

| Bodies | Controller wall | Simulation, no commands | Simulation, 100 `ApplyForce` per step |
|---:|---:|---:|---:|
| 256 | 1.14 ms | 1.34 ms | 1.70 ms |
| 4,096 | 15.98 ms | 18.10 ms | 18.62 ms |
| 20,000 | 76.42 ms | 89.83 ms | 90.82 ms |

Compared with the previous API table, these dynamic-world medians are similar
or faster; differences include normal run-to-run timing variation. For a
same-run comparison, dense normal reuse reduced the measured full step plus
all-body lookup from 22.33 to 18.51 ms at 4,096 bodies, and from 97.86 to
91.19 ms at 20,000 bodies, relative to fresh snapshot construction.

These measurements are specific to this machine, runtime, and workload; use the
benchmark for comparisons rather than treating the timings as guarantees.

## Adaptive Snapshot Storage Compared with `aeadef`

Measured with the release benchmark on 2026-10-31. Each value is the median of
five samples of 12 successful physics steps. Scenario setup is excluded. The
`aeadef` row uses a benchmark-only reproduction of that commit's compact
`Vec<SnapshotEntry>` plus `HashMap<BodyId, usize>` layout and buffer recycling,
without the later shrink policy. Both layouts run the same world mutations,
1,000 body lookups, complete body iteration, and watch publication per step.
The 1,000 lookup calls cycle through live IDs when a scenario has fewer than
1,000 bodies.

### Timings

Times are milliseconds per 12 steps.

| Scenario | Layout | Rebuild | 1k lookups | Full iteration | Publish | Physics + snapshot full time |
|---|---|---:|---:|---:|---:|---:|
| Dense 20k stable | Adaptive | 1.95 | 0.049 | 0.20 | 0.012 | 85.71 |
|  | `aeadef` | 13.11 | 0.258 | 0.25 | 0.010 | 96.34 |
| Sparse, 20k slots / 100 live | Adaptive | 0.05 | 0.223 | <0.01 | 0.003 | 0.85 |
|  | `aeadef` | 0.06 | 0.184 | <0.01 | 0.002 | 0.80 |
| Shrink, 20k to 100 | Adaptive | 0.17 | 0.221 | <0.01 | 0.002 | 0.93 |
|  | `aeadef` | 0.09 | 0.196 | <0.01 | 0.005 | 0.91 |
| Grow, 100 to 20k | Adaptive | 3.01 | 0.048 | 0.21 | 0.014 | 86.24 |
|  | `aeadef` | 15.45 | 0.391 | 0.37 | 0.010 | 99.20 |
| Oscillation, 180 / 220 live | Adaptive | 0.05 | 0.053 | <0.01 | 0.004 | 1.11 |
|  | `aeadef` | 0.10 | 0.181 | <0.01 | 0.003 | 1.22 |
| Retained Arc, 20k to 100 | Adaptive | 0.07 | 0.223 | <0.01 | 0.004 | 0.83 |
|  | `aeadef` | 0.07 | 0.184 | <0.01 | 0.003 | 0.84 |

Dense lookup uses direct slot indexing and reduced lookup time from 0.258 ms to
0.049 ms per sample. The dense snapshot estimate fell from 3,172.0 KiB to
2,656.2 KiB (16.3%). At 20k bodies, rebuilding and all measured snapshot work
also took less time in this run. Sparse steady-state storage and timings were
similar. Shrink adds about 0.08 ms of build time while releasing more than
3 MiB; later stable sparse rebuilds use the smaller buffers.

### Capacity, Estimated Memory, and Recycling

Capacities are `entries / active index`. Memory estimates count the `Vec` and
`HashMap` payload capacities for one currently published snapshot; they exclude
allocator metadata, hash table control bytes, and separately retained older
snapshots. Reused and reclaimed values are buffer counts over 12 publications.
Switches count Dense/Sparse changes between consecutive published snapshots.
The legacy layout has no such modes.

| Scenario | Layout | Capacity | Estimated memory | Reused / reclaimed | Index switches |
|---|---|---:|---:|---:|---:|
| Dense 20k stable | Adaptive | 20,000 / 20,000 | 2,656.2 KiB | 11 / 12 | 0 |
|  | `aeadef` | 20,000 / 28,672 | 3,172.0 KiB | 11 / 12 | — |
| Sparse, 20k slots / 100 live | Adaptive | 100 / 112 | 15.1 KiB | 11 / 12 | 0 |
|  | `aeadef` | 100 / 112 | 15.1 KiB | 11 / 12 | — |
| Shrink, 20k to 100 | Adaptive | 100 / 112 | 15.1 KiB | 11 / 12 | 1 |
|  | `aeadef` | 20,000 / 28,672 | 3,172.0 KiB | 11 / 12 | — |
| Grow, 100 to 20k | Adaptive | 20,000 / 20,000 | 2,656.2 KiB | 11 / 12 | 1 |
|  | `aeadef` | 20,000 / 28,672 | 3,172.0 KiB | 11 / 12 | — |
| Oscillation, 180 / 220 live | Adaptive | 220 / 1,000 | 35.3 KiB | 12 / 12 | 0 |
|  | `aeadef` | 1,000 / 1,792 | 167.0 KiB | 12 / 12 | — |
| Retained Arc, 20k to 100 | Adaptive | 100 / 112 | 15.1 KiB | 0 / 0 | 1 |
|  | `aeadef` | 100 / 112 | 15.1 KiB | 0 / 0 | — |

After shrinking from 20,000 bodies to 100, the adaptive buffer retained 15.1 KiB
instead of the legacy layout's 3,172.0 KiB, a 99.5% reduction in estimated
snapshot payload. The high-slot sparse scenario confirms that the active index
tracks 100 live bodies rather than the 20,000-slot span. Oscillation kept the
Dense index selected between the hysteresis thresholds without repeated mode
changes; it starts with a warmed Dense buffer for the 1,000-slot world before
alternating between 180 and 220 live bodies. In the retained-Arc scenario, the
benchmark verified that the old 20,000-body frame still contained its original
data while the current 100-body frame was published; recycling remained
unavailable while readers held frames.

These are single-machine microbenchmark results. The retained-Arc memory row
describes the current snapshot only; reader-held historical snapshots add their
own memory until those `Arc`s are dropped.
