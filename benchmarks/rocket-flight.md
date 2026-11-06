# Rocket flight and runtime scaling benchmark

This report records the fifth stage of the runtime audit and benchmark on
`feat/collisions`. The benchmark uses the same mission model as the interactive
`rocket_flight` example. The headless run uses fixed physics steps and does not
wait for wall time or render telemetry.

## Environment

- Windows 11 Pro, build 26300; `x86_64-pc-windows-msvc`
- AMD Ryzen 9 7900X3D, 12 cores / 24 logical processors
- Rust `rustc 1.98.0 (88d9e12ae 2026-08-18)`, Cargo `1.98.0`
- Release profile (`optimized`, `debug_assertions=false`), default workspace
  profile, no custom profile settings
- `std::thread::available_parallelism()`: 24

## Headless flight

Each result is one of five independent Release runs after one complete warm-up
flight. Mission setup is outside the timer. The allocation sample is a separate
run after a one-second simulated warm-up; it includes physics event transitions
but excludes world and mission setup.

| Measure | Result |
|---|---:|
| Fixed steps / simulated time | 1,679 / 13.992 s |
| Mission wall time, min / median / max | 12.808 / 13.727 / 14.485 ms |
| Median throughput | 1,019 simulated s / wall s |
| Step CPU p50 / p95 / p99 | 5.9 / 14.8 / 16.2 μs |
| Maximum altitude / speed | 147.256 m / 53.548 m/s |
| Ballistic reference at t = 8 s | position `(0, 6371145.897, 0)` m; speed 5.208 m/s |
| Relative specific-energy drift at t = 8 s | 0.00000192% |
| Stage impacts / bodies remaining | 2 / planet only |
| Post-warm-up allocations per step, mean / max | 0.033 / 36 |
| Requested bytes per step, mean / p95 / max | 44.5 / 0 / 61,176 B |
| Live heap before sample / peak extra / estimated peak | 701,611 / 42,384 / 743,995 B |

The maximum allocation sample occurs at a collision/event transition. Allocation
counts and heap values come from the process allocator hook; peak live heap is
an estimate of tracked live allocations, not RSS or committed memory.

## Tokio runtime scaling

The runtime benchmark uses four Tokio worker threads and a four-thread Rayon
pool. Each scenario warms for 32 steps, then measures three 100 ms windows.
Each scaling row uses dynamic bodies and the listed collider count. CPU
percentiles use distinct `total_steps` values observed while polling metrics
every millisecond, rather than timing every completed step. Allocation values
cover only the three sampling windows; each fixture's 32-step warm-up, command
burst, and teardown are outside that interval. `live / extra / peak` is tracked
allocator baseline, peak extra live bytes, and their sum.

| Bodies (dynamic) | Colliders | Steps/s | Sim/wall | Step CPU p50 / p95 / p99 μs (observed samples) | Allocs / step | Requested B / step | Live / extra / peak B | Peak debt | Deadline misses |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 1 | 119.7 | 1.00x | 6.4 / 19.6 / 24.5 (21) | 3.077 | 372.5 | 303,802 / 1,736 / 305,538 | 0 ms | 0 |
| 100 | 100 | 120.4 | 1.00x | 78.3 / 124.2 / 132.4 (21) | 7.026 | 527.0 | 621,586 / 1,520 / 623,106 | 0 ms | 0 |
| 1,000 | 100 | 120.0 | 1.00x | 334.8 / 429.9 / 432.3 (21) | 11.026 | 687.0 | 1,959,058 / 1,520 / 1,960,578 | 0 ms | 0 |
| 10,000 | 100 | 118.8 | 0.99x | 2,705.5 / 3,303.2 / 3,562.5 (38) | 18.605 | 991.2 | 18,251,194 / 1,736 / 18,252,930 | 6.772 ms | 0 |

The 10,000-body scene stayed close to the configured 120 Hz, with the largest
sampled step at 3.56 ms. The benchmark did not produce collisions or lost event
batches in these separated scenes.

### Time multiplier

These runs use 100 dynamic bodies, 100 colliders, four active workers, and three
100 ms measurement windows. The engine preserves its fixed physics step. The
measured ratio is simulated time advanced divided by wall time during the
windows; high requested multipliers accumulate debt instead of being reported
as achieved.

| Requested | Steps/s | Measured sim/wall | Step CPU p50 / p95 / p99 μs | Peak debt | Settled within 100 ms after pause |
|---:|---:|---:|---:|---:|---|
| 0.1x | 18.2 | 0.15x | 120.8 / 144.1 / 144.1 | 0 ms | yes |
| 1x | 119.0 | 0.99x | 109.4 / 136.0 / 154.4 | 0 ms | yes |
| 10x | 502.8 | 4.19x | 68.8 / 119.7 / 153.8 | 1.730 s | no |
| 100x | 497.2 | 4.14x | 63.0 / 107.2 / 107.9 | 29.736 s | no |
| 1,000x | 500.0 | 4.17x | 66.2 / 109.5 / 158.3 | 307.767 s | no |

No deadline misses were counted in these windows. At 10x and above the runtime
reached about 4.1x on this workload, and the scheduler debt remained after the
100 ms pause check. These results do not support promising the requested warp
when the machine cannot process the required steps.

### Persistent force sources

This comparison uses 1 dynamic body and 999 static bodies, with 100 colliders.
The target body acquired finite positive velocity for one and ten sources. The
small differences below are within run noise and do not indicate a source-count
speedup or slowdown.

| Sources | Steps/s | Sim/wall | Step CPU p50 / p95 / p99 μs | Allocs / step | Requested B / step | Live / extra / peak B |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | 117.4 | 0.98x | 91.9 / 185.0 / 192.1 | 3.026 | 368.0 | 1,215,122 / 1,520 / 1,216,642 |
| 1 | 119.5 | 1.00x | 88.0 / 174.4 / 178.6 | 2.974 | 361.4 | 1,217,842 / 1,520 / 1,219,362 |
| 10 | 118.8 | 0.99x | 84.9 / 178.8 / 182.5 | 3.026 | 367.0 | 1,218,898 / 1,520 / 1,220,418 |

This deliberately keeps the active-body ratio below the sharded-path threshold.
The confirmed P1 issue in [`runtime-final-audit.md`](../docs/reviews/runtime-final-audit.md)
means this benchmark does not validate persistent force-source updates in the
parallel shard branch.

## Queue pressure and command latency

Eight concurrent writers filled the paused runtime queue, whose capacity is
1,024 batches. Each batch contains 128 commands. The writers then resumed the
runtime and retried the intact overflow buffer.

| Measure | Result |
|---|---:|
| Accepted / rejected-full batches | 1,025 / 10 |
| Commands accepted / failed batches | 131,200 / 0 |
| Returned overflow buffer length / capacity | 128 / 128 |
| Drain time / batch throughput | 1,058.954 ms / 968 batches/s |
| Submit-to-result latency p50 / p95 / p99 | 564.395 / 1,023.480 / 1,068.110 ms |
| Allocation calls / requested bytes | 6,673 / 127,730,224 B |
| Live heap before drain / peak extra / estimated peak | 106,737 / 65,081,456 / 65,188,193 B |

The result latency includes time spent waiting behind the paused queue and is
not an isolated command-application duration. The separate scaling scenarios'
32-batch command burst reported p50 acknowledgment times between 15.4 and
30.6 ms at 1x, depending on scene size and scheduling.

## Existing collision benchmark comparison

The core `parallel_step` benchmark was run on this machine at the original
audit HEAD `15a0559596deda4292e5037e10c5e75c2484714b` in a temporary detached
worktree, then twice on the current implementation. Each number below is the
benchmark's median over five samples; current shows the range of the two
consecutive run medians.

| Workload | Workers | Audit HEAD ms/step | Current ms/step (two run medians) |
|---|---:|---:|---:|
| 256 dynamic bodies | 1 | 0.087 | 0.087–0.087 |
|  | 2 | 0.089 | 0.087–0.091 |
|  | 4 | 0.087 | 0.089–0.089 |
| 4,096 dynamic bodies | 1 | 1.427 | 1.431–1.452 |
|  | 2 | 1.436 | 1.442–1.446 |
|  | 4 | 1.461 | 1.432–1.475 |
| 20,000 dynamic bodies | 1 | 7.099 | 7.070–7.099 |
|  | 2 | 7.006 | 7.001–7.253 |
|  | 4 | 7.034 | 7.102–7.189 |
| 4,096 bodies, 2,048 contact islands | 1 | 34.016 | 34.881–34.923 |
|  | 2 | 25.021 | 25.363–25.762 |
|  | 4 | 16.635 | 17.503–17.937 |

Separated-body workloads remain within about 3.5% of the audit-HEAD medians.
The contact-island case is 1–3% higher at one and two workers and 5–8% higher
at four workers in these runs. This is a focused performance signal, not a
causal attribution; it should be rechecked under a controlled CPU load before
using the contact-island figures as a release guarantee.

The core collision harness was also compared with the medians in
[`collisions.md`](collisions.md):

| Workload | Historical median ms | Current median ms | Change |
|---|---:|---:|---:|
| 256 separated Dynamic spheres | 0.216 | 0.259 | +19.9% (+0.043 ms) |
| 4,096 separated Dynamic spheres | 5.189 | 4.960 | -4.4% |
| 20,000 separated Dynamic spheres | 29.510 | 29.232 | -0.9% |
| 20,000 bodies, 128 localized contacts | 32.208 | 32.781 | +1.8% |
| 1,000 bodies, one contact island | 19.356 | 19.117 | -1.2% |
| 20,000 statics + 100 dynamic bodies | 24.285 | 23.233 | -4.3% |

The small 256-body case is slower by 0.043 ms; the larger listed collision
scenes range from 4.4% faster to 1.8% slower. This comparison uses historical
single medians from the existing report rather than a same-session paired
baseline, so small differences are not conclusive.

## Reproduction

Run these commands from the workspace root. The ignored runtime benchmarks
must run serially because the test allocator counters are process-global.

```powershell
cargo test -p kapitsa --release --lib --features bench-internals benchmark_rocket_flight -- --ignored --nocapture --test-threads=1
cargo bench -p kapitsa --bench collisions
cargo bench -p kapitsa --bench parallel_step
cargo bench -p kapitsa-core --features bench-internals --bench collisions
cargo bench -p kapitsa-core --features bench-internals --bench collision_storage -- workers_4096
cargo run --release -p kapitsa --example rocket_flight -- --warp=10 --no-ansi
```

The test command includes the fixed-step flight, runtime scaling, time
multiplier, persistent-source, and queue-pressure scenarios. The interactive
example command is a separate end-to-end run; its terminal renderer is not part
of benchmark timing. Allocation-hook bytes represent requested allocation
sizes and tracked live bytes, not allocator usable size, fragmentation, or
process RSS.
