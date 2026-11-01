# Time Warp Scheduler Diagnostic

This benchmark isolates scheduler and publication phases from physical
integration, then repeats the existing end-to-end runtime scaling workload.
It is diagnostic only; no scheduler behavior is changed by this stage.

## Environment and reproduction

- Windows 11 Pro, build 26300; `x86_64-pc-windows-msvc`
- AMD Ryzen 9 7900X3D, 12 cores / 24 logical processors
- Rust `rustc 1.98.0 (88d9e12ae 2026-08-18)`, Cargo `1.98.0`
- Release profile (`optimized`, `debug_assertions=false`)
- Four Tokio workers and four Rayon workers

Run from the workspace root:

```powershell
cargo test -p kapitsa --release --lib --features bench-internals benchmark_time_warp_scheduler_overhead -- --ignored --nocapture --test-threads=1
```

The phase harness uses a 100-body/100-collider scene, 32 warm-up physics
steps, and five independent timing samples. Physics and StepRunner samples
contain 128 steps each. Timer measurements contain five waits per sample; the
timer waker is separately wrapped to count wake callbacks. The runtime scaling
part reuses the established 100-body benchmark fixture: 32 real-time warm-up
steps and three 100 ms windows per requested multiplier. World, collider,
snapshot, and original command setup stay outside the measured step intervals.

## Isolated phase measurements

Values are microseconds per operation, min / median / max unless marked as
percentiles. The direct world and StepRunner rows use the same separated
100-body workload.

| Phase | Measurement |
|---|---:|
| `PhysicsWorld::step` | 49.355 / 49.663 / 49.723 µs/step |
| `StepRunner::step` total wall time | 55.052 / 58.700 / 63.010 µs/step |
| StepRunner engine work | p50 / p95 / p99: 47.000 / 62.100 / 90.400 µs |
| StepRunner dispatch and completion wait | p50 / p95 / p99: 7.900 / 14.800 / 25.500 µs |
| Rayon spawn plus oneshot round trip | 0.300 / 0.700 / 14.800 µs/task |
| `tokio::select!` with an immediately-ready branch | 0.005 / 0.005 / 0.014 µs/iteration |
| `tokio::select!` with zero-delay scaled timer | 14,525.712 / 15,425.288 / 15,446.025 µs/iteration |
| `tokio::task::yield_now()` | 0.010 / 0.022 / 0.024 µs/yield |
| Snapshot rebuild | 0.588 / 0.590 / 0.712 µs/step |
| Watch publication of an existing snapshot `Arc` | 0.064 / 0.064 / 0.288 µs/step |
| Snapshot rebuild plus `Arc` and watch publication | 0.538 / 0.540 / 0.703 µs/step |
| Empty collision-event batch construction and broadcast | 0.017 / 0.017 / 0.022 µs/step |
| Metrics clone and watch publication | 0.050 / 0.051 / 0.073 µs/step |
| Empty command poll and result drain | 0.009 / 0.010 / 0.036 µs/step |

The one-millisecond timer request took about 15.3–15.5 ms at requested
multipliers 1x, 10x, 100x, and 1,000x. The separate counting pass observed one
timer wake callback per wait. This identifies a roughly 15 ms timer/wakeup
round-trip on this machine; the `select!` macro itself and `yield_now()` are
many orders of magnitude cheaper.

## End-to-end Time Warp

The scene has 100 active dynamic bodies and 100 separated colliders. `dt` stays
at `1 / 120` second. Each measurement combines three 100 ms windows; results
below show completed physics steps and observed simulation-time rate.

| Requested warp | Steps | Steps/s | Achieved sim/wall | Peak time debt |
|---:|---:|---:|---:|---:|
| 0.1x | 6 | 18.5 | 0.15x | 0 ms |
| 1x | 39 | 118.8 | 0.99x | 0 ms |
| 10x | 162 | 500.3 | 4.17x | 1.749 s |
| 100x | 162 | 496.4 | 4.14x | 29.809 s |
| 1,000x | 162 | 496.0 | 4.13x | 310.291 s |

Across the high-warp windows, observed StepRunner CPU p50 was 54.2–56.2 µs.
The runtime therefore has ample per-step compute capacity compared with the
roughly 15.4 ms zero-delay timer wait. When the accumulator has debt,
`remaining` is zero. The loop nevertheless passes through
`tokio::select! { _ = wait_for_scaled_step(Duration::ZERO, multiplier) }`
after each `max_catch_up_steps` batch. That sleep round-trip costs about
15.4 ms even though the timer is already due. With the configured eight-step
catch-up batch, the timer alone implies roughly `8 / 0.0154`, or 519
steps/second (about 4.3x at 120 Hz), close to the measured 496–500 steps/s.

The catch-up limit is therefore not the sole cause: it bounds work before
control is checked, while the zero-duration Tokio timer between batches
creates the measured wall-time gap. `yield_now()` is not that gap. Snapshot,
event, metrics, and empty-command publication phases measured below one
microsecond each and cannot explain the plateau. The separate benchmark also
shows that physics plus StepRunner execution takes about 59 µs per step, far
below the roughly 1.9 ms of timer round-trip per completed step at high warp.

## Limitations

Timer resolution and task wake latency are platform and load dependent. These
measurements establish the cause on the recorded Windows host, not a universal
Tokio timer floor. `StepRunner::step` engine time includes physics and its
in-engine executor work; the separate spawn/oneshot measurement isolates only
top-level Rayon task handoff. Empty-event publication does not represent a
collision-heavy event batch.
