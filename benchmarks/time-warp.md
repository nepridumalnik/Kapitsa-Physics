# Time Warp Scheduler Diagnostic

This report isolates scheduler and publication phases from physical
integration, records the diagnosis of the original runtime, and compares the
end-to-end workload before and after the scheduler fix.

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

## Baseline isolated phase measurements (before scheduler fix)

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

## Before scheduler fix: end-to-end Time Warp

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

## Scheduler fix verification

After the diagnosis, the loop was changed to wait on a scaled timer only while
there is no complete physics step in the accumulator. When debt already
contains a ready step, it yields through `tokio::select!`, keeping stop and
multiplier changes ahead of the yield branch. `max_catch_up_steps` still
bounds uninterrupted physics work. The timer microbench above remains slow;
the backlog path no longer calls that timer.

The same Release command was rerun with the same host and fixtures. Before /
after achieved scaling and step throughput:

| Requested warp | Before | After | Steps/s after | Peak debt after |
|---:|---:|---:|---:|---:|
| 0.1x | 0.15x | 0.15x | 18.5 | 0 ms |
| 1x | 0.99x | 1.00x | 120.3 | 0 ms |
| 10x | 4.14–4.17x | 9.62x | 1,155 | 49.350 ms |
| 100x | 4.14x | 96.74x | 11,609 | 1.633 s |
| 1,000x | 4.13x | 111.67x | 13,401 | 266.815 s |

The 1,000x case did not settle within the post-measurement 100 ms pause check
and retained the reported time debt. Its approximately 13.4k steps/s is
consistent with the measured ~55 µs step CPU plus runtime overhead; achieved
warp is now limited by available compute rather than a 15 ms timer between
catch-up batches. At 10x and 100x, the accumulated debt drained during the
pause check. Fixed `dt` remains 1/120 s, and steps, snapshots, and simulation
timestamps advance in their existing order.

## Adaptive timestep accuracy and direct integration throughput

Measured on the same AMD Ryzen 9 7900X3D host with `rustc 1.98.0`
(`x86_64-pc-windows-msvc`) using the default optimized Release profile. Run:

```powershell
cargo test -p kapitsa --release --lib benchmark_adaptive_time_warp_orbit -- --ignored --nocapture --test-threads=1
```

The harness warms each path, then takes five independent timed samples. World
setup is outside each timed interval. It integrates a one-body central-gravity
orbit for 0.99999996 simulated seconds (120 truncated 1/120 s intervals) and
compares the final state against a 100 µs fixed-step reference. The
estimator-only row warms up, then calls the adaptive step selector 10,000
times per sample. These are direct `PhysicsWorld` timings; they exclude the
asynchronous runtime scheduler and snapshot publication.

| Policy | Steps | Wall ms min / median / max | Sim s / wall s at median | µs / step | Position error m | Velocity error m/s |
|---|---:|---:|---:|---:|---:|---:|
| Fixed, 8.333333 ms | 120 | 0.046 / 0.046 / 0.047 | 21,551.72 | 0.387 | 0.504667 | 0.773039 |
| Adaptive, requested 1x | 318 | 0.243 / 0.273 / 0.280 | 3,660.32 | 0.859 | 0.190242 | 0.297289 |
| Adaptive, requested 10x | 318 | 0.212 / 0.213 / 0.231 | 4,703.67 | 0.669 | 0.190242 | 0.297289 |
| Adaptive, requested 100x | 318 | 0.211 / 0.211 / 0.214 | 4,741.58 | 0.663 | 0.190242 | 0.297289 |
| Adaptive, requested 1,000x | 318 | 0.210 / 0.211 / 0.214 | 4,743.83 | 0.663 | 0.190242 | 0.297289 |

Adaptive energy drift was 0.009866% and angular-momentum drift was below the
printed precision for all four warp values. The chooser cost on this one-body
gravity world was 0.228 / 0.232 / 0.236 µs per decision (min / median / max).
With the default 1% relative tolerance, gravity bounds the step to about
3.14 ms, so requested warp does not increase the integration step size in this
scenario. The adaptive path takes 2.65x as many steps as fixed 120 Hz and is
about 4.6x slower per simulated second in this direct harness, while its final
orbit position and velocity errors are about 62% lower. This is an explicit
accuracy/throughput tradeoff; it does not indicate a regression in the fixed
compatibility path.

Automated tests also integrate the orbit, a damped spring, and a fixed-joint
anchor pair at equal simulated time for 1x, 10x, 100x, and 1,000x requested
warp. They compare against a 100 µs reference and verify repeatability for the
same warp. The fixed-joint test checks anchor separation and relative
orientation. These checks establish same-configuration reproducibility and
bounded error for these fixtures; they do not claim bitwise equality across
different warp factors.
