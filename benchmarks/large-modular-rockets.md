# Large modular rocket scaling

## Environment and reproduction

- Windows 11 Pro, `x86_64-pc-windows-msvc`, build 26300
- AMD Ryzen 9 7900X3D, 12 cores / 24 logical processors, 63.2 GiB RAM
- Rust `1.98.0`, optimized Release profile

The shared fixture creates two stages with a deterministic linear trunk and
side branches. Each requested part is one dynamic rigid body with its own mass
and inertia. The rocket has `parts - 1` fixed joints, two root engines, fuel
tanks, and one removable interstage joint. Only the two stage roots have
colliders; this isolates joint and root-to-pad contact costs rather than
modeling collisions between every internal module.

Run the measurements with:

```powershell
cargo bench -p kapitsa --features bench-internals --bench large_modular_rocket
cargo test --release -p kapitsa fixed_step_rockets_report_stability_and_reproducibility_at_all_sizes -- --ignored --nocapture
cargo test --release -p kapitsa adaptive_time_warp_reports_speed_debt_and_trajectory_accuracy -- --ignored --nocapture
cargo test -p kapitsa modular_rocket_remains_bounded_and_reproducible_through_adaptive_warp
```

The release step benchmark uses five warm-up steps, 60 steady samples, 30
startup samples, and 30 separation samples at 1, 2, 4, and 8 workers. Its p99
is a sampled upper-tail estimate from 60 (or 30 event) observations. Allocation
counts and requested bytes are collected after warm-up. Scratch memory is the
engine's retained-capacity estimate, not process RSS. The benchmark feature
also times joint constraint materialization separately; solver preparation and
iteration timings include the combined joint/contact solver island.

Startup and separation samples each use a fresh fixture after five resting
warm-up steps. The separation sample times joint removal, engine switching,
balanced impulses, and the following physics step; it does not include the
powered ascent before separation.

The fixed stress scenario runs 180 steps at `1/120 s` (1.5 simulated seconds):
pad rest, booster ignition, lower-stage fuel burn, separation with balanced
`2 N·s` impulses, upper-stage fuel burn and cutoff, then coast and pad impact.
It repeats each configuration with identical inputs. The Time Warp test uses
the clock-driven runtime with one physics worker for a 100 ms high-altitude
coast sample and compares the resulting state at the same simulated time with
a `1/120 s` reference.

## Release scaling

The table uses one worker for a consistent connected-island baseline. Memory
shows retained scratch after warm-up and after the measured separation step.
The Time Warp column is the achieved rate at a requested `100x` during the
high-altitude coast sample.

| Parts | Joints | Steady islands / largest bodies | Step p50 | Step p99 | p50 budget margin | Scratch steady / separated | Max anchor error at 120 Hz | Achieved Time Warp |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 99 | 1 / 100 | 4.725 ms | 6.246 ms | +3.608 ms | 0.385 / 0.512 MiB | 0.205 m | 3.80x |
| 200 | 199 | 1 / 200 | 9.673 ms | 9.884 ms | -1.340 ms | 0.763 / 1.016 MiB | 0.283 m | 2.42x |
| 500 | 499 | 1 / 501 | 24.345 ms | 25.162 ms | -16.012 ms | 1.847 / 2.231 MiB | 0.728 m | 0.86x |
| 1000 | 999 | 1 / 1001 | 48.753 ms | 49.700 ms | -40.419 ms | 3.681 / 4.455 MiB | 2.759 m | 0.29x |

The 120 Hz budget is 8.333 ms. On this machine, 100 parts is the largest
configuration whose one-worker p99 stays within budget. The 200-part p50
already misses the budget. For 100 parts, the steady step used zero heap
allocations and zero requested bytes after warm-up. At 1000 parts, the
separation step requested about 1.84 MB across 54 allocations and increased
retained scratch by about 1.15 MB.

| Parts | Steady allocations / bytes | Ignition allocations / bytes | Separation-step allocations / bytes |
|---:|---:|---:|---:|
| 100 | 0 / 0 | 1 / 2,368 | 39 / 221,276 |
| 200 | 0 / 0 | 1 / 2,368 | 44 / 439,308 |
| 500 | 0 / 0 | 1 / 2,368 | 49 / 921,372 |
| 1000 | 0 / 0 | 1 / 2,368 | 54 / 1,839,500 |

### Worker sensitivity and phase costs

Before separation the whole rocket is one connected solver island. Increasing
workers from 1 to 8 did not improve steady-state time: the solver processes one
island serially. The worker p50s were:

| Parts | 1 worker | 2 workers | 4 workers | 8 workers |
|---:|---:|---:|---:|---:|
| 100 | 4.725 ms | 4.710 ms | 4.819 ms | 4.872 ms |
| 200 | 9.673 ms | 9.705 ms | 9.705 ms | 9.705 ms |
| 500 | 24.345 ms | 24.311 ms | 24.425 ms | 24.344 ms |
| 1000 | 48.753 ms | 48.724 ms | 48.773 ms | 48.728 ms |

The largest steady solver island contains the connected rocket. It also
contains the static pad and one contact manifold for the 500- and 1000-part
samples. Separation creates two independent stage islands.

With one worker, the 100-to-1000 part steady-state phase measurements were:

| Parts | Joint constraint prepare | Solver prepare | Solver iterations | Collision phases | Island build |
|---:|---:|---:|---:|---:|---:|
| 100 | 4.6 µs | 5.1 µs | 4.650 ms | 16.0 µs | 1.1 µs |
| 200 | 11.8 µs | 12.1 µs | 9.528 ms | 16.8 µs | 2.6 µs |
| 500 | 29.3 µs | 32.3 µs | 24.016 ms | 18.9 µs | 6.3 µs |
| 1000 | 58.6 µs | 65.5 µs | 48.092 ms | 21.2 µs | 12.4 µs |

Solver iterations dominate, and their cost grows roughly in proportion to the
number of joints. Collision detection and island construction are small in
this fixture. Separation creates two independent stage islands, where worker
parallelism helps: at 1000 parts, the following step fell from 48.57 ms at one
worker to 31.33 ms at two workers. It still misses the 120 Hz budget by a wide
margin.

## Fixed-step stability results

The existing adaptive 100-part flight regression passes its 1x/10x/100x
trajectory checks over its 0.5-second scenario. The longer fixed-step stress
test uses the same shared generator but exercises pad support, fuel changes,
stage separation, powered rotation, coast, and impact. Its strict reporting
limits are max/RMS anchor error below 0.03/0.01 m, max/RMS frame error below
0.1/0.03 rad, and coast energy drift below 10%. No configuration passes all
these limits at `1/120 s`.

| Parts | Steps completed | Scenario | Max / RMS anchor | Max / RMS frame | Coast energy drift | Pad begin / impact | Joint breaks |
|---:|---:|---|---:|---:|---:|---:|---:|
| 100 | 180 | complete | 0.205 / 0.0140 m | 1.489 / 0.0934 rad | 3.26% | 1 / 110 | 0 |
| 200 | 180 | complete | 0.283 / 0.0283 m | 1.413 / 0.1218 rad | 66.48% | 2 / 91 | 0 |
| 500 | 180 | complete | 0.728 / 0.0640 m | 2.781 / 0.2790 rad | 59.27% | 2 / 46 | 0 |
| 1000 | 180 | incomplete: no surface events | 2.759 / 0.1616 m | 3.142 / 0.6350 rad | 45.05% | 0 / 0 | 0 |

All four runs remained finite, reproduced identical final states on repeat,
and conserved total linear momentum across the balanced separation impulse to
within `2.1e-13 kg·m/s`. There were no solver step errors or joint break
events. The 1000-part run completed its steps but did not register its pad
contact/impact scenario. The 100-part run is the first reproducible numerical
limit: its 0.205 m maximum anchor error is already far above the 0.03 m bound,
although its coast energy drift remains below 10%. These are reported
failures, not relaxed test thresholds.

The largest configuration that passed the high-altitude adaptive coast
comparison was 1000 parts. At requested 100x, maximum position and velocity
errors remained below 0.08 m and 0.30 m/s for all four sizes. That result is
limited to a 100 ms, no-thrust, no-contact coast sample; it does not establish
powered-flight stability for 1000 parts.

## Time Warp and operating limits

Achieved speed is simulated seconds divided by wall seconds. The values below
are separate from trajectory accuracy. At the end of the 100 ms sample, the
runtime had accumulated the following simulation-time debt:

| Parts | Requested 1x: achieved / debt | Requested 10x: achieved / debt | Requested 100x: achieved / debt |
|---:|---:|---:|---:|
| 100 | 0.95x / 0 ms | 3.54x / 656 ms | 3.80x / 9.80 s |
| 200 | 0.69x / 25 ms | 2.29x / 817 ms | 2.42x / 9.84 s |
| 500 | 0.23x / 52 ms | 0.79x / 804 ms | 0.86x / 9.57 s |
| 1000 | 0.08x / 44 ms | 0.27x / 592 ms | 0.29x / 5.48 s |

The optional 1000x high-altitude sample with 100 parts achieved 3.36x and
ended with 96.12 s of debt. Adaptive stepping kept the measured trajectories
within the comparison limits, but it could not deliver the requested rate.
The lower achieved rates on larger rockets are CPU saturation, not a
trajectory error.

## Findings

- **Real-time capacity:** 100 connected parts / 99 joints is the largest
  configuration measured below the 8.333 ms p99 budget on this machine.
- **Strict full-rocket stability:** none of the four configurations passes the
  stated 120 Hz anchor, frame, and energy limits. The 100-part adaptive flight
  regression passes only its shorter, separate scenario.
- **Largest stable coast tested:** 1000 parts remains finite and within the
  0.5 m / 2.5 m/s trajectory comparison for the high-altitude adaptive coast.
- **Main bottleneck:** one connected island serializes joint iterations. More
  workers do not accelerate it; they help only after the stages form separate
  islands.
- **Scaling:** steady cost is approximately linear in joint count for these
  topologies. Collision detection is not the limiting phase.
- **Next optimization to evaluate:** deterministic graph coloring or another
  parallel schedule for independent constraints within one island, validated
  against these unchanged anchor, frame, energy, and reference-trajectory
  limits. No solver algorithm or numerical threshold was changed for this
  study.

The tests intentionally keep the 1000-part case and the fixed-step numerical
failures visible. On the tested Ryzen 9 7900X3D, Kapitsa can compute a 100-part
modular rocket at 120 Hz in steady state, but the full powered/contact stress
scenario does not meet the strict joint-stability limits even at that size.
