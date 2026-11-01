# Joint stress and rocket flight results

## Environment and reproduction

- Windows 11, `x86_64-pc-windows-msvc`
- AMD Ryzen 9 7900X3D, 12 cores / 24 logical processors
- Rust `rustc 1.98.0 (88d9e12ae 2026-08-18)`, Cargo `1.98.0`
- Optimized Release profile

The joint stress harness reports min / median / max wall time, solver phase
times, retained and peak scratch capacity, and logical capacity growth. Normal
step scenes use one warm-up and three measured steps. The destructive mass
break case measures three separate first steps. Worker counts are 1, 2, 4,
and 8.

```powershell
cargo bench -p kapitsa-core --features bench-internals --bench joint_stress
cargo test -p kapitsa --release --lib --features bench-internals benchmark_rocket_flight_headless -- --ignored --nocapture --test-threads=1
cargo test -p kapitsa --release --lib --features bench-internals benchmark_rocket_flight_runtime_mission -- --ignored --nocapture --test-threads=1
```

The branching fixture exposed diverging joint errors. The recorded run includes
the rows below, but only workers 1 and 2 were recorded for the final
mass-separation scenario; workers 4 and 8 are missing. The benchmark does not
install an allocator counter, so its scratch and capacity figures are not
total allocation counts.

## Joint and collision stress

Times are median milliseconds per step unless otherwise noted. Scratch is the
retained workspace capacity after the measured step.

| Scene, 14,400 bodies | Workers | Median ms | Retained scratch MiB | Result |
|---|---:|---:|---:|---|
| Fixed-joint chain, 14,399 joints, 1 island | 1 | 328.380 | 57.43 | Anchor error remained 0.01 m |
| Fixed-joint chain, 14,399 joints, 1 island | 8 | 327.476 | 57.43 | Little scaling for one connected island |
| Independent pairs, 7,200 joints / islands | 1 | 200.261 | 52.30 | Final anchor error 3.64e-12 m |
| Independent pairs, 7,200 joints / islands | 4 | 76.636 | 52.30 | 2.61x faster than one worker |
| Independent pairs, 7,200 joints / islands | 8 | 43.286 | 52.30 | 4.63x faster than one worker |
| Dense vertical contacts, 14,400 candidates and contacts, 1 island | 1 | 261.585 | 68.46 | Solver iterations 160.816 ms; narrow phase 3.537 ms |
| Mass joint break, 7,200 break events | 1 | 165.534 | 55.19 | First step; scratch grew by 57,870,144 logical bytes |
| Mass joint break, 7,200 break events | 4 | 65.049 | 55.19 | First step |
| Mass joint break, 7,200 break events | 8 | 39.374 | 55.19 | First step |

A fleet of 100 two-body rockets (100 joints and 100 islands) measured 0.712 ms
at four workers and 0.638 ms at eight workers. Removing 7,200 joints in the
mass-separation fixture took 0.555 ms median at one worker; the subsequent
detached step took 3.879 ms. That separation run was incomplete at workers 4
and 8 when the stress run was stopped.

### Branching joint correctness failure

The branching fixture starts with a 0.01 m anchor offset and 0.001 rad frame
offset. At 144 bodies, its final maximum anchor error reached `1.64e20 m`; at
14,400 bodies it reached `1.94e44 m` after a step. This is a correctness
failure, so the branching timing numbers are not useful as valid solver
performance results. The connected chain and independent-pair fixtures do not
show this explosion. `SolverWorkspace::solve_with_joints()` prepares the joint
rows for an iteration before solving those rows; this is a lead for a solver
investigation, not a confirmed root cause. No solver algorithm was changed in
this benchmark stage.

The independent-pair measurements show that many independent islands scale
across workers on this host. The chain and contact stack each form one large
island and do not gain from extra workers. The retained scratch figures show
workspace size, not the fraction of step time spent preparing that workspace.

## Rocket mission

The CLI and OpenGL viewer now use the same mission fixture and flight runner,
including the attached two-stage fixed joint, fuel mass updates, separation,
engine cutoff, and stage impacts. The Release headless benchmark completed a
14.108 s simulated mission in 17.381 ms median across five samples after
warm-up (1,693 steps, approximately 812x simulated time per wall second).
Maximum altitude was 150.465 m, maximum speed 54.084 m/s, and measured energy
drift was 0.00000192%. Both stages impacted; two impact events were observed.

After one second of warm-up, the allocation sample over 1,573 steps averaged
0.021 allocation calls and 39.6 requested bytes per step (maximum 25 calls and
32,648 bytes in a step). Setup retained 717,441 bytes, with 37,212 peak extra
live bytes. The requested-byte p95 was zero, so the per-step maximum is an
outlier rather than a typical cost.

The full runtime mission benchmark at 1x warp completed 13.917 simulated
seconds in 14,071.864 ms median and emitted 15 snapshots. A separate runtime
CLI run requested 1,000x warp and completed 13.950 simulated seconds in about
53 ms wall time (about 263x observed), with both terminal bodies removed.

The generated GIF is [`target/rocket-flight-full.gif`](../target/rocket-flight-full.gif):
320 x 180 pixels, 335 frames, 157,078 bytes. It was decoded after generation
to verify its dimensions and frame count. The target directory is ignored by
Git, so the GIF is a local artifact and is not part of the commit.

## Validation

The following passed for the mission refactor and tests:

```powershell
cargo fmt --all -- --check
cargo check --workspace --all-features
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
cargo doc --workspace --all-features --no-deps
```

The expanded joint benchmark also passed formatting and targeted Clippy. The
recorded stress run is incomplete for workers 4 and 8 in the final
mass-separation case, as noted above.
