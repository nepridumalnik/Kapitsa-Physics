# Collision pipeline benchmarks

## Refactoring baseline: pinned source

The pre-refactor reference was measured on 2026-11-01 from a temporary `git archive` of source commit `27b9f12a27152de89d0208acfffd9e28f018e0f8`. Benchmark-only capacity and logical-copy instrumentation plus the updated harness were overlaid onto that archive; no collision algorithms were changed. Runs used release builds, one warm-up, five independently built samples, and report min/median/max. The measured region excludes scene setup except where the row explicitly measures repeated API operations. The same harness is retained for later comparisons.

Timings include the `bench-internals` phase timers and the small counter overhead in island build/write-back paths. `Peak sampled` is the largest retained-capacity estimate observed at a phase boundary, not allocator peak or process RSS. `Logical capacity growth` is the net increase in retained vector/hash capacity during the step; it is not a count of malloc calls. Solver body/manifold copies count explicit value copies at the instrumented materialization and write-back sites, not compiler-generated traffic.

### Core steps

Times are milliseconds per step. Phase columns show the median sample.

| Scenario | Step min / median / max | Index | Broad | Narrow | CCD | Island build | Solver | Candidates | Contacts | Islands | Body copies | Manifold copies | Growth bytes | Retained KiB |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 256 separated Dynamic spheres | 0.201 / 0.216 / 0.241 | 0.117 | 0.030 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | 0 | 0 | 0 | 0 | 172,224 | 168 |
| 4,096 separated Dynamic spheres | 5.156 / 5.189 / 5.208 | 2.390 | 1.500 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | 0 | 0 | 0 | 0 | 2,752,704 | 2,688 |
| 20,000 separated Dynamic spheres | 28.257 / 29.510 / 30.860 | 13.979 | 9.415 | 0.000 | 0.000 | 0.001 | 0.000 | 0 | 0 | 0 | 0 | 0 | 16,814,656 | 16,420 |
| 20,000 bodies, 128 localized sphere pairs | 31.326 / 32.208 / 32.988 | 13.852 | 9.831 | 0.033 | 0.154 | 0.003 | 1.763 | 128 | 128 | 128 | 256 | 384 | 19,026,112 | 18,580 |
| 1,000 overlapping cuboids, one island | 18.754 / 19.356 / 20.055 | 0.424 | 0.178 | 3.427 | 0.003 | 0.014 | 13.833 | 999 | 999 | 1 | 1,000 | 2,997 | 4,131,456 | 4,034 |
| 20,000 Static + 100 separated Dynamic spheres | 22.909 / 24.285 / 25.263 | 12.436 | 9.862 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | 0 | 0 | 0 | 0 | 16,840,928 | 16,446 |
| Large triangle mesh + 100 movers | 1.482 / 1.559 / 1.659 | 0.066 | 0.027 | 0.521 | 0.001 | 0.003 | 0.793 | 100 | 100 | 100 | 200 | 300 | 778,197 | 759 |
| HeightField + 512 movers | 6.919 / 7.068 / 7.276 | 0.358 | 0.204 | 0.994 | 0.004 | 0.009 | 4.603 | 512 | 512 | 512 | 1,024 | 1,536 | 4,250,929 | 4,151 |
| LevelSet + 256 movers | 3.802 / 3.896 / 4.617 | 0.140 | 0.085 | 0.792 | 0.001 | 0.005 | 2.369 | 256 | 256 | 256 | 512 | 768 | 2,126,001 | 2,076 |
| 64 fast CCD bodies vs. wall | 279.832 / 284.261 / 292.727 | 0.034 | 0.018 | 0.069 | 284.076 | 0.001 | 0.000 | 64 | 0 | 0 | 0 | 0 | 149,380 | 145 |

### Storage, query, sweep, and skin workloads

| Scenario | Time min / median / max | Candidates | Contacts | Islands | Body copies | Manifold copies | Growth / retained bytes |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1 island × 1,000 Dynamic bodies | 18.956 / 20.052 / 21.432 ms | 999 | 999 | 1 | 1,000 | 2,997 | 4,131,456 |
| 500 islands × 2 Dynamic bodies | 8.059 / 8.644 / 10.115 ms | 500 | 500 | 500 | 1,000 | 1,500 | 4,394,344 |
| 500 Dynamic–Static pairs | 5.703 / 5.884 / 6.649 ms | 500 | 500 | 500 | 1,000 | 1,500 | 4,394,344 |
| 1,000 Dynamic / 4,000 Static, 4,000 contacts | 43.963 / 44.633 / 46.995 ms | 4,000 | 4,000 | 1,000 | 5,000 | 12,000 | 19,723,720 |
| 20,000 Static + 1,000 raycasts | 13,808 / 14,239 / 14,314 ms | — | — | — | — | — | — |
| 1,000 sweeps × 19,800-triangle mesh | 269.327 / 278.131 / 293.291 ms | — | — | — | — | — | — |
| 1,000 SkinPose updates, 1,024 vertices | 39.160 / 41.041 / 70.505 ms | — | — | — | — | — | — |
| 1,000 SkinPose updates, 10,000 vertices | 536.726 / 550.005 / 604.568 ms | — | — | — | — | — | — |
| 1,000 SkinPose updates, 100,172 vertices | 8,013.037 / 8,227.822 / 8,490.997 ms | — | — | — | — | — | — |

### Tokio controller, localized collisions

Times are milliseconds for eight steps; controller/world creation is outside the measured interval.

| Bodies | `PhysicsWorld::step` min / median / max | Controller ×1 | Controller ×2 | Controller ×4 |
|---:|---:|---:|---:|---:|
| 256 | 2.88 / 2.89 / 2.96 | 3.61 / 3.68 / 4.21 | 3.75 / 3.99 / 4.39 | 4.70 / 5.05 / 5.43 |
| 4,096 | 43.67 / 43.99 / 45.73 | 46.40 / 48.05 / 48.33 | 35.09 / 37.83 / 39.92 | 35.16 / 37.07 / 38.58 |
| 20,000 | 171.31 / 173.59 / 182.39 | 171.93 / 176.27 / 213.53 | 203.67 / 218.11 / 230.13 | 184.58 / 194.51 / 213.38 |

### Allocation profiler availability

Windows Performance Recorder and `xperf` are installed, and the Heap profile is listed. `wpr -start Heap -filemode` failed with `0x80070005 Access is denied` in this session, so no external heap trace or malloc counts were collected. The report therefore uses the explicitly labeled logical capacity metrics above; it does not present them as allocation counts.

### CCD candidate refresh, stage 2

After rebuilding CCD candidates after each resolved impact, the 64-body wall case measured 140.571 / 142.769 / 149.024 ms per step; its median CCD phase was 142.557 ms. The pinned baseline above was 279.832 / 284.261 / 292.727 ms per step with a 284.076 ms CCD phase. Median full-step time fell by 49.8%; the ranges do not overlap. The updated final candidate set contained 155 pairs versus 64 in the baseline. This is an intermediate stage measurement, before the remaining refactors.

### Integrator-aligned CCD trajectories, stage 3

CCD samples now use the rigid-body integrator from each body's active segment start, including multi-turn angular motion. Path bounds include center-of-mass and collider offsets, and convex pairs use relative translation when both collider origins move linearly. The existing endpoint-based query sweep API keeps its endpoint interpolation behavior.

Release run on 2026-11-01 on the same machine and five-sample harness:

| Scenario | Step min / median / max |
|---|---:|
| 20,000 separated Dynamic spheres | 26.946 / 27.696 / 28.994 ms |
| 20,000 bodies, 128 localized contacts | 30.156 / 30.225 / 31.645 ms |
| 64 fast CCD bodies vs. wall | 128.527 / 130.633 / 134.667 ms |
| 20,000 static colliders + 1,000 raycasts | 12,494 / 12,602 / 12,791 ms |

The CCD workload resolved 64 impacts and retained 155 final candidates. Its stage 3 median step time was 8.5% below the stage 2 median and 54.0% below the pinned baseline median; the stage 2 and stage 3 ranges do not overlap. The separated-body and query medians also improved relative to baseline, but those changes are directional measurements because the separated-body ranges overlap and query timings are load-sensitive. Other collision workload ranges overlap the baseline. Timings include the benchmark instrumentation; no allocator profile was available.

Release measurements taken on 2026-10-31 with `rustc 1.98.0` (`x86_64-pc-windows-msvc`) on an AMD Ryzen 9 7900X3D. Each row is the median of five independently constructed worlds; world construction is outside the timed region. Timings are local measurements, not performance targets.

Run the core phase benchmark with:

```powershell
cargo bench -p kapitsa-core --features bench-internals --bench collisions
```

Run the Tokio worker comparison with:

```powershell
cargo bench -p kapitsa-tokio --bench collisions
```

The `bench-internals` feature adds timings and retained-capacity estimates only to the core benchmark build. Default builds do not record timings. Phase durations are inclusive of the work named for that phase; small timer overhead applies to feature-enabled runs.

## Core pipeline

Times are milliseconds per one `PhysicsWorld::step`. `Index` includes proxy construction and spatial-index synchronization. `Broad` is candidate generation. `Narrow` is discrete contact generation. `CCD`, island construction, solving, event staging, and commit are recorded separately. Contacts counts discrete contact points; CCD impacts are reported separately.

| Scenario | Step | Index | Broad | Narrow | CCD | Island ms | Solver | Events | Commit | Candidates | Contacts | CCD impacts | Island count | Retained scratch |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 256 separated dynamic spheres | 0.229 | 0.121 | 0.038 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.001 | 0 | 0 | 0 | 0 | 168 KiB |
| 4,096 separated dynamic spheres | 4.749 | 2.352 | 1.323 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.027 | 0 | 0 | 0 | 0 | 2,688 KiB |
| 20,000 separated dynamic spheres | 27.552 | 12.684 | 9.143 | 0.000 | 0.000 | 0.001 | 0.001 | 0.000 | 0.267 | 0 | 0 | 0 | 0 | 16,420 KiB |
| 20,000 bodies, 128 localized sphere pairs | 32.544 | 13.801 | 10.294 | 0.034 | 0.197 | 0.003 | 1.721 | 0.031 | 0.224 | 128 | 128 | 0 | 128 | 18,580 KiB |
| 1,000 overlapping cuboids, one island | 18.548 | 0.478 | 0.207 | 3.257 | 0.002 | 0.024 | 13.146 | 0.065 | 0.012 | 999 | 999 | 0 | 1 | 4,034 KiB |
| 20,000 static + 100 separated dynamic spheres | 24.666 | 12.354 | 10.585 | 0.000 | 0.000 | 0.001 | 0.000 | 0.000 | 0.246 | 0 | 0 | 0 | 0 | 16,446 KiB |
| Large triangle mesh + 100 movers | 1.495 | 0.063 | 0.026 | 0.501 | 0.000 | 0.002 | 0.751 | 0.023 | 0.001 | 100 | 100 | 0 | 100 | 759 KiB |
| Heightfield + 512 movers | 6.385 | 0.239 | 0.179 | 0.958 | 0.001 | 0.005 | 4.199 | 0.107 | 0.003 | 512 | 512 | 0 | 512 | 4,151 KiB |
| Level set + 256 movers | 3.329 | 0.143 | 0.086 | 0.763 | 0.000 | 0.003 | 1.934 | 0.049 | 0.003 | 256 | 256 | 0 | 256 | 2,076 KiB |
| 64 fast CCD bodies vs. thin wall | 262.388 | 0.032 | 0.018 | 0.060 | 262.219 | 0.001 | 0.000 | 0.019 | 0.001 | 64 | 0 | 64 | 0 | 145 KiB |

The 20,000 separated-body case emitted zero candidate pairs. Its index and broadphase times were 12.684 ms and 9.143 ms. Against the 4,096-body case, the body count grew 4.88x and those phases grew 5.39x and 6.91x respectively. This sample gives no sign of an all-pairs candidate scan; it does not establish scaling for every distribution.

Spatial-query load used 20,000 static spheres and 100 raycasts per sample. The median was 1,293.473 ms for 100 rays (12,934.729 us per query). This is a notably expensive workload and should be treated as a follow-up optimization target if query-heavy scenes matter.

## Tokio worker comparison

Each entry is the median elapsed time for eight collision steps. The worlds contain 16 or 128 overlapping dynamic sphere pairs with remaining spheres separated well away. Controller setup is outside the timed region.

| Bodies | `PhysicsWorld::step` | Controller x1 | Controller x2 | Controller x4 |
|---:|---:|---:|---:|---:|
| 256 | 2.90 ms | 3.05 ms (0.95x) | 3.93 ms (0.74x) | 4.19 ms (0.69x) |
| 4,096 | 41.52 ms | 43.15 ms (0.96x) | 36.46 ms (1.14x) | 35.85 ms (1.16x) |
| 20,000 | 190.25 ms | 183.87 ms (1.03x) | 193.12 ms (0.99x) | 176.75 ms (1.08x) |

Worker scheduling cost more for the small scene. At 4,096 bodies, two and four workers improved this collision workload by 14% and 16%. At 20,000 bodies, two workers were 1% slower and four workers improved it by 8%. These results apply to this workload and machine, not every collision distribution.

## Memory estimate limits

`Retained scratch` estimates capacities of collision-owned vectors and hash tables after the measured step. `High-water` is the maximum of the five post-step estimates. The estimate excludes `PhysicsWorld` bodies and collider geometry, allocator metadata and fragmentation, runtime/task stacks, and process-wide allocations. It is not process RSS or a peak allocation measurement.

The interaction-index and table estimates use `capacity × size_of::<(key, value)>()` for hash-table entries. They exclude hash-table control bytes and allocator overhead.

## Workload notes

- The 20,000-body localized-contact scene has 128 overlapping sphere pairs and the remaining spheres far away. It yields 128 broadphase candidates and contacts.
- The 1,000-body island is a chain of overlapping cuboids; it creates 999 candidates and one connected solver island.
- The 20,000-static/100-dynamic scene keeps the dynamic bodies separated to isolate mixed body-count indexing and broadphase cost.
- Triangle mesh, heightfield, and level-set scenes move spheres into their surface during the measured step.
- The CCD workload sends 64 continuous spheres at a thin static wall and resolves 64 impacts.
- No collision thresholds or algorithms were tuned to reach these values.

## Indexed interactions, stage 9

This release microbenchmark compares the previous linear `Vec::iter_mut().find`
lookup with the pair-indexed `HashMap`. Each workload inserts M persistent
sensor interactions, then feeds the same M pairs again as transient CCD sensor
interactions. The duplicate pass exercises the existing persistent-over-
transient merge rule. Both containers reserve M entries before timing, matching
the steady-state scratch capacity retained across physics steps. Five samples
were measured on 2026-11-01 with the release profile; setup and reservation are
outside the timed region.

| Unique pairs M | Linear min / median / max | Indexed min / median / max | Median speedup | Linear retained estimate | Indexed retained estimate |
|---:|---:|---:|---:|---:|---:|
| 100 | 0.0054 / 0.0057 / 0.0065 ms | 0.0094 / 0.0096 / 0.0098 ms | 0.6× | 11,200 B | 15,680 B |
| 1,000 | 0.4814 / 0.4816 / 0.4882 ms | 0.0745 / 0.0746 / 0.0798 ms | 6.5× | 112,000 B | 183,680 B |
| 5,000 | 12.3071 / 12.3588 / 12.5609 ms | 0.3932 / 0.3990 / 0.4297 ms | 31.0× | 560,000 B | 846,720 B |
| 20,000 | 302.9426 / 304.2486 / 307.7812 ms | 3.2054 / 3.2247 / 3.2568 ms | 94.3× | 2,240,000 B | 3,386,880 B |

At M=100 the indexed path is about 4 microseconds slower and retains about
4.4 KiB more entry payload. This absolute cost is small, so the implementation
does not add a size threshold or a second lookup path. At 1,000 pairs and
above, the measured lookup savings grow quickly; the indexed path is used for
all sizes.

Run this focused comparison with:

```powershell
cargo bench -p kapitsa-core --features bench-internals --bench collision_interactions
```

## Committed spatial queries, stage 10

Queries use a separate committed-geometry AABB tree. Its leaf bounds are
expanded by 0.1 world units; small motion stays inside a proxy, and the tree
replaces a proxy only when committed geometry leaves it. Exact ray, sweep, and
overlap routines still determine hits. Direct mutable access marks the index
dirty and queries use the full-scan fallback until a successful step rebuilds
it. Infinite planes are tested separately.

Release samples on 2026-11-01, with 20,000 static spheres and the same ray
distribution used in the earlier 100-ray full-scan sample:

| Workload | Min / median / max | Per ray median |
|---|---:|---:|
| Full scan, 1,000 rays (stage 3) | 12,494 / 12,602 / 12,791 ms | 12.602 ms |
| Committed tree, 1,000 rays (stage 10) | 0.792 / 0.797 / 0.798 ms | 0.797 us |

The per-ray medians differ by about 15,800x. Setup is outside timing and the
ray scene is the same. The query index
retained-capacity estimate was 8,864 KiB for these 20,000 colliders; the
existing step scratch estimate was 16,420 KiB. Estimates cover vector
capacities and hash-table entry payload, not allocator metadata or control
bytes.

Index maintenance was measured separately on 1,000 separated Dynamic spheres
(five independently built worlds; one warmup):

| Workload | Step min / median / max | Commit median |
|---|---:|---:|
| Stationary | 0.834 / 0.868 / 0.899 ms | 0.006 ms |
| Moving at 1 unit/s | 0.914 / 0.942 / 0.981 ms | 0.111 ms |

The moving median was 0.074 ms above the stationary median, with commit time
0.105 ms higher. Fat proxies avoided exact tree reinsertion for this single
small movement. For context, the 20,000-body no-contact benchmark's step median
was 27.516 ms versus 27.696 ms in the pre-index stage 3 sample; those ranges
overlap, so the full-step comparison does not show a measurable change.
