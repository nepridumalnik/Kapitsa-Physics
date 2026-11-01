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

Run the `kapitsa` worker comparison with:

```powershell
cargo bench -p kapitsa --bench collisions
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

## Query workspaces, stage 11

Repeated sweeps against the same 20,000-triangle mesh compared the existing
one-shot `world.sweep` call with a prewarmed `PhysicsQuery::sweep` candidate.
Each row is five release samples; the world and mesh are built outside timing.
The query candidate retained 480 B of scratch capacity before and after each
timed loop, so it had no logical capacity growth after warm-up. This is a
capacity estimate, not an allocation count.

| Sweeps | One-shot min / median / max | Reusable query min / median / max | Median change |
|---:|---:|---:|---:|
| 100 | 23.875 / 23.931 / 24.028 ms | 24.314 / 24.374 / 25.873 ms | 1.9% slower |
| 1,000 | 238.931 / 239.504 / 239.545 ms | 243.072 / 243.612 / 258.110 ms | 1.7% slower |
| 10,000 | 2,507.238 / 2,512.695 / 2,552.082 ms | 2,539.116 / 2,547.752 / 2,566.764 ms | 1.4% slower |

The 10,000-call ranges overlap, and no sample showed a reusable-query speedup.
The optional public query object was therefore rejected. `QueryScratch` remains
internal and shares mesh, heightfield, level-set, and collider traversal
buffers across candidate colliders within each `PhysicsWorld` query call. No
external allocation trace was collected; the earlier Windows heap profiler
attempt was denied by the environment.

## Transactional skinned-mesh refit, stage 12

Each update deforms vertices into a staging vector, prepares triangle and node
bounds in reusable scratch arrays, and only then swaps the committed vertices
and triangle bounds and copies staged node bounds into the existing BVH. The
prebuilt mesh BVH is moved into the skin pose on its first update, so the
collider retains one BVH topology rather than a base tree plus a skinned copy.
The benchmark world, mesh, pose, and bone matrices are prepared outside the
timed 1,000-update loop. The mesh has 64 bones, with one influence per vertex
distributed round-robin; the two modes change only bone 0 or all 64 bones.

| Vertices | Changed bones | Min / median / max | Retained capacity before / after loop |
|---:|---|---:|---:|
| 1,024 | 1 | 37.118 / 37.168 / 37.793 ms | 575,064 / 575,064 B |
| 1,024 | 64 | 37.176 / 37.210 / 37.820 ms | 575,064 / 575,064 B |
| 10,000 | 1 | 422.469 / 422.883 / 424.618 ms | 6,043,544 / 6,043,544 B |
| 10,000 | 64 | 444.014 / 445.759 / 449.529 ms | 6,043,544 / 6,043,544 B |
| 100,172 | 1 | 4,385.139 / 4,396.816 / 4,993.160 ms | 60,471,656 / 60,471,656 B |
| 100,172 | 64 | 4,361.058 / 5,585.949 / 5,668.439 ms | 60,471,656 / 60,471,656 B |

No retained-capacity growth occurred during these measured loops. The estimate
includes both vertex buffers, the single BVH, refit scratch arrays, and the
pose structure; it excludes allocator overhead and fragmentation. It is
not an allocation count, and no external heap trace was available. Compared
with the earlier 1-bone baseline medians of 41.041, 550.005, and 8,227.822 ms,
the new 1-bone medians were 9.4%, 23.1%, and 46.6% lower. The old and new
one-bone ranges do not overlap at any tested size. Updating 64 bones costs
somewhat more at 10,000 vertices, while both modes still recompute every
skinned vertex. The 100,172-vertex all-bones run had a wide range and is
load-sensitive.

## Solver island layout comparison, stage 13

The full-step samples use the current copied-per-island `SolverIsland` path.
Each sample builds a new world, advances it for three untimed steady-state
steps, then times one step. One separate world is used as a process warm-up;
each row below is five release samples on 2026-11-01. The added phase metrics
separate island preparation, solver iterations, and staged write-back. Buffer
growth is reported as logical retained-capacity growth, not allocator calls.

| Workload | Full step min / median / max | Prepare / iterations / write-back median | Logical body / manifold copies | Capacity growth / retained scratch |
|---|---:|---:|---:|---:|
| 1 island, 1,000 Dynamic | 20.205 / 20.293 / 20.400 ms | 0.055 / 14.428 / 0.192 ms | 1,000 / 2,997 | 0 / 4,986,160 B |
| 500 islands, 2 Dynamic each | 7.200 / 7.247 / 7.473 ms | 0.038 / 5.555 / 0.082 ms | 1,000 / 1,500 | 0 / 4,803,368 B |
| 500 Dynamic, one shared Static plane | 4.252 / 4.256 / 4.287 ms | 0.033 / 3.050 / 0.059 ms | 1,000 / 1,500 | 0 / 4,258,013 B |
| 1,000 Dynamic, 4,000 contacts | 39.671 / 39.956 / 40.098 ms | 0.430 / 27.860 / 1.005 ms | 5,000 / 12,000 | 0 / 22,995,912 B |
| 20,000 Dynamic, 128 contact pairs | 20.864 / 21.148 / 21.315 ms | 0.032 / 1.444 / 0.023 ms | 256 / 384 | 0 / 19,130,048 B |

The shared-plane case has 500 islands; its one Static body is copied into each
island as a read-only solver body. Dynamic bodies remain in one island each.
The 20,000-body sparse case creates only 128 contact islands; most retained
scratch belongs to the world step and spatial index rather than island arrays.

The layout-only `layouts` benchmark implements the current per-island grouping
and a packed-range candidate. Both produce identical sorted body indices,
manifold indices, and local manifold body indices in all six generated
workloads. It measures preparation and remapping only; shared `SolverWorkspace`
contact iterations and full-world commit are not part of this prototype
comparison. Each entry is the median of five samples, each timing 250 warmed
preparations. Retained bytes cover only the two layout structures, not world
scratch or allocator metadata.

| Workload | Copied layout | Packed layout | Logical body / manifold copies (both) | Retained bytes: copied / packed |
|---|---:|---:|---:|---:|
| 1 island, 1,000 Dynamic | 0.0137 ms | 0.0041 ms | 1,000 / 999 | 41,248 / 48,216 |
| 500 islands, 2 Dynamic each | 0.0034 ms | 0.0078 ms | 1,000 / 500 | 100,000 / 60,000 |
| 500 Dynamic, shared Static | 0.0036 ms | 0.0072 ms | 1,000 / 500 | 100,000 / 56,008 |
| 1,000 Dynamic, 4,000 contacts | 0.0291 ms | 0.0302 ms | 5,000 / 4,000 | 232,000 / 264,000 |
| 4,096 Dynamic, 2,048 islands | 0.0141 ms | 0.0316 ms | 4,096 / 2,048 | 409,600 / 245,760 |
| 20,000 bodies, 128 contacts | 0.0008 ms | 0.0018 ms | 256 / 128 | 25,600 / 173,312 |

Packed storage helps the single large island's layout preparation by about 10
microseconds, but is slower for four of the other five cases and has no
measurable preparation win for 4,000 contacts. It keeps the same logical body
and manifold copy counts. The sparse 20,000-body layout uses a body-to-local
map and retains about 6.8 times the copied layout's capacity. Since the packed
prototype did not establish an end-to-end step improvement and several
workloads regressed at the preparation phase, the production `SolverIsland`
layout was retained.

The 4,096-Dynamic / 2,048-island workload also uses the normal sharded step
entry point to measure 1, 2, and 4 workers. Each sample has three untimed steps
on that world before the measured one; retained capacity had zero logical
growth on all measured steps.

| Workers | Full step min / median / max | Prepare / iterations / write-back median | Logical copies | Retained scratch |
|---:|---:|---:|---:|---:|
| 1 | 33.897 / 33.997 / 35.400 ms | 0.371 / 23.011 / 0.479 ms | 4,096 bodies / 6,144 manifolds | 19,583,168 B |
| 2 | 22.713 / 23.122 / 24.965 ms | 0.399 / 12.028 / 0.534 ms | 4,096 bodies / 6,144 manifolds | 19,583,168 B |
| 4 | 16.915 / 17.030 / 17.785 ms | 0.413 / 6.388 / 0.499 ms | 4,096 bodies / 6,144 manifolds | 19,583,168 B |

Median full-step speedup was 1.47x at two workers and 2.00x at four workers
relative to one worker. This measures the current copied layout; it is not a
packed-layout parallel speedup comparison.

## Fixed-capacity contact manifolds, stage 14

`ContactManifold` now stores `[ContactPoint; 4]` and a `u8` active count; its
slice exposes only the counted contacts. Unused entries contain a private
valid sentinel. `world_points` remains because contact reduction uses the
cached positions. Mesh and heightfield manifold merging and reversed manifold
creation now reuse those cached positions instead of transforming the same
local points again. Replacing a deeper duplicate updates its cached position
alongside its contact.

On this target, `size_of::<ContactManifold>()` fell from 648 B to 616 B, a
32-byte (4.9%) reduction per inline manifold value. The following five-sample
release comparisons use the same scenarios and three untimed warm-up steps as
the solver-island results above:

| Workload | Before min / median / max | After min / median / max |
|---|---:|---:|
| 1 island, 1,000 Dynamic | 20.205 / 20.293 / 20.400 ms | 20.245 / 20.280 / 20.651 ms |
| 500 islands, 2 Dynamic each | 7.200 / 7.247 / 7.473 ms | 7.199 / 7.227 / 7.269 ms |
| 500 Dynamic, shared Static plane | 4.252 / 4.256 / 4.287 ms | 4.263 / 4.364 / 4.463 ms |
| 1,000 Dynamic, 4,000 contacts | 39.671 / 39.956 / 40.098 ms | 39.930 / 40.045 / 40.475 ms |

All before/after ranges overlap, so these full-step runs show no measurable
speed change. The storage reduction is established by the type-size check; the
cached-position reuse also removes repeated transform calculations in the
mesh and heightfield merge path. Logical capacity is not an allocator count.

Run the focused groups with:

```powershell
cargo bench -p kapitsa-core --features bench-internals --bench collision_storage -- layouts
cargo bench -p kapitsa-core --features bench-internals --bench collision_storage -- workers_4096
```

## Final baseline comparison, stage 18

The final comparison uses the original implementation at benchmark commit
`336582d` and the completed implementation, built in Release mode on the same
machine. Each row is the median of five samples; ranges are shown where they
were captured. World construction is outside the timed region. Measurements
include the benchmark instrumentation. Changes smaller than the sample spread
are not treated as established improvements.

| Workload | Baseline | Final | Median change | Reading |
|---|---:|---:|---:|---|
| 20,000 separated Dynamic spheres, one full step | 25.757 ms (25.366–25.965) | 26.620 ms (26.132–26.869) | +3.4% | Small regression; ranges do not overlap |
| 1,000 Dynamic bodies, one 999-contact island, full step | 17.148 ms | 17.336 ms | +1.1% | Solver phase 12.201 → 12.138 ms; no material solver change |
| 500 islands, two Dynamic bodies each | 7.784 ms | 7.994 ms | +2.7% | Ranges overlap; no measurable change |
| 64 fast CCD impacts | 250.860 ms | 126.093 ms | −49.7% | Final CCD phase 125.888 ms; 155 candidates vs. 64 at baseline |
| 1,000 raycasts against 20,000 static colliders | 12,258.859 ms | 0.787 ms | −99.994% | Indexed queries; final query-index estimate is 8,864 KiB |
| 1,000 sweeps against a 20,000-triangle mesh | 240.198 ms | 245.160 ms | +2.1% | Small regression; ranges do not overlap |
| 1,000 SkinPose updates, 1,024 vertices | 36.693 ms | 36.952 ms | +0.7% | Ranges overlap; no measurable change |
| 1,000 Dynamic / 4,000 Static contacts | 38.643 ms | 35.360 ms | −8.5% | Improvement in this workload |

For the one-island storage sample, estimated peak sampled and retained scratch
changed from 4,034 KiB to 4,024 KiB. Explicit materialization counts remained
1,000 body and 2,997 manifold copies per step. Logical capacity growth changed
from 4,131,456 B to 4,121,568 B; this is a retained-capacity delta, not a
malloc count. `ContactManifold` fell from 648 B to 616 B.

The additional Tokio collision-worker workload uses 4,096 Dynamic spheres in
2,048 two-body contact islands, with three warm-up steps and twelve measured
steps per sample. Baseline and final medians per step were 30.62 → 30.81 ms at
one worker, 11.04 → 10.72 ms at two workers, and 9.48 → 9.53 ms at four
workers. These small differences do not establish a before/after worker
speedup. The final run's full-step ranges and phase medians are in the Stage 13
worker table above. Existing Tokio localized-collision comparisons also varied
by scene size; see the stage-specific table earlier in this document.

The primary gains are CCD candidate refresh and committed spatial queries.
The no-contact step and mesh sweep regressions are recorded as tradeoffs. The
packed solver-island prototype, a public reusable query object, and removing
the adaptive Dense/Sparse snapshot index were not adopted: their workload
comparisons did not show a consistent end-to-end benefit. Full correctness,
memory-estimate limits, and implementation details are in
[`docs/reviews/collision-refactoring-report.md`](../docs/reviews/collision-refactoring-report.md).

## CCD Chaos refactor, stage 1 baseline

The current CCD implementation at source commit
`8c359eaf96a994c1b5a0c97fdd05fef7663d11e5` was measured before changing its
algorithm. Runs used the existing Release benchmark, one warm-up, five
independently built worlds, and a 1/60-second step. The 64 continuous spheres
start at 1,000 m/s and hit one static cuboid wall. An uninstrumented run had a
136.898 / 145.533 / 158.071 ms full-step range and a 145.273 ms median CCD
phase. With the new `bench-internals` counters enabled, the full-step range was
138.483 / 146.847 / 148.403 ms and the median CCD phase was 146.563 ms. The
medians are within the observed run-to-run variation; default builds compile
out these counters and their pair-tracking set.

| Initial candidates | TOI calls | Repeated TOI calls | Broadphase builds | CCD refreshes | TOI search | Refresh | Contact resolution | CCD impacts | Retained scratch |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 64 | 5,560 | 5,405 | 65 | 64 | 142.518 ms | 3.313 ms | 0.666 ms | 64 | ~234 KiB |

`TOI search` covers each candidate scan, including narrowphase calls. `Refresh`
covers the full proxy update and candidate rebuild, so its time overlaps the
separate spatial-index and broadphase phase timers. The broadphase build count
includes the initial build; 64 subsequent builds refreshed candidates after
impacts. A repeated TOI call means the same collider pair was evaluated again
later in this step; pairs skipped by the existing processed-pair rule are not
counted. The 5,405 repeated evaluations identify the global rescanning cost
that the next stages target. The retained-scratch estimate includes the
benchmark-only set used to count repeats.

The stage 2 graph builder produced 64 CCD islands for the same scene, while
the existing global CCD solver still processed them together. Its median full
step was 144.215 ms and median CCD phase 143.986 ms; these ranges overlap the
stage 1 run, so this is not a performance change. The graph tests cover a
dynamic chain, repeated colliders on one body, shared kinematic and static
surfaces, empty candidate sets, and stable island ordering.

### Stage 3: local CCD processing

Each island now keeps absolute TOI times in a reusable cache. After an impact,
only entries that involve either changed dynamic body are invalidated. The
single-island path still uses the global refresh fallback so the established
wall-rebound-to-new-static-target scenario remains correct while selective
cross-island resweep is implemented in stage 4.

| Scenario | Step min / median / max | CCD | Initial candidates | TOI calls | Repeated calls | Broadphase builds | Refreshes | CCD islands | Impacts | Retained scratch |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 64 fast spheres vs. one wall | 5.585 / 5.614 / 6.017 ms | 5.447 ms | 64 | 64 | 0 | 1 | 0 | 64 | 64 | ~164 KiB |

Against the stage 1 instrumented baseline, the median CCD phase fell from
146.563 ms to 5.447 ms and full-step median from 146.847 ms to 5.614 ms.
Candidate work fell from 5,560 TOI calls (5,405 repeats) to one call per sphere.
The release benchmark includes counters; all 459 core unit tests, core
integration tests, and four Tokio collision simulation tests passed. A
feature-enabled regression test asserts the two-island case performs exactly
two TOI calls and no candidate refreshes.

### Stages 4–7: selective resweep, worker scheduling, clipping, and TOI sweep

The Release `bench-internals` profile after stages 1–6 measured the 64-sphere
wall case at about 2.05 ms per step, with a 1.858 ms CCD phase and 0.932 ms in
TOI search. The sphere–cuboid specialization solves linear translation against
the rounded box analytically and uses conservative advancement with exact
point-to-box distance for sampled nonlinear sphere paths; rotation of the
cuboid still uses GJK. On the same
64-impact scene, the final Release run measured 0.677 ms median step time,
0.539 ms CCD, and 0.069 ms TOI search. This reduced the CCD phase by 71% and
full-step time by 67% relative to the pre-specialization measurement. The
benchmark wall contains the entire grid, so all 64 spheres produce impacts.

The sphere workload is now cheap enough that scoped workers provide no stable
gain: at 64 spheres, 1/2/4 workers measured 0.694/0.681/0.688 ms per step, but
the sample ranges overlap. Parallel TOI preparation is therefore skipped for
analytic sphere pairs. For GJK-heavy independent cuboid islands, preparation
starts at 256 candidates:

| Cuboid bodies | Workers | Step median | CCD median |
|---:|---:|---:|---:|
| 64 | 1 / 2 / 4 | 0.988 / 0.984 / 1.001 ms | 0.818 / 0.821 / 0.827 ms |
| 128 | 1 / 2 / 4 | 2.022 / 2.022 / 1.991 ms | 1.683 / 1.672 / 1.665 ms |
| 256 | 1 / 2 / 4 | 4.058 / 4.018 / 3.835 ms | 3.351 / 3.276 / 3.099 ms |
| 512 | 1 / 2 / 4 | 8.459 / 7.815 / 7.736 ms | 6.998 / 6.345 / 6.236 ms |

At 256 candidates, four workers reduced median step time by 5.5%, although the
sample ranges overlap. At 512, the median step improved by 8.5% and the ranges do
not overlap. The 64 and 128 cases showed no stable gain. This cutoff is for
candidate TOI preparation only; island impact resolution remains ordered so
trajectory changes and cross-island contacts retain deterministic handling.
The worker benchmarks use static shared walls and separate dynamic islands.
Their retained scratch estimates were 683 KiB at 256 candidates and 1,364 KiB
at 512; parallel preparation returned the same estimate as the sequential
path.

The specialized solver tests face, edge, corner and tangential contact,
near-misses, fixed rotated boxes, translating pairs, initial overlap, and
reversed pair order. Rotating box motion continues through the general GJK
path.

## CCD Chaos refactor, final stage 9 measurements

The final Release run used one warm-up and five independently built worlds per
scene on the same machine. The 64-impact baseline comparison above uses the
same 1/60-second step, sphere layout, static wall, and benchmark instrumentation
as stage 1. The final stage 1 source measured 146.847 ms full step and
146.563 ms CCD; the final implementation measured 0.677 ms and 0.539 ms.
That is a 217x reduction in full-step median and a 272x reduction in CCD
median. Candidate evaluations fell from 5,560 (5,405 repeated) to 64 (zero
repeated), broadphase builds from 65 to 1, and CCD refreshes from 64 to zero.
Both runs resolved 64 impacts. Retained CCD scratch fell from about 234 KiB to
171 KiB. The separately quoted historical 126.093 ms CCD result is not the
stage 1 source baseline; the final 0.539 ms result is about 234x faster than
that reference.

| Scene | Step min / median / max | CCD min / median / max | CCD islands | Initial / final candidates | TOI calls / repeats | Resweeps | Impacts | New pairs | Retained scratch |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 64 fast spheres vs. wall | 0.667 / 0.677 / 0.704 ms | 0.534 / 0.539 / 0.541 ms | 64 | 64 / 64 | 64 / 0 | 64 | 64 | 0 | 171 KiB |
| 64 fast bodies, no impacts | 0.488 / 0.498 / 0.535 ms | 0.307 / 0.314 / 0.325 ms | 1 | 531 / 531 | 531 / 0 | 0 | 0 | 0 | 623 KiB |
| 64-body swept chain, one large island | 0.133 / 0.135 / 0.155 ms | 0.055 / 0.057 / 0.058 ms | 1 | 64 / 64 | 67 / 3 | 2 | 2 | 0 | 144 KiB |
| One sphere, sequential wall bounces | 0.058 / 0.058 / 0.058 ms | 0.055 / 0.055 / 0.055 ms | 1 | 2 / 2 | 8 / 6 | 6 | 6 | 0 | 8 KiB |
| Rebound discovers a new pair | 0.029 / 0.029 / 0.031 ms | 0.025 / 0.025 / 0.027 ms | 1 | 1 / 2 | 4 / 2 | 3 | 3 | 1 | 8 KiB |

The 64-body chain forms one island from overlapping swept candidate bounds; only
the leading body reaches the wall. The no-impact scene forms one island from
531 conservative broadphase candidates and performs no contact resolution.
These cases measure both the cost of large candidate groups and the cost of
non-contacting fast motion. The sequential-bounce case reaches six actual
impacts in a single step. The rebound case begins with one candidate, discovers
the previously absent static-target pair after trajectory change, and resolves
all three impacts.

The following tables show step and CCD min/median/max by worker count.
Each reported island count and initial candidate count equals the body count;
resweeps and impacts also equal it. Repeated TOIs, new pairs, and fallbacks are
zero. Retained scratch was 44 / 171 / 342 / 683 KiB for sphere counts 16 / 64 /
128 / 256, and 171 / 342 / 683 / 1,364 KiB for cuboid counts 64 / 128 / 256 /
512. Parallel TOI preparation returned the same retained-scratch estimate.

| Sphere bodies vs. wall | Workers | Step min / median / max (ms) | CCD min / median / max (ms) |
|---:|---:|---:|---:|
| 16 | 1 | 0.165 / 0.169 / 0.170 | 0.133 / 0.134 / 0.137 |
| 16 | 2 | 0.164 / 0.165 / 0.167 | 0.132 / 0.133 / 0.133 |
| 16 | 4 | 0.164 / 0.164 / 0.168 | 0.132 / 0.133 / 0.136 |
| 64 | 1 | 0.687 / 0.694 / 0.714 | 0.538 / 0.549 / 0.553 |
| 64 | 2 | 0.672 / 0.681 / 0.686 | 0.537 / 0.541 / 0.549 |
| 64 | 4 | 0.680 / 0.688 / 0.708 | 0.540 / 0.547 / 0.553 |
| 128 | 1 | 1.397 / 1.422 / 1.440 | 1.107 / 1.110 / 1.139 |
| 128 | 2 | 1.394 / 1.399 / 1.419 | 1.095 / 1.097 / 1.105 |
| 128 | 4 | 1.395 / 1.404 / 1.443 | 1.107 / 1.116 / 1.118 |
| 256 | 1 | 2.858 / 2.878 / 2.942 | 2.245 / 2.282 / 2.297 |
| 256 | 2 | 2.849 / 2.895 / 2.934 | 2.243 / 2.273 / 2.310 |
| 256 | 4 | 2.853 / 2.914 / 2.929 | 2.260 / 2.276 / 2.295 |

The sphere path scales close to proportionally from 64 to 256 bodies: a 4x
increase in body count produced 4.15x step time and 4.16x CCD time at one
worker. Differences between worker counts overlap sample ranges, so the sphere
workload has no established parallel speedup.

| Cuboid bodies vs. wall (GJK) | Workers | Step min / median / max (ms) | CCD min / median / max (ms) |
|---:|---:|---:|---:|
| 64 | 1 | 0.977 / 0.988 / 0.994 | 0.815 / 0.818 / 0.829 |
| 64 | 2 | 0.971 / 0.984 / 1.008 | 0.815 / 0.821 / 0.823 |
| 64 | 4 | 0.976 / 1.001 / 1.008 | 0.818 / 0.827 / 0.840 |
| 128 | 1 | 2.005 / 2.022 / 2.065 | 1.656 / 1.683 / 1.714 |
| 128 | 2 | 2.001 / 2.022 / 2.066 | 1.666 / 1.672 / 1.681 |
| 128 | 4 | 1.987 / 1.991 / 2.057 | 1.647 / 1.665 / 1.680 |
| 256 | 1 | 4.041 / 4.058 / 4.109 | 3.340 / 3.351 / 3.385 |
| 256 | 2 | 3.805 / 4.018 / 4.189 | 3.115 / 3.276 / 3.453 |
| 256 | 4 | 3.820 / 3.835 / 3.987 | 3.081 / 3.099 / 3.241 |
| 512 | 1 | 8.384 / 8.459 / 8.531 | 6.957 / 6.998 / 7.058 |
| 512 | 2 | 7.750 / 7.815 / 8.104 | 6.266 / 6.345 / 6.594 |
| 512 | 4 | 7.684 / 7.736 / 7.853 | 6.190 / 6.236 / 6.309 |

Doubling GJK-heavy candidates from 64 to 512 increased one-worker CCD time by
8.56x, close to the 8x candidate increase. Four workers reduced the median
step by 5.5% at 256 candidates (ranges overlap) and by 8.5% at 512 (ranges do
not overlap); the 512-candidate CCD median improved by 10.9%. The measured
cutoff stays at 256 candidates; below it, scoped worker overhead outweighs
preparation gains. No-contact 20,000-body full-step
time was 26.641 ms median with about 16,420 KiB retained scratch. The current
1,000 raycasts against 20,000 static colliders took 0.772 ms median. The
existing same-machine sweep comparison remains 240.198 ms baseline to
245.160 ms final for 1,000 sweeps against a 20,000-triangle mesh (+2.1%).

The final refactor review, architecture limitations, implementation size,
commit list, and validation matrix are in
[`docs/reviews/ccd-chaos-refactoring-report.md`](../docs/reviews/ccd-chaos-refactoring-report.md).

## CCD follow-up baseline: static-scene resweep cost

This baseline was measured from the completed CCD refactor at commit
`038e4d4` before the follow-up optimizations. Release benchmark settings were
one warm-up and five samples at `dt = 1/60 s`; the table reports min/median/max.
The new mixed scenes contain 20,000 static collider proxies and 64, 128, or
256 independent dynamic CCD spheres. One quarter of the dynamic spheres hit a
wall; the rest have no obstacle.

| Scene | Step min / median / max | CCD min / median / max | CCD islands / candidates / TOI calls / impacts | Resweeps | Proxy visits | Proxies updated | Local / global TOI recalculations | Retained scratch |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 64 fast spheres vs. wall | 0.685 / 0.690 / 0.700 ms | 0.542 / 0.545 / 0.556 ms | 64 / 64 / 64 / 64 | 64 | 8,320 | 64 | 0 / 0 | 171 KiB |
| 20k statics + 64 mixed | 21.793 / 22.172 / 22.896 ms | 0.654 / 0.669 / 0.690 ms | 16 / 16 / 16 / 16 | 16 | 642,048 | 16 | 0 / 0 | 12,396 KiB |
| 20k statics + 128 mixed | 22.298 / 23.140 / 23.460 ms | 1.300 / 1.329 / 1.364 ms | 32 / 32 / 32 / 32 | 32 | 1,288,192 | 32 | 0 / 0 | 12,467 KiB |
| 20k statics + 256 mixed | 23.725 / 24.126 / 24.787 ms | 2.550 / 2.607 / 2.659 ms | 64 / 64 / 64 / 64 | 64 | 2,592,768 | 64 | 0 / 0 | 12,609 KiB |

Each changed body causes two full proxy-list passes in the existing resweep:
one to find that body's colliders, then another to locate unbounded colliders.
The visit totals are exactly `2 * 20,064 proxies * impacts`. As a result,
resweep refresh time grows from 0.506 ms at 16 impacts to 2.008 ms at 64,
despite updating only one proxy per impact. This isolates a CCD cost that is
linear in unrelated static proxies and motivates the next linked-list and
unbounded-proxy lookup change.

The full step exceeds a 16.67 ms frame at this scene size, but CCD accounts for
0.669–2.607 ms of the median step; spatial index construction and broadphase
account for roughly 21 ms combined. Those timings are recorded as context and
are outside this resweep optimization's scope.

## CCD follow-up stage 2: body-local resweep

`resweep_ccd_bodies()` now walks the changed body's existing collider links.
Each collider ID is checked against the owning body and the broadphase proxy
lookup before its swept bounds are updated. New candidates still come from the
spatial index; unbounded colliders are read from the world's maintained
unbounded-collider list instead of scanning every proxy.

The test covers compound bodies with 1, 10, 100, and 1,000 colliders, removes
and replaces a component before stepping, and keeps 64 unrelated static
colliders in the scene. In every case, one impact updates and visits exactly
the number of live colliders attached to the moving body.

| Scene | Step min / median / max | CCD min / median / max | Impacts / resweeps | Resweep visits / proxies updated | Refresh median |
|---|---:|---:|---:|---:|---:|
| 64 fast spheres vs. wall | 0.664 / 0.678 / 0.692 ms | 0.529 / 0.533 / 0.540 ms | 64 / 64 | 64 / 64 | 0.046 ms |
| 1k statics + 64 mixed | 0.814 / 0.867 / 0.883 ms | 0.141 / 0.143 / 0.145 ms | 16 / 16 | 16 / 16 | 0.015 ms |
| 20k statics + 64 mixed | 20.695 / 20.938 / 21.533 ms | 0.168 / 0.172 / 0.178 ms | 16 / 16 | 16 / 16 | 0.021 ms |
| 20k statics + 128 mixed | 20.693 / 21.025 / 21.744 ms | 0.313 / 0.320 / 0.330 ms | 32 / 32 | 32 / 32 | 0.037 ms |
| 20k statics + 256 mixed | 21.432 / 21.479 / 21.817 ms | 0.608 / 0.615 / 0.644 ms | 64 / 64 | 64 / 64 | 0.078 ms |

At 20k static colliders, direct visits fell from 642,048 / 1,288,192 /
2,592,768 to 16 / 32 / 64 for the three mixed scenes. The refresh medians
dropped from 0.506 / 1.046 / 2.008 ms to 0.021 / 0.037 / 0.078 ms. The 1k
and 20k scenes with 64 dynamic bodies both visit 16 proxies, showing that
unrelated finite colliders do not add work to the changed-body traversal.

## CCD follow-up stage 3: local impact-budget clipping

The source-compatible `CcdLimitPolicy::ClipIsland` setting now clips only
dynamic bodies participating in the unresolved contact. Their state is fixed
at that contact pose for the remainder of the step, their CCD candidates are
invalidated and reswept, and the solver continues with the other bodies in the
same island. A moving kinematic body keeps its authored trajectory. The
benchmark-only clipped-body count records distinct bodies across a global
replay.

Regression coverage includes a clipped body inside a 100-body CCD island while
another body continues moving, two budget-exhausted bodies meeting at a shared
contact, repeated wall impacts, a moving kinematic contact, contact-event
publication, and `AbortStep` rollback. The full `kapitsa-core` test suite with
`bench-internals` passed 474 unit tests, 16 integration/query/geometry tests,
and the doctests. Core and workspace Clippy passed with warnings denied.

## CCD follow-up stage 4: localized cross-island replay

When a resweep discovers a dynamic pair assigned to another CCD island, the
candidate becomes a temporary graph bridge. The engine restores and replays
only the resulting connected component, invalidating the TOI cache and staged
interactions for its incident candidates. Completed independent islands remain
staged and are skipped. A bridge with no hit on the initial trajectories stays
deferred in that component so impacts created by earlier rebounds can activate
it later. Static and kinematic surfaces remain shared without joining dynamic
components. Full-scene replay is retained for the bounded non-convergence path.

Tests cover two independent A-to-B-to-C rebound cascades, a dependency on an
already completed island, an independent body using the same static wall,
deterministic impact-event ordering, and `AbortStep` rollback followed by a
successful retry. These scenarios report local replay and zero global fallback.
The existing layout tests also verify that shared static and kinematic bodies
do not merge otherwise independent dynamic islands. Final release timing for
the rebound and cascade scenes is included in the stage 9 benchmark matrix.

## CCD follow-up stage 5: island result staging

Initial per-island TOI evaluation now writes a reusable private result containing
the island's dynamic-body snapshot, candidate TOIs, and staged interactions.
Workers read the shared world and use local narrow-phase scratch; the coordinator
merges results by island and candidate order. This keeps staged data private to
the step and provides the output shape for isolated island execution. The impact
resolver remains sequential in this stage. The expensive-island worker
equivalence test passes, and the result buffers retain their capacity between
steps.

## CCD follow-up stage 6: parallel independent impacts

Independent islands containing one candidate now evaluate and solve their
first impact in bounded worker tasks. Cost-weighted contiguous ranges account
for the shape pair's expected sweep cost. Each task owns its result and solver
scratch; the coordinator applies staged body states and interactions in island
order, then updates proxies and performs resweeps. A newly discovered bridge
uses the existing localized replay path. Small scenes and islands with multiple
candidates or an exhausted CCD budget retain the sequential resolver. The
parallel-island metric reports how many island tasks used this path; the normal
worker-task metric reports the number of tasks launched.

Core equivalence tests compare the sequential `step()` API with 2 and 4 workers
for independent fast bodies hitting a shared static wall. They compare body
states and ordered collision events. Existing Tokio checks for independent
islands, connected cascades, localized replay, and sequential/worker equivalence
also pass.

## CCD follow-up stage 7: reusable collision executor

This section records the pre-unification executor. CCD phases then used one
lazily initialized Rayon pool shared by `kapitsa-core`; Stage 10 moved pool
ownership to the optional `kapitsa` async runtime. The measurements remain a
historical scoped-thread versus pooled comparison, not the current pool
lifecycle.

The scoped-thread baseline and pooled executor were measured with the same
Release harness on 2026-11-01. Each value is the median of five independently
constructed worlds after one warm-up. Results below compare GJK-heavy cuboid
islands, where both implementations take the parallel path:

| Bodies | Workers | Scoped step / CCD ms | Reused pool step / CCD ms |
|---:|---:|---:|---:|
| 64 | 4 | 0.852 / 0.651 | 0.765 / 0.524 |
| 128 | 4 | 1.608 / 1.167 | 1.421 / 0.981 |
| 256 | 4 | 3.148 / 2.232 | 3.034 / 1.966 |

The medians trend lower with the reusable pool, while sample ranges overlap;
these runs do not establish a general speedup. For analytic sphere-wall CCD,
parallel work below the estimated cost threshold regressed on 8 to 64 islands,
so that workload now stays sequential until the estimated island cost reaches
256. Small scenes therefore avoid both worker setup and solver scratch growth.
The expanded benchmark covers up to 1,024 independent CCD islands.

## CCD follow-up stage 9: final benchmark matrix

Release benchmarks were rerun on 2026-11-01 with `rustc 1.98.0` on the same
AMD Ryzen 9 7900X3D / Windows MSVC machine as the follow-up baseline. Each
scenario used one warm-up and five samples; world construction was excluded
from step timings. Times are min / median / max in milliseconds unless noted.

### Resweep against a large static scene

The original baseline is the completed CCD refactor at `038e4d4`, before this
follow-up. These mixed workloads contain 20,000 static proxies and 64, 128, or
256 fast dynamic bodies; one quarter of the bodies hit static walls. The
resweep counters count visits to live colliders of changed bodies.

| Scenario | Baseline step / CCD median | Final step / CCD min / median / max | Baseline / final proxy visits | Final proxies updated |
|---|---:|---:|---:|---:|
| 20k statics + 64 dynamic | 22.172 / 0.669 | 19.656 / 20.470 / 20.585; 0.167 / 0.170 / 0.173 | 642,048 / 16 | 16 |
| 20k statics + 128 dynamic | 23.140 / 1.329 | 19.964 / 20.724 / 21.238; 0.311 / 0.321 / 0.333 | 1,288,192 / 32 | 32 |
| 20k statics + 256 dynamic | 24.126 / 2.607 | 21.070 / 21.584 / 21.824; 0.612 / 0.619 / 0.644 | 2,592,768 / 64 | 64 |

The median CCD phase fell by 74.6%, 75.8%, and 76.3%. Full-step medians
fell by 7.7%, 10.4%, and 10.5%. The final 64 / 128 / 256 impact cases each
made 16 / 32 / 64 local resweeps, one per impacted body, with zero global
fallbacks and zero extra candidate pairs. The 20k static-scene step still
exceeds the 16.67 ms frame budget: index and broadphase work dominate while
CCD itself is 0.170–0.619 ms.

### CCD worker scaling

For the 64-sphere analytic wall case, the original 038e4d4 baseline was
0.685 / 0.690 / 0.700 ms per step and 0.542 / 0.545 / 0.556 ms in CCD. The
final run measured 0.666 / 0.673 / 0.690 ms per step and 0.526 / 0.537 /
0.547 ms in CCD. These ranges overlap; this fast path has no established
before/after change and stays sequential at this size.

The final GJK-heavy 1,024-island workload compares active worker counts:

| Workers | Step min / median / max | CCD min / median / max | Parallel islands / tasks | Retained scratch |
|---:|---:|---:|---:|---:|
| 1 | 16.937 / 17.499 / 17.521 | 13.893 / 14.308 / 14.393 | 0 / 0 | 2,746 KiB |
| 2 | 13.209 / 13.631 / 14.363 | 8.975 / 9.374 / 9.995 | 1,024 / 4 | 6,033 KiB |
| 4 | 10.237 / 10.459 / 11.251 | 6.117 / 6.282 / 7.014 | 1,024 / 8 | 6,033 KiB |

Four workers reduced the median step by 40.2% relative to one worker; the
sample ranges do not overlap. The four-worker step is below the 16.67 ms
budget, while the one-worker step is slightly above it. Parallel scratch is
about 2.2x the one-worker estimate. The parallel impact batch covers
independent islands with one initial candidate; multi-candidate islands and
later impacts discovered during resweep continue through the ordered
coordinator path.

For the contact solver, 4,096 dynamic bodies in 2,048 two-body islands measured
41.723 / 30.846 / 23.765 ms median step time at 1 / 2 / 4 workers. The four
worker result is 1.76x faster than one worker but remains above 16.67 ms.

### Other measured workloads

| Scenario | Final min / median / max | Notes |
|---|---:|---|
| 20k dynamic bodies, no contacts | 25.984 / 26.131 / 29.360 ms | Baseline and final ranges overlap; still above frame budget |
| 1k bodies, one contact island | 17.237 / 17.248 / 17.392 ms | 999 contacts, one solver island; slightly above frame budget |
| 20k static + 100 separated dynamic | 20.908 / 21.295 / 21.585 ms | No contacts; index and broadphase dominate |
| 20k-static raycast load | 0.769 / 0.778 / 0.778 ms per 1,000 rays | 0.778 us per ray |
| 1,000 sweeps against the triangle mesh | 281.311 / 282.237 / 285.843 ms | Overlapping range with the earlier 269.327 / 278.131 / 293.291 ms baseline |
| 20k dynamic + 128 contact pairs | 23.985 / 24.938 / 29.030 ms | 128 contacts; no logical scratch growth |

The Tokio localized-collision harness measured eight steps. At 20,000 bodies,
`PhysicsWorld::step()` was 200.72 / 203.40 / 212.04 ms and the four-worker
controller was 190.12 / 195.77 / 198.63 ms (1.04x by medians). The ranges do
not overlap for this run, but the absolute improvement is small. The broader
`parallel_step` harness confirms workload sensitivity: dynamic-heavy scenes
benefit from dynamic buckets at 4,096 and 20,000 bodies, while 256-body,
mixed, and sleeping-heavy scenes generally lose time to scheduling.

All four required Release benchmark commands completed. Full workspace Release
tests, both Clippy commands, formatting check, and workspace documentation
also passed; the final report records the exact validation commands and
remaining performance limits.

## Final conclusions

The follow-up reduced 20k-static CCD resweep visits from a scene-wide scan to
the changed body's live collider list and removed the global replay from the
measured rebound/cascade cases. For a 1,024-island GJK-heavy workload, four
workers reduced median step time from 17.499 ms to 10.459 ms. The 64-sphere
analytic path stays sequential and remains within the same measurement range
as the 038e4d4 baseline.

The resolver parallelizes initial TOI work and the first impact for independent
islands that each have one candidate. The coordinator performs resweeps in
stable order; multi-candidate islands, later impacts, and discovered bridges
continue through ordered resolution and localized replay. See the [final CCD
optimization report](../docs/reviews/ccd-final-optimization-report.md) for
algorithm details, transaction guarantees, workload limitations, and the
complete validation record.

## Runtime benchmark comparisons

The earlier measurements below predate the unified public facade and are kept
for historical comparison. The current `kapitsa::Simulation` API and lifecycle
are described in [`docs/runtime.md`](../docs/runtime.md).

The earlier Stage 7 pool description is historical. Each async simulation now
owns its pool unless it receives a shared host-owned executor; explicit
`shutdown().await` waits for worker exit. CCD motion sampling stores a compact
`CcdTrajectory` value, so the compute path has no `dyn Fn` callback.

### Paired 20k-static CCD comparison

The pre-runtime CCD commit (`ff32c76`) and the current runtime migration were
measured in Release on the same machine with one warm-up and five samples per
scene. The old commit ran in a temporary detached worktree, which was removed
after the comparison. Step and CCD columns show min / median / max in ms.

| Scene | Pre-runtime step | Current step | Pre-runtime CCD | Current CCD | Resweep visits / proxies updated |
|---|---:|---:|---:|---:|---:|
| 20k statics + 64 CCD bodies | 23.376 / 25.727 / 25.997 | 23.888 / 24.355 / 24.990 | 0.167 / 0.185 / 0.279 | 0.169 / 0.176 / 0.222 | 16 / 16 |
| 20k statics + 128 CCD bodies | 23.191 / 24.328 / 25.514 | 23.886 / 25.046 / 25.418 | 0.333 / 0.391 / 0.569 | 0.396 / 0.451 / 0.470 | 32 / 32 |
| 20k statics + 256 CCD bodies | 25.535 / 26.175 / 26.788 | 25.017 / 26.748 / 27.339 | 0.678 / 0.743 / 0.916 | 0.653 / 0.695 / 0.907 | 64 / 64 |

All before/after step and CCD sample ranges overlap. These paired runs show no
measurable change from runtime consolidation. The resweep count remains tied
to changed body colliders, while index and broadphase work dominate the 20k
static full step. The run also measured 31.860 ms for 20k separated dynamic
bodies versus 32.293 ms before consolidation; these ranges overlap as well.

### Unified API and persistent-force-source regression check

Release comparison on 2026-11-01, AMD Ryzen 9 7900X3D, `rustc 1.98.0`.
Both revisions used this benchmark with one warm-up and five samples. Stage 3
is commit `5496401`; Stage 4 is commit `51ce104`. These scenes contain no force
sources, so the comparison checks the added empty-source path against the
previous CCD and collision pipeline.

| Scenario | Stage 3 min / median / max ms | Stage 4 min / median / max ms |
|---|---:|---:|
| 20,000 separated Dynamic spheres | 26.573 / 27.160 / 27.307 | 27.044 / 27.854 / 28.153 |
| 20,000 bodies, localized contacts | 30.150 / 30.605 / 30.670 | 30.192 / 31.773 / 32.137 |
| 64 fast CCD bodies, total step | 0.692 / 0.694 / 0.718 | 0.682 / 0.697 / 0.705 |
| 64 fast CCD bodies, CCD phase | 0.545 / 0.548 / 0.558 | 0.546 / 0.548 / 0.562 |
| 20,000 statics + 64 CCD bodies, total step | 20.683 / 20.877 / 21.016 | 20.375 / 21.009 / 22.116 |
| 20,000 statics + 64 CCD bodies, CCD phase | 0.171 / 0.180 / 0.192 | 0.183 / 0.188 / 0.196 |

Every paired min/max range overlaps. The CCD phase median is unchanged for the
fast-body case and 0.008 ms higher for the static-load case; the latter ranges
overlap. No consistent regression is visible in these runs. With no configured
sources, each step exits the new preparation path on an empty-pool check and
does not walk bodies or allocate force-source scratch.

### Async runtime worker measurements

The `kapitsa` async collision benchmark now includes independent GJK CCD
islands and times a single impact step per sample. World creation and pool
creation are outside the timed region. These medians are milliseconds:

| GJK CCD bodies | `PhysicsWorld::step` | Async x1 | Async x2 | Async x4 | x4 speedup |
|---:|---:|---:|---:|---:|---:|
| 256 | 4.029 | 4.086 | 3.003 | 2.036 | 1.98x |
| 1,024 | 16.567 | 16.697 | 12.038 | 9.498 | 1.74x |

For 1,024 bodies, the x1 range was 16.143–17.246 ms and the four-worker range
was 9.248–9.629 ms. In the separate solver workload with 4,096 dynamic bodies
and 2,048 two-body contact islands, 12-step medians were 30.58 / 22.78 / 16.02
ms at x1 / x2 / x4. Four workers were 1.91x faster than one in that run. The
broader `parallel_step` matrix still shows scheduling losses for small, mixed,
and sleeping-heavy scenes.

The updated worker benches use explicit scoped executors in core and the real
Rayon/Tokio runtime in `kapitsa`. This matters because the compatibility
`commit_step_shards_with_workers` API is intentionally sequential now.

Run the unified-runtime comparisons with:

```powershell
cargo bench -p kapitsa-core --features bench-internals --bench collisions
cargo bench -p kapitsa-core --features bench-internals --bench collision_storage -- workers_4096
cargo bench -p kapitsa --bench collisions
cargo bench -p kapitsa --bench parallel_step
```
