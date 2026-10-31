# Collision pipeline benchmarks

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

## Workload notes

- The 20,000-body localized-contact scene has 128 overlapping sphere pairs and the remaining spheres far away. It yields 128 broadphase candidates and contacts.
- The 1,000-body island is a chain of overlapping cuboids; it creates 999 candidates and one connected solver island.
- The 20,000-static/100-dynamic scene keeps the dynamic bodies separated to isolate mixed body-count indexing and broadphase cost.
- Triangle mesh, heightfield, and level-set scenes move spheres into their surface during the measured step.
- The CCD workload sends 64 continuous spheres at a thin static wall and resolves 64 impacts.
- No collision thresholds or algorithms were tuned to reach these values.
