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
| 256 separated dynamic spheres | 0.200 | 0.100 | 0.033 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.001 | 0 | 0 | 0 | 0 | 168 KiB |
| 4,096 separated dynamic spheres | 4.653 | 2.305 | 1.207 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.034 | 0 | 0 | 0 | 0 | 2,688 KiB |
| 20,000 separated dynamic spheres | 27.336 | 12.872 | 8.748 | 0.000 | 0.000 | 0.001 | 0.000 | 0.001 | 0.232 | 0 | 0 | 0 | 0 | 16,420 KiB |
| 20,000 bodies, 128 localized sphere pairs | 30.423 | 12.653 | 9.829 | 0.033 | 0.188 | 0.003 | 1.605 | 0.010 | 0.208 | 128 | 128 | 0 | 128 | 18,580 KiB |
| 1,000 overlapping cuboids, one island | 17.596 | 0.427 | 0.157 | 3.112 | 0.002 | 0.018 | 12.539 | 0.041 | 0.010 | 999 | 999 | 0 | 1 | 4,034 KiB |
| 20,000 static + 100 separated dynamic spheres | 22.299 | 11.539 | 9.277 | 0.000 | 0.000 | 0.001 | 0.000 | 0.000 | 0.224 | 0 | 0 | 0 | 0 | 16,446 KiB |
| Large triangle mesh + 100 movers | 1.468 | 0.059 | 0.028 | 0.501 | 0.000 | 0.003 | 0.750 | 0.020 | 0.001 | 100 | 100 | 0 | 100 | 759 KiB |
| Heightfield + 512 movers | 6.360 | 0.245 | 0.188 | 0.940 | 0.001 | 0.005 | 4.224 | 0.102 | 0.003 | 512 | 512 | 0 | 512 | 4,151 KiB |
| Level set + 256 movers | 3.294 | 0.128 | 0.087 | 0.784 | 0.001 | 0.004 | 1.893 | 0.050 | 0.002 | 256 | 256 | 0 | 256 | 2,076 KiB |
| 64 fast CCD bodies vs. thin wall | 260.634 | 0.033 | 0.018 | 0.063 | 260.455 | 0.000 | 0.000 | 0.021 | 0.001 | 64 | 0 | 64 | 0 | 145 KiB |

The 20,000 separated-body case emitted zero candidate pairs. Its index and broadphase times were 12.872 ms and 8.748 ms. Against the 4,096-body case, the body count grew 4.88x and those phases grew 5.58x and 7.25x respectively. This sample gives no sign of an all-pairs candidate scan; it does not establish scaling for every distribution.

Spatial-query load used 20,000 static spheres and 100 raycasts per sample. The median was 1,294.473 ms for 100 rays (12,944.727 us per query). This is a notably expensive workload and should be treated as a follow-up optimization target if query-heavy scenes matter.

## Tokio worker comparison

Each entry is the median elapsed time for eight collision steps. The worlds contain 16 or 128 overlapping dynamic sphere pairs with remaining spheres separated well away. Controller setup is outside the timed region.

| Bodies | `PhysicsWorld::step` | Controller x1 | Controller x2 | Controller x4 |
|---:|---:|---:|---:|---:|
| 256 | 2.82 ms | 3.33 ms (0.85x) | 3.64 ms (0.77x) | 3.95 ms (0.71x) |
| 4,096 | 40.97 ms | 42.13 ms (0.97x) | 36.26 ms (1.13x) | 35.73 ms (1.15x) |
| 20,000 | 184.62 ms | 184.09 ms (1.00x) | 182.22 ms (1.01x) | 173.23 ms (1.07x) |

Worker scheduling cost more for the small scene. At 4,096 bodies, two and four workers improved this collision workload by 13% and 15%. At 20,000 bodies, two and four workers improved it by 1% and 7%. These results apply to this workload and machine, not every collision distribution.

## Memory estimate limits

`Retained scratch` estimates capacities of collision-owned vectors and hash tables after the measured step. `High-water` is the maximum of the five post-step estimates. The estimate excludes `PhysicsWorld` bodies and collider geometry, allocator metadata and fragmentation, runtime/task stacks, and process-wide allocations. It is not process RSS or a peak allocation measurement.

## Workload notes

- The 20,000-body localized-contact scene has 128 overlapping sphere pairs and the remaining spheres far away. It yields 128 broadphase candidates and contacts.
- The 1,000-body island is a chain of overlapping cuboids; it creates 999 candidates and one connected solver island.
- The 20,000-static/100-dynamic scene keeps the dynamic bodies separated to isolate mixed body-count indexing and broadphase cost.
- Triangle mesh, heightfield, and level-set scenes move spheres into their surface during the measured step.
- The CCD workload sends 64 continuous spheres at a thin static wall and resolves 64 impacts.
- No collision thresholds or algorithms were tuned to reach these values.
