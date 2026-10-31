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
| 4,096 separated dynamic spheres | 4.780 | 2.355 | 1.377 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.036 | 0 | 0 | 0 | 0 | 2,688 KiB |
| 20,000 separated dynamic spheres | 27.327 | 12.710 | 8.741 | 0.000 | 0.000 | 0.001 | 0.000 | 0.000 | 0.227 | 0 | 0 | 0 | 0 | 16,420 KiB |
| 20,000 bodies, 128 localized sphere pairs | 30.513 | 12.938 | 9.509 | 0.032 | 0.192 | 0.004 | 1.658 | 0.011 | 0.228 | 128 | 128 | 0 | 128 | 18,580 KiB |
| 1,000 overlapping cuboids, one island | 17.839 | 0.441 | 0.196 | 3.134 | 0.002 | 0.020 | 12.672 | 0.053 | 0.011 | 999 | 999 | 0 | 1 | 4,034 KiB |
| 20,000 static + 100 separated dynamic spheres | 23.030 | 11.910 | 9.396 | 0.000 | 0.000 | 0.001 | 0.000 | 0.000 | 0.234 | 0 | 0 | 0 | 0 | 16,446 KiB |
| Large triangle mesh + 100 movers | 1.492 | 0.057 | 0.033 | 0.499 | 0.000 | 0.002 | 0.783 | 0.021 | 0.001 | 100 | 100 | 0 | 100 | 759 KiB |
| Heightfield + 512 movers | 6.364 | 0.234 | 0.172 | 0.941 | 0.001 | 0.005 | 4.210 | 0.103 | 0.005 | 512 | 512 | 0 | 512 | 4,151 KiB |
| Level set + 256 movers | 3.346 | 0.121 | 0.076 | 0.750 | 0.000 | 0.002 | 1.986 | 0.047 | 0.002 | 256 | 256 | 0 | 256 | 2,076 KiB |
| 64 fast CCD bodies vs. thin wall | 259.819 | 0.042 | 0.017 | 0.062 | 259.615 | 0.001 | 0.000 | 0.016 | 0.000 | 64 | 0 | 64 | 0 | 145 KiB |

The 20,000 separated-body case emitted zero candidate pairs. Its index and broadphase times were 12.710 ms and 8.741 ms. Against the 4,096-body case, the body count grew 4.88x and those phases grew 5.40x and 6.35x respectively. This sample gives no sign of an all-pairs candidate scan; it does not establish scaling for every distribution.

Spatial-query load used 20,000 static spheres and 100 raycasts per sample. The median was 1,296.339 ms for 100 rays (12,963.394 us per query). This is a notably expensive workload and should be treated as a follow-up optimization target if query-heavy scenes matter.

## Tokio worker comparison

Each entry is the median elapsed time for eight collision steps. The worlds contain 16 or 128 overlapping dynamic sphere pairs with remaining spheres separated well away. Controller setup is outside the timed region.

| Bodies | `PhysicsWorld::step` | Controller x1 | Controller x2 | Controller x4 |
|---:|---:|---:|---:|---:|
| 256 | 2.85 ms | 3.18 ms (0.89x) | 4.00 ms (0.71x) | 3.82 ms (0.75x) |
| 4,096 | 44.48 ms | 42.09 ms (1.06x) | 36.47 ms (1.22x) | 34.52 ms (1.29x) |
| 20,000 | 185.88 ms | 189.03 ms (0.98x) | 184.15 ms (1.01x) | 175.59 ms (1.06x) |

Worker scheduling cost more for the small scene. At 4,096 bodies, two and four workers improved this collision workload by 22% and 29%; at 20,000 bodies, four workers improved it by 6%. These results apply to this workload and machine, not every collision distribution.

## Memory estimate limits

`Retained scratch` estimates capacities of collision-owned vectors and hash tables after the measured step. `High-water` is the maximum of the five post-step estimates. The estimate excludes `PhysicsWorld` bodies and collider geometry, allocator metadata and fragmentation, runtime/task stacks, and process-wide allocations. It is not process RSS or a peak allocation measurement.

## Workload notes

- The 20,000-body localized-contact scene has 128 overlapping sphere pairs and the remaining spheres far away. It yields 128 broadphase candidates and contacts.
- The 1,000-body island is a chain of overlapping cuboids; it creates 999 candidates and one connected solver island.
- The 20,000-static/100-dynamic scene keeps the dynamic bodies separated to isolate mixed body-count indexing and broadphase cost.
- Triangle mesh, heightfield, and level-set scenes move spheres into their surface during the measured step.
- The CCD workload sends 64 continuous spheres at a thin static wall and resolves 64 impacts.
- No collision thresholds or algorithms were tuned to reach these values.
