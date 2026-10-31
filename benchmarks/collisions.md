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
| 256 separated dynamic spheres | 0.227 | 0.134 | 0.033 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.001 | 0 | 0 | 0 | 0 | 168 KiB |
| 4,096 separated dynamic spheres | 4.759 | 2.386 | 1.245 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.021 | 0 | 0 | 0 | 0 | 2,688 KiB |
| 20,000 separated dynamic spheres | 27.293 | 13.015 | 8.557 | 0.000 | 0.000 | 0.001 | 0.000 | 0.000 | 0.258 | 0 | 0 | 0 | 0 | 16,420 KiB |
| 20,000 bodies, 128 localized plane contacts | 37.566 | 12.964 | 9.394 | 4.613 | 0.197 | 0.002 | 0.839 | 0.010 | 0.211 | 20,000 | 128 | 0 | 128 | 34,343 KiB |
| 1,000 overlapping cuboids, one island | 17.538 | 0.400 | 0.191 | 3.020 | 0.002 | 0.014 | 12.449 | 0.054 | 0.011 | 999 | 999 | 0 | 1 | 4,034 KiB |
| 20,000 static + 100 separated dynamic spheres | 23.550 | 12.399 | 9.295 | 0.000 | 0.000 | 0.001 | 0.000 | 0.000 | 0.229 | 0 | 0 | 0 | 0 | 16,446 KiB |
| Large triangle mesh + 100 movers | 1.507 | 0.060 | 0.028 | 0.490 | 0.000 | 0.001 | 0.762 | 0.018 | 0.001 | 100 | 100 | 0 | 100 | 759 KiB |
| Heightfield + 512 movers | 6.338 | 0.268 | 0.173 | 0.927 | 0.001 | 0.007 | 4.195 | 0.098 | 0.004 | 512 | 512 | 0 | 512 | 4,151 KiB |
| Level set + 256 movers | 3.305 | 0.139 | 0.078 | 0.760 | 0.000 | 0.004 | 1.929 | 0.045 | 0.002 | 256 | 256 | 0 | 256 | 2,076 KiB |
| 64 fast CCD bodies vs. thin wall | 259.986 | 0.034 | 0.019 | 0.061 | 259.802 | 0.001 | 0.000 | 0.023 | 0.001 | 64 | 0 | 64 | 0 | 145 KiB |

The 20,000 separated-body case emitted zero candidate pairs. Its index and broadphase times were 13.015 ms and 8.557 ms. Against the 4,096-body case, the body count grew 4.88x and those phases grew 5.45x and 6.87x respectively. This sample gives no sign of an all-pairs candidate scan; it does not establish scaling for every distribution.

Spatial-query load used 20,000 static spheres and 100 raycasts per sample. The median was 1,254.974 ms for 100 rays (12,549.743 us per query). This is a notably expensive workload and should be treated as a follow-up optimization target if query-heavy scenes matter.

## Tokio worker comparison

Each entry is the median elapsed time for eight collision steps. The worlds contain up to 128 isolated sphere-plane contacts, with remaining dynamic spheres separated above the plane. Controller setup is outside the timed region.

| Bodies | `PhysicsWorld::step` | Controller x1 | Controller x2 | Controller x4 |
|---:|---:|---:|---:|---:|
| 256 | 3.81 ms | 4.04 ms (0.94x) | 4.79 ms (0.80x) | 5.16 ms (0.74x) |
| 4,096 | 54.13 ms | 53.89 ms (1.00x) | 53.22 ms (1.02x) | 55.39 ms (0.98x) |
| 20,000 | 287.58 ms | 265.41 ms (1.08x) | 281.81 ms (1.02x) | 281.84 ms (1.02x) |

Worker scheduling cost more for the small scene. At 4,096 and 20,000 bodies, this collision workload was near parity across worker counts; contact processing remains part of the step and the benchmark does not claim a parallel speedup for every collision distribution.

## Memory estimate limits

`Retained scratch` estimates capacities of collision-owned vectors and hash tables after the measured step. `High-water` is the maximum of the five post-step estimates. The estimate excludes `PhysicsWorld` bodies and collider geometry, allocator metadata and fragmentation, runtime/task stacks, and process-wide allocations. It is not process RSS or a peak allocation measurement.

## Workload notes

- The 20,000-body localized-contact scene has one infinite plane, 128 spheres touching it, and the remaining spheres elevated. The plane yields 20,000 broadphase candidates, of which 128 create contacts.
- The 1,000-body island is a chain of overlapping cuboids; it creates 999 candidates and one connected solver island.
- The 20,000-static/100-dynamic scene keeps the dynamic bodies separated to isolate mixed body-count indexing and broadphase cost.
- Triangle mesh, heightfield, and level-set scenes move spheres into their surface during the measured step.
- The CCD workload sends 64 continuous spheres at a thin static wall and resolves 64 impacts.
- No collision thresholds or algorithms were tuned to reach these values.
