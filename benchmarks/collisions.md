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
| 256 separated dynamic spheres | 0.285 | 0.159 | 0.042 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.002 | 0 | 0 | 0 | 0 | 168 KiB |
| 4,096 separated dynamic spheres | 5.178 | 2.478 | 1.461 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.046 | 0 | 0 | 0 | 0 | 2,688 KiB |
| 20,000 separated dynamic spheres | 27.443 | 13.230 | 8.467 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.228 | 0 | 0 | 0 | 0 | 16,420 KiB |
| 20,000 bodies, 128 localized sphere pairs | 30.116 | 12.978 | 9.204 | 0.032 | 0.188 | 0.003 | 1.616 | 0.009 | 0.175 | 128 | 128 | 0 | 128 | 18,580 KiB |
| 1,000 overlapping cuboids, one island | 17.570 | 0.395 | 0.197 | 3.075 | 0.002 | 0.016 | 12.479 | 0.062 | 0.011 | 999 | 999 | 0 | 1 | 4,034 KiB |
| 20,000 static + 100 separated dynamic spheres | 22.922 | 12.111 | 9.297 | 0.000 | 0.000 | 0.000 | 0.001 | 0.001 | 0.242 | 0 | 0 | 0 | 0 | 16,446 KiB |
| Large triangle mesh + 100 movers | 1.484 | 0.069 | 0.024 | 0.487 | 0.000 | 0.002 | 0.746 | 0.017 | 0.001 | 100 | 100 | 0 | 100 | 759 KiB |
| Heightfield + 512 movers | 6.446 | 0.244 | 0.176 | 0.958 | 0.001 | 0.005 | 4.258 | 0.103 | 0.004 | 512 | 512 | 0 | 512 | 4,151 KiB |
| Level set + 256 movers | 3.365 | 0.120 | 0.079 | 0.753 | 0.000 | 0.002 | 1.992 | 0.049 | 0.001 | 256 | 256 | 0 | 256 | 2,076 KiB |
| 64 fast CCD bodies vs. thin wall | 261.606 | 0.062 | 0.027 | 0.062 | 261.379 | 0.001 | 0.000 | 0.022 | 0.001 | 64 | 0 | 64 | 0 | 145 KiB |

The 20,000 separated-body case emitted zero candidate pairs. Its index and broadphase times were 13.230 ms and 8.467 ms. Against the 4,096-body case, the body count grew 4.88x and those phases grew 5.34x and 5.80x respectively. This sample gives no sign of an all-pairs candidate scan; it does not establish scaling for every distribution.

Spatial-query load used 20,000 static spheres and 100 raycasts per sample. The median was 1,352.353 ms for 100 rays (13,523.535 us per query). This is a notably expensive workload and should be treated as a follow-up optimization target if query-heavy scenes matter.

## Tokio worker comparison

Each entry is the median elapsed time for eight collision steps. The worlds contain 16 or 128 overlapping dynamic sphere pairs with remaining spheres separated well away. Controller setup is outside the timed region.

| Bodies | `PhysicsWorld::step` | Controller x1 | Controller x2 | Controller x4 |
|---:|---:|---:|---:|---:|
| 256 | 2.89 ms | 3.18 ms (0.91x) | 4.39 ms (0.66x) | 4.17 ms (0.69x) |
| 4,096 | 42.66 ms | 44.67 ms (0.96x) | 37.88 ms (1.13x) | 37.11 ms (1.15x) |
| 20,000 | 198.16 ms | 192.26 ms (1.03x) | 195.21 ms (1.02x) | 176.26 ms (1.12x) |

Worker scheduling cost more for the small scene. At 4,096 bodies, two and four workers improved this collision workload by 13% and 15%. At 20,000 bodies, two and four workers improved it by 2% and 12%. These results apply to this workload and machine, not every collision distribution.

## Memory estimate limits

`Retained scratch` estimates capacities of collision-owned vectors and hash tables after the measured step. `High-water` is the maximum of the five post-step estimates. The estimate excludes `PhysicsWorld` bodies and collider geometry, allocator metadata and fragmentation, runtime/task stacks, and process-wide allocations. It is not process RSS or a peak allocation measurement.

## Workload notes

- The 20,000-body localized-contact scene has 128 overlapping sphere pairs and the remaining spheres far away. It yields 128 broadphase candidates and contacts.
- The 1,000-body island is a chain of overlapping cuboids; it creates 999 candidates and one connected solver island.
- The 20,000-static/100-dynamic scene keeps the dynamic bodies separated to isolate mixed body-count indexing and broadphase cost.
- Triangle mesh, heightfield, and level-set scenes move spheres into their surface during the measured step.
- The CCD workload sends 64 continuous spheres at a thin static wall and resolves 64 impacts.
- No collision thresholds or algorithms were tuned to reach these values.
