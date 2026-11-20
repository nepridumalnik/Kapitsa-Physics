# Scalar aerodynamic baseline

Recorded 2026-11-14 from production baseline `97e9f36`, with only the benchmark
and relevant documentation added. No physics equations or production code changed.
This is a pre-SIMD local comparison baseline, not a performance acceptance gate.

## Reproduction and environment

```powershell
cargo bench -p kapitsa-core --bench aerodynamics
```

Source: [permanent benchmark](../crates/kapitsa-core/benches/aerodynamics.rs).
It follows the neighboring [math baseline](math-baseline.md): harness-free Cargo
bench target, `std::time::Instant`, seven samples, median and min/max, and
`std::hint::black_box` at kernel input/output boundaries. It needs no extra
dependencies or feature flags. Cargo's default optimized bench profile is used;
there is no explicit target-cpu/native setting, SIMD kernel or parallel dispatch.

- AMD Ryzen 9 7900X3D, 12 cores / 24 logical processors.
- Windows 11 Pro 64-bit, version 10.0.26300.
- `rustc 1.98.0 (88d9e12ae 2026-08-18)`, LLVM 22.1.8.
- `cargo 1.98.0 (797e8a9bc 2026-08-05)`, target `x86_64-pc-windows-msvc`.
- `RUSTFLAGS` and `CARGO_ENCODED_RUSTFLAGS` unset/empty.
- Ordinary local run: no CPU affinity, frequency pinning or isolated-host control.

Each kernel has three untimed warmup operations. All geometry/model construction
and correctness assertions occur before timing. Each operation uses fixed inputs;
only cache-miss geometry versions advance to force invalidation. Results are
consumed through `black_box`. Timings include loop overhead, successful `Result`
unwrapping and output destruction; allocating preparation includes allocation
and freeing, not input construction. There are no timing assertions or arbitrary
pass thresholds. Sample ranges are observed variability, not confidence intervals.

## Workloads and untimed validation

Both sizes use one deterministic open mesh instance, body ownership, identity
transform and `SurfaceOnly`. Each tile contains a rear unit square at X=0,
an exact coplanar duplicate, and an upstream unit square at X=1 offset by 0.5 in
Y. Tiles are separated by 3 m in Y. Every square has two +X triangles. This
exercises duplicate overlap removal, partial occlusion, clipping/triangulation,
and unobstructed upstream geometry in the same workload, not merely a fully
visible mesh. Uniform upstream direction is +X. Closure welding is disabled.

Cell size is 0.25 m; budgets are 100,000 cells and 100,000,000 voxel/patch work
units. Compatibility exposure epsilon is 0.01 m (it does not offset rays).

| Size | Tiles | Input triangles | Grid dimensions | Cells / occupied | Visible patches | Exposed area | Wing surfaces |
| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| Small | 1 | 6 | 6 x 8 x 6 | 288 / 50 | 5 | 1.5 m² | 2 |
| Moderate | 16 | 96 | 6 x 188 x 6 | 6768 / 800 | 80 | 24 m² | 32 |

Before timing the executable verifies repeatable voxel and patch preparation,
exact occupied count/volume (50 cells per tile), exposed area 1.5 per tile rather
than unfiltered area 3, and physical +X normals. Cache checks establish equality
with standalone preparation, retained pointer identity on a hit, and no current
entry after a version change, followed by correct rebuilding.

Pressure uses the prepared visible patches, density 1 kg/m³, sound speed 340 m/s,
COM velocity (2000,0,0) m/s, angular velocity (0.1,0,0) rad/s, zero wind/COM,
and identity pose. The expected force is (-4,000,000 * exposed_area,0,0) N;
rotation adds transverse point velocity but does not change Newtonian normal
pressure here. The moment is checked against the area-weighted patch-center
cross-product sum. Visibility is not recomputed in this kernel.

Wing loads use 2 or 32 symmetric explicit `WingSurface`s, each of area 1 m²,
with aspect ratio 8, efficiency 0.8, profile drag 0.02, zero-lift angle 0,
and attached limit 0.2 rad. Velocity is (60,0,-3) m/s, angular velocity
(0,0.1,0) rad/s, and other atmospheric/pose settings match pressure. Symmetric
Y offsets are parallel to the rotation axis, so each wing's point velocity is
known and equal. Untimed checks compare the sum with the single `FiniteWing`
force times surface count, require drag/positive lift and near-zero total moment.
This measures surface transforms, point-flow evaluation, coefficient/force work
and summation, not just the coefficient primitive.

## Release measurements

All values are **ns per whole operation**, not ns per triangle, patch or wing.

| Kernel | Iterations/sample small / moderate | Small median | Small min–max | Moderate median | Moderate min–max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Voxel preparation | 100 / 10 | 14256.00 | 14182.00–14504.00 | 271460.00 | 267480.00–328570.00 |
| Partial visibility | 100 / 10 | 16986.00 | 16597.00–20186.00 | 12010820.00 | 11709500.00–12358250.00 |
| Cache hit | 100000 / 100000 | 7.59 | 7.44–8.81 | 7.54 | 7.54–7.69 |
| Cache miss + preparation | 100 / 10 | 14538.00 | 14418.00–14612.00 | 279010.00 | 275470.00–280210.00 |
| Prepared pressure | 10000 / 10000 | 544.07 | 543.02–554.51 | 8211.08 | 8176.47–8357.42 |
| Finite-wing surfaces | 10000 / 10000 | 372.55 | 359.42–389.31 | 5667.14 | 5608.61–5893.17 |

Voxel preparation invokes `PreparedGeometry::prepare`, including topology checks,
rasterization and section construction. Partial visibility invokes
`exposed_surface_patches` on already prepared geometry, including transformation,
canonicalization, blocker processing and patch allocation, **without voxelization**.
Cache hit times the public synchronous `prepare_instances` path, including
request/config comparisons and ticket clone/drop. Cache miss changes the version
on the same single-entry cache and includes voxel rebuilding, publication and
replacement/destruction of the previous entry; it does **not** include visibility.
Prepared pressure times `evaluate_prepared_mesh`; wings time a checked evaluator
call per surface plus local wrench accumulation. No world stepping is timed.

Visibility dominates this moderate workload (~12 ms versus ~0.27 ms for voxel
preparation). That observation motivates measuring exposure separately; it does
not establish universal scaling or show that SIMD alone addresses blocker work.

The visibility implementation now builds a temporary, deterministic uniform grid
of transformed source triangles for each preparation. Upstream queries conservatively
walk the swept source bounds and deduplicate triangle candidates before the exact
shadow predicates and clipping run. The index is charged to the caller's patch
work budget and uses fallible allocation; it is deliberately not retained in
pose-dependent geometry caches. This is an expected scalability rationale, not a
new measured result.

## Approximation and coverage limits

See [physical models and references](../docs/aerodynamics.md),
[geometry implementation](../crates/kapitsa-core/src/aerodynamics/geometry.rs),
[pressure](../crates/kapitsa-core/src/aerodynamics/pressure.rs), and
[wing](../crates/kapitsa-core/src/aerodynamics/wing.rs). No math source was changed.

- Open plates do not benchmark closed-solid flood fill, welding, curved collider
  tessellation, assembly owner attribution, extreme predicates or large ships.
  Moderate means 96 input triangles, not a production-complexity hull.
- Occupied cells conservatively thicken the plates; volume is voxel bookkeeping,
  not their physical volume. Pressure uses original-surface normals and areas,
  never staircase voxel faces.
- Public visibility uses one uniform direction. Rotation-dependent per-source
  visibility used by world integration is not benchmarked. The chosen axial
  pressure rotation leaves X-normal incidence unchanged; general rotational
  quadrature and clipping validity require separate numerical tests.
- Pressure is Newtonian hypersonic impact (Mach >= 5), not low-speed hull flow.
  Wing evaluation is attached incompressible flow (Mach <= 0.3); no stall,
  compressibility, interference or inferred wing geometry is included.
- Repeated fixed data favors warm caches. Cache-miss timing is geometry-identity
  invalidation, not a claim about CPU hardware cache misses. No cold-memory,
  atmosphere lookup, world distribution, timestep integration, multithreading,
  SIMD throughput or cross-machine target is established.

## Verification

The scoped formatting check and core checks passed:

```powershell
rustfmt --edition 2024 --check crates/kapitsa-core/benches/aerodynamics.rs
cargo check -p kapitsa-core --all-targets
cargo clippy -p kapitsa-core --all-targets -- -D warnings
cargo test -p kapitsa-core aerodynamics
cargo bench -p kapitsa-core --bench aerodynamics
```

The focused test command passed 113 tests (792 unrelated unit tests filtered
out). The final release benchmark passed all untimed checks for both sizes and
produced the table above. `cargo fmt --all -- --check` was also run; it fails on
pre-existing formatting in `src/world.rs`, `src/world/cluster.rs` and
`src/world/aerodynamics_tests.rs` under `kapitsa-core`. Incidental formatting
changes there were discarded to preserve the benchmark-only scope. No production
source changes are included. Workspace-wide checks remain the supervising stage.

## Voxel wing shielding cache

`cargo bench -p kapitsa-core --bench wing_shielding` measures full sequential
world steps in an optimized build. The synthetic scene has one body, a
20 x 4 x 0.5 m geometry-only sensor cuboid, and 1 or 32 identical 20 x 4 m
rectangular wing footprints completely enclosed by it. The 0.125 m geometry
cell size gives 5,120 visibility samples per wing. The benchmark checks before
timing that the unshielded reference wing changes velocity and the enclosed
wing does not. It uses three warmups, seven samples of 30 tiny steps, and reports
median and individual sample times. It does not measure topology edits or a
representative fleet of distinct shapes.

On the same local machine with rustc 1.98.0 and cargo 1.98.0, a sequential run
before caching measured medians of 232.12 us/step for one enclosed wing and
7,262.25 us/step for 32. After caching, the corresponding medians were 1.89 and
12.18 us/step. The post-change unshielded reference was 8.19 and 20.44 us/step;
it continues evaluating wing forces, so its timings are not directly comparable
to fully shielded wings that contribute no force. These measurements are not a
performance guarantee or an accuracy validation.

## Wing force scaling fallback

The permanent `aerodynamics` benchmark was rerun locally after adding
exponent-scaled wing-force components. For the 32-wing workload, the scaled-only
implementation measured a 9,767 ns/operation median. Adding a guarded ordinary
range fast path reduced the median to 6,780 ns/operation; the two-wing medians
were 608 ns and 431 ns respectively. Both runs used seven samples on the same
machine and sequential execution. Untimed checks in the benchmark passed. The
extreme-value fallback is covered separately by unit tests and is not measured
by this ordinary-flight workload. These timings are observations, not thresholds.

﻿
## Surface visibility hardening

Local Windows x64 run on production commit `e412640`. Both benchmark targets
completed. The workspace test suite was active during this measurement.

```powershell
cargo bench -p kapitsa-core --bench aerodynamics --bench wing_shielding
```

| Partial-visibility fixture | Source triangles | Exposed patches | Median | Min/max |
| --- | ---: | ---: | ---: | ---: |
| Small | 6 | 5 | 26.35 us | 26.18 to 27.90 us |
| Moderate | 96 | 80 | 3.155 ms | 3.148 to 3.176 ms |

The wing-shielding benchmark measured medians of 8.49 and 2.23 us/step for
one wing without and with a footprint, respectively. For 32 wings the
medians were 21.44 and 12.94 us/step. These are workload timings. They do
not establish peak allocation or an isolated speedup over the historical
baseline.

The follow-up run on `07c7285`, with independent predicate scaling on each
axis, executed both targets before the workspace tests. The fixtures and
benchmark procedure were unchanged.

| Partial-visibility fixture | Median | Min/max |
| --- | ---: | ---: |
| Small | 25.96 us | 25.66 to 27.82 us |
| Moderate | 2.166 ms | 2.109 to 2.403 ms |

Wing-shielding medians without and with a footprint were 8.27 and
2.18 us/step for one wing, and 20.82 and 12.56 us/step for 32 wings.

The axial projection follow-up on `d4dbf07` measured 25.61 us
(25.07 to 28.51 us) for the small partial-visibility fixture and 2.076 ms
(2.069 to 2.083 ms) for the moderate fixture. Wing-shielding medians without
and with a footprint were 8.17 and 2.15 us/step for one wing, and 20.73 and
12.51 us/step for 32 wings.

The direct bucket construction on `7d6e5a6` measured 25.44 us
(24.94 to 26.62 us) for the small fixture and 2.057 ms (2.052 to
2.070 ms) for the moderate fixture. The grid builder needs one triangle
index per triangle-to-cell entry, two cell-sized arrays, and one additional
prefix offset during construction. This is a logical storage count. The
measurement does not include allocator overhead or process peak memory.

The split-buffer reservation change on `8cf7271` measured 25.65 us
(25.23 to 27.05 us) for the small fixture and 2.068 ms (2.061 to
2.093 ms) for the moderate fixture. These ranges overlap the preceding
measurements. The change bounds the requested split storage; the timing
run provides no evidence of a speedup.

The coordinate-aligned depth change on `9ac852c` measured 20.70 us
(20.19 to 22.10 us) for the small fixture and 1.301 ms (1.288 to
1.339 ms) for the moderate fixture. These fixtures use coordinate-aligned
plates. The results give no timing estimate for inclined geometry.

The exact depth fallback on `0337ef5` measured 20.66 us (20.13 to
22.22 us) on the small fixture and 1.284 ms (1.279 to 1.287 ms)
on the moderate fixture. These fixtures exercise the common path.
The fallback itself has no timing measurement in this run.

The projected orientation fallback on `cdc3e66` adds a synthetic
two-triangle fixture with separation `2^-500 m` and oblique flow. The
benchmark checks that only the blocker remains visible with area 0.5 m².
End-to-end visibility preparation for this fixture measured 6.37 us
(6.29 to 6.47 us). The run included the active workspace test suite and
does not isolate determinant evaluation from geometry preparation.

The area fallback revision `aec8f8d` measured medians of 21.35 us and
1.308 ms for the small and moderate partial-visibility fixtures. The wide
projected-edge fixture measured 6.24 us (6.21 to 6.29 us). The benchmark
does not isolate the exact area fallback. Its subnormal-area behavior is
covered by the focused regression.

Projection from original coordinates on `d5624ae` measured medians of
21.90 us and 1.346 ms on the small and moderate fixtures. The wide
projected-edge fixture measured 6.06 us (6.05 to 6.20 us).

The erased-endpoint check on `b449468` measured medians of 22.18 us
(21.69 to 24.09 us) and 1.381 ms (1.376 to 1.383 ms) on the small and
moderate fixtures. The wide projected-edge fixture measured 6.09 us
(6.01 to 6.31 us).

The zero-determinant recheck on `e3cb8b5` measured medians of
21.98 us (21.68 to 23.29 us) and 1.380 ms (1.375 to 1.384 ms)
on the small and moderate fixtures. The wide projected-edge fixture
measured 5.99 us (5.95 to 6.08 us).
