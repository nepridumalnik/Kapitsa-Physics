# Collision refactoring report

## Scope and method

This report covers the work from the pinned starting source (`27b9f12`, with
the benchmark-only baseline recorded in `336582d`) through stage 17. The
comparison uses Release builds, the same machine and scene setup, five samples,
and setup outside the timed region unless a row says otherwise. The machine is
an AMD Ryzen 9 7900X3D running Windows x86_64 MSVC with rustc 1.98.0. Results
are local measurements, not performance targets. The benchmark tables and
instrumentation limits are maintained in [`benchmarks/collisions.md`](../../benchmarks/collisions.md).

## A. Correctness

### Reproduced and fixed

- **CCD candidate invalidation after impacts.** The original candidate list
  was built before impacts and could not include a later collision on a
  changed trajectory. CCD now rebuilds conservative candidates over the
  remaining interval after each impact. The chained-impact case verifies both
  contacts, final velocity, sync/Tokio agreement, and rollback when the impact
  limit is exceeded.
- **CCD trajectory mismatch.** Endpoint SLERP could select the shortest
  quaternion path and miss multi-turn motion. CCD now samples from the active
  segment start through the rigid-body integrator, and rotational bounds use
  the path rather than only endpoint orientation. Tests cover quarter-turn,
  three-quarter-turn, four-pi, pure rotation, and intermediate samples.
- **Skinned collider bounds.** Broadphase and CCD bounds now use the current
  deformed vertices. Skin-pose changes are defined as discrete at the step
  boundary; the engine does not claim a continuous bone trajectory it cannot
  know. Tests exercise displaced geometry and Static/Kinematic colliders.
- **Finite triangle versus convex contact.** The single support-point heuristic
  was replaced with a finite triangle-convex contact path. Degenerate convex
  inputs and reversed geometry are covered without weakening numeric
  tolerances.
- **Persistent friction impulses.** Tangent impulses are cached in world space
  and reprojected to the current contact basis, avoiding frame-to-frame basis
  drift.
- **Collision event lifecycle.** Event identity and transition behavior are
  staged with the step and committed atomically. Tests cover stable contacts,
  enter/stay/exit behavior, and rollback.
- **Query and skin updates.** Committed query proxies and skinned BVH refits
  publish only after successful preparation. Failure paths retain the last
  committed geometry.
- **Collider material insertion.** Invalid material IDs are rejected before a
  collider is inserted. Direct mutable collider access remains checked during
  step validation.
- **Step completion/cancellation.** Sequential, prepare-completed, and commit
  paths now share a finalizer. Deterministic phase-gate tests cancel and resume
  after Prepare and Integrate, checking that the in-flight World remains
  unavailable until resumed.

The workspace tests also cover momentum conservation, restitution and friction
limits, contact-normal orientation, finite outputs, shape local transforms,
and 1/2/4-worker execution. No numerical tolerance was increased to make a
test pass.

### Audit notes that were not defects

- Automatic sleeping is not part of the current contract: bodies sleep only
  after an explicit `sleep()` call. The docs now say so; no auto-sleep system
  was added.
- The transactional double-buffer in skinned refit is needed to keep committed
  geometry valid if preparation fails. Replacing it with in-place refit would
  weaken rollback.
- The adaptive Dense/Sparse `PhysicsSnapshot` index was not redundant: its
  dense, sparse, oscillating, and resize comparisons remained favorable to
  the adaptive path.

## B. Architecture

- CCD motion is sampled from segment state through the existing integrator;
  candidate generation is refreshed for the remaining step after impacts.
- Skinned geometry bounds, query bounds, and refit staging now follow the
  deformed mesh. The mesh BVH topology is moved into the skin pose and reused.
- Committed ray, sweep, and overlap queries use a separate AABB tree with fat
  proxies. Dirty mutable access falls back to a full scan until a successful
  step synchronizes the committed index. Exact narrow queries still decide
  hits; infinite planes remain a separate path.
- Query scratch and skin-refit arrays are retained between calls. Pair-based
  interaction lookup replaces repeated scans in the contact cache.
- Contact manifolds use four inline contact slots and a counted slice.
  Existing world-space contact points are reused during merge/reversal.
- Solver islands remain value-materialized and deterministic. Dynamic bodies
  are copied into islands and written back through staged state; shared
  read-only bodies may appear in multiple islands. The packed-range candidate
  was measured and rejected (see Performance).
- `StepOutcome` and a shared finalizer remove repeated completion logic without
  changing the prepare/integrate/commit ownership boundary. Error and
  cancellation paths do not publish partially staged bodies, cache, events,
  or snapshots.
- Removed unused skinned accessors, dead helpers, broad dead-code allowances,
  and obsolete snapshot storage metrics. The adaptive snapshot layout and
  transactional skin cache remain.

## C. Performance

The table reports comparable baseline and final medians from the same Release
benchmark harness. The sample ranges are included where available. Changes
within the observed spread are not called improvements.

| Workload | Baseline | Final | Change and interpretation |
|---|---:|---:|---|
| 20,000 separated Dynamic bodies, full step | 25.757 ms (25.366–25.965) | 26.620 ms (26.132–26.869) | +3.4%; ranges do not overlap, small regression |
| 1,000 Dynamic bodies, one 999-contact island | 17.148 ms | 17.336 ms | +1.1%; solver phase 12.201 → 12.138 ms |
| 500 two-body islands | 7.784 ms | 7.994 ms | +2.7%; ranges overlap, no measurable change |
| 64 fast CCD impacts | 250.860 ms | 126.093 ms | −49.7%; final CCD phase 125.888 ms, 155 candidates vs. 64 baseline |
| 1,000 raycasts vs. 20,000 static colliders | 12,258.859 ms | 0.787 ms | ~15,580× lower median; query index retains an estimated 8,864 KiB |
| 1,000 sweeps vs. a 20,000-triangle mesh | 240.198 ms | 245.160 ms | +2.1%; ranges do not overlap |
| 1,000 SkinPose updates, 1,024 vertices | 36.693 ms | 36.952 ms | +0.7%; ranges overlap, no measurable change |
| 1,000 Dynamic / 4,000 Static contacts | 38.643 ms | 35.360 ms | −8.5% |

### Phase, copies, and retained memory

For 4,096 Dynamic bodies and 2,048 two-body contact islands, the final
`collision_storage` benchmark measured:

| Workers | Step min / median / max | Island prepare / solver / write-back medians |
|---:|---:|---:|
| 1 | 34.524 / 34.898 / 35.213 ms | 0.385 / 23.544 / 0.578 ms |
| 2 | 22.765 / 23.067 / 28.339 ms | 0.363 / 11.956 / 0.505 ms |
| 4 | 16.713 / 17.196 / 17.910 ms | 0.391 / 6.144 / 0.499 ms |

The final parallel Tokio benchmark measured the same scene at one, two, and
four active workers: 30.81, 10.72, and 9.53 ms per step. The baseline medians
were 30.62, 11.04, and 9.48 ms. These medians show no material before/after
change in the sharded path; they do show the expected within-build benefit of
additional workers for this collision-heavy scene. Other Tokio localized
collision cases varied by body count and are detailed in the benchmark log.

For the 1,000-body single-island sample, baseline and final each copied 1,000
bodies and 2,997 manifolds. Estimated peak sampled and retained scratch was
4,034 KiB → 4,024 KiB; logical capacity growth was 4,131,456 B → 4,121,568 B.
`ContactManifold` size fell from 648 B to 616 B (4.9%). Capacity growth and
retained memory are estimates, not allocator-call counts or process RSS. A
Windows heap trace was unavailable because the WPR Heap profile was denied
with `0x80070005`; no malloc counts are claimed.

The committed spatial index adds retained memory: its estimate is 8,864 KiB
for 20,000 sphere colliders. This is a meaningful cost of the raycast result.
The no-contact full-step regression and mesh-sweep regression are also
reported rather than attributed to noise. Full phase tables and scenario
notes are in [`benchmarks/collisions.md`](../../benchmarks/collisions.md).

### Candidate optimizations not adopted

- **Packed solver island layout:** both layouts produced equivalent local
  indices. Packed preparation was faster for the single large island but
  slower in four of five other cases, and sparse retained layout bytes grew
  from 25,600 B to 173,312 B. It did not establish an end-to-end step win, so
  copied `SolverIsland` storage remains.
- **Public reusable query object:** the reusable query candidate was 1.4–1.9%
  slower across tested sweep counts, with overlapping ranges at 10,000 calls.
  Internal query scratch is reused within a world query instead.
- **Replacing adaptive snapshots:** parallel_step comparisons kept the
  adaptive Dense/Sparse index ahead of the old `Vec + HashMap` alternative in
  dense workloads; sparse and oscillating cases did not justify a simpler
  replacement.

## D. Code size

Counts are nonblank physical Rust lines in `kapitsa-core/src` and
`kapitsa-tokio/src`. Comments and attributes count. Benchmark/docs files are
excluded; standalone `*_tests.rs` files and inline `#[cfg(test)]` regions are
classified as test lines rather than production.

| Lines | Baseline | Final | Change |
|---|---:|---:|---:|
| Production | 16,970 | 18,669 | +1,699 (+10.0%) |
| Rust tests | 8,759 | 10,591 | +1,832 (+20.9%) |

The production code did not shrink overall. The growth is concentrated in
correctness paths and reusable spatial/query infrastructure. By physical line
count, the two changed Rust files that became shorter were
`kapitsa-tokio/src/marshalling/snapshot.rs` (−10) and
`kapitsa-core/src/rigid_body/state.rs` (−4). Larger additions include
`collision/spatial_index.rs` (+177 net), `collision/ccd.rs` (+175),
`collision/narrow_phase.rs` (+159), and `world.rs` (+623). Tests grew most in
`world_tests.rs` (+553) and `collision/geometry_matrix_tests.rs` (+384).
This is a correctness and measured hot-path refactor, not a code-size
reduction.

## E. API

- Added `PhysicsWorld::set_collider_mode` and completed the typed Tokio client
  command surface for collision configuration, CCD mode, collider mode,
  skin pose, and material updates. Existing single command queue and
  ownership model remain.
- `PhysicsWorld::add_collider` now rejects an unknown material before
  insertion. `Collider::set_material` remains useful while preparing an
  unattached collider; world-managed changes should use the validated world
  method.
- `Ray` fields are private and read through `origin()`, `direction()`, and
  `max_distance()`. This intentionally breaks direct public-field access for
  the 0.1.0 API; constructor and query behavior remain available. Example:

  ```rust
  let ray = Ray::new(origin, direction, max_distance)?;
  let hit = world.raycast(ray, filter)?;
  let distance = ray.max_distance();
  ```

- Other existing method call patterns remain source-compatible; the new
  world/client setters are additive. No collision internals were exposed as
  public APIs.

## F. Commits

Stages 1–17, in order:

| Stage | SHA | Commit |
|---:|---|---|
| 1 | `336582dcd72a02bca960f6b55f53433a6abbd7c5` | `bench(physics): establish collision refactoring baseline` |
| 2 | `ebb6ad32d15e8ccb729302a14770d30c153359b2` | `fix(core): refresh continuous collision candidates after impacts` |
| 3 | `8c57869f7dc12b6760c29dd5302a2e4d1523ccc1` | `fix(core): align CCD trajectories with rigid body integration` |
| 4 | `46fa925a8fabaae82960bf31c579a15e42411be9` | `fix(core): use deformed geometry for skinned collision bounds` |
| 5 | `df2536a60986b7ca264403b198cde76036148a59` | `fix(core): implement robust finite triangle convex contacts` |
| 6 | `7f3b57bf9388dfa82d8417f5c3ef7de2721de407` | `test(core): verify degenerate convex collision cases` |
| 7 | `5974caf6ac50059c2efa198c71e7f5874cd3e608` | `fix(core): preserve world space tangent impulses across frames` |
| 8 | `ba2762447b5b86948f8285885c2eeaee10603b9e` | `fix(core): define and preserve collision event lifecycle` |
| 9 | `baec17eb5b9fcc4a51c87081b929facc500c89af` | `perf(core): index collision interactions by pair` |
| 10 | `95a78ebfd485d926ef9961882873da660340d448` | `perf(core): accelerate committed world spatial queries` |
| 11 | `a232409798b29bba7bf307397aeb9e96b089308c` | `perf(core): reuse collision query workspaces` |
| 12 | `3601368214603bce20691ffab38a4cca39b95130` | `perf(core): reuse transactional skinned mesh refit buffers` |
| 13 | `659440744cb205877cc02063dc6a4a930a1e15a0` | `bench(core): compare packed and copied solver island layouts` |
| 14 | `96675d61233cdaa7842795cc9fe273890320b171` | `refactor(core): simplify fixed capacity contact manifolds` |
| 15 | `67a571c7e39bc223b01473908b14e15c50273e22` | `refactor(core): unify collision index preparation and geometry routing` |
| 16 | `1ffc39fada82270b325deec9d6e78d66fff2bf4a` | `feat(api): complete validated runtime collision configuration` |
| 17 | `355c7f7308de95988cd49dd22769ce02df49d080` | `refactor(physics): remove redundant internal code and preserve lifecycle` |

Stage 18 is this report and benchmark documentation, committed as
`bench(physics): document collision refactoring performance`. Its full SHA is
included in the final handoff because a commit cannot contain its own
content-derived hash.

## G. Remaining risks and limits

- Indexed queries retain an estimated 8,864 KiB for 20,000 sphere colliders;
  query-heavy workloads benefit strongly, while memory-constrained scenes may
  value this tradeoff differently.
- The 20,000-body separated full step and 1,000 mesh-sweep sample regressed in
  the final comparison. These are measured results and should remain in future
  performance tracking.
- CCD candidate refresh performs a conservative rebuild after impacts. The
  measured fast-impact case is about 50% faster than baseline, but scenes with
  many impacts and much larger broadphases should continue to be profiled.
- No external allocator trace was available, so allocation counts are not
  claimed. Capacity growth and retained-byte figures are logical estimates.
- `Ray` direct-field access is a deliberate source break; callers should use
  its getters.
- Bone poses are discrete at physics-step boundaries; continuous motion of
  unknown skeletal animation paths is outside the current CCD contract.

## Validation

All requested release checks completed successfully:

```text
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy -p kapitsa-core --all-targets --features bench-internals -- -D warnings
cargo test --workspace --release
cargo doc --workspace --no-deps
cargo bench -p kapitsa-core --features bench-internals --bench collisions
cargo bench -p kapitsa-core --features bench-internals --bench collision_storage
cargo bench -p kapitsa-tokio --bench collisions
cargo bench -p kapitsa-tokio --bench parallel_step
```

The added Tokio collision benchmark covers 4,096 dynamic bodies and 2,048
contact islands at 1/2/4 workers.
