# CCD refactor review: Chaos-inspired islands and bounded processing

## 1. Scope and starting point

This work replaces the scene-wide repeated CCD candidate scan with a CCD
island pipeline inspired by the ideas in Unreal Chaos Physics. No Unreal source
was copied. The starting source revision was
`8c359eaf96a994c1b5a0c97fdd05fef7663d11e5` on `feat/collisions`.

The measured failure case is 64 continuous spheres moving at 1,000 m/s into a
static cuboid wall during a 1/60-second step. The original CCD loop rebuilt
broadphase candidates after each resolved impact, so it repeatedly evaluated
pairs unrelated to the body just changed.

## 2. Baseline and final result

The baseline was measured before changing the CCD algorithm, with one warm-up,
five independently built worlds, and `bench-internals` instrumentation. The
final run used the same Release profile, machine, scene, and sample method.

| Metric | Stage 1 baseline | Final | Change |
|---|---:|---:|---:|
| Full-step median | 146.847 ms | 0.677 ms | 217x faster (−99.539%) |
| CCD median | 146.563 ms | 0.539 ms | 272x faster (−99.632%) |
| TOI evaluations | 5,560 | 64 | 86.9x fewer |
| Repeated TOI evaluations | 5,405 | 0 | eliminated in this scene |
| Broadphase builds | 65 | 1 | 64 fewer rebuilds |
| CCD candidate refreshes | 64 | 0 | eliminated in this scene |
| Resweeps | not separately tracked | 64 | one local resweep per impact |
| Resolved impacts | 64 | 64 | unchanged |
| Retained CCD scratch estimate | ~234 KiB | ~171 KiB | −27% |

The 0.539 ms CCD phase is also about 234x faster than the separately quoted
historical 126.093 ms figure. That older value is not the measured stage 1
source baseline, so the row above is the reproducible before/after comparison.
The phase timer definitions and detailed final samples are in
[`benchmarks/collisions.md`](../../benchmarks/collisions.md).

## 3. Architecture

`CcdIslandLayout` groups dynamic bodies using dynamic-dynamic swept candidates.
Static and kinematic bodies are leaves: sharing a wall or moving platform does
not merge otherwise independent dynamic islands. Candidate pairs from all
colliders on a body map to that body's island. Reusable vectors hold the graph,
per-island candidate ranges, TOI cache, elapsed body states, and resweep query
results.

Each island evaluates its candidates and processes the earliest absolute TOI.
After an impact, the solver invalidates TOIs involving the changed bodies,
updates only their swept proxies, queries the spatial index for their remaining
motion, and evaluates the affected pairs. Newly discovered pairs are added to
the active CCD set. The ordinary wall case therefore needs one initial
broadphase build rather than one scene rebuild per impact.

When resweep discovers a valid dynamic-dynamic pair between separate islands,
the implementation restarts the pass with a single connected CCD island. This
rare path favors ordered correctness over retaining parallel execution after
the dependency appears. Benchmark-only counters record its use.

Expensive initial GJK TOIs are prepared by deterministic island chunks using at
most the configured worker count. Results are sorted by candidate index before
publication. Impact resolution remains ordered on the owning world so it can
resweep changed paths without concurrent body writes. The core receives the
worker count through the existing Tokio controller's transactional commit.
Inside that commit it uses a bounded `std::thread::scope` batch, not one task
per island. The worker batch is only enabled for at least 256 GJK-heavy
candidates; analytic sphere pairs stay on the sequential path.

## 4. Differences from Chaos

This is a smaller Rust implementation, not a port of `FCCDManager`. It uses
Kapitsa's collision context, spatial index, body prediction, transaction
commit/rollback, and controller worker count. It has no separate persistent
CCD manager or worker pool. CCD islands are built from swept candidate pairs
for the current step; they are not the existing discrete solver islands.

The parallel phase prepares TOIs only. Chaos-style parallel island solving is
not used because a collision can change a trajectory and create a dependency
between previously separate islands. The implementation handles that case by
rescheduling sequentially as one island. Scoped worker threads are created per
qualifying preparation batch, bounded by the existing configured worker count;
they are not long-lived worker threads. This keeps the core independent of the
Tokio runtime but leaves some scheduling overhead at small scene sizes.

## 5. Bounded work and accuracy compromises

`CollisionConfig::max_ccd_impacts` defaults to eight impacts per continuous
body per step. The default `CcdLimitPolicy::ClipIsland` advances every dynamic
body in the connected island to the limiting contact, sets its linear and
angular velocity to zero, and skips its remaining motion for that step. This
prevents a small-TOI repeat loop and avoids tunneling by stopping at the
obstacle; it intentionally loses velocity and bounce energy. `AbortStep` is
available when callers prefer a `CcdLimitExceeded` error and transactional
rollback of both body states and staged events.

Pairs already processed at the same time are suppressed, while TOIs involving
changed trajectories are invalidated and recalculated. The impact budget is
the final bound on repeated near-zero events. Tests cover clipping at a
contact, clipping a newly connected dynamic pair, and rollback under
`AbortStep`.

For TOI, linear Sphere–Cuboid motion uses an analytic sweep against the rounded
box, including face, edge, corner, tangent, and initial-overlap cases. A
sampled nonlinear sphere path uses conservative advancement with exact
point-to-box distance. Rotating cuboids continue through GJK. The nonlinear
path is capped at 64 conservative-advancement iterations; arbitrary skinned
geometry still uses the existing discrete collision path because the engine
does not have a continuous bone-pose trajectory.

Adding impact-budget fields to public `CollisionConfig` means downstream
exhaustive struct literals must specify the new fields or use struct update
syntax with `..Default::default()`.

## 6. Scaling and supplemental CCD scenarios

For the analytic sphere-wall workload, the one-worker CCD medians at 16, 64,
128, and 256 bodies were 0.134, 0.549, 1.110, and 2.282 ms. A fourfold rise
from 64 to 256 bodies took 4.16x as much CCD time. This is close to
proportional for the tested layout; it is not a claim of linear complexity for
every broadphase distribution.

For GJK-heavy cuboid-wall candidates at 64, 128, 256, and 512 bodies, the
one-worker CCD medians were 0.818, 1.683, 3.351, and 6.998 ms. An eightfold
candidate increase produced 8.56x CCD time, also close to proportional in this
scene. At 512 candidates, four workers reduced median full-step time from
8.459 to 7.736 ms (8.5%); the sample ranges did not overlap. CCD time improved
10.9%. At 256, the 5.5% step median improvement had overlapping ranges. Sphere
sweeps showed no stable worker benefit.

The final benchmark also measured:

| Scenario | Step min / median / max | CCD min / median / max | Islands | TOIs / repeats | Resweeps / impacts | New pairs | Retained scratch |
|---|---:|---:|---:|---:|---:|---:|---:|
| 64 fast bodies, no impacts | 0.488 / 0.498 / 0.535 ms | 0.307 / 0.314 / 0.325 ms | 1 | 531 / 0 | 0 / 0 | 0 | 623 KiB |
| 64-body swept chain, one island | 0.133 / 0.135 / 0.155 ms | 0.055 / 0.057 / 0.058 ms | 1 | 67 / 3 | 2 / 2 | 0 | 144 KiB |
| One sphere, six wall bounces | 0.058 / 0.058 / 0.058 ms | 0.055 / 0.055 / 0.055 ms | 1 | 8 / 6 | 6 / 6 | 0 | 8 KiB |
| Rebound discovers a new pair | 0.029 / 0.029 / 0.031 ms | 0.025 / 0.025 / 0.027 ms | 1 | 4 / 2 | 3 / 3 | 1 | 8 KiB |
| 20,000 separated dynamic bodies | 26.482 / 26.641 / 27.243 ms | 0 / 0 / 0 ms | 0 | 0 / 0 | 0 / 0 | 0 | 16,420 KiB |

The 20,000-body step has no CCD work and is a general broadphase/index check.
Its previous comparable collision-suite result was 25.757 ms and the
recorded final comparison was 26.620 ms (+3.4%); the latest run is noisier and
does not isolate a CCD change. Ordinary discrete contact processing was
17.436 ms for one 999-contact island in the final suite; the existing
same-machine comparison was 17.148 to 17.336 ms (+1.1%). These small
non-CCD costs are retained as observed, not presented as CCD gains.

The current 1,000-ray / 20,000-static-collider sample measured 0.772 ms. The
existing mesh-sweep comparison measured 240.198 ms before and 245.160 ms after
for 1,000 sweeps against a 20,000-triangle mesh (+2.1%). The indexed raycast
gain is from the earlier spatial-query refactor; the mesh sweep increase is a
small observed regression and was not changed as part of CCD work.

## 7. Regression coverage

The release workspace suite covers island composition and stable ordering,
multi-collider bodies, shared static and kinematic surfaces, same-time
contacts, sequential impacts, resweep-discovered contacts, inter-island
dynamic-pair rescheduling, worker-count determinism, clipping, and transactional
rollback. TOI tests cover face/edge/corner and tangential Sphere–Cuboid contact,
near misses, rotated boxes, translating pairs, initial overlap, offset center
of mass, and multiple rotation turns. Existing collision coverage includes
TriangleMesh, HeightField, LevelSet, skinned geometry, sensors, collision
event lifecycle, and finite numeric results.

The final benchmark scenes also exercise a one-island 64-body candidate chain,
531 fast no-contact candidates, six sequential impacts, and a rebound that
discovers a new pair. The dynamic inter-island fallback is asserted by a unit
test; the timing table's rebound scene discovers a new pair against a static
target and therefore does not trigger that fallback.

## 8. Production code size

From the stage 1 commit to the completed CCD implementation, the textual diff
adds 2,715 and removes 491 lines in Rust production source, for a net increase
of 2,224 lines across `kapitsa-core` and `kapitsa-tokio`. This excludes the
standalone test files. Most added code is the reusable CCD TOI and island
machinery, local resweep and transactional integration, and controller worker
count plumbing. The detailed file list is available from the stage commits.

## 9. Validation

The following required checks passed on the final implementation. Cargo builds
were run serially (`CARGO_BUILD_JOBS=1`) to avoid MSVC PDB contention observed
in parallel builds.

| Check | Result |
|---|---|
| `cargo fmt --all -- --check` | PASS |
| `cargo clippy --workspace --all-targets -- -D warnings` | PASS |
| `cargo clippy -p kapitsa-core --all-targets --features bench-internals -- -D warnings` | PASS |
| `cargo test --workspace --release` | PASS; 468 core unit tests, 90 Tokio unit tests, integration tests, and doctests |
| `cargo doc --workspace --no-deps` | PASS |
| `cargo bench -p kapitsa-core --features bench-internals --bench collisions` | PASS; final scenes and worker scaling recorded above |
| `cargo bench -p kapitsa-core --features bench-internals --bench collision_storage` | PASS |
| `cargo bench -p kapitsa-tokio --bench collisions` | PASS |
| `cargo bench -p kapitsa-tokio --bench parallel_step` | PASS |

## 10. Commits

Each implementation stage was committed locally. No push was performed.

| Stage | Commit | Subject |
|---:|---|---|
| 1 | `9ec8a28` | `bench(ccd): establish detailed CCD baseline` |
| 2 | `be18bdc` | `refactor(ccd): introduce independent CCD islands` |
| 3 | `5a50f01` | `refactor(ccd): process collisions per island` |
| 4 | `4abfe14` | `perf(ccd): implement selective resweep` |
| 5 | `9c702b7` | `perf(ccd): parallelize independent islands` |
| 6 | `ea5a88f` | `feat(ccd): add bounded collision processing` |
| 7 | `3ded599` | `perf(ccd): optimize measured TOI hotspots` |
| 7 follow-up | `f8f4883` | `fix(ccd): sweep sampled sphere paths against fixed boxes` |
| 8 | `d994de6` | `test(ccd): cover island scheduling and bounded CCD` |
| 9 | this report commit | `bench(ccd): document CCD refactoring results` |
