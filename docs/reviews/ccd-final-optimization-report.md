# CCD Final Optimization Report

- **Branch:** `feat/collisions`
- **Starting commit:** `038e4d4a6c5147a0053b855758f26ea14e250fff`
- **Benchmark machine:** AMD Ryzen 9 7900X3D, Windows MSVC, `rustc 1.98.0`
- **Benchmark run:** 2026-11-01, Release, one warm-up, five samples, `dt = 1/60 s`

## 1. Initial bottlenecks

- CCD resweep searched the scene-wide collider set to find colliders attached
  to bodies changed by an impact. In the 20,000-static workload, this produced
  642,048 / 1,288,192 / 2,592,768 proxy visits for 16 / 32 / 64 impacts.
- Impact-budget clipping stopped every dynamic body in an island even when one
  unresolved contact only required stopping its participants.
- A new dynamic pair between completed CCD islands triggered a full-scene
  replay, invalidating independent staged results.
- Parallel phases shared mutable step output indirectly, and scoped OS threads
  were created for repeated parallel phases.

## 2. Algorithm changes

- Resweep now walks `BodyEntry::first_collider` and each collider's `next`
  link. It validates live IDs and ownership, updates only those proxies, and
  uses the spatial index to discover new pairs. Unbounded colliders use the
  maintained unbounded-collider list. No scene-wide collider scan is needed to
  identify a changed body's colliders.
- CCD island output uses reusable private result buffers for body predictions,
  candidate TOIs, solver output, and staged interactions. The coordinator
  merges results in stable island and candidate order.
- Independent islands with one initial candidate can evaluate and solve their
  first impact in parallel. Cost-weighted batches bound task count to the
  configured worker count. Small scenes, multi-candidate islands, exhausted
  budgets, and later impacts use the ordered coordinator path.
- A newly discovered dynamic bridge invalidates and replays only the connected
  component. Unaffected completed islands retain their staged result. The
  global replay remains available for bounded non-convergence recovery.

## 3. CCD execution model

`kapitsa-core` owns one lazily initialized process-wide Rayon 1.12.0 pool,
sized from `available_parallelism`. Scoped tasks borrow the current step's
world data safely; the pool is reused across steps. Each phase launches no more
tasks than its active worker count. `PhysicsWorld::step()` remains synchronous
and core remains independent of Tokio. `PhysicsController` still runs its
synchronous world commit through Tokio's blocking task.

The scoped-thread versus reused-pool comparison showed lower medians on the
GJK-heavy cases tested, but the sample ranges overlapped. The pool is retained
to meet the persistent-worker requirement without adding a custom executor;
those measurements do not establish a general speedup attributable to Rayon.

## 4. Impact-budget clipping

`CcdLimitPolicy::ClipIsland` remains available for API compatibility. When the
budget is exhausted, only dynamic bodies in the unresolved contact are fixed
at the contact pose for the rest of the step. Their candidate state is
invalidated and reswept. Other dynamic bodies continue, and moving kinematic
bodies keep their authored trajectories. `AbortStep` still returns an error
without publishing staged body state or collision events.

## 5. Local replay and transactional guarantees

When resweep finds a new dynamic pair connecting separate islands, the pair is
recorded as a temporary graph bridge. The engine combines the affected
components, restores their starting predictions, invalidates incident TOIs and
staged interactions, and replays that component in TOI order. Independent
components keep their completed results. Static and kinematic surfaces do not
merge dynamic components. The measured rebound and cascade workloads report
local replay and zero global fallback.

CCD body states and interactions remain staged until validation and collision
processing succeed. Worker errors are collected before results are committed.
The parallel AbortStep regression test first resolves 128 independent islands,
then triggers a second impact after resweep; it verifies that no body state or
event from that parallel batch was published. Tokio cancellation tests verify
that an interrupted in-flight step replays the same transaction.

## 6. Performance before and after

The baseline rows below are from `038e4d4`, before the follow-up changes. Final
values are medians from the Stage 9 Release run. All step values are ms.

| Workload | Baseline step / CCD | Final step / CCD | Resweep visits, baseline → final | Result |
|---|---:|---:|---:|---|
| 64 fast spheres vs. wall | 0.690 / 0.545 | 0.673 / 0.537 | — | Ranges overlap; no established change |
| 20k statics + 64 CCD bodies | 22.172 / 0.669 | 20.470 / 0.170 | 642,048 → 16 | CCD −74.6%, step −7.7% |
| 20k statics + 128 CCD bodies | 23.140 / 1.329 | 20.724 / 0.321 | 1,288,192 → 32 | CCD −75.8%, step −10.4% |
| 20k statics + 256 CCD bodies | 24.126 / 2.607 | 21.584 / 0.619 | 2,592,768 → 64 | CCD −76.3%, step −10.5% |
| 20k dynamic bodies, no contacts | 29.510 / 0.000 | 26.131 / 0.000 | — | Ranges overlap; still above frame budget |
| 1k dynamic bodies, one contact island | 19.356 / 0.003 | 17.248 / 0.002 | — | Lower median; still slightly above frame budget |

In the 20k-static scenes, index and broadphase construction dominate the full
step. CCD resweep itself is below 0.62 ms at 256 moving bodies, but the full
step remains above the 16.67 ms budget.

## 7. Worker scaling

The independent GJK-heavy 1,024-island workload measured:

| CCD workers | Step min / median / max | CCD min / median / max | Retained scratch |
|---:|---:|---:|---:|
| 1 | 16.937 / 17.499 / 17.521 | 13.893 / 14.308 / 14.393 | 2,746 KiB |
| 2 | 13.209 / 13.631 / 14.363 | 8.975 / 9.374 / 9.995 | 6,033 KiB |
| 4 | 10.237 / 10.459 / 11.251 | 6.117 / 6.282 / 7.014 | 6,033 KiB |

Four workers lowered the median step by 40.2% versus one worker; ranges do not
overlap. Two and four workers were below 16.67 ms, while one worker was just
above it. The retained-capacity estimate rose about 2.2x for the parallel
path. For 4,096 dynamic bodies in 2,048 contact islands, median step time was
41.723 / 30.846 / 23.765 ms at 1 / 2 / 4 workers: a 1.76x four-worker speedup,
but still above the frame budget.

## 8. Regressions, limits, and remaining risks

- Parallel impact resolution currently handles first impacts for independent
  islands with one initial candidate. Multi-candidate islands and later hits
  stay ordered on the coordinator. The benchmark and tests prove the parallel
  path for the one-candidate workload; they do not prove parallel completion of
  every possible multi-impact island.
- Large no-contact scenes and 20k-static scenes remain above 16.67 ms because
  broadphase/index work dominates. This task reduces CCD resweep cost; it does
  not solve total physics-step cost for those worlds.
- Parallel solver scratch is larger than the sequential estimate. Concurrent
  worlds share one fixed process-wide pool and may contend for its workers; the
  active task count is bounded per phase, but there is no per-world fairness
  policy.
- Tokio integration benchmarks are workload-sensitive. Dynamic-heavy scenes
  benefited from dynamic buckets at 4,096 and 20,000 bodies, while small,
  mixed, and sleeping-heavy scenes often lost time to scheduling.
- Retained-capacity values estimate selected collision-owned vectors and hash
  tables. They are not allocator call counts, peak heap use, or process RSS; no
  external heap profile was collected.
- The fixed-seed stress test covers one reproducible 192-body distribution.
  It checks worker equivalence, event order, and finite states, but does not
  replace broader fuzzing or application-level profiling.

## 9. Production source size

Compared with `038e4d4`, production Rust files under `kapitsa-core/src` changed
by **+1,982 / −404 lines**, a net increase of **1,578 lines**. Test files and
benchmarks are excluded. `kapitsa-tokio` production source did not grow for
this refactor. The largest change is in `world.rs`, which coordinates staged
CCD execution and localized replay; the pool itself is isolated in a small
executor module.

## 10. Validation and commits

Validation passed:

- `cargo fmt --all -- --check`
- `cargo clippy --workspace --all-targets -- -D warnings`
- `cargo clippy -p kapitsa-core --all-targets --features bench-internals -- -D warnings`
- `cargo test --workspace --release` (including core, Tokio, integration, query,
  simulation, marshalling, and doctests)
- `cargo test -p kapitsa-core --features bench-internals` (479 unit tests, 16
  integration/query/geometry tests, and doctests)
- Targeted Tokio tests for independent islands, connected/rescheduled islands,
  and cancellation before commit
- `cargo doc --workspace --no-deps`
- `cargo bench -p kapitsa-core --features bench-internals --bench collisions`
- `cargo bench -p kapitsa-core --features bench-internals --bench collision_storage`
- `cargo bench -p kapitsa-tokio --bench collisions`
- `cargo bench -p kapitsa-tokio --bench parallel_step`

The first nine stage commits are:

| Stage | Commit | Subject |
|---:|---|---|
| 1 | `16b55b191079247c863a155ffa069ca094d41c5a` | `bench(ccd): establish remaining bottlenecks baseline` |
| 2 | `963021089cd675ed7982212e6cdbe510296cf39a` | `perf(ccd): restrict resweep to body colliders` |
| 3 | `23e3a60f0740c142eaa67008be6a000fab7479eb` | `refactor(ccd): localize collision budget clipping` |
| 4 | `2d81247c920f1f9b51ba4128be86d2f6cdc85b2d` | `refactor(ccd): replay only affected island components` |
| 5 | `ddf7e271b6ff2911101ce4c78ba9881de3ade52f` | `refactor(ccd): isolate island execution state` |
| 6 | `dae9e5b2802cbc6781acbb2154d0cabcc2e319ae` | `perf(ccd): resolve independent islands in parallel` |
| 7 | `1a1971184f868e6f7e70e5ff7ebbb7f96ee36544` | `perf(physics): reuse collision execution workers` |
| 8 | `c8a24190d84a471b71e656bd571cf5ac374059a5` | `test(ccd): cover parallel islands and localized replay` |
| 9 | `ff32c763be17c6f0ae62f1a6c6abbe4b85e01aa3` | `bench(ccd): capture final benchmark matrix` |

Stage 10 is committed as `bench(ccd): document final optimization results`.
No push was performed.
