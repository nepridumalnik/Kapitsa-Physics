# CCD Final Optimization Report

> Historical architecture and benchmark snapshot. Its API examples describe
> the runtime before the unified public facade. See [`docs/runtime.md`](../runtime.md)
> for the current integration API.

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

The process-wide core pool described in the original Stage 9 report was
replaced by the unified runtime. `kapitsa-core` now owns the generic
`StepExecutor` contract and all collision algorithms, but no scheduler
dependency. `PhysicsWorld::step()` and `runtime::sync::Simulation` execute
sequentially. The optional `runtime::asynchronous::Simulation` owns a fixed
Rayon pool; its `spawn_handler` starts workers with the host Tokio
`Handle::spawn_blocking()`. Active task counts adapt without resizing that
pool. `PhysicsService` adds clock-driven stepping, catch-up, commands,
snapshots, and collision events around the manual-step API.

Pool workers are joined during explicit async shutdown before the world is
returned. A host-owned executor can be shared across simulations. `CcdMotion`
stores a copyable `CcdTrajectory` instead of a `dyn Fn`, removing the last
trait-object callback from the CCD sampling path. Executor calls use static
dispatch.

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

## 9. Production source size before runtime consolidation

Compared with `038e4d4`, production Rust files under `kapitsa-core/src` changed
by **+1,982 / −404 lines**, a net increase of **1,578 lines**. Test files and
benchmarks are excluded. This is the pre-consolidation count; `kapitsa-tokio`
was later removed from the workspace and its service API moved under
`kapitsa::runtime::asynchronous`. The largest CCD change remains in `world.rs`,
which coordinates staged execution and localized replay.

## 10. CCD follow-up validation and commits

Validation passed:

- `cargo fmt --all -- --check`
- `cargo check --workspace`
- `cargo check -p kapitsa --no-default-features`
- `cargo check -p kapitsa --features async`
- `cargo clippy --workspace --all-targets -- -D warnings`
- `cargo test --workspace --release`
- `cargo test -p kapitsa --no-default-features`
- `cargo test -p kapitsa --features async` (64 unit and 10 integration tests)
- `cargo test -p kapitsa-core --features bench-internals --release` (482 unit,
  16 integration/query/geometry tests, and doctests)
- `cargo doc --workspace --no-deps`
- `cargo bench -p kapitsa-core --features bench-internals --bench collisions`
- `cargo bench -p kapitsa-core --features bench-internals --bench collision_storage`
- `cargo bench -p kapitsa-core --features bench-internals --bench collision_interactions`
- `cargo bench -p kapitsa --features async --bench collisions`
- `cargo bench -p kapitsa --features async --bench parallel_step`

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

CCD follow-up Stage 10 is committed as `bench(ccd): document final optimization
results`. No push was performed.

## 11. Unified runtime architecture and validation

The runtime migration keeps the workspace's three-layer dependency direction:

- `kapitsa-math` contains math types and operations.
- `kapitsa-core` contains `PhysicsWorld`, collision algorithms, and the generic
  `StepExecutor` contract. It has no Tokio or Rayon dependency.
- `kapitsa` provides `runtime::sync` by default and
  `runtime::asynchronous` behind the optional `async` feature.

Manual stepping has matching one-step APIs:

```rust,ignore
use kapitsa::runtime::{SimulationConfig, sync::Simulation};

let mut simulation = Simulation::new(world, SimulationConfig::default())?;
simulation.step()?;
```

```rust,ignore
use kapitsa::runtime::{SimulationConfig, asynchronous::Simulation};

let mut simulation = Simulation::new(world, SimulationConfig::default())?;
simulation.step().await?;
let world = simulation.shutdown().await?;
```

The async implementation creates a fixed Rayon pool using the host's Tokio
`Handle` and `ThreadPoolBuilder::spawn_handler()`. Each Rayon worker runs inside
`Handle::spawn_blocking()`; no Tokio runtime is constructed by Kapitsa. Pool
creation happens once per owning simulation. Active worker counts change task
batch sizes without resizing the pool. `shutdown().await` signals and joins all
workers before returning the world. `new_with_executor` permits applications
to share a separately owned pool.

`PhysicsService` preserves the migrated `kapitsa-tokio` behavior: start,
stop, restart, fixed-rate stepping, catch-up, bounded commands, snapshots,
client queries, collision event streams, metrics, and simulation-time debt.
It remains an optional clock-driven API beside manual `Simulation::step()`.

### Runtime performance

The pre-runtime CCD commit and current tree were run in Release on the same
machine, each with one warm-up and five samples. Min / median / max are ms.
Sample ranges overlap for each workload, so these measurements do not establish
a runtime-consolidation regression or gain:

| Workload | Pre-runtime step | Current step | Pre-runtime CCD | Current CCD |
|---|---:|---:|---:|---:|
| 20k static + 64 CCD bodies | 23.376 / 25.727 / 25.997 | 23.888 / 24.355 / 24.990 | 0.167 / 0.185 / 0.279 | 0.169 / 0.176 / 0.222 |
| 20k static + 128 CCD bodies | 23.191 / 24.328 / 25.514 | 23.886 / 25.046 / 25.418 | 0.333 / 0.391 / 0.569 | 0.396 / 0.451 / 0.470 |
| 20k static + 256 CCD bodies | 25.535 / 26.175 / 26.788 | 25.017 / 26.748 / 27.339 | 0.678 / 0.743 / 0.916 | 0.653 / 0.695 / 0.907 |

Resweep visits and proxy updates remained 16 / 32 / 64 for the three static
scenes. The async runtime's single-step GJK benchmark measured medians of
4.029 ms (`PhysicsWorld::step`) versus 2.036 ms (x4) for 256 bodies, and
16.567 ms versus 9.498 ms for 1,024 bodies. For 1,024 bodies, x1 ranged from
16.143–17.246 ms and x4 from 9.248–9.629 ms. In the 4,096-body,
2,048-contact-island solver benchmark, x1 / x2 / x4 medians were 30.58 /
22.78 / 16.02 ms (1.91x at x4). Small, mixed, and sleeping-heavy scenes can
lose time to scheduling, and large static scenes still exceed a 16.67 ms full
step budget because indexing and broadphase dominate.

### Runtime migration commits

| Stage | Commit | Subject |
|---:|---|---|
| 1 | `42e286b` | `test(runtime): verify rayon workers on tokio blocking pool` |
| 2 | `29bff89` | `refactor(runtime): establish unified crate and feature layout` |
| 3 | `e9d3485` | `refactor(runtime): unify simulation configuration and metrics` |
| 4 | `b333965` | `refactor(core): abstract step execution with static dispatch` |
| 5 | `b76a127` | `feat(runtime): add built-in synchronous simulation` |
| 6 | `a6004a1` | `feat(runtime): execute rayon workers through tokio` |
| 7 | `904011c` | `feat(runtime): add asynchronous simulation` |
| 8 | `fe9884e` | `refactor(runtime): adapt active parallelism from step metrics` |
| 9 | `6a6188f` | `refactor(runtime): migrate tokio services into optional runtime` |
| 10 | local commit | `bench(runtime): validate unified simulation execution` |

The complete runtime API and benchmark matrix are documented in
[`docs/runtime.md`](../runtime.md) and
[`benchmarks/collisions.md`](../../benchmarks/collisions.md). No push was
performed.
