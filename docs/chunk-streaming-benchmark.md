# Synthetic rocket flight and chunk streaming benchmark

This tool measures Kapitsa's prepared mesh/chunk APIs. It contains no planetary
LOD, quadtree or Delta-V scheduler. Meshes tile a flat local tangent surface;
the rocket is a sphere of radius 0.5 m. Geometry and nominal routes are fixed.
Async job readiness deliberately depends on real scheduling and CPU load.

The benchmark is never part of ordinary `cargo test --workspace --all-features`.
Compile without running workloads:

```powershell
cargo check -p kapitsa --bench chunk_streaming --features bench-internals
cargo test -p kapitsa --test chunk_streaming_support --features bench-internals
```

Run on the measurement machine (release build):

```powershell
cargo bench -p kapitsa --bench chunk_streaming --features bench-internals -- --output target/chunk-streaming-report.md
```

After writing the Markdown report, the tool prints its absolute canonical path.
The default matrix uses 1/2/4/8 preparation workers crossed independently with
1/2/4/8 physics workers, 4/64/1024 active chunks, cold/warm caches, 60 seconds at
20 ms dt and three independent repetitions. It is a substantial measurement run.
The existing production `RayonExecutor` executes physical steps; a separate
Rayon pool prepares geometry. Simultaneous pools can oversubscribe the machine;
both budgets and logical CPU count are included in the report.

A focused run and a separate extreme-load matrix:

```powershell
cargo bench -p kapitsa --bench chunk_streaming --features bench-internals -- --scenarios shuttle,boundary --prepare-workers 1,2,4,8 --physics-workers 1 --active-chunks 4,64 --seconds 10 --repeats 3 --output target/chunk-focus.md
cargo bench -p kapitsa --bench chunk_streaming --features bench-internals -- --active-chunks 4,64,1024,16384 --meshes-per-chunk 4 --grid 8 --mode throughput,realtime --output target/chunk-extreme.md
```

Extreme settings can require substantial memory, particularly with warm route
preparation. The benchmark does not silently reduce requested workloads.

## Scenarios

| Name | Workload |
|---|---|
| overflight | Straight surface traversal, 200 m/s, prescribed kinematic probe |
| shuttle | Repeated 256 m route, 8 second period, kinematic probe |
| boundary | Oscillation around an X chunk boundary while moving along Z |
| landing | Real dynamic sphere, initial velocity (3,-1,0) m/s and gravity |
| ccd | Real continuous dynamic sphere, initial velocity (500,-200,0) m/s and gravity |
| bulk | Equal total geometry through individual collider, individual chunk and batch APIs |

Landing/CCD positions evolve through the actual solver, with no pose correction
or replacement by prescribed trajectories. Gravity is a per-step physical force.
Other scenarios are deterministic streaming load traces, not flight dynamics.

Each flight frame requests a fixed-size working set around the probe and one
step of lookahead. Cold evicts old data; warm prewarms the nominal route and
keeps removed prepared chunks. Initial coverage is prepared in both modes so
landing does not begin with absent ground. In-flight jobs are limited by
`--in-flight`; completed geometry publishes at frame boundaries. Jobs left at
the end are drained before the next run, and their preparation times/builds are
included. The timed frame interval excludes this final drain.

Throughput has no artificial delay. Real-time sleeps to cumulative fixed-step
deadlines, counting overruns rather than dropping simulated steps. Preparation,
publication, query correctness checks and physical stepping are separate samples.
Frame-work timing includes the workload generator and checks; it is not presented
as engine-only time. Pool setup and prewarm are outside frame measurements.

Bulk uses batches 1/4/16/64/256/1024 by default. `--active-chunks` is the *same
total workload* for each API; a final short batch is allowed. Single APIs are
measured once, batch APIs for each batch size. Add/remove operation distributions
and equal-work totals are both reported. Bulk has no physical stepping or
real-time pacing and does not repeat the irrelevant physics-worker axis.

## Options and evidence

`--help` lists all options. Lists are comma-separated. `--grid N` gives N×N
cells, two triangles per cell **per mesh**. Meshes subdivide a chunk into
non-overlapping X strips. `--meshes-per-chunk` varies collider count independently
of chunk count. `--flight-batch` controls publication/unload batch size;
`--batch-sizes` applies to bulk. `--bulk-iterations` defaults to 20. Repetitions
are independent worlds, with per-run percentile tables instead of pooled results.

The report includes rustc, commit, system/configuration, sample counts,
nearest-rank p50/p95/p99, successful BVH builds and verified preparation identities
through registration/removal/reuse. Fewer than 100 samples make p99 coarse;
inspect sample counts. Cold startup and warm prewarm are explicitly labelled.

Coverage misses mean a downward ray excluding the rocket finds no active terrain.
Penetration is the physical sphere bottom below the known flat surface. Contact
errors count penetrations greater than 0.01 m when the pre-step ray had coverage;
this is a diagnostic threshold, not a changed solver tolerance. Stale-ID checks
verify removed IDs no longer resolve. Failures are preserved in the report.

Memory is **tracked completed mesh/BVH retained buffer capacity**, current at
the end of measurement and peak at sample points. It excludes process RSS,
world/contact/query storage, in-flight allocations and allocator overhead.
Process peak memory and allocations/frame are reported as unavailable rather
than estimated. Successful timing completion is not a correctness PASS: inspect
coverage, contact, stale-ID and reuse-error counters before interpreting speed.
Fatal workload errors, stale IDs, reuse mismatches or contact errors produce a
nonzero process exit code after the report has been saved. Coverage misses remain
reported load/readiness observations and do not alone change the exit code.
