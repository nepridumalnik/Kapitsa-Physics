# Joint solver stability and rocket flight

## Environment and reproduction

- Windows 11, `x86_64-pc-windows-msvc`
- AMD Ryzen 9 7900X3D, 12 cores / 24 logical processors
- Rust `1.98.0`, optimized Release profile

The joint harness uses three warm-up steps and ten measured samples for steady
scenes, with worker counts 1, 2, 4, and 8. It reports wall and process CPU
time, solver phase timing, counted allocations, retained scratch, island and
contact counts, and maximum anchor/frame error. The Windows process CPU clock
is quantized at roughly 15.6 ms, so its values are coarse for short scenes.
Mass-break results measure the first step on fresh worlds; separation reports
joint removal and the subsequent detached step separately.

```powershell
cargo bench -p kapitsa-core --features bench-internals --bench joint_stress
cargo test -p kapitsa --lib modular_rocket_remains_bounded_and_reproducible_through_adaptive_warp -- --test-threads=1
cargo test -p kapitsa --release --lib --features bench-internals benchmark_rocket_flight_headless -- --ignored --nocapture --test-threads=1
cargo test -p kapitsa --release --lib --features bench-internals benchmark_rocket_flight_runtime_mission -- --ignored --nocapture --test-threads=1
```

## Solver findings

The original branching failure came from preparing every joint row from one
shared body pose, then applying those rows after other constraints had already
changed bodies in the same island. Connector transforms, effective masses,
and Jacobians therefore became stale during Gauss-Seidel solving. The fixed
joint also had a Baumgarte velocity bias on top of position projection; in a
large fixed tree that correction added energy. Kapitsa now refreshes and solves
each joint immediately in ordered passes, and fixed joints rely on position
projection plus the velocity lock.

Constraint warm starts are matched by `JointId` and a compact semantic row key.
This prevents a limit or drive row from inheriting a multiplier from a
different degree of freedom. Multipliers reset at physical step boundaries and
persist only through the iterations where the row identity remains valid.

The Chaos comparison followed the update/apply pattern in its
[`PBDJointContainerSolver`](https://github.com/nepridumalnik/UnrealEngine/blob/260bb2e1c5610b31c63a36206eedd289409c5f11/Engine/Source/Runtime/Experimental/Chaos/Private/Chaos/Joint/PBDJointContainerSolver.cpp)
and the sequential Gauss-Seidel solvers
([`PBDJointSolverGaussSeidel`](https://github.com/nepridumalnik/UnrealEngine/blob/260bb2e1c5610b31c63a36206eedd289409c5f11/Engine/Source/Runtime/Experimental/Chaos/Private/Chaos/Joint/PBDJointSolverGaussSeidel.cpp),
[`PBDJointCachedSolverGaussSeidel`](https://github.com/nepridumalnik/UnrealEngine/blob/260bb2e1c5610b31c63a36206eedd289409c5f11/Engine/Source/Runtime/Experimental/Chaos/Private/Chaos/Joint/PBDJointCachedSolverGaussSeidel.cpp)).
Kapitsa keeps its joint/contact island and its tested phase ordering; it does not
copy Chaos implementation code. The inspected Chaos version has optional
projection and late-iteration shock propagation, including a documented
reverse-order limitation. The Kapitsa mass-ratio and branching fixtures pass
without either mechanism, so no extra public setting or solver pass was added.
This is an implementation comparison, not a claim of equivalent performance
or general stability.

## Release stress results

Times below are median wall milliseconds per step. The fixed-chain anchors now
match their configured one-meter spacing. The branching assembly begins with a
0.01 m anchor offset and 0.001 rad frame offset; the solver runs thirteen
successive steps. The harness fails if the body states or the configured joint
error bounds are invalid.

| Scene, 14,400 bodies | Workers | Median ms | Retained scratch | Final anchor / frame error |
|---|---:|---:|---:|---:|
| Fixed-joint chain, 14,399 joints, 1 island | 1 | 302.608 | 51.06 MiB | 0 m / 0 rad |
| Fixed-joint chain, 14,399 joints, 1 island | 8 | 303.844 | 51.06 MiB | 0 m / 0 rad |
| Branching fixed-joint assembly, 14,399 joints, 1 island | 1 | 385.536 | 51.06 MiB | 0.000456 m / 0.0000456 rad |
| Branching fixed-joint assembly, 14,399 joints, 1 island | 8 | 383.445 | 51.06 MiB | 0.000456 m / 0.0000456 rad |
| Independent pairs, 7,200 joints / islands | 1 | 189.707 | 62.41 MiB | 3.64e-12 m / 5.42e-20 rad |
| Independent pairs, 7,200 joints / islands | 4 | 73.162 | 62.41 MiB | 3.64e-12 m / 5.42e-20 rad |
| Independent pairs, 7,200 joints / islands | 8 | 42.087 | 62.41 MiB | 3.64e-12 m / 5.42e-20 rad |
| Dense vertical contact stack, 14,400 contacts, 1 island | 1 | 261.743 | 68.46 MiB | n/a |

For the 14,400-body chain at one worker, wall min/median/p95/max was
302.129/302.608/304.325/304.325 ms and process CPU min/median/p95/max was
296.875/296.875/312.500/312.500 ms. Independent pairs at eight workers took
39.883/42.087/42.915/42.915 ms wall; process CPU reported
203.125/265.625/281.250/281.250 ms. CPU samples can exceed wall time because
they sum work across threads, and their clock quantization limits precision.

The 100 two-body rocket fleet (100 joints and islands) measured 0.699 ms at
four workers and 0.658 ms at eight. The 14,400-element first-step break case
reported 7,200 break events, 173.502 ms at one worker and 67.773 ms at four.
It used 79,274 allocations / 81,912,856 requested bytes and retained 65.30
MiB of scratch. Removing 7,200 joints took 0.408 ms at one worker; its
detached step took 3.832 ms. The eight-worker separation medians were 0.655 ms
and 4.451 ms, respectively.

### Comparison with the pre-optimization run

The comparison checkout was `bed9e9e`; both runs used the same revised harness
and 10-sample settings. On the aligned chain, one-worker median time changed
from 290.620 ms to 302.608 ms (+4.1%); the eight-worker result changed from
290.786 ms to 303.844 ms (+4.5%). The sequential refresh has a measurable cost
on one large connected island. Independent pairs improved from 202.788 ms to
189.707 ms at one worker (-6.5%), 76.032 ms to 73.162 ms at four workers
(-3.8%), and 43.106 ms to 42.087 ms at eight workers (-2.4%).

The pre-optimization branching run is not a valid timing baseline: after the
same repeated workload its 14,400-body error grew to `2.714e148 m` and `0.425
rad`. The corrected run ends at `4.56e-4 m` and `4.56e-5 rad`, within the
benchmark bounds of 0.002 m and 0.0002 rad. Its timing is reported as a
correctness result, not as a speed comparison.

The hot-path changes remove per-row joint-ID storage, avoid rebuilding owner
metadata during sequential updates, and copy equal-sized row sets in place.
The first-step break case uses about 3% less retained scratch than the
pre-fast-path implementation on the corrected branch. Against `bed9e9e`,
first-step break scratch is higher (65.30 vs 55.19 MiB). These figures are
workspace capacity, not process RSS. No performance parity with Chaos is
claimed.

## Modular rocket regression

`modular_rocket_remains_bounded_and_reproducible_through_adaptive_warp` builds
120 dynamic parts as two 60-part binary fixed-joint trees. Part masses vary;
both stage roots change fuel mass, engines apply force away from the root
center of mass, and the inter-stage joint is removed at a fixed simulation
time. A plane collider exercises pad contact in the same connected solver
island. The fixture compares a 5 ms fixed reference with adaptive 1x, 10x, and
100x target steps over 0.5 simulated seconds.

Each warp run requires finite states, a remaining two-tree joint count,
completed fuel updates and stage separation, surface contact, anchor error
below 0.03 m, frame error below 0.1 rad, per-part position error below 0.5 m,
and velocity error below 2.5 m/s against the reference. The 100x run is repeated
with identical inputs and all 120 part states must match exactly. These are
regression tolerances for this short test, not a formal adaptive-integrator
error guarantee.

This fixture is evidence for a 120-part assembly over this event sequence. It
does not establish stability for 500-part vehicles, dense colliders on every
part, long missions, or arbitrary joint graphs. The scene uses root colliders
for the pad-contact path and deliberately omits detailed aerodynamic forces.

## Existing rocket mission and visual demo

The CLI and OpenGL viewer continue to use the shared two-stage mission fixture,
including fuel-mass updates, stage separation, engine cutoff, and stage
impacts. The headless Release mission previously completed 14.108 simulated
seconds in 17.381 ms median across five samples (1,693 steps, about 812x
simulated time per wall second). Maximum altitude was 150.465 m, maximum speed
54.084 m/s, and measured energy drift was 0.00000192%. Both stages impacted
and two impact events were observed.

The existing GIF remains at [`target/rocket-flight-full.gif`](../target/rocket-flight-full.gif)
as an ignored local artifact; no rendering dependency was added to core crates.
