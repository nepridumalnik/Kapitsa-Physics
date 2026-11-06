# Momentum-form constrained island integration

Date: 2026-11-06. Branch: `feat/collisions`. Starting commit: `8dec14c`.

## Production behavior and admission

The specialized whole-world Ball-pair implementation has been removed. Eligible
`SolverIsland` instances now integrate through `joint::constrained` from original
body states. Independent ineligible islands in the same world retain the legacy
solver. The public `PhysicsWorld` API is unchanged.

Admission requires awake dynamic bodies, no colliders on any participating body,
no continuous-CCD body mode, no clusters, no accumulated external force/torque,
no central gravity, and supported enabled hard bilateral joints. Explicit
nondefault benchmark solver options retain their requested legacy strategies.

**Initial geometry must already be compatible:** every equality coordinate must
be within 1e-9 before selection. RATTLE assumes an initial constraint manifold;
converting a pre-existing legacy position-projection defect into a start impulse
would inject unphysical energy. This is an explicit preflight applicability
condition, not a fallback after numerical failure. Incompatible *velocities* are
supported; their initial constraint accommodation can dissipate energy.

| Joint | Equality coordinates | Supported settings |
|---|---:|---|
| Ball | 3 | No swing/twist limits |
| Fixed | 6 | Hard, undamped |
| Hinge | 5 | No limit or motor |
| Slider | 5 | No limit or motor |
| Cylindrical | 4 | No limits or motors |
| Planar | 3 | No limits or motors |
| Distance | 1 | Fixed positive length, noncoincident anchors |
| D6 | 0-6 | Locked/Free axes, no drives |
| ConeTwist | 3-6 | Each swing coordinate either locked at zero or fully free (pi); twist either fixed or fully free (-pi..pi) |

Spring, compliant/damped constraints, break thresholds, bounded unilateral
limits, motors, static/kinematic/sleeping participants, collider/CCD/cluster
islands and incompatible initial poses keep the existing solver. Zero-length
Distance derivatives, singular swing/twist decompositions, the rotation-log pi
branch and antiparallel hinge/plane axes produce explicit errors when admitted;
the implementation does not invent an arbitrary derivative at a singularity.

## API and architecture

The requested internal API is implemented in:

- `joint/constrained/body.rs`: `MomentumBody` stores COM, rotation, linear and
  world spin momenta, full mass properties and inverse local inertia.
- `joint/constrained/geometry.rs`: `EqualityRow`, `MAX_EQUALITY_ROWS = 6` and
  `write_equality_rows(JointConstraint, &[MomentumBody], &mut [EqualityRow; 6])`.
- `joint/constrained/solver.rs`: `ConstrainedWorkspace`, `IslandEligibility` and
  `ConstrainedIntegrator::{eligible, solve}` with the requested signatures.
- `Joint::supports_free_bilateral()` rejects unsupported joint settings before
  selecting constrained integration.

`StepContext` carries optional original solver bodies together with per-body
admission metadata. `IslandLayout` retains responsibility for connectivity.
Each `SolverIsland` owns its original-state buffer and a separate reusable
`ConstrainedWorkspace`. All ordinary/executor/full-shard/low-level-shard paths
converge on the same island dispatch. Low-level shard results are validated
before their inputs can reach the island solver.

The shared island dispatcher selects one of two operations: constrained
integration, or the existing joint/contact solve. Thus an accepted constrained
island never receives a second legacy velocity/XPBD position projection.
All island results remain staged until the world commits. Numeric failure
returns `CollisionError::NonConvergent` or `ArithmeticOverflow`; source body
states and pending forces survive rollback. Collision lifecycle events and the
transactional warm-start cache still use the shared world commit/abort path.

## Equations and invariant checks

The start reaction is applied through the initial analytic Jacobian transpose.
The centers then drift with constant linear momenta and rotation uses implicit
midpoint iteration carrying constant world spin momenta. An outer quasi-Newton
iteration solves the endpoint geometry by changing only the start reaction and
repeating drift from the saved original state. A separate endpoint reaction
solves velocity compatibility. There is no post-hoc momentum restoration or
energy normalization.

For each scalar equality row, tests check both rigid-motion identities, even
off the constraint manifold:

```
linear_a + linear_b = 0
COM_a x linear_a + angular_a + COM_b x linear_b + angular_b = 0
```

Consequently every generalized internal kick preserves total linear momentum
and orbital-plus-spin angular momentum. Straight COM drift preserves orbital
angular momentum and free rotational drift preserves world spin momentum.
Roundoff in the velocity/momentum conversion is the remaining measured drift.

Linear coordinates use connector-frame axes and include the derivative of the
moving axis. Fixed and locked D6 rotation coordinates reuse the existing
rotation-log analytic derivatives. Hinge/Cylindrical/Planar alignment uses
invariant axis dot products. ConeTwist swing/twist rows differentiate the
quaternion decomposition analytically. Finite differences appear only in tests,
which check all supported row kinds with rotated local frames and noncentral
COMs. Full off-diagonal inertia tensors are exercised throughout.

The nonlinear position tolerance is 1e-13, with a derived coordinate-roundoff
floor for positional rows: `max(1e-13, 32*EPSILON*(|anchor_a|+|anchor_b|))`.
The endpoint velocity residual must be at most 1e-12 per row. This roundoff floor
is essential for Distance as well as the other positional rows at large world
coordinates; the regression geometry budget remains 1e-7.

## Linear solution, memory and parallelism

The Schur operation `J M^-1 J^T` is matrix-free, using reusable per-body
six-component accumulators and cached world inverse inertia for each solve.
Branching acyclic graphs use Jacobi-preconditioned CG. Its true residual is
checked before success. Cyclic graphs use matrix-free CGLS to accommodate
singular/redundant row spaces without softening the physical constraints; the
outer solve still checks every original equality and endpoint velocity.

A path admits an exact structural solve. Joints are ordered along each path;
with at most six rows per joint, the symmetric Schur matrix has Cholesky
bandwidth at most 11. A diagonally scaled band factorization and checked
iterative refinement use O(N) storage/work instead of the poor O(N^2)
propagation observed with Jacobi-CG on long chains. No global dense matrix or
finite-difference Newton solve is built.

Independent islands run through the existing `StepExecutor` dispatch and ordered
writeback. There are no shared velocity atomics or global integration locks.
One large island remains sequential: the measurements below do not justify
introducing internal barriers or reductions.

After warm-up, the public step allocated **0 objects / 0 bytes** for 16 measured
steps at each of 100, 500 and 1000 Ball-chain bodies. The preserved pair check
also reported zero allocations over 600 warmed steps. These use the existing
process-wide allocation counter, run alone, outside general parallel tests.

## Conservation measurements

Compatible original fixtures, five seconds at 120 Hz:

| Fixture / metric | Before (`8dec14c`) | Universal island path |
|---|---:|---:|
| Ball pair absolute dP | 6.426255e-16 | 1.639786e-15 |
| Ball pair absolute dL | 1.777905e-14 | 1.759696e-14 |
| Ball pair absolute relative dE/E | 9.503626e-11 | 9.503334e-11 |
| Ball pair maximum anchor error, m | 1.128120e-15 | 7.200419e-16 |
| Ball pair maximum anchor speed, m/s | 8.694409e-17 | 1.340123e-16 |
| 31-body Ball tree absolute dP | 5.777428e-15 | 2.521942e-15 |
| 31-body Ball tree absolute dL | 1.824162e-3 | 1.540221e-13 |
| 31-body Ball tree absolute relative dE/E | 1.757225e-4 | 1.052124e-10 |
| 31-body Ball tree maximum anchor error, m | 7.403942e-6 | 3.700165e-15 |
| 31-body Ball tree maximum anchor speed, m/s | 1.256053e-4 | 3.831380e-13 |

The pair remains below every original bound. Its absolute relative energy errors
at 60/120/240 Hz are approximately 3.800e-10 / 9.503e-11 / 2.381e-11.
All supported joint kinds, partial bilateral ConeTwist settings, mixed trees,
nonredundant Ball triangles and redundant Fixed loops have deterministic
five-second invariant checks. Incompatible initial velocities are checked against
the independent reference without requiring conservation of projection energy.

Large-tree validation covers 100/500/1000 bodies, both paths and binary trees,
Ball-only and mixed joints, at all three frequencies for five seconds. Momentum
bounds scale with initial invariant magnitude; absolute angular roundoff on a
1000-body chain with kilometer-scale extent is about 1e-8, while its relative
error is near floating-point roundoff. This is distinct from the absolute 1e-10
requirement on the original two-body fixture.

Representative large-island results from the final five-second run at 120 Hz:

| Graph | Bodies | Absolute dP | Absolute dL | Absolute relative dE/E | Maximum equality residual |
|---|---:|---:|---:|---:|---:|
| Ball path | 1000 | 3.724645e-11 | 1.054063e-8 | 1.747825e-12 | 1.439290e-12 |
| Mixed path | 1000 | 6.551648e-11 | 7.766122e-9 | 3.248061e-14 | 2.779825e-13 |
| Ball binary tree | 1000 | 3.437147e-13 | 1.312717e-12 | 9.362222e-11 | 3.424864e-15 |
| Mixed binary tree | 1000 | 1.369012e-12 | 1.301826e-12 | 2.125570e-10 | 3.363629e-15 |

Residuals are coordinate values: metres for positional rows, radians for angle
rows, and dimensionless axis-dot/quaternion coordinates for alignment/swing rows.

The mixed binary-tree energy errors at 60/120/240 Hz were
8.502317e-10 / 2.125570e-10 / 5.313677e-11. All 36 large configurations completed;
the explicitly invoked release test took 294.42 seconds. Very small mixed-path
energy errors approach roundoff rather than yielding an indefinitely exact
fourfold ratio; all regression budgets remain enforced.

## Performance measurements

Windows, AMD Ryzen 9 7900X3D (12 cores / 24 logical processors), rustc 1.98.0.
Release, persistent Rayon pools, eight warm-ups and 32 samples; p50 wall time:

| Islands x bodies | 1 worker | 2 workers | 4 workers | 8 workers |
|---|---:|---:|---:|---:|
| 1 x 100 | 0.995 ms | 1.005 ms | 1.000 ms | 0.999 ms |
| 1 x 500 | 5.016 ms | 5.023 ms | 5.034 ms | 5.049 ms |
| 1 x 1000 | 10.387 ms | 10.114 ms | 10.152 ms | 10.120 ms |
| 8 x 100 | 6.828 ms | 4.228 ms | 2.987 ms | 2.505 ms |

For 8 x 100 bodies, p95 was 7.007 / 5.099 / 3.400 / 2.767 ms at
1/2/4/8 workers.

The repeated states match exactly across 1/2/4/8 workers. The final warmed
measurements above were repeated after the broad verification processes ended.
The eight-island case improved by about 2.73x at eight workers. Branching graphs retain
higher iterative cost: illustrative cold 120-Hz steps for 100/500/1000 bodies
were 1.73 / 12.30 / 28.74 ms for Ball trees and 2.39 / 20.66 / 54.43 ms for mixed
trees. These are not warmed percentile results and do not establish real-time
performance at all sizes. Banded path storage grows linearly; branching Schur
storage also grows linearly, while CG work grows with conditioning.

## Regression scope and limitations

Tests cover mixed eligible/legacy worlds with independent contacts and force
sources, four world step routes, stale shards, repeated-state determinism,
full inertia tensors, arbitrary local frames, finite-difference row derivatives,
rigid-motion invariance, degenerate geometry and atomic multi-island failure.
Existing contacts, CCD, forces, limits, motors, springs and cluster tests remain
part of the workspace verification.

Legacy counter/cache tests now explicitly use a collider where they intend to
exercise the legacy solver; their numerical budgets are unchanged. An existing
free Slider energy-convergence assertion now accepts the analytically expected
roundoff floor `128*EPSILON*E0`, because the new path reaches roughly 2.8e-15
relative energy error and cannot decrease strictly below that floor.

This implementation is not a solver for externally forced constrained motion,
contact/TOI trajectories, compliant or unilateral constraints. General large
closed mechanisms can be poorly conditioned; the small redundant/nonredundant
loop regressions do not guarantee convergence for every topology. The admission
preflight and explicit atomic error path are part of the supported behavior.

## Verification

- `cargo fmt --all -- --check`: PASS.
- `cargo check --workspace --all-features`: PASS.
- `cargo clippy --workspace --all-targets --all-features -- -D warnings`: PASS.
- `cargo test --workspace --all-features --no-fail-fast -- --test-threads=1`: **1015 passed, 0 failed, 50 ignored**, including doctests.
- `cargo test --workspace --release --all-features --no-fail-fast -- --test-threads=1`: **1015 passed, 0 failed, 50 ignored**, including doctests.
- `cargo test -p kapitsa-core --release articulated_graph_ -- --nocapture`: **11 passed**.
- The explicitly invoked `constrained_large_graphs_five_second_convergence` release test: **PASS**, all 36 large configurations.
- Explicit persistent-Rayon scaling and warmed-allocation tests: **PASS**. Three new long/process-wide tests are intentionally ignored in ordinary runs and were invoked explicitly; pre-existing ignored tests were not changed.
- `git diff --check`: PASS.

## Local implementation commits

- `a8ed751`: momentum-form matrix-free kernel and unit tests.
- `22e3f92`: invariant analytic geometry for supported joint kinds.
- `1babe7c`: universal island solve and singular/cyclic equality solution.
- `05e9069`: production island dispatch, original states, migration and banded paths.
- `bb028f5`: persistent-executor scaling and warmed allocation measurements.
- `4b89519`: long-run graph regressions, partial ConeTwist and geometry hardening.
- The commit containing this report records final verification.
