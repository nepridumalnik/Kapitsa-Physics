# Voxel hull acceptance review: closure and rotation

**Status: numerical opt-in, not accepted for physical flight prediction.**
`MeshPressureModel::ModifiedNewtonianTriangleQuadrature` is already exposed
as an explicit prepared-geometry option; it is not the default. This review
documents what is supported by the evidence, not a new runtime gate.

## What the pressure source supports

R. L. Stallings Jr., [NASA TR R-274 (1967)](https://ntrs.nasa.gov/citations/19670030271),
Appendix A, derives blunt-body drag by integrating the modified Newtonian
pressure coefficient `Cp = Cp,max cos²(theta)`, with `theta` measured from the
surface normal. For a windward patch the same equation is
`Cp = Cp,max max(0, n · u/|u|)²`; use the normal-shock stagnation coefficient
for calorically perfect air as given in [Aerodynamics](aerodynamics.md#modified-newtonian-compression-below-mach-5).
The patch references are `F = -rho |u|² Cp A n / 2` and `tau = r × F`, where
`u = v_COM - wind(COM) + omega × r` and `r` is measured from the COM.

The report tests nonrotating bodies of revolution at zero incidence with an
afterbody diameter of 0.191 m and a diameter-based Reynolds number of
1.88 million. Its pressure-distribution
discussion and figures 4–7 support a narrow starting point: windward
compression on a hemisphere-like blunt nose in steady supersonic flow near
the upper test Mach numbers (4.06 and 4.63). The report shows limits for
shoulder pressure. Figure 8 compares sphere flight drag with integrated model
pressures using different assumed base pressures; it is not a direct
validation of the code's zero-base-pressure total wrench. Neither the experiment nor
those equations justify arbitrary geometry, incidence, Reynolds numbers,
stabilizer interactions, rapid rotation or a transonic trajectory. The two
test Mach numbers are evidence points, not validated interval endpoints for
arbitrary hulls. The evaluator's Mach 0–5 input guard is not this evidence.

### Acceptance boundary (1.2)

- **Physical use supported by the source:** an engineering estimate of
  windward nose compression on an isolated hemisphere-like blunt body,
  in steady continuum air at zero incidence and negligible rotation, at the
  Mach and Reynolds conditions above. No error bound is assigned for a
  Kapitsa geometry, total force or moment. Viscosity and mean free path are
  not inputs to this closure; passing the Mach guard cannot establish Re or
  continuum applicability. The subsonic stagnation formula and continuity
  at Mach 1 do not validate the surface-pressure law in those regimes.
- **Numerically supported geometry:** the specific sphere, nested sphere,
  closed cube, off-center planar patches, and open-bore annulus cases below,
  with fixed geometry and the stated mesh/grid refinements. These are not
  universal minimum feature sizes or a guarantee for arbitrary orientations,
  thin walls, small gaps, coincident surfaces or cavities. Inspect exposure
  and wrench sensitivity separately; a resolved occupancy grid does not
  prove convergence of visibility or pressure quadrature.
- **Work limits:** the fixtures set `max_cells = 1,048,576` and
  `max_work = max_patch_work = 100,000,000`. Preparation checks the padded
  grid and its own work budget. Exposure uses a separate, capped surface-work
  counter; three-point integration then uses three samples per returned
  triangle fragment. The benchmark below records padded grid sizes, a
  measured exposure threshold and memory on a specific union; it does not
  establish a whole-step time or memory bound for other meshes. An exhausted
  budget is a typed failure, not permission to use partial loads.
- **Not accepted:** physical stability, maneuver or descent predictions,
  arbitrary rotating or stabilizer-equipped hulls, and all-Mach trajectories.
  A synthetic trajectory that stays inside its own numerical Mach window
  is still not evidence for the geometry or the physical coefficients.

## Can a pure-rotation regression stay inside a narrow local Mach band?

Yes, by bounding the radius of **all** candidate pressure points away from
the spin axis. In still air with stationary COM and uniform COM sound speed
`a`, `M = |omega| r_perp/a`, where `r_perp` is distance to the axis. A centered
solid plate extending to the axis generally fails a positive lower bound:
windward points can approach the axis with nonzero velocity, and coarse
centroids can mask the failure. Testing only nonzero sampled forces is not
evidence that refinement will preserve the domain.

For an equation/domain regression, consider one closed annular prism with
nested regular 16-gons, inner apothem 10 m, outer circumradius 11 m, and
height 0.5 m. Put the COM on the symmetry axis. In a synthetic still atmosphere
with `a = 340 m/s`, choose `|omega| = 4.2 a / (10 m) = 142.8 rad/s`.
Every point on the mesh, including any clipped fragment centroid within a
source triangle, has `10 m <= r_perp <= 11 m`; thus its local Mach is between
4.20 and 4.62. The lower bound follows because each source triangle is on the
outside of at least one inner polygon support line; the upper bound follows
from convexity of the cylinder of radius 11 m. End faces have tangential flow,
and side faces have windward regions. For any windward pressure force,
`omega · tau = u · F < 0`; opposite visible patches cancel net force by
half-turn symmetry but add their axial moments. These are independent sign
and symmetry checks, not evidence for quasi-steady rotating aerodynamics.
In particular this spin case is not a physically supported flight demo.

The actual test uses the scaled-down geometry and separate voxel and mesh
refinements described below. The full-scale example is a Mach-bound argument,
not a tested geometry or work-budget result.

## Remaining acceptance gate

### OpenSpec implementation audit

The prepared-only world option supports standalone primary mesh
bodies and bound clusters. A standalone unit cube and the same mesh and mass
in a single-part cooked cluster agree in both step paths within `1e-10`
relative to the reference force, force times the 1 m edge, and velocity.
The annular bore stays clear on its axis and its axial exposed projected
area matches the polygon reference at 0.05, 0.025 and 0.0125 m cell sizes.
The interior wall sample at `(1.05, 0, 0)` m is occupied at all three
resolutions. Previously the 0.025 m grid missed it: reconstructing voxel
centers in world coordinates introduced roundoff at grid-aligned faces,
allowing exterior flood fill into the wall. Rasterization now tests triangle
overlap in grid coordinates. The projected-area tolerance is unchanged.
For task 2.2, an analytical two-owner, off-center plate fixture checks each
world-space force and COM moment, owner reconstruction, rigid-frame covariance,
and exact zero wrenches in vacuum. The two 0.5 m² triangles are offset by
4 m, with a COM at `(0, 1, 0)` m, a 0.125 m grid and Mach 4.63. Force tolerance
is `1e-10` times the reference force; moment tolerance uses the same scale
times the 1 m reference length. These are roundoff-scale fixture tolerances,
not physical uncertainty. See the rotation results below for the distinct
discretization error. A cooked pressure mesh and explicit part
wings on the same part are rejected rather than double-loaded. Use
`set_part_wing_shielding_mesh_shared` to explicitly associate the whole part
mesh with its configured wings. The mesh remains in the union and can shield
downstream hull pressure, but receives no pressure load itself. The ordinary
and sharded regression verifies both effects and one surviving wing wrench.
The regression also verifies that an independent, offset wing and an exposed
hull produce separate loads, and that changing deflection changes the single
wing wrench without adding a hull-pressure owner. Task 2.3 remains open:
cross-part validation rejects positive-area overlap between a declared wing
rectangle and a coplanar pressure triangle. Distinct parallel surfaces and
edge-only contact are not rejected by this check. It does not infer a wing
footprint when none is declared or classify noncoplanar wing representations.
The per-body check allocates no scratch and stops with
`GeometryError::WorkBudgetExceeded` after one million part visits, wing
visits and triangle tests combined. The
ordinary and sharded pending-policy matrix covers
merge and stale bindings, but does not exercise both split and removal during
an active cook (3.1). The world-step geometry-budget rollback and API baseline
checks are still open (3.2), as are the viewer trajectories and timestep
evidence (4.1–4.2). Task 3.3 measurements follow below.

### Preparation and repeated evaluation cost (3.3)

Run `cargo bench -p kapitsa-core --bench voxel_hull`. The synthetic fixture
is a closed 1 m cube with a concentric 0.25 m cube in a second owner slot
(24 source triangles total). At Mach 4.63 along +x in still air it gives two
exposed patches; pressure uses three-point triangle quadrature. The same
meshes are cooked at 0.0625 m (5,832 padded cells, 18³) and approximately
0.0101523 m (1,030,301 padded cells, 101³, 98.3% of the hard cell ceiling).

Optimized Rust 1.98.0, Windows x86-64/MSVC, Ryzen 9 7900X3D. Times are medians
of seven warmed batches (20 cooks per batch for the representative case,
one near-ceiling cook per batch, 500 operations per batch otherwise). The
allocation pass is separate: requests count successful allocations and
reallocations; peak/live report incremental requested heap bytes during a
single operation and at its return, respectively.

| Fixture / operation | µs/op | Requests/op | Peak bytes | Live bytes |
|---|---:|---:|---:|---:|
| Representative cook | 365.320 | 11 | 200,800 | 142,416 |
| Near-ceiling cook | 19,438.700 | 11 | 35,042,706 | 24,739,632 |
| Representative exposure + pressure | 28.682 | 29 | 5,872 | 0 |
| Near-ceiling exposure + pressure | 28.498 | 29 | 5,872 | 0 |
| Near-ceiling exposure + pressure at work threshold | 28.460 | 29 | 5,872 | 0 |
| Cell budget rejection | 2.541 | 4 | 458 | 106 |
| Patch work budget rejection | 27.173 | 32 | 5,872 | 106 |
| Pressure workspace setup (either grid) | 0.062 | 2 | 288 | 288 |
| Pressure on frozen patches (representative / near ceiling) | 1.038 / 1.035 | 0 | 0 | 0 |

The benchmark checks `CellBudgetExceeded` with a cap one cell below the
near-ceiling grid. For that same fixture, probing the patch work cap finds
that 7,301 accounting units suffice and give the same wrench as the default
cap; 7,300 returns `WorkBudgetExceeded`. This is a fixture-specific threshold,
not a stress test at the hard 100,000,000-unit ceiling. The live bytes on
rejection belong to the returned error. On success, the cook peak includes
transient allocations and retained output (peak exceeds retained bytes by
58,384 and 10,303,074 bytes, respectively); evaluation's 5,872 bytes are
entirely transient, in addition to the already allocated workspace. Counts
exclude preexisting meshes and instance lists, allocator overhead, stack
usage and system RSS. The benchmark isolates geometry and wrench staging;
it does not measure world stepping, async scheduling, worker concurrency,
or more complicated visibility workloads. The small difference between the
two evaluation timings is not evidence of a speedup at finer voxel sizes.

### Numerical evidence

`cargo test -p kapitsa-core --lib voxel_hull_acceptance_tests -- --nocapture`
passed all six focused tests. The moving pressure fixtures here use synthetic
still air (`rho = 1 kg/m³`, `a = 340 m/s`); they do not match tunnel Reynolds
conditions. Selected results and tolerances:

- For a 1 m radius sphere, `Cd = Cp,max/2` is the exact windward compression
  integral, with `pi R²` as reference area. At Mach 4.06 and 4.63 the absolute
  errors were respectively 0.093996/0.094550, 0.023120/0.023257, and
  0.005766/0.005800 at 8, 16 and 32 azimuth segments and voxel sizes 0.25,
  0.125 and 0.0625 m. This is joint mesh/voxel refinement, not an isolated
  voxel error measurement. The regression requires monotone error reduction
  and a finest-grid absolute `Cd` error below 0.03, with transverse force
  below `1e-10 qA` and COM moment below `1e-10 qA R`. These are numerical
  regression limits, not agreement bands from tunnel scatter. A one-cell preparation cap and a one-unit exposure
  work cap returned their distinct typed failures. This checks the equation,
  not agreement with the report's measured drag.
- The tested annulus has inner apothem 1 m, outer circumradius 1.1 m and
  height 0.05 m, with `|omega| = 1428 rad/s`. It ran at voxel sizes
  0.05, 0.025 and 0.0125 m; axial projected area agrees with the difference
  of the polygon areas to `1e-10` relative at all three sizes. The source visibility
  samples passed the Mach check, and the analytic bound also covers fragment
  centroids. All three resolutions produced net force below `2e-12 N` and
  axial moment `-519.778 N m`. However, all three also produced a transverse
  moment of about `-6.467 N m` on x, which half-turn symmetry does not permit.
  This is an open discrepancy, not a toleranced pass for full wrench symmetry;
  voxel refinement alone did not reduce it.

  Generating exactly antipodal coordinates and flipping the source triangle
  diagonals without changing outward winding both leave the transverse moment
  unchanged. The clipped fragments at 0.025 m had 32 of 48 centroids without
  a half-turn counterpart of the same area and normal, while their integrated
  transverse first area moments for constant pressure cancel to about
  `2e-20 m³`. This points to the interaction of fragment triangulation and
  nonlinear centroid pressure quadrature, not physical shape asymmetry or
  a net imbalance detectable by constant pressure. A convergent integration
  fix must preserve visibility semantics and work budgets; no correction is
  applied to the result. The opt-in three-point triangle rule in
  `OwnerPressureWorkspace` reduces the transverse component to about
  `-0.000548 N m` on the same exposed fragments without changing visibility.
  It also changes the axial component from `-519.778` to `-779.799 N m`;
  a composite midpoint calculation on the same frozen fragments gives
  `-775.758`, `-778.806`, and `-779.569 N m` at 8, 16, and 32 subdivisions
  per edge. Successive differences shrink by about fourfold. This tests
  axial quadrature convergence, not convergence of rotational visibility.
  The three-point rule has equal weights at
  the barycentric permutations of `(2/3, 1/6, 1/6)` (degree-two exact).
  The transverse residual on the coarse mesh is not zero; the refinement
  checks below quantify its sensitivity. The world
  leaves this quadrature off by default. Select
  `MeshPressureModel::ModifiedNewtonianTriangleQuadrature` to use it with the
  bound cooked assemblies, including single-part clusters. In air, unbound
  or stale mesh assemblies fail with `ShapeRegistryError::InvalidBinding`
  rather than cooking during the step. For standalone `Mesh` bodies call
  `PhysicsWorld::cook_primary_hull` before stepping. Mesh replacement or
  cavity-policy changes require another cook. Standalone `Colliders` are not
  supported by this option. Existing assembly pending
  policies apply; `KeepPrevious` requires unchanged topology, ownership and
  cook-frame transforms. This does not extend the physical applicability of
  the closure.

  A separate visibility-only regression fixes the voxel size at 0.025 m
  and subdivides each source triangle into four, keeping the same annular
  surface and rotation. At 128 source triangles the query returns 48 fragments,
  `0.171679483374 m²` of exposed area, and a transverse first area moment
  of about `3.1e-20 m³`. At 512 and 2048 triangles the geometry prepares
  successfully. Before the grazing-incidence fix, both rotational visibility
  queries returned `PrecisionLoss` before pressure quadrature.
  The production fix retains exact signed-dot evaluation near cancellation and
  falls back to a genuinely windward source-vertex velocity when the clipped
  representative rounds to a tangent. This does not discard small positive
  fragments. A triangle with no positive vertex incidence has no windward
  area; if a positive region cannot supply a representable windward ray,
  the visibility query still returns `PrecisionLoss`.
  In those previous failures, `normal · upstream <= 0` after interpolation
  and normalization of the clipped polygon's representative velocity. The windward clip produces
  grazing slivers with source-triangle area fractions of about `5.77e-31`
  and `2.14e-30`, respectively. In both cases the normalized representative
  direction rounds to `(-0.5555702330196022, -0.8314696123025452, 0)` against
  normal `(-0.8314696123025452, 0.5555702330196022, 0)`, so the dot product
  cancels to zero. No area threshold or incidence tolerance has been changed.
  After the fix, visibility-only refinement returns 48, 136 and 352 fragments
  for 128, 512 and 2048 source triangles. Exposed area remains
  `0.171679483374 m²`; transverse first moments are approximately
  `3.0e-20`, `-7.3e-20`, and `-1.1e-19 m³`, while the axial first moment is
  `0.01842115313226861`, `0.018421153132268584`, and
  `0.018421153132268577 m³`. This verifies visibility refinement independently
  of pressure quadrature.

  The same mesh sequence with three-point pressure integration gives axial
  moments of `-779.799398`, `-779.799398`, and `-779.821194 N m`; the
  transverse moment falls from `5.48e-4 N m` to roundoff on the two refined
  meshes. Net force remains below `5e-13 N`, consistent with half-turn
  symmetry. With the coarse triangulation fixed, changing voxel size across
  `0.05`, `0.025`, and `0.0125 m` leaves the integrated moment unchanged
  within `1e-12` relative to the axial moment. These checks establish
  numerical consistency on this fixture, not a general convergence rate
  or physical validation of the rotating-flow pressure closure.
  At the two refined meshes, the regression requires transverse moment below
  `1e-12 |tau_z|` and axial change below `1e-4` of the coarse axial moment.
  The net-force limit is `1e-12 |tau_z| / (1.1 m)`.

- With nested concentric spheres (1 m and 0.25 m radii, 0.125 m grid) at
  Mach 4.63, evaluating the cooked union
  attributes no direct pressure to the enclosed slot and matches the outer
  sphere alone in force and COM moment to `1e-10` of the respective reference
  scales. This checks cooked source-slot attribution, not world part binding.

The blunt-body source does not establish physical applicability for either
viewer configuration, with or without stabilizers, or for hull–wing
interference. Demonstration evidence must be reported separately, including
geometry and timestep refinement, work limits and local model domains over
the full demonstrated trajectory. A physical-flight claim would require
matched pressure/force/moment data and, for explicit wings, separately sourced
wing inputs; staying inside the solver's guard is not an alternative.
