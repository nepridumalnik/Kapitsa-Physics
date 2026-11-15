# Aerodynamics: physical references and implementation boundaries

Kapitsa's geometry-based aerodynamics is an independent implementation. FAR is a behavioral and architectural reference, not a source of MIT-licensed implementation code. Voxelization represents geometry; it does not solve the fluid flow or determine aerodynamic coefficients by itself.

This document distinguishes physical equations from numerical approximations. Inclusion of a formula below does **not** mean that the corresponding model is implemented. Implementation status must be established from code and tests.

## Scalar geometry preparation

`kapitsa::aerodynamics::geometry::PreparedGeometry::prepare` prepares borrowed `GeometryInstance` meshes in one stable body-local frame. Each instance has a local transform and `GeometryOwner::Body` or `GeometryOwner::Part(PartId)`. It does not change the body, sample an atmosphere, calculate forces, or automatically prepare `Aerodynamics::Mesh` during a physics step.

```rust
use kapitsa::aerodynamics::geometry::{
    GeometryConfig, GeometryInstance, GeometryOwner, OpenMeshPolicy, PreparedGeometry,
};

let prepared = PreparedGeometry::prepare(
    &[GeometryInstance {
        mesh: &mesh,
        local_transform: Transform::identity(),
        owner: GeometryOwner::Body,
    }],
    GeometryConfig {
        cell_size: 0.25,
        max_cells: 100_000,
        max_work: 10_000_000,
        open_mesh_policy: OpenMeshPolicy::Reject,
        weld_tolerance: None,
        max_patch_work: 100_000_000,
    },
)?;
```

The scalar reference uses a uniform grid with a padded boundary, conservative triangle/box separating-axis intersection, and six-connected exterior flood filling for each closed instance. Instances are unioned before extracting sections and exposed faces. All cells touching a triangle are fully occupied, including boundary contact; this preserves thin geometry but overestimates its volume. Each axis section reports `A = occupied_cells * h²` and the mean of occupied cell centers. Volume is `occupied_cells * h³`. Exposed faces are streamed without a separate face buffer. Occupancy is x-fastest: `(z * ny + y) * nx + x`.

Overlaps count once; the lowest `GeometryOwner` wins attribution ties, independent of input order. This is geometric bookkeeping, not a physical load-sharing rule. Preparing parts separately and adding their exposed areas is **not** equivalent to preparing their union.

`Reject` requires paired oppositely directed indexed edges and nonzero signed volume. `SurfaceOnly` rasterizes nonclosed meshes without interior filling. By default, seams must be welded in the source index topology; `GeometryConfig::weld_tolerance` enables an explicit, per-instance positional welding pass used only for the closure check. It never mutates source vertices, never welds across instances or owners, and can change topology only within the configured tolerance. Self-intersections are not detected; sealed cavities use solid-fill semantics. Thin surfaces are preserved, but their physical thickness is not reconstructed. Grid-face normals remain axis-aligned and do not converge to smooth triangle normals for pressure integration.

`max_cells` and `max_work` are mandatory budgets, with hard ceilings of 1,048,576 cells and 100,000,000 work units. Preparation bounds triangle/box tests, input vertex scans and per-instance cell scans before allocating the grid; edge closure sorting adds `O(triangles log triangles)` work. Triangle/box tests are restricted to each triangle's conservative grid-index bounds; per-instance occupancy and flood-fill passes still traverse the grid. The scalar baseline is measured in [the aerodynamic benchmark report](../benchmarks/aerodynamics.md). Allocation failures and invalid/unrepresentable input are explicit errors. This preparation primitive supplies no SIMD or background cache publication; world force integration and collider conversion are described below. World movement and center-of-mass edits do not change the prepared local geometry; construction edits require an explicit new preparation.

References for geometric concepts: [Voxel](https://en.wikipedia.org/wiki/Voxel), [Hyperplane separation theorem — separating axis theorem](https://en.wikipedia.org/wiki/Hyperplane_separation_theorem), [Flood fill](https://en.wikipedia.org/wiki/Flood_fill). No FAR implementation code is used.

## World Mesh and Colliders integration

Non-cluster dynamic bodies support `Aerodynamics::Mesh` and `Aerodynamics::Colliders` through `PreparedGeometryCache`, direction-dependent exposed physical patches, and the pressure evaluator. The current pressure policy requires Mach >= 5 and samples primary atmosphere at the COM. An optional atmospheric sound-speed profile supplies local values; otherwise `PhysicsWorld::set_mesh_sound_speed` and `SimulationBuilder::mesh_sound_speed` configure the uniform fallback in m/s. The fallback defaults to 340 m/s and is not inferred from density. Both Tokio and Rayon builders apply it before service start. Additional explicit surfaces sample their own positions even when the COM is outside the atmosphere. Existing cluster bodies use explicit per-part geometry configuration. Converting a body with Mesh or Colliders into a cluster transfers its primary geometry model to the root part.

Each geometry-bearing primary configuration has a private shared identity token. Replacing a model or the whole `RigidBody` changes identity; world motion, COM edits and edits to other bodies do not. Prepared world entries compare identity before use and discard both the cache and collider-derived mesh on mismatch. The token remains alive in the cached entry, preventing allocator-address ABA. Collider topology and tessellation mutations retain their explicit per-body invalidation. Global motion `body_revision` is no longer used as geometry identity.

Collider geometry is derived from actual attached surfaces in body-local coordinates, including each collider's local rigid transform. Supported shapes are finite `Cuboid` (12 outward triangles), `Sphere` (latitude/longitude tessellation), `Cylinder` (endpoint axis with planar caps), `Convex` (indexed faces), `Triangle`, and `TriangleMesh` (source winding). Sensors are included, just like solid colliders. The world collision API independently restricts dynamic collider attachment to its supported convex shapes; conversion does not relax that restriction. Plane, HeightField, LevelSet, SkinnedTriangleMesh, Capsule, tapered shapes and any other unsupported shape return `UnsupportedModel`, with no AABB fallback. Empty collider sets also return `UnsupportedModel` in air.

`ColliderTessellation::new(radial_segments, sphere_rings, max_triangles)` validates 3..=128 radial segments, 3..=128 sphere rings (including poles), and 1..=65,536 aggregate triangles per body. Defaults are 16 segments, 8 rings, and 16,384 triangles. Budget exhaustion returns `TessellationBudgetExceeded` before emitting an over-budget shape. Referenced source faces, rather than unreferenced vertex arrays, are copied; allocation failures and unrepresentable transformed triangles are errors. `PhysicsWorld::set_collider_tessellation` changes a body's settings and invalidates preparation. Collider insertion, removal and mutable access invalidate that body's derived geometry/cache; motion and world pose do not. The pressure exposure is regenerated for each step's upstream direction (from each patch toward the oncoming air); fluid velocity is the opposite direction. Collider-derived triangle seams use explicit 1e-10 positional welding for closure detection and `SurfaceOnly` for open geometry. Existing voxel budgets (1,048,576 cells, 100,000,000 work units), 0.125 m cells and 0.01 m exposure epsilon still apply and may reject large or complex bodies independently of the tessellation budget.

This is a faceted, scalar hypersonic approximation, not an analytic sphere/cylinder fluid solution. Resolution affects forces as well as geometry. Overlapping colliders are prepared together; duplicate surfaces and occlusion use the same conservative occupancy rules as Mesh. No wing, SIMD or transonic model is introduced.

## Assembly part geometry

`PhysicsWorld::set_part_aerodynamic_mesh(part, Some(mesh))` assigns a physical mesh in the stable local frame of that `PartId`; `None` disables geometric loads. `set_part_aerodynamic_colliders(part, true)` selects actual part-owned colliders in place of Mesh. Disabling collider mode preserves an already selected Mesh. The private configuration is one enum, so both modes cannot coexist for one part. Matching `PhysicsClient` commands apply these operations in queue order, including while paused; metadata commands retain the existing unsupported-in-atomic-motion-batch policy.

`enable_cluster` and `enable_cluster_with_mass` transfer an existing body Mesh/Colliders model to the root part. Mesh ownership moves without copying vertices. Part IDs and configurations survive merge/split, and each rigid connected component has one solver body. Preparation unions configured instances in the current body's local frame and retains `GeometryOwner::Part` attribution. Coincident or shielded geometry is not independently added to pressure loads. Explicit additional surfaces remain additive by policy; configure each physical contribution once.

One body-wide `ColliderTessellation` applies to collider parts, including its aggregate output triangle budget. Split inherits these settings. Merge rejects differing settings before mutation; reconcile them explicitly rather than discarding one body's configuration. `set_part_collider_transform` validates ownership and composes a part-local transform before updating collision and aerodynamic state. Collider insertion, removal, transform changes and tessellation edits invalidate that assembly's preparation. Geometry and source collider meshes are reused on ordinary steps, pose/COM edits and unrelated-body edits. Tests prove cache identity preservation using preparation tickets, not allocator-address coincidence.

A preparation entry compares instance membership, mesh identity and part-local transforms. Changed unions are prepared in temporary entries and published only on success. Failed preparation does not apply partial forces. Collider transforms are already body-local after conversion and are not applied twice. When collider sources are present, the existing 1e-10 positional welding tolerance is used for closure checks across all instances; it does not move pressure vertices, change source normals or offset shadow rays. Geometry failures retain their `GeometryError` through `AerodynamicsError::Geometry` and its error source.

Part pressure currently retains Mach >= 5, COM atmosphere sampling and centroid rotational-flow quadrature. Vacuum skips preparation. Tests cover ordinary/sharded paths, mixed Mesh/Colliders assemblies, merge/split, overlap ownership, triangle budgets, configuration replacement, preparation errors, cleanup, and runtime commands on Tokio/Rayon. General physical equivalence to FAR is not established; the explicit subsonic hull and wing models below do not supply transonic hull physics.

## Conventions

Use SI units. Let `v = v_COM + omega × (x - x_COM) - wind(x)` be the body's velocity relative to the atmosphere at a force application point. The incoming fluid velocity is `-v`. Surface normals point outwards. Forces and torques are accumulated in world coordinates; geometry is prepared in body-local coordinates, independent of the center of mass.

Spherical atmospheres optionally interpolate world-space wind linearly with altitude between the reference radius and outer radius using `wind` and `wind_at_top`; values outside this interval clamp to the nearest endpoint. The default is constant wind. Explicit surfaces sample their own locations; mesh pressure samples wind at the COM for the whole mesh. The simple-surface timestep limiter also uses COM wind, so a wind gradient across a body is an approximation there. This profile does not model time variation or atmospheric rotation.

`sound_speed_profile(reference, top)` optionally interpolates positive sound speeds over the same altitude interval, clamped at both ends. Hull and wing loads sample it locally; mesh pressure uses the COM sample. This is an input profile, not a temperature or gas-composition model.

For distributed loads, `F = sum(F_i)` and `tau_COM = sum((x_i - x_COM) × F_i)`. A force and a torque are the authoritative output; a unique center of pressure need not exist.

References: [Rigid body — velocity](https://en.wikipedia.org/wiki/Rigid_body#Linear_and_angular_velocity), [Torque](https://en.wikipedia.org/wiki/Torque).

## Dynamic pressure and drag

`q = rho |v|² / 2`, `D = q C_D A`, `F_D = -D v / |v|` for nonzero speed.

Reference: [Drag equation](https://en.wikipedia.org/wiki/Drag_equation).

The reference area must match the coefficient's convention. This equation defines the drag coefficient; it is **not** a predictive closure for `C_D`. A constant drag coefficient is a deliberate simplified model, not a universal description across Reynolds and Mach numbers. Zero density and zero speed produce zero load.

## Distributed hull resistance: subsonic constants or explicit Mach data

`Aerodynamics::Hull(Vec<hull::HullSection>)` enables caller-parameterized sections
in ordinary and sharded body world stepping. `HullSection::new` takes a
section-to-body transform, local axis, friction development length, axial,
crossflow and wetted areas, axial and crossflow drag coefficients, and optional
`SkinFriction`. The axis is normalized; length must be finite and positive, and
areas and coefficients finite and nonnegative. Section origins are load points
in the stable body frame, not offsets from COM. Every point samples its own
atmosphere; point velocity includes `omega × r` and local wind. Sound speed uses
the local atmospheric profile when configured, otherwise the world fallback. Cluster conversion moves primary Hull
sections to the identity root part. `set_part_aerodynamic_hull(part, sections)` and
`part_aerodynamic_hull(part)` configure/query sections in a stable part-local frame.
They are stored with the part, alongside explicit wings, and follow its ownership
through topology changes. Part transforms and the assembly COM are applied once;
the evaluator already returns moment about that COM. Hull, wing and geometry
loads are additive; callers must avoid duplicated physical contributions. An
empty section list clears the part's hull loads; Simple, Wings and hypersonic
Mesh keep their existing semantics.

`PhysicsClient::set_part_aerodynamic_wings(part, wings)` and
`PhysicsClient::set_part_aerodynamic_hull(part, sections)` expose the corresponding
core setters through `PhysicsCommand::SetPartAerodynamicWings` and
`PhysicsCommand::SetPartAerodynamicHull`. Both replace the entire list in stable
part-local coordinates; an empty vector clears it. They use the existing ordered
command queue, including while paused, and reject stale/foreign part IDs through
the core's `ClusterError`. Like other metadata edits they are unsupported in
atomic motion batches (the entire batch is rejected without mutation).

Construct configurations through `kapitsa::aerodynamics::wing` and
`kapitsa::aerodynamics::hull`. The runtime transfers owned vectors in-process;
it does not require or provide wire serialization. `WingSurface` is `Clone` (not
`Copy`); cloning shares its optional immutable polar through `Arc`. `HullSection`
is `Clone`, and its immutable Mach curve shares owned `Arc` data.
No borrowed world references or mutable shared configuration cross the queue.
The setters do not change physical models, infer coefficients, exclude overlapping
loads or alter topology ownership.

The part-owned, data-driven Mach and runtime API extensions are **unverified**:
source inspection and formatting only; their regression tests and workspace
compilation were not run, at the user's request.

For world axis `a`, split relative velocity into `u_axial = a (v · a)` and
`u_cross = v - u_axial`. Each component contributes
`F_i = -0.5 rho |u_i| A_i Cd_i u_i`, using its **own** speed, not total-flow
dynamic pressure. Axial area must match axial/form Cd, crossflow area must match
projected-crossflow Cd, and skin friction uses wetted area. These are applications
of the [drag equation](https://en.wikipedia.org/wiki/Drag_equation), not inferred
hull coefficients. Sum forces and `r × F` moments about the current COM; no
intrinsic pitching moment or end-cap load is added. Choose enough sections to
resolve rotation and spatial atmospheric variation; this is point quadrature.

The default constant-Cd domain is local total Mach `|v| / sound_speed <= 0.3`,
including rotational velocity. An explicit `section.with_mach_curve(curve)`
replaces **both** constant coefficients with caller data over the curve's inclusive
Mach domain, also below 0.3; it never falls back to constants outside that domain.
`MachCurve::new(Vec<MachKnot>)` validates at least two strictly increasing finite
nonnegative Mach knots and finite nonnegative `axial_cd`/`crossflow_cd`. Coefficients
are piecewise linear between knots, exact at endpoints, with no extrapolation or
clamping. Mach 1 is an ordinary finite interpolation point. The immutable curve
owns its data in an internal `Arc`; cloning it reuses that allocation across
sections. `HullSection` is now `Clone`, not `Copy`, and evaluation borrows it.

This is a **DATA-DRIVEN transonic resistance interface**, not a predictive FAR
solver. There are no default invented transonic coefficients. `NormalForceCurve`
provides an optional immutable rectangular CFD/experimental axisymmetric table of
`C_N(Mach, alpha)` values and an explicit positive reference area;
`with_normal_curve` attaches it to a hull section. Its `angle_of_attack` field
uses **unsigned incidence** `alpha = atan2(|v_perp|, v_axial)` in `[0, pi]`, not
signed planar angle of attack; negative angle knots are rejected. Inside the
inclusive Mach/incidence domain, coefficients are bilinearly interpolated and
applied as `F_N = -q C_N A_ref v_perp / |v_perp|`, with full-speed
`q = rho |v|^2 / 2`. Positive `C_N` opposes crossflow and dissipates relative-flow
energy; finite negative `C_N` is permitted and intentionally reverses the force.
The force is normal to the **axis**, not to velocity, so it is not pure lift.
Reversing crossflow reverses force without changing incidence. Reversing the
axis maps incidence to `pi - alpha`, so asymmetric tables can change magnitude.
At zero crossflow (including zero speed), normal force is zero and its table's
Mach/incidence checks are skipped. Enabled drag and normal tables have independent
domains: unused constant drag does not impose Mach 0.3 on normal-only sections,
and absent normal data never restricts a configured Mach drag table. Vacuum and
fully disabled loads skip all coefficient-domain checks. Outside a sampled table's
domain evaluation fails with `UnsupportedMach` (also used for incidence range).
The caller must supply measured/CFD coefficients using this axisymmetric
incidence, force-sign and full-speed reference-pressure convention; a signed
planar lift table cannot be used unchanged. This incidence/domain correction and
its regression fixtures are source-only and unverified: tests, builds, checks,
Clippy and benchmarks were not run, as requested. No automatic lift, transonic closure,
or Newtonian/wing model is supplied. The same [drag equation](https://en.wikipedia.org/wiki/Drag_equation),
[Mach number](https://en.wikipedia.org/wiki/Mach_number), and [angle of attack](https://en.wikipedia.org/wiki/Angle_of_attack)
references define quantities, not coefficient data. Supply measured or
CFD data consistent with the sectional reference areas and component-speed
convention above, Reynolds conditions and angle of attack. A Mach-only curve does
not model variation with Reynolds number or angle of attack; applicability of
the supplied data remains the caller's responsibility. Definitions, not table
data: [drag coefficient](https://en.wikipedia.org/wiki/Drag_coefficient),
[Mach number](https://en.wikipedia.org/wiki/Mach_number),
[transonic](https://en.wikipedia.org/wiki/Transonic).

Optional `SkinFriction::new` requires explicit positive
finite dynamic viscosity, `BoundaryLayer::Laminar` or `Turbulent`, and positive
finite inclusive Reynolds bounds. It uses `Re_L = rho |v · a| L / mu` and the
mean flat-plate coefficients derived in [Skin friction](#skin-friction):
`Cf = 1.328 / sqrt(Re_L)` or `0.072 / Re_L^0.2`. Friction opposes axial flow
only; length is per-section development length, not accumulated automatically.
No transition threshold or mixed-regime closure is inferred. Zero axial speed
skips friction and its Reynolds check. Omitting friction is inviscid. These
flat-plate correlations are incompressible only: an enabled wetted friction load
above total Mach 0.3 returns `UnsupportedSkinFrictionMach`, even with an explicit
Mach Cd curve and zero axial speed. No compressible viscosity correlation is
available; viscosity alone cannot enable one. Vacuum and fully disabled loads
still skip these domain checks.

Finite kinematics and positive finite sound speed are validated even in vacuum
or with all areas disabled; these zero-load cases then skip Mach and Reynolds
checks. Invalid inputs, unsupported Mach/Reynolds and unrepresentable arithmetic
are explicit `HullError` values (retained through `AerodynamicsError::Hull` in
world integration). Products use binary exponent scaling so intermediate
pressure or Reynolds numerators need not fit when the complete result does;
truly nonfinite loads still fail rather than being clamped or discarded.

The empirical sectional resistance alone is **not potential-flow normal force or
lift**, predictive transonic continuation, area-rule wave drag, interference,
shielding, separation prediction or FAR equivalence. Geometry/voxel preparation does not
infer these empirical hull coefficients or automatically generate these resistance sections. Explicit
wings are separate; callers must avoid counting a physical contribution twice.

### Steady slender-body normal loads

`slender::hull_sections_from_voxels` creates explicit distributed hull sections
from the filtered axial area distribution of a prepared geometry union. It uses
the steady, small-incidence incompressible relation `dF_normal = rho * U *
(dA/dx) * v_cross * dx`, with the positive body axis directed toward the nose.
This is the body term of [Barrowman, equation 3-59](https://ntrs.nasa.gov/api/citations/20010047838/downloads/20010047838.pdf),
with signs converted to the body-relative velocity convention used here.

The caller selects the axis, centerline, smoothing radius and small-incidence
limit and must establish that the geometry is slender and axisymmetric and that
flow is steady and spatially uniform. Local Mach above 0.3, reverse flow at
nonzero incidence, incidence outside the selected limit, and any nonzero body
angular velocity are rejected. No pitch damping, wind-gradient derivative,
separation, wave drag or transonic continuation is inferred. The sections are a
snapshot and must be regenerated when geometry changes. They can be configured
through the existing body or part hull APIs and are additive to other models.
An analytic closed-body area fixture checks zero net normal force and moment
convergence; it is not a validation for arbitrary voxelized vehicles.

## Pressure loads

`C_p = (p - p_infinity) / q`, so a patch of area `A` and outward normal `n` has excess-pressure force `F = -q C_p A n`.

Reference: [Pressure coefficient](https://en.wikipedia.org/wiki/Pressure_coefficient).

Newtonian impact theory gives `C_p = 2 sin²(theta)`, where `theta` is the angle between an incident streamline and the surface tangent plane. For a windward patch this is `C_p = 2 max(0, n · v_hat)²`. Assigning zero excess pressure on leeward faces is an additional impact-model assumption, not a resolved wake solution.

This is a hypersonic impact approximation. It must **not** be represented as FAR's complete subsonic or transonic model. It does not predict attached-flow suction, boundary layers, base drag, or wing circulation. Voxel face orientation introduces a further geometry approximation; convergence of volume alone does not prove convergence of pressure loads.

Modified Newtonian theory replaces `2` with a stagnation pressure coefficient; it requires appropriate gas and Mach data. Do not infer those from density alone.

### Implemented patch pressure evaluator

`kapitsa::aerodynamics::pressure::PressurePatch` implements the unmodified Newtonian equation above for caller-selected physical surface patches. `PressureFlow` supplies an explicit sound speed, the atmosphere sampled at the patch, the body's rigid transform, body-local COM, and world-space linear/angular velocities. The evaluator computes point velocity `v_COM + omega × r`, subtracts wind, and returns world-space `PressureWrench { force, moment }` about the COM. Patch centers are body-origin-relative, not COM-relative; changing COM does not rebuild patches. Normals are normalized at construction. Evaluation allocates nothing and does not mutate the body.

For nonzero density, patch area and relative speed, Mach below 5 returns `UnsupportedMach`. This threshold is a declared engineering policy, not a universal accuracy boundary derived from the source. Vacuum, zero-area patches and zero relative speed return zero loads, after validating motion and sound speed. Invalid input and unrepresentable arithmetic are explicit errors.

`PreparedGeometry::surface_triangles(instances, max_work)` transforms source triangles while preserving physical normals. `exposed_surface_patches` partitions windward triangles along the projected edges and depth planes of candidate blockers, then tests the resulting polygon triangles upstream. Coplanar overlap is assigned once, in canonical owner/vertex order. Fully visible source triangles retain their original centroid sampling. Partially visible polygons are triangulated for area and first-moment integration under uniform pressure.

The shadow construction uses adaptive orientation predicates from the MIT/Apache-2.0 `robust` crate. Clipping is performed in the source triangle's affine domain, with an area-partition consistency check. Unsupported predicate ranges and inconsistent partitions return `PrecisionLoss`. Constructed intersections still use floating-point arithmetic; this is not an exact computational-geometry kernel. `epsilon` remains an API compatibility parameter and never offsets rays or ignores nearby blockers. The work budget checks the quadratic preflight, a preprocessing estimate, grid construction and sorting estimates, visited grid cells and entries, clipping vertex visits and final shadow tests. Exceeding it rejects the result, rather than returning partial loads. Vector growth uses fallible reservation. A complete bound for every intermediate operation still requires audit before this preparation can be accepted as production-complete.

`kapitsa::aerodynamics::pressure::evaluate_prepared_mesh(&PreparedSurfacePatches, PressureFlow)` evaluates each prepared physical patch through `PressurePatch` and returns the checked sum of world-space force and COM moment. It allocates nothing, mutates nothing and returns an error (not a partial wrench) if any patch evaluation or accumulation fails. A single explicit atmospheric state and sound speed are used for the whole set; no atmosphere sampling or per-owner load distribution occurs. Empty sets return zero after motion and sound-speed validation. The existing patch force convention and Mach >= 5 policy are preserved, without a new pressure closure.

Preparation is direction-dependent. The public uniform-direction primitive uses a body-local upstream direction. World preparation instead evaluates `v_COM - wind(COM) + omega ? (centroid - COM)` in the body frame for each original source triangle and prepares that triangle's visibility in its own upstream direction. A zero local velocity contributes no patch; stationary geometry in still air therefore remains at rest. Pure rotation can produce loads even with zero COM velocity. Pressure evaluation uses each surviving fragment's own centroid velocity.

This is centroid quadrature, not an exact integration of rotational flow: visibility direction is held constant within each source triangle. Refinement must resolve variation within triangles. A rotating plate regression compares force and moment with analytic integrals of `x?`, `z x?` and `x?` and verifies second-order error reduction across 1, 2, 4 and 8 subdivisions. Mach restrictions still apply to every evaluated pressure patch; rotational flow does not enable the hypersonic model at low Mach. Primary geometry still samples one atmospheric state at the COM; additional explicit surfaces sample their own positions.

The evaluator does not accept unfiltered mesh triangles as a complete ship model. Caller-selected instances must use consistent outward winding, and open-mesh policy remains explicit. The patch preparation is deliberately separate from `PreparedGeometry` voxel normals: those staircase normals would introduce a persistent orientation bias. This module is a pressure component, not complete FAR-like aerodynamics. World integration is limited to the supported geometry models above; atmosphere defaults and solver behavior are unchanged.

## Lift and finite wings

`wing::WingSurface` evaluates an explicitly positioned wing and returns world
force and moment about the current COM. Its transform maps wing coordinates
to body coordinates, with the origin at the application point. Wing axes are
+X toward the leading edge, +Y spanwise and +Z toward positive lift. For body
velocity relative to air, `alpha = atan2(-v_z, v_x)`: a nose-up wing in
horizontal flight has positive incidence. Point velocity includes rotation
and wind. By default, physical sideslip and reversed flow are unsupported; spanwise
components within 64 machine epsilons of planar speed are discarded solely
to tolerate rigid-transform roundoff. `WingSurface::with_spanwise_projection`
optionally enables a bounded normal-flow independence approximation, using a
caller-supplied maximum sideslip angle in radians. It evaluates Mach, dynamic
pressure and the supplied polar using only flow perpendicular to the span;
the computed force has no spanwise component. This approximation does not
predict finite-wing side force, sweep, tip effects or separated flow. The
caller must choose a suitable angular range and coefficients. It is based on
the normal-velocity concept described in [NACA TN 2335](https://ntrs.nasa.gov/api/citations/19930082969/downloads/19930082969.pdf). `Aerodynamics::Wing(surface)` enables
this model for a body's primary loads in ordinary and sharded world steps.
`Aerodynamics::Wings(Vec<WingSurface>)` evaluates multiple explicitly placed
wings; an empty list contributes no loads. Each wing samples its own atmosphere.
Atmosphere is sampled at the wing application point; sound speed uses the
local profile when configured, otherwise the world fallback. Errors retain `AerodynamicsError::Wing`
and step rollback semantics. Cluster conversion transfers primary wings into
root-part ownership; part-local application points remain stable as COM changes.
The wing does not exclude geometry from hull loads, and timestep convergence
must be checked (there is no implicit large-step wing stabilizer).

`aerodynamics::wing::FiniteWing` provides an independent coefficient primitive
for attached incompressible flow. It uses a thin-airfoil slope of `2 pi`, the
finite-span slope `a = a0 / (1 + a0 / (pi e AR))`, and induced drag
`CL² / (pi e AR)`, plus caller-specified profile drag. Aspect ratio, span
efficiency, zero-lift angle and attached-flow angle limit are explicit inputs.
By default, Mach above 0.3 or angles beyond that limit return `UnsupportedFlow`; no stall
or compressibility continuation is implied. `WingSurface` uses this primitive
for the world loads described above; it does not replace `Simple` lift or infer
wings from meshes. The finite-span slope relation is the classical lifting-line
approximation, not a FAR code port.

`FiniteWing::with_subsonic_compressibility(max_mach)` optionally uses a
Prandtl-Glauert lifting-line approximation, with slope
`2 pi / (sqrt(1 - M^2) + 2/(e AR))`. The caller must choose a maximum Mach
strictly below 1 and below the wing's critical Mach. This approximation is
for small-disturbance attached subsonic flow; it does not predict shock onset,
wave drag, separation or stall. Profile drag remains caller-supplied. The
incompressible default and its Mach 0.3 limit are unchanged. The underlying
compressibility transformation for finite wings is discussed in [NACA TN 1854](https://ntrs.nasa.gov/api/citations/19930082527/downloads/19930082527.pdf).

`FiniteWing::with_ndarc_lift_correction(divergence_mach, max_mach)` offers a
separate empirical lift-slope approximation from [NASA TP-2015-218751, section
8-8](https://rotorcraft.arc.nasa.gov/Publications/files/Johnson_NASA%20TP-2015-218751-app3.pdf).
It uses a piecewise correction through Mach 1 and replaces the optional
Prandtl-Glauert lifting-line approximation. Both parameters must be selected
for the intended configuration; the model does not infer shock onset or stall.
Its Mach limit, the attached-flow limit and any drag-rise correlation's domain
remain independent. The supersonic branch is an engineering approximation, not
a validated general supersonic wing model.

`FiniteWing::with_drag_rise(critical_mach, scale)` optionally adds the empirical
compressibility drag increment described in [NASA TP-2015-218751, section 8-8](https://rotorcraft.arc.nasa.gov/Publications/files/Johnson_NASA%20TP-2015-218751-app3.pdf).
The caller supplies both the critical Mach and the dimensionless coefficient
scale for the wing and its lift conditions; neither is inferred from geometry.
The correlation applies up to `M / critical_mach = 1.1`, while the wing's
independent configured Mach and attached-flow limits also apply. Outside these
limits the evaluator returns an error. This engineering correlation does not
predict stall or replace validation against suitable aerodynamic data.

`FiniteWing::with_lift_dependent_drag_rise` additionally uses the same NASA
correlation's `M_critical = M_critical_zero - sensitivity *
(0.03 CL + 0.19 CL^2 + 0.78 CL^3)`. The caller supplies the zero-lift critical
Mach, sensitivity and drag scale. With nonzero sensitivity, the correlation
rejects lift coefficients outside [0, 1]; it does not clamp or extrapolate.

`FiniteWing::with_ndarc_drag_rise` explicitly selects the bounded-ratio policy
of the NASA correlation: above `M / M_critical = 1.1` the increment is held at
its upper-bound value. This plateau is an engineering approximation, not a
general high-supersonic drag model. The wing's configured maximum Mach and
other model domains remain in force. The other drag-rise builders continue to
reject flows above the correlation's upper ratio.

### Voxel-volume shielding of explicit wings

`WingSurface::with_rectangular_footprint(center, chord, span)` opts a part-owned
wing into a conservative volume-occlusion approximation when its assembly has
configured aerodynamic geometry. The rectangle is in wing-local coordinates;
its area must match the wing planform area. The world samples the rectangle at
the prepared geometry's 0.125 m cell scale and excludes points inside occupied
voxels. It scales the wing load by the visible fraction and shifts the force
application point by the visible centroid's displacement from the rectangle
center. The original wing application point remains the atmospheric sampling
point. Without a footprint or without assembly geometry, existing wing behavior
is unchanged.

At most 1,000,000 sample points are processed per body during visibility
preparation; exceeding the budget returns an error and rolls the step back.
Visibility is reused while the source geometry, owning-part transform, wing
footprint and set of active wings remain unchanged. Changes to those inputs
recompute it; ordinary body motion and center-of-mass edits preserve it when
part-local transforms are unchanged. Solid meshes fill sealed
interiors; open surface-only meshes represent only their voxelized surfaces.
The caller must not include a wing's own footprint as solid hull geometry unless
it should be shielded. Voxel resolution affects partial exposure. This model
handles geometric volume enclosure only: it does not predict wake, flow shadow
behind an external body, aerodynamic wing-wing interference, or a FAR-equivalent
flow field. Caller-provided wing coefficients and the other aerodynamic models
remain subject to their documented domains and double-counting constraints.

### Explicit wing polars and control deflection — UNVERIFIED

`WingPolar::new(mach, angles, coefficients)` validates a rectangular grid:
there must be at least two finite strictly increasing knots on each axis,
Mach is nonnegative, and values have exactly `mach.len() * angles.len()` entries.
Angles are radians. Values are Mach-major, with AoA varying fastest; CL is
finite and signed, CD is finite and nonnegative. Coefficients are TOTAL
finite-wing values referenced to the surface planform area: `L = q S CL`,
`D = q S CD`, `q = rho V² / 2`. They are **not** 2D section coefficients.
Use `surface.with_polar(Arc::new(polar))` to replace the analytic coefficient
source, without adding `FiniteWing` profile or induced drag a second time.
Bilinear interpolation includes boundary knots; outside either supplied axis
returns `UnsupportedFlow`, never clamping or extrapolating. Transonic drag rise
or poststall lift reduction is represented only if the caller supplies such
values. Table support is not prediction of those phenomena. There is no default
poststall dataset; test numbers are synthetic regression fixtures, not source data.

`surface.with_control(WingControl::new(deflection, max_deflection, effectiveness)?)`
adds `effectiveness * deflection` to physical AoA for coefficient lookup only.
Deflection is radians, bounded by the supplied nonnegative maximum below pi/2;
invalid values are rejected, not saturated. Effectiveness is caller-supplied
and signed. Positive deflection with positive effectiveness increases effective
incidence. Physical drag and lift directions continue to follow the rotated
planar relative flow, not an artificially rotated velocity. Both analytic and
tabulated coefficient domains still apply after the shift. This is a limited
quasi-steady polar shift, **not** a universal flap/hinge/slat model, dynamic stall,
actuator dynamics or geometry-based effectiveness calculation. No intrinsic
pitching coefficient `C_m` is implemented; all moments remain application-arm
cross force about the current COM. Existing primary/part-owned wing setters
coexist with hull and mesh setters without suppressing overlapping loads.

Definitions only (not sources of numerical polar data):
[Lift coefficient](https://en.wikipedia.org/wiki/Lift_coefficient),
[Drag coefficient](https://en.wikipedia.org/wiki/Drag_coefficient),
[Stall](https://en.wikipedia.org/wiki/Stall_(fluid_dynamics)), and
[Flight control surfaces](https://en.wikipedia.org/wiki/Flight_control_surfaces).
Stall denotes the loss of attached-flow lift behavior, not a prescribed universal
curve. Control surfaces modify aerodynamic forces and moments; the incidence
shift above is only a caller-calibrated approximation of those effects.

This stage is **UNVERIFIED**: regression fixtures were added but not ignored;
no tests, build, check, clippy or benchmarks were executed at the user's request.

For a thin symmetric two-dimensional airfoil at small angle of attack, `c_l = 2 pi alpha`, with `alpha` in radians. More generally, `c_l = 2 pi (alpha - alpha_zero_lift)`.

Reference: [Airfoil — thin airfoil theory](https://en.wikipedia.org/wiki/Airfoil#Thin_airfoil_theory).

Assumptions: incompressible, inviscid, attached, small-disturbance flow and infinite span. This relation does not predict stall and must not be used as a finite-wing or arbitrary-body law.

For finite wings, `AR = b² / S` and `C_D_induced = C_L² / (pi AR e)`.

Reference: [Lift-induced drag](https://en.wikipedia.org/wiki/Lift-induced_drag).

The span efficiency `e`, wing span `b`, reference area `S`, profile drag and stall behavior require explicit modeling or data. A voxel surface is not enough to uniquely infer these. A wing-specific model must avoid counting the same geometry's full loads twice.

## Flow regimes

`M = |v| / a`, with sound speed `a`.

Reference: [Mach number](https://en.wikipedia.org/wiki/Mach_number).

`Re = rho |v| L / mu = |v| L / nu`, where `mu` is dynamic viscosity and `nu` is kinematic viscosity.

Reference: [Reynolds number](https://en.wikipedia.org/wiki/Reynolds_number#Definition).

Characteristic length `L` must match the correlation (for example chord or diameter). Density and wind alone do not determine Mach or Reynolds number.

For linearized two-dimensional subsonic flow, the Prandtl–Glauert correction is `C_p = C_p_incompressible / sqrt(1 - M²)`.

Reference: [Prandtl–Glauert transformation](https://en.wikipedia.org/wiki/Prandtl%E2%80%93Glauert_transformation).

It is not a transonic model: the singularity at Mach 1 is a breakdown of the approximation, not a physical infinite load. Clamping the denominator or interpolating around Mach 1 creates an engineering approximation that must be documented and validated separately. It does not reproduce FAR's area-distribution-based wave-drag model.

## Skin friction

For a laminar flat plate, the local Blasius relation is `c_f(x) = 0.664 / sqrt(Re_x)`. Integrating from the leading edge to plate length `L` gives the streamwise average `C_f = 1.328 / sqrt(Re_L)`. The distinction between local and averaged coefficients matters when summing sections.

The local turbulent one-seventh-power approximation is `c_f(x) = 0.0576 / Re_x^(1/5)`; its corresponding average is `C_f = 0.072 / Re_L^(1/5)`.

Reference: [Skin friction drag](https://en.wikipedia.org/wiki/Skin_friction_drag).

These are flat-plate boundary-layer approximations, not universal hull correlations. The source does not specify a universal transition Reynolds number. Automatic switching between these equations needs a separate explicit transition assumption; do not attribute a chosen threshold to Wikipedia. For uniform conditions, `D_f = q C_f S_wetted`.

## Area distribution and wave drag

Reference: [Whitcomb area rule](https://en.wikipedia.org/wiki/Whitcomb_area_rule).

This explains why the longitudinal total cross-sectional area distribution matters to wave drag. The article does not provide a complete wave-drag integral suitable for implementing FAR's transonic closure. Smoothing areas and differentiating them is geometry processing, not by itself a physical wave-drag formula.

For a slender axisymmetric body in linearized supersonic flow, the Kármán–Moore expression is `D_w = -rho U² / (4 pi) * integral integral S''(x) S''(y) ln|x-y| dx dy`.

Reference: [Sears–Haack body](https://en.wikipedia.org/wiki/Sears%E2%80%93Haack_body).

Its derivation assumes smooth pointed endpoints `S(0) = S(L) = 0` and a slender body. A numerical logarithm needs a dimensionless argument and consistent endpoint assumptions; the diagonal logarithmic singularity must be integrated rather than evaluated at zero. The analytic Sears–Haack benchmark is `D_w = 64 rho U² V² / (pi L⁴)`. The Wikipedia page does not provide a general Mach-number coefficient for this body formula; do not insert one by analogy with wing theory. This supersonic result is not a transonic closure, nor does it justify applying axial sections unmodified to arbitrary wing-body configurations. Differentiating noisy voxel areas twice requires a separately validated regularization strategy.

The Wikipedia article titled `Slender-body theory` concerns low-Reynolds-number Stokes flow; it must not be cited as justification for an aerodynamic lifting-body section formula.

### Explicit slender-body wave drag

`HullSection::with_wave_drag` accepts equally spaced samples of `dA/dx` in
metres with zero endpoints and precomputes the NACA TN 3478 linearized wave
integral. It contributes `rho * speed^2 * factor` at the section origin through
ordinary and sharded body and part stepping. The caller specifies that load point;
the integral does not determine a center of pressure. The model requires
supersonic flow aligned with the section axis and an axisymmetric slender body.
The caller is responsible for ensuring the supplied distribution is physically
valid and for avoiding double counting with other hull or mesh loads. This is
not a transonic transition model or a geometry-derived configuration.
By default nonzero density and speed outside the stated flow domain produce an
error. `WaveMachCurve` optionally supplies explicit dimensionless multipliers as
piecewise-linear Mach data; its domain is inclusive and never extrapolated. A
zero multiplier contributes no wave force, and nonzero multipliers still require
aligned flow. This is a data-driven transition, not a predictive transonic
closure; the caller must establish the applicability of the supplied data.

### Explicit voxel sections for wave drag

`HullSection::with_voxel_wave_drag(&prepared, axis, smoothing_radius)` can
configure the same slender-body wave-drag model from a prepared geometry union.
The selected axis must match the section's body-local axis. A centered box
filter with the caller-specified radius is applied before centered area
differencing; padding supplies zero slopes at both ends. The section's origin
remains the caller-selected application point. This is a one-time preparation:
geometry or topology changes require constructing a new configuration.

For explicit callers using `PreparedGeometryCache`,
`configure_voxel_wave_drag(section, axis, smoothing_radius)` reuses the derived
wave integral only while the current geometry request and filter settings match.
After a request changes, it refuses to configure a section until the matching
geometry has been prepared. The returned section is a snapshot: it is not
updated automatically when the world or assembly changes.

`Aerodynamics::VoxelWave(VoxelWave::new(...))` provides an opt-in body mode
that prepares and caches the wave configuration automatically during world
steps. Replacing the body's model changes geometry identity and triggers
preparation of the replacement. Vacuum skips preparation; failures return an
error without applying a partial step. This mode supplies only the configured
section loads and does not activate mesh-pressure loads.

Cluster conversion transfers the model to the root part. Alternatively,
`set_part_voxel_wave` selects one model owned by a part; `set_part_wave_source_mesh`
and `set_part_wave_source_colliders` select geometry-only inputs on other parts.
The union is prepared in the model owner's local frame, so a rotated owner is
supported. Geometry, collider, and topology changes invalidate preparation;
ordinary motion and COM edits do not. Splitting preserves the mode on its owner.
A merge of two clusters that both own a wave model is rejected before mutation;
remove or reconcile one model before merging. Source-only pieces detached from
the model do not produce wave or pressure loads by themselves.

Pressure-mode meshes and colliders remain additive. They participate in the
wave geometry union and can also produce Newtonian pressure loads; configure
contributions carefully to avoid counting the same physical effect twice.
Wave-model and geometry-only source surfaces do not produce separate Newtonian
pressure loads, but can shield pressure surfaces. Wave and pressure sample the
atmosphere independently at the wave application point and assembly COM,
respectively. These modes, including the corresponding `PhysicsClient` setters,
are not evidence of general FAR-equivalent hull or wing physics.

Voxel occupancy conservatively thickens geometry, and differentiation can
amplify grid noise. Callers must check convergence under grid and smoothing
refinement and establish that the shape and flow meet the slender axisymmetric
assumptions. For a triangulated slender Sears-Haack shape, a focused fixture at
cell sizes 0.05, 0.025, 0.0125 and 0.00625 m, with filter radii 1, 2, 4 and 8
cells, produced wave drags approximately 0.0891, 0.0456, 0.0323 and 0.0276 N
versus an analytic 0.0279 N at density 1 kg/m^3 and speed 2 m/s. The finest result
is about 1% low. This fixture does not establish convergence or accuracy for
arbitrary voxelized vessels or wave loads.

## Numerical validation requirements

World pressure failures retain their original `PressureError` in
`AerodynamicsError::Pressure`, including `std::error::Error::source`.
In particular, unsupported Mach is not reported as an unsupported geometry
model and does not silently select another physical model. Regression tests
check motion rollback for body and assembly geometry in ordinary and sharded steps.

- Validate forces and torques, not only occupied voxels.
- Measure geometric and force convergence separately as resolution increases.
- Check symmetry, frame covariance, rotational damping and force direction.
- Check union behavior for overlapping geometry and fully enclosed components.
- Preserve part attribution and load ownership through topology changes.
- A center-of-mass change must alter moment arms without requiring geometry preparation.
- Reject stale prepared results after changes to geometry, topology or preparation settings.
- Open surfaces need an explicit policy; thin wings must not silently disappear.

## FAR reference scope and licensing

The inspected upstream files are [VehicleVoxel.cs](https://github.com/ferram4/Ferram-Aerospace-Research/blob/master/FerramAerospaceResearch/FARPartGeometry/VehicleVoxel.cs) and [VehicleAerodynamics.cs](https://github.com/ferram4/Ferram-Aerospace-Research/blob/master/FerramAerospaceResearch/FARAeroComponents/VehicleAerodynamics.cs). They demonstrate geometry preparation, cross-section analysis, aerodynamic sections and separate wing integration. Their headers state GPL version 3 or later; Kapitsa is MIT.

Translating GPL source into Rust is not an independent implementation merely because the language differs. Equations and assumptions here are independently documented from mathematical references. Wikipedia article prose and diagrams are not copied into the code. Wikipedia is a convenient formula reference, not an experimental validation dataset or a guarantee that a collection of equations reproduces FAR.
