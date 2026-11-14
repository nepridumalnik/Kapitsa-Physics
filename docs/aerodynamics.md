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
    },
)?;
```

The scalar reference uses a uniform grid with a padded boundary, conservative triangle/box separating-axis intersection, and six-connected exterior flood filling for each closed instance. Instances are unioned before extracting sections and exposed faces. All cells touching a triangle are fully occupied, including boundary contact; this preserves thin geometry but overestimates its volume. Each axis section reports `A = occupied_cells * h²` and the mean of occupied cell centers. Volume is `occupied_cells * h³`. Exposed faces are streamed without a separate face buffer. Occupancy is x-fastest: `(z * ny + y) * nx + x`.

Overlaps count once; the lowest `GeometryOwner` wins attribution ties, independent of input order. This is geometric bookkeeping, not a physical load-sharing rule. Preparing parts separately and adding their exposed areas is **not** equivalent to preparing their union.

`Reject` requires paired oppositely directed indexed edges and nonzero signed volume. `SurfaceOnly` rasterizes nonclosed meshes without interior filling. By default, seams must be welded in the source index topology; `GeometryConfig::weld_tolerance` enables an explicit, per-instance positional welding pass used only for the closure check. It never mutates source vertices, never welds across instances or owners, and can change topology only within the configured tolerance. Self-intersections are not detected; sealed cavities use solid-fill semantics. Thin surfaces are preserved, but their physical thickness is not reconstructed. Grid-face normals remain axis-aligned and do not converge to smooth triangle normals for pressure integration.

`max_cells` and `max_work` are mandatory budgets, with hard ceilings of 1,048,576 cells and 100,000,000 work units. Preparation bounds triangle/box tests, input vertex scans and per-instance cell scans before allocating the grid; edge closure sorting adds `O(triangles log triangles)` work. This deliberately simple scalar reference scans the full grid per triangle and is not yet optimized for large ships. Allocation failures and invalid/unrepresentable input are explicit errors. This preparation primitive supplies no SIMD or background cache publication; world force integration and collider conversion are described below. World movement and center-of-mass edits do not change the prepared local geometry; construction edits require an explicit new preparation.

References for geometric concepts: [Voxel](https://en.wikipedia.org/wiki/Voxel), [Hyperplane separation theorem — separating axis theorem](https://en.wikipedia.org/wiki/Hyperplane_separation_theorem), [Flood fill](https://en.wikipedia.org/wiki/Flood_fill). No FAR implementation code is used.

## World Mesh and Colliders integration

Non-cluster dynamic bodies support `Aerodynamics::Mesh` and `Aerodynamics::Colliders` through `PreparedGeometryCache`, direction-dependent exposed physical patches, and the pressure evaluator. The current pressure policy requires Mach >= 5 and samples primary atmosphere at the COM. `PhysicsWorld::set_mesh_sound_speed` and `SimulationBuilder::mesh_sound_speed` configure a finite positive uniform sound speed in m/s; callers must match their atmospheric medium. The compatibility default remains 340 m/s and is not inferred from density. Both Tokio and Rayon builders apply the configured value before service start. Additional explicit surfaces sample their own positions even when the COM is outside the atmosphere. Clusters still reject the geometry models.

Each geometry-bearing primary configuration has a private shared identity token. Replacing a model or the whole `RigidBody` changes identity; world motion, COM edits and edits to other bodies do not. Prepared world entries compare identity before use and discard both the cache and collider-derived mesh on mismatch. The token remains alive in the cached entry, preventing allocator-address ABA. Collider topology and tessellation mutations retain their explicit per-body invalidation. Global motion `body_revision` is no longer used as geometry identity.

Collider geometry is derived from actual attached surfaces in body-local coordinates, including each collider's local rigid transform. Supported shapes are finite `Cuboid` (12 outward triangles), `Sphere` (latitude/longitude tessellation), `Cylinder` (endpoint axis with planar caps), `Convex` (indexed faces), `Triangle`, and `TriangleMesh` (source winding). Sensors are included, just like solid colliders. The world collision API independently restricts dynamic collider attachment to its supported convex shapes; conversion does not relax that restriction. Plane, HeightField, LevelSet, SkinnedTriangleMesh, Capsule, tapered shapes and any other unsupported shape return `UnsupportedModel`, with no AABB fallback. Empty collider sets also return `UnsupportedModel` in air.

`ColliderTessellation::new(radial_segments, sphere_rings, max_triangles)` validates 3..=128 radial segments, 3..=128 sphere rings (including poles), and 1..=65,536 aggregate triangles per body. Defaults are 16 segments, 8 rings, and 16,384 triangles. Budget exhaustion returns `TessellationBudgetExceeded` before emitting an over-budget shape. Referenced source faces, rather than unreferenced vertex arrays, are copied; allocation failures and unrepresentable transformed triangles are errors. `PhysicsWorld::set_collider_tessellation` changes a body's settings and invalidates preparation. Collider insertion, removal and mutable access invalidate that body's derived geometry/cache; motion and world pose do not. The pressure exposure is regenerated for each step's upstream direction (from each patch toward the oncoming air); fluid velocity is the opposite direction. Collider-derived triangle seams use explicit 1e-10 positional welding for closure detection and `SurfaceOnly` for open geometry. Existing voxel budgets (1,048,576 cells, 100,000,000 work units), 0.125 m cells and 0.01 m exposure epsilon still apply and may reject large or complex bodies independently of the tessellation budget.

This is a faceted, scalar hypersonic approximation, not an analytic sphere/cylinder fluid solution. Resolution affects forces as well as geometry. Overlapping colliders are prepared together; duplicate surfaces and occlusion use the same conservative occupancy rules as Mesh. No wing, SIMD or transonic model is introduced.

## Conventions

Use SI units. Let `v = v_COM + omega × (x - x_COM) - wind(x)` be the body's velocity relative to the atmosphere at a force application point. The incoming fluid velocity is `-v`. Surface normals point outwards. Forces and torques are accumulated in world coordinates; geometry is prepared in body-local coordinates, independent of the center of mass.

For distributed loads, `F = sum(F_i)` and `tau_COM = sum((x_i - x_COM) × F_i)`. A force and a torque are the authoritative output; a unique center of pressure need not exist.

References: [Rigid body — velocity](https://en.wikipedia.org/wiki/Rigid_body#Linear_and_angular_velocity), [Torque](https://en.wikipedia.org/wiki/Torque).

## Dynamic pressure and drag

`q = rho |v|² / 2`, `D = q C_D A`, `F_D = -D v / |v|` for nonzero speed.

Reference: [Drag equation](https://en.wikipedia.org/wiki/Drag_equation).

The reference area must match the coefficient's convention. This equation defines the drag coefficient; it is **not** a predictive closure for `C_D`. A constant drag coefficient is a deliberate simplified model, not a universal description across Reynolds and Mach numbers. Zero density and zero speed produce zero load.

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

The implementation uses floating-point predicates; it is not an exact computational-geometry kernel. `epsilon` remains an API compatibility parameter and never offsets rays or ignores nearby blockers. The work budget checks the quadratic preflight, clipping vertex visits and final ray tests. Exceeding it rejects the result, rather than returning partial loads. Vector growth uses fallible reservation. A complete bound for preprocessing, sorting and every intermediate operation still requires audit before this preparation can be accepted as production-complete.

`kapitsa::aerodynamics::pressure::evaluate_prepared_mesh(&PreparedSurfacePatches, PressureFlow)` evaluates each prepared physical patch through `PressurePatch` and returns the checked sum of world-space force and COM moment. It allocates nothing, mutates nothing and returns an error (not a partial wrench) if any patch evaluation or accumulation fails. A single explicit atmospheric state and sound speed are used for the whole set; no atmosphere sampling or per-owner load distribution occurs. Empty sets return zero after motion and sound-speed validation. The existing patch force convention and Mach >= 5 policy are preserved, without a new pressure closure.

Preparation is direction-dependent: callers must regenerate/select exposure for the current **body-local upstream** direction (pointing toward the oncoming air). Evaluation does not refresh shielding when motion or wind changes. Angular velocity is included in each centroid's point velocity, but single-direction visibility is only an approximation when rotational flow varies across the body. Centroid sampling does not integrate a spatially varying pressure distribution within a triangle. World integration uses this same evaluator for Mesh and supported Colliders, as described above.

The evaluator does not accept unfiltered mesh triangles as a complete ship model. Caller-selected instances must use consistent outward winding, and open-mesh policy remains explicit. The patch preparation is deliberately separate from `PreparedGeometry` voxel normals: those staircase normals would introduce a persistent orientation bias. This module is a pressure component, not complete FAR-like aerodynamics. World integration is limited to the supported geometry models above; atmosphere defaults and solver behavior are unchanged.

## Lift and finite wings

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

## Numerical validation requirements

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
