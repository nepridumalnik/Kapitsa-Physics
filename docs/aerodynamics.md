# Aerodynamics: physical references and implementation boundaries

Kapitsa's geometry-based aerodynamics is an independent implementation. FAR is a behavioral and architectural reference, not a source of MIT-licensed implementation code. Voxelization represents geometry; it does not solve the fluid flow or determine aerodynamic coefficients by itself.

This document distinguishes physical equations from numerical approximations. Inclusion of a formula below does **not** mean that the corresponding model is implemented. Implementation status must be established from code and tests.

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
