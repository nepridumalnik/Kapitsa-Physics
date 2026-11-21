# Voxel aerodynamics acceptance status

This is a starting map against the supplied production plan, not an acceptance
report. A checked build or an analytic-model test is not independent physical
validation. FAR is a behavioral reference only; no GPL source is incorporated.
The per-model domains in `docs/aerodynamics.md` are authoritative and have
advanced since this map was written, notably for high-speed skin friction,
rarefied particle drag, subsonic Mesh/Colliders compression pressure, a
geometry-derived predictive hull transonic drag-rise estimate, a bounded
linearized supersonic finite-wing lift/wave-drag estimate and a bounded
incompressible potential-flow panel model for closed isolated meshes
(including quasi-steady rigid rotation), a Kutta lifting-line bridge, a
bounded 3×6 horseshoe vortex-lattice wake for closed thin wing meshes and a
caller-blended flat-plate post-stall wing extension, a `CL_max`/geometry
stall-onset estimate, an Øye/Bergami rate-dependent separation state, a
bounded empirical turbulent-base pressure add-on for closed primary
Mesh/Colliders bodies and a fully rough Prandtl–Schlichting friction option;
the acceptance gaps below remain open.

| Plan | Available evidence | Remaining acceptance work |
| --- | --- | --- |
| 1.1 Predictive transonic hull | Caller-supplied Mach/normal-force tables; geometry-derived Korn critical Mach with Lock drag rise and a bounded Mach 1.05–1.2 handoff; interpolation and world tests. Supersonic slender wave primitive. | Validate the equivalent-section estimate against independent body-of-revolution pressure/drag data, add lift/sweep effects, establish the supersonic handoff, and compare Sears–Haack and blunt-body curves with independent reference data. Tables and the Korn/Lock correlation are not a validated TSD solver. |
| 1.2 Subsonic Mesh/Colliders Cp | Default unmodified Newtonian requires Mach >= 5. Opt-in modified Newtonian compression covers Mach 0–5 with isentropic/Rayleigh-pitot `Cp_max`. An opt-in incompressible source-panel model covers closed isolated Mesh/Colliders bodies at Mach ≤ 0.2 and ≤ 256 triangles. A Kutta lifting-line bridge and a 3×6 horseshoe vortex-lattice wake cover closed thin wing meshes (`span ≥ chord`, `thickness ≤ 0.15 span`, Mach ≤ 0.3), with local leading/trailing edges for sweep/taper, sampled camber and world tests. Explicit base Cp supports selected faces and visibility checks. An opt-in empirical base-pressure add-on covers closed primary Mesh/Colliders at Mach 1–5, holds the Love/MIL-HDBK-762 correlations at Mach 2.5 between Mach 1 and 2.5, and applies Fig. 5-141 relief from a caller base-area ratio. | The panel model has no general wake/circulation; its rotation is quasi-steady and omits unsteady added mass and rotating-wake effects. The vortex lattice is only 3×6, assumes a thin isolated wing, and omits finite thickness, separation, wake roll-up, control surfaces and experimental validation. The empirical base model does not resolve subsonic/transonic base physics, unsteady wake or vortex shedding, does not filter internal cavity faces, ignores rotation in base classification and lacks independent validation. Compare with independent pressure/lift data, establish lattice convergence and rotating moments, and add multi-surface wake handling before accepting general subsonic lifting-body Cp. |
| 1.3–1.6 Stall, interference, body lift, failures | Existing polars, geometric shielding, slender-body loads, an Ackeret `4/sqrt(M²−1)` finite-wing estimate with an `AR/(AR+2)` factor, a caller-blended flat-plate post-stall extension, a `CL_max`/geometry lift-curve-intersection stall-onset estimate and an Øye/Bergami rate-dependent separation state with hysteresis cover parts of the infrastructure. | The supersonic wing model lacks thickness, sweep, tip-relief and shock-detachment validation. The stall onset still depends on caller `CL_max` and has no CFD/experimental validation; the dynamic model has no leading-edge vortex or pitching-moment state. Predictive stall/spin demo, downwash interaction, non-slender crossflow validation and aerodynamic weld failure must be assessed; none is accepted here. |
| Friction/roughness | Incompressible, transitioned, compressible and high-speed flat-plate friction; an opt-in fully rough Prandtl–Schlichting mean coefficient for `100 ≤ L/ks ≤ 1e6` at Mach ≤ 0.3. | The rough model does not cover the transitionally rough regime, pressure gradients, shock-boundary-layer interaction, real-gas properties or independent experimental validation. |
| 2.1–2.5 Existing-model extensions | Some related features already exist; see `docs/aerodynamics.md` for their individual domains. | Audit each requested extension against current code and tests before declaring completion. |
| 2.6–2.8 Performance, runtime and tools | Benchmarks and runtime controls exist. | Production-complexity benchmark and remaining APIs/tools are not established; the supplied plan identifies these as post-merge roadmap. |
| 3 Fixes | Some independent checks have passed. | Entire section deferred by explicit user direction; do not represent it as completed. |
| 4.1–4.8 Refactoring | Existing code has model modules and partial context grouping. | Audit and implement each requested decomposition, common grid, preparation, composition, errors, context, fixtures and conventions; none accepted here. |
| 5.1 Verification/CI | Full workspace tests, Clippy `-D warnings` over all targets, formatting, rustdoc `-D warnings`, the declared MSRV 1.95 check and benchmark compilation passed at the stabilization head; final scalar aero and wing-shielding measurements are recorded in `benchmarks/aerodynamics.md`. `.github/workflows/ci.yml` defines the corresponding hosted checks (formatting, Clippy, Linux/Windows tests, MSRV check, rustdoc, benchmark compilation, public API snapshot). | Confirm the first hosted CI run and all remaining acceptance criteria; the physical-validation gaps above stay open and section 3 remains deferred. Historical commit messages cannot be rewritten without authorization. |
| 5.2–5.5 Benchmarks, physical report, release, merge | Scalar baseline and analytical regression tests exist. | Production benchmark scenarios and thresholds, independent validation/FAR-parity report, release migration materials, review and authorized merge remain open. |

The production plan includes P1 work beyond the P0 closures. Sections 1.7–1.9
and 2.6–2.8 are identified there as post-merge roadmap. The agreed post-merge
aerodynamics roadmap (shared prepared shapes, asynchronous cooking, runtime
control surfaces) is tracked in `openspec/roadmap/aerodynamics.md` and its
planned changes; it is infrastructure and control work, not new physical
models, and it does not close the validation gaps above. This document does not
change those priorities, grant merge authorization, or imply stage 3 is complete.
