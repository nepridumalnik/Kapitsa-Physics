# Voxel aerodynamics acceptance status

This is a starting map against the supplied production plan, not an acceptance
report. A checked build or an analytic-model test is not independent physical
validation. FAR is a behavioral reference only; no GPL source is incorporated.
The per-model domains in `docs/aerodynamics.md` are authoritative and have
advanced since this map was written, notably for high-speed skin friction,
rarefied particle drag, subsonic Mesh/Colliders compression pressure, a
geometry-derived predictive hull transonic drag-rise estimate and a bounded
linearized supersonic finite-wing lift/wave-drag estimate; the acceptance gaps
below remain open.

| Plan | Available evidence | Remaining acceptance work |
| --- | --- | --- |
| 1.1 Predictive transonic hull | Caller-supplied Mach/normal-force tables; geometry-derived Korn critical Mach with Lock drag rise and a bounded Mach 1.05–1.2 handoff; interpolation and world tests. Supersonic slender wave primitive. | Validate the equivalent-section estimate against independent body-of-revolution pressure/drag data, add lift/sweep effects, establish the supersonic handoff, and compare Sears–Haack and blunt-body curves with independent reference data. Tables and the Korn/Lock correlation are not a validated TSD solver. |
| 1.2 Subsonic Mesh/Colliders Cp | Default unmodified Newtonian requires Mach >= 5. Opt-in modified Newtonian compression covers Mach 0–5 with isentropic/Rayleigh-pitot `Cp_max`; tests cover oblique/leeward patches, prepared and owner aggregation, mesh and collider world steps, atomic domain failure. Explicit base Cp supports selected faces and visibility checks. | The modified model is compression-only: no attached suction, potential-flow lift, wake or base physics. Compare it with independent sphere/cylinder pressure data at matched conditions and establish mesh convergence and rotating moments. This does not accept a subsonic panel solution. |
| 1.3–1.6 Stall, interference, body lift, failures | Existing polars, geometric shielding, slender-body loads and an Ackeret `4/sqrt(M²−1)` finite-wing estimate with an `AR/(AR+2)` factor cover parts of the underlying infrastructure. | The supersonic wing model lacks thickness, sweep, tip-relief and shock-detachment validation. Predictive stall and spin demo, downwash interaction, non-slender crossflow validation and aerodynamic weld failure must be assessed against their individual criteria; none is accepted here. |
| 2.1–2.5 Existing-model extensions | Some related features already exist; see `docs/aerodynamics.md` for their individual domains. | Audit each requested extension against current code and tests before declaring completion. |
| 2.6–2.8 Performance, runtime and tools | Benchmarks and runtime controls exist. | Production-complexity benchmark and remaining APIs/tools are not established; the supplied plan identifies these as post-merge roadmap. |
| 3 Fixes | Some independent checks have passed. | Entire section deferred by explicit user direction; do not represent it as completed. |
| 4.1–4.8 Refactoring | Existing code has model modules and partial context grouping. | Audit and implement each requested decomposition, common grid, preparation, composition, errors, context, fixtures and conventions; none accepted here. |
| 5.1 Verification/CI | Workspace tests, Clippy and formatting passed locally; rustdoc and scalar aero benchmark ran locally. `.github/workflows/ci.yml` defines corresponding CI checks. | Confirm hosted CI, remaining verification markers and all acceptance criteria; section 3 remains deferred. Historical commit messages cannot be rewritten without authorization. |
| 5.2–5.5 Benchmarks, physical report, release, merge | Scalar baseline and analytical regression tests exist. | Production benchmark scenarios and thresholds, independent validation/FAR-parity report, release migration materials, review and authorized merge remain open. |

The production plan includes P1 work beyond the P0 closures. Sections 1.7–1.9
and 2.6–2.8 are identified there as post-merge roadmap. This document does not
change those priorities, grant merge authorization, or imply stage 3 is complete.
