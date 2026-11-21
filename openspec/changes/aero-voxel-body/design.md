# Design

## Context

FAR is a reference for gameplay outcomes: vehicle geometry, shielding, and assembly changes should visibly affect flight in the simulator. Voxelization supplies geometry, not a fluid solution. Existing `CookedShape` stores occupancy, sections and volume; shape bindings map owner slots to parts. Existing pressure evaluators and wing polars have their own domains.

## Goals / Non-Goals

- Deliver a bounded hull load model that works for a single body and its connected assembly without changing existing default models.
- Keep skin friction, base/wake drag prediction, transonic wave drag, and automatic selection out of the first hull closure. Existing separate models keep their documented behavior; no promise of a continuous atmosphere-to-ground trajectory under one closure.

## Decisions

1. Reuse `PreparedGeometry`, `CookedShape`, `ShapeRegistry`, and background preparation. Evaluate the union in the current body frame; retain deterministic ownership on exposed patches. Do not sum individual part hull evaluations. Treat unresolved openings, thin walls, coincident faces, and resolution-dependent features as explicit limitations; select supported fixtures and geometry budgets before enabling the path.
2. First integrate an existing explicit impact-pressure closure, with modified Newtonian as the initial candidate. Its current API accepts Mach 0–5; that guard is not evidence of accuracy throughout that interval. Before enabling the hull path, document the supported envelope, provenance, assumptions and which demonstrations it supports. Do not select a different closure silently when a local condition falls outside the advertised domain. A broader flight envelope is follow-up model work, not a tuning task.
3. Stage hull loads using local body-relative flow at each exposed patch, including angular motion about the current COM. Use the existing COM atmosphere convention for the hull; separately modeled surfaces keep their documented sampling convention. Sum each force and its lever-arm moment once; owner diagnostics use consistent frames and must reconstruct the body wrench when shifted to the same reference. Visibility under rotational flow is approximate; characterize angular and voxel resolution sensitivity rather than implying exact local flow.
4. Separate shielding geometry from load ownership. Geometry belonging to an explicit `WingSurface` may occlude other surfaces, but any physical surface represented by the wing model must not also produce hull pressure. Require an explicit association when a wing has corresponding hull geometry; reject ambiguous overlap configurations rather than guessing from proximity. Preserve intentional independent hull and wing contributions and test baseline compatibility.
5. Bind loads to the geometry revision and owner mapping used to stage them. While replacement cooks, reuse a previous shape only if the existing pending policy permits it and membership, owner mapping and reference frame are still valid. Otherwise use the existing explicit pending behavior without synchronous voxelization in the step. Stale results never rebind old topology. Share validation/staging across step paths and preserve typed failures and transactional step semantics. Use bounded per-worker scratch; measure geometry preparation separately from repeated evaluation.

## Risks / Trade-offs

- Stable flight, instability, oscillation and tumbling are outcomes of specific fixtures, not requirements that every vehicle exhibit them. Use repeatable simulated craft cases and record why each outcome is expected; do not require gameplay phases that exceed the selected model envelope.
- Interior suppression and symmetry can depend on voxel scale. Record mesh/voxel resolution, timestep, force/moment error and convergence before calibrating demo assets. Analytical and synthetic references validate implementation behavior, not agreement with flight data.
