# Tasks

## 1. Model and acceptance envelope

- [x] 1.1 Audit existing geometry, pressure, wave and wing paths; record which components are reused and which load terms the first hull model excludes in `docs/aerodynamics.md`.
- [x] 1.2 Cite the chosen closure and establish a supported flow and geometry envelope before enabling it; record limits, fixture scales, reference equations, numerical tolerances and bounded geometry work in the acceptance review. Do not treat the evaluator's input guard as validation.

## 2. Geometry and wrench evaluation

- [x] 2.1 Evaluate exposed union geometry and owner mapping for individual bodies and assemblies; test an enclosed part, equivalent representations and resolution sensitivity at supported openings.
- [x] 2.2 Stage distributed force and COM moment with local relative velocity; test vacuum, symmetry, off-center loads, rotating geometry, frame covariance and owner-wrench reconstruction against independent references.
- [x] 2.3 Add explicit surface association separating shielding and load attribution; test shared geometry, independent wings, ambiguity and control deflection without duplicate load.

## 3. World integration and cost

- [x] 3.1 Bind staged loads to geometry revision and topology; test pending policies, stale results, removal, merge and split while cooking, with no synchronous cooking inside the step.
- [x] 3.2 Verify ordinary/sharded parity and step rollback for unsupported local conditions and exhausted geometry budgets; document errors and verify legacy model behavior plus public API baselines.
- [x] 3.3 Benchmark preparation separately from repeated evaluation; log time, allocation and scratch memory for representative and budget-limit cases in the acceptance review.

## 4. Simulation acceptance

- [x] 4.1 Add reproducible viewer fixtures with and without stabilizers, restricted to supported conditions; record initial state, geometry/model parameters, timestep, expected qualitative outcome and diagnostic traces.
- [x] 4.2 Record geometry and timestep refinement results and identify which launch, maneuver, descent or landing phases remain outside the envelope; distinguish numerical regression, gameplay demonstrations and any independently validated claims.
