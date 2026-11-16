# Agent Engineering Guidelines

These instructions apply to **the entire Kapitsa Physics repository**, including all crates, examples, tests, benchmarks, scripts, documentation, and tooling. Follow existing public contracts and repository conventions unless an explicit task requires a change.

## Priority and decision order

1. **Correctness first:** physical validity, mathematical and numerical correctness, safety, and predictable error handling.
2. **Clarity and maintainability:** straightforward code, explicit invariants, and limited responsibilities.
3. **Algorithmic efficiency:** appropriate time and space complexity, bounded work, and suitable data structures.
4. **Runtime performance:** reduce allocations, copies, contention, and overhead only without compromising the above.

Do not conceal a correctness defect behind a fallback, tolerance, optimization, or passing regression test. If requirements conflict, preserve correctness and document the trade-off.

## Design principles

- Apply **DRY, KISS, and YAGNI**. Reuse real shared behavior, prefer the simplest correct design, and avoid speculative functionality.
- Prefer small, cohesive modules and explicit data flow. Introduce abstractions only when they simplify current code or enforce an important invariant; favor zero-cost abstractions where appropriate.
- Follow Rust idioms and existing naming, module, ownership, and error conventions. Avoid unnecessary traits, indirection, dynamic dispatch, heap ownership, or generic parameters.
- Keep core physics independent of runtime scheduling, networking, UI, and application concerns. `kapitsa-core` owns physics; `kapitsa-math` owns mathematical primitives; `kapitsa` exposes the public simulation/runtime interface.
- Preserve API compatibility and deterministic behavior unless the task explicitly authorizes a breaking change. Do not silently change units, coordinate frames, sign conventions, or numerical domains.
- Avoid unrelated cleanup and broad rewrites. Refactor the smallest coherent area needed for the requested behavior.

## Size guidelines — refactoring signals, not hard limits

- **900 lines per file** is a review threshold for human-maintained source, tests, scripts, and documentation.
- **450 lines per function or method** is an upper review threshold, not a target. Most functions should be substantially shorter.
- Exceeding a threshold is a **code smell**, not an automatic failure or a reason to reject an otherwise correct change. Explain why the larger unit is cohesive, or split it along meaningful responsibilities when doing so improves the design.
- Do not create trivial wrapper functions, artificial modules, or extra layers merely to satisfy line counts. Avoid scattering one algorithm so widely that its invariants become harder to verify.
- Existing oversized files are not grounds for unrelated mass refactoring. Improve them opportunistically when modifying their relevant responsibilities. Generated content, lockfiles, and machine-generated fixtures are exempt.

## Rust correctness and error handling

- **Do not introduce `.unwrap()` or `.expect()`**, including in new tests, examples, and scripts written in Rust. Prefer `?`, `Result`, `Option`, `ok_or_else`, `if let`, `let ... else`, and explicit assertions in tests.
- Do not use `panic!`, `todo!`, `unimplemented!`, or `unreachable!` as normal production error handling. Assertions may be used in tests; internal debug assertions must not be the sole validation of external input.
- Validate external input and configuration at their boundaries. Reject invalid, non-finite, negative, degenerate, out-of-domain, or otherwise unsupported states explicitly where applicable.
- Preserve error causes and context. Do not silently turn invalid input, numerical failure, or an unsupported physical regime into zero force, a default value, or a different model.
- Prefer checked arithmetic when values can overflow, underflow in a significant way, or become non-finite. Distinguish mathematical invariants from assumptions about ordinary-sized inputs.
- Maintain consistent state on failure. Validate before mutation where practical; use transactional preparation/commit patterns for operations that must not partially apply.
- Use `unsafe` only when necessary, with documented safety invariants, focused tests, and a demonstrated benefit; safe Rust is the default.

## Constants, units, and numerical algorithms

- Give domain-specific coefficients, tolerances, limits, and tuning parameters **descriptive names**. Prefer `const` for compile-time values; use configuration for quantities that genuinely vary at runtime.
- Avoid unexplained magic numbers. Simple structural numbers (`0`, `1`, `2`), indices, and standard mathematical identities need not become constants just to satisfy a rule.
- Document units, coordinate frames, reference areas, orientations, sign conventions, and valid parameter ranges at public interfaces and wherever ambiguity could cause a bug. Use SI units unless a documented API contract states otherwise.
- For physical constants, empirical coefficients, and correlations, record provenance and applicability. Do not fabricate measured values, validation thresholds, or scientific references.
- Distinguish **implemented equations**, **assumed/empirical models**, **numerical approximations**, and **independent experimental validation**. Analytical consistency tests alone are not physical validation.
- Examine stability, conditioning, discretization error, conservation laws, symmetry, limiting behavior, and timestep/grid convergence where relevant. Do not claim accuracy beyond tested conditions.
- Use numerical tolerances justified by scale, units, and error sources. Never adjust tolerances merely to silence a failing test without diagnosing the cause.
- Keep unsupported Mach, Reynolds, geometry, and other physical domains explicit. Do not extrapolate, clamp, or blend models without a justified and documented policy.

## Algorithmic complexity and resource control

- Evaluate **time and space complexity** when designing or materially changing an algorithm; account for worst-case input sizes, not only typical workloads.
- Avoid unnecessary quadratic or higher complexity, repeated full scans, nested searches, and repeated sorting. Use indexing, spatial partitioning, caching, or better algorithms when warranted by actual access patterns.
- A higher-complexity algorithm is acceptable when the domain is bounded and its simplicity or numerical correctness justifies it. State the relevant bound and rationale.
- Bound work and memory for potentially large or untrusted inputs. Prefer explicit errors to partial calculations when a resource budget is exhausted.
- Consider cache locality, contiguous storage, data-oriented iteration, and parallel scheduling, but do not obscure ownership or invariants for presumed speed.
- Do not add parallelism to a hot path without considering synchronization, determinism, work granularity, and measured overhead.

## Allocation, copying, and hot paths

- **Aim for zero heap allocations in steady-state hot paths** such as physics integration, solver iterations, and per-body/per-contact evaluation. Prepare and reserve storage outside these paths; reuse buffers and scratch memory across steps.
- Avoid unnecessary `clone()`, `to_vec()`, intermediate collections, heap boxing, and repeated conversions. Prefer borrowing, slices, iterators where efficient, and ownership moves.
- **Small `Copy` values may be passed or returned by value** when that is as efficient as or better than borrowing. Do not force references or introduce indirection simply to eliminate copies.
- Allocations during initialization, geometry preparation, caching, or infrequent configuration changes are acceptable when justified. Do not sacrifice correctness or clarity to force zero allocations everywhere.
- Measure performance-sensitive changes under representative workloads; record before/after results and verify equivalent numerical behavior. Avoid unverified performance claims or premature micro-optimization.

## Testing and acceptance

- Add focused tests for new behavior and regressions. Cover valid cases, failures, boundary conditions, extreme but representable values, and configuration lifecycle changes.
- For physics and math, test relevant invariants: force and torque directions, energy/momentum behavior where applicable, symmetry, frame covariance, deterministic results, and numerical convergence.
- For multithreaded and asynchronous paths, test ordering, ownership, cancellation, partial failures, state publication, and consistency with single-threaded execution where required.
- Use independent analytical or experimental references when claiming physical accuracy. Clearly label synthetic fixtures and research diagnostics; never present them as experimental validation.
- Run the smallest relevant checks first; expand verification when the change affects shared contracts. Report what was actually run, what failed, and what remains unverified. **Never claim tests passed unless they were executed.**
- Do not change the intended physics, weaken assertions, or hide errors solely to make a test pass.

## Documentation and research

- Update relevant API comments and documentation when changing behavior, assumptions, units, compatibility, or supported domains.
- Keep architectural information in `README.md` and relevant `docs/` files; use `docs/aerodynamics.md` for aerodynamic assumptions and model limitations.
- For science-based changes, cite the actual source, equation or section, and applicability. When a `SOURCES.md` registry exists, keep its entries consistent with the implementation.
- Treat reference implementations, including third-party physics engines, as references only. Respect licenses; do not copy incompatible implementation code into this MIT-licensed repository.
- Separate research prototypes and diagnostics from production algorithms. Moving a numerical experiment into Rust does not by itself make it a validated solver.

## Agent workflow and Git

1. Inspect the relevant code, contracts, existing tests, and working-tree state before editing. Do not assume a planned feature is already implemented.
2. Break work into **small, atomic, independently understandable steps**, each with a clear expected result.
3. Implement the smallest correct change. Add or adapt focused tests and documentation as part of that step when needed.
4. Review the diff for unrelated edits, accidental allocations/copies, changed complexity, unsupported assumptions, and error-path regressions.
5. **Commit each completed atomic step separately** with a descriptive commit message. Do not combine unrelated changes or rewrite unrelated history.
6. **Do not push**, merge, or create releases unless explicitly instructed. Do not discard or overwrite pre-existing user changes.
7. Summarize completed changes, commit hashes, actual verification, remaining limitations, and any unresolved blockers without overstating readiness.

The objective is **correct, auditable, maintainable, efficient physics code**—in that order. A larger clear function, a justified allocation, or a cheap `Copy` is preferable to a superficially optimized implementation that is harder to prove correct.
