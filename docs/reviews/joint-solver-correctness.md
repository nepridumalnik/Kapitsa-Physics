# Joint solver correctness audit

Baseline: `f9e0d17b1a1c4370e2bfd0d360fb7f5749935d09` on `audit/physics-remediation`. The original entries record static findings; dated updates below supersede their statements about runtime validation and current status.

## Pipeline and contracts

`PhysicsWorld` prepares predicted body states and contact/joint data; `CollisionSystem::solve_contacts` partitions them into islands, materializes island bodies and constraints, and calls `SolverWorkspace::solve_with_joints`. Independent islands may be processed by the executor; one island remains sequential. `JointWorkspace` retains rows and multipliers between preparation passes. The legacy options use scalar projection, fixed iterations and no joint warm start. World state, event data and current caches are published only at commit; abort discards staged cache and lifecycle data. Body-local joint frames are composed with body transforms, and linear row lever arms are measured from world COM. Static and kinematic masses do not receive dynamic corrections.

For a fixed connector, the world-space point error is `d = anchor_b - anchor_a`. For a fixed world unit axis `e`, `C = e·d`, with linear Jacobians `(-e,+e)` and angular Jacobians `(-(r_a×e), r_b×e)`. The generalized inverse mass is `K = J M^-1 J^T`; XPBD uses `(K + alpha I) Δλ = -(C + alpha λ)`, `alpha = compliance / dt²`. Constraint impulses have units of momentum; XPBD position multipliers have different units and are not interchangeable. In the fixed position solver, pose correction is separate from velocity impulse solving and velocity bias is disabled for FixedJoint; this is not by itself a proof of energy conservation for the entire step.

## Findings

1. **Confirmed — FixedJoint translational row derivative.** `append_fixed_rows` projects world anchor separation onto axes rotated with frame A, but `push_linear_row` uses Jacobians for a fixed world axis. Away from zero separation, differentiating `e_A·d` adds `(e_A × d)·omega_A`; the existing row omits this term. This is a mathematical inconsistency; it is not proven to be the dominant cause of the rocket errors. A fixed world basis represents the same three-dimensional `d` and matches the existing point Jacobians without the omitted derivative.
2. **Confirmed limitation — angular linearization.** FixedJoint uses a quaternion logarithm in A's frame, then the local components with Jacobian axes rotated by A. Those axes are the small-error approximation to the derivative of the log map, not its exact SO(3) Jacobian away from zero. At π, selecting a shortest quaternion representative is necessarily discontinuous. A safe finite-rotation correction requires a consistent inverse SO(3) Jacobian and validation across branches; the current code must not be presented as exact there.
3. **Confirmed limitation — convergence metric.** Adaptive residual includes compliance and projected limits, but its position component is a constraint-equation residual rather than an independent maximum geometric anchor/frame error; velocity residual does not fully encode all damped and compliant row equations. The path is experimental and is not the production default.
4. **Confirmed limitations — cost and concurrency.** Per-joint row preparation redoes pose-dependent work, and changed row counts use `Vec::splice`; mass cache, Block3 and adaptive iterations are not legacy production defaults. Parallelism is across islands, not independent constraints inside one island. These facts do not prove a specific cost reduction or explain the observed instability.
5. **Unverified hypothesis — rocket instability.** Coupled rows, matrix reuse and island partitioning may affect convergence and cost, but the measured error alone does not establish a single root cause. Mass changes, contact response and fixture dynamics also participate.
6. **Cache lifecycle.** Joint warm-start is disabled by default. Staging and commit/abort are transactional, but matching by joint generation, kind, row key and axis alone does not prove compatibility after changes to mass, local frames or drive configuration. Do not enable it by default without a stronger invalidation contract.

## External references

Jolt Physics `8bc5cdee81aa896038fe878f163ae6125954876d`: `FixedConstraint.cpp`, `PointConstraintPart.h`, `RotationEulerConstraintPart.h`, `CalculateSolverSteps.h`, `LargeIslandSplitter.cpp`, `StaticCompoundShape.cpp`. The point part forms coupled inverse mass from COM lever arms and inverse inertia; rotation and position/velocity passes are separate; impulses can be warm-started. Jolt uses a small-angle rotation approximation in the referenced rotation part. This is a reference for contracts, not a basis for importing its numerical constants.

Chaos release `260bb2e1c5610b31c63a36206eedd289409c5f11`: the requested Gauss-Seidel solver, joint container, joint constraints and coloring graph files were inspected. They distinguish position and velocity phases, update solver/connector state, and provide graph coloring and shock-propagation machinery. These do not establish that directly transplanting their ordering, convergence thresholds or parallel mutation model into Kapitsa is correct.

## Scope and unresolved gates

The 3×3 block solver is present behind non-default controls and retains a scalar fallback; it has not been proven production-ready for every supported joint, angle or degeneracy. Exact nonlinear angular Jacobians, convergence policy, strict cache invalidation, safe intra-island parallelism and rigid assembly lifecycle require further work. No runtime behavior or performance claim follows from a static check alone.

## FixedJoint point basis correction

The three translational rows now project `d` onto world X/Y/Z instead of frame-A axes. The Jacobian remains `(-e, -(r_a × e), e, r_b × e)`; `e` is constant, so it is the derivative of the projected position error. All three components still constrain the same world-space vector, and angular rows retain their existing basis. No dynamic validation has been run; the effect on finite-iteration behavior and the rocket scenario is unknown.

## Block preparation

The experimental Block3 assembly evaluates the six independent entries of the symmetric `J M^-1 J^T` matrix and mirrors them, instead of evaluating nine entries independently. This reduces repeated cross-mass evaluations and avoids numerical asymmetry in the assembled matrix. The solver still checks positive definiteness and retains its existing scalar fallback. This does not establish the conditioning, finite-angle angular Jacobian or production suitability of the block path; no speedup is claimed without measurement.

## Experimental warm-start invalidation

When experimental joint warm-start is enabled, exposing a body through `body_mut` now invalidates cached impulses for its attached joints before the caller can change mass, inertia, pose or velocity. Removing a body invalidates each removed joint's cache; toggling the experimental strategy clears both cache generations. Joint update, disabling and direct removal retain their existing invalidation. The scan is linear in joint count on a mutable body borrow only while warm-start is enabled, so this is a correctness-first mitigation, not a hot-path optimization. Cache generation matching still protects reused slots. Warm-start remains disabled by default and internal pathways, numerical behavior and performance still need dynamic validation.

## Singular twist coordinate

`twist_angle` uses the quaternion's `(z, w)` projection. When both components vanish (a 180-degree swing perpendicular to the twist axis), the decomposition has no unique twist. It now reports `NonConvergent` rather than silently assigning zero. This is an observable change: a step reaching that singularity can fail and roll back. The ordinary angle remains wrapped to `[-π, π)`; multi-turn tracking is not implemented. Behavior near the singularity and all existing scenarios remain dynamically unverified in this pass.

## Remaining phase assessment

| Step | Result in this pass | Remaining work |
|---|---|---|
| 1: invariant audit | Audit and reference comparison recorded. | Rocket root cause remains unproven. |
| 2: geometry/Jacobians | Fixed point rows use world axes; undefined twist is rejected. | SO(3) log Jacobian for finite angles and continuous multi-turn state are not implemented. |
| 3: Fixed Block3 | Existing experimental block matrix uses six independent symmetric entries; checked row extraction no longer unwraps. | Full finite-angle angular block, compliance coverage and production enablement are not validated. |
| 4: position/velocity | Existing split pose/velocity paths and FixedJoint's disabled velocity bias inspected. | Damping, spring, motor, restitution and energy interactions still require a complete derivation and dynamic checks. |
| 5: convergence | Existing KKT handling and residual paths inspected. | Independently measure geometric error and report iteration-limit status; velocity residual must represent the damped/compliant equation actually solved. |
| 6: preparation/mass | Reduced Block3 matrix assembly from nine cross-entry evaluations to six. | Repeated row preparation, cross-iteration factorization and row-key lookup remain; `splice` is still used when row counts change. Mass-cache correctness and performance need validation. |
| 7: warm-start | Invalidate attached-joint cache on mutable body access and joint removal; clear it on strategy toggles. | Scan cost can be linear in all joints per mutation, and internal edits, rollback and physical response need dynamic validation. Warm-start stays disabled by default. |
| 8: large islands | Existing executor and coloring reference inspected. | `StepExecutor` grants disjoint mutable chunks, not arbitrary pairs in a shared body slice. A safe per-color gather/solve/commit design must preserve joint/contact ordering, deterministic results and rollback; it is not implemented. |
| 9: rigid assembly | Applicability and data requirements reviewed. | No compound/cluster shape, member identity, collider remapping or split lifecycle exists in the current `Shape`/world interfaces. Do not replace a set of live bodies and force sources with an incomplete surrogate. |
| 10: integration | Legacy scalar default and transactional commit/abort retained; experimental strategies remain opt-in. | Full Block3, convergence, cache and intra-island strategies have not been integrated into production. |

For a future rigid assembly, mass must be `M = Σm_i`, center `c = Σm_i c_i / M`, and inertia about that center must include each rotated center-of-mass inertia and `m_i (||r_i||²I - r_i r_iᵀ)`. A split must reconstruct each member's velocity `v_i = v_com + ω × r_i`, preserve the intended momentum exchange, remap colliders and force sources, and stage the entire operation atomically. Breakable joints, motors and variable fuel mass prevent an unconditional replacement. These formulas are requirements, not an implementation.

## Energy and velocity projection follow-up (2026-11-03)

- `predict_dynamic()` now uses midpoint gyroscopic acceleration. The free asymmetric rigid-body regression confirms near second-order energy and angular-momentum convergence at `dt=1/120` and `1/240`; this validates a free body's integrator only.
- Rocket coast potential now uses the central field's physical `−mu*m/r` with zero at infinity. Drift is normalized by `|T|+|R|+|U|` at coast start, avoiding arbitrary surface-potential offsets and near-zero total-energy denominators. The previous surface-zero metric overstated drift: the 500-part gravity-only/contact-free run changed from `1.351126%` to `4.758411e−7%` under the physically referenced metric. The strict scalar rocket energy metric is now within threshold at all sizes, while its anchor and frame limits still fail.
- For velocity projection, measured kinetic change agrees with the exact generalized-impulse work identity to below `1.5e−16 J` in an isolated two-body FixedJoint. This is admissible numerical projection work when post-integration `Jv != 0`; it is not evidence by itself of an energy bug. Reducing `dt` halves the aggregate loss over equal simulation time in the single-joint test.
- A 100-body isolated chain distinguishes this projection from scalar under-convergence: initial `max |Jv|=4.77e−4 m/s`; scalar finishes at `7.36e−2 m/s` with `−120.23 J` velocity-solver change, whereas cap-64 PCG finishes at `3.37e−15 m/s` with `−1.977 J`. The applied-work identity matches both. Thus the scalar result has large residual and excess numerical dissipation; PCG removes that residual, but its nonzero work is consistent with the required projection of velocities generated by separately integrated bodies. A constrained integration formulation or a broader energy analysis is still needed before calling the coupled assembly energy defect fixed.
- Position corrections changed kinetic energy by `−5.91e−5 J` in the 100-part scalar assembly, compared with `−1.51e−2 J` during velocity solving. In the gravity-only rocket run, position correction changed potential energy by `−5.72e−5 J` over 180 steps, compared with `−261.79 J` integration total-energy change. These fixtures do not support position correction as the dominant energy source.
- Strict Release scalar geometry remains outside limits at 100/200/500/1,000 parts; measured anchor maxima are `0.1334/0.2907/0.7411/2.5072 m` and frame maxima `1.5689/1.5450/2.7995/3.1412 rad`. A preliminary concurrent cap-64 report showed geometry within limits but is not a valid strict or performance run. A sequential cap-1 sweep has failed at least 100/200/500 parts; cap search and strict confirmation remain open.

## Validation in this pass

After each code change, `cargo +1.98.0 fmt --all -- --check`, `cargo +1.98.0 check --workspace --all-features --locked --offline -j 4`, `cargo +1.98.0 clippy --workspace --all-features --locked --offline -j 4 -- -D warnings`, and `git diff --check` passed. No test, benchmark, or numerical scene was run, and no new test was added. Previously measured rocket errors and timing were produced before this pass and cannot validate these changes. In particular, neither convergence improvement nor a speedup is established.

## Dynamic energy and projection update (2026-11-03)

The dated energy audit supersedes the earlier static-only validation note above.

- A free anisotropic rotor test validates the midpoint gyroscopic update over ten seconds: energy drift is `−4.3135e−4%` at `dt=1/120` and `−1.0779e−4%` at `dt=1/240`; angular-momentum error decreases from `3.50e−5` to `8.36e−6`.
- On a free two-body FixedJoint, post-integration `|Jv|` is `4.5551e−4 m/s` at `dt=1/120` and `2.2780e−4 m/s` at half dt. Over the same simulated time the Velocity Solver energy loss halves from `−8.2531e−6 J` to `−4.1268e−6 J`. The measured change agrees with the applied generalized-impulse work within `1.5e−16 J`.
- On an isolated 100-body fixed chain, Scalar leaves `7.3648e−2 m/s / 1.976e−3 rad/s` velocity residual and loses `120.23 J`; IC(0) PCG 64 reduces it to roundoff and loses `1.977 J`. The correction identity closes to `3.7e−13 J`; this distinguishes admissible projection work from excess loss due to scalar under-convergence.
- Post-position velocity residual remains near the post-velocity value in the tested single joint, chain, and free rocket assembly. Position-stage energy changes are orders smaller than the velocity projection. The split predictor still creates timestep-dependent `Jv` for constrained bodies; a fully coupled constrained midpoint integrator is not implemented.
- Coast-energy diagnostics now use `U=−μm/r`, zero at infinity, normalized by initial `|T|+|R|+|U|`. For the same 500-part full-flight PCG trajectory, the corrected metric is `4.598934067e−7%`; the former surface-zero formula reports `88.94944039%` on identical states. The arbitrary reference nearly cancels the gravitational term and is not a physical drift measure.
- Strict scalar rocket geometry still fails at all four sizes. Benchmark-only PCG caps `5/5` with 16 outer velocity sweeps pass unchanged strict criteria at 100/200/500/1,000 parts, but are `2.7x/3.5x/6.4x/9.0x` slower than the original strict timing and have frequent capped local solves. PCG is not the production default.
- Release diagnostics, full serial workspace tests, all-features check and strict Clippy passed on the final code checkpoint. See [`joint-solver-energy-parallel-results.md`](joint-solver-energy-parallel-results.md) for the full stage ledger, cap screen, and performance table.
