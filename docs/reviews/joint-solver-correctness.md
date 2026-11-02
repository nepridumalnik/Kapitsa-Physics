# Joint solver correctness audit

Baseline: `f9e0d17b1a1c4370e2bfd0d360fb7f5749935d09` on `audit/physics-remediation`. This document records static findings; this pass does not execute tests, benchmarks or simulations.

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
