# FixedJoint convergence diagnostics

Baseline: `feat/collisions`, HEAD `9e6ce59`; sequential solver, dt 1/120 s. These are diagnostic physics runs, not performance benchmarks. Reproduce the full-rocket traces with:

```powershell
$env:KAPITSA_ROCKET_TRACE_JOINTS='1'
cargo test -p kapitsa --release --features bench-internals --lib diagnose_rocket_first_fixed_joint_divergence -- --ignored --nocapture --test-threads=1
cargo test -p kapitsa-core --release --features bench-internals --lib diagnose_equal_mass_graph_against_analytic_projection -- --ignored --nocapture --test-threads=1
```

The first test uses the actual 1000-part fixture and preserves both Scalar and FixedBlock3 traces. The 3 cm anchor criterion first fails at zero-based tick 12 for both. FixedBlock3 already has 0.304 mm error at tick 1, first trunk joint dense index 0 next to the lower root, 19.56 mm at tick 10, and 22.74 mm at tick 11, while supported by contact and before thrust starts. At tick 12, the first threshold crossing is 38.35 mm on joint dense index 499 (upper-stage first trunk joint), frame 0.0200 rad. Before velocity solving, maximum anchor error is 175.6 mm; after the position sweeps it is 38.35 mm. First/final velocity residuals are 11.62/2.033 m/s and 7.426/0.609 rad/s. The maxima can refer to different rows. Maximum over 180 ticks is 2.498 m, RMS 0.1415 m, maximum frame 3.141 rad, RMS 0.4117 rad. The stage-separation linear-momentum delta is 5.68e-14 in the fixture's units.

For 1000 bodies and FixedBlock3 with 8 velocity/3 position sweeps, the isolated modes give maximum anchor errors: no load 3.74e-15 m, gravity without contact 1.07e-6 m, gravity with contact 0.342 m, thrust without gravity/contact 2.403 m, and full scenario 2.498 m. Gravity with contact first crosses 3 cm at tick 14; thrust without contact at tick 13. These runs distinguish load cases but do not isolate a unique mathematical defect. Cache on/off produce the same reported 180-tick extrema and threshold ticks to the printed precision.

The independent velocity oracle uses equal unit masses at coincident anchors, initial velocity 1 m/s on one body and zero on the others, no contacts or external forces. Exact joint-compatible velocity is 1/N m/s on each body; exact linear momentum is 1 kg m/s. It tests chain, binary tree, and star graphs for N=2,10,50,100,500,1000 with Scalar and FixedBlock3 at 8,16,32 velocity sweeps. On a 1000-body chain, FixedBlock3 velocity error versus the oracle is 0.19538, 0.13895, 0.09835 m/s for 8,16,32 sweeps, respectively; initial/final residual at 8 is 0.25/0.02698 m/s. Momentum error is at most 1.11e-16. At 8 sweeps a 1000-body tree has maximum error 0.01088 m/s and a 1000-body star 0.10374 m/s. This establishes slow, topology-sensitive propagation for this input; it does not establish that no additional implementation error exists.

The experimental global velocity PCG with IC(0) on the same 1000-body chain reaches 1.78e-15 m/s error and reports 8 converged calls, 0 iteration-limit calls, 1 total inner iteration. The 1000-body tree reaches 1.11e-4 m/s, 24 total inner iterations and 8 converged calls. The 1000-body star PCG run was interrupted because the high-degree coupled system was too costly; it is skipped in the test. A 500-body star did converge, but this is not evidence that the algorithm is ready for production.

The free asymmetric 100-body fixture at dt=1/120 s for 180 steps reports kinetic energy change -1.046% with Scalar, -0.0172% with diagnostic global PCG. The recorded velocity impulse work plus quadratic work agrees with measured kinetic-energy change to 3.2e-12 J for Scalar and 1.1e-12 J for PCG. Energy is not expected to remain unchanged through an inelastic velocity projection; these figures are a diagnostic, not a general conservation proof.

Existing finite-difference tests cover fixed rotation and offset-COM point Jacobians, block effective mass, in-place vs generic rows, and mass-cache comparisons. They do not prove correctness for every loaded graph, contact, topology change or force history. Per-sweep residuals beyond first/final and a production-quality fix remain outstanding. Do not enable diagnostic PCG or experimental coloring solely based on these results.


The full FixedBlock3 1000-body sweep matrix (`diagnose_fixed_block_rocket_sweep_matrix`, 180 ticks) produced the following maximum anchor errors in meters. These runs change iteration counts only for diagnosis:

| Velocity sweeps | Position 3 | Position 6 | Position 12 |
| ---: | ---: | ---: | ---: |
| 8 | 2.4983 | 1.0406 | 0.5411 |
| 16 | 1.0691 | 0.6309 | 0.3438 |
| 32 | 0.6844 | 0.5407 | 0.2533 |

All nine runs cross the 3 cm anchor limit by tick 12 or 13; none meets the strict threshold. At 32/12, maximum frame error is 1.418 rad and RMS anchor error is 0.0190 m. This does not justify changing the default sweeps.

A diagnostic error was found in the pre-existing `CenteredThrustWithoutGravityOrContact` mode: it used local point `(0,0,0)` even though the root COM is `(-0.05,0,0)`. `centered_diagnostic_thrust_uses_root_center_of_mass` failed before the fix and passes after it. Earlier centered-thrust measurements are invalid. With the corrected diagnostic and FixedBlock3 on 1000 bodies, centered thrust without gravity/contact gives first 3 cm crossing at tick 13, maximum/RMS anchor 2.3949/0.1557 m, maximum/RMS frame 3.1409/0.4373 rad, normalized coast drift 0.4787. Original offset thrust gives tick 13, 2.4033/0.1568 m, 3.1415/0.4059 rad and normalized coast drift 0.4694. Initial state and force magnitude are the same, but these are distinct physical load locations.


Additional experiments on the full 1000-part, 180-tick rocket:

- Diagnostic velocity+position PCG, IC(0) velocity preconditioner, at the original 8/3 outer sweeps and inner caps 5/5, produced max/RMS anchor 0.033844/0.002273 m, max/RMS frame 0.010363/0.001348 rad and first 3 cm crossing at tick 30. The velocity PCG reported 1094 converged and 1354 iteration-limit calls, with peak residual 1.587 m/s and 0.566 rad/s. The diagnostic test completing is not a strict stability pass: max anchor is above 0.03 m, and the fixture's maximum altitude 1.451 m is below its 1.6 m scenario criterion.
- FixedBlock3 with existing experimental warm starting enabled by `KAPITSA_ROCKET_WARM_START=1` gave max/RMS anchor 2.3549/0.1368 m and frame 3.1416/0.4083 rad, with the same first anchor/frame threshold ticks 12/19. This did not establish an acceptable fix.

Both options are diagnostic-only. Neither changes the production default, and neither satisfies the full-rocket strict criteria in these experiments. The PCG sample above used the existing `diagnose_rocket_global_constraint_pcg` test with `KAPITSA_ROCKET_DIAGNOSTIC_PARTS=1000`, `KAPITSA_ROCKET_VELOCITY_PCG_CAP=5` and `KAPITSA_ROCKET_POSITION_PCG_CAP=5`.


After the diagnostic-only changes, the strict 300-step sequential FixedBlock3 scenario was rerun: it completed 300 ticks with no step error, 998 joints after separation and zero break events, but FAILED the 0.03 m maximum / 0.01 m RMS anchor criteria. Measured anchor max/RMS was 2.6480/0.1686 m, frame max/RMS 3.1415/0.4397 rad, velocity residual max 10.92 m/s / 17.44 rad/s, and stage separation linear-momentum delta 5.68e-14. The test's two runs had identical body states before reaching its strict assertion.


## Controlled integration-order comparison

`loaded_fixed_joint_prediction_precedes_velocity_projection` is an ignored baseline A/B diagnostic, not a production step. It compares one step of equal unit masses in chains and binary trees of 2, 10, 50, 100 and 1000 bodies, with hard FixedJoints and no collisions. A 1200 N force acts along the root connector for dt=1/120 s (root velocity increment 10 m/s, no torque). Both paths use the same force, 8 velocity sweeps and 3 position sweeps. Path A advances the root before velocity projection; path B solves velocity on the original pose, then advances all bodies using the solved velocities. The A result is checked against a real `PhysicsWorld::step` for Scalar. It is deliberately a restricted linear experiment; it does not model CCD, rotating inertia or contact generation.

For the 1000-body chain, A/B maximum anchor error before position is 0.083333/0.002248 m and after position 0.006510/0.001624 m. For the 1000-body tree, A/B after position is 0.003418/0.000154 m. In both orders the total linear momentum is 10 kg m/s and COM acceleration is the analytical 1.2 m/s?. The velocities remain underconverged: maximum residual for the 1000-body tree after 8 sweeps is 0.04870 m/s in both orders. Thus late integration removes a confirmed source of initial displacement in this restricted input but does not fix iterative velocity convergence.

`loaded_fixed_joint_chain_stays_close_after_root_force` is an ignored known-failing regression gate for a real 10-body `PhysicsWorld` step: actual maximum anchor error 0.006510 m exceeds the 0.003 m criterion, which is 0.3% of the one-meter link length; exact hard-constraint error is zero. It must not be marked as passing until the physical correction is implemented and verified.

The requested Jolt reference revision describes velocity constraint solving followed by integration and CCD in [Architecture.md](https://github.com/jrouwe/JoltPhysics/blob/0f69a6b8f0bd562f23a29b27a665d2dab1524917/Docs/Architecture.md#the-simulation-step-in-detail). Jolt's FixedConstraint uses separately prepared point and rotation constraint parts and recalculates them for position solving. This comparison is not sufficient to establish equivalence with Kapitsa's contact or CCD pipeline.


The complementary `diagnose_loaded_graph_integration_order_trajectory` runs the same controlled axial input for 180 steps at 1/120 s, 1200 N every step, 8/3 sweeps, chain and binary-tree graphs of 2/10/50/100/1000 unit-mass bodies. Scalar and FixedBlock3 are both exercised. The A path's final body poses and velocities are checked against a separate full `PhysicsWorld::step` simulation for every graph and size. B is a restricted test contour without contacts, CCD or angular motion; it is not a production implementation.

For 1000 bodies, FixedBlock3 chain A/B maximum and RMS anchor drift across 180 steps were 0.57965/0.06315 m and 0.57077/0.06188 m, respectively; first 0.03 m threshold ticks were 8 and 11. Tree A/B was 0.15614/0.00487 m and 0.15793/0.00482 m, first crossing ticks 61 and 59. Peak velocity residuals were identical within each A/B pair: 1.0700 m/s chain, 0.1700 m/s tree. Frame error is zero in this axis-aligned experiment. Both paths conserve linear momentum 1800 kg m/s, yielding the analytical COM acceleration 1.2 m/s?; final kinetic energies are 26073 J chain and 1657 J tree. These results establish that fixing pose order alone is insufficient for the 180-step graph; they do not quantify the effect on the full rocket with contacts and torque.

The strict 180-step full-rocket test `fixed_step_rockets_report_stability_and_reproducibility_at_all_sizes` was rerun at baseline with Scalar, no report-only option. It completes all scenarios but FAILS numerical stability for all measured sizes. Maximum/RMS anchor errors for 100, 200, 500, 1000 bodies are 0.1334/0.0111, 0.2907/0.0280, 0.7411/0.0608, 2.5072/0.1439 m respectively; maximum frame errors 1.569, 1.545, 2.799, 3.141 rad. First anchor threshold ticks: 15, 14, 12, 12. This is a failing baseline, not evidence of an integrated-pose-order fix.
