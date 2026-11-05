# FixedJoint convergence diagnostics

Baseline: `feat/collisions`, HEAD `9e6ce59`; sequential solver, dt 1/120 s. These are diagnostic physics runs, not performance benchmarks. Reproduce the full-rocket traces with:

```powershell
$env:KAPITSA_ROCKET_TRACE_JOINTS='1'
cargo test -p kapitsa --release --features bench-internals --lib diagnose_rocket_first_fixed_joint_divergence -- --ignored --nocapture --test-threads=1
cargo test -p kapitsa-core --release --features bench-internals --lib diagnose_equal_mass_graph_against_analytic_projection -- --ignored --nocapture --test-threads=1
```

The first test uses the actual 1000-part fixture and preserves both Scalar and FixedBlock3 traces. The 3 cm anchor criterion first fails at zero-based tick 12 for both. FixedBlock3 already has 0.304 mm error at tick 1, 19.56 mm at tick 10, and 22.74 mm at tick 11, while supported by contact and before thrust starts. At tick 12, the first threshold crossing is 38.35 mm on joint dense index 499 (upper-stage first trunk joint), frame 0.0200 rad. Before velocity solving, maximum anchor error is 175.6 mm; after the position sweeps it is 38.35 mm. First/final velocity residuals are 11.62/2.033 m/s and 7.426/0.609 rad/s. The maxima can refer to different rows. Maximum over 180 ticks is 2.498 m, RMS 0.1415 m, maximum frame 3.141 rad, RMS 0.4117 rad. The stage-separation linear-momentum delta is 5.68e-14 in the fixture's units.

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
