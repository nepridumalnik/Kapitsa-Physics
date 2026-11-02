# Joint solver work log

## 2026-11-02 — baseline and projection comparison

- `6294fdf` records the local scalar-solver baseline at the original remote HEAD. Workspace fmt/check/Clippy passed; workspace tests passed (775 passed, 4 ignored). The strict Release rocket test failed the original error limits for all sizes.
- The strict fixture completed all 180 steps at 100, 200, 500, and 1,000 parts. It exercised gravity, pad contact, offset thrust, fuel mass changes, and stage separation. Separation momentum error stayed at or below `2.629e-13`; no step failed and no joint broke.
- A Release comparison with identical fixture, 3 position sweeps, and 8 velocity sweeps changed only scalar vs `FixedBlock3`. Both completed at all sizes, but neither met strict limits. Block3 lowered maximum and RMS frame error for 200/500/1,000 parts; both frame errors increased at 100 parts. It did not improve maximum anchor error at any size and increased coast energy drift at 500 parts. Keep Block3 experimental until geometry and pipeline correctness are addressed.

| Parts | Projection | Anchor max / RMS, m | Frame max / RMS, rad | Coast energy drift |
|---:|---|---:|---:|---:|
| 100 | Scalar | 0.1538798 / 0.01302640 | 1.333782 / 0.08493318 | 4.1225% |
| 100 | FixedBlock3 | 0.2213087 / 0.01416238 | 1.442233 / 0.08996116 | 3.2120% |
| 200 | Scalar | 0.2846846 / 0.02980872 | 1.565341 / 0.1226705 | 69.9332% |
| 200 | FixedBlock3 | 0.2883405 / 0.02782598 | 1.201834 / 0.09864404 | 60.8098% |
| 500 | Scalar | 0.6644705 / 0.06259963 | 3.141284 / 0.3265347 | 59.3510% |
| 500 | FixedBlock3 | 0.7414070 / 0.06269059 | 3.140986 / 0.3017271 | 62.0578% |
| 1,000 | Scalar | 2.814815 / 0.1660589 | 3.141479 / 0.6627714 | 44.4683% |
| 1,000 | FixedBlock3 | 2.866444 / 0.1481344 | 3.141102 / 0.4548406 | 44.1231% |

The near-π relative-frame errors and the known small-angle angular Jacobian are the next concrete mathematical check. Add a finite-difference test for the FixedJoint rotation-log derivative, correct the Jacobian if confirmed, then repeat this exact rocket test before changing production defaults.

## 2026-11-02 — angular Jacobian and singular twist regression

- Added the finite-angle transpose inverse SO(3) left-Jacobian to FixedJoint angular rows. Central finite differences pass for all three error coordinates and world axes at `1e-5`, `0.7`, `2.8`, and `π - 1e-5` radians. A separate finite-difference test covers fixed connector-point Jacobians with nonzero local centers of mass and nontrivial local frames.
- `twist_angle` now represents the undefined 180-degree swing coordinate as `None`; constraints that specifically require that twist coordinate are skipped until it is defined. It does not substitute an arbitrary angle or abort the step. Preparation tests cover Ball, Hinge, Cylindrical, Planar, and ConeTwist; a world integration test confirms a BallJoint step remains finite.
- Validation: `fmt --check`, `check -p kapitsa-core`, warning-free `clippy -p kapitsa-core`, all 581 core unit tests, 28 integration/query/geometry tests, 2 doctests, and `git diff --check` passed.
- The strict Release rocket test still fails at every size. The finite-angle Jacobian did not fix the drift; this change is not evidence for the rocket root cause. The report-only and strict runs completed all 180 ticks, repeated deterministically, retained fuel and separation invariants, and had no step failure or joint break.

| Parts | Anchor max / RMS, m | Frame max / RMS, rad | Coast energy drift | Elapsed |
|---:|---:|---:|---:|---:|
| 100 | 0.1559734 / 0.01254084 | 1.358748 / 0.08467397 | 4.1414% | 852 ms |
| 200 | 0.2910659 / 0.02982499 | 1.713914 / 0.1226009 | 70.8814% | 1,724 ms |
| 500 | 0.7676298 / 0.06260248 | 3.139344 / 0.2949535 | 58.3762% | 4,408 ms |
| 1,000 | 3.023810 / 0.1625110 | 3.141562 / 0.6276528 | 45.5216% | 8,836 ms |

These values remain far above the unchanged strict geometry limits. Contacts occurred at every size; the stage-separation momentum error remained at or below `1.990e-13`; fuel masses reached the expected values; and every run recorded zero joint breaks. The math slice is verified, but the cause of the instability remains unproven.
