# Joint solver local baseline

Date: 2026-11-02  
Branch: `audit/physics-remediation`  
Revision: `1e083a51be401539d327f1951ff877d24822e524`  
Remote: `origin/audit/physics-remediation` matched the local HEAD before measurement.  
Toolchain: Rust 1.98.0, Windows x86_64 MSVC, AMD Ryzen 9 7900X3D (12 cores / 24 logical processors).

This is a local pre-fix baseline. It measures the code at the revision above; it is not a before/after comparison and it does not establish production readiness.

## Verification

| Command | Result |
|---|---|
| `cargo +1.98.0 fmt --all -- --check` | PASS |
| `cargo +1.98.0 check --workspace --all-features --locked` | PASS |
| `cargo +1.98.0 clippy --workspace --all-features --locked -- -D warnings` | PASS |
| `cargo +1.98.0 test --workspace --locked` | PASS: 775 passed, 4 ignored |
| Release rocket test with `KAPITSA_ROCKET_REPORT_ONLY=1` | Diagnostic report completed; the ignored test's return value is not a stability PASS |
| Release rocket test without `REPORT_ONLY` | FAIL: the original numerical limits are violated at all four sizes |

The benchmark was run twice sequentially with no concurrent test or build. The second run captured the complete output at `target/joint-solver-baseline-bench.txt`; it emitted the expected 16 steady, 16 start, and 16 separation sample records and contains no runtime error or panic. The PowerShell redirection wrapper returned exit status 1 despite producing the complete report. The first, unredirected run of the requested benchmark command returned 0. Its steady-state run uses 5 warmups and 60 measured steps; start and separation each use 30 samples. Step is `1/120 s`. The harness now reports p50 and p95 of solver iteration, joint preparation, and solver preparation times over each phase's samples.

## Strict rocket baseline

The report-only and strict runs produced identical state and diagnostic values. Every scenario completed all 180 ticks, repeated deterministically, consumed both fuels to 3.5 mass units, separated the stages with zero remaining inter-stage joint, and emitted no joint-break event. A nonzero surface-contact event count was observed at every size. `elapsed` is one scenario run; the test executes each size twice.

| Parts | Run ms | Anchor max / RMS, m | Frame max / RMS, rad | Coast energy drift | Contacts begin / end / impact | Separation momentum error | Max angular speed, rad/s | Result |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 100 | 843 | 0.1538798 / 0.01302640 | 1.333782 / 0.08493318 | 4.1225% | 2 / 1 / 124 | `1.421e-14` | 5.039 | FAIL |
| 200 | 1,729 | 0.2846846 / 0.02980872 | 1.565341 / 0.1226705 | 69.9332% | 3 / 2 / 85 | `0` | 14.052 | FAIL |
| 500 | 4,533 | 0.6644705 / 0.06259963 | 3.141284 / 0.3265347 | 59.3510% | 3 / 2 / 54 | `2.629e-13` | 19.121 | FAIL |
| 1,000 | 9,061 | 2.814815 / 0.1660589 | 3.141479 / 0.6627714 | 44.4683% | 1 / 1 / 12 | `1.421e-13` | 24.855 | FAIL |

Strict limits include maximum anchor error below `0.03 m`, RMS anchor error below `0.01 m`, maximum frame error below `0.1 rad`, RMS frame error below `0.03 rad`, and coast energy drift below `10%`. All four sizes exceed both anchor limits and both frame limits. The 200–1,000-part runs exceed the energy-drift limit.

The fixture includes central gravity, two root colliders on the pad, offset thrust at the root bodies, two fuel-mass changes, and stage separation at tick 54 with equal and opposite impulses. The measured momentum error at separation is near roundoff, all steps completed, and fuel reached its requested mass. These checks validate those fixture events, but do not isolate their contribution to the observed Joint and energy errors. The strict failure therefore remains a solver/fixture investigation gate; it does not prove one exclusive cause.

## Steady-state performance

The following table uses the full 60-sample steady phase. Solver times are p50/p95 across each phase's samples; step times are p50/p95 of complete world steps. Preparation times are microseconds. `Rows` is the number of prepared joint rows reported for the p50 step.

| Parts | Workers | Step p50 / p95, ms | Solver iterations p50 / p95, ms | Joint prepare p50 / p95, µs | Solver prepare p50 / p95, µs | Rows | Allocations p50 / p95 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 1 | 5.5820 / 5.6334 | 5.5049 / 5.5585 | 4.30 / 5.00 | 4.80 / 5.40 | 6,534 | 0 / 0 |
| 100 | 2 | 5.5679 / 5.6609 | 5.4921 / 5.5849 | 4.50 / 6.40 | 4.50 / 5.10 | 6,534 | 0 / 0 |
| 100 | 4 | 5.5775 / 5.6052 | 5.5002 / 5.5338 | 4.20 / 5.00 | 4.40 / 5.00 | 6,534 | 0 / 0 |
| 100 | 8 | 5.5682 / 5.6735 | 5.4944 / 5.5761 | 4.60 / 7.60 | 4.40 / 5.00 | 6,534 | 0 / 0 |
| 200 | 1 | 11.2930 / 12.0888 | 11.1546 / 11.9129 | 12.80 / 19.10 | 9.80 / 13.80 | 13,134 | 0 / 0 |
| 200 | 2 | 11.2251 / 11.4826 | 11.0797 / 11.3391 | 13.00 / 16.00 | 10.50 / 11.00 | 13,134 | 0 / 0 |
| 200 | 4 | 11.1855 / 11.2640 | 11.0422 / 11.1157 | 12.00 / 16.40 | 10.40 / 11.10 | 13,134 | 0 / 0 |
| 200 | 8 | 11.1933 / 11.3612 | 11.0543 / 11.2224 | 11.80 / 14.20 | 10.30 / 10.80 | 13,134 | 0 / 0 |
| 500 | 1 | 28.4157 / 30.8523 | 28.0364 / 30.4204 | 47.60 / 54.20 | 30.50 / 46.10 | 32,934 | 0 / 0 |
| 500 | 2 | 28.2265 / 29.2547 | 27.8658 / 28.8958 | 42.00 / 47.00 | 29.30 / 31.60 | 32,934 | 0 / 0 |
| 500 | 4 | 28.1472 / 29.0928 | 27.7951 / 28.7462 | 38.00 / 43.70 | 27.60 / 56.90 | 32,934 | 0 / 0 |
| 500 | 8 | 28.1920 / 28.6015 | 27.8526 / 28.2289 | 35.10 / 44.00 | 25.30 / 29.40 | 32,934 | 0 / 0 |
| 1,000 | 1 | 56.5401 / 57.0648 | 55.8675 / 56.4010 | 77.30 / 85.70 | 62.70 / 70.10 | 65,934 | 0 / 0 |
| 1,000 | 2 | 56.6033 / 57.5728 | 55.9170 / 56.8322 | 79.90 / 92.80 | 63.90 / 78.20 | 65,934 | 0 / 0 |
| 1,000 | 4 | 58.0692 / 67.5805 | 57.3625 / 66.8485 | 84.20 / 104.20 | 68.80 / 125.30 | 65,934 | 0 / 0 |
| 1,000 | 8 | 57.3505 / 59.9927 | 56.6205 / 59.2834 | 85.80 / 100.80 | 64.60 / 77.00 | 65,934 | 0 / 0 |

At 1,000 parts the one-worker steady step median is `56.5401 ms`, about 6.8 times the `8.33 ms` step budget. Worker-count changes do not materially improve the connected-island median. Eight workers are below the machine's 24 logical processors, but the solver still processes the single connected island sequentially; these runs do not demonstrate intra-island scaling.

The warm steady phase reported zero allocations at p50/p95. The start phase reported one allocation and 2,368 requested bytes at p50/p95. Separation reported 58 / 66 / 74 / 82 allocations and about 0.44 / 0.88 / 1.81 / 3.63 MB requested bytes at 100 / 200 / 500 / 1,000 parts, using one worker. Separation created two islands and increased retained scratch from 620 / 1,226 / 2,606 / 5,199 KiB to 857 / 1,703 / 3,604 / 7,196 KiB.

The measured production configuration uses scalar Gauss–Seidel, 3 position sweeps and 8 velocity sweeps. The benchmark reports zero Block3 rows, zero warm starts, zero intra-island color batches, and zero mass-cache inertia updates. This identifies the optimization gates still requiring an apples-to-apples, numerically valid comparison.
