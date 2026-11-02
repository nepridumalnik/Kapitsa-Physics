# Joint solver baseline

## Revision and measurement method

- Baseline logic revision: `db165339d04ca57b01945fb022f28c98d0260d1a` (`feat/collisions`). The working tree was clean before this stage.
- Re-measured: 2026-11-02, Windows 11 Pro build 26300, `x86_64-pc-windows-msvc`, AMD Ryzen 9 7900X3D 12-Core Processor, Rust/Cargo 1.98.0, optimized Release profile.
- Step size: `1/120 s`; steady benchmark: 5 warm-up steps and 60 samples; startup and separation: 30 fresh-world samples each; worker counts: 1, 2, 4, and 8.
- The rocket fixture is deterministic and uses no random seed. Each requested size has `parts - 1` fixed joints, two stage roots, and a plane contact. The steady sample begins after warm-up. Startup enables the lower engine. Separation removes the interstage joint, switches engines, applies balanced `2 N·s` stage impulses, and measures the next step.
- Reproduction commands:

  ```powershell
  cargo bench -p kapitsa --features bench-internals --bench large_modular_rocket
  cargo test --release -p kapitsa fixed_step_rockets_report_stability_and_reproducibility_at_all_sizes -- --ignored --nocapture --test-threads=1
  ```

The benchmark selects the sorted observation at index `round((n - 1) * p)`. With 60 steady samples, p99 is the second-highest sample. Phase timings are samples from the p50 step, not averages across all phases. Retained scratch is an internal capacity estimate, not process RSS. The new row counters run only with `bench-internals`; they do not alter the normal runtime path.

## Release steady-state results

One-worker results below are from the instrumented baseline run. The 120 Hz step budget is `8.333 ms`.

| Parts | Joints | Step p50 / p99 | Budget margin at p99 | Joint prepare | Solver prepare | Solver iterations | Collision phases | Island build | Rows prepared / solved | Position / velocity sweeps | Steady allocations | Retained scratch |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 99 | 4.752 / 5.812 ms | +2.521 ms | 4.4 μs | 4.6 μs | 4.679 ms | 16.2 μs | 1.1 μs | 6,534 / 6,534 | 3 / 8 | 0 / 0 B | 433 KiB |
| 200 | 199 | 9.479 / 9.622 ms | -1.289 ms | 13.7 μs | 13.0 μs | 9.329 ms | 19.9 μs | 3.4 μs | 13,134 / 13,134 | 3 / 8 | 0 / 0 B | 853 KiB |
| 500 | 499 | 23.764 / 24.799 ms | -16.466 ms | 30.1 μs | 25.6 μs | 23.443 ms | 18.8 μs | 6.2 μs | 32,934 / 32,934 | 3 / 8 | 0 / 0 B | 1,891 KiB |
| 1,000 | 999 | 47.997 / 50.209 ms | -41.876 ms | 89.2 μs | 61.1 μs | 47.290 ms | 25.7 μs | 16.9 μs | 65,934 / 65,934 | 3 / 8 | 0 / 0 B | 3,769 KiB |

At every size, the steady rocket is one connected solver island. The largest island contains one contact manifold. Workers did not materially reduce step p50: 100 / 200 / 500 / 1,000 parts measured 4.752 / 9.479 / 23.764 / 47.997 ms at one worker and 4.775 / 9.458 / 23.733 / 47.666 ms at eight. The solver iteration phase dominates the cost. The baseline uses scalar joint rows; derived inverse-inertia cache updates, block rows, warm starts, and color batches are all zero. The legacy effective-mass calculation remains active.

After warm-up, steady steps allocated no heap memory at all four sizes. Startup used 1 allocation / 2,368 requested bytes per sample. Separation used 39 / 221,276 B, 44 / 439,308 B, 49 / 921,372 B, and 54 / 1,839,500 B for 100 / 200 / 500 / 1,000 parts, respectively. The corresponding retained scratch after separation was 524 / 1,040 / 2,285 / 4,563 KiB.

## Full fixed-step flight and determinism

The 180-step report scenario includes pad contact, lower-stage ignition, fuel-mass changes, balanced stage separation, upper-stage propulsion and cutoff, coast, and impacts. The test constructs each size twice and asserts exact final body-state equality, equal completed tick count, and equal step-error state. It passed as a repeatability/reporting test (`1 passed`); its separate strict-stability result remains visible below.

The unchanged limits are max/RMS anchor error below `0.03 / 0.01 m`, max/RMS frame error below `0.1 / 0.03 rad`, and coast energy drift below `10%`.

| Parts | Scenario | Max / RMS anchor | Max / RMS frame | Coast energy drift | Surface begin / impact | Separation momentum error | Strict stability |
|---:|---|---:|---:|---:|---:|---:|---|
| 100 | Complete | 0.204898 / 0.014001 m | 1.489061 / 0.093369 rad | 3.2567% | 1 / 110 | 3.553e-15 kg·m/s | FAIL |
| 200 | Complete | 0.282695 / 0.028332 m | 1.413066 / 0.121849 rad | 66.4778% | 2 / 91 | 1.421e-14 kg·m/s | FAIL |
| 500 | Complete | 0.727550 / 0.064004 m | 2.780816 / 0.279044 rad | 59.2713% | 2 / 46 | 2.061e-13 kg·m/s | FAIL |
| 1,000 | Incomplete: no surface events | 2.759083 / 0.161574 m | 3.141570 / 0.635048 rad | 45.0492% | 0 / 0 | 5.684e-14 kg·m/s | FAIL |

All four sizes are already numerical failures at this baseline. The 1,000-part run completed all ticks without a step error but did not register the expected pad contact or impact. No thresholds were relaxed. The stress report records per-step maximum and RMS connector geometry error; a post-sweep velocity/KKT residual is not available yet and belongs to the next diagnostics stage.

## Stage 00 conclusions

- 100 parts is the only measured size whose one-worker steady p99 fits the 8.333 ms budget; 200 parts already exceeds it.
- The existing full-flight stability criteria fail at every size, including 100 parts. These results remain the numerical correctness gate for later stages.
- The benchmark counters match the configured legacy path: 3 position sweeps, 8 velocity sweeps, scalar rows only, no block solving, no persistent joint warm start, and no coloring.
- No full-flight performance or stability claim is inferred from the high-altitude Time Warp coast benchmark.
