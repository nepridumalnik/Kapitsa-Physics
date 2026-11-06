# Physics remediation: status and limitations

Baseline: `8be00b228c347f999d99a23f279bfc95e4bac8c4` (`feat/collisions`). This is a partial register of the 50 stages in the supplied task, not the requested 61-item historical audit: that audit is not present in the task or repository. No E-number mapping can be inferred reliably except E-001.

E-001: the claimed double warm-start application is unverified; cached impulses followed by incremental impulses are not by themselves double application. No change was made on this basis.

Statuses below describe current evidence, not an assertion that unaudited stages are defective. BLOCKED means the task is not completed; NOT AUDITED means no conclusion has been established.

| Stage | Requested area | Current state |
|---|---|---|
| S01 | Реестр состояния | NOT AUDITED |
| S02 | Корректность ракетной фикстуры | MITIGATED — pad start corrected and tested; separate flight, separation and collision configurations remain outstanding |
| S03 | Строгая оценка стабильности | MITIGATED — c29c698; static check only |
| S04 | Корректная энергетическая диагностика | PARTIAL — midpoint gyro and physical coast potential validated; constrained predict/project dissipation characterized, coupled integration remains open |
| S05 | Диагностика сходимости Joint | PARTIAL — geometric/Jv residuals and PCG cap outcomes measured; production convergence policy remains open |
| S06 | Многоточечный Plane × Convex manifold | NOT AUDITED |
| S07 | Многоточечный Convex × Convex manifold | NOT AUDITED |
| S08 | Стабильная идентичность геометрических фич | NOT AUDITED |
| S09 | Масштабно-корректные геометрические допуски | MITIGATED — b1c2738; translation/rounding behavior requires numerical validation |
| S10 | Геометрическая валидация ContactCache | MITIGATED — validate both body-local anchors within contact_offset; cache regression and collision suite passed |
| S11 | Корректное масштабирование warm-start | MITIGATED — 00b191d; policy requires numerical validation |
| S12 | Persistent contact manifold | NOT AUDITED |
| S13 | Численная устойчивость трения | MITIGATED — c1e2fd4; conditioning threshold requires numerical validation |
| S14 | Подготовленные effective masses контактов | NOT AUDITED |
| S15 | Актуальность контактных Jacobian | NOT AUDITED |
| S16 | Семантика нормальных импульсов | MITIGATED — internal field renamed and documented as sum of positive updates; net change and impact classification remain separate work |
| S17 | Корректный `Impact` | NOT AUDITED |
| S18 | Согласованные данные событий | MITIGATED — document representative geometry timing and manifold impulse semantics; CCD and discrete samples can still be combined |
| S19 | Взаимодействие CCD и дискретного solver | NOT AUDITED |
| S20 | Достоверность GJK fallback | NOT AUDITED |
| S21 | Критерии завершения EPA | NOT AUDITED |
| S22 | Эффективность EPA horizon | NOT AUDITED |
| S23 | Корректные вращательные Jacobian | NOT AUDITED |
| S24 | Непрерывные координаты вращения | NOT AUDITED |
| S25 | Физические единицы и compliance | NOT AUDITED |
| S26 | Корректный adaptive residual | MITIGATED — f076d1e corrects fixed bounds; damped and compliant velocity residuals still require review |
| S27 | Реакции и разрушение Joint | NOT AUDITED |
| S28 | Согласованность position/velocity solver | PARTIAL — stage energy and post-position Jv measured; split integration still creates constrained-velocity mismatch |
| S29 | Изменение массовых характеристик | MITIGATED — documented state-edit contract; physical mass transfer remains unimplemented |
| S30 | Инвалидирование Joint warm-start | NOT AUDITED |
| S31 | Повторное использование Joint row topology | NOT AUDITED |
| S32 | Устранение `Vec::splice` из горячего пути | NOT AUDITED |
| S33 | Единое эффективное использование SolverMass | NOT AUDITED |
| S34 | Сокращение копирования островов | NOT AUDITED |
| S35 | Broadphase для неограниченных поверхностей | NOT AUDITED |
| S36 | Удаление тел по смежности | NOT AUDITED |
| S37 | Adaptive timestep без повторных SPD solve | NOT AUDITED |
| S38 | Уменьшение затрат query API | MITIGATED — expose caller-owned QueryScratch for raycast, sweep and overlaps; repeated query integration test passed |
| S39 | Эффективные snapshots | MITIGATED — reuse existing index when body IDs and order are unchanged; all snapshot tests passed; performance not measured |
| S40 | Буферизация событий | MITIGATED — event batches share nonempty immutable buffers across broadcast subscribers; initial publication still copies world events |
| S41 | Graph Coloring внутри solver islands | MITIGATED — level-ordered conflict-free execution is implemented; branch speedups measured, connected-chain scaling not achieved |
| S42 | Rigid Clusters / Compound Bodies | NOT AUDITED |
| S43 | Featherstone ABA | BLOCKED — mathematical and numerical validation required before integration |
| S44 | Транзакционная основа TGS | BLOCKED — existing CCD/forces/cache/rollback blocker; see temporal-substeps-blocker-report.md |
| S45 | Temporal Gauss–Seidel | BLOCKED — depends on S44 and numerical validation |
| S46 | On-Rails propagation | NOT AUDITED |
| S47 | Разделение общих solver-примитивов | MITIGATED — staged SolverBody and tangent basis moved into common constraint module; joint code still uses collision errors |
| S48 | Разделение внутренних модулей | NOT AUDITED |
| S49 | Публичная конфигурация и feature gates | NOT AUDITED |
| S50 | Аудит аллокаций и оставшихся hot paths | MITIGATED — reviewed contact cache, joint row storage, snapshots and event publication; broader hot-path audit remains; diagnostic snapshot scratch is bench-only |

## Verification and risks

- Rust 1.98: `cargo fmt --all -- --check`, `cargo check --workspace --all-features --locked --offline`, and `cargo clippy --workspace --all-features --locked --offline -- -D warnings` passed after S13.
- `cargo +1.98.0 test --workspace --locked --offline -j 4` passed 771 checks including doctests; 4 tests were ignored. On the initial run, the triangle/cuboid SAT tangent case failed both on this branch and baseline 8be00b2: the SAT oracle measured a 1.39e-17 gap after rotation. Commit 13a19f3 accounts for projection roundoff in the reference comparison; the geometry matrix then passed all 10 tests. Commit 2e73537 adds a timestep reset check. These checks do not establish general physical accuracy, convergence, event semantics, or performance.
- S09 uses both body-local anchors; accumulated world-coordinate rounding and conservative duplicate rejection still need translated-scene validation.
- S11 resets rather than scales cached impulses outside [0.5, 2]; the thresholds are a policy, not an experimentally established optimum.
- S13 skips the tangential correction when the normalized determinant is at most 64 machine epsilons; this conservative threshold and its effects need numerical validation.
- S03 strict evaluation includes scenario assumptions (including impacts), which must be reviewed alongside S02 before interpreting a failure as numerical instability.
- TGS, rigid clusters, ABA, on-rails propagation and other unreviewed strategies are not claimed as implemented.

- The ignored release rocket scenario was explicitly run and failed strict criteria for 100, 200, 500 and 1000 parts. Maximum joint anchor errors were 0.2048983, 0.2826946, 0.7275499 and 2.759083 metres respectively. The 1000-part scenario also reported no surface events; the fixture and energy diagnostic remain to be reviewed. The fixed-bound residual correction was made after this run and affects the adaptive path; no rerun has established an effect on the default rocket scenario.

- After S02 and S10, the release rocket scenario was rerun. All four sizes still failed the numerical criteria; maximum anchor errors were 0.2432003, 0.2867673, 0.7363993 and 2.911675 m for 100, 200, 500 and 1000 parts. All four completed the scenario. The root cause of the joint instability remains unverified.

## Allocation review (partial)

- `joint/workspace.rs` retains row buffers but still uses `Vec::splice` when a joint changes its row count; repeated tail movement remains (S32).
- `collision/cache.rs` retains separate current and next hash maps for transactional commit/abort; those buffers are intentional and can grow when contact count grows.
- `asynchronous/service.rs` still copies world events once for publication; immutable batches now share nonempty vectors between subscribers. This adds an Arc allocation for a nonempty batch, so the net benefit depends on subscriber count.
- Snapshot buffers are recycled only when no subscriber retains the prior Arc. Unchanged body IDs reuse their lookup index, but every body state is still copied to preserve immutable snapshots.
- The rocket benchmark observed zero allocator calls at p50/p95 for warmed steady samples; it does not prove zero allocations for every step or scenario. Separation samples allocated. See `rocket-performance-observation.md` for conditions and interference.

- Subsequent workspace run completed 773 unit/integration checks with 4 ignored, but its doctest phase failed because a concurrent build changed an artifact. A separate `cargo +1.98.0 test --workspace --doc --locked --offline -j 4` then passed both doctests. The impulse-field rename was checked separately with 8 collision-solver tests and strict workspace Clippy.

## Joint solver update (2026-11-03)

- The energy stage ledger and projection-work accounting are dynamically validated; see [`joint-solver-energy-parallel-results.md`](joint-solver-energy-parallel-results.md). Velocity projection loss on a free fixed assembly closes against impulse work. Scalar graph under-convergence is demonstrated; this does not mean ideal rigid assembly energy has been solved by the integrator.
- Corrected physical coast drift and the former surface-zero metric were compared on the same 500-part PCG full-flight trajectory: `4.598934067e−7%` versus `88.94944039%`. The old denominator is sensitive to the arbitrary potential offset. The previously cited roughly 6% experimental-PCG figure has no matching saved configuration and is not reproduced by current free-assembly PCG runs (`0.004841–0.004958%`); attribution remains unverified.
- The strict scalar rocket test fails anchor/frame limits at 100/200/500/1,000 parts. The benchmark-only IC(0) PCG 5/5 configuration with 16 outer velocity sweeps passes all four unchanged strict scenarios; it remains non-production because it is substantially slower and many local solves hit their cap.
- Updated validation on this checkpoint: `cargo +1.98.0 fmt --all -- --check`, all-features workspace `check`, strict all-features Clippy, and serial workspace tests/doctests passed. Focused Release rotor, projection work, free assembly, gravity-only rocket, strict PCG screen, and same-state metric comparison passed. The default scalar strict rocket failure is retained as a failure.
- Production gaps remain: no fully coupled constrained midpoint integrator, no production PCG selection, no complete audit of the 50/61 requested stages, and no verified persistent worker pool. The 6% historical PCG value remains unresolved until its exact scenario/configuration is recovered.
