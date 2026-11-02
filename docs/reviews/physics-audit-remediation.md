# Physics remediation: status and limitations

Baseline: `8be00b228c347f999d99a23f279bfc95e4bac8c4` (`feat/collisions`). This is a partial register of the 50 stages in the supplied task, not the requested 61-item historical audit: that audit is not present in the task or repository. No E-number mapping can be inferred reliably except E-001.

E-001: the claimed double warm-start application is unverified; cached impulses followed by incremental impulses are not by themselves double application. No change was made on this basis.

Statuses below describe current evidence, not an assertion that unaudited stages are defective. BLOCKED means the task is not completed; NOT AUDITED means no conclusion has been established.

| Stage | Requested area | Current state |
|---|---|---|
| S01 | Реестр состояния | NOT AUDITED |
| S02 | Корректность ракетной фикстуры | MITIGATED — pad start corrected and tested; separate flight, separation and collision configurations remain outstanding |
| S03 | Строгая оценка стабильности | MITIGATED — c29c698; static check only |
| S04 | Корректная энергетическая диагностика | NOT AUDITED |
| S05 | Диагностика сходимости Joint | NOT AUDITED |
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
| S16 | Семантика нормальных импульсов | NOT AUDITED |
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
| S28 | Согласованность position/velocity solver | NOT AUDITED |
| S29 | Изменение массовых характеристик | MITIGATED — documented state-edit contract; physical mass transfer remains unimplemented |
| S30 | Инвалидирование Joint warm-start | NOT AUDITED |
| S31 | Повторное использование Joint row topology | NOT AUDITED |
| S32 | Устранение `Vec::splice` из горячего пути | NOT AUDITED |
| S33 | Единое эффективное использование SolverMass | NOT AUDITED |
| S34 | Сокращение копирования островов | NOT AUDITED |
| S35 | Broadphase для неограниченных поверхностей | NOT AUDITED |
| S36 | Удаление тел по смежности | NOT AUDITED |
| S37 | Adaptive timestep без повторных SPD solve | NOT AUDITED |
| S38 | Уменьшение затрат query API | NOT AUDITED |
| S39 | Эффективные snapshots | NOT AUDITED |
| S40 | Буферизация событий | NOT AUDITED |
| S41 | Graph Coloring внутри solver islands | NOT AUDITED |
| S42 | Rigid Clusters / Compound Bodies | NOT AUDITED |
| S43 | Featherstone ABA | BLOCKED — mathematical and numerical validation required before integration |
| S44 | Транзакционная основа TGS | BLOCKED — existing CCD/forces/cache/rollback blocker; see temporal-substeps-blocker-report.md |
| S45 | Temporal Gauss–Seidel | BLOCKED — depends on S44 and numerical validation |
| S46 | On-Rails propagation | NOT AUDITED |
| S47 | Разделение общих solver-примитивов | MITIGATED — staged SolverBody and tangent basis moved into common constraint module; joint code still uses collision errors |
| S48 | Разделение внутренних модулей | NOT AUDITED |
| S49 | Публичная конфигурация и feature gates | NOT AUDITED |
| S50 | Аудит аллокаций и оставшихся hot paths | NOT AUDITED |

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
