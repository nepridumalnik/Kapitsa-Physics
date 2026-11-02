# Physics remediation: status and limitations

Baseline: `8be00b228c347f999d99a23f279bfc95e4bac8c4` (`feat/collisions`). This is a partial register of the 50 stages in the supplied task, not the requested 61-item historical audit: that audit is not present in the task or repository. No E-number mapping can be inferred reliably except E-001.

E-001: the claimed double warm-start application is unverified; cached impulses followed by incremental impulses are not by themselves double application. No change was made on this basis.

Statuses below describe current evidence, not an assertion that unaudited stages are defective. BLOCKED means the task is not completed; NOT AUDITED means no conclusion has been established.

| Stage | Requested area | Current state |
|---|---|---|
| S01 | Реестр состояния | NOT AUDITED |
| S02 | Корректность ракетной фикстуры | NOT AUDITED |
| S03 | Строгая оценка стабильности | MITIGATED — c29c698; static check only |
| S04 | Корректная энергетическая диагностика | NOT AUDITED |
| S05 | Диагностика сходимости Joint | NOT AUDITED |
| S06 | Многоточечный Plane × Convex manifold | NOT AUDITED |
| S07 | Многоточечный Convex × Convex manifold | NOT AUDITED |
| S08 | Стабильная идентичность геометрических фич | NOT AUDITED |
| S09 | Масштабно-корректные геометрические допуски | MITIGATED — b1c2738; translation/rounding behavior requires numerical validation |
| S10 | Геометрическая валидация ContactCache | NOT AUDITED |
| S11 | Корректное масштабирование warm-start | MITIGATED — 00b191d; policy requires numerical validation |
| S12 | Persistent contact manifold | NOT AUDITED |
| S13 | Численная устойчивость трения | MITIGATED — c1e2fd4; conditioning threshold requires numerical validation |
| S14 | Подготовленные effective masses контактов | NOT AUDITED |
| S15 | Актуальность контактных Jacobian | NOT AUDITED |
| S16 | Семантика нормальных импульсов | NOT AUDITED |
| S17 | Корректный `Impact` | NOT AUDITED |
| S18 | Согласованные данные событий | NOT AUDITED |
| S19 | Взаимодействие CCD и дискретного solver | NOT AUDITED |
| S20 | Достоверность GJK fallback | NOT AUDITED |
| S21 | Критерии завершения EPA | NOT AUDITED |
| S22 | Эффективность EPA horizon | NOT AUDITED |
| S23 | Корректные вращательные Jacobian | NOT AUDITED |
| S24 | Непрерывные координаты вращения | NOT AUDITED |
| S25 | Физические единицы и compliance | NOT AUDITED |
| S26 | Корректный adaptive residual | NOT AUDITED |
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
| S47 | Разделение общих solver-примитивов | NOT AUDITED |
| S48 | Разделение внутренних модулей | NOT AUDITED |
| S49 | Публичная конфигурация и feature gates | NOT AUDITED |
| S50 | Аудит аллокаций и оставшихся hot paths | NOT AUDITED |

## Verification and risks

- Rust 1.98: `cargo fmt --all -- --check`, `cargo check --workspace --all-features --locked --offline`, and `cargo clippy --workspace --all-features --locked --offline -- -D warnings` passed after S13.
- After the user lifted the verification restriction, `cargo +1.98.0 test --workspace --locked --offline -j 4` ran: the main kapitsa unit suite passed 118 tests with 4 ignored, integration suites passed 12, 3 and 1 tests, and kapitsa-core had 574 passed and one failed; Cargo stopped before the remaining workspace suites. The failing independent triangle/cuboid SAT tangent case also fails on unmodified baseline 8be00b2. The SAT oracle measured a separating gap of approximately 1.39e-17 after rotation; the discrepancy requires further investigation. Benchmarks and ignored simulations have not been run. Static checks cannot establish physical accuracy, convergence, event semantics, or performance.
- S09 uses both body-local anchors; accumulated world-coordinate rounding and conservative duplicate rejection still need translated-scene validation.
- S11 resets rather than scales cached impulses outside [0.5, 2]; the thresholds are a policy, not an experimentally established optimum.
- S13 skips the tangential correction when the normalized determinant is at most 64 machine epsilons; this conservative threshold and its effects need numerical validation.
- S03 strict evaluation includes scenario assumptions (including impacts), which must be reviewed alongside S02 before interpreting a failure as numerical instability.
- TGS, rigid clusters, ABA, on-rails propagation and other unreviewed strategies are not claimed as implemented.
