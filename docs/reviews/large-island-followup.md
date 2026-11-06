# Large-island follow-up: allocation and FixedJoint diagnostics

Measurements below are for the `feat/collisions` experimental partial-coloring
solver, not evidence for enabling it in production. The rocket uses a 1/120 s
step, 8 velocity iterations and FixedBlock3. The recorded release runs used
1000 parts and 999 initial joints.

## Allocation investigation

With four Rayon workers, light diagnostics, 200 warmup steps and 1000 measured
steps, the partial-coloring run had 16 steps with one allocation each. Removing
the surface collider also produced 16 such steps. A one-worker sequential run
with the same warmup and sample counts had no allocations. A separate run with
only five warmup steps had 15 allocating steps in 1000 samples.

The benchmark allocator counts all threads. A separate diagnostic run captured
an allocation of 1520 bytes on the calling thread, outside a Rayon worker. Its
stack was `ThreadPool::install -> Registry::inject -> Injector::push`. Tracing
was disabled for the 1000-step counts; the allocation counter itself performs
only atomic operations. The local `crossbeam-deque` 0.8.8 implementation uses
blocks containing 63 entries and allocates the next block as the queue advances.
Rayon 1.13.0 executes nested `install` directly when already on a worker of the
same registry; the traced allocation was from the external entry, not evidence
that nested `install` allocates. More warmup does not eliminate a recurring
queue-block allocation.

The single-island executor dispatch is currently performed before the solver
knows whether a useful color schedule exists. In the earlier five-run screen,
100, 200 and 500 part fixtures fell back to sequential solving but still paid
that dispatch. There is no confirmed allocation-free replacement for the
external entry in the current experiment. Moving the entire measurement loop
inside a pool worker would change the caller context and is not a fix for
external callers.

The benchmark accepts `KAPITSA_BENCH_WARMUPS`, `KAPITSA_BENCH_SAMPLES`, and
`KAPITSA_BENCH_NO_CONTACT` for reproducing these measurements. For example:

```powershell
$env:KAPITSA_BENCH_WARMUPS = '200'
$env:KAPITSA_BENCH_SAMPLES = '1000'
cargo bench -p kapitsa --bench large_modular_rocket --features bench-internals -- --parts=1000 --workers=4 --block3 --steady-only --light-diagnostics --partial-coloring
```

## Loading and convergence

The ignored `diagnose_fixed_block_rocket_loading_and_convergence` test runs
180 steps, separately varying the load and position iterations. These are
measured maxima, not acceptable tolerances:

| Scenario, 8 velocity / 3 position | Sequential max anchor / frame | Partial 4 workers max anchor / frame | First threshold crossing, sequential / partial (anchor, frame) |
|---|---:|---:|---:|
| No loads | 3.74e-15 m / 0 rad | 2.71e-15 m / 0 rad | none / none |
| Gravity without contact | 1.07e-6 m / 3.80e-7 rad | 1.68e-6 m / 3.51e-7 rad | none / none |
| Gravity with contact | 0.342 m / 1.092 rad | 0.676 m / 2.576 rad | 14, 41 / 13, 35 |
| Thrust without gravity or contact | 2.403 m / 3.141 rad | 3.664 m / 3.141 rad | 13, 19 / 13, 17 |
| Full staged scenario | 2.498 m / 3.141 rad | 2.156 m / 3.141 rad | 12, 19 / 12, 16 |

The limits used to report first crossings are 0.03 m and 0.1 rad. Ignition is
at tick 12 and separation at tick 54, so both full-scenario crossings precede
separation and occur far below a pi rotation. Partial coloring changes the
result and is worse in the contact-only and thrust-only screens; the full test
fails for both schedules. It is not established that the observed degradation
has a single cause.

For the full scenario, 3, 6, and 12 position sweeps yielded sequential maximum
anchor errors of 2.498, 1.041, and 0.541 m, and maximum frame errors of 3.141,
3.119, and 2.755 rad. Partial coloring yielded 2.156, 1.333, and 0.776 m, and
3.141, 3.137, and 2.930 rad. Increasing iterations improves these maxima in
this screen but still misses the limits; it is a diagnostic, not a proposed
physics fix.

`diagnose_fixed_block_iterations_from_same_pose` rebuilds the same rocket for
each iteration count, checks that its joint errors match immediately before
ignition, and measures the following step. It isolates the effect of position
sweeps within each schedule; the sequential and colored initial poses are not
asserted equal to each other. Setting the iteration count resets the contact
cache equally for every variant. `KAPITSA_PARTIAL_WORKERS=4` selects the colored
variant. `KAPITSA_ROCKET_TRACE_JOINTS=1` prints a persistent joint, worst
linear and angular joint IDs, endpoint bodies, local anchors and world anchors
at selected ticks. World IDs differ across independently constructed fixtures.

The existing finite-difference Jacobian test covers angles through pi minus
1e-5, and the two-body thrust test covers mass ratios through 1:10000. Both
pass, but neither proves correctness for the loaded 1000-body graph. The
sequential full-scenario screen also produced the same printed maxima, RMS
errors and first crossing ticks with the solver mass cache enabled and
disabled: 2.498309317 m / 3.140541081 rad, and ticks 12 / 19. This rules out
the cache as the explanation for those observations, within the resolution of
the reported metrics; it does not establish every intermediate state is equal.
Use the ignored `diagnose_fixed_block_rocket_mass_cache` test and set
`KAPITSA_ROCKET_NO_MASS_CACHE=1` for the uncached run.

The specific roles of accumulated multipliers, Jacobian refresh and impulse
propagation on the loaded graph remain to be isolated. No production solver
change or hybrid schedule is justified by the current evidence.
