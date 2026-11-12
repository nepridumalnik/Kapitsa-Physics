# Step task execution

`kapitsa::execution::StepExecutor` supports borrowed independent chunks and
`join(left, right)`. A branch may call the same executor recursively. Returning
from an operation joins its children; results express dependencies through normal
Rust control flow. There is no detached work inside a physics step and no new
async runtime or task graph allocation.

```rust,ignore
use kapitsa::execution::{RayonPool, StepExecutor};

let pool = RayonPool::new(tokio_runtime.handle(), 8)?;
let mut executor = pool.executor()?;
executor.set_parallelism(2);
let (a, b) = executor.join(
    || executor.join(|| compute_a(), || compute_b()),
    || compute_c(),
);
consume(a, b);
```

The existing Tokio runtime hosts a fixed Rayon pool. Its maximum defaults to
`std::thread::available_parallelism()` in `Simulation`; `WorkerConfig` can lower
it. One root computation and its children share a budget, including through
cloned executor handles. Nonblocking permits bound additional branches. When no
permit is available, work runs locally, so a parent never blocks waiting for a
child to obtain a permit. Permits are released during panic unwinding too.

Call `set_parallelism` only between completed root computations. Separate roots
submitted with `spawn` are orchestration tasks, not independent holders of this
compute budget. Obtain separate executors for independent simulations. Existing
custom `StepExecutor` implementations retain their chunk behavior and use the
sequential default `join` until they override it.

The adaptive scheduler starts at one worker. It measures elapsed step time,
smoothed with EWMA, against `1 / tick_rate` seconds: 8.333333 ms at 120 Hz.
The existing default tick rate remains 60 Hz; set `SimulationConfig.tick_rate`
to 120 for the 120 Hz budget. Sustained utilization at or above 70% probes
`min(workers * 2, max_workers)`; at or below 30% it selects
`max(workers / 2, 1)`. The existing stabilization windows and speedup probe
prevent repeated unproductive growth. These thresholds do not change physics
iterations, tolerances, timestep, or solver selection.

Gravity is computed independently per body in ordered reusable output buffers.
Errors are checked in body order before applying outputs. Integration and
collision tasks retain their existing dependency order, rollback, island
partitioning, and solver algorithms. Summations within a body are unchanged.
`AerodynamicsTask` checks aerodynamic force calculations without advancing the
world. World steps apply drag and optional lift using atmospheric conditions
sampled at each surface. `HydrodynamicsTask` remains an extension point without
an active force model.

Validation commands:

```text
cargo test --workspace --all-features --release --no-fail-fast -- --test-threads=1
cargo clippy --workspace --all-features --all-targets
cargo bench --workspace --all-features --benches --no-fail-fast
```

`parallel_step` exercises the production executor with a fixed eight-thread
pool and active limits of 1/2/4/8, including sharded integration with gravity.
Nested-task tests check the shared concurrency bound, overlapping independent
branches, borrowed results, indexed chunks and permit recovery after panic.
A 120-step test changes worker counts and checks body-state bits against the
sequential path on every step.
