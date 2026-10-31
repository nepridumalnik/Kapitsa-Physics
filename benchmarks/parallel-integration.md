# Parallel Integration Baseline

Measured with `cargo bench -p kapitsa-tokio --bench parallel_step` on 2026-10-31.

- OS: Windows 11 Pro
- CPU: AMD Ryzen 9 7900X3D, 12 cores
- Rust/Cargo: 1.98.0
- Workload: 20,000 dynamic bodies, 24 fixed steps, median of 5 runs

| Mode | Median | Speedup vs sequential |
|---|---:|---:|
| Synchronous `PhysicsWorld::step` | 150.90 ms | 1.00x |
| Tokio, 1 worker | 177.51 ms | 0.85x |
| Tokio, 2 workers | 102.63 ms | 1.47x |
| Tokio, 4 workers | 63.58 ms | 2.37x |

The one-worker path is slower because snapshotting, task dispatch, and commit
cost more than they save for this workload. Parallel speedup appears at two
workers and reaches 2.37x at four. Results are specific to this CPU, Tokio
runtime, and workload; small worlds should use one worker to avoid dispatch
overhead.
