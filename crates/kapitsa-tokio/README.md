# kapitsa-tokio

`PhysicsController` runs fixed physics steps on Tokio's blocking pool. The crate
uses the caller's runtime; it never creates a production runtime. Each step
copies only body data into balanced shards, runs at most `active_workers` shard
tasks, waits for all results, and commits atomically.

## Adaptive policy

For each completed step, the working budget is `fixed_dt * target_utilization`
and the overload threshold is that budget multiplied by
`1 + overload_tolerance`. The regulator updates its bounded integral as
`clamp(previous * 0.9 + clamp(duration / threshold - 1, 0, 1), 0, 4)`.
It changes settings only after at least three consecutive over-threshold steps
and an integral of at least `2.2`.

Worker increases double the current count up to `max_workers`. Each increase is
measured over 12 completed steps and retained only when average full-step time
improves by at least 5%; otherwise the prior count is restored. An ineffective
increase is not retried until the integral clears below `0.4` or measured load
rises by at least 25%. Changes have a 12-step stabilization period.

After parallelism is exhausted or rejected, frequency can decrease through
`60, 50, 40, 30 Hz`, filtered by the configured target and minimum. Recovery
requires 24 consecutive steps below 55% of the working budget; frequency rises
one level at a time before worker count is reduced. Setting `adaptive` to
`false` keeps the configured frequency and selected worker count fixed.

Catch-up work is bounded by `max_catch_up_steps`. Unprocessed simulation time
remains in the accumulator and is reported as debt; no physics time is silently
dropped. If the `Duration` accumulator would overflow, `advance` returns an
error without changing the stored debt.
