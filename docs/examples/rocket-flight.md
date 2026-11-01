# Two-stage rocket flight example

Run the real-time Tokio runtime example with:

```powershell
cargo run --release -p kapitsa --example rocket_flight -- --warp=1
```

Use `--no-ansi` for terminals that do not support cursor clearing. The accepted
warp values are `0.1`, `1`, `10`, `100`, and `1000`; the display reports both
the requested multiplier and the measured runtime rate. The stage controller
observes published snapshots every 25 ms of wall time, so high warp can move
the mission past a control boundary before the next update. Use `1x` or `10x`
for the complete separation and impact demonstration.

The headless mission used by the benchmark shares the same fixed-step rate,
gravity, masses, thrust, burn schedule, and separation impulse. It runs without
wall-clock waits or terminal output. The planet is stationary and uses the
central point-mass field. Before separation the two stages are two colliders on
one rigid body. Separation gives the lower stage its own body and velocity,
updates the upper stage mass and engine, and reattaches each collider to its
stage. Both bodies are removed after confirmed planet impact.

Fuel is represented by finite, fixed-duration engine burns and a linear display
estimate. The stage masses remain constant during each burn; propellant mass
flow and changing center of mass are not modeled. Aerodynamics, joints,
rotation, and thermal effects are also outside this example.
