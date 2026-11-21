# Synthetic hull yaw fixtures (OpenSpec 4.1–4.2)

```sh
cargo run -q -p rocket-flight-viewer -- --hull-fixture-trace > hull-fixture.csv
cargo run -q -p rocket-flight-viewer -- --hull-world-trace > hull-world.csv
cargo test -p rocket-flight-viewer hull_fixture
```

The standalone flag emits 250 data rows without starting the GUI or changing the
normal flight scenario. Plot `yaw_rad` and `mz_nm` against `time_s`, grouping by
`stabilized,cell_m,dt_s`. The CSV includes the initial and final states, all
world-frame force and COM moment components, and conservative local Mach bounds.

## Setup and expected behavior

- Both variants start at the origin with +0.15 rad yaw about world +Z, zero yaw
  rate, and prescribed +X translation at 1574.2 m/s. Uniform still air has
  density 0.2 kg/m³ and sound speed 340 m/s. COM stays at the body origin;
  yaw inertia is prescribed at 100,000 kg·m² in both cases. Translation is
  not integrated; the only state evolved is yaw and yaw rate for 1 s, using
  symplectic Euler without gravity, thrust, or control commands.
- The centered hull is a 2 × 0.5 × 0.5 m cuboid (X, Y, Z). The stabilized
  variant adds an aft 0.25 × 0.25 × 2 m crossbar centered at X = −1.125 m;
  it is passive hull geometry, not a separate wing force. The centered box
  has zero yaw moment by symmetry at zero spin. The aft crossbar exposes
  side area behind the COM that produces a restoring moment for this yaw sign.
- Fixed triangle facets, voxel cell sizes 0.125, 0.0625 and 0.03125 m;
  budgets are 1,048,576 cells and 100 million work units each for geometry
  preparation and patch work. Visibility uses the COM incident direction;
  modified Newtonian triangle quadrature uses local rotational velocity for
  pressure. The prescribed test window is Mach 4.60–4.66: every evaluation
  bounds the local speeds by COM speed ± |yaw rate| × largest vertex radius
  and stops on violation. This is a fixture constraint, not a physical model
  envelope. Across the recorded runs the extreme bounds were 4.6298182 and
  4.6301818.

## Numerical results

At Δt = 0.03125 s all three grids agreed throughout the trace: relative force
difference below 10⁻¹⁰, moment difference below 10⁻¹⁰ × |force| × 2 m, and yaw
difference below 10⁻¹² rad. This is grid insensitivity for these resolved,
fixed box facets, not mesh convergence or a result for thin features/openings.
The bare hull held 0.15 rad at all three timesteps. At the finest grid:

| Δt (s) | Crossbar final yaw (rad) | Crossbar final yaw moment (N·m) |
| ---: | ---: | ---: |
| 0.125 | 0.127462965 | −3044.72 |
| 0.0625 | 0.128677985 | −3102.75 |
| 0.03125 | 0.129281837 | −3131.79 |

Successive final-yaw differences were 0.00121502 and 0.000603852 rad (ratio
0.497), consistent with first-order time integration over this interval.
At the finest grid and timestep the initial yaw moment was approximately
−4211.14 N·m with the crossbar and zero to roundoff without it. This is an
early restoring response, not proof of asymptotic stability.

## Cooked `PhysicsWorld` fixture

The second command emits 118 data rows. Both geometries and initial conditions
are unchanged, but the world now integrates both translation and rotation for
1 s. Mass is 1,000,000 kg and all principal inertias are 100,000 kg·m² to
keep this synthetic trajectory well within the Mach window while making the yaw
response observable. These are prescribed mass properties, not material data.
There are no gravity sources, contacts, thrust or separate wings. A spherical
atmosphere centered at the origin has a 2000 m constant-density reference
radius; the outer profile (1000 m extent and exponential scale height) is not
sampled. Every sampled state checks the radius and conservative local Mach
bounds using actual COM and angular speeds.

The primary mesh is cooked before the first `PhysicsWorld::step` using the
world's fixed 0.125 m grid. This path evaluates visibility with local flow and
publishes the standalone body's hull wrench. CSV `incoming_*` columns belong
to the step ending on that row; they are blank at time zero. The test checks
that the published linear and angular impulses match the actual state changes.
The bare hull held 0.15 rad to roundoff; the crossbar reduced yaw monotonically:

| Δt (s) | Crossbar final yaw (rad) | Last step's yaw moment (N·m) |
| ---: | ---: | ---: |
| 0.125 | 0.129684479 | −3378.30 |
| 0.0625 | 0.129778704 | −3270.69 |
| 0.03125 | 0.129830061 | −3215.90 |

Successive yaw differences have ratio 0.545, consistent with first-order
behavior over this interval. Grid refinement remains a separate diagnostic:
the world cook does not expose a resolution setting. The two traces are not
expected to agree numerically: one prescribes speed and uses COM-direction
visibility, while the other integrates velocity and uses local-flow visibility.

## Acceptance gap

This is numerical regression and a game-viewer diagnostic, not independent
physical validation or a validated gameplay trajectory. There is no accepted
flow/geometry envelope for either cuboid configuration: no independent
force/moment reference establishes accuracy for these bluff bodies and their
interference, and the pressure-only closure omits base/wake forces, skin
friction and other potentially relevant effects. The evaluator's Mach guard
does not establish applicability at Mach 4.63. Task 1.2 owns any supported
physical envelope; the synthetic window here does not stand in for it. The
stepped fixture covers a pre-cooked standalone body, not async geometry
binding, assembly lifecycle, or separate wing attribution. Consequently no launch,
powered maneuver, descent, or landing phase is accepted; in particular,
low-speed and transonic parts of those phases are not exercised at all.
Normal viewer flight and graphical playback were not used to collect these results.
