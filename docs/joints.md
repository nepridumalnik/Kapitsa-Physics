# Joint constraint solving

Kapitsa solves joints and contacts in the same island using static dispatch
through `Joint`. Each solver iteration processes contact rows, then visits
joints in order. Before each joint is solved, its connector frames and rows are
rebuilt from the current body poses; the row corrections are applied
immediately. A later joint that shares a body therefore sees the changes made
by earlier rows in that pass.

Position rows use XPBD multipliers. A row's warm-start identity combines its
owning generational `JointId` with a compact semantic key for its constraint
kind and axis. This keeps multipliers attached to the same degree of freedom
when limits or drives are enabled or disabled. Accumulated position and
velocity values reset at the start of each physical step and are retained
across solver iterations only while the row identity matches.

Fixed joints correct connector position during the position phase and lock
relative connector velocity in the velocity phase. They do not apply a
Baumgarte velocity target based on position error: the additional target
duplicated position projection and increased energy in large trees. The solver
collects break reactions after its passes and publishes break events only
after the world step succeeds.

## Chaos comparison

The relevant reference is the Unreal Engine Chaos source at commit
[`260bb2e1`](https://github.com/nepridumalnik/UnrealEngine/tree/260bb2e1c5610b31c63a36206eedd289409c5f11/Engine/Source/Runtime/Experimental/Chaos):

- [`PBDJointContainerSolver.cpp`](https://github.com/nepridumalnik/UnrealEngine/blob/260bb2e1c5610b31c63a36206eedd289409c5f11/Engine/Source/Runtime/Experimental/Chaos/Private/Chaos/Joint/PBDJointContainerSolver.cpp)
- [`PBDJointSolverGaussSeidel.cpp`](https://github.com/nepridumalnik/UnrealEngine/blob/260bb2e1c5610b31c63a36206eedd289409c5f11/Engine/Source/Runtime/Experimental/Chaos/Private/Chaos/Joint/PBDJointSolverGaussSeidel.cpp)
- [`PBDJointCachedSolverGaussSeidel.cpp`](https://github.com/nepridumalnik/UnrealEngine/blob/260bb2e1c5610b31c63a36206eedd289409c5f11/Engine/Source/Runtime/Experimental/Chaos/Private/Chaos/Joint/PBDJointCachedSolverGaussSeidel.cpp)
- [`PBDJointConstraintTypes.h`](https://github.com/nepridumalnik/UnrealEngine/blob/260bb2e1c5610b31c63a36206eedd289409c5f11/Engine/Source/Runtime/Experimental/Chaos/Public/Chaos/Joint/PBDJointConstraintTypes.h)

The adopted idea is to refresh the current joint immediately before solving it
in the sequential pass. Kapitsa retains its own XPBD rows, contact/joint island,
and phase ordering. Chaos's optional projection and shock propagation were
reviewed but not added: the Kapitsa stress and mass-ratio fixtures converge
within their stated bounds without those mechanisms, and no repeatable case
showed that an extra pass would improve the result without risking contacts or
adding energy.

## Regression coverage

Core tests cover hub-and-leaf assemblies, chains, branching trees, reversed
body order, static connections, multiple steps, mass ratios through 1:10,000,
and off-center force. The runtime test also builds a 120-part two-stage rocket,
changes fuel mass, separates its stages, exercises pad contact, and compares
adaptive 1x, 10x, and 100x steps against a fixed reference. It checks finite
state, anchor and frame error, per-part integration error, and same-input
repeatability. The fixture's bounds and Release measurements are recorded in
[`benchmarks/joints-and-flight.md`](../benchmarks/joints-and-flight.md).

The modular rocket is a focused solver regression, not a complete vehicle
model: it uses colliders on the stage roots, and does not model aerodynamics,
flexible joints, or long-duration flight. Passing this fixture does not imply
general stability for all large joint graphs.
