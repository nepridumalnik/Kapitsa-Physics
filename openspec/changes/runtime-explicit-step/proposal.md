# Proposal

## Why

Applications with their own game loop need to advance Kapitsa directly rather than coordinate with an autonomous physics service. The core already supports blocking execution; the public runtime needs a convenient way to own a world and select its executor without Tokio.

## What Changes

- Add a public `PhysicsSystem` that runs exactly one blocking step for a caller-supplied duration.
- Support sequential execution and, with `parallel`, either a system-owned or application-provided Rayon pool.
- Return borrowed committed results on success only; preserve existing world and async APIs.
- Exclude scheduling, automatic subdivision of host ticks, render interpolation and changes to DeltaV.

## Capabilities

### New Capabilities

- `host-driven-physics`: Explicit blocking physics steps with configurable execution and coherent results.

### Modified Capabilities

None.

## Impact

Additive `kapitsa` API, a committed-result accessor in the core, documentation, examples and public API baseline. No new dependencies or async behavior changes.
