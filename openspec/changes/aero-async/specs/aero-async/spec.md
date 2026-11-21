# Aero async preparation

## ADDED Requirements

### Requirement: Cook shapes outside the physics step

Shape cooking SHALL run on worker threads outside the physics step. A vessel
construction change SHALL schedule preparation through a task queue instead of
voxelizing synchronously inside the step. The queue SHALL cancel or reject
results made stale by a later geometry or topology change.

#### Scenario: Construction change does not voxelize in the step

- **WHEN** a vessel's geometry or topology changes and a step runs before the
  cook finishes
- **THEN** no synchronous voxelization occurs inside that step

#### Scenario: Superseded cook result is discarded

- **WHEN** geometry changes again while a cook is in flight
- **THEN** the earlier result is cancelled or rejected by identity/revision
  and never installed

### Requirement: Expose explicit preparation states

A shape SHALL expose `Ready`, `Cooking` and `Failed` states with completion
notification. A failed cook SHALL preserve the last successful shape and
report the error cause.

#### Scenario: Failed cook keeps the last good shape

- **WHEN** a cook fails
- **THEN** the state is `Failed`, the last successful shape remains in effect
  and the failure cause is preserved

### Requirement: Define pending-step behavior by policy

While a shape is not ready, step behavior SHALL follow the selected
`PendingAero` policy: `KeepPrevious` for topology-preserving edits,
`ZeroForce` after a topology change until the new shape installs, and
`BlockStep` for offline use. After a topology change a stale shape SHALL NOT
be used.

#### Scenario: Topology change contributes no load until install

- **WHEN** topology changes under `ZeroForce` and the new shape is still
  `Cooking`
- **THEN** the affected body contributes no aerodynamic load until the new
  shape installs atomically

#### Scenario: KeepPrevious does not survive a topology change

- **WHEN** topology changes under `KeepPrevious`
- **THEN** the stale shape is not used; the declared topology-change behavior
  applies instead

### Requirement: Integrate with PhysicsClient and preserve determinism

`PhysicsClient` SHALL submit geometry/topology changes, observe preparation
state and receive completion or failure. Install and abort SHALL be atomic,
and the committed state SHALL remain deterministic and identical between
ordinary and sharded step paths.

#### Scenario: Concurrent edits publish atomically

- **WHEN** concurrent geometry edits and cancellations race with publication
- **THEN** only one revision installs atomically, motion state is unchanged on
  rejection and both step paths agree on the committed result
