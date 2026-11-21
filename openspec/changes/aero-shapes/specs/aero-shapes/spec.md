# Aero shapes

## ADDED Requirements

### Requirement: Share one immutable prepared shape between bodies

A `CookedShape` SHALL be an immutable prepared vessel shape identified by
`ShapeId` in a registry, independent of physics bodies. Shape owners SHALL be
addressed by owner slots, not `PartId` or `BodyId`. A `ShapeBinding` SHALL map
`OwnerSlot -> PartId` with per-part transforms for one body.

#### Scenario: Two bodies share one cooked shape

- **WHEN** two bodies bind the same `ShapeId` with different
  `OwnerSlot -> PartId` mappings
- **THEN** voxelization runs once and both bodies report correct per-part
  wrenches through their own bindings

#### Scenario: One vessel's geometry edit leaves a shared shape intact

- **WHEN** one body's part geometry changes while another body uses the same
  registered shape
- **THEN** the other body's prepared shape and cache ticket remain valid

### Requirement: Install and replace shapes atomically with revision checks

Shape installation and replacement SHALL validate identity and revision and
apply atomically. Stale revisions and foreign shape identities SHALL be
rejected without partial application, preserving the previous shape and
motion state.

#### Scenario: Stale revision is rejected

- **WHEN** a cook result carrying an outdated revision or a foreign identity
  is installed
- **THEN** the installation is rejected, the previous shape stays in effect
  and the world state is unchanged

### Requirement: Keep geometry preparation out of the physics step

Installed shapes SHALL NOT be re-voxelized inside the physics step.
Preparation SHALL happen at explicit cook/install boundaries, reusing the
existing `PreparedGeometry`, `GeometryPreparation` and publication machinery
with a shape-identity cache key instead of a parallel cache.

#### Scenario: Steady-state steps do not prepare geometry

- **WHEN** a body with an installed shape steps without geometry or topology
  changes
- **THEN** no voxelization runs and no shape-related heap allocation occurs
  in the step

### Requirement: Preserve load attribution through topology changes

Per-part pressure, wave and wing-shielding loads SHALL keep their ownership
through merge, split and root transfer without double counting, and ordinary
and sharded step paths SHALL agree.

#### Scenario: Merge and split keep part loads

- **WHEN** an assembly with a bound shape merges and then splits
- **THEN** per-part wrenches match the standalone preparation and no load is
  counted twice in either step path
