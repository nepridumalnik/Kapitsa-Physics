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
- **THEN** the other body's prepared shape stays valid and its evaluation keeps
  using it, and only the edited body falls back to body-local preparation

#### Scenario: Distinct mesh allocations are rejected

- **WHEN** a body binds a shape whose cooked source mesh is structurally equal
  but a different allocation than the body's part mesh
- **THEN** the binding is rejected with `InvalidBinding`, because reused
  occupancy cannot be attributed unambiguously

### Requirement: Install and replace shapes atomically with revision checks

Shape installation and replacement SHALL validate identity and revision and
apply atomically. Stale revisions and foreign shape identities SHALL be
rejected without partial application, preserving the previous shape and
motion state. Replacement SHALL keep the shape identity and advance the
revision, staling existing bindings.

#### Scenario: Stale revision is rejected

- **WHEN** a cook result carrying an outdated revision or a foreign identity
  is installed
- **THEN** the installation is rejected, the previous shape stays in effect
  and the world state is unchanged

#### Scenario: Replacement stales bindings lazily

- **WHEN** a registered shape is replaced
- **THEN** existing bindings fail revision validation on their next
  evaluation and that body falls back to body-local preparation until rebound

### Requirement: Keep geometry preparation out of the physics step

Installed shapes SHALL NOT be re-voxelized inside the physics step.
Preparation SHALL happen at explicit cook/install boundaries, reusing the
existing `PreparedGeometry` and `GeometryPreparation` machinery through the
registry-owned cooked geometry instead of a parallel cache.

#### Scenario: Steady-state steps do not prepare geometry

- **WHEN** a body with an installed shape steps without geometry or topology
  changes
- **THEN** no body-local voxelization or shape-storage allocation runs in the
  step; only the temporary instance/patch buffers used by both paths are
  allocated

### Requirement: Preserve load attribution through topology changes

Per-part pressure, wave and wing-shielding loads SHALL keep their ownership
through merge, split and root transfer without double counting, and ordinary
and sharded step paths SHALL agree.

#### Scenario: Merge and split keep part loads

- **WHEN** an assembly with a bound shape merges and then splits
- **THEN** per-part wrenches match the standalone preparation and no load is
  counted twice in either step path

#### Scenario: Topology change stales the binding

- **WHEN** membership changes so the bound part set no longer matches the
  cluster (merge adds parts, or a part is destroyed)
- **THEN** the binding is dropped before the next ordinary step and pressure
  is applied through body-local preparation; a split that leaves exactly the
  bound parts keeps sharing valid
