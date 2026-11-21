# Spec Delta

## ADDED Requirements

### Requirement: Hull exposure follows connected geometry
An opt-in hull evaluation SHALL use the exposed geometry of the complete connected rigid body or assembly and SHALL NOT load internal or fully shielded surfaces as independently exposed hulls within the supported geometry domain.

#### Scenario: A part is enclosed by another part
- **WHEN** a supported rigid assembly contains a fully enclosed part
- **THEN** that part contributes no direct hull pressure and is not counted as a separate exposed body

#### Scenario: A single hull is represented as an equivalent assembly
- **WHEN** two supported representations have equivalent union geometry and mass properties
- **THEN** their total hull wrenches agree within a documented discretization tolerance

### Requirement: Distributed loads and model domain
The hull evaluation SHALL account for local relative airflow including angular velocity, report force and moment about the current COM, and attribute contributions to their owners without double application. It SHALL enforce explicit model and geometry limits with typed errors, without silently substituting another model.

#### Scenario: Rotation with stationary COM in still air
- **WHEN** supported exposed surfaces move relative to the air solely through body rotation
- **THEN** their local flow is evaluated and each nonzero force contributes its corresponding COM moment

#### Scenario: Invalid local conditions or exhausted geometry budget
- **WHEN** an evaluation exceeds its documented domain or work limit
- **THEN** it returns a typed cause and commits neither a partial wrench nor a partially integrated step

#### Scenario: Owner wrenches are reduced to the body reference
- **WHEN** owner contributions are shifted to the current COM and summed
- **THEN** they reproduce the committed hull wrench within documented numeric tolerance

### Requirement: Explicit surfaces do not duplicate hull loads
The hull model SHALL compose with controllable wings while excluding any surface represented by the wing load model from hull load integration; exclusion from load integration SHALL NOT by itself remove a surface from shielding geometry.

#### Scenario: Wing represented in hull geometry
- **WHEN** an exposed controllable surface is explicitly associated with hull geometry
- **THEN** the wing contributes its configured aerodynamic loads once and its geometry remains available for occlusion

### Requirement: Geometry revisions and step-path parity
The world SHALL evaluate each committed hull load from a compatible geometry revision and owner binding, following explicit async pending policy and without synchronous cooking inside the physics step. Equivalent ordinary and sharded steps SHALL agree on committed state and failures.

#### Scenario: Assembly splits while cooking
- **WHEN** a result arrives for a previous assembly topology
- **THEN** it cannot bind or apply stale owner loads to the resulting bodies

### Requirement: Reproducible model acceptance
The change SHALL provide analytical and numerical regression evidence for supported hull behavior and SHALL document which flight-viewer cases are synthetic demonstrations rather than physical validation.

#### Scenario: Acceptance of a supported model envelope
- **WHEN** the hull model is enabled for a documented envelope
- **THEN** acceptance includes symmetry, COM moment, geometry and timestep refinement, and repeatable simulated craft cases within that envelope
