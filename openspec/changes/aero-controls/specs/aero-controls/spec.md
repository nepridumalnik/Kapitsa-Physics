# Aero control surfaces

## ADDED Requirements

### Requirement: Deflect wing controls at runtime without recreation

`WingControl` deflection SHALL be mutable at runtime without recreating
`WingSurface`. A deflection change SHALL NOT invalidate visibility preparation
or rebuild geometry, and SHALL perform no steady-state per-step allocation.

#### Scenario: Runtime deflection takes effect without rebuild

- **WHEN** a wing's control deflection changes at runtime
- **THEN** the next step uses the new deflection without recreating the wing
  surface, rebuilding geometry or invalidating visibility preparation

#### Scenario: Neutral restores baseline loads

- **WHEN** a deflected surface returns to neutral
- **THEN** its force and moment match the undeflected baseline

### Requirement: Validate and apply control commands atomically

`PhysicsClient` commands SHALL drive individual rudders and flaps. Deflection
SHALL be bounded by configured angle limits; invalid, non-finite or
out-of-range values and stale or foreign part/wing identities SHALL be rejected
without mutation. Batched commands SHALL apply atomically.

#### Scenario: Out-of-range deflection is rejected

- **WHEN** a command requests a deflection beyond the configured limit or a
  non-finite value
- **THEN** the command is rejected and the wing state is unchanged

#### Scenario: Stale identity is rejected without mutation

- **WHEN** a command targets a stale or foreign part/wing identity
- **THEN** it is rejected without mutating any wing, and a batch containing it
  applies none of its commands

### Requirement: Compute correct loads and expose diagnostics

Deflected surfaces SHALL produce correct force and moment about the body COM
using the existing full-angle polars, including reverse flow, in both world
step paths. Diagnostics SHALL expose per-wing wrenches consistent with the
applied loads.

#### Scenario: Deflection signs hold across Mach regimes

- **WHEN** representative deflections are evaluated at Mach 0.5, 2 and 8
  across the supported polar domains
- **THEN** force and moment signs and magnitudes match the polar prediction

#### Scenario: Reverse flow uses the full-angle polar

- **WHEN** the relative flow reverses on a deflected surface
- **THEN** the full-angle polar supplies the load in both ordinary and sharded
  step paths

#### Scenario: Diagnostics match applied loads

- **WHEN** per-wing wrench diagnostics are read after a step
- **THEN** they sum to the aerodynamic load applied to the body
