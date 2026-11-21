# Spec Delta

## Purpose

Allow an application to advance a physics world on its own schedule and inspect coherent committed results without an autonomous service.

## ADDED Requirements

### Requirement: One explicit blocking step
The host-driven API SHALL perform exactly one blocking core step per successful update for the supplied duration in seconds. It SHALL NOT schedule future steps, accumulate time, or subdivide a host update into multiple core steps.

#### Scenario: Ordinary synchronous main
- **WHEN** a host without a Tokio runtime supplies two different valid durations in successive calls
- **THEN** each call completes exactly one step and the resulting world advances according to those durations

### Requirement: Committed results and failure atomicity
A successful update SHALL provide access to body states, collision and joint-break events, contact impulses and diagnostics of the committed step. A failed update SHALL return the underlying error without publishing a new result or partially committing the step. Existing `simulate` and `fetch_results` behavior SHALL remain compatible.

#### Scenario: A step fails after prior success
- **WHEN** the host encounters a validation or physics failure after a committed update
- **THEN** the failed update returns an error and the prior committed state and results remain available

### Requirement: Optional independent parallel execution
With `parallel`, the host SHALL be able to use a privately owned Rayon pool or supply an existing pool without requiring Tokio. Sequential and parallel execution SHALL satisfy the core's existing deterministic result contract. Dropping a system SHALL release its resources without shutting down a pool still owned by the application.

#### Scenario: Shared application pool
- **WHEN** independent systems use the same app-owned pool for blocking updates
- **THEN** each advances its own world, returns only after its work completes, and dropping either does not prevent the host from reusing the pool

### Requirement: Autonomous runtime compatibility
The existing asynchronous `Simulation` SHALL retain its behavior and public API when the host-driven API is added.

#### Scenario: Existing service application
- **WHEN** an existing consumer runs with the `service` feature
- **THEN** its autonomous scheduling, commands and snapshot behavior remain compatible
