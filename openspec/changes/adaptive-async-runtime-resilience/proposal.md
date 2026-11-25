# Proposal

## Why

The autonomous Kapitsa service already supports Tokio orchestration, Rayon execution, adaptive time warp, bounded command queues and immutable snapshots. However, a failed integration step exits the service loop; a physical step estimate below the configured minimum can hold forever; CPU pacing has a nonzero floor that may accumulate unbounded debt; and the step estimate is currently selected before queued mutations are applied. A real-time application needs to stay responsive and make physically valid forward progress through ordinary overload and recoverable numerical failures, without implementing its own solver retry loop.

Baseline: master at 826832c3568d6eb2856f1b91a5a85de3b6ebe22b. Read AGENTS.md first. This change is Kapitsa-only; no application-specific types, terrain, graphics, orbital mechanics or changes to other repositories.

## What Changes

- Add an opt-in resilient recovery policy to the autonomous service. Preserve the legacy fixed-step and fail-fast contracts by default.
- Distinguish CPU overload, numerical rejection, impossible physical bounds, invalid commands and unrecoverable corruption; use bounded retries with a smaller safe dt only for classified errors.
- Guarantee exactly-once command application and atomic attempted integration; publish only committed physics time, snapshots and events.
- Keep the service, control plane and last committed snapshot available during recoverable failures, overload and explicit no-progress states. Allow corrective commands to resume execution without recreating the world.
- Bound catch-up work, time debt and deferred results; allow achieved simulation speed to fall below the old load floor rather than accumulating an unbounded backlog.
- Reuse the existing asynchronous service, its owned/shared Rayon pool and host-provided Tokio runtime; add a documented dedicated-runtime example and deterministic integration fixtures.
- Make step boundaries, time horizon, command results and typed recovery diagnostics usable by an independent host application.

## Capabilities

### New Capabilities

- `resilient-async-runtime`: bounded adaptive numerical recovery, transactional publication, overload pacing, responsive asynchronous lifecycle and host integration.

### Modified Capabilities

None. The existing unarchived `runtime-explicit-step` change defines the exact-one-step synchronous API. Do not change that contract or edit that change. The new behavior is opt-in; existing service consumers retain fail-fast behavior unless they select resilience.

## Impact

Primary crate: `kapitsa` (runtime configuration, scheduler, service, metrics, marshalling and tests). Modify `kapitsa-core` only where tests demonstrate incomplete transactional guarantees or where a typed error is missing. No Tokio dependency for synchronous builds. Public API baseline, docs/runtime.md and examples need updating. No new runtime dependencies expected.

## Success criteria

A supported finite world with positive requested speed, sufficient horizon and a safe positive representable step makes progress under overload and transient numerical faults without a service restart. When no physically valid step is available, report an explicit blocked state, remain responsive, and resume after corrective input. No fake clock advancement, dropped collisions, fabricated forces, unlimited retries or blanket swallowing of errors. Absolute progress for arbitrary invalid states, permanently hung workers or exhausted system resources is not a physically meaningful guarantee.
