# Proposal

## Why

Ferram Aerospace Research (FAR), a Kerbal Space Program mod, is the gameplay reference: the assembled vehicle's shape and shielding should matter more than an independent drag approximation for each part. Kapitsa already cooks and shares geometry; this change focuses on turning that geometry into coherent, testable loads for simulated craft.

## What Changes

- Add an opt-in geometry-based hull load path for single rigid bodies and connected rigid assemblies using the existing prepared geometry and cooking lifecycle.
- Define distributed hull loads, COM moment and owner attribution, and composition with controllable surfaces without duplicate load contributions.
- Deliver one explicitly bounded model regime first, with analytical and numerical checks before adding game-flight demonstrations.
- Exclude FAR compatibility, CFD, automatic regime blending, heating, and claims of real-flight or all-Mach accuracy.

## Capabilities

### New Capabilities

- `voxel-hull-aerodynamics`: Exposure-aware hull loads and their integration with existing aerodynamic surfaces and geometry lifecycle.

### Modified Capabilities

None.

## Impact

Core aerodynamic evaluation and world stepping, existing async shape binding, flight viewer fixtures, aerodynamic documentation, acceptance evidence, and public API baselines. No dependency on snapshot interpolation.
