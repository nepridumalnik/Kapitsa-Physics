//! Aerodynamics task entry point. No physical model is enabled yet.

use crate::world::PhysicsWorld;

/// A step-local placeholder; deliberately contributes no forces or state changes.
#[derive(Clone, Copy, Debug, Default)]
pub struct AerodynamicsTask;

impl AerodynamicsTask {
    pub fn execute(self, _world: &PhysicsWorld) {}
}
