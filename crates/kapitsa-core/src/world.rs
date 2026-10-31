mod error;
mod id;

pub use error::WorldError;
pub use id::{BodyId, ColliderId};

use crate::{containers::DensePool, rigid_body::RigidBody};

struct BodyEntry {
    body: RigidBody,
}

/// Owns rigid bodies and colliders and assigns IDs scoped to this world instance.
pub struct PhysicsWorld {
    bodies: DensePool<BodyEntry>,
}

impl PhysicsWorld {
    pub const fn new() -> Self {
        Self {
            bodies: DensePool::new(),
        }
    }

    pub fn add_body(&mut self, body: RigidBody) -> Result<BodyId, WorldError> {
        self.bodies
            .insert(BodyEntry { body })
            .map(BodyId)
            .map_err(WorldError::from)
    }

    pub fn body(&self, id: BodyId) -> Option<&RigidBody> {
        self.bodies.get(id.0).map(|entry| &entry.body)
    }

    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut RigidBody> {
        self.bodies.get_mut(id.0).map(|entry| &mut entry.body)
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "world_tests.rs"]
mod tests;
