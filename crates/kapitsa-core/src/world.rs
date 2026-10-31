mod error;
mod id;

pub use error::WorldError;
pub use id::{BodyId, ColliderId};

use crate::{collider::Collider, containers::DensePool, rigid_body::RigidBody};

struct BodyEntry {
    body: RigidBody,
    first_collider: Option<ColliderId>,
}

struct ColliderEntry {
    collider: Collider,
    body: BodyId,
    prev: Option<ColliderId>,
    next: Option<ColliderId>,
}

/// Owns rigid bodies and colliders and assigns IDs scoped to this world instance.
pub struct PhysicsWorld {
    bodies: DensePool<BodyEntry>,
    colliders: DensePool<ColliderEntry>,
}

impl PhysicsWorld {
    pub const fn new() -> Self {
        Self {
            bodies: DensePool::new(),
            colliders: DensePool::new(),
        }
    }

    pub fn add_body(&mut self, body: RigidBody) -> Result<BodyId, WorldError> {
        self.bodies
            .insert(BodyEntry {
                body,
                first_collider: None,
            })
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

    pub fn add_collider(
        &mut self,
        body: BodyId,
        collider: Collider,
    ) -> Result<ColliderId, WorldError> {
        let first = self
            .bodies
            .get(body.0)
            .ok_or(WorldError::InvalidBodyId)?
            .first_collider;
        if first.is_some_and(|id| self.colliders.get(id.0).is_none()) {
            return Err(WorldError::InvalidColliderId);
        }

        let id = self
            .colliders
            .insert(ColliderEntry {
                collider,
                body,
                prev: None,
                next: first,
            })
            .map(ColliderId)
            .map_err(WorldError::from)?;

        if let Some(first) = first
            && let Some(entry) = self.colliders.get_mut(first.0)
        {
            entry.prev = Some(id);
        }
        if let Some(entry) = self.bodies.get_mut(body.0) {
            entry.first_collider = Some(id);
        }
        Ok(id)
    }

    pub fn collider(&self, id: ColliderId) -> Option<&Collider> {
        self.colliders.get(id.0).map(|entry| &entry.collider)
    }

    pub fn collider_mut(&mut self, id: ColliderId) -> Option<&mut Collider> {
        self.colliders
            .get_mut(id.0)
            .map(|entry| &mut entry.collider)
    }

    pub fn collider_body(&self, id: ColliderId) -> Option<BodyId> {
        self.colliders.get(id.0).map(|entry| entry.body)
    }

    pub fn remove_collider(&mut self, id: ColliderId) -> Option<Collider> {
        let (body, prev, next) = {
            let entry = self.colliders.get(id.0)?;
            (entry.body, entry.prev, entry.next)
        };
        let body_entry = self.bodies.get(body.0)?;
        if prev.is_none() && body_entry.first_collider != Some(id) {
            return None;
        }
        if prev.is_some_and(|prev| {
            self.colliders
                .get(prev.0)
                .is_none_or(|entry| entry.body != body || entry.next != Some(id))
        }) || next.is_some_and(|next| {
            self.colliders
                .get(next.0)
                .is_none_or(|entry| entry.body != body || entry.prev != Some(id))
        }) {
            return None;
        }

        let removed = self.colliders.remove(id.0)?;
        if let Some(prev) = prev {
            if let Some(entry) = self.colliders.get_mut(prev.0) {
                entry.next = next;
            }
        } else if let Some(entry) = self.bodies.get_mut(body.0) {
            entry.first_collider = next;
        }
        if let Some(next) = next
            && let Some(entry) = self.colliders.get_mut(next.0)
        {
            entry.prev = prev;
        }
        Some(removed.collider)
    }

    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    pub fn remove_body(&mut self, id: BodyId) -> Option<RigidBody> {
        let mut next = self.bodies.get(id.0)?.first_collider;
        while let Some(collider) = next {
            next = self.colliders.get(collider.0)?.next;
            self.remove_collider(collider)?;
        }
        self.bodies.remove(id.0).map(|entry| entry.body)
    }

    /// Iterates over bodies in dense storage order, which may change after removal.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &RigidBody)> {
        self.bodies
            .iter()
            .map(|(id, entry)| (BodyId(id), &entry.body))
    }

    /// Mutably iterates over bodies in dense storage order, which may change after removal.
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = (BodyId, &mut RigidBody)> {
        self.bodies
            .iter_mut()
            .map(|(id, entry)| (BodyId(id), &mut entry.body))
    }

    /// Iterates over colliders in dense storage order, which may change after removal.
    pub fn colliders(&self) -> impl Iterator<Item = (ColliderId, BodyId, &Collider)> {
        self.colliders
            .iter()
            .map(|(id, entry)| (ColliderId(id), entry.body, &entry.collider))
    }

    /// Iterates over the colliders attached to `body` in linked-list order.
    pub fn colliders_of(
        &self,
        body: BodyId,
    ) -> Option<impl Iterator<Item = (ColliderId, &Collider)> + '_> {
        let mut next = self.bodies.get(body.0)?.first_collider;
        Some(std::iter::from_fn(move || {
            let id = next?;
            let entry = self.colliders.get(id.0)?;
            next = entry.next;
            Some((id, &entry.collider))
        }))
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
