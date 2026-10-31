mod error;
mod id;

pub use error::{StepError, WorldError};
pub use id::{BodyId, ColliderId};

use crate::{
    collider::Collider,
    containers::DensePool,
    rigid_body::{BodyState, RigidBody, predict},
};

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
    step_scratch: Vec<(BodyId, BodyState)>,
}

/// Opaque copied body data used to predict and atomically commit a world step.
#[doc(hidden)]
#[derive(Clone, Copy, Debug)]
pub struct BodyStep {
    body_id: BodyId,
    source: RigidBody,
    predicted: Option<BodyState>,
}

impl BodyStep {
    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    pub fn integrate(&mut self, dt: f64) -> Result<(), StepError> {
        self.predicted = None;
        self.predicted =
            Some(
                predict(&self.source, dt).map_err(|cause| StepError::BodyIntegration {
                    body_id: self.body_id,
                    cause,
                })?,
            );
        Ok(())
    }
}

impl PhysicsWorld {
    pub const fn new() -> Self {
        Self {
            bodies: DensePool::new(),
            colliders: DensePool::new(),
            step_scratch: Vec::new(),
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

    /// Advances every body by one fixed timestep, atomically committing the results.
    pub fn step(&mut self, dt: f64) -> Result<(), StepError> {
        self.step_scratch.clear();
        if !dt.is_finite() || dt <= 0.0 {
            return Err(StepError::InvalidDeltaTime);
        }

        for (dense_id, entry) in self.bodies.iter() {
            let body_id = BodyId(dense_id);
            match predict(&entry.body, dt) {
                Ok(state) => self.step_scratch.push((body_id, state)),
                Err(cause) => {
                    self.step_scratch.clear();
                    return Err(StepError::BodyIntegration { body_id, cause });
                }
            }
        }

        for ((dense_id, entry), (body_id, state)) in self
            .bodies
            .iter_mut()
            .zip(self.step_scratch.iter().copied())
        {
            debug_assert_eq!(BodyId(dense_id), body_id);
            entry.body.commit_integrated_state(state);
        }
        self.step_scratch.clear();
        Ok(())
    }

    /// Fills balanced, ordered body snapshots into reusable shard buffers.
    #[doc(hidden)]
    pub fn fill_step_shards(&self, shards: &mut [Vec<BodyStep>]) -> Result<(), StepError> {
        if shards.is_empty() {
            return Err(StepError::InvalidStepResults);
        }
        for shard in shards.iter_mut() {
            shard.clear();
        }

        let base_size = self.body_count() / shards.len();
        let remainder = self.body_count() % shards.len();
        let mut shard_index = 0;
        let mut shard_remaining = base_size + usize::from(remainder > 0);

        for (dense_id, entry) in self.bodies.iter() {
            while shard_remaining == 0 {
                shard_index += 1;
                shard_remaining = base_size + usize::from(shard_index < remainder);
            }
            shards[shard_index].push(BodyStep {
                body_id: BodyId(dense_id),
                source: entry.body,
                predicted: None,
            });
            shard_remaining -= 1;
        }
        Ok(())
    }

    /// Validates all shard results before committing any body state.
    #[doc(hidden)]
    pub fn commit_step_shards(&mut self, shards: &[Vec<BodyStep>]) -> Result<(), StepError> {
        let mut steps = shards.iter().flatten();
        for (dense_id, entry) in self.bodies.iter() {
            let Some(step) = steps.next() else {
                return Err(StepError::InvalidStepResults);
            };
            if step.body_id != BodyId(dense_id)
                || step.source != entry.body
                || step.predicted.is_none()
            {
                return Err(StepError::InvalidStepResults);
            }
        }
        if steps.next().is_some() {
            return Err(StepError::InvalidStepResults);
        }

        for ((_, entry), step) in self.bodies.iter_mut().zip(shards.iter().flatten()) {
            if let Some(state) = step.predicted {
                entry.body.commit_integrated_state(state);
            }
        }
        Ok(())
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
