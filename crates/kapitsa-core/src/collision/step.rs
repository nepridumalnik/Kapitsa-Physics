use crate::{aabb::Aabb, rigid_body::BodyState, world::BodyId};

use super::CollisionEvent;

/// Scratch state for a transactional world step.
pub(crate) struct StepContext {
    pub(crate) predicted_states: Vec<(BodyId, BodyState)>,
    pub(crate) spatial_proxies: Vec<(crate::world::ColliderId, Aabb)>,
    pub(crate) events: Vec<CollisionEvent>,
}

impl StepContext {
    pub(crate) const fn new() -> Self {
        Self {
            predicted_states: Vec::new(),
            spatial_proxies: Vec::new(),
            events: Vec::new(),
        }
    }

    pub(crate) fn clear(&mut self) {
        self.predicted_states.clear();
        self.spatial_proxies.clear();
        self.events.clear();
    }
}
