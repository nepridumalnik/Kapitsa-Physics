use crate::{
    aabb::Aabb,
    rigid_body::BodyState,
    world::{BodyId, ColliderId},
};

use super::{CollisionEvent, CollisionInteraction, CollisionProxy};

/// Scratch state for a transactional world step.
pub(crate) struct StepContext {
    pub(crate) predicted_states: Vec<(BodyId, BodyState)>,
    pub(crate) colliders: Vec<CollisionProxy>,
    pub(crate) spatial_proxies: Vec<(ColliderId, Aabb)>,
    pub(crate) interactions: Vec<CollisionInteraction>,
    pub(crate) events: Vec<CollisionEvent>,
}

impl StepContext {
    pub(crate) const fn new() -> Self {
        Self {
            predicted_states: Vec::new(),
            colliders: Vec::new(),
            spatial_proxies: Vec::new(),
            interactions: Vec::new(),
            events: Vec::new(),
        }
    }

    pub(crate) fn clear(&mut self) {
        self.predicted_states.clear();
        self.colliders.clear();
        self.spatial_proxies.clear();
        self.interactions.clear();
        self.events.clear();
    }
}
