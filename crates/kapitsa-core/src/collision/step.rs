use crate::{
    aabb::Aabb,
    rigid_body::{BodyState, BodyType},
    world::{BodyId, ColliderId},
};

use super::{
    CollisionEvent, CollisionInteraction, CollisionProxy,
    manifold::ContactManifold,
    solver::{SolverBody, SolverManifold},
};

/// Scratch state for a transactional world step.
pub(crate) struct StepContext {
    pub(crate) predicted_states: Vec<(BodyId, BodyState)>,
    pub(crate) body_indices: Vec<Option<(BodyId, usize)>>,
    pub(crate) solver_body_indices: Vec<Option<(BodyId, usize)>>,
    pub(crate) body_types: Vec<BodyType>,
    pub(crate) colliders: Vec<CollisionProxy>,
    pub(crate) spatial_proxies: Vec<(ColliderId, Aabb)>,
    pub(crate) manifolds: Vec<ContactManifold>,
    pub(crate) solver_bodies: Vec<SolverBody>,
    pub(crate) solver_body_predictions: Vec<usize>,
    pub(crate) ccd_states: Vec<BodyState>,
    pub(crate) ccd_elapsed: Vec<f64>,
    pub(crate) ccd_impact_counts: Vec<u32>,
    pub(crate) solver_manifolds: Vec<SolverManifold>,
    pub(crate) interactions: Vec<CollisionInteraction>,
    pub(crate) events: Vec<CollisionEvent>,
}

impl StepContext {
    pub(crate) const fn new() -> Self {
        Self {
            predicted_states: Vec::new(),
            body_indices: Vec::new(),
            solver_body_indices: Vec::new(),
            body_types: Vec::new(),
            colliders: Vec::new(),
            spatial_proxies: Vec::new(),
            manifolds: Vec::new(),
            solver_bodies: Vec::new(),
            solver_body_predictions: Vec::new(),
            ccd_states: Vec::new(),
            ccd_elapsed: Vec::new(),
            ccd_impact_counts: Vec::new(),
            solver_manifolds: Vec::new(),
            interactions: Vec::new(),
            events: Vec::new(),
        }
    }

    pub(crate) fn clear(&mut self) {
        self.predicted_states.clear();
        self.body_indices.clear();
        self.solver_body_indices.clear();
        self.body_types.clear();
        self.colliders.clear();
        self.spatial_proxies.clear();
        self.manifolds.clear();
        self.solver_bodies.clear();
        self.solver_body_predictions.clear();
        self.ccd_states.clear();
        self.ccd_elapsed.clear();
        self.ccd_impact_counts.clear();
        self.solver_manifolds.clear();
        self.interactions.clear();
        self.events.clear();
    }
}
