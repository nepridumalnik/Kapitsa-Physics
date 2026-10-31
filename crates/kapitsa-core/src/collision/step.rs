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
    pub(crate) ccd_processed_pairs: Vec<super::CollisionPair>,
    pub(crate) ccd_processed_time: f64,
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
            ccd_processed_pairs: Vec::new(),
            ccd_processed_time: 0.0,
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
        self.ccd_processed_pairs.clear();
        self.ccd_processed_time = 0.0;
        self.solver_manifolds.clear();
        self.interactions.clear();
        self.events.clear();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        [
            self.predicted_states.capacity() * size_of::<(BodyId, BodyState)>(),
            self.body_indices.capacity() * size_of::<Option<(BodyId, usize)>>(),
            self.solver_body_indices.capacity() * size_of::<Option<(BodyId, usize)>>(),
            self.body_types.capacity() * size_of::<BodyType>(),
            self.colliders.capacity() * size_of::<CollisionProxy>(),
            self.spatial_proxies.capacity() * size_of::<(ColliderId, Aabb)>(),
            self.manifolds.capacity() * size_of::<ContactManifold>(),
            self.solver_bodies.capacity() * size_of::<SolverBody>(),
            self.solver_body_predictions.capacity() * size_of::<usize>(),
            self.ccd_states.capacity() * size_of::<BodyState>(),
            self.ccd_elapsed.capacity() * size_of::<f64>(),
            self.ccd_impact_counts.capacity() * size_of::<u32>(),
            self.ccd_processed_pairs.capacity() * size_of::<super::CollisionPair>(),
            self.solver_manifolds.capacity() * size_of::<SolverManifold>(),
            self.interactions.capacity() * size_of::<CollisionInteraction>(),
            self.events.capacity() * size_of::<CollisionEvent>(),
        ]
        .into_iter()
        .fold(0_usize, usize::saturating_add)
    }
}
