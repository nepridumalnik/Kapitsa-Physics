use std::collections::{HashMap, HashSet};

use crate::{
    aabb::Aabb,
    rigid_body::{BodyState, BodyType},
    world::{BodyId, ColliderId},
};

use super::{
    CollisionEvent, CollisionInteraction, CollisionPair, CollisionProxy,
    ccd::CcdCandidateToi,
    manifold::ContactManifold,
    solver::{SolverBody, SolverManifold},
};

/// Scratch state for a transactional world step.
pub(crate) struct StepContext {
    pub(crate) predicted_states: Vec<(BodyId, BodyState)>,
    pub(crate) body_indices: Vec<Option<(BodyId, usize)>>,
    pub(crate) body_indices_ready: bool,
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
    pub(crate) ccd_body_types: Vec<BodyType>,
    pub(crate) ccd_continuous_bodies: Vec<bool>,
    pub(crate) ccd_candidate_bodies: Vec<(usize, usize)>,
    pub(crate) ccd_candidate_pairs: Vec<super::CollisionPair>,
    pub(crate) ccd_candidate_pair_set: Option<HashSet<super::CollisionPair>>,
    pub(crate) ccd_toi_cache: Vec<CcdCandidateToi>,
    pub(crate) ccd_resweep_candidates: Vec<usize>,
    pub(crate) ccd_query_results: Vec<ColliderId>,
    pub(crate) ccd_processed_pairs: Vec<super::CollisionPair>,
    pub(crate) ccd_processed_time: f64,
    #[cfg(feature = "bench-internals")]
    pub(crate) ccd_toi_seen_pairs: Option<HashSet<super::CollisionPair>>,
    pub(crate) solver_manifolds: Vec<SolverManifold>,
    pub(crate) interactions: Vec<CollisionInteraction>,
    /// Created on first use so the owning PhysicsWorld can remain const-constructible.
    pub(crate) interaction_indices: Option<HashMap<CollisionPair, usize>>,
    pub(crate) events: Vec<CollisionEvent>,
}

impl StepContext {
    pub(crate) const fn new() -> Self {
        Self {
            predicted_states: Vec::new(),
            body_indices: Vec::new(),
            body_indices_ready: false,
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
            ccd_body_types: Vec::new(),
            ccd_continuous_bodies: Vec::new(),
            ccd_candidate_bodies: Vec::new(),
            ccd_candidate_pairs: Vec::new(),
            ccd_candidate_pair_set: None,
            ccd_toi_cache: Vec::new(),
            ccd_resweep_candidates: Vec::new(),
            ccd_query_results: Vec::new(),
            ccd_processed_pairs: Vec::new(),
            ccd_processed_time: 0.0,
            #[cfg(feature = "bench-internals")]
            ccd_toi_seen_pairs: None,
            solver_manifolds: Vec::new(),
            interactions: Vec::new(),
            interaction_indices: None,
            events: Vec::new(),
        }
    }

    pub(crate) fn clear(&mut self) {
        self.predicted_states.clear();
        self.body_indices.clear();
        self.body_indices_ready = false;
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
        self.ccd_body_types.clear();
        self.ccd_continuous_bodies.clear();
        self.ccd_candidate_bodies.clear();
        self.ccd_candidate_pairs.clear();
        if let Some(pairs) = &mut self.ccd_candidate_pair_set {
            pairs.clear();
        }
        self.ccd_toi_cache.clear();
        self.ccd_resweep_candidates.clear();
        self.ccd_query_results.clear();
        self.ccd_processed_pairs.clear();
        self.ccd_processed_time = 0.0;
        #[cfg(feature = "bench-internals")]
        if let Some(pairs) = &mut self.ccd_toi_seen_pairs {
            pairs.clear();
        }
        self.solver_manifolds.clear();
        self.interactions.clear();
        if let Some(indices) = &mut self.interaction_indices {
            indices.clear();
        }
        self.events.clear();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        let ccd_toi_seen_pairs = self.ccd_toi_seen_pairs.as_ref().map_or(0, |pairs| {
            pairs.capacity() * size_of::<super::CollisionPair>()
        });

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
            self.ccd_body_types.capacity() * size_of::<BodyType>(),
            self.ccd_continuous_bodies.capacity() * size_of::<bool>(),
            self.ccd_candidate_bodies.capacity() * size_of::<(usize, usize)>(),
            self.ccd_candidate_pairs.capacity() * size_of::<super::CollisionPair>(),
            self.ccd_candidate_pair_set.as_ref().map_or(0, |pairs| {
                pairs.capacity() * size_of::<(super::CollisionPair, ())>()
            }),
            self.ccd_toi_cache.capacity() * size_of::<CcdCandidateToi>(),
            self.ccd_resweep_candidates.capacity() * size_of::<usize>(),
            self.ccd_query_results.capacity() * size_of::<ColliderId>(),
            self.ccd_processed_pairs.capacity() * size_of::<super::CollisionPair>(),
            ccd_toi_seen_pairs,
            self.solver_manifolds.capacity() * size_of::<SolverManifold>(),
            self.interactions.capacity() * size_of::<CollisionInteraction>(),
            self.interaction_indices.as_ref().map_or(0, |indices| {
                indices.capacity() * size_of::<(CollisionPair, usize)>()
            }),
            self.events.capacity() * size_of::<CollisionEvent>(),
        ]
        .into_iter()
        .fold(0_usize, usize::saturating_add)
    }
}
