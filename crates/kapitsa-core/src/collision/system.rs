use super::{
    CollisionConfig, CollisionError, CollisionEvent, CollisionLifecycle, CollisionPair,
    StepContext,
    broad_phase::BroadPhase,
    cache::ContactCache,
    ccd::{CcdHit, CcdMotion},
    island::IslandLayout,
    manifold::ContactManifold,
    mesh::TriangleBvh,
    narrow_phase::NarrowPhase,
    solver::{SolverBody, SolverIsland, SolverManifold, SolverMaterial, SolverWorkspace},
    spatial_index::SpatialIndex,
};
use crate::collider::CollisionMode;
use crate::{
    aabb::Aabb,
    execution::StepExecutor,
    rigid_body::BodyState,
    shape::Shape,
    transform::Transform,
    world::{BodyId, ColliderId},
};

#[cfg(feature = "bench-internals")]
use super::CollisionStepMetrics;

/// Owns reusable collision-step scratch and the last committed event batch.
pub(crate) struct CollisionSystem {
    config: CollisionConfig,
    pub(crate) context: StepContext,
    events: Vec<CollisionEvent>,
    spatial_index: Option<SpatialIndex>,
    broad_phase: BroadPhase,
    contact_cache: Option<ContactCache>,
    lifecycle: CollisionLifecycle,
    lifecycle_staged: bool,
    narrow_phase: NarrowPhase,
    islands: IslandLayout,
    ccd_islands: super::CcdIslandLayout,
    solver_islands: Vec<SolverIsland>,
    ccd_solver: SolverWorkspace,
    cache_staged: bool,
    #[cfg(feature = "bench-internals")]
    benchmark_metrics: CollisionStepMetrics,
    #[cfg(feature = "bench-internals")]
    benchmark_capacity_before_bytes: usize,
}

impl CollisionSystem {
    pub(crate) const fn new() -> Self {
        Self {
            config: CollisionConfig::default_const(),
            context: StepContext::new(),
            events: Vec::new(),
            spatial_index: None,
            broad_phase: BroadPhase::new(),
            contact_cache: None,
            lifecycle: CollisionLifecycle::new(),
            lifecycle_staged: false,
            narrow_phase: NarrowPhase::new(),
            islands: IslandLayout::new(),
            ccd_islands: super::CcdIslandLayout::new(),
            solver_islands: Vec::new(),
            ccd_solver: SolverWorkspace::new(),
            cache_staged: false,
            #[cfg(feature = "bench-internals")]
            benchmark_metrics: CollisionStepMetrics::ZERO,
            #[cfg(feature = "bench-internals")]
            benchmark_capacity_before_bytes: 0,
        }
    }

    pub(crate) fn config(&self) -> CollisionConfig {
        self.config
    }

    pub(crate) fn set_config(&mut self, config: CollisionConfig) -> Result<(), CollisionError> {
        config.validate()?;
        self.config = config;
        if let Some(cache) = self.contact_cache.as_mut() {
            cache.clear();
        }
        Ok(())
    }

    pub(crate) fn events(&self) -> &[CollisionEvent] {
        &self.events
    }

    pub(crate) fn joint_break_events(&self) -> &[crate::joint::JointBreakEvent] {
        &self.context.joint_break_events
    }

    pub(crate) fn begin_step(&mut self) {
        self.context.clear();
        self.lifecycle_staged = false;
        self.cache_staged = false;
        #[cfg(feature = "bench-internals")]
        {
            self.benchmark_capacity_before_bytes = self.retained_scratch_bytes();
            self.benchmark_metrics = CollisionStepMetrics::ZERO;
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_spatial_index(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.spatial_index += elapsed;
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_broad_phase(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.broad_phase += elapsed;
        self.benchmark_metrics.candidate_count = self.candidate_count();
        self.benchmark_metrics.ccd_broad_phase_build_count = self
            .benchmark_metrics
            .ccd_broad_phase_build_count
            .saturating_add(1);
        if self.benchmark_metrics.ccd_broad_phase_build_count == 1 {
            self.benchmark_metrics.ccd_initial_candidate_count = self.candidate_count();
        }
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.ccd += elapsed;
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_island_prepare(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.ccd_island_prepare += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_island_execute(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.ccd_island_execute += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_island_commit(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.ccd_island_commit += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_impact(&mut self) {
        self.benchmark_metrics.ccd_impact_count =
            self.benchmark_metrics.ccd_impact_count.saturating_add(1);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_clipped_contact(&mut self) {
        self.benchmark_metrics.ccd_clipped_island_count = self
            .benchmark_metrics
            .ccd_clipped_island_count
            .saturating_add(1);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_clipped_body(&mut self) {
        self.benchmark_metrics.ccd_clipped_body_count = self
            .benchmark_metrics
            .ccd_clipped_body_count
            .saturating_add(1);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_toi_search(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.ccd_toi_search += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_resweep(
        &mut self,
        elapsed: std::time::Duration,
        invalidated_count: usize,
        colliders_visited: usize,
        proxies_updated: usize,
        found_interisland_dependency: bool,
    ) {
        self.benchmark_metrics.ccd_candidate_refresh += elapsed;
        self.benchmark_metrics.ccd_resweep_count =
            self.benchmark_metrics.ccd_resweep_count.saturating_add(1);
        self.benchmark_metrics.ccd_resweep_colliders_visited = self
            .benchmark_metrics
            .ccd_resweep_colliders_visited
            .saturating_add(colliders_visited);
        self.benchmark_metrics.ccd_resweep_proxies_updated = self
            .benchmark_metrics
            .ccd_resweep_proxies_updated
            .saturating_add(proxies_updated);
        if found_interisland_dependency {
            self.benchmark_metrics.ccd_interisland_resweep_count = self
                .benchmark_metrics
                .ccd_interisland_resweep_count
                .saturating_add(1);
        } else {
            self.benchmark_metrics.ccd_local_resweep_count = self
                .benchmark_metrics
                .ccd_local_resweep_count
                .saturating_add(1);
        }
        self.benchmark_metrics.ccd_invalidated_toi_count = self
            .benchmark_metrics
            .ccd_invalidated_toi_count
            .saturating_add(invalidated_count);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_local_recalculation(&mut self, candidate_count: usize) {
        self.benchmark_metrics.ccd_local_toi_recalculation_count = self
            .benchmark_metrics
            .ccd_local_toi_recalculation_count
            .saturating_add(candidate_count);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_global_fallback(
        &mut self,
        candidate_count: usize,
        island_count: usize,
    ) {
        self.benchmark_metrics.ccd_global_fallback_count = self
            .benchmark_metrics
            .ccd_global_fallback_count
            .saturating_add(1);
        self.benchmark_metrics.ccd_global_toi_recalculation_count = self
            .benchmark_metrics
            .ccd_global_toi_recalculation_count
            .saturating_add(candidate_count);
        self.benchmark_metrics.ccd_replayed_island_count = self
            .benchmark_metrics
            .ccd_replayed_island_count
            .saturating_add(island_count);
        self.benchmark_metrics.ccd_impact_count = 0;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_local_replay(&mut self, island_count: usize) {
        self.benchmark_metrics.ccd_local_replay_count = self
            .benchmark_metrics
            .ccd_local_replay_count
            .saturating_add(1);
        self.benchmark_metrics.ccd_replayed_island_count = self
            .benchmark_metrics
            .ccd_replayed_island_count
            .saturating_add(island_count);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_worker_tasks_started(&mut self, count: usize) {
        self.benchmark_metrics.worker_tasks_started = self
            .benchmark_metrics
            .worker_tasks_started
            .saturating_add(count);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_parallel_islands(&mut self, island_count: usize, task_count: usize) {
        self.benchmark_metrics.ccd_parallel_island_count = self
            .benchmark_metrics
            .ccd_parallel_island_count
            .saturating_add(island_count);
        self.record_worker_tasks_started(task_count);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_final_candidate_count(&mut self, count: usize) {
        self.benchmark_metrics.ccd_final_candidate_count = count;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_final_impact_count(&mut self, count: usize) {
        self.benchmark_metrics.ccd_impact_count = count;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_contact_resolution(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.ccd_contact_resolution += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_toi_call(&mut self) {
        self.benchmark_metrics.ccd_toi_call_count =
            self.benchmark_metrics.ccd_toi_call_count.saturating_add(1);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_toi_evaluation(&mut self, pair: CollisionPair) {
        let repeated = self
            .context
            .ccd_toi_seen_pairs
            .as_mut()
            .is_some_and(|pairs| !pairs.insert(pair));
        self.record_ccd_toi_call();
        if repeated {
            self.record_ccd_repeated_toi_call();
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_repeated_toi_call(&mut self) {
        self.benchmark_metrics.ccd_repeated_toi_call_count = self
            .benchmark_metrics
            .ccd_repeated_toi_call_count
            .saturating_add(1);
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_narrow_phase(
        &mut self,
        elapsed: std::time::Duration,
        contact_count: usize,
    ) {
        self.benchmark_metrics.narrow_phase += elapsed;
        self.benchmark_metrics.contact_count = self
            .benchmark_metrics
            .contact_count
            .saturating_add(contact_count);
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_island_build(
        &mut self,
        elapsed: std::time::Duration,
        island_count: usize,
    ) {
        self.benchmark_metrics.island_build += elapsed;
        self.benchmark_metrics.island_count = island_count;
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_island_build(&mut self, elapsed: std::time::Duration, count: usize) {
        self.benchmark_metrics.ccd_island_count = count;
        self.benchmark_metrics.island_build += elapsed;
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_solver(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.solver += elapsed;
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_events(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.events += elapsed;
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_commit(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.commit += elapsed;
        self.record_peak_scratch();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn benchmark_metrics(&self) -> CollisionStepMetrics {
        let mut metrics = self.benchmark_metrics;
        metrics.retained_scratch_bytes = self.retained_scratch_bytes();
        metrics.logical_capacity_growth_bytes = metrics
            .retained_scratch_bytes
            .saturating_sub(self.benchmark_capacity_before_bytes);
        metrics
    }

    #[cfg(feature = "bench-internals")]
    fn retained_scratch_bytes(&self) -> usize {
        let mut retained = self.context.retained_capacity_bytes()
            + self.events.capacity() * std::mem::size_of::<CollisionEvent>()
            + self.broad_phase.retained_capacity_bytes()
            + self.narrow_phase.retained_capacity_bytes()
            + self.islands.retained_capacity_bytes()
            + self.ccd_islands.retained_capacity_bytes()
            + self.ccd_solver.retained_capacity_bytes()
            + self
                .solver_islands
                .capacity()
                .saturating_mul(std::mem::size_of::<SolverIsland>());
        if let Some(index) = &self.spatial_index {
            retained = retained.saturating_add(index.retained_capacity_bytes());
        }
        if let Some(cache) = &self.contact_cache {
            retained = retained.saturating_add(cache.retained_capacity_bytes());
        }
        for island in &self.solver_islands {
            retained = retained.saturating_add(island.retained_capacity_bytes());
        }
        retained
    }

    #[cfg(feature = "bench-internals")]
    fn record_peak_scratch(&mut self) {
        self.benchmark_metrics.peak_scratch_bytes = self
            .benchmark_metrics
            .peak_scratch_bytes
            .max(self.retained_scratch_bytes());
    }

    pub(crate) fn stage_prediction(&mut self, body_id: BodyId, state: BodyState) {
        self.context.body_indices_ready = false;
        self.context.predicted_states.push((body_id, state));
    }

    pub(crate) fn synchronize_spatial_index(&mut self) -> Result<(), CollisionError> {
        if self.spatial_index.is_none() && self.context.spatial_proxies.is_empty() {
            return Ok(());
        }
        let index = self.spatial_index.get_or_insert_with(SpatialIndex::new);
        match index.synchronize(&self.context.spatial_proxies) {
            Ok(()) => Ok(()),
            Err(error) => {
                index.mark_dirty();
                Err(error)
            }
        }
    }

    pub(crate) fn generate_candidates(&mut self) -> Result<(), CollisionError> {
        let result = self
            .broad_phase
            .generate(self.spatial_index.as_mut(), &self.context.colliders);
        if let Err(error) = result {
            self.mark_spatial_dirty();
            return Err(error);
        }
        Ok(())
    }

    pub(crate) fn candidate_count(&self) -> usize {
        self.broad_phase.candidates().len()
    }

    pub(crate) fn candidate(&self, index: usize) -> Option<CollisionPair> {
        self.broad_phase.candidates().get(index).copied()
    }

    pub(crate) fn build_ccd_islands(
        &mut self,
        force_single_island: bool,
    ) -> Result<(), CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = std::time::Instant::now();
        if self.context.ccd_toi_cache.len() != self.context.ccd_candidate_bodies.len()
            || self.context.ccd_candidate_active.len() != self.context.ccd_candidate_bodies.len()
        {
            return Err(CollisionError::InvalidContact);
        }
        self.ccd_islands.build_active(
            &self.context.ccd_body_types,
            &self.context.ccd_continuous_bodies,
            &self.context.ccd_candidate_bodies,
            &self.context.ccd_candidate_active,
            force_single_island,
        )?;
        let island_count = self.ccd_island_count();
        for island in 0..island_count {
            if self
                .ccd_island_candidates(island)
                .is_none_or(<[usize]>::is_empty)
            {
                return Err(CollisionError::InvalidContact);
            }
        }
        #[cfg(feature = "bench-internals")]
        self.record_ccd_island_build(started.elapsed(), island_count);
        Ok(())
    }

    pub(crate) fn ccd_island_count(&self) -> usize {
        self.ccd_islands.island_count()
    }

    pub(crate) fn ccd_island_candidates(&self, island: usize) -> Option<&[usize]> {
        self.ccd_islands.candidates(island)
    }

    pub(crate) fn ccd_island_bodies(&self, island: usize) -> Option<&[usize]> {
        self.ccd_islands.bodies(island)
    }

    pub(crate) fn append_ccd_island_bodies_to_replay(
        &mut self,
        island: usize,
    ) -> Result<(), CollisionError> {
        let (layout, context) = (&self.ccd_islands, &mut self.context);
        let bodies = layout
            .bodies(island)
            .ok_or(CollisionError::InvalidContact)?;
        context
            .ccd_replay_bodies
            .try_reserve(bodies.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context.ccd_replay_bodies.extend_from_slice(bodies);
        Ok(())
    }

    pub(crate) fn ccd_island_completed(&self, island: usize) -> Result<bool, CollisionError> {
        let bodies = self
            .ccd_island_bodies(island)
            .ok_or(CollisionError::InvalidContact)?;
        Ok(bodies.iter().all(|body| {
            self.context
                .ccd_completed_bodies
                .get(*body)
                .copied()
                .unwrap_or(false)
        }))
    }

    pub(crate) fn mark_ccd_island_completed(
        &mut self,
        island: usize,
    ) -> Result<(), CollisionError> {
        let bodies = self
            .ccd_islands
            .bodies(island)
            .ok_or(CollisionError::InvalidContact)?;
        for body in bodies {
            *self
                .context
                .ccd_completed_bodies
                .get_mut(*body)
                .ok_or(CollisionError::InvalidContact)? = true;
        }
        Ok(())
    }

    pub(crate) fn mark_ccd_island_incomplete(
        &mut self,
        island: usize,
    ) -> Result<(), CollisionError> {
        let bodies = self
            .ccd_islands
            .bodies(island)
            .ok_or(CollisionError::InvalidContact)?;
        for body in bodies {
            *self
                .context
                .ccd_completed_bodies
                .get_mut(*body)
                .ok_or(CollisionError::InvalidContact)? = false;
        }
        Ok(())
    }

    pub(crate) fn collect_ccd_replay_candidates(&mut self) -> Result<(), CollisionError> {
        let (layout, context) = (&self.ccd_islands, &mut self.context);
        let candidates = &mut context.ccd_resweep_candidates;
        candidates.clear();
        for body in context.ccd_replay_bodies.iter().copied() {
            let body_candidates = layout
                .body_candidates(body)
                .ok_or(CollisionError::InvalidContact)?;
            candidates
                .try_reserve(body_candidates.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            candidates.extend_from_slice(body_candidates);
        }
        candidates.sort_unstable();
        candidates.dedup();
        Ok(())
    }

    pub(crate) fn ccd_body_island(&self, body: usize) -> Option<usize> {
        self.ccd_islands.body_island(body)
    }

    pub(crate) fn invalidate_ccd_candidates(
        &mut self,
        body_a: usize,
        body_a_changed: bool,
        body_b: usize,
        body_b_changed: bool,
    ) -> Result<(), CollisionError> {
        let context = &mut self.context;
        context.ccd_resweep_candidates.clear();
        for (body, changed) in [(body_a, body_a_changed), (body_b, body_b_changed)] {
            if !changed {
                continue;
            }
            let candidates = self
                .ccd_islands
                .body_candidates(body)
                .ok_or(CollisionError::InvalidContact)?;
            context
                .ccd_resweep_candidates
                .try_reserve(candidates.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context.ccd_resweep_candidates.extend_from_slice(candidates);
        }
        context.ccd_resweep_candidates.sort_unstable();
        context.ccd_resweep_candidates.dedup();
        for candidate in context.ccd_resweep_candidates.iter().copied() {
            let cache = context
                .ccd_toi_cache
                .get_mut(candidate)
                .ok_or(CollisionError::InvalidContact)?;
            cache.evaluated = false;
            cache.hit = None;
        }
        Ok(())
    }

    pub(crate) fn ccd_resweep_candidate_count(&self) -> usize {
        self.context.ccd_resweep_candidates.len()
    }

    pub(crate) fn set_ccd_candidate_active(
        &mut self,
        candidate: usize,
        active: bool,
    ) -> Result<(), CollisionError> {
        *self
            .context
            .ccd_candidate_active
            .get_mut(candidate)
            .ok_or(CollisionError::InvalidContact)? = active;
        Ok(())
    }

    pub(crate) fn sort_ccd_resweep_candidates(&mut self) {
        self.context.ccd_resweep_candidates.sort_unstable();
        self.context.ccd_resweep_candidates.dedup();
    }

    pub(crate) fn ccd_resweep_candidate(&self, index: usize) -> Option<usize> {
        self.context.ccd_resweep_candidates.get(index).copied()
    }

    pub(crate) fn ccd_candidate_pair(&self, index: usize) -> Option<CollisionPair> {
        self.context.ccd_candidate_pairs.get(index).copied()
    }

    pub(crate) fn ccd_candidate_count(&self) -> usize {
        self.context.ccd_candidate_pairs.len()
    }

    pub(crate) fn ccd_candidate_body_pair(&self, index: usize) -> Option<(usize, usize)> {
        self.context.ccd_candidate_bodies.get(index).copied()
    }

    pub(crate) fn ccd_query_result(&self, index: usize) -> Option<ColliderId> {
        self.context.ccd_query_results.get(index).copied()
    }

    pub(crate) fn ccd_pair_allowed(
        &self,
        collider_a: ColliderId,
        collider_b: ColliderId,
    ) -> Result<bool, CollisionError> {
        let proxy_a = self.ccd_proxy(collider_a)?;
        let proxy_b = self.ccd_proxy(collider_b)?;
        Ok(BroadPhase::allows_pair(&proxy_a, &proxy_b))
    }

    pub(crate) fn ccd_proxy(
        &self,
        collider: ColliderId,
    ) -> Result<super::CollisionProxy, CollisionError> {
        let index = self
            .broad_phase
            .proxy_index(collider)
            .ok_or(CollisionError::InvalidContact)?;
        self.context
            .colliders
            .get(index)
            .copied()
            .ok_or(CollisionError::InvalidContact)
    }

    pub(crate) fn prepare_ccd_resweep_proxy(
        &mut self,
        collider: ColliderId,
        bounds: Option<Aabb>,
        unbounded_query_colliders: &[ColliderId],
    ) -> Result<(usize, usize), CollisionError> {
        let proxy_index = self
            .broad_phase
            .proxy_index(collider)
            .ok_or(CollisionError::InvalidContact)?;
        self.context
            .colliders
            .get_mut(proxy_index)
            .ok_or(CollisionError::InvalidContact)?
            .bounds = bounds;

        let (spatial_index, context, broad_phase) = (
            &mut self.spatial_index,
            &mut self.context,
            &self.broad_phase,
        );
        context.ccd_query_results.clear();
        let mut colliders_visited = 0;
        if let Some(bounds) = bounds {
            let index = spatial_index.get_or_insert_with(SpatialIndex::new);
            index.update(collider, bounds)?;
            index.query(bounds, &mut context.ccd_query_results)?;
            colliders_visited = unbounded_query_colliders.len();
            for collider in unbounded_query_colliders.iter().copied() {
                let proxy_index = broad_phase
                    .proxy_index(collider)
                    .ok_or(CollisionError::InvalidContact)?;
                let proxy = context
                    .colliders
                    .get(proxy_index)
                    .filter(|proxy| proxy.collider == collider && proxy.bounds.is_none())
                    .ok_or(CollisionError::InvalidContact)?;
                context
                    .ccd_query_results
                    .try_reserve(1)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                context.ccd_query_results.push(proxy.collider);
            }
        } else if let Some(index) = spatial_index.as_mut() {
            index.remove(collider)?;
        }
        Ok((context.ccd_query_results.len(), colliders_visited))
    }

    pub(crate) fn add_ccd_candidate(
        &mut self,
        pair: CollisionPair,
        body_a: usize,
        body_b: usize,
    ) -> Result<(bool, bool), CollisionError> {
        let already_present = self
            .context
            .ccd_candidate_pair_set
            .as_ref()
            .ok_or(CollisionError::InvalidContact)?
            .contains(&pair);
        if already_present {
            return Ok((false, false));
        }

        let continuous_a = *self
            .context
            .ccd_continuous_bodies
            .get(body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let continuous_b = *self
            .context
            .ccd_continuous_bodies
            .get(body_b)
            .ok_or(CollisionError::InvalidContact)?;
        let ccd_candidate = continuous_a || continuous_b;

        if ccd_candidate {
            let candidate_index = self.context.ccd_candidate_pairs.len();
            self.context
                .ccd_candidate_pairs
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.context
                .ccd_candidate_bodies
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.context
                .ccd_toi_cache
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.context
                .ccd_candidate_active
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.context
                .ccd_resweep_candidates
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.context.ccd_candidate_pairs.push(pair);
            self.context.ccd_candidate_bodies.push((body_a, body_b));
            self.context
                .ccd_toi_cache
                .push(super::CcdCandidateToi::default());
            self.context.ccd_candidate_active.push(false);
            self.context.ccd_resweep_candidates.push(candidate_index);
        }
        let candidate_set = self
            .context
            .ccd_candidate_pair_set
            .as_mut()
            .ok_or(CollisionError::InvalidContact)?;
        candidate_set
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        candidate_set.insert(pair);
        self.broad_phase.add_candidate(pair)?;
        #[cfg(feature = "bench-internals")]
        {
            self.benchmark_metrics.ccd_new_candidate_count = self
                .benchmark_metrics
                .ccd_new_candidate_count
                .saturating_add(1);
        }
        Ok((true, ccd_candidate))
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn detect(
        &mut self,
        pair: CollisionPair,
        shape_a: &Shape,
        collider_a: &Transform,
        body_a: &Transform,
        mode_a: CollisionMode,
        bvh_a: Option<&TriangleBvh>,
        vertices_a: Option<&[kapitsa_math::Vec3]>,
        shape_b: &Shape,
        collider_b: &Transform,
        body_b: &Transform,
        mode_b: CollisionMode,
        bvh_b: Option<&TriangleBvh>,
        vertices_b: Option<&[kapitsa_math::Vec3]>,
    ) -> Result<Option<ContactManifold>, CollisionError> {
        self.narrow_phase.detect_with_geometry(
            pair,
            shape_a,
            collider_a,
            body_a,
            mode_a,
            bvh_a,
            vertices_a,
            shape_b,
            collider_b,
            body_b,
            mode_b,
            bvh_b,
            vertices_b,
            self.config.contact_offset,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn time_of_impact(
        &mut self,
        shape_a: &Shape,
        motion_a: &CcdMotion,
        bvh_a: Option<&TriangleBvh>,
        vertices_a: Option<&[kapitsa_math::Vec3]>,
        shape_b: &Shape,
        motion_b: &CcdMotion,
        bvh_b: Option<&TriangleBvh>,
        vertices_b: Option<&[kapitsa_math::Vec3]>,
        sensor: bool,
    ) -> Result<Option<CcdHit>, CollisionError> {
        let result = self.narrow_phase.time_of_impact(
            shape_a, motion_a, bvh_a, vertices_a, shape_b, motion_b, bvh_b, vertices_b, sensor,
        );
        #[cfg(feature = "bench-internals")]
        self.record_ccd_toi_call();
        result
    }

    pub(crate) fn solve_contacts(
        &mut self,
        worker_count: usize,
        executor: &impl StepExecutor,
        dt: f64,
    ) -> Result<(), CollisionError> {
        self.context.joint_break_events.clear();
        if let Some(cache) = self.contact_cache.as_ref() {
            cache.warm_start_solver(&mut self.context.solver_manifolds, dt)?;
        }
        #[cfg(feature = "bench-internals")]
        let island_started = std::time::Instant::now();
        self.islands.build_with_joints(
            &self.context.body_types,
            &self.context.solver_manifolds,
            &self.context.joint_constraints,
        )?;
        let island_count = self.islands.island_count();
        #[cfg(feature = "bench-internals")]
        self.record_island_build(island_started.elapsed(), island_count);
        #[cfg(feature = "bench-internals")]
        let solver_started = std::time::Instant::now();
        #[cfg(feature = "bench-internals")]
        let prepare_started = std::time::Instant::now();
        if self.solver_islands.len() < island_count {
            self.solver_islands
                .try_reserve(island_count - self.solver_islands.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.solver_islands
                .resize_with(island_count, SolverIsland::new);
        }
        for island in self.solver_islands.iter_mut().take(island_count) {
            island.clear();
        }
        for (manifold_index, manifold) in self.context.solver_manifolds.iter().enumerate() {
            let Some(island_index) = self.islands.manifold_island(manifold_index) else {
                continue;
            };
            let island = self
                .solver_islands
                .get_mut(island_index)
                .ok_or(CollisionError::InvalidContact)?;
            island
                .manifold_indices
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            island.manifold_indices.push(manifold_index);
            island
                .body_indices
                .try_reserve(2)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            island.body_indices.push(manifold.body_a);
            island.body_indices.push(manifold.body_b);
        }
        for (joint_index, joint) in self.context.joint_constraints.iter().enumerate() {
            let Some(island_index) = self.islands.joint_island(joint_index) else {
                continue;
            };
            let island = self
                .solver_islands
                .get_mut(island_index)
                .ok_or(CollisionError::InvalidContact)?;
            island
                .joint_indices
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            island.joint_indices.push(joint_index);
            island
                .body_indices
                .try_reserve(2)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            island.body_indices.push(joint.body_a);
            island.body_indices.push(joint.body_b);
        }
        for island in self.solver_islands.iter_mut().take(island_count) {
            island.body_indices.sort_unstable();
            island.body_indices.dedup();
            island
                .bodies
                .try_reserve(island.body_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            for body_index in island.body_indices.iter().copied() {
                let body = self
                    .context
                    .solver_bodies
                    .get(body_index)
                    .ok_or(CollisionError::InvalidContact)?;
                let solver_body = SolverBody::from_prediction(*body.body(), body.state());
                #[cfg(feature = "bench-internals")]
                {
                    self.benchmark_metrics.solver_body_copies =
                        self.benchmark_metrics.solver_body_copies.saturating_add(1);
                }
                island.bodies.push(solver_body);
            }
            island
                .manifolds
                .try_reserve(island.manifold_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            for manifold_index in island.manifold_indices.iter().copied() {
                let mut manifold = *self
                    .context
                    .solver_manifolds
                    .get(manifold_index)
                    .ok_or(CollisionError::InvalidContact)?;
                #[cfg(feature = "bench-internals")]
                {
                    self.benchmark_metrics.solver_manifold_copies = self
                        .benchmark_metrics
                        .solver_manifold_copies
                        .saturating_add(1);
                }
                manifold.body_a = island
                    .body_indices
                    .binary_search(&manifold.body_a)
                    .map_err(|_| CollisionError::InvalidContact)?;
                manifold.body_b = island
                    .body_indices
                    .binary_search(&manifold.body_b)
                    .map_err(|_| CollisionError::InvalidContact)?;
                island.manifolds.push(manifold);
            }
            island
                .joints
                .try_reserve(island.joint_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            for joint_index in island.joint_indices.iter().copied() {
                let mut joint = *self
                    .context
                    .joint_constraints
                    .get(joint_index)
                    .ok_or(CollisionError::InvalidContact)?;
                joint.body_a = island
                    .body_indices
                    .binary_search(&joint.body_a)
                    .map_err(|_| CollisionError::InvalidContact)?;
                joint.body_b = island
                    .body_indices
                    .binary_search(&joint.body_b)
                    .map_err(|_| CollisionError::InvalidContact)?;
                island.joints.push(joint);
            }
        }
        #[cfg(feature = "bench-internals")]
        {
            self.benchmark_metrics.solver_prepare = prepare_started.elapsed();
        }
        let config = self.config;
        #[cfg(feature = "bench-internals")]
        let iterations_started = std::time::Instant::now();
        if worker_count <= 1 || island_count <= 1 {
            for island in self.solver_islands.iter_mut().take(island_count) {
                island.workspace.solve_with_joints(
                    &mut island.bodies,
                    &mut island.manifolds,
                    &mut island.joint_workspace,
                    &island.joints,
                    dt,
                    config,
                )?;
            }
        } else {
            let worker_count = worker_count.min(island_count);
            let chunk_size = island_count.div_ceil(worker_count);
            #[cfg(feature = "bench-internals")]
            self.record_worker_tasks_started(island_count.div_ceil(chunk_size));
            let solved = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                executor.for_each_chunk_mut(
                    &mut self.solver_islands[..island_count],
                    chunk_size,
                    |_, chunk| {
                        for island in chunk {
                            if let Err(error) = island.workspace.solve_with_joints(
                                &mut island.bodies,
                                &mut island.manifolds,
                                &mut island.joint_workspace,
                                &island.joints,
                                dt,
                                config,
                            ) {
                                island.worker_error = Some(error);
                                break;
                            }
                        }
                    },
                )
            }));
            if !matches!(solved, Ok(Ok(()))) {
                return Err(CollisionError::WorkerFailed);
            }
            if let Some(error) = self
                .solver_islands
                .iter()
                .take(island_count)
                .find_map(|island| island.worker_error)
            {
                return Err(error);
            }
        }

        for island in self.solver_islands.iter().take(island_count) {
            self.context
                .joint_break_events
                .try_reserve(island.joint_workspace.break_events().len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.context
                .joint_break_events
                .extend_from_slice(island.joint_workspace.break_events());
        }
        self.context
            .joint_break_events
            .sort_unstable_by_key(|event| event.joint());
        self.context
            .joint_break_events
            .dedup_by_key(|event| event.joint());
        #[cfg(feature = "bench-internals")]
        {
            self.benchmark_metrics.solver_iterations = iterations_started.elapsed();
            self.record_solver(solver_started.elapsed());
        }

        #[cfg(feature = "bench-internals")]
        let writeback_started = std::time::Instant::now();
        let context = &mut self.context;
        for island in self.solver_islands.iter().take(island_count) {
            for (local_index, global_index) in island.body_indices.iter().copied().enumerate() {
                if context.body_types.get(global_index)
                    == Some(&crate::rigid_body::BodyType::Dynamic)
                {
                    let body = island
                        .bodies
                        .get(local_index)
                        .ok_or(CollisionError::InvalidContact)?;
                    let prediction_index = *context
                        .solver_body_predictions
                        .get(global_index)
                        .ok_or(CollisionError::InvalidContact)?;
                    let predicted = context
                        .predicted_states
                        .get_mut(prediction_index)
                        .ok_or(CollisionError::InvalidContact)?;
                    predicted.1 = body.state();
                }
            }
            for (local_index, global_index) in island.manifold_indices.iter().copied().enumerate() {
                let solved = island
                    .manifolds
                    .get(local_index)
                    .ok_or(CollisionError::InvalidContact)?;
                let original = context
                    .solver_manifolds
                    .get_mut(global_index)
                    .ok_or(CollisionError::InvalidContact)?;
                original.manifold = solved.manifold;
                #[cfg(feature = "bench-internals")]
                {
                    self.benchmark_metrics.solver_manifold_copies = self
                        .benchmark_metrics
                        .solver_manifold_copies
                        .saturating_add(1);
                }
            }
        }
        for (manifold, solver) in context.manifolds.iter_mut().zip(&context.solver_manifolds) {
            *manifold = solver.manifold;
            #[cfg(feature = "bench-internals")]
            {
                self.benchmark_metrics.solver_manifold_copies = self
                    .benchmark_metrics
                    .solver_manifold_copies
                    .saturating_add(1);
            }
        }
        if !self.context.manifolds.is_empty() || self.contact_cache.is_some() {
            self.contact_cache
                .get_or_insert_with(ContactCache::new)
                .stage(&self.context.manifolds, dt)?;
            self.cache_staged = true;
        }
        #[cfg(feature = "bench-internals")]
        {
            self.benchmark_metrics.solver_writeback = writeback_started.elapsed();
        }
        Ok(())
    }

    pub(crate) fn solve_ccd_contact(
        &mut self,
        body_a: crate::rigid_body::RigidBody,
        state_a: BodyState,
        body_b: crate::rigid_body::RigidBody,
        state_b: BodyState,
        manifold: ContactManifold,
        material: SolverMaterial,
    ) -> Result<(BodyState, BodyState, ContactManifold), CollisionError> {
        Self::solve_ccd_contact_with_workspace(
            &mut self.ccd_solver,
            body_a,
            state_a,
            body_b,
            state_b,
            manifold,
            material,
            self.config,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn solve_ccd_contact_with_workspace(
        workspace: &mut SolverWorkspace,
        body_a: crate::rigid_body::RigidBody,
        state_a: BodyState,
        body_b: crate::rigid_body::RigidBody,
        state_b: BodyState,
        manifold: ContactManifold,
        material: SolverMaterial,
        config: CollisionConfig,
    ) -> Result<(BodyState, BodyState, ContactManifold), CollisionError> {
        let mut bodies = [
            SolverBody::from_prediction(body_a, state_a),
            SolverBody::from_prediction(body_b, state_b),
        ];
        let mut manifolds = [SolverManifold {
            body_a: 0,
            body_b: 1,
            manifold,
            material,
        }];
        workspace.solve(&mut bodies, &mut manifolds, config)?;
        Ok((bodies[0].state(), bodies[1].state(), manifolds[0].manifold))
    }

    pub(crate) fn stage_lifecycle_events(&mut self) -> Result<(), CollisionError> {
        self.lifecycle
            .stage(&mut self.context.interactions, &mut self.context.events)?;
        self.lifecycle_staged = true;
        Ok(())
    }

    pub(crate) fn mark_spatial_dirty(&mut self) {
        if let Some(index) = self.spatial_index.as_mut() {
            index.mark_dirty();
        }
    }

    pub(crate) fn invalidate_collider_contacts(&mut self, collider: crate::world::ColliderId) {
        if let Some(cache) = self.contact_cache.as_mut() {
            cache.invalidate_collider(collider);
        }
    }

    pub(crate) fn abort_step(&mut self) {
        if let Some(cache) = self.contact_cache.as_mut() {
            cache.abort();
        }
        self.lifecycle.abort();
        self.lifecycle_staged = false;
        self.cache_staged = false;
        self.mark_spatial_dirty();
        self.context.clear();
    }

    pub(crate) fn commit_step(&mut self) {
        std::mem::swap(&mut self.events, &mut self.context.events);
        if self.lifecycle_staged {
            self.lifecycle.commit();
            self.lifecycle_staged = false;
        }
        if self.cache_staged {
            if let Some(cache) = self.contact_cache.as_mut() {
                cache.commit();
            }
            self.cache_staged = false;
        }
        self.context.clear();
    }
}
