use super::{
    CollisionConfig, CollisionError, CollisionEvent, CollisionLifecycle, CollisionPair,
    StepContext,
    broad_phase::BroadPhase,
    cache::ContactCache,
    ccd::CcdHit,
    island::IslandLayout,
    manifold::ContactManifold,
    mesh::TriangleBvh,
    narrow_phase::NarrowPhase,
    solver::{SolverBody, SolverIsland, SolverManifold, SolverMaterial, SolverWorkspace},
    spatial_index::SpatialIndex,
};
use crate::collider::CollisionMode;
use crate::{rigid_body::BodyState, shape::Shape, transform::Transform, world::BodyId};

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
    solver_islands: Vec<SolverIsland>,
    ccd_solver: SolverWorkspace,
    cache_staged: bool,
    #[cfg(feature = "bench-internals")]
    benchmark_metrics: CollisionStepMetrics,
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
            solver_islands: Vec::new(),
            ccd_solver: SolverWorkspace::new(),
            cache_staged: false,
            #[cfg(feature = "bench-internals")]
            benchmark_metrics: CollisionStepMetrics::ZERO,
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

    pub(crate) fn begin_step(&mut self) {
        self.context.clear();
        self.lifecycle_staged = false;
        self.cache_staged = false;
        #[cfg(feature = "bench-internals")]
        {
            self.benchmark_metrics = CollisionStepMetrics::ZERO;
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_spatial_index(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.spatial_index += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_broad_phase(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.broad_phase += elapsed;
        self.benchmark_metrics.candidate_count = self.candidate_count();
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.ccd += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_ccd_impact(&mut self) {
        self.benchmark_metrics.ccd_impact_count =
            self.benchmark_metrics.ccd_impact_count.saturating_add(1);
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
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_island_build(
        &mut self,
        elapsed: std::time::Duration,
        island_count: usize,
    ) {
        self.benchmark_metrics.island_build += elapsed;
        self.benchmark_metrics.island_count = island_count;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_solver(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.solver += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_events(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.events += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn record_commit(&mut self, elapsed: std::time::Duration) {
        self.benchmark_metrics.commit += elapsed;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn benchmark_metrics(&self) -> CollisionStepMetrics {
        let mut metrics = self.benchmark_metrics;
        let mut retained = self.context.retained_capacity_bytes()
            + self.events.capacity() * std::mem::size_of::<CollisionEvent>()
            + self.broad_phase.retained_capacity_bytes()
            + self.narrow_phase.retained_capacity_bytes()
            + self.islands.retained_capacity_bytes()
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
        metrics.retained_scratch_bytes = retained;
        metrics
    }

    pub(crate) fn stage_prediction(&mut self, body_id: BodyId, state: BodyState) {
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
        start_a: Transform,
        end_a: Transform,
        bvh_a: Option<&TriangleBvh>,
        vertices_a: Option<&[kapitsa_math::Vec3]>,
        shape_b: &Shape,
        start_b: Transform,
        end_b: Transform,
        bvh_b: Option<&TriangleBvh>,
        vertices_b: Option<&[kapitsa_math::Vec3]>,
        sensor: bool,
    ) -> Result<Option<CcdHit>, CollisionError> {
        self.narrow_phase.time_of_impact(
            shape_a, start_a, end_a, bvh_a, vertices_a, shape_b, start_b, end_b, bvh_b, vertices_b,
            sensor,
        )
    }

    pub(crate) fn solve_contacts(&mut self, worker_count: usize) -> Result<(), CollisionError> {
        if let Some(cache) = self.contact_cache.as_ref() {
            cache.warm_start_solver(&mut self.context.solver_manifolds);
        }
        #[cfg(feature = "bench-internals")]
        let island_started = std::time::Instant::now();
        self.islands
            .build(&self.context.body_types, &self.context.solver_manifolds)?;
        let island_count = self.islands.island_count();
        #[cfg(feature = "bench-internals")]
        self.record_island_build(island_started.elapsed(), island_count);
        #[cfg(feature = "bench-internals")]
        let solver_started = std::time::Instant::now();
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
                island
                    .bodies
                    .push(SolverBody::from_prediction(*body.body(), body.state()));
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
        }
        let config = self.config;
        if worker_count <= 1 || island_count <= 1 {
            for island in self.solver_islands.iter_mut().take(island_count) {
                island
                    .workspace
                    .solve(&mut island.bodies, &mut island.manifolds, config)?;
            }
        } else {
            let worker_count = worker_count.min(island_count);
            let chunk_size = island_count.div_ceil(worker_count);
            std::thread::scope(|scope| {
                let mut workers = Vec::new();
                workers
                    .try_reserve(worker_count)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                for chunk in self.solver_islands[..island_count].chunks_mut(chunk_size) {
                    workers.push(scope.spawn(move || {
                        for island in chunk {
                            island.workspace.solve(
                                &mut island.bodies,
                                &mut island.manifolds,
                                config,
                            )?;
                        }
                        Ok::<(), CollisionError>(())
                    }));
                }
                for worker in workers {
                    worker.join().map_err(|_| CollisionError::WorkerFailed)??;
                }
                Ok::<(), CollisionError>(())
            })?;
        }
        #[cfg(feature = "bench-internals")]
        self.record_solver(solver_started.elapsed());

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
            }
        }
        for (manifold, solver) in context.manifolds.iter_mut().zip(&context.solver_manifolds) {
            *manifold = solver.manifold;
        }
        if !self.context.manifolds.is_empty() || self.contact_cache.is_some() {
            self.contact_cache
                .get_or_insert_with(ContactCache::new)
                .stage(&self.context.manifolds)?;
            self.cache_staged = true;
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
        self.ccd_solver
            .solve(&mut bodies, &mut manifolds, self.config)?;
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
