use super::{
    BodyId, CcdBodyReplaySnapshot, PhysicsWorld, body_point_velocity, collider_entries_of,
    remove_interactions_for_pairs, remove_transient_sensor_interaction,
    swept_bounds_for_trajectory, upsert_interaction,
};
use crate::{
    collision::{
        CcdCandidateToi, CcdHit, CcdIslandBodyResult, CcdIslandCandidateResult,
        CcdIslandImpactResult, CcdIslandResult, CcdLimitPolicy, CcdMotion, CcdTrajectory,
        CollisionError, CollisionInteraction, CollisionPair, CollisionSystem, ContactInfo,
        ContactManifold, ContactPoint, NarrowPhase,
    },
    execution::StepExecutor,
    rigid_body::{BodyState, BodyType, CcdMode, predict, predict_from_state},
    shape::Shape,
};
use kapitsa_math::Vec3;
#[cfg(feature = "bench-internals")]
use std::time::Instant;

impl PhysicsWorld {
    pub(super) fn process_continuous_collisions(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
    ) -> Result<(), CollisionError> {
        if dt <= 0.0 || self.collision_system.candidate_count() == 0 {
            return Ok(());
        }
        if !self.bodies.iter().any(|(_, entry)| {
            entry.body.physics().body_type() == BodyType::Dynamic
                && entry.body.ccd_mode() == CcdMode::Continuous
        }) {
            return Ok(());
        }
        Self::rebuild_body_indices(&mut self.collision_system.context)?;

        let prediction_count = self.collision_system.context.predicted_states.len();
        {
            let context = &mut self.collision_system.context;
            context.ccd_states.clear();
            context.ccd_elapsed.clear();
            context.ccd_impact_counts.clear();
            context.ccd_clipped_bodies.clear();
            context.ccd_completed_bodies.clear();
            #[cfg(feature = "bench-internals")]
            context.ccd_clipped_metrics.clear();
            context.ccd_body_types.clear();
            context.ccd_continuous_bodies.clear();
            context
                .ccd_states
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_elapsed
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_impact_counts
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_clipped_bodies
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_completed_bodies
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            #[cfg(feature = "bench-internals")]
            context
                .ccd_clipped_metrics
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_body_types
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context
                .ccd_continuous_bodies
                .try_reserve(prediction_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        for index in 0..prediction_count {
            let body_id = self.collision_system.context.predicted_states[index].0;
            let body = self
                .bodies
                .get(body_id.dense)
                .filter(|_| body_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?;
            let context = &mut self.collision_system.context;
            context.ccd_states.push(*body.state());
            context.ccd_elapsed.push(0.0);
            context.ccd_impact_counts.push(0);
            context.ccd_clipped_bodies.push(false);
            context.ccd_completed_bodies.push(false);
            #[cfg(feature = "bench-internals")]
            context.ccd_clipped_metrics.push(false);
            context.ccd_body_types.push(body.physics().body_type());
            context.ccd_continuous_bodies.push(
                body.physics().body_type() == BodyType::Dynamic
                    && body.ccd_mode() == CcdMode::Continuous,
            );
        }
        #[cfg(feature = "bench-internals")]
        let ccd_prepare_started = Instant::now();
        self.rebuild_ccd_candidate_bodies()?;
        let mut force_single_island = false;
        self.collision_system
            .build_ccd_islands(force_single_island)?;
        self.collision_system.context.ccd_processed_pairs.clear();
        self.collision_system.context.ccd_processed_time = 0.0;
        #[cfg(feature = "bench-internals")]
        {
            let candidate_count = self.collision_system.candidate_count();
            let seen_pairs = self
                .collision_system
                .context
                .ccd_toi_seen_pairs
                .get_or_insert_with(std::collections::HashSet::new);
            seen_pairs
                .try_reserve(candidate_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        self.precompute_ccd_tois_parallel(dt, worker_count, executor)?;
        self.collision_system.build_ccd_islands(false)?;
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_island_prepare(ccd_prepare_started.elapsed());

        #[cfg(feature = "bench-internals")]
        let ccd_execute_started = Instant::now();
        let time_tolerance = f64::EPSILON * dt.max(1.0) * 128.0;
        self.resolve_single_candidate_ccd_islands_parallel(
            dt,
            worker_count,
            executor,
            time_tolerance,
            &mut force_single_island,
        )?;
        'passes: loop {
            let mut island_index = 0;
            while island_index < self.collision_system.ccd_island_count() {
                if self.collision_system.ccd_island_completed(island_index)? {
                    island_index += 1;
                    continue;
                }
                self.collision_system.context.ccd_processed_pairs.clear();
                self.collision_system.context.ccd_processed_time = 0.0;
                let mut elapsed = 0.0;
                loop {
                    let remaining = (dt - elapsed).max(0.0);
                    if remaining <= f64::EPSILON * dt.max(1.0) {
                        break;
                    }

                    let mut earliest: Option<(CollisionPair, CcdHit, f64)> = None;
                    let island_candidate_count = self
                        .collision_system
                        .ccd_island_candidates(island_index)
                        .map(<[usize]>::len)
                        .ok_or(CollisionError::InvalidContact)?;
                    #[cfg(feature = "bench-internals")]
                    let toi_search_started = Instant::now();
                    for island_candidate_index in 0..island_candidate_count {
                        let candidate_index = self
                            .collision_system
                            .ccd_island_candidates(island_index)
                            .and_then(|candidates| candidates.get(island_candidate_index))
                            .copied()
                            .ok_or(CollisionError::InvalidContact)?;
                        let pair = self
                            .collision_system
                            .ccd_candidate_pair(candidate_index)
                            .ok_or(CollisionError::InvalidContact)?;
                        let cached = self
                            .collision_system
                            .context
                            .ccd_toi_cache
                            .get(candidate_index)
                            .copied()
                            .ok_or(CollisionError::InvalidContact)?;
                        if cached.evaluated {
                            if let Some(hit) = cached.hit
                                && earliest.is_none_or(|(current_pair, _, current_time)| {
                                    cached.impact_time < current_time
                                        || (cached.impact_time == current_time
                                            && pair < current_pair)
                                })
                            {
                                earliest = Some((pair, hit, cached.impact_time));
                            }
                            continue;
                        }
                        let context = &self.collision_system.context;
                        if elapsed <= context.ccd_processed_time + time_tolerance
                            && context.ccd_processed_pairs.contains(&pair)
                        {
                            self.collision_system.context.ccd_toi_cache[candidate_index] =
                                crate::collision::CcdCandidateToi {
                                    evaluated: true,
                                    ..Default::default()
                                };
                            continue;
                        }
                        let (collider_a, collider_b) = {
                            let collider_a = self
                                .colliders
                                .get(pair.a.dense)
                                .filter(|_| pair.a.world == self.world_id)
                                .ok_or(CollisionError::InvalidContact)?;
                            let collider_b = self
                                .colliders
                                .get(pair.b.dense)
                                .filter(|_| pair.b.world == self.world_id)
                                .ok_or(CollisionError::InvalidContact)?;
                            (collider_a, collider_b)
                        };
                        let body_a = self
                            .bodies
                            .get(collider_a.body.dense)
                            .filter(|_| collider_a.body.world == self.world_id)
                            .map(|entry| entry.body)
                            .ok_or(CollisionError::InvalidContact)?;
                        let body_b = self
                            .bodies
                            .get(collider_b.body.dense)
                            .filter(|_| collider_b.body.world == self.world_id)
                            .map(|entry| entry.body)
                            .ok_or(CollisionError::InvalidContact)?;
                        if ![body_a, body_b].into_iter().any(|body| {
                            body.physics().body_type() == BodyType::Dynamic
                                && body.ccd_mode() == CcdMode::Continuous
                        }) {
                            self.collision_system.context.ccd_toi_cache[candidate_index]
                                .evaluated = true;
                            continue;
                        }

                        let prediction_a = Self::predicted_body_index(
                            &self.collision_system.context.body_indices,
                            self.world_id,
                            collider_a.body,
                        )?;
                        let prediction_b = Self::predicted_body_index(
                            &self.collision_system.context.body_indices,
                            self.world_id,
                            collider_b.body,
                        )?;
                        let state_a =
                            self.body_state_at_ccd(collider_a.body, prediction_a, elapsed, dt)?;
                        let state_b =
                            self.body_state_at_ccd(collider_b.body, prediction_b, elapsed, dt)?;
                        let end_state_a = self
                            .collision_system
                            .context
                            .predicted_states
                            .get(prediction_a)
                            .map(|(_, state)| *state)
                            .ok_or(CollisionError::InvalidContact)?;
                        let end_state_b = self
                            .collision_system
                            .context
                            .predicted_states
                            .get(prediction_b)
                            .map(|(_, state)| *state)
                            .ok_or(CollisionError::InvalidContact)?;
                        let start_transform_a = state_a
                            .transform()
                            .compose(&collider_a.collider.local_transform())
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let start_transform_b = state_b
                            .transform()
                            .compose(&collider_b.collider.local_transform())
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let end_transform_a = end_state_a
                            .transform()
                            .compose(&collider_a.collider.local_transform())
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let end_transform_b = end_state_b
                            .transform()
                            .compose(&collider_b.collider.local_transform())
                            .map_err(|_| CollisionError::ArithmeticOverflow)?;
                        let local_transform_a = collider_a.collider.local_transform();
                        let local_transform_b = collider_b.collider.local_transform();
                        let context = &self.collision_system.context;
                        let trajectory_a = CcdTrajectory::new(
                            body_a,
                            context.ccd_states[prediction_a],
                            context.ccd_elapsed[prediction_a],
                            elapsed,
                            dt,
                            local_transform_a,
                            context.ccd_clipped_bodies[prediction_a],
                        );
                        let trajectory_b = CcdTrajectory::new(
                            body_b,
                            context.ccd_states[prediction_b],
                            context.ccd_elapsed[prediction_b],
                            elapsed,
                            dt,
                            local_transform_b,
                            context.ccd_clipped_bodies[prediction_b],
                        );
                        let (translation_bound_a, rotation_bound_a, linear_translation_a) =
                            trajectory_a.path_bounds(end_state_a)?;
                        let (translation_bound_b, rotation_bound_b, linear_translation_b) =
                            trajectory_b.path_bounds(end_state_b)?;
                        let motion_a = CcdMotion::trajectory(
                            start_transform_a,
                            end_transform_a,
                            translation_bound_a,
                            linear_translation_a,
                            rotation_bound_a,
                            trajectory_a,
                        )?;
                        let motion_b = CcdMotion::trajectory(
                            start_transform_b,
                            end_transform_b,
                            translation_bound_b,
                            linear_translation_b,
                            rotation_bound_b,
                            trajectory_b,
                        )?;
                        let sensor = collider_a.collider.mode()
                            == crate::collider::CollisionMode::Sensor
                            || collider_b.collider.mode() == crate::collider::CollisionMode::Sensor;
                        #[cfg(feature = "bench-internals")]
                        let repeated_toi_call = if let Some(seen_pairs) =
                            &mut self.collision_system.context.ccd_toi_seen_pairs
                        {
                            !seen_pairs.insert(pair)
                        } else {
                            false
                        };
                        #[cfg(feature = "bench-internals")]
                        if repeated_toi_call {
                            self.collision_system.record_ccd_repeated_toi_call();
                        }
                        let hit = self.collision_system.time_of_impact(
                            collider_a.collider.shape(),
                            &motion_a,
                            collider_a.collider.query_bvh(),
                            collider_a.collider.query_vertices(),
                            collider_b.collider.shape(),
                            &motion_b,
                            collider_b.collider.query_bvh(),
                            collider_b.collider.query_vertices(),
                            sensor,
                        )?;
                        let cached = self
                            .collision_system
                            .context
                            .ccd_toi_cache
                            .get_mut(candidate_index)
                            .ok_or(CollisionError::InvalidContact)?;
                        cached.evaluated = true;
                        cached.hit = None;
                        let Some(hit) = hit else {
                            continue;
                        };
                        if !hit.toi.is_finite() || !(0.0..=1.0).contains(&hit.toi) {
                            return Err(CollisionError::InvalidContact);
                        }
                        if sensor {
                            upsert_interaction(
                                &mut self.collision_system.context.interactions,
                                &mut self.collision_system.context.interaction_indices,
                                CollisionInteraction {
                                    pair,
                                    sensor: true,
                                    transient: true,
                                    contact: None,
                                    impact: false,
                                },
                            )?;
                            continue;
                        }
                        if hit.toi <= f64::EPSILON * 128.0 {
                            let velocity_a = body_point_velocity(body_a, state_a, hit.point_a)?;
                            let velocity_b = body_point_velocity(body_b, state_b, hit.point_b)?;
                            let normal_speed = velocity_b
                                .checked_sub(velocity_a)
                                .and_then(|velocity| velocity.checked_dot(hit.normal))
                                .ok_or(CollisionError::ArithmeticOverflow)?;
                            if normal_speed >= 0.0 {
                                continue;
                            }
                        }
                        let impact_time = elapsed + remaining * hit.toi;
                        cached.impact_time = impact_time;
                        cached.hit = Some(hit);
                        self.collision_system
                            .set_ccd_candidate_active(candidate_index, true)?;
                        if earliest.is_none_or(|(current_pair, _, current_time)| {
                            impact_time < current_time
                                || (impact_time == current_time && pair < current_pair)
                        }) {
                            earliest = Some((pair, hit, impact_time));
                        }
                    }

                    #[cfg(feature = "bench-internals")]
                    self.collision_system
                        .record_ccd_toi_search(toi_search_started.elapsed());

                    let Some((pair, hit, impact_elapsed)) = earliest else {
                        break;
                    };

                    let (body_a_id, body_b_id) = {
                        let collider_a = self
                            .colliders
                            .get(pair.a.dense)
                            .filter(|_| pair.a.world == self.world_id)
                            .ok_or(CollisionError::InvalidContact)?;
                        let collider_b = self
                            .colliders
                            .get(pair.b.dense)
                            .filter(|_| pair.b.world == self.world_id)
                            .ok_or(CollisionError::InvalidContact)?;
                        (collider_a.body, collider_b.body)
                    };
                    let prediction_a = Self::predicted_body_index(
                        &self.collision_system.context.body_indices,
                        self.world_id,
                        body_a_id,
                    )?;
                    let prediction_b = Self::predicted_body_index(
                        &self.collision_system.context.body_indices,
                        self.world_id,
                        body_b_id,
                    )?;
                    if impact_elapsed < elapsed {
                        return Err(CollisionError::NonConvergent);
                    }
                    let (body_a, body_b) = {
                        (
                            self.bodies
                                .get(body_a_id.dense)
                                .filter(|_| body_a_id.world == self.world_id)
                                .map(|entry| entry.body)
                                .ok_or(CollisionError::InvalidContact)?,
                            self.bodies
                                .get(body_b_id.dense)
                                .filter(|_| body_b_id.world == self.world_id)
                                .map(|entry| entry.body)
                                .ok_or(CollisionError::InvalidContact)?,
                        )
                    };
                    #[cfg(feature = "bench-internals")]
                    let contact_resolution_started = Instant::now();
                    let state_a =
                        self.body_state_at_ccd(body_a_id, prediction_a, impact_elapsed, dt)?;
                    let state_b =
                        self.body_state_at_ccd(body_b_id, prediction_b, impact_elapsed, dt)?;
                    let body_transform_a = state_a.transform();
                    let body_transform_b = state_b.transform();
                    let separation = hit
                        .point_b
                        .checked_sub(hit.point_a)
                        .and_then(|delta| delta.checked_dot(hit.normal))
                        .ok_or(CollisionError::ArithmeticOverflow)?;
                    let contact = ContactPoint::from_world(
                        &body_transform_a,
                        &body_transform_b,
                        hit.point_a,
                        hit.point_b,
                        hit.normal,
                        (-separation).max(0.0),
                        0,
                    )?;
                    let contact_position = hit
                        .point_a
                        .checked_add(hit.point_b)
                        .and_then(|point| point.checked_mul_scalar(0.5))
                        .ok_or(CollisionError::ArithmeticOverflow)?;
                    let mut manifold = ContactManifold::new(pair);
                    manifold.insert(contact, contact_position)?;

                    let (body_a, body_b, mode_a, mode_b, material_a, material_b) = {
                        let collider_a = self
                            .colliders
                            .get(pair.a.dense)
                            .ok_or(CollisionError::InvalidContact)?;
                        let collider_b = self
                            .colliders
                            .get(pair.b.dense)
                            .ok_or(CollisionError::InvalidContact)?;
                        (
                            body_a,
                            body_b,
                            collider_a.collider.mode(),
                            collider_b.collider.mode(),
                            collider_a.collider.material(),
                            collider_b.collider.material(),
                        )
                    };
                    if mode_a == crate::collider::CollisionMode::Sensor
                        || mode_b == crate::collider::CollisionMode::Sensor
                    {
                        return Err(CollisionError::InvalidContact);
                    }
                    let mut budget_exhausted = false;
                    for (body, prediction) in [(body_a, prediction_a), (body_b, prediction_b)] {
                        if body.physics().body_type() == BodyType::Dynamic
                            && body.ccd_mode() == CcdMode::Continuous
                        {
                            let impacts =
                                self.collision_system.context.ccd_impact_counts[prediction];
                            budget_exhausted |=
                                impacts >= self.collision_system.config().max_ccd_impacts;
                        }
                    }
                    let already_clipped = self.collision_system.context.ccd_clipped_bodies
                        [prediction_a]
                        || self.collision_system.context.ccd_clipped_bodies[prediction_b];
                    let body_a_dynamic = body_a.physics().body_type() == BodyType::Dynamic;
                    let body_b_dynamic = body_b.physics().body_type() == BodyType::Dynamic;
                    let limit_policy = self.collision_system.config().ccd_limit_policy;
                    if budget_exhausted && limit_policy == CcdLimitPolicy::AbortStep {
                        return Err(CollisionError::CcdLimitExceeded);
                    }
                    if budget_exhausted || already_clipped {
                        self.clip_ccd_contact_bodies(
                            [
                                (body_a_id, prediction_a, body_a_dynamic),
                                (body_b_id, prediction_b, body_b_dynamic),
                            ],
                            impact_elapsed,
                            dt,
                        )?;
                        let has_prior_impact = self
                            .collision_system
                            .context
                            .interaction_indices
                            .as_ref()
                            .and_then(|indices| indices.get(&pair))
                            .and_then(|index| {
                                self.collision_system.context.interactions.get(*index)
                            })
                            .is_some_and(|interaction| interaction.impact);
                        if !has_prior_impact {
                            upsert_interaction(
                                &mut self.collision_system.context.interactions,
                                &mut self.collision_system.context.interaction_indices,
                                CollisionInteraction {
                                    pair,
                                    sensor: false,
                                    transient: false,
                                    contact: Some(ContactInfo {
                                        point: contact_position,
                                        normal: hit.normal,
                                        penetration: (-separation).max(0.0),
                                        normal_impulse: 0.0,
                                    }),
                                    impact: false,
                                },
                            )?;
                        }
                        #[cfg(feature = "bench-internals")]
                        self.collision_system.record_ccd_clipped_contact();
                        self.mark_ccd_pair_processed(pair, impact_elapsed, time_tolerance)?;
                        elapsed = impact_elapsed;
                        if dt - impact_elapsed > f64::EPSILON * dt.max(1.0)
                            && self.resweep_ccd_after_impact(
                                island_index,
                                [
                                    (body_a_id, prediction_a, body_a_dynamic),
                                    (body_b_id, prediction_b, body_b_dynamic),
                                ],
                                impact_elapsed,
                                dt,
                                time_tolerance,
                                &mut force_single_island,
                            )?
                        {
                            continue 'passes;
                        }
                        continue;
                    }
                    let material = self.combined_solver_material(material_a, material_b)?;
                    let (solved_a, solved_b, solved_manifold) = self
                        .collision_system
                        .solve_ccd_contact(body_a, state_a, body_b, state_b, manifold, material)?;
                    let mut normal_impulse = 0.0;
                    let mut impulse_delta = 0.0;
                    for point in solved_manifold.points() {
                        normal_impulse += point.normal_impulse;
                        impulse_delta += point.normal_impulse_increase_sum;
                    }
                    if !normal_impulse.is_finite() || !impulse_delta.is_finite() {
                        return Err(CollisionError::ArithmeticOverflow);
                    }
                    if impulse_delta <= 0.0 {
                        return Err(CollisionError::NonConvergent);
                    }
                    let solved_point = solved_manifold
                        .points()
                        .first()
                        .ok_or(CollisionError::InvalidContact)?;
                    let point_a = solved_a.transform().transform_point(solved_point.local_a);
                    let point_b = solved_b.transform().transform_point(solved_point.local_b);
                    let point = point_a
                        .checked_add(point_b)
                        .and_then(|sum| sum.checked_mul_scalar(0.5))
                        .ok_or(CollisionError::ArithmeticOverflow)?;
                    #[cfg(feature = "bench-internals")]
                    let island_commit_started = Instant::now();
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        CollisionInteraction {
                            pair,
                            sensor: false,
                            transient: false,
                            contact: Some(ContactInfo {
                                point,
                                normal: solved_point.normal,
                                penetration: solved_point.penetration,
                                normal_impulse,
                            }),
                            impact: true,
                        },
                    )?;
                    #[cfg(feature = "bench-internals")]
                    {
                        self.collision_system.record_ccd_impact();
                        self.collision_system
                            .record_ccd_contact_resolution(contact_resolution_started.elapsed());
                    }
                    let remaining_after_impact = (dt - impact_elapsed).max(0.0);
                    for (body, prediction, solved) in [
                        (body_a, prediction_a, solved_a),
                        (body_b, prediction_b, solved_b),
                    ] {
                        if body.physics().body_type() != BodyType::Dynamic {
                            continue;
                        }
                        self.collision_system.context.ccd_states[prediction] = solved;
                        self.collision_system.context.ccd_elapsed[prediction] = impact_elapsed;
                        self.collision_system.context.predicted_states[prediction].1 =
                            if remaining_after_impact <= f64::EPSILON * dt.max(1.0) {
                                solved
                            } else {
                                let mut body = body;
                                body.set_state_preserving_forces(solved);
                                predict(&body, remaining_after_impact)
                                    .map_err(|_| CollisionError::ArithmeticOverflow)?
                            };
                    }
                    if body_a.physics().body_type() == BodyType::Dynamic
                        && body_a.ccd_mode() == CcdMode::Continuous
                    {
                        self.collision_system.context.ccd_impact_counts[prediction_a] += 1;
                    }
                    if body_b.physics().body_type() == BodyType::Dynamic
                        && body_b.ccd_mode() == CcdMode::Continuous
                    {
                        self.collision_system.context.ccd_impact_counts[prediction_b] += 1;
                    }
                    #[cfg(feature = "bench-internals")]
                    self.collision_system
                        .record_ccd_island_commit(island_commit_started.elapsed());
                    self.mark_ccd_pair_processed(pair, impact_elapsed, time_tolerance)?;
                    elapsed = impact_elapsed;
                    if remaining_after_impact > f64::EPSILON * dt.max(1.0)
                        && self.resweep_ccd_after_impact(
                            island_index,
                            [
                                (body_a_id, prediction_a, body_a_dynamic),
                                (body_b_id, prediction_b, body_b_dynamic),
                            ],
                            impact_elapsed,
                            dt,
                            time_tolerance,
                            &mut force_single_island,
                        )?
                    {
                        continue 'passes;
                    }
                }
                self.collision_system
                    .mark_ccd_island_completed(island_index)?;
                island_index += 1;
            }
            break;
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_island_execute(ccd_execute_started.elapsed());
        #[cfg(feature = "bench-internals")]
        {
            self.collision_system
                .record_ccd_final_candidate_count(self.collision_system.candidate_count());
            let impact_count = self
                .collision_system
                .context
                .interactions
                .iter()
                .filter(|interaction| interaction.impact)
                .count();
            self.collision_system
                .record_ccd_final_impact_count(impact_count);
        }
        Ok(())
    }

    fn resolve_single_candidate_ccd_islands_parallel(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
        time_tolerance: f64,
        force_single_island: &mut bool,
    ) -> Result<(), CollisionError> {
        const MIN_PARALLEL_ISLANDS: usize = 4;

        let island_count = self.collision_system.ccd_island_count();
        if worker_count <= 1 || island_count < MIN_PARALLEL_ISLANDS {
            return Ok(());
        }

        {
            let results = &mut self.collision_system.context.ccd_island_results;
            if results.len() < island_count {
                results
                    .try_reserve(island_count - results.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                results.resize_with(island_count, CcdIslandResult::default);
            }
            for result in results.iter_mut().take(island_count) {
                result.clear();
            }
        }
        let mut eligible_count = 0;
        let mut total_cost = 0_usize;
        for island in 0..island_count {
            let candidates = self
                .collision_system
                .ccd_island_candidates(island)
                .ok_or(CollisionError::InvalidContact)?;
            let cost = if candidates.len() == 1 {
                eligible_count += 1;
                self.ccd_candidate_work_cost(candidates[0])?
            } else {
                0
            };
            total_cost = total_cost.saturating_add(cost);
            self.collision_system.context.ccd_island_results[island].work_cost = cost;
        }
        if eligible_count < MIN_PARALLEL_ISLANDS || total_cost < 256 {
            return Ok(());
        }

        let active_workers = worker_count.min(eligible_count);
        self.collision_system
            .context
            .ccd_worker_chunks
            .try_reserve(active_workers)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let mut chunk_sizes = std::mem::take(&mut self.collision_system.context.ccd_worker_chunks);
        chunk_sizes.clear();
        let mut start = 0;
        let mut remaining_cost = total_cost;
        for worker in 0..active_workers {
            let workers_left = active_workers - worker;
            let target = remaining_cost.div_ceil(workers_left).max(1);
            let mut chunk_cost = 0;
            let mut end = start;
            while end < island_count
                && (chunk_cost < target || chunk_cost == 0)
                && island_count - end > workers_left - 1
            {
                chunk_cost = chunk_cost.saturating_add(
                    self.collision_system.context.ccd_island_results[end].work_cost,
                );
                end += 1;
            }
            if worker + 1 == active_workers {
                end = island_count;
                chunk_cost = remaining_cost;
            }
            if end == start {
                return Err(CollisionError::InvalidContact);
            }
            chunk_sizes.push(end - start);
            start = end;
            remaining_cost = remaining_cost.saturating_sub(chunk_cost);
        }

        let mut results = std::mem::take(&mut self.collision_system.context.ccd_island_results);
        let shared_world = &*self;
        let computed = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            executor.for_each_chunk_layout_mut(
                &mut results[..island_count],
                &chunk_sizes,
                |chunk_start, chunk| {
                    let mut narrow_phase = NarrowPhase::new();
                    for (offset, result) in chunk.iter_mut().enumerate() {
                        let island = chunk_start + offset;
                        let cost = result.work_cost;
                        if cost == 0 {
                            continue;
                        }
                        result.clear();
                        result.work_cost = cost;
                        if let Err(error) = shared_world.prepare_single_candidate_ccd_island(
                            island,
                            dt,
                            &mut narrow_phase,
                            result,
                        ) {
                            result.worker_error = Some(error);
                            break;
                        }
                    }
                },
            )
        }));
        self.collision_system.context.ccd_island_results = results;
        self.collision_system.context.ccd_worker_chunks = chunk_sizes;
        if !matches!(computed, Ok(Ok(()))) {
            return Err(CollisionError::WorkerFailed);
        }
        if let Some(error) = self
            .collision_system
            .context
            .ccd_island_results
            .iter()
            .take(island_count)
            .find_map(|result| result.worker_error)
        {
            return Err(error);
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_parallel_islands(eligible_count, active_workers);

        let mut layout_changed = false;
        for island in 0..island_count {
            let weight = self.collision_system.context.ccd_island_results[island].work_cost;
            if weight == 0 {
                continue;
            }
            let candidate_count = self.collision_system.context.ccd_island_results[island]
                .candidates
                .len();
            for index in 0..candidate_count {
                let candidate =
                    self.collision_system.context.ccd_island_results[island].candidates[index];
                let previous =
                    self.collision_system.context.ccd_candidate_active[candidate.candidate];
                self.collision_system.context.ccd_toi_cache[candidate.candidate] = candidate.toi;
                self.collision_system
                    .set_ccd_candidate_active(candidate.candidate, candidate.toi.hit.is_some())?;
                layout_changed |= previous != candidate.toi.hit.is_some();
                if let Some(interaction) = candidate.interaction {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        interaction,
                    )?;
                }
                #[cfg(feature = "bench-internals")]
                if candidate.evaluated_now {
                    self.collision_system
                        .record_ccd_toi_evaluation(candidate.pair);
                }
                #[cfg(not(feature = "bench-internals"))]
                let _ = candidate.evaluated_now;
            }
        }
        if layout_changed {
            self.collision_system.build_ccd_islands(false)?;
        }

        for island in 0..island_count {
            let weight = self.collision_system.context.ccd_island_results[island].work_cost;
            if weight == 0 || !self.collision_system.context.ccd_island_results[island].handled {
                continue;
            }
            let result = &self.collision_system.context.ccd_island_results[island];
            for body in &result.bodies {
                if let Some(completed) = self
                    .collision_system
                    .context
                    .ccd_completed_bodies
                    .get_mut(body.prediction)
                {
                    *completed = true;
                }
            }
            let Some(impact) = result.impact else {
                continue;
            };

            #[cfg(feature = "bench-internals")]
            let commit_started = Instant::now();
            upsert_interaction(
                &mut self.collision_system.context.interactions,
                &mut self.collision_system.context.interaction_indices,
                impact.interaction,
            )?;
            for ((body_id, prediction, dynamic), solved) in [
                (impact.bodies[0], impact.solved_a),
                (impact.bodies[1], impact.solved_b),
            ] {
                if !dynamic {
                    continue;
                }
                self.collision_system.context.ccd_states[prediction] = solved;
                self.collision_system.context.ccd_elapsed[prediction] = impact.impact_elapsed;
                let remaining_after_impact = (dt - impact.impact_elapsed).max(0.0);
                self.collision_system.context.predicted_states[prediction].1 =
                    if remaining_after_impact <= f64::EPSILON * dt.max(1.0) {
                        solved
                    } else {
                        let mut body = self
                            .bodies
                            .get(body_id.dense)
                            .filter(|_| body_id.world == self.world_id)
                            .map(|entry| entry.body)
                            .ok_or(CollisionError::InvalidContact)?;
                        body.set_state_preserving_forces(solved);
                        predict(&body, remaining_after_impact)
                            .map_err(|_| CollisionError::ArithmeticOverflow)?
                    };
            }
            for (body_id, prediction, _) in impact.bodies {
                let body = self
                    .bodies
                    .get(body_id.dense)
                    .filter(|_| body_id.world == self.world_id)
                    .map(|entry| entry.body)
                    .ok_or(CollisionError::InvalidContact)?;
                if body.physics().body_type() == BodyType::Dynamic
                    && body.ccd_mode() == CcdMode::Continuous
                {
                    self.collision_system.context.ccd_impact_counts[prediction] += 1;
                }
            }
            self.mark_ccd_pair_processed(impact.pair, impact.impact_elapsed, time_tolerance)?;
            #[cfg(feature = "bench-internals")]
            {
                self.collision_system.record_ccd_impact();
                self.collision_system
                    .record_ccd_contact_resolution(impact.solve_time);
                self.collision_system
                    .record_ccd_island_commit(commit_started.elapsed());
            }

            if dt - impact.impact_elapsed <= f64::EPSILON * dt.max(1.0) {
                continue;
            }
            let island_index = impact
                .bodies
                .iter()
                .filter(|body| body.2)
                .find_map(|body| self.collision_system.ccd_body_island(body.1))
                .ok_or(CollisionError::InvalidContact)?;
            let previous_candidate_count = self.collision_system.ccd_candidate_count();
            if self.resweep_ccd_after_impact(
                island_index,
                impact.bodies,
                impact.impact_elapsed,
                dt,
                time_tolerance,
                force_single_island,
            )? {
                return Ok(());
            }
            let current_island = impact
                .bodies
                .iter()
                .filter(|body| body.2)
                .find_map(|body| self.collision_system.ccd_body_island(body.1));
            if current_island.is_some_and(|current| {
                self.collision_system
                    .ccd_island_candidates(current)
                    .is_some_and(|candidates| {
                        self.collision_system.ccd_candidate_count() > previous_candidate_count
                            || candidates.len() > 1
                    })
            }) {
                let current_island = current_island.ok_or(CollisionError::InvalidContact)?;
                self.collision_system
                    .mark_ccd_island_incomplete(current_island)?;
            }
        }
        Ok(())
    }

    fn ccd_candidate_work_cost(&self, candidate: usize) -> Result<usize, CollisionError> {
        let pair = self
            .collision_system
            .ccd_candidate_pair(candidate)
            .ok_or(CollisionError::InvalidContact)?;
        let collider_a = self
            .colliders
            .get(pair.a.dense)
            .filter(|_| pair.a.world == self.world_id)
            .ok_or(CollisionError::InvalidContact)?;
        let collider_b = self
            .colliders
            .get(pair.b.dense)
            .filter(|_| pair.b.world == self.world_id)
            .ok_or(CollisionError::InvalidContact)?;
        let cost = match (collider_a.collider.shape(), collider_b.collider.shape()) {
            (Shape::Sphere(_), Shape::Sphere(_)) => 1,
            (Shape::Sphere(_), Shape::Cuboid(_)) | (Shape::Cuboid(_), Shape::Sphere(_)) => 2,
            (Shape::TriangleMesh(_), _) | (_, Shape::TriangleMesh(_)) => 16,
            (Shape::HeightField(_), _) | (_, Shape::HeightField(_)) => 12,
            (Shape::LevelSet(_), _) | (_, Shape::LevelSet(_)) => 12,
            _ => 8,
        };
        Ok(cost)
    }

    fn prepare_single_candidate_ccd_island(
        &self,
        island: usize,
        dt: f64,
        narrow_phase: &mut NarrowPhase,
        result: &mut CcdIslandResult,
    ) -> Result<(), CollisionError> {
        self.snapshot_ccd_island_bodies(island, result)?;
        let candidates = self
            .collision_system
            .ccd_island_candidates(island)
            .ok_or(CollisionError::InvalidContact)?;
        let [candidate] = candidates else {
            return Ok(());
        };
        let pair = self
            .collision_system
            .ccd_candidate_pair(*candidate)
            .ok_or(CollisionError::InvalidContact)?;
        let cached = self
            .collision_system
            .context
            .ccd_toi_cache
            .get(*candidate)
            .copied()
            .ok_or(CollisionError::InvalidContact)?;
        let evaluated_now = !cached.evaluated;
        let (toi, interaction) = if !evaluated_now {
            (cached, None)
        } else {
            self.evaluate_ccd_candidate(pair, 0.0, dt, narrow_phase)?
        };
        result
            .candidates
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        result.candidates.push(CcdIslandCandidateResult {
            candidate: *candidate,
            pair,
            toi,
            interaction,
            evaluated_now,
        });
        if let Some(interaction) = interaction
            && interaction.sensor
        {
            result.handled = true;
            return Ok(());
        }
        let Some(hit) = toi.hit else {
            result.handled = true;
            return Ok(());
        };

        let (body_a_id, body_b_id) = {
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            (collider_a.body, collider_b.body)
        };
        let prediction_a = Self::predicted_body_index(
            &self.collision_system.context.body_indices,
            self.world_id,
            body_a_id,
        )?;
        let prediction_b = Self::predicted_body_index(
            &self.collision_system.context.body_indices,
            self.world_id,
            body_b_id,
        )?;
        let (body_a, body_b) = (
            self.bodies
                .get(body_a_id.dense)
                .filter(|_| body_a_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?,
            self.bodies
                .get(body_b_id.dense)
                .filter(|_| body_b_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?,
        );
        let exhausted = [(body_a, prediction_a), (body_b, prediction_b)]
            .into_iter()
            .any(|(body, prediction)| {
                body.physics().body_type() == BodyType::Dynamic
                    && body.ccd_mode() == CcdMode::Continuous
                    && self.collision_system.context.ccd_impact_counts[prediction]
                        >= self.collision_system.config().max_ccd_impacts
            });
        if exhausted
            || self.collision_system.context.ccd_clipped_bodies[prediction_a]
            || self.collision_system.context.ccd_clipped_bodies[prediction_b]
        {
            return Ok(());
        }

        let impact_elapsed = toi.impact_time;
        let state_a = self.body_state_at_ccd(body_a_id, prediction_a, impact_elapsed, dt)?;
        let state_b = self.body_state_at_ccd(body_b_id, prediction_b, impact_elapsed, dt)?;
        let separation = hit
            .point_b
            .checked_sub(hit.point_a)
            .and_then(|delta| delta.checked_dot(hit.normal))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let contact = ContactPoint::from_world(
            &state_a.transform(),
            &state_b.transform(),
            hit.point_a,
            hit.point_b,
            hit.normal,
            (-separation).max(0.0),
            0,
        )?;
        let contact_position = hit
            .point_a
            .checked_add(hit.point_b)
            .and_then(|point| point.checked_mul_scalar(0.5))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let mut manifold = ContactManifold::new(pair);
        manifold.insert(contact, contact_position)?;
        let (mode_a, mode_b, material_a, material_b) = {
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .ok_or(CollisionError::InvalidContact)?;
            (
                collider_a.collider.mode(),
                collider_b.collider.mode(),
                collider_a.collider.material(),
                collider_b.collider.material(),
            )
        };
        if mode_a == crate::collider::CollisionMode::Sensor
            || mode_b == crate::collider::CollisionMode::Sensor
        {
            return Err(CollisionError::InvalidContact);
        }
        let material = self.combined_solver_material(material_a, material_b)?;
        #[cfg(feature = "bench-internals")]
        let solve_started = Instant::now();
        let (solved_a, solved_b, solved_manifold) =
            CollisionSystem::solve_ccd_contact_with_workspace(
                &mut result.solver,
                body_a,
                state_a,
                body_b,
                state_b,
                manifold,
                material,
                self.collision_system.config(),
            )?;
        #[cfg(feature = "bench-internals")]
        let solve_time = solve_started.elapsed();
        let mut normal_impulse = 0.0;
        let mut impulse_delta = 0.0;
        for point in solved_manifold.points() {
            normal_impulse += point.normal_impulse;
            impulse_delta += point.normal_impulse_increase_sum;
        }
        if !normal_impulse.is_finite() || !impulse_delta.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if impulse_delta <= 0.0 {
            return Err(CollisionError::NonConvergent);
        }
        let solved_point = solved_manifold
            .points()
            .first()
            .ok_or(CollisionError::InvalidContact)?;
        let point_a = solved_a.transform().transform_point(solved_point.local_a);
        let point_b = solved_b.transform().transform_point(solved_point.local_b);
        let point = point_a
            .checked_add(point_b)
            .and_then(|sum| sum.checked_mul_scalar(0.5))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        result.impact = Some(CcdIslandImpactResult {
            pair,
            bodies: [
                (
                    body_a_id,
                    prediction_a,
                    body_a.physics().body_type() == BodyType::Dynamic,
                ),
                (
                    body_b_id,
                    prediction_b,
                    body_b.physics().body_type() == BodyType::Dynamic,
                ),
            ],
            impact_elapsed,
            solved_a,
            solved_b,
            interaction: CollisionInteraction {
                pair,
                sensor: false,
                transient: false,
                contact: Some(ContactInfo {
                    point,
                    normal: solved_point.normal,
                    penetration: solved_point.penetration,
                    normal_impulse,
                }),
                impact: true,
            },
            #[cfg(feature = "bench-internals")]
            solve_time,
        });
        result.handled = true;
        Ok(())
    }

    fn precompute_ccd_island(
        &self,
        island: usize,
        dt: f64,
        narrow_phase: &mut NarrowPhase,
        result: &mut CcdIslandResult,
    ) -> Result<(), CollisionError> {
        self.snapshot_ccd_island_bodies(island, result)?;
        let candidates = self
            .collision_system
            .ccd_island_candidates(island)
            .ok_or(CollisionError::InvalidContact)?;
        result
            .candidates
            .try_reserve(candidates.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for candidate_index in candidates.iter().copied() {
            let pair = self
                .collision_system
                .ccd_candidate_pair(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let (toi, interaction) = self.evaluate_ccd_candidate(pair, 0.0, dt, narrow_phase)?;
            result.candidates.push(CcdIslandCandidateResult {
                candidate: candidate_index,
                pair,
                toi,
                interaction,
                evaluated_now: true,
            });
        }
        Ok(())
    }

    fn snapshot_ccd_island_bodies(
        &self,
        island: usize,
        result: &mut CcdIslandResult,
    ) -> Result<(), CollisionError> {
        let bodies = self
            .collision_system
            .ccd_island_bodies(island)
            .ok_or(CollisionError::InvalidContact)?;
        result
            .bodies
            .try_reserve(bodies.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for prediction in bodies.iter().copied() {
            let context = &self.collision_system.context;
            result.bodies.push(CcdIslandBodyResult {
                prediction,
                ccd_state: *context
                    .ccd_states
                    .get(prediction)
                    .ok_or(CollisionError::InvalidContact)?,
                predicted_state: context
                    .predicted_states
                    .get(prediction)
                    .map(|(_, state)| *state)
                    .ok_or(CollisionError::InvalidContact)?,
                elapsed: *context
                    .ccd_elapsed
                    .get(prediction)
                    .ok_or(CollisionError::InvalidContact)?,
                impact_count: *context
                    .ccd_impact_counts
                    .get(prediction)
                    .ok_or(CollisionError::InvalidContact)?,
                clipped: context
                    .ccd_clipped_bodies
                    .get(prediction)
                    .copied()
                    .ok_or(CollisionError::InvalidContact)?,
            });
        }
        Ok(())
    }

    fn mark_ccd_pair_processed(
        &mut self,
        pair: CollisionPair,
        impact_elapsed: f64,
        time_tolerance: f64,
    ) -> Result<(), CollisionError> {
        let context = &mut self.collision_system.context;
        if impact_elapsed > context.ccd_processed_time + time_tolerance {
            context.ccd_processed_pairs.clear();
            context.ccd_processed_time = impact_elapsed;
        }
        if context.ccd_processed_pairs.contains(&pair) {
            return Err(CollisionError::NonConvergent);
        }
        context
            .ccd_processed_pairs
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context.ccd_processed_pairs.push(pair);
        Ok(())
    }

    fn resweep_ccd_after_impact(
        &mut self,
        island_index: usize,
        bodies: [(BodyId, usize, bool); 2],
        impact_elapsed: f64,
        dt: f64,
        time_tolerance: f64,
        force_single_island: &mut bool,
    ) -> Result<bool, CollisionError> {
        self.collision_system.invalidate_ccd_candidates(
            bodies[0].1,
            bodies[0].2,
            bodies[1].1,
            bodies[1].2,
        )?;
        let invalidated_count = self.collision_system.ccd_resweep_candidate_count();
        let new_ccd_candidates =
            self.resweep_ccd_bodies(island_index, bodies, impact_elapsed, dt, invalidated_count)?;
        let reevaluate_count = self.collision_system.ccd_resweep_candidate_count();
        if reevaluate_count == 0 {
            return Ok(false);
        }

        self.collision_system.sort_ccd_resweep_candidates();
        let reevaluate_count = self.collision_system.ccd_resweep_candidate_count();
        let mut narrow_phase = NarrowPhase::new();
        let mut layout_changed = false;
        for index in 0..reevaluate_count {
            let candidate_index = self
                .collision_system
                .ccd_resweep_candidate(index)
                .ok_or(CollisionError::InvalidContact)?;
            let candidate_pair = self
                .collision_system
                .ccd_candidate_pair(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let (candidate_a, candidate_b) = self
                .collision_system
                .ccd_candidate_body_pair(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let candidate_is_cross_island = self.collision_system.context.ccd_body_types
                [candidate_a]
                == BodyType::Dynamic
                && self.collision_system.context.ccd_body_types[candidate_b] == BodyType::Dynamic
                && self.collision_system.ccd_body_island(candidate_a)
                    != self.collision_system.ccd_body_island(candidate_b);
            if candidate_is_cross_island {
                self.replay_ccd_component(candidate_index, dt, force_single_island)?;
                return Ok(true);
            }
            let candidate_elapsed = impact_elapsed
                .max(self.collision_system.context.ccd_elapsed[candidate_a])
                .max(self.collision_system.context.ccd_elapsed[candidate_b]);
            if impact_elapsed <= self.collision_system.context.ccd_processed_time + time_tolerance
                && self
                    .collision_system
                    .context
                    .ccd_processed_pairs
                    .contains(&candidate_pair)
            {
                self.collision_system.context.ccd_toi_cache[candidate_index] = CcdCandidateToi {
                    evaluated: true,
                    ..Default::default()
                };
                continue;
            }
            remove_transient_sensor_interaction(
                &mut self.collision_system.context.interactions,
                &mut self.collision_system.context.interaction_indices,
                candidate_pair,
            )?;
            let (toi, interaction) = self.evaluate_ccd_candidate(
                candidate_pair,
                candidate_elapsed,
                dt,
                &mut narrow_phase,
            )?;
            #[cfg(feature = "bench-internals")]
            self.collision_system.record_ccd_local_recalculation(1);
            self.collision_system.context.ccd_toi_cache[candidate_index] = toi;
            if toi.hit.is_some() {
                layout_changed |=
                    !self.collision_system.context.ccd_candidate_active[candidate_index];
                self.collision_system
                    .set_ccd_candidate_active(candidate_index, true)?;
            }
            if let Some(interaction) = interaction {
                upsert_interaction(
                    &mut self.collision_system.context.interactions,
                    &mut self.collision_system.context.interaction_indices,
                    interaction,
                )?;
            }
            #[cfg(feature = "bench-internals")]
            self.collision_system
                .record_ccd_toi_evaluation(candidate_pair);
        }
        if new_ccd_candidates > 0 || layout_changed {
            self.collision_system
                .build_ccd_islands(*force_single_island)?;
        }
        Ok(false)
    }

    fn replay_ccd_component(
        &mut self,
        bridge_candidate: usize,
        dt: f64,
        force_single_island: &mut bool,
    ) -> Result<(), CollisionError> {
        let (body_a, _body_b) = self
            .collision_system
            .ccd_candidate_body_pair(bridge_candidate)
            .ok_or(CollisionError::InvalidContact)?;
        #[cfg(feature = "bench-internals")]
        let prior_island_a = self.collision_system.ccd_body_island(body_a);
        #[cfg(feature = "bench-internals")]
        let prior_island_b = self.collision_system.ccd_body_island(_body_b);
        if !self
            .collision_system
            .context
            .ccd_deferred_candidates
            .contains(&bridge_candidate)
        {
            self.collision_system
                .context
                .ccd_deferred_candidates
                .try_reserve(1)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.collision_system
                .context
                .ccd_deferred_candidates
                .push(bridge_candidate);
        }
        self.collision_system
            .set_ccd_candidate_active(bridge_candidate, true)?;
        self.collision_system.build_ccd_islands(false)?;

        let island = self
            .collision_system
            .ccd_body_island(body_a)
            .ok_or(CollisionError::InvalidContact)?;
        self.collision_system.context.ccd_replay_bodies.clear();
        self.collision_system
            .append_ccd_island_bodies_to_replay(island)?;

        let prediction_count = self.collision_system.context.predicted_states.len();
        for replay_pass in 0..=prediction_count {
            let replay_count = self.collision_system.context.ccd_replay_bodies.len();
            for index in 0..replay_count {
                let prediction = self.collision_system.context.ccd_replay_bodies[index];
                let body_id = self
                    .collision_system
                    .context
                    .predicted_states
                    .get(prediction)
                    .map(|(id, _)| *id)
                    .ok_or(CollisionError::InvalidContact)?;
                let body = self
                    .bodies
                    .get(body_id.dense)
                    .filter(|_| body_id.world == self.world_id)
                    .map(|entry| entry.body)
                    .ok_or(CollisionError::InvalidContact)?;
                let end_state =
                    predict(&body, dt).map_err(|_| CollisionError::ArithmeticOverflow)?;
                let context = &mut self.collision_system.context;
                context.predicted_states[prediction].1 = end_state;
                context.ccd_states[prediction] = *body.state();
                context.ccd_elapsed[prediction] = 0.0;
                context.ccd_impact_counts[prediction] = 0;
                context.ccd_clipped_bodies[prediction] = false;
                context.ccd_completed_bodies[prediction] = false;
            }

            for index in 0..replay_count {
                let prediction = self.collision_system.context.ccd_replay_bodies[index];
                let body_id = self
                    .collision_system
                    .context
                    .predicted_states
                    .get(prediction)
                    .map(|(id, _)| *id)
                    .ok_or(CollisionError::InvalidContact)?;
                self.collision_system
                    .invalidate_ccd_candidates(prediction, true, prediction, false)?;
                let invalidated = self.collision_system.ccd_resweep_candidate_count();
                self.resweep_ccd_bodies(
                    island,
                    [(body_id, prediction, true), (body_id, prediction, false)],
                    0.0,
                    dt,
                    invalidated,
                )?;
            }
            self.collision_system.build_ccd_islands(false)?;
            self.collision_system.collect_ccd_replay_candidates()?;
            let candidate_count = self.collision_system.ccd_resweep_candidate_count();
            let context = &mut self.collision_system.context;
            context.ccd_replay_pairs.clear();
            context
                .ccd_replay_pairs
                .try_reserve(candidate_count)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            for index in 0..candidate_count {
                let candidate = *context
                    .ccd_resweep_candidates
                    .get(index)
                    .ok_or(CollisionError::InvalidContact)?;
                let pair = context
                    .ccd_candidate_pairs
                    .get(candidate)
                    .copied()
                    .ok_or(CollisionError::InvalidContact)?;
                context.ccd_replay_pairs.push(pair);
                *context
                    .ccd_toi_cache
                    .get_mut(candidate)
                    .ok_or(CollisionError::InvalidContact)? = CcdCandidateToi::default();
                *context
                    .ccd_candidate_active
                    .get_mut(candidate)
                    .ok_or(CollisionError::InvalidContact)? = false;
            }
            context.ccd_replay_pairs.sort_unstable();
            context.ccd_replay_pairs.dedup();
            remove_interactions_for_pairs(
                &mut context.interactions,
                &mut context.interaction_indices,
                &context.ccd_replay_pairs,
            )?;
            context.ccd_processed_pairs.clear();
            context.ccd_processed_time = 0.0;

            let mut narrow_phase = NarrowPhase::new();
            let mut expanded = false;
            for index in 0..candidate_count {
                let candidate = *self
                    .collision_system
                    .context
                    .ccd_resweep_candidates
                    .get(index)
                    .ok_or(CollisionError::InvalidContact)?;
                let pair = self
                    .collision_system
                    .ccd_candidate_pair(candidate)
                    .ok_or(CollisionError::InvalidContact)?;
                let (candidate_a, candidate_b) = self
                    .collision_system
                    .ccd_candidate_body_pair(candidate)
                    .ok_or(CollisionError::InvalidContact)?;
                let outside = self.ccd_external_dynamic_body(candidate_a, candidate_b);
                let saved = if let Some(prediction) = outside {
                    Some(self.reset_ccd_body_for_replay(prediction, dt)?)
                } else {
                    None
                };
                let (toi, interaction) =
                    self.evaluate_ccd_candidate(pair, 0.0, dt, &mut narrow_phase)?;
                let deferred_bridge = toi.hit.is_none()
                    && self
                        .collision_system
                        .context
                        .ccd_deferred_candidates
                        .contains(&candidate);
                if toi.hit.is_some() || deferred_bridge {
                    self.collision_system
                        .set_ccd_candidate_active(candidate, true)?;
                    if let Some(prediction) = outside {
                        let bodies = &mut self.collision_system.context.ccd_replay_bodies;
                        if !bodies.contains(&prediction) {
                            bodies
                                .try_reserve(1)
                                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                            bodies.push(prediction);
                            expanded = true;
                        }
                    }
                } else {
                    self.collision_system
                        .set_ccd_candidate_active(candidate, false)?;
                    if let Some(saved) = saved {
                        self.restore_ccd_body_after_replay(saved)?;
                    }
                }
                self.collision_system.context.ccd_toi_cache[candidate] = if deferred_bridge {
                    CcdCandidateToi::default()
                } else {
                    toi
                };
                if let Some(interaction) = interaction {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        interaction,
                    )?;
                }
                #[cfg(feature = "bench-internals")]
                {
                    self.collision_system.record_ccd_local_recalculation(1);
                    self.collision_system.record_ccd_toi_evaluation(pair);
                }
                if expanded {
                    break;
                }
            }
            self.collision_system.build_ccd_islands(false)?;

            let replay_count = self.collision_system.context.ccd_replay_bodies.len();
            for index in 0..replay_count {
                let prediction = self.collision_system.context.ccd_replay_bodies[index];
                if let Some(island) = self.collision_system.ccd_body_island(prediction) {
                    let member_count = self
                        .collision_system
                        .ccd_island_bodies(island)
                        .ok_or(CollisionError::InvalidContact)?
                        .len();
                    for member_index in 0..member_count {
                        let member = self
                            .collision_system
                            .ccd_island_bodies(island)
                            .and_then(|members| members.get(member_index))
                            .copied()
                            .ok_or(CollisionError::InvalidContact)?;
                        if !self
                            .collision_system
                            .context
                            .ccd_replay_bodies
                            .contains(&member)
                        {
                            self.collision_system
                                .context
                                .ccd_replay_bodies
                                .try_reserve(1)
                                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                            self.collision_system.context.ccd_replay_bodies.push(member);
                            expanded = true;
                        }
                    }
                }
            }
            if !expanded {
                #[cfg(feature = "bench-internals")]
                self.collision_system.record_ccd_local_replay(
                    usize::from(prior_island_a.is_some()) + usize::from(prior_island_b.is_some()),
                );
                return Ok(());
            }
            self.collision_system
                .context
                .ccd_replay_bodies
                .sort_unstable();
            if replay_pass == prediction_count {
                *force_single_island = true;
                #[cfg(feature = "bench-internals")]
                self.collision_system.record_ccd_global_fallback(
                    self.collision_system.ccd_candidate_count(),
                    self.collision_system.ccd_island_count(),
                );
                self.reset_ccd_pass(dt, true)?;
                return Ok(());
            }
        }
        Err(CollisionError::NonConvergent)
    }

    fn ccd_external_dynamic_body(&self, body_a: usize, body_b: usize) -> Option<usize> {
        let bodies = &self.collision_system.context.ccd_replay_bodies;
        let selected_a = bodies.contains(&body_a);
        let selected_b = bodies.contains(&body_b);
        match (
            self.collision_system.context.ccd_body_types[body_a] == BodyType::Dynamic,
            self.collision_system.context.ccd_body_types[body_b] == BodyType::Dynamic,
            selected_a,
            selected_b,
        ) {
            (true, true, true, false) => Some(body_b),
            (true, true, false, true) => Some(body_a),
            _ => None,
        }
    }

    fn reset_ccd_body_for_replay(
        &mut self,
        prediction: usize,
        dt: f64,
    ) -> Result<CcdBodyReplaySnapshot, CollisionError> {
        let context = &self.collision_system.context;
        let body_id = context
            .predicted_states
            .get(prediction)
            .map(|(id, _)| *id)
            .ok_or(CollisionError::InvalidContact)?;
        let saved = CcdBodyReplaySnapshot {
            prediction,
            ccd_state: context.ccd_states[prediction],
            predicted_state: context.predicted_states[prediction].1,
            elapsed: context.ccd_elapsed[prediction],
            impact_count: context.ccd_impact_counts[prediction],
            clipped: context.ccd_clipped_bodies[prediction],
            completed: context.ccd_completed_bodies[prediction],
        };
        let body = self
            .bodies
            .get(body_id.dense)
            .filter(|_| body_id.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let end_state = predict(&body, dt).map_err(|_| CollisionError::ArithmeticOverflow)?;
        let context = &mut self.collision_system.context;
        context.ccd_states[prediction] = *body.state();
        context.predicted_states[prediction].1 = end_state;
        context.ccd_elapsed[prediction] = 0.0;
        context.ccd_impact_counts[prediction] = 0;
        context.ccd_clipped_bodies[prediction] = false;
        context.ccd_completed_bodies[prediction] = false;
        Ok(saved)
    }

    fn restore_ccd_body_after_replay(
        &mut self,
        saved: CcdBodyReplaySnapshot,
    ) -> Result<(), CollisionError> {
        let CcdBodyReplaySnapshot {
            prediction,
            ccd_state,
            predicted_state,
            elapsed,
            impact_count,
            clipped,
            completed,
        } = saved;
        let context = &mut self.collision_system.context;
        *context
            .ccd_states
            .get_mut(prediction)
            .ok_or(CollisionError::InvalidContact)? = ccd_state;
        context
            .predicted_states
            .get_mut(prediction)
            .ok_or(CollisionError::InvalidContact)?
            .1 = predicted_state;
        context.ccd_elapsed[prediction] = elapsed;
        context.ccd_impact_counts[prediction] = impact_count;
        context.ccd_clipped_bodies[prediction] = clipped;
        context.ccd_completed_bodies[prediction] = completed;
        Ok(())
    }

    fn precompute_ccd_tois_parallel(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
    ) -> Result<(), CollisionError> {
        let island_count = self.collision_system.ccd_island_count();
        if worker_count <= 1 || island_count <= 1 {
            return Ok(());
        }
        // Cuboid GJK sweeps first repay task-dispatch overhead at 256 candidates.
        const MIN_PARALLEL_EXPENSIVE_TOIS: usize = 256;
        if self.expensive_ccd_candidate_count()? < MIN_PARALLEL_EXPENSIVE_TOIS {
            return Ok(());
        }
        let worker_count = worker_count.min(island_count);
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        let chunk_size = island_count.div_ceil(worker_count);
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_worker_tasks_started(island_count.div_ceil(chunk_size));

        {
            let results = &mut self.collision_system.context.ccd_island_results;
            if results.len() < island_count {
                results
                    .try_reserve(island_count - results.len())
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                results.resize_with(island_count, CcdIslandResult::default);
            }
            for result in results.iter_mut().take(island_count) {
                result.clear();
            }
        }
        let mut results = std::mem::take(&mut self.collision_system.context.ccd_island_results);
        debug_assert!(results.len() >= island_count);
        let shared_world = &*self;
        let evaluated = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            executor.for_each_chunk_mut(
                &mut results[..island_count],
                chunk_size,
                |worker_index, island_results| {
                    let start = worker_index * chunk_size;
                    let mut narrow_phase = NarrowPhase::new();
                    for (island, result) in
                        (start..start + island_results.len()).zip(island_results)
                    {
                        result.clear();
                        if let Err(error) = shared_world.precompute_ccd_island(
                            island,
                            dt,
                            &mut narrow_phase,
                            result,
                        ) {
                            result.worker_error = Some(error);
                            break;
                        }
                    }
                },
            )
        }));
        self.collision_system.context.ccd_island_results = results;
        if !matches!(evaluated, Ok(Ok(()))) {
            return Err(CollisionError::WorkerFailed);
        }
        if let Some(error) = self
            .collision_system
            .context
            .ccd_island_results
            .iter()
            .take(island_count)
            .find_map(|result| result.worker_error)
        {
            return Err(error);
        }

        for island in 0..island_count {
            let body_count = self.collision_system.context.ccd_island_results[island]
                .bodies
                .len();
            for body_index in 0..body_count {
                let body =
                    self.collision_system.context.ccd_island_results[island].bodies[body_index];
                let context = &mut self.collision_system.context;
                context.ccd_states[body.prediction] = body.ccd_state;
                context.predicted_states[body.prediction].1 = body.predicted_state;
                context.ccd_elapsed[body.prediction] = body.elapsed;
                context.ccd_impact_counts[body.prediction] = body.impact_count;
                context.ccd_clipped_bodies[body.prediction] = body.clipped;
            }
            let candidate_count = self.collision_system.context.ccd_island_results[island]
                .candidates
                .len();
            for candidate_index in 0..candidate_count {
                let result = self.collision_system.context.ccd_island_results[island].candidates
                    [candidate_index];
                *self
                    .collision_system
                    .context
                    .ccd_toi_cache
                    .get_mut(result.candidate)
                    .ok_or(CollisionError::InvalidContact)? = result.toi;
                self.collision_system
                    .set_ccd_candidate_active(result.candidate, result.toi.hit.is_some())?;
                if let Some(interaction) = result.interaction {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        interaction,
                    )?;
                }
                #[cfg(feature = "bench-internals")]
                self.collision_system.record_ccd_toi_evaluation(result.pair);
                #[cfg(not(feature = "bench-internals"))]
                let _ = result.pair;
            }
            debug_assert!(
                self.collision_system.context.ccd_island_results[island]
                    .dependencies
                    .is_empty()
            );
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_ccd_toi_search(started.elapsed());
        Ok(())
    }

    fn expensive_ccd_candidate_count(&self) -> Result<usize, CollisionError> {
        let mut count = 0_usize;
        for candidate_index in 0..self.collision_system.ccd_candidate_count() {
            let pair = self
                .collision_system
                .ccd_candidate_pair(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let mut rotational_motion = [false; 2];
            for (index, body_id) in [collider_a.body, collider_b.body].into_iter().enumerate() {
                let body = self
                    .bodies
                    .get(body_id.dense)
                    .filter(|_| body_id.world == self.world_id)
                    .map(|entry| entry.body)
                    .ok_or(CollisionError::InvalidContact)?;
                let prediction = Self::predicted_body_index(
                    &self.collision_system.context.body_indices,
                    self.world_id,
                    body_id,
                )?;
                let end_state = self
                    .collision_system
                    .context
                    .predicted_states
                    .get(prediction)
                    .map(|(_, state)| *state)
                    .ok_or(CollisionError::InvalidContact)?;
                rotational_motion[index] = body.state().angular_velocity() != Vec3::ZERO
                    || end_state.angular_velocity() != Vec3::ZERO
                    || body.state().transform().rotation() != end_state.transform().rotation();
            }
            let analytic_pair = match (collider_a.collider.shape(), collider_b.collider.shape()) {
                (Shape::Sphere(_), Shape::Sphere(_)) => true,
                (Shape::Sphere(_), Shape::Cuboid(_)) => !rotational_motion[1],
                (Shape::Cuboid(_), Shape::Sphere(_)) => !rotational_motion[0],
                _ => false,
            };
            if !analytic_pair {
                count = count.saturating_add(1);
            }
        }
        Ok(count)
    }

    fn evaluate_ccd_candidate(
        &self,
        pair: CollisionPair,
        elapsed: f64,
        dt: f64,
        narrow_phase: &mut NarrowPhase,
    ) -> Result<(CcdCandidateToi, Option<CollisionInteraction>), CollisionError> {
        let (collider_a, collider_b) = {
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            (collider_a, collider_b)
        };
        if self
            .collision_system
            .context
            .suppresses_joint_collision(collider_a.body, collider_b.body)
        {
            return Ok((
                CcdCandidateToi {
                    evaluated: true,
                    ..Default::default()
                },
                None,
            ));
        }
        let body_a = self
            .bodies
            .get(collider_a.body.dense)
            .filter(|_| collider_a.body.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let body_b = self
            .bodies
            .get(collider_b.body.dense)
            .filter(|_| collider_b.body.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let mut cache = CcdCandidateToi {
            evaluated: true,
            ..Default::default()
        };
        if ![body_a, body_b].into_iter().any(|body| {
            body.physics().body_type() == BodyType::Dynamic
                && body.ccd_mode() == CcdMode::Continuous
        }) {
            return Ok((cache, None));
        }

        let prediction_a = Self::predicted_body_index(
            &self.collision_system.context.body_indices,
            self.world_id,
            collider_a.body,
        )?;
        let prediction_b = Self::predicted_body_index(
            &self.collision_system.context.body_indices,
            self.world_id,
            collider_b.body,
        )?;
        let state_a = self.body_state_at_ccd(collider_a.body, prediction_a, elapsed, dt)?;
        let state_b = self.body_state_at_ccd(collider_b.body, prediction_b, elapsed, dt)?;
        let end_state_a = self
            .collision_system
            .context
            .predicted_states
            .get(prediction_a)
            .map(|(_, state)| *state)
            .ok_or(CollisionError::InvalidContact)?;
        let end_state_b = self
            .collision_system
            .context
            .predicted_states
            .get(prediction_b)
            .map(|(_, state)| *state)
            .ok_or(CollisionError::InvalidContact)?;
        let start_transform_a = state_a
            .transform()
            .compose(&collider_a.collider.local_transform())
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let start_transform_b = state_b
            .transform()
            .compose(&collider_b.collider.local_transform())
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let end_transform_a = end_state_a
            .transform()
            .compose(&collider_a.collider.local_transform())
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let end_transform_b = end_state_b
            .transform()
            .compose(&collider_b.collider.local_transform())
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let trajectory_a = CcdTrajectory::new(
            body_a,
            self.collision_system.context.ccd_states[prediction_a],
            self.collision_system.context.ccd_elapsed[prediction_a],
            elapsed,
            dt,
            collider_a.collider.local_transform(),
            self.collision_system.context.ccd_clipped_bodies[prediction_a],
        );
        let trajectory_b = CcdTrajectory::new(
            body_b,
            self.collision_system.context.ccd_states[prediction_b],
            self.collision_system.context.ccd_elapsed[prediction_b],
            elapsed,
            dt,
            collider_b.collider.local_transform(),
            self.collision_system.context.ccd_clipped_bodies[prediction_b],
        );
        let (translation_bound_a, rotation_bound_a, linear_translation_a) =
            trajectory_a.path_bounds(end_state_a)?;
        let (translation_bound_b, rotation_bound_b, linear_translation_b) =
            trajectory_b.path_bounds(end_state_b)?;
        let motion_a = CcdMotion::trajectory(
            start_transform_a,
            end_transform_a,
            translation_bound_a,
            linear_translation_a,
            rotation_bound_a,
            trajectory_a,
        )?;
        let motion_b = CcdMotion::trajectory(
            start_transform_b,
            end_transform_b,
            translation_bound_b,
            linear_translation_b,
            rotation_bound_b,
            trajectory_b,
        )?;
        let sensor = collider_a.collider.mode() == crate::collider::CollisionMode::Sensor
            || collider_b.collider.mode() == crate::collider::CollisionMode::Sensor;
        let hit = narrow_phase.time_of_impact(
            collider_a.collider.shape(),
            &motion_a,
            collider_a.collider.query_bvh(),
            collider_a.collider.query_vertices(),
            collider_b.collider.shape(),
            &motion_b,
            collider_b.collider.query_bvh(),
            collider_b.collider.query_vertices(),
            sensor,
        )?;
        let Some(hit) = hit else {
            return Ok((cache, None));
        };
        if !hit.toi.is_finite() || !(0.0..=1.0).contains(&hit.toi) {
            return Err(CollisionError::InvalidContact);
        }
        if sensor {
            return Ok((
                cache,
                Some(CollisionInteraction {
                    pair,
                    sensor: true,
                    transient: true,
                    contact: None,
                    impact: false,
                }),
            ));
        }
        if hit.toi <= f64::EPSILON * 128.0 {
            let velocity_a = body_point_velocity(body_a, state_a, hit.point_a)?;
            let velocity_b = body_point_velocity(body_b, state_b, hit.point_b)?;
            let normal_speed = velocity_b
                .checked_sub(velocity_a)
                .and_then(|velocity| velocity.checked_dot(hit.normal))
                .ok_or(CollisionError::ArithmeticOverflow)?;
            if normal_speed >= 0.0 {
                return Ok((cache, None));
            }
        }
        cache.impact_time = elapsed + (dt - elapsed) * hit.toi;
        cache.hit = Some(hit);
        Ok((cache, None))
    }

    fn rebuild_ccd_candidate_bodies(&mut self) -> Result<(), CollisionError> {
        let candidate_count = self.collision_system.candidate_count();
        let context = &mut self.collision_system.context;
        context.ccd_candidate_bodies.clear();
        context.ccd_candidate_pairs.clear();
        context
            .ccd_candidate_bodies
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context
            .ccd_candidate_pairs
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let candidate_set = context
            .ccd_candidate_pair_set
            .get_or_insert_with(std::collections::HashSet::new);
        candidate_set.clear();
        candidate_set
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for candidate_index in 0..candidate_count {
            let pair = self
                .collision_system
                .candidate(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let body_a = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_a.body,
            )?;
            let body_b = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_b.body,
            )?;
            let context = &mut self.collision_system.context;
            context.ccd_candidate_pairs.push(pair);
            context.ccd_candidate_bodies.push((body_a, body_b));
            context
                .ccd_candidate_pair_set
                .as_mut()
                .ok_or(CollisionError::InvalidContact)?
                .insert(pair);
        }
        let context = &mut self.collision_system.context;
        let cache = &mut context.ccd_toi_cache;
        cache.clear();
        cache
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        cache.resize(
            candidate_count,
            crate::collision::CcdCandidateToi::default(),
        );
        context.ccd_candidate_active.clear();
        context
            .ccd_candidate_active
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context.ccd_candidate_active.resize(candidate_count, true);
        context.ccd_resweep_candidates.clear();
        context.ccd_query_results.clear();
        Ok(())
    }

    fn resweep_ccd_bodies(
        &mut self,
        _island_index: usize,
        bodies: [(BodyId, usize, bool); 2],
        elapsed: f64,
        dt: f64,
        _invalidated_count: usize,
    ) -> Result<usize, CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        let mut new_ccd_candidates = 0;
        #[cfg(feature = "bench-internals")]
        let mut colliders_visited = 0_usize;
        #[cfg(feature = "bench-internals")]
        let mut proxies_updated = 0_usize;
        #[cfg(feature = "bench-internals")]
        let mut found_interisland_dependency = false;
        let world_id = self.world_id;
        let (body_pool, colliders, collision_system, unbounded_query_colliders) = (
            &self.bodies,
            &self.colliders,
            &mut self.collision_system,
            &self.unbounded_query_colliders,
        );

        for (body_id, prediction_index, changed) in bodies {
            if !changed {
                continue;
            }
            let body_entry = body_pool
                .get(body_id.dense)
                .filter(|_| body_id.world == world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let body = body_entry.body;
            let end_state = collision_system
                .context
                .predicted_states
                .get(prediction_index)
                .map(|(_, state)| *state)
                .ok_or(CollisionError::InvalidContact)?;

            let colliders_of_body = collider_entries_of(world_id, body_pool, colliders, body_id)
                .ok_or(CollisionError::InvalidContact)?;
            for (collider_id, collider_body, collider) in colliders_of_body {
                if collider_id.world != world_id || collider_body != body_id {
                    return Err(CollisionError::InvalidContact);
                }
                let proxy = collision_system.ccd_proxy(collider_id)?;
                if proxy.body != body_id {
                    return Err(CollisionError::InvalidContact);
                }
                #[cfg(feature = "bench-internals")]
                {
                    colliders_visited = colliders_visited.saturating_add(1);
                    proxies_updated = proxies_updated.saturating_add(1);
                }
                let bounds = {
                    let trajectory = CcdTrajectory::new(
                        body,
                        collision_system.context.ccd_states[prediction_index],
                        collision_system.context.ccd_elapsed[prediction_index],
                        elapsed,
                        dt,
                        collider.local_transform(),
                        collision_system.context.ccd_clipped_bodies[prediction_index],
                    );
                    swept_bounds_for_trajectory(collider, &trajectory, end_state)?
                };
                let (query_count, _unbounded_colliders_visited) = collision_system
                    .prepare_ccd_resweep_proxy(collider_id, bounds, unbounded_query_colliders)?;
                #[cfg(feature = "bench-internals")]
                {
                    colliders_visited =
                        colliders_visited.saturating_add(_unbounded_colliders_visited);
                }
                for query_index in 0..query_count {
                    let other_id = collision_system
                        .ccd_query_result(query_index)
                        .ok_or(CollisionError::InvalidContact)?;
                    let other_proxy = collision_system.ccd_proxy(other_id)?;
                    if bounds.is_some_and(|bounds| {
                        other_proxy
                            .bounds
                            .is_some_and(|other_bounds| !bounds.intersects(&other_bounds))
                    }) {
                        continue;
                    }
                    if !collision_system.ccd_pair_allowed(collider_id, other_id)? {
                        continue;
                    }
                    let pair = CollisionPair::new(collider_id, other_id)
                        .ok_or(CollisionError::InvalidContact)?;
                    let other_prediction = Self::predicted_body_index(
                        &collision_system.context.body_indices,
                        world_id,
                        other_proxy.body,
                    )?;
                    let (body_a, body_b) = if pair.a == collider_id {
                        (prediction_index, other_prediction)
                    } else {
                        (other_prediction, prediction_index)
                    };
                    let (added, is_ccd) =
                        collision_system.add_ccd_candidate(pair, body_a, body_b)?;
                    if added && is_ccd {
                        new_ccd_candidates += 1;
                        #[cfg(feature = "bench-internals")]
                        if collision_system.context.ccd_body_types[other_prediction]
                            == BodyType::Dynamic
                            && collision_system.ccd_body_island(other_prediction)
                                != Some(_island_index)
                        {
                            found_interisland_dependency = true;
                        }
                    }
                }
            }
        }

        #[cfg(feature = "bench-internals")]
        collision_system.record_ccd_resweep(
            started.elapsed(),
            _invalidated_count,
            colliders_visited,
            proxies_updated,
            found_interisland_dependency,
        );
        Ok(new_ccd_candidates)
    }

    fn reset_ccd_pass(&mut self, dt: f64, force_single_island: bool) -> Result<(), CollisionError> {
        let prediction_count = self.collision_system.context.predicted_states.len();
        for prediction in 0..prediction_count {
            let body_id = self.collision_system.context.predicted_states[prediction].0;
            let body = self
                .bodies
                .get(body_id.dense)
                .filter(|_| body_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?;
            let end_state = predict(&body, dt).map_err(|_| CollisionError::ArithmeticOverflow)?;
            let context = &mut self.collision_system.context;
            context.predicted_states[prediction].1 = end_state;
            context.ccd_states[prediction] = *body.state();
            context.ccd_elapsed[prediction] = 0.0;
            context.ccd_impact_counts[prediction] = 0;
            context.ccd_clipped_bodies[prediction] = false;
            context.ccd_completed_bodies[prediction] = false;
        }
        {
            let context = &mut self.collision_system.context;
            context.ccd_processed_pairs.clear();
            context.ccd_processed_time = 0.0;
            context.ccd_toi_cache.fill(Default::default());
            context.ccd_candidate_active.fill(true);
            context.interactions.clear();
            context.colliders.clear();
            context.spatial_proxies.clear();
            if let Some(indices) = &mut context.interaction_indices {
                indices.clear();
            }
        }

        self.stage_spatial_proxies(dt)?;
        self.collision_system.synchronize_spatial_index()?;
        self.collision_system.build_ccd_islands(force_single_island)
    }

    fn body_state_at_ccd(
        &self,
        id: BodyId,
        prediction_index: usize,
        elapsed: f64,
        dt: f64,
    ) -> Result<BodyState, CollisionError> {
        let body = self
            .bodies
            .get(id.dense)
            .filter(|_| id.world == self.world_id)
            .map(|entry| entry.body)
            .ok_or(CollisionError::InvalidContact)?;
        let state_start = *self
            .collision_system
            .context
            .ccd_states
            .get(prediction_index)
            .ok_or(CollisionError::InvalidContact)?;
        let start_time = *self
            .collision_system
            .context
            .ccd_elapsed
            .get(prediction_index)
            .ok_or(CollisionError::InvalidContact)?;
        if elapsed < start_time {
            return Err(CollisionError::InvalidContact);
        }
        if elapsed == start_time {
            return Ok(state_start);
        }
        if self
            .collision_system
            .context
            .ccd_clipped_bodies
            .get(prediction_index)
            .copied()
            .ok_or(CollisionError::InvalidContact)?
        {
            return Ok(state_start);
        }
        let duration = elapsed - start_time;
        if dt - start_time <= 0.0 {
            return Err(CollisionError::InvalidContact);
        }
        predict_from_state(&body, state_start, duration)
            .map_err(|_| CollisionError::ArithmeticOverflow)
    }

    fn clip_ccd_contact_bodies(
        &mut self,
        bodies: [(BodyId, usize, bool); 2],
        elapsed: f64,
        dt: f64,
    ) -> Result<(), CollisionError> {
        let first_prediction = bodies[0].1;
        for (body_index, (body_id, prediction, changed)) in bodies.into_iter().enumerate() {
            if !changed
                || self.collision_system.context.ccd_body_types[prediction] != BodyType::Dynamic
                || (body_index > 0 && first_prediction == prediction)
            {
                continue;
            }
            let state = self.body_state_at_ccd(body_id, prediction, elapsed, dt)?;
            let mut body = self
                .bodies
                .get(body_id.dense)
                .filter(|_| body_id.world == self.world_id)
                .map(|entry| entry.body)
                .ok_or(CollisionError::InvalidContact)?;
            body.set_state_preserving_forces(state);
            body.set_linear_velocity(Vec3::ZERO)
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            body.set_angular_velocity(Vec3::ZERO)
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            let clipped_state = *body.state();
            let context = &mut self.collision_system.context;
            #[cfg(feature = "bench-internals")]
            let count_clipped_body = !context.ccd_clipped_metrics[prediction];
            context.ccd_states[prediction] = clipped_state;
            context.ccd_elapsed[prediction] = elapsed;
            context.ccd_clipped_bodies[prediction] = true;
            context.predicted_states[prediction].1 = clipped_state;
            #[cfg(feature = "bench-internals")]
            {
                context.ccd_clipped_metrics[prediction] = true;
            }
            #[cfg(feature = "bench-internals")]
            if count_clipped_body {
                self.collision_system.record_ccd_clipped_body();
            }
        }
        Ok(())
    }
}
