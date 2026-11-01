use std::{error::Error, io, time::Duration};

#[path = "../../../crates/kapitsa/examples/rocket_flight/mission_spec.rs"]
#[allow(dead_code)]
mod spec;

use kapitsa::{
    BodyId, ColliderId, ForceSourceId, JointId, PhysicsBatchResults, PhysicsClient,
    PhysicsCommandBuffer, PhysicsSnapshot, Simulation, SimulationConfig, TimeStepPolicy,
    collision::{CollisionEventKind, CollisionPair},
    math::Vec3,
    rigid_body::BodyState,
};

#[allow(unused_imports)]
pub(crate) use spec::{PLANET_RADIUS, STAGE_ONE_BURN_END, STAGE_TWO_BURN_END};
#[allow(dead_code)]
pub(crate) const FLAME_END: f64 = 2.5;

#[derive(Clone, Copy)]
pub(crate) struct SceneFrame {
    pub(crate) step: u64,
    pub(crate) simulation_time: f64,
    pub(crate) stage_one: Option<BodyState>,
    pub(crate) stage_two: Option<BodyState>,
}

#[derive(Clone, Copy)]
pub(crate) struct FlightConfig {
    pub(crate) warp: f64,
    pub(crate) maximum_steps: u64,
    pub(crate) fps: u32,
}

#[derive(Clone, Copy)]
struct FlightBodies {
    stage_one: Option<BodyId>,
    stage_two: Option<BodyId>,
}

struct FrameSampler {
    start_step: u64,
    next_frame: u64,
    frame_index: u64,
    fps: u32,
    flight_start: Duration,
}

impl FrameSampler {
    fn new(
        snapshot: &PhysicsSnapshot,
        fps: u32,
        flight_start: Duration,
        bodies: FlightBodies,
    ) -> (Self, SceneFrame) {
        let sampler = Self {
            start_step: snapshot.step(),
            next_frame: snapshot.step() + frame_step(1, fps),
            frame_index: 1,
            fps,
            flight_start,
        };
        (sampler, scene_frame(snapshot, bodies, flight_start))
    }

    fn observe(
        &mut self,
        snapshot: &PhysicsSnapshot,
        bodies: FlightBodies,
    ) -> Result<Option<SceneFrame>, Box<dyn Error>> {
        if snapshot.step() > self.next_frame {
            return Err(io::Error::other(format!(
                "snapshot stream skipped requested frame step {} (received {})",
                self.next_frame,
                snapshot.step()
            ))
            .into());
        }
        if snapshot.step() == self.next_frame {
            self.frame_index += 1;
            self.next_frame = self.start_step + frame_step(self.frame_index, self.fps);
            return Ok(Some(scene_frame(snapshot, bodies, self.flight_start)));
        }
        Ok(None)
    }
}

fn scene_frame(
    snapshot: &PhysicsSnapshot,
    bodies: FlightBodies,
    flight_start: Duration,
) -> SceneFrame {
    SceneFrame {
        step: snapshot.step(),
        simulation_time: snapshot
            .simulation_time()
            .saturating_sub(flight_start)
            .as_secs_f64(),
        stage_one: bodies.stage_one.and_then(|id| snapshot.body(id)).copied(),
        stage_two: bodies.stage_two.and_then(|id| snapshot.body(id)).copied(),
    }
}

fn frame_step(frame_index: u64, fps: u32) -> u64 {
    frame_index
        .saturating_mul(u64::from(spec::TICK_RATE))
        .div_ceil(u64::from(fps))
}

pub(crate) async fn run(config: FlightConfig) -> Result<Vec<SceneFrame>, Box<dyn Error>> {
    let physics_config = SimulationConfig {
        tick_rate: spec::TICK_RATE,
        max_catch_up_steps: 1,
        central_gravity: Some(spec::gravity()?),
        time_step_policy: TimeStepPolicy::Fixed,
        ..SimulationConfig::default()
    };
    let mut simulation = Simulation::new_from_config(physics_config)?;
    simulation.set_time_multiplier(config.warp)?;
    let mut physics = simulation.client();
    let mut collisions = physics.subscribe_collision_events();
    let mut results = physics.subscribe_batch_results();
    simulation.start()?;

    let flight = async {
        let (planet_collider, stage_one, stage_two, lower_collider, upper_collider, joint, engine_one, engine_two) =
            setup(&physics).await?;
        let mut initial_batch = physics.commands();
        initial_batch.set_force_source(
            engine_one,
            spec::engine_point(),
            Vec3::Y * spec::thrust(spec::Stage::One),
        );
        submit_batch(&physics, &mut results, initial_batch).await?;

        let initial = physics.snapshot();
        let flight_start = initial.simulation_time();
        let deadline_step = initial.step().saturating_add(config.maximum_steps);
        let mut bodies = FlightBodies {
            stage_one: Some(stage_one),
            stage_two: Some(stage_two),
        };
        let (mut sampler, initial_frame) = FrameSampler::new(&initial, config.fps, flight_start, bodies);
        let (frame_tx, mut frame_rx) = tokio::sync::mpsc::unbounded_channel();
        let recorder_client = physics.clone();
        let recorder = tokio::spawn(async move {
            let mut snapshots = recorder_client;
            loop {
                let snapshot = match snapshots.next_snapshot().await {
                    Ok(snapshot) => snapshot,
                    Err(kapitsa::PhysicsClientError::NotRunning) => return Ok(()),
                    Err(error) => return Err(error.to_string()),
                };
                if let Some(frame) = sampler.observe(&snapshot, bodies).map_err(|error| error.to_string())? {
                    frame_tx.send(Ok(frame)).map_err(|_| "frame receiver closed".to_owned())?;
                }
            }
        });
        let mut frames = vec![initial_frame];
        let mut snapshot = initial;
        let mut separated = false;
        let mut stage_two_cutoff = false;
        let mut stage_one_hit = false;
        let mut stage_two_hit = false;
        let mut next_mass_update = 0.1;

        while snapshot.step() < deadline_step && !(stage_one_hit && stage_two_hit) {
            tokio::select! {
                next = physics.next_snapshot() => {
                    snapshot = next?;
                    receive_frames(&mut frame_rx, &mut frames)?;
                    let elapsed = snapshot.simulation_time().saturating_sub(flight_start).as_secs_f64();
                    if !separated && elapsed >= STAGE_ONE_BURN_END {
                        let lower = snapshot.body(stage_one).copied().ok_or_else(|| io::Error::other("lower stage is absent before separation"))?;
                        let upper = snapshot.body(stage_two).copied().ok_or_else(|| io::Error::other("upper stage is absent before separation"))?;
                        let (lower_velocity, upper_velocity) = spec::separation_velocities(lower, upper)?;
                        let mut batch = physics.commands();
                        batch.remove_joint(joint);
                        batch.set_linear_velocity(stage_one, lower_velocity);
                        batch.set_linear_velocity(stage_two, upper_velocity);
                        batch.set_mass(
                            stage_one,
                            spec::mass_properties(
                                spec::Stage::One,
                                spec::stage_mass(
                                    spec::Stage::One,
                                    spec::burn_duration(spec::Stage::One),
                                ),
                            )?,
                        );
                        batch.set_force_source_enabled(engine_one, false);
                        batch.set_force_source(
                            engine_two,
                            spec::engine_point(),
                            Vec3::Y * spec::thrust(spec::Stage::Two),
                        );
                        submit_batch(&physics, &mut results, batch).await?;
                        physics.remove_force_source(engine_one).await?;
                        snapshot = physics.snapshot();
                        receive_frames(&mut frame_rx, &mut frames)?;
                        separated = true;
                        next_mass_update = STAGE_ONE_BURN_END + 0.1;
                    } else if !separated && elapsed >= next_mass_update {
                        let mass = spec::stage_mass(spec::Stage::One, elapsed);
                        let mut batch = physics.commands();
                        batch.set_mass(stage_one, spec::mass_properties(spec::Stage::One, mass)?);
                        submit_batch(&physics, &mut results, batch).await?;
                        snapshot = physics.snapshot();
                        receive_frames(&mut frame_rx, &mut frames)?;
                        next_mass_update += 0.1;
                    }
                    let elapsed = snapshot.simulation_time().saturating_sub(flight_start).as_secs_f64();
                    if separated && !stage_two_cutoff && elapsed >= STAGE_TWO_BURN_END {
                        let mut batch = physics.commands();
                        batch.set_mass(
                            stage_two,
                            spec::mass_properties(
                                spec::Stage::Two,
                                spec::stage_mass(
                                    spec::Stage::Two,
                                    spec::burn_duration(spec::Stage::Two),
                                ),
                            )?,
                        );
                        batch.set_force_source_enabled(engine_two, false);
                        submit_batch(&physics, &mut results, batch).await?;
                        snapshot = physics.snapshot();
                        receive_frames(&mut frame_rx, &mut frames)?;
                        stage_two_cutoff = true;
                    } else if separated && !stage_two_cutoff && elapsed >= next_mass_update {
                        let burn_time = elapsed - STAGE_ONE_BURN_END;
                        let mass = spec::stage_mass(spec::Stage::Two, burn_time);
                        let mut batch = physics.commands();
                        batch.set_mass(stage_two, spec::mass_properties(spec::Stage::Two, mass)?);
                        submit_batch(&physics, &mut results, batch).await?;
                        snapshot = physics.snapshot();
                        receive_frames(&mut frame_rx, &mut frames)?;
                        next_mass_update += 0.1;
                    }
                }
                event = collisions.next_batch() => {
                    let event = event?;
                    for collision in event.events() {
                        if !matches!(collision.kind, CollisionEventKind::Begin | CollisionEventKind::Impact)
                            || !pair_contains(collision.pair, planet_collider)
                        {
                            continue;
                        }
                        if bodies.stage_one.is_some_and(|_| pair_contains(collision.pair, lower_collider)) {
                            physics.remove_body(stage_one).await?;
                            bodies.stage_one = None;
                            stage_one_hit = true;
                        }
                        if bodies.stage_two.is_some_and(|_| pair_contains(collision.pair, upper_collider)) {
                            physics.remove_body(stage_two).await?;
                            bodies.stage_two = None;
                            stage_two_hit = true;
                        }
                        snapshot = physics.snapshot();
                        receive_frames(&mut frame_rx, &mut frames)?;
                    }
                }
            }
        }
        recorder.abort();
        match recorder.await {
            Ok(Ok(())) => {}
            Ok(Err(error)) => return Err(io::Error::other(error).into()),
            Err(error) if error.is_cancelled() => {}
            Err(error) => return Err(Box::<dyn Error>::from(error)),
        }
        receive_frames(&mut frame_rx, &mut frames)?;
        frames.retain(|frame| frame.step <= deadline_step);
        if snapshot.step() <= deadline_step
            && frames.last().is_none_or(|frame| frame.step != snapshot.step())
        {
            frames.push(scene_frame(&snapshot, bodies, flight_start));
        }
        Ok::<_, Box<dyn Error>>(frames)
    }
    .await;

    let stopped = simulation.stop().await;
    let frames = flight?;
    stopped?;
    Ok(frames)
}

async fn setup(
    physics: &PhysicsClient,
) -> Result<
    (
        ColliderId,
        BodyId,
        BodyId,
        ColliderId,
        ColliderId,
        JointId,
        ForceSourceId,
        ForceSourceId,
    ),
    Box<dyn Error>,
> {
    let planet = physics.add_body(spec::planet_body()).await?;
    let planet_collider = physics
        .add_collider(planet, spec::planet_collider()?)
        .await?;
    let lower = physics
        .add_body(spec::stage_body(spec::Stage::One)?)
        .await?;
    let upper = physics
        .add_body(spec::stage_body(spec::Stage::Two)?)
        .await?;
    let lower_collider = physics.add_collider(lower, spec::stage_collider()?).await?;
    let upper_collider = physics.add_collider(upper, spec::stage_collider()?).await?;
    let joint = physics.add_joint(lower, upper, spec::fixed_joint()).await?;
    let engine_one = physics
        .add_force_source(lower, spec::engine_point(), Vec3::ZERO)
        .await?;
    let engine_two = physics
        .add_force_source(upper, spec::engine_point(), Vec3::ZERO)
        .await?;
    Ok((
        planet_collider,
        lower,
        upper,
        lower_collider,
        upper_collider,
        joint,
        engine_one,
        engine_two,
    ))
}

async fn submit_batch(
    physics: &PhysicsClient,
    results: &mut PhysicsBatchResults,
    batch: PhysicsCommandBuffer,
) -> Result<(), Box<dyn Error>> {
    let id = physics.try_submit(batch)?;
    let result = results.next_result().await?;
    if result.batch_id != id {
        return Err(io::Error::other("received an unexpected command result").into());
    }
    result
        .result
        .map_err(|error| io::Error::other(format!("flight batch failed: {error:?}")))?;
    Ok(())
}

fn receive_frames(
    receiver: &mut tokio::sync::mpsc::UnboundedReceiver<Result<SceneFrame, String>>,
    frames: &mut Vec<SceneFrame>,
) -> Result<(), Box<dyn Error>> {
    while let Ok(frame) = receiver.try_recv() {
        frames.push(frame.map_err(io::Error::other)?);
    }
    Ok(())
}

fn pair_contains(pair: CollisionPair, collider: ColliderId) -> bool {
    pair.a == collider || pair.b == collider
}
