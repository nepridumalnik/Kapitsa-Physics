use std::{error::Error, io, time::Duration};

use crate::camera::{PLANET_RADIUS, SceneFrame};
use kapitsa::{
    AdaptiveTimeStepConfig, BodyId, ColliderId, ForceSourceId, JointId, PhysicsBatchResults,
    PhysicsClient, PhysicsCommandBuffer, PhysicsSnapshot, Simulation, SimulationConfig,
    TimeStepPolicy, Transform,
    collider::Collider,
    collision::{CollisionEventKind, CollisionPair},
    gravity::CentralGravity,
    joint::{FixedJoint, Joint, JointCommon, JointFrame},
    mass::MassProperties,
    math::{Mat3, Quat, Vec3},
    rigid_body::RigidBody,
    shape::{Cuboid, Shape, Sphere},
};

const TICK_RATE: u32 = 120;
const PLANET_MU: f64 = 3.986_004_418e14;
const STAGE_ONE_MASS: f64 = 12_500.0;
const STAGE_TWO_MASS: f64 = 8_000.0;
const STAGE_ONE_THRUST: f64 = 300_000.0;
const STAGE_TWO_THRUST: f64 = 140_000.0;
pub(crate) const STAGE_ONE_BURN_END: f64 = 2.0;
pub(crate) const STAGE_TWO_BURN_END: f64 = 5.0;
const SEPARATION_IMPULSE: f64 = 16_000.0;
const STAGE_HALF_HEIGHT: f64 = 2.0;
pub(crate) const FLAME_END: f64 = 2.5;

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
        .saturating_mul(u64::from(TICK_RATE))
        .div_ceil(u64::from(fps))
}

pub(crate) async fn run(config: FlightConfig) -> Result<Vec<SceneFrame>, Box<dyn Error>> {
    let physics_config = SimulationConfig {
        tick_rate: TICK_RATE,
        max_catch_up_steps: 1,
        central_gravity: Some(CentralGravity::new(Vec3::ZERO, PLANET_MU)?),
        time_step_policy: TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default()),
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
            Vec3::Y * -STAGE_HALF_HEIGHT,
            Vec3::Y * STAGE_ONE_THRUST,
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

        while snapshot.step() < deadline_step && !(stage_one_hit && stage_two_hit) {
            tokio::select! {
                next = physics.next_snapshot() => {
                    snapshot = next?;
                    receive_frames(&mut frame_rx, &mut frames)?;
                    let elapsed = snapshot.simulation_time().saturating_sub(flight_start).as_secs_f64();
                    if !separated && elapsed >= STAGE_ONE_BURN_END {
                        let lower = snapshot.body(stage_one).copied().ok_or_else(|| io::Error::other("lower stage is absent before separation"))?;
                        let upper = snapshot.body(stage_two).copied().ok_or_else(|| io::Error::other("upper stage is absent before separation"))?;
                        let direction = upper.transform().rotation()
                            .rotate_vector(Vec3::Y)
                            .ok_or_else(|| io::Error::other("invalid rocket orientation"))?;
                        let mut batch = physics.commands();
                        batch.remove_joint(joint);
                        batch.set_linear_velocity(stage_one, lower.linear_velocity() - direction * (SEPARATION_IMPULSE / STAGE_ONE_MASS));
                        batch.set_linear_velocity(stage_two, upper.linear_velocity() + direction * (SEPARATION_IMPULSE / STAGE_TWO_MASS));
                        batch.set_force_source_enabled(engine_one, false);
                        batch.set_force_source(engine_two, Vec3::Y * -STAGE_HALF_HEIGHT, Vec3::Y * STAGE_TWO_THRUST);
                        submit_batch(&physics, &mut results, batch).await?;
                        physics.remove_force_source(engine_one).await?;
                        snapshot = physics.snapshot();
                        receive_frames(&mut frame_rx, &mut frames)?;
                        separated = true;
                    }
                    let elapsed = snapshot.simulation_time().saturating_sub(flight_start).as_secs_f64();
                    if separated && !stage_two_cutoff && elapsed >= STAGE_TWO_BURN_END {
                        let mut batch = physics.commands();
                        batch.set_force_source_enabled(engine_two, false);
                        submit_batch(&physics, &mut results, batch).await?;
                        snapshot = physics.snapshot();
                        receive_frames(&mut frame_rx, &mut frames)?;
                        stage_two_cutoff = true;
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
    let planet = physics
        .add_body(RigidBody::new_static(Transform::identity()))
        .await?;
    let planet_collider = physics
        .add_collider(
            planet,
            Collider::new(
                Shape::Sphere(Sphere::new(PLANET_RADIUS)?),
                Transform::identity(),
            ),
        )
        .await?;
    let lower = physics
        .add_body(stage_body(Vec3::Y * (PLANET_RADIUS + 3.0), STAGE_ONE_MASS)?)
        .await?;
    let upper = physics
        .add_body(stage_body(Vec3::Y * (PLANET_RADIUS + 7.0), STAGE_TWO_MASS)?)
        .await?;
    let lower_collider = physics
        .add_collider(
            lower,
            Collider::new(
                Shape::Cuboid(Cuboid::new(1.2, 2.0, 1.2)?),
                Transform::identity(),
            ),
        )
        .await?;
    let upper_collider = physics
        .add_collider(
            upper,
            Collider::new(
                Shape::Cuboid(Cuboid::new(1.2, 2.0, 1.2)?),
                Transform::identity(),
            ),
        )
        .await?;
    let common = JointCommon::new(
        JointFrame::new(Vec3::Y * STAGE_HALF_HEIGHT, Quat::IDENTITY),
        JointFrame::new(Vec3::Y * -STAGE_HALF_HEIGHT, Quat::IDENTITY),
    );
    let joint = physics
        .add_joint(lower, upper, Joint::Fixed(FixedJoint::new(common)))
        .await?;
    let engine_one = physics
        .add_force_source(lower, Vec3::Y * -STAGE_HALF_HEIGHT, Vec3::ZERO)
        .await?;
    let engine_two = physics
        .add_force_source(upper, Vec3::Y * -STAGE_HALF_HEIGHT, Vec3::ZERO)
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

fn stage_body(position: Vec3, mass: f64) -> Result<RigidBody, Box<dyn Error>> {
    let properties = MassProperties::new(
        mass,
        Vec3::ZERO,
        Mat3::from_diagonal(Vec3::new(1.0e5, 1.0e5, 1.0e5)),
    )?;
    Ok(RigidBody::new_dynamic(
        Transform::new(position, Quat::IDENTITY)?,
        properties,
    )?)
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

#[cfg(test)]
mod tests {
    use super::{FlightConfig, run};

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn records_physics_snapshots_on_the_requested_step_schedule() {
        let frames = run(FlightConfig {
            warp: 1.0,
            maximum_steps: 24,
            fps: 24,
        })
        .await
        .expect("flight snapshots");

        assert!(frames.len() >= 5);
        assert!(frames[0].stage_one.is_some());
        assert!(frames[0].stage_two.is_some());
        assert!(frames.last().expect("last frame").step - frames[0].step <= 24);
        assert!(
            frames
                .windows(2)
                .all(|pair| pair[1].step > pair[0].step && pair[1].step - pair[0].step <= 5)
        );
    }
}
