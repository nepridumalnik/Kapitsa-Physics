#[path = "rocket_flight/mission.rs"]
#[allow(dead_code)]
mod mission;

use std::{
    collections::VecDeque,
    io::{self, IsTerminal, Write},
    time::Duration,
};

use kapitsa::{
    PhysicsBatchResults, PhysicsClient, PhysicsCommandBuffer, Simulation, SimulationConfig,
    collision::CollisionEventKind, math::Vec3,
};
use mission::{FlightAction, MissionResult};
use tokio::{sync::watch, time::MissedTickBehavior};

const CONTROL_INTERVAL: Duration = Duration::from_millis(25);
const RENDER_INTERVAL: Duration = Duration::from_millis(125);
const TRAJECTORY_POINTS: usize = 72;

#[derive(Clone, Debug, Default)]
struct Telemetry {
    simulation_seconds: f64,
    altitude: f64,
    speed: f64,
    acceleration: f64,
    phase: &'static str,
    engine: &'static str,
    thrust: f64,
    fuel_fraction: f64,
    requested_warp: f64,
    actual_warp: f64,
    step: u64,
    step_cpu_us: u128,
    stage_one_altitude: Option<f64>,
    stage_two_altitude: Option<f64>,
    trajectory: VecDeque<f64>,
    last_event: String,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() -> MissionResult<()> {
    let (warp, force_plain) = arguments()?;
    let config = SimulationConfig {
        tick_rate: mission::TICK_RATE,
        max_catch_up_steps: 8,
        central_gravity: Some(mission::gravity()?),
        ..SimulationConfig::default()
    };
    let mut simulation = Simulation::new_from_config(config)?;
    simulation.set_time_multiplier(warp)?;
    let physics = simulation.client();
    let mut collisions = physics.subscribe_collision_events();
    let mut batch_results = physics.subscribe_batch_results();
    simulation.start()?;

    let (
        stack,
        planet_collider,
        mut stage_one_collider,
        mut stage_two_collider,
        stage_one_engine,
        stage_two_engine,
    ) = setup(&physics).await?;
    let flight_start = physics.snapshot().simulation_time();

    let (telemetry_tx, telemetry_rx) = watch::channel(Telemetry {
        requested_warp: warp,
        phase: "LAUNCH",
        engine: "S1",
        thrust: mission::STAGE_ONE_THRUST,
        ..Telemetry::default()
    });
    let use_ansi = !force_plain && io::stdout().is_terminal();
    let renderer = tokio::spawn(render(telemetry_rx, use_ansi));

    let mut interval = tokio::time::interval(CONTROL_INTERVAL);
    interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut separated = false;
    let mut cutoff = false;
    let mut stage_one_body = None;
    let mut stage_two_body = Some(stack);
    let mut stage_one_hit = false;
    let mut stage_two_hit = false;
    let mut trajectory = VecDeque::with_capacity(TRAJECTORY_POINTS);
    let mut previous_velocity = Vec3::ZERO;
    let mut previous_seconds = 0.0;
    let mut last_event = String::from("engines armed");

    loop {
        tokio::select! {
            _ = interval.tick() => {
                let snapshot = physics.snapshot();
                let seconds = snapshot.simulation_time().saturating_sub(flight_start).as_secs_f64();
                if seconds >= mission::MAX_FLIGHT_SECONDS {
                    last_event = String::from("mission time limit reached");
                    break;
                }

                if !separated && seconds >= mission::STAGE_ONE_BURN_END {
                    let state = *snapshot.body(stack).ok_or_else(|| io::Error::other("rocket snapshot disappeared"))?;
                    let (lower_transform, lower_velocity, upper_transform, upper_velocity) =
                        mission::separation_states(state)?;
                    let lower = physics
                        .add_body(mission::dynamic_body(
                            lower_transform,
                            mission::STAGE_ONE_MASS,
                            lower_velocity,
                        )?)
                        .await?;
                    let mut batch = physics.commands();
                    batch.set_force_source_enabled(stage_one_engine, false);
                    batch.set_mass(stack, mission::mass_properties(mission::STAGE_TWO_MASS)?);
                    batch.set_transform(stack, upper_transform);
                    batch.set_linear_velocity(stack, upper_velocity);
                    batch.set_force_source(
                        stage_two_engine,
                        mission::stage_two_engine_point_after_separation(),
                        Vec3::Y * mission::STAGE_TWO_THRUST,
                    );
                    submit_batch(&physics, &mut batch_results, batch).await?;
                    physics.remove_force_source(stage_one_engine).await?;
                    physics.remove_collider(stage_one_collider).await?;
                    physics.remove_collider(stage_two_collider).await?;
                    stage_one_collider = physics
                        .add_collider(lower, mission::detached_stage_collider()?)
                        .await?;
                    stage_two_collider = physics
                        .add_collider(stack, mission::detached_stage_collider()?)
                        .await?;
                    stage_one_body = Some(lower);
                    stage_two_body = Some(stack);
                    separated = true;
                    last_event = String::from("stage separation");
                }

                let snapshot = physics.snapshot();
                let seconds = snapshot.simulation_time().saturating_sub(flight_start).as_secs_f64();
                if separated && !cutoff && seconds >= mission::STAGE_TWO_BURN_END {
                    let mut batch = physics.commands();
                    batch.set_force_source_enabled(stage_two_engine, false);
                    submit_batch(&physics, &mut batch_results, batch).await?;
                    cutoff = true;
                    last_event = String::from("stage two cutoff");
                }

                let snapshot = physics.snapshot();
                let seconds = snapshot.simulation_time().saturating_sub(flight_start).as_secs_f64();
                let active_body = stage_two_body.or(stage_one_body);
                let (altitude, speed, acceleration) = if let Some(id) = active_body {
                    let state = snapshot.body(id).ok_or_else(|| io::Error::other("active stage missing from snapshot"))?;
                    let position = state.transform().translation();
                    let altitude = position.length() - mission::PLANET_RADIUS;
                    let velocity = state.linear_velocity();
                    let elapsed = seconds - previous_seconds;
                    let acceleration = if elapsed > 0.0 {
                        (velocity - previous_velocity).length() / elapsed
                    } else {
                        0.0
                    };
                    previous_velocity = velocity;
                    previous_seconds = seconds;
                    trajectory.push_back(altitude);
                    if trajectory.len() > TRAJECTORY_POINTS {
                        trajectory.pop_front();
                    }
                    (altitude, velocity.length(), acceleration)
                } else {
                    (0.0, 0.0, 0.0)
                };
                let stage_altitude = |id: Option<kapitsa::BodyId>| {
                    id.and_then(|body| snapshot.body(body))
                        .map(|state| state.transform().translation().length() - mission::PLANET_RADIUS)
                };
                let metrics = simulation.metrics();
                let action = mission::action_at(seconds, separated);
                let (phase, engine, thrust, fuel_fraction) = match action {
                    FlightAction::StageOnePowered => ("ASCENT", "S1", mission::STAGE_ONE_THRUST, (1.0 - seconds / mission::STAGE_ONE_BURN_END).clamp(0.0, 1.0)),
                    FlightAction::Separate => ("SEPARATION", "OFF", 0.0, 0.0),
                    FlightAction::StageTwoPowered => ("ASCENT", "S2", mission::STAGE_TWO_THRUST, (1.0 - (seconds - mission::STAGE_ONE_BURN_END) / (mission::STAGE_TWO_BURN_END - mission::STAGE_ONE_BURN_END)).clamp(0.0, 1.0)),
                    FlightAction::Coast => ("COAST", "OFF", 0.0, 0.0),
                };
                telemetry_tx.send_replace(Telemetry {
                    simulation_seconds: seconds,
                    altitude,
                    speed,
                    acceleration,
                    phase,
                    engine,
                    thrust,
                    fuel_fraction,
                    requested_warp: metrics.requested_time_multiplier,
                    actual_warp: metrics.actual_time_multiplier,
                    step: snapshot.step(),
                    step_cpu_us: metrics.last_step_duration.as_micros(),
                    stage_one_altitude: stage_altitude(stage_one_body),
                    stage_two_altitude: stage_altitude(stage_two_body),
                    trajectory: trajectory.clone(),
                    last_event: last_event.clone(),
                });

                if stage_one_hit && stage_two_hit {
                    break;
                }
            }
            event = collisions.next_batch() => {
                let batch = match event {
                    Ok(batch) => batch,
                    Err(error) => {
                        return Err(io::Error::other(format!("collision event stream failed: {error}")).into());
                    }
                };
                for event in batch.events() {
                    if !matches!(event.kind, CollisionEventKind::Begin | CollisionEventKind::Impact)
                        || !pair_contains(event.pair, planet_collider)
                    {
                        continue;
                    }
                    if pair_contains(event.pair, stage_one_collider)
                        && let Some(body) = stage_one_body.take()
                    {
                        physics.remove_body(body).await?;
                        stage_one_hit = true;
                        last_event = String::from("stage one impact; body removed");
                    }
                    if pair_contains(event.pair, stage_two_collider)
                        && let Some(body) = stage_two_body.take()
                    {
                        physics.remove_body(body).await?;
                        stage_two_hit = true;
                        last_event = String::from("stage two impact; body removed");
                    }
                }
            }
        }
    }

    telemetry_tx.send_modify(|telemetry| telemetry.last_event = last_event);
    simulation.stop().await?;
    renderer.abort();
    let _ = renderer.await;
    println!("flight complete: stage 1 impact={stage_one_hit}, stage 2 impact={stage_two_hit}");
    Ok(())
}

async fn setup(
    physics: &PhysicsClient,
) -> MissionResult<(
    kapitsa::BodyId,
    kapitsa::ColliderId,
    kapitsa::ColliderId,
    kapitsa::ColliderId,
    kapitsa::ForceSourceId,
    kapitsa::ForceSourceId,
)> {
    let planet = physics.add_body(mission::planet_body()).await?;
    let planet_collider = physics
        .add_collider(planet, mission::planet_collider()?)
        .await?;
    let stack = physics.add_body(mission::stack_body()?).await?;
    let stage_one_collider = physics
        .add_collider(
            stack,
            mission::stack_stage_collider(-mission::STAGE_HALF_HEIGHT)?,
        )
        .await?;
    let stage_two_collider = physics
        .add_collider(
            stack,
            mission::stack_stage_collider(mission::STAGE_HALF_HEIGHT)?,
        )
        .await?;
    let stage_one_engine = physics
        .add_force_source(
            stack,
            mission::stage_one_engine_point(),
            Vec3::Y * mission::STAGE_ONE_THRUST,
        )
        .await?;
    let stage_two_engine = physics
        .add_force_source(
            stack,
            mission::stage_two_engine_point_before_separation(),
            Vec3::ZERO,
        )
        .await?;
    Ok((
        stack,
        planet_collider,
        stage_one_collider,
        stage_two_collider,
        stage_one_engine,
        stage_two_engine,
    ))
}

async fn submit_batch(
    physics: &PhysicsClient,
    results: &mut PhysicsBatchResults,
    batch: PhysicsCommandBuffer,
) -> MissionResult<()> {
    let id = physics
        .try_submit(batch)
        .map_err(|error| io::Error::other(format!("command batch rejected: {error:?}")))?;
    let result = results.next_result().await?;
    if result.batch_id != id {
        return Err(io::Error::other("received an unexpected batch result").into());
    }
    result
        .result
        .map_err(|error| io::Error::other(format!("command batch failed: {error:?}")))?;
    Ok(())
}

async fn render(telemetry: watch::Receiver<Telemetry>, ansi: bool) -> io::Result<()> {
    let mut interval = tokio::time::interval(RENDER_INTERVAL);
    interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
    loop {
        interval.tick().await;
        let frame = telemetry.borrow().clone();
        draw_frame(&frame, ansi)?;
    }
}

fn draw_frame(frame: &Telemetry, ansi: bool) -> io::Result<()> {
    let mut grid = vec![vec![' '; 37]; 13];
    for (index, altitude) in frame.trajectory.iter().enumerate() {
        let row = altitude_to_row(*altitude);
        let col = (5 + index * 26 / TRAJECTORY_POINTS).min(35);
        if grid[row][col] == ' ' {
            grid[row][col] = '.';
        }
    }
    if let Some(altitude) = frame.stage_one_altitude {
        grid[altitude_to_row(altitude)][9] = '1';
    }
    if let Some(altitude) = frame.stage_two_altitude {
        grid[altitude_to_row(altitude)][27] = '2';
    }
    grid[11][15] = '[';
    grid[11][16] = 'P';
    grid[11][17] = ']';

    let mut stdout = io::stdout().lock();
    if ansi {
        write!(stdout, "\x1b[2J\x1b[H")?;
    }
    writeln!(stdout, "KAPITSA FLIGHT SIMULATION")?;
    for row in &grid {
        writeln!(stdout, "{}", row.iter().collect::<String>())?;
    }
    writeln!(
        stdout,
        "T+ {:6.2}s  ALT {:8.1} m  VEL {:7.2} m/s  ACC {:6.2} m/s²",
        frame.simulation_seconds, frame.altitude, frame.speed, frame.acceleration
    )?;
    writeln!(
        stdout,
        "STAGE {:12} ENGINE {} {:7.0} N  FUEL {:3.0}%",
        frame.phase,
        frame.engine,
        frame.thrust,
        frame.fuel_fraction * 100.0
    )?;
    writeln!(
        stdout,
        "WARP requested {:6.1}x  actual {:6.1}x  STEP {}  CPU {} us",
        frame.requested_warp, frame.actual_warp, frame.step, frame.step_cpu_us
    )?;
    writeln!(stdout, "EVENT: {}\n", frame.last_event)?;
    stdout.flush()
}

fn altitude_to_row(altitude: f64) -> usize {
    let scaled = (altitude.max(0.0).sqrt() / 4.0) as usize;
    10usize.saturating_sub(scaled.min(9))
}

fn pair_contains(pair: kapitsa::CollisionPair, collider: kapitsa::ColliderId) -> bool {
    pair.a == collider || pair.b == collider
}

fn arguments() -> MissionResult<(f64, bool)> {
    let mut warp = 1.0;
    let mut plain = false;
    for argument in std::env::args().skip(1) {
        if let Some(value) = argument.strip_prefix("--warp=") {
            warp = value.parse()?;
        } else if argument == "--no-ansi" {
            plain = true;
        } else {
            return Err(io::Error::other(format!("unknown argument: {argument}")).into());
        }
    }
    if ![0.1, 1.0, 10.0, 100.0, 1_000.0].contains(&warp) {
        return Err(io::Error::other("--warp must be 0.1, 1, 10, 100, or 1000").into());
    }
    Ok((warp, plain))
}

#[cfg(test)]
mod tests {
    use super::mission::{FlightAction, HeadlessFlight};

    #[test]
    fn deterministic_flight_separates_and_destroys_both_stages()
    -> Result<(), Box<dyn std::error::Error>> {
        let mut flight = HeadlessFlight::new()?;
        let mut saw_stage_two = false;
        let mut saw_coast = false;
        for _ in 0..(super::mission::MAX_FLIGHT_SECONDS / super::mission::STEP_SECONDS) as usize {
            match flight.step_once()? {
                FlightAction::StageTwoPowered => saw_stage_two = true,
                FlightAction::Coast => saw_coast = true,
                FlightAction::StageOnePowered | FlightAction::Separate => {}
            }
            if flight.is_complete() {
                break;
            }
        }
        assert!(saw_stage_two, "second stage never ignited");
        assert!(saw_coast, "flight never reached ballistic coast");
        assert!(
            flight.is_complete(),
            "both stages should impact before timeout"
        );
        assert_eq!(flight.collision_count, 2);
        assert!(flight.maximum_altitude.is_finite() && flight.maximum_altitude > 0.0);
        assert!(flight.maximum_speed.is_finite() && flight.maximum_speed > 0.0);
        Ok(())
    }
}
