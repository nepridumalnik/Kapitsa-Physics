use std::io;

use kapitsa_core::{
    collision::{CollisionEventKind, CollisionPair},
    world::{BodyId, ColliderId, ForceSourceId, JointId, PhysicsWorld},
};
use kapitsa_math::Vec3;

#[path = "mission_spec.rs"]
mod spec;
pub use spec::*;

pub struct HeadlessFlight {
    pub world: PhysicsWorld,
    planet_collider: ColliderId,
    stage_one_body: Option<BodyId>,
    stage_two_body: Option<BodyId>,
    stage_one_collider: Option<ColliderId>,
    stage_two_collider: Option<ColliderId>,
    stage_joint: Option<JointId>,
    stage_one_engine: ForceSourceId,
    stage_two_engine: ForceSourceId,
    separated: bool,
    stage_two_cutoff: bool,
    pub stage_one_impact: bool,
    pub stage_two_impact: bool,
    pub collision_count: u64,
    pub maximum_altitude: f64,
    pub maximum_speed: f64,
    pub ballistic_energy_start: Option<f64>,
    pub ballistic_reference: Option<FlightReference>,
    pub step: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightReference {
    pub simulation_seconds: f64,
    pub position: kapitsa_math::Vec3,
    pub velocity: kapitsa_math::Vec3,
    pub specific_energy: f64,
}

impl HeadlessFlight {
    pub fn new() -> MissionResult<Self> {
        let mut world = PhysicsWorld::new();
        world.set_central_gravity(Some(gravity()?));
        let planet = world.add_body(planet_body())?;
        let planet_collider = world.add_collider(planet, planet_collider()?)?;
        let stage_one = world.add_body(spec::stage_body(Stage::One)?)?;
        let stage_two = world.add_body(spec::stage_body(Stage::Two)?)?;
        let stage_one_collider = world.add_collider(stage_one, spec::stage_collider()?)?;
        let stage_two_collider = world.add_collider(stage_two, spec::stage_collider()?)?;
        let stage_joint = world.add_joint(stage_one, stage_two, fixed_joint())?;
        let stage_one_engine =
            world.add_force_source(stage_one, Vec3::Y * -STAGE_HALF_HEIGHT, Vec3::ZERO)?;
        let stage_two_engine =
            world.add_force_source(stage_two, Vec3::Y * -STAGE_HALF_HEIGHT, Vec3::ZERO)?;
        Ok(Self {
            world,
            planet_collider,
            stage_one_body: Some(stage_one),
            stage_two_body: Some(stage_two),
            stage_one_collider: Some(stage_one_collider),
            stage_two_collider: Some(stage_two_collider),
            stage_joint: Some(stage_joint),
            stage_one_engine,
            stage_two_engine,
            separated: false,
            stage_two_cutoff: false,
            stage_one_impact: false,
            stage_two_impact: false,
            collision_count: 0,
            maximum_altitude: START_ALTITUDE,
            maximum_speed: 0.0,
            ballistic_energy_start: None,
            ballistic_reference: None,
            step: 0,
        })
    }

    pub fn is_complete(&self) -> bool {
        self.stage_one_impact && self.stage_two_impact
    }

    pub fn step_once(&mut self) -> MissionResult<FlightAction> {
        let elapsed = self.step as f64 * STEP_SECONDS;
        let action = action_at(elapsed, self.separated);
        match action {
            FlightAction::StageOnePowered => {
                self.set_stage_mass(Stage::One, elapsed)?;
                self.world.set_force_source(
                    self.stage_one_engine,
                    Vec3::Y * -STAGE_HALF_HEIGHT,
                    Vec3::Y * STAGE_ONE_THRUST,
                )?;
            }
            FlightAction::Separate => self.separate()?,
            FlightAction::StageTwoPowered => {
                self.set_stage_mass(Stage::Two, elapsed - STAGE_ONE_BURN_END)?;
            }
            FlightAction::Coast if !self.stage_two_cutoff => {
                self.world
                    .set_force_source_enabled(self.stage_two_engine, false)?;
                self.world.remove_force_source(self.stage_two_engine)?;
                self.stage_two_cutoff = true;
            }
            FlightAction::Coast => {}
        }
        self.world.step(STEP_SECONDS)?;
        self.record_collisions();
        self.record_extrema();
        self.record_ballistic_reference(action);
        self.step += 1;
        Ok(action)
    }

    fn set_stage_mass(&mut self, stage: Stage, elapsed: f64) -> MissionResult<()> {
        let body = match stage {
            Stage::One => self.stage_one_body,
            Stage::Two => self.stage_two_body,
        }
        .ok_or_else(|| io::Error::other("rocket stage disappeared before fuel cutoff"))?;
        let properties = mass_properties(stage, stage_mass(stage, elapsed))?;
        self.world
            .body_mut(body)
            .ok_or_else(|| io::Error::other("rocket stage body disappeared"))?
            .set_mass(properties)?;
        Ok(())
    }

    fn separate(&mut self) -> MissionResult<()> {
        let lower_id = self
            .stage_one_body
            .ok_or_else(|| io::Error::other("lower stage disappeared"))?;
        let upper_id = self
            .stage_two_body
            .ok_or_else(|| io::Error::other("upper stage disappeared"))?;
        let lower = *self
            .world
            .body(lower_id)
            .ok_or_else(|| io::Error::other("lower stage body disappeared"))?
            .state();
        let upper = *self
            .world
            .body(upper_id)
            .ok_or_else(|| io::Error::other("upper stage body disappeared"))?
            .state();
        let (lower_velocity, upper_velocity) = separation_velocities(lower, upper)?;
        if let Some(joint) = self.stage_joint.take() {
            let _ = self.world.remove_joint(joint);
        }
        self.world
            .set_force_source_enabled(self.stage_one_engine, false)?;
        self.world.remove_force_source(self.stage_one_engine)?;
        self.world
            .body_mut(
                self.stage_one_body
                    .ok_or_else(|| io::Error::other("lower stage disappeared"))?,
            )
            .ok_or_else(|| io::Error::other("lower stage body disappeared"))?
            .set_linear_velocity(lower_velocity)?;
        self.world
            .body_mut(
                self.stage_two_body
                    .ok_or_else(|| io::Error::other("upper stage disappeared"))?,
            )
            .ok_or_else(|| io::Error::other("upper stage body disappeared"))?
            .set_linear_velocity(upper_velocity)?;
        self.world.set_force_source(
            self.stage_two_engine,
            Vec3::Y * -STAGE_HALF_HEIGHT,
            Vec3::Y * STAGE_TWO_THRUST,
        )?;
        self.separated = true;
        Ok(())
    }

    fn record_ballistic_reference(&mut self, action: FlightAction) {
        if action != FlightAction::Coast {
            return;
        }
        let Some(body_id) = self.stage_two_body else {
            return;
        };
        let Some(body) = self.world.body(body_id) else {
            return;
        };
        let seconds = (self.step + 1) as f64 * STEP_SECONDS;
        let position = body.state().transform().translation();
        let velocity = body.state().linear_velocity();
        let radius = position.length();
        let specific_energy = 0.5 * velocity.length_squared() - PLANET_MU / radius;
        if self.ballistic_energy_start.is_none() {
            self.ballistic_energy_start = Some(specific_energy);
        }
        if seconds >= 8.0 && self.ballistic_reference.is_none() {
            self.ballistic_reference = Some(FlightReference {
                simulation_seconds: seconds,
                position,
                velocity,
                specific_energy,
            });
        }
    }

    fn record_collisions(&mut self) {
        let events = self.world.collision_events();
        let stage_one_hit = self
            .stage_one_collider
            .is_some_and(|collider| has_planet_impact(events, self.planet_collider, collider));
        let stage_two_hit = self
            .stage_two_collider
            .is_some_and(|collider| has_planet_impact(events, self.planet_collider, collider));
        if stage_one_hit && let Some(body) = self.stage_one_body.take() {
            let _ = self.world.remove_body(body);
            self.stage_one_collider = None;
            self.stage_one_impact = true;
            self.collision_count += 1;
        }
        if stage_two_hit && let Some(body) = self.stage_two_body.take() {
            let _ = self.world.remove_body(body);
            self.stage_two_collider = None;
            self.stage_two_impact = true;
            self.collision_count += 1;
        }
    }

    fn record_extrema(&mut self) {
        for id in [self.stage_one_body, self.stage_two_body]
            .into_iter()
            .flatten()
        {
            if let Some(body) = self.world.body(id) {
                let radius = body
                    .world_center_of_mass()
                    .ok()
                    .flatten()
                    .map_or(PLANET_RADIUS, kapitsa_math::Vec3::length);
                self.maximum_altitude = self.maximum_altitude.max(radius - PLANET_RADIUS);
                self.maximum_speed = self
                    .maximum_speed
                    .max(body.state().linear_velocity().length());
            }
        }
    }
}

fn has_planet_impact(
    events: &[kapitsa_core::collision::CollisionEvent],
    planet: ColliderId,
    stage: ColliderId,
) -> bool {
    events.iter().any(|event| {
        matches!(
            event.kind,
            CollisionEventKind::Begin | CollisionEventKind::Impact
        ) && pair_contains(event.pair, planet)
            && pair_contains(event.pair, stage)
    })
}

fn pair_contains(pair: CollisionPair, collider: ColliderId) -> bool {
    pair.a == collider || pair.b == collider
}
