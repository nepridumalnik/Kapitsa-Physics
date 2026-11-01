use std::error::Error;
use std::io;

use kapitsa_core::{
    collider::Collider,
    collision::{CollisionEventKind, CollisionPair},
    gravity::CentralGravity,
    mass::MassProperties,
    rigid_body::{BodyState, RigidBody},
    shape::{Cuboid, Shape, Sphere},
    transform::Transform,
    world::{BodyId, ColliderId, ForceSourceId, PhysicsWorld},
};
use kapitsa_math::{Mat3, Quat, Vec3};

pub const TICK_RATE: u32 = 120;
pub const STEP_SECONDS: f64 = 1.0 / TICK_RATE as f64;
pub const PLANET_RADIUS: f64 = 6_371_000.0;
pub const PLANET_MU: f64 = 3.986_004_418e14;
pub const START_ALTITUDE: f64 = 5.0;
pub const STAGE_ONE_MASS: f64 = 12_500.0;
pub const STAGE_TWO_MASS: f64 = 8_000.0;
pub const STAGE_ONE_THRUST: f64 = 300_000.0;
pub const STAGE_TWO_THRUST: f64 = 140_000.0;
pub const STAGE_ONE_BURN_END: f64 = 2.0;
pub const STAGE_TWO_BURN_END: f64 = 5.0;
pub const SEPARATION_IMPULSE: f64 = 16_000.0;
pub const STAGE_HALF_HEIGHT: f64 = 2.0;
pub const MAX_FLIGHT_SECONDS: f64 = 35.0;

pub type MissionResult<T> = Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightAction {
    StageOnePowered,
    Separate,
    StageTwoPowered,
    Coast,
}

pub fn action_at(simulation_seconds: f64, separated: bool) -> FlightAction {
    if simulation_seconds < STAGE_ONE_BURN_END {
        FlightAction::StageOnePowered
    } else if !separated {
        FlightAction::Separate
    } else if simulation_seconds < STAGE_TWO_BURN_END {
        FlightAction::StageTwoPowered
    } else {
        FlightAction::Coast
    }
}

pub fn gravity() -> MissionResult<CentralGravity> {
    Ok(CentralGravity::new(Vec3::ZERO, PLANET_MU)?)
}

pub fn planet_body() -> RigidBody {
    RigidBody::new_static(Transform::identity())
}

pub fn planet_collider() -> MissionResult<Collider> {
    Ok(Collider::new(
        Shape::Sphere(Sphere::new(PLANET_RADIUS)?),
        Transform::identity(),
    ))
}

pub fn stack_body() -> MissionResult<RigidBody> {
    let transform = Transform::new(Vec3::Y * (PLANET_RADIUS + START_ALTITUDE), Quat::IDENTITY)?;
    dynamic_body(transform, STAGE_ONE_MASS + STAGE_TWO_MASS, Vec3::ZERO)
}

pub fn dynamic_body(transform: Transform, mass: f64, velocity: Vec3) -> MissionResult<RigidBody> {
    let properties = mass_properties(mass)?;
    let mut body = RigidBody::new_dynamic(transform, properties)?;
    body.set_linear_velocity(velocity)?;
    Ok(body)
}

pub fn mass_properties(mass: f64) -> MissionResult<MassProperties> {
    Ok(MassProperties::new(
        mass,
        Vec3::ZERO,
        Mat3::from_diagonal(Vec3::new(1.0e5, 1.0e5, 1.0e5)),
    )?)
}

pub fn stack_stage_collider(local_y: f64) -> MissionResult<Collider> {
    Ok(Collider::new(
        Shape::Cuboid(Cuboid::new(1.2, STAGE_HALF_HEIGHT, 1.2)?),
        Transform::new(Vec3::Y * local_y, Quat::IDENTITY)?,
    ))
}

pub fn detached_stage_collider() -> MissionResult<Collider> {
    stack_stage_collider(0.0)
}

pub fn separation_states(state: BodyState) -> MissionResult<(Transform, Vec3, Transform, Vec3)> {
    let rotation = state.transform().rotation();
    let offset = rotation
        .rotate_vector(Vec3::Y * STAGE_HALF_HEIGHT)
        .ok_or_else(|| io::Error::other("invalid rocket rotation"))?;
    let lower_position = state
        .transform()
        .translation()
        .checked_sub(offset)
        .ok_or_else(|| io::Error::other("stage position overflow"))?;
    let upper_position = state
        .transform()
        .translation()
        .checked_add(offset)
        .ok_or_else(|| io::Error::other("stage position overflow"))?;
    let impulse = rotation
        .rotate_vector(Vec3::Y * SEPARATION_IMPULSE)
        .ok_or_else(|| io::Error::other("invalid separation impulse"))?;
    let lower_velocity = state
        .linear_velocity()
        .checked_sub(impulse / STAGE_ONE_MASS)
        .ok_or_else(|| io::Error::other("lower-stage velocity overflow"))?;
    let upper_velocity = state
        .linear_velocity()
        .checked_add(impulse / STAGE_TWO_MASS)
        .ok_or_else(|| io::Error::other("upper-stage velocity overflow"))?;
    Ok((
        Transform::new(lower_position, rotation)?,
        lower_velocity,
        Transform::new(upper_position, rotation)?,
        upper_velocity,
    ))
}

pub fn stage_one_engine_point() -> Vec3 {
    Vec3::Y * (-2.0 * STAGE_HALF_HEIGHT)
}

pub fn stage_two_engine_point_before_separation() -> Vec3 {
    Vec3::ZERO
}

pub fn stage_two_engine_point_after_separation() -> Vec3 {
    Vec3::Y * -STAGE_HALF_HEIGHT
}

pub struct HeadlessFlight {
    pub world: PhysicsWorld,
    planet_collider: ColliderId,
    stack_body: BodyId,
    stage_one_body: Option<BodyId>,
    stage_two_body: Option<BodyId>,
    stage_one_collider: Option<ColliderId>,
    stage_two_collider: Option<ColliderId>,
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
    pub position: Vec3,
    pub velocity: Vec3,
    pub specific_energy: f64,
}

impl HeadlessFlight {
    pub fn new() -> MissionResult<Self> {
        let mut world = PhysicsWorld::new();
        world.set_central_gravity(Some(gravity()?));
        let planet = world.add_body(planet_body())?;
        let planet_collider = world.add_collider(planet, planet_collider()?)?;
        let stack_body = world.add_body(stack_body()?)?;
        let stage_one_collider = world.add_collider(stack_body, stack_stage_collider(-2.0)?)?;
        let stage_two_collider = world.add_collider(stack_body, stack_stage_collider(2.0)?)?;
        let stage_one_engine = world.add_force_source(
            stack_body,
            stage_one_engine_point(),
            Vec3::Y * STAGE_ONE_THRUST,
        )?;
        let stage_two_engine = world.add_force_source(
            stack_body,
            stage_two_engine_point_before_separation(),
            Vec3::ZERO,
        )?;
        Ok(Self {
            world,
            planet_collider,
            stack_body,
            stage_one_body: None,
            stage_two_body: Some(stack_body),
            stage_one_collider: Some(stage_one_collider),
            stage_two_collider: Some(stage_two_collider),
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
            FlightAction::StageOnePowered => {}
            FlightAction::Separate => self.separate()?,
            FlightAction::StageTwoPowered => {}
            FlightAction::Coast if !self.stage_two_cutoff => {
                self.world
                    .set_force_source_enabled(self.stage_two_engine, false)?;
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

    fn separate(&mut self) -> MissionResult<()> {
        let stack = *self
            .world
            .body(self.stack_body)
            .ok_or_else(|| io::Error::other("rocket stack body disappeared"))?
            .state();
        let (lower_transform, lower_velocity, upper_transform, upper_velocity) =
            separation_states(stack)?;
        self.world
            .set_force_source_enabled(self.stage_one_engine, false)?;
        self.world.remove_force_source(self.stage_one_engine)?;
        if let Some(collider) = self.stage_one_collider.take() {
            let _ = self.world.remove_collider(collider);
        }
        if let Some(collider) = self.stage_two_collider.take() {
            let _ = self.world.remove_collider(collider);
        }

        let lower = self.world.add_body(dynamic_body(
            lower_transform,
            STAGE_ONE_MASS,
            lower_velocity,
        )?)?;
        let upper = self.stack_body;
        let upper_body = self
            .world
            .body_mut(upper)
            .ok_or_else(|| io::Error::other("upper-stage body disappeared"))?;
        upper_body.set_mass(mass_properties(STAGE_TWO_MASS)?)?;
        upper_body.set_transform(upper_transform)?;
        upper_body.set_linear_velocity(upper_velocity)?;
        let lower_collider = self.world.add_collider(lower, detached_stage_collider()?)?;
        let upper_collider = self.world.add_collider(upper, detached_stage_collider()?)?;
        self.world.set_force_source(
            self.stage_two_engine,
            stage_two_engine_point_after_separation(),
            Vec3::Y * STAGE_TWO_THRUST,
        )?;
        self.stage_one_body = Some(lower);
        self.stage_two_body = Some(upper);
        self.stage_one_collider = Some(lower_collider);
        self.stage_two_collider = Some(upper_collider);
        self.separated = true;
        Ok(())
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
                    .map_or(PLANET_RADIUS, Vec3::length);
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
