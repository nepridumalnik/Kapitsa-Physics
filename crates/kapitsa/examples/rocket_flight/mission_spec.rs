use std::{error::Error, io};

use kapitsa::{
    BodyState, CentralGravity, Collider, MassProperties, RigidBody, Shape, Transform,
    joint::{FixedJoint, Joint, JointCommon, JointFrame},
    math::{Mat3, Quat, Vec3},
    shape::{Cuboid, Sphere},
};

pub const TICK_RATE: u32 = 120;
pub const STEP_SECONDS: f64 = 1.0 / TICK_RATE as f64;
pub const PLANET_RADIUS: f64 = 6_371_000.0;
pub const PLANET_MU: f64 = 3.986_004_418e14;
pub const START_ALTITUDE: f64 = 5.0;
pub const STAGE_ONE_MASS: f64 = 12_500.0;
pub const STAGE_TWO_MASS: f64 = 8_000.0;
pub const STAGE_ONE_THRUST: f64 = 300_000.0;
pub const STAGE_TWO_THRUST: f64 = 140_000.0;
pub const STAGE_ONE_SPECIFIC_IMPULSE: f64 = 300.0;
pub const STAGE_TWO_SPECIFIC_IMPULSE: f64 = 320.0;
pub const STAGE_ONE_BURN_END: f64 = 2.0;
pub const STAGE_TWO_BURN_END: f64 = 5.0;
pub const SEPARATION_IMPULSE: f64 = 16_000.0;
pub const STAGE_HALF_HEIGHT: f64 = 2.0;
pub const MAX_FLIGHT_SECONDS: f64 = 35.0;
const STANDARD_GRAVITY: f64 = 9.806_65;

pub type MissionResult<T> = Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    One,
    Two,
}

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

pub fn stage_position(stage: Stage) -> MissionResult<Transform> {
    let offset = match stage {
        Stage::One => STAGE_HALF_HEIGHT + 1.0,
        Stage::Two => STAGE_HALF_HEIGHT * 3.0 + 1.0,
    };
    Ok(Transform::new(
        Vec3::Y * (PLANET_RADIUS + offset),
        Quat::IDENTITY,
    )?)
}

pub fn stage_body(stage: Stage) -> MissionResult<RigidBody> {
    let mass = initial_mass(stage);
    Ok(RigidBody::new_dynamic(
        stage_position(stage)?,
        mass_properties(stage, mass)?,
    )?)
}

pub fn stage_collider() -> MissionResult<Collider> {
    Ok(Collider::new(
        Shape::Cuboid(Cuboid::new(1.2, STAGE_HALF_HEIGHT, 1.2)?),
        Transform::identity(),
    ))
}

pub fn fixed_joint() -> Joint {
    let common = JointCommon::new(
        JointFrame::new(Vec3::Y * STAGE_HALF_HEIGHT, Quat::IDENTITY),
        JointFrame::new(Vec3::Y * -STAGE_HALF_HEIGHT, Quat::IDENTITY),
    );
    Joint::Fixed(FixedJoint::new(common))
}

pub fn initial_mass(stage: Stage) -> f64 {
    match stage {
        Stage::One => STAGE_ONE_MASS,
        Stage::Two => STAGE_TWO_MASS,
    }
}

pub fn thrust(stage: Stage) -> f64 {
    match stage {
        Stage::One => STAGE_ONE_THRUST,
        Stage::Two => STAGE_TWO_THRUST,
    }
}

pub fn specific_impulse(stage: Stage) -> f64 {
    match stage {
        Stage::One => STAGE_ONE_SPECIFIC_IMPULSE,
        Stage::Two => STAGE_TWO_SPECIFIC_IMPULSE,
    }
}

#[allow(dead_code)]
pub fn engine_point() -> Vec3 {
    Vec3::Y * -STAGE_HALF_HEIGHT
}

pub fn burn_duration(stage: Stage) -> f64 {
    match stage {
        Stage::One => STAGE_ONE_BURN_END,
        Stage::Two => STAGE_TWO_BURN_END - STAGE_ONE_BURN_END,
    }
}

pub fn stage_mass(stage: Stage, elapsed: f64) -> f64 {
    let initial = initial_mass(stage);
    let thrust = thrust(stage);
    let duration = burn_duration(stage);
    let mass_flow = thrust / (specific_impulse(stage) * STANDARD_GRAVITY);
    let propellant = mass_flow * duration;
    (initial - mass_flow * elapsed.clamp(0.0, duration)).max(initial - propellant)
}

pub fn mass_properties(stage: Stage, mass: f64) -> MissionResult<MassProperties> {
    let inertia = 1.0e5 * mass / initial_mass(stage);
    Ok(MassProperties::new(
        mass,
        Vec3::ZERO,
        Mat3::from_diagonal(Vec3::splat(inertia)),
    )?)
}

pub fn separation_velocities(lower: BodyState, upper: BodyState) -> MissionResult<(Vec3, Vec3)> {
    let direction = upper
        .transform()
        .rotation()
        .rotate_vector(Vec3::Y)
        .ok_or_else(|| io::Error::other("invalid rocket rotation"))?;
    let lower_velocity = lower.linear_velocity()
        - direction * (SEPARATION_IMPULSE / stage_mass(Stage::One, burn_duration(Stage::One)));
    let upper_velocity =
        upper.linear_velocity() + direction * (SEPARATION_IMPULSE / initial_mass(Stage::Two));
    Ok((lower_velocity, upper_velocity))
}
