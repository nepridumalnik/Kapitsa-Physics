use std::time::Duration;

use kapitsa_core::{
    collision::CollisionEventKind,
    joint::{Joint, JointDrive},
    rigid_body::{BodyType, CcdMode, RigidBody},
    world::{ForceSource, PhysicsWorld},
};
use kapitsa_math::Vec3;

use super::config::{AdaptiveTimeStepConfig, SimulationConfig};

#[derive(Debug)]
pub(super) enum StepChoice {
    Wait,
    Limited,
    Step(Duration),
}

pub(super) fn choose_step(
    world: &PhysicsWorld,
    config: &SimulationConfig,
    target_step: Duration,
    accumulated: Duration,
    base_step: Duration,
) -> StepChoice {
    let SimulationConfig {
        time_step_policy: super::config::TimeStepPolicy::Adaptive(adaptive),
        ..
    } = config
    else {
        return StepChoice::Wait;
    };

    if accumulated < adaptive.minimum_step {
        return StepChoice::Wait;
    }

    let mut limit = target_step
        .max(adaptive.minimum_step)
        .min(accumulated)
        .min(adaptive.maximum_step)
        .as_secs_f64();
    let characteristic_length = estimate_motion_limit(world, *adaptive, &mut limit);
    estimate_acceleration_limit(world, *adaptive, characteristic_length, &mut limit);
    estimate_contact_limit(world, base_step, &mut limit);
    estimate_joint_limit(world, *adaptive, base_step, &mut limit);

    if limit < adaptive.minimum_step.as_secs_f64() {
        return if accumulated >= adaptive.minimum_step {
            StepChoice::Limited
        } else {
            StepChoice::Wait
        };
    }

    let Ok(step) = Duration::try_from_secs_f64(limit) else {
        return StepChoice::Limited;
    };
    if step < adaptive.minimum_step {
        StepChoice::Wait
    } else {
        StepChoice::Step(step)
    }
}

fn estimate_motion_limit(
    world: &PhysicsWorld,
    adaptive: AdaptiveTimeStepConfig,
    limit: &mut f64,
) -> f64 {
    let error_fraction = adaptive.relative_error_tolerance.sqrt();
    let mut characteristic_length = f64::INFINITY;
    for (_, body_id, collider) in world.colliders() {
        let Some(body) = world.body(body_id) else {
            cap(limit, 0.0);
            continue;
        };
        if body.physics().body_type() == BodyType::Static {
            continue;
        }
        let Ok(Some(bounds)) = collider.shape().local_bounds() else {
            continue;
        };
        let radius = vector_length(bounds.min().abs().max(bounds.max().abs()));
        if !radius.is_finite() {
            cap(limit, 0.0);
            continue;
        }
        if radius <= f64::MIN_POSITIVE {
            continue;
        }
        characteristic_length = characteristic_length.min(radius);
        let center_offset = body
            .physics()
            .mass()
            .map_or(0.0, |mass| vector_length(mass.center_of_mass()));
        let orbit_radius =
            radius + vector_length(collider.local_transform().translation()) + center_offset;
        let state = body.state();
        let surface_speed = vector_length(state.linear_velocity())
            + vector_length(state.angular_velocity()) * orbit_radius;
        if surface_speed > 0.0 {
            let travel_fraction = if body.ccd_mode() == CcdMode::Continuous {
                0.5
            } else {
                error_fraction
            };
            cap(limit, radius * travel_fraction / surface_speed);
        }
    }
    if characteristic_length.is_finite() {
        characteristic_length
    } else {
        1.0
    }
}

fn estimate_acceleration_limit(
    world: &PhysicsWorld,
    adaptive: AdaptiveTimeStepConfig,
    characteristic_length: f64,
    limit: &mut f64,
) {
    let error_fraction = adaptive.relative_error_tolerance;
    let mut max_acceleration: f64 = 0.0;
    let mut max_angular_acceleration: f64 = 0.0;
    let mut source_linear_acceleration = 0.0;
    let mut source_angular_acceleration = 0.0;
    for (_, source) in world.force_sources() {
        if !source.is_enabled() {
            continue;
        }
        accumulate_source_acceleration(
            source,
            world,
            &mut source_linear_acceleration,
            &mut source_angular_acceleration,
            limit,
        );
    }

    let gravity = world.central_gravity();
    for (_, body) in world.bodies() {
        if body.physics().body_type() != BodyType::Dynamic {
            continue;
        }
        let Some(mass) = body.physics().mass() else {
            continue;
        };
        let force_acceleration = vector_length(body.forces().force()) / mass.mass();
        let gravity_acceleration = gravity
            .and_then(|gravity| {
                body.world_center_of_mass()
                    .ok()
                    .flatten()
                    .and_then(|position| gravity.acceleration_at(position).ok())
            })
            .map_or(0.0, vector_length);
        max_acceleration = max_acceleration.max(force_acceleration + gravity_acceleration);

        let local_torque = body
            .state()
            .transform()
            .rotation()
            .conjugate()
            .rotate_vector(body.forces().torque());
        let angular_acceleration = local_torque
            .and_then(|torque| mass.inertia().solve_spd(torque).ok())
            .map_or(0.0, vector_length);
        max_angular_acceleration = max_angular_acceleration.max(angular_acceleration);

        if let (Some(gravity), Ok(Some(center))) = (gravity, body.world_center_of_mass()) {
            let offset = center - gravity.center();
            let radius = vector_length(offset);
            if radius > 0.0 {
                let acceleration = gravity
                    .acceleration_at(center)
                    .map_or(f64::INFINITY, vector_length);
                cap(
                    limit,
                    adaptive.relative_error_tolerance * (radius / acceleration).sqrt(),
                );
            } else if gravity.gravitational_parameter() > 0.0 {
                cap(limit, 0.0);
            }
        }
    }

    max_acceleration += source_linear_acceleration;
    max_angular_acceleration += source_angular_acceleration;
    let allowed_displacement = characteristic_length * error_fraction;
    if max_acceleration > 0.0 {
        cap(
            limit,
            (2.0 * allowed_displacement / max_acceleration).sqrt(),
        );
    }
    if max_angular_acceleration > 0.0 {
        cap(
            limit,
            (2.0 * error_fraction / max_angular_acceleration).sqrt(),
        );
    }
}

fn accumulate_source_acceleration(
    source: &ForceSource,
    world: &PhysicsWorld,
    linear: &mut f64,
    angular: &mut f64,
    limit: &mut f64,
) {
    let Some(body) = world.body(source.body()) else {
        cap(limit, 0.0);
        return;
    };
    let Some(mass) = body.physics().mass() else {
        cap(limit, 0.0);
        return;
    };
    let force = source.local_force();
    *linear += vector_length(force) / mass.mass();
    let torque = (source.local_point() - mass.center_of_mass()).cross(force);
    match mass.inertia().solve_spd(torque) {
        Ok(acceleration) => *angular += vector_length(acceleration),
        Err(_) => cap(limit, 0.0),
    }
}

fn estimate_contact_limit(world: &PhysicsWorld, base_step: Duration, limit: &mut f64) {
    if world
        .collision_events()
        .iter()
        .any(|event| !event.sensor && event.kind != CollisionEventKind::End)
    {
        cap(limit, base_step.as_secs_f64());
    }
}

fn estimate_joint_limit(
    world: &PhysicsWorld,
    adaptive: AdaptiveTimeStepConfig,
    base_step: Duration,
    limit: &mut f64,
) {
    let tolerance = adaptive.relative_error_tolerance;
    for (_, body_a_id, body_b_id, joint) in world.joints() {
        let common = joint.common();
        if !common.enabled {
            continue;
        }
        let (Some(body_a), Some(body_b)) = (world.body(body_a_id), world.body(body_b_id)) else {
            cap(limit, 0.0);
            continue;
        };
        let (linear_stiffness, angular_stiffness, linear_damping, angular_damping) =
            joint_coefficients(joint);
        let effective_mass = effective_linear_mass(body_a, body_b);
        let effective_inertia = effective_angular_inertia(body_a, body_b);
        if common.compliance > 0.0 {
            let stiffness = 1.0 / common.compliance;
            cap_stiffness(limit, tolerance, effective_mass, stiffness);
            cap_stiffness(limit, tolerance, effective_inertia, stiffness);
        }
        cap_stiffness(limit, tolerance, effective_mass, linear_stiffness);
        cap_stiffness(limit, tolerance, effective_inertia, angular_stiffness);
        cap_damping(
            limit,
            tolerance,
            effective_mass,
            linear_damping.max(common.damping),
        );
        cap_damping(
            limit,
            tolerance,
            effective_inertia,
            angular_damping.max(common.damping),
        );

        if common.break_force.is_some() || common.break_torque.is_some() {
            cap(limit, base_step.as_secs_f64());
        }
        let frame_a = body_a
            .state()
            .transform()
            .transform_point(common.frame_a.local_anchor);
        let frame_b = body_b
            .state()
            .transform()
            .transform_point(common.frame_b.local_anchor);
        let point_velocity_a = point_velocity(body_a, frame_a);
        let point_velocity_b = point_velocity(body_b, frame_b);
        let relative_speed = vector_length(point_velocity_b - point_velocity_a);
        let anchor_scale = vector_length(frame_a - frame_b)
            .max(vector_length(common.frame_a.local_anchor))
            .max(vector_length(common.frame_b.local_anchor))
            .max(1.0e-3);
        if relative_speed > 0.0 {
            cap(limit, tolerance * anchor_scale / relative_speed);
        }
    }
}

fn joint_coefficients(joint: &Joint) -> (f64, f64, f64, f64) {
    let mut linear_stiffness: f64 = 0.0;
    let mut angular_stiffness: f64 = 0.0;
    let mut linear_damping: f64 = 0.0;
    let mut angular_damping: f64 = 0.0;
    let mut add_linear = |drive: JointDrive| {
        if drive.enabled {
            linear_stiffness = linear_stiffness.max(drive.stiffness);
            linear_damping = linear_damping.max(drive.damping);
        }
    };
    let mut add_angular = |drive: JointDrive| {
        if drive.enabled {
            angular_stiffness = angular_stiffness.max(drive.stiffness);
            angular_damping = angular_damping.max(drive.damping);
        }
    };
    match joint {
        Joint::Hinge(joint) => add_angular(joint.drive),
        Joint::Slider(joint) => add_linear(joint.drive),
        Joint::Cylindrical(joint) => {
            add_linear(joint.translation_drive);
            add_angular(joint.angular_drive);
        }
        Joint::Planar(joint) => {
            for drive in joint.translation_drives {
                add_linear(drive);
            }
            add_angular(joint.rotation_drive);
        }
        Joint::Spring(joint) => {
            linear_stiffness = linear_stiffness.max(joint.stiffness);
            linear_damping = linear_damping.max(joint.spring_damping);
        }
        Joint::D6(joint) => {
            for drive in joint.linear_drives {
                add_linear(drive);
            }
            for drive in joint.angular_drives {
                add_angular(drive);
            }
        }
        Joint::Fixed(_) | Joint::Ball(_) | Joint::Distance(_) | Joint::ConeTwist(_) => {}
    }
    (
        linear_stiffness,
        angular_stiffness,
        linear_damping,
        angular_damping,
    )
}

fn effective_linear_mass(body_a: &RigidBody, body_b: &RigidBody) -> f64 {
    let mass_a = dynamic_mass(body_a);
    let mass_b = dynamic_mass(body_b);
    match (mass_a, mass_b) {
        (Some(a), Some(b)) => 1.0 / (1.0 / a + 1.0 / b),
        (Some(mass), None) | (None, Some(mass)) => mass,
        (None, None) => 0.0,
    }
}

fn effective_angular_inertia(body_a: &RigidBody, body_b: &RigidBody) -> f64 {
    let inertia_a = dynamic_minimum_inertia(body_a);
    let inertia_b = dynamic_minimum_inertia(body_b);
    match (inertia_a, inertia_b) {
        (Some(a), Some(b)) => 1.0 / (1.0 / a + 1.0 / b),
        (Some(inertia), None) | (None, Some(inertia)) => inertia,
        (None, None) => 0.0,
    }
}

fn dynamic_mass(body: &RigidBody) -> Option<f64> {
    (body.physics().body_type() == BodyType::Dynamic)
        .then(|| body.physics().mass().map(|mass| mass.mass()))
        .flatten()
}

fn dynamic_minimum_inertia(body: &RigidBody) -> Option<f64> {
    if body.physics().body_type() != BodyType::Dynamic {
        return None;
    }
    let inertia = body.physics().mass()?.inertia();
    let mut inverse_frobenius_squared = 0.0;
    for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        let column = inertia.solve_spd(axis).ok()?;
        inverse_frobenius_squared += column.length_squared();
    }
    (inverse_frobenius_squared.is_finite() && inverse_frobenius_squared > 0.0)
        .then(|| 1.0 / inverse_frobenius_squared.sqrt())
}

fn cap_stiffness(limit: &mut f64, tolerance: f64, effective_mass: f64, stiffness: f64) {
    if stiffness <= 0.0 || effective_mass <= 0.0 {
        return;
    }
    cap(limit, (tolerance * effective_mass / stiffness).sqrt());
}

fn cap_damping(limit: &mut f64, tolerance: f64, effective_mass: f64, damping: f64) {
    if damping <= 0.0 || effective_mass <= 0.0 {
        return;
    }
    cap(limit, tolerance * effective_mass / damping);
}

fn point_velocity(body: &RigidBody, point: Vec3) -> Vec3 {
    let center = body
        .world_center_of_mass()
        .ok()
        .flatten()
        .unwrap_or_else(|| body.state().transform().translation());
    body.state().linear_velocity() + body.state().angular_velocity().cross(point - center)
}

fn vector_length(vector: Vec3) -> f64 {
    let length = vector.length();
    if length.is_finite() {
        length
    } else {
        f64::INFINITY
    }
}

fn cap(limit: &mut f64, candidate: f64) {
    if candidate.is_nan() || candidate <= 0.0 {
        *limit = 0.0;
    } else if candidate.is_finite() {
        *limit = limit.min(candidate);
    }
}

#[cfg(test)]
mod tests {
    use std::{
        hint::black_box,
        time::{Duration, Instant},
    };

    use kapitsa_core::{
        collider::Collider,
        collision::CollisionEventKind,
        gravity::CentralGravity,
        joint::{FixedJoint, Joint, JointCommon, JointFrame, SpringJoint},
        mass::MassProperties,
        rigid_body::RigidBody,
        shape::{Shape, Sphere},
        transform::Transform,
        world::{BodyId, PhysicsWorld},
    };
    use kapitsa_math::{Mat3, Quat, Vec3};

    use super::{StepChoice, choose_step};
    use crate::runtime::{AdaptiveTimeStepConfig, SimulationConfig, TimeStepPolicy};

    fn adaptive_config() -> SimulationConfig {
        SimulationConfig {
            tick_rate: 120,
            time_step_policy: TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default()),
            ..SimulationConfig::default()
        }
    }

    fn dynamic_body(position: Vec3, velocity: Vec3) -> RigidBody {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
        let mut body = RigidBody::new_dynamic(
            Transform::new(position, Quat::IDENTITY).expect("transform"),
            mass,
        )
        .expect("body");
        body.set_linear_velocity(velocity).expect("velocity");
        body
    }

    fn choice(world: &PhysicsWorld, warp: f64, accumulated: Duration) -> StepChoice {
        let config = adaptive_config();
        let base = config.step_budget();
        let target = Duration::try_from_secs_f64(base.as_secs_f64() * warp).expect("target");
        choose_step(world, &config, target, accumulated, base)
    }

    fn orbit_world() -> (PhysicsWorld, BodyId) {
        let mut world = PhysicsWorld::new();
        world.set_central_gravity(Some(
            CentralGravity::new(Vec3::ZERO, 10_000.0).expect("gravity"),
        ));
        let body = world
            .add_body(dynamic_body(
                Vec3::X * 10.0,
                Vec3::Z * (10_000.0_f64 / 10.0).sqrt(),
            ))
            .expect("body");
        (world, body)
    }

    fn spring_world() -> (PhysicsWorld, BodyId) {
        let mut world = PhysicsWorld::new();
        let fixed = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("fixed body");
        let moving = world
            .add_body(dynamic_body(Vec3::X * 1.2, Vec3::ZERO))
            .expect("moving body");
        let common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        world
            .add_joint(
                fixed,
                moving,
                Joint::Spring(SpringJoint::new(common, 1.0, 400.0, 2.0)),
            )
            .expect("spring joint");
        (world, moving)
    }

    fn collision_world() -> (PhysicsWorld, BodyId, BodyId) {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(dynamic_body(Vec3::X * -2.0, Vec3::X * 4.0))
            .expect("body a");
        let body_b = world
            .add_body(dynamic_body(Vec3::X * 2.0, Vec3::X * -4.0))
            .expect("body b");
        world
            .add_collider(
                body_a,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                    Transform::identity(),
                ),
            )
            .expect("collider a");
        world
            .add_collider(
                body_b,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                    Transform::identity(),
                ),
            )
            .expect("collider b");
        (world, body_a, body_b)
    }

    fn collision_state(warp: Option<f64>, end: Duration) -> (RigidBody, RigidBody, bool, usize) {
        let (mut world, body_a, body_b) = collision_world();
        let mut config = adaptive_config();
        if let TimeStepPolicy::Adaptive(mut adaptive) = config.time_step_policy {
            adaptive.minimum_step = Duration::from_nanos(1);
            config.time_step_policy = TimeStepPolicy::Adaptive(adaptive);
        }
        let base_step = config.step_budget();
        let target_step = warp.map(|warp| {
            Duration::try_from_secs_f64(base_step.as_secs_f64() * warp).expect("target step")
        });
        let reference_step = Duration::from_micros(100);
        let mut simulation_time = Duration::ZERO;
        let mut steps = 0;
        let mut contact_started = false;
        while simulation_time < end {
            let remaining = end.saturating_sub(simulation_time);
            let step = if let Some(target_step) = target_step {
                match choose_step(&world, &config, target_step, remaining, base_step) {
                    StepChoice::Step(step) => step,
                    other => panic!("adaptive collision did not progress: {other:?}"),
                }
            } else {
                reference_step.min(remaining)
            };
            world.step(step.as_secs_f64()).expect("collision step");
            contact_started |= world
                .collision_events()
                .iter()
                .any(|event| event.kind == CollisionEventKind::Begin);
            simulation_time += step;
            steps += 1;
        }
        (
            *world.body(body_a).expect("body a state"),
            *world.body(body_b).expect("body b state"),
            contact_started,
            steps,
        )
    }

    fn collision_after_warp_switch(
        switch_at: Duration,
        end: Duration,
    ) -> (RigidBody, RigidBody, bool, usize) {
        let (mut world, body_a, body_b) = collision_world();
        let mut config = adaptive_config();
        if let TimeStepPolicy::Adaptive(mut adaptive) = config.time_step_policy {
            adaptive.minimum_step = Duration::from_nanos(1);
            config.time_step_policy = TimeStepPolicy::Adaptive(adaptive);
        }
        let base_step = config.step_budget();
        let mut simulation_time = Duration::ZERO;
        let mut steps = 0;
        let mut contact_started = false;
        while simulation_time < end {
            let warp = if simulation_time < switch_at {
                1_000.0
            } else {
                1.0
            };
            let target_step =
                Duration::try_from_secs_f64(base_step.as_secs_f64() * warp).expect("target step");
            let remaining = end.saturating_sub(simulation_time);
            let step = match choose_step(&world, &config, target_step, remaining, base_step) {
                StepChoice::Step(step) => step,
                other => panic!("warp transition made no progress: {other:?}"),
            };
            world.step(step.as_secs_f64()).expect("collision step");
            contact_started |= world
                .collision_events()
                .iter()
                .any(|event| event.kind == CollisionEventKind::Begin);
            simulation_time += step;
            steps += 1;
        }
        (
            *world.body(body_a).expect("body a state"),
            *world.body(body_b).expect("body b state"),
            contact_started,
            steps,
        )
    }

    fn fixed_joint_world() -> (PhysicsWorld, BodyId, BodyId, Vec3, Vec3) {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::ZERO))
            .expect("body a");
        let body_b = world
            .add_body(dynamic_body(Vec3::X, Vec3::Y))
            .expect("body b");
        let anchor_a = Vec3::X * 0.5;
        let anchor_b = Vec3::X * -0.5;
        let common = JointCommon::new(
            JointFrame::new(anchor_a, Quat::IDENTITY),
            JointFrame::new(anchor_b, Quat::IDENTITY),
        );
        world
            .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
            .expect("fixed joint");
        (world, body_a, body_b, anchor_a, anchor_b)
    }

    fn orbit_state(warp: Option<f64>, end: Duration) -> (RigidBody, usize) {
        let (mut world, body_id) = orbit_world();
        let steps = integrate_orbit(&mut world, body_id, warp, end);
        (*world.body(body_id).expect("orbit body"), steps)
    }

    fn integrate_orbit(
        world: &mut PhysicsWorld,
        body_id: BodyId,
        warp: Option<f64>,
        end: Duration,
    ) -> usize {
        let config = adaptive_config();
        let base_step = config.step_budget();
        let target_step = warp.map(|warp| {
            Duration::try_from_secs_f64(base_step.as_secs_f64() * warp).expect("target step")
        });
        let reference_step = Duration::from_micros(100);
        let mut simulation_time = Duration::ZERO;
        let mut steps = 0;
        while simulation_time < end {
            let remaining = end.saturating_sub(simulation_time);
            let step = if let Some(target_step) = target_step {
                match choose_step(world, &config, target_step, remaining, base_step) {
                    StepChoice::Step(step) => step,
                    other => panic!("adaptive orbit did not progress: {other:?}"),
                }
            } else {
                reference_step.min(remaining)
            };
            world.step(step.as_secs_f64()).expect("orbit step");
            simulation_time += step;
            steps += 1;
        }
        black_box(world.body(body_id).expect("orbit result"));
        steps
    }

    fn integrate_fixed_orbit(
        world: &mut PhysicsWorld,
        body_id: BodyId,
        fixed_step: Duration,
        end: Duration,
    ) -> usize {
        let mut simulation_time = Duration::ZERO;
        let mut steps = 0;
        while simulation_time < end {
            let step = fixed_step.min(end.saturating_sub(simulation_time));
            world.step(step.as_secs_f64()).expect("fixed orbit step");
            simulation_time += step;
            steps += 1;
        }
        black_box(world.body(body_id).expect("fixed orbit result"));
        steps
    }

    fn spring_state(warp: Option<f64>, end: Duration) -> (RigidBody, usize) {
        let (mut world, body_id) = spring_world();
        let config = adaptive_config();
        let base_step = config.step_budget();
        let target_step = warp.map(|warp| {
            Duration::try_from_secs_f64(base_step.as_secs_f64() * warp).expect("target step")
        });
        let reference_step = Duration::from_micros(100);
        let mut simulation_time = Duration::ZERO;
        let mut steps = 0;
        while simulation_time < end {
            let remaining = end.saturating_sub(simulation_time);
            let step = if let Some(target_step) = target_step {
                match choose_step(&world, &config, target_step, remaining, base_step) {
                    StepChoice::Step(step) => step,
                    other => panic!("adaptive spring did not progress: {other:?}"),
                }
            } else {
                reference_step.min(remaining)
            };
            world.step(step.as_secs_f64()).expect("spring step");
            simulation_time += step;
            steps += 1;
        }
        (*world.body(body_id).expect("moving body"), steps)
    }

    fn fixed_joint_state(
        warp: Option<f64>,
        end: Duration,
    ) -> (RigidBody, RigidBody, Vec3, Vec3, usize) {
        let (mut world, body_a, body_b, anchor_a, anchor_b) = fixed_joint_world();
        let config = adaptive_config();
        let base_step = config.step_budget();
        let target_step = warp.map(|warp| {
            Duration::try_from_secs_f64(base_step.as_secs_f64() * warp).expect("target step")
        });
        let reference_step = Duration::from_micros(100);
        let mut simulation_time = Duration::ZERO;
        let mut steps = 0;
        while simulation_time < end {
            let remaining = end.saturating_sub(simulation_time);
            let step = if let Some(target_step) = target_step {
                match choose_step(&world, &config, target_step, remaining, base_step) {
                    StepChoice::Step(step) => step,
                    other => panic!("adaptive fixed joint did not progress: {other:?}"),
                }
            } else {
                reference_step.min(remaining)
            };
            world.step(step.as_secs_f64()).expect("fixed joint step");
            simulation_time += step;
            steps += 1;
        }
        (
            *world.body(body_a).expect("body a"),
            *world.body(body_b).expect("body b"),
            anchor_a,
            anchor_b,
            steps,
        )
    }

    #[test]
    fn calm_free_motion_uses_the_target_interval_up_to_the_safety_cap() {
        let world = PhysicsWorld::new();
        let accumulated = Duration::from_secs(2);
        assert!(matches!(
            choice(&world, 100.0, accumulated),
            StepChoice::Step(step) if step == Duration::from_millis(500)
        ));
        assert!(matches!(
            choice(&world, 1.0, accumulated),
            StepChoice::Step(step) if step == Duration::from_nanos(8_333_333)
        ));
    }

    #[test]
    fn motion_and_active_contacts_reduce_the_step() {
        let mut world = PhysicsWorld::new();
        let body = world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::X * 20.0))
            .expect("body");
        world
            .add_collider(
                body,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                    Transform::identity(),
                ),
            )
            .expect("collider");

        let unconstrained = match choice(&world, 100.0, Duration::from_secs(2)) {
            StepChoice::Step(step) => step,
            other => panic!("unexpected step choice: {other:?}"),
        };
        assert!(unconstrained < Duration::from_millis(5));

        let mut contact_world = PhysicsWorld::new();
        let a = contact_world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::ZERO))
            .expect("body a");
        let b = contact_world
            .add_body(dynamic_body(Vec3::X, Vec3::ZERO))
            .expect("body b");
        for body in [a, b] {
            contact_world
                .add_collider(
                    body,
                    Collider::new(
                        Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                        Transform::identity(),
                    ),
                )
                .expect("collider");
        }
        contact_world.step(1.0 / 120.0).expect("contact step");
        assert!(matches!(
            choice(&contact_world, 100.0, Duration::from_secs(2)),
            StepChoice::Step(step) if step <= Duration::from_nanos(8_333_334)
        ));
    }

    #[test]
    fn gravity_and_stiff_joints_limit_the_step() {
        let mut world = PhysicsWorld::new();
        world
            .add_body(dynamic_body(Vec3::new(10.0, 0.0, 0.0), Vec3::ZERO))
            .expect("body a");
        world
            .add_body(dynamic_body(Vec3::new(11.0, 0.0, 0.0), Vec3::ZERO))
            .expect("body b");
        world.set_central_gravity(Some(
            CentralGravity::new(Vec3::ZERO, 10_000.0).expect("gravity"),
        ));
        let gravity_step = match choice(&world, 100.0, Duration::from_secs(2)) {
            StepChoice::Step(step) => step,
            other => panic!("unexpected step choice: {other:?}"),
        };
        assert!(gravity_step < Duration::from_millis(100));

        let mut joint_world = PhysicsWorld::new();
        let body_a = joint_world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::ZERO))
            .expect("body a");
        let body_b = joint_world
            .add_body(dynamic_body(Vec3::X, Vec3::ZERO))
            .expect("body b");
        let common = JointCommon::new(
            JointFrame::new(Vec3::X * 0.5, Quat::IDENTITY),
            JointFrame::new(Vec3::X * -0.5, Quat::IDENTITY),
        );
        joint_world
            .add_joint(
                body_a,
                body_b,
                Joint::Spring(SpringJoint::new(common, 1.0, 400.0, 0.0)),
            )
            .expect("spring");
        let joint_step = match choice(&joint_world, 100.0, Duration::from_secs(2)) {
            StepChoice::Step(step) => step,
            other => panic!("unexpected step choice: {other:?}"),
        };
        assert!(joint_step < Duration::from_millis(20));
    }

    #[test]
    fn accumulated_and_persistent_forces_limit_the_step() {
        let mut world = PhysicsWorld::new();
        let body = world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::ZERO))
            .expect("body");
        world
            .body_mut(body)
            .expect("body")
            .apply_force(Vec3::X * 1_000.0)
            .expect("force");
        let accumulated_force_step = match choice(&world, 100.0, Duration::from_secs(2)) {
            StepChoice::Step(step) => step,
            other => panic!("unexpected step choice: {other:?}"),
        };
        assert!(accumulated_force_step < Duration::from_millis(20));

        let mut world = PhysicsWorld::new();
        let body = world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::ZERO))
            .expect("body");
        world
            .add_force_source(body, Vec3::ZERO, Vec3::X * 1_000.0)
            .expect("force source");
        let persistent_force_step = match choice(&world, 100.0, Duration::from_secs(2)) {
            StepChoice::Step(step) => step,
            other => panic!("unexpected step choice: {other:?}"),
        };
        assert!(persistent_force_step < Duration::from_millis(20));
    }

    #[test]
    fn adaptive_warps_track_a_fine_fixed_orbit_at_the_same_simulation_time() {
        let end = Duration::from_nanos(8_333_333 * 120);
        let (reference, _) = orbit_state(None, end);
        let reference_position = reference.state().transform().translation();
        let reference_velocity = reference.state().linear_velocity();
        let reference_radius = reference_position.length();
        let reference_energy =
            0.5 * reference_velocity.length_squared() - 10_000.0 / reference_radius;
        let reference_momentum = reference_position.cross(reference_velocity).length();

        for warp in [1.0, 10.0, 100.0, 1_000.0] {
            let (body, steps) = orbit_state(Some(warp), end);
            let position = body.state().transform().translation();
            let velocity = body.state().linear_velocity();
            let specific_energy = 0.5 * velocity.length_squared() - 10_000.0 / position.length();
            let angular_momentum = position.cross(velocity).length();
            let position_error = (position - reference_position).length();
            let velocity_error = (velocity - reference_velocity).length();
            let energy_drift = (specific_energy - reference_energy).abs() / reference_energy.abs();
            let momentum_drift = (angular_momentum - reference_momentum).abs() / reference_momentum;
            assert!(position_error < 0.25, "warp={warp}x: {position_error} m");
            assert!(velocity_error < 0.4, "warp={warp}x: {velocity_error} m/s");
            assert!(energy_drift < 0.0002, "warp={warp}x: {energy_drift}");
            assert!(momentum_drift < 1.0e-10, "warp={warp}x: {momentum_drift}");
            assert!(steps <= 400, "warp={warp}x: {steps} steps");

            let (repeat, _) = orbit_state(Some(warp), end);
            assert_eq!(
                body.state().transform().translation(),
                repeat.state().transform().translation(),
                "same adaptive configuration must reproduce the position"
            );
            assert_eq!(
                body.state().linear_velocity(),
                repeat.state().linear_velocity()
            );
            assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
            assert_eq!(
                body.state().transform().rotation(),
                Quat::IDENTITY,
                "central gravity must not introduce rotation"
            );
        }
    }

    #[test]
    fn adaptive_warps_track_a_fine_fixed_spring_at_the_same_simulation_time() {
        let end = Duration::from_millis(500);
        let (reference, _) = spring_state(None, end);
        let reference_position = reference.state().transform().translation();
        let reference_velocity = reference.state().linear_velocity();
        for warp in [1.0, 10.0, 100.0, 1_000.0] {
            let (body, steps) = spring_state(Some(warp), end);
            let (repeat, _) = spring_state(Some(warp), end);
            let position = body.state().transform().translation();
            let velocity = body.state().linear_velocity();
            let position_error = (position - reference_position).length();
            let velocity_error = (velocity - reference_velocity).length();
            let anchor_delta = (position.length() - reference_position.length()).abs();
            assert!(position_error < 0.02, "warp={warp}x: {position_error} m");
            assert!(velocity_error < 0.04, "warp={warp}x: {velocity_error} m/s");
            assert!(anchor_delta < 0.02, "warp={warp}x: {anchor_delta} m");
            assert!(steps <= 150, "warp={warp}x: {steps} steps");
            assert_eq!(position, repeat.state().transform().translation());
            assert_eq!(velocity, repeat.state().linear_velocity());
        }
    }

    #[test]
    fn adaptive_warps_preserve_contact_outcomes_at_the_same_simulation_time() {
        let end = Duration::from_millis(500);
        let (reference_a, reference_b, reference_contact, _) = collision_state(None, end);
        assert!(
            reference_contact,
            "fine reference should observe the contact"
        );
        for warp in [1.0, 10.0, 100.0, 1_000.0] {
            let (body_a, body_b, contact_started, steps) = collision_state(Some(warp), end);
            let (repeat_a, repeat_b, repeat_contact, _) = collision_state(Some(warp), end);
            let position_error = (body_a.state().transform().translation()
                - reference_a.state().transform().translation())
            .length()
                + (body_b.state().transform().translation()
                    - reference_b.state().transform().translation())
                .length();
            let velocity_error =
                (body_a.state().linear_velocity() - reference_a.state().linear_velocity()).length()
                    + (body_b.state().linear_velocity() - reference_b.state().linear_velocity())
                        .length();
            assert!(contact_started, "warp={warp}x missed the contact");
            assert!(position_error < 0.1, "warp={warp}x: {position_error} m");
            assert!(velocity_error < 0.2, "warp={warp}x: {velocity_error} m/s");
            assert!(steps <= 150, "warp={warp}x: {steps} steps");
            assert!(body_a.state().linear_velocity().x < 0.0);
            assert!(body_b.state().linear_velocity().x > 0.0);
            assert!(
                body_a.state().transform().translation().x
                    < body_b.state().transform().translation().x
            );
            assert_eq!(contact_started, repeat_contact);
            assert_eq!(body_a.state(), repeat_a.state());
            assert_eq!(body_b.state(), repeat_b.state());
        }
    }

    #[test]
    fn adaptive_warps_preserve_fixed_joint_anchors_at_the_same_simulation_time() {
        let end = Duration::from_millis(500);
        let (reference_a, reference_b, anchor_a, anchor_b, _) = fixed_joint_state(None, end);
        let reference_anchor_a = reference_a.state().transform().transform_point(anchor_a);
        let reference_anchor_b = reference_b.state().transform().transform_point(anchor_b);
        let reference_gap = (reference_anchor_b - reference_anchor_a).length();
        let reference_rotation = reference_b.state().transform().rotation();
        for warp in [1.0, 10.0, 100.0, 1_000.0] {
            let (body_a, body_b, anchor_a, anchor_b, steps) = fixed_joint_state(Some(warp), end);
            let (repeat_a, repeat_b, _, _, _) = fixed_joint_state(Some(warp), end);
            let world_anchor_a = body_a.state().transform().transform_point(anchor_a);
            let world_anchor_b = body_b.state().transform().transform_point(anchor_b);
            let gap = (world_anchor_b - world_anchor_a).length();
            let rotation = body_b.state().transform().rotation();
            let quaternion_dot = (rotation.x * reference_rotation.x
                + rotation.y * reference_rotation.y
                + rotation.z * reference_rotation.z
                + rotation.w * reference_rotation.w)
                .abs()
                .min(1.0);
            let rotation_error = 2.0 * quaternion_dot.acos();
            assert!((gap - reference_gap).abs() < 0.01, "warp={warp}x: {gap} m");
            assert!(gap < 0.02, "warp={warp}x: anchor gap {gap} m");
            assert!(rotation_error < 0.02, "warp={warp}x: {rotation_error} rad");
            assert!(steps <= 150, "warp={warp}x: {steps} steps");
            assert_eq!(body_a.state(), repeat_a.state());
            assert_eq!(body_b.state(), repeat_b.state());
        }
    }

    #[test]
    fn adaptive_joint_break_uses_the_selected_substep_duration() {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::ZERO))
            .expect("body a");
        let body_b = world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::X * 10.0))
            .expect("body b");
        let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        common.break_force = Some(100.0);
        let joint_id = world
            .add_joint(body_a, body_b, Joint::Fixed(FixedJoint::new(common)))
            .expect("breakable joint");
        let mut config = adaptive_config();
        if let TimeStepPolicy::Adaptive(mut adaptive) = config.time_step_policy {
            adaptive.minimum_step = Duration::from_nanos(1);
            config.time_step_policy = TimeStepPolicy::Adaptive(adaptive);
        }
        let base_step = config.step_budget();
        let target_step = base_step * 1_000;
        let step = match choose_step(
            &world,
            &config,
            target_step,
            Duration::from_secs(1),
            base_step,
        ) {
            StepChoice::Step(step) => step,
            other => panic!("adaptive breakable joint made no progress: {other:?}"),
        };
        assert!(
            step < base_step,
            "the relative anchor motion should subdivide dt"
        );

        world.step(step.as_secs_f64()).expect("break step");

        assert_eq!(world.joint_count(), 0);
        assert_eq!(world.joint_events().len(), 1);
        let event = world.joint_events()[0];
        assert_eq!(event.joint(), joint_id);
        let impulse = world
            .body(body_a)
            .expect("body a after break")
            .state()
            .linear_velocity()
            .length();
        assert!((event.force() - impulse / step.as_secs_f64()).abs() < 1.0e-6);
        assert!(event.force() > 100.0);
        assert!(event.torque() < 1.0e-6);
    }

    #[test]
    fn adaptive_warp_can_drop_from_one_thousand_before_contact() {
        let end = Duration::from_millis(500);
        let (reference_a, reference_b, reference_contact, _) = collision_state(None, end);
        assert!(reference_contact);
        let switch_at = Duration::from_millis(350);
        let (body_a, body_b, contact_started, steps) = collision_after_warp_switch(switch_at, end);
        let (repeat_a, repeat_b, repeat_contact, _) = collision_after_warp_switch(switch_at, end);
        assert!(
            contact_started,
            "the scheduled collision should be observed"
        );
        assert!(steps <= 150);
        assert!(
            (body_a.state().transform().translation()
                - reference_a.state().transform().translation())
            .length()
                < 0.1
        );
        assert!(
            (body_b.state().transform().translation()
                - reference_b.state().transform().translation())
            .length()
                < 0.1
        );
        assert_eq!(contact_started, repeat_contact);
        assert_eq!(body_a.state(), repeat_a.state());
        assert_eq!(body_b.state(), repeat_b.state());
    }

    #[test]
    #[ignore = "Release adaptive Time Warp accuracy and scaling benchmark"]
    fn benchmark_adaptive_time_warp_orbit() {
        const SAMPLES: usize = 5;
        let end = Duration::from_nanos(8_333_333 * 120);
        let (reference, _) = orbit_state(None, end);
        let reference_position = reference.state().transform().translation();
        let reference_velocity = reference.state().linear_velocity();
        let reference_energy =
            0.5 * reference_velocity.length_squared() - 10_000.0 / reference_position.length();
        let reference_momentum = reference_position.cross(reference_velocity).length();

        let base_step = adaptive_config().step_budget();
        let (mut fixed_warmup_world, fixed_warmup_body) = orbit_world();
        black_box(integrate_fixed_orbit(
            &mut fixed_warmup_world,
            fixed_warmup_body,
            base_step,
            end,
        ));
        let mut fixed_samples = Vec::with_capacity(SAMPLES);
        let mut fixed_state = None;
        let mut fixed_steps = 0;
        for _ in 0..SAMPLES {
            let (mut world, body_id) = orbit_world();
            let started = Instant::now();
            fixed_steps = integrate_fixed_orbit(&mut world, body_id, base_step, end);
            fixed_samples.push(started.elapsed());
            fixed_state = world.body(body_id).copied();
        }
        fixed_samples.sort_unstable();
        let fixed_median = fixed_samples[SAMPLES / 2];
        let fixed_state = fixed_state.expect("fixed benchmark body");
        println!(
            "fixed orbit dt_ns={} steps={} wall_ms min/median/max={:.3}/{:.3}/{:.3} sim_s_per_wall_s={:.2} us_per_step={:.3} position_error_m={:.6} velocity_error_m_s={:.6}",
            base_step.as_nanos(),
            fixed_steps,
            fixed_samples[0].as_secs_f64() * 1.0e3,
            fixed_median.as_secs_f64() * 1.0e3,
            fixed_samples[SAMPLES - 1].as_secs_f64() * 1.0e3,
            end.as_secs_f64() / fixed_median.as_secs_f64(),
            fixed_median.as_secs_f64() * 1.0e6 / fixed_steps as f64,
            (fixed_state.state().transform().translation() - reference_position).length(),
            (fixed_state.state().linear_velocity() - reference_velocity).length(),
        );

        let (estimator_world, _) = orbit_world();
        let estimator_config = adaptive_config();
        let estimator_base = estimator_config.step_budget();
        let estimator_target = Duration::try_from_secs_f64(estimator_base.as_secs_f64() * 1_000.0)
            .expect("estimator target");
        for _ in 0..10_000 {
            black_box(choose_step(
                &estimator_world,
                &estimator_config,
                estimator_target,
                end,
                estimator_base,
            ));
        }
        let mut estimator_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..10_000 {
                black_box(choose_step(
                    &estimator_world,
                    &estimator_config,
                    estimator_target,
                    end,
                    estimator_base,
                ));
            }
            estimator_samples.push(started.elapsed().div_f64(10_000.0));
        }
        estimator_samples.sort_unstable();
        println!(
            "adaptive estimator on 1-body gravity world: min/median/max={:.3}/{:.3}/{:.3} us/choice",
            estimator_samples[0].as_secs_f64() * 1.0e6,
            estimator_samples[SAMPLES / 2].as_secs_f64() * 1.0e6,
            estimator_samples[SAMPLES - 1].as_secs_f64() * 1.0e6,
        );

        for warp in [1.0, 10.0, 100.0, 1_000.0] {
            let (mut warmup_world, warmup_body) = orbit_world();
            black_box(integrate_orbit(
                &mut warmup_world,
                warmup_body,
                Some(warp),
                end,
            ));

            let mut samples = Vec::with_capacity(SAMPLES);
            let mut last_state = None;
            let mut steps = 0;
            for _ in 0..SAMPLES {
                let (mut world, body_id) = orbit_world();
                let started = Instant::now();
                steps = integrate_orbit(&mut world, body_id, Some(warp), end);
                samples.push(started.elapsed());
                last_state = world.body(body_id).copied();
            }
            samples.sort_unstable();
            let median = samples[SAMPLES / 2];
            let body = last_state.expect("benchmark body");
            let position = body.state().transform().translation();
            let velocity = body.state().linear_velocity();
            let energy = 0.5 * velocity.length_squared() - 10_000.0 / position.length();
            let momentum = position.cross(velocity).length();
            println!(
                "adaptive orbit warp={warp:.0}x steps={steps} wall_ms min/median/max={:.3}/{:.3}/{:.3} sim_s_per_wall_s={:.2} us_per_step={:.3} position_error_m={:.6} velocity_error_m_s={:.6} energy_drift_percent={:.6} angular_momentum_drift_percent={:.9}",
                samples[0].as_secs_f64() * 1.0e3,
                median.as_secs_f64() * 1.0e3,
                samples[SAMPLES - 1].as_secs_f64() * 1.0e3,
                end.as_secs_f64() / median.as_secs_f64(),
                median.as_secs_f64() * 1.0e6 / steps as f64,
                (position - reference_position).length(),
                (velocity - reference_velocity).length(),
                100.0 * (energy - reference_energy).abs() / reference_energy.abs(),
                100.0 * (momentum - reference_momentum).abs() / reference_momentum,
            );
        }
    }

    #[test]
    fn an_unachievable_minimum_step_is_reported_without_progress() {
        let mut config = adaptive_config();
        config.time_step_policy = TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig {
            minimum_step: Duration::from_millis(5),
            maximum_step: Duration::from_millis(10),
            relative_error_tolerance: 0.01,
        });
        let base = config.step_budget();
        let target = Duration::from_millis(100);
        let mut world = PhysicsWorld::new();
        let body = world
            .add_body(dynamic_body(Vec3::ZERO, Vec3::X * 1_000.0))
            .expect("body");
        world
            .add_collider(
                body,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.01).expect("sphere")),
                    Transform::identity(),
                ),
            )
            .expect("collider");
        assert!(matches!(
            choose_step(&world, &config, target, Duration::from_secs(1), base),
            StepChoice::Limited
        ));
    }

    #[test]
    fn subminimum_target_accumulates_and_paused_debt_can_drain() {
        let mut config = adaptive_config();
        config.time_step_policy = TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig {
            minimum_step: Duration::from_millis(5),
            maximum_step: Duration::from_millis(10),
            relative_error_tolerance: 0.01,
        });
        let world = PhysicsWorld::new();
        let base = config.step_budget();
        let target = Duration::from_millis(1);
        assert!(matches!(
            choose_step(&world, &config, target, Duration::from_millis(3), base),
            StepChoice::Wait
        ));
        assert!(matches!(
            choose_step(&world, &config, target, Duration::from_millis(7), base),
            StepChoice::Step(step) if step == Duration::from_millis(5)
        ));
        assert!(matches!(
            choose_step(&world, &config, Duration::ZERO, Duration::from_millis(7), base),
            StepChoice::Step(step) if step == Duration::from_millis(5)
        ));
    }
}
