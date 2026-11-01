use kapitsa_core::{
    collider::Collider,
    gravity::CentralGravity,
    joint::{FixedJoint, Joint, JointCommon, JointFrame},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Cuboid, Plane, Shape},
    transform::Transform,
    world::{BodyId, ColliderId, ForceSourceId, JointId, PhysicsWorld},
};
use kapitsa_math::{Mat3, Quat, Vec3};

pub struct ModularRocket {
    pub world: PhysicsWorld,
    pub surface_collider: ColliderId,
    pub parts: Vec<BodyId>,
    pub lower_parts: Vec<BodyId>,
    pub upper_parts: Vec<BodyId>,
    pub lower_root: BodyId,
    pub upper_root: BodyId,
    pub separation_joint: JointId,
    pub lower_engine: ForceSourceId,
    pub upper_engine: ForceSourceId,
    pub lower_tanks: Vec<BodyId>,
    pub upper_tanks: Vec<BodyId>,
}

impl ModularRocket {
    pub fn new(total_parts: usize) -> Self {
        assert!(total_parts >= 8 && total_parts.is_multiple_of(2));
        let stage_parts = total_parts / 2;
        let mut world = PhysicsWorld::new();
        let gravity_radius = 6_371_000.0;
        world.set_central_gravity(Some(
            CentralGravity::new(
                Vec3::new(0.0, -gravity_radius, 0.0),
                9.81 * gravity_radius * gravity_radius,
            )
            .expect("surface gravity"),
        ));

        let surface = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("surface body");
        let surface_collider = world
            .add_collider(
                surface,
                Collider::new(
                    Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0).expect("surface shape")),
                    Transform::identity(),
                ),
            )
            .expect("surface collider");

        let mut stages: [Vec<BodyId>; 2] = std::array::from_fn(|_| Vec::with_capacity(stage_parts));
        let mut tanks: [Vec<BodyId>; 2] = std::array::from_fn(|_| Vec::new());
        for stage in 0..2 {
            let root_position = Vec3::new(0.0, if stage == 0 { 0.5 } else { 1.5 }, 0.0);
            let trunk_parts = ((stage_parts as f64).sqrt() as usize).max(2);
            let mut positions = Vec::with_capacity(stage_parts);
            for index in 0..stage_parts {
                let (position, parent) = if index == 0 {
                    (root_position, None)
                } else if index < trunk_parts {
                    let offset = Vec3::new(0.0, 0.22, 0.0);
                    (positions[index - 1] + offset, Some((index - 1, offset)))
                } else {
                    let branch = index - trunk_parts;
                    let parent = (branch / 2) % (trunk_parts - 1);
                    let layer = branch / (2 * (trunk_parts - 1));
                    let side = if branch.is_multiple_of(2) { 1.0 } else { -1.0 };
                    let offset = Vec3::new(
                        side * (0.16 + layer as f64 * 0.01),
                        0.12 + layer as f64 * 0.01,
                        if branch.is_multiple_of(3) {
                            0.08
                        } else {
                            -0.08
                        },
                    );
                    (positions[parent] + offset, Some((parent, offset)))
                };

                let root = index == 0;
                let mass = if root {
                    5.0
                } else if index.is_multiple_of(5) {
                    1.2 + (index % 3) as f64 * 0.2
                } else {
                    0.45 + (index % 7) as f64 * 0.09
                };
                let body = RigidBody::new_dynamic(
                    Transform::new(position, Quat::IDENTITY).expect("part transform"),
                    rocket_part_mass(root, mass),
                )
                .expect("rocket part");
                let body = world.add_body(body).expect("rocket part body");
                stages[stage].push(body);
                positions.push(position);
                if let Some((parent, offset)) = parent {
                    world
                        .add_joint(stages[stage][parent], body, fixed_joint_for_offset(offset))
                        .expect("rocket module joint");
                }
                if root || index.is_multiple_of(5) {
                    tanks[stage].push(body);
                }
                if root {
                    world
                        .add_collider(
                            body,
                            Collider::new(
                                Shape::Cuboid(
                                    Cuboid::new(0.5, 0.5, 0.5).expect("rocket root shape"),
                                ),
                                Transform::identity(),
                            ),
                        )
                        .expect("rocket root collider");
                }
            }
        }

        let lower_root = stages[0][0];
        let upper_root = stages[1][0];
        let separation_joint = world
            .add_joint(lower_root, upper_root, fixed_joint_for_offset(Vec3::Y))
            .expect("stage connection");
        let stage_masses: [f64; 2] = std::array::from_fn(|stage| {
            stages[stage]
                .iter()
                .map(|&id| {
                    world
                        .body(id)
                        .and_then(|body| body.physics().mass())
                        .expect("stage part mass")
                        .mass()
                })
                .sum::<f64>()
        });
        let lower_engine = world
            .add_force_source(
                lower_root,
                Vec3::new(-0.04, -0.2, 0.0),
                Vec3::Y * (stage_masses[0] + stage_masses[1]) * 9.81 * 1.3,
            )
            .expect("booster engine");
        let upper_engine = world
            .add_force_source(
                upper_root,
                Vec3::new(-0.04, -0.2, 0.0),
                Vec3::Y * stage_masses[1] * 9.81 * 1.35,
            )
            .expect("upper stage engine");
        world
            .set_force_source_enabled(lower_engine, false)
            .expect("booster engine cutoff before launch");
        world
            .set_force_source_enabled(upper_engine, false)
            .expect("upper engine cutoff");

        let mut parts = stages[0].clone();
        parts.extend_from_slice(&stages[1]);
        Self {
            world,
            surface_collider,
            parts,
            lower_parts: stages[0].clone(),
            upper_parts: stages[1].clone(),
            lower_root,
            upper_root,
            separation_joint,
            lower_engine,
            upper_engine,
            lower_tanks: tanks[0].clone(),
            upper_tanks: tanks[1].clone(),
        }
    }
}

pub fn rocket_part_mass(root: bool, mass: f64) -> MassProperties {
    let center = if root {
        Vec3::new(-0.05, 0.0, 0.0)
    } else {
        Vec3::ZERO
    };
    let scale = if root {
        2.0
    } else {
        mass * (0.035 + (mass % 3.0) * 0.002)
    };
    MassProperties::new(
        mass,
        center,
        Mat3::from_diagonal(Vec3::new(scale, scale * 1.2, scale * 0.8)),
    )
    .expect("part mass")
}

pub fn fixed_joint_for_offset(offset: Vec3) -> Joint {
    let common = JointCommon::new(
        JointFrame::new(offset * 0.5, Quat::IDENTITY),
        JointFrame::new(offset * -0.5, Quat::IDENTITY),
    );
    Joint::Fixed(FixedJoint::new(common))
}

#[cfg(all(test, not(feature = "bench-internals")))]
mod tests {
    use std::collections::{HashMap, VecDeque};

    use super::ModularRocket;

    #[test]
    fn requested_rocket_sizes_have_connected_repeatable_joint_graphs() {
        for total_parts in [100, 200, 500, 1000] {
            let first = ModularRocket::new(total_parts);
            let repeat = ModularRocket::new(total_parts);
            assert_eq!(first.parts.len(), total_parts);
            assert_eq!(first.world.joint_count(), total_parts - 1);
            assert_eq!(
                first.lower_parts.len() + first.upper_parts.len(),
                total_parts
            );
            assert!(!first.lower_tanks.is_empty() && !first.upper_tanks.is_empty());

            let indices = first
                .parts
                .iter()
                .enumerate()
                .map(|(index, &body)| (body, index))
                .collect::<HashMap<_, _>>();
            let mut edges = Vec::with_capacity(total_parts - 1);
            let mut adjacency = vec![Vec::new(); total_parts];
            let mut degree = vec![0; total_parts];
            for (_, body_a, body_b, _) in first.world.joints() {
                let a = indices[&body_a];
                let b = indices[&body_b];
                edges.push((a.min(b), a.max(b)));
                adjacency[a].push(b);
                adjacency[b].push(a);
                degree[a] += 1;
                degree[b] += 1;
            }
            edges.sort_unstable();
            let mut visited = vec![false; total_parts];
            let mut queue = VecDeque::from([0]);
            visited[0] = true;
            while let Some(current) = queue.pop_front() {
                for &next in &adjacency[current] {
                    if !visited[next] {
                        visited[next] = true;
                        queue.push_back(next);
                    }
                }
            }
            assert!(visited.into_iter().all(|seen| seen), "{total_parts} parts");
            assert!(edges.contains(&(0, 1)), "linear trunk");
            assert!(degree.iter().any(|&degree| degree >= 3), "branches");

            let repeated_indices = repeat
                .parts
                .iter()
                .enumerate()
                .map(|(index, &body)| (body, index))
                .collect::<HashMap<_, _>>();
            let mut repeated_edges = repeat
                .world
                .joints()
                .map(|(_, a, b, _)| {
                    let a = repeated_indices[&a];
                    let b = repeated_indices[&b];
                    (a.min(b), a.max(b))
                })
                .collect::<Vec<_>>();
            repeated_edges.sort_unstable();
            assert_eq!(edges, repeated_edges, "topology must be deterministic");

            let masses = first
                .parts
                .iter()
                .map(|&id| {
                    first
                        .world
                        .body(id)
                        .and_then(|body| body.physics().mass())
                        .expect("dynamic part has mass")
                        .mass()
                        .to_bits()
                })
                .collect::<std::collections::HashSet<_>>();
            assert!(masses.len() > 2, "parts use varied masses");
        }
    }
}

#[cfg(all(test, not(feature = "bench-internals")))]
mod stability_tests {
    use std::time::{Duration, Instant};

    use kapitsa_core::collision::CollisionEventKind;
    use kapitsa_math::Vec3;

    use super::{ModularRocket, rocket_part_mass};

    const DT: f64 = 1.0 / 120.0;
    const TICKS: usize = 180;
    const SURFACE_RADIUS: f64 = 6_371_000.0;
    const GRAVITATIONAL_PARAMETER: f64 = 9.81 * SURFACE_RADIUS * SURFACE_RADIUS;

    #[derive(Debug, PartialEq)]
    struct PartState {
        position: Vec3,
        rotation: kapitsa_math::Quat,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    }

    struct StabilityRun {
        states: Vec<PartState>,
        max_anchor_error: f64,
        rms_anchor_error: f64,
        max_frame_error: f64,
        rms_frame_error: f64,
        max_coast_energy_drift: f64,
        max_altitude: f64,
        max_angular_speed: f64,
        separation_momentum_error: f64,
        surface_begins: usize,
        surface_ends: usize,
        surface_impacts: usize,
        lower_fuel_mass: f64,
        upper_fuel_mass: f64,
        joints_after_separation: usize,
        joint_break_events: usize,
        elapsed: Duration,
        completed_ticks: usize,
        step_failure: Option<String>,
    }

    #[test]
    #[ignore = "full 1/120 s modular rocket scale and repeatability test; run explicitly in release"]
    fn fixed_step_rockets_report_stability_and_reproducibility_at_all_sizes() {
        for part_count in [100, 200, 500, 1000] {
            let first = run_fixed_scenario(part_count);
            let repeated = run_fixed_scenario(part_count);
            assert_eq!(first.states, repeated.states, "{part_count} part repeat");
            assert_eq!(first.completed_ticks, repeated.completed_ticks);
            assert_eq!(first.step_failure, repeated.step_failure);
            let scenario_complete = first.step_failure.is_none()
                && first.completed_ticks == TICKS
                && first.max_altitude > 1.6
                && first.max_angular_speed > 0.01
                && first.separation_momentum_error < 1.0e-9
                && first.surface_begins > 0
                && first.surface_impacts > 0
                && first.lower_fuel_mass == 3.5
                && first.upper_fuel_mass == 3.5
                && first.joints_after_separation == part_count - 2
                && first.joint_break_events == 0;
            let numerical_stability = scenario_complete
                && first.max_anchor_error < 0.03
                && first.rms_anchor_error < 0.01
                && first.max_frame_error < 0.1
                && first.rms_frame_error < 0.03
                && first.max_coast_energy_drift < 0.1;
            println!(
                "parts={part_count} joints={} elapsed_ms={} completed_ticks={} scenario={} numerical_stability={} step_failure={:?} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad coast_energy_drift={:.4}% upper_root_max_y={:.3}m omega={:.3}rad_s separation_dp={:.3e} contacts(begin/end/impact)={}/{}/{} breaks={}",
                part_count - 1,
                first.elapsed.as_millis(),
                first.completed_ticks,
                if scenario_complete {
                    "COMPLETE"
                } else {
                    "INCOMPLETE"
                },
                if numerical_stability { "PASS" } else { "FAIL" },
                first.step_failure,
                first.max_anchor_error,
                first.rms_anchor_error,
                first.max_frame_error,
                first.rms_frame_error,
                first.max_coast_energy_drift * 100.0,
                first.max_altitude,
                first.max_angular_speed,
                first.separation_momentum_error,
                first.surface_begins,
                first.surface_ends,
                first.surface_impacts,
                first.joint_break_events,
            );
        }
    }

    fn run_fixed_scenario(part_count: usize) -> StabilityRun {
        let started = Instant::now();
        let mut rocket = ModularRocket::new(part_count);
        let initial_lower_mass = stage_mass(&rocket, &rocket.lower_parts);
        let initial_upper_mass = stage_mass(&rocket, &rocket.upper_parts);
        let mut max_anchor_error: f64 = 0.0;
        let mut anchor_square_sum = 0.0;
        let mut max_frame_error: f64 = 0.0;
        let mut frame_square_sum = 0.0;
        let mut joint_error_samples = 0usize;
        let mut max_coast_energy_drift: f64 = 0.0;
        let mut coast_energy_start = None;
        let mut max_altitude: f64 = 0.0;
        let mut max_angular_speed: f64 = 0.0;
        let mut separation_momentum_error = 0.0;
        let mut surface_begins = 0;
        let mut surface_ends = 0;
        let mut surface_impacts = 0;
        let mut joints_after_separation = 0;
        let mut completed_ticks = 0;
        let mut step_failure = None;
        let mut joint_break_events = 0;

        for tick in 0..TICKS {
            match tick {
                12 => rocket
                    .world
                    .set_force_source_enabled(rocket.lower_engine, true)
                    .expect("booster ignition"),
                24 => burn_tanks(&mut rocket, true),
                54 => {
                    let momentum_before = stage_momentum(&rocket, &rocket.lower_parts)
                        + stage_momentum(&rocket, &rocket.upper_parts);
                    rocket
                        .world
                        .remove_joint(rocket.separation_joint)
                        .expect("stage separation");
                    joints_after_separation = rocket.world.joint_count();
                    rocket
                        .world
                        .set_force_source_enabled(rocket.lower_engine, false)
                        .expect("booster cutoff");
                    rocket
                        .world
                        .set_force_source_enabled(rocket.upper_engine, true)
                        .expect("upper-stage ignition");
                    let impulse = Vec3::Y * 2.0;
                    let lower_parts = rocket.lower_parts.clone();
                    let upper_parts = rocket.upper_parts.clone();
                    apply_stage_impulse(&mut rocket, &lower_parts, -impulse);
                    apply_stage_impulse(&mut rocket, &upper_parts, impulse);
                    let momentum_after = stage_momentum(&rocket, &rocket.lower_parts)
                        + stage_momentum(&rocket, &rocket.upper_parts);
                    separation_momentum_error = (momentum_after - momentum_before).length();
                }
                72 => burn_tanks(&mut rocket, false),
                90 => rocket
                    .world
                    .set_force_source_enabled(rocket.upper_engine, false)
                    .expect("upper-stage cutoff"),
                _ => {}
            }

            if let Err(error) = rocket.world.step(DT) {
                step_failure = Some(format!("tick={tick}: {error:?}"));
                break;
            }
            completed_ticks += 1;
            joint_break_events += rocket.world.joint_events().len();
            for event in rocket.world.collision_events() {
                if event.sensor
                    || (event.pair.a != rocket.surface_collider
                        && event.pair.b != rocket.surface_collider)
                {
                    continue;
                }
                match event.kind {
                    CollisionEventKind::Begin => surface_begins += 1,
                    CollisionEventKind::End => surface_ends += 1,
                    CollisionEventKind::Impact => surface_impacts += 1,
                    CollisionEventKind::Persist => {}
                }
            }

            for (_, anchor_error, frame_error) in joint_errors(&rocket) {
                max_anchor_error = max_anchor_error.max(anchor_error);
                anchor_square_sum += anchor_error * anchor_error;
                max_frame_error = max_frame_error.max(frame_error);
                frame_square_sum += frame_error * frame_error;
                joint_error_samples += 1;
            }
            for &id in &rocket.parts {
                let body = rocket.world.body(id).expect("rocket body");
                let state = body.state();
                let transform = state.transform();
                let position = transform.translation();
                let rotation = transform.rotation();
                assert!(
                    position.is_finite()
                        && rotation.is_normalized()
                        && state.linear_velocity().is_finite()
                        && state.angular_velocity().is_finite(),
                    "non-finite part state at tick {tick}"
                );
                if id == rocket.upper_root {
                    max_altitude = max_altitude.max(position.y);
                }
                max_angular_speed = max_angular_speed.max(state.angular_velocity().length());
            }

            if tick >= 90 {
                let energy = mechanical_energy(&rocket, &rocket.upper_parts);
                let initial = *coast_energy_start.get_or_insert(energy);
                max_coast_energy_drift =
                    max_coast_energy_drift.max((energy - initial).abs() / initial.abs().max(1.0));
            }
        }

        if completed_ticks == TICKS {
            assert!(initial_lower_mass > stage_mass(&rocket, &rocket.lower_parts));
            assert!(initial_upper_mass > stage_mass(&rocket, &rocket.upper_parts));
        }
        let states = rocket
            .parts
            .iter()
            .map(|&id| {
                let body = rocket.world.body(id).expect("final rocket body");
                PartState {
                    position: body.state().transform().translation(),
                    rotation: body.state().transform().rotation(),
                    linear_velocity: body.state().linear_velocity(),
                    angular_velocity: body.state().angular_velocity(),
                }
            })
            .collect();
        let lower_fuel_mass = rocket
            .world
            .body(rocket.lower_root)
            .expect("lower stage root")
            .physics()
            .mass()
            .expect("lower stage mass")
            .mass();
        let upper_fuel_mass = rocket
            .world
            .body(rocket.upper_root)
            .expect("upper stage root")
            .physics()
            .mass()
            .expect("upper stage mass")
            .mass();
        let samples = joint_error_samples.max(1) as f64;
        StabilityRun {
            states,
            max_anchor_error,
            rms_anchor_error: (anchor_square_sum / samples).sqrt(),
            max_frame_error,
            rms_frame_error: (frame_square_sum / samples).sqrt(),
            max_coast_energy_drift,
            max_altitude,
            max_angular_speed,
            separation_momentum_error,
            surface_begins,
            surface_ends,
            surface_impacts,
            lower_fuel_mass,
            upper_fuel_mass,
            joints_after_separation,
            joint_break_events,
            elapsed: started.elapsed(),
            completed_ticks,
            step_failure,
        }
    }

    fn burn_tanks(rocket: &mut ModularRocket, lower: bool) {
        let root = if lower {
            rocket.lower_root
        } else {
            rocket.upper_root
        };
        let tanks = if lower {
            &rocket.lower_tanks
        } else {
            &rocket.upper_tanks
        };
        for &id in tanks {
            let mass = rocket
                .world
                .body(id)
                .and_then(|body| body.physics().mass())
                .expect("fuel tank mass")
                .mass();
            let next_mass = if id == root { 3.5 } else { mass * 0.8 };
            rocket
                .world
                .body_mut(id)
                .expect("fuel tank body")
                .set_mass(rocket_part_mass(id == root, next_mass))
                .expect("fuel mass update");
        }
    }

    fn stage_mass(rocket: &ModularRocket, parts: &[kapitsa_core::world::BodyId]) -> f64 {
        parts
            .iter()
            .map(|&id| {
                rocket
                    .world
                    .body(id)
                    .and_then(|body| body.physics().mass())
                    .expect("stage part mass")
                    .mass()
            })
            .sum()
    }

    fn stage_momentum(rocket: &ModularRocket, parts: &[kapitsa_core::world::BodyId]) -> Vec3 {
        parts.iter().fold(Vec3::ZERO, |momentum, &id| {
            let body = rocket.world.body(id).expect("stage part");
            let mass = body.physics().mass().expect("stage part mass").mass();
            momentum + body.state().linear_velocity() * mass
        })
    }

    fn apply_stage_impulse(
        rocket: &mut ModularRocket,
        parts: &[kapitsa_core::world::BodyId],
        impulse: Vec3,
    ) {
        let delta_velocity = impulse / stage_mass(rocket, parts);
        for &id in parts {
            let velocity = rocket
                .world
                .body(id)
                .expect("stage part")
                .state()
                .linear_velocity();
            rocket
                .world
                .body_mut(id)
                .expect("stage part")
                .set_linear_velocity(velocity + delta_velocity)
                .expect("separation impulse");
        }
    }

    fn joint_errors(rocket: &ModularRocket) -> Vec<(kapitsa_core::world::JointId, f64, f64)> {
        rocket
            .world
            .joints()
            .map(|(id, body_a_id, body_b_id, joint)| {
                let common = joint.common();
                let body_a = rocket.world.body(body_a_id).expect("joint body a");
                let body_b = rocket.world.body(body_b_id).expect("joint body b");
                let transform_a = body_a.state().transform();
                let transform_b = body_b.state().transform();
                let anchor_a = transform_a.transform_point(common.frame_a.local_anchor);
                let anchor_b = transform_b.transform_point(common.frame_b.local_anchor);
                let rotation_a = transform_a.rotation() * common.frame_a.local_rotation;
                let rotation_b = transform_b.rotation() * common.frame_b.local_rotation;
                let dot = (rotation_a.x * rotation_b.x
                    + rotation_a.y * rotation_b.y
                    + rotation_a.z * rotation_b.z
                    + rotation_a.w * rotation_b.w)
                    .abs()
                    .min(1.0);
                (id, (anchor_a - anchor_b).length(), 2.0 * dot.acos())
            })
            .collect()
    }

    fn mechanical_energy(rocket: &ModularRocket, parts: &[kapitsa_core::world::BodyId]) -> f64 {
        parts
            .iter()
            .map(|&id| {
                let body = rocket.world.body(id).expect("energy body");
                let mass = body.physics().mass().expect("energy mass");
                let state = body.state();
                let transform = state.transform();
                let position = transform.transform_point(mass.center_of_mass());
                let center_of_gravity = Vec3::new(0.0, -SURFACE_RADIUS, 0.0);
                let radius = (position - center_of_gravity).length();
                let velocity = state.linear_velocity();
                let omega = state.angular_velocity();
                let rotation = transform.rotation();
                let local_omega = rotation
                    .conjugate()
                    .rotate_vector(omega)
                    .expect("local angular velocity");
                let local_i_omega = mass.inertia() * local_omega;
                let world_i_omega = rotation
                    .rotate_vector(local_i_omega)
                    .expect("world angular momentum");
                0.5 * mass.mass() * velocity.length_squared()
                    + 0.5 * omega.dot(world_i_omega)
                    + GRAVITATIONAL_PARAMETER * mass.mass() * (1.0 / SURFACE_RADIUS - 1.0 / radius)
            })
            .sum()
    }
}
