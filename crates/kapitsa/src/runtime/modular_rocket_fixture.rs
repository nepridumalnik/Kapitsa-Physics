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
            // Root cuboids have a full height of 0.5 m. Place the lower
            // face on the pad instead of starting the rocket in free fall.
            let root_position = Vec3::new(0.0, if stage == 0 { 0.25 } else { 1.25 }, 0.0);
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
    fn lower_root_starts_on_the_pad() {
        let rocket = ModularRocket::new(8);
        let root = rocket.world.body(rocket.lower_root).unwrap();
        let collider = rocket
            .world
            .colliders()
            .find(|(_, body, _)| *body == rocket.lower_root)
            .map(|(_, _, collider)| collider)
            .unwrap();
        let kapitsa_core::shape::Shape::Cuboid(cuboid) = collider.shape() else {
            panic!("root collider must be a cuboid");
        };
        assert_eq!(collider.local_transform().translation().y, 0.0);
        assert_eq!(
            root.state().transform().translation().y,
            cuboid.half_extents().y
        );
    }

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
mod time_warp_tests {
    use std::{
        error::Error,
        num::NonZeroUsize,
        time::{Duration, Instant},
    };

    use kapitsa_core::transform::Transform;
    use kapitsa_math::{Quat, Vec3};

    use crate::runtime::{
        AdaptiveTimeStepConfig, Simulation, SimulationConfig, TimeStepPolicy, WorkerConfig,
    };

    use super::ModularRocket;

    const SAMPLE: Duration = Duration::from_millis(100);
    const FIXED_STEP: Duration = Duration::from_nanos(8_333_333);
    const ALTITUDE: f64 = 2_000.0;

    #[derive(Debug, PartialEq)]
    struct PartState {
        position: Vec3,
        rotation: Quat,
        velocity: Vec3,
    }

    struct WarpSample {
        requested: f64,
        actual: f64,
        elapsed_wall: Duration,
        simulated_time: Duration,
        max_time_debt: Duration,
        ending_time_debt: Duration,
        steps: u64,
        average_step_cost: Duration,
        max_step_cost: Duration,
        adaptive_limited: bool,
        states: Vec<PartState>,
    }

    #[test]
    #[ignore = "clock-driven Time Warp study; run explicitly in release"]
    fn adaptive_time_warp_reports_speed_debt_and_trajectory_accuracy() -> Result<(), Box<dyn Error>>
    {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(4)
            .enable_all()
            .build()?;
        for part_count in [100, 200, 500, 1000] {
            for warp in [1.0, 10.0, 100.0] {
                let sample = runtime.block_on(run_warp_sample(part_count, warp))?;
                report_accuracy(part_count, sample);
            }
        }
        let safe_warp = runtime.block_on(run_warp_sample(100, 1_000.0))?;
        report_accuracy(100, safe_warp);
        Ok(())
    }

    async fn run_warp_sample(
        part_count: usize,
        requested_warp: f64,
    ) -> Result<WarpSample, Box<dyn Error>> {
        let rocket = free_flight_rocket(part_count)?;
        let config = SimulationConfig {
            tick_rate: 120,
            max_catch_up_steps: 8,
            time_step_policy: TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default()),
            workers: WorkerConfig {
                max_workers: NonZeroUsize::new(1),
                ..WorkerConfig::default()
            },
            ..SimulationConfig::default()
        };
        let mut simulation = Simulation::new_with_world(rocket.world, config)?;
        let client = simulation.client();
        simulation.start()?;
        simulation.set_time_multiplier(requested_warp)?;
        let initial_time = client.snapshot().simulation_time();
        let initial_metrics = simulation.metrics();
        let started = Instant::now();
        let deadline = started + SAMPLE;
        let mut max_time_debt = Duration::ZERO;
        let mut max_step_cost = Duration::ZERO;
        let mut observed_steps = initial_metrics.total_steps;
        let mut window = tokio::time::interval(Duration::from_millis(1));
        window.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        while Instant::now() < deadline {
            window.tick().await;
            let metrics = simulation.metrics();
            max_time_debt = max_time_debt.max(metrics.simulation_time_debt);
            if metrics.total_steps != observed_steps {
                observed_steps = metrics.total_steps;
                max_step_cost = max_step_cost.max(metrics.last_step_duration);
            }
        }
        let elapsed_wall = started.elapsed();
        let sampled_metrics = simulation.metrics();
        let sample_snapshot = client.snapshot();
        let simulated_time = sample_snapshot
            .simulation_time()
            .saturating_sub(initial_time);
        max_time_debt = max_time_debt.max(sampled_metrics.simulation_time_debt);
        simulation.set_time_multiplier(0.0)?;
        simulation.stop().await?;
        let final_snapshot = client.snapshot();
        let states = rocket
            .parts
            .iter()
            .map(|&id| {
                let state = final_snapshot
                    .body(id)
                    .ok_or_else(|| std::io::Error::other("runtime snapshot lost a rocket part"))?;
                Ok(PartState {
                    position: state.transform().translation(),
                    rotation: state.transform().rotation(),
                    velocity: state.linear_velocity(),
                })
            })
            .collect::<Result<Vec<_>, std::io::Error>>()?;
        let actual = simulated_time.as_secs_f64() / elapsed_wall.as_secs_f64();
        Ok(WarpSample {
            requested: requested_warp,
            actual,
            elapsed_wall,
            simulated_time,
            max_time_debt,
            ending_time_debt: sampled_metrics.simulation_time_debt,
            steps: sampled_metrics
                .total_steps
                .saturating_sub(initial_metrics.total_steps),
            average_step_cost: sampled_metrics.average_step_duration,
            max_step_cost,
            adaptive_limited: sampled_metrics.adaptive_step_limited,
            states,
        })
    }

    fn free_flight_rocket(part_count: usize) -> Result<ModularRocket, Box<dyn Error>> {
        let mut rocket = ModularRocket::new(part_count);
        for &id in &rocket.parts {
            let body = rocket.world.body(id).ok_or("rocket body is missing")?;
            let transform = body.state().transform();
            let elevated = Transform::new(
                transform.translation() + Vec3::Y * ALTITUDE,
                transform.rotation(),
            )?;
            rocket
                .world
                .body_mut(id)
                .ok_or("rocket body is missing")?
                .set_transform(elevated)?;
        }
        Ok(rocket)
    }

    fn fixed_reference(part_count: usize, simulated_time: Duration) -> Vec<PartState> {
        let mut rocket = free_flight_rocket(part_count).expect("reference rocket");
        let mut elapsed = Duration::ZERO;
        while elapsed < simulated_time {
            let step = FIXED_STEP.min(simulated_time.saturating_sub(elapsed));
            rocket
                .world
                .step(step.as_secs_f64())
                .expect("fine fixed reference step");
            elapsed += step;
        }
        rocket
            .parts
            .iter()
            .map(|&id| {
                let body = rocket.world.body(id).expect("reference part");
                PartState {
                    position: body.state().transform().translation(),
                    rotation: body.state().transform().rotation(),
                    velocity: body.state().linear_velocity(),
                }
            })
            .collect()
    }

    fn report_accuracy(part_count: usize, sample: WarpSample) {
        let reference = fixed_reference(part_count, sample.simulated_time);
        let (max_position_error, max_velocity_error) = sample.states.iter().zip(&reference).fold(
            (0.0_f64, 0.0_f64),
            |(position, velocity), (actual, expected)| {
                (
                    position.max((actual.position - expected.position).length()),
                    velocity.max((actual.velocity - expected.velocity).length()),
                )
            },
        );
        let max_frame_error = sample
            .states
            .iter()
            .zip(&reference)
            .map(|(actual, expected)| {
                let dot = (actual.rotation.x * expected.rotation.x
                    + actual.rotation.y * expected.rotation.y
                    + actual.rotation.z * expected.rotation.z
                    + actual.rotation.w * expected.rotation.w)
                    .abs()
                    .min(1.0);
                2.0 * dot.acos()
            })
            .fold(0.0_f64, f64::max);
        let finite = sample.states.iter().all(|state| {
            state.position.is_finite()
                && state.rotation.is_normalized()
                && state.velocity.is_finite()
        });
        let accuracy = if max_position_error < 0.5 && max_velocity_error < 2.5 {
            "PASS"
        } else {
            "FAIL"
        };
        println!(
            "warp parts={part_count} requested={}x achieved={:.2}x sim_ms={} wall_ms={} steps={} avg/max_step_ms={:.3}/{:.3} max_debt_ms={} end_debt_ms={} adaptive_limited={} finite={} accuracy={} max_error_m/m_s/rad={:.6e}/{:.6e}/{:.6e}",
            sample.requested,
            sample.actual,
            sample.simulated_time.as_secs_f64() * 1.0e3,
            sample.elapsed_wall.as_secs_f64() * 1.0e3,
            sample.steps,
            sample.average_step_cost.as_secs_f64() * 1.0e3,
            sample.max_step_cost.as_secs_f64() * 1.0e3,
            sample.max_time_debt.as_secs_f64() * 1.0e3,
            sample.ending_time_debt.as_secs_f64() * 1.0e3,
            sample.adaptive_limited,
            finite,
            accuracy,
            max_position_error,
            max_velocity_error,
            max_frame_error,
        );
        assert!(
            finite,
            "Time Warp produced a non-finite state for {part_count} parts"
        );
    }
}
