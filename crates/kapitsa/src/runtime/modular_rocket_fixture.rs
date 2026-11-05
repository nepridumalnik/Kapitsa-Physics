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

#[cfg(test)]
mod stability_tests {
    use std::time::{Duration, Instant};

    use kapitsa_core::collision::CollisionEventKind;
    #[cfg(feature = "bench-internals")]
    use kapitsa_core::world::PhysicsWorld;
    #[cfg(feature = "bench-internals")]
    use kapitsa_core::{mass::MassProperties, rigid_body::RigidBody, transform::Transform};
    use kapitsa_math::Vec3;

    use super::{ModularRocket, rocket_part_mass};

    const DT: f64 = 1.0 / 120.0;
    const TICKS: usize = 180;

    #[cfg(feature = "bench-internals")]
    mod experiment_executor {
        include!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../kapitsa-core/benches/support/rayon_executor.rs"
        ));
    }

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
        max_coast_energy_drift_tick: Option<usize>,
        final_coast_energy_signed_drift: f64,
        #[cfg(feature = "bench-internals")]
        legacy_surface_reference_max_drift: f64,
        coast_energy_start_components: (f64, f64, f64),
        coast_energy_reference_scale_j: f64,
        coast_energy_final_components: (f64, f64, f64),
        max_altitude: f64,
        upper_root_final_velocity: Vec3,
        max_angular_speed: f64,
        separation_momentum_error: f64,
        surface_begins: usize,
        surface_ends: usize,
        surface_impacts: usize,
        other_contact_events: usize,
        post_cutoff_surface_events: usize,
        post_cutoff_upper_surface_events: usize,
        post_cutoff_upper_surface_event_ticks: Vec<usize>,
        coast_energy_trace: Vec<(usize, f64)>,
        lower_fuel_mass: f64,
        upper_fuel_mass: f64,
        joints_after_separation: usize,
        joint_break_events: usize,
        elapsed: Duration,
        completed_ticks: usize,
        step_failure: Option<String>,
        first_anchor_limit_tick: Option<usize>,
        first_frame_limit_tick: Option<usize>,
        velocity_pcg_converged_calls: u64,
        velocity_pcg_iteration_limit_calls: u64,
        velocity_pcg_iterations: u64,
        position_pcg_iterations: u64,
        velocity_pcg_linear_residual_mps: f64,
        velocity_pcg_angular_residual_radps: f64,
        position_pcg_converged_calls: u64,
        position_pcg_iteration_limit_calls: u64,
        position_pcg_linear_residual_m: f64,
        position_pcg_angular_residual_rad: f64,
        #[cfg(feature = "bench-internals")]
        solver_stage_trace: Vec<SolverStageSample>,
        #[cfg(feature = "bench-internals")]
        energy_trace: Vec<kapitsa_core::joint::diagnostics::JointSolverMetrics>,
    }

    #[derive(Default)]
    struct PcgSummary {
        velocity_converged_calls: u64,
        velocity_iteration_limit_calls: u64,
        velocity_linear_residual_mps: f64,
        velocity_angular_residual_radps: f64,
        velocity_iterations: u64,
        position_iterations: u64,
        position_converged_calls: u64,
        position_iteration_limit_calls: u64,
        position_linear_residual_m: f64,
        position_angular_residual_rad: f64,
    }

    #[cfg(feature = "bench-internals")]
    #[derive(Clone, Copy, Debug)]
    struct SolverStageSample {
        tick: usize,
        anchor_error_m: f64,
        frame_error_rad: f64,
        pre_velocity_anchor_error_m: f64,
        pre_velocity_frame_error_rad: f64,
        post_velocity_anchor_error_m: f64,
        post_velocity_frame_error_rad: f64,
        pre_joint_anchor_error_m: f64,
        pre_joint_frame_error_rad: f64,
        post_joint_anchor_error_m: f64,
        post_joint_frame_error_rad: f64,
        post_velocity_linear_residual_mps: f64,
        post_velocity_angular_residual_radps: f64,
        post_position_linear_residual_mps: f64,
        post_position_angular_residual_radps: f64,
        first_velocity_linear_residual_mps: f64,
        first_velocity_angular_residual_radps: f64,
        first_post_joint_anchor_error_m: f64,
        first_post_joint_frame_error_rad: f64,
        pre_joint_velocity_linear_residual_mps: f64,
        pre_joint_velocity_angular_residual_radps: f64,
        velocity_pcg_iterations: u64,
        position_pcg_iterations: u64,
    }

    #[allow(dead_code)] // Non-default source isolation modes are selected by an ignored diagnostic.
    #[derive(Clone, Copy, Debug)]
    enum ScenarioMode {
        Full,
        NoLoads,
        GravityWithoutContact,
        GravityWithContact,
        ThrustWithoutGravityOrContact,
        ThrustWithGravityWithoutContact,
        CenteredThrustWithoutGravityOrContact,
        StagedThrustWithoutGravityOrContact,
        StagedThrustWithGravityWithoutContact,
        StagedThrustWithGravityAndBodyColliders,
        RocketCoastWithoutContactOrThrust,
        RocketFreeAssemblyWithoutLoads,
    }

    #[test]
    #[ignore = "full 1/120 s modular rocket scale and repeatability test; run explicitly in release"]
    fn fixed_step_rockets_report_stability_and_reproducibility_at_all_sizes() {
        // Reporting is opt-in; ordinary verification must fail on violated limits.
        let report_only = std::env::var_os("KAPITSA_ROCKET_REPORT_ONLY").is_some();
        let velocity_pcg_cap = std::env::var("KAPITSA_ROCKET_VELOCITY_PCG_CAP")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid PCG iteration cap"));
        let position_pcg_cap = std::env::var("KAPITSA_ROCKET_POSITION_PCG_CAP")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid PCG iteration cap"));
        let outer_velocity_sweeps = std::env::var("KAPITSA_ROCKET_OUTER_VELOCITY_SWEEPS")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid velocity sweep count"));
        let velocity_pcg_preconditioner =
            std::env::var("KAPITSA_ROCKET_VELOCITY_PCG_PRECONDITIONER")
                .unwrap_or_else(|_| "ic0".to_owned());
        let mut all_stable = true;
        for part_count in [100, 200, 500, 1000] {
            let first = run_fixed_scenario_with_pcg(
                part_count,
                velocity_pcg_cap,
                position_pcg_cap,
                outer_velocity_sweeps,
            );
            let repeated = run_fixed_scenario_with_pcg(
                part_count,
                velocity_pcg_cap,
                position_pcg_cap,
                outer_velocity_sweeps,
            );
            assert_eq!(first.states, repeated.states, "{part_count} part repeat");
            assert_eq!(first.completed_ticks, repeated.completed_ticks);
            assert_eq!(first.step_failure, repeated.step_failure);
            let scenario_complete = first.step_failure.is_none()
                && first.completed_ticks == TICKS
                && first.max_altitude > 1.6
                && first.max_angular_speed > 0.01
                && first.separation_momentum_error < 1.0e-9
                && first.surface_begins > 0
                && first.lower_fuel_mass == 3.5
                && first.upper_fuel_mass == 3.5
                && first.joints_after_separation == part_count - 2
                && first.joint_break_events == 0
                && !first.coast_energy_trace.is_empty();
            let numerical_stability = scenario_complete
                && first.max_anchor_error < 0.03
                && first.rms_anchor_error < 0.01
                && first.max_frame_error < 0.1
                && first.rms_frame_error < 0.03
                && first.max_coast_energy_drift < 0.1;
            all_stable &= numerical_stability;
            println!(
                "parts={part_count} solver={} outer_velocity_sweeps={outer_velocity_sweeps:?} joints={} elapsed_ms={} completed_ticks={} scenario={} numerical_stability={} step_failure={:?} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad first_limit_ticks(anchor/frame)={:?}/{:?} collision_free_coast_drift={:+.6e}% energy_reference_scale={:.6e}J coast_samples={} upper_coast_contacts={} first_upper_coast_contact={:?} coast_energy_start(T/R/U)={:.6e}/{:.6e}/{:.6e}J final(T/R/U)={:.6e}/{:.6e}/{:.6e}J upper_root_max_y={:.3}m omega={:.3}rad_s separation_dp={:.3e} contacts(begin/end/impact)={}/{}/{} breaks={} velocity_pcg_converged/limit={}/{} position_pcg_converged/limit={}/{} velocity/position_pcg_iterations={}/{} velocity_residual_peak={:.6e}m/s/{:.6e}rad/s position_residual_peak={:.6e}m/{:.6e}rad",
                match (velocity_pcg_cap, position_pcg_cap) {
                    (None, None) => "scalar".to_owned(),
                    (velocity, position) => format!(
                        "global_pcg_{}_v{velocity:?}_p{position:?}",
                        velocity_pcg_preconditioner
                    ),
                },
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
                first.first_anchor_limit_tick,
                first.first_frame_limit_tick,
                first.final_coast_energy_signed_drift * 100.0,
                first.coast_energy_reference_scale_j,
                first.coast_energy_trace.len(),
                first.post_cutoff_upper_surface_events,
                first.post_cutoff_upper_surface_event_ticks.first(),
                first.coast_energy_start_components.0,
                first.coast_energy_start_components.1,
                first.coast_energy_start_components.2,
                first.coast_energy_final_components.0,
                first.coast_energy_final_components.1,
                first.coast_energy_final_components.2,
                first.max_altitude,
                first.max_angular_speed,
                first.separation_momentum_error,
                first.surface_begins,
                first.surface_ends,
                first.surface_impacts,
                first.joint_break_events,
                first.velocity_pcg_converged_calls,
                first.velocity_pcg_iteration_limit_calls,
                first.position_pcg_converged_calls,
                first.position_pcg_iteration_limit_calls,
                first.velocity_pcg_iterations,
                first.position_pcg_iterations,
                first.velocity_pcg_linear_residual_mps,
                first.velocity_pcg_angular_residual_radps,
                first.position_pcg_linear_residual_m,
                first.position_pcg_angular_residual_rad,
            );
        }
        assert!(
            report_only || all_stable,
            "rocket stability criteria failed"
        );
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "one-step position convergence from identical rocket poses"]
    fn diagnose_fixed_block_iterations_from_same_pose() {
        let workers = std::env::var("KAPITSA_PARTIAL_WORKERS")
            .ok()
            .and_then(|s| s.parse::<usize>().ok());
        let executor =
            workers.map(|count| experiment_executor::RayonExecutor::new(count).expect("rayon"));
        let mut initial_errors = None;
        for position_iterations in 1..=12 {
            let mut rocket = ModularRocket::new(1000);
            rocket.world.set_joint_projection_for_bench(true);
            rocket
                .world
                .set_partial_coloring_for_bench(workers.is_some());
            for _ in 0..12 {
                if let (Some(executor), Some(workers)) = (&executor, workers) {
                    rocket
                        .world
                        .step_with_executor(DT, executor, workers)
                        .expect("warm step");
                } else {
                    rocket.world.step(DT).expect("warm step");
                }
            }
            let before = joint_errors(&rocket)
                .into_iter()
                .map(|(_, a, b)| (a, b))
                .collect::<Vec<_>>();
            if let Some(ref expected) = initial_errors {
                assert_eq!(&before, expected, "initial joint errors changed");
            } else {
                initial_errors = Some(before);
            }
            let mut config = rocket.world.collision_config();
            config.velocity_iterations = 8;
            config.position_iterations = position_iterations;
            rocket
                .world
                .set_collision_config(config)
                .expect("iterations");
            rocket
                .world
                .set_force_source_enabled(rocket.lower_engine, true)
                .expect("ignition");
            if let (Some(executor), Some(workers)) = (&executor, workers) {
                rocket
                    .world
                    .step_with_executor(DT, executor, workers)
                    .expect("measured step");
            } else {
                rocket.world.step(DT).expect("measured step");
            }
            let errors = joint_errors(&rocket);
            let linear = errors.iter().map(|entry| entry.1).fold(0.0_f64, f64::max);
            let angular = errors.iter().map(|entry| entry.2).fold(0.0_f64, f64::max);
            let metrics = rocket.world.physics_step_metrics().joints;
            println!(
                "same_pose workers={workers:?} position={position_iterations} anchor={linear:.9e} frame={angular:.9e} velocity={:.9e}/{:.9e}",
                metrics.post_velocity_linear_residual_mps,
                metrics.post_velocity_angular_residual_radps
            );
            assert!(linear.is_finite() && angular.is_finite());
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "compare rocket stability with and without the solver mass cache"]
    fn diagnose_fixed_block_rocket_mass_cache() {
        let run = run_fixed_scenario(1000, true);
        println!(
            "mass_cache={} ticks={} anchor={:.9e}/{:.9e} frame={:.9e}/{:.9e} first={:?}/{:?} failure={:?}",
            std::env::var_os("KAPITSA_ROCKET_NO_MASS_CACHE").is_none(),
            run.completed_ticks,
            run.max_anchor_error,
            run.rms_anchor_error,
            run.max_frame_error,
            run.rms_frame_error,
            run.first_anchor_limit_tick,
            run.first_frame_limit_tick,
            run.step_failure,
        );
        assert_eq!(run.completed_ticks, TICKS);
        assert!(run.step_failure.is_none());
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "trace initial divergence in the 1000-part loaded rocket"]
    fn diagnose_rocket_first_fixed_joint_divergence() {
        for block3 in [false, true] {
            let run = run_scenario(
                1000,
                block3,
                ScenarioMode::Full,
                false,
                None,
                false,
                None,
                None,
                None,
            );
            println!(
                "projection={} ticks={} first_anchor={:?} first_frame={:?} max_anchor/rms={:.9e}/{:.9e} max_frame/rms={:.9e}/{:.9e} coast_drift={:.9e} separation_dp={:.9e} failure={:?}",
                if block3 { "FixedBlock3" } else { "Scalar" },
                run.completed_ticks,
                run.first_anchor_limit_tick,
                run.first_frame_limit_tick,
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_coast_energy_drift,
                run.separation_momentum_error,
                run.step_failure
            );
            for sample in run.solver_stage_trace.iter().filter(|s| {
                s.tick <= 20
                    || Some(s.tick) == run.first_anchor_limit_tick
                    || Some(s.tick) == run.first_frame_limit_tick
            }) {
                println!(
                    "projection={} tick={} anchor/frame={:.9e}/{:.9e} pre_velocity={:.9e}/{:.9e} post_velocity={:.9e}/{:.9e} pre_position={:.9e}/{:.9e} first_position={:.9e}/{:.9e} final_position={:.9e}/{:.9e} velocity_first={:.9e}/{:.9e} velocity_final={:.9e}/{:.9e} velocity_after_position={:.9e}/{:.9e}",
                    if block3 { "FixedBlock3" } else { "Scalar" },
                    sample.tick,
                    sample.anchor_error_m,
                    sample.frame_error_rad,
                    sample.pre_velocity_anchor_error_m,
                    sample.pre_velocity_frame_error_rad,
                    sample.post_velocity_anchor_error_m,
                    sample.post_velocity_frame_error_rad,
                    sample.pre_joint_anchor_error_m,
                    sample.pre_joint_frame_error_rad,
                    sample.first_post_joint_anchor_error_m,
                    sample.first_post_joint_frame_error_rad,
                    sample.post_joint_anchor_error_m,
                    sample.post_joint_frame_error_rad,
                    sample.first_velocity_linear_residual_mps,
                    sample.first_velocity_angular_residual_radps,
                    sample.post_velocity_linear_residual_mps,
                    sample.post_velocity_angular_residual_radps,
                    sample.post_position_linear_residual_mps,
                    sample.post_position_angular_residual_radps
                );
            }
            assert_eq!(run.completed_ticks, TICKS);
            assert!(run.step_failure.is_none());
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "diagnostic FixedBlock3 sweep matrix for the loaded 1000-part rocket"]
    fn diagnose_fixed_block_rocket_sweep_matrix() {
        for velocity in [8, 16, 32] {
            for position in [3, 6, 12] {
                let run = run_scenario(
                    1000,
                    true,
                    ScenarioMode::Full,
                    false,
                    Some((velocity, position)),
                    false,
                    None,
                    None,
                    None,
                );
                let first = run
                    .solver_stage_trace
                    .iter()
                    .find(|s| Some(s.tick) == run.first_anchor_limit_tick);
                println!(
                    "velocity={velocity} position={position} ticks={} anchor_max/rms={:.9e}/{:.9e} frame_max/rms={:.9e}/{:.9e} first_anchor={:?} first_frame={:?} crossing_pre/post_position={:.9e}/{:.9e} crossing_velocity_first/final={:.9e}/{:.9e} energy_drift={:.9e} separation_dp={:.9e} failure={:?}",
                    run.completed_ticks,
                    run.max_anchor_error,
                    run.rms_anchor_error,
                    run.max_frame_error,
                    run.rms_frame_error,
                    run.first_anchor_limit_tick,
                    run.first_frame_limit_tick,
                    first.map_or(0.0, |s| s.pre_joint_anchor_error_m),
                    first.map_or(0.0, |s| s.post_joint_anchor_error_m),
                    first.map_or(0.0, |s| s.first_velocity_linear_residual_mps),
                    first.map_or(0.0, |s| s.post_velocity_linear_residual_mps),
                    run.max_coast_energy_drift,
                    run.separation_momentum_error,
                    run.step_failure
                );
                assert_eq!(run.completed_ticks, TICKS);
                assert!(run.step_failure.is_none());
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "compare centered and offset thrust in the same 1000-part FixedBlock3 fixture"]
    fn diagnose_fixed_block_centered_and_offset_thrust() {
        for mode in [
            ScenarioMode::CenteredThrustWithoutGravityOrContact,
            ScenarioMode::ThrustWithoutGravityOrContact,
        ] {
            let run = run_scenario(1000, true, mode, false, None, false, None, None, None);
            println!(
                "mode={mode:?} ticks={} first_anchor={:?} first_frame={:?} anchor_max/rms={:.9e}/{:.9e} frame_max/rms={:.9e}/{:.9e} coast_drift={:.9e} failure={:?}",
                run.completed_ticks,
                run.first_anchor_limit_tick,
                run.first_frame_limit_tick,
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_coast_energy_drift,
                run.step_failure
            );
            assert_eq!(run.completed_ticks, TICKS);
            assert!(run.step_failure.is_none());
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "diagnostic FixedBlock3 loading and iteration convergence"]
    fn diagnose_fixed_block_rocket_loading_and_convergence() {
        for (parts, mode, position) in [
            (1000, ScenarioMode::NoLoads, 3),
            (1000, ScenarioMode::GravityWithoutContact, 3),
            (1000, ScenarioMode::GravityWithContact, 3),
            (1000, ScenarioMode::ThrustWithoutGravityOrContact, 3),
            (1000, ScenarioMode::Full, 3),
            (1000, ScenarioMode::Full, 6),
            (1000, ScenarioMode::Full, 12),
        ] {
            let run = run_scenario(
                parts,
                true,
                mode,
                false,
                Some((8, position)),
                false,
                None,
                None,
                None,
            );
            println!(
                "parts={parts} mode={mode:?} velocity=8 position={position} ticks={} first_anchor={:?} first_frame={:?} max_anchor={:.8e} rms_anchor={:.8e} max_frame={:.8e} rms_frame={:.8e} drift={:.8e} separation_dp={:.8e} joints={} failure={:?}",
                run.completed_ticks,
                run.first_anchor_limit_tick,
                run.first_frame_limit_tick,
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_coast_energy_drift,
                run.separation_momentum_error,
                run.joints_after_separation,
                run.step_failure
            );
            for sample in run.solver_stage_trace.iter().filter(|sample| {
                [0, 10, 11, 12, 13, 53, 54, 55, 89, 90, 179].contains(&sample.tick)
                    || Some(sample.tick) == run.first_anchor_limit_tick
                    || Some(sample.tick) == run.first_frame_limit_tick
            }) {
                println!(
                    "stage {mode:?} p={position} tick={} anchor={:.6e} frame={:.6e} pre_velocity={:.6e}/{:.6e} post_velocity={:.6e}/{:.6e} pre_position={:.6e}/{:.6e} first_position={:.6e}/{:.6e} final_position={:.6e}/{:.6e} velocity_residual={:.6e}/{:.6e}",
                    sample.tick,
                    sample.anchor_error_m,
                    sample.frame_error_rad,
                    sample.pre_velocity_anchor_error_m,
                    sample.pre_velocity_frame_error_rad,
                    sample.post_velocity_anchor_error_m,
                    sample.post_velocity_frame_error_rad,
                    sample.pre_joint_anchor_error_m,
                    sample.pre_joint_frame_error_rad,
                    sample.first_post_joint_anchor_error_m,
                    sample.first_post_joint_frame_error_rad,
                    sample.post_joint_anchor_error_m,
                    sample.post_joint_frame_error_rad,
                    sample.post_velocity_linear_residual_mps,
                    sample.post_velocity_angular_residual_radps
                );
            }
            assert!(run.step_failure.is_none());
            assert_eq!(run.completed_ticks, TICKS);
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "300-step FixedBlock3 partial coloring stability comparison"]
    fn partial_coloring_rocket_stability() {
        let first = run_fixed_scenario(1000, true);
        let second = run_fixed_scenario(1000, true);
        assert_eq!(first.states, second.states, "repeat must be deterministic");
        let velocity = first
            .solver_stage_trace
            .iter()
            .map(|s| s.post_velocity_linear_residual_mps)
            .fold(0.0_f64, f64::max);
        let angular_velocity = first
            .solver_stage_trace
            .iter()
            .map(|s| s.post_velocity_angular_residual_radps)
            .fold(0.0_f64, f64::max);
        println!(
            "ticks={} error={:?} anchor={:.6e}/{:.6e} frame={:.6e}/{:.6e} energy_drift={:.6e} velocity={:.6e}/{:.6e} separation_dp={:.6e} joints={} breaks={}",
            first.completed_ticks,
            first.step_failure,
            first.max_anchor_error,
            first.rms_anchor_error,
            first.max_frame_error,
            first.rms_frame_error,
            first.max_coast_energy_drift,
            velocity,
            angular_velocity,
            first.separation_momentum_error,
            first.joints_after_separation,
            first.joint_break_events
        );
        assert_eq!(first.completed_ticks, 300);
        assert!(first.step_failure.is_none());
        assert_eq!(first.joints_after_separation, 998);
        assert_eq!(first.joint_break_events, 0);
        assert!(first.separation_momentum_error < 1.0e-9);
        assert!(first.max_anchor_error < 0.03 && first.rms_anchor_error < 0.01);
        assert!(first.max_frame_error < 0.1 && first.rms_frame_error < 0.03);
        assert!(first.max_coast_energy_drift < 0.1);
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release scalar versus FixedBlock3 comparison using the strict rocket fixture"]
    fn compare_scalar_and_fixed_block3_rocket_stability() {
        for part_count in [100, 200, 500, 1000] {
            for block3 in [false, true] {
                let run = run_fixed_scenario(part_count, block3);
                println!(
                    "parts={part_count} projection={} elapsed_ms={} ticks={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad coast_energy_drift={:.4}% contacts(begin/end/impact)={}/{}/{} separation_dp={:.3e} failure={:?}",
                    if block3 { "FixedBlock3" } else { "Scalar" },
                    run.elapsed.as_millis(),
                    run.completed_ticks,
                    run.max_anchor_error,
                    run.rms_anchor_error,
                    run.max_frame_error,
                    run.rms_frame_error,
                    run.max_coast_energy_drift * 100.0,
                    run.surface_begins,
                    run.surface_ends,
                    run.surface_impacts,
                    run.separation_momentum_error,
                    run.step_failure,
                );
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "diagnostic source isolation for modular rocket joint drift"]
    fn diagnose_rocket_joint_error_sources() {
        for (mode, adaptive) in [
            (ScenarioMode::NoLoads, false),
            (ScenarioMode::GravityWithoutContact, false),
            (ScenarioMode::GravityWithContact, false),
            (ScenarioMode::GravityWithContact, true),
            (ScenarioMode::ThrustWithoutGravityOrContact, false),
            (ScenarioMode::ThrustWithoutGravityOrContact, true),
            (ScenarioMode::CenteredThrustWithoutGravityOrContact, false),
            (ScenarioMode::Full, false),
        ] {
            let run = run_scenario(100, false, mode, adaptive, None, false, None, None, None);
            println!(
                "mode={mode:?} adaptive={adaptive} elapsed_ms={} ticks={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad first_limit_ticks(anchor/frame)={:?}/{:?} coast_energy_drift={:.4}% contacts(begin/end/impact)={}/{}/{} separation_dp={:.3e} failure={:?}",
                run.elapsed.as_millis(),
                run.completed_ticks,
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.first_anchor_limit_tick,
                run.first_frame_limit_tick,
                run.max_coast_energy_drift * 100.0,
                run.surface_begins,
                run.surface_ends,
                run.surface_impacts,
                run.separation_momentum_error,
                run.step_failure,
            );
            if matches!(mode, ScenarioMode::Full) && !adaptive {
                for sample in run
                    .solver_stage_trace
                    .iter()
                    .filter(|sample| (10..=16).contains(&sample.tick))
                {
                    println!(
                        "solver_stage tick={} actual={:.6e}m/{:.6e}rad pre_velocity={:.6e}m/{:.6e}rad post_velocity={:.6e}m/{:.6e}rad after_contact_before_joint={:.6e}m/{:.6e}rad post_joint_first/final={:.6e}m/{:.6e}rad -> {:.6e}m/{:.6e}rad contact_velocity_residual={:.6e}m/s/{:.6e}rad/s velocity_residual_first/final={:.6e}m/s/{:.6e}rad/s -> {:.6e}m/s/{:.6e}rad/s after_position={:.6e}m/s/{:.6e}rad/s",
                        sample.tick,
                        sample.anchor_error_m,
                        sample.frame_error_rad,
                        sample.pre_velocity_anchor_error_m,
                        sample.pre_velocity_frame_error_rad,
                        sample.post_velocity_anchor_error_m,
                        sample.post_velocity_frame_error_rad,
                        sample.pre_joint_anchor_error_m,
                        sample.pre_joint_frame_error_rad,
                        sample.first_post_joint_anchor_error_m,
                        sample.first_post_joint_frame_error_rad,
                        sample.post_joint_anchor_error_m,
                        sample.post_joint_frame_error_rad,
                        sample.pre_joint_velocity_linear_residual_mps,
                        sample.pre_joint_velocity_angular_residual_radps,
                        sample.first_velocity_linear_residual_mps,
                        sample.first_velocity_angular_residual_radps,
                        sample.post_velocity_linear_residual_mps,
                        sample.post_velocity_angular_residual_radps,
                        sample.post_position_linear_residual_mps,
                        sample.post_position_angular_residual_radps,
                    );
                }
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "diagnostic comparison of velocity and position sweep convergence under loads"]
    fn diagnose_rocket_solver_iteration_residuals() {
        for mode in [
            ScenarioMode::ThrustWithoutGravityOrContact,
            ScenarioMode::GravityWithContact,
        ] {
            for (velocity_iterations, position_iterations) in [(8, 3), (16, 3), (8, 6), (16, 6)] {
                let run = run_scenario(
                    100,
                    false,
                    mode,
                    false,
                    Some((velocity_iterations, position_iterations)),
                    false,
                    None,
                    None,
                    None,
                );
                let crossing = run
                    .first_anchor_limit_tick
                    .and_then(|tick| run.solver_stage_trace.get(tick));
                let (
                    tick,
                    before_velocity,
                    after_velocity,
                    first_velocity_residual,
                    velocity_residual,
                    first_joint,
                    after_joint,
                    contact_velocity_residual,
                ) = crossing.map_or((None, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), |sample| {
                    (
                        Some(sample.tick),
                        sample.pre_velocity_anchor_error_m,
                        sample.post_velocity_anchor_error_m,
                        sample.first_velocity_linear_residual_mps,
                        sample.post_velocity_linear_residual_mps,
                        sample.first_post_joint_anchor_error_m,
                        sample.post_joint_anchor_error_m,
                        sample.pre_joint_velocity_linear_residual_mps,
                    )
                });
                println!(
                    "mode={mode:?} iterations={velocity_iterations}v/{position_iterations}p elapsed_ms={} anchor_max/rms={:.6e}/{:.6e}m frame_max={:.6e}rad first_anchor_limit_tick={tick:?} pre/post_velocity={before_velocity:.6e}/{after_velocity:.6e}m contact_velocity_residual={contact_velocity_residual:.6e}m/s velocity_residual_first/final={first_velocity_residual:.6e}/{velocity_residual:.6e}m/s post_joint_first/final={first_joint:.6e}/{after_joint:.6e}m coast_energy_drift={:.4}%",
                    run.elapsed.as_millis(),
                    run.max_anchor_error,
                    run.rms_anchor_error,
                    run.max_frame_error,
                    run.max_coast_energy_drift * 100.0,
                );
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "diagnostic comparison of forward and reverse joint sweep order under loads"]
    fn diagnose_rocket_joint_sweep_order() {
        for mode in [
            ScenarioMode::Full,
            ScenarioMode::ThrustWithoutGravityOrContact,
            ScenarioMode::GravityWithContact,
        ] {
            for reverse_joint_order in [false, true] {
                let run = run_scenario(
                    100,
                    false,
                    mode,
                    false,
                    None,
                    reverse_joint_order,
                    None,
                    None,
                    None,
                );
                let crossing = run
                    .first_anchor_limit_tick
                    .and_then(|tick| run.solver_stage_trace.get(tick));
                let (tick, velocity_residual, contact_residual, position_error) =
                    crossing.map_or((None, 0.0, 0.0, 0.0), |sample| {
                        (
                            Some(sample.tick),
                            sample.post_velocity_linear_residual_mps,
                            sample.pre_joint_velocity_linear_residual_mps,
                            sample.post_joint_anchor_error_m,
                        )
                    });
                println!(
                    "mode={mode:?} joint_order={} elapsed_ms={} anchor_max/rms={:.6e}/{:.6e}m frame_max={:.6e}rad first_anchor_limit_tick={tick:?} contact_velocity_residual={contact_residual:.6e}m/s post_velocity_residual={velocity_residual:.6e}m/s post_joint_anchor={position_error:.6e}m coast_energy_drift={:.4}%",
                    if reverse_joint_order {
                        "reverse"
                    } else {
                        "forward"
                    },
                    run.elapsed.as_millis(),
                    run.max_anchor_error,
                    run.rms_anchor_error,
                    run.max_frame_error,
                    run.max_coast_energy_drift * 100.0,
                );
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release screen for minimum global velocity PCG iterations on the strict rocket"]
    fn diagnose_rocket_global_velocity_pcg_cap_sweep() {
        for max_iterations in [24, 32, 48] {
            for part_count in [100, 200, 500, 1000] {
                let run = run_scenario(
                    part_count,
                    false,
                    ScenarioMode::Full,
                    false,
                    None,
                    false,
                    Some(max_iterations),
                    None,
                    None,
                );
                let crossing = run.first_anchor_limit_tick.and_then(|tick| {
                    run.solver_stage_trace
                        .iter()
                        .find(|sample| sample.tick == tick)
                });
                let peak = run.solver_stage_trace.iter().max_by(|left, right| {
                    left.post_joint_anchor_error_m
                        .total_cmp(&right.post_joint_anchor_error_m)
                });
                println!(
                    "parts={part_count} solver=global_velocity_pcg_ic0 max_iterations={max_iterations} elapsed_ms={} ticks={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad coast_energy_drift={:.4}% first_anchor_limit={:?} crossing_pre/post_velocity={:.6e}/{:.6e}m post_joint={:.6e}m velocity_residual={:.6e}/{:.6e}m/s/{:.6e}/{:.6e}rad/s pcg_iterations={} peak_tick={:?} peak_pre/post_velocity={:.6e}/{:.6e}m peak_after_position={:.6e}m peak_velocity_residual={:.6e}/{:.6e}m/s contacts(begin/end/impact)={}/{}/{} separation_dp={:.3e} failure={:?}",
                    run.elapsed.as_millis(),
                    run.completed_ticks,
                    run.max_anchor_error,
                    run.rms_anchor_error,
                    run.max_frame_error,
                    run.rms_frame_error,
                    run.max_coast_energy_drift * 100.0,
                    run.first_anchor_limit_tick,
                    crossing.map_or(0.0, |sample| sample.pre_velocity_anchor_error_m),
                    crossing.map_or(0.0, |sample| sample.post_velocity_anchor_error_m),
                    crossing.map_or(0.0, |sample| sample.post_joint_anchor_error_m),
                    crossing.map_or(0.0, |sample| sample.first_velocity_linear_residual_mps),
                    crossing.map_or(0.0, |sample| sample.post_velocity_linear_residual_mps),
                    crossing.map_or(0.0, |sample| sample.first_velocity_angular_residual_radps),
                    crossing.map_or(0.0, |sample| sample.post_velocity_angular_residual_radps),
                    crossing.map_or(0, |sample| sample.velocity_pcg_iterations),
                    peak.map(|sample| sample.tick),
                    peak.map_or(0.0, |sample| sample.pre_velocity_anchor_error_m),
                    peak.map_or(0.0, |sample| sample.post_velocity_anchor_error_m),
                    peak.map_or(0.0, |sample| sample.post_joint_anchor_error_m),
                    peak.map_or(0.0, |sample| sample.post_velocity_linear_residual_mps),
                    peak.map_or(0.0, |sample| sample.post_velocity_angular_residual_radps),
                    run.surface_begins,
                    run.surface_ends,
                    run.surface_impacts,
                    run.separation_momentum_error,
                    run.step_failure,
                );
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release check of contact/joint coupling versus outer velocity sweeps"]
    fn diagnose_rocket_pcg_outer_velocity_sweeps() {
        let part_count = std::env::var("KAPITSA_ROCKET_DIAGNOSTIC_PARTS")
            .ok()
            .map(|value| value.parse::<usize>().expect("valid rocket part count"))
            .unwrap_or(1000);
        let velocity_cap = std::env::var("KAPITSA_ROCKET_VELOCITY_PCG_CAP")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid PCG iteration cap"))
            .unwrap_or(16);
        let position_cap = std::env::var("KAPITSA_ROCKET_POSITION_PCG_CAP")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid PCG iteration cap"))
            .unwrap_or(16);
        let position_preconditioner = std::env::var("KAPITSA_ROCKET_POSITION_PCG_PRECONDITIONER")
            .unwrap_or_else(|_| "ic0".to_owned());
        let velocity_sweeps = std::env::var("KAPITSA_ROCKET_OUTER_VELOCITY_SWEEPS")
            .ok()
            .map(|values| {
                values
                    .split(',')
                    .map(|value| {
                        value
                            .parse::<u32>()
                            .expect("valid outer velocity sweep count")
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_else(|| vec![1, 2, 4, 8, 12, 16]);
        for velocity_iterations in velocity_sweeps {
            let run = run_scenario(
                part_count,
                false,
                ScenarioMode::Full,
                false,
                Some((velocity_iterations, 3)),
                false,
                Some(velocity_cap),
                None,
                Some(position_cap),
            );
            let strict_pass = run.step_failure.is_none()
                && run.completed_ticks == TICKS
                && run.max_altitude > 1.6
                && run.separation_momentum_error < 1.0e-9
                && run.surface_begins > 0
                && run.lower_fuel_mass == 3.5
                && run.upper_fuel_mass == 3.5
                && run.joints_after_separation == part_count - 2
                && run.joint_break_events == 0
                && !run.coast_energy_trace.is_empty()
                && run.max_anchor_error < 0.03
                && run.rms_anchor_error < 0.01
                && run.max_frame_error < 0.1
                && run.rms_frame_error < 0.03
                && run.max_coast_energy_drift < 0.1;
            let position_prepare_ms: f64 = run
                .energy_trace
                .iter()
                .map(|metrics| metrics.position_pcg_system_prepare_time.as_secs_f64() * 1000.0)
                .sum();
            let position_operator_ms: f64 = run
                .energy_trace
                .iter()
                .map(|metrics| metrics.position_pcg_matrix_vector_time.as_secs_f64() * 1000.0)
                .sum();
            let position_preconditioner_ms: f64 = run
                .energy_trace
                .iter()
                .map(|metrics| metrics.position_pcg_preconditioner_time.as_secs_f64() * 1000.0)
                .sum();
            let position_reduction_ms: f64 = run
                .energy_trace
                .iter()
                .map(|metrics| metrics.position_pcg_reduction_time.as_secs_f64() * 1000.0)
                .sum();
            println!(
                "parts={part_count} solver=global_pcg position_preconditioner={position_preconditioner} caps={velocity_cap}/{position_cap} outer_velocity_sweeps={velocity_iterations} elapsed_ms={} ticks={} strict={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad max_altitude={:.3}m upper_final_vy={:.6e}m/s velocity_pcg_converged/limit={}/{} position_pcg_converged/limit={}/{} velocity/position_iterations={}/{} position_residual_peak={:.6e}m/{:.6e}rad position_pcg_ms(system/operator/preconditioner/reduction)={:.3}/{:.3}/{:.3}/{:.3} contacts(begin/end/impact)={}/{}/{} separation_dp={:.3e} failure={:?}",
                run.elapsed.as_millis(),
                run.completed_ticks,
                if strict_pass { "PASS" } else { "FAIL" },
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_altitude,
                run.upper_root_final_velocity.y,
                run.velocity_pcg_converged_calls,
                run.velocity_pcg_iteration_limit_calls,
                run.position_pcg_converged_calls,
                run.position_pcg_iteration_limit_calls,
                run.velocity_pcg_iterations,
                run.position_pcg_iterations,
                run.position_pcg_linear_residual_m,
                run.position_pcg_angular_residual_rad,
                position_prepare_ms,
                position_operator_ms,
                position_preconditioner_ms,
                position_reduction_ms,
                run.surface_begins,
                run.surface_ends,
                run.surface_impacts,
                run.separation_momentum_error,
                run.step_failure,
            );
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release screen for the minimum PCG cap with 16 outer velocity sweeps"]
    fn diagnose_rocket_pcg_minimum_caps() {
        for max_iterations in [1, 2, 3] {
            let run = run_scenario(
                1000,
                false,
                ScenarioMode::Full,
                false,
                Some((16, 3)),
                false,
                Some(max_iterations),
                None,
                Some(max_iterations),
            );
            println!(
                "parts=1000 solver=global_pcg caps={max_iterations}/{max_iterations} outer_velocity_sweeps=16 elapsed_ms={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad max_altitude={:.3}m coast_energy_drift={:.4}% pcg_converged/limit={}/{} pcg_iterations={}/{} contacts(begin/end/impact)={}/{}/{} separation_dp={:.3e} failure={:?}",
                run.elapsed.as_millis(),
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_altitude,
                run.max_coast_energy_drift * 100.0,
                run.velocity_pcg_converged_calls,
                run.velocity_pcg_iteration_limit_calls,
                run.velocity_pcg_iterations,
                run.position_pcg_iterations,
                run.surface_begins,
                run.surface_ends,
                run.surface_impacts,
                run.separation_momentum_error,
                run.step_failure,
            );
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release evaluation of relaxed Gauss-Seidel on the strict rocket fixture"]
    fn diagnose_rocket_relaxed_gauss_seidel() {
        for part_count in [100, 200, 500, 1000] {
            let run = run_scenario(
                part_count,
                false,
                ScenarioMode::Full,
                false,
                None,
                false,
                None,
                Some(1.3),
                None,
            );
            println!(
                "parts={part_count} solver=relaxed_gs relaxation=1.30 elapsed_ms={} ticks={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad coast_energy_drift={:.4}% contacts(begin/end/impact)={}/{}/{} separation_dp={:.3e} failure={:?}",
                run.elapsed.as_millis(),
                run.completed_ticks,
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_coast_energy_drift * 100.0,
                run.surface_begins,
                run.surface_ends,
                run.surface_impacts,
                run.separation_momentum_error,
                run.step_failure,
            );
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release measurement of position projection convergence on the 1000-part rocket"]
    fn diagnose_rocket_position_sweeps_at_scale() {
        for position_iterations in [3, 6, 12] {
            let run = run_scenario(
                1000,
                false,
                ScenarioMode::Full,
                false,
                Some((8, position_iterations)),
                false,
                None,
                None,
                None,
            );
            let peak = run.solver_stage_trace.iter().max_by(|left, right| {
                left.post_joint_anchor_error_m
                    .total_cmp(&right.post_joint_anchor_error_m)
            });
            println!(
                "parts=1000 velocity/position_sweeps=8/{position_iterations} elapsed_ms={} ticks={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad coast_energy_drift={:.4}% first_anchor_limit={:?} peak_tick={:?} peak_pre_velocity={:.6e}m peak_post_position={:.6e}m peak_velocity_residual={:.6e}m/s/{:.6e}rad/s failure={:?}",
                run.elapsed.as_millis(),
                run.completed_ticks,
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_coast_energy_drift * 100.0,
                run.first_anchor_limit_tick,
                peak.map(|sample| sample.tick),
                peak.map_or(0.0, |sample| sample.pre_velocity_anchor_error_m),
                peak.map_or(0.0, |sample| sample.post_joint_anchor_error_m),
                peak.map_or(0.0, |sample| sample.post_velocity_linear_residual_mps),
                peak.map_or(0.0, |sample| sample.post_velocity_angular_residual_radps),
                run.step_failure,
            );
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release measurement of combined global velocity and position PCG on rocket sizes"]
    fn diagnose_rocket_global_constraint_pcg() {
        let part_counts = std::env::var("KAPITSA_ROCKET_DIAGNOSTIC_PARTS")
            .ok()
            .map(|value| vec![value.parse::<usize>().expect("valid rocket part count")])
            .unwrap_or_else(|| vec![100, 200, 500, 1000]);
        let velocity_cap = std::env::var("KAPITSA_ROCKET_VELOCITY_PCG_CAP")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid PCG iteration cap"))
            .unwrap_or(64);
        let position_cap = std::env::var("KAPITSA_ROCKET_POSITION_PCG_CAP")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid PCG iteration cap"))
            .unwrap_or(64);
        let velocity_pcg_preconditioner =
            std::env::var("KAPITSA_ROCKET_VELOCITY_PCG_PRECONDITIONER")
                .unwrap_or_else(|_| "ic0".to_owned());
        for part_count in part_counts {
            let run = run_scenario(
                part_count,
                false,
                ScenarioMode::Full,
                false,
                None,
                false,
                Some(velocity_cap),
                None,
                Some(position_cap),
            );
            let peak = run.solver_stage_trace.iter().max_by(|left, right| {
                left.post_joint_anchor_error_m
                    .total_cmp(&right.post_joint_anchor_error_m)
            });
            println!(
                "parts={part_count} solver=global_velocity_position_pcg_{velocity_pcg_preconditioner} caps={velocity_cap}/{position_cap} elapsed_ms={} ticks={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad coast_energy_drift={:.4}% first_anchor_limit={:?} peak_tick={:?} peak_pre_velocity={:.6e}m peak_post_position={:.6e}m peak_velocity_pcg_iterations={} peak_position_pcg_iterations={} total_velocity/position_pcg_iterations={}/{} pcg_converged/iteration_limit={}/{} pcg_residual_peak={:.6e}m/s/{:.6e}rad/s max_altitude={:.3}m failure={:?}",
                run.elapsed.as_millis(),
                run.completed_ticks,
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_coast_energy_drift * 100.0,
                run.first_anchor_limit_tick,
                peak.map(|sample| sample.tick),
                peak.map_or(0.0, |sample| sample.pre_velocity_anchor_error_m),
                peak.map_or(0.0, |sample| sample.post_joint_anchor_error_m),
                peak.map_or(0, |sample| sample.velocity_pcg_iterations),
                peak.map_or(0, |sample| sample.position_pcg_iterations),
                run.velocity_pcg_iterations,
                run.position_pcg_iterations,
                run.velocity_pcg_converged_calls,
                run.velocity_pcg_iteration_limit_calls,
                run.velocity_pcg_linear_residual_mps,
                run.velocity_pcg_angular_residual_radps,
                run.max_altitude,
                run.step_failure,
            );
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release source isolation of joint energy drift under thrust without gravity or contacts"]
    fn diagnose_rocket_global_constraint_pcg_thrust_only() {
        let run = run_scenario(
            500,
            false,
            ScenarioMode::ThrustWithoutGravityOrContact,
            false,
            None,
            false,
            Some(64),
            None,
            Some(64),
        );
        println!(
            "parts=500 mode=thrust_without_gravity_or_contact solver=global_velocity_position_pcg_ic0 elapsed_ms={} ticks={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad coast_energy_drift={:.4}% first_anchor_limit={:?} contacts(begin/end/impact)={}/{}/{} failure={:?}",
            run.elapsed.as_millis(),
            run.completed_ticks,
            run.max_anchor_error,
            run.rms_anchor_error,
            run.max_frame_error,
            run.rms_frame_error,
            run.max_coast_energy_drift * 100.0,
            run.first_anchor_limit_tick,
            run.surface_begins,
            run.surface_ends,
            run.surface_impacts,
            run.step_failure,
        );
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release isolated coast energy check after stage separation"]
    fn diagnose_rocket_global_constraint_pcg_isolated_coast() {
        let part_count = std::env::var("KAPITSA_ROCKET_DIAGNOSTIC_PARTS")
            .ok()
            .map(|value| value.parse::<usize>().expect("valid rocket part count"))
            .unwrap_or(500);
        let velocity_cap = std::env::var("KAPITSA_ROCKET_VELOCITY_PCG_CAP")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid PCG iteration cap"))
            .unwrap_or(64);
        let position_cap = std::env::var("KAPITSA_ROCKET_POSITION_PCG_CAP")
            .ok()
            .map(|value| value.parse::<u32>().expect("valid PCG iteration cap"))
            .unwrap_or(64);
        let selected_mode = std::env::var("KAPITSA_ROCKET_DIAGNOSTIC_MODE").ok();
        for (mode, label) in [
            (
                ScenarioMode::StagedThrustWithoutGravityOrContact,
                "staged_thrust_without_gravity_or_contact",
            ),
            (
                ScenarioMode::StagedThrustWithGravityWithoutContact,
                "staged_thrust_with_gravity_without_contact",
            ),
            (
                ScenarioMode::StagedThrustWithGravityAndBodyColliders,
                "staged_thrust_with_gravity_contacts_and_fuel",
            ),
        ] {
            if selected_mode
                .as_deref()
                .is_some_and(|selected| selected != label)
            {
                continue;
            }
            let run = run_scenario(
                part_count,
                false,
                mode,
                false,
                None,
                false,
                Some(velocity_cap),
                None,
                Some(position_cap),
            );
            println!(
                "parts={part_count} mode={label} solver=global_velocity_position_pcg caps={velocity_cap}/{position_cap} elapsed_ms={} ticks={} joints_after_separation={} anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad max_altitude={:.3}m upper_root_final_vy={:.6e}m/s coast_energy_drift={:.4}% pcg_converged/iteration_limit={}/{} pcg_residual_peak={:.6e}m/s/{:.6e}rad/s surface_contacts(begin/end/impact)={}/{}/{} other_contact_events={} separation_dp={:.3e} failure={:?}",
                run.elapsed.as_millis(),
                run.completed_ticks,
                run.joints_after_separation,
                run.max_anchor_error,
                run.rms_anchor_error,
                run.max_frame_error,
                run.rms_frame_error,
                run.max_altitude,
                run.upper_root_final_velocity.y,
                run.max_coast_energy_drift * 100.0,
                run.velocity_pcg_converged_calls,
                run.velocity_pcg_iteration_limit_calls,
                run.velocity_pcg_linear_residual_mps,
                run.velocity_pcg_angular_residual_radps,
                run.surface_begins,
                run.surface_ends,
                run.surface_impacts,
                run.other_contact_events,
                run.separation_momentum_error,
                run.step_failure,
            );
            assert_eq!(run.completed_ticks, TICKS);
            assert_eq!(run.joints_after_separation, part_count - 2);
            assert_eq!(
                run.surface_begins + run.surface_ends + run.surface_impacts,
                0
            );
            assert!(run.step_failure.is_none());
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release full-scene coast energy trace with global constraint PCG"]
    fn diagnose_rocket_global_constraint_pcg_full_energy_trace() {
        let run = run_scenario(
            500,
            false,
            ScenarioMode::Full,
            false,
            None,
            false,
            Some(64),
            None,
            Some(64),
        );
        println!(
            "parts=500 mode=full solver=global_velocity_position_pcg_ic0 elapsed_ms={} ticks={} physical_energy_drift_max={:.9e}% legacy_surface_reference_drift_max={:.9e}% at_tick={:?} final_signed={:+.9e}% start_energy(T/R/U)={:.6e}/{:.6e}/{:.6e}J final_energy(T/R/U)={:.6e}/{:.6e}/{:.6e}J anchor_max/rms={:.6e}/{:.6e}m frame_max/rms={:.6e}/{:.6e}rad surface_contacts={}/{}/{} post_cutoff_surface_events={} post_cutoff_upper_surface_events={} upper_contact_ticks={:?} other_contact_events={} altitude_max={:.3}m failure={:?}",
            run.elapsed.as_millis(),
            run.completed_ticks,
            run.max_coast_energy_drift * 100.0,
            run.legacy_surface_reference_max_drift * 100.0,
            run.max_coast_energy_drift_tick,
            run.final_coast_energy_signed_drift * 100.0,
            run.coast_energy_start_components.0,
            run.coast_energy_start_components.1,
            run.coast_energy_start_components.2,
            run.coast_energy_final_components.0,
            run.coast_energy_final_components.1,
            run.coast_energy_final_components.2,
            run.max_anchor_error,
            run.rms_anchor_error,
            run.max_frame_error,
            run.rms_frame_error,
            run.surface_begins,
            run.surface_ends,
            run.surface_impacts,
            run.post_cutoff_surface_events,
            run.post_cutoff_upper_surface_events,
            run.post_cutoff_upper_surface_event_ticks,
            run.other_contact_events,
            run.max_altitude,
            run.step_failure,
        );
        let initial = run.coast_energy_trace[0].1;
        for (tick, energy) in run.coast_energy_trace.iter().rev().take(6).rev() {
            println!(
                "free_coast_tick={tick} signed_drift={:+.4}%",
                (energy - initial) / run.coast_energy_reference_scale_j.max(1.0) * 100.0,
            );
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release per-stage energy trace for an unforced contact-free rocket"]
    fn diagnose_rocket_unforced_contact_free_energy_stages() {
        for (label, velocity_pcg, position_pcg) in
            [("scalar", None, None), ("global_pcg", Some(64), Some(64))]
        {
            let run = run_scenario(
                500,
                false,
                ScenarioMode::RocketCoastWithoutContactOrThrust,
                false,
                None,
                false,
                velocity_pcg,
                None,
                position_pcg,
            );
            let zero = kapitsa_core::joint::diagnostics::EnergyComponents::default();
            println!(
                "parts=500 mode=unforced_contact_free_rocket solver={label} ticks={} collision_free_coast_drift={:+.9e}% other_contact_events={} step_failure={:?} coast_energy_start(T/R/U)={:.9e}/{:.9e}/{:.9e}J coast_energy_final(T/R/U)={:.9e}/{:.9e}/{:.9e}J",
                run.completed_ticks,
                run.max_coast_energy_drift * 100.0,
                run.other_contact_events,
                run.step_failure,
                run.coast_energy_start_components.0,
                run.coast_energy_start_components.1,
                run.coast_energy_start_components.2,
                run.coast_energy_final_components.0,
                run.coast_energy_final_components.1,
                run.coast_energy_final_components.2,
            );
            print_energy_delta(
                "sum_180_integration",
                zero,
                sum_energy_deltas(
                    &run.energy_trace,
                    |m| m.energy_before_integration,
                    |m| m.energy_after_integration,
                ),
            );
            print_energy_delta(
                "sum_180_velocity_pcg",
                zero,
                sum_energy_deltas(&run.energy_trace, |_| zero, |m| m.velocity_pcg_energy_delta),
            );
            print_energy_delta(
                "sum_180_velocity_solver",
                zero,
                sum_energy_deltas(
                    &run.energy_trace,
                    |m| m.energy_after_integration,
                    |m| m.energy_after_velocity_solver,
                ),
            );
            print_energy_delta(
                "sum_180_position_pcg",
                zero,
                sum_energy_deltas(&run.energy_trace, |_| zero, |m| m.position_pcg_energy_delta),
            );
            print_energy_delta(
                "sum_180_position_solver",
                zero,
                sum_energy_deltas(
                    &run.energy_trace,
                    |m| m.energy_after_velocity_solver,
                    |m| m.energy_after_position_solver,
                ),
            );
            let first = run.energy_trace.first().expect("first step energy trace");
            let last = run.energy_trace.last().expect("last step energy trace");
            print_energy_delta(
                "step0_integration",
                first.energy_before_integration,
                first.energy_after_integration,
            );
            print_energy_delta(
                "step0_velocity_pcg",
                first.energy_before_velocity_pcg,
                first.energy_after_velocity_pcg,
            );
            print_energy_delta(
                "step0_position_pcg",
                first.energy_before_position_pcg,
                first.energy_after_position_pcg,
            );
            print_energy_delta(
                "step179_integration",
                last.energy_before_integration,
                last.energy_after_integration,
            );
            print_energy_delta(
                "step179_velocity_pcg",
                last.energy_before_velocity_pcg,
                last.energy_after_velocity_pcg,
            );
            print_energy_delta(
                "step179_position_pcg",
                last.energy_before_position_pcg,
                last.energy_after_position_pcg,
            );
            assert_eq!(run.completed_ticks, TICKS);
            assert_eq!(run.other_contact_events, 0);
            assert!(run.step_failure.is_none());
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release energy changes during gravity plus thrust with contacts disabled"]
    fn diagnose_rocket_loaded_contact_free_energy_stages() {
        let run = run_scenario(
            100,
            false,
            ScenarioMode::ThrustWithGravityWithoutContact,
            false,
            None,
            false,
            Some(64),
            None,
            Some(64),
        );
        for tick in [11, 12, 13, 14, 15] {
            let metrics = &run.energy_trace[tick];
            println!("energy_trace_tick={tick}");
            print_energy_delta(
                "integration",
                metrics.energy_before_integration,
                metrics.energy_after_integration,
            );
            print_energy_delta(
                "velocity_pcg",
                metrics.energy_before_velocity_pcg,
                metrics.energy_after_velocity_pcg,
            );
            print_energy_delta(
                "velocity_solver",
                metrics.energy_after_integration,
                metrics.energy_after_velocity_solver,
            );
            print_energy_delta(
                "position_pcg",
                metrics.energy_before_position_pcg,
                metrics.energy_after_position_pcg,
            );
            print_energy_delta(
                "position_solver",
                metrics.energy_after_velocity_solver,
                metrics.energy_after_position_solver,
            );
        }
        println!(
            "parts=100 contacts={} failure={:?} anchor_max={:.6e}m frame_max={:.6e}rad",
            run.surface_begins + run.surface_ends + run.surface_impacts + run.other_contact_events,
            run.step_failure,
            run.max_anchor_error,
            run.max_frame_error,
        );
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    fn global_pcg_free_rigid_rocket_assembly_limits_kinetic_energy_drift() {
        let mut pcg_conserved = false;
        for (label, velocity_pcg, position_pcg) in [
            ("scalar_reference", None, None),
            ("global_pcg", Some(64), Some(64)),
        ] {
            let run = run_scenario(
                100,
                false,
                ScenarioMode::RocketFreeAssemblyWithoutLoads,
                false,
                None,
                false,
                velocity_pcg,
                None,
                position_pcg,
            );
            let first = run.energy_trace.first().expect("first step energy trace");
            let last = run.energy_trace.last().expect("last step energy trace");
            let mut integration_delta = (0.0, 0.0, 0.0);
            let mut velocity_pcg_delta = (0.0, 0.0, 0.0);
            let mut velocity_solver_delta = (0.0, 0.0, 0.0);
            let mut position_pcg_delta = (0.0, 0.0, 0.0);
            let mut position_solver_delta = (0.0, 0.0, 0.0);
            let mut impulse_work = 0.0;
            let mut quadratic_work = 0.0;
            for metrics in &run.energy_trace {
                accumulate_energy_delta(
                    &mut integration_delta,
                    metrics.energy_before_integration,
                    metrics.energy_after_integration,
                );
                accumulate_energy_delta(
                    &mut velocity_pcg_delta,
                    kapitsa_core::joint::diagnostics::EnergyComponents::default(),
                    metrics.velocity_pcg_energy_delta,
                );
                accumulate_energy_delta(
                    &mut velocity_solver_delta,
                    metrics.energy_after_integration,
                    metrics.energy_after_velocity_solver,
                );
                accumulate_energy_delta(
                    &mut position_pcg_delta,
                    kapitsa_core::joint::diagnostics::EnergyComponents::default(),
                    metrics.position_pcg_energy_delta,
                );
                accumulate_energy_delta(
                    &mut position_solver_delta,
                    metrics.energy_after_velocity_solver,
                    metrics.energy_after_position_solver,
                );
                impulse_work += metrics.velocity_projection_impulse_work_j;
                quadratic_work += metrics.velocity_projection_quadratic_work_j;
            }
            println!(
                "parts=100 mode=free_rigid_assembly solver={label} ticks={} kinetic_energy_drift={:.6}% contacts={} failure={:?}",
                run.completed_ticks,
                run.max_coast_energy_drift * 100.0,
                run.other_contact_events,
                run.step_failure,
            );
            print_energy_delta(
                "sum_180_integration",
                kapitsa_core::joint::diagnostics::EnergyComponents::default(),
                energy_components_from_delta(integration_delta),
            );
            print_energy_delta(
                "sum_180_velocity_pcg",
                kapitsa_core::joint::diagnostics::EnergyComponents::default(),
                energy_components_from_delta(velocity_pcg_delta),
            );
            print_energy_delta(
                "sum_180_velocity_solver",
                kapitsa_core::joint::diagnostics::EnergyComponents::default(),
                energy_components_from_delta(velocity_solver_delta),
            );
            print_energy_delta(
                "sum_180_position_pcg",
                kapitsa_core::joint::diagnostics::EnergyComponents::default(),
                energy_components_from_delta(position_pcg_delta),
            );
            print_energy_delta(
                "sum_180_position_solver",
                kapitsa_core::joint::diagnostics::EnergyComponents::default(),
                energy_components_from_delta(position_solver_delta),
            );
            println!(
                "velocity_projection solver={label} post_integration_max_Jv={:.6e}m/s/{:.6e}rad/s post_velocity_solver_max_Jv={:.6e}m/s/{:.6e}rad/s post_position_solver_max_Jv={:.6e}m/s/{:.6e}rad/s impulse_work={impulse_work:+.9e}J quadratic_work={quadratic_work:+.9e}J predicted_delta={:+.9e}J measured_delta={:+.9e}J closure_error={:+.9e}J",
                first.initial_velocity_linear_residual_mps,
                first.initial_velocity_angular_residual_radps,
                last.post_velocity_linear_residual_mps,
                last.post_velocity_angular_residual_radps,
                last.post_position_linear_residual_mps,
                last.post_position_angular_residual_radps,
                impulse_work + quadratic_work,
                velocity_solver_delta.0 + velocity_solver_delta.1 + velocity_solver_delta.2,
                impulse_work + quadratic_work
                    - velocity_solver_delta.0
                    - velocity_solver_delta.1
                    - velocity_solver_delta.2,
            );
            print_energy_delta(
                "step0_integration",
                first.energy_before_integration,
                first.energy_after_integration,
            );
            print_energy_delta(
                "step0_velocity_solver",
                first.energy_after_integration,
                first.energy_after_velocity_solver,
            );
            print_energy_delta(
                "step0_position_solver",
                first.energy_after_velocity_solver,
                first.energy_after_position_solver,
            );
            print_energy_delta(
                "step0_velocity_pcg",
                first.energy_before_velocity_pcg,
                first.energy_after_velocity_pcg,
            );
            print_energy_delta(
                "step0_position_pcg",
                first.energy_before_position_pcg,
                first.energy_after_position_pcg,
            );
            print_energy_delta(
                "step179_integration",
                last.energy_before_integration,
                last.energy_after_integration,
            );
            print_energy_delta(
                "step179_velocity_solver",
                last.energy_after_integration,
                last.energy_after_velocity_solver,
            );
            print_energy_delta(
                "step179_position_solver",
                last.energy_after_velocity_solver,
                last.energy_after_position_solver,
            );
            print_energy_delta(
                "step179_velocity_pcg",
                last.energy_before_velocity_pcg,
                last.energy_after_velocity_pcg,
            );
            print_energy_delta(
                "step179_position_pcg",
                last.energy_before_position_pcg,
                last.energy_after_position_pcg,
            );
            assert_eq!(run.completed_ticks, TICKS);
            assert_eq!(run.other_contact_events, 0);
            assert!(run.step_failure.is_none());
            if label == "global_pcg" {
                pcg_conserved = run.max_coast_energy_drift < 1.0e-4;
            }
        }
        assert!(
            pcg_conserved,
            "global PCG free rigid assembly exceeded the 0.01% kinetic-energy drift limit"
        );
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    fn free_rigid_rocket_assembly_tracks_its_equivalent_single_body() {
        let mut rocket = ModularRocket::new(100);
        rocket.world.set_central_gravity(None);
        let colliders = rocket
            .world
            .colliders()
            .map(|(id, _, _)| id)
            .collect::<Vec<_>>();
        for collider in colliders {
            rocket
                .world
                .remove_collider(collider)
                .expect("remove collider");
        }
        rocket
            .world
            .set_force_source_enabled(rocket.lower_engine, false)
            .expect("disable lower engine");
        rocket
            .world
            .set_force_source_enabled(rocket.upper_engine, false)
            .expect("disable upper engine");
        set_rigid_assembly_velocity(&mut rocket);

        let parts = rocket
            .parts
            .iter()
            .map(|id| {
                let body = rocket.world.body(*id).expect("assembly part");
                (
                    body.physics().mass().expect("part mass"),
                    body.state().transform(),
                )
            })
            .collect::<Vec<_>>();
        let combined_mass = MassProperties::combine(&parts).expect("combined mass properties");
        let total_mass = combined_mass.mass();
        let center_velocity = rocket
            .parts
            .iter()
            .map(|id| {
                let body = rocket.world.body(*id).expect("assembly part");
                body.state().linear_velocity() * body.physics().mass().expect("part mass").mass()
            })
            .fold(Vec3::ZERO, |sum, momentum| sum + momentum)
            / total_mass;
        let angular_velocity = rocket
            .world
            .body(rocket.parts[0])
            .expect("assembly part")
            .state()
            .angular_velocity();
        let mut reference_body =
            RigidBody::new_dynamic(Transform::identity(), combined_mass).expect("reference body");
        reference_body
            .set_linear_velocity(center_velocity)
            .expect("finite COM velocity");
        reference_body
            .set_angular_velocity(angular_velocity)
            .expect("finite angular velocity");
        let mut reference_world = PhysicsWorld::new();
        let reference_id = reference_world
            .add_body(reference_body)
            .expect("add reference body");

        rocket
            .world
            .set_joint_global_velocity_pcg_for_bench(64, true);
        rocket
            .world
            .set_joint_global_position_pcg_for_bench(64, true);
        let initial_energy = world_kinetic_energy(&rocket.world);
        let initial_momentum = system_momentum(&rocket.world);
        for _ in 0..TICKS {
            rocket.world.step(DT).expect("free rigid assembly step");
            reference_world.step(DT).expect("free reference-body step");
        }

        let final_energy = world_kinetic_energy(&rocket.world);
        let reference_energy = world_kinetic_energy(&reference_world);
        let final_momentum = system_momentum(&rocket.world);
        let reference_momentum = system_momentum(&reference_world);
        let final_center = system_center_of_mass(&rocket.world);
        let reference_center = system_center_of_mass(&reference_world);
        let relative_energy_error = (final_energy - reference_energy).abs() / initial_energy;
        let relative_angular_momentum_error =
            (final_momentum.1 - reference_momentum.1).length() / initial_momentum.1.length();

        println!(
            "free_rigid_assembly_reference parts={} ticks={TICKS} relative_energy_error={relative_energy_error:.9e} relative_angular_momentum_error={relative_angular_momentum_error:.9e} COM_error={:.9e}m",
            rocket.parts.len(),
            (final_center - reference_center).length(),
        );
        assert!(relative_energy_error < 1.0e-4);
        assert!(relative_angular_momentum_error < 1.0e-4);
        assert!((final_momentum.0 - reference_momentum.0).length() < 1.0e-9);
        assert!((final_center - reference_center).length() < 1.0e-7);
        assert_eq!(reference_world.body_count(), 1);
        assert!(reference_world.body(reference_id).is_some());
    }

    #[cfg(feature = "bench-internals")]
    fn world_kinetic_energy(world: &PhysicsWorld) -> f64 {
        world
            .bodies()
            .filter_map(|(_, body)| {
                let mass = body.physics().mass()?;
                let rotation = body.state().transform().rotation();
                let local_angular_velocity = rotation
                    .conjugate()
                    .rotate_vector(body.state().angular_velocity())
                    .expect("local angular velocity");
                Some(
                    0.5 * mass.mass() * body.state().linear_velocity().length_squared()
                        + 0.5 * local_angular_velocity.dot(mass.inertia() * local_angular_velocity),
                )
            })
            .sum()
    }

    #[cfg(feature = "bench-internals")]
    fn system_momentum(world: &PhysicsWorld) -> (Vec3, Vec3) {
        world.bodies().fold(
            (Vec3::ZERO, Vec3::ZERO),
            |(mut linear, mut angular), (_, body)| {
                let Some(mass) = body.physics().mass() else {
                    return (linear, angular);
                };
                let state = body.state();
                let momentum = state.linear_velocity() * mass.mass();
                let rotation = state.transform().rotation();
                let local_angular_velocity = rotation
                    .conjugate()
                    .rotate_vector(state.angular_velocity())
                    .expect("local angular velocity");
                let spin = rotation
                    .rotate_vector(mass.inertia() * local_angular_velocity)
                    .expect("world angular momentum");
                let center = body
                    .world_center_of_mass()
                    .expect("valid COM")
                    .expect("dynamic body COM");
                linear += momentum;
                angular += spin + center.cross(momentum);
                (linear, angular)
            },
        )
    }

    #[cfg(feature = "bench-internals")]
    fn system_center_of_mass(world: &PhysicsWorld) -> Vec3 {
        let (mass, weighted_center) = world.bodies().fold(
            (0.0, Vec3::ZERO),
            |(total_mass, weighted_center), (_, body)| {
                let Some(body_mass) = body.physics().mass().map(|mass| mass.mass()) else {
                    return (total_mass, weighted_center);
                };
                let center = body
                    .world_center_of_mass()
                    .expect("valid COM")
                    .expect("dynamic body COM");
                (total_mass + body_mass, weighted_center + center * body_mass)
            },
        );
        weighted_center / mass
    }

    fn run_fixed_scenario(part_count: usize, block3: bool) -> StabilityRun {
        run_scenario(
            part_count,
            block3,
            ScenarioMode::Full,
            false,
            None,
            false,
            None,
            None,
            None,
        )
    }

    fn run_fixed_scenario_with_pcg(
        part_count: usize,
        global_velocity_pcg_iterations: Option<u32>,
        global_position_pcg_iterations: Option<u32>,
        outer_velocity_sweeps: Option<u32>,
    ) -> StabilityRun {
        run_scenario(
            part_count,
            false,
            ScenarioMode::Full,
            false,
            outer_velocity_sweeps.map(|velocity| (velocity, 3)),
            false,
            global_velocity_pcg_iterations,
            None,
            global_position_pcg_iterations,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn run_scenario(
        part_count: usize,
        block3: bool,
        mode: ScenarioMode,
        adaptive: bool,
        iteration_counts: Option<(u32, u32)>,
        reverse_joint_order: bool,
        global_velocity_pcg_iterations: Option<u32>,
        velocity_relaxation: Option<f64>,
        global_position_pcg_iterations: Option<u32>,
    ) -> StabilityRun {
        let started = Instant::now();
        let mut rocket = ModularRocket::new(part_count);
        #[cfg(feature = "bench-internals")]
        let partial_workers = std::env::var("KAPITSA_PARTIAL_WORKERS")
            .ok()
            .and_then(|v| v.parse::<usize>().ok());
        #[cfg(feature = "bench-internals")]
        let experiment_executor = partial_workers
            .map(|workers| experiment_executor::RayonExecutor::new(workers).expect("rayon"));
        #[cfg(feature = "bench-internals")]
        rocket
            .world
            .set_partial_coloring_for_bench(partial_workers.is_some());
        let ticks = if std::env::var_os("KAPITSA_ROCKET_300_TICKS").is_some() {
            300
        } else {
            TICKS
        };
        #[cfg(feature = "bench-internals")]
        if reverse_joint_order {
            reverse_joint_storage_order(&mut rocket);
        }
        #[cfg(not(feature = "bench-internals"))]
        assert!(
            !reverse_joint_order,
            "reverse sweep diagnostic needs bench-internals"
        );
        if let Some((velocity_iterations, position_iterations)) = iteration_counts {
            let mut config = rocket.world.collision_config();
            config.velocity_iterations = velocity_iterations;
            config.position_iterations = position_iterations;
            rocket
                .world
                .set_collision_config(config)
                .expect("valid diagnostic solver iteration counts");
        }
        #[cfg(feature = "bench-internals")]
        rocket
            .world
            .set_joint_adaptive_iterations_for_bench(adaptive);
        #[cfg(not(feature = "bench-internals"))]
        assert!(!adaptive, "adaptive comparison requires bench-internals");
        #[cfg(feature = "bench-internals")]
        if let Some(max_iterations) = global_velocity_pcg_iterations {
            match std::env::var("KAPITSA_ROCKET_VELOCITY_PCG_PRECONDITIONER")
                .as_deref()
                .unwrap_or("ic0")
            {
                "ic0" => rocket
                    .world
                    .set_joint_global_velocity_pcg_for_bench(max_iterations, true),
                "diagonal" => rocket
                    .world
                    .set_joint_global_velocity_pcg_for_bench(max_iterations, false),
                "block_jacobi" => rocket
                    .world
                    .set_joint_global_velocity_block_pcg_for_bench(max_iterations),
                other => panic!("unknown velocity PCG preconditioner: {other}"),
            }
        }
        #[cfg(feature = "bench-internals")]
        if let Some(relaxation) = velocity_relaxation {
            rocket
                .world
                .set_joint_velocity_relaxation_for_bench(relaxation);
        }
        #[cfg(feature = "bench-internals")]
        if let Some(max_iterations) = global_position_pcg_iterations {
            match std::env::var("KAPITSA_ROCKET_POSITION_PCG_PRECONDITIONER")
                .as_deref()
                .unwrap_or("ic0")
            {
                "ic0" => rocket
                    .world
                    .set_joint_global_position_pcg_for_bench(max_iterations, true),
                "diagonal" => rocket
                    .world
                    .set_joint_global_position_pcg_for_bench(max_iterations, false),
                other => panic!("unknown position PCG preconditioner: {other}"),
            }
        }
        #[cfg(not(feature = "bench-internals"))]
        assert!(
            global_velocity_pcg_iterations.is_none(),
            "global velocity PCG comparison requires bench-internals"
        );
        #[cfg(not(feature = "bench-internals"))]
        assert!(
            velocity_relaxation.is_none(),
            "relaxed Gauss-Seidel comparison requires bench-internals"
        );
        #[cfg(not(feature = "bench-internals"))]
        assert!(
            global_position_pcg_iterations.is_none(),
            "global position PCG comparison requires bench-internals"
        );
        if matches!(
            mode,
            ScenarioMode::NoLoads
                | ScenarioMode::GravityWithoutContact
                | ScenarioMode::ThrustWithoutGravityOrContact
                | ScenarioMode::ThrustWithGravityWithoutContact
                | ScenarioMode::CenteredThrustWithoutGravityOrContact
                | ScenarioMode::StagedThrustWithoutGravityOrContact
                | ScenarioMode::StagedThrustWithGravityWithoutContact
                | ScenarioMode::StagedThrustWithGravityAndBodyColliders
                | ScenarioMode::RocketCoastWithoutContactOrThrust
                | ScenarioMode::RocketFreeAssemblyWithoutLoads
        ) {
            rocket
                .world
                .remove_collider(rocket.surface_collider)
                .expect("remove diagnostic pad collider");
        }
        if matches!(
            mode,
            ScenarioMode::NoLoads
                | ScenarioMode::ThrustWithoutGravityOrContact
                | ScenarioMode::CenteredThrustWithoutGravityOrContact
                | ScenarioMode::StagedThrustWithoutGravityOrContact
                | ScenarioMode::RocketFreeAssemblyWithoutLoads
        ) {
            rocket.world.set_central_gravity(None);
        }
        if matches!(
            mode,
            ScenarioMode::StagedThrustWithoutGravityOrContact
                | ScenarioMode::StagedThrustWithGravityWithoutContact
                | ScenarioMode::ThrustWithGravityWithoutContact
                | ScenarioMode::RocketCoastWithoutContactOrThrust
                | ScenarioMode::RocketFreeAssemblyWithoutLoads
        ) {
            let colliders = rocket
                .world
                .colliders()
                .map(|(id, _, _)| id)
                .collect::<Vec<_>>();
            for collider in colliders {
                rocket.world.remove_collider(collider);
            }
        }
        if matches!(mode, ScenarioMode::CenteredThrustWithoutGravityOrContact) {
            center_diagnostic_thrust(&mut rocket);
        }
        if matches!(mode, ScenarioMode::RocketFreeAssemblyWithoutLoads) {
            set_rigid_assembly_velocity(&mut rocket);
        }
        #[cfg(feature = "bench-internals")]
        rocket.world.set_joint_projection_for_bench(block3);
        #[cfg(feature = "bench-internals")]
        if block3 && std::env::var_os("KAPITSA_REDUCED_FIXED_TREE").is_some() {
            rocket.world.set_reduced_fixed_tree_for_bench(true);
        }
        #[cfg(feature = "bench-internals")]
        if std::env::var_os("KAPITSA_ROCKET_NO_MASS_CACHE").is_some() {
            rocket.world.set_joint_mass_cache_for_bench(false);
        }
        #[cfg(not(feature = "bench-internals"))]
        assert!(!block3, "FixedBlock3 comparison requires bench-internals");
        let initial_lower_mass = stage_mass(&rocket, &rocket.lower_parts);
        let initial_upper_mass = stage_mass(&rocket, &rocket.upper_parts);
        let mut max_anchor_error: f64 = 0.0;
        let mut anchor_square_sum = 0.0;
        let mut max_frame_error: f64 = 0.0;
        let mut frame_square_sum = 0.0;
        let mut joint_error_samples = 0usize;
        let mut max_coast_energy_drift: f64 = 0.0;
        let mut max_coast_energy_drift_tick = None;
        let mut final_coast_energy_signed_drift: f64 = 0.0;
        let mut coast_energy_start = None;
        #[cfg(feature = "bench-internals")]
        let mut legacy_surface_reference_start = None;
        #[cfg(feature = "bench-internals")]
        let mut legacy_surface_reference_max_drift: f64 = 0.0;
        let mut coast_energy_start_components = None;
        let mut coast_energy_reference_scale = None;
        let mut coast_energy_final_components = (0.0, 0.0, 0.0);
        let mut max_altitude: f64 = 0.0;
        let mut max_angular_speed: f64 = 0.0;
        let trace_flight = std::env::var_os("KAPITSA_ROCKET_TRACE_FLIGHT").is_some();
        #[cfg(feature = "bench-internals")]
        let trace_joint = std::env::var_os("KAPITSA_ROCKET_TRACE_JOINTS").is_some();
        #[cfg(feature = "bench-internals")]
        let tracked_joint = rocket.world.joints().next().map(|(id, _, _, _)| id);
        let mut separation_momentum_error = 0.0;
        let mut surface_begins = 0;
        let mut surface_ends = 0;
        let mut surface_impacts = 0;
        let mut other_contact_events = 0;
        let mut post_cutoff_surface_events = 0;
        let mut post_cutoff_upper_surface_events = 0;
        let mut post_cutoff_upper_surface_event_ticks = Vec::new();
        let mut coast_energy_trace = Vec::new();
        let mut upper_coast_contact_seen = false;
        let mut joints_after_separation = 0;
        let mut completed_ticks = 0;
        let mut step_failure = None;
        let mut joint_break_events = 0;
        let mut first_anchor_limit_tick = None;
        let mut first_frame_limit_tick = None;
        #[cfg(feature = "bench-internals")]
        let mut solver_stage_trace = Vec::with_capacity(ticks);
        #[cfg(feature = "bench-internals")]
        let mut energy_trace = Vec::with_capacity(ticks);

        for tick in 0..ticks {
            match mode {
                ScenarioMode::Full => match tick {
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
                },
                ScenarioMode::ThrustWithoutGravityOrContact
                | ScenarioMode::ThrustWithGravityWithoutContact
                | ScenarioMode::CenteredThrustWithoutGravityOrContact => match tick {
                    12 => rocket
                        .world
                        .set_force_source_enabled(rocket.lower_engine, true)
                        .expect("booster ignition"),
                    90 => rocket
                        .world
                        .set_force_source_enabled(rocket.lower_engine, false)
                        .expect("booster cutoff"),
                    _ => {}
                },
                ScenarioMode::StagedThrustWithoutGravityOrContact
                | ScenarioMode::StagedThrustWithGravityWithoutContact
                | ScenarioMode::StagedThrustWithGravityAndBodyColliders => match tick {
                    12 => rocket
                        .world
                        .set_force_source_enabled(rocket.lower_engine, true)
                        .expect("booster ignition"),
                    24 if matches!(mode, ScenarioMode::StagedThrustWithGravityAndBodyColliders) => {
                        burn_tanks(&mut rocket, true)
                    }
                    54 => {
                        let momentum_before = stage_momentum(&rocket, &rocket.lower_parts)
                            + stage_momentum(&rocket, &rocket.upper_parts);
                        rocket
                            .world
                            .remove_joint(rocket.separation_joint)
                            .expect("diagnostic stage separation");
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
                    72 if matches!(mode, ScenarioMode::StagedThrustWithGravityAndBodyColliders) => {
                        burn_tanks(&mut rocket, false)
                    }
                    90 => rocket
                        .world
                        .set_force_source_enabled(rocket.upper_engine, false)
                        .expect("upper-stage cutoff"),
                    _ => {}
                },
                ScenarioMode::NoLoads
                | ScenarioMode::GravityWithoutContact
                | ScenarioMode::GravityWithContact
                | ScenarioMode::RocketCoastWithoutContactOrThrust
                | ScenarioMode::RocketFreeAssemblyWithoutLoads => {}
            }

            #[cfg(feature = "bench-internals")]
            let result =
                if let (Some(executor), Some(workers)) = (&experiment_executor, partial_workers) {
                    rocket.world.step_with_executor(DT, executor, workers)
                } else {
                    rocket.world.step(DT)
                };
            #[cfg(not(feature = "bench-internals"))]
            let result = rocket.world.step(DT);
            if let Err(error) = result {
                step_failure = Some(format!("tick={tick}: {error:?}"));
                break;
            }
            completed_ticks += 1;
            #[cfg(feature = "bench-internals")]
            energy_trace.push(rocket.world.physics_step_metrics().joints);
            joint_break_events += rocket.world.joint_events().len();
            for event in rocket.world.collision_events() {
                if event.sensor {
                    continue;
                }
                if event.pair.a != rocket.surface_collider
                    && event.pair.b != rocket.surface_collider
                {
                    other_contact_events += 1;
                    continue;
                }
                if tick >= 90 {
                    post_cutoff_surface_events += 1;
                    let vehicle_collider = if event.pair.a == rocket.surface_collider {
                        event.pair.b
                    } else {
                        event.pair.a
                    };
                    if rocket
                        .world
                        .collider_body(vehicle_collider)
                        .is_some_and(|body| rocket.upper_parts.contains(&body))
                    {
                        post_cutoff_upper_surface_events += 1;
                        post_cutoff_upper_surface_event_ticks.push(tick);
                        upper_coast_contact_seen = true;
                    }
                }
                match event.kind {
                    CollisionEventKind::Begin => surface_begins += 1,
                    CollisionEventKind::End => surface_ends += 1,
                    CollisionEventKind::Impact => surface_impacts += 1,
                    CollisionEventKind::Persist => {}
                }
            }

            let mut tick_anchor_error: f64 = 0.0;
            let mut tick_frame_error: f64 = 0.0;
            let errors = joint_errors(&rocket);
            #[cfg(feature = "bench-internals")]
            if trace_joint
                && ([0, 10, 11, 12, 13, 19, 20, 40, 41, 53, 54, 55, 89, 90, 179].contains(&tick)
                    || (first_anchor_limit_tick.is_none()
                        && errors.iter().any(|row| row.1 >= 0.03))
                    || (first_frame_limit_tick.is_none() && errors.iter().any(|row| row.2 >= 0.1)))
            {
                let worst = errors.iter().max_by(|a, b| a.1.total_cmp(&b.1));
                let worst_frame = errors.iter().max_by(|a, b| a.2.total_cmp(&b.2));
                let tracked = errors.iter().find(|row| Some(row.0) == tracked_joint);
                if let Some(worst) = worst {
                    if let Some((_, body_a, body_b, joint)) =
                        rocket.world.joints().find(|row| row.0 == worst.0)
                    {
                        let transform_a = rocket
                            .world
                            .body(body_a)
                            .expect("body A")
                            .state()
                            .transform();
                        let transform_b = rocket
                            .world
                            .body(body_b)
                            .expect("body B")
                            .state()
                            .transform();
                        let anchor_a =
                            transform_a.transform_point(joint.common().frame_a.local_anchor);
                        let anchor_b =
                            transform_b.transform_point(joint.common().frame_b.local_anchor);
                        println!(
                            "joint_trace tick={tick} tracked={tracked:?} worst_anchor={worst:?} worst_frame={worst_frame:?} body_a={body_a:?} body_b={body_b:?} local_a={:?} local_b={:?} world_a={anchor_a:?} world_b={anchor_b:?}",
                            joint.common().frame_a.local_anchor,
                            joint.common().frame_b.local_anchor
                        );
                    }
                }
            }
            for (_, anchor_error, frame_error) in errors {
                tick_anchor_error = tick_anchor_error.max(anchor_error);
                tick_frame_error = tick_frame_error.max(frame_error);
                max_anchor_error = max_anchor_error.max(anchor_error);
                anchor_square_sum += anchor_error * anchor_error;
                max_frame_error = max_frame_error.max(frame_error);
                frame_square_sum += frame_error * frame_error;
                joint_error_samples += 1;
            }
            if first_anchor_limit_tick.is_none() && tick_anchor_error >= 0.03 {
                first_anchor_limit_tick = Some(tick);
            }
            if first_frame_limit_tick.is_none() && tick_frame_error >= 0.1 {
                first_frame_limit_tick = Some(tick);
            }
            #[cfg(feature = "bench-internals")]
            {
                let joint_metrics = rocket.world.physics_step_metrics().joints;
                solver_stage_trace.push(SolverStageSample {
                    tick,
                    anchor_error_m: tick_anchor_error,
                    frame_error_rad: tick_frame_error,
                    pre_velocity_anchor_error_m: joint_metrics.pre_velocity_anchor_error_m,
                    pre_velocity_frame_error_rad: joint_metrics.pre_velocity_frame_error_rad,
                    post_velocity_anchor_error_m: joint_metrics.post_velocity_anchor_error_m,
                    post_velocity_frame_error_rad: joint_metrics.post_velocity_frame_error_rad,
                    pre_joint_anchor_error_m: joint_metrics.pre_joint_anchor_error_m,
                    pre_joint_frame_error_rad: joint_metrics.pre_joint_frame_error_rad,
                    post_joint_anchor_error_m: joint_metrics.post_joint_anchor_error_m,
                    post_joint_frame_error_rad: joint_metrics.post_joint_frame_error_rad,
                    post_velocity_linear_residual_mps: joint_metrics
                        .post_velocity_linear_residual_mps,
                    post_velocity_angular_residual_radps: joint_metrics
                        .post_velocity_angular_residual_radps,
                    post_position_linear_residual_mps: joint_metrics
                        .post_position_linear_residual_mps,
                    post_position_angular_residual_radps: joint_metrics
                        .post_position_angular_residual_radps,
                    first_velocity_linear_residual_mps: joint_metrics
                        .first_velocity_linear_residual_mps,
                    first_velocity_angular_residual_radps: joint_metrics
                        .first_velocity_angular_residual_radps,
                    first_post_joint_anchor_error_m: joint_metrics.first_post_joint_anchor_error_m,
                    first_post_joint_frame_error_rad: joint_metrics
                        .first_post_joint_frame_error_rad,
                    pre_joint_velocity_linear_residual_mps: joint_metrics
                        .pre_joint_velocity_linear_residual_mps,
                    pre_joint_velocity_angular_residual_radps: joint_metrics
                        .pre_joint_velocity_angular_residual_radps,
                    velocity_pcg_iterations: joint_metrics.velocity_pcg_iterations,
                    position_pcg_iterations: joint_metrics.position_pcg_iterations,
                });
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
                    if trace_flight
                        && matches!(
                            tick,
                            11 | 12 | 23 | 24 | 53 | 54 | 55 | 71 | 72 | 89 | 90 | 120 | 179
                        )
                    {
                        println!(
                            "flight_trace tick={tick} y={:.6} vy={:.6}m/s",
                            position.y,
                            state.linear_velocity().y,
                        );
                    }
                }
                max_angular_speed = max_angular_speed.max(state.angular_velocity().length());
            }

            if tick >= 90 && !upper_coast_contact_seen {
                let energy_parts = if matches!(
                    mode,
                    ScenarioMode::RocketCoastWithoutContactOrThrust
                        | ScenarioMode::RocketFreeAssemblyWithoutLoads
                ) {
                    &rocket.parts
                } else {
                    &rocket.upper_parts
                };
                let components = mechanical_energy_components(&rocket, energy_parts);
                let energy = match mode {
                    ScenarioMode::StagedThrustWithoutGravityOrContact => {
                        kinetic_energy(&rocket, energy_parts)
                    }
                    ScenarioMode::RocketFreeAssemblyWithoutLoads => {
                        kinetic_energy(&rocket, energy_parts)
                    }
                    _ => components.0 + components.1 + components.2,
                };
                let initial = *coast_energy_start.get_or_insert_with(|| {
                    coast_energy_start_components = Some(components);
                    energy
                });
                let reference_scale = *coast_energy_reference_scale.get_or_insert_with(|| {
                    components.0.abs() + components.1.abs() + components.2.abs()
                });
                let signed_drift = (energy - initial) / reference_scale.max(1.0);
                let drift = signed_drift.abs();
                #[cfg(feature = "bench-internals")]
                {
                    let surface_reference_offset =
                        rocket.world.central_gravity().map_or(0.0, |gravity| {
                            let coast_mass = energy_parts
                                .iter()
                                .map(|id| {
                                    rocket
                                        .world
                                        .body(*id)
                                        .expect("coast energy body")
                                        .physics()
                                        .mass()
                                        .expect("coast energy mass")
                                        .mass()
                                })
                                .sum::<f64>();
                            gravity.gravitational_parameter() * coast_mass
                                / gravity.center().length()
                        });
                    let surface_reference_energy = energy + surface_reference_offset;
                    let surface_reference_initial =
                        *legacy_surface_reference_start.get_or_insert(surface_reference_energy);
                    legacy_surface_reference_max_drift = legacy_surface_reference_max_drift.max(
                        (surface_reference_energy - surface_reference_initial).abs()
                            / surface_reference_initial.abs().max(1.0),
                    );
                }
                coast_energy_trace.push((tick, energy));
                final_coast_energy_signed_drift = signed_drift;
                coast_energy_final_components = components;
                if drift > max_coast_energy_drift {
                    max_coast_energy_drift = drift;
                    max_coast_energy_drift_tick = Some(tick);
                }
            }
        }

        if completed_ticks == ticks && matches!(mode, ScenarioMode::Full) {
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
        #[cfg(feature = "bench-internals")]
        let pcg_summary =
            energy_trace
                .iter()
                .fold(PcgSummary::default(), |mut summary, metrics| {
                    summary.velocity_converged_calls += metrics.velocity_pcg_converged_calls;
                    summary.velocity_iteration_limit_calls +=
                        metrics.velocity_pcg_iteration_limit_calls;
                    summary.velocity_linear_residual_mps = summary
                        .velocity_linear_residual_mps
                        .max(metrics.velocity_pcg_linear_residual_mps);
                    summary.velocity_angular_residual_radps = summary
                        .velocity_angular_residual_radps
                        .max(metrics.velocity_pcg_angular_residual_radps);
                    summary.velocity_iterations += metrics.velocity_pcg_iterations;
                    summary.position_iterations += metrics.position_pcg_iterations;
                    summary.position_converged_calls += metrics.position_pcg_converged_calls;
                    summary.position_iteration_limit_calls +=
                        metrics.position_pcg_iteration_limit_calls;
                    summary.position_linear_residual_m = summary
                        .position_linear_residual_m
                        .max(metrics.position_pcg_linear_residual_m);
                    summary.position_angular_residual_rad = summary
                        .position_angular_residual_rad
                        .max(metrics.position_pcg_angular_residual_rad);
                    summary
                });
        #[cfg(not(feature = "bench-internals"))]
        let pcg_summary = PcgSummary::default();
        let samples = joint_error_samples.max(1) as f64;
        StabilityRun {
            states,
            max_anchor_error,
            rms_anchor_error: (anchor_square_sum / samples).sqrt(),
            max_frame_error,
            rms_frame_error: (frame_square_sum / samples).sqrt(),
            max_coast_energy_drift,
            max_coast_energy_drift_tick,
            final_coast_energy_signed_drift,
            #[cfg(feature = "bench-internals")]
            legacy_surface_reference_max_drift,
            coast_energy_start_components: coast_energy_start_components.unwrap_or_default(),
            coast_energy_reference_scale_j: coast_energy_reference_scale.unwrap_or_default(),
            coast_energy_final_components,
            max_altitude,
            upper_root_final_velocity: rocket
                .world
                .body(rocket.upper_root)
                .expect("upper root")
                .state()
                .linear_velocity(),
            max_angular_speed,
            separation_momentum_error,
            surface_begins,
            surface_ends,
            surface_impacts,
            other_contact_events,
            post_cutoff_surface_events,
            post_cutoff_upper_surface_events,
            post_cutoff_upper_surface_event_ticks,
            coast_energy_trace,
            lower_fuel_mass,
            upper_fuel_mass,
            joints_after_separation,
            joint_break_events,
            elapsed: started.elapsed(),
            completed_ticks,
            step_failure,
            first_anchor_limit_tick,
            first_frame_limit_tick,
            velocity_pcg_converged_calls: pcg_summary.velocity_converged_calls,
            velocity_pcg_iteration_limit_calls: pcg_summary.velocity_iteration_limit_calls,
            velocity_pcg_iterations: pcg_summary.velocity_iterations,
            position_pcg_iterations: pcg_summary.position_iterations,
            velocity_pcg_linear_residual_mps: pcg_summary.velocity_linear_residual_mps,
            velocity_pcg_angular_residual_radps: pcg_summary.velocity_angular_residual_radps,
            position_pcg_converged_calls: pcg_summary.position_converged_calls,
            position_pcg_iteration_limit_calls: pcg_summary.position_iteration_limit_calls,
            position_pcg_linear_residual_m: pcg_summary.position_linear_residual_m,
            position_pcg_angular_residual_rad: pcg_summary.position_angular_residual_rad,
            #[cfg(feature = "bench-internals")]
            solver_stage_trace,
            #[cfg(feature = "bench-internals")]
            energy_trace,
        }
    }

    #[cfg(feature = "bench-internals")]
    fn print_energy_delta(
        label: &str,
        before: kapitsa_core::joint::diagnostics::EnergyComponents,
        after: kapitsa_core::joint::diagnostics::EnergyComponents,
    ) {
        let translational = after.translational_j - before.translational_j;
        let rotational = after.rotational_j - before.rotational_j;
        let potential = after.potential_j - before.potential_j;
        println!(
            "energy_stage={label} d_translation={translational:+.9e}J d_rotation={rotational:+.9e}J d_potential={potential:+.9e}J d_total={:+.9e}J",
            translational + rotational + potential,
        );
    }

    #[cfg(feature = "bench-internals")]
    fn accumulate_energy_delta(
        total: &mut (f64, f64, f64),
        before: kapitsa_core::joint::diagnostics::EnergyComponents,
        after: kapitsa_core::joint::diagnostics::EnergyComponents,
    ) {
        total.0 += after.translational_j - before.translational_j;
        total.1 += after.rotational_j - before.rotational_j;
        total.2 += after.potential_j - before.potential_j;
    }

    #[cfg(feature = "bench-internals")]
    fn energy_components_from_delta(
        delta: (f64, f64, f64),
    ) -> kapitsa_core::joint::diagnostics::EnergyComponents {
        kapitsa_core::joint::diagnostics::EnergyComponents {
            translational_j: delta.0,
            rotational_j: delta.1,
            potential_j: delta.2,
        }
    }

    #[cfg(feature = "bench-internals")]
    fn sum_energy_deltas(
        metrics: &[kapitsa_core::joint::diagnostics::JointSolverMetrics],
        before: impl Fn(
            &kapitsa_core::joint::diagnostics::JointSolverMetrics,
        ) -> kapitsa_core::joint::diagnostics::EnergyComponents,
        after: impl Fn(
            &kapitsa_core::joint::diagnostics::JointSolverMetrics,
        ) -> kapitsa_core::joint::diagnostics::EnergyComponents,
    ) -> kapitsa_core::joint::diagnostics::EnergyComponents {
        let mut total = kapitsa_core::joint::diagnostics::EnergyComponents::default();
        for metric in metrics {
            let delta = after(metric).delta(before(metric));
            total.translational_j += delta.translational_j;
            total.rotational_j += delta.rotational_j;
            total.potential_j += delta.potential_j;
        }
        total
    }

    fn set_rigid_assembly_velocity(rocket: &mut ModularRocket) {
        let origin = rocket
            .world
            .body(rocket.lower_root)
            .and_then(|body| body.world_center_of_mass().ok().flatten())
            .expect("assembly origin");
        let base_velocity = Vec3::new(0.5, -0.25, 0.1);
        let angular_velocity = Vec3::new(0.15, -0.12, 0.2);
        for id in rocket.parts.clone() {
            let center = rocket
                .world
                .body(id)
                .and_then(|body| body.world_center_of_mass().ok().flatten())
                .expect("part center of mass");
            let linear_velocity = base_velocity
                + angular_velocity
                    .checked_cross(center - origin)
                    .expect("rigid assembly velocity");
            let body = rocket.world.body_mut(id).expect("assembly part");
            body.set_linear_velocity(linear_velocity)
                .expect("finite assembly velocity");
            body.set_angular_velocity(angular_velocity)
                .expect("finite assembly spin");
        }
    }

    #[cfg(feature = "bench-internals")]
    fn reverse_joint_storage_order(rocket: &mut ModularRocket) {
        let joints = rocket
            .world
            .joints()
            .map(|(id, body_a, body_b, joint)| (id, body_a, body_b, *joint))
            .collect::<Vec<_>>();
        let original_pairs = joints
            .iter()
            .map(|(_, body_a, body_b, _)| (*body_a, *body_b))
            .collect::<Vec<_>>();
        let old_separation_joint = rocket.separation_joint;
        for (id, _, _, _) in joints.iter().rev() {
            rocket
                .world
                .remove_joint(*id)
                .expect("remove diagnostic joint");
        }
        let mut expected_pairs = original_pairs;
        expected_pairs.reverse();
        for (old_id, body_a, body_b, joint) in joints.into_iter().rev() {
            let new_id = rocket
                .world
                .add_joint(body_a, body_b, joint)
                .expect("restore diagnostic joint in reverse order");
            if old_id == old_separation_joint {
                rocket.separation_joint = new_id;
            }
        }
        let reversed_pairs = rocket
            .world
            .joints()
            .map(|(_, body_a, body_b, _)| (body_a, body_b))
            .collect::<Vec<_>>();
        assert_eq!(reversed_pairs, expected_pairs);
    }

    fn center_diagnostic_thrust(rocket: &mut ModularRocket) {
        let thrust = rocket
            .world
            .force_source(rocket.lower_engine)
            .expect("lower engine")
            .local_force();
        let center = rocket
            .world
            .body(rocket.lower_root)
            .expect("lower root")
            .physics()
            .mass()
            .expect("root mass")
            .center_of_mass();
        rocket
            .world
            .set_force_source(rocket.lower_engine, center, thrust)
            .expect("centered diagnostic engine");
    }

    #[test]
    fn centered_diagnostic_thrust_uses_root_center_of_mass() {
        let mut rocket = ModularRocket::new(100);
        center_diagnostic_thrust(&mut rocket);
        let mass = rocket
            .world
            .body(rocket.lower_root)
            .unwrap()
            .physics()
            .mass()
            .unwrap();
        let point = rocket
            .world
            .force_source(rocket.lower_engine)
            .unwrap()
            .local_point();
        assert_eq!(point, mass.center_of_mass());
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

    fn mechanical_energy_components(
        rocket: &ModularRocket,
        parts: &[kapitsa_core::world::BodyId],
    ) -> (f64, f64, f64) {
        let gravity = rocket.world.central_gravity();
        let mut translational = 0.0;
        let mut rotational = 0.0;
        let mut potential = 0.0;
        for &id in parts {
            let body = rocket.world.body(id).expect("energy body");
            let mass = body.physics().mass().expect("energy mass");
            let state = body.state();
            let transform = state.transform();
            let velocity = state.linear_velocity();
            let omega = state.angular_velocity();
            let rotation = transform.rotation();
            let local_omega = rotation
                .conjugate()
                .rotate_vector(omega)
                .expect("local angular velocity");
            let world_i_omega = rotation
                .rotate_vector(mass.inertia() * local_omega)
                .expect("world angular momentum");
            translational += 0.5 * mass.mass() * velocity.length_squared();
            rotational += 0.5 * omega.dot(world_i_omega);
            if let Some(gravity) = gravity {
                let position = transform.transform_point(mass.center_of_mass());
                let radius = (position - gravity.center()).length();
                if radius > 0.0 {
                    // Keep the physical zero at infinity; a surface-relative
                    // offset makes percent drift depend on an arbitrary constant.
                    potential -= gravity.gravitational_parameter() * mass.mass() / radius;
                }
            }
        }
        (translational, rotational, potential)
    }

    fn kinetic_energy(rocket: &ModularRocket, parts: &[kapitsa_core::world::BodyId]) -> f64 {
        parts
            .iter()
            .map(|&id| {
                let body = rocket.world.body(id).expect("energy body");
                let mass = body.physics().mass().expect("energy mass");
                let state = body.state();
                let omega = state.angular_velocity();
                let rotation = state.transform().rotation();
                let local_omega = rotation
                    .conjugate()
                    .rotate_vector(omega)
                    .expect("local angular velocity");
                let world_i_omega = rotation
                    .rotate_vector(mass.inertia() * local_omega)
                    .expect("world angular momentum");
                0.5 * mass.mass() * state.linear_velocity().length_squared()
                    + 0.5 * omega.dot(world_i_omega)
            })
            .sum()
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
