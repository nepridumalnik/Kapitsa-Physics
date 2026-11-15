use super::*;
use crate::{
    aerodynamics::hull::HullSection,
    aerodynamics::{Aerodynamics, AtmosphericState},
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
};
use kapitsa_math::{Mat3, Quat, Vec3};

fn section(point: Vec3) -> HullSection {
    HullSection::new(
        Transform::new(point, Quat::IDENTITY).expect("test transform"),
        Vec3::X,
        2.0,
        3.0,
        4.0,
        5.0,
        0.2,
        0.8,
        None,
    )
    .expect("test hull section")
}

#[test]
fn primary_hull_transfers_to_root_part_without_changing_source_coordinates() {
    let mut world = PhysicsWorld::new();
    world.set_atmosphere(test_atmosphere(
        AtmosphericState::new(1.2, Vec3::ZERO).expect("state"),
    ));
    let hull = vec![section(Vec3::X * 2.0), section(-Vec3::Y)];
    let mut body = RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(10.0, Vec3::ZERO, Mat3::IDENTITY * 10.0).expect("mass"),
    )
    .expect("body");
    body.set_aerodynamics(Aerodynamics::Hull(hull.clone()));
    let id = world.add_body(body).expect("body id");
    let part = world.enable_cluster(id).expect("cluster");
    assert_eq!(world.part_aerodynamic_hull(part), Some(hull.as_slice()));
    assert!(matches!(
        world.body(id).expect("body").aerodynamics(),
        Aerodynamics::None
    ));
}

#[test]
fn part_hull_accessor_and_late_failure_preserve_configuration() {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("body"))
        .expect("body id");
    let part = world.enable_cluster(body).expect("cluster");
    let hull = vec![section(Vec3::Z)];
    world
        .set_part_aerodynamic_hull(part, hull.clone())
        .expect("setter");
    assert_eq!(world.part_aerodynamic_hull(part), Some(hull.as_slice()));
    let mut foreign_world = PhysicsWorld::new();
    let foreign_body = foreign_world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("foreign body"))
        .expect("foreign id");
    let foreign_part = foreign_world
        .enable_cluster(foreign_body)
        .expect("foreign part");
    assert_eq!(
        world.set_part_aerodynamic_hull(foreign_part, hull),
        Err(ClusterError::InvalidPartId)
    );
    assert_eq!(
        world.part_aerodynamic_hull(part),
        Some([section(Vec3::Z)].as_slice())
    );
}

#[test]
fn equal_net_drag_on_tree_can_imply_different_internal_loads()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::{
        cluster::{
            ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength,
            balance::{BalanceError, PartBalanceInput, tree_reactions},
        },
        joint::JointFrame,
    };
    const DT: f64 = 1e-4;
    let mut velocities = Vec::new();
    for (distributed, sharded) in [(false, false), (true, true)] {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(test_atmosphere(AtmosphericState::new(1.2, Vec3::ZERO)?));
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::X * 10.0)?;
        let id = world.add_body(body)?;
        let root = world.enable_cluster(id)?;
        let child = world.add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::identity(),
                mass,
                colliders: Vec::new(),
                linear_velocity: Vec3::X * 10.0,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame::IDENTITY,
                frame_b: JointFrame::IDENTITY,
                strength: WeldStrength::Unbreakable,
                stiffness: WeldStiffness {
                    linear: 1.0,
                    angular: 1.0,
                },
            },
        )?;
        let drag_section = |area| {
            HullSection::new(
                Transform::identity(),
                Vec3::X,
                1.0,
                area,
                0.0,
                0.0,
                1.0,
                0.0,
                None,
            )
        };
        if distributed {
            world.set_part_aerodynamic_hull(root, vec![drag_section(0.5)?])?;
            world.set_part_aerodynamic_hull(child, vec![drag_section(0.5)?])?;
        } else {
            world.set_part_aerodynamic_hull(child, vec![drag_section(1.0)?])?;
        }
        if sharded {
            world.step_with_shards(
                DT,
                &mut [Vec::new(), Vec::new()],
                &super::tests::ScopedThreadExecutor,
                2,
            )?;
        } else {
            world.step(DT)?;
        }
        let snapshot = world.part_hull_wrenches().to_vec();
        let load_states = world.cluster_load_states().to_vec();
        let load_parts = world.cluster_load_parts().to_vec();
        let load_welds = world.cluster_load_welds().to_vec();
        assert_eq!(load_states.len(), 1);
        assert_eq!(load_states[0].body, id);
        assert_eq!(load_states[0].root, root);
        assert_eq!(load_states[0].state.linear_velocity(), Vec3::X * 10.0);
        assert_eq!(load_parts.len(), 2);
        assert!(
            load_parts
                .iter()
                .any(|part| part.part == child && part.mass == mass)
        );
        assert_eq!(load_welds.len(), 1);
        assert_eq!(load_welds[0].value.parts(), (root, child));
        let sum = snapshot
            .iter()
            .fold(Vec3::ZERO, |sum, load| sum + load.wrench.force);
        assert!((sum - Vec3::X * -60.0).length() < 1e-12);
        assert!(
            snapshot
                .iter()
                .all(|load| load.body == id && load.wrench.moment == Vec3::ZERO)
        );
        assert_eq!(snapshot.len(), if distributed { 2 } else { 1 });
        assert!(snapshot.iter().any(|load| load.part == child));
        if distributed {
            assert!(snapshot.iter().any(|load| load.part == root));
        }
        let state = world.body(id).ok_or("cluster body")?.state();
        assert!((state.linear_velocity().x - (10.0 - 30.0 * DT)).abs() < 1e-12);
        assert!(state.angular_velocity().length() < 1e-12);
        // This tightly controlled fixture has only part hull loads, no
        // contacts, gravity, joints, or other force sources. The snapshots
        // are evaluated at the initial pose, which is identity here.
        let mut inputs = [PartBalanceInput {
            mass,
            pose: Transform::identity(),
            external_force: Vec3::ZERO,
            external_moment: Vec3::ZERO,
            loads_complete: true,
        }; 2];
        for load in &snapshot {
            let index = if load.part == root { 0 } else { 1 };
            inputs[index].external_force += load.wrench.force;
            inputs[index].external_moment += load.wrench.moment;
        }
        let balance = |inputs: &[PartBalanceInput]| {
            tree_reactions(
                inputs,
                &[0, 0],
                Vec3::ZERO,
                Vec3::X * -30.0,
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
                1e-10,
                1e-10,
            )
        };
        let reactions = balance(&inputs).map_err(|error| format!("balance: {error:?}"))?;
        let expected = if distributed {
            Vec3::ZERO
        } else {
            Vec3::X * 30.0
        };
        assert!((reactions[1].0 - expected).length() < 1e-10);
        assert!(reactions[1].1.length() < 1e-10);
        inputs[1].loads_complete = false;
        assert_eq!(balance(&inputs), Err(BalanceError::IncompleteLoads));
        velocities.push(state.linear_velocity());
        let center = world
            .body(id)
            .ok_or("body")?
            .motion()
            .world_center_of_mass()?
            .ok_or("center")?;
        let bad = world.add_gravity(crate::gravity::CentralGravity::new(center, 1.0)?)?;
        let failed = if sharded {
            world.step_with_shards(
                DT,
                &mut [Vec::new(), Vec::new()],
                &super::tests::ScopedThreadExecutor,
                2,
            )
        } else {
            world.step(DT)
        };
        assert!(failed.is_err());
        assert_eq!(world.part_hull_wrenches(), snapshot.as_slice());
        assert_eq!(world.cluster_load_states(), load_states.as_slice());
        assert_eq!(world.cluster_load_parts(), load_parts.as_slice());
        assert_eq!(world.cluster_load_welds(), load_welds.as_slice());
        world.remove_gravity(bad).ok_or("gravity")?;
    }
    assert!((velocities[0] - velocities[1]).length() < 1e-12);
    // Equal total drag can nevertheless produce different cut reactions.
    Ok(())
}

#[test]
fn part_hull_snapshot_preserves_offset_moment() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    world.set_atmosphere(test_atmosphere(AtmosphericState::new(1.2, Vec3::ZERO)?));
    let mass = MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY * 2.0)?;
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
    body.set_linear_velocity(Vec3::X * 10.0)?;
    let id = world.add_body(body)?;
    let part = world.enable_cluster(id)?;
    world.set_part_aerodynamic_hull(
        part,
        vec![HullSection::new(
            Transform::new(Vec3::Y, Quat::IDENTITY)?,
            Vec3::X,
            1.0,
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            None,
        )?],
    )?;
    world.step(1e-6)?;
    let loads = world.part_hull_wrenches();
    assert_eq!(loads.len(), 1);
    assert_eq!(loads[0].part, part);
    assert!((loads[0].wrench.force + Vec3::X * 60.0).length() < 1e-12);
    assert!((loads[0].wrench.moment - Vec3::Z * 60.0).length() < 1e-12);
    let state = world.body(id).ok_or("body")?.state();
    assert!((state.angular_velocity().z - 30.0e-6).abs() < 1e-12);
    Ok(())
}

#[test]
fn part_simple_surfaces_share_body_level_drag_limit() -> Result<(), Box<dyn std::error::Error>> {
    use crate::{
        aerodynamics::AerodynamicProperties,
        cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength},
        joint::JointFrame,
    };

    // q = 0.5 * 1.2 * 10² = 60 Pa. Each unit-area, Cd=1 surface
    // requests 60 N, but the aggregate drag impulse is limited to the
    // 20 kg m/s momentum of this 2 kg cluster over a one-second step.
    for sharded in [false, true] {
        for distributed in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(test_atmosphere(AtmosphericState::new(1.2, Vec3::ZERO)?));
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::X * 10.0)?;
            let id = world.add_body(body)?;
            let root = world.enable_cluster(id)?;
            let child = world.add_cluster_part(
                root,
                ClusterPartDesc {
                    local_transform: Transform::identity(),
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::X * 10.0,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    frame_a: JointFrame::IDENTITY,
                    frame_b: JointFrame::IDENTITY,
                    strength: WeldStrength::Unbreakable,
                    stiffness: WeldStiffness {
                        linear: 1.0,
                        angular: 1.0,
                    },
                },
            )?;
            let properties = AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO)?;
            world.add_aerodynamic_surface_for_part(child, properties)?;
            if distributed {
                world.add_aerodynamic_surface_for_part(root, properties)?;
            }
            if sharded {
                world.step_with_shards(
                    1.0,
                    &mut [Vec::new(), Vec::new()],
                    &super::tests::ScopedThreadExecutor,
                    2,
                )?;
            } else {
                world.step(1.0)?;
            }
            let state = world.body(id).ok_or("body")?.state();
            assert!(state.linear_velocity().length() < 1e-12);
            assert!(state.angular_velocity().length() < 1e-12);
            let snapshot = world.simple_surface_corrections().to_vec();
            assert_eq!(snapshot.len(), 1);
            let correction = snapshot[0];
            let raw = world.raw_simple_surface_wrenches().to_vec();
            assert_eq!(
                correction.check_part_attribution(&raw),
                Err(SimpleSurfaceAttributionError::CorrectedAggregate)
            );
            assert_eq!(raw.len(), if distributed { 2 } else { 1 });
            assert!(
                raw.iter()
                    .all(|entry| entry.body == id && entry.drag.force == Vec3::X * -60.0)
            );
            assert!(
                raw.iter().any(
                    |entry| entry.target == crate::aerodynamics::AerodynamicTarget::Part(child)
                )
            );
            if distributed {
                assert!(raw.iter().any(
                    |entry| entry.target == crate::aerodynamics::AerodynamicTarget::Part(root)
                ));
            }
            assert_eq!(
                raw.iter()
                    .fold(Vec3::ZERO, |sum, entry| sum + entry.drag.force),
                correction.raw_drag.force
            );
            assert_eq!(correction.body, id);
            let expected_raw = if distributed { -120.0 } else { -60.0 };
            assert!((correction.raw_drag.force.x - expected_raw).abs() < 1e-12);
            assert!((correction.applied_drag.force.x + 20.0).abs() < 1e-12);
            assert_eq!(correction.raw_drag.moment, Vec3::ZERO);
            assert_eq!(correction.applied_drag.moment, Vec3::ZERO);
            assert_eq!(correction.raw_lift.force, Vec3::ZERO);
            assert_eq!(correction.applied_lift.force, Vec3::ZERO);
            let center = world
                .body(id)
                .ok_or("body")?
                .motion()
                .world_center_of_mass()?
                .ok_or("center")?;
            let bad = world.add_gravity(crate::gravity::CentralGravity::new(center, 1.0)?)?;
            let failed = if sharded {
                world.step_with_shards(
                    1.0,
                    &mut [Vec::new(), Vec::new()],
                    &super::tests::ScopedThreadExecutor,
                    2,
                )
            } else {
                world.step(1.0)
            };
            assert!(failed.is_err());
            assert_eq!(world.simple_surface_corrections(), snapshot.as_slice());
            assert_eq!(world.raw_simple_surface_wrenches(), raw.as_slice());
            world.remove_gravity(bad).ok_or("gravity")?;
            world.set_atmosphere(crate::aerodynamics::Atmosphere::Vacuum);
            world.step(1.0)?;
            assert!(world.simple_surface_corrections().is_empty());
            assert!(world.raw_simple_surface_wrenches().is_empty());
        }
    }
    Ok(())
}

#[test]
fn simple_surface_attribution_accepts_only_complete_uncorrected_part_loads()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::{AerodynamicProperties, AerodynamicTarget};
    for sharded in [false, true] {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(test_atmosphere(AtmosphericState::new(1.2, Vec3::ZERO)?));
        let mass = MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY * 2.0)?;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::X * 10.0)?;
        let id = world.add_body(body)?;
        let part = world.enable_cluster(id)?;
        world.add_aerodynamic_surface_for_part(
            part,
            AerodynamicProperties::new(1.0, 1.0, Vec3::Y)?,
        )?;
        if sharded {
            world.step_with_shards(
                1e-6,
                &mut [Vec::new(), Vec::new()],
                &super::tests::ScopedThreadExecutor,
                2,
            )?;
        } else {
            world.step(1e-6)?;
        }
        let correction = *world
            .simple_surface_corrections()
            .first()
            .ok_or("correction")?;
        let raw = world.raw_simple_surface_wrenches();
        assert_eq!(correction.check_part_attribution(raw), Ok(()));
        assert_eq!(raw[0].target, AerodynamicTarget::Part(part));
        assert_eq!(raw[0].drag.force, Vec3::X * -60.0);
        assert_eq!(raw[0].drag.moment, Vec3::Z * 60.0);
        assert_eq!(
            correction.check_part_attribution(&[]),
            Err(SimpleSurfaceAttributionError::InconsistentSnapshot)
        );
        let mut body_target = raw.to_vec();
        body_target[0].target = AerodynamicTarget::Body;
        assert_eq!(
            correction.check_part_attribution(&body_target),
            Err(SimpleSurfaceAttributionError::BodyTarget)
        );
    }
    Ok(())
}
