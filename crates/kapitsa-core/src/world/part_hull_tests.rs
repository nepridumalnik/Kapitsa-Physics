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
            balance::{
                BalanceError, PartBalanceInput, RecordedPartLoads, prepare_recorded_loads,
                prepare_tree, tree_reactions,
            },
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
        let tree = prepare_tree(&load_states[0], &load_parts, &load_welds)
            .map_err(|error| format!("tree: {error:?}"))?;
        assert_eq!(tree.len(), 2);
        assert_eq!(
            (tree[0].part, tree[0].parent, tree[0].weld),
            (root, 0, None)
        );
        assert_eq!(
            (tree[1].part, tree[1].parent, tree[1].weld),
            (child, 0, Some(load_welds[0].weld))
        );
        assert_eq!(tree[1].mass, mass);
        assert_eq!(tree[1].pose, Transform::identity());
        assert!(!tree[1].incomplete_balance_input().loads_complete);
        let recorded = prepare_recorded_loads(
            &load_states[0],
            &tree,
            RecordedPartLoads {
                sources: &[],
                gravity: &[],
                hull: &snapshot,
                wings: &[],
            },
            Vec3::X,
            Vec3::ZERO,
        )
        .map_err(|error| format!("loads: {error:?}"))?;
        assert!(recorded.iter().all(|input| !input.loads_complete));
        assert_eq!(
            recorded[1].external_force,
            if distributed {
                Vec3::X * -30.0
            } else {
                Vec3::X * -60.0
            }
        );
        assert_eq!(recorded[1].external_moment, Vec3::ZERO);
        assert!(matches!(
            prepare_tree(&load_states[0], &load_parts, &[]),
            Err(BalanceError::InvalidTree)
        ));
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
fn automatic_weld_failure_multiple_clusters_is_atomic_on_rejection()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength};
    use crate::joint::JointFrame;
    for sharded in [false, true] {
        let mut world = PhysicsWorld::new();
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let mut clusters = Vec::new();
        for _ in 0..2 {
            let body = world.add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)?;
            let root = world.enable_cluster(body)?;
            let child = world.add_cluster_part(
                root,
                ClusterPartDesc {
                    local_transform: Transform::identity(),
                    mass,
                    colliders: Vec::new(),
                    linear_velocity: Vec3::ZERO,
                    angular_velocity: Vec3::ZERO,
                },
                WeldDesc {
                    frame_a: JointFrame::IDENTITY,
                    frame_b: JointFrame::IDENTITY,
                    strength: WeldStrength::Breakable {
                        max_force: Some(4.0),
                        max_torque: None,
                    },
                    stiffness: WeldStiffness {
                        linear: 1.0,
                        angular: 1.0,
                    },
                },
            )?;
            let weld = world.cluster(body).ok_or("cluster")?.welds()[0];
            world.add_force_source_for_part(child, Vec3::ZERO, Vec3::X * 10.0)?;
            clusters.push((body, root, child, weld));
        }
        world
            .set_automatic_weld_failure(Some((1e-10, 1e-10)))
            .map_err(|e| format!("{e:?}"))?;
        world.set_part_aerodynamic_hull(clusters[1].1, vec![section(Vec3::ZERO)])?;
        let step = |world: &mut PhysicsWorld| {
            if sharded {
                world.step_with_shards(
                    0.01,
                    &mut [Vec::new(), Vec::new()],
                    &super::tests::ScopedThreadExecutor,
                    2,
                )
            } else {
                world.step(0.01)
            }
        };
        assert!(matches!(step(&mut world), Err(StepError::WeldBalance(_))));
        assert_eq!(world.body_count(), 2);
        for (body, _, child, weld) in &clusters {
            assert_eq!(
                world.body(*body).ok_or("body")?.state().linear_velocity(),
                Vec3::ZERO
            );
            assert_eq!(world.part_body(*child), Some(*body));
            assert!(world.weld(*weld).is_some());
        }
        world.set_part_aerodynamic_hull(clusters[1].1, Vec::new())?;
        step(&mut world)?;
        assert_eq!(world.body_count(), 4);
        for (body, _, child, weld) in &clusters {
            assert!(world.weld(*weld).is_none());
            assert_ne!(world.part_body(*child), Some(*body));
        }
        assert_eq!(world.validate_clusters(), Ok(()));
    }
    Ok(())
}

#[test]
fn automatic_weld_failure_splits_for_sequential_and_sharded_steps()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength};
    use crate::joint::JointFrame;
    for sharded in [false, true] {
        let mut world = PhysicsWorld::new();
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let body = world.add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)?;
        let root = world.enable_cluster(body)?;
        let child = world.add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::identity(),
                mass,
                colliders: Vec::new(),
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame::IDENTITY,
                frame_b: JointFrame::IDENTITY,
                strength: WeldStrength::Breakable {
                    max_force: Some(4.0),
                    max_torque: None,
                },
                stiffness: WeldStiffness {
                    linear: 1.0,
                    angular: 1.0,
                },
            },
        )?;
        let weld = world.cluster(body).ok_or("cluster")?.welds()[0];
        world.add_force_source_for_part(child, Vec3::ZERO, Vec3::X * 10.0)?;
        world
            .set_automatic_weld_failure(Some((1e-10, 1e-10)))
            .map_err(|e| format!("{e:?}"))?;
        world.set_part_aerodynamic_hull(root, vec![section(Vec3::ZERO)])?;
        let before = *world.body(body).ok_or("body")?.state();
        let rejected = if sharded {
            world.step_with_shards(
                0.01,
                &mut [Vec::new(), Vec::new()],
                &super::tests::ScopedThreadExecutor,
                2,
            )
        } else {
            world.step(0.01)
        };
        assert!(matches!(rejected, Err(StepError::WeldBalance(_))));
        assert_eq!(*world.body(body).ok_or("body")?.state(), before);
        assert!(world.weld(weld).is_some());
        world.set_part_aerodynamic_hull(root, Vec::new())?;
        if sharded {
            world.step_with_shards(
                0.01,
                &mut [Vec::new(), Vec::new()],
                &super::tests::ScopedThreadExecutor,
                2,
            )?;
        } else {
            world.step(0.01)?;
        }
        assert!(world.weld(weld).is_none());
        let child_body = world.part_body(child).ok_or("child")?;
        assert_ne!(child_body, body);
        assert!(
            (world
                .body(child_body)
                .ok_or("child body")?
                .state()
                .linear_velocity()
                .x
                - 0.05)
                .abs()
                < 1e-12
        );
        assert_eq!(world.validate_clusters(), Ok(()));
    }
    Ok(())
}

#[test]
fn force_free_snapshot_is_certified_but_aerodynamic_snapshot_is_not()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::cluster::balance::{BalanceError, prepare_force_free_inputs, prepare_tree};
    use crate::cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength};
    use crate::joint::JointFrame;
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let body = world.add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)?;
    let root = world.enable_cluster(body)?;
    let child = world.add_cluster_part(
        root,
        ClusterPartDesc {
            local_transform: Transform::identity(),
            mass,
            colliders: Vec::new(),
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        },
        WeldDesc {
            frame_a: JointFrame::IDENTITY,
            frame_b: JointFrame::IDENTITY,
            strength: WeldStrength::Breakable {
                max_force: Some(4.0),
                max_torque: None,
            },
            stiffness: WeldStiffness {
                linear: 1.0,
                angular: 1.0,
            },
        },
    )?;
    world.step(0.01)?;
    let reactions = world
        .force_free_weld_reactions(body, 0.0, 0.0)
        .map_err(|error| format!("{error:?}"))?;
    assert_eq!(reactions.len(), 1);
    assert_eq!(reactions[0].child, child);
    assert_eq!(reactions[0].force, Vec3::ZERO);
    assert_eq!(reactions[0].moment, Vec3::ZERO);
    world.add_force_source_for_part(child, Vec3::ZERO, Vec3::X * 10.0)?;
    let staged = world.with_force_source_updates(|world| {
        let candidates = world
            .pending_weld_failure_candidates(body, 1e-12, 1e-12)
            .map_err(|_| StepError::InvalidDeltaTime)?;
        assert_eq!(candidates.len(), 1);
        assert_eq!(candidates[0].reaction.force, -Vec3::X * 5.0);
        Err::<(), _>(StepError::InvalidDeltaTime)
    });
    assert!(staged.is_err());
    assert!(
        world
            .part_source_weld_failure_candidates(body, 1e-12, 1e-12)
            .map_err(|error| format!("{error:?}"))?
            .is_empty()
    );
    world.step(0.01)?;
    let reactions = world
        .part_source_weld_reactions(body, 1e-12, 1e-12)
        .map_err(|error| format!("{error:?}"))?;
    assert_eq!(reactions[0].force, -Vec3::X * 5.0);
    let candidates = world
        .part_source_weld_failure_candidates(body, 1e-12, 1e-12)
        .map_err(|error| format!("{error:?}"))?;
    assert_eq!(candidates.len(), 1);
    assert!(candidates[0].force_exceeded);
    assert!(!candidates[0].torque_exceeded);
    assert!(world.weld(candidates[0].reaction.weld).is_some());
    assert!(matches!(
        world.force_free_weld_reactions(body, 1e-12, 1e-12),
        Err(BalanceError::IncompleteLoads)
    ));
    let state = world.cluster_load_states()[0];
    let tree = prepare_tree(
        &state,
        world.cluster_load_parts(),
        world.cluster_load_welds(),
    )
    .map_err(|error| format!("{error:?}"))?;
    assert!(matches!(
        prepare_force_free_inputs(&state, &tree),
        Err(BalanceError::IncompleteLoads)
    ));
    world.set_part_aerodynamic_hull(root, vec![section(Vec3::ZERO)])?;
    world.step(0.01)?;
    assert!(matches!(
        world.force_free_weld_reactions(body, 0.0, 0.0),
        Err(BalanceError::IncompleteLoads)
    ));
    assert!(matches!(
        world.part_source_weld_reactions(body, 1e-12, 1e-12),
        Err(BalanceError::IncompleteLoads)
    ));
    assert!(matches!(
        world.part_source_weld_failure_candidates(body, 1e-12, 1e-12),
        Err(BalanceError::IncompleteLoads)
    ));
    let state = world.cluster_load_states()[0];
    assert!(matches!(
        prepare_force_free_inputs(&state, &tree),
        Err(BalanceError::IncompleteLoads)
    ));
    Ok(())
}

#[test]
fn rotating_force_free_cluster_has_centripetal_weld_reaction()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength};
    use crate::joint::JointFrame;
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let mut rigid = RigidBody::new_dynamic(Transform::identity(), mass)?;
    rigid.set_angular_velocity(Vec3::Z)?;
    let body = world.add_body(rigid)?;
    let root = world.enable_cluster(body)?;
    let child = world.add_cluster_part(
        root,
        ClusterPartDesc {
            local_transform: Transform::new(Vec3::X * 2.0, Quat::IDENTITY)?,
            mass,
            colliders: Vec::new(),
            linear_velocity: Vec3::Y * 2.0,
            angular_velocity: Vec3::Z,
        },
        WeldDesc {
            frame_a: JointFrame::new(Vec3::X * 2.0, Quat::IDENTITY),
            frame_b: JointFrame::IDENTITY,
            strength: WeldStrength::Unbreakable,
            stiffness: WeldStiffness {
                linear: 1.0,
                angular: 1.0,
            },
        },
    )?;
    world.step(1e-4)?;
    let reactions = world
        .force_free_weld_reactions(body, 1e-10, 1e-10)
        .map_err(|error| format!("{error:?}"))?;
    assert_eq!(reactions.len(), 1);
    assert_eq!(reactions[0].child, child);
    assert!((reactions[0].force + Vec3::X).length() < 1e-10);
    assert!(reactions[0].moment.length() < 1e-10);
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
