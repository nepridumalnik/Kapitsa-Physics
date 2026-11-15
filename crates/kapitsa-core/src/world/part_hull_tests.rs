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
        cluster::{ClusterPartDesc, WeldDesc, WeldStiffness, WeldStrength},
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
        world.remove_gravity(bad).ok_or("gravity")?;
    }
    assert!((velocities[0] - velocities[1]).length() < 1e-12);
    // By Newton's law, each 1 kg part accelerates at -30 m/s². The cut
    // reaction is 30 N when drag is all on the child, but zero when each
    // part receives 30 N. The current implementation does not evaluate it.
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
            world.remove_gravity(bad).ok_or("gravity")?;
            world.set_atmosphere(crate::aerodynamics::Atmosphere::Vacuum);
            world.step(1.0)?;
            assert!(world.simple_surface_corrections().is_empty());
        }
    }
    Ok(())
}
