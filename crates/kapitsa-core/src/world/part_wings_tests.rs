use super::*;
use crate::{
    aerodynamics::{
        Aerodynamics, AtmosphericState,
        wing::{FiniteWing, WingCoefficients, WingControl, WingPolar, WingSurface},
    },
    cluster::{MassChangeMode, WeldDesc, WeldStiffness, WeldStrength},
    joint::JointFrame,
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
};
use kapitsa_math::{Mat3, Quat};

fn wing(local: Transform) -> Result<WingSurface, crate::Error> {
    WingSurface::new(FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2)?, local, 1.0)
}

#[test]
fn wing_control_ids_survive_deflection_but_not_replacement()
-> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
    )?)?;
    let part = world.enable_cluster(body)?;
    let surface = wing(Transform::identity())?.with_control(WingControl::new(0.0, 0.2, 1.0)?);
    world.set_part_aerodynamic_wings(part, vec![surface.clone()])?;
    let id = world.part_wing_id(part, 0).ok_or("missing wing")?;
    world.set_part_wing_deflection(id, 0.1)?;
    assert_eq!(world.part_wing_id(part, 0), Some(id));
    let revision = world.body_revision;
    assert!(world.set_part_wing_deflection(id, f64::NAN).is_err());
    assert_eq!(world.body_revision, revision);
    world.set_part_aerodynamic_wings(part, vec![surface])?;
    assert_ne!(world.part_wing_id(part, 0), Some(id));
    let error = world
        .set_part_wing_deflection(id, 0.1)
        .err()
        .ok_or("stale ID accepted")?;
    use crate::aerodynamics::wing::WingError;
    assert_eq!(
        error.downcast_ref::<WingError>(),
        Some(&WingError::InvalidWingId)
    );
    assert!(
        PhysicsWorld::new()
            .set_part_wing_deflection(id, 0.1)
            .is_err()
    );
    Ok(())
}
fn weld() -> WeldDesc {
    WeldDesc {
        frame_a: JointFrame::new(Vec3::ZERO, Quat::IDENTITY),
        frame_b: JointFrame::new(Vec3::ZERO, Quat::IDENTITY),
        strength: WeldStrength::Unbreakable,
        stiffness: WeldStiffness {
            linear: 1.0,
            angular: 1.0,
        },
    }
}

#[test]
fn wing_deflection_batch_validates_before_mutating() -> Result<(), Box<dyn std::error::Error>> {
    use crate::command::{BatchScratch, PhysicsCommand, apply_command_batch};
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
    )?)?;
    let part = world.enable_cluster(body)?;
    let surface = wing(Transform::identity())?.with_control(WingControl::new(0.0, 0.2, 1.0)?);
    world.set_part_aerodynamic_wings(part, vec![surface.clone(), surface])?;
    let command = |index, deflection| -> Result<_, Box<dyn std::error::Error>> {
        Ok(PhysicsCommand::SetPartWingDeflection {
            wing: world.part_wing_id(part, index).ok_or("missing wing")?,
            deflection,
        })
    };
    let invalid = [command(0, 0.1)?, command(1, 0.3)?];
    let valid = [command(0, 0.1)?, command(1, -0.1)?];
    let values = |world: &PhysicsWorld| -> Result<Vec<f64>, Box<dyn std::error::Error>> {
        world
            .part_aerodynamic_wings(part)
            .ok_or("missing wings")?
            .iter()
            .map(|surface| Ok(surface.control().ok_or("missing control")?.deflection()))
            .collect()
    };
    let mut scratch = BatchScratch::default();
    assert!(apply_command_batch(&mut world, &invalid, &mut scratch).is_err());
    assert_eq!(values(&world)?, [0.0, 0.0]);
    apply_command_batch(&mut world, &valid, &mut scratch)?;
    assert_eq!(values(&world)?, [0.1, -0.1]);
    Ok(())
}
fn advance(world: &mut PhysicsWorld, sharded: bool) -> Result<(), crate::Error> {
    if sharded {
        world.step_with_shards(
            1e-5,
            &mut [Vec::new(), Vec::new()],
            &crate::execution::SequentialExecutor,
            1,
        )
    } else {
        world.step(1e-5)
    }
}

#[test]
fn glider_ailerons_bank_and_turn_in_free_flight() -> Result<(), Box<dyn std::error::Error>> {
    const DT: f64 = 0.005;
    const STEPS: usize = 200;
    const DEFLECTION: f64 = 0.02;
    // The banked wings see spanwise flow. Use the model's bounded projection.
    const MAX_SIDESLIP: f64 = 0.1;

    let fly = |density, command| -> Result<_, Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(test_atmosphere(AtmosphericState::new(
            density,
            Vec3::X * -5.0,
        )?));
        let mass = MassProperties::new(250.0, Vec3::ZERO, Mat3::IDENTITY * 500.0)?;
        let mut glider = RigidBody::new_dynamic(Transform::identity(), mass)?;
        glider.set_linear_velocity(Vec3::X * 30.0)?;
        let model = FiniteWing::new(8.0, 0.8, 0.02, -0.06, 0.2)?;
        let body = world.add_body(glider)?;
        let part = world.enable_cluster(body)?;

        // The symmetric wings start with positive lift. Apply differential
        // incidence after one neutral step, without changing the rigid state.
        for deflection in [0.0, command] {
            let mut surfaces = Vec::new();
            for side in [-1.0, 1.0] {
                surfaces.push(
                    WingSurface::new(
                        model,
                        Transform::new(Vec3::Y * (2.0 * side), Quat::IDENTITY)?,
                        2.0,
                    )?
                    .with_spanwise_projection(MAX_SIDESLIP)?
                    .with_control(WingControl::new(
                        side * deflection,
                        DEFLECTION,
                        1.0,
                    )?),
                );
            }
            world.set_part_aerodynamic_wings(part, surfaces)?;
            world.step(DT)?;
        }
        for _ in 2..STEPS {
            world.step(DT)?;
        }
        Ok(world.body(body).ok_or("glider body")?.motion())
    };

    let neutral = fly(1.2, 0.0)?;
    let vacuum = fly(0.0, 0.0)?;
    let neutral_state = neutral.state();
    assert!(neutral_state.linear_velocity().z > 0.0);
    assert_eq!(neutral_state.linear_velocity().y, 0.0);
    assert_eq!(neutral_state.angular_velocity().x, 0.0);

    for side in [-1.0, 1.0] {
        let deflection = side * DEFLECTION;
        let controlled = fly(1.2, deflection)?;
        let state = controlled.state();
        let wing_span = state.transform().transform_vector(Vec3::Y);
        assert!(side * wing_span.z > 0.0, "wing span {wing_span:?}");
        assert!(side * state.linear_velocity().y < 0.0);
        assert!(side * state.transform().translation().y < 0.0);
        assert_eq!(fly(0.0, deflection)?, vacuum);
    }
    Ok(())
}

#[test]
fn automatic_weld_failure_uses_part_wing_snapshot() -> Result<(), Box<dyn std::error::Error>> {
    use crate::cluster::ClusterPartDesc;
    for sharded in [false, true] {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(test_atmosphere(AtmosphericState::new(1.2, Vec3::ZERO)?));
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let velocity = Vec3::new(50.0, 0.0, -3.0);
        let mut rigid = RigidBody::new_dynamic(Transform::identity(), mass)?;
        rigid.set_linear_velocity(velocity)?;
        let body = world.add_body(rigid)?;
        let root = world.enable_cluster(body)?;
        let mut description = weld();
        description.strength = WeldStrength::Breakable {
            max_force: Some(1.0),
            max_torque: None,
        };
        let child = world.add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::identity(),
                mass,
                colliders: Vec::new(),
                linear_velocity: velocity,
                angular_velocity: Vec3::ZERO,
            },
            description,
        )?;
        let weld_id = world.cluster(body).ok_or("cluster")?.welds()[0];
        world.set_part_aerodynamic_wings(child, vec![wing(Transform::identity())?])?;
        world
            .set_automatic_weld_failure(Some((1e-8, 1e-8)))
            .map_err(|e| format!("{e:?}"))?;
        advance(&mut world, sharded)?;
        assert!(world.weld(weld_id).is_none());
        assert_ne!(world.part_body(child), Some(body));
        let loads = world.part_wing_wrenches();
        assert_eq!(loads.len(), 1);
        let reactions = world
            .part_aerodynamic_weld_reactions(body, 1e-8, 1e-8)
            .map_err(|e| format!("{e:?}"))?;
        assert!((reactions[0].force + loads[0].wrench.force * 0.5).length() < 1e-8);
        assert_eq!(world.validate_clusters(), Ok(()));
    }
    Ok(())
}

#[test]
fn part_wing_snapshot_sums_force_and_moment_and_reuses_buffers()
-> Result<(), Box<dyn std::error::Error>> {
    for sharded in [false, true] {
        let mut world = PhysicsWorld::new();
        let atmosphere = AtmosphericState::new(1.2, Vec3::ZERO)?;
        world.set_atmosphere(test_atmosphere(atmosphere));
        let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::new(50.0, 0.0, -3.0))?;
        let id = world.add_body(body)?;
        let part = world.enable_cluster(id)?;
        let first = wing(Transform::new(Vec3::Y, Quat::IDENTITY)?)?;
        let second = wing(Transform::new(-Vec3::Y, Quat::IDENTITY)?)?;
        let flow = crate::aerodynamics::pressure::PressureFlow {
            atmosphere,
            sound_speed: world.mesh_sound_speed,
            transform: Transform::identity(),
            center_of_mass: Vec3::ZERO,
            linear_velocity: Vec3::new(50.0, 0.0, -3.0),
            angular_velocity: Vec3::ZERO,
        };
        let a = first.evaluate(flow)?;
        let b = second.evaluate(flow)?;
        world.set_part_aerodynamic_wings(part, vec![first, second])?;
        advance(&mut world, sharded)?;
        let loads = world.part_wing_wrenches();
        assert_eq!(loads.len(), 2);
        assert_eq!(loads[0].wrench, a);
        assert_eq!(loads[1].wrench, b);
        assert_eq!(Some(loads[0].wing), world.part_wing_id(part, 0));
        assert_eq!(Some(loads[1].wing), world.part_wing_id(part, 1));
        assert_ne!(loads[0].wing, loads[1].wing);
        let sum = loads[0].wrench.force + loads[1].wrench.force;
        let moment = loads[0].wrench.moment + loads[1].wrench.moment;
        assert!(moment.length() < 1e-12);
        let velocity = world.body(id).ok_or("body")?.state().linear_velocity();
        assert!((velocity - (flow.linear_velocity + sum * (1e-5 / 100.0))).length() < 1e-12);
        let capacity = world
            .part_wing_buffers
            .first()
            .ok_or("wing buffer")?
            .capacity();
        assert!(capacity >= 2);
        advance(&mut world, sharded)?;
        assert!(
            world
                .part_wing_buffers
                .first()
                .ok_or("reused wing buffer")?
                .capacity()
                >= capacity
        );
    }
    Ok(())
}

#[test]
fn part_wings_match_body_wings_after_mass_merge_split_and_root_deletion()
-> Result<(), Box<dyn std::error::Error>> {
    let mut reference = None;
    for sharded in [false, true] {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(test_atmosphere(AtmosphericState::new(1.2, Vec3::X * 3.0)?));
        let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::new(50.0, 0.0, -3.0))?;
        body.set_angular_velocity(Vec3::Y * 0.1)?;
        let first = world.add_body(RigidBody::from_motion(body.motion()))?;
        let first_part = world.enable_cluster(first)?;
        let mut second_body = RigidBody::new_dynamic(
            Transform::new(
                Vec3::X * 3.0,
                Quat::from_axis_angle(Vec3::Y, 0.03).ok_or("rotation")?,
            )?,
            mass,
        )?;
        second_body.set_linear_velocity(Vec3::new(50.0, 0.0, -3.3))?;
        second_body.set_angular_velocity(Vec3::Y * 0.1)?;
        let second = world.add_body(second_body)?;
        let second_part = world.enable_cluster(second)?;
        let surfaces = vec![
            wing(Transform::new(
                Vec3::X * 2.0,
                Quat::from_axis_angle(Vec3::Y, 0.02).ok_or("rotation")?,
            )?)?,
            wing(Transform::new(
                -Vec3::X,
                Quat::from_axis_angle(Vec3::Y, -0.01).ok_or("rotation")?,
            )?)?,
        ];
        world.set_part_aerodynamic_wings(second_part, surfaces.clone())?;
        let mut connection = weld();
        connection.frame_a = JointFrame::new(
            Vec3::X * 3.0,
            Quat::from_axis_angle(Vec3::Y, 0.03).ok_or("rotation")?,
        );
        world
            .merge_clusters(first, second, connection)
            .map_err(|e| format!("merge: {e}"))?;
        world.set_part_mass(
            first_part,
            MassProperties::new(120.0, Vec3::X, Mat3::IDENTITY * 100.0)?,
            MassChangeMode::KeepVelocityField,
        )?;
        assert_eq!(
            world.part_aerodynamic_wings(second_part),
            Some(surfaces.as_slice())
        );
        // Evaluate while the wing owner has a nonidentity transform in the
        // merged body, against independently composed body-local surfaces.
        let entry = world.bodies.get(first.dense).ok_or("entry")?;
        let local = world
            .cluster_part(second_part)
            .ok_or("part")?
            .local_transform();
        assert_ne!(local, Transform::identity());
        let composed = surfaces
            .iter()
            .map(|s| -> Result<_, Box<dyn std::error::Error>> {
                Ok(wing(local.compose(&s.local_transform())?)?)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mut actual_loads = entry.body.motion();
        let mut expected_loads = actual_loads;
        PhysicsWorld::apply_aerodynamics(
            world.atmosphere(),
            &Aerodynamics::None,
            &[],
            first,
            &mut actual_loads,
            &world.parts,
            AerodynamicsInputs::new(world.mesh_sound_speed, world.mesh_pressure_model)
                .with_cluster(entry.cluster.as_ref()),
        )?;
        PhysicsWorld::apply_aerodynamics(
            world.atmosphere(),
            &Aerodynamics::Wings(composed),
            &[],
            first,
            &mut expected_loads,
            &world.parts,
            AerodynamicsInputs::new(world.mesh_sound_speed, world.mesh_pressure_model),
        )?;
        assert!((actual_loads.forces().force() - expected_loads.forces().force()).length() < 1e-12);
        assert!(
            (actual_loads.forces().torque() - expected_loads.forces().torque()).length() < 1e-12
        );
        let edge = *world
            .cluster(first)
            .ok_or("cluster")?
            .welds()
            .first()
            .ok_or("weld")?;
        world.break_weld(edge).map_err(|e| format!("split: {e}"))?;
        let owner = world.cluster_part(second_part).ok_or("part")?.body();
        let first_owner = world.cluster_part(first_part).ok_or("first owner")?.body();
        world
            .merge_clusters(first_owner, owner, connection)
            .map_err(|e| format!("remerge: {e}"))?;
        world
            .destroy_cluster_part(first_part)
            .map_err(|e| format!("delete root: {e}"))?;
        assert!(world.part_aerodynamic_wings(first_part).is_none());
        let owner = world
            .cluster_part(second_part)
            .ok_or("surviving part")?
            .body();
        let part_local = world
            .cluster_part(second_part)
            .ok_or("part")?
            .local_transform();
        let mut expected = RigidBody::from_motion(world.body(owner).ok_or("body")?.motion());
        // Build an independent body-wing oracle in the resulting body frame.
        let transformed = surfaces
            .iter()
            .map(|surface| {
                wing(part_local.compose(&surface.local_transform())?)
                    .map_err(|e| -> Box<dyn std::error::Error> { Box::new(e) })
            })
            .collect::<Result<Vec<_>, Box<dyn std::error::Error>>>()?;
        expected.set_aerodynamics(Aerodynamics::Wings(transformed));
        let mut oracle = PhysicsWorld::new();
        oracle.set_atmosphere(world.atmosphere());
        let expected_id = oracle.add_body(expected)?;
        advance(&mut oracle, sharded)?;
        advance(&mut world, sharded)?;
        let actual = world.body(owner).ok_or("body")?.motion();
        let expected = oracle.body(expected_id).ok_or("oracle")?.motion();
        assert!(
            (actual.state().linear_velocity() - expected.state().linear_velocity()).length()
                < 1e-12
        );
        assert!(
            (actual.state().angular_velocity() - expected.state().angular_velocity()).length()
                < 1e-12
        );
        if let Some(previous) = reference {
            assert_eq!(actual, previous);
        } else {
            reference = Some(actual);
        }
        world.remove_body(owner).ok_or("remove last body")?;
        assert!(world.part_aerodynamic_wings(second_part).is_none());
    }
    Ok(())
}

#[test]
fn part_wings_sample_application_point_with_rotated_part_and_com_in_vacuum()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::{Atmosphere, DensityProfile, SphericalAtmosphere};
    let mut reference = None;
    for sharded in [false, true] {
        let atmosphere = Atmosphere::Sphere(
            SphericalAtmosphere::builder()
                .reference_radius(5.0)
                .reference_density(1.2)
                .height(5.0)
                .profile(DensityProfile::Exponential { scale_height: 2.0 })
                .wind(Vec3::X * 3.0)
                .build()?,
        );
        let rotation = Quat::from_axis_angle(Vec3::Y, 0.03).ok_or("rotation")?;
        let pose = Transform::new(Vec3::X * 20.0, rotation)?;
        let mut body = RigidBody::new_dynamic(
            pose,
            MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?,
        )?;
        body.set_linear_velocity(Vec3::new(50.0, 0.0, -3.0))?;
        body.set_angular_velocity(Vec3::Y * 0.1)?;
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(atmosphere);
        let id = world.add_body(body)?;
        let root = world.enable_cluster(id)?;
        // Source coordinates remain fixed even when the assembly COM changes.
        let surface = wing(Transform::new(Vec3::X * -13.0, Quat::IDENTITY)?)?
            .with_polar(std::sync::Arc::new(WingPolar::new(
                vec![0.0, 1.2],
                vec![-0.5, 0.5],
                vec![
                    WingCoefficients {
                        lift: 0.5,
                        drag: 0.1
                    };
                    4
                ],
            )?))
            .with_control(WingControl::new(0.1, 0.2, 0.5)?);
        world.set_part_aerodynamic_wings(root, vec![surface.clone()])?;
        world.set_part_mass(
            root,
            MassProperties::new(110.0, Vec3::X, Mat3::IDENTITY * 100.0)?,
            MassChangeMode::KeepVelocityField,
        )?;
        assert_eq!(
            world.part_aerodynamic_wings(root),
            Some(std::slice::from_ref(&surface))
        );
        let body = world.body(id).ok_or("body")?;
        assert_eq!(
            atmosphere
                .state_at(body.world_center_of_mass()?.ok_or("center")?)?
                .density(),
            0.0
        );
        let point = pose.transform_point(surface.local_transform().translation());
        let state = atmosphere.state_at(point)?;
        assert!(state.density() > 0.0);
        let wrench = surface.evaluate(crate::aerodynamics::pressure::PressureFlow {
            atmosphere: state,
            sound_speed: world.mesh_sound_speed,
            transform: pose,
            center_of_mass: body.physics().mass().ok_or("mass")?.center_of_mass(),
            linear_velocity: body.state().linear_velocity(),
            angular_velocity: body.state().angular_velocity(),
        })?;
        let before = body.state().linear_velocity();
        advance(&mut world, sharded)?;
        let snapshot = world.part_wing_wrenches().to_vec();
        assert_eq!(snapshot.len(), 1);
        assert_eq!(snapshot[0].part, root);
        assert_eq!(snapshot[0].body, id);
        assert_eq!(snapshot[0].wrench, wrench);
        let actual = world.body(id).ok_or("body")?.state().linear_velocity();
        assert!((actual - (before + wrench.force * (1e-5 / 110.0))).length() < 1e-12);
        let center = world
            .body(id)
            .ok_or("body")?
            .motion()
            .world_center_of_mass()?
            .ok_or("center")?;
        let bad = world.add_gravity(crate::gravity::CentralGravity::new(center, 1.0)?)?;
        assert!(advance(&mut world, sharded).is_err());
        assert_eq!(world.part_wing_wrenches(), snapshot.as_slice());
        world.remove_gravity(bad).ok_or("gravity")?;
        if let Some(previous) = reference {
            assert_eq!(actual, previous);
        } else {
            reference = Some(actual);
        }
    }
    Ok(())
}

#[test]
fn part_wings_later_failure_rolls_back_and_primary_transfer_rejects_atomically()
-> Result<(), Box<dyn std::error::Error>> {
    for sharded in [false, true] {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(test_atmosphere(AtmosphericState::new(1.2, Vec3::ZERO)?));
        let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
        body.set_linear_velocity(Vec3::new(50.0, 0.0, -3.0))?;
        let good = wing(Transform::identity())?;
        let bad = wing(Transform::new(
            Vec3::ZERO,
            Quat::from_axis_angle(Vec3::Y, 1.0).ok_or("rotation")?,
        )?)?;
        body.set_aerodynamics(Aerodynamics::Wings(vec![good.clone(), bad.clone()]));
        let id = world.add_body(body)?;
        let before = world.body(id).ok_or("body")?.motion();
        assert_eq!(
            world.enable_cluster_with_mass(
                id,
                MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?
            ),
            Err(crate::Error::from(ClusterError::InvalidMass))
        );
        assert_eq!(world.body(id).ok_or("body")?.motion(), before);
        assert!(
            matches!(world.body(id).ok_or("body")?.aerodynamics(), Aerodynamics::Wings(w) if w == &[good.clone(), bad.clone()])
        );
        let root = world.enable_cluster(id)?;
        assert_eq!(
            world.part_aerodynamic_wings(root),
            Some([good.clone(), bad].as_slice())
        );
        assert_eq!(
            advance(&mut world, sharded),
            Err(crate::Error::from(StepError::Aerodynamics {
                body_id: id,
                cause: crate::Error::from(crate::aerodynamics::AerodynamicsError::Wing(
                    crate::Error::from(crate::aerodynamics::wing::WingError::UnsupportedFlow)
                ))
            }))
        );
        assert_eq!(world.body(id).ok_or("body")?.motion(), before);
        world.set_part_aerodynamic_wings(root, vec![good])?;
        advance(&mut world, sharded)?;
        assert!(world.body(id).ok_or("body")?.state().linear_velocity().z > -3.0);
    }
    Ok(())
}
