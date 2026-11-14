use super::*;
use crate::{
    aerodynamics::{
        Aerodynamics, AtmosphericState,
        wing::{FiniteWing, WingSurface},
    },
    cluster::{MassChangeMode, WeldDesc, WeldStiffness, WeldStrength},
    joint::JointFrame,
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
};
use kapitsa_math::{Mat3, Quat};

fn wing(local: Transform) -> Result<WingSurface, crate::aerodynamics::wing::WingError> {
    WingSurface::new(FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2)?, local, 1.0)
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
fn advance(world: &mut PhysicsWorld, sharded: bool) -> Result<(), StepError> {
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
            entry.cluster.as_ref(),
            None,
            None,
            world.mesh_sound_speed,
        )?;
        PhysicsWorld::apply_aerodynamics(
            world.atmosphere(),
            &Aerodynamics::Wings(composed),
            &[],
            first,
            &mut expected_loads,
            &world.parts,
            None,
            None,
            None,
            world.mesh_sound_speed,
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
        let surface = wing(Transform::new(Vec3::X * -13.0, Quat::IDENTITY)?)?;
        world.set_part_aerodynamic_wings(root, vec![surface])?;
        world.set_part_mass(
            root,
            MassProperties::new(110.0, Vec3::X, Mat3::IDENTITY * 100.0)?,
            MassChangeMode::KeepVelocityField,
        )?;
        assert_eq!(
            world.part_aerodynamic_wings(root),
            Some([surface].as_slice())
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
        let actual = world.body(id).ok_or("body")?.state().linear_velocity();
        assert!((actual - (before + wrench.force * (1e-5 / 110.0))).length() < 1e-12);
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
        body.set_aerodynamics(Aerodynamics::Wings(vec![good, bad]));
        let id = world.add_body(body)?;
        let before = world.body(id).ok_or("body")?.motion();
        assert_eq!(
            world.enable_cluster_with_mass(
                id,
                MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?
            ),
            Err(ClusterError::InvalidMass)
        );
        assert_eq!(world.body(id).ok_or("body")?.motion(), before);
        assert!(
            matches!(world.body(id).ok_or("body")?.aerodynamics(), Aerodynamics::Wings(w) if w == &[good, bad])
        );
        let root = world.enable_cluster(id)?;
        assert_eq!(
            world.part_aerodynamic_wings(root),
            Some([good, bad].as_slice())
        );
        assert_eq!(
            advance(&mut world, sharded),
            Err(StepError::Aerodynamics {
                body_id: id,
                cause: crate::aerodynamics::AerodynamicsError::Wing(
                    crate::aerodynamics::wing::WingError::UnsupportedFlow
                )
            })
        );
        assert_eq!(world.body(id).ok_or("body")?.motion(), before);
        world.set_part_aerodynamic_wings(root, vec![good])?;
        advance(&mut world, sharded)?;
        assert!(world.body(id).ok_or("body")?.state().linear_velocity().z > -3.0);
    }
    Ok(())
}
