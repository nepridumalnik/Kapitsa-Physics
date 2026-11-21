use super::{PhysicsWorld, StepError, shape_binding_tests::cube, test_atmosphere};
use crate::{
    aerodynamics::{
        Aerodynamics, AerodynamicsError, AtmosphericState, MeshPressureModel,
        geometry::GeometryError, pressure::PressureError,
    },
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
};
use kapitsa_math::{Mat3, Vec3};

#[rstest::rstest]
#[case::ordinary(false)]
#[case::sharded(true)]
fn rejected_hull_step_keeps_motion_and_published_pressure(
    #[case] sharded: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    const DT: f64 = 1e-8;
    let mut world = PhysicsWorld::new();
    world.set_atmosphere(test_atmosphere(AtmosphericState::new(1.0, Vec3::ZERO)?));
    world.set_mesh_pressure_model(MeshPressureModel::ModifiedNewtonianTriangleQuadrature);
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let mut moving = RigidBody::new_dynamic(Transform::identity(), mass)?;
    moving.set_linear_velocity(Vec3::Y)?;
    let unrelated = world.add_body(moving)?;
    let mut hull = RigidBody::new_dynamic(Transform::identity(), mass)?;
    hull.set_linear_velocity(Vec3::X * 1500.0)?;
    hull.set_aerodynamics(Aerodynamics::Mesh(cube()?));
    let body = world.add_body(hull)?;
    world.cook_primary_hull(body)?;

    let step = |world: &mut PhysicsWorld| {
        if sharded {
            world.step_with_shards(
                DT,
                &mut [Vec::new(), Vec::new()],
                &super::tests::ScopedThreadExecutor,
                2,
            )
        } else {
            world.step(DT)
        }
    };
    step(&mut world)?;
    let before = world.body(body).ok_or("hull")?.motion();
    let unrelated_before = world.body(unrelated).ok_or("unrelated body")?.motion();
    let published = world.owner_mesh_pressure(body).ok_or("pressure")?.to_vec();
    assert!(published.iter().any(|(_, wrench)| wrench.force.x < 0.0));

    world.set_mesh_sound_speed(200.0)?;
    let error = step(&mut world).err().ok_or("expected Mach rejection")?;
    assert!(
        matches!(
            error.downcast_ref::<StepError>(),
            Some(StepError::Aerodynamics { body_id, cause }) if *body_id == body
                && cause.downcast_ref::<AerodynamicsError>()
                    == Some(&AerodynamicsError::Pressure(PressureError::UnsupportedMach.into()))
        ),
        "{error:?}"
    );
    assert_eq!(world.body(body).ok_or("hull")?.motion(), before);
    assert_eq!(
        world.body(unrelated).ok_or("unrelated body")?.motion(),
        unrelated_before
    );
    assert_eq!(world.owner_mesh_pressure(body), Some(published.as_slice()));

    world.set_mesh_sound_speed(340.0)?;
    // Limit only per-step visibility work: the installed cooked geometry stays valid.
    let prepared = world
        .bodies
        .get_mut(body.dense)
        .ok_or("body entry")?
        .mesh_aerodynamics
        .as_mut()
        .ok_or("mesh settings")?;
    let budget = prepared.config.max_patch_work;
    prepared.config.max_patch_work = 1;
    let error = step(&mut world).err().ok_or("expected budget rejection")?;
    assert!(
        matches!(
            error.downcast_ref::<StepError>(),
            Some(StepError::Aerodynamics { body_id, cause }) if *body_id == body
                && cause.downcast_ref::<AerodynamicsError>()
                    == Some(&AerodynamicsError::Geometry(GeometryError::WorkBudgetExceeded.into()))
        ),
        "{error:?}"
    );
    assert_eq!(world.body(body).ok_or("hull")?.motion(), before);
    assert_eq!(
        world.body(unrelated).ok_or("unrelated body")?.motion(),
        unrelated_before
    );
    assert_eq!(world.owner_mesh_pressure(body), Some(published.as_slice()));

    world
        .bodies
        .get_mut(body.dense)
        .ok_or("body entry")?
        .mesh_aerodynamics
        .as_mut()
        .ok_or("mesh settings")?
        .config
        .max_patch_work = budget;
    step(&mut world)?;
    assert_ne!(world.body(body).ok_or("hull")?.motion(), before);
    Ok(())
}
