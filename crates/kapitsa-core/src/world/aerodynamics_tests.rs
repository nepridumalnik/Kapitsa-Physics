use kapitsa_math::{Mat3, Vec3};

use crate::{
    aerodynamics::{AerodynamicProperties, Aerodynamics, AtmosphericState},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::TriangleMesh,
    transform::Transform,
    world::PhysicsWorld,
};

#[test]
fn spherical_vacuum_skips_collider_preparation() -> Result<(), Box<dyn std::error::Error>> {
    use crate::{
        aerodynamics::{Atmosphere, ColliderTessellation, SphericalAtmosphere},
        collider::Collider,
        shape::{Capsule, Shape, Sphere},
    };
    for sharded in [false, true] {
        for geometry in 0..3 {
            // Also exercise a COM outside the atmosphere with an additional
            // surface inside it: skipping primary geometry must not skip surfaces.
            for surface_in_air in [false, true] {
                let mut world = PhysicsWorld::new();
                world.set_atmosphere(Atmosphere::Sphere(
                    SphericalAtmosphere::builder()
                        .reference_radius(5.0)
                        .reference_density(if surface_in_air { 1.0 } else { 0.0 })
                        .height(5.0)
                        .profile(crate::aerodynamics::DensityProfile::Exponential {
                            scale_height: 1.0,
                        })
                        .build()?,
                ));
                let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
                let position = if surface_in_air {
                    Vec3::X * 20.0
                } else {
                    Vec3::ZERO
                };
                let mut body = RigidBody::new_dynamic(
                    Transform::new(position, kapitsa_math::Quat::IDENTITY)?,
                    mass,
                )?;
                body.set_linear_velocity(Vec3::X * 10.0)?;
                body.set_aerodynamics(Aerodynamics::Colliders);
                let id = world.add_body(body)?;
                if geometry != 0 {
                    let shape = if geometry == 1 {
                        Shape::Capsule(Capsule::new(-Vec3::Z, Vec3::Z, 0.5)?)
                    } else {
                        world
                            .set_collider_tessellation(id, ColliderTessellation::new(12, 6, 1)?)?;
                        Shape::Sphere(Sphere::new(0.5)?)
                    };
                    world.add_collider(id, Collider::new(shape, Transform::identity()))?;
                }
                if surface_in_air {
                    world.add_aerodynamic_surface(
                        id,
                        AerodynamicProperties::new(1.0, 1.0, -Vec3::X * 20.0)?,
                    )?;
                }
                if sharded {
                    world.step_with_shards(
                        0.01,
                        &mut [Vec::new()],
                        &crate::execution::SequentialExecutor,
                        1,
                    )?;
                } else {
                    world.step(0.01)?;
                }
                let body = world.body(id).ok_or("body")?;
                if surface_in_air {
                    assert!(body.state().linear_velocity().x < 10.0);
                } else {
                    assert_eq!(body.state().linear_velocity(), Vec3::X * 10.0);
                    assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
                }
                let entry = world.bodies.get(id.dense).ok_or("entry")?;
                assert!(entry.collider_aerodynamic_mesh.is_none());
                assert!(entry.mesh_aerodynamics.is_none());
            }
        }
    }
    Ok(())
}

#[test]
fn vacuum_skips_aerodynamics_in_both_world_paths() {
    for sharded in [false, true] {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::X * (-0.75 * f64::MAX))
            .unwrap();
        let mut world = PhysicsWorld::new();
        let id = world
            .add_body(RigidBody::from_motion(body.motion()))
            .unwrap();
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(0.0, Vec3::X * (0.75 * f64::MAX)).unwrap(),
        ));
        world
            .set_aerodynamics(
                id,
                Aerodynamics::Simple(AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap()),
            )
            .unwrap();
        if sharded {
            world
                .step_with_shards(
                    1e-310,
                    &mut [Vec::new()],
                    &crate::execution::SequentialExecutor,
                    1,
                )
                .unwrap();
        } else {
            world.step(1e-310).unwrap();
        }
        assert_eq!(
            world.body(id).unwrap().state().linear_velocity(),
            body.state().linear_velocity()
        );
    }
}

fn lift_trajectory(dt: f64, steps: usize, sharded: bool, drag: f64) -> Vec3 {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::X * 10.0).unwrap();
    let mut world = PhysicsWorld::new();
    let id = world.add_body(body).unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    world
        .set_aerodynamics(
            id,
            Aerodynamics::Simple(
                AerodynamicProperties::new(1.0, drag, Vec3::ZERO)
                    .unwrap()
                    .with_lift(1.0, Vec3::Y)
                    .unwrap(),
            ),
        )
        .unwrap();
    for _ in 0..steps {
        if sharded {
            world
                .step_with_shards(
                    dt,
                    &mut [Vec::new()],
                    &crate::execution::SequentialExecutor,
                    1,
                )
                .unwrap();
        } else {
            world.step(dt).unwrap();
        }
    }
    world.body(id).unwrap().state().linear_velocity()
}

#[test]
fn lift_preserves_energy_over_many_steps_in_both_paths() {
    for sharded in [false, true] {
        let velocity = lift_trajectory(0.01, 1000, sharded, 0.0);
        assert!((velocity.length() - 10.0).abs() < 1e-10, "{velocity:?}");
        let with_drag = lift_trajectory(0.1, 10, sharded, 0.2);
        assert!(with_drag.length() <= 10.0 + 1e-10, "{with_drag:?}");
    }
}

#[test]
fn lift_trajectory_converges_as_step_shrinks() {
    let reference = lift_trajectory(0.0001, 5000, false, 0.0);
    let coarse = (lift_trajectory(0.02, 25, false, 0.0) - reference).length();
    let fine = (lift_trajectory(0.01, 50, false, 0.0) - reference).length();
    assert!(fine < coarse * 0.6, "coarse={coarse}, fine={fine}");
}

#[test]
fn offset_lift_preserves_energy_with_rotation_and_wind() {
    for sharded in [false, true] {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::X * 12.0).unwrap();
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        let wind = Vec3::X * 2.0;
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.0, wind).unwrap(),
        ));
        world
            .set_aerodynamics(
                id,
                Aerodynamics::Simple(
                    AerodynamicProperties::new(1.0, 0.0, Vec3::X)
                        .unwrap()
                        .with_lift(0.1, Vec3::Y)
                        .unwrap(),
                ),
            )
            .unwrap();
        for _ in 0..2000 {
            if sharded {
                world
                    .step_with_shards(
                        0.001,
                        &mut [Vec::new()],
                        &crate::execution::SequentialExecutor,
                        1,
                    )
                    .unwrap();
            } else {
                world.step(0.001).unwrap();
            }
        }
        let state = world.body(id).unwrap().state();
        let relative = state.linear_velocity() - wind;
        let metric =
            relative.dot(relative) + state.angular_velocity().dot(state.angular_velocity());
        assert!((metric - 100.0).abs() < 1e-8, "{metric}");
    }
}

#[test]
fn aerodynamic_task_checks_without_mutating_the_world() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::X * 1e200).unwrap();
    let mut world = PhysicsWorld::new();
    let id = world
        .add_body(RigidBody::from_motion(body.motion()))
        .unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    assert!(
        crate::aerodynamics::AerodynamicsTask
            .execute(&world)
            .is_ok()
    );
    world
        .set_aerodynamics(
            id,
            Aerodynamics::Simple(AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap()),
        )
        .unwrap();
    assert!(
        crate::aerodynamics::AerodynamicsTask
            .execute(&world)
            .is_err()
    );
    assert_eq!(world.body(id).unwrap().motion(), body.motion());
}

#[test]
fn lift_energy_drift_converges_for_anisotropic_inertia() {
    fn drift(dt: f64, steps: usize) -> f64 {
        let inertia = Mat3::from_diagonal(Vec3::new(1.0, 1.5, 2.0));
        let mass = MassProperties::new(1.0, Vec3::ZERO, inertia).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::X * 5.0).unwrap();
        body.set_angular_velocity(Vec3::new(0.5, 0.3, 0.2)).unwrap();
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
        ));
        world
            .set_aerodynamics(
                id,
                Aerodynamics::Simple(
                    AerodynamicProperties::new(0.2, 0.0, Vec3::X)
                        .unwrap()
                        .with_lift(0.3, Vec3::Y)
                        .unwrap(),
                ),
            )
            .unwrap();
        let energy = |world: &PhysicsWorld| {
            let state = world.body(id).unwrap().state();
            let omega = state
                .transform()
                .rotation()
                .conjugate()
                .rotate_vector(state.angular_velocity())
                .unwrap();
            state.linear_velocity().dot(state.linear_velocity()) + omega.dot(inertia * omega)
        };
        let initial = energy(&world);
        for _ in 0..steps {
            world.step(dt).unwrap();
        }
        (energy(&world) - initial).abs()
    }
    let coarse = drift(0.01, 100);
    let fine = drift(0.005, 200);
    assert!(
        fine < coarse * 0.7 && fine < 1e-3,
        "coarse={coarse}, fine={fine}"
    );
}

#[test]
fn atmosphere_without_surfaces_does_not_evaluate_relative_wind() {
    let mut results = Vec::new();
    for sharded in [false, true] {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::X * (-f64::MAX * 0.75))
            .unwrap();
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.0, Vec3::X * (f64::MAX * 0.75)).unwrap(),
        ));
        if sharded {
            world
                .step_with_shards(
                    1e-310,
                    &mut [Vec::new()],
                    &crate::execution::SequentialExecutor,
                    1,
                )
                .unwrap();
        } else {
            world.step(1e-310).unwrap();
        }
        results.push(world.body(id).unwrap().motion());
    }
    assert_eq!(results[0].state(), results[1].state());
}

#[test]
fn aerodynamic_drag_is_applied_during_world_step() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    world
        .set_aerodynamics(
            body_id,
            Aerodynamics::Simple(AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap()),
        )
        .unwrap();

    world.step(0.1).unwrap();

    assert_eq!(
        world.body(body_id).unwrap().state().linear_velocity(),
        Vec3::new(5.0, 0.0, 0.0)
    );
}

#[test]
fn aerodynamic_drag_produces_torque_at_an_offset_point() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    world
        .set_aerodynamics(
            body_id,
            Aerodynamics::Simple(AerodynamicProperties::new(2.0, 0.5, Vec3::Y).unwrap()),
        )
        .unwrap();

    world.step(0.1).unwrap();

    let angular_velocity = world.body(body_id).unwrap().state().angular_velocity();
    assert!(angular_velocity.z > 0.0);
}

#[test]
fn body_can_combine_multiple_aerodynamic_surfaces() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    let surface = AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap();
    assert_eq!(world.add_aerodynamic_surface(body_id, surface).unwrap(), 0);
    assert_eq!(world.add_aerodynamic_surface(body_id, surface).unwrap(), 1);
    assert_eq!(world.aerodynamic_surfaces(body_id).unwrap().len(), 2);

    world.step(0.1).unwrap();

    assert_eq!(
        world.body(body_id).unwrap().state().linear_velocity(),
        Vec3::ZERO
    );
}

#[test]
fn strong_drag_does_not_reverse_linear_velocity_in_one_step() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    world
        .set_aerodynamics(
            body_id,
            Aerodynamics::Simple(AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap()),
        )
        .unwrap();

    world.step(0.5).unwrap();

    assert_eq!(
        world.body(body_id).unwrap().state().linear_velocity(),
        Vec3::ZERO
    );
}

#[test]
fn ordinary_and_sharded_steps_match_with_aerodynamics() {
    let make_world = || {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
        body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();
        let mut world = PhysicsWorld::new();
        let body_id = world.add_body(body).unwrap();
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
        ));
        world
            .set_aerodynamics(
                body_id,
                Aerodynamics::Simple(AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap()),
            )
            .unwrap();
        (world, body_id)
    };

    let (mut ordinary, ordinary_id) = make_world();
    let (mut sharded, sharded_id) = make_world();
    ordinary.step(0.1).unwrap();
    sharded
        .step_with_shards(
            0.1,
            &mut [Vec::new(), Vec::new()],
            &crate::execution::SequentialExecutor,
            2,
        )
        .unwrap();

    assert_eq!(
        ordinary.body(ordinary_id).unwrap().motion(),
        sharded.body(sharded_id).unwrap().motion()
    );
}

#[test]
fn failed_step_restores_body_after_aerodynamics_and_gravity_preparation() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::from_diagonal(Vec3::ONE)).unwrap();
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    body.set_linear_velocity(Vec3::new(10.0, 0.0, 0.0)).unwrap();

    let mut world = PhysicsWorld::new();
    let body_id = world.add_body(body).unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    world
        .set_aerodynamics(
            body_id,
            Aerodynamics::Simple(AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap()),
        )
        .unwrap();
    world.set_central_gravity(Some(
        crate::gravity::CentralGravity::new(Vec3::ZERO, 1.0).unwrap(),
    ));
    let before = world.body(body_id).unwrap().motion();

    assert!(world.step(0.1).is_err());
    assert_eq!(world.body(body_id).unwrap().motion(), before);
}

#[test]
fn wind_accelerates_a_stationary_body() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).unwrap())
        .unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::X * 10.0).unwrap(),
    ));
    world
        .set_aerodynamics(
            body,
            Aerodynamics::Simple(AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap()),
        )
        .unwrap();
    world.step(0.5).unwrap();
    let velocity = world.body(body).unwrap().state().linear_velocity().x;
    assert!(velocity > 0.0 && velocity <= 10.0);
}

#[test]
fn rotating_body_experiences_drag_at_an_offset() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut rigid_body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    rigid_body.set_angular_velocity(Vec3::Z).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world.add_body(rigid_body).unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    world
        .set_aerodynamics(
            body,
            Aerodynamics::Simple(AerodynamicProperties::new(1.0, 1.0, Vec3::X).unwrap()),
        )
        .unwrap();
    world.step(0.1).unwrap();
    assert!(world.body(body).unwrap().state().angular_velocity().z < 1.0);
}

#[test]
fn many_surfaces_cannot_reverse_motion_under_large_drag() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut rigid_body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
    rigid_body.set_linear_velocity(Vec3::X * 10.0).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world.add_body(rigid_body).unwrap();
    world.set_atmosphere(super::test_atmosphere(
        AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
    ));
    for _ in 0..3 {
        world
            .add_aerodynamic_surface(
                body,
                AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap(),
            )
            .unwrap();
    }
    world.step(0.5).unwrap();
    let velocity = world.body(body).unwrap().state().linear_velocity().x;
    assert!((-1e-12..=10.0).contains(&velocity));
}

#[test]
fn geometry_modes_preserve_mesh_in_vacuum_and_reject_air_atomically() {
    use crate::{aerodynamics::AerodynamicsError, shape::TriangleMesh, world::StepError};

    for sharded in [false, true] {
        for mesh_mode in [false, true] {
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
            assert!(matches!(body.aerodynamics(), Aerodynamics::None));
            body.set_linear_velocity(Vec3::X).unwrap();
            let mesh =
                TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]).unwrap();
            let pointer = mesh.vertices().as_ptr();
            body.set_aerodynamics(if mesh_mode {
                Aerodynamics::Mesh(mesh)
            } else {
                Aerodynamics::Colliders
            });
            let mut world = PhysicsWorld::new();
            let id = world.add_body(body).unwrap();
            if sharded {
                world
                    .step_with_shards(
                        0.1,
                        &mut vec![Vec::new(); 2],
                        &crate::execution::SequentialExecutor,
                        1,
                    )
                    .unwrap();
            } else {
                world.step(0.1).unwrap();
            }
            if mesh_mode {
                let Aerodynamics::Mesh(mesh) = world.body(id).unwrap().aerodynamics() else {
                    panic!("vacuum step discarded mesh");
                };
                assert_eq!(mesh.vertices().as_ptr(), pointer);
            }
            world.set_atmosphere(super::test_atmosphere(
                AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
            ));
            let before = world.body(id).unwrap().motion();
            let result = if sharded {
                world.step_with_shards(
                    0.1,
                    &mut vec![Vec::new(); 2],
                    &crate::execution::SequentialExecutor,
                    1,
                )
            } else {
                world.step(0.1)
            };
            if mesh_mode {
                assert_eq!(result, Ok(()));
            } else {
                assert_eq!(
                    result,
                    Err(StepError::Aerodynamics {
                        body_id: id,
                        cause: AerodynamicsError::UnsupportedModel,
                    })
                );
                assert_eq!(world.body(id).unwrap().motion(), before);
            }
        }
    }
}

#[test]
fn failed_surface_removal_does_not_invalidate_prepared_motion() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut world = PhysicsWorld::new();
    let id = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).unwrap())
        .unwrap();
    let mut shards = vec![Vec::new()];
    world.fill_step_shards(&mut shards).unwrap();
    shards[0][0].integrate(0.1).unwrap();
    assert_eq!(
        world.remove_aerodynamic_surface(id, usize::MAX),
        Err(crate::world::WorldError::InvalidAerodynamicSurface)
    );
    assert!(world.commit_step_shards(&shards).is_ok());
}

#[test]
fn changing_aerodynamics_invalidates_prepared_motion() {
    use crate::world::StepError;

    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut world = PhysicsWorld::new();
    let id = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).unwrap())
        .unwrap();
    let mut shards = vec![Vec::new()];
    world.fill_step_shards(&mut shards).unwrap();
    shards[0][0].integrate(0.1).unwrap();
    let before = world.body(id).unwrap().motion();
    world.set_aerodynamics(id, Aerodynamics::Colliders).unwrap();
    assert!(matches!(
        world.commit_step_shards(&shards),
        Err(StepError::InvalidStepResults)
    ));
    assert_eq!(world.body(id).unwrap().motion(), before);
    assert!(matches!(
        world.body(id).unwrap().aerodynamics(),
        Aerodynamics::Colliders
    ));
}

#[test]
fn cluster_rejects_unsupported_geometry_without_mutating_state() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).unwrap())
        .unwrap();
    let part = world.enable_cluster(body).unwrap();
    let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]).unwrap();
    assert!(matches!(
        world.body(body).unwrap().aerodynamics(),
        Aerodynamics::None
    ));
    assert_eq!(
        world.set_aerodynamics(body, Aerodynamics::Mesh(mesh)),
        Err(crate::world::WorldError::UnsupportedAerodynamics)
    );
    assert!(matches!(
        world.body(body).unwrap().aerodynamics(),
        Aerodynamics::None
    ));
    assert!(world.cluster(body).is_some());
    assert_eq!(world.cluster_part(part).unwrap().body(), body);
    assert_eq!(
        world.set_aerodynamics(body, Aerodynamics::Colliders),
        Err(crate::world::WorldError::UnsupportedAerodynamics)
    );
}

#[test]
fn collider_pressure_cache_invalidation_and_sharded_parity()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::{
        aerodynamics::ColliderTessellation,
        collider::Collider,
        shape::{Cuboid, Shape, Sphere},
    };
    let mut outcomes = Vec::new();
    for sharded in [false, true] {
        for sphere in [false, true] {
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::test_atmosphere(AtmosphericState::new(
                0.001,
                Vec3::ZERO,
            )?));
            let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::X * 2000.0)?;
            body.set_aerodynamics(Aerodynamics::Colliders);
            let id = world.add_body(body)?;
            let shape = if sphere {
                Shape::Sphere(Sphere::new(0.5)?)
            } else {
                Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0)?)
            };
            let collider = world.add_collider(
                id,
                Collider::new(
                    shape,
                    Transform::new(Vec3::Y, kapitsa_math::Quat::IDENTITY)?,
                ),
            )?;
            let step = |world: &mut PhysicsWorld| {
                if sharded {
                    world.step_with_shards(
                        0.00001,
                        &mut [Vec::new()],
                        &crate::execution::SequentialExecutor,
                        1,
                    )
                } else {
                    world.step(0.00001)
                }
            };
            step(&mut world)?;
            let motion = world.body(id).ok_or("body")?.motion();
            assert!(motion.state().linear_velocity().x < 2000.0);
            assert!(motion.state().angular_velocity().z > 0.0);
            let entry = world.bodies.get(id.dense).ok_or("entry")?;
            let pointer = entry
                .collider_aerodynamic_mesh
                .as_ref()
                .ok_or("mesh")?
                .vertices()
                .as_ptr();
            let geometry = entry
                .mesh_aerodynamics
                .as_ref()
                .ok_or("cache")?
                .cache
                .last_prepared()
                .ok_or("prepared")?
                .1 as *const _;
            let unrelated = world.add_body(RigidBody::new_static(Transform::identity()))?;
            world.set_aerodynamics(unrelated, Aerodynamics::None)?;
            let current_transform = world.body(id).ok_or("body")?.state().transform();
            world
                .body_mut(id)
                .ok_or("body")?
                .set_transform(current_transform)?;
            world
                .body_mut(id)
                .ok_or("body")?
                .set_mass(MassProperties::new(
                    100.0,
                    Vec3::Y * 0.1,
                    Mat3::IDENTITY * 100.0,
                )?)?;
            world.body_mut(id).ok_or("body")?.set_mass(mass)?;
            step(&mut world)?;
            let entry = world.bodies.get(id.dense).ok_or("entry")?;
            assert_eq!(
                pointer,
                entry
                    .collider_aerodynamic_mesh
                    .as_ref()
                    .ok_or("mesh")?
                    .vertices()
                    .as_ptr()
            );
            assert_eq!(
                geometry,
                entry
                    .mesh_aerodynamics
                    .as_ref()
                    .ok_or("cache")?
                    .cache
                    .last_prepared()
                    .ok_or("prepared")?
                    .1 as *const _
            );
            let previous_identity = world
                .bodies
                .get(id.dense)
                .ok_or("entry")?
                .mesh_aerodynamics
                .as_ref()
                .ok_or("cache")?
                .identity
                .clone()
                .ok_or("identity")?;
            let mut replacement = RigidBody::from_motion(world.body(id).ok_or("body")?.motion());
            replacement.set_aerodynamics(Aerodynamics::Colliders);
            world.replace_body_state(id, replacement)?;
            step(&mut world)?;
            let current_identity = world
                .bodies
                .get(id.dense)
                .ok_or("entry")?
                .mesh_aerodynamics
                .as_ref()
                .ok_or("cache")?
                .identity
                .as_ref()
                .ok_or("identity")?;
            assert!(!std::sync::Arc::ptr_eq(
                &previous_identity,
                current_identity
            ));
            outcomes.push(world.body(id).ok_or("body")?.motion());
            world.set_collider_tessellation(id, ColliderTessellation::new(12, 6, 1024)?)?;
            assert!(
                world
                    .bodies
                    .get(id.dense)
                    .ok_or("entry")?
                    .collider_aerodynamic_mesh
                    .is_none()
            );
            step(&mut world)?;
            let _ = world.collider_mut(collider).ok_or("collider")?;
            assert!(
                world
                    .bodies
                    .get(id.dense)
                    .ok_or("entry")?
                    .mesh_aerodynamics
                    .is_none()
            );
            step(&mut world)?;
            world.remove_collider(collider).ok_or("remove")?;
            let before = world.body(id).ok_or("body")?.motion();
            assert!(step(&mut world).is_err());
            assert_eq!(before, world.body(id).ok_or("body")?.motion());
            world.add_collider(
                id,
                Collider::new(
                    Shape::Capsule(crate::shape::Capsule::new(-Vec3::Z, Vec3::Z, 0.5)?),
                    Transform::identity(),
                ),
            )?;
            assert_eq!(
                step(&mut world),
                Err(crate::world::StepError::Aerodynamics {
                    body_id: id,
                    cause: crate::aerodynamics::AerodynamicsError::UnsupportedModel
                })
            );
            assert_eq!(before, world.body(id).ok_or("body")?.motion());
        }
    }
    assert_eq!(outcomes[0], outcomes[2]);
    assert_eq!(outcomes[1], outcomes[3]);
    Ok(())
}

#[test]
fn changing_body_aerodynamics_none_preserves_part_surfaces() {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).unwrap())
        .unwrap();
    let part = world.enable_cluster(body).unwrap();
    let properties = AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap();
    world
        .add_aerodynamic_surface_for_part(part, properties)
        .unwrap();
    world
        .set_aerodynamics(body, Aerodynamics::Simple(properties))
        .unwrap();
    let surfaces = world.aerodynamic_surfaces(body).unwrap();
    assert_eq!(surfaces.len(), 1);
    assert_eq!(
        surfaces[0].target(),
        crate::aerodynamics::AerodynamicTarget::Part(part)
    );
    assert!(matches!(
        world.body(body).unwrap().aerodynamics(),
        Aerodynamics::Simple(value) if *value == properties
    ));
    world.set_aerodynamics(body, Aerodynamics::None).unwrap();
    let surfaces = world.aerodynamic_surfaces(body).unwrap();
    assert_eq!(surfaces.len(), 1);
    assert_eq!(
        surfaces[0].target(),
        crate::aerodynamics::AerodynamicTarget::Part(part)
    );
}

#[test]
fn sphere_samples_surface_position_in_both_world_paths() -> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::{Atmosphere, DensityProfile, SphericalAtmosphere};
    for sharded in [false, true] {
        for offset in [0.0, 20.0] {
            let mut world = PhysicsWorld::new();
            assert_eq!(world.atmosphere(), Atmosphere::Vacuum);
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
            body.set_linear_velocity(Vec3::X * 10.0)?;
            let id = world.add_body(body)?;
            let sphere = SphericalAtmosphere::builder()
                .reference_radius(5.0)
                .reference_density(1.0)
                .height(5.0)
                .profile(DensityProfile::Exponential { scale_height: 1.0 })
                .build()?;
            world.set_atmosphere(Atmosphere::Sphere(sphere));
            world.add_aerodynamic_surface(
                id,
                AerodynamicProperties::new(1.0, 1.0, Vec3::X * offset)?,
            )?;
            if sharded {
                world.step_with_shards(
                    0.01,
                    &mut [Vec::new()],
                    &crate::execution::SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(0.01)?;
            }
            let body = world.body(id).ok_or("missing body")?;
            if offset == 0.0 {
                assert!(body.state().linear_velocity().x < 10.0);
            } else {
                assert_eq!(body.state().linear_velocity(), Vec3::X * 10.0);
            }
            world.set_atmosphere(Atmosphere::Vacuum);
            assert_eq!(world.atmosphere(), Atmosphere::Vacuum);
        }
    }
    Ok(())
}

#[test]
fn mesh_sound_speed_rejects_invalid_changes() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new();
    world.set_mesh_sound_speed(123.0)?;
    for invalid in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert_eq!(
            world.set_mesh_sound_speed(invalid),
            Err(crate::aerodynamics::pressure::PressureError::InvalidSoundSpeed)
        );
        assert_eq!(world.mesh_sound_speed(), 123.0);
    }
    Ok(())
}
