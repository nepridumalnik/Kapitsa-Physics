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
fn altitude_wind_profile_changes_world_drag() {
    use crate::aerodynamics::{Atmosphere, DensityProfile, SphericalAtmosphere};
    let mut world = PhysicsWorld::new();
    world.set_atmosphere(Atmosphere::Sphere(
        SphericalAtmosphere::builder()
            .reference_radius(100.0)
            .reference_density(1.0)
            .height(100.0)
            .profile(DensityProfile::Exponential {
                scale_height: 1000.0,
            })
            .wind_at_top(Vec3::X * 10.0)
            .build()
            .unwrap(),
    ));
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    let mut ids = Vec::new();
    for altitude in [100.0, 150.0] {
        let transform = Transform::new(Vec3::Y * altitude, kapitsa_math::Quat::IDENTITY).unwrap();
        let id = world
            .add_body(RigidBody::new_dynamic(transform, mass).unwrap())
            .unwrap();
        world
            .set_aerodynamics(
                id,
                Aerodynamics::Simple(AerodynamicProperties::new(1.0, 1.0, Vec3::ZERO).unwrap()),
            )
            .unwrap();
        ids.push(id);
    }
    world.step(0.1).unwrap();
    assert_eq!(
        world.body(ids[0]).unwrap().state().linear_velocity(),
        Vec3::ZERO
    );
    assert!(world.body(ids[1]).unwrap().state().linear_velocity().x > 0.0);
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
fn mesh_uses_atmosphere_sound_speed_in_world_paths() {
    use crate::aerodynamics::{Atmosphere, DensityProfile, SphericalAtmosphere};
    use crate::shape::TriangleMesh;
    for sharded in [false, true] {
        for cluster in [false, true] {
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
            body.set_linear_velocity(Vec3::X * 100.0).unwrap();
            body.set_aerodynamics(Aerodynamics::Mesh(
                TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap(),
            ));
            let mut world = PhysicsWorld::new();
            let id = world.add_body(body).unwrap();
            if cluster {
                world.enable_cluster(id).unwrap();
            }
            world.set_atmosphere(Atmosphere::Sphere(
                SphericalAtmosphere::builder()
                    .reference_radius(100.0)
                    .reference_density(1.0)
                    .height(100.0)
                    .profile(DensityProfile::Exponential {
                        scale_height: 100.0,
                    })
                    .sound_speed_profile(10.0, 20.0)
                    .build()
                    .unwrap(),
            ));
            let result = if sharded {
                world.step_with_shards(
                    0.001,
                    &mut vec![Vec::new(); 2],
                    &crate::execution::SequentialExecutor,
                    1,
                )
            } else {
                world.step(0.001)
            };
            assert!(result.is_ok(), "{result:?}");
            assert!(world.body(id).unwrap().state().linear_velocity().x < 100.0);
        }
    }
}

#[test]
fn mesh_pressure_errors_preserve_cause_and_motion_in_all_world_paths() {
    use crate::{
        aerodynamics::{AerodynamicsError, pressure::PressureError},
        shape::TriangleMesh,
        world::StepError,
    };
    for sharded in [false, true] {
        for cluster in [false, true] {
            let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
            let mut body = RigidBody::new_dynamic(Transform::identity(), mass).unwrap();
            body.set_linear_velocity(Vec3::X * 100.0).unwrap();
            body.set_aerodynamics(Aerodynamics::Mesh(
                TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]).unwrap(),
            ));
            let mut world = PhysicsWorld::new();
            let id = world.add_body(body).unwrap();
            if cluster {
                world.enable_cluster(id).unwrap();
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
            assert_eq!(
                result,
                Err(StepError::Aerodynamics {
                    body_id: id,
                    cause: AerodynamicsError::Pressure(PressureError::UnsupportedMach),
                })
            );
            assert_eq!(world.body(id).unwrap().motion(), before);
            let error = AerodynamicsError::Pressure(PressureError::UnsupportedMach);
            assert_eq!(
                std::error::Error::source(&error).unwrap().to_string(),
                PressureError::UnsupportedMach.to_string()
            );
        }
    }
}

#[test]
fn wing_world_loads_and_failures_match_between_step_paths() {
    use crate::aerodynamics::{
        AerodynamicsError,
        wing::{FiniteWing, WingError, WingSurface},
    };
    let mut reference = None;
    for sharded in [false, true] {
        for speed in [50.0, 500.0] {
            let mut body = RigidBody::new_dynamic(
                Transform::identity(),
                MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
            )
            .unwrap();
            body.set_linear_velocity(Vec3::new(speed, 0.0, -speed * 0.1))
                .unwrap();
            body.set_aerodynamics(Aerodynamics::Wing(
                WingSurface::new(
                    FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2).unwrap(),
                    Transform::identity(),
                    2.0,
                )
                .unwrap(),
            ));
            let mut world = PhysicsWorld::new();
            let id = world.add_body(body).unwrap();
            world.set_atmosphere(super::test_atmosphere(
                AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
            ));
            let before = world.body(id).unwrap().motion();
            let result = if sharded {
                world.step_with_shards(
                    1e-4,
                    &mut vec![Vec::new(); 2],
                    &crate::execution::SequentialExecutor,
                    1,
                )
            } else {
                world.step(1e-4)
            };
            if speed == 50.0 {
                result.unwrap();
                let velocity = world.body(id).unwrap().state().linear_velocity();
                assert!(velocity.z > -5.0);
                // Lift can have a positive X component while descending;
                // drag must remove energy, not necessarily reduce world X.
                assert!(velocity.length_squared() < Vec3::new(50.0, 0.0, -5.0).length_squared());
                let motion = world.body(id).unwrap().motion();
                if let Some(expected) = reference {
                    assert_eq!(motion, expected);
                } else {
                    reference = Some(motion);
                }
            } else {
                assert_eq!(
                    result,
                    Err(super::StepError::Aerodynamics {
                        body_id: id,
                        cause: AerodynamicsError::Wing(WingError::UnsupportedFlow)
                    })
                );
                assert_eq!(world.body(id).unwrap().motion(), before);
            }
        }
    }
}

#[test]
fn wing_trajectory_converges_with_timestep() {
    use crate::aerodynamics::wing::{FiniteWing, WingSurface};
    let simulate = |steps: usize| {
        let mut body = RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
        )
        .unwrap();
        body.set_linear_velocity(Vec3::new(50.0, 0.0, -5.0))
            .unwrap();
        body.set_aerodynamics(Aerodynamics::Wing(
            WingSurface::new(
                FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2).unwrap(),
                Transform::identity(),
                2.0,
            )
            .unwrap(),
        ));
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
        ));
        for _ in 0..steps {
            world.step(0.1 / steps as f64).unwrap();
        }
        let velocity = world.body(id).unwrap().state().linear_velocity();
        assert!(velocity.length_squared() < 2525.0);
        velocity
    };
    let reference = simulate(1024);
    let coarse = (simulate(16) - reference).length();
    let medium = (simulate(32) - reference).length();
    let fine = (simulate(64) - reference).length();
    assert!(medium < coarse * 0.6, "{coarse} {medium}");
    assert!(fine < medium * 0.6, "{medium} {fine}");
}

fn hull_section(point: Vec3) -> crate::aerodynamics::hull::HullSection {
    crate::aerodynamics::hull::HullSection::new(
        Transform::new(point, kapitsa_math::Quat::IDENTITY).unwrap(),
        Vec3::X,
        2.0,
        1.0,
        2.0,
        3.0,
        0.2,
        0.8,
        None,
    )
    .unwrap()
}

#[test]
fn hull_world_parity_and_cluster_transfer_preserve_model() {
    let mut reference = None;
    for sharded in [false, true] {
        let sections = vec![hull_section(-Vec3::Y * 2.0), hull_section(Vec3::Y * 2.0)];
        let mut body = RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
        )
        .unwrap();
        body.set_linear_velocity(Vec3::X * 50.0).unwrap();
        body.set_aerodynamics(Aerodynamics::Hull(sections.clone()));
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        let before = world.body(id).unwrap().motion();
        let root = world.enable_cluster(id).unwrap();
        assert_eq!(world.body(id).unwrap().motion(), before);
        assert_eq!(world.part_aerodynamic_hull(root), Some(sections.as_slice()));
        assert!(matches!(
            world.body(id).unwrap().aerodynamics(),
            Aerodynamics::None
        ));
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
        ));
        if sharded {
            world
                .step_with_shards(
                    1e-4,
                    &mut vec![Vec::new(); 2],
                    &crate::execution::SequentialExecutor,
                    1,
                )
                .unwrap();
        } else {
            world.step(1e-4).unwrap();
        }
        let b = world.body(id).unwrap();
        assert!(b.state().linear_velocity().x < 50.0);
        assert_eq!(b.state().angular_velocity(), Vec3::ZERO);
        if let Some(expected) = reference {
            assert_eq!(b.motion(), expected);
        } else {
            reference = Some(b.motion());
        }
    }
}

#[test]
fn data_driven_hull_body_part_dispatch_and_range_failure_rollback() {
    use crate::aerodynamics::{
        AerodynamicsError,
        hull::{HullError, MachCurve, MachKnot},
    };
    // Synthetic coefficients exercise dispatch, not physical prediction.
    for part_owned in [false, true] {
        for sharded in [false, true] {
            let curve = |min| {
                MachCurve::new(vec![
                    MachKnot {
                        mach: min,
                        axial_cd: 0.2,
                        crossflow_cd: 0.8,
                    },
                    MachKnot {
                        mach: 1.5,
                        axial_cd: 0.4,
                        crossflow_cd: 1.0,
                    },
                ])
                .unwrap()
            };
            let sections = vec![
                hull_section(-Vec3::Y).with_mach_curve(curve(0.5)),
                hull_section(Vec3::Y).with_mach_curve(curve(1.1)),
            ];
            let mut body = RigidBody::new_dynamic(
                Transform::new(Vec3::new(12.0, 4.0, -8.0), kapitsa_math::Quat::IDENTITY).unwrap(),
                MassProperties::new(100.0, Vec3::X * 0.5, Mat3::IDENTITY * 100.0).unwrap(),
            )
            .unwrap();
            // Relative to wind the load points see exactly Mach 1.
            body.set_linear_velocity(Vec3::X * 350.0).unwrap();
            body.set_aerodynamics(Aerodynamics::Hull(sections.clone()));
            let mut world = PhysicsWorld::new();
            world.set_atmosphere(super::test_atmosphere(
                AtmosphericState::new(1.2, Vec3::X * 10.0).unwrap(),
            ));
            let id = world.add_body(body).unwrap();
            let root = part_owned.then(|| world.enable_cluster(id).unwrap());
            let before = world.body(id).unwrap().motion();
            let result = if sharded {
                world.step_with_shards(
                    1e-6,
                    &mut vec![Vec::new(); 2],
                    &crate::execution::SequentialExecutor,
                    1,
                )
            } else {
                world.step(1e-6)
            };
            assert_eq!(
                result,
                Err(super::StepError::Aerodynamics {
                    body_id: id,
                    cause: AerodynamicsError::Hull(HullError::UnsupportedMach),
                })
            );
            assert_eq!(world.body(id).unwrap().motion(), before);
            if let Some(root) = root {
                assert_eq!(world.part_aerodynamic_hull(root), Some(sections.as_slice()));
                world
                    .set_part_aerodynamic_hull(root, vec![sections[0].clone()])
                    .unwrap();
            } else {
                assert!(matches!(world.body(id).unwrap().aerodynamics(),
                    Aerodynamics::Hull(s) if s == &sections));
                world
                    .body_mut(id)
                    .unwrap()
                    .set_aerodynamics(Aerodynamics::Hull(vec![sections[0].clone()]));
            }
            if sharded {
                world
                    .step_with_shards(
                        1e-6,
                        &mut vec![Vec::new(); 2],
                        &crate::execution::SequentialExecutor,
                        1,
                    )
                    .unwrap();
            } else {
                world.step(1e-6).unwrap();
            }
            let state = world.body(id).unwrap().state();
            assert!(state.linear_velocity().x < 350.0);
            assert!(state.angular_velocity().z < 0.0);
        }
    }
}

#[test]
fn normal_hull_table_range_failure_rolls_back_body_and_part_loads() {
    use crate::aerodynamics::{
        AerodynamicsError,
        hull::{HullError, HullSection, NormalForceCurve, NormalForceKnot},
    };
    // Synthetic axisymmetric tables; exercise Mach and incidence failures.
    for part_owned in [false, true] {
        for sharded in [false, true] {
            for angle_failure in [false, true] {
                let section = |point, invalid| {
                    let min_mach = if invalid && !angle_failure { 1.1 } else { 0.5 };
                    let max_angle = if invalid && angle_failure {
                        0.5
                    } else {
                        std::f64::consts::PI
                    };
                    let knots = [min_mach, 1.5]
                        .into_iter()
                        .flat_map(|mach| {
                            [0.0, max_angle].into_iter().map(move |angle_of_attack| {
                                NormalForceKnot {
                                    mach,
                                    angle_of_attack,
                                    normal_cn: 0.5,
                                }
                            })
                        })
                        .collect();
                    HullSection::new(
                        Transform::new(point, kapitsa_math::Quat::IDENTITY).unwrap(),
                        Vec3::X,
                        1.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        None,
                    )
                    .unwrap()
                    .with_normal_curve(NormalForceCurve::new(2.0, knots).unwrap())
                };
                let sections = vec![section(-Vec3::X, false), section(Vec3::X, true)];
                let mut body = RigidBody::new_dynamic(
                    Transform::identity(),
                    MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0).unwrap(),
                )
                .unwrap();
                body.set_linear_velocity(Vec3::Y * 340.0).unwrap();
                body.set_aerodynamics(Aerodynamics::Hull(sections.clone()));
                let mut world = PhysicsWorld::new();
                world.set_atmosphere(super::test_atmosphere(
                    AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
                ));
                let id = world.add_body(body).unwrap();
                let root = part_owned.then(|| world.enable_cluster(id).unwrap());
                let before = world.body(id).unwrap().motion();
                let result = if sharded {
                    world.step_with_shards(
                        1e-6,
                        &mut vec![Vec::new(); 2],
                        &crate::execution::SequentialExecutor,
                        1,
                    )
                } else {
                    world.step(1e-6)
                };
                assert_eq!(
                    result,
                    Err(super::StepError::Aerodynamics {
                        body_id: id,
                        cause: AerodynamicsError::Hull(HullError::UnsupportedMach),
                    })
                );
                assert_eq!(world.body(id).unwrap().motion(), before);
                if let Some(root) = root {
                    assert_eq!(world.part_aerodynamic_hull(root), Some(sections.as_slice()));
                    world
                        .set_part_aerodynamic_hull(root, vec![sections[0].clone()])
                        .unwrap();
                } else {
                    assert!(
                        matches!(world.body(id).unwrap().aerodynamics(), Aerodynamics::Hull(s) if s == &sections)
                    );
                    world
                        .body_mut(id)
                        .unwrap()
                        .set_aerodynamics(Aerodynamics::Hull(vec![sections[0].clone()]));
                }
                world.step(1e-6).unwrap();
                assert!(world.body(id).unwrap().state().linear_velocity().y < 340.0);
            }
        }
    }
}

#[test]
fn hull_sections_sample_spatial_density_independently() {
    use crate::aerodynamics::{Atmosphere, DensityProfile, SphericalAtmosphere};
    let atmosphere = Atmosphere::Sphere(
        SphericalAtmosphere::builder()
            .reference_radius(1.0)
            .reference_density(1.2)
            .height(100.0)
            .profile(DensityProfile::Exponential { scale_height: 10.0 })
            .build()
            .unwrap(),
    );
    let mut body = RigidBody::new_dynamic(
        Transform::new(Vec3::Y * 10.0, kapitsa_math::Quat::IDENTITY).unwrap(),
        MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
    )
    .unwrap();
    body.set_linear_velocity(Vec3::X * 50.0).unwrap();
    body.set_aerodynamics(Aerodynamics::Hull(vec![
        hull_section(-Vec3::Y * 2.0),
        hull_section(Vec3::Y * 2.0),
    ]));
    let mut world = PhysicsWorld::new();
    let id = world.add_body(body).unwrap();
    world.set_atmosphere(atmosphere);
    world.step(1e-5).unwrap();
    // Equal point sampling at COM would incorrectly cancel this torque.
    assert!(world.body(id).unwrap().state().angular_velocity().z < 0.0);
}

#[test]
fn hull_later_section_error_rolls_back_all_loads() {
    use crate::aerodynamics::{AerodynamicsError, hull::HullError};
    for sharded in [false, true] {
        let mut body = RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
        )
        .unwrap();
        body.set_linear_velocity(Vec3::X * 50.0).unwrap();
        body.set_angular_velocity(Vec3::Z * 10.0).unwrap();
        let sections = vec![hull_section(Vec3::ZERO), hull_section(Vec3::X * 20.0)];
        body.set_aerodynamics(Aerodynamics::Hull(sections.clone()));
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
        ));
        let before = world.body(id).unwrap().motion();
        let result = if sharded {
            world.step_with_shards(
                1e-4,
                &mut vec![Vec::new(); 2],
                &crate::execution::SequentialExecutor,
                1,
            )
        } else {
            world.step(1e-4)
        };
        assert_eq!(
            result,
            Err(super::StepError::Aerodynamics {
                body_id: id,
                cause: AerodynamicsError::Hull(HullError::UnsupportedMach)
            })
        );
        assert_eq!(world.body(id).unwrap().motion(), before);
        assert!(
            matches!(world.body(id).unwrap().aerodynamics(), Aerodynamics::Hull(s) if s == &sections)
        );
        world
            .body_mut(id)
            .unwrap()
            .set_aerodynamics(Aerodynamics::Hull(vec![sections[0].clone()]));
        world.step(1e-4).unwrap();
        assert!(world.body(id).unwrap().state().linear_velocity().x < 50.0);
    }
}

#[test]
fn later_wing_failure_rolls_back_earlier_loads() {
    use crate::aerodynamics::{
        AerodynamicsError,
        wing::{FiniteWing, WingError, WingSurface},
    };
    for sharded in [false, true] {
        let model = FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2).unwrap();
        let valid = WingSurface::new(model, Transform::identity(), 1.0).unwrap();
        let invalid = WingSurface::new(
            model,
            Transform::new(
                Vec3::ZERO,
                kapitsa_math::Quat::from_axis_angle(Vec3::Y, 1.0).unwrap(),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();
        let mut body = RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
        )
        .unwrap();
        body.set_linear_velocity(Vec3::new(50.0, 0.0, -5.0))
            .unwrap();
        body.set_aerodynamics(Aerodynamics::Wings(vec![valid.clone(), invalid]));
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
        ));
        let before = world.body(id).unwrap().motion();
        let result = if sharded {
            world.step_with_shards(
                1e-4,
                &mut vec![Vec::new(); 2],
                &crate::execution::SequentialExecutor,
                1,
            )
        } else {
            world.step(1e-4)
        };
        assert_eq!(
            result,
            Err(super::StepError::Aerodynamics {
                body_id: id,
                cause: AerodynamicsError::Wing(WingError::UnsupportedFlow)
            })
        );
        assert_eq!(world.body(id).unwrap().motion(), before);
        world
            .body_mut(id)
            .unwrap()
            .set_aerodynamics(Aerodynamics::Wings(vec![valid]));
        world.step(1e-4).unwrap();
        assert!(world.body(id).unwrap().state().linear_velocity().z > -5.0);
    }
}

#[test]
fn symmetric_wings_sum_loads_and_cancel_roll_moment() {
    use crate::aerodynamics::wing::{FiniteWing, WingSurface};
    let mut reference = None;
    for sharded in [false, true] {
        let model = FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2).unwrap();
        let wings: Vec<_> = [-2.0, 2.0]
            .into_iter()
            .map(|y| {
                WingSurface::new(
                    model,
                    Transform::new(Vec3::Y * y, kapitsa_math::Quat::IDENTITY).unwrap(),
                    1.0,
                )
                .unwrap()
            })
            .collect();
        let mut body = RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
        )
        .unwrap();
        body.set_linear_velocity(Vec3::new(50.0, 0.0, -5.0))
            .unwrap();
        body.set_aerodynamics(Aerodynamics::Wings(wings));
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        world.set_atmosphere(super::test_atmosphere(
            AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
        ));
        if sharded {
            world
                .step_with_shards(
                    1e-4,
                    &mut vec![Vec::new(); 2],
                    &crate::execution::SequentialExecutor,
                    1,
                )
                .unwrap();
        } else {
            world.step(1e-4).unwrap();
        }
        let body = world.body(id).unwrap();
        assert_eq!(body.state().angular_velocity(), Vec3::ZERO);
        assert!(body.state().linear_velocity().z > -5.0);
        if let Some(expected) = reference {
            assert_eq!(body.motion(), expected);
        } else {
            reference = Some(body.motion());
        }
    }
}

#[test]
fn wing_cluster_transfer_preserves_body_and_configuration() {
    use crate::aerodynamics::wing::{FiniteWing, WingSurface};
    let mut body = RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY).unwrap(),
    )
    .unwrap();
    body.set_aerodynamics(Aerodynamics::Wing(
        WingSurface::new(
            FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2).unwrap(),
            Transform::identity(),
            2.0,
        )
        .unwrap(),
    ));
    let mut world = PhysicsWorld::new();
    let id = world.add_body(body).unwrap();
    let before = world.body(id).unwrap().motion();
    let original = match world.body(id).unwrap().aerodynamics() {
        Aerodynamics::Wing(wing) => wing.clone(),
        _ => panic!("wing"),
    };
    let root = world.enable_cluster(id).unwrap();
    assert_eq!(world.body(id).unwrap().motion(), before);
    assert!(matches!(
        world.body(id).unwrap().aerodynamics(),
        Aerodynamics::None
    ));
    assert_eq!(
        world.part_aerodynamic_wings(root),
        Some([original].as_slice())
    );
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
                .1
                .occupancy()
                .as_ptr();
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
                    .1
                    .occupancy()
                    .as_ptr()
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

#[test]
fn mesh_zero_translation_preserves_rest_and_resolves_rotational_loads()
-> Result<(), Box<dyn std::error::Error>> {
    for sharded in [false, true] {
        for centered in [false, true] {
            for rotating in [false, true] {
                let mut world = PhysicsWorld::new();
                world.set_atmosphere(super::test_atmosphere(AtmosphericState::new(
                    0.001,
                    Vec3::ZERO,
                )?));
                world.set_mesh_sound_speed(100.0)?;
                let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
                let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
                if rotating {
                    body.set_angular_velocity(Vec3::Z * 10_000.0)?;
                }
                let mesh = if centered {
                    TriangleMesh::new(
                        vec![
                            Vec3::new(-2.0, 0.0, 0.0),
                            Vec3::new(1.0, 0.0, -1.0),
                            Vec3::new(1.0, 0.0, 1.0),
                        ],
                        vec![[0, 2, 1]],
                    )?
                } else {
                    TriangleMesh::new(
                        vec![
                            Vec3::X,
                            Vec3::X * 2.0,
                            Vec3::X + Vec3::Z,
                            Vec3::X * 2.0 + Vec3::Z,
                        ],
                        vec![[0, 2, 1], [1, 2, 3]],
                    )?
                };
                body.set_aerodynamics(Aerodynamics::Mesh(mesh));
                let id = world.add_body(body)?;
                if sharded {
                    world.step_with_shards(
                        1e-8,
                        &mut [Vec::new()],
                        &crate::execution::SequentialExecutor,
                        1,
                    )?;
                } else {
                    world.step(1e-8)?;
                }
                let state = world.body(id).ok_or("body")?.state();
                if rotating {
                    assert!(state.linear_velocity().y < 0.0);
                    assert!(state.angular_velocity().z < 10_000.0);
                } else {
                    assert_eq!(state.linear_velocity(), Vec3::ZERO);
                    assert_eq!(state.angular_velocity(), Vec3::ZERO);
                }
            }
        }
    }
    Ok(())
}

#[test]
fn nonzero_unrepresentable_mesh_flow_is_not_silently_treated_as_rest()
-> Result<(), Box<dyn std::error::Error>> {
    for sharded in [false, true] {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::test_atmosphere(AtmosphericState::new(
            0.001,
            Vec3::ZERO,
        )?));
        let mut body = RigidBody::new_dynamic(
            Transform::identity(),
            MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY)?,
        )?;
        body.set_linear_velocity(Vec3::X * f64::MAX)?;
        body.set_aerodynamics(Aerodynamics::Mesh(TriangleMesh::new(
            vec![Vec3::ZERO, Vec3::Y, Vec3::Z],
            vec![[0, 1, 2]],
        )?));
        let id = world.add_body(body)?;
        let before = world.body(id).ok_or("body")?.motion();
        let result = if sharded {
            world.step_with_shards(
                1e-8,
                &mut [Vec::new()],
                &crate::execution::SequentialExecutor,
                1,
            )
        } else {
            world.step(1e-8)
        };
        assert!(
            matches!(result, Err(crate::world::StepError::Aerodynamics { .. })),
            "result={result:?}"
        );
        assert_eq!(world.body(id).ok_or("body")?.motion(), before);
    }
    Ok(())
}

#[test]
fn wave_drag_is_applied_to_body_and_cluster() -> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::hull::HullSection;
    for clustered in [false, true] {
        for sharded in [false, true] {
            let section = HullSection::new(
                Transform::identity(),
                Vec3::X,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                None,
            )?
            .with_wave_drag(&[0.0, 1.0, -1.0, 0.0])?;
            let mut body = RigidBody::new_dynamic(
                Transform::identity(),
                MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY)?,
            )?;
            body.set_linear_velocity(Vec3::X * 680.0)?;
            body.set_aerodynamics(Aerodynamics::Hull(vec![section]));
            let mut world = PhysicsWorld::new();
            let id = world.add_body(body)?;
            if clustered {
                world.enable_cluster(id)?;
            }
            world.set_atmosphere(super::test_atmosphere(AtmosphericState::new(
                1.2,
                Vec3::ZERO,
            )?));
            if sharded {
                world.step_with_shards(
                    1e-4,
                    &mut vec![Vec::new(); 2],
                    &crate::execution::SequentialExecutor,
                    1,
                )?;
            } else {
                world.step(1e-4)?;
            }
            let velocity = world
                .body(id)
                .ok_or("missing body")?
                .state()
                .linear_velocity();
            assert!(velocity.x < 680.0 && velocity.x > 0.0);
        }
    }
    Ok(())
}

#[test]
fn wave_transition_world_and_rollback() -> Result<(), Box<dyn std::error::Error>> {
    use crate::aerodynamics::{
        AerodynamicsError,
        hull::{HullError, HullSection},
        wave::{WaveDragError, WaveMachCurve},
    };
    for clustered in [false, true] {
        for sharded in [false, true] {
            for (speed, in_range) in [(340.0, true), (510.0, false)] {
                let section = HullSection::new(
                    Transform::identity(),
                    Vec3::X,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    None,
                )?
                .with_wave_drag(&[0.0, 1.0, -1.0, 0.0])?
                .with_wave_mach_curve(WaveMachCurve::new(vec![
                    (0.8, 0.0),
                    (1.0, 0.5),
                    (1.2, 1.0),
                ])?);
                let mut body = RigidBody::new_dynamic(
                    Transform::identity(),
                    MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY)?,
                )?;
                body.set_linear_velocity(Vec3::X * speed)?;
                body.set_aerodynamics(Aerodynamics::Hull(vec![section]));
                let mut world = PhysicsWorld::new();
                let id = world.add_body(body)?;
                if clustered {
                    world.enable_cluster(id)?;
                }
                world.set_atmosphere(super::test_atmosphere(AtmosphericState::new(
                    1.2,
                    Vec3::ZERO,
                )?));
                let before = world.body(id).ok_or("body")?.motion();
                let result = if sharded {
                    world.step_with_shards(
                        1e-4,
                        &mut [Vec::new(), Vec::new()],
                        &crate::execution::SequentialExecutor,
                        1,
                    )
                } else {
                    world.step(1e-4)
                };
                if in_range {
                    result?;
                    assert!(world.body(id).ok_or("body")?.state().linear_velocity().x < speed);
                } else {
                    assert!(matches!(
                        result,
                        Err(crate::world::StepError::Aerodynamics {
                            cause: AerodynamicsError::Hull(HullError::Wave(
                                WaveDragError::UnsupportedMach
                            )),
                            ..
                        })
                    ));
                    assert_eq!(world.body(id).ok_or("body")?.motion(), before);
                }
            }
        }
    }
    Ok(())
}
