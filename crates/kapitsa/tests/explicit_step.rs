use kapitsa::{
    PhysicsSystem, PhysicsWorld, math::Vec3, rigid_body::RigidBody, transform::Transform,
};

#[test]
fn integration_failure_preserves_committed_results() -> Result<(), Box<dyn std::error::Error>> {
    let mut system = PhysicsSystem::builder(PhysicsWorld::new()).build()?;
    let mut moving = RigidBody::new_kinematic(Transform::identity());
    moving.set_linear_velocity(Vec3::X)?;
    let moving_id = system.world_mut().add_body(moving)?;
    system.update(0.125)?;
    let mut overflowing = RigidBody::new_kinematic(Transform::identity());
    overflowing.set_linear_velocity(Vec3::X * f64::MAX)?;
    let overflowing_id = system.world_mut().add_body(overflowing)?;
    let before = system
        .world()
        .bodies()
        .map(|(_, body)| body.motion())
        .collect::<Vec<_>>();
    let committed = system.world().committed_step_results();
    let diagnostics = committed.diagnostics();
    let events = committed.collision_events().to_vec();
    let impulses = committed.contact_impulses().to_vec();
    assert!(system.update(2.0).is_err());
    assert!(
        system
            .world()
            .bodies()
            .map(|(_, body)| body.motion())
            .eq(before)
    );
    let committed = system.world().committed_step_results();
    assert_eq!(committed.diagnostics(), diagnostics);
    assert_eq!(committed.collision_events(), events);
    assert_eq!(committed.contact_impulses(), impulses);
    system.world_mut().remove_body(overflowing_id);
    system.update(0.125)?;
    assert_eq!(
        system
            .world()
            .body(moving_id)
            .map(|body| body.state().transform().translation()),
        Some(Vec3::X * 0.25)
    );
    Ok(())
}

#[cfg(feature = "parallel")]
#[test]
fn contact_results_match_between_executors() -> Result<(), Box<dyn std::error::Error>> {
    use kapitsa::{
        Collider,
        mass::MassProperties,
        math::{Mat3, Quat},
        shape::{Shape, Sphere},
    };
    use std::num::NonZeroUsize;

    fn fixture() -> Result<PhysicsWorld, kapitsa::Error> {
        let mut world = PhysicsWorld::new();
        for index in 0..16 {
            let origin = Vec3::Z * (index as f64 * 8.0);
            let fixed = world.add_body(RigidBody::new_static(Transform::new(
                origin,
                Quat::IDENTITY,
            )?))?;
            let mut body = RigidBody::new_dynamic(
                Transform::new(origin + Vec3::X * 1.9, Quat::IDENTITY)?,
                MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
            )?;
            body.set_linear_velocity(-Vec3::X)?;
            let moving = world.add_body(body)?;
            for id in [fixed, moving] {
                world.add_collider(
                    id,
                    Collider::new(Shape::Sphere(Sphere::new(1.0)?), Transform::identity()),
                )?;
            }
        }
        Ok(world)
    }
    let mut sequential = PhysicsSystem::builder(fixture()?).build()?;
    let mut parallel = PhysicsSystem::builder(fixture()?)
        .workers(NonZeroUsize::new(4).ok_or("positive workers required")?)
        .build()?;
    for step in 0..3 {
        let a = sequential.update(0.01)?;
        let b = parallel.update(0.01)?;
        if step == 0 {
            assert!(!a.collision_events().is_empty());
            assert!(!a.contact_impulses().is_empty());
        }
        assert_eq!(a.diagnostics(), b.diagnostics());
        assert_eq!(a.joint_events(), b.joint_events());
        assert!(
            a.collision_events()
                .iter()
                .map(|e| (e.kind, e.sensor, e.contact))
                .eq(b
                    .collision_events()
                    .iter()
                    .map(|e| (e.kind, e.sensor, e.contact)))
        );
        assert!(
            a.contact_impulses()
                .iter()
                .map(|i| (i.points, i.impulse_on_b, i.ccd))
                .eq(b
                    .contact_impulses()
                    .iter()
                    .map(|i| (i.points, i.impulse_on_b, i.ccd)))
        );
        assert!(
            sequential
                .world()
                .bodies()
                .map(|(_, body)| body.motion())
                .eq(parallel.world().bodies().map(|(_, body)| body.motion()))
        );
    }
    Ok(())
}

#[cfg(feature = "parallel")]
#[test]
fn private_and_shared_pools_advance_independent_worlds() -> Result<(), Box<dyn std::error::Error>> {
    use std::{num::NonZeroUsize, sync::Arc};

    let shared = Arc::new(rayon::ThreadPoolBuilder::new().num_threads(2).build()?);
    let mut systems = [
        PhysicsSystem::builder(PhysicsWorld::new())
            .workers(NonZeroUsize::new(2).ok_or("worker count must be positive")?)
            .build()?,
        PhysicsSystem::builder(PhysicsWorld::new())
            .rayon_pool(Arc::clone(&shared))
            .build()?,
    ];
    for (index, system) in systems.iter_mut().enumerate() {
        let mut body = RigidBody::new_kinematic(Transform::identity());
        body.set_linear_velocity(Vec3::X)?;
        let id = system.world_mut().add_body(body)?;
        let dt = 0.125 * (index + 1) as f64;
        system.update(dt)?;
        assert_eq!(
            system
                .world()
                .body(id)
                .map(|body| body.state().transform().translation()),
            Some(Vec3::X * dt)
        );
    }
    drop(systems);
    assert_eq!(Arc::strong_count(&shared), 1);
    assert_eq!(shared.install(rayon::current_num_threads), 2);
    Ok(())
}

#[test]
fn host_durations_advance_the_world_and_invalid_steps_preserve_it() -> Result<(), kapitsa::Error> {
    let mut world = PhysicsWorld::new();
    let mut body = RigidBody::new_kinematic(Transform::identity());
    body.set_linear_velocity(Vec3::X)?;
    let id = world.add_body(body)?;
    let mut system = PhysicsSystem::builder(world).build()?;
    let mut elapsed = 0.0;
    for dt in [0.125, 0.25] {
        elapsed += dt;
        system.update(dt)?;
        assert_eq!(
            system
                .world()
                .body(id)
                .map(|body| body.state().transform().translation()),
            Some(Vec3::X * elapsed)
        );
    }
    let diagnostics = system.world().committed_step_results().diagnostics();
    assert!(system.update(f64::NAN).is_err());
    assert_eq!(
        system.world().committed_step_results().diagnostics(),
        diagnostics
    );
    assert_eq!(
        system
            .world()
            .body(id)
            .map(|body| body.state().transform().translation()),
        Some(Vec3::X * elapsed)
    );
    Ok(())
}
