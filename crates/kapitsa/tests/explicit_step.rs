use kapitsa::{
    PhysicsSystem, PhysicsWorld, math::Vec3, rigid_body::RigidBody, transform::Transform,
};

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
