use std::{error::Error, sync::Arc, time::Duration};

use kapitsa::{
    AdaptiveTimeStepConfig, MassProperties, RecoveryConfig, RigidBody, Simulation,
    StepRecoveryPolicy, TimeStepPolicy, Transform,
    math::{Mat3, Vec3},
};

fn main() -> Result<(), Box<dyn Error>> {
    // The service owns its clock runtime; this pool remains owned by the host.
    let pool = Arc::new(rayon::ThreadPoolBuilder::new().num_threads(2).build()?);
    let control = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    let mut simulation = Simulation::builder()
        .frequency(100)
        .time_step_policy(TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default()))
        .step_recovery_policy(StepRecoveryPolicy::Resilient(RecoveryConfig::default()))
        .time_horizon(Duration::ZERO)
        .spawn_on_rayon(Arc::clone(&pool))?;
    let physics = simulation.client();
    let mut results = physics.subscribe_batch_results();

    // Setup and environment changes happen on a control path, outside the host loop.
    let (body, source) = control.block_on(async {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let body = physics
            .add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)
            .await?;
        let source = physics.add_force_source(body, Vec3::ZERO, Vec3::X).await?;
        physics.set_central_gravity(None).await?;
        Ok::<_, kapitsa::Error>((body, source))
    })?;
    simulation.set_time_horizon(Some(Duration::from_millis(100)));

    let mut submitted = None;
    for frame in 0..100 {
        if frame == 0 {
            let mut commands = physics.commands();
            commands.set_force_source(source, Vec3::ZERO, Vec3::Y);
            submitted = Some(physics.try_submit(commands)?);
        }
        let snapshot = physics.snapshot();
        if let Some(state) = snapshot.body(body) {
            std::hint::black_box((snapshot.simulation_time(), state.linear_velocity()));
        }
        // Represents other host work; it does not wait on the physics service.
        std::thread::sleep(Duration::from_millis(2));
    }

    control.block_on(async {
        let result = results.next_result().await?;
        if Some(result.batch_id) != submitted {
            return Err("unexpected batch result".into());
        }
        result.result?;
        tokio::time::timeout(Duration::from_secs(2), async {
            while physics.snapshot().simulation_time() < Duration::from_millis(100) {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await?;
        // The old horizon guarantees this acknowledgement precedes later physics.
        physics.set_central_gravity(None).await?;
        Ok::<_, Box<dyn Error>>(())
    })?;
    simulation.set_time_horizon(Some(Duration::from_millis(200)));
    control.block_on(async {
        tokio::time::timeout(Duration::from_secs(2), async {
            while physics.snapshot().simulation_time() < Duration::from_millis(200) {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        })
        .await?;
        simulation.stop().await?;
        Ok::<_, Box<dyn Error>>(())
    })?;
    println!(
        "committed simulation time: {:?}",
        physics.snapshot().simulation_time()
    );
    Ok(())
}
