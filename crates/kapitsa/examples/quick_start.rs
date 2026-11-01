use std::error::Error;

use kapitsa::math::{Mat3, Vec3};
use kapitsa::{MassProperties, RigidBody, Simulation, SimulationConfig, Transform};

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn Error>> {
    let mut simulation = Simulation::new_from_config(SimulationConfig::default())?;
    let physics = simulation.client();
    let mut results = physics.subscribe_batch_results();
    simulation.start()?;

    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let body = physics
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)
        .await?;
    let source = physics
        .add_force_source(body, Vec3::ZERO, Vec3::X * 10.0)
        .await?;

    let mut commands = physics.commands();
    commands.set_force_source(source, Vec3::ZERO, Vec3::Y * 10.0);
    let batch_id = physics.try_submit(commands)?;
    let result = results.next_result().await?;
    if result.batch_id != batch_id {
        return Err("received an unexpected command batch result".into());
    }
    result
        .result
        .map_err(|error| format!("batch rejected: {error:?}"))?;

    if let Some(state) = physics.snapshot().body(body) {
        println!("body velocity: {:?}", state.linear_velocity());
    }

    physics.remove_force_source(source).await?;
    simulation.stop().await?;
    Ok(())
}
