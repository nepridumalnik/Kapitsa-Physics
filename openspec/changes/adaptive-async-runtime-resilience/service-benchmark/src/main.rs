use kapitsa::{
    AdaptiveTimeStepConfig, Collider, MassProperties, RigidBody, Shape, Simulation, TimeStepPolicy,
    Transform,
    math::{Mat3, Quat, Vec3},
    rigid_body::CcdMode,
    shape::{Cuboid, Sphere},
};
use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

fn quantile(samples: &mut [u128], numerator: usize) -> u128 {
    samples.sort_unstable();
    samples[(samples.len() - 1) * numerator / 100]
}

fn main() -> Result<(), Box<dyn Error>> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(4)
        .enable_all()
        .build()?;
    runtime.block_on(async {
        for scenario in ["ordinary", "contact", "overload"] {
            for repetition in 0..6 {
                let builder = Simulation::builder().frequency(100).max_workers(4)
                    .time_step_policy(TimeStepPolicy::Adaptive(AdaptiveTimeStepConfig::default()))
                    .time_horizon(Duration::ZERO);
                #[cfg(feature = "resilient")]
                let builder = builder.step_recovery_policy(kapitsa::StepRecoveryPolicy::Resilient(kapitsa::RecoveryConfig::default()));
                let mut simulation = builder.spawn_on_tokio(&tokio::runtime::Handle::current())?;
                let client = simulation.client();
                let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
                let mut body = RigidBody::new_dynamic(Transform::new(Vec3::new(-2.0, 0.0, 0.0), Quat::IDENTITY)?, mass)?;
                if scenario == "contact" { body.set_linear_velocity(Vec3::X * 100.0)?; body.set_ccd_mode(CcdMode::Continuous); }
                let body_id = client.add_body(body).await?;
                if scenario == "contact" {
                    client.add_collider(body_id, Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity())).await?;
                    let wall = client.add_body(RigidBody::new_static(Transform::identity())).await?;
                    client.add_collider(wall, Collider::new(Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0)?), Transform::identity())).await?;
                }
                client.set_linear_velocity(body_id, Vec3::ZERO).await?;
                simulation.set_time_multiplier(1.0)?;
                simulation.set_time_horizon(Some(Duration::from_millis(50)));
                tokio::time::timeout(Duration::from_secs(5), async {
                    while client.snapshot().simulation_time() < Duration::from_millis(50) { tokio::task::yield_now().await; }
                }).await?;
                if scenario == "contact" { client.set_linear_velocity(body_id, Vec3::X * 100.0).await?; }
                simulation.set_time_multiplier(if scenario == "overload" { 1000.0 } else { 1.0 })?;
                let initial = simulation.metrics();
                let start = Instant::now();
                simulation.set_time_horizon(Some(Duration::from_millis(250)));
                let mut reads = Vec::new();
                let mut steps = Vec::new();
                let mut observed = initial.total_steps;
                let mut max_debt = Duration::ZERO;
                tokio::time::timeout(Duration::from_secs(10), async {
                    loop {
                        let read_start = Instant::now();
                        let snapshot = client.snapshot();
                        black_box(snapshot.body(body_id));
                        reads.push(read_start.elapsed().as_nanos());
                        let metrics = simulation.metrics();
                        max_debt = max_debt.max(metrics.simulation_time_debt);
                        if metrics.total_steps != observed { observed = metrics.total_steps; steps.push(metrics.last_step_duration.as_nanos()); }
                        if snapshot.simulation_time() >= Duration::from_millis(250) { break; }
                        tokio::time::sleep(Duration::from_micros(100)).await;
                    }
                }).await?;
                let elapsed = start.elapsed();
                let final_metrics = simulation.metrics();
                let committed = client.snapshot().simulation_time() - Duration::from_millis(50);
                if repetition > 0 {
                    if steps.is_empty() { return Err("no step samples".into()); }
                    let step_median = quantile(&mut steps, 50);
                    let step_p95 = quantile(&mut steps, 95);
                    let read_median = quantile(&mut reads, 50);
                    let read_p95 = quantile(&mut reads, 95);
                    #[cfg(feature = "resilient")]
                    let retries = final_metrics.total_retries - initial.total_retries;
                    #[cfg(not(feature = "resilient"))]
                    let retries = 0;
                    println!("scenario={scenario} repetition={repetition} committed_ns={} wall_ns={} warp={:.3} steps={} observed={} step_median_ns={step_median} step_p95_ns={step_p95} snapshot_median_ns={read_median} snapshot_p95_ns={read_p95} max_debt_ns={} retries={retries}", committed.as_nanos(), elapsed.as_nanos(), committed.as_secs_f64() / elapsed.as_secs_f64(), final_metrics.total_steps - initial.total_steps, steps.len(), max_debt.as_nanos());
                }
                simulation.stop().await?;
            }
        }
        Ok::<(), Box<dyn Error>>(())
    })
}
