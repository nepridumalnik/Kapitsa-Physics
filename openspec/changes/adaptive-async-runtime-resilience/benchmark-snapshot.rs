use std::{error::Error, hint::black_box, time::{Duration, Instant}};
use kapitsa::{Simulation, RigidBody, Transform};

fn main() -> Result<(), Box<dyn Error>> {
    let runtime = tokio::runtime::Builder::new_current_thread().enable_all().build()?;
    runtime.block_on(async {
        let mut simulation = Simulation::builder().time_horizon(Duration::ZERO).spawn_on_tokio(&tokio::runtime::Handle::current())?;
        let client = simulation.client();
        let body = client.add_body(RigidBody::new_static(Transform::identity())).await?;
        for _ in 0..10_000 { black_box(client.snapshot().body(body)); }
        for run in 0..5 {
            let mut durations = Vec::with_capacity(100_000);
            for _ in 0..100_000 {
                let start = Instant::now();
                black_box(client.snapshot().body(body));
                durations.push(start.elapsed().as_nanos());
            }
            durations.sort_unstable();
            println!("sample={run} reads={} median_ns={} p95_ns={}", durations.len(), durations[50_000], durations[95_000]);
        }
        simulation.stop().await?;
        Ok::<(), Box<dyn Error>>(())
    })
}
