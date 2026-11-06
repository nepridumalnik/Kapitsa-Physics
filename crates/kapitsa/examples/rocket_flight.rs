#[path = "../../../examples/rocket-flight-viewer/src/camera.rs"]
#[allow(dead_code)]
mod camera;
#[path = "../../../examples/rocket-flight-viewer/src/flight.rs"]
#[allow(dead_code)]
mod flight;

use std::{error::Error, time::Instant};

use flight::{FlightConfig, run};

fn main() -> Result<(), Box<dyn Error>> {
    let (warp, maximum_steps) = arguments()?;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(4)
        .enable_all()
        .build()?;
    let started = Instant::now();
    let frames = runtime.block_on(run(FlightConfig {
        warp,
        maximum_steps,
        fps: 1,
    }))?;
    let elapsed = started.elapsed();
    let first = frames.first().ok_or("flight produced no frames")?;
    let last = frames.last().ok_or("flight produced no frames")?;
    println!(
        "Rocket runtime mission: steps {}..{}, simulated {:.3}s, {} samples, wall {:.3}s, requested warp {:.1}x",
        first.step,
        last.step,
        last.simulation_time,
        frames.len(),
        elapsed.as_secs_f64(),
        warp,
    );
    println!(
        "terminal bodies: stage one={}, stage two={}",
        last.stage_one.is_some(),
        last.stage_two.is_some(),
    );
    Ok(())
}

fn arguments() -> Result<(f64, u64), Box<dyn Error>> {
    let mut warp: f64 = 1.0;
    let mut maximum_steps = 4_200;
    for argument in std::env::args().skip(1) {
        if let Some(value) = argument.strip_prefix("--warp=") {
            warp = value.parse()?;
        } else if let Some(value) = argument.strip_prefix("--steps=") {
            maximum_steps = value.parse()?;
        } else if argument == "--no-ansi" {
            continue;
        } else {
            return Err(format!("unknown argument: {argument}").into());
        }
    }
    if !warp.is_finite() || warp <= 0.0 || maximum_steps == 0 {
        return Err("warp and steps must be positive".into());
    }
    Ok((warp, maximum_steps))
}
