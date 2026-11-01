use crate::flight::{FlightConfig, STAGE_TWO_BURN_END, run};

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn records_physics_snapshots_on_the_requested_step_schedule() {
    let frames = run(FlightConfig {
        warp: 1.0,
        maximum_steps: 24,
        fps: 24,
    })
    .await
    .expect("flight snapshots");

    assert!(frames.len() >= 5);
    assert!(frames[0].stage_one.is_some());
    assert!(frames[0].stage_two.is_some());
    assert!(frames.last().expect("last frame").step - frames[0].step <= 24);
    assert!(
        frames
            .windows(2)
            .all(|pair| pair[1].step > pair[0].step && pair[1].step - pair[0].step <= 5)
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn completes_both_stage_impacts_after_mass_depletion_and_separation() {
    let frames = run(FlightConfig {
        warp: 1_000.0,
        maximum_steps: 4_200,
        fps: 1,
    })
    .await
    .expect("complete rocket mission");

    assert!(
        frames
            .iter()
            .any(|frame| frame.stage_one.is_none() && frame.stage_two.is_some())
    );
    let final_frame = frames.last().expect("terminal mission frame");
    assert!(final_frame.simulation_time >= STAGE_TWO_BURN_END);
    assert!(final_frame.stage_one.is_none());
    assert!(final_frame.stage_two.is_none());
}
