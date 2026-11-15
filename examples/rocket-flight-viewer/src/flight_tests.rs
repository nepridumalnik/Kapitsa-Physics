use crate::flight::{FlightConfig, STAGE_TWO_BURN_END, run};

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn viewer_flight_with_explicit_aerodynamics_reaches_separation()
-> Result<(), Box<dyn std::error::Error>> {
    use crate::flight::{FlightAerodynamics, run_with_aerodynamics};
    use kapitsa::{
        Transform,
        aerodynamics::{
            Aerodynamics, Atmosphere, DensityProfile, SphericalAtmosphere,
            hull::{HullSection, MachCurve, MachKnot},
            wing::{FiniteWing, WingCoefficients, WingPolar, WingSurface},
        },
        math::{Quat, Vec3},
    };
    use std::sync::Arc;
    // Synthetic data: checks wiring and staging, not physical prediction.
    let atmosphere = Atmosphere::Sphere(
        SphericalAtmosphere::builder()
            .reference_radius(crate::flight::PLANET_RADIUS)
            .reference_density(1.2)
            .height(100_000.0)
            .profile(DensityProfile::Exponential {
                scale_height: 8_000.0,
            })
            .wind(-Vec3::Y * 10.0)
            .build()?,
    );
    let hull = HullSection::new(
        Transform::identity(),
        Vec3::Y,
        4.0,
        1.0,
        4.0,
        0.0,
        0.3,
        0.8,
        None,
    )?
    .with_mach_curve(MachCurve::new(vec![
        MachKnot {
            mach: 0.0,
            axial_cd: 0.3,
            crossflow_cd: 0.8,
        },
        MachKnot {
            mach: 2.0,
            axial_cd: 0.5,
            crossflow_cd: 1.0,
        },
    ])?);
    let polar = Arc::new(WingPolar::new(
        vec![0.0, 2.0],
        vec![-std::f64::consts::PI, std::f64::consts::PI],
        vec![
            WingCoefficients {
                lift: 0.1,
                drag: 0.1
            };
            4
        ],
    )?);
    let wing = WingSurface::new(
        FiniteWing::new(6.0, 0.8, 0.02, 0.0, 0.2)?,
        Transform::new(
            Vec3::Y,
            Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2).ok_or("rotation")?,
        )?,
        1.0,
    )?
    .with_polar(polar);
    let frames = run_with_aerodynamics(
        FlightConfig {
            warp: 1.0,
            maximum_steps: 360,
            fps: 24,
        },
        Some(FlightAerodynamics {
            atmosphere,
            lower: Aerodynamics::Hull(vec![hull]),
            upper: Aerodynamics::Wing(wing),
        }),
    )
    .await?;
    let after = frames
        .iter()
        .find(|f| f.simulation_time > 2.5)
        .ok_or("no frame after separation")?;
    let lower = after.stage_one.ok_or("lower stage missing")?;
    let upper = after.stage_two.ok_or("upper stage missing")?;
    assert!((upper.linear_velocity() - lower.linear_velocity()).length() > 0.1);
    assert!(upper.angular_velocity().length() > 1e-7);
    println!(
        "time={} lower_velocity={:?} upper_velocity={:?} upper_angular={:?}",
        after.simulation_time,
        lower.linear_velocity(),
        upper.linear_velocity(),
        upper.angular_velocity()
    );
    Ok(())
}

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
