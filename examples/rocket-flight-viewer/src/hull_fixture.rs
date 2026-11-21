//! Headless synthetic yaw diagnostics: prescribed speed and cooked world steps.
use std::{error::Error, io::Write};

use kapitsa::{
    aerodynamics::{
        Aerodynamics, Atmosphere, AtmosphericState, DensityProfile, MeshPressureModel,
        SphericalAtmosphere,
        colliders::{ColliderTessellation, collider_mesh},
        geometry::{
            GeometryConfig, GeometryInstance, GeometryOwner, OpenMeshPolicy, PreparedGeometry,
        },
        pressure::{OwnerPressureWorkspace, PressureFlow, PressureWrench},
    },
    collider::Collider,
    mass::MassProperties,
    math::{Mat3, Quat, Vec3},
    rigid_body::RigidBody,
    shape::{Cuboid, Shape, TriangleMesh},
    transform::Transform,
    world::PhysicsWorld,
};

const SOUND_SPEED: f64 = 340.0;
const SPEED: f64 = 4.63 * SOUND_SPEED;
const DENSITY: f64 = 0.2;
const YAW_INERTIA: f64 = 100_000.0;
const YAW_INITIAL: f64 = 0.15;
const DURATION: f64 = 1.0;
const WORK: usize = 100_000_000;
const CELLS: usize = 1_048_576;
const CELL_SIZES: [f64; 3] = [0.125, 0.0625, 0.03125];
const STEP_COUNTS: [usize; 3] = [8, 16, 32];

#[derive(Debug, PartialEq)]
struct Sample {
    yaw: f64,
    rate: f64,
    wrench: PressureWrench,
    mach: [f64; 2],
}

// Every visibility and pressure query lies on/in the transformed mesh.
// |omega x r| is bounded by |omega| times the maximum vertex radius;
// the reverse triangle inequality bounds Mach throughout the mesh at each step.
fn checked_mach(speed: f64, rate: f64, radius: f64) -> Result<[f64; 2], Box<dyn Error>> {
    let variation = rate.abs() * radius;
    let bounds = [
        (speed - variation) / SOUND_SPEED,
        (speed + variation) / SOUND_SPEED,
    ];
    if !bounds[0].is_finite() || bounds[0] < 4.60 || bounds[1] > 4.66 {
        return Err(format!("outside synthetic fixture Mach window: {bounds:?}").into());
    }
    Ok(bounds)
}

fn fixture_mesh(stabilized: bool) -> Result<(TriangleMesh, f64), Box<dyn Error>> {
    let mut colliders = vec![Collider::new(
        Shape::Cuboid(Cuboid::new(2.0, 0.5, 0.5)?),
        Transform::identity(),
    )];
    if stabilized {
        // One passive aft crossbar; it is hull geometry, not a WingSurface.
        colliders.push(Collider::new(
            Shape::Cuboid(Cuboid::new(0.25, 0.25, 2.0)?),
            Transform::new(Vec3::new(-1.125, 0.0, 0.0), Quat::IDENTITY)?,
        ));
    }
    let mesh = collider_mesh(&colliders, ColliderTessellation::new(16, 9, 4096)?)?;
    let radius = mesh
        .vertices()
        .iter()
        .map(|v| v.length())
        .fold(0.0, f64::max);
    Ok((mesh, radius))
}

fn simulate(stabilized: bool, cell: f64, steps: usize) -> Result<Vec<Sample>, Box<dyn Error>> {
    let (mesh, radius) = fixture_mesh(stabilized)?;
    let instances = [GeometryInstance {
        mesh: &mesh,
        local_transform: Transform::identity(),
        owner: GeometryOwner::Body,
    }];
    let geometry = PreparedGeometry::prepare(
        &instances,
        GeometryConfig {
            cell_size: cell,
            max_cells: CELLS,
            max_work: WORK,
            max_patch_work: WORK,
            open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
            weld_tolerance: Some(1e-10),
        },
    )?;
    let mut pressure =
        OwnerPressureWorkspace::new(&[GeometryOwner::Body]).ok_or("fixture owner")?;
    pressure.set_pressure_model(MeshPressureModel::ModifiedNewtonianTriangleQuadrature);
    let dt = DURATION / steps as f64;
    let (mut yaw, mut rate) = (YAW_INITIAL, 0.0);
    let mut samples = Vec::with_capacity(steps + 1);
    for step in 0..=steps {
        let mach = checked_mach(SPEED, rate, radius)?;
        let rotation = Quat::from_axis_angle(Vec3::Z, yaw).ok_or("invalid fixture yaw")?;
        let (sin, cos) = yaw.sin_cos();
        // Public visibility API uses one incident direction (the COM flow).
        // Pressure quadrature still evaluates local rotational velocity.
        let patches =
            geometry.exposed_surface_patches(&instances, Vec3::new(cos, -sin, 0.0), 0.01, WORK)?;
        pressure.prepare(
            &patches,
            PressureFlow {
                atmosphere: AtmosphericState::new(DENSITY, Vec3::ZERO)?,
                sound_speed: SOUND_SPEED,
                transform: Transform::new(Vec3::ZERO, rotation)?,
                center_of_mass: Vec3::ZERO,
                linear_velocity: Vec3::X * SPEED,
                angular_velocity: Vec3::Z * rate,
            },
        )?;
        let wrench = pressure
            .staged_total()?
            .ok_or("fixture wrench not staged")?;
        samples.push(Sample {
            yaw,
            rate,
            wrench,
            mach,
        });
        if step < steps {
            rate += dt * wrench.moment.z / YAW_INERTIA;
            yaw += dt * rate;
        }
    }
    Ok(samples)
}

const WORLD_MASS: f64 = 1_000_000.0;
const ATMOSPHERE_REFERENCE_RADIUS: f64 = 2_000.0;

#[derive(Debug, PartialEq)]
struct WorldSample {
    position: Vec3,
    velocity: Vec3,
    yaw: f64,
    rate: f64,
    mach: [f64; 2],
    // Published loads belong to the step ending at this row, not its end state.
    incoming_wrench: Option<PressureWrench>,
}

fn simulate_world(stabilized: bool, steps: usize) -> Result<Vec<WorldSample>, Box<dyn Error>> {
    let (mesh, radius) = fixture_mesh(stabilized)?;
    let mut world = PhysicsWorld::new();
    world.set_atmosphere(Atmosphere::Sphere(
        SphericalAtmosphere::builder()
            .reference_radius(ATMOSPHERE_REFERENCE_RADIUS)
            .reference_density(DENSITY)
            .height(1_000.0)
            .profile(DensityProfile::Exponential {
                scale_height: 1_000.0,
            })
            .build()?,
    ));
    world.set_mesh_sound_speed(SOUND_SPEED)?;
    world.set_mesh_pressure_model(MeshPressureModel::ModifiedNewtonianTriangleQuadrature);
    let rotation = Quat::from_axis_angle(Vec3::Z, YAW_INITIAL).ok_or("fixture yaw")?;
    let mut body = RigidBody::new_dynamic(
        Transform::new(Vec3::ZERO, rotation)?,
        MassProperties::new(WORLD_MASS, Vec3::ZERO, Mat3::IDENTITY * YAW_INERTIA)?,
    )?;
    body.set_linear_velocity(Vec3::X * SPEED)?;
    let id = world.add_body(body)?;
    world.set_aerodynamics(id, Aerodynamics::Mesh(mesh))?;
    world.cook_primary_hull(id)?;

    let dt = DURATION / steps as f64;
    let mut samples = Vec::with_capacity(steps + 1);
    for step in 0..=steps {
        let state = world.body(id).ok_or("fixture body")?.state();
        let transform = state.transform();
        let position = transform.translation();
        if position.length() >= ATMOSPHERE_REFERENCE_RADIUS {
            return Err("fixture left the constant-density region".into());
        }
        let velocity = state.linear_velocity();
        let angular_velocity = state.angular_velocity();
        let mach = checked_mach(velocity.length(), angular_velocity.length(), radius)?;
        let axis = transform.transform_vector(Vec3::X);
        let incoming_wrench = if step == 0 {
            None
        } else {
            let Some([(GeometryOwner::Body, wrench)]) = world.owner_mesh_pressure(id) else {
                return Err("expected published standalone hull wrench".into());
            };
            Some(*wrench)
        };
        samples.push(WorldSample {
            position,
            velocity,
            yaw: axis.y.atan2(axis.x),
            rate: angular_velocity.z,
            mach,
            incoming_wrench,
        });
        if step < steps {
            world.step(dt)?;
        }
    }
    Ok(samples)
}

pub fn world_trace(mut output: impl Write) -> Result<(), Box<dyn Error>> {
    writeln!(
        output,
        "stabilized,dt_s,time_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,yaw_rad,rate_rad_s,mach_lower,mach_upper,incoming_fx_n,incoming_fy_n,incoming_fz_n,incoming_mx_nm,incoming_my_nm,incoming_mz_nm"
    )?;
    for stabilized in [false, true] {
        for steps in STEP_COUNTS {
            let dt = DURATION / steps as f64;
            for (step, sample) in simulate_world(stabilized, steps)?.into_iter().enumerate() {
                let WorldSample {
                    position: p,
                    velocity: v,
                    yaw,
                    rate,
                    mach,
                    incoming_wrench,
                } = sample;
                write!(
                    output,
                    "{stabilized},{dt},{},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{yaw:.12e},{rate:.12e},{:.12e},{:.12e}",
                    step as f64 * dt,
                    p.x,
                    p.y,
                    p.z,
                    v.x,
                    v.y,
                    v.z,
                    mach[0],
                    mach[1]
                )?;
                if let Some(wrench) = incoming_wrench {
                    let (f, m) = (wrench.force, wrench.moment);
                    writeln!(
                        output,
                        ",{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e}",
                        f.x, f.y, f.z, m.x, m.y, m.z
                    )?;
                } else {
                    writeln!(output, ",,,,,,")?;
                }
            }
        }
    }
    Ok(())
}

pub fn trace(mut output: impl Write) -> Result<(), Box<dyn Error>> {
    writeln!(
        output,
        "stabilized,cell_m,dt_s,time_s,yaw_rad,rate_rad_s,fx_n,fy_n,fz_n,mx_nm,my_nm,mz_nm,mach_lower,mach_upper"
    )?;
    for stabilized in [false, true] {
        for (cell, steps) in CELL_SIZES
            .into_iter()
            .map(|cell| (cell, STEP_COUNTS[2]))
            .chain(STEP_COUNTS[..2].iter().map(|&steps| (CELL_SIZES[2], steps)))
        {
            let dt = DURATION / steps as f64;
            for (
                step,
                Sample {
                    yaw,
                    rate,
                    wrench,
                    mach,
                },
            ) in simulate(stabilized, cell, steps)?.into_iter().enumerate()
            {
                let f = wrench.force;
                let m = wrench.moment;
                writeln!(
                    output,
                    "{stabilized},{cell},{dt},{},{yaw:.12e},{rate:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e}",
                    step as f64 * dt,
                    f.x,
                    f.y,
                    f.z,
                    m.x,
                    m.y,
                    m.z,
                    mach[0],
                    mach[1]
                )?;
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cooked_world_applies_hull_wrench_and_refines_with_timestep() -> Result<(), Box<dyn Error>> {
        for stabilized in [false, true] {
            let mut final_yaws = Vec::new();
            for steps in STEP_COUNTS {
                let samples = simulate_world(stabilized, steps)?;
                assert_eq!(samples, simulate_world(stabilized, steps)?);
                let dt = DURATION / steps as f64;
                for pair in samples.windows(2) {
                    let [start, end] = pair else {
                        return Err("missing step states".into());
                    };
                    let load = end.incoming_wrench.ok_or("missing step load")?;
                    assert!(load.force.x < 0.0);
                    let expected_delta_v = load.force * (dt / WORLD_MASS);
                    // Velocity subtraction loses precision at the large COM speed.
                    assert!(
                        (end.velocity - start.velocity - expected_delta_v).length()
                            < 8.0 * f64::EPSILON * start.velocity.length()
                    );
                    let expected_delta_rate = dt * load.moment.z / YAW_INERTIA;
                    assert!((end.rate - start.rate - expected_delta_rate).abs() < 1e-12);
                    if stabilized {
                        assert!(load.moment.z < 0.0);
                        assert!(end.yaw > 0.0 && end.yaw < start.yaw);
                    } else {
                        assert!((end.yaw - YAW_INITIAL).abs() < 1e-12);
                    }
                }
                final_yaws.push(samples.last().ok_or("missing final state")?.yaw);
            }
            if stabilized {
                let first = (final_yaws[1] - final_yaws[0]).abs();
                let second = (final_yaws[2] - final_yaws[1]).abs();
                assert!((0.4..0.6).contains(&(second / first)));
            }
        }
        Ok(())
    }

    #[test]
    fn rejects_local_mach_outside_fixture_window() -> Result<(), Box<dyn Error>> {
        assert_eq!(checked_mach(SPEED, 0.0, 2.0)?, [4.63; 2]);
        assert!(checked_mach(SPEED, 10.0, 2.0).is_err());
        assert!(checked_mach(SPEED, f64::NAN, 2.0).is_err());
        Ok(())
    }

    #[test]
    fn yaw_and_loads_under_grid_and_time_refinement() -> Result<(), Box<dyn Error>> {
        for stabilized in [false, true] {
            let fine = simulate(stabilized, CELL_SIZES[2], STEP_COUNTS[2])?;
            assert_eq!(fine, simulate(stabilized, CELL_SIZES[2], STEP_COUNTS[2])?);
            let end = fine.last().ok_or("missing final sample")?;
            for sample in &fine {
                assert!(sample.wrench.force.x < 0.0);
                assert!(sample.wrench.force.is_finite() && sample.wrench.moment.is_finite());
                if stabilized {
                    assert!(sample.wrench.moment.z < 0.0);
                    assert!(sample.yaw > 0.0 && sample.yaw <= YAW_INITIAL);
                } else {
                    // Centered windward faces have zero moment at zero spin.
                    assert!(sample.wrench.moment.length() < 1e-10 * sample.wrench.force.length());
                    assert!((sample.yaw - YAW_INITIAL).abs() < 1e-12);
                }
            }
            for cell in &CELL_SIZES[..2] {
                let trial = simulate(stabilized, *cell, STEP_COUNTS[2])?;
                for (a, b) in trial.iter().zip(&fine) {
                    // Fixed facets and resolved box geometry; allow accumulation
                    // roundoff relative to force and force times the 2 m scale.
                    let scale = b.wrench.force.length();
                    assert!((a.wrench.force - b.wrench.force).length() < 1e-10 * scale);
                    assert!((a.wrench.moment - b.wrench.moment).length() < 2e-10 * scale);
                    assert!((a.yaw - b.yaw).abs() < 1e-12);
                }
            }
            let final_yaw = |steps| -> Result<f64, Box<dyn Error>> {
                simulate(stabilized, CELL_SIZES[2], steps)?
                    .last()
                    .map(|sample| sample.yaw)
                    .ok_or("missing final sample".into())
            };
            let coarse = final_yaw(STEP_COUNTS[0])?;
            let medium = final_yaw(STEP_COUNTS[1])?;
            if stabilized {
                assert!(end.yaw < YAW_INITIAL && end.rate < 0.0);
                let coarse_delta = (coarse - medium).abs();
                let fine_delta = (medium - end.yaw).abs();
                // Symplectic Euler is first order, so successive differences
                // should approximately halve over this short smooth trace.
                assert!((0.4..0.6).contains(&(fine_delta / coarse_delta)));
            } else {
                assert!((coarse - end.yaw).abs() < 1e-12);
                assert!((medium - end.yaw).abs() < 1e-12);
            }
        }
        Ok(())
    }
}
