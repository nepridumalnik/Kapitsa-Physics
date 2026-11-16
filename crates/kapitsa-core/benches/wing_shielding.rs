//! End-to-end voxel wing shielding workload. Run with
//! `cargo bench -p kapitsa-core --bench wing_shielding`.
use std::{error::Error, hint::black_box, time::Instant};

use kapitsa_core::{
    aerodynamics::{
        Atmosphere, DensityProfile, SphericalAtmosphere,
        wing::{FiniteWing, WingSurface},
    },
    collider::{Collider, CollisionMode},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Cuboid, Shape},
    transform::Transform,
    world::{BodyId, PhysicsWorld},
};
use kapitsa_math::{Mat3, Vec3};

fn world(wing_count: usize, footprint: bool) -> Result<(PhysicsWorld, BodyId), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    world.set_atmosphere(Atmosphere::Sphere(
        SphericalAtmosphere::builder()
            .reference_radius(1_000_000.0)
            .reference_density(1.0)
            .height(1_000_000.0)
            .profile(DensityProfile::Exponential {
                scale_height: 1_000_000.0,
            })
            .build()?,
    ));
    let mass = MassProperties::new(100.0, Vec3::ZERO, Mat3::IDENTITY * 100.0)?;
    let mut body = RigidBody::new_dynamic(Transform::identity(), mass)?;
    body.set_linear_velocity(Vec3::new(50.0, 0.0, -5.0))?;
    let id = world.add_body(body)?;
    let part = world.enable_cluster(id)?;
    let mut collider = Collider::new(
        Shape::Cuboid(Cuboid::new(20.0, 4.0, 0.5)?),
        Transform::identity(),
    );
    collider.set_mode(CollisionMode::Sensor);
    world.add_part_collider(part, collider)?;
    world.set_part_wave_source_colliders(part, true)?;
    let model = FiniteWing::new(5.0, 0.8, 0.02, 0.0, 0.3)?;
    let mut wings = Vec::new();
    wings.try_reserve_exact(wing_count)?;
    for _ in 0..wing_count {
        let wing = WingSurface::new(model, Transform::identity(), 80.0)?;
        wings.push(if footprint {
            wing.with_rectangular_footprint(Vec3::ZERO, 20.0, 4.0)?
        } else {
            wing
        });
    }
    world.set_part_aerodynamic_wings(part, wings)?;
    Ok((world, id))
}

fn measure(name: &str, world: &mut PhysicsWorld, iterations: usize) -> Result<(), Box<dyn Error>> {
    for _ in 0..3 {
        world.step(1e-10)?;
    }
    let mut samples = Vec::new();
    for _ in 0..7 {
        let start = Instant::now();
        for _ in 0..iterations {
            world.step(black_box(1e-10))?;
        }
        samples.push(start.elapsed().as_secs_f64() * 1e6 / iterations as f64);
    }
    samples.sort_by(f64::total_cmp);
    let median = samples.get(samples.len() / 2).ok_or("empty benchmark")?;
    println!("{name}: median={median:.2} us/step samples={samples:?}");
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    for count in [1, 32] {
        let (mut baseline, baseline_id) = world(count, false)?;
        let (mut shielded, shielded_id) = world(count, true)?;
        let baseline_before = baseline
            .body(baseline_id)
            .ok_or("baseline body")?
            .state()
            .linear_velocity();
        let shielded_before = shielded
            .body(shielded_id)
            .ok_or("shielded body")?
            .state()
            .linear_velocity();
        baseline.step(1e-10)?;
        shielded.step(1e-10)?;
        let baseline_after = baseline
            .body(baseline_id)
            .ok_or("baseline body")?
            .state()
            .linear_velocity();
        let shielded_after = shielded
            .body(shielded_id)
            .ok_or("shielded body")?
            .state()
            .linear_velocity();
        assert!(baseline_after.z > baseline_before.z);
        assert_eq!(shielded_after, shielded_before);
        measure(&format!("wings={count} no footprint"), &mut baseline, 30)?;
        measure(&format!("wings={count} footprint"), &mut shielded, 30)?;
    }
    Ok(())
}
