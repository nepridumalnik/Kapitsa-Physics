use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    collision::{CollisionStepMetrics, QueryFilter, Ray},
    mass::MassProperties,
    rigid_body::{CcdMode, RigidBody},
    shape::{Cuboid, HeightField, LevelSet, Shape, Sphere, TriangleMesh},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec2, Vec3};

const SAMPLES: usize = 5;
const WARMUP_RUNS: usize = 1;
const DT: f64 = 1.0 / 60.0;

struct Sample {
    elapsed: Duration,
    metrics: CollisionStepMetrics,
}

fn main() -> Result<(), Box<dyn Error>> {
    println!(
        "Kapitsa collision benchmark: warmups={WARMUP_RUNS}, samples={SAMPLES}, statistics=min/median/max"
    );
    for (name, build) in [
        ("256 Dynamic Sphere", separated_spheres as fn() -> _),
        ("4096 Dynamic Sphere", || separated_spheres_of(4_096)),
        ("20,000 Dynamic Sphere, no contacts", || {
            separated_spheres_of(20_000)
        }),
        ("20,000 bodies, localized contacts", || {
            contact_pairs(20_000)
        }),
        ("1,000 bodies, one contact island", dense_contact_group),
        ("20,000 Static + 100 Dynamic", mixed_static_dynamic),
        ("Large TriangleMesh + movers", triangle_mesh_scene),
        ("HeightField + colliders", height_field_scene),
        ("LevelSet + colliders", level_set_scene),
        ("CCD fast bodies", ccd_scene),
    ] {
        report_world(name, build)?;
    }
    report_queries()?;
    Ok(())
}

fn separated_spheres() -> Result<PhysicsWorld, Box<dyn Error>> {
    separated_spheres_of(256)
}

fn separated_spheres_of(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    for index in 0..count {
        let x = (index % 100) as f64 * 2.0;
        let y = (index / 100) as f64 * 2.0;
        add_sphere(&mut world, Vec3::new(x, y, 0.0), Vec3::ZERO, mass, false)?;
    }
    Ok(world)
}

fn contact_pairs(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    let pair_count = (count / 2).min(128);
    for pair in 0..pair_count {
        let x = (pair % 32) as f64 * 3.0;
        let z = (pair / 32) as f64 * 3.0;
        add_sphere(&mut world, Vec3::new(x, 0.0, z), Vec3::ZERO, mass, false)?;
        add_sphere(
            &mut world,
            Vec3::new(x + 0.9, 0.0, z),
            Vec3::ZERO,
            mass,
            false,
        )?;
    }
    for index in pair_count * 2..count {
        let index = index - pair_count * 2;
        let x = (index % 200) as f64 * 2.0;
        let z = (index / 200) as f64 * 2.0;
        add_sphere(&mut world, Vec3::new(x, 10.0, z), Vec3::ZERO, mass, false)?;
    }
    Ok(world)
}

fn dense_contact_group() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    for index in 0..1_000 {
        let body = world.add_body(RigidBody::new_dynamic(
            Transform::new(Vec3::new(index as f64 * 0.99, 0.0, 0.0), Quat::IDENTITY)?,
            mass,
        )?)?;
        world.add_collider(
            body,
            Collider::new(
                Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0)?),
                Transform::identity(),
            ),
        )?;
    }
    Ok(world)
}

fn mixed_static_dynamic() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    for index in 0..20_000 {
        let x = (index % 200) as f64 * 4.0;
        let z = (index / 200) as f64 * 4.0;
        add_static_sphere(&mut world, Vec3::new(x, 0.0, z))?;
    }
    let mass = mass()?;
    for index in 0..100 {
        let x = (index % 20) as f64 * 40.0 + 0.9;
        let z = (index / 20) as f64 * 40.0;
        add_sphere(&mut world, Vec3::new(x, 100.0, z), Vec3::ZERO, mass, false)?;
    }
    Ok(world)
}

fn triangle_mesh_scene() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mut vertices = Vec::with_capacity(101 * 101);
    for z in 0..=100 {
        for x in 0..=100 {
            vertices.push(Vec3::new(x as f64 - 50.0, 0.0, z as f64 - 50.0));
        }
    }
    let mut triangles = Vec::with_capacity(100 * 100 * 2);
    for z in 0..100 {
        for x in 0..100 {
            let a = z * 101 + x;
            let b = a + 1;
            let c = a + 101;
            let d = c + 1;
            triangles.push([a, b, d]);
            triangles.push([a, d, c]);
        }
    }
    let mesh = TriangleMesh::new(vertices, triangles)?;
    let mesh_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        mesh_body,
        Collider::new(Shape::TriangleMesh(mesh), Transform::identity()),
    )?;
    let mass = mass()?;
    add_surface_movers(&mut world, 100, mass)?;
    Ok(world)
}

fn height_field_scene() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let field = HeightField::from_vec(65, 65, Vec2::splat(2.0), vec![0.0; 65 * 65])?;
    let field_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let rotation = Quat::from_axis_angle(Vec3::X, std::f64::consts::FRAC_PI_2)
        .ok_or("invalid field rotation")?;
    world.add_collider(
        field_body,
        Collider::new(
            Shape::HeightField(field),
            Transform::new(Vec3::new(-64.0, 0.0, -64.0), rotation)?,
        ),
    )?;
    let mass = mass()?;
    add_surface_movers(&mut world, 512, mass)?;
    Ok(world)
}

fn level_set_scene() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let dimensions = [65, 4, 65];
    let mut distances = Vec::with_capacity(dimensions.iter().product());
    for _z in 0..dimensions[2] {
        for y in 0..dimensions[1] {
            for _x in 0..dimensions[0] {
                distances.push(y as f64 - 2.0);
            }
        }
    }
    let field = LevelSet::from_vec(
        Vec3::new(-32.0, -2.0, -32.0),
        dimensions,
        Vec3::ONE,
        distances,
    )?;
    let field_body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        field_body,
        Collider::new(Shape::LevelSet(field), Transform::identity()),
    )?;
    let mass = mass()?;
    add_surface_movers(&mut world, 256, mass)?;
    Ok(world)
}

fn ccd_scene() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let wall = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        wall,
        Collider::new(
            Shape::Cuboid(kapitsa_core::shape::Cuboid::new(0.1, 200.0, 200.0)?),
            Transform::identity(),
        ),
    )?;
    let mass = mass()?;
    for index in 0..64 {
        let y = (index % 8) as f64 * 2.0;
        let z = (index / 8) as f64 * 2.0;
        add_sphere_with_velocity(
            &mut world,
            Vec3::new(-5.0, y, z),
            Vec3::new(1000.0, 0.0, 0.0),
            mass,
            true,
        )?;
    }
    Ok(world)
}

fn add_surface_movers(
    world: &mut PhysicsWorld,
    count: usize,
    mass: MassProperties,
) -> Result<(), Box<dyn Error>> {
    for index in 0..count {
        let x = (index % 32) as f64 * 2.0 - 31.0;
        let z = (index / 32) as f64 * 2.0 - 15.0;
        add_sphere_with_velocity(
            world,
            Vec3::new(x, 0.6, z),
            Vec3::new(0.0, -10.0, 0.0),
            mass,
            false,
        )?;
    }
    Ok(())
}

fn add_static_sphere(world: &mut PhysicsWorld, position: Vec3) -> Result<(), Box<dyn Error>> {
    let body = world.add_body(RigidBody::new_static(Transform::new(
        position,
        Quat::IDENTITY,
    )?))?;
    world.add_collider(
        body,
        Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
    )?;
    Ok(())
}

fn add_sphere(
    world: &mut PhysicsWorld,
    position: Vec3,
    velocity: Vec3,
    mass: MassProperties,
    continuous: bool,
) -> Result<(), Box<dyn Error>> {
    add_sphere_with_velocity(world, position, velocity, mass, continuous)
}

fn add_sphere_with_velocity(
    world: &mut PhysicsWorld,
    position: Vec3,
    velocity: Vec3,
    mass: MassProperties,
    continuous: bool,
) -> Result<(), Box<dyn Error>> {
    let mut body = RigidBody::new_dynamic(Transform::new(position, Quat::IDENTITY)?, mass)?;
    body.set_linear_velocity(velocity)?;
    if continuous {
        body.set_ccd_mode(CcdMode::Continuous);
    }
    let id = world.add_body(body)?;
    world.add_collider(
        id,
        Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
    )?;
    Ok(())
}

fn mass() -> Result<MassProperties, Box<dyn Error>> {
    Ok(MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?)
}

fn report_world(
    name: &str,
    build: impl Fn() -> Result<PhysicsWorld, Box<dyn Error>>,
) -> Result<(), Box<dyn Error>> {
    for _ in 0..WARMUP_RUNS {
        let mut world = build()?;
        black_box(world.step(DT)?);
    }
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let mut world = build()?;
        let started = Instant::now();
        world.step(DT)?;
        samples.push(Sample {
            elapsed: started.elapsed(),
            metrics: world.collision_step_metrics(),
        });
    }
    samples.sort_unstable_by_key(|sample| sample.elapsed);
    let median = &samples[SAMPLES / 2];
    let min = samples.first().ok_or("missing benchmark sample")?;
    let max = samples.last().ok_or("missing benchmark sample")?;
    let high_water = samples
        .iter()
        .map(|sample| sample.metrics.retained_scratch_bytes)
        .max()
        .unwrap_or(0);
    println!(
        "{name:38} step={:.3}/{:8.3}/{:8.3} ms min/median/max | candidates={} contacts={} ccd_impacts={} islands={} | index={:.3} broad={:.3} narrow={:.3} ccd={:.3} islands={:.3} solver={:.3} (prepare={:.3}, iterations={:.3}, writeback={:.3}) events={:.3} commit={:.3} ms | body_copies={} manifold_copies={} growth={} B retained_scratch~{} KiB query_index~{} KiB peak_sampled~{} KiB",
        millis(min.elapsed),
        millis(median.elapsed),
        millis(max.elapsed),
        median.metrics.candidate_count,
        median.metrics.contact_count,
        median.metrics.ccd_impact_count,
        median.metrics.island_count,
        millis(median.metrics.spatial_index),
        millis(median.metrics.broad_phase),
        millis(median.metrics.narrow_phase),
        millis(median.metrics.ccd),
        millis(median.metrics.island_build),
        millis(median.metrics.solver),
        millis(median.metrics.solver_prepare),
        millis(median.metrics.solver_iterations),
        millis(median.metrics.solver_writeback),
        millis(median.metrics.events),
        millis(median.metrics.commit),
        median.metrics.solver_body_copies,
        median.metrics.solver_manifold_copies,
        median.metrics.logical_capacity_growth_bytes,
        median.metrics.retained_scratch_bytes / 1024,
        median.metrics.committed_query_index_bytes / 1024,
        median.metrics.peak_scratch_bytes.max(high_water) / 1024,
    );
    Ok(())
}

fn report_queries() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    for index in 0..20_000 {
        let x = (index % 200) as f64 * 2.0;
        let y = (index / 200) as f64 * 2.0;
        add_static_sphere(&mut world, Vec3::new(x, y, 0.0))?;
    }
    let ray = Ray::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::X, 500.0)?;
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..WARMUP_RUNS {
        for _ in 0..1_000 {
            black_box(world.raycast(ray, QueryFilter::default())?);
        }
    }
    for _ in 0..SAMPLES {
        let started = Instant::now();
        for _ in 0..1_000 {
            black_box(world.raycast(ray, QueryFilter::default())?);
        }
        samples.push(started.elapsed());
    }
    samples.sort_unstable();
    println!(
        "{:38} 1,000 rays={:.3}/{:.3}/{:.3} ms min/median/max | colliders={} | median={:.3} us/query",
        "Spatial queries under load",
        millis(*samples.first().ok_or("missing benchmark sample")?),
        millis(samples[SAMPLES / 2]),
        millis(*samples.last().ok_or("missing benchmark sample")?),
        world.colliders().count(),
        samples[SAMPLES / 2].as_secs_f64() * 1.0e6 / 1_000.0,
    );
    Ok(())
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
}
