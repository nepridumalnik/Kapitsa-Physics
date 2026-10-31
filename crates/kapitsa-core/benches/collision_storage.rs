use std::{
    error::Error,
    hint::black_box,
    time::{Duration, Instant},
};

use kapitsa_core::{
    collider::Collider,
    collision::{CollisionStepMetrics, QueryFilter, Ray},
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{BoneInfluence, Cuboid, Shape, SkinnedTriangleMesh, Sphere, TriangleMesh},
    transform::Transform,
    world::{ColliderId, PhysicsWorld},
};
use kapitsa_math::{Affine3, Mat3, Quat, Vec3};

const WARMUPS: usize = 1;
const SAMPLES: usize = 5;
const DT: f64 = 1.0 / 60.0;

type BuildWorld = fn() -> Result<PhysicsWorld, Box<dyn Error>>;

fn main() -> Result<(), Box<dyn Error>> {
    let selected = std::env::args().skip(1).find(|argument| {
        matches!(
            argument.as_str(),
            "island_1000"
                | "islands_500"
                | "stationary_1000"
                | "moving_1000"
                | "dynamic_static_500"
                | "contacts_1000"
                | "queries_1000"
                | "sweeps_1000"
                | "skin_1k"
                | "skin_10k"
                | "skin_100k"
        )
    });
    println!(
        "Collision storage benchmark: warmups={WARMUPS}, samples={SAMPLES}, statistics=min/median/max"
    );

    for (tag, label, build) in [
        (
            "island_1000",
            "1 island x 1,000 dynamic",
            one_island as BuildWorld,
        ),
        (
            "islands_500",
            "500 islands x 2 dynamic",
            five_hundred_islands,
        ),
        (
            "stationary_1000",
            "1,000 separated Dynamic, stationary",
            stationary_dynamic_1000,
        ),
        (
            "moving_1000",
            "1,000 separated Dynamic, moving",
            moving_dynamic_1000,
        ),
        (
            "dynamic_static_500",
            "500 Dynamic-Static pairs",
            dynamic_static_pairs,
        ),
        (
            "contacts_1000",
            "1,000 Dynamic / 4,000 Static",
            thousand_dynamic_static_contacts,
        ),
    ] {
        if selected.as_deref().is_none_or(|filter| filter == tag) {
            report_steps(label, build)?;
        }
    }

    if selected
        .as_deref()
        .is_none_or(|filter| filter == "queries_1000")
    {
        report_queries()?;
    }
    if selected
        .as_deref()
        .is_none_or(|filter| filter == "sweeps_1000")
    {
        report_sweeps()?;
    }
    for (tag, count) in [
        ("skin_1k", 1_024),
        ("skin_10k", 10_000),
        ("skin_100k", 100_172),
    ] {
        if selected.as_deref().is_none_or(|filter| filter == tag) {
            report_skin_updates(tag, count)?;
        }
    }
    Ok(())
}

fn report_steps(label: &str, build: BuildWorld) -> Result<(), Box<dyn Error>> {
    for _ in 0..WARMUPS {
        let mut world = build()?;
        black_box(world.step(DT)?);
    }

    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let mut world = build()?;
        let started = Instant::now();
        black_box(world.step(DT)?);
        samples.push((started.elapsed(), world.collision_step_metrics()));
    }
    samples.sort_unstable_by_key(|sample| sample.0);
    let (min, _) = samples.first().ok_or("missing benchmark sample")?;
    let (median, metrics) = samples.get(SAMPLES / 2).ok_or("missing benchmark sample")?;
    let (max, _) = samples.last().ok_or("missing benchmark sample")?;
    print_step_metrics(label, *min, *median, *max, *metrics);
    Ok(())
}

fn print_step_metrics(
    label: &str,
    min: Duration,
    median: Duration,
    max: Duration,
    metrics: CollisionStepMetrics,
) {
    println!(
        "{label:40} step={:.3}/{:.3}/{:.3} ms min/median/max | candidates={} contacts={} islands={} | island={:.3} ms solver={:.3} ms events={:.3} ms commit={:.3} ms | body_copies={} manifold_copies={} growth={} B retained_scratch={} B query_index={} B peak_sampled={} B",
        millis(min),
        millis(median),
        millis(max),
        metrics.candidate_count,
        metrics.contact_count,
        metrics.island_count,
        millis(metrics.island_build),
        millis(metrics.solver),
        millis(metrics.events),
        millis(metrics.commit),
        metrics.solver_body_copies,
        metrics.solver_manifold_copies,
        metrics.logical_capacity_growth_bytes,
        metrics.retained_scratch_bytes,
        metrics.committed_query_index_bytes,
        metrics.peak_scratch_bytes,
    );
}

fn one_island() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    for index in 0..1_000 {
        let position = Vec3::new(index as f64 * 0.99, 0.0, 0.0);
        add_dynamic(
            &mut world,
            position,
            Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0)?),
            mass,
        )?;
    }
    Ok(world)
}

fn five_hundred_islands() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    let sphere = || Sphere::new(0.5);
    for pair in 0..500 {
        let x = pair as f64 * 3.0;
        add_dynamic(
            &mut world,
            Vec3::new(x, 0.0, 0.0),
            Shape::Sphere(sphere()?),
            mass,
        )?;
        add_dynamic(
            &mut world,
            Vec3::new(x + 0.9, 0.0, 0.0),
            Shape::Sphere(sphere()?),
            mass,
        )?;
    }
    Ok(world)
}

fn dynamic_static_pairs() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    for pair in 0..500 {
        let x = pair as f64 * 3.0;
        add_static_sphere(&mut world, Vec3::new(x, 0.0, 0.0))?;
        add_dynamic(
            &mut world,
            Vec3::new(x + 0.9, 0.0, 0.0),
            Shape::Sphere(Sphere::new(0.5)?),
            mass,
        )?;
    }
    Ok(world)
}

fn thousand_dynamic_static_contacts() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    for index in 0..1_000 {
        let center = Vec3::new(index as f64 * 5.0, 0.0, 0.0);
        add_dynamic(&mut world, center, Shape::Sphere(Sphere::new(0.5)?), mass)?;
        for offset in [
            Vec3::new(0.9, 0.0, 0.0),
            Vec3::new(-0.9, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 0.9),
            Vec3::new(0.0, 0.0, -0.9),
        ] {
            add_static_sphere(&mut world, center + offset)?;
        }
    }
    Ok(world)
}

fn stationary_dynamic_1000() -> Result<PhysicsWorld, Box<dyn Error>> {
    separated_dynamic_1000(Vec3::ZERO)
}

fn moving_dynamic_1000() -> Result<PhysicsWorld, Box<dyn Error>> {
    separated_dynamic_1000(Vec3::X)
}

fn separated_dynamic_1000(velocity: Vec3) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    for index in 0..1_000 {
        let mut body = RigidBody::new_dynamic(
            Transform::new(Vec3::new(index as f64 * 2.0, 0.0, 0.0), Quat::IDENTITY)?,
            mass,
        )?;
        body.set_linear_velocity(velocity)?;
        let body = world.add_body(body)?;
        world.add_collider(
            body,
            Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
        )?;
    }
    Ok(world)
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
    for _ in 0..WARMUPS {
        run_rays(&world, ray)?;
    }
    for _ in 0..SAMPLES {
        let started = Instant::now();
        run_rays(&world, ray)?;
        samples.push(started.elapsed());
    }
    samples.sort_unstable();
    println!(
        "{:<32} 1,000 rays={:.3}/{:.3}/{:.3} ms min/median/max | {} colliders",
        "20,000 Static + 1,000 Raycast",
        millis(*samples.first().ok_or("missing benchmark sample")?),
        millis(samples[SAMPLES / 2]),
        millis(*samples.last().ok_or("missing benchmark sample")?),
        world.colliders().count(),
    );
    Ok(())
}

fn run_rays(world: &PhysicsWorld, ray: Ray) -> Result<(), Box<dyn Error>> {
    for _ in 0..1_000 {
        black_box(world.raycast(ray, QueryFilter::default())?);
    }
    Ok(())
}

fn report_sweeps() -> Result<(), Box<dyn Error>> {
    for count in [100, 1_000, 10_000] {
        report_sweep_variant(count)?;
    }
    Ok(())
}

fn report_sweep_variant(count: usize) -> Result<(), Box<dyn Error>> {
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..WARMUPS {
        let (world, shape, transform) = sweep_scene()?;
        run_world_sweeps(&world, &shape, transform, count)?;
    }
    for _ in 0..SAMPLES {
        let (world, shape, transform) = sweep_scene()?;
        black_box(world.sweep(
            &shape,
            transform,
            Vec3::new(0.0, -3.0, 0.0),
            QueryFilter::default(),
        )?);
        let started = Instant::now();
        run_world_sweeps(&world, &shape, transform, count)?;
        samples.push(started.elapsed());
    }
    samples.sort_unstable();
    println!(
        "{:28} {count:>5} sweeps={:.3}/{:.3}/{:.3} ms min/median/max",
        "world.sweep",
        millis(*samples.first().ok_or("missing benchmark sample")?),
        millis(samples[SAMPLES / 2]),
        millis(*samples.last().ok_or("missing benchmark sample")?),
    );
    Ok(())
}

fn sweep_scene() -> Result<(PhysicsWorld, Shape, Transform), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mesh = grid_mesh(100, 101)?;
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        body,
        Collider::new(Shape::TriangleMesh(mesh), Transform::identity()),
    )?;
    let shape = Shape::Sphere(Sphere::new(0.5)?);
    let transform = Transform::new(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY)?;
    Ok((world, shape, transform))
}

fn run_world_sweeps(
    world: &PhysicsWorld,
    shape: &Shape,
    transform: Transform,
    count: usize,
) -> Result<(), Box<dyn Error>> {
    for _ in 0..count {
        black_box(world.sweep(
            shape,
            transform,
            Vec3::new(0.0, -3.0, 0.0),
            QueryFilter::default(),
        )?);
    }
    Ok(())
}

fn report_skin_updates(tag: &str, vertex_count: usize) -> Result<(), Box<dyn Error>> {
    const BONE_COUNT: usize = 64;
    for all_bones in [false, true] {
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..WARMUPS {
            let (mut world, collider) = skin_scene(vertex_count, BONE_COUNT)?;
            let mut matrices = vec![Affine3::IDENTITY; BONE_COUNT];
            update_skin_pose(&mut world, collider, &mut matrices, all_bones)?;
        }
        for _ in 0..SAMPLES {
            let (mut world, collider) = skin_scene(vertex_count, BONE_COUNT)?;
            let mut matrices = vec![Affine3::IDENTITY; BONE_COUNT];
            let before = world
                .collider(collider)
                .map_or(0, Collider::skin_pose_retained_capacity_bytes);
            let started = Instant::now();
            update_skin_pose(&mut world, collider, &mut matrices, all_bones)?;
            let elapsed = started.elapsed();
            let after = world
                .collider(collider)
                .map_or(0, Collider::skin_pose_retained_capacity_bytes);
            samples.push((elapsed, before, after));
        }
        samples.sort_unstable_by_key(|sample| sample.0);
        let (min, _, _) = samples.first().ok_or("missing benchmark sample")?;
        let (median, before, after) = samples.get(SAMPLES / 2).ok_or("missing benchmark sample")?;
        let (max, _, _) = samples.last().ok_or("missing benchmark sample")?;
        println!(
            "{tag:10} {:10} 1,000 updates={:.3}/{:.3}/{:.3} ms min/median/max | vertices={vertex_count} retained={} -> {} B growth={} B",
            if all_bones { "all bones" } else { "one bone" },
            millis(*min),
            millis(*median),
            millis(*max),
            before,
            after,
            after.saturating_sub(*before),
        );
    }
    Ok(())
}

fn update_skin_pose(
    world: &mut PhysicsWorld,
    collider: ColliderId,
    matrices: &mut [Affine3],
    all_bones: bool,
) -> Result<(), Box<dyn Error>> {
    let changed_bones = if all_bones { matrices.len() } else { 1 };
    for update in 0..1_000 {
        let offset = if update % 2 == 0 { 0.001 } else { -0.001 };
        for (bone, matrix) in matrices.iter_mut().take(changed_bones).enumerate() {
            let bone_scale = if all_bones { (bone + 1) as f64 } else { 1.0 };
            *matrix = Affine3::from_translation(Vec3::new(offset * bone_scale, 0.0, 0.0));
        }
        black_box(world.set_skin_pose(collider, matrices)?);
    }
    Ok(())
}

fn skin_scene(
    vertex_count: usize,
    bone_count: usize,
) -> Result<(PhysicsWorld, ColliderId), Box<dyn Error>> {
    let width = (vertex_count as f64).sqrt() as usize;
    let height = vertex_count.div_ceil(width);
    let mesh = grid_mesh(width, height)?;
    let mut influences = Vec::with_capacity(mesh.vertices().len());
    for vertex in 0..mesh.vertices().len() {
        influences.push(vec![BoneInfluence::new(vertex % bone_count, 1.0)?]);
    }
    let skinned = SkinnedTriangleMesh::new(
        mesh,
        (0..bone_count).map(|bone| format!("bone-{bone}")).collect(),
        influences,
        Affine3::IDENTITY,
        vec![Affine3::IDENTITY; bone_count],
    )?;
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let collider = world.add_collider(
        body,
        Collider::new(Shape::SkinnedTriangleMesh(skinned), Transform::identity()),
    )?;
    world.set_skin_pose(collider, &vec![Affine3::IDENTITY; bone_count])?;
    Ok((world, collider))
}

fn grid_mesh(width: usize, height: usize) -> Result<TriangleMesh, Box<dyn Error>> {
    let mut vertices = Vec::with_capacity(width * height);
    for z in 0..height {
        for x in 0..width {
            vertices.push(Vec3::new(x as f64, 0.0, z as f64));
        }
    }
    let mut triangles = Vec::with_capacity((width - 1) * (height - 1) * 2);
    for z in 0..height - 1 {
        for x in 0..width - 1 {
            let a = z * width + x;
            let b = a + 1;
            let c = a + width;
            let d = c + 1;
            triangles.push([a, b, d]);
            triangles.push([a, d, c]);
        }
    }
    Ok(TriangleMesh::new(vertices, triangles)?)
}

fn add_dynamic(
    world: &mut PhysicsWorld,
    position: Vec3,
    shape: Shape,
    mass: MassProperties,
) -> Result<(), Box<dyn Error>> {
    let body = world.add_body(RigidBody::new_dynamic(
        Transform::new(position, Quat::IDENTITY)?,
        mass,
    )?)?;
    world.add_collider(body, Collider::new(shape, Transform::identity()))?;
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

fn mass() -> Result<MassProperties, Box<dyn Error>> {
    Ok(MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?)
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1_000.0
}
