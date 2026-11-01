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
    shape::{BoneInfluence, Cuboid, Plane, Shape, SkinnedTriangleMesh, Sphere, TriangleMesh},
    transform::Transform,
    world::{BodyStep, ColliderId, PhysicsWorld, StepError},
};
use kapitsa_math::{Affine3, Mat3, Quat, Vec3};

#[path = "support/scoped_executor.rs"]
mod scoped_executor;
use scoped_executor::ScopedThreadExecutor;

const WARMUPS: usize = 1;
const STEADY_STATE_WARMUPS: usize = 3;
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
                | "sparse_20k"
                | "queries_1000"
                | "sweeps_1000"
                | "skin_1k"
                | "skin_10k"
                | "skin_100k"
                | "workers_4096"
                | "layouts"
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
        (
            "sparse_20k",
            "20,000 Dynamic / 128 contact pairs",
            sparse_dynamic_contacts_20k,
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
    if selected
        .as_deref()
        .is_none_or(|filter| filter == "workers_4096")
    {
        report_solver_workers()?;
    }
    if selected.as_deref().is_none_or(|filter| filter == "layouts") {
        report_layout_comparison()?;
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

fn report_solver_workers() -> Result<(), Box<dyn Error>> {
    for worker_count in [1, 2, 4] {
        for _ in 0..WARMUPS {
            let mut world = parallel_contact_pairs(4_096)?;
            run_worker_step(&mut world, worker_count)?;
        }

        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let mut world = parallel_contact_pairs(4_096)?;
            let mut shards = vec![Vec::new(); worker_count];
            for _ in 0..STEADY_STATE_WARMUPS {
                if worker_count == 1 {
                    black_box(world.step(DT)?);
                } else {
                    world.fill_step_shards(&mut shards)?;
                    run_filled_shard_step(&mut world, &mut shards, worker_count)?;
                }
            }
            if worker_count > 1 {
                world.fill_step_shards(&mut shards)?;
            }
            let started = Instant::now();
            if worker_count == 1 {
                black_box(world.step(DT)?);
            } else {
                run_filled_shard_step(&mut world, &mut shards, worker_count)?;
            }
            samples.push((started.elapsed(), world.collision_step_metrics()));
        }
        samples.sort_unstable_by_key(|sample| sample.0);
        let (min, _) = samples.first().ok_or("missing worker sample")?;
        let (median, metrics) = samples.get(SAMPLES / 2).ok_or("missing worker median")?;
        let (max, _) = samples.last().ok_or("missing worker sample")?;
        println!(
            "4,096 dynamic / 2,048 contact islands, workers={worker_count} step={:.3}/{:.3}/{:.3} ms min/median/max | islands={} solver={:.3} ms prep={:.3} solve={:.3} writeback={:.3} ms body_copies={} manifold_copies={} growth={} B retained={} B",
            millis(*min),
            millis(*median),
            millis(*max),
            metrics.island_count,
            millis(metrics.solver),
            millis(metrics.solver_prepare),
            millis(metrics.solver_iterations),
            millis(metrics.solver_writeback),
            metrics.solver_body_copies,
            metrics.solver_manifold_copies,
            metrics.logical_capacity_growth_bytes,
            metrics.retained_scratch_bytes,
        );
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct LayoutContactInput {
    island: usize,
    body_a: usize,
    body_b: usize,
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
struct LayoutContact {
    body_a: usize,
    body_b: usize,
}

#[derive(Default)]
struct CopiedIslandLayout {
    body_indices: Vec<usize>,
    manifold_indices: Vec<usize>,
    manifolds: Vec<LayoutContact>,
}

#[derive(Default)]
struct CopiedSolverLayout {
    islands: Vec<CopiedIslandLayout>,
}

impl CopiedSolverLayout {
    fn prepare(&mut self, inputs: &[LayoutContactInput], island_count: usize) {
        self.islands
            .resize_with(island_count, CopiedIslandLayout::default);
        self.islands.truncate(island_count);
        for island in &mut self.islands {
            island.body_indices.clear();
            island.manifold_indices.clear();
            island.manifolds.clear();
        }
        for (index, input) in inputs.iter().enumerate() {
            let island = &mut self.islands[input.island];
            island.manifold_indices.push(index);
            island.body_indices.extend([input.body_a, input.body_b]);
        }
        for island in &mut self.islands {
            island.body_indices.sort_unstable();
            island.body_indices.dedup();
            for input_index in island.manifold_indices.iter().copied() {
                let input = inputs[input_index];
                island.manifolds.push(LayoutContact {
                    body_a: island
                        .body_indices
                        .binary_search(&input.body_a)
                        .expect("body belongs to this island"),
                    body_b: island
                        .body_indices
                        .binary_search(&input.body_b)
                        .expect("body belongs to this island"),
                });
            }
        }
    }

    fn retained_capacity_bytes(&self) -> usize {
        self.islands.capacity() * std::mem::size_of::<CopiedIslandLayout>()
            + self
                .islands
                .iter()
                .map(|island| {
                    island.body_indices.capacity() * std::mem::size_of::<usize>()
                        + island.manifold_indices.capacity() * std::mem::size_of::<usize>()
                        + island.manifolds.capacity() * std::mem::size_of::<LayoutContact>()
                })
                .sum::<usize>()
    }

    fn logical_copies(&self) -> (usize, usize) {
        self.islands
            .iter()
            .fold((0, 0), |(bodies, manifolds), island| {
                (
                    bodies + island.body_indices.len(),
                    manifolds + island.manifolds.len(),
                )
            })
    }

    fn checksum(&self) -> usize {
        self.islands
            .iter()
            .flat_map(|island| &island.manifolds)
            .fold(0, |sum, contact| {
                sum.wrapping_add(contact.body_a ^ contact.body_b)
            })
    }
}

#[derive(Clone, Copy, Default)]
struct PackedIslandRange {
    manifold_start: usize,
    manifold_end: usize,
    manifold_cursor: usize,
    body_start: usize,
    body_end: usize,
    raw_body_start: usize,
    raw_body_end: usize,
    body_cursor: usize,
}

#[derive(Default)]
struct PackedSolverLayout {
    islands: Vec<PackedIslandRange>,
    body_indices: Vec<usize>,
    manifold_indices: Vec<usize>,
    manifolds: Vec<LayoutContact>,
    local_by_body: Vec<usize>,
}

impl PackedSolverLayout {
    fn prepare(&mut self, inputs: &[LayoutContactInput], body_count: usize, island_count: usize) {
        self.islands
            .resize(island_count, PackedIslandRange::default());
        self.islands.truncate(island_count);
        self.islands.fill(PackedIslandRange::default());
        for input in inputs {
            let range = &mut self.islands[input.island];
            range.manifold_end += 1;
            range.body_end += 2;
        }

        let mut manifold_cursor = 0;
        let mut body_cursor = 0;
        for range in &mut self.islands {
            let manifold_count = range.manifold_end;
            let body_candidate_count = range.body_end;
            range.manifold_start = manifold_cursor;
            range.manifold_end = manifold_cursor + manifold_count;
            range.manifold_cursor = range.manifold_start;
            manifold_cursor = range.manifold_end;
            range.raw_body_start = body_cursor;
            range.raw_body_end = body_cursor + body_candidate_count;
            range.body_cursor = range.raw_body_start;
            body_cursor = range.raw_body_end;
        }

        self.manifold_indices.resize(inputs.len(), 0);
        self.manifolds
            .resize(inputs.len(), LayoutContact::default());
        self.body_indices.resize(body_cursor, 0);
        self.local_by_body.resize(body_count, 0);
        for (index, input) in inputs.iter().enumerate() {
            let range = &mut self.islands[input.island];
            self.manifold_indices[range.manifold_cursor] = index;
            range.manifold_cursor += 1;
            self.body_indices[range.body_cursor] = input.body_a;
            self.body_indices[range.body_cursor + 1] = input.body_b;
            range.body_cursor += 2;
        }

        let mut packed_body_end = 0;
        for range in &mut self.islands {
            self.body_indices[range.raw_body_start..range.raw_body_end].sort_unstable();
            let mut unique_end = range.raw_body_start;
            for read in range.raw_body_start..range.raw_body_end {
                let body = self.body_indices[read];
                if unique_end == range.raw_body_start || self.body_indices[unique_end - 1] != body {
                    self.body_indices[unique_end] = body;
                    unique_end += 1;
                }
            }
            let unique_count = unique_end - range.raw_body_start;
            let packed_body_start = packed_body_end;
            self.body_indices
                .copy_within(range.raw_body_start..unique_end, packed_body_start);
            packed_body_end += unique_count;
            range.body_start = packed_body_start;
            range.body_end = packed_body_end;
            for (local, body) in self.body_indices[range.body_start..range.body_end]
                .iter()
                .copied()
                .enumerate()
            {
                self.local_by_body[body] = local;
            }
            for slot in range.manifold_start..range.manifold_end {
                let input = inputs[self.manifold_indices[slot]];
                self.manifolds[slot] = LayoutContact {
                    body_a: self.local_by_body[input.body_a],
                    body_b: self.local_by_body[input.body_b],
                };
            }
        }
        self.body_indices.truncate(packed_body_end);
    }

    fn retained_capacity_bytes(&self) -> usize {
        self.islands.capacity() * std::mem::size_of::<PackedIslandRange>()
            + self.body_indices.capacity() * std::mem::size_of::<usize>()
            + self.manifold_indices.capacity() * std::mem::size_of::<usize>()
            + self.manifolds.capacity() * std::mem::size_of::<LayoutContact>()
            + self.local_by_body.capacity() * std::mem::size_of::<usize>()
    }

    fn logical_copies(&self) -> (usize, usize) {
        (self.body_indices.len(), self.manifolds.len())
    }

    fn checksum(&self) -> usize {
        self.manifolds.iter().fold(0, |sum, contact| {
            sum.wrapping_add(contact.body_a ^ contact.body_b)
        })
    }

    fn matches(&self, copied: &CopiedSolverLayout) -> bool {
        self.islands.len() == copied.islands.len()
            && self
                .islands
                .iter()
                .zip(&copied.islands)
                .all(|(range, island)| {
                    self.body_indices[range.body_start..range.body_end] == island.body_indices
                        && self.manifold_indices[range.manifold_start..range.manifold_end]
                            == island.manifold_indices
                        && self.manifolds[range.manifold_start..range.manifold_end]
                            == island.manifolds
                })
    }
}

fn report_layout_comparison() -> Result<(), Box<dyn Error>> {
    for (name, body_count, island_count, inputs) in solver_layout_cases() {
        let mut copied = CopiedSolverLayout::default();
        let mut packed = PackedSolverLayout::default();
        copied.prepare(&inputs, island_count);
        packed.prepare(&inputs, body_count, island_count);
        if !packed.matches(&copied) {
            return Err("packed solver layout differs from copied layout".into());
        }
        let iterations = 250;
        let mut copied_samples = Vec::with_capacity(SAMPLES);
        let mut packed_samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let started = Instant::now();
            for _ in 0..iterations {
                copied.prepare(black_box(&inputs), island_count);
                black_box((copied.logical_copies(), copied.checksum()));
            }
            copied_samples.push(started.elapsed());

            let started = Instant::now();
            for _ in 0..iterations {
                packed.prepare(black_box(&inputs), body_count, island_count);
                black_box((packed.logical_copies(), packed.checksum()));
            }
            packed_samples.push(started.elapsed());
        }
        copied_samples.sort_unstable();
        packed_samples.sort_unstable();
        let copied_median = copied_samples[SAMPLES / 2].as_secs_f64() * 1.0e3 / iterations as f64;
        let packed_median = packed_samples[SAMPLES / 2].as_secs_f64() * 1.0e3 / iterations as f64;
        let copied_copies = copied.logical_copies();
        let packed_copies = packed.logical_copies();
        println!(
            "layout prep {name:35} copied={:.4} ms packed={:.4} ms ratio={:.2}x | copies bodies/manifolds={}/{} vs {}/{} | retained={} B vs {} B",
            copied_median,
            packed_median,
            copied_median / packed_median.max(f64::MIN_POSITIVE),
            copied_copies.0,
            copied_copies.1,
            packed_copies.0,
            packed_copies.1,
            copied.retained_capacity_bytes(),
            packed.retained_capacity_bytes(),
        );
    }
    Ok(())
}

fn solver_layout_cases() -> [(&'static str, usize, usize, Vec<LayoutContactInput>); 6] {
    let mut one_island = Vec::with_capacity(999);
    for body in 0..999 {
        one_island.push(LayoutContactInput {
            island: 0,
            body_a: body,
            body_b: body + 1,
        });
    }

    let mut many_dynamic = Vec::with_capacity(500);
    let mut shared_static = Vec::with_capacity(500);
    for island in 0..500 {
        many_dynamic.push(LayoutContactInput {
            island,
            body_a: island * 2,
            body_b: island * 2 + 1,
        });
        shared_static.push(LayoutContactInput {
            island,
            body_a: island + 1,
            body_b: 0,
        });
    }

    let mut four_contacts = Vec::with_capacity(4_000);
    for dynamic in 0..1_000 {
        for contact in 0..4 {
            four_contacts.push(LayoutContactInput {
                island: dynamic,
                body_a: dynamic,
                body_b: 1_000 + dynamic * 4 + contact,
            });
        }
    }

    let mut parallel = Vec::with_capacity(2_048);
    for island in 0..2_048 {
        parallel.push(LayoutContactInput {
            island,
            body_a: island * 2,
            body_b: island * 2 + 1,
        });
    }

    let mut sparse = Vec::with_capacity(128);
    for island in 0..128 {
        sparse.push(LayoutContactInput {
            island,
            body_a: island * 2,
            body_b: island * 2 + 1,
        });
    }

    [
        ("1 island x 1,000 dynamic", 1_000, 1, one_island),
        ("500 islands x 2 dynamic", 1_000, 500, many_dynamic),
        ("500 dynamic / shared static", 501, 500, shared_static),
        (
            "1,000 dynamic / 4,000 contacts",
            5_000,
            1_000,
            four_contacts,
        ),
        ("4,096 dynamic / 2,048 islands", 4_096, 2_048, parallel),
        ("20,000 bodies / 128 contacts", 20_000, 128, sparse),
    ]
}

fn run_worker_step(world: &mut PhysicsWorld, worker_count: usize) -> Result<(), Box<dyn Error>> {
    if worker_count == 1 {
        world.step(DT)?;
        return Ok(());
    }
    let mut shards = vec![Vec::new(); worker_count];
    world.fill_step_shards(&mut shards)?;
    run_filled_shard_step(world, &mut shards, worker_count)
}

fn run_filled_shard_step(
    world: &mut PhysicsWorld,
    shards: &mut [Vec<BodyStep>],
    worker_count: usize,
) -> Result<(), Box<dyn Error>> {
    std::thread::scope(|scope| -> Result<(), Box<dyn Error>> {
        let mut workers = Vec::with_capacity(shards.len());
        for shard in &mut *shards {
            workers.push(scope.spawn(move || {
                for step in shard {
                    step.integrate(DT)?;
                }
                Ok::<(), StepError>(())
            }));
        }
        for worker in workers {
            worker
                .join()
                .map_err(|_| std::io::Error::other("body prediction worker panicked"))??;
        }
        Ok(())
    })?;
    world.commit_step_shards_with_executor(shards, &ScopedThreadExecutor, worker_count)?;
    Ok(())
}

fn parallel_contact_pairs(count: usize) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    for pair in 0..count / 2 {
        let x = pair as f64 * 3.0;
        add_dynamic(
            &mut world,
            Vec3::new(x, 0.0, 0.0),
            Shape::Sphere(Sphere::new(0.5)?),
            mass,
        )?;
        add_dynamic(
            &mut world,
            Vec3::new(x + 0.9, 0.0, 0.0),
            Shape::Sphere(Sphere::new(0.5)?),
            mass,
        )?;
    }
    Ok(world)
}

fn report_steps(label: &str, build: BuildWorld) -> Result<(), Box<dyn Error>> {
    for _ in 0..WARMUPS {
        let mut world = build()?;
        black_box(world.step(DT)?);
    }

    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let mut world = build()?;
        for _ in 0..STEADY_STATE_WARMUPS {
            black_box(world.step(DT)?);
        }
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
        "{label:40} step={:.3}/{:.3}/{:.3} ms min/median/max | candidates={} contacts={} islands={} | island={:.3} ms solver={:.3} ms (prepare={:.3}, iterations={:.3}, writeback={:.3}) events={:.3} ms commit={:.3} ms | body_copies={} manifold_copies={} growth={} B retained_scratch={} B query_index={} B peak_sampled={} B",
        millis(min),
        millis(median),
        millis(max),
        metrics.candidate_count,
        metrics.contact_count,
        metrics.island_count,
        millis(metrics.island_build),
        millis(metrics.solver),
        millis(metrics.solver_prepare),
        millis(metrics.solver_iterations),
        millis(metrics.solver_writeback),
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
    let ground = world.add_body(RigidBody::new_static(Transform::identity()))?;
    world.add_collider(
        ground,
        Collider::new(
            Shape::Plane(Plane::new(0.0, 1.0, 0.0, 0.0)?),
            Transform::identity(),
        ),
    )?;
    let mass = mass()?;
    for pair in 0..500 {
        let x = pair as f64 * 3.0;
        add_dynamic(
            &mut world,
            Vec3::new(x, 0.4, 0.0),
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

fn sparse_dynamic_contacts_20k() -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = mass()?;
    for pair in 0..128 {
        let x = (pair % 32) as f64 * 3.0;
        let z = (pair / 32) as f64 * 3.0;
        add_dynamic(
            &mut world,
            Vec3::new(x, 0.0, z),
            Shape::Sphere(Sphere::new(0.5)?),
            mass,
        )?;
        add_dynamic(
            &mut world,
            Vec3::new(x + 0.9, 0.0, z),
            Shape::Sphere(Sphere::new(0.5)?),
            mass,
        )?;
    }
    for index in 256..20_000 {
        let index = index - 256;
        let x = (index % 200) as f64 * 2.0;
        let z = (index / 200) as f64 * 2.0;
        add_dynamic(
            &mut world,
            Vec3::new(x, 10.0, z),
            Shape::Sphere(Sphere::new(0.5)?),
            mass,
        )?;
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
