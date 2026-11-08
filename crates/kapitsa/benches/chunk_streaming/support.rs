use kapitsa_core::{
    chunk::{Chunk, ChunkBuilder},
    collider::Collider,
    collision::{QueryFilter, Ray},
    mass::MassProperties,
    rigid_body::{CcdMode, RigidBody},
    shape::{Shape, Sphere, TriangleMesh},
    transform::Transform,
    world::{BodyId, ChunkId, PhysicsWorld},
};
use kapitsa_math::{Quat, Vec3};
use rayon::prelude::*;
use std::{
    collections::{BTreeMap, HashMap, HashSet},
    error::Error,
    fmt::Write,
    path::PathBuf,
    time::{Duration, Instant},
};

pub type BenchResult<T> = Result<T, Box<dyn Error + Send + Sync>>;
pub type Key = (i64, i64);
pub const WIDTH: f64 = 32.0;

#[derive(Clone, Debug)]
pub struct Config {
    pub prepare_workers: Vec<usize>,
    pub physics_workers: Vec<usize>,
    pub active_chunks: Vec<usize>,
    pub batch_sizes: Vec<usize>,
    pub scenarios: Vec<String>,
    pub modes: Vec<String>,
    pub caches: Vec<String>,
    pub seconds: f64,
    pub dt: f64,
    pub repeats: usize,
    pub grid: usize,
    pub meshes: usize,
    pub in_flight: usize,
    pub flight_batch: usize,
    pub bulk_iterations: usize,
    pub output: PathBuf,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            prepare_workers: vec![1, 2, 4, 8],
            physics_workers: vec![1, 2, 4, 8],
            active_chunks: vec![4, 64, 1024],
            batch_sizes: vec![1, 4, 16, 64, 256, 1024],
            scenarios: [
                "overflight",
                "shuttle",
                "boundary",
                "landing",
                "ccd",
                "bulk",
            ]
            .map(String::from)
            .to_vec(),
            modes: vec!["throughput".into()],
            caches: vec!["cold".into(), "warm".into()],
            seconds: 60.0,
            dt: 0.02,
            repeats: 3,
            grid: 8,
            meshes: 1,
            in_flight: 64,
            flight_batch: 16,
            bulk_iterations: 20,
            output: PathBuf::from("target/chunk-streaming-report.md"),
        }
    }
}

impl Config {
    pub fn parse(args: impl IntoIterator<Item = String>) -> BenchResult<Self> {
        let mut config = Self::default();
        let mut args = args.into_iter();
        while let Some(arg) = args.next() {
            // Cargo forwards its benchmark marker to harness=false binaries.
            if arg == "--bench" {
                continue;
            }
            let value = args
                .next()
                .ok_or_else(|| format!("missing value for {arg}"))?;
            match arg.as_str() {
                "--prepare-workers" => config.prepare_workers = numbers(&value)?,
                "--physics-workers" => config.physics_workers = numbers(&value)?,
                "--active-chunks" => config.active_chunks = numbers(&value)?,
                "--batch-sizes" => config.batch_sizes = numbers(&value)?,
                "--scenarios" => config.scenarios = strings(&value),
                "--mode" => config.modes = strings(&value),
                "--cache" => config.caches = strings(&value),
                "--seconds" => config.seconds = value.parse()?,
                "--dt" => config.dt = value.parse()?,
                "--repeats" => config.repeats = value.parse()?,
                "--grid" => config.grid = value.parse()?,
                "--meshes-per-chunk" => config.meshes = value.parse()?,
                "--in-flight" => config.in_flight = value.parse()?,
                "--flight-batch" => config.flight_batch = value.parse()?,
                "--bulk-iterations" => config.bulk_iterations = value.parse()?,
                "--output" => config.output = value.into(),
                _ => return Err(format!("unknown option: {arg}").into()),
            }
        }
        config.validate()?;
        Ok(config)
    }

    fn validate(&self) -> BenchResult<()> {
        if !self.seconds.is_finite()
            || !self.dt.is_finite()
            || self.seconds <= 0.0
            || self.dt <= 0.0
            || self.seconds > 3600.0
            || self.seconds < self.dt
            || self.seconds / self.dt > 1_000_000.0
        {
            return Err("require 0 < dt <= seconds <= 3600 and at most 1,000,000 steps".into());
        }
        for counts in [
            &self.prepare_workers,
            &self.physics_workers,
            &self.active_chunks,
            &self.batch_sizes,
        ] {
            if counts.is_empty() || counts.iter().any(|&n| n == 0 || n > 1_000_000) {
                return Err("counts must be in 1..=1,000,000".into());
            }
        }
        if self
            .prepare_workers
            .iter()
            .chain(&self.physics_workers)
            .any(|&n| n > 256)
        {
            return Err("worker count exceeds 256".into());
        }
        if !(1..=128).contains(&self.grid)
            || !(1..=1024).contains(&self.meshes)
            || self.repeats == 0
            || self.in_flight == 0
            || self.flight_batch == 0
            || self.bulk_iterations == 0
        {
            return Err("invalid geometry, repetition or queue limits".into());
        }
        for (actual, allowed) in [
            (
                &self.scenarios,
                &[
                    "overflight",
                    "shuttle",
                    "boundary",
                    "landing",
                    "ccd",
                    "bulk",
                ][..],
            ),
            (&self.modes, &["throughput", "realtime"][..]),
            (&self.caches, &["cold", "warm"][..]),
        ] {
            if actual.is_empty() || actual.iter().any(|s| !allowed.contains(&s.as_str())) {
                return Err("unknown scenario, mode or cache policy".into());
            }
        }
        Ok(())
    }

    pub fn frames(&self) -> usize {
        (self.seconds / self.dt).floor() as usize
    }
}

fn strings(value: &str) -> Vec<String> {
    value.split(',').map(String::from).collect()
}
fn numbers(value: &str) -> BenchResult<Vec<usize>> {
    value
        .split(',')
        .map(|v| v.parse().map_err(Into::into))
        .collect()
}

pub fn transform(position: Vec3) -> BenchResult<Transform> {
    Ok(Transform::new(position, Quat::IDENTITY)?)
}

/// A deterministic workload over a flat local tangent surface, without planet/LOD logic.
pub fn position(scenario: &str, frame: usize, dt: f64) -> Vec3 {
    let time = frame as f64 * dt;
    match scenario {
        "shuttle" => Vec3::new(256.0 * (1.0 - (time % 8.0 - 4.0).abs() / 4.0), 20.0, 0.0),
        "boundary" => Vec3::new(WIDTH + 0.1 * (time * 20.0).sin(), 20.0, time * 100.0),
        "landing" => Vec3::new(time * 3.0, 2.0, 0.0),
        "ccd" => Vec3::new(time * 500.0, 3.0, 0.0),
        _ => Vec3::new(time * 200.0, 20.0, 0.0),
    }
}

pub fn working_set(position: Vec3, count: usize) -> Vec<Key> {
    let center = (
        (position.x / WIDTH).floor() as i64,
        (position.z / WIDTH).floor() as i64,
    );
    let mut keys = Vec::with_capacity(count);
    let mut radius = 0i64;
    while keys.len() < count {
        for z in -radius..=radius {
            for x in -radius..=radius {
                if x.abs().max(z.abs()) == radius {
                    keys.push((center.0 + x, center.1 + z));
                    if keys.len() == count {
                        return keys;
                    }
                }
            }
        }
        radius += 1;
    }
    keys
}

pub struct Prepared {
    pub key: Key,
    pub chunk: Chunk,
    pub serials: Vec<u64>,
    pub bytes: usize,
    pub geometry: Duration,
    pub build: Duration,
    pub reused: bool,
}

pub fn serials(chunk: &Chunk) -> BenchResult<Vec<u64>> {
    chunk
        .colliders()
        .iter()
        .map(|c| {
            c.mesh_bvh_serial()
                .ok_or_else(|| "unprepared triangle mesh".into())
        })
        .collect()
}

pub fn prepare(key: Key, grid: usize, meshes: usize) -> BenchResult<Prepared> {
    let started = Instant::now();
    let mut builder = ChunkBuilder::new();
    for mesh in 0..meshes {
        // Meshes tile one chunk without overlapping collision surfaces.
        let x0 = WIDTH * mesh as f64 / meshes as f64;
        let dx = WIDTH / meshes as f64 / grid as f64;
        let dz = WIDTH / grid as f64;
        let mut vertices = Vec::with_capacity((grid + 1) * (grid + 1));
        for z in 0..=grid {
            for x in 0..=grid {
                vertices.push(Vec3::new(x0 + x as f64 * dx, 0.0, z as f64 * dz));
            }
        }
        let mut triangles = Vec::with_capacity(2 * grid * grid);
        for z in 0..grid {
            for x in 0..grid {
                let a = z * (grid + 1) + x;
                let b = a + grid + 1;
                triangles.extend([[a, b, a + 1], [a + 1, b, b + 1]]);
            }
        }
        builder.add_mesh(TriangleMesh::new(vertices, triangles)?)?;
    }
    let geometry = started.elapsed();
    let started = Instant::now();
    let chunk = builder.build(transform(Vec3::new(
        key.0 as f64 * WIDTH,
        0.0,
        key.1 as f64 * WIDTH,
    ))?)?;
    let build = started.elapsed();
    let bytes = chunk
        .colliders()
        .iter()
        .map(Collider::mesh_retained_capacity_bytes)
        .sum();
    Ok(Prepared {
        key,
        serials: serials(&chunk)?,
        chunk,
        bytes,
        geometry,
        build,
        reused: false,
    })
}

#[derive(Default)]
pub struct Stats {
    pub samples: BTreeMap<&'static str, Vec<Duration>>,
    pub builds: u64,
    pub reused: usize,
    pub reuse_errors: usize,
    pub misses: usize,
    pub contact_errors: usize,
    pub stale_ids: usize,
    pub stale_results: usize,
    pub deadline_misses: usize,
    pub active_peak: usize,
    pub collider_peak: usize,
    pub bytes_current: usize,
    pub bytes_peak: usize,
    pub penetration_max: f64,
    pub error: Option<String>,
}

impl Stats {
    pub fn failed(&self) -> bool {
        self.error.is_some()
            || self.reuse_errors > 0
            || self.contact_errors > 0
            || self.stale_ids > 0
    }

    pub fn status(&self) -> &str {
        self.error.as_deref().unwrap_or(if self.failed() {
            "completed with correctness errors"
        } else if self.misses > 0 {
            "completed with coverage misses"
        } else {
            "completed"
        })
    }

    pub fn record(&mut self, phase: &'static str, elapsed: Duration) {
        self.samples.entry(phase).or_default().push(elapsed);
    }
    fn preparation(&mut self, prepared: &Prepared) {
        self.record("geometry", prepared.geometry);
        self.record("build (BVH preparation)", prepared.build);
    }
    fn memory(&mut self, bytes: usize) {
        self.bytes_current = bytes;
        self.bytes_peak = self.bytes_peak.max(bytes);
    }
}

pub fn quantile(samples: &[Duration], percentile: usize) -> Option<Duration> {
    if samples.is_empty() {
        return None;
    }
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    sorted
        .get((sorted.len() * percentile).div_ceil(100).saturating_sub(1))
        .copied()
}

pub fn report_run(out: &mut String, label: &str, stats: &Stats) -> std::fmt::Result {
    writeln!(out, "\n## {label}\n\nStatus: {}\n", stats.status())?;
    writeln!(
        out,
        "| Phase | Samples | p50 ms | p95 ms | p99 ms |\n|---|---:|---:|---:|---:|"
    )?;
    for (phase, values) in &stats.samples {
        let ms = |p| quantile(values, p).map_or(0.0, |d| d.as_secs_f64() * 1000.0);
        writeln!(
            out,
            "| {phase} | {} | {:.6} | {:.6} | {:.6} |",
            values.len(),
            ms(50),
            ms(95),
            ms(99)
        )?;
    }
    writeln!(
        out,
        "\nBVH builds: {}; verified reused BVHs: {}; reuse mismatches: {}; coverage misses: {}; contact errors: {}; stale IDs: {}; stale job results: {}; missed frame deadlines: {}.\n\nPeak active chunks: {}; peak colliders: {}; tracked mesh/BVH bytes current/peak: {}/{}; maximum flat-surface penetration: {:.9} m.\n",
        stats.builds,
        stats.reused,
        stats.reuse_errors,
        stats.misses,
        stats.contact_errors,
        stats.stale_ids,
        stats.stale_results,
        stats.deadline_misses,
        stats.active_peak,
        stats.collider_peak,
        stats.bytes_current,
        stats.bytes_peak,
        stats.penetration_max
    )
}

struct Registered {
    id: ChunkId,
    serials: Vec<u64>,
    bytes: usize,
}

fn register(
    world: &mut PhysicsWorld,
    body: BodyId,
    prepared: Vec<Prepared>,
    active: &mut HashMap<Key, Registered>,
    stats: &mut Stats,
) -> BenchResult<()> {
    let mut chunks = Vec::with_capacity(prepared.len());
    let mut metadata = Vec::with_capacity(prepared.len());
    for data in prepared {
        metadata.push((data.key, data.serials, data.bytes, data.reused));
        chunks.push(data.chunk);
    }
    let started = Instant::now();
    let ids = world.add_chunk_batch(body, chunks)?;
    stats.record("add batch", started.elapsed());
    for (id, (key, expected, bytes, reused)) in ids.into_iter().zip(metadata) {
        let actual: Vec<_> = world
            .chunk_colliders(id)
            .ok_or("missing registered chunk")?
            .iter()
            .map(|&id| world.collider(id).and_then(Collider::mesh_bvh_serial))
            .collect();
        let matches = actual
            .iter()
            .copied()
            .eq(expected.iter().copied().map(Some));
        if !matches {
            stats.reuse_errors += 1;
        }
        if matches && reused {
            stats.reused += expected.len();
        }
        active.insert(
            key,
            Registered {
                id,
                serials: expected,
                bytes,
            },
        );
    }
    stats.active_peak = stats.active_peak.max(world.chunk_count());
    stats.collider_peak = stats.collider_peak.max(world.collider_count());
    Ok(())
}

fn retire(
    world: &mut PhysicsWorld,
    keys: &[Key],
    active: &mut HashMap<Key, Registered>,
    cache: &mut HashMap<Key, Prepared>,
    warm: bool,
    stats: &mut Stats,
) -> BenchResult<()> {
    let ids: Vec<_> = keys
        .iter()
        .map(|key| {
            active
                .get(key)
                .map(|entry| entry.id)
                .ok_or("missing active chunk")
        })
        .collect::<Result<_, _>>()?;
    let started = Instant::now();
    let removed = world.remove_chunk_batch(ids.clone())?;
    stats.record("remove batch", started.elapsed());
    for ((key, id), chunk) in keys.iter().zip(ids).zip(removed) {
        if world.chunk_body(id).is_some() {
            stats.stale_ids += 1;
        }
        let entry = active.remove(key).ok_or("missing retired chunk")?;
        if serials(&chunk)? != entry.serials {
            stats.reuse_errors += 1;
        }
        if warm {
            cache.insert(
                *key,
                Prepared {
                    key: *key,
                    chunk,
                    serials: entry.serials,
                    bytes: entry.bytes,
                    geometry: Duration::ZERO,
                    build: Duration::ZERO,
                    reused: true,
                },
            );
        }
    }
    Ok(())
}

/// Runs synthetic streaming. Outstanding preparation jobs are drained even on errors.
pub struct FlightRun<'a> {
    pub scenario: &'a str,
    pub count: usize,
    pub warm: bool,
    pub realtime: bool,
    pub physics_workers: usize,
}

pub fn flight(
    config: &Config,
    run: FlightRun<'_>,
    pool: &rayon::ThreadPool,
    executor: &impl kapitsa_core::execution::StepExecutor,
) -> Stats {
    let FlightRun {
        scenario,
        count,
        warm,
        realtime,
        physics_workers,
    } = run;
    let mut stats = Stats::default();
    let overall_started = Instant::now();
    let builds_before = Collider::mesh_bvh_build_count();
    let (tx, rx) = std::sync::mpsc::channel::<(Key, Instant, BenchResult<Prepared>)>();
    let mut pending = HashSet::new();
    let run = (|| -> BenchResult<()> {
        let mut world = PhysicsWorld::new();
        let terrain = world.add_body(RigidBody::new_static(Transform::identity()))?;
        let physical = scenario == "landing" || scenario == "ccd";
        let shape = Shape::Sphere(Sphere::new(0.5)?);
        let mass = MassProperties::from_shape(&shape, 1000.0)?;
        let mut rocket = if physical {
            RigidBody::new_dynamic(transform(position(scenario, 0, config.dt))?, mass)?
        } else {
            RigidBody::new_kinematic(transform(position(scenario, 0, config.dt))?)
        };
        if physical {
            rocket.set_linear_velocity(Vec3::new(
                if scenario == "ccd" { 500.0 } else { 3.0 },
                if scenario == "ccd" { -200.0 } else { -1.0 },
                0.0,
            ))?;
        }
        if physical {
            rocket.set_ccd_mode(CcdMode::Continuous);
        }
        let rocket = world.add_body(rocket)?;
        world.add_collider(rocket, Collider::new(shape, Transform::identity()))?;
        let mut cache = HashMap::new();
        let mut active = HashMap::new();
        let initial = working_set(position(scenario, 0, config.dt), count);
        let mut prewarm: HashSet<Key> = initial.iter().copied().collect();
        if warm {
            for frame in 0..config.frames() {
                prewarm.extend(working_set(position(scenario, frame, config.dt), count));
            }
        }
        let mut keys: Vec<_> = prewarm.into_iter().collect();
        keys.sort_unstable();
        let started = Instant::now();
        let ready: Vec<_> = pool.install(|| {
            keys.par_iter()
                .map(|&key| prepare(key, config.grid, config.meshes))
                .collect::<BenchResult<_>>()
        })?;
        stats.record("startup/prewarm wall", started.elapsed());
        for mut data in ready {
            stats.preparation(&data);
            data.reused = warm;
            cache.insert(data.key, data);
        }
        let ready = initial
            .iter()
            .map(|key| cache.remove(key).ok_or("initial geometry missing"))
            .collect::<Result<Vec<_>, _>>()?;
        let mut ready = ready.into_iter();
        loop {
            let batch: Vec<_> = ready.by_ref().take(config.flight_batch).collect();
            if batch.is_empty() {
                break;
            }
            register(&mut world, terrain, batch, &mut active, &mut stats)?;
        }
        stats.memory(
            active.values().map(|d| d.bytes).sum::<usize>()
                + cache.values().map(|d| d.bytes).sum::<usize>(),
        );
        let epoch = Instant::now();
        for frame in 0..config.frames() {
            let frame_started = Instant::now();
            let current = if physical {
                world
                    .body(rocket)
                    .ok_or("missing rocket")?
                    .state()
                    .transform()
                    .translation()
            } else {
                position(scenario, frame, config.dt)
            };
            if !physical {
                world
                    .body_mut(rocket)
                    .ok_or("missing rocket")?
                    .set_transform(transform(current)?)?;
            }
            let desired = working_set(current, count);
            let desired_set: HashSet<_> = desired.iter().copied().collect();
            let future = if physical {
                current
                    + world
                        .body(rocket)
                        .ok_or("missing rocket")?
                        .state()
                        .linear_velocity()
                        * config.dt
            } else {
                position(scenario, frame + 1, config.dt)
            };
            let future = working_set(future, count);
            let future_set: HashSet<_> = future.iter().copied().collect();
            if !warm {
                cache.retain(|key, _| desired_set.contains(key) || future_set.contains(key));
            }
            let mut obsolete: Vec<_> = active
                .keys()
                .filter(|key| !desired_set.contains(key))
                .copied()
                .collect();
            obsolete.sort_unstable();
            for keys in obsolete.chunks(config.flight_batch) {
                retire(&mut world, keys, &mut active, &mut cache, warm, &mut stats)?;
            }
            while let Ok((key, queued, data)) = rx.try_recv() {
                pending.remove(&key);
                let data: Prepared = data?;
                stats.record("job completion latency", queued.elapsed());
                stats.preparation(&data);
                if desired_set.contains(&key) || future_set.contains(&key) || warm {
                    cache.insert(key, data);
                } else {
                    stats.stale_results += 1;
                }
            }
            for &key in desired.iter().chain(&future) {
                if !active.contains_key(&key)
                    && !cache.contains_key(&key)
                    && !pending.contains(&key)
                    && pending.len() < config.in_flight
                {
                    pending.insert(key);
                    let tx = tx.clone();
                    let grid = config.grid;
                    let meshes = config.meshes;
                    let queued = Instant::now();
                    pool.spawn(move || {
                        let data = prepare(key, grid, meshes);
                        // A closed receiver explicitly cancels result publication.
                        let _ = tx.send((key, queued, data));
                    });
                }
            }
            let mut ready = Vec::new();
            for key in &desired {
                if !active.contains_key(key)
                    && let Some(data) = cache.remove(key)
                {
                    ready.push(data);
                }
                if ready.len() == config.flight_batch {
                    register(
                        &mut world,
                        terrain,
                        std::mem::take(&mut ready),
                        &mut active,
                        &mut stats,
                    )?;
                }
            }
            if !ready.is_empty() {
                register(&mut world, terrain, ready, &mut active, &mut stats)?;
            }
            let filter = QueryFilter {
                exclude_body: Some(rocket),
                ..QueryFilter::default()
            };
            let ground = world.raycast(
                Ray::new(Vec3::new(current.x, 100.0, current.z), -Vec3::Y, 200.0)?,
                filter,
            )?;
            if ground.is_none() {
                stats.misses += 1;
            }
            if physical {
                world
                    .body_mut(rocket)
                    .ok_or("missing rocket")?
                    .apply_force(-Vec3::Y * (mass.mass() * 9.81))?;
            }
            let started = Instant::now();
            world.step_with_executor(config.dt, executor, physics_workers)?;
            stats.record("physics step", started.elapsed());
            if physical {
                let y = world
                    .body(rocket)
                    .ok_or("missing rocket")?
                    .state()
                    .transform()
                    .translation()
                    .y;
                let penetration = (0.5 - y).max(0.0);
                stats.penetration_max = stats.penetration_max.max(penetration);
                if ground.is_some() && penetration > 0.01 {
                    stats.contact_errors += 1;
                }
            }
            stats.memory(
                active.values().map(|d| d.bytes).sum::<usize>()
                    + cache.values().map(|d| d.bytes).sum::<usize>(),
            );
            stats.record("frame work (includes checks)", frame_started.elapsed());
            if realtime {
                let deadline = epoch + Duration::from_secs_f64((frame + 1) as f64 * config.dt);
                if let Some(delay) = deadline.checked_duration_since(Instant::now()) {
                    std::thread::sleep(delay);
                } else {
                    stats.deadline_misses += 1;
                }
            }
        }
        stats.record("frame loop wall (includes pacing)", epoch.elapsed());
        Ok(())
    })();
    if let Err(error) = run {
        stats.error = Some(error.to_string());
    }
    let drain_started = Instant::now();
    while !pending.is_empty() {
        match rx.recv() {
            Ok((key, _, result)) => {
                pending.remove(&key);
                match result {
                    Ok(data) => stats.preparation(&data),
                    Err(error) => {
                        stats
                            .error
                            .get_or_insert_with(|| format!("preparation drain: {error}"));
                    }
                }
            }
            Err(error) => {
                stats
                    .error
                    .get_or_insert_with(|| format!("preparation channel: {error}"));
                break;
            }
        }
    }
    stats.record("tail drain wall", drain_started.elapsed());
    stats.record("run wall including setup/drain", overall_started.elapsed());
    stats.builds = Collider::mesh_bvh_build_count() - builds_before;
    stats
}

/// Equal total geometry for all three registration APIs. No physical step is timed here.
pub fn bulk(
    config: &Config,
    count: usize,
    batch_size: usize,
    api: &str,
    warm: bool,
    pool: &rayon::ThreadPool,
) -> Stats {
    let mut stats = Stats::default();
    let builds_before = Collider::mesh_bvh_build_count();
    let run = (|| -> BenchResult<()> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
        let keys = working_set(Vec3::ZERO, count);
        let mut chunks = Vec::new();
        let mut raw = Vec::new();
        let mut expected = Vec::new();
        for iteration in 0..config.bulk_iterations {
            if !warm || iteration == 0 {
                let prepared: Vec<_> = pool.install(|| {
                    keys.par_iter()
                        .map(|&key| prepare(key, config.grid, config.meshes))
                        .collect::<BenchResult<_>>()
                })?;
                let mut bytes = 0;
                expected.clear();
                for data in prepared {
                    stats.preparation(&data);
                    bytes += data.bytes;
                    expected.extend(data.serials);
                    if api == "collider" {
                        raw.extend(data.chunk.into_colliders());
                    } else {
                        chunks.push(data.chunk);
                    }
                }
                stats.memory(bytes);
            }
            let mut collider_ids = Vec::new();
            let mut chunk_ids = Vec::new();
            let mut add_total = Duration::ZERO;
            if api == "collider" {
                collider_ids.reserve(raw.len());
                for collider in raw.drain(..) {
                    let start = Instant::now();
                    let id = world.add_collider(body, collider)?;
                    let elapsed = start.elapsed();
                    stats.record("add operation", elapsed);
                    add_total += elapsed;
                    collider_ids.push(id);
                }
            } else if api == "chunk" {
                chunk_ids.reserve(chunks.len());
                for chunk in chunks.drain(..) {
                    let start = Instant::now();
                    let id = world.add_chunk(body, chunk)?;
                    let elapsed = start.elapsed();
                    stats.record("add operation", elapsed);
                    add_total += elapsed;
                    chunk_ids.push(id);
                }
            } else {
                let mut input = std::mem::take(&mut chunks).into_iter();
                loop {
                    let batch: Vec<_> = input.by_ref().take(batch_size).collect();
                    if batch.is_empty() {
                        break;
                    }
                    let start = Instant::now();
                    let ids = world.add_chunk_batch(body, batch)?;
                    let elapsed = start.elapsed();
                    stats.record("add operation", elapsed);
                    add_total += elapsed;
                    chunk_ids.extend(ids);
                }
            }
            stats.record("add equal-work total", add_total);
            let actual: Vec<_> = if api == "collider" {
                collider_ids
                    .iter()
                    .map(|&id| world.collider(id).and_then(Collider::mesh_bvh_serial))
                    .collect()
            } else {
                let mut values = Vec::new();
                for &id in &chunk_ids {
                    for &id in world.chunk_colliders(id).ok_or("missing chunk")? {
                        values.push(world.collider(id).and_then(Collider::mesh_bvh_serial));
                    }
                }
                values
            };
            if !actual
                .iter()
                .copied()
                .eq(expected.iter().copied().map(Some))
            {
                stats.reuse_errors += 1;
            } else if warm && iteration > 0 {
                stats.reused += expected.len();
            }
            stats.active_peak = stats.active_peak.max(world.chunk_count());
            stats.collider_peak = stats.collider_peak.max(world.collider_count());
            let ray = Ray::new(Vec3::new(1.0, 10.0, 1.0), -Vec3::Y, 20.0)?;
            if world.raycast(ray, QueryFilter::default())?.is_none() {
                stats.misses += 1;
            }
            let mut remove_total = Duration::ZERO;
            if api == "collider" {
                for id in collider_ids {
                    let start = Instant::now();
                    let collider = world.remove_collider(id).ok_or("collider removal failed")?;
                    let elapsed = start.elapsed();
                    stats.record("remove operation", elapsed);
                    remove_total += elapsed;
                    if world.collider(id).is_some() {
                        stats.stale_ids += 1;
                    }
                    if warm {
                        raw.push(collider);
                    }
                }
            } else if api == "chunk" {
                for id in chunk_ids {
                    let start = Instant::now();
                    let chunk = world.remove_chunk(id).ok_or("chunk removal failed")?;
                    let elapsed = start.elapsed();
                    stats.record("remove operation", elapsed);
                    remove_total += elapsed;
                    if world.chunk_body(id).is_some() {
                        stats.stale_ids += 1;
                    }
                    if warm {
                        chunks.push(chunk);
                    }
                }
            } else {
                for batch in chunk_ids.chunks(batch_size) {
                    let input = batch.to_vec();
                    let start = Instant::now();
                    let removed = world.remove_chunk_batch(input)?;
                    let elapsed = start.elapsed();
                    stats.record("remove operation", elapsed);
                    remove_total += elapsed;
                    if batch.iter().any(|&id| world.chunk_body(id).is_some()) {
                        stats.stale_ids += 1;
                    }
                    if warm {
                        chunks.extend(removed);
                    }
                }
            }
            stats.record("remove equal-work total", remove_total);
            if world.raycast(ray, QueryFilter::default())?.is_some() {
                stats.stale_ids += 1;
            }
            if !warm {
                stats.memory(0);
            }
        }
        Ok(())
    })();
    stats.error = run.err().map(|error| error.to_string());
    stats.builds = Collider::mesh_bvh_build_count() - builds_before;
    stats
}

#[cfg(test)]
mod tests {
    #[test]
    fn publication_and_cache_retain_verified_bvh_ids() -> super::BenchResult<()> {
        use super::*;
        let mut world = PhysicsWorld::new();
        let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
        let mut active = HashMap::new();
        let mut cache = HashMap::new();
        let mut stats = Stats::default();
        register(
            &mut world,
            body,
            vec![prepare((0, 0), 1, 2)?, prepare((1, 0), 1, 2)?],
            &mut active,
            &mut stats,
        )?;
        retire(
            &mut world,
            &[(0, 0), (1, 0)],
            &mut active,
            &mut cache,
            true,
            &mut stats,
        )?;
        assert!(active.is_empty());
        assert_eq!(cache.len(), 2);
        let input = [(0, 0), (1, 0)]
            .iter()
            .map(|key| cache.remove(key).ok_or("missing cached data"))
            .collect::<Result<Vec<_>, _>>()?;
        register(&mut world, body, input, &mut active, &mut stats)?;
        assert_eq!(stats.reused, 4);
        assert_eq!(stats.reuse_errors, 0);
        assert_eq!(stats.stale_ids, 0);
        assert_eq!(world.collider_count(), 4);
        Ok(())
    }
}
