use std::{
    collections::{HashMap, VecDeque},
    error::Error,
    future::Future,
    hint::black_box,
    mem::size_of,
    pin::Pin,
    sync::{Arc, Mutex},
    task::{Context, Poll, Waker},
    time::{Duration, Instant},
};

use kapitsa_core::{
    mass::MassProperties,
    rigid_body::{BodyState, RigidBody},
    transform::Transform,
    world::{BodyId, BodyStep, PhysicsWorld, StepError},
};
use kapitsa_math::{Mat3, Vec3};
use kapitsa_tokio::{PhysicsClientError, PhysicsController, PhysicsSimulation, SchedulerConfig};
use tokio::{
    runtime::{Builder, Runtime},
    sync::{mpsc, watch},
};

#[allow(dead_code, unused_imports)]
mod marshalling {
    #[path = "../../src/marshalling/client.rs"]
    mod client;
    #[path = "../../src/marshalling/command.rs"]
    mod command;
    #[path = "../../src/marshalling/snapshot.rs"]
    mod snapshot;

    pub(crate) use client::PhysicsClient;
    pub(crate) use command::{PhysicsCommand, PhysicsCommandResult, apply_command};
    pub(crate) use snapshot::PhysicsSnapshot;
}

const BODY_COUNTS: [(&str, usize); 3] = [("small", 256), ("medium", 4_096), ("large", 20_000)];
const WORKLOADS: [Workload; 3] = [
    Workload {
        name: "dynamic",
        static_percent: 0,
        sleeping_percent: 0,
    },
    Workload {
        name: "mixed",
        static_percent: 50,
        sleeping_percent: 25,
    },
    Workload {
        name: "sleeping-heavy",
        static_percent: 10,
        sleeping_percent: 80,
    },
];
const STEPS: usize = 12;
const SAMPLES: usize = 5;

#[derive(Clone, Copy)]
struct Workload {
    name: &'static str,
    static_percent: usize,
    sleeping_percent: usize,
}

fn main() -> Result<(), Box<dyn Error>> {
    let runtime = Builder::new_current_thread().enable_time().build()?;
    println!("kapitsa integration benchmark; steps={STEPS}, samples={SAMPLES}, statistic=median");
    if std::env::var_os("KAPITSA_MARSHALLING_ONLY").is_some() {
        return runtime.block_on(async {
            tokio::time::pause();
            report_marshalling().await
        });
    }

    for (size, body_count) in BODY_COUNTS {
        for workload in WORKLOADS {
            let sequential = sample(SAMPLES, || {
                let mut world = make_world(body_count, workload)?;
                let dt = 1.0 / 60.0;
                let started = Instant::now();
                for _ in 0..STEPS {
                    world.step(dt)?;
                }
                Ok(started.elapsed())
            })?;
            println!("\n{size} world ({}): {body_count} bodies", workload.name);
            report("PhysicsWorld::step", sequential, sequential);

            let fast_path = sample(SAMPLES, || {
                let world = make_world(body_count, workload)?;
                let mut controller = PhysicsController::new(world, config(1))?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    runtime.block_on(controller.step())?;
                }
                Ok(started.elapsed())
            })?;
            report("Tokio fast path", fast_path, sequential);

            let static_shards = sample(SAMPLES, || {
                let mut world = make_world(body_count, workload)?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    world = runtime.block_on(static_sharded_step(world, 1.0 / 60.0, 2))?;
                }
                Ok(started.elapsed())
            })?;
            report("static shards x2", static_shards, sequential);

            let dynamic_buckets = sample(SAMPLES, || {
                let world = make_world(body_count, workload)?;
                let mut controller = PhysicsController::new(world, config(2))?;
                controller.set_active_workers(2)?;
                let started = Instant::now();
                for _ in 0..STEPS {
                    runtime.block_on(controller.step())?;
                }
                Ok(started.elapsed())
            })?;
            report("dynamic buckets x2", dynamic_buckets, sequential);
        }
    }
    for workload in WORKLOADS {
        report_phases(&runtime, workload)?;
    }
    runtime.block_on(async {
        tokio::time::pause();
        report_marshalling().await
    })?;
    Ok(())
}

fn report_phases(runtime: &Runtime, workload: Workload) -> Result<(), Box<dyn Error>> {
    let mut samples = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    for _ in 0..SAMPLES {
        let mut world = make_world(20_000, workload)?;
        let mut buckets = (0..8).map(|_| Vec::new()).collect::<Vec<Vec<BodyStep>>>();
        let mut times = [Duration::ZERO; 4];
        for _ in 0..STEPS {
            let started = Instant::now();
            world.fill_step_shards(&mut buckets)?;
            times[0] += started.elapsed();

            let started = Instant::now();
            let queue = Arc::new(Mutex::new(
                buckets.into_iter().enumerate().collect::<VecDeque<_>>(),
            ));
            let mut tasks = Vec::new();
            for _ in 0..2 {
                let queue = Arc::clone(&queue);
                tasks.push(runtime.spawn_blocking(
                    move || -> Result<Vec<(usize, Vec<BodyStep>)>, StepError> {
                        let mut completed = Vec::new();
                        loop {
                            let next = queue
                                .lock()
                                .unwrap_or_else(std::sync::PoisonError::into_inner)
                                .pop_front();
                            let Some((index, mut bucket)) = next else {
                                break;
                            };
                            for body in &mut bucket {
                                body.integrate(1.0 / 60.0)?;
                            }
                            completed.push((index, bucket));
                        }
                        Ok(completed)
                    },
                ));
            }
            times[1] += started.elapsed();

            let started = Instant::now();
            let completed = runtime.block_on(async {
                let mut completed = Vec::new();
                for task in tasks {
                    completed.extend(task.await??);
                }
                Ok::<_, Box<dyn Error>>(completed)
            })?;
            let mut ordered = (0..8).map(|_| None).collect::<Vec<_>>();
            for (index, bucket) in completed {
                ordered[index] = Some(bucket);
            }
            buckets = ordered
                .into_iter()
                .map(|bucket| bucket.ok_or("missing bucket"))
                .collect::<Result<_, _>>()?;
            times[2] += started.elapsed();

            let started = Instant::now();
            world.commit_step_shards(&buckets)?;
            times[3] += started.elapsed();
        }
        for (sample, time) in samples.iter_mut().zip(times) {
            sample.push(time);
        }
    }
    for sample in &mut samples {
        sample.sort_unstable();
    }
    println!(
        "\n{} raw bucket phases, 20,000 bodies (ms / {STEPS} steps): prepare={:.2}, dispatch={:.2}, join+collect={:.2}, commit={:.2}",
        workload.name,
        samples[0][SAMPLES / 2].as_secs_f64() * 1000.0,
        samples[1][SAMPLES / 2].as_secs_f64() * 1000.0,
        samples[2][SAMPLES / 2].as_secs_f64() * 1000.0,
        samples[3][SAMPLES / 2].as_secs_f64() * 1000.0
    );
    Ok(())
}

#[derive(Clone, Copy, Default)]
struct PhaseTimes {
    total: Duration,
    commands: Duration,
    physics: Duration,
    snapshot_build: Duration,
    publication: Duration,
}

#[derive(Clone, Copy, Default)]
struct SimulationTimes {
    wall: Duration,
    physics: Duration,
    steps: u64,
}

#[derive(Clone, Copy, Default)]
struct SnapshotSample {
    total: Duration,
    physics: Duration,
    build: Duration,
    lookup: Duration,
    iteration: Duration,
    publication: Duration,
    entries_capacity: usize,
    index_capacity: usize,
    storage_bytes: usize,
    reused_buffers: usize,
    reclaimed_buffers: usize,
    index_switches: usize,
}

trait SnapshotBench: Sized {
    fn from_world(world: &PhysicsWorld, step: u64, time: Duration) -> Self;
    fn take_reusable(reusable: &mut Option<Self>) -> (Self, bool);
    fn recycle(previous: Arc<Self>, reusable: &mut Option<Self>) -> bool;
    fn rebuild(&mut self, world: &PhysicsWorld, step: u64, time: Duration);
    fn body(&self, id: BodyId) -> Option<&BodyState>;
    fn visit_bodies(&self);
    fn len(&self) -> usize;
    fn step(&self) -> u64;
    fn capacities(&self) -> (usize, usize);
    fn storage_bytes(&self) -> usize;
    fn dense_index(&self) -> Option<bool>;
}

impl SnapshotBench for marshalling::PhysicsSnapshot {
    fn from_world(world: &PhysicsWorld, step: u64, time: Duration) -> Self {
        Self::from_world(world, step, time)
    }

    fn take_reusable(reusable: &mut Option<Self>) -> (Self, bool) {
        Self::take_reusable(reusable)
    }

    fn recycle(previous: Arc<Self>, reusable: &mut Option<Self>) -> bool {
        Self::recycle(previous, reusable)
    }

    fn rebuild(&mut self, world: &PhysicsWorld, step: u64, time: Duration) {
        Self::rebuild(self, world, step, time);
    }

    fn body(&self, id: BodyId) -> Option<&BodyState> {
        Self::body(self, id)
    }

    fn visit_bodies(&self) {
        for (id, state) in self.bodies() {
            black_box((id, state));
        }
    }

    fn len(&self) -> usize {
        Self::len(self)
    }

    fn step(&self) -> u64 {
        Self::step(self)
    }

    fn capacities(&self) -> (usize, usize) {
        self.buffer_capacities()
    }

    fn storage_bytes(&self) -> usize {
        self.estimated_storage_bytes()
    }

    fn dense_index(&self) -> Option<bool> {
        Some(self.uses_dense_index())
    }
}

#[derive(Clone, Copy)]
struct LegacyEntry {
    id: BodyId,
    state: BodyState,
}

/// Benchmark-only copy of the aeadef Vec + HashMap snapshot layout.
struct LegacySnapshot {
    step: u64,
    entries: Vec<LegacyEntry>,
    indices: HashMap<BodyId, usize>,
}

impl LegacySnapshot {
    fn empty() -> Self {
        Self {
            step: 0,
            entries: Vec::new(),
            indices: HashMap::new(),
        }
    }
}

impl SnapshotBench for LegacySnapshot {
    fn from_world(world: &PhysicsWorld, step: u64, _time: Duration) -> Self {
        let mut snapshot = Self::empty();
        snapshot.rebuild(world, step, Duration::ZERO);
        snapshot
    }

    fn take_reusable(reusable: &mut Option<Self>) -> (Self, bool) {
        match reusable.take() {
            Some(snapshot) => (snapshot, true),
            None => (Self::empty(), false),
        }
    }

    fn recycle(previous: Arc<Self>, reusable: &mut Option<Self>) -> bool {
        match Arc::try_unwrap(previous) {
            Ok(snapshot) => {
                *reusable = Some(snapshot);
                true
            }
            Err(_) => false,
        }
    }

    fn rebuild(&mut self, world: &PhysicsWorld, step: u64, _time: Duration) {
        let body_count = world.body_count();
        self.entries.clear();
        self.indices.clear();
        self.entries.reserve(body_count);
        self.indices.reserve(body_count);
        for (id, body) in world.bodies() {
            let position = self.entries.len();
            self.entries.push(LegacyEntry {
                id,
                state: *body.state(),
            });
            self.indices.insert(id, position);
        }
        self.step = step;
    }

    fn body(&self, id: BodyId) -> Option<&BodyState> {
        self.indices
            .get(&id)
            .and_then(|position| self.entries.get(*position))
            .map(|entry| &entry.state)
    }

    fn visit_bodies(&self) {
        for entry in &self.entries {
            black_box((entry.id, &entry.state));
        }
    }

    fn len(&self) -> usize {
        self.entries.len()
    }

    fn step(&self) -> u64 {
        self.step
    }

    fn capacities(&self) -> (usize, usize) {
        (self.entries.capacity(), self.indices.capacity())
    }

    fn storage_bytes(&self) -> usize {
        self.entries.capacity() * size_of::<LegacyEntry>()
            + self.indices.capacity() * size_of::<(BodyId, usize)>()
    }

    fn dense_index(&self) -> Option<bool> {
        None
    }
}

async fn report_marshalling() -> Result<(), Box<dyn Error>> {
    println!(
        "\nphysics marshalling (dynamic bodies, {STEPS} steps, {SAMPLES} samples, median; Tokio timer paused)"
    );
    for (size, body_count) in BODY_COUNTS {
        let mut controller_wall = Vec::with_capacity(SAMPLES);
        let mut no_command = Vec::with_capacity(SAMPLES);
        let mut with_commands = Vec::with_capacity(SAMPLES);
        let mut simulation_wall = Vec::with_capacity(SAMPLES);
        let mut simulation_commands_wall = Vec::with_capacity(SAMPLES);
        let mut snapshot_reads = Vec::with_capacity(SAMPLES);

        for _ in 0..SAMPLES {
            controller_wall.push(controller_sample(body_count).await?);
            no_command.push(pipeline_sample(body_count, 0).await?);
            with_commands.push(pipeline_sample(body_count, 100).await?);
            simulation_wall.push(simulation_sample(body_count, 0).await?);
            simulation_commands_wall.push(simulation_sample(body_count, 100).await?);
            snapshot_reads.push(snapshot_read_sample(body_count)?);
        }

        let baseline = median(&mut controller_wall);
        let plain = median_phase(&mut no_command);
        let commanded = median_phase(&mut with_commands);
        let actual_simulation = median_simulation(&mut simulation_wall);
        let actual_commands = median_simulation(&mut simulation_commands_wall);
        let reads = median(&mut snapshot_reads);
        println!(
            "\n{size}: {body_count} bodies | PhysicsController::step wall={:.2} ms | simulation API no commands={:.2} ms ({} steps, physics={:.2} ms), 100 ApplyForce/step={:.2} ms ({} steps, physics={:.2} ms)",
            millis(baseline),
            millis(actual_simulation.wall),
            actual_simulation.steps,
            millis(actual_simulation.physics),
            millis(actual_commands.wall),
            actual_commands.steps,
            millis(actual_commands.physics),
        );
        println!(
            "  isolated pipeline / {STEPS} steps: commands={:.3} ms, controller={:.2} ms, snapshot build={:.2} ms, watch publication={:.3} ms, total={:.2} ms",
            millis(commanded.commands),
            millis(commanded.physics),
            millis(commanded.snapshot_build),
            millis(commanded.publication),
            millis(commanded.total),
        );
        println!(
            "  no-command pipeline: controller={:.2} ms, snapshot build={:.2} ms, watch publication={:.3} ms, total={:.2} ms; 4 clients x 1,000 snapshot reads={:.3} ms",
            millis(plain.physics),
            millis(plain.snapshot_build),
            millis(plain.publication),
            millis(plain.total),
            millis(reads),
        );
    }
    report_snapshot_recycling().await?;
    Ok(())
}

async fn report_snapshot_recycling() -> Result<(), Box<dyn Error>> {
    println!(
        "\nadaptive snapshots vs aeadef Vec + HashMap ({STEPS} steps/sample, {SAMPLES} samples, median)"
    );
    println!(
        "Each step measures rebuild, 1,000 lookups, full bodies iteration, watch publication, and physics-step wall time. World setup and scenario mutations are excluded."
    );

    for scenario in SnapshotScenario::ALL {
        let mut adaptive = snapshot_samples::<marshalling::PhysicsSnapshot>(scenario).await?;
        let mut legacy = snapshot_samples::<LegacySnapshot>(scenario).await?;
        report_snapshot_sample(scenario.name, "adaptive", median_snapshot(&mut adaptive));
        report_snapshot_sample(scenario.name, "aeadef", median_snapshot(&mut legacy));
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct SnapshotScenario {
    name: &'static str,
    initial_slots: usize,
    starting_live: usize,
    target_live: usize,
    sparse_start: bool,
    shrink_before_measurement: bool,
    oscillate: bool,
    retain_old_frames: bool,
}

impl SnapshotScenario {
    const ALL: [Self; 6] = [
        Self {
            name: "Dense 20k stable",
            initial_slots: 20_000,
            starting_live: 20_000,
            target_live: 20_000,
            sparse_start: false,
            shrink_before_measurement: false,
            oscillate: false,
            retain_old_frames: false,
        },
        Self {
            name: "Sparse 20k slots / 100 live",
            initial_slots: 20_000,
            starting_live: 100,
            target_live: 100,
            sparse_start: true,
            shrink_before_measurement: false,
            oscillate: false,
            retain_old_frames: false,
        },
        Self {
            name: "Shrink 20k -> 100",
            initial_slots: 20_000,
            starting_live: 20_000,
            target_live: 100,
            sparse_start: false,
            shrink_before_measurement: true,
            oscillate: false,
            retain_old_frames: false,
        },
        Self {
            name: "Grow 100 -> 20k",
            initial_slots: 20_000,
            starting_live: 100,
            target_live: 20_000,
            sparse_start: true,
            shrink_before_measurement: false,
            oscillate: false,
            retain_old_frames: false,
        },
        Self {
            name: "Oscillation 180/220",
            initial_slots: 1_000,
            starting_live: 1_000,
            target_live: 220,
            sparse_start: false,
            shrink_before_measurement: true,
            oscillate: true,
            retain_old_frames: false,
        },
        Self {
            name: "Retained Arc, 20k -> 100",
            initial_slots: 20_000,
            starting_live: 20_000,
            target_live: 100,
            sparse_start: false,
            shrink_before_measurement: true,
            oscillate: false,
            retain_old_frames: true,
        },
    ];
}

async fn snapshot_samples<S: SnapshotBench>(
    scenario: SnapshotScenario,
) -> Result<Vec<SnapshotSample>, Box<dyn Error>> {
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        samples.push(snapshot_sample::<S>(scenario).await?);
    }
    Ok(samples)
}

async fn snapshot_sample<S: SnapshotBench>(
    scenario: SnapshotScenario,
) -> Result<SnapshotSample, Box<dyn Error>> {
    let mut world = make_world(scenario.initial_slots, WORKLOADS[0])?;
    if scenario.sparse_start {
        retain_last_bodies(&mut world, scenario.starting_live)?;
    }
    let mut controller = PhysicsController::new(world, config(1))?;
    let initial = Arc::new(S::from_world(controller.world()?, 0, Duration::ZERO));
    let initial_body = controller
        .world()?
        .bodies()
        .next()
        .ok_or("snapshot benchmark world is empty")?
        .0;
    let held_initial = scenario.retain_old_frames.then(|| Arc::clone(&initial));
    let (snapshot_tx, snapshot_rx) = watch::channel(initial);
    let mut reusable = if scenario.oscillate {
        Some(S::from_world(controller.world()?, 0, Duration::ZERO))
    } else {
        None
    };

    if scenario.shrink_before_measurement {
        retain_last_bodies(controller.world_mut()?, scenario.target_live)?;
    } else if scenario.target_live > scenario.starting_live {
        grow_world(controller.world_mut()?, scenario.target_live)?;
    }
    let mut lookup_ids = controller
        .world()?
        .bodies()
        .map(|(id, _)| id)
        .collect::<Vec<_>>();
    let mut retained = VecDeque::with_capacity(3);
    let mut last_mode = snapshot_rx.borrow().dense_index();
    let mut simulation_time = Duration::ZERO;
    let dt = Duration::from_secs_f64(1.0 / 60.0);
    let mut sample = SnapshotSample::default();

    for step_index in 0..STEPS {
        if scenario.oscillate {
            oscillate_world(controller.world_mut()?, &mut lookup_ids)?;
        }

        let step_started = Instant::now();
        let stats = controller.step().await?;
        sample.physics += stats.step_duration;
        simulation_time = simulation_time
            .checked_add(dt)
            .ok_or("benchmark simulation time overflow")?;

        let (mut snapshot, reused) = S::take_reusable(&mut reusable);
        sample.reused_buffers += usize::from(reused);
        let build_started = Instant::now();
        snapshot.rebuild(
            controller.world()?,
            controller.metrics().total_steps,
            simulation_time,
        );
        sample.build += build_started.elapsed();

        let mode = snapshot.dense_index();
        if last_mode.is_some() && mode.is_some() && last_mode != mode {
            sample.index_switches += 1;
        }
        last_mode = mode;
        let (entries_capacity, index_capacity) = snapshot.capacities();
        sample.entries_capacity = entries_capacity;
        sample.index_capacity = index_capacity;
        sample.storage_bytes = snapshot.storage_bytes();

        let lookup_started = Instant::now();
        for index in 0..1_000 {
            let id = lookup_ids[index % lookup_ids.len()];
            black_box(snapshot.body(id));
        }
        sample.lookup += lookup_started.elapsed();

        let iteration_started = Instant::now();
        snapshot.visit_bodies();
        sample.iteration += iteration_started.elapsed();

        let publication_started = Instant::now();
        let previous = snapshot_tx.send_replace(Arc::new(snapshot));
        sample.publication += publication_started.elapsed();
        if scenario.retain_old_frames {
            retained.push_back(Arc::clone(&previous));
            if retained.len() > 3 {
                retained.pop_front();
            }
        }
        if S::recycle(previous, &mut reusable) {
            sample.reclaimed_buffers += 1;
        }
        sample.total += step_started.elapsed();

        if snapshot_rx.borrow().step() != controller.metrics().total_steps {
            return Err("benchmark publication does not match the completed step".into());
        }
        if scenario.oscillate && lookup_ids.len() != if step_index % 2 == 0 { 180 } else { 220 } {
            return Err("oscillation benchmark did not reach its requested live count".into());
        }
    }

    if let Some(old) = held_initial {
        if old.len() != scenario.initial_slots
            || old.step() != 0
            || old.body(initial_body).is_none()
        {
            return Err("retained benchmark snapshot was changed or lost".into());
        }
        if controller.world()?.body_count() != scenario.target_live {
            return Err("retained benchmark world did not shrink".into());
        }
    }
    Ok(sample)
}

fn retain_last_bodies(world: &mut PhysicsWorld, live: usize) -> Result<(), Box<dyn Error>> {
    let ids = world.bodies().map(|(id, _)| id).collect::<Vec<_>>();
    let remove_count = ids.len().saturating_sub(live);
    for id in &ids[..remove_count] {
        world.remove_body(*id).ok_or("missing benchmark body")?;
    }
    Ok(())
}

fn grow_world(world: &mut PhysicsWorld, target: usize) -> Result<(), Box<dyn Error>> {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    while world.body_count() < target {
        let index = world.body_count();
        world.add_body(make_dynamic_body(index, &mass)?)?;
    }
    Ok(())
}

fn oscillate_world(
    world: &mut PhysicsWorld,
    lookup_ids: &mut Vec<BodyId>,
) -> Result<(), Box<dyn Error>> {
    if lookup_ids.len() == 220 {
        for id in lookup_ids.iter().take(40) {
            world
                .remove_body(*id)
                .ok_or("missing oscillating benchmark body")?;
        }
    } else {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        for _ in 0..40 {
            let index = world.body_count();
            world.add_body(make_dynamic_body(index, &mass)?)?;
        }
    }
    *lookup_ids = world.bodies().map(|(id, _)| id).collect();
    Ok(())
}

fn median_snapshot(samples: &mut [SnapshotSample]) -> SnapshotSample {
    samples.sort_unstable_by_key(|sample| sample.total);
    samples[samples.len() / 2]
}

fn report_snapshot_sample(scenario: &str, implementation: &str, sample: SnapshotSample) {
    println!(
        "{scenario:31} {implementation:8} build={:7.2} ms lookup1k={:6.3} ms iterate={:7.2} ms publish={:6.3} ms full={:7.2} ms cap={:6}/{:6} memory={:7.1} KiB reused={} reclaimed={} switches={}",
        millis(sample.build),
        millis(sample.lookup),
        millis(sample.iteration),
        millis(sample.publication),
        millis(sample.total),
        sample.entries_capacity,
        sample.index_capacity,
        sample.storage_bytes as f64 / 1024.0,
        sample.reused_buffers,
        sample.reclaimed_buffers,
        sample.index_switches,
    );
}

async fn controller_sample(body_count: usize) -> Result<Duration, Box<dyn Error>> {
    let world = make_world(body_count, WORKLOADS[0])?;
    let mut controller = PhysicsController::new(world, config(1))?;
    let started = Instant::now();
    for _ in 0..STEPS {
        black_box(controller.step().await?);
    }
    Ok(started.elapsed())
}

async fn pipeline_sample(
    body_count: usize,
    commands_per_step: usize,
) -> Result<PhaseTimes, Box<dyn Error>> {
    let world = make_world(body_count, WORKLOADS[0])?;
    let ids = world.bodies().map(|(id, _)| id).collect::<Vec<_>>();
    let mut controller = PhysicsController::new(world, config(1))?;
    let initial = Arc::new(marshalling::PhysicsSnapshot::from_world(
        controller.world()?,
        0,
        Duration::ZERO,
    ));
    let (snapshot_tx, _snapshot_rx) = watch::channel(initial);
    let mut reusable = None;
    let mut simulation_time = Duration::ZERO;
    let dt = Duration::from_secs_f64(1.0 / 60.0);
    let mut times = PhaseTimes::default();
    let started = Instant::now();

    for _ in 0..STEPS {
        let commands = ids
            .iter()
            .take(commands_per_step)
            .map(|id| marshalling::PhysicsCommand::ApplyForce {
                id: *id,
                force: Vec3::Y * 0.1,
            })
            .collect::<Vec<_>>();
        let phase = Instant::now();
        let world = controller.world_mut()?;
        for command in commands {
            black_box(marshalling::apply_command(world, command)?);
        }
        times.commands += phase.elapsed();

        let stats = controller.step().await?;
        times.physics += stats.step_duration;
        simulation_time = simulation_time
            .checked_add(dt)
            .ok_or("benchmark simulation time overflow")?;

        let phase = Instant::now();
        let (mut snapshot, _) = marshalling::PhysicsSnapshot::take_reusable(&mut reusable);
        snapshot.rebuild(
            controller.world()?,
            controller.metrics().total_steps,
            simulation_time,
        );
        times.snapshot_build += phase.elapsed();

        let phase = Instant::now();
        let previous = snapshot_tx.send_replace(Arc::new(snapshot));
        marshalling::PhysicsSnapshot::recycle(previous, &mut reusable);
        times.publication += phase.elapsed();
    }
    times.total = started.elapsed();
    Ok(times)
}

async fn simulation_sample(
    body_count: usize,
    commands_per_step: usize,
) -> Result<SimulationTimes, Box<dyn Error>> {
    let world = make_world(body_count, WORKLOADS[0])?;
    let command_ids = world
        .bodies()
        .take(commands_per_step)
        .map(|(id, _)| id)
        .collect::<Vec<_>>();
    let mut simulation = PhysicsSimulation::with_config(
        world,
        SchedulerConfig {
            max_workers: 1,
            max_catch_up_steps: 1,
            adaptive: false,
            ..SchedulerConfig::default()
        },
    )?;
    let client = simulation.client();
    let mut reader = client.clone();
    simulation.start()?;
    tokio::task::yield_now().await;
    let start_snapshot = reader.snapshot();
    tokio::task::yield_now().await;

    let started = Instant::now();
    let mut expected_step = start_snapshot.step().saturating_add(1);
    let mut physics = Duration::ZERO;
    for _ in 0..STEPS {
        let mut submissions = Vec::with_capacity(command_ids.len());
        for id in &command_ids {
            submissions.push(Box::pin(client.apply_force(*id, Vec3::Y * 0.1)));
        }
        for submission in &mut submissions {
            if !matches!(poll_once(submission.as_mut()), Poll::Pending) {
                return Err("physics command completed before a step".into());
            }
        }

        let snapshot =
            tokio::time::timeout(Duration::from_secs(5), reader.next_snapshot()).await??;
        if snapshot.step() != expected_step {
            return Err("simulation did not advance exactly one benchmark step".into());
        }
        expected_step = snapshot.step().saturating_add(1);
        for submission in submissions {
            submission.await?;
        }
        let metrics = simulation.metrics();
        if metrics.total_steps != snapshot.step() {
            return Err("command batch was not applied in one physical step".into());
        }
        physics += metrics.step_duration;
        tokio::task::yield_now().await;
    }
    let elapsed = started.elapsed();
    simulation.stop().await?;
    Ok(SimulationTimes {
        wall: elapsed,
        physics,
        steps: simulation.metrics().total_steps,
    })
}

fn snapshot_read_sample(body_count: usize) -> Result<Duration, Box<dyn Error>> {
    let world = make_world(body_count, WORKLOADS[0])?;
    let first_id = world.bodies().next().ok_or("empty benchmark world")?.0;
    let initial = Arc::new(marshalling::PhysicsSnapshot::from_world(
        &world,
        0,
        Duration::ZERO,
    ));
    let (command_tx, _command_rx) = mpsc::channel(1);
    let (_command_state_tx, command_state_rx) = watch::channel(Some(command_tx));
    let (_snapshot_tx, snapshot_rx) = watch::channel(initial);
    let clients = (0..4)
        .map(|_| marshalling::PhysicsClient::new(command_state_rx.clone(), snapshot_rx.clone()))
        .collect::<Vec<_>>();

    let started = Instant::now();
    for _ in 0..1_000 {
        for client in &clients {
            let snapshot = client.snapshot();
            black_box(snapshot.body(first_id));
        }
    }
    Ok(started.elapsed())
}

fn poll_once<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
    let waker = Waker::noop();
    future.poll(&mut Context::from_waker(waker))
}

fn median(samples: &mut [Duration]) -> Duration {
    samples.sort_unstable();
    samples[samples.len() / 2]
}

fn median_phase(samples: &mut [PhaseTimes]) -> PhaseTimes {
    samples.sort_unstable_by_key(|sample| sample.total);
    samples[samples.len() / 2]
}

fn median_simulation(samples: &mut [SimulationTimes]) -> SimulationTimes {
    samples.sort_unstable_by_key(|sample| sample.wall);
    samples[samples.len() / 2]
}

fn millis(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1000.0
}

fn config(workers: usize) -> SchedulerConfig {
    SchedulerConfig {
        max_workers: workers,
        max_catch_up_steps: 1,
        adaptive: false,
        ..SchedulerConfig::default()
    }
}

fn make_world(body_count: usize, workload: Workload) -> Result<PhysicsWorld, Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let static_count = body_count * workload.static_percent / 100;
    let sleeping_count = body_count * workload.sleeping_percent / 100;
    for index in 0..body_count {
        if index < static_count {
            world.add_body(RigidBody::new_static(Transform::identity()))?;
            continue;
        }

        let mut body = make_dynamic_body(index, &mass)?;
        if index < static_count + sleeping_count {
            body.sleep()?;
        }
        world.add_body(body)?;
    }
    Ok(world)
}

fn make_dynamic_body(index: usize, mass: &MassProperties) -> Result<RigidBody, Box<dyn Error>> {
    let scale = index as f64 * 0.001 + 1.0;
    let mut body = RigidBody::new_dynamic(Transform::identity(), *mass)?;
    body.set_linear_velocity(Vec3::new(scale, -scale, 0.5 * scale))?;
    body.set_angular_velocity(Vec3::new(0.1 * scale, 0.2, -0.3))?;
    body.apply_force(Vec3::new(0.25, -0.5 * scale, 0.75))?;
    body.apply_torque(Vec3::new(-0.1 * scale, 0.2, 0.05))?;
    Ok(body)
}

async fn static_sharded_step(
    world: PhysicsWorld,
    dt: f64,
    workers: usize,
) -> Result<PhysicsWorld, Box<dyn Error>> {
    let (world, mut shards, result) = tokio::task::spawn_blocking(move || {
        let mut shards = (0..workers).map(|_| Vec::new()).collect::<Vec<_>>();
        let result = world.fill_step_shards(&mut shards);
        (world, shards, result)
    })
    .await?;
    result?;

    let tasks = shards
        .drain(..)
        .map(|mut shard| {
            tokio::task::spawn_blocking(move || {
                let mut error = None;
                for body in &mut shard {
                    if let Err(step_error) = body.integrate(dt) {
                        error = Some(step_error);
                        break;
                    }
                }
                (shard, error)
            })
        })
        .collect::<Vec<_>>();
    let mut buffers = Vec::with_capacity(tasks.len());
    let mut integration_error = None;
    for task in tasks {
        let (shard, error) = task.await?;
        buffers.push(shard);
        if integration_error.is_none() {
            integration_error = error;
        }
    }
    if let Some(error) = integration_error {
        return Err(Box::new(error));
    }

    let (world, result) = tokio::task::spawn_blocking(move || {
        let mut world = world;
        let result = world.commit_step_shards(&buffers);
        (world, result)
    })
    .await?;
    result?;
    Ok(world)
}

fn sample(
    count: usize,
    mut run: impl FnMut() -> Result<Duration, Box<dyn Error>>,
) -> Result<Duration, Box<dyn Error>> {
    let mut timings = Vec::with_capacity(count);
    for _ in 0..count {
        timings.push(run()?);
    }
    timings.sort_unstable();
    Ok(black_box(timings[count / 2]))
}

fn report(label: &str, duration: Duration, sequential: Duration) {
    println!(
        "{label:20} median={:8.2} ms  speedup={:.2}x",
        duration.as_secs_f64() * 1_000.0,
        sequential.as_secs_f64() / duration.as_secs_f64()
    );
}
