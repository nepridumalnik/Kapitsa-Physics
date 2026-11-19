use kapitsa_core::world::PhysicsWorld;
use tokio::sync::oneshot;

use super::PhysicsClientError;

pub(crate) use kapitsa_core::command::BatchScratch;
#[cfg(test)]
pub(crate) use kapitsa_core::command::SMALL_STAGING_LINEAR_SEARCH_LIMIT;
pub use kapitsa_core::command::{
    CommandError, PhysicsCommand, PhysicsCommandBuffer, PhysicsCommandResult,
};

impl From<CommandError> for PhysicsClientError {
    fn from(error: CommandError) -> Self {
        match error {
            CommandError::World(error) => Self::World(error),
            CommandError::Body(error) => Self::Body(error),
            CommandError::Cluster(error) => Self::Cluster(error),
            CommandError::Query(error) => Self::Query(error),
            CommandError::ForceSource(error) => Self::ForceSource(error),
            CommandError::UnsupportedBatchCommand => Self::UnsupportedBatchCommand,
            CommandError::TooManyCommands { max_commands } => {
                Self::TooManyCommands { max_commands }
            }
            CommandError::Unified(error) => Self::Unified(error),
        }
    }
}

/// Applies one command through the core dispatcher and maps its typed failure.
pub(crate) fn apply_command(
    world: &mut PhysicsWorld,
    command: PhysicsCommand,
) -> Result<PhysicsCommandResult, PhysicsClientError> {
    world
        .apply_command(command)
        .map_err(PhysicsClientError::from)
}

/// Applies one atomic batch through the core staging path.
pub(crate) fn apply_command_batch(
    world: &mut PhysicsWorld,
    commands: &[PhysicsCommand],
    scratch: &mut BatchScratch,
) -> Result<(), kapitsa_core::Error> {
    world.apply_commands(commands, scratch)
}

/// Correlates an accepted command buffer with its asynchronous result.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BatchId(u64);

impl BatchId {
    pub(crate) fn new(value: u64) -> Self {
        Self(value)
    }

    pub fn get(self) -> u64 {
        self.0
    }
}

/// A synchronous failure while submitting a command buffer.
#[derive(Debug)]
pub enum PhysicsSubmitError {
    EmptyBuffer(PhysicsCommandBuffer),
    TooManyCommands {
        buffer: PhysicsCommandBuffer,
        max_commands: usize,
    },
    BatchIdExhausted(PhysicsCommandBuffer),
    NotRunning(PhysicsCommandBuffer),
    QueueFull(PhysicsCommandBuffer),
    ChannelClosed(PhysicsCommandBuffer),
}

impl PhysicsSubmitError {
    pub fn into_buffer(self) -> PhysicsCommandBuffer {
        match self {
            Self::EmptyBuffer(buffer)
            | Self::BatchIdExhausted(buffer)
            | Self::NotRunning(buffer)
            | Self::QueueFull(buffer)
            | Self::ChannelClosed(buffer) => buffer,
            Self::TooManyCommands { buffer, .. } => buffer,
        }
    }
}

impl std::fmt::Display for PhysicsSubmitError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::EmptyBuffer(_) => "physics command buffer is empty",
            Self::TooManyCommands { max_commands, .. } => {
                return write!(
                    formatter,
                    "physics command buffer exceeds the {max_commands}-command limit"
                );
            }
            Self::BatchIdExhausted(_) => "physics batch identifier space is exhausted",
            Self::NotRunning(_) => "physics simulation is not running",
            Self::QueueFull(_) => "physics command queue is full",
            Self::ChannelClosed(_) => "physics command channel is closed",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for PhysicsSubmitError {}

// Keep single-command requests inline so the existing async path adds no per-command allocation.
#[allow(clippy::large_enum_variant)]
pub(crate) enum CommandRequest {
    Single {
        command: PhysicsCommand,
        reply: oneshot::Sender<Result<PhysicsCommandResult, PhysicsClientError>>,
    },
    Batch {
        id: BatchId,
        buffer: PhysicsCommandBuffer,
    },
    Chunk {
        command: super::chunk::ChunkCommand,
        reply: oneshot::Sender<super::chunk::ChunkReply>,
    },
}

impl CommandRequest {
    pub(crate) fn command_count(&self) -> usize {
        match self {
            Self::Single { .. } => 1,
            Self::Batch { buffer, .. } => buffer.len(),
            Self::Chunk { .. } => 1,
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    #[cfg(feature = "bench-internals")]
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::collections::HashMap;
    use std::hint::black_box;
    use std::time::{Duration, Instant};

    use kapitsa_core::{
        collider::{Collider, CollisionFilter, CollisionMode},
        collision::{QueryError, QueryFilter, Ray},
        joint::{FixedJoint, Joint, JointCommon, JointFrame},
        mass::MassProperties,
        material::{MaterialStorage, PhysicsMaterial},
        rigid_body::{BodyError, RigidBody},
        shape::{Shape, Sphere},
        transform::Transform,
        world::{ForceSourceError, PhysicsWorld, WorldError},
    };
    use kapitsa_math::{Mat3, Quat, Vec3};

    use super::super::PhysicsClientError;
    use super::{
        BatchScratch, CommandError, PhysicsCommand, PhysicsCommandResult,
        SMALL_STAGING_LINEAR_SEARCH_LIMIT, apply_command, apply_command_batch,
    };

    #[allow(dead_code)] // Mirrors the staging record for the allocation-control test.
    struct StagedForceSource {
        id: kapitsa_core::world::ForceSourceId,
        local_point: Vec3,
        local_force: Vec3,
        enabled: bool,
        command_index: usize,
    }

    #[cfg(feature = "bench-internals")]
    use std::sync::Arc;
    #[cfg(feature = "bench-internals")]
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    #[cfg(feature = "bench-internals")]
    use tokio::sync::{broadcast, mpsc, watch};

    #[cfg(feature = "bench-internals")]
    static TRACK_ALLOCATIONS: AtomicBool = AtomicBool::new(false);
    #[cfg(feature = "bench-internals")]
    static ALLOCATION_COUNT: AtomicUsize = AtomicUsize::new(0);
    #[cfg(feature = "bench-internals")]
    static ALLOCATION_BYTES: AtomicUsize = AtomicUsize::new(0);
    #[cfg(feature = "bench-internals")]
    static LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
    #[cfg(feature = "bench-internals")]
    static BASE_LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
    #[cfg(feature = "bench-internals")]
    static PEAK_EXTRA_BYTES: AtomicUsize = AtomicUsize::new(0);

    #[cfg(feature = "bench-internals")]
    struct TrackingAllocator;

    #[cfg(feature = "bench-internals")]
    unsafe impl GlobalAlloc for TrackingAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let pointer = unsafe { System.alloc(layout) };
            if !pointer.is_null() {
                LIVE_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
                if TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
                    ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
                    ALLOCATION_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
                    record_peak_extra_bytes();
                }
            }
            pointer
        }

        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            let pointer = unsafe { System.alloc_zeroed(layout) };
            if !pointer.is_null() {
                LIVE_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
                if TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
                    ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
                    ALLOCATION_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
                    record_peak_extra_bytes();
                }
            }
            pointer
        }

        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let new_pointer = unsafe { System.realloc(pointer, layout, size) };
            if !new_pointer.is_null() {
                LIVE_BYTES.fetch_sub(layout.size(), Ordering::Relaxed);
                LIVE_BYTES.fetch_add(size, Ordering::Relaxed);
                if TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
                    ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
                    ALLOCATION_BYTES.fetch_add(size, Ordering::Relaxed);
                    record_peak_extra_bytes();
                }
            }
            new_pointer
        }

        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            unsafe { System.dealloc(pointer, layout) };
            LIVE_BYTES.fetch_sub(layout.size(), Ordering::Relaxed);
        }
    }

    #[cfg(feature = "bench-internals")]
    fn record_peak_extra_bytes() {
        let live = LIVE_BYTES.load(Ordering::Relaxed);
        let baseline = BASE_LIVE_BYTES.load(Ordering::Relaxed);
        PEAK_EXTRA_BYTES.fetch_max(live.saturating_sub(baseline), Ordering::Relaxed);
    }

    #[cfg(feature = "bench-internals")]
    #[global_allocator]
    static TEST_ALLOCATOR: TrackingAllocator = TrackingAllocator;

    #[cfg(feature = "bench-internals")]
    struct AllocationTracking;

    #[cfg(feature = "bench-internals")]
    impl AllocationTracking {
        fn start() -> Self {
            ALLOCATION_COUNT.store(0, Ordering::Relaxed);
            ALLOCATION_BYTES.store(0, Ordering::Relaxed);
            BASE_LIVE_BYTES.store(LIVE_BYTES.load(Ordering::Relaxed), Ordering::Relaxed);
            PEAK_EXTRA_BYTES.store(0, Ordering::Relaxed);
            TRACK_ALLOCATIONS.store(true, Ordering::Relaxed);
            Self
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) mod allocation_tracking {
        pub(crate) fn start() {
            super::ALLOCATION_COUNT.store(0, super::Ordering::Relaxed);
            super::ALLOCATION_BYTES.store(0, super::Ordering::Relaxed);
            super::BASE_LIVE_BYTES.store(
                super::LIVE_BYTES.load(super::Ordering::Relaxed),
                super::Ordering::Relaxed,
            );
            super::PEAK_EXTRA_BYTES.store(0, super::Ordering::Relaxed);
            super::TRACK_ALLOCATIONS.store(true, super::Ordering::Relaxed);
        }

        pub(crate) fn stop() -> (usize, usize, usize, usize) {
            super::TRACK_ALLOCATIONS.store(false, super::Ordering::Relaxed);
            (
                super::ALLOCATION_COUNT.load(super::Ordering::Relaxed),
                super::ALLOCATION_BYTES.load(super::Ordering::Relaxed),
                super::PEAK_EXTRA_BYTES.load(super::Ordering::Relaxed),
                super::BASE_LIVE_BYTES.load(super::Ordering::Relaxed),
            )
        }

        pub(crate) fn reset_counts() {
            super::ALLOCATION_COUNT.store(0, super::Ordering::Relaxed);
            super::ALLOCATION_BYTES.store(0, super::Ordering::Relaxed);
        }

        pub(crate) fn counts() -> (usize, usize) {
            (
                super::ALLOCATION_COUNT.load(super::Ordering::Relaxed),
                super::ALLOCATION_BYTES.load(super::Ordering::Relaxed),
            )
        }
    }

    #[cfg(feature = "bench-internals")]
    impl Drop for AllocationTracking {
        fn drop(&mut self) {
            TRACK_ALLOCATIONS.store(false, Ordering::Relaxed);
        }
    }

    #[cfg(feature = "bench-internals")]
    #[derive(Clone, Copy)]
    struct BatchAllocationStats {
        count: usize,
        bytes: usize,
    }

    #[cfg(feature = "bench-internals")]
    fn tracked_batch_allocations(
        world: &mut PhysicsWorld,
        commands: &[PhysicsCommand],
        scratch: &mut BatchScratch,
    ) -> BatchAllocationStats {
        let tracking = AllocationTracking::start();
        let result = apply_command_batch(world, commands, scratch);
        drop(tracking);
        result.expect("benchmark batch applies");
        BatchAllocationStats {
            count: ALLOCATION_COUNT.load(Ordering::Relaxed),
            bytes: ALLOCATION_BYTES.load(Ordering::Relaxed),
        }
    }

    #[derive(Clone, Copy)]
    enum BatchScenario {
        RepeatedBody,
        DistinctBodies,
        MixedBodiesAndSources,
    }

    impl BatchScenario {
        const ALL: [Self; 3] = [
            Self::RepeatedBody,
            Self::DistinctBodies,
            Self::MixedBodiesAndSources,
        ];

        fn name(self) -> &'static str {
            match self {
                Self::RepeatedBody => "same-body",
                Self::DistinctBodies => "distinct-bodies",
                Self::MixedBodiesAndSources => "mixed-body-source",
            }
        }
    }

    struct BatchCase {
        world: PhysicsWorld,
        commands: Vec<PhysicsCommand>,
        scratch: BatchScratch,
        staged_bodies: Vec<kapitsa_core::world::BodyId>,
        staged_sources: Vec<kapitsa_core::world::ForceSourceId>,
    }

    fn batch_case(scenario: BatchScenario, command_count: usize) -> BatchCase {
        let mut world = PhysicsWorld::new();
        let mut commands = Vec::with_capacity(command_count);
        let mut staged_bodies = Vec::new();
        let mut staged_sources = Vec::new();

        match scenario {
            BatchScenario::RepeatedBody => {
                let body = world.add_body(dynamic_body()).expect("body");
                staged_bodies.push(body);
                for index in 0..command_count {
                    commands.push(PhysicsCommand::SetLinearVelocity {
                        id: body,
                        velocity: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
                    });
                }
            }
            BatchScenario::DistinctBodies => {
                for index in 0..command_count {
                    let body = world.add_body(dynamic_body()).expect("body");
                    staged_bodies.push(body);
                    commands.push(PhysicsCommand::SetLinearVelocity {
                        id: body,
                        velocity: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
                    });
                }
            }
            BatchScenario::MixedBodiesAndSources => {
                let body = world.add_body(dynamic_body()).expect("body");
                staged_bodies.push(body);
                for index in 0..command_count / 2 {
                    let source = world
                        .add_force_source(body, Vec3::ZERO, Vec3::X)
                        .expect("force source");
                    staged_sources.push(source);
                    commands.push(PhysicsCommand::SetLinearVelocity {
                        id: body,
                        velocity: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
                    });
                    commands.push(PhysicsCommand::SetForceSource {
                        id: source,
                        local_point: Vec3::Y,
                        local_force: Vec3::new(index as f64 + 2.0, 0.0, 0.0),
                    });
                }
            }
        }

        BatchCase {
            world,
            commands,
            scratch: BatchScratch::default(),
            staged_bodies,
            staged_sources,
        }
    }

    fn apply_batch(case: &mut BatchCase) {
        apply_command_batch(
            &mut case.world,
            black_box(&case.commands),
            &mut case.scratch,
        )
        .expect("benchmark batch applies");
    }

    fn staged_buffer_control(case: &BatchCase) {
        let mut bodies = Vec::with_capacity(case.commands.len());
        for id in &case.staged_bodies {
            bodies.push((*id, case.world.body(*id).expect("staged body").motion()));
        }

        let mut sources = Vec::new();
        for (command_index, id) in case.staged_sources.iter().copied().enumerate() {
            let source = *case.world.force_source(id).expect("staged source");
            sources.push(StagedForceSource {
                id,
                local_point: source.local_point(),
                local_force: source.local_force(),
                enabled: source.is_enabled(),
                command_index,
            });
        }
        black_box((&bodies, &sources));
    }

    fn average_ns(samples: &[f64]) -> (f64, f64, f64) {
        let mut sorted = samples.to_vec();
        sorted.sort_by(f64::total_cmp);
        (
            sorted[0],
            sorted[sorted.len() / 2],
            sorted[sorted.len() - 1],
        )
    }

    fn repeats_for(target: Duration, pilot: Duration) -> usize {
        let target_ns = target.as_nanos().max(1);
        let pilot_ns = pilot.as_nanos().max(1);
        usize::try_from(target_ns.div_ceil(pilot_ns))
            .unwrap_or(usize::MAX)
            .clamp(1, 65_536)
    }

    fn calibrated_repeats(
        scenario: BatchScenario,
        command_count: usize,
        target: Duration,
        mut run: impl FnMut(&mut BatchCase),
    ) -> usize {
        const CALIBRATION_REPEATS: usize = 4;
        let mut case = batch_case(scenario, command_count);
        run(&mut case);
        let started = Instant::now();
        for _ in 0..CALIBRATION_REPEATS {
            run(&mut case);
        }
        repeats_for(target, started.elapsed() / CALIBRATION_REPEATS as u32)
    }

    fn run_samples(
        scenario: BatchScenario,
        command_count: usize,
        repeats: usize,
        mut run: impl FnMut(&mut BatchCase),
        mut verify: impl FnMut(&BatchCase),
    ) -> (f64, f64, f64) {
        const SAMPLES: usize = 5;
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let mut case = batch_case(scenario, command_count);
            run(&mut case);
            let started = Instant::now();
            for _ in 0..repeats {
                run(&mut case);
            }
            samples.push(started.elapsed().as_secs_f64() * 1.0e9 / repeats as f64);
            verify(&case);
        }
        average_ns(&samples)
    }

    fn verify_batch_result(case: &BatchCase) {
        let mut expected_bodies = HashMap::new();
        let mut expected_sources = HashMap::new();
        for command in &case.commands {
            match command {
                PhysicsCommand::SetLinearVelocity { id, velocity } => {
                    expected_bodies.insert(*id, *velocity);
                }
                PhysicsCommand::SetForceSource {
                    id,
                    local_point,
                    local_force,
                } => {
                    expected_sources.insert(*id, (*local_point, *local_force));
                }
                _ => panic!("unexpected command in benchmark batch"),
            }
        }
        for (id, expected) in expected_bodies {
            assert_eq!(
                case.world
                    .body(id)
                    .expect("updated body")
                    .state()
                    .linear_velocity(),
                expected
            );
        }
        for (id, (expected_point, expected_force)) in expected_sources {
            let source = case.world.force_source(id).expect("updated source");
            assert_eq!(source.local_point(), expected_point);
            assert_eq!(source.local_force(), expected_force);
        }
    }

    fn mass(value: f64) -> MassProperties {
        MassProperties::new(value, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass")
    }

    fn dynamic_body() -> RigidBody {
        RigidBody::new_dynamic(Transform::identity(), mass(1.0)).expect("dynamic body")
    }

    fn collider() -> Collider {
        Collider::new(
            Shape::Sphere(Sphere::new(1.0).expect("valid sphere")),
            Transform::identity(),
        )
    }

    fn assert_done(result: Result<PhysicsCommandResult, PhysicsClientError>) {
        assert!(matches!(result, Ok(PhysicsCommandResult::Done)));
    }

    #[test]
    fn add_remove_and_reused_body_ids_are_handled_by_world() {
        let mut world = PhysicsWorld::new();
        let PhysicsCommandResult::Body(first) = apply_command(
            &mut world,
            PhysicsCommand::AddBody {
                body: dynamic_body(),
            },
        )
        .expect("add body") else {
            panic!("add body must return its ID");
        };
        assert!(world.body(first).is_some());
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::RemoveBody { id: first },
        ));
        assert!(world.body(first).is_none());

        let PhysicsCommandResult::Body(replacement) = apply_command(
            &mut world,
            PhysicsCommand::AddBody {
                body: dynamic_body(),
            },
        )
        .expect("reuse body slot") else {
            panic!("add body must return its ID");
        };
        assert_eq!(first.index(), replacement.index());
        assert_ne!(first.generation(), replacement.generation());
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::SetTransform {
                    id: first,
                    transform: Transform::identity(),
                }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
    }

    #[test]
    fn set_aerodynamics_replaces_the_body_model() {
        use kapitsa_core::aerodynamics::{AerodynamicProperties, Aerodynamics};

        let mut world = PhysicsWorld::new();
        let PhysicsCommandResult::Body(body) = apply_command(
            &mut world,
            PhysicsCommand::AddBody {
                body: dynamic_body(),
            },
        )
        .expect("add body") else {
            panic!("add body must return its ID");
        };
        let simple = Aerodynamics::Simple(
            AerodynamicProperties::new(2.0, 1.0, Vec3::ZERO).expect("simple aero"),
        );
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::SetAerodynamics {
                body,
                model: simple,
            },
        ));
        assert!(matches!(
            world.body(body).expect("body").aerodynamics(),
            Aerodynamics::Simple(_)
        ));
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::SetAerodynamics {
                body,
                model: Aerodynamics::Colliders,
            },
        ));
        assert!(matches!(
            world.body(body).expect("body").aerodynamics(),
            Aerodynamics::Colliders
        ));
    }

    #[test]
    fn motion_batch_preserves_owned_aerodynamic_mesh() {
        use kapitsa_core::{aerodynamics::Aerodynamics, shape::TriangleMesh};

        let mesh = TriangleMesh::new(vec![Vec3::ZERO, Vec3::X, Vec3::Y], vec![[0, 1, 2]]).unwrap();
        let vertices = mesh.vertices().as_ptr();
        let mut body = dynamic_body();
        body.set_aerodynamics(Aerodynamics::Mesh(mesh));
        let mut world = PhysicsWorld::new();
        let id = world.add_body(body).unwrap();
        let mut scratch = BatchScratch::default();
        apply_command_batch(
            &mut world,
            &[PhysicsCommand::SetLinearVelocity {
                id,
                velocity: Vec3::X,
            }],
            &mut scratch,
        )
        .unwrap();
        let body = world.body(id).unwrap();
        let Aerodynamics::Mesh(mesh) = body.aerodynamics() else {
            panic!("motion batch discarded aerodynamic mesh");
        };
        assert_eq!(mesh.vertices().as_ptr(), vertices);
        assert_eq!(body.state().linear_velocity(), Vec3::X);

        let result = apply_command_batch(
            &mut world,
            &[
                PhysicsCommand::SetLinearVelocity {
                    id,
                    velocity: Vec3::Y,
                },
                PhysicsCommand::SetAngularVelocity {
                    id,
                    velocity: Vec3::splat(f64::NAN),
                },
            ],
            &mut scratch,
        );
        assert!(result.is_err());
        let body = world.body(id).unwrap();
        assert_eq!(body.state().linear_velocity(), Vec3::X);
        let Aerodynamics::Mesh(mesh) = body.aerodynamics() else {
            panic!("failed motion batch discarded aerodynamic mesh");
        };
        assert_eq!(mesh.vertices().as_ptr(), vertices);
    }

    #[test]
    fn commands_reject_body_ids_from_another_world() {
        let mut source = PhysicsWorld::new();
        let foreign = source
            .add_body(dynamic_body())
            .expect("source body insertion");
        let mut target = PhysicsWorld::new();
        let local = target
            .add_body(dynamic_body())
            .expect("target body insertion");

        assert!(matches!(
            apply_command(
                &mut target,
                PhysicsCommand::SetLinearVelocity {
                    id: foreign,
                    velocity: Vec3::X,
                }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
        assert!(matches!(
            apply_command(&mut target, PhysicsCommand::RemoveBody { id: foreign }),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
        assert_eq!(target.body_count(), 1);
        assert!(target.body(local).is_some());
        assert_eq!(
            target
                .body(local)
                .expect("local body remains")
                .state()
                .linear_velocity(),
            Vec3::ZERO
        );
    }

    #[test]
    fn body_commands_delegate_to_existing_body_operations() {
        let mut world = PhysicsWorld::new();
        let id = world.add_body(dynamic_body()).expect("body");
        let transform = Transform::new(Vec3::X, Quat::IDENTITY).expect("transform");
        let commands = [
            PhysicsCommand::SetTransform { id, transform },
            PhysicsCommand::SetLinearVelocity {
                id,
                velocity: Vec3::X,
            },
            PhysicsCommand::SetAngularVelocity {
                id,
                velocity: Vec3::Y,
            },
            PhysicsCommand::SetMass {
                id,
                mass: mass(2.0),
            },
            PhysicsCommand::ApplyForce { id, force: Vec3::X },
            PhysicsCommand::ApplyForceAtPoint {
                id,
                force: Vec3::Y,
                point: Vec3::Z,
            },
            PhysicsCommand::ApplyTorque {
                id,
                torque: Vec3::Z,
            },
            PhysicsCommand::ApplyImpulse {
                id,
                impulse: Vec3::X,
            },
            PhysicsCommand::ApplyImpulseAtPoint {
                id,
                impulse: Vec3::Y,
                point: Vec3::Z,
            },
        ];
        for command in commands {
            assert_done(apply_command(&mut world, command));
        }
        let body = world.body(id).expect("body remains");
        assert_eq!(body.state().transform(), transform);
        assert_eq!(body.physics().mass(), Some(mass(2.0)));
        assert_ne!(body.forces().force(), Vec3::ZERO);
        assert_ne!(body.forces().torque(), Vec3::ZERO);
        assert_eq!(body.state().linear_velocity(), Vec3::new(1.5, 0.5, 0.0));

        assert_done(apply_command(&mut world, PhysicsCommand::Sleep { id }));
        assert!(world.body(id).expect("body").state().is_sleeping());
        assert_done(apply_command(&mut world, PhysicsCommand::WakeUp { id }));
        assert!(!world.body(id).expect("body").state().is_sleeping());

        apply_command(
            &mut world,
            PhysicsCommand::ApplyForce { id, force: Vec3::X },
        )
        .expect("apply force");
        apply_command(&mut world, PhysicsCommand::ClearForces { id }).expect("clear forces");
        assert_eq!(world.body(id).expect("body").forces().force(), Vec3::ZERO);
        assert_eq!(world.body(id).expect("body").forces().torque(), Vec3::ZERO);
    }

    #[test]
    fn invalid_body_and_numeric_commands_return_errors_without_partial_changes() {
        let mut world = PhysicsWorld::new();
        let id = world.add_body(dynamic_body()).expect("body");
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::ApplyForce {
                    id,
                    force: Vec3::new(f64::INFINITY, 0.0, 0.0),
                }
            ),
            Err(PhysicsClientError::Body(BodyError::NonFiniteValue))
        ));
        let body = world.body(id).expect("body unchanged");
        assert_eq!(body.forces().force(), Vec3::ZERO);

        let invalid = world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .unwrap();
        assert!(matches!(
            apply_command(&mut world, PhysicsCommand::Sleep { id: invalid }),
            Err(PhysicsClientError::Body(BodyError::BodyMustBeDynamic))
        ));
    }

    #[test]
    fn collider_commands_update_remove_and_follow_body_removal() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let PhysicsCommandResult::Collider(collider_id) = apply_command(
            &mut world,
            PhysicsCommand::AddCollider {
                body,
                collider: collider(),
            },
        )
        .expect("add collider") else {
            panic!("add collider must return its ID");
        };
        let filter = CollisionFilter::new(3, 7);
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::SetColliderFilter {
                id: collider_id,
                filter,
            },
        ));
        assert_eq!(
            world.collider(collider_id).expect("collider").filter(),
            filter
        );
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::SetColliderMode {
                id: collider_id,
                mode: CollisionMode::Sensor,
            },
        ));
        assert_eq!(
            world.collider(collider_id).expect("collider").mode(),
            CollisionMode::Sensor
        );
        apply_command(
            &mut world,
            PhysicsCommand::SetColliderMaterial {
                id: collider_id,
                material: None,
            },
        )
        .expect("clear material");
        assert_eq!(
            world.collider(collider_id).expect("collider").material(),
            None
        );
        apply_command(&mut world, PhysicsCommand::RemoveBody { id: body })
            .expect("remove body and attached collider");
        assert!(world.collider(collider_id).is_none());
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::RemoveCollider { id: collider_id }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidColliderId))
        ));
    }

    #[test]
    fn collider_material_command_rejects_ids_from_other_storages() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let collider = world.add_collider(body, collider()).expect("collider");
        let mut foreign_materials = MaterialStorage::new();
        let foreign_id = foreign_materials
            .insert(PhysicsMaterial::default())
            .expect("foreign material");

        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::SetColliderMaterial {
                    id: collider,
                    material: Some(foreign_id),
                }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidMaterialId))
        ));
        assert_eq!(world.collider(collider).expect("collider").material(), None);
    }

    #[test]
    fn spatial_queries_run_through_the_world_command_handler() {
        use kapitsa_core::collision::{OverlapHit, RayHit, ShapeHit};

        let mut world = PhysicsWorld::new();
        let body = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let collider = world
            .add_collider(
                body,
                Collider::new(
                    Shape::Sphere(Sphere::new(1.0).unwrap()),
                    Transform::new(Vec3::new(3.0, 0.0, 0.0), kapitsa_math::Quat::IDENTITY).unwrap(),
                ),
            )
            .unwrap();
        let filter = QueryFilter::default();
        let ray = Ray::new(Vec3::ZERO, Vec3::X, 10.0).unwrap();
        let result = apply_command(&mut world, PhysicsCommand::Raycast { ray, filter }).unwrap();
        assert!(
            matches!(result, PhysicsCommandResult::Raycast(Some(RayHit { collider: id, .. })) if id == collider)
        );

        let result = apply_command(
            &mut world,
            PhysicsCommand::Sweep {
                shape: Shape::Sphere(Sphere::new(0.5).unwrap()),
                transform: Transform::identity(),
                displacement: Vec3::new(5.0, 0.0, 0.0),
                filter,
            },
        )
        .unwrap();
        assert!(
            matches!(result, PhysicsCommandResult::Sweep(Some(ShapeHit { collider: id, .. })) if id == collider)
        );

        let result = apply_command(
            &mut world,
            PhysicsCommand::Overlaps {
                shape: Shape::Sphere(Sphere::new(1.0).unwrap()),
                transform: Transform::new(Vec3::new(3.0, 0.0, 0.0), kapitsa_math::Quat::IDENTITY)
                    .unwrap(),
                filter,
            },
        )
        .unwrap();
        assert_eq!(
            result,
            PhysicsCommandResult::Overlaps(vec![OverlapHit { body, collider }])
        );
    }

    #[test]
    fn invalid_ray_is_rejected_before_it_can_be_submitted() {
        assert_eq!(
            Ray::new(Vec3::ZERO, Vec3::ZERO, 1.0),
            Err(kapitsa_core::Error::from(QueryError::InvalidInput))
        );
    }

    #[test]
    fn indexed_batch_keeps_order_for_repeated_body_and_source_ids() {
        const UNIQUE_IDS: usize = SMALL_STAGING_LINEAR_SEARCH_LIMIT + 1;

        let mut world = PhysicsWorld::new();
        let bodies: Vec<_> = (0..UNIQUE_IDS)
            .map(|_| world.add_body(dynamic_body()).expect("body"))
            .collect();
        let sources: Vec<_> = (0..UNIQUE_IDS)
            .map(|_| {
                world
                    .add_force_source(bodies[0], Vec3::ZERO, Vec3::X)
                    .expect("force source")
            })
            .collect();
        let mut commands = Vec::with_capacity(UNIQUE_IDS * 2 + 3);
        for (index, id) in bodies.iter().copied().enumerate() {
            commands.push(PhysicsCommand::SetLinearVelocity {
                id,
                velocity: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
            });
        }
        for (index, id) in sources.iter().copied().enumerate() {
            commands.push(PhysicsCommand::SetForceSource {
                id,
                local_point: Vec3::ZERO,
                local_force: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
            });
        }
        commands.push(PhysicsCommand::SetLinearVelocity {
            id: bodies[0],
            velocity: -Vec3::X,
        });
        commands.push(PhysicsCommand::SetForceSource {
            id: sources[0],
            local_point: Vec3::Y,
            local_force: Vec3::Z,
        });
        commands.push(PhysicsCommand::SetForceSourceEnabled {
            id: sources[0],
            enabled: false,
        });

        apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect("indexed batch applies");

        assert_eq!(
            world
                .body(bodies[0])
                .expect("first body")
                .state()
                .linear_velocity(),
            -Vec3::X
        );
        assert_eq!(
            world
                .body(bodies[1])
                .expect("second body")
                .state()
                .linear_velocity(),
            Vec3::new(2.0, 0.0, 0.0)
        );
        let first_source = world.force_source(sources[0]).expect("first source");
        assert_eq!(first_source.local_point(), Vec3::Y);
        assert_eq!(first_source.local_force(), Vec3::Z);
        assert!(!first_source.is_enabled());
        assert_eq!(
            world
                .force_source(sources[1])
                .expect("second source")
                .local_force(),
            Vec3::new(2.0, 0.0, 0.0)
        );
    }

    // Authored for the runtime API stage; not executed at the user's request.
    #[test]
    fn part_aerodynamic_metadata_rejects_atomic_motion_batches_without_mutation() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let part = world.enable_cluster(body).expect("cluster");
        for metadata in [
            PhysicsCommand::SetPartAerodynamicWings {
                part,
                wings: Vec::new(),
            },
            PhysicsCommand::SetPartAerodynamicHull {
                part,
                sections: Vec::new(),
            },
        ] {
            let before = world.body(body).expect("body").state().linear_velocity();
            let commands = [
                PhysicsCommand::SetLinearVelocity {
                    id: body,
                    velocity: Vec3::X * 42.0,
                },
                metadata,
            ];
            let error = apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
                .expect_err("metadata cannot join atomic motion batch");
            assert_eq!(error.command_index(), Some(1));
            assert!(matches!(
                error.downcast_ref::<CommandError>(),
                Some(CommandError::UnsupportedBatchCommand)
            ));
            assert_eq!(
                world.body(body).expect("body").state().linear_velocity(),
                before
            );
            assert!(
                world
                    .part_aerodynamic_wings(part)
                    .expect("wings")
                    .is_empty()
            );
            assert!(world.part_aerodynamic_hull(part).expect("hull").is_empty());
        }
    }

    #[test]
    fn body_batch_preserves_order_and_applies_impulse_with_updated_mass() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let commands = [
            PhysicsCommand::SetMass {
                id: body,
                mass: mass(2.0),
            },
            PhysicsCommand::ApplyImpulse {
                id: body,
                impulse: Vec3::new(10.0, 0.0, 0.0),
            },
        ];

        apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect("atomic batch");

        assert_eq!(
            world.body(body).expect("body").state().linear_velocity(),
            Vec3::new(5.0, 0.0, 0.0)
        );
    }

    #[test]
    fn failed_body_batch_does_not_commit_earlier_body_changes() {
        let mut world = PhysicsWorld::new();
        let dynamic = world.add_body(dynamic_body()).expect("dynamic body");
        let static_body = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("static body");
        let commands = [
            PhysicsCommand::SetLinearVelocity {
                id: dynamic,
                velocity: Vec3::X,
            },
            PhysicsCommand::SetMass {
                id: static_body,
                mass: mass(2.0),
            },
        ];

        let mut scratch = BatchScratch::default();
        let error = apply_command_batch(&mut world, &commands, &mut scratch)
            .expect_err("static mass rejected");

        assert_eq!(error.command_index(), Some(1));
        assert!(matches!(
            error.downcast_ref::<CommandError>(),
            Some(CommandError::Body(BodyError::BodyMustBeDynamic))
        ));
        assert_eq!(
            world
                .body(dynamic)
                .expect("dynamic body")
                .state()
                .linear_velocity(),
            Vec3::ZERO
        );
        assert!(scratch.is_empty());
        let retained = scratch.capacities();
        let retained_body_capacity = retained[0];
        let retained_index_capacity = retained[1];

        apply_command_batch(
            &mut world,
            &[PhysicsCommand::SetLinearVelocity {
                id: dynamic,
                velocity: Vec3::Y,
            }],
            &mut scratch,
        )
        .expect("scratch can be reused after a failed batch");
        assert_eq!(scratch.capacities()[0], retained_body_capacity);
        assert_eq!(scratch.capacities()[1], retained_index_capacity);
        assert_eq!(
            world
                .body(dynamic)
                .expect("updated body")
                .state()
                .linear_velocity(),
            Vec3::Y
        );
    }

    #[test]
    fn force_source_updates_share_atomic_body_batch_commit() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let source = world
            .add_force_source(body, Vec3::ZERO, Vec3::X)
            .expect("force source");
        let commands = [
            PhysicsCommand::SetForceSource {
                id: source,
                local_point: Vec3::Y,
                local_force: Vec3::Z,
            },
            PhysicsCommand::SetForceSourceEnabled {
                id: source,
                enabled: false,
            },
            PhysicsCommand::SetLinearVelocity {
                id: body,
                velocity: Vec3::X,
            },
        ];

        apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect("atomic batch");

        let source = world.force_source(source).expect("updated source");
        assert_eq!(source.local_point(), Vec3::Y);
        assert_eq!(source.local_force(), Vec3::Z);
        assert!(!source.is_enabled());
        assert_eq!(
            world.body(body).expect("body").state().linear_velocity(),
            Vec3::X
        );
    }

    #[test]
    fn joint_batch_updates_enablement_and_removal_in_fifo_order() {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("body A");
        let body_b = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("body B");
        let id = world
            .add_joint(
                body_a,
                body_b,
                Joint::Fixed(FixedJoint::new(JointCommon::new(
                    JointFrame::IDENTITY,
                    JointFrame::IDENTITY,
                ))),
            )
            .expect("joint");
        let mut replacement = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
        replacement.break_force = Some(25.0);
        let commands = [
            PhysicsCommand::SetJointEnabled { id, enabled: false },
            PhysicsCommand::UpdateJoint {
                id,
                joint: Joint::Fixed(FixedJoint::new(replacement)),
            },
            PhysicsCommand::SetJointEnabled { id, enabled: false },
        ];

        apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect("atomic joint batch");

        let joint = world.joint(id).expect("live joint");
        assert!(!joint.common().enabled);
        assert_eq!(joint.common().break_force, Some(25.0));

        let remove = [PhysicsCommand::RemoveJoint { id }];
        apply_command_batch(&mut world, &remove, &mut BatchScratch::default())
            .expect("joint removal batch");
        assert_eq!(world.joint_count(), 0);
        assert!(world.remove_joint(id).is_none());
    }

    #[test]
    fn joint_batch_preserves_fifo_order_for_dense_pool_removals() {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("body A");
        let body_b = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("body B");
        let make_joint = || {
            Joint::Fixed(FixedJoint::new(JointCommon::new(
                JointFrame::IDENTITY,
                JointFrame::IDENTITY,
            )))
        };
        let ids = (0..5)
            .map(|_| {
                world
                    .add_joint(body_a, body_b, make_joint())
                    .expect("joint")
            })
            .collect::<Vec<_>>();
        let commands = [
            PhysicsCommand::SetJointEnabled {
                id: ids[0],
                enabled: false,
            },
            PhysicsCommand::SetJointEnabled {
                id: ids[2],
                enabled: false,
            },
            PhysicsCommand::RemoveJoint { id: ids[2] },
            PhysicsCommand::RemoveJoint { id: ids[0] },
        ];

        apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect("atomic FIFO batch");

        let remaining = world.joints().map(|(id, _, _, _)| id).collect::<Vec<_>>();
        assert_eq!(remaining, [ids[3], ids[1], ids[4]]);
    }

    #[test]
    fn failed_joint_batch_leaves_joint_and_body_unchanged() {
        let mut world = PhysicsWorld::new();
        let dynamic = world.add_body(dynamic_body()).expect("dynamic body");
        let fixed = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("static body");
        let id = world
            .add_joint(
                dynamic,
                fixed,
                Joint::Fixed(FixedJoint::new(JointCommon::new(
                    JointFrame::IDENTITY,
                    JointFrame::IDENTITY,
                ))),
            )
            .expect("joint");
        let commands = [
            PhysicsCommand::SetJointEnabled { id, enabled: false },
            PhysicsCommand::SetLinearVelocity {
                id: fixed,
                velocity: Vec3::X,
            },
        ];

        let error = apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect_err("static velocity mutation is invalid");

        assert_eq!(error.command_index(), Some(1));
        assert!(world.joint(id).expect("unchanged joint").common().enabled);
        assert_eq!(
            world
                .body(dynamic)
                .expect("dynamic body")
                .state()
                .linear_velocity(),
            Vec3::ZERO
        );
    }

    #[test]
    fn single_joint_command_returns_its_id_and_stale_joint_ids_are_rejected() {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("body A");
        let body_b = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("body B");
        let joint = Joint::Fixed(FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        )));
        let PhysicsCommandResult::Joint(id) = apply_command(
            &mut world,
            PhysicsCommand::AddJoint {
                body_a,
                body_b,
                joint,
            },
        )
        .expect("joint add") else {
            panic!("add joint returns its ID");
        };

        world.remove_joint(id).expect("joint remove");
        let result = apply_command(
            &mut world,
            PhysicsCommand::SetJointEnabled { id, enabled: true },
        );
        assert_eq!(
            result,
            Err(PhysicsClientError::World(WorldError::InvalidJointId))
        );
    }

    #[test]
    fn failed_force_source_batch_does_not_commit_source_or_body_changes() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let source = world
            .add_force_source(body, Vec3::ZERO, Vec3::X)
            .expect("force source");
        let mut other_world = PhysicsWorld::new();
        let other_body = other_world.add_body(dynamic_body()).expect("other body");
        let foreign_source = other_world
            .add_force_source(other_body, Vec3::ZERO, Vec3::Y)
            .expect("foreign source");
        let commands = [
            PhysicsCommand::SetForceSource {
                id: source,
                local_point: Vec3::Y,
                local_force: Vec3::Z,
            },
            PhysicsCommand::SetLinearVelocity {
                id: body,
                velocity: Vec3::X,
            },
            PhysicsCommand::SetForceSourceEnabled {
                id: foreign_source,
                enabled: false,
            },
        ];

        let mut scratch = BatchScratch::default();
        let error =
            apply_command_batch(&mut world, &commands, &mut scratch).expect_err("foreign source");

        assert_eq!(error.command_index(), Some(2));
        assert!(matches!(
            error.downcast_ref::<CommandError>(),
            Some(CommandError::ForceSource(
                ForceSourceError::InvalidForceSourceId
            ))
        ));
        let source = world.force_source(source).expect("unchanged source");
        assert_eq!(source.local_point(), Vec3::ZERO);
        assert_eq!(source.local_force(), Vec3::X);
        assert!(source.is_enabled());
        assert_eq!(
            world.body(body).expect("body").state().linear_velocity(),
            Vec3::ZERO
        );
        assert!(scratch.is_empty());
    }

    #[test]
    #[ignore = "Release benchmark; run with the command batch scaling filter"]
    fn benchmark_apply_command_batch_scaling() {
        const COMMAND_COUNTS: [usize; 7] = [128, 256, 512, 1_024, 2_048, 4_096, 8_192];
        const TARGET_SAMPLE_TIME: Duration = Duration::from_millis(5);

        println!(
            "scenario,commands,repeats,min_us_per_batch,median_us_per_batch,max_us_per_batch,median_ns_per_command,one_shot_buffer_control_us,one_shot_control_share_pct"
        );
        for scenario in BatchScenario::ALL {
            for command_count in COMMAND_COUNTS {
                let repeats =
                    calibrated_repeats(scenario, command_count, TARGET_SAMPLE_TIME, apply_batch);
                let (min_ns, median_ns, max_ns) = run_samples(
                    scenario,
                    command_count,
                    repeats,
                    apply_batch,
                    verify_batch_result,
                );
                let temp_buffer_repeats =
                    calibrated_repeats(scenario, command_count, TARGET_SAMPLE_TIME, |case| {
                        staged_buffer_control(case)
                    });
                let (_, temp_median_ns, _) = run_samples(
                    scenario,
                    command_count,
                    temp_buffer_repeats,
                    |case| staged_buffer_control(case),
                    |_| {},
                );

                println!(
                    "{},{command_count},{repeats},{:.3},{:.3},{:.3},{:.2},{:.3},{:.1}",
                    scenario.name(),
                    min_ns / 1_000.0,
                    median_ns / 1_000.0,
                    max_ns / 1_000.0,
                    median_ns / command_count as f64,
                    temp_median_ns / 1_000.0,
                    temp_median_ns * 100.0 / median_ns,
                );
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "allocation counts; run separately with bench-internals"]
    fn benchmark_apply_command_batch_allocations() {
        const COMMAND_COUNTS: [usize; 7] = [128, 256, 512, 1_024, 2_048, 4_096, 8_192];

        println!(
            "scenario,commands,warm_allocations,warm_requested_bytes,body_vec_capacity,body_index_capacity,source_vec_capacity,source_index_capacity"
        );
        for scenario in BatchScenario::ALL {
            for command_count in COMMAND_COUNTS {
                let mut case = batch_case(scenario, command_count);
                apply_batch(&mut case);
                let allocations =
                    tracked_batch_allocations(&mut case.world, &case.commands, &mut case.scratch);
                verify_batch_result(&case);
                println!(
                    "{},{command_count},{},{},{},{},{},{}",
                    scenario.name(),
                    allocations.count,
                    allocations.bytes,
                    case.scratch.capacities()[0],
                    case.scratch.capacities()[1],
                    case.scratch.capacities()[2],
                    case.scratch.capacities()[3],
                );
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Release full-cycle timing and allocations; run separately with bench-internals"]
    fn benchmark_command_buffer_pool_full_cycle() {
        use super::super::{CommandBufferPool, PhysicsClient, PhysicsSnapshot};
        use std::sync::atomic::AtomicU64;

        const WARMUP_CYCLES: usize = 500;
        const CYCLES_PER_SAMPLE: usize = 10_000;
        const SAMPLE_COUNT: usize = 5;

        let (command_tx, receiver) = mpsc::channel(1);
        let (_state_tx, state_rx) = watch::channel(Some(command_tx));
        let (collision_events, _) = broadcast::channel(4);
        let (joint_break_events, _) = broadcast::channel(4);
        let (batch_results, _) = broadcast::channel(4);
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body ID");
        let (snapshot_tx, snapshot_rx) = watch::channel(Arc::new(PhysicsSnapshot::from_world(
            &world,
            0,
            Duration::ZERO,
        )));
        let pool = Arc::new(CommandBufferPool::default());
        let client = PhysicsClient::new(
            state_rx,
            snapshot_rx,
            collision_events,
            joint_break_events,
            batch_results,
            Arc::new(AtomicU64::new(1)),
            Arc::clone(&pool),
        );
        drop(snapshot_tx);
        let mut harness = FullCycleHarness {
            client,
            receiver,
            pool,
            world,
            body,
            scratch: BatchScratch::default(),
        };

        println!(
            "mode,commands_per_batch,samples,min_us_per_cycle,median_us_per_cycle,max_us_per_cycle,try_submit_ns_per_call,batches_per_second,commands_per_second,allocations_per_cycle,requested_bytes_per_cycle,retained_buffers,retained_pool_bytes"
        );
        for use_pool in [false, true] {
            run_full_cycles(&mut harness, use_pool, WARMUP_CYCLES);

            let mut cycle_ns = Vec::with_capacity(SAMPLE_COUNT);
            let mut submit_ns = Vec::with_capacity(SAMPLE_COUNT);
            for _ in 0..SAMPLE_COUNT {
                let measurement = run_full_cycles(&mut harness, use_pool, CYCLES_PER_SAMPLE);
                cycle_ns.push(measurement.elapsed.as_nanos() as f64 / CYCLES_PER_SAMPLE as f64);
                submit_ns
                    .push(measurement.submit_elapsed.as_nanos() as f64 / CYCLES_PER_SAMPLE as f64);
            }
            let allocations =
                tracked_full_cycle_allocations(&mut harness, use_pool, CYCLES_PER_SAMPLE);
            cycle_ns.sort_by(f64::total_cmp);
            submit_ns.sort_by(f64::total_cmp);
            let median_cycle_ns = cycle_ns[SAMPLE_COUNT / 2];
            let (retained_buffers, retained_command_bytes) = harness.pool.stats();
            let retained_bytes = if use_pool {
                retained_command_bytes + CommandBufferPool::fixed_memory_bytes()
            } else {
                0
            };
            let mode = if use_pool { "pooled" } else { "new-and-drop" };
            println!(
                "{mode},128,{SAMPLE_COUNT},{:.3},{:.3},{:.3},{:.1},{:.0},{:.0},{:.3},{:.1},{retained_buffers},{retained_bytes}",
                cycle_ns[0] / 1_000.0,
                median_cycle_ns / 1_000.0,
                cycle_ns[SAMPLE_COUNT - 1] / 1_000.0,
                submit_ns[SAMPLE_COUNT / 2],
                1.0e9 / median_cycle_ns,
                128.0e9 / median_cycle_ns,
                allocations.count as f64 / CYCLES_PER_SAMPLE as f64,
                allocations.bytes as f64 / CYCLES_PER_SAMPLE as f64,
            );
        }
    }

    #[cfg(feature = "bench-internals")]
    struct FullCycleMeasurement {
        elapsed: Duration,
        submit_elapsed: Duration,
    }

    #[cfg(feature = "bench-internals")]
    struct FullCycleHarness {
        client: super::super::PhysicsClient,
        receiver: tokio::sync::mpsc::Receiver<super::super::CommandRequest>,
        pool: Arc<super::super::CommandBufferPool>,
        world: PhysicsWorld,
        body: kapitsa_core::world::BodyId,
        scratch: BatchScratch,
    }

    #[cfg(feature = "bench-internals")]
    fn run_full_cycles(
        harness: &mut FullCycleHarness,
        use_pool: bool,
        cycles: usize,
    ) -> FullCycleMeasurement {
        const COMMANDS_PER_BATCH: usize = 128;

        let started = Instant::now();
        let mut submit_elapsed = Duration::ZERO;
        for cycle in 0..cycles {
            let mut buffer = if use_pool {
                harness.client.commands()
            } else {
                super::super::PhysicsCommandBuffer::new()
            };
            for command_index in 0..COMMANDS_PER_BATCH {
                let value = (cycle * COMMANDS_PER_BATCH + command_index) as f64;
                buffer.set_linear_velocity(harness.body, Vec3::new(value, 1.0, 0.0));
            }

            let submit_started = Instant::now();
            let batch_id = harness
                .client
                .try_submit(black_box(buffer))
                .expect("submit batch");
            submit_elapsed += submit_started.elapsed();
            let Some(super::super::CommandRequest::Batch {
                id: queued_id,
                buffer,
            }) = harness.receiver.try_recv().ok()
            else {
                panic!("submitted batch must be queued");
            };
            assert_eq!(queued_id, batch_id);
            apply_command_batch(
                &mut harness.world,
                black_box(buffer.commands()),
                &mut harness.scratch,
            )
            .expect("batch applies");
            let expected = Vec3::new(((cycle + 1) * COMMANDS_PER_BATCH - 1) as f64, 1.0, 0.0);
            assert_eq!(
                harness
                    .world
                    .body(harness.body)
                    .expect("updated body")
                    .state()
                    .linear_velocity(),
                expected
            );
            black_box(expected);

            if use_pool {
                harness.pool.recycle(buffer);
            }
        }
        FullCycleMeasurement {
            elapsed: started.elapsed(),
            submit_elapsed,
        }
    }

    #[cfg(feature = "bench-internals")]
    fn tracked_full_cycle_allocations(
        harness: &mut FullCycleHarness,
        use_pool: bool,
        cycles: usize,
    ) -> BatchAllocationStats {
        let tracking = AllocationTracking::start();
        let _ = run_full_cycles(harness, use_pool, cycles);
        drop(tracking);
        BatchAllocationStats {
            count: ALLOCATION_COUNT.load(Ordering::Relaxed),
            bytes: ALLOCATION_BYTES.load(Ordering::Relaxed),
        }
    }
}
