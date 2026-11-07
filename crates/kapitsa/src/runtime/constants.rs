//! Runtime limits and tuning thresholds. Values must preserve documented runtime contracts.

// planner.rs
pub(crate) const EARLY_WARNING_AT: f64 = 0.70;
pub(crate) const MIN_SCALE_UP_STEPS: u32 = 3;
pub(crate) const MIN_SCALE_DOWN_STEPS: u32 = 24;
pub(crate) const STABILIZATION_STEPS: u32 = 12;
pub(crate) const WORKER_PROBE_STEPS: usize = 12;
pub(crate) const REQUIRED_SPEEDUP: f64 = 0.05;
pub(crate) const SIGNIFICANT_LOAD_INCREASE: f64 = 1.25;
pub(crate) const SCALE_UP_HYSTERESIS: f64 = 0.10;
pub(crate) const SCALE_DOWN_HYSTERESIS: f64 = 1.15;

// config.rs
pub(crate) const MAX_TICK_RATE: u32 = 1_000_000_000;
pub(crate) const MIN_LOAD_SPEED: f64 = 0.30;
pub(crate) const DEFAULT_TICK_RATE: u32 = 60;
pub(crate) const DEFAULT_MAX_CATCH_UP_STEPS: usize = 4;
pub(crate) const DEFAULT_MIN_BATCH_SIZE: usize = 128;
pub(crate) const DEFAULT_MAX_BATCHES_PER_WORKER: usize = 4;
pub(crate) const DEFAULT_SCALE_UP_AT: f64 = 0.70;
pub(crate) const DEFAULT_SCALE_DOWN_AT: f64 = 0.30;
pub(crate) const DEFAULT_SMOOTHING: f64 = 0.20;

// engine.rs
pub(crate) const METRICS_WINDOW: usize = 120;

// asynchronous/service.rs
pub(crate) const COMMAND_QUEUE_CAPACITY: usize = 1024;
pub(crate) const MAX_COMMANDS_PER_STEP: usize = 1024;
pub(crate) const COLLISION_EVENT_BATCH_CAPACITY: usize = 64;
pub(crate) const JOINT_BREAK_EVENT_BATCH_CAPACITY: usize = 64;
pub(crate) const PHYSICS_BATCH_RESULT_CAPACITY: usize = 256;

// asynchronous/marshalling/snapshot.rs
pub(crate) const DENSE_ENTER_RATIO: usize = 3;
pub(crate) const DENSE_EXIT_RATIO: usize = 6;
pub(crate) const CAPACITY_SHRINK_RATIO: usize = 4;
pub(crate) const MIN_RETAINED_CAPACITY: usize = 256;

// asynchronous/marshalling/buffer_pool.rs
pub(crate) const MAX_POOLED_BUFFERS: usize = 16;
pub(crate) const MAX_POOL_MEMORY_BYTES: usize = 256 * 1024;

// asynchronous/marshalling/command.rs
pub(crate) const SMALL_STAGING_LINEAR_SEARCH_LIMIT: usize = 64;
pub(crate) const MAX_COMMANDS_PER_BATCH: usize = 128;

// A queued batch must fit within a single service turn.
const _: () = assert!(MAX_COMMANDS_PER_BATCH <= MAX_COMMANDS_PER_STEP);
