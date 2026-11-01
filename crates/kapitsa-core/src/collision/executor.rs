use std::sync::OnceLock;

use super::CollisionError;

static COLLISION_WORKERS: OnceLock<Result<rayon::ThreadPool, ()>> = OnceLock::new();

pub(crate) fn collision_worker_pool() -> Result<&'static rayon::ThreadPool, CollisionError> {
    COLLISION_WORKERS
        .get_or_init(|| {
            let worker_count = std::thread::available_parallelism()
                .map(std::num::NonZeroUsize::get)
                .unwrap_or(1);
            rayon::ThreadPoolBuilder::new()
                .num_threads(worker_count)
                .thread_name(|index| format!("kapitsa-collision-{index}"))
                .build()
                .map_err(|_| ())
        })
        .as_ref()
        .map_err(|_| CollisionError::WorkerFailed)
}
