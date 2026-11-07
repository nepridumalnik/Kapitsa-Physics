use std::{
    panic::{AssertUnwindSafe, catch_unwind},
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
};

use kapitsa_core::execution::{ExecutionError, StepExecutor};
use rayon::{ThreadPool, ThreadPoolBuilder};
use tokio::runtime::Handle;
#[cfg(test)]
use tokio::sync::mpsc as async_mpsc;

/// Owns a Rayon pool whose long-lived workers run on a host Tokio blocking pool.
pub struct RayonPool {
    pool: Option<Arc<ThreadPool>>,
    #[cfg(test)]
    worker_exits: async_mpsc::UnboundedReceiver<usize>,
    #[cfg(test)]
    worker_count: usize,
}

/// Cloneable, statically dispatched access to a `RayonPool`.
///
/// One root computation and its nested `join`/chunk calls share the active
/// worker budget. Clones share that budget too. Change it only between completed
/// root computations; independent simulations should obtain separate executors.
/// `spawn` submits a root task and does not itself reserve a compute permit.
#[derive(Clone)]
pub struct RayonExecutor {
    pool: Arc<ThreadPool>,
    spare_workers: Arc<AtomicUsize>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RayonPoolError {
    InvalidWorkerCount,
    BuildFailed,
    RuntimeUnavailable,
    #[cfg(test)]
    ExecutorsStillActive,
    #[cfg(test)]
    WorkerExitedEarly,
    PoolAlreadyShutDown,
}

impl std::fmt::Display for RayonPoolError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::InvalidWorkerCount => "Rayon worker count must be positive",
            Self::BuildFailed => "Rayon worker pool could not be created",
            Self::RuntimeUnavailable => "the host Tokio runtime cannot start Rayon workers",
            #[cfg(test)]
            Self::ExecutorsStillActive => "Rayon executor handles are still active",
            #[cfg(test)]
            Self::WorkerExitedEarly => "a Rayon worker exited before shutdown completed",
            Self::PoolAlreadyShutDown => "Rayon worker pool is already shut down",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for RayonPoolError {}

impl RayonPool {
    /// Creates a fixed-size pool using the supplied host runtime for worker threads.
    pub fn new(handle: &Handle, worker_count: usize) -> Result<Self, RayonPoolError> {
        if worker_count == 0 {
            return Err(RayonPoolError::InvalidWorkerCount);
        }

        #[cfg(test)]
        let (worker_exit_tx, worker_exits) = async_mpsc::unbounded_channel();
        let handle = handle.clone();
        let pool = catch_unwind(AssertUnwindSafe(|| {
            ThreadPoolBuilder::new()
                .num_threads(worker_count)
                .thread_name(|index| format!("kapitsa-rayon-{index}"))
                .spawn_handler(move |thread| {
                    #[cfg(test)]
                    let index = thread.index();
                    #[cfg(test)]
                    let worker_exit_tx = worker_exit_tx.clone();
                    handle.spawn_blocking(move || {
                        #[cfg(test)]
                        let _exit_signal = WorkerExitSignal {
                            index,
                            sender: worker_exit_tx,
                        };
                        thread.run();
                    });
                    Ok(())
                })
                .build()
        }))
        .map_err(|_| RayonPoolError::RuntimeUnavailable)?
        .map_err(|_| RayonPoolError::BuildFailed)?;

        Ok(Self {
            pool: Some(Arc::new(pool)),
            #[cfg(test)]
            worker_exits,
            #[cfg(test)]
            worker_count,
        })
    }

    /// Returns a cloneable handle for scoped step work.
    pub fn executor(&self) -> Result<RayonExecutor, RayonPoolError> {
        self.pool
            .as_ref()
            .map(|pool| RayonExecutor {
                pool: Arc::clone(pool),
                spare_workers: Arc::new(AtomicUsize::new(pool.current_num_threads() - 1)),
            })
            .ok_or(RayonPoolError::PoolAlreadyShutDown)
    }

    #[cfg(test)]
    pub fn spawn<F>(&self, operation: F) -> Result<(), RayonPoolError>
    where
        F: FnOnce() + Send + 'static,
    {
        self.executor()?.spawn(operation)
    }

    /// Stops the pool and asynchronously waits for each Tokio blocking worker to exit.
    #[cfg(test)]
    pub async fn shutdown(&mut self) -> Result<(), RayonPoolError> {
        let Some(pool) = self.pool.as_ref() else {
            return Err(RayonPoolError::PoolAlreadyShutDown);
        };
        if Arc::strong_count(pool) != 1 {
            return Err(RayonPoolError::ExecutorsStillActive);
        }

        let pool = self
            .pool
            .take()
            .ok_or(RayonPoolError::PoolAlreadyShutDown)?;
        let pool = Arc::try_unwrap(pool).map_err(|_| RayonPoolError::ExecutorsStillActive)?;
        drop(pool);

        for _ in 0..self.worker_count {
            self.worker_exits
                .recv()
                .await
                .ok_or(RayonPoolError::WorkerExitedEarly)?;
        }
        Ok(())
    }
}

impl StepExecutor for RayonExecutor {
    fn set_parallelism(&mut self, workers: usize) {
        self.spare_workers
            .store(workers.clamp(1, self.worker_count()) - 1, Ordering::Relaxed);
    }

    fn join<A, B, RA, RB>(&self, left: A, right: B) -> (RA, RB)
    where
        A: FnOnce() -> RA + Send,
        B: FnOnce() -> RB + Send,
        RA: Send,
        RB: Send,
    {
        if self
            .spare_workers
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |available| {
                available.checked_sub(1)
            })
            .is_err()
        {
            return (left(), right());
        }
        // Never wait for a permit: nested work runs locally when the budget is
        // exhausted. Keep this permit until BOTH branches finish, also on panic.
        let _permit = WorkerPermit(&self.spare_workers);
        self.pool.install(|| rayon::join(left, right))
    }

    fn for_each_chunk_mut<T, F>(
        &self,
        values: &mut [T],
        chunk_size: usize,
        operation: F,
    ) -> Result<(), ExecutionError>
    where
        T: Send,
        F: Fn(usize, &mut [T]) + Send + Sync,
    {
        if chunk_size == 0 {
            return Err(ExecutionError::InvalidChunkLayout);
        }
        self.for_chunks(values, chunk_size, 0, &operation);
        Ok(())
    }

    fn for_each_chunk_layout_mut<T, F>(
        &self,
        values: &mut [T],
        chunk_sizes: &[usize],
        operation: F,
    ) -> Result<(), ExecutionError>
    where
        T: Send,
        F: Fn(usize, &mut [T]) + Send + Sync,
    {
        validate_chunk_layout(values.len(), chunk_sizes)?;
        self.for_layout(values, chunk_sizes, 0, &operation);
        Ok(())
    }
}

impl RayonExecutor {
    /// Creates an independent worker budget over an existing Rayon pool.
    pub fn from_pool(pool: Arc<ThreadPool>) -> Self {
        let spare = pool.current_num_threads().saturating_sub(1);
        Self {
            pool,
            spare_workers: Arc::new(AtomicUsize::new(spare)),
        }
    }
    fn for_chunks<T: Send, F: Fn(usize, &mut [T]) + Send + Sync>(
        &self,
        values: &mut [T],
        size: usize,
        start: usize,
        operation: &F,
    ) {
        let count = values.len().div_ceil(size);
        if count <= 1 || self.spare_workers.load(Ordering::Relaxed) == 0 {
            for (index, chunk) in values.chunks_mut(size).enumerate() {
                operation(start + index, chunk);
            }
            return;
        }
        let middle = count / 2;
        let (left, right) = values.split_at_mut(middle * size);
        self.join(
            || self.for_chunks(left, size, start, operation),
            || self.for_chunks(right, size, start + middle, operation),
        );
    }

    fn for_layout<T: Send, F: Fn(usize, &mut [T]) + Send + Sync>(
        &self,
        mut values: &mut [T],
        sizes: &[usize],
        mut start: usize,
        operation: &F,
    ) {
        if sizes.len() <= 1 || self.spare_workers.load(Ordering::Relaxed) == 0 {
            for &size in sizes {
                let (chunk, tail) = values.split_at_mut(size);
                operation(start, chunk);
                start += size;
                values = tail;
            }
            return;
        }
        let middle = sizes.len() / 2;
        let offset = sizes[..middle].iter().sum();
        let (left, right) = values.split_at_mut(offset);
        self.join(
            || self.for_layout(left, &sizes[..middle], start, operation),
            || self.for_layout(right, &sizes[middle..], start + offset, operation),
        );
    }

    /// Schedules a top-level task on the shared pool.
    pub fn spawn<F>(&self, operation: F) -> Result<(), RayonPoolError>
    where
        F: FnOnce() + Send + 'static,
    {
        self.pool.spawn(operation);
        Ok(())
    }

    /// Returns the fixed number of worker threads in this pool.
    pub fn worker_count(&self) -> usize {
        self.pool.current_num_threads()
    }
}

struct WorkerPermit<'a>(&'a AtomicUsize);

impl Drop for WorkerPermit<'_> {
    fn drop(&mut self) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
struct WorkerExitSignal {
    index: usize,
    sender: async_mpsc::UnboundedSender<usize>,
}

#[cfg(test)]
impl Drop for WorkerExitSignal {
    fn drop(&mut self) {
        let _ = self.sender.send(self.index);
    }
}

fn validate_chunk_layout(length: usize, chunk_sizes: &[usize]) -> Result<(), ExecutionError> {
    let total = chunk_sizes.iter().try_fold(0_usize, |total, &size| {
        if size == 0 {
            return None;
        }
        total.checked_add(size)
    });
    if total == Some(length) {
        Ok(())
    } else {
        Err(ExecutionError::InvalidChunkLayout)
    }
}

#[cfg(test)]
mod tests {
    use std::{
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
        time::Duration,
    };

    use tokio::{runtime::Builder, sync::oneshot, time::timeout};

    use super::RayonPool;
    use kapitsa_core::execution::StepExecutor;

    #[test]
    fn nested_tasks_share_worker_limit_and_return_borrowed_results() {
        fn tree(
            executor: &super::RayonExecutor,
            depth: usize,
            active: &AtomicUsize,
            peak: &AtomicUsize,
        ) -> usize {
            if depth == 0 {
                let running = active.fetch_add(1, Ordering::SeqCst) + 1;
                peak.fetch_max(running, Ordering::SeqCst);
                std::thread::sleep(Duration::from_millis(1));
                active.fetch_sub(1, Ordering::SeqCst);
                return 1;
            }
            let (left, right) = executor.join(
                || tree(executor, depth - 1, active, peak),
                || tree(executor, depth - 1, active, peak),
            );
            left + right
        }
        let runtime = Builder::new_multi_thread()
            .worker_threads(1)
            .build()
            .unwrap();
        let mut pool = RayonPool::new(runtime.handle(), 4).unwrap();
        let mut executor = pool.executor().unwrap();
        for workers in [1, 2, 3, 4] {
            executor.set_parallelism(workers);
            let active = AtomicUsize::new(0);
            let peak = AtomicUsize::new(0);
            assert_eq!(tree(&executor, 6, &active, &peak), 64);
            assert!(peak.load(Ordering::SeqCst) <= workers);
            assert_eq!(active.load(Ordering::SeqCst), 0);
            assert_eq!(executor.spare_workers.load(Ordering::Relaxed), workers - 1);
            let mut values = [0; 23];
            executor
                .for_each_chunk_layout_mut(&mut values, &[2, 5, 1, 15], |start, chunk| {
                    executor
                        .for_each_chunk_mut(chunk, 1, |index, value| value[0] = start + index)
                        .unwrap();
                })
                .unwrap();
            assert_eq!(values, std::array::from_fn::<_, 23, _>(|i| i));
        }
        drop(executor);
        runtime.block_on(pool.shutdown()).unwrap();
    }

    #[test]
    fn independent_branches_overlap_and_panics_release_permits() {
        let runtime = Builder::new_multi_thread()
            .worker_threads(1)
            .build()
            .unwrap();
        let mut pool = RayonPool::new(runtime.handle(), 2).unwrap();
        let executor = pool.executor().unwrap();
        let barrier = std::sync::Barrier::new(2);
        let values = [3, 7];
        let result = executor.join(
            || {
                barrier.wait();
                &values[0]
            },
            || {
                barrier.wait();
                &values[1]
            },
        );
        assert_eq!(result, (&3, &7));
        let completed = AtomicUsize::new(0);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            executor.join(
                || panic!("task failed"),
                || {
                    completed.fetch_add(1, Ordering::Relaxed);
                },
            );
        }));
        assert!(result.is_err());
        assert_eq!(completed.load(Ordering::Relaxed), 1);
        assert_eq!(executor.spare_workers.load(Ordering::Relaxed), 1);
        drop(executor);
        runtime.block_on(pool.shutdown()).unwrap();
    }

    #[test]
    fn rayon_pool_reuses_scoped_workers_and_shuts_down_on_host_runtime() {
        const WORKER_COUNT: usize = 4;
        const BATCH_COUNT: usize = 8;
        const TIMEOUT: Duration = Duration::from_secs(3);

        let runtime = Builder::new_multi_thread()
            .worker_threads(2)
            .max_blocking_threads(1)
            .enable_all()
            .build()
            .expect("host runtime should build");
        let mut pool = RayonPool::new(runtime.handle(), WORKER_COUNT).expect("Rayon pool");
        let starts = Arc::new(AtomicUsize::new(0));
        let (result_tx, result_rx) = oneshot::channel();
        let executor = pool.executor().expect("executor handle");
        let starts_in_job = Arc::clone(&starts);

        pool.spawn(move || {
            starts_in_job.fetch_add(1, Ordering::Relaxed);
            let mut batches = Vec::with_capacity(BATCH_COUNT);
            for batch in 0..BATCH_COUNT {
                let mut values = [0_usize; 64];
                executor
                    .for_each_chunk_mut(&mut values, 8, |chunk_index, chunk| {
                        chunk.fill(chunk_index + batch);
                    })
                    .expect("valid chunks");
                batches.push(values.into_iter().sum::<usize>());
            }
            drop(executor);
            let _ = result_tx.send(batches);
        })
        .expect("Rayon task should be submitted");

        let batches = runtime
            .block_on(async { timeout(TIMEOUT, result_rx).await })
            .expect("nested Rayon work must not starve the one-slot blocking pool")
            .expect("Rayon task should return results");
        assert_eq!(starts.load(Ordering::Relaxed), 1);
        assert_eq!(
            batches,
            (0..BATCH_COUNT)
                .map(|batch| (0..8).map(|chunk| (chunk + batch) * 8).sum())
                .collect::<Vec<usize>>()
        );

        runtime
            .block_on(async { timeout(TIMEOUT, pool.shutdown()).await })
            .expect("pool shutdown should complete before the host runtime stops")
            .expect("all Rayon workers should exit");
        drop(runtime);
    }
}
