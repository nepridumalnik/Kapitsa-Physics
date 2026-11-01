use std::{
    panic::{AssertUnwindSafe, catch_unwind},
    sync::Arc,
};

use kapitsa_core::execution::{ExecutionError, StepExecutor};
use rayon::{ThreadPool, ThreadPoolBuilder};
use tokio::{runtime::Handle, sync::mpsc as async_mpsc};

/// Owns a Rayon pool whose long-lived workers run on a host Tokio blocking pool.
pub struct RayonPool {
    pool: Option<Arc<ThreadPool>>,
    worker_exits: async_mpsc::UnboundedReceiver<usize>,
    worker_count: usize,
}

/// Cloneable, statically dispatched access to a `RayonPool`.
#[derive(Clone)]
pub struct RayonExecutor {
    pool: Arc<ThreadPool>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RayonPoolError {
    InvalidWorkerCount,
    BuildFailed,
    RuntimeUnavailable,
    ExecutorsStillActive,
    WorkerExitedEarly,
    PoolAlreadyShutDown,
}

impl std::fmt::Display for RayonPoolError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::InvalidWorkerCount => "Rayon worker count must be positive",
            Self::BuildFailed => "Rayon worker pool could not be created",
            Self::RuntimeUnavailable => "the host Tokio runtime cannot start Rayon workers",
            Self::ExecutorsStillActive => "Rayon executor handles are still active",
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

        let (worker_exit_tx, worker_exits) = async_mpsc::unbounded_channel();
        let handle = handle.clone();
        let pool = catch_unwind(AssertUnwindSafe(|| {
            ThreadPoolBuilder::new()
                .num_threads(worker_count)
                .thread_name(|index| format!("kapitsa-rayon-{index}"))
                .spawn_handler(move |thread| {
                    let index = thread.index();
                    let worker_exit_tx = worker_exit_tx.clone();
                    handle.spawn_blocking(move || {
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
            worker_exits,
            worker_count,
        })
    }

    /// Returns a cloneable handle for scoped step work.
    pub fn executor(&self) -> Result<RayonExecutor, RayonPoolError> {
        self.pool
            .as_ref()
            .map(|pool| RayonExecutor {
                pool: Arc::clone(pool),
            })
            .ok_or(RayonPoolError::PoolAlreadyShutDown)
    }

    /// Schedules a top-level task on this pool.
    pub fn spawn<F>(&self, operation: F) -> Result<(), RayonPoolError>
    where
        F: FnOnce() + Send + 'static,
    {
        self.executor()?.spawn(operation)
    }

    /// Stops the pool and asynchronously waits for each Tokio blocking worker to exit.
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
        self.pool.scope(|scope| {
            for (index, chunk) in values.chunks_mut(chunk_size).enumerate() {
                let operation = &operation;
                scope.spawn(move |_| operation(index, chunk));
            }
        });
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
        self.pool.scope(|scope| {
            let mut remaining = values;
            let mut start = 0;
            for &chunk_size in chunk_sizes {
                let (chunk, tail) = remaining.split_at_mut(chunk_size);
                let operation = &operation;
                scope.spawn(move |_| operation(start, chunk));
                start += chunk_size;
                remaining = tail;
            }
        });
        Ok(())
    }
}

impl RayonExecutor {
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

struct WorkerExitSignal {
    index: usize,
    sender: async_mpsc::UnboundedSender<usize>,
}

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
