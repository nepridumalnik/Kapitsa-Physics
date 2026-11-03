use kapitsa_core::execution::{ExecutionError, StepExecutor};
use rayon::prelude::*;

pub(crate) struct RayonExecutor {
    pool: rayon::ThreadPool,
}

impl RayonExecutor {
    pub(crate) fn new(workers: usize) -> Result<Self, rayon::ThreadPoolBuildError> {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(workers)
            .build()?;
        Ok(Self { pool })
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
        self.pool.install(|| {
            values
                .par_chunks_mut(chunk_size)
                .enumerate()
                .for_each(|(index, chunk)| operation(index, chunk));
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
        let total = chunk_sizes.iter().try_fold(0_usize, |total, &size| {
            (size > 0).then(|| total.checked_add(size)).flatten()
        });
        if total != Some(values.len()) {
            return Err(ExecutionError::InvalidChunkLayout);
        }

        self.pool.install(|| {
            rayon::scope(|scope| {
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
        });
        Ok(())
    }
}
