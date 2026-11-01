/// Schedules independent, mutable chunks without owning the values being processed.
///
/// Implementations must wait until every callback has completed before returning.
/// Callbacks may borrow shared step input and write only to their assigned chunk.
pub trait StepExecutor: Sync {
    fn for_each_chunk_mut<T, F>(
        &self,
        values: &mut [T],
        chunk_size: usize,
        operation: F,
    ) -> Result<(), ExecutionError>
    where
        T: Send,
        F: Fn(usize, &mut [T]) + Send + Sync;

    fn for_each_chunk_layout_mut<T, F>(
        &self,
        values: &mut [T],
        chunk_sizes: &[usize],
        operation: F,
    ) -> Result<(), ExecutionError>
    where
        T: Send,
        F: Fn(usize, &mut [T]) + Send + Sync;
}

/// Sequential executor used by `PhysicsWorld::step` for backwards compatibility.
#[derive(Clone, Copy, Debug, Default)]
pub struct SequentialExecutor;

impl StepExecutor for SequentialExecutor {
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
        for (index, chunk) in values.chunks_mut(chunk_size).enumerate() {
            operation(index, chunk);
        }
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
        let mut remaining = values;
        let mut start = 0;
        for &chunk_size in chunk_sizes {
            let (chunk, tail) = remaining.split_at_mut(chunk_size);
            operation(start, chunk);
            start += chunk_size;
            remaining = tail;
        }
        Ok(())
    }
}

/// Failure in the executor's chunk partition contract.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExecutionError {
    InvalidChunkLayout,
}

impl std::fmt::Display for ExecutionError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("executor received an invalid chunk layout")
    }
}

impl std::error::Error for ExecutionError {}

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
    use super::{ExecutionError, SequentialExecutor, StepExecutor};

    #[test]
    fn sequential_executor_processes_fixed_chunks_in_order() {
        let executor = SequentialExecutor;
        let mut values = [0; 5];

        executor
            .for_each_chunk_mut(&mut values, 2, |chunk_index, chunk| {
                for value in chunk {
                    *value = chunk_index + 1;
                }
            })
            .expect("valid chunk size");

        assert_eq!(values, [1, 1, 2, 2, 3]);
    }

    #[test]
    fn sequential_executor_preserves_cost_weighted_chunk_layout() {
        let executor = SequentialExecutor;
        let mut values = [0; 6];

        executor
            .for_each_chunk_layout_mut(&mut values, &[1, 3, 2], |start, chunk| {
                for value in chunk {
                    *value = start;
                }
            })
            .expect("valid chunk layout");

        assert_eq!(values, [0, 1, 1, 1, 4, 4]);
    }

    #[test]
    fn executor_rejects_invalid_chunk_layout_before_processing() {
        let executor = SequentialExecutor;
        let mut values = [0; 3];

        assert_eq!(
            executor.for_each_chunk_layout_mut(&mut values, &[1, 0, 2], |_, chunk| {
                chunk.fill(1);
            }),
            Err(ExecutionError::InvalidChunkLayout)
        );
        assert_eq!(values, [0; 3]);
    }
}
