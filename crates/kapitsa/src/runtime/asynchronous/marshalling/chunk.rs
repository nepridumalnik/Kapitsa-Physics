use std::fmt;

use super::PhysicsClientError;
use kapitsa_core::{
    chunk::Chunk,
    world::{
        AddChunkBatchError, AddChunkError, BodyId, ChunkId, ChunkOperationError, ColliderId,
        PhysicsWorld, StepError, WorldError,
    },
};

/// A chunk operation's failure, preserving its input or applied output when available.
pub enum ChunkClientError<I, O> {
    /// The service did not accept the operation; its original input is returned.
    NotSubmitted { cause: PhysicsClientError, input: I },
    /// Validation or reservation failed before the world was changed.
    Rejected {
        cause: ChunkOperationError,
        input: I,
    },
    /// The operation was applied, but the subsequent simulation step failed.
    /// The actual output is returned because the world mutation was not rolled back.
    StepFailedAfterApply {
        step: u64,
        cause: StepError,
        output: O,
    },
    /// The response channel closed after enqueueing; the operation's outcome is unknown.
    ResponseLost,
}

impl<I, O> fmt::Debug for ChunkClientError<I, O> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotSubmitted { cause, .. } => f.debug_tuple("NotSubmitted").field(cause).finish(),
            Self::Rejected { cause, .. } => f.debug_tuple("Rejected").field(cause).finish(),
            Self::StepFailedAfterApply { step, cause, .. } => f
                .debug_struct("StepFailedAfterApply")
                .field("step", step)
                .field("cause", cause)
                .finish_non_exhaustive(),
            Self::ResponseLost => f.write_str("ResponseLost"),
        }
    }
}

impl<I, O> fmt::Display for ChunkClientError<I, O> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotSubmitted { cause, .. } => write!(f, "chunk operation not submitted: {cause}"),
            Self::Rejected { cause, .. } => write!(f, "chunk operation rejected: {cause}"),
            Self::StepFailedAfterApply { step, cause, .. } => write!(
                f,
                "physics step {step} failed after chunk operation: {cause}"
            ),
            Self::ResponseLost => f.write_str("chunk operation response was lost after enqueueing"),
        }
    }
}

impl<I, O> std::error::Error for ChunkClientError<I, O> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::NotSubmitted { cause, .. } => Some(cause),
            Self::Rejected { cause, .. } => Some(cause),
            Self::StepFailedAfterApply { cause, .. } => Some(cause),
            Self::ResponseLost => None,
        }
    }
}

pub(crate) enum ChunkCommand {
    Add { body: BodyId, chunk: Chunk },
    AddBatch { body: BodyId, chunks: Vec<Chunk> },
    Remove { id: ChunkId },
    RemoveBatch { ids: Vec<ChunkId> },
    Count,
    Body { id: ChunkId },
    Colliders { id: ChunkId },
}

pub(crate) enum ChunkOutcome {
    Add(Result<ChunkId, AddChunkError>),
    AddBatch(Result<Vec<ChunkId>, AddChunkBatchError>),
    Remove(Option<Chunk>),
    RemoveBatch {
        ids: Vec<ChunkId>,
        result: Result<Vec<Chunk>, ChunkOperationError>,
    },
    Count(usize),
    Body(Option<BodyId>),
    Colliders(Result<Option<Vec<ColliderId>>, WorldError>),
}

impl ChunkOutcome {
    pub(crate) fn applied(&self) -> bool {
        match self {
            Self::Add(result) => result.is_ok(),
            Self::AddBatch(result) => result.as_ref().is_ok_and(|ids| !ids.is_empty()),
            Self::Remove(result) => result.is_some(),
            Self::RemoveBatch { result, .. } => {
                result.as_ref().is_ok_and(|chunks| !chunks.is_empty())
            }
            Self::Count(_) | Self::Body(_) | Self::Colliders(_) => false,
        }
    }
}

pub(crate) enum ChunkReply {
    NotSubmitted {
        command: ChunkCommand,
        cause: PhysicsClientError,
    },
    Completed {
        outcome: ChunkOutcome,
        step_failure: Option<(u64, StepError)>,
    },
}

pub(crate) fn apply_chunk(world: &mut PhysicsWorld, command: ChunkCommand) -> ChunkOutcome {
    match command {
        ChunkCommand::Add { body, chunk } => ChunkOutcome::Add(world.add_chunk(body, chunk)),
        ChunkCommand::AddBatch { body, chunks } => {
            ChunkOutcome::AddBatch(world.add_chunk_batch(body, chunks))
        }
        ChunkCommand::Remove { id } => ChunkOutcome::Remove(world.remove_chunk(id)),
        ChunkCommand::RemoveBatch { ids } => {
            let mut input = Vec::new();
            let result = match input.try_reserve_exact(ids.len()) {
                Ok(()) => {
                    input.extend_from_slice(&ids);
                    world.remove_chunk_batch(input)
                }
                Err(_) => Err(ChunkOperationError::World(
                    WorldError::DensePoolCapacityExceeded,
                )),
            };
            ChunkOutcome::RemoveBatch { ids, result }
        }
        ChunkCommand::Count => ChunkOutcome::Count(world.chunk_count()),
        ChunkCommand::Body { id } => ChunkOutcome::Body(world.chunk_body(id)),
        ChunkCommand::Colliders { id } => ChunkOutcome::Colliders((|| {
            let Some(ids) = world.chunk_colliders(id) else {
                return Ok(None);
            };
            let mut colliders = Vec::new();
            colliders
                .try_reserve_exact(ids.len())
                .map_err(|_| WorldError::DensePoolCapacityExceeded)?;
            colliders.extend_from_slice(ids);
            Ok(Some(colliders))
        })()),
    }
}
