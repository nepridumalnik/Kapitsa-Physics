use kapitsa_core::{
    chunk::Chunk,
    world::{BodyId, ChunkId, ChunkOperationError, ColliderId, StepError},
};
use tokio::sync::oneshot;

use super::super::chunk::{ChunkClientError, ChunkCommand, ChunkOutcome, ChunkReply};
use super::{CommandRequest, PhysicsClient, PhysicsClientError};

fn finish<I, O>(
    result: Result<O, (ChunkOperationError, I)>,
    failure: Option<(u64, StepError)>,
) -> Result<O, ChunkClientError<I, O>> {
    match result {
        Err((cause, input)) => Err(ChunkClientError::Rejected { cause, input }),
        Ok(output) => match failure {
            Some((step, cause)) => Err(ChunkClientError::StepFailedAfterApply {
                step,
                cause,
                output,
            }),
            None => Ok(output),
        },
    }
}

impl PhysicsClient {
    async fn request_chunk(&self, command: ChunkCommand) -> Result<ChunkReply, PhysicsClientError> {
        let sender = match self.command_tx.borrow().clone() {
            Some(sender) => sender,
            None => {
                return Ok(ChunkReply::NotSubmitted {
                    command,
                    cause: PhysicsClientError::NotRunning,
                });
            }
        };
        // Reserve queue capacity before transferring ownership, so failures while
        // waiting for backpressure can return the original input.
        let permit = match sender.reserve().await {
            Ok(permit) => permit,
            Err(_) => {
                return Ok(ChunkReply::NotSubmitted {
                    command,
                    cause: PhysicsClientError::ChannelClosed,
                });
            }
        };
        let (reply, response) = oneshot::channel();
        permit.send(CommandRequest::Chunk { command, reply });
        response
            .await
            .map_err(|_| PhysicsClientError::ChannelClosed)
    }

    /// Registers a prepared chunk at a simulation boundary.
    /// Cancelling after enqueue does not cancel the operation; keep the future
    /// alive until completion to receive the ID or returned geometry.
    // Return the chunk directly, without allocating on the failure path.
    #[allow(clippy::result_large_err)]
    pub async fn add_chunk(
        &self,
        body: BodyId,
        chunk: Chunk,
    ) -> Result<ChunkId, ChunkClientError<Chunk, ChunkId>> {
        match self.request_chunk(ChunkCommand::Add { body, chunk }).await {
            Ok(ChunkReply::NotSubmitted {
                command: ChunkCommand::Add { chunk, .. },
                cause,
            }) => Err(ChunkClientError::NotSubmitted {
                cause,
                input: chunk,
            }),
            Ok(ChunkReply::Completed {
                outcome: ChunkOutcome::Add(result),
                step_failure,
            }) => finish(
                result.map_err(|error| (error.cause, error.chunk)),
                step_failure,
            ),
            _ => Err(ChunkClientError::ResponseLost),
        }
    }

    /// Atomically registers all prepared chunks on one body, preserving input order.
    pub async fn add_chunk_batch(
        &self,
        body: BodyId,
        chunks: Vec<Chunk>,
    ) -> Result<Vec<ChunkId>, ChunkClientError<Vec<Chunk>, Vec<ChunkId>>> {
        match self
            .request_chunk(ChunkCommand::AddBatch { body, chunks })
            .await
        {
            Ok(ChunkReply::NotSubmitted {
                command: ChunkCommand::AddBatch { chunks, .. },
                cause,
            }) => Err(ChunkClientError::NotSubmitted {
                cause,
                input: chunks,
            }),
            Ok(ChunkReply::Completed {
                outcome: ChunkOutcome::AddBatch(result),
                step_failure,
            }) => finish(
                result.map_err(|error| (error.cause, error.chunks)),
                step_failure,
            ),
            _ => Err(ChunkClientError::ResponseLost),
        }
    }

    /// Removes a chunk and returns its prepared geometry for caching/reuse.
    // Preserve the returned chunk if the subsequent step fails, without boxing it.
    #[allow(clippy::result_large_err)]
    pub async fn remove_chunk(
        &self,
        id: ChunkId,
    ) -> Result<Option<Chunk>, ChunkClientError<ChunkId, Option<Chunk>>> {
        match self.request_chunk(ChunkCommand::Remove { id }).await {
            Ok(ChunkReply::NotSubmitted {
                command: ChunkCommand::Remove { id },
                cause,
            }) => Err(ChunkClientError::NotSubmitted { cause, input: id }),
            Ok(ChunkReply::Completed {
                outcome: ChunkOutcome::Remove(result),
                step_failure,
            }) => finish(Ok(result), step_failure),
            _ => Err(ChunkClientError::ResponseLost),
        }
    }

    /// Atomically removes all IDs; returned chunks follow the input order.
    pub async fn remove_chunk_batch(
        &self,
        ids: Vec<ChunkId>,
    ) -> Result<Vec<Chunk>, ChunkClientError<Vec<ChunkId>, Vec<Chunk>>> {
        match self.request_chunk(ChunkCommand::RemoveBatch { ids }).await {
            Ok(ChunkReply::NotSubmitted {
                command: ChunkCommand::RemoveBatch { ids },
                cause,
            }) => Err(ChunkClientError::NotSubmitted { cause, input: ids }),
            Ok(ChunkReply::Completed {
                outcome: ChunkOutcome::RemoveBatch { ids, result },
                step_failure,
            }) => finish(result.map_err(|error| (error, ids)), step_failure),
            _ => Err(ChunkClientError::ResponseLost),
        }
    }

    /// Reads the number of registered chunks at this request's FIFO position.
    pub async fn chunk_count(&self) -> Result<usize, PhysicsClientError> {
        match self.request_chunk(ChunkCommand::Count).await? {
            ChunkReply::NotSubmitted { cause, .. } => Err(cause),
            ChunkReply::Completed {
                outcome: ChunkOutcome::Count(value),
                ..
            } => Ok(value),
            _ => Err(PhysicsClientError::UnexpectedResult),
        }
    }

    /// Resolves a chunk's owning body at this request's FIFO position.
    pub async fn chunk_body(&self, id: ChunkId) -> Result<Option<BodyId>, PhysicsClientError> {
        match self.request_chunk(ChunkCommand::Body { id }).await? {
            ChunkReply::NotSubmitted { cause, .. } => Err(cause),
            ChunkReply::Completed {
                outcome: ChunkOutcome::Body(value),
                ..
            } => Ok(value),
            _ => Err(PhysicsClientError::UnexpectedResult),
        }
    }

    /// Reads IDs of the colliders owned by a registered chunk.
    pub async fn chunk_colliders(
        &self,
        id: ChunkId,
    ) -> Result<Option<Vec<ColliderId>>, PhysicsClientError> {
        match self.request_chunk(ChunkCommand::Colliders { id }).await? {
            ChunkReply::NotSubmitted { cause, .. } => Err(cause),
            ChunkReply::Completed {
                outcome: ChunkOutcome::Colliders(value),
                ..
            } => value.map_err(Into::into),
            _ => Err(PhysicsClientError::UnexpectedResult),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_core::{chunk::ChunkBuilder, shape::TriangleMesh, transform::Transform};
    use kapitsa_math::Vec3;
    use std::error::Error;

    #[test]
    fn failed_step_keeps_removed_chunk_in_client_error() -> Result<(), Box<dyn Error>> {
        let mut builder = ChunkBuilder::new();
        builder.add_mesh(TriangleMesh::new(
            vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            vec![[0, 1, 2]],
        )?)?;
        let chunk = builder.build(Transform::identity())?;
        let result = finish::<(), _>(Ok(Some(chunk)), Some((17, StepError::InvalidDeltaTime)));
        match result {
            Err(ChunkClientError::StepFailedAfterApply {
                step: 17,
                cause: StepError::InvalidDeltaTime,
                output: Some(chunk),
            }) => assert_eq!(chunk.colliders().len(), 1),
            _ => return Err("applied removal result was lost".into()),
        }
        Ok(())
    }
}
