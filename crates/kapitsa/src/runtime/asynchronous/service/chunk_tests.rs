use super::{
    CommandQueue, CommandRequest, DeferredCommandResult, SimulationEvents, apply_chunk,
    publish_failed_command_results, reject_request,
};
use crate::runtime::asynchronous::marshalling::{
    ChunkCommand, ChunkOutcome, ChunkReply, CommandBufferPool,
};
use kapitsa_core::{
    chunk::{Chunk, ChunkBuilder},
    rigid_body::RigidBody,
    shape::TriangleMesh,
    transform::Transform,
    world::{PhysicsWorld, StepError},
};
use kapitsa_math::Vec3;
use std::{error::Error, sync::Arc};
use tokio::sync::{broadcast, mpsc, oneshot};

fn chunk() -> Result<Chunk, Box<dyn Error>> {
    let mut builder = ChunkBuilder::new();
    builder.add_mesh(TriangleMesh::new(
        vec![Vec3::ZERO, Vec3::X, Vec3::Y],
        vec![[0, 1, 2]],
    )?)?;
    Ok(builder.build(Transform::identity())?)
}

fn events() -> SimulationEvents {
    let (collisions, _) = broadcast::channel(1);
    let (joint_breaks, _) = broadcast::channel(1);
    let (batch_results, _) = broadcast::channel(1);
    SimulationEvents {
        collisions,
        joint_breaks,
        batch_results,
    }
}

#[tokio::test]
async fn failed_step_preserves_applied_ids_and_removed_geometry() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let (_sender, receiver) = mpsc::channel(1);
    let mut queue = CommandQueue::new(receiver, Arc::new(CommandBufferPool::default()));
    let events = events();
    let (reply, receive) = oneshot::channel();
    let outcome = apply_chunk(
        &mut world,
        ChunkCommand::Add {
            body,
            chunk: chunk()?,
        },
    );
    queue
        .pending_results
        .push(DeferredCommandResult::Chunk { reply, outcome });
    publish_failed_command_results(&mut queue, &events, 7, StepError::InvalidDeltaTime);
    let id = match receive.await? {
        ChunkReply::Completed {
            outcome: ChunkOutcome::Add(Ok(id)),
            step_failure: Some((7, StepError::InvalidDeltaTime)),
        } => id,
        _ => return Err("missing applied chunk ID or failed step".into()),
    };
    assert_eq!(world.chunk_body(id), Some(body));
    let (reply, receive) = oneshot::channel();
    let outcome = apply_chunk(&mut world, ChunkCommand::Remove { id });
    queue
        .pending_results
        .push(DeferredCommandResult::Chunk { reply, outcome });
    publish_failed_command_results(&mut queue, &events, 8, StepError::InvalidDeltaTime);
    match receive.await? {
        ChunkReply::Completed {
            outcome: ChunkOutcome::Remove(Some(chunk)),
            step_failure: Some((8, StepError::InvalidDeltaTime)),
        } => assert_eq!(chunk.colliders().len(), 1),
        _ => return Err("removed geometry was not preserved".into()),
    }
    assert!(world.chunk_body(id).is_none());
    Ok(())
}

#[tokio::test]
async fn stopping_before_application_returns_chunk_to_caller() -> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let (reply, receive) = oneshot::channel();
    let request = CommandRequest::Chunk {
        command: ChunkCommand::Add {
            body,
            chunk: chunk()?,
        },
        reply,
    };
    reject_request(request, 0, &events(), &CommandBufferPool::default());
    match receive.await? {
        ChunkReply::NotSubmitted {
            command:
                ChunkCommand::Add {
                    body: returned,
                    chunk,
                },
            cause: crate::PhysicsClientError::NotRunning,
        } => {
            assert_eq!(returned, body);
            assert_eq!(chunk.colliders().len(), 1);
        }
        _ => return Err("rejected request did not retain its chunk".into()),
    }
    assert_eq!(world.chunk_count(), 0);
    Ok(())
}

#[tokio::test]
async fn failed_step_does_not_turn_empty_batch_or_query_into_applied_errors()
-> Result<(), Box<dyn Error>> {
    let mut world = PhysicsWorld::new();
    let body = world.add_body(RigidBody::new_static(Transform::identity()))?;
    let (_sender, receiver) = mpsc::channel(1);
    let mut queue = CommandQueue::new(receiver, Arc::new(CommandBufferPool::default()));
    let (add_reply, add_result) = oneshot::channel();
    let (query_reply, query_result) = oneshot::channel();
    queue.pending_results.push(DeferredCommandResult::Chunk {
        reply: add_reply,
        outcome: apply_chunk(
            &mut world,
            ChunkCommand::AddBatch {
                body,
                chunks: Vec::new(),
            },
        ),
    });
    queue.pending_results.push(DeferredCommandResult::Chunk {
        reply: query_reply,
        outcome: apply_chunk(&mut world, ChunkCommand::Count),
    });
    publish_failed_command_results(&mut queue, &events(), 3, StepError::InvalidDeltaTime);
    assert!(
        matches!(add_result.await?, ChunkReply::Completed { outcome: ChunkOutcome::AddBatch(Ok(ids)), step_failure: None } if ids.is_empty())
    );
    assert!(matches!(
        query_result.await?,
        ChunkReply::Completed {
            outcome: ChunkOutcome::Count(0),
            step_failure: None
        }
    ));
    Ok(())
}
