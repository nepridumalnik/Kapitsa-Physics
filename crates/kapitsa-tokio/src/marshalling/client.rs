use std::sync::Arc;

use kapitsa_core::{
    collider::{Collider, CollisionFilter},
    mass::MassProperties,
    material::MaterialId,
    rigid_body::RigidBody,
    transform::Transform,
    world::{BodyId, ColliderId},
};
use kapitsa_math::Vec3;
use tokio::sync::{mpsc, oneshot, watch};

use super::{PhysicsCommand, PhysicsCommandResult, PhysicsSnapshot, command::CommandRequest};
use crate::PhysicsClientError;

/// Thread-safe handle for submitting physics commands and reading snapshots.
#[derive(Clone)]
pub struct PhysicsClient {
    command_tx: watch::Receiver<Option<mpsc::Sender<CommandRequest>>>,
    snapshot_rx: watch::Receiver<Arc<PhysicsSnapshot>>,
}

impl PhysicsClient {
    #[allow(dead_code)]
    pub(crate) fn new(
        command_tx: watch::Receiver<Option<mpsc::Sender<CommandRequest>>>,
        snapshot_rx: watch::Receiver<Arc<PhysicsSnapshot>>,
    ) -> Self {
        Self {
            command_tx,
            snapshot_rx,
        }
    }

    pub async fn submit(
        &self,
        command: PhysicsCommand,
    ) -> Result<PhysicsCommandResult, PhysicsClientError> {
        let sender = self
            .command_tx
            .borrow()
            .clone()
            .ok_or(PhysicsClientError::NotRunning)?;
        let (reply, response) = oneshot::channel();
        sender
            .send(CommandRequest { command, reply })
            .await
            .map_err(|_| PhysicsClientError::ChannelClosed)?;
        response
            .await
            .map_err(|_| PhysicsClientError::ChannelClosed)?
    }

    pub async fn add_body(&self, body: RigidBody) -> Result<BodyId, PhysicsClientError> {
        match self.submit(PhysicsCommand::AddBody { body }).await? {
            PhysicsCommandResult::Body(id) => Ok(id),
            PhysicsCommandResult::Done | PhysicsCommandResult::Collider(_) => {
                Err(PhysicsClientError::UnexpectedResult)
            }
        }
    }

    pub async fn remove_body(&self, id: BodyId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::RemoveBody { id }).await?)
    }

    pub async fn add_collider(
        &self,
        body: BodyId,
        collider: Collider,
    ) -> Result<ColliderId, PhysicsClientError> {
        match self
            .submit(PhysicsCommand::AddCollider { body, collider })
            .await?
        {
            PhysicsCommandResult::Collider(id) => Ok(id),
            PhysicsCommandResult::Done | PhysicsCommandResult::Body(_) => {
                Err(PhysicsClientError::UnexpectedResult)
            }
        }
    }

    pub async fn remove_collider(&self, id: ColliderId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::RemoveCollider { id }).await?)
    }

    pub async fn set_transform(
        &self,
        id: BodyId,
        transform: Transform,
    ) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::SetTransform { id, transform })
                .await?,
        )
    }

    pub async fn set_linear_velocity(
        &self,
        id: BodyId,
        velocity: Vec3,
    ) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::SetLinearVelocity { id, velocity })
                .await?,
        )
    }

    pub async fn set_angular_velocity(
        &self,
        id: BodyId,
        velocity: Vec3,
    ) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::SetAngularVelocity { id, velocity })
                .await?,
        )
    }

    pub async fn set_mass(
        &self,
        id: BodyId,
        mass: MassProperties,
    ) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::SetMass { id, mass }).await?)
    }

    pub async fn sleep(&self, id: BodyId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::Sleep { id }).await?)
    }

    pub async fn wake_up(&self, id: BodyId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::WakeUp { id }).await?)
    }

    pub async fn apply_force(&self, id: BodyId, force: Vec3) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::ApplyForce { id, force })
                .await?,
        )
    }

    pub async fn apply_force_at_point(
        &self,
        id: BodyId,
        force: Vec3,
        point: Vec3,
    ) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::ApplyForceAtPoint { id, force, point })
                .await?,
        )
    }

    pub async fn apply_torque(&self, id: BodyId, torque: Vec3) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::ApplyTorque { id, torque })
                .await?,
        )
    }

    pub async fn apply_impulse(&self, id: BodyId, impulse: Vec3) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::ApplyImpulse { id, impulse })
                .await?,
        )
    }

    pub async fn apply_impulse_at_point(
        &self,
        id: BodyId,
        impulse: Vec3,
        point: Vec3,
    ) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::ApplyImpulseAtPoint { id, impulse, point })
                .await?,
        )
    }

    pub async fn clear_forces(&self, id: BodyId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::ClearForces { id }).await?)
    }

    pub async fn set_collider_filter(
        &self,
        id: ColliderId,
        filter: CollisionFilter,
    ) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::SetColliderFilter { id, filter })
                .await?,
        )
    }

    pub async fn set_collider_material(
        &self,
        id: ColliderId,
        material: Option<MaterialId>,
    ) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::SetColliderMaterial { id, material })
                .await?,
        )
    }

    pub fn snapshot(&self) -> Arc<PhysicsSnapshot> {
        Arc::clone(&self.snapshot_rx.borrow())
    }

    pub async fn next_snapshot(&mut self) -> Result<Arc<PhysicsSnapshot>, PhysicsClientError> {
        loop {
            if self.command_tx.borrow().is_none() {
                return Err(PhysicsClientError::NotRunning);
            }
            tokio::select! {
                changed = self.snapshot_rx.changed() => {
                    changed.map_err(|_| PhysicsClientError::ChannelClosed)?;
                    return Ok(Arc::clone(&self.snapshot_rx.borrow()));
                }
                changed = self.command_tx.changed() => {
                    if changed.is_err() {
                        return Err(PhysicsClientError::ChannelClosed);
                    }
                    if self.command_tx.borrow().is_none() {
                        return Err(PhysicsClientError::NotRunning);
                    }
                }
            }
        }
    }
}

fn expect_done(result: PhysicsCommandResult) -> Result<(), PhysicsClientError> {
    match result {
        PhysicsCommandResult::Done => Ok(()),
        PhysicsCommandResult::Body(_) | PhysicsCommandResult::Collider(_) => {
            Err(PhysicsClientError::UnexpectedResult)
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{
        future::Future,
        pin::Pin,
        sync::Arc,
        task::{Context, Poll, Waker},
        time::Duration,
    };

    use kapitsa_core::{rigid_body::RigidBody, transform::Transform, world::PhysicsWorld};
    use tokio::sync::{mpsc, oneshot, watch};

    use super::PhysicsClient;
    use crate::{
        PhysicsClientError,
        marshalling::{
            PhysicsCommand, PhysicsCommandResult, PhysicsSnapshot,
            command::{CommandRequest, apply_command},
        },
    };

    struct ClientHarness {
        client: PhysicsClient,
        receiver: mpsc::Receiver<CommandRequest>,
        _command_tx: watch::Sender<Option<mpsc::Sender<CommandRequest>>>,
        _snapshot_tx: watch::Sender<Arc<PhysicsSnapshot>>,
    }

    fn running_client(capacity: usize) -> ClientHarness {
        let (command_tx, command_rx) = mpsc::channel(capacity);
        let (command_state_tx, command_state_rx) = watch::channel(Some(command_tx));
        let initial = Arc::new(PhysicsSnapshot::from_world(
            &PhysicsWorld::new(),
            0,
            Duration::ZERO,
        ));
        let (snapshot_tx, snapshot_rx) = watch::channel(initial);
        ClientHarness {
            client: PhysicsClient::new(command_state_rx, snapshot_rx),
            receiver: command_rx,
            _command_tx: command_state_tx,
            _snapshot_tx: snapshot_tx,
        }
    }

    fn add_body() -> PhysicsCommand {
        PhysicsCommand::AddBody {
            body: RigidBody::new_static(Transform::identity()),
        }
    }

    fn poll_once<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
        let waker = Waker::noop();
        future.poll(&mut Context::from_waker(waker))
    }

    #[test]
    fn client_is_send_sync_and_cloneable() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<PhysicsClient>();

        let harness = running_client(1);
        let _clone = harness.client.clone();
    }

    #[tokio::test]
    async fn add_body_waits_for_and_returns_the_world_result() {
        let ClientHarness {
            client,
            mut receiver,
            ..
        } = running_client(1);
        let server = tokio::spawn(async move {
            let request = receiver.recv().await.expect("request");
            let mut world = PhysicsWorld::new();
            let result = apply_command(&mut world, request.command);
            request.reply.send(result).expect("client is waiting");
            world.body_count()
        });

        let id = client
            .add_body(RigidBody::new_static(Transform::identity()))
            .await
            .expect("body ID");
        assert_eq!(server.await.expect("server task"), 1);
        assert_eq!(id.index(), 0);
    }

    #[tokio::test]
    async fn operation_errors_are_returned_without_becoming_channel_errors() {
        let ClientHarness {
            client,
            mut receiver,
            ..
        } = running_client(1);
        let server = tokio::spawn(async move {
            let request = receiver.recv().await.expect("request");
            request
                .reply
                .send(Err(PhysicsClientError::NotRunning))
                .expect("client is waiting");
        });
        assert!(matches!(
            client.submit(add_body()).await,
            Err(PhysicsClientError::NotRunning)
        ));
        server.await.expect("server task");
    }

    #[tokio::test]
    async fn full_queue_applies_backpressure_until_capacity_is_available() {
        let ClientHarness {
            client,
            mut receiver,
            ..
        } = running_client(1);
        let (reply, _response) = oneshot::channel();
        let sender = client
            .command_tx
            .borrow()
            .as_ref()
            .expect("running sender")
            .clone();
        sender
            .try_send(CommandRequest {
                command: add_body(),
                reply,
            })
            .expect("fill bounded queue");

        let task = tokio::spawn(async move { client.submit(add_body()).await });
        let first = receiver.recv().await.expect("first queued command");
        assert!(matches!(first.command, PhysicsCommand::AddBody { .. }));
        let second = receiver.recv().await.expect("backpressured command");
        second
            .reply
            .send(Ok(PhysicsCommandResult::Done))
            .expect("submit still waiting");
        assert!(matches!(
            task.await.expect("submit task"),
            Ok(PhysicsCommandResult::Done)
        ));
    }

    #[tokio::test]
    async fn cancelling_before_send_does_not_enqueue_a_command() {
        let ClientHarness {
            client,
            mut receiver,
            ..
        } = running_client(1);
        let (reply, _response) = oneshot::channel();
        client
            .command_tx
            .borrow()
            .as_ref()
            .expect("running sender")
            .try_send(CommandRequest {
                command: add_body(),
                reply,
            })
            .expect("fill bounded queue");

        let mut pending = Box::pin(client.submit(add_body()));
        assert!(matches!(poll_once(pending.as_mut()), Poll::Pending));
        drop(pending);

        receiver.recv().await.expect("original command");
        assert!(receiver.try_recv().is_err());
    }

    #[tokio::test]
    async fn cancelling_after_send_does_not_remove_the_queued_command() {
        let ClientHarness {
            client,
            mut receiver,
            ..
        } = running_client(1);
        let task = tokio::spawn(async move { client.submit(add_body()).await });
        let request = receiver.recv().await.expect("command was enqueued");
        task.abort();
        assert!(
            task.await
                .expect_err("client future cancelled")
                .is_cancelled()
        );
        assert!(matches!(request.command, PhysicsCommand::AddBody { .. }));
        assert!(request.reply.send(Ok(PhysicsCommandResult::Done)).is_err());
    }

    #[tokio::test]
    async fn stopped_client_rejects_submission_without_queueing() {
        let (state_tx, state_rx) = watch::channel(None);
        let initial = Arc::new(PhysicsSnapshot::from_world(
            &PhysicsWorld::new(),
            0,
            Duration::ZERO,
        ));
        let (_snapshot_tx, snapshot_rx) = watch::channel(initial);
        let client = PhysicsClient::new(state_rx, snapshot_rx);
        assert!(matches!(
            client.submit(add_body()).await,
            Err(PhysicsClientError::NotRunning)
        ));
        drop(state_tx);
    }
}
