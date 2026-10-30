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
    pub(crate) fn new(
        command_tx: watch::Receiver<Option<mpsc::Sender<CommandRequest>>>,
        snapshot_rx: watch::Receiver<Arc<PhysicsSnapshot>>,
    ) -> Self {
        Self {
            command_tx,
            snapshot_rx,
        }
    }

    /// Queues a command and waits until it has been applied to the world.
    ///
    /// Cancelling before the bounded send completes prevents enqueueing. Once
    /// enqueued, the command remains active even if this future is cancelled.
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

    /// Adds a body and returns its world-scoped ID after insertion.
    pub async fn add_body(&self, body: RigidBody) -> Result<BodyId, PhysicsClientError> {
        match self.submit(PhysicsCommand::AddBody { body }).await? {
            PhysicsCommandResult::Body(id) => Ok(id),
            PhysicsCommandResult::Done | PhysicsCommandResult::Collider(_) => {
                Err(PhysicsClientError::UnexpectedResult)
            }
        }
    }

    /// Removes a body and all colliders attached to it.
    pub async fn remove_body(&self, id: BodyId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::RemoveBody { id }).await?)
    }

    /// Attaches a collider to a body and returns the collider ID.
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

    /// Removes a collider from the world.
    pub async fn remove_collider(&self, id: ColliderId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::RemoveCollider { id }).await?)
    }

    /// Replaces a body's transform while preserving its velocities.
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

    /// Sets a body's world-space linear velocity.
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

    /// Sets a body's world-space angular velocity.
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

    /// Replaces a dynamic body's mass properties.
    pub async fn set_mass(
        &self,
        id: BodyId,
        mass: MassProperties,
    ) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::SetMass { id, mass }).await?)
    }

    /// Puts a dynamic body to sleep.
    pub async fn sleep(&self, id: BodyId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::Sleep { id }).await?)
    }

    /// Wakes a dynamic body.
    pub async fn wake_up(&self, id: BodyId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::WakeUp { id }).await?)
    }

    /// Accumulates a world-space force on a dynamic body.
    pub async fn apply_force(&self, id: BodyId, force: Vec3) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::ApplyForce { id, force })
                .await?,
        )
    }

    /// Accumulates a world-space force at a world-space point.
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

    /// Accumulates a world-space torque on a dynamic body.
    pub async fn apply_torque(&self, id: BodyId, torque: Vec3) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::ApplyTorque { id, torque })
                .await?,
        )
    }

    /// Applies a world-space impulse immediately to a dynamic body.
    pub async fn apply_impulse(&self, id: BodyId, impulse: Vec3) -> Result<(), PhysicsClientError> {
        expect_done(
            self.submit(PhysicsCommand::ApplyImpulse { id, impulse })
                .await?,
        )
    }

    /// Applies a world-space impulse at a world-space point immediately.
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

    /// Clears accumulated force and torque on a body.
    pub async fn clear_forces(&self, id: BodyId) -> Result<(), PhysicsClientError> {
        expect_done(self.submit(PhysicsCommand::ClearForces { id }).await?)
    }

    /// Replaces a collider's collision filter.
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

    /// Assigns or clears a collider material ID without resolving it.
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

    /// Returns an `Arc` to the latest published snapshot without waiting.
    pub fn snapshot(&self) -> Arc<PhysicsSnapshot> {
        Arc::clone(&self.snapshot_rx.borrow())
    }

    /// Waits for a publication not yet observed by this client.
    ///
    /// Intermediate snapshots may be skipped. Returns [`PhysicsClientError::NotRunning`]
    /// when the simulation stops before publishing another snapshot.
    pub async fn next_snapshot(&mut self) -> Result<Arc<PhysicsSnapshot>, PhysicsClientError> {
        if self.command_tx.borrow().is_none() {
            return Err(PhysicsClientError::NotRunning);
        }

        // A new call always starts waiting from the latest publication, even
        // when another read left the watch version marked as unseen.
        {
            let _ = self.snapshot_rx.borrow_and_update();
        }

        loop {
            tokio::select! {
                changed = self.snapshot_rx.changed() => {
                    changed.map_err(|_| PhysicsClientError::ChannelClosed)?;
                    return Ok(Arc::clone(&self.snapshot_rx.borrow_and_update()));
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

    fn snapshot(step: u64) -> Arc<PhysicsSnapshot> {
        Arc::new(PhysicsSnapshot::from_world(
            &PhysicsWorld::new(),
            step,
            Duration::ZERO,
        ))
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
    async fn available_mpsc_capacity_accepts_concurrent_requests_on_first_poll() {
        let ClientHarness {
            client,
            mut receiver,
            ..
        } = running_client(128);
        let mut submissions = (0..100)
            .map(|_| Box::pin(client.submit(add_body())))
            .collect::<Vec<_>>();
        for submission in &mut submissions {
            assert!(matches!(poll_once(submission.as_mut()), Poll::Pending));
        }

        let mut received = 0;
        while receiver.try_recv().is_ok() {
            received += 1;
        }
        assert_eq!(received, 100);
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

    #[tokio::test]
    async fn snapshot_then_next_snapshot_waits_for_new_publication() {
        let ClientHarness {
            mut client,
            _command_tx,
            _snapshot_tx,
            ..
        } = running_client(1);
        let current = client.snapshot();
        let mut next = Box::pin(client.next_snapshot());
        assert!(matches!(poll_once(next.as_mut()), Poll::Pending));

        let published = snapshot(0);
        _snapshot_tx.send_replace(Arc::clone(&published));
        let received = match poll_once(next.as_mut()) {
            Poll::Ready(Ok(snapshot)) => snapshot,
            Poll::Ready(Err(error)) => panic!("unexpected error: {error}"),
            Poll::Pending => panic!("publication should wake the receiver"),
        };

        assert!(!Arc::ptr_eq(&current, &received));
        assert!(Arc::ptr_eq(&published, &received));
    }

    #[tokio::test]
    async fn next_snapshot_does_not_return_preexisting_unread_version() {
        let ClientHarness {
            mut client,
            _command_tx,
            _snapshot_tx,
            ..
        } = running_client(1);
        let preexisting = snapshot(1);
        _snapshot_tx.send_replace(Arc::clone(&preexisting));
        let _ = client.snapshot();

        let mut next = Box::pin(client.next_snapshot());
        assert!(matches!(poll_once(next.as_mut()), Poll::Pending));

        let published = snapshot(1);
        _snapshot_tx.send_replace(Arc::clone(&published));
        let received = match poll_once(next.as_mut()) {
            Poll::Ready(Ok(snapshot)) => snapshot,
            Poll::Ready(Err(error)) => panic!("unexpected error: {error}"),
            Poll::Pending => panic!("publication should wake the receiver"),
        };

        assert!(!Arc::ptr_eq(&preexisting, &received));
        assert!(Arc::ptr_eq(&published, &received));
    }

    #[tokio::test]
    async fn next_snapshot_receives_publication_after_call() {
        let ClientHarness {
            mut client,
            _command_tx,
            _snapshot_tx,
            ..
        } = running_client(1);
        let mut next = Box::pin(client.next_snapshot());
        assert!(matches!(poll_once(next.as_mut()), Poll::Pending));

        let published = snapshot(1);
        _snapshot_tx.send_replace(Arc::clone(&published));
        let received = match poll_once(next.as_mut()) {
            Poll::Ready(Ok(snapshot)) => snapshot,
            Poll::Ready(Err(error)) => panic!("unexpected error: {error}"),
            Poll::Pending => panic!("publication should wake the receiver"),
        };
        assert!(Arc::ptr_eq(&published, &received));
    }

    #[tokio::test]
    async fn next_snapshot_does_not_lose_racing_publication() {
        let ClientHarness {
            mut client,
            _command_tx,
            _snapshot_tx,
            ..
        } = running_client(1);
        let mut next = Box::pin(client.next_snapshot());

        // The first poll fixes the baseline and registers the watch wait. A
        // publication before the future is polled again must still be seen.
        assert!(matches!(poll_once(next.as_mut()), Poll::Pending));
        let published = snapshot(0);
        _snapshot_tx.send_replace(Arc::clone(&published));

        let received = match poll_once(next.as_mut()) {
            Poll::Ready(Ok(snapshot)) => snapshot,
            Poll::Ready(Err(error)) => panic!("unexpected error: {error}"),
            Poll::Pending => panic!("racing publication should remain observable"),
        };
        assert!(Arc::ptr_eq(&published, &received));
    }

    #[tokio::test]
    async fn next_snapshot_returns_not_running_on_stop() {
        let ClientHarness {
            mut client,
            _command_tx,
            _snapshot_tx,
            ..
        } = running_client(1);
        let mut next = Box::pin(client.next_snapshot());
        assert!(matches!(poll_once(next.as_mut()), Poll::Pending));

        _command_tx.send_replace(None);
        assert!(matches!(
            poll_once(next.as_mut()),
            Poll::Ready(Err(PhysicsClientError::NotRunning))
        ));
    }

    #[test]
    fn snapshot_remains_nonblocking() {
        let harness = running_client(1);
        let current = harness.client.snapshot();
        assert_eq!(current.step(), 0);
        assert_eq!(current.len(), 0);
    }
}
