use std::fmt;

use kapitsa_core::collision::CollisionEvent;
use tokio::sync::{broadcast, mpsc, watch};

use super::PhysicsClientError;
use super::command::{BatchId, CommandRequest};

/// Collision events committed by one physics step.
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionEventBatch {
    step: u64,
    events: Vec<CollisionEvent>,
}

impl CollisionEventBatch {
    pub(crate) fn new(step: u64, events: Vec<CollisionEvent>) -> Self {
        Self { step, events }
    }

    /// Returns the successful physics step associated with this batch.
    pub fn step(&self) -> u64 {
        self.step
    }

    /// Returns the collision lifecycle events committed by this step.
    pub fn events(&self) -> &[CollisionEvent] {
        &self.events
    }
}

/// Failure returned while receiving collision event batches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionEventStreamError {
    /// The bounded event channel discarded this many earlier batches.
    Lagged { missed_batches: u64 },
    /// The simulation stopped before another batch was received.
    NotRunning,
    /// The simulation and its event sender were dropped.
    ChannelClosed,
}

impl fmt::Display for CollisionEventStreamError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Lagged { missed_batches } => {
                write!(
                    formatter,
                    "collision event stream missed {missed_batches} batches"
                )
            }
            Self::NotRunning => formatter.write_str("physics simulation is not running"),
            Self::ChannelClosed => formatter.write_str("collision event channel is closed"),
        }
    }
}

impl std::error::Error for CollisionEventStreamError {}

/// A batch failure. `command_index` is `None` when the package was not rejected
/// by an individual command, such as stopping before apply or a failed step
/// after successful apply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhysicsBatchError {
    pub command_index: Option<usize>,
    pub error: PhysicsClientError,
}

/// Outcome of an accepted command buffer after its step boundary.
///
/// An `Ok(())` result guarantees that the snapshot for `step` has already been
/// published. The latest snapshot may have advanced beyond this step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhysicsBatchResult {
    pub batch_id: BatchId,
    pub step: u64,
    pub result: Result<(), PhysicsBatchError>,
}

/// Failure returned while receiving command-buffer results.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhysicsBatchResultStreamError {
    Lagged { missed_results: u64 },
    NotRunning,
    ChannelClosed,
}

impl fmt::Display for PhysicsBatchResultStreamError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Lagged { missed_results } => {
                write!(
                    formatter,
                    "physics batch result stream missed {missed_results} results"
                )
            }
            Self::NotRunning => formatter.write_str("physics simulation is not running"),
            Self::ChannelClosed => formatter.write_str("physics batch result channel is closed"),
        }
    }
}

impl std::error::Error for PhysicsBatchResultStreamError {}

/// Subscription to bounded command-buffer execution results.
pub struct PhysicsBatchResults {
    receiver: broadcast::Receiver<PhysicsBatchResult>,
    running: watch::Receiver<Option<mpsc::Sender<CommandRequest>>>,
}

impl PhysicsBatchResults {
    pub(crate) fn new(
        receiver: broadcast::Receiver<PhysicsBatchResult>,
        running: watch::Receiver<Option<mpsc::Sender<CommandRequest>>>,
    ) -> Self {
        Self { receiver, running }
    }

    /// Waits for the next result and reports any results lost to channel lag.
    pub async fn next_result(
        &mut self,
    ) -> Result<PhysicsBatchResult, PhysicsBatchResultStreamError> {
        loop {
            if self.running.borrow().is_none() {
                return self.try_result_after_stop();
            }
            tokio::select! {
                result = self.receiver.recv() => match result {
                    Ok(result) => return Ok(result),
                    Err(broadcast::error::RecvError::Lagged(missed_results)) => {
                        return Err(PhysicsBatchResultStreamError::Lagged { missed_results });
                    }
                    Err(broadcast::error::RecvError::Closed) => {
                        return Err(PhysicsBatchResultStreamError::ChannelClosed);
                    }
                },
                changed = self.running.changed() => {
                    if changed.is_err() {
                        return Err(PhysicsBatchResultStreamError::ChannelClosed);
                    }
                    if self.running.borrow().is_none() {
                        return self.try_result_after_stop();
                    }
                }
            }
        }
    }

    fn try_result_after_stop(
        &mut self,
    ) -> Result<PhysicsBatchResult, PhysicsBatchResultStreamError> {
        match self.receiver.try_recv() {
            Ok(result) => Ok(result),
            Err(broadcast::error::TryRecvError::Lagged(missed_results)) => {
                Err(PhysicsBatchResultStreamError::Lagged { missed_results })
            }
            Err(broadcast::error::TryRecvError::Empty) => {
                Err(PhysicsBatchResultStreamError::NotRunning)
            }
            Err(broadcast::error::TryRecvError::Closed) => {
                Err(PhysicsBatchResultStreamError::ChannelClosed)
            }
        }
    }
}

/// Subscription to bounded collision event batches.
pub struct CollisionEvents {
    receiver: broadcast::Receiver<CollisionEventBatch>,
    running: watch::Receiver<Option<mpsc::Sender<CommandRequest>>>,
}

impl CollisionEvents {
    pub(crate) fn new(
        receiver: broadcast::Receiver<CollisionEventBatch>,
        running: watch::Receiver<Option<mpsc::Sender<CommandRequest>>>,
    ) -> Self {
        Self { receiver, running }
    }

    /// Waits for the next committed event batch.
    ///
    /// Returns `Lagged` with the number of discarded batches instead of hiding loss.
    /// A stopped simulation returns `NotRunning`; the same subscription can be
    /// used again after the simulation restarts.
    pub async fn next_batch(&mut self) -> Result<CollisionEventBatch, CollisionEventStreamError> {
        loop {
            if self.running.borrow().is_none() {
                return match self.receiver.try_recv() {
                    Ok(batch) => Ok(batch),
                    Err(broadcast::error::TryRecvError::Lagged(missed_batches)) => {
                        Err(CollisionEventStreamError::Lagged { missed_batches })
                    }
                    Err(broadcast::error::TryRecvError::Empty) => {
                        Err(CollisionEventStreamError::NotRunning)
                    }
                    Err(broadcast::error::TryRecvError::Closed) => {
                        Err(CollisionEventStreamError::ChannelClosed)
                    }
                };
            }
            tokio::select! {
                batch = self.receiver.recv() => match batch {
                    Ok(batch) => return Ok(batch),
                    Err(broadcast::error::RecvError::Lagged(missed_batches)) => {
                        return Err(CollisionEventStreamError::Lagged { missed_batches });
                    }
                    Err(broadcast::error::RecvError::Closed) => {
                        return Err(CollisionEventStreamError::ChannelClosed);
                    }
                },
                changed = self.running.changed() => {
                    if changed.is_err() {
                        return Err(CollisionEventStreamError::ChannelClosed);
                    }
                    if self.running.borrow().is_none() {
                        return Err(CollisionEventStreamError::NotRunning);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{
        future::Future,
        task::{Context, Poll, Waker},
    };

    use tokio::sync::{broadcast, mpsc, watch};

    use super::{
        CollisionEventBatch, CollisionEventStreamError, CollisionEvents, PhysicsBatchResult,
        PhysicsBatchResultStreamError, PhysicsBatchResults,
    };
    use crate::runtime::asynchronous::marshalling::BatchId;

    type RunningState = Option<mpsc::Sender<super::CommandRequest>>;

    fn running_state() -> (watch::Sender<RunningState>, watch::Receiver<RunningState>) {
        let (commands, _receiver) = mpsc::channel(1);
        watch::channel(Some(commands))
    }

    #[tokio::test]
    async fn lagged_batches_report_the_exact_loss_and_receiver_can_continue() {
        let (events, receiver) = broadcast::channel(1);
        let (_running_tx, running_rx) = running_state();
        let mut subscription = CollisionEvents::new(receiver, running_rx);
        for step in 1..=3 {
            events
                .send(CollisionEventBatch::new(step, Vec::new()))
                .unwrap();
        }

        assert_eq!(
            subscription.next_batch().await,
            Err(CollisionEventStreamError::Lagged { missed_batches: 2 })
        );
        assert_eq!(subscription.next_batch().await.unwrap().step(), 3);
    }

    #[tokio::test]
    async fn subscription_reports_stop_and_resumes_after_restart() {
        let (events, receiver) = broadcast::channel(4);
        let (running_tx, running_rx) = running_state();
        let mut subscription = CollisionEvents::new(receiver, running_rx);
        running_tx.send_replace(None);
        assert_eq!(
            subscription.next_batch().await,
            Err(CollisionEventStreamError::NotRunning)
        );

        let (commands, _receiver) = mpsc::channel(1);
        running_tx.send_replace(Some(commands));
        events
            .send(CollisionEventBatch::new(7, Vec::new()))
            .unwrap();
        assert_eq!(subscription.next_batch().await.unwrap().step(), 7);
    }

    #[tokio::test]
    async fn batch_results_report_lag_and_can_continue_receiving() {
        let (results, receiver) = broadcast::channel(1);
        let (_running_tx, running_rx) = running_state();
        let mut subscription = PhysicsBatchResults::new(receiver, running_rx);
        for id in 1..=3 {
            results
                .send(PhysicsBatchResult {
                    batch_id: BatchId::new(id),
                    step: id,
                    result: Ok(()),
                })
                .expect("subscriber exists");
        }

        assert_eq!(
            subscription.next_result().await,
            Err(PhysicsBatchResultStreamError::Lagged { missed_results: 2 })
        );
        let result = subscription.next_result().await.expect("latest result");
        assert_eq!(result.batch_id.get(), 3);
        assert_eq!(result.step, 3);
    }

    #[tokio::test]
    async fn stop_does_not_hide_a_result_published_at_the_same_time() {
        let (results, receiver) = broadcast::channel(4);
        let (running_tx, running_rx) = running_state();
        let mut subscription = PhysicsBatchResults::new(receiver, running_rx);
        let mut next = Box::pin(subscription.next_result());
        let waker = Waker::noop();
        let mut context = Context::from_waker(waker);
        assert!(matches!(next.as_mut().poll(&mut context), Poll::Pending));

        let expected = PhysicsBatchResult {
            batch_id: BatchId::new(1),
            step: 7,
            result: Ok(()),
        };
        results.send(expected).expect("subscriber exists");
        running_tx.send_replace(None);

        assert_eq!(next.await, Ok(expected));
    }
}
