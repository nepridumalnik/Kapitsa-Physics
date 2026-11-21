//! Bounded, off-thread cooking of immutable shape inputs.

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::{io, sync::mpsc, thread};

use super::{CookedShape, CookedSource, GeometryConfig};

/// A single background cook worker with bounded waiting capacity.
#[derive(Clone)]
pub struct ShapeWorker {
    requests: mpsc::SyncSender<Request>,
}

struct Request {
    cancelled: Arc<AtomicBool>,
    sources: Vec<CookedSource>,
    config: GeometryConfig,
    oriented_cavities: bool,
    result: mpsc::SyncSender<Result<CookedShape, crate::Error>>,
}

/// Failure to enqueue a cook, before any work is published.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubmitError {
    /// All waiting slots are occupied.
    Full,
    /// The worker is no longer running.
    Disconnected,
}

impl std::fmt::Display for SubmitError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::Full => "shape cook queue is full",
            Self::Disconnected => "shape cook worker is disconnected",
        })
    }
}

impl std::error::Error for SubmitError {}

/// Aerodynamic behavior while an asynchronously managed shape is pending.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PendingAero {
    /// Use the last good shape only while its topology still matches.
    KeepPrevious,
    /// Suppress aerodynamic loads until publication.
    ZeroForce,
    /// Wait for the background result before evaluating the step.
    BlockStep,
}

/// Action at a step boundary before aerodynamic evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PendingAction {
    /// Evaluate using the last successfully installed shape.
    Previous,
    /// Do not apply the pending body's aerodynamic loads.
    Suppress,
    /// Wait for the worker before evaluating aerodynamics.
    Wait,
}

impl PendingAero {
    /// Chooses an action. Pass `false` if there is no usable previous shape.
    pub fn action(self, previous_topology_matches: bool) -> PendingAction {
        match self {
            Self::KeepPrevious if previous_topology_matches => PendingAction::Previous,
            Self::KeepPrevious | Self::ZeroForce => PendingAction::Suppress,
            Self::BlockStep => PendingAction::Wait,
        }
    }
}

/// One pending cook. Dropping it discards the result without blocking.
pub struct PendingShape {
    #[cfg(test)]
    pub(crate) on_wait: Option<mpsc::Sender<()>>,
    cancelled: Arc<AtomicBool>,
    result: std::sync::Mutex<mpsc::Receiver<Result<CookedShape, crate::Error>>>,
}

impl Drop for PendingShape {
    fn drop(&mut self) {
        self.cancelled.store(true, Ordering::Relaxed);
    }
}

impl PendingShape {
    #[cfg(test)]
    pub(crate) fn test_channel() -> (mpsc::SyncSender<Result<CookedShape, crate::Error>>, Self) {
        let (sender, receiver) = mpsc::sync_channel(1);
        (
            sender,
            Self {
                on_wait: None,
                cancelled: Arc::new(AtomicBool::new(false)),
                result: std::sync::Mutex::new(receiver),
            },
        )
    }
    pub(crate) fn wait(&mut self) -> Result<Result<CookedShape, crate::Error>, mpsc::TryRecvError> {
        #[cfg(test)]
        if let Some(signal) = self.on_wait.take() {
            let _ = signal.send(());
        }
        self.result
            .get_mut()
            .map_err(|_| mpsc::TryRecvError::Disconnected)?
            .recv()
            .map_err(|_| mpsc::TryRecvError::Disconnected)
    }
    /// Polls the result without blocking. Disconnection is distinct from a cook error.
    pub fn try_recv(&mut self) -> Result<Result<CookedShape, crate::Error>, mpsc::TryRecvError> {
        self.result
            .get_mut()
            .map_err(|_| mpsc::TryRecvError::Disconnected)?
            .try_recv()
    }

    /// Waits for this cook outside the simulation step.
    pub fn recv(self) -> Result<Result<CookedShape, crate::Error>, mpsc::RecvError> {
        self.result.lock().map_err(|_| mpsc::RecvError)?.recv()
    }
}

impl ShapeWorker {
    /// Starts one worker. Capacity counts waiting requests, not the active cook.
    pub fn new(capacity: usize) -> io::Result<Self> {
        let (requests, receiver) = mpsc::sync_channel::<Request>(capacity);
        thread::Builder::new()
            .name("kapitsa-shape-cook".into())
            .spawn(move || {
                while let Ok(request) = receiver.recv() {
                    if request.cancelled.load(Ordering::Relaxed) {
                        continue;
                    }
                    let result = CookedShape::cook(
                        &request.sources,
                        request.config,
                        request.oriented_cavities,
                    );
                    let _ = request.result.send(result);
                }
            })?;
        Ok(Self { requests })
    }

    /// Queues owned cook inputs without blocking the caller.
    pub fn submit(
        &self,
        sources: Vec<CookedSource>,
        config: GeometryConfig,
        oriented_cavities: bool,
    ) -> Result<PendingShape, SubmitError> {
        let (result, receiver) = mpsc::sync_channel(1);
        let cancelled = Arc::new(AtomicBool::new(false));
        self.requests
            .try_send(Request {
                cancelled: Arc::clone(&cancelled),
                sources,
                config,
                oriented_cavities,
                result,
            })
            .map_err(|error| match error {
                mpsc::TrySendError::Full(_) => SubmitError::Full,
                mpsc::TrySendError::Disconnected(_) => SubmitError::Disconnected,
            })?;
        Ok(PendingShape {
            #[cfg(test)]
            on_wait: None,
            cancelled,
            result: std::sync::Mutex::new(receiver),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::geometry::GeometryError;

    #[test]
    fn pending_policy_never_retains_a_shape_across_topology_changes() {
        use PendingAction::{Previous, Suppress, Wait};
        for (policy, unchanged, changed) in [
            (PendingAero::KeepPrevious, Previous, Suppress),
            (PendingAero::ZeroForce, Suppress, Suppress),
            (PendingAero::BlockStep, Wait, Wait),
        ] {
            assert_eq!(policy.action(true), unchanged);
            assert_eq!(policy.action(false), changed);
        }
    }

    #[test]
    fn cancelled_result_cannot_replace_the_next_request() -> Result<(), Box<dyn std::error::Error>>
    {
        let worker = ShapeWorker::new(2)?;
        let config = super::super::assembly_pressure_config();
        let obsolete = worker
            .submit(Vec::new(), config, false)
            .map_err(|_| "obsolete submission rejected")?;
        let mesh = crate::shape::TriangleMesh::new(
            vec![
                kapitsa_math::Vec3::ZERO,
                kapitsa_math::Vec3::Y,
                kapitsa_math::Vec3::Z,
            ],
            vec![[0, 1, 2]],
        )?;
        let current = worker
            .submit(
                vec![CookedSource {
                    mesh: std::sync::Arc::new(mesh),
                    local_transform: crate::transform::Transform::identity(),
                    slot: super::super::OwnerSlot::new(0),
                }],
                config,
                false,
            )
            .map_err(|_| "current submission rejected")?;
        drop(obsolete);
        assert_eq!(current.recv()??.sources().len(), 1);
        let invalid = worker
            .submit(Vec::new(), config, false)
            .map_err(|_| "invalid submission rejected")?;
        let error = invalid.recv()?.err().ok_or("empty cook accepted")?;
        assert_eq!(
            error.downcast_ref::<GeometryError>(),
            Some(&GeometryError::EmptyGeometry)
        );
        Ok(())
    }
}
