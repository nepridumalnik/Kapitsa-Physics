//! Shared cooked vessel shapes bound to body clusters.
//!
//! Cooking and binding are explicit, fallible operations outside the physics
//! step. A binding is validated against the live world (cluster membership,
//! part meshes, transforms, preparation configuration and cavity policy) so an
//! edited body can never evaluate another body's stale geometry. Evaluation
//! re-validates the binding every step and falls back to body-local
//! preparation on mismatch.

use crate::aerodynamics::shape::worker::{PendingAero, PendingShape};
use std::collections::HashMap;
use std::sync::mpsc::TryRecvError;

pub(super) struct PendingBodyShape {
    parts: Vec<PartId>,
    welds: Vec<super::WeldId>,
    policy: PendingAero,
    result: Result<PendingShape, crate::Error>,
    completion: Option<BodyShapeCompletion>,
}

pub(super) struct ManagedBodyShape {
    policy: PendingAero,
    worker: crate::aerodynamics::shape::worker::ShapeWorker,
}

/// Sends the outcome once the world publishes or rejects a background result.
pub type BodyShapeCompletion = std::sync::mpsc::Sender<Result<ShapeId, crate::Error>>;

/// Status of background preparation for an assembly.
#[derive(Clone, Debug, PartialEq)]
pub enum BodyShapeStatus {
    /// No active cook or valid installed binding.
    Unmanaged,
    /// The latest request is awaiting publication.
    Cooking,
    /// The installed shape matches the assembly.
    Ready(ShapeId),
    /// The latest request failed; the original cause is preserved.
    Failed(crate::Error),
}

pub(super) fn suppress_pending_shape(
    pending: Option<&HashMap<BodyId, PendingBodyShape>>,
    body: BodyId,
    previous_is_current: impl FnOnce() -> bool,
) -> bool {
    pending
        .and_then(|jobs| jobs.get(&body))
        .is_some_and(|job| job.policy != PendingAero::KeepPrevious || !previous_is_current())
}

use super::{BodyEntry, BodyId, ClusterError, PartId, PhysicsWorld};
use crate::aerodynamics::shape::{
    CookedShape, CookedSource, ShapeBinding, ShapeId, ShapeRegistry, ShapeRegistryError,
    assembly_pressure_config,
};
use crate::cluster::ClusterPart;
use crate::containers::DensePool;

/// Resolves the validated shared shape for `body_id`, if one is bound and
/// still matches the live geometry.
///
/// Takes the individual world fields so callers can borrow the assembly cache
/// mutably at the same time. Returns `None` on any mismatch: evaluation then
/// uses body-local preparation, which is the same physics without sharing.
pub(super) fn resolve_shared_shape<'a>(
    shapes: &'a ShapeRegistry,
    bindings: Option<&'a HashMap<BodyId, ShapeBinding>>,
    parts: &DensePool<ClusterPart>,
    geometry: Option<&HashMap<PartId, super::assembly_aerodynamics::PartGeometry>>,
    settings: (bool, Option<&HashMap<BodyId, PendingBodyShape>>),
    body_id: BodyId,
    entry: &BodyEntry,
) -> Option<(&'a ShapeBinding, &'a CookedShape)> {
    let binding = bindings?.get(&body_id)?;
    let shape = shapes.get(binding.shape())?;
    let (oriented_cavities, pending) = settings;
    if let Some(job) = pending.and_then(|jobs| jobs.get(&body_id))
        && entry
            .cluster
            .as_ref()
            .is_none_or(|cluster| cluster.welds() != job.welds)
    {
        return None;
    }
    if let Some(job) = pending.and_then(|jobs| jobs.get(&body_id))
        && job.policy == PendingAero::KeepPrevious
        && let Some(cluster) = &entry.cluster
        && cluster.welds() == job.welds
        && cluster.parts().len() == binding.parts().len()
        && shape.revision() == binding.revision()
        && shape.oriented_cavities() == oriented_cavities
        && binding.parts().iter().all(|part| {
            cluster.parts().contains(part)
                && matches!(
                    geometry.and_then(|items| items.get(part)),
                    Some(super::assembly_aerodynamics::PartGeometry::Mesh(_))
                )
        })
    {
        return Some((binding, shape));
    }
    super::assembly_aerodynamics::binding_matches(
        body_id,
        entry,
        parts,
        geometry,
        oriented_cavities,
        shape,
        binding,
    )
    .ok()?;
    Some((binding, shape))
}

impl PhysicsWorld {
    /// Starts background preparation and automatically queues a replacement
    /// at a full step boundary when the installed binding becomes invalid.
    pub fn manage_body_shape(
        &mut self,
        body: BodyId,
        policy: PendingAero,
        worker: &crate::aerodynamics::shape::worker::ShapeWorker,
    ) -> Result<(), crate::Error> {
        self.manage_body_shape_with_completion(body, policy, worker, None)
    }

    /// Enables automatic recooking and reports the initial publication outcome.
    pub fn manage_body_shape_with_completion(
        &mut self,
        body: BodyId,
        policy: PendingAero,
        worker: &crate::aerodynamics::shape::worker::ShapeWorker,
        completion: Option<BodyShapeCompletion>,
    ) -> Result<(), crate::Error> {
        self.queue_body_shape_with_completion(body, policy, worker, completion)?;
        self.managed_shapes.get_or_insert_with(HashMap::new).insert(
            body,
            ManagedBodyShape {
                policy,
                worker: worker.clone(),
            },
        );
        Ok(())
    }

    /// Disables automatic preparation and cancels any pending request.
    pub fn stop_managing_body_shape(&mut self, body: BodyId) {
        if let Some(managed) = &mut self.managed_shapes {
            managed.remove(&body);
        }
        self.cancel_body_shape(body);
    }

    fn schedule_managed_shapes(&mut self) -> Result<(), crate::Error> {
        let Some(managed) = self.managed_shapes.take() else {
            return Ok(());
        };
        let result = managed.iter().try_for_each(|(&body, control)| {
            if self
                .pending_shapes
                .as_ref()
                .is_some_and(|jobs| jobs.contains_key(&body))
                || matches!(self.body_shape_status(body), BodyShapeStatus::Ready(_))
            {
                return Ok(());
            }
            self.queue_body_shape(body, control.policy, &control.worker)
                .map_err(|cause| {
                    crate::Error::from(super::StepError::Aerodynamics {
                        body_id: body,
                        cause,
                    })
                })
        });
        self.managed_shapes = Some(managed);
        result
    }
    /// Queues preparation of the assembly's current explicit meshes.
    pub fn queue_body_shape(
        &mut self,
        body: BodyId,
        policy: PendingAero,
        worker: &crate::aerodynamics::shape::worker::ShapeWorker,
    ) -> Result<(), crate::Error> {
        self.queue_body_shape_with_completion(body, policy, worker, None)
    }

    /// Queues cooking and optionally reports its publication outcome.
    /// Cancellation or supersession disconnects the completion channel.
    pub fn queue_body_shape_with_completion(
        &mut self,
        body: BodyId,
        policy: PendingAero,
        worker: &crate::aerodynamics::shape::worker::ShapeWorker,
        completion: Option<BodyShapeCompletion>,
    ) -> Result<(), crate::Error> {
        let parts = self
            .cluster(body)
            .ok_or(ClusterError::NotClustered)?
            .parts()
            .to_vec();
        let sources = parts
            .iter()
            .enumerate()
            .map(|(index, &part)| {
                Ok(CookedSource {
                    mesh: self
                        .part_aerodynamic_mesh_arc(part)
                        .ok_or(ShapeRegistryError::UnsupportedSource)?
                        .clone(),
                    local_transform: self
                        .cluster_part(part)
                        .ok_or(ClusterError::InvalidPartId)?
                        .local_transform(),
                    slot: crate::aerodynamics::geometry::OwnerSlot::new(
                        u32::try_from(index).map_err(|_| ShapeRegistryError::InvalidBinding)?,
                    ),
                })
            })
            .collect::<Result<Vec<_>, crate::Error>>()?;
        let cook = worker
            .submit(
                sources,
                assembly_pressure_config(),
                self.preserve_assembly_mesh_cavities,
            )
            .map_err(|error| crate::Error::from_source(crate::ErrorKind::Aerodynamics, error))?;
        self.track_body_shape(body, parts, policy, cook)?;
        if let Some(job) = self
            .pending_shapes
            .as_mut()
            .and_then(|jobs| jobs.get_mut(&body))
        {
            job.completion = completion;
        }
        Ok(())
    }
    /// Tracks an already queued cook for this assembly, superseding any older request.
    pub fn track_body_shape(
        &mut self,
        body: BodyId,
        parts: Vec<PartId>,
        policy: PendingAero,
        cook: PendingShape,
    ) -> Result<(), crate::Error> {
        let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
        if parts.len() != cluster.parts().len()
            || !cluster.parts().iter().all(|part| parts.contains(part))
        {
            return Err(ShapeRegistryError::InvalidBinding.into());
        }
        let welds = cluster.welds().to_vec();
        self.pending_shapes.get_or_insert_with(HashMap::new).insert(
            body,
            PendingBodyShape {
                parts,
                welds,
                policy,
                result: Ok(cook),
                completion: None,
            },
        );
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(())
    }

    /// Publishes completed work without advancing the simulation. A pending
    /// `BlockStep` request is only waited on by the physics step, not here.
    pub fn poll_body_shapes(&mut self) {
        self.collect_pending_shapes(false);
    }

    /// Stops tracking the latest request without changing the installed binding.
    /// An in-flight result for that request can no longer be published.
    pub fn cancel_body_shape(&mut self, body: BodyId) -> bool {
        let cancelled = self
            .pending_shapes
            .as_mut()
            .is_some_and(|jobs| jobs.remove(&body).is_some());
        if cancelled {
            self.body_revision = self.body_revision.wrapping_add(1);
        }
        cancelled
    }

    /// Returns the state of the latest request, or of the installed binding.
    pub fn body_shape_status(&self, body: BodyId) -> BodyShapeStatus {
        if let Some(job) = self
            .pending_shapes
            .as_ref()
            .and_then(|jobs| jobs.get(&body))
        {
            return match &job.result {
                Ok(_) => BodyShapeStatus::Cooking,
                Err(error) => BodyShapeStatus::Failed(error.clone()),
            };
        }
        self.body_entry_for_shape_status(body)
            .and_then(|entry| self.shared_shape(body, entry))
            .map_or(BodyShapeStatus::Unmanaged, |(binding, _)| {
                BodyShapeStatus::Ready(binding.shape())
            })
    }

    fn body_entry_for_shape_status(&self, body: BodyId) -> Option<&BodyEntry> {
        if body.world != self.world_id {
            return None;
        }
        self.bodies.get(body.dense)
    }

    pub(super) fn finish_pending_shapes(&mut self) -> Result<(), crate::Error> {
        self.schedule_managed_shapes()?;
        self.collect_pending_shapes(true);
        // Choose a stable error when multiple requests fail at one boundary.
        if let Some((body_id, cause)) = self
            .pending_shapes
            .iter()
            .flat_map(|jobs| jobs.iter())
            .filter(|(_, job)| job.policy == PendingAero::BlockStep)
            .filter_map(|(&body, job)| job.result.as_ref().err().map(|error| (body, error)))
            .min_by_key(|(body, _)| *body)
        {
            return Err(super::StepError::Aerodynamics {
                body_id,
                cause: cause.clone(),
            }
            .into());
        }
        Ok(())
    }

    fn collect_pending_shapes(&mut self, wait: bool) {
        let Some(mut pending) = self.pending_shapes.take() else {
            return;
        };
        pending.retain(|&body, job| {
            let Some(cluster) = self.cluster(body) else {
                return false;
            };
            if cluster.welds() != job.welds || cluster.parts().len() != job.parts.len() {
                let error = crate::Error::from(ShapeRegistryError::StaleBinding);
                if let Some(completion) = job.completion.take() {
                    let _ = completion.send(Err(error.clone()));
                }
                job.result = Err(error);
                return true;
            }
            let Ok(cook) = &mut job.result else {
                return true;
            };
            let result = if wait && job.policy == PendingAero::BlockStep {
                cook.wait()
            } else {
                cook.try_recv()
            };
            let result = match result {
                Ok(result) => result,
                Err(TryRecvError::Empty) => return true,
                Err(error @ TryRecvError::Disconnected) => Err(crate::Error::from_source(
                    crate::ErrorKind::Aerodynamics,
                    error,
                )),
            };
            let outcome =
                result.and_then(|shape| self.publish_body_shape(body, shape, job.parts.clone()));
            if let Some(completion) = job.completion.take() {
                let _ = completion.send(outcome.clone());
            }
            match outcome {
                Ok(_) => false,
                Err(error) => {
                    job.result = Err(error);
                    true
                }
            }
        });
        self.pending_shapes = Some(pending);
    }

    /// Failure of the latest managed cook, if it failed or became stale.
    pub fn body_shape_failure(&self, body: BodyId) -> Option<&crate::Error> {
        self.pending_shapes
            .as_ref()?
            .get(&body)?
            .result
            .as_ref()
            .err()
    }
    /// Cooked shapes owned by this world.
    pub fn shape_registry(&self) -> &ShapeRegistry {
        &self.shapes
    }

    /// Cooks `sources` with the assembly pressure configuration and the
    /// world's current oriented-cavity policy, then registers the shape.
    ///
    /// The shape is body-independent: cook once and bind it to several
    /// vessels with [`PhysicsWorld::bind_shape`]. Failure leaves the registry
    /// unchanged.
    pub fn cook_shape(
        &mut self,
        sources: &[CookedSource],
        oriented_cavities: bool,
    ) -> Result<ShapeId, crate::Error> {
        if oriented_cavities != self.preserve_assembly_mesh_cavities {
            return Err(crate::Error::from(ShapeRegistryError::InvalidBinding));
        }
        self.shapes
            .cook(sources, assembly_pressure_config(), oriented_cavities)
    }

    /// Re-cooks `sources` and atomically replaces the shape behind `id`,
    /// keeping its identity and advancing its revision. The expected revision
    /// must match the registry before any cooking is performed.
    ///
    /// Existing bindings become stale: they no longer match the new revision,
    /// so evaluation falls back to body-local preparation until the body is
    /// re-bound. A failed cook or stale/foreign ID leaves the registry
    /// unchanged. Returns the new revision.
    pub fn replace_shape(
        &mut self,
        id: ShapeId,
        expected_revision: u64,
        sources: &[CookedSource],
    ) -> Result<u64, crate::Error> {
        let current = self.shapes.get(id).ok_or(ShapeRegistryError::StaleShape)?;
        if current.revision() != expected_revision {
            return Err(ShapeRegistryError::StaleBinding.into());
        }
        let shape = CookedShape::cook(
            sources,
            assembly_pressure_config(),
            self.preserve_assembly_mesh_cavities,
        )?;
        self.publish_shape_replacement(id, expected_revision, shape)
    }

    /// Installs a shape prepared outside the world, if the target revision and
    /// assembly preparation settings still match. Rejection changes nothing.
    pub fn publish_shape_replacement(
        &mut self,
        id: ShapeId,
        expected_revision: u64,
        shape: CookedShape,
    ) -> Result<u64, crate::Error> {
        let current = self.shapes.get(id).ok_or(ShapeRegistryError::StaleShape)?;
        if current.revision() != expected_revision {
            return Err(ShapeRegistryError::StaleBinding.into());
        }
        if shape.config() != assembly_pressure_config()
            || shape.oriented_cavities() != self.preserve_assembly_mesh_cavities
        {
            return Err(ShapeRegistryError::InvalidBinding.into());
        }
        self.shapes.replace(id, shape)
    }

    /// Binds `shape` to `body`, mapping slot `i` to `parts[i]`.
    ///
    /// The body must be clustered, `parts` must cover exactly the cluster's
    /// membership in cook order without duplicates, every part must still own
    /// the mesh it was cooked from with an identical local transform, and the
    /// shape must match the assembly pressure configuration and cavity policy.
    /// Any mismatch is rejected before the previous binding is replaced.
    pub fn bind_shape(
        &mut self,
        body: BodyId,
        shape: ShapeId,
        parts: Vec<PartId>,
    ) -> Result<(), crate::Error> {
        if body.world != self.world_id {
            return Err(crate::Error::from(ClusterError::InvalidBodyId));
        }
        let entry = self
            .bodies
            .get(body.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?;
        let cooked = self
            .shapes
            .get(shape)
            .ok_or_else(|| crate::Error::from(ShapeRegistryError::StaleShape))?;
        let binding = ShapeBinding::new(shape, cooked.revision(), parts)?;
        super::assembly_aerodynamics::binding_matches(
            body,
            entry,
            &self.parts,
            self.part_aerodynamic_geometry.as_ref(),
            self.preserve_assembly_mesh_cavities,
            cooked,
            &binding,
        )?;
        self.shape_bindings
            .get_or_insert_with(HashMap::new)
            .insert(body, binding);
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        Ok(())
    }

    /// Installs and binds an off-thread result as a single operation.
    /// The supplied slots must still match the live assembly, including mesh
    /// identity and transforms. A rejected result leaves the old binding intact.
    pub fn publish_body_shape(
        &mut self,
        body: BodyId,
        cooked: CookedShape,
        parts: Vec<PartId>,
    ) -> Result<ShapeId, crate::Error> {
        // Registry insertion is private until this exclusive world operation
        // returns. Roll back the unpublished ID if binding validation fails.
        let id = self.shapes.insert(cooked)?;
        if let Err(error) = self.bind_shape(body, id, parts) {
            self.shapes.remove(id);
            return Err(error);
        }
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(id)
    }

    /// Removes a body's binding, returning it. IDs from another world and
    /// bodies without a binding yield `None`.
    pub fn unbind_shape(&mut self, body: BodyId) -> Option<ShapeBinding> {
        if body.world != self.world_id {
            return None;
        }
        self.shape_bindings.as_mut()?.remove(&body)
    }

    /// Binding currently associated with `body`, whether or not it still
    /// matches the live geometry.
    pub fn shape_binding(&self, body: BodyId) -> Option<&ShapeBinding> {
        if body.world != self.world_id {
            return None;
        }
        self.shape_bindings.as_ref()?.get(&body)
    }

    /// Validated shared shape for `body`, or `None` when it has no binding or
    /// the binding no longer matches the live geometry.
    pub(super) fn shared_shape<'a>(
        &'a self,
        body: BodyId,
        entry: &BodyEntry,
    ) -> Option<(&'a ShapeBinding, &'a CookedShape)> {
        resolve_shared_shape(
            &self.shapes,
            self.shape_bindings.as_ref(),
            &self.parts,
            self.part_aerodynamic_geometry.as_ref(),
            (
                self.preserve_assembly_mesh_cavities,
                self.pending_shapes.as_ref(),
            ),
            body,
            entry,
        )
    }

    /// Drops bindings that no longer match the live world.
    ///
    /// Correctness does not depend on this: every evaluation re-validates its
    /// binding and falls back to body-local preparation on mismatch. Pruning
    /// only avoids repeating known-dead checks and frees their part lists. It
    /// allocates nothing unless a stale binding exists.
    pub(super) fn prune_stale_shape_bindings(&mut self) {
        let Some(bindings) = &self.shape_bindings else {
            return;
        };
        let mut stale = Vec::new();
        for (&body, binding) in bindings {
            if self
                .pending_shapes
                .as_ref()
                .is_some_and(|jobs| jobs.contains_key(&body))
            {
                continue;
            }
            let matches = self
                .bodies
                .get(body.dense)
                .map(|entry| {
                    let shape = self.shapes.get(binding.shape());
                    shape.is_some_and(|shape| {
                        super::assembly_aerodynamics::binding_matches(
                            body,
                            entry,
                            &self.parts,
                            self.part_aerodynamic_geometry.as_ref(),
                            self.preserve_assembly_mesh_cavities,
                            shape,
                            binding,
                        )
                        .is_ok()
                    })
                })
                .unwrap_or(false);
            if !matches && stale.try_reserve(1).is_ok() {
                stale.push(body);
            }
        }
        if stale.is_empty() {
            return;
        }
        if let Some(bindings) = &mut self.shape_bindings {
            for body in stale {
                bindings.remove(&body);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::geometry::OwnerSlot;
    use crate::aerodynamics::shape::worker::ShapeWorker;
    use crate::{shape::TriangleMesh, transform::Transform};
    use kapitsa_math::Vec3;
    use std::sync::Arc;

    #[rstest::rstest]
    #[case(false)]
    #[case(true)]
    fn managed_mesh_edit_does_not_cook_on_step_thread(
        #[case] sharded: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(
            crate::aerodynamics::AtmosphericState::new(1.0, Vec3::X * -2000.0)?,
        ));
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        world.set_part_aerodynamic_mesh(part, Some(super::super::shape_binding_tests::cube()?))?;
        let worker = ShapeWorker::new(2)?;
        world.manage_body_shape(body, PendingAero::BlockStep, &worker)?;
        world.finish_pending_shapes()?;
        let installed = world
            .shape_binding(body)
            .ok_or("missing initial shape")?
            .shape();
        world.manage_body_shape(body, PendingAero::ZeroForce, &worker)?;
        world.cancel_body_shape(body);
        // Local preparation would fail the geometry budget and abort this step.
        world.set_part_aerodynamic_mesh(
            part,
            Some(TriangleMesh::new(
                vec![Vec3::ZERO, Vec3::Y * 1e6, Vec3::Z * 1e6],
                vec![[0, 1, 2]],
            )?),
        )?;
        let mut snapshots = [Vec::new()];
        world.fill_step_shards(&mut snapshots)?;
        let snapshot = snapshots[0].first_mut().ok_or("missing snapshot")?;
        snapshot.integrate(1e-8)?;
        assert_eq!(
            snapshot
                .predicted
                .ok_or("missing prediction")?
                .linear_velocity(),
            Vec3::ZERO
        );
        if sharded {
            world.step_with_shards(
                1e-8,
                &mut [Vec::new()],
                &crate::execution::SequentialExecutor,
                1,
            )?;
        } else {
            world.step(1e-8)?;
        }
        assert!(world.owner_mesh_pressure(body).is_none());
        assert!(matches!(
            world.body_shape_status(body),
            BodyShapeStatus::Cooking | BodyShapeStatus::Failed(_)
        ));
        assert_eq!(
            world.shape_binding(body).map(ShapeBinding::shape),
            Some(installed)
        );
        Ok(())
    }

    #[test]
    fn managed_merge_recooks_the_surviving_assembly() -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        let mass = crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?;
        let first = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            mass,
        )?)?;
        let second = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            mass,
        )?)?;
        let a = world.enable_cluster(first)?;
        let b = world.enable_cluster(second)?;
        for part in [a, b] {
            world.set_part_aerodynamic_mesh(
                part,
                Some(super::super::shape_binding_tests::cube()?),
            )?;
        }
        let worker = ShapeWorker::new(2)?;
        world.manage_body_shape(first, PendingAero::BlockStep, &worker)?;
        world.step(1e-8)?;
        let initial = world.shape_binding(first).ok_or("initial binding")?.shape();
        world.merge_clusters(
            first,
            second,
            super::super::shape_binding_tests::merge_weld(),
        )?;
        world.step(1e-8)?;
        let current = world.shape_binding(first).ok_or("merged binding")?;
        assert_ne!(initial, current.shape());
        assert_eq!(current.parts(), &[a, b]);
        Ok(())
    }

    #[rstest::rstest]
    #[case(false)]
    #[case(true)]
    fn block_step_failure_keeps_motion(
        #[case] sharded: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let before = *world.body(body).ok_or("body")?.state();
        let (sender, pending) = PendingShape::test_channel();
        world.track_body_shape(body, vec![part], PendingAero::BlockStep, pending)?;
        drop(sender);
        let result = if sharded {
            world.step_with_shards(
                1e-8,
                &mut [Vec::new()],
                &crate::execution::SequentialExecutor,
                1,
            )
        } else {
            world.step(1e-8)
        };
        let error = result.err().ok_or("failed cook allowed the step")?;
        assert!(
            matches!(error.downcast_ref::<super::super::StepError>(), Some(super::super::StepError::Aerodynamics { body_id, cause })
            if *body_id == body && cause.downcast_ref::<TryRecvError>() == Some(&TryRecvError::Disconnected))
        );
        assert_eq!(*world.body(body).ok_or("body")?.state(), before);
        Ok(())
    }

    #[test]
    fn disconnected_cook_reports_transport_failure() -> Result<(), crate::Error> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let (sender, pending) = PendingShape::test_channel();
        world.track_body_shape(body, vec![part], PendingAero::ZeroForce, pending)?;
        assert_eq!(world.body_shape_status(body), BodyShapeStatus::Cooking);
        drop(sender);
        world.poll_body_shapes();
        assert!(
            matches!(world.body_shape_status(body), BodyShapeStatus::Failed(error)
            if error.downcast_ref::<TryRecvError>() == Some(&TryRecvError::Disconnected))
        );
        assert!(world.shape_registry().is_empty());
        Ok(())
    }

    #[rstest::rstest]
    #[case(false, PendingAero::ZeroForce, false)]
    #[case(true, PendingAero::ZeroForce, false)]
    #[case(false, PendingAero::BlockStep, false)]
    #[case(true, PendingAero::BlockStep, false)]
    #[case(false, PendingAero::KeepPrevious, false)]
    #[case(true, PendingAero::KeepPrevious, false)]
    #[case(false, PendingAero::KeepPrevious, true)]
    #[case(true, PendingAero::KeepPrevious, true)]
    fn pending_policy_controls_pressure(
        #[case] sharded: bool,
        #[case] policy: PendingAero,
        #[case] merge: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(
            crate::aerodynamics::AtmosphericState::new(1.0, Vec3::X * -2000.0)?,
        ));
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let mesh = Arc::new(super::super::shape_binding_tests::cube()?);
        world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(&mesh)))?;
        if policy == PendingAero::KeepPrevious {
            let sources = [CookedSource {
                mesh: Arc::clone(&mesh),
                local_transform: Transform::identity(),
                slot: OwnerSlot::new(0),
            }];
            let previous = world.cook_shape(&sources, false)?;
            world.bind_shape(body, previous, vec![part])?;
        }
        let (sender, mut pending) = PendingShape::test_channel();
        let (entered, wait_started) = std::sync::mpsc::channel();
        if policy == PendingAero::BlockStep {
            pending.on_wait = Some(entered);
        }
        world.track_body_shape(body, vec![part], policy, pending)?;
        let mesh = if policy == PendingAero::KeepPrevious {
            let replacement = Arc::new(super::super::shape_binding_tests::cube()?);
            world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(&replacement)))?;
            replacement
        } else {
            mesh
        };
        let shape = CookedShape::cook(
            &[CookedSource {
                mesh,
                local_transform: Transform::identity(),
                slot: OwnerSlot::new(0),
            }],
            assembly_pressure_config(),
            false,
        )?;
        let mut shape = Some(shape);
        let advance = |world: &mut PhysicsWorld| {
            if sharded {
                world.step_with_shards(
                    1e-8,
                    &mut [Vec::new()],
                    &crate::execution::SequentialExecutor,
                    1,
                )
            } else {
                world.step(1e-8)
            }
        };
        if policy == PendingAero::KeepPrevious && !merge {
            let previous = world.shape_binding(body).ok_or("previous binding")?.shape();
            advance(&mut world)?;
            assert_eq!(
                world.shape_binding(body).map(ShapeBinding::shape),
                Some(previous)
            );
            assert!(world.owner_mesh_pressure(body).is_some());
        }
        if merge {
            let second = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
                Transform::identity(),
                crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
            )?)?;
            world.enable_cluster(second)?;
            world.merge_clusters(
                body,
                second,
                super::super::shape_binding_tests::merge_weld(),
            )?;
            advance(&mut world)?;
            assert!(world.owner_mesh_pressure(body).is_none());
            assert_eq!(
                world
                    .body_shape_failure(body)
                    .and_then(|e| e.downcast_ref::<ShapeRegistryError>()),
                Some(&ShapeRegistryError::StaleBinding)
            );
            return Ok(());
        }
        if policy == PendingAero::ZeroForce {
            let mut snapshots = [Vec::new()];
            world.fill_step_shards(&mut snapshots)?;
            let snapshot = snapshots[0].first_mut().ok_or("missing snapshot")?;
            snapshot.integrate(1e-8)?;
            assert_eq!(
                snapshot
                    .predicted
                    .ok_or("missing prediction")?
                    .linear_velocity(),
                Vec3::ZERO
            );
            advance(&mut world)?;
            assert!(world.owner_mesh_pressure(body).is_none());
            assert_eq!(
                world.body(body).ok_or("body")?.state().linear_velocity(),
                Vec3::ZERO
            );
        }
        let shape = shape.take().ok_or("shape")?;
        if policy == PendingAero::BlockStep {
            let producer = std::thread::spawn(move || {
                wait_started
                    .recv_timeout(std::time::Duration::from_secs(5))
                    .map_err(|_| "step never entered wait")?;
                sender.send(Ok(shape)).map_err(|_| "disconnected")
            });
            let result = advance(&mut world);
            producer.join().map_err(|_| "producer panicked")??;
            result?;
        } else {
            sender.send(Ok(shape)).map_err(|_| "disconnected")?;
            advance(&mut world)?;
        }
        assert!(world.shape_binding(body).is_some());
        assert!(
            world
                .body(body)
                .ok_or("body")?
                .state()
                .linear_velocity()
                .length()
                > 1e-6
        );
        Ok(())
    }

    #[test]
    fn body_publication_preserves_binding_when_mesh_identity_changes()
    -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let make_mesh = || TriangleMesh::new(vec![Vec3::ZERO, Vec3::Y, Vec3::Z], vec![[0, 1, 2]]);
        let mesh = Arc::new(make_mesh()?);
        world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(&mesh)))?;
        let sources = [CookedSource {
            mesh,
            local_transform: Transform::identity(),
            slot: OwnerSlot::new(0),
        }];
        let worker = ShapeWorker::new(2)?;
        let first = worker
            .submit(sources.to_vec(), assembly_pressure_config(), false)?
            .recv()??;
        let id = world.publish_body_shape(body, first, vec![part])?;
        let old = worker.submit(sources.to_vec(), assembly_pressure_config(), false)?;
        world.set_part_aerodynamic_mesh(part, Some(make_mesh()?))?;
        let error = world
            .publish_body_shape(body, old.recv()??, vec![part])
            .err()
            .ok_or("stale mesh accepted")?;
        assert_eq!(
            error.downcast_ref::<ShapeRegistryError>(),
            Some(&ShapeRegistryError::InvalidBinding)
        );
        assert_eq!(world.shape_registry().len(), 1);
        assert_eq!(world.shape_binding(body).map(ShapeBinding::shape), Some(id));
        let (sender, pending) = PendingShape::test_channel();
        world.track_body_shape(body, vec![part], PendingAero::ZeroForce, pending)?;
        sender
            .send(Ok(worker
                .submit(sources.to_vec(), assembly_pressure_config(), false)?
                .recv()??))
            .map_err(|_| "cancel fixture disconnected")?;
        assert!(world.cancel_body_shape(body));
        assert!(!world.cancel_body_shape(body));
        world.poll_body_shapes();
        assert_eq!(world.shape_registry().len(), 1);
        assert_eq!(world.shape_binding(body).map(ShapeBinding::shape), Some(id));
        Ok(())
    }

    #[test]
    fn completed_worker_result_cannot_overwrite_newer_revision()
    -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        let source = CookedSource {
            mesh: Arc::new(TriangleMesh::new(
                vec![Vec3::ZERO, Vec3::Y, Vec3::Z],
                vec![[0, 1, 2]],
            )?),
            local_transform: Transform::identity(),
            slot: OwnerSlot::new(0),
        };
        let id = world.cook_shape(std::slice::from_ref(&source), false)?;
        let worker = ShapeWorker::new(2)?;
        let old = worker
            .submit(vec![source.clone()], assembly_pressure_config(), false)
            .map_err(|_| "old submission rejected")?;
        let new = worker
            .submit(vec![source], assembly_pressure_config(), false)
            .map_err(|_| "new submission rejected")?;
        assert_eq!(world.publish_shape_replacement(id, 1, new.recv()??)?, 2);
        let current = world
            .shape_registry()
            .get(id)
            .ok_or("missing shape")?
            .geometry() as *const _;
        let error = world
            .publish_shape_replacement(id, 1, old.recv()??)
            .err()
            .ok_or("stale result installed")?;
        assert_eq!(
            error.downcast_ref::<ShapeRegistryError>(),
            Some(&ShapeRegistryError::StaleBinding)
        );
        let stored = world.shape_registry().get(id).ok_or("missing shape")?;
        assert_eq!(stored.revision(), 2);
        assert!(std::ptr::eq(current, stored.geometry()));
        Ok(())
    }
}
