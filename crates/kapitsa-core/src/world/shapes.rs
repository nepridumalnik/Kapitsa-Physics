//! Shared cooked vessel shapes bound to body clusters.
//!
//! Cooking and binding are explicit, fallible operations outside the physics
//! step. A binding is validated against the live world (cluster membership,
//! part meshes, transforms, preparation configuration and cavity policy) so an
//! edited body can never evaluate another body's stale geometry. Evaluation
//! re-validates the binding every step and falls back to body-local
//! preparation on mismatch.

use crate::aerodynamics::shape::worker::{PendingAero, PendingShape};
use std::collections::{BTreeMap, HashMap};
use std::sync::mpsc::TryRecvError;

pub(super) struct PendingBodyShape {
    parts: Vec<PartId>,
    welds: Vec<super::WeldId>,
    policy: PendingAero,
    result: Result<PendingShape, crate::Error>,
    completion: Option<BodyShapeCompletion>,
    inputs: Option<(Vec<CookedSource>, bool)>,
    ready: Option<CookedShape>,
    staged: Option<StagedShape>,
}

struct StagedShape {
    id: ShapeId,
    previous: Option<ShapeBinding>,
    checkpoint: crate::aerodynamics::shape::ShapeInsertCheckpoint,
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
    pending: Option<&BTreeMap<BodyId, PendingBodyShape>>,
    body: BodyId,
    previous_is_current: impl FnOnce() -> bool,
) -> bool {
    pending.and_then(|jobs| jobs.get(&body)).is_some_and(|job| {
        job.staged.is_none() && (job.policy != PendingAero::KeepPrevious || !previous_is_current())
    })
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
    settings: (bool, Option<&BTreeMap<BodyId, PendingBodyShape>>),
    body_id: BodyId,
    entry: &BodyEntry,
) -> Option<(&'a ShapeBinding, &'a CookedShape)> {
    let binding = bindings?.get(&body_id)?;
    let shape = shapes.get(binding.shape())?;
    let (oriented_cavities, pending) = settings;
    if let Some(job) = pending
        .and_then(|jobs| jobs.get(&body_id))
        .filter(|job| job.staged.is_none())
        && entry
            .cluster
            .as_ref()
            .is_none_or(|cluster| cluster.welds() != job.welds)
    {
        return None;
    }
    if let Some(job) = pending
        .and_then(|jobs| jobs.get(&body_id))
        .filter(|job| job.staged.is_none())
        && job.policy == PendingAero::KeepPrevious
        && let Some(cluster) = &entry.cluster
        && cluster.welds() == job.welds
        && cluster.parts().len() == binding.parts().len()
        && shape.revision() == binding.revision()
        && shape.oriented_cavities() == oriented_cavities
        && binding.parts().len() == shape.sources().len()
        && binding
            .parts()
            .iter()
            .zip(shape.sources())
            .all(|(part, source)| {
                cluster.parts().contains(part)
                    && parts.get(part.dense).is_some_and(|live| {
                        live.body() == body_id && live.local_transform() == source.local_transform
                    })
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

pub(super) struct CookedPrimaryHull {
    pub(super) geometry: Box<crate::aerodynamics::geometry::PreparedGeometry>,
    identity: std::sync::Arc<()>,
}

impl CookedPrimaryHull {
    fn matches(&self, entry: &BodyEntry) -> bool {
        entry.cluster.is_none()
            && entry
                .body
                .aerodynamic_geometry_identity
                .as_ref()
                .is_some_and(|identity| std::sync::Arc::ptr_eq(identity, &self.identity))
    }
}

pub(super) fn require_cooked_pressure(
    model: crate::aerodynamics::MeshPressureModel,
    body_id: BodyId,
    entry: &BodyEntry,
    geometry: Option<&HashMap<PartId, super::assembly_aerodynamics::PartGeometry>>,
    parts: &DensePool<ClusterPart>,
    has_binding: bool,
) -> Result<(), crate::Error> {
    use super::assembly_aerodynamics::PartGeometry;
    use crate::aerodynamics::{Aerodynamics, MeshPressureModel};
    if model == MeshPressureModel::ModifiedNewtonianTriangleQuadrature
        && (matches!(
            entry.body.aerodynamics(),
            Aerodynamics::Mesh(_) | Aerodynamics::Colliders
        ) && !entry
            .cooked_primary_hull
            .as_ref()
            .is_some_and(|hull| hull.matches(entry))
            || !has_binding && super::assembly_aerodynamics::configured(entry, geometry))
    {
        return Err(super::StepError::Aerodynamics {
            body_id,
            cause: ShapeRegistryError::InvalidBinding.into(),
        }
        .into());
    }
    if model == MeshPressureModel::ModifiedNewtonianTriangleQuadrature
        && has_binding
        && entry.cluster.as_ref().is_some_and(|cluster| {
            cluster.parts().iter().any(|part| {
                let has_wings = parts
                    .get(part.dense)
                    .is_some_and(|live| !live.aerodynamic_wings.is_empty());
                match geometry.and_then(|sources| sources.get(part)) {
                    Some(PartGeometry::Mesh(_)) => has_wings,
                    Some(PartGeometry::WingShieldingMesh(_)) => !has_wings,
                    _ => false,
                }
            })
        })
    {
        return Err(super::StepError::Aerodynamics {
            body_id,
            cause: crate::aerodynamics::AerodynamicsError::UnsupportedModel.into(),
        }
        .into());
    }
    if model == MeshPressureModel::ModifiedNewtonianTriangleQuadrature
        && has_binding
        && let (Some(cluster), Some(geometry)) = (&entry.cluster, geometry)
    {
        use crate::aerodynamics::{geometry::GeometryError, wing::WingError};
        const MAX_OVERLAP_WORK: usize = 1_000_000;
        let mut remaining = MAX_OVERLAP_WORK;
        let context = |cause| crate::Error::from(super::StepError::Aerodynamics { body_id, cause });
        let budget = || context(GeometryError::WorkBudgetExceeded.into());
        for wing_id in cluster.parts() {
            remaining = remaining.checked_sub(1).ok_or_else(budget)?;
            let Some(owner) = parts.get(wing_id.dense) else {
                continue;
            };
            for wing in &owner.aerodynamic_wings {
                remaining = remaining.checked_sub(1).ok_or_else(budget)?;
                if wing.footprint_signature().is_none() {
                    continue;
                }
                let to_wing = owner
                    .local_transform()
                    .compose(&wing.local_transform())
                    .and_then(|t| t.inverse())
                    .map_err(|_| context(WingError::ArithmeticOverflow.into()))?;
                for hull_id in cluster.parts() {
                    remaining = remaining.checked_sub(1).ok_or_else(budget)?;
                    let Some(PartGeometry::Mesh(mesh)) = geometry.get(hull_id) else {
                        continue;
                    };
                    remaining = remaining
                        .checked_sub(mesh.triangles().len())
                        .ok_or_else(budget)?;
                    let hull = parts
                        .get(hull_id.dense)
                        .ok_or_else(|| context(ShapeRegistryError::InvalidBinding.into()))?;
                    let transform = to_wing
                        .compose(&hull.local_transform())
                        .map_err(|_| context(WingError::ArithmeticOverflow.into()))?;
                    for indices in mesh.triangles() {
                        if wing
                            .overlaps_footprint(transform, indices.map(|i| mesh.vertices()[i]))
                            .map_err(context)?
                        {
                            return Err(context(
                                crate::aerodynamics::AerodynamicsError::UnsupportedModel.into(),
                            ));
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

impl PhysicsWorld {
    /// Cooks the standalone body's explicit primary mesh outside the step.
    /// Replacing the mesh or changing the cavity policy requires a new cook.
    pub fn cook_primary_hull(&mut self, body: BodyId) -> Result<(), crate::Error> {
        use crate::aerodynamics::Aerodynamics;
        let entry = self
            .body_entry_for_shape_status(body)
            .ok_or(ClusterError::InvalidBodyId)?;
        let Aerodynamics::Mesh(mesh) = entry.body.aerodynamics() else {
            return Err(ShapeRegistryError::UnsupportedSource.into());
        };
        if entry.cluster.is_some() {
            return Err(ShapeRegistryError::UnsupportedSource.into());
        }
        let identity = entry
            .body
            .aerodynamic_geometry_identity
            .clone()
            .ok_or(ShapeRegistryError::InvalidBinding)?;
        use crate::aerodynamics::geometry::{GeometryInstance, GeometryOwner, PreparedGeometry};
        let instances = [GeometryInstance {
            mesh,
            local_transform: crate::transform::Transform::identity(),
            owner: GeometryOwner::Body,
        }];
        let geometry = if self.preserve_body_mesh_cavities {
            PreparedGeometry::prepare_with_oriented_cavities(
                &instances,
                assembly_pressure_config(),
            )?
        } else {
            PreparedGeometry::prepare(&instances, assembly_pressure_config())?
        };
        let entry = self
            .bodies
            .get_mut(body.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        entry.cooked_primary_hull = Some(CookedPrimaryHull {
            geometry: Box::new(geometry),
            identity,
        });
        Ok(())
    }
    /// Queries exposed physical patches of a currently bound cooked assembly.
    ///
    /// `upstream` points toward oncoming air in the body-local cook frame;
    /// `epsilon` is the exposure distance tolerance in metres. Returned patch
    /// owners are live parts, not cook slots. This opt-in geometry query does
    /// not cook, apply forces, or change the active aerodynamic model. It
    /// rejects missing or stale bindings, including during replacement cooks
    /// when a previous binding no longer matches the live sources.
    pub fn bound_shape_exposure(
        &self,
        body: BodyId,
        upstream: kapitsa_math::Vec3,
        epsilon: f64,
    ) -> Result<crate::aerodynamics::geometry::PreparedSurfacePatches, crate::Error> {
        let entry = self
            .body_entry_for_shape_status(body)
            .ok_or(ClusterError::InvalidBodyId)?;
        let binding = self
            .shape_binding(body)
            .ok_or(ShapeRegistryError::InvalidBinding)?;
        let shape = self
            .shape_registry()
            .get(binding.shape())
            .ok_or(ShapeRegistryError::StaleShape)?;
        super::assembly_aerodynamics::binding_matches(
            body,
            entry,
            &self.parts,
            self.part_aerodynamic_geometry.as_ref(),
            self.preserve_assembly_mesh_cavities,
            shape,
            binding,
        )?;
        let mut instances = Vec::new();
        binding.write_instances(shape, &mut instances)?;
        shape.geometry().exposed_surface_patches(
            &instances,
            upstream,
            epsilon,
            shape.config().max_patch_work,
        )
    }

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
        self.managed_shapes
            .get_or_insert_with(BTreeMap::new)
            .insert(
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
                .and_then(|jobs| jobs.get(&body))
                .is_some_and(|job| self.pending_inputs_match(body, job))
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

    fn pending_inputs_match(&self, body: BodyId, job: &PendingBodyShape) -> bool {
        let Some((sources, cavities)) = &job.inputs else {
            return true;
        };
        *cavities == self.preserve_assembly_mesh_cavities
            && self
                .cluster(body)
                .is_some_and(|cluster| cluster.parts() == job.parts && cluster.welds() == job.welds)
            && job.parts.iter().zip(sources).all(|(&part, source)| {
                self.part_aerodynamic_mesh_arc(part)
                    .is_some_and(|mesh| std::sync::Arc::ptr_eq(mesh, &source.mesh))
                    && self
                        .cluster_part(part)
                        .is_some_and(|entry| entry.local_transform() == source.local_transform)
            })
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
                sources.clone(),
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
            job.inputs = Some((sources, self.preserve_assembly_mesh_cavities));
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
        self.pending_shapes
            .get_or_insert_with(BTreeMap::new)
            .insert(
                body,
                PendingBodyShape {
                    parts,
                    welds,
                    policy,
                    result: Ok(cook),
                    completion: None,
                    inputs: None,
                    ready: None,
                    staged: None,
                },
            );
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(())
    }

    /// Publishes completed work without advancing the simulation. A pending
    /// `BlockStep` request is only waited on by the physics step, not here.
    pub fn poll_body_shapes(&mut self) {
        self.collect_pending_shapes(false, false);
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
        self.collect_pending_shapes(true, true);
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

    fn collect_pending_shapes(&mut self, wait: bool, stage: bool) {
        let Some(mut pending) = self.pending_shapes.take() else {
            return;
        };
        pending.retain(|&body, job| {
            if job.staged.is_some() {
                return true;
            }
            let Some(cluster) = self.cluster(body) else {
                return false;
            };
            if cluster.welds() != job.welds
                || cluster.parts().len() != job.parts.len()
                || !self.pending_inputs_match(body, job)
            {
                let error = crate::Error::from(ShapeRegistryError::StaleBinding);
                if let Some(completion) = job.completion.take() {
                    let _ = completion.send(Err(error.clone()));
                }
                job.result = Err(error);
                return true;
            }
            let result = if let Some(shape) = job.ready.take() {
                Ok(Ok(shape))
            } else if let Ok(cook) = &mut job.result {
                if wait && job.policy == PendingAero::BlockStep {
                    cook.wait()
                } else {
                    cook.try_recv()
                }
            } else {
                return true;
            };
            let result = match result {
                Ok(result) => result,
                Err(TryRecvError::Empty) => return true,
                Err(error @ TryRecvError::Disconnected) => Err(crate::Error::from_source(
                    crate::ErrorKind::Aerodynamics,
                    error,
                )),
            };
            let previous = self
                .shape_bindings
                .as_ref()
                .and_then(|bindings| bindings.get(&body))
                .cloned();
            let checkpoint = stage.then(|| self.shapes.insert_checkpoint());
            let outcome =
                result.and_then(|shape| self.publish_body_shape(body, shape, job.parts.clone()));
            if let (Some(checkpoint), Ok(id)) = (checkpoint, &outcome) {
                job.staged = Some(StagedShape {
                    id: *id,
                    previous,
                    checkpoint,
                });
                return true;
            }
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

    pub(super) fn commit_pending_shapes(&mut self) {
        if let Some(pending) = &mut self.pending_shapes {
            pending.retain(|_, job| {
                let Some(staged) = job.staged.take() else {
                    return true;
                };
                if let Some(completion) = job.completion.take() {
                    let _ = completion.send(Ok(staged.id));
                }
                false
            });
        }
    }

    pub(super) fn rollback_pending_shapes(&mut self) -> Result<(), crate::Error> {
        if let Some(pending) = &mut self.pending_shapes {
            for (body, job) in pending.iter_mut().rev() {
                let Some(staged) = job.staged.take() else {
                    continue;
                };
                job.ready = Some(
                    self.shapes
                        .undo_insertion(staged.id, staged.checkpoint)
                        .ok_or(super::StepError::InvalidStepResults)?,
                );
                self.body_revision = self.body_revision.wrapping_sub(1);
                if let Some(bindings) = &mut self.shape_bindings {
                    if let Some(previous) = staged.previous {
                        bindings.insert(*body, previous);
                    } else {
                        bindings.remove(body);
                    }
                }
            }
        }
        Ok(())
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
        let checkpoint = self.shapes.insert_checkpoint();
        let id = self.shapes.insert(cooked)?;
        if let Err(error) = self.bind_shape(body, id, parts) {
            self.shapes
                .undo_insertion(id, checkpoint)
                .ok_or(super::StepError::InvalidStepResults)?;
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

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "Run alone: process-wide allocation counter"]
    fn pending_shape_status_and_collection_do_not_allocate() -> Result<(), crate::Error> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let (producer, pending) = PendingShape::test_channel();
        world.track_body_shape(body, vec![part], PendingAero::ZeroForce, pending)?;
        world.finish_pending_shapes()?;
        let counts = super::super::tests::tracked_world_allocations(|| {
            for _ in 0..128 {
                assert!(world.finish_pending_shapes().is_ok());
                assert_eq!(world.body_shape_status(body), BodyShapeStatus::Cooking);
            }
        });
        drop(producer);
        println!("128 pending checks: {counts:?} (allocations, bytes)");
        assert_eq!(counts, (0, 0));
        Ok(())
    }

    #[test]
    fn polling_rejects_edited_inputs_before_the_worker_finishes()
    -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let cube = super::super::shape_binding_tests::cube;
        world.set_part_aerodynamic_mesh(part, Some(cube()?))?;
        let worker = ShapeWorker::new(2)?;
        let (completion, outcome) = std::sync::mpsc::channel();
        world.queue_body_shape_with_completion(
            body,
            PendingAero::BlockStep,
            &worker,
            Some(completion),
        )?;
        let (producer, pending) = PendingShape::test_channel();
        world
            .pending_shapes
            .as_mut()
            .and_then(|jobs| jobs.get_mut(&body))
            .ok_or("missing queued job")?
            .result = Ok(pending);
        world.set_part_aerodynamic_mesh(part, Some(cube()?))?;
        world.poll_body_shapes();
        let expected = crate::Error::from(ShapeRegistryError::StaleBinding);
        assert_eq!(outcome.try_recv()?, Err(expected.clone()));
        assert_eq!(
            world.body_shape_status(body),
            BodyShapeStatus::Failed(expected.clone())
        );
        assert!(producer.send(Err(expected)).is_err());
        assert!(world.shape_registry().is_empty());
        Ok(())
    }

    #[rstest::rstest]
    #[case(false)]
    #[case(true)]
    fn splitting_assembly_rejects_pending_cook_for_old_topology(
        #[case] sharded: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        use super::super::shape_binding_tests::{cube, merge_weld};
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(
            crate::aerodynamics::AtmosphericState::new(1.0, -Vec3::X * 1500.0)?,
        ));
        world.set_mesh_pressure_model(
            crate::aerodynamics::MeshPressureModel::ModifiedNewtonianTriangleQuadrature,
        );
        let mut bodies = Vec::new();
        let mut parts = Vec::new();
        let mesh = Arc::new(cube()?);
        for _ in 0..2 {
            let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
                Transform::identity(),
                crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
            )?)?;
            let part = world.enable_cluster(body)?;
            world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(&mesh)))?;
            bodies.push(body);
            parts.push(part);
        }
        let body = bodies[0];
        world.merge_clusters(body, bodies[1], merge_weld())?;
        let weld = *world
            .cluster(body)
            .and_then(|cluster| cluster.welds().first())
            .ok_or("missing weld")?;
        let sources = [0, 1].map(|slot| CookedSource {
            mesh: Arc::clone(&mesh),
            local_transform: Transform::identity(),
            slot: OwnerSlot::new(slot),
        });
        let previous = world.cook_shape(&sources, false)?;
        world.bind_shape(body, previous, parts.clone())?;
        let replacement = CookedShape::cook(&sources, assembly_pressure_config(), false)?;
        let (sender, pending) = PendingShape::test_channel();
        world.track_body_shape(body, parts, PendingAero::KeepPrevious, pending)?;
        let step = |world: &mut PhysicsWorld| {
            if sharded {
                world.step_with_shards(
                    1e-8,
                    &mut [Vec::new(), Vec::new()],
                    &crate::execution::SequentialExecutor,
                    2,
                )
            } else {
                world.step(1e-8)
            }
        };
        step(&mut world)?;
        assert!(world.owner_mesh_pressure(body).is_some());
        let split = world.break_weld(weld)?;
        assert_eq!(split.len(), 2);
        sender
            .send(Ok(replacement))
            .map_err(|_| "pending receiver closed before publication")?;
        let before = split
            .iter()
            .map(|&id| world.body(id).map(|body| body.state().linear_velocity()))
            .collect::<Vec<_>>();
        // The detached body is unmanaged and has no prepared hull yet. The
        // prepared-only model must reject the step rather than cook it inline.
        let detached = *split
            .iter()
            .find(|&&id| id != body)
            .ok_or("detached body")?;
        let error = step(&mut world)
            .err()
            .ok_or("unprepared hull allowed step")?;
        assert!(matches!(
            error.downcast_ref::<super::super::StepError>(),
            Some(super::super::StepError::Aerodynamics { body_id, cause })
                if *body_id == detached
                    && cause.downcast_ref::<ShapeRegistryError>() == Some(&ShapeRegistryError::InvalidBinding)
        ));
        assert_eq!(world.shape_registry().len(), 1);
        assert_eq!(
            world
                .body_shape_failure(body)
                .and_then(|e| e.downcast_ref::<ShapeRegistryError>()),
            Some(&ShapeRegistryError::StaleBinding)
        );
        for (body, velocity) in split.into_iter().zip(before) {
            assert!(world.bound_shape_exposure(body, Vec3::X, 0.01).is_err());
            assert_eq!(
                world
                    .body(body)
                    .ok_or("split body")?
                    .state()
                    .linear_velocity(),
                velocity.ok_or("pre-step velocity")?
            );
        }
        Ok(())
    }

    #[rstest::rstest]
    #[case(false)]
    #[case(true)]
    fn removing_body_discards_ready_unpublished_cook(
        #[case] sharded: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let mesh = Arc::new(super::super::shape_binding_tests::cube()?);
        world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(&mesh)))?;
        let shape = CookedShape::cook(
            &[CookedSource {
                mesh,
                local_transform: Transform::identity(),
                slot: OwnerSlot::new(0),
            }],
            assembly_pressure_config(),
            false,
        )?;
        let (sender, pending) = PendingShape::test_channel();
        world.track_body_shape(body, vec![part], PendingAero::KeepPrevious, pending)?;
        sender
            .send(Ok(shape))
            .map_err(|_| "cook receiver closed early")?;
        world.remove_body(body).ok_or("body removal failed")?;
        let replacement = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
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
        assert!(world.shape_registry().is_empty());
        assert!(world.shape_binding(replacement).is_none());
        assert!(world.owner_mesh_pressure(replacement).is_none());
        assert_eq!(world.body_shape_status(body), BodyShapeStatus::Unmanaged);
        Ok(())
    }

    #[test]
    fn cancelling_ready_result_preserves_live_binding_and_motion()
    -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let mesh = Arc::new(super::super::shape_binding_tests::cube()?);
        world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(&mesh)))?;
        let sources = [CookedSource {
            mesh,
            local_transform: Transform::identity(),
            slot: OwnerSlot::new(0),
        }];
        let id = world.cook_shape(&sources, false)?;
        world.bind_shape(body, id, vec![part])?;
        let before = *world.body(body).ok_or("missing body")?.state();
        let (sender, pending) = PendingShape::test_channel();
        world.track_body_shape(body, vec![part], PendingAero::BlockStep, pending)?;
        sender
            .send(Ok(CookedShape::cook(
                &sources,
                assembly_pressure_config(),
                false,
            )?))
            .map_err(|_| "pending channel disconnected")?;
        assert!(world.cancel_body_shape(body));
        world.poll_body_shapes();
        assert_eq!(world.body_shape_status(body), BodyShapeStatus::Ready(id));
        assert_eq!(world.shape_registry().len(), 1);
        assert_eq!(*world.body(body).ok_or("missing body")?.state(), before);
        Ok(())
    }

    #[rstest::rstest]
    #[case(false, false)]
    #[case(false, true)]
    #[case(true, false)]
    #[case(true, true)]
    fn failed_step_does_not_publish_a_ready_aerodynamic_shape(
        #[case] sharded: bool,
        #[case] has_previous: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(
            crate::aerodynamics::AtmosphericState::new(1.0, Vec3::ZERO)?,
        ));
        let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
            Transform::identity(),
            crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?,
        )?)?;
        let part = world.enable_cluster(body)?;
        let mesh = Arc::new(super::super::shape_binding_tests::cube()?);
        world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(&mesh)))?;
        let sources = [CookedSource {
            mesh,
            local_transform: Transform::identity(),
            slot: OwnerSlot::new(0),
        }];
        let previous = if has_previous {
            let id = world.cook_shape(&sources, false)?;
            world.bind_shape(body, id, vec![part])?;
            Some(id)
        } else {
            None
        };
        let (sender, pending) = PendingShape::test_channel();
        world.track_body_shape(body, vec![part], PendingAero::BlockStep, pending)?;
        let (completion, result) = std::sync::mpsc::channel();
        world
            .pending_shapes
            .as_mut()
            .and_then(|jobs| jobs.get_mut(&body))
            .ok_or("missing job")?
            .completion = Some(completion);
        sender
            .send(Ok(CookedShape::cook(
                &sources,
                assembly_pressure_config(),
                false,
            )?))
            .map_err(|_| "pending channel disconnected")?;
        world.set_central_gravity(Some(crate::gravity::CentralGravity::new(Vec3::ZERO, 1.0)?));
        let original_revision = world.body_revision;

        let mut shards = vec![Vec::new(); 2];
        let step = if sharded {
            world.step_with_shards(0.01, &mut shards, &crate::execution::SequentialExecutor, 2)
        } else {
            world.step(0.01)
        };
        let error = step.err().ok_or("step unexpectedly succeeded")?;
        assert!(matches!(
            error.downcast_ref::<super::super::StepError>(),
            Some(super::super::StepError::Gravity { .. })
        ));
        assert_eq!(
            world.shape_binding(body).map(|binding| binding.shape()),
            previous
        );
        assert_eq!(world.shape_registry().len(), usize::from(has_previous));
        assert_eq!(world.body_revision, original_revision);
        assert!(matches!(result.try_recv(), Err(TryRecvError::Empty)));
        world.set_central_gravity(None);
        if sharded {
            world.step_with_shards(0.01, &mut shards, &crate::execution::SequentialExecutor, 2)?;
        } else {
            world.step(0.01)?;
        }
        let id = result.try_recv()??;
        assert_eq!(
            world.shape_binding(body).map(|binding| binding.shape()),
            Some(id)
        );
        assert_eq!(world.body_shape_status(body), BodyShapeStatus::Ready(id));
        Ok(())
    }

    #[test]
    fn failure_restores_multiple_staged_shapes_in_reverse_order()
    -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(
            crate::aerodynamics::AtmosphericState::new(1.0, Vec3::ZERO)?,
        ));
        let mesh = Arc::new(super::super::shape_binding_tests::cube()?);
        let mass = crate::mass::MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)?;
        let mut bodies = Vec::new();
        for position in [Vec3::ZERO, Vec3::Y * 3.0] {
            let body = world.add_body(crate::rigid_body::RigidBody::new_dynamic(
                Transform::new(position, kapitsa_math::Quat::IDENTITY)?,
                mass,
            )?)?;
            let part = world.enable_cluster(body)?;
            world.set_part_aerodynamic_mesh_shared(part, Some(Arc::clone(&mesh)))?;
            let sources = [CookedSource {
                mesh: Arc::clone(&mesh),
                local_transform: Transform::identity(),
                slot: OwnerSlot::new(0),
            }];
            let (sender, pending) = PendingShape::test_channel();
            world.track_body_shape(body, vec![part], PendingAero::BlockStep, pending)?;
            sender
                .send(Ok(CookedShape::cook(
                    &sources,
                    assembly_pressure_config(),
                    false,
                )?))
                .map_err(|_| "pending channel disconnected")?;
            bodies.push(body);
        }
        world.set_central_gravity(Some(crate::gravity::CentralGravity::new(Vec3::ZERO, 1.0)?));
        assert!(world.step(0.01).is_err());
        assert!(world.shape_registry().is_empty());
        assert!(
            bodies
                .iter()
                .all(|body| world.shape_binding(*body).is_none())
        );
        world.set_central_gravity(None);
        world.step(0.01)?;
        assert_eq!(world.shape_registry().len(), bodies.len());
        assert!(
            bodies
                .iter()
                .all(|body| world.shape_binding(*body).is_some())
        );
        Ok(())
    }

    fn managed_edit_supersedes_in_flight_inputs(
        sharded: bool,
    ) -> Result<
        (
            crate::rigid_body::BodyState,
            crate::aerodynamics::pressure::PressureWrench,
        ),
        Box<dyn std::error::Error>,
    > {
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
        let (sender, pending) = PendingShape::test_channel();
        let job = world
            .pending_shapes
            .as_mut()
            .and_then(|jobs| jobs.get_mut(&body))
            .ok_or("missing initial job")?;
        job.result = Ok(pending);
        world.set_part_aerodynamic_mesh(part, Some(super::super::shape_binding_tests::cube()?))?;
        if sharded {
            world.step_with_shards(
                1e-8,
                &mut [Vec::new(), Vec::new()],
                &crate::execution::SequentialExecutor,
                2,
            )?;
        } else {
            world.step(1e-8)?;
        }
        assert!(
            sender
                .send(Err(ShapeRegistryError::StaleBinding.into()))
                .is_err()
        );
        let BodyShapeStatus::Ready(id) = world.body_shape_status(body) else {
            return Err("replacement was not published".into());
        };
        assert_eq!(world.shape_registry().len(), 1);
        let shape = world
            .shape_registry()
            .get(id)
            .ok_or("missing installed shape")?;
        assert!(Arc::ptr_eq(
            &shape.sources()[0].mesh,
            world
                .part_aerodynamic_mesh_arc(part)
                .ok_or("missing live mesh")?
        ));
        let loads = world
            .owner_mesh_pressure(body)
            .ok_or("missing pressure load")?;
        assert_eq!(loads.len(), 1);
        assert!(loads[0].1.force.length() > 1.0);
        Ok((*world.body(body).ok_or("missing body")?.state(), loads[0].1))
    }

    #[test]
    fn superseded_cook_commits_identical_motion_in_both_step_paths()
    -> Result<(), Box<dyn std::error::Error>> {
        assert_eq!(
            managed_edit_supersedes_in_flight_inputs(false)?,
            managed_edit_supersedes_in_flight_inputs(true)?
        );
        Ok(())
    }

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
        use crate::aerodynamics::AerodynamicsTask;
        AerodynamicsTask.execute(&world)?;
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
        world.stop_managing_body_shape(body);
        assert!(
            world.fill_step_shards(&mut snapshots).is_err(),
            "the same oversized mesh must fail synchronous local preparation"
        );
        Ok(())
    }

    #[rstest::rstest]
    #[case(false)]
    #[case(true)]
    fn managed_merge_and_root_transfer_recook_the_surviving_assembly(
        #[case] sharded: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let step = |world: &mut PhysicsWorld| {
            if sharded {
                world.step_with_shards(
                    1e-8,
                    &mut [Vec::new(), Vec::new()],
                    &crate::execution::SequentialExecutor,
                    2,
                )
            } else {
                world.step(1e-8)
            }
        };
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
        step(&mut world)?;
        let initial = world.shape_binding(first).ok_or("initial binding")?.shape();
        world.merge_clusters(
            first,
            second,
            super::super::shape_binding_tests::merge_weld(),
        )?;
        step(&mut world)?;
        let current = world.shape_binding(first).ok_or("merged binding")?;
        assert_ne!(initial, current.shape());
        assert_eq!(current.parts(), &[a, b]);
        let bridge = *world
            .cluster(first)
            .ok_or("merged cluster")?
            .welds()
            .first()
            .ok_or("missing bridge")?;
        let merged = current.shape();
        world.break_weld(bridge)?;
        step(&mut world)?;
        let current = world.shape_binding(first).ok_or("split binding")?;
        assert_ne!(current.shape(), merged);
        assert_eq!(current.parts(), &[a]);
        let detached = world.part_body(b).ok_or("detached owner")?;
        assert_eq!(
            world.body_shape_status(detached),
            BodyShapeStatus::Unmanaged
        );
        world.merge_clusters(
            first,
            detached,
            super::super::shape_binding_tests::merge_weld(),
        )?;
        step(&mut world)?;
        let current = world.shape_binding(first).ok_or("remerged binding")?;
        let merged = current.shape();
        world.destroy_cluster_part(a)?;
        step(&mut world)?;
        let current = world.shape_binding(first).ok_or("root-transfer binding")?;
        assert_ne!(current.shape(), merged);
        assert_eq!(current.parts(), &[b]);
        assert_eq!(world.cluster(first).ok_or("surviving cluster")?.root(), b);
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
        #[values(false, true)] prepared_only: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut world = PhysicsWorld::new();
        world.set_atmosphere(super::super::test_atmosphere(
            crate::aerodynamics::AtmosphericState::new(
                1.0,
                Vec3::X * if prepared_only { -1500.0 } else { -2000.0 },
            )?,
        ));
        if prepared_only {
            world.set_mesh_pressure_model(
                crate::aerodynamics::MeshPressureModel::ModifiedNewtonianTriangleQuadrature,
            );
        }
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
            sender
                .send(Ok(shape.take().ok_or("shape")?))
                .map_err(|_| "pending receiver closed before merge result")?;
            advance(&mut world)?;
            assert_eq!(world.shape_registry().len(), 1);
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
        let before = *world.body(body).ok_or("missing body")?.state();
        let revision = world.body_revision;
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
        assert_eq!(*world.body(body).ok_or("missing body")?.state(), before);
        assert_eq!(world.body_revision, revision);
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
