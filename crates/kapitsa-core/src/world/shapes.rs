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
    policy: PendingAero,
    result: Result<PendingShape, crate::Error>,
}

pub(super) fn suppress_pending_shape(
    pending: Option<&HashMap<BodyId, PendingBodyShape>>,
    body: BodyId,
    previous_is_current: impl FnOnce() -> bool,
) -> bool {
    pending
        .and_then(|jobs| jobs.get(&body))
        .is_some_and(|job| job.policy == PendingAero::ZeroForce || !previous_is_current())
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
    oriented_cavities: bool,
    body_id: BodyId,
    entry: &BodyEntry,
) -> Option<(&'a ShapeBinding, &'a CookedShape)> {
    let binding = bindings?.get(&body_id)?;
    let shape = shapes.get(binding.shape())?;
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
        self.pending_shapes.get_or_insert_with(HashMap::new).insert(
            body,
            PendingBodyShape {
                parts,
                policy,
                result: Ok(cook),
            },
        );
        Ok(())
    }

    pub(super) fn finish_pending_shapes(&mut self) {
        let Some(mut pending) = self.pending_shapes.take() else {
            return;
        };
        pending.retain(|&body, job| {
            if self.cluster(body).is_none() {
                return false;
            }
            let Ok(cook) = &mut job.result else {
                return true;
            };
            let result = if job.policy == PendingAero::BlockStep {
                cook.wait()
            } else {
                cook.try_recv()
            };
            let result = match result {
                Ok(result) => result,
                Err(TryRecvError::Empty) => return true,
                Err(TryRecvError::Disconnected) => Err(ShapeRegistryError::StaleBinding.into()),
            };
            match result.and_then(|shape| self.publish_body_shape(body, shape, job.parts.clone())) {
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
            self.preserve_assembly_mesh_cavities,
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
    #[case(false, PendingAero::ZeroForce)]
    #[case(true, PendingAero::ZeroForce)]
    #[case(false, PendingAero::BlockStep)]
    #[case(true, PendingAero::BlockStep)]
    fn pending_policy_controls_pressure(
        #[case] sharded: bool,
        #[case] policy: PendingAero,
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
        let (sender, pending) = PendingShape::test_channel();
        world.track_body_shape(body, vec![part], policy, pending)?;
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
        sender
            .send(Ok(shape.take().ok_or("shape")?))
            .map_err(|_| "disconnected")?;
        advance(&mut world)?;
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
