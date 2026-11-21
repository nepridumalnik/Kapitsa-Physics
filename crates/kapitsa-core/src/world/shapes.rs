//! Shared cooked vessel shapes bound to body clusters.
//!
//! Cooking and binding are explicit, fallible operations outside the physics
//! step. A binding is validated against the live world (cluster membership,
//! part meshes, transforms, preparation configuration and cavity policy) so an
//! edited body can never evaluate another body's stale geometry. Evaluation
//! re-validates the binding every step and falls back to body-local
//! preparation on mismatch.

use std::collections::HashMap;

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
    /// keeping its identity and advancing its revision.
    ///
    /// Existing bindings become stale: they no longer match the new revision,
    /// so evaluation falls back to body-local preparation until the body is
    /// re-bound. A failed cook or stale/foreign ID leaves the registry
    /// unchanged. Returns the new revision.
    pub fn replace_shape(
        &mut self,
        id: ShapeId,
        sources: &[CookedSource],
    ) -> Result<u64, crate::Error> {
        let shape = CookedShape::cook(
            sources,
            assembly_pressure_config(),
            self.preserve_assembly_mesh_cavities,
        )?;
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
        Ok(())
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
