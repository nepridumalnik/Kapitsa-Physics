//! Shared cooked vessel shapes bound to body clusters.
//!
//! Cooking and binding are explicit, fallible operations outside the physics
//! step. A binding is validated against the live world (cluster membership,
//! part meshes, transforms, preparation configuration and cavity policy) so an
//! edited body can never evaluate another body's stale geometry. Evaluation
//! re-validates the binding every step and falls back to body-local
//! preparation on mismatch.

use std::collections::HashMap;

use super::{BodyId, ClusterError, PartId, PhysicsWorld};
use crate::aerodynamics::shape::{
    CookedSource, ShapeBinding, ShapeId, ShapeRegistry, ShapeRegistryError,
    assembly_pressure_config,
};

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
}
