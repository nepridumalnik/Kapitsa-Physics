//! Immutable prepared vessel shapes shared between bodies.
//!
//! A [`CookedShape`] bakes voxel occupancy, sections and volume once, using
//! binding-independent [`OwnerSlot`] owners. Bodies bind a shape through
//! their own slot-to-part mappings; editing one body's geometry never
//! invalidates another body's shared shape. Cooking happens at explicit
//! cook/install boundaries, never inside a physics step.
//!
//! [`ShapeRegistry`] mirrors the material storage pattern: registry-scoped
//! generational IDs over a dense pool. Replacement keeps the shape identity
//! and advances a revision so stale cook results and bindings can be
//! rejected. A failed cook or replacement never mutates the registry.

use std::fmt;

pub mod worker;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::containers::{DenseId, DensePool, DensePoolError};
use crate::shape::TriangleMesh;
use crate::transform::Transform;
use crate::world::PartId;

use super::geometry::{
    GeometryConfig, GeometryError, GeometryInstance, GeometryOwner, OpenMeshPolicy, OwnerSlot,
    PreparedGeometry,
};

/// Preparation configuration of the assembly mesh-pressure union for vessels
/// whose sources are all explicit meshes (no collider tessellation).
///
/// A shared shape must be cooked with exactly this configuration and the
/// body's current oriented-cavity policy to bind to a body; binding validates
/// both. The values are the assembly union's production settings, kept here
/// so cooking and evaluation share one definition.
pub fn assembly_pressure_config() -> GeometryConfig {
    GeometryConfig {
        cell_size: 0.125,
        max_cells: 1_048_576,
        max_work: 100_000_000,
        open_mesh_policy: OpenMeshPolicy::SurfaceOnly,
        weld_tolerance: None,
        max_patch_work: 100_000_000,
    }
}

/// One source instance of a vessel shape in its fixed cook frame.
///
/// The mesh is shared, not copied. `local_transform` is baked into the cooked
/// occupancy at cook time; a binding whose part transforms differ requires a
/// re-cook, not a silent re-framing.
#[derive(Clone)]
pub struct CookedSource {
    /// Shared part mesh in its own local frame.
    pub mesh: Arc<TriangleMesh>,
    /// Transform into the shape's cook frame.
    pub local_transform: Transform,
    /// Binding-independent owner of this source.
    pub slot: OwnerSlot,
}

impl fmt::Debug for CookedSource {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("CookedSource")
            .field("mesh_triangles", &self.mesh.triangles().len())
            .field("local_transform", &self.local_transform)
            .field("slot", &self.slot)
            .finish()
    }
}

/// Immutable prepared vessel shape, independent of any physics body.
///
/// Obtain shapes through [`CookedShape::cook`] and store them in a
/// [`ShapeRegistry`]. The revision starts at 1 and is advanced by
/// [`ShapeRegistry::replace`]; it is not caller-settable.
#[derive(Debug)]
pub struct CookedShape {
    geometry: PreparedGeometry,
    config: GeometryConfig,
    oriented_cavities: bool,
    sources: Vec<CookedSource>,
    revision: u64,
}

/// Failure modes specific to shape registration and replacement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeRegistryError {
    /// The shape ID does not resolve in this registry: foreign registry or
    /// stale generation after removal.
    StaleShape,
    /// The shape revision counter cannot advance.
    RevisionExhausted,
    /// A binding targets a revision that is no longer current.
    StaleBinding,
    /// A binding is empty, duplicates a part, or does not cover exactly one
    /// part per shape source.
    InvalidBinding,
    /// A shape source cannot be shared: collider tessellation is regenerated
    /// per body and cannot be pointer-verified for lazy invalidation.
    UnsupportedSource,
}

impl fmt::Display for ShapeRegistryError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::StaleShape => formatter.write_str("shape ID is foreign or stale"),
            Self::RevisionExhausted => formatter.write_str("shape revision counter is exhausted"),
            Self::StaleBinding => formatter.write_str("shape binding revision is stale"),
            Self::InvalidBinding => {
                formatter.write_str("shape binding does not map one part per source")
            }
            Self::UnsupportedSource => {
                formatter.write_str("shape source cannot be shared between bodies")
            }
        }
    }
}
impl std::error::Error for ShapeRegistryError {}

impl CookedShape {
    /// Prepares the union of `sources` in the cook frame with slot owners.
    ///
    /// Slots must be dense and ordered: source `i` must own slot `i`. This
    /// fixes overlap tie-breaks and canonical patch ordering for every body
    /// that later binds the shape. Budgets and geometry validation are the
    /// same as for direct [`PreparedGeometry`] preparation; failure leaves
    /// nothing published.
    pub fn cook(
        sources: &[CookedSource],
        config: GeometryConfig,
        oriented_cavities: bool,
    ) -> Result<Self, crate::Error> {
        for (index, source) in sources.iter().enumerate() {
            let expected = u32::try_from(index)
                .map_err(|_| crate::Error::from(GeometryError::InvalidGeometry))?;
            if source.slot.index() != expected {
                return Err(crate::Error::from(GeometryError::InvalidGeometry));
            }
        }
        let mut instances = Vec::new();
        instances
            .try_reserve(sources.len())
            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
        for source in sources {
            instances.push(GeometryInstance {
                mesh: &source.mesh,
                local_transform: source.local_transform,
                owner: GeometryOwner::Slot(source.slot),
            });
        }
        let geometry = if oriented_cavities {
            PreparedGeometry::prepare_with_oriented_cavities(&instances, config)?
        } else {
            PreparedGeometry::prepare(&instances, config)?
        };
        let mut owned = Vec::new();
        owned
            .try_reserve(sources.len())
            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
        for source in sources {
            owned.push(source.clone());
        }
        Ok(Self {
            geometry,
            config,
            oriented_cavities,
            sources: owned,
            revision: 1,
        })
    }

    /// Prepared occupancy, sections and volume with slot owners.
    pub fn geometry(&self) -> &PreparedGeometry {
        &self.geometry
    }

    /// Preparation configuration the shape was cooked with.
    pub fn config(&self) -> GeometryConfig {
        self.config
    }

    /// Occupancy rule used at cook time; part of the shape's identity.
    pub fn oriented_cavities(&self) -> bool {
        self.oriented_cavities
    }

    /// Cook-frame sources retained for evaluation-time instance construction.
    pub fn sources(&self) -> &[CookedSource] {
        &self.sources
    }

    /// Current revision, advanced by registry replacement.
    pub fn revision(&self) -> u64 {
        self.revision
    }
}

/// Identifies a shape in the registry that created it.
///
/// The registry identity prevents an ID from resolving in another
/// [`ShapeRegistry`]; the generation prevents reuse after removal. Obtain IDs
/// through [`ShapeRegistry::insert`] or [`ShapeRegistry::cook`].
///
/// ```compile_fail
/// use kapitsa_core::aerodynamics::shape::ShapeId;
/// use kapitsa_core::containers::DensePool;
///
/// let mut pool = DensePool::new();
/// let id = match pool.insert(()) {
///     Ok(id) => id,
///     Err(_) => return,
/// };
/// let _ = ShapeId { registry: 1, dense: id };
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ShapeId {
    registry: u64,
    dense: DenseId,
}

static NEXT_REGISTRY_ID: AtomicU64 = AtomicU64::new(1);

fn allocate_registry_id() -> Result<u64, DensePoolError> {
    NEXT_REGISTRY_ID
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |next| {
            next.checked_add(1)
        })
        .map_err(|_| DensePoolError::IdentifierExhausted)
}

/// Owns cooked vessel shapes and assigns each one a typed generational ID.
///
/// Several bodies may reference one shape; the registry itself holds no
/// body or part identities. Bindings live with the bodies that use a shape.
pub struct ShapeRegistry {
    shapes: DensePool<CookedShape>,
    registry_id: u64,
}

impl ShapeRegistry {
    /// Creates an empty registry. Its identity is allocated on first insert.
    pub const fn new() -> Self {
        Self {
            shapes: DensePool::new(),
            registry_id: 0,
        }
    }

    /// Stores a cooked shape and returns its identity.
    pub fn insert(&mut self, shape: CookedShape) -> Result<ShapeId, crate::Error> {
        if self.registry_id == 0 {
            self.registry_id = allocate_registry_id()?;
        }
        self.shapes.insert(shape).map(|dense| ShapeId {
            registry: self.registry_id,
            dense,
        })
    }

    /// Cooks and stores a shape in one transaction. A failed cook leaves the
    /// registry unchanged.
    pub fn cook(
        &mut self,
        sources: &[CookedSource],
        config: GeometryConfig,
        oriented_cavities: bool,
    ) -> Result<ShapeId, crate::Error> {
        let shape = CookedShape::cook(sources, config, oriented_cavities)?;
        self.insert(shape)
    }

    /// Resolves a current shape, or `None` for foreign/stale IDs.
    pub fn get(&self, id: ShapeId) -> Option<&CookedShape> {
        (id.registry == self.registry_id)
            .then(|| self.shapes.get(id.dense))
            .flatten()
    }

    /// Removes a shape. Its ID stops resolving; a later insert may reuse the
    /// slot with a new generation.
    pub fn remove(&mut self, id: ShapeId) -> Option<CookedShape> {
        (id.registry == self.registry_id)
            .then(|| self.shapes.remove(id.dense))
            .flatten()
    }

    /// Whether the ID resolves in this registry.
    pub fn contains(&self, id: ShapeId) -> bool {
        id.registry == self.registry_id && self.shapes.contains(id.dense)
    }

    /// Number of stored shapes.
    pub fn len(&self) -> usize {
        self.shapes.len()
    }

    /// Whether no shapes are stored.
    pub fn is_empty(&self) -> bool {
        self.shapes.is_empty()
    }

    /// Atomically replaces the shape behind `id`, keeping the identity and
    /// advancing the revision. The replacement's own revision is ignored.
    /// Stale or foreign IDs and revision exhaustion are rejected without
    /// mutation. Returns the new revision.
    pub fn replace(&mut self, id: ShapeId, mut shape: CookedShape) -> Result<u64, crate::Error> {
        let current = self
            .get(id)
            .ok_or_else(|| crate::Error::from(ShapeRegistryError::StaleShape))?;
        let revision = current
            .revision
            .checked_add(1)
            .ok_or_else(|| crate::Error::from(ShapeRegistryError::RevisionExhausted))?;
        shape.revision = revision;
        let slot = self
            .shapes
            .get_mut(id.dense)
            .ok_or_else(|| crate::Error::from(ShapeRegistryError::StaleShape))?;
        *slot = shape;
        Ok(revision)
    }
}

impl Default for ShapeRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Slot-to-part mapping that lets one body evaluate a shared [`CookedShape`].
///
/// The binding records the shape ID and the revision it was validated against.
/// Slot `i` maps to `parts[i]`, preserving the cook-time source order and
/// therefore overlap tie-breaks. Evaluation compares the live shape revision
/// with the bound one and falls back to body-local preparation on mismatch
/// instead of using stale geometry; it never mutates the binding in place.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ShapeBinding {
    shape: ShapeId,
    revision: u64,
    parts: Vec<PartId>,
}

impl ShapeBinding {
    /// Creates a slot-ordered binding.
    ///
    /// Rejects an empty mapping and duplicate parts. The part count must equal
    /// the shape's source count; the world checks that when binding because
    /// only the registry can resolve the shape.
    pub fn new(shape: ShapeId, revision: u64, parts: Vec<PartId>) -> Result<Self, crate::Error> {
        if parts.is_empty() {
            return Err(crate::Error::from(ShapeRegistryError::InvalidBinding));
        }
        let mut sorted = Vec::new();
        sorted
            .try_reserve(parts.len())
            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
        sorted.extend_from_slice(&parts);
        sorted.sort_unstable();
        if sorted.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(crate::Error::from(ShapeRegistryError::InvalidBinding));
        }
        Ok(Self {
            shape,
            revision,
            parts,
        })
    }

    /// Shape this binding refers to.
    pub fn shape(&self) -> ShapeId {
        self.shape
    }

    /// Revision the binding was validated against.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Parts in slot order; slot `i` is `parts()[i]`.
    pub fn parts(&self) -> &[PartId] {
        &self.parts
    }

    /// Part that owns `slot`, or `None` when the slot is outside the binding.
    pub fn part(&self, slot: OwnerSlot) -> Option<PartId> {
        self.parts.get(usize::try_from(slot.index()).ok()?).copied()
    }

    /// Slot owned by `part`, or `None` when the part is not bound. Linear in
    /// the source count, which is small and bounded by the vessel parts.
    pub fn slot_of(&self, part: PartId) -> Option<OwnerSlot> {
        self.parts
            .iter()
            .position(|bound| *bound == part)
            .and_then(|index| u32::try_from(index).ok())
            .map(OwnerSlot::new)
    }

    /// Appends geometry instances for `shape` with this binding's part owners.
    ///
    /// The shape must still be at the bound revision and expose exactly one
    /// source per bound part; otherwise the result is a stale or invalid
    /// binding and nothing is appended.
    pub fn write_instances<'a>(
        &self,
        shape: &'a CookedShape,
        instances: &mut Vec<GeometryInstance<'a>>,
    ) -> Result<(), crate::Error> {
        if shape.revision() != self.revision {
            return Err(crate::Error::from(ShapeRegistryError::StaleBinding));
        }
        let sources = shape.sources();
        if sources.len() != self.parts.len() {
            return Err(crate::Error::from(ShapeRegistryError::InvalidBinding));
        }
        instances
            .try_reserve(sources.len())
            .map_err(|_| crate::Error::from(GeometryError::AllocationFailed))?;
        for (source, part) in sources.iter().zip(&self.parts) {
            instances.push(GeometryInstance {
                mesh: &source.mesh,
                local_transform: source.local_transform,
                owner: GeometryOwner::Part(*part),
            });
        }
        Ok(())
    }
}

#[cfg(test)]
#[path = "shape_tests.rs"]
mod tests;
