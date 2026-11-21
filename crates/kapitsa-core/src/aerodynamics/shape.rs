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
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::containers::{DenseId, DensePool, DensePoolError};
use crate::shape::TriangleMesh;
use crate::transform::Transform;

use super::geometry::{
    GeometryConfig, GeometryError, GeometryInstance, GeometryOwner, OwnerSlot, PreparedGeometry,
};

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
}

impl fmt::Display for ShapeRegistryError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::StaleShape => formatter.write_str("shape ID is foreign or stale"),
            Self::RevisionExhausted => formatter.write_str("shape revision counter is exhausted"),
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

#[cfg(test)]
#[path = "shape_tests.rs"]
mod tests;
