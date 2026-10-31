use std::sync::atomic::{AtomicU64, Ordering};

use crate::containers::{DenseId, DensePool, DensePoolError};

use super::PhysicsMaterial;

/// Identifies a material in the storage instance that created it.
///
/// The storage identity prevents an ID from resolving in another [`MaterialStorage`]. Obtain IDs
/// through [`MaterialStorage::insert`].
///
/// ```compile_fail
/// use kapitsa_core::containers::DensePool;
/// use kapitsa_core::material::MaterialId;
///
/// let mut pool = DensePool::new();
/// let id = match pool.insert(()) {
///     Ok(id) => id,
///     Err(_) => return,
/// };
/// let _ = MaterialId(id);
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MaterialId {
    storage: u64,
    dense: DenseId,
}

static NEXT_STORAGE_ID: AtomicU64 = AtomicU64::new(1);

fn allocate_storage_id() -> Result<u64, DensePoolError> {
    NEXT_STORAGE_ID
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |next| {
            next.checked_add(1)
        })
        .map_err(|_| DensePoolError::IdentifierExhausted)
}

/// Owns physics materials and assigns each one a typed generational ID.
pub struct MaterialStorage {
    materials: DensePool<PhysicsMaterial>,
    storage_id: u64,
}

impl MaterialStorage {
    pub const fn new() -> Self {
        Self {
            materials: DensePool::new(),
            storage_id: 0,
        }
    }

    pub fn insert(&mut self, material: PhysicsMaterial) -> Result<MaterialId, DensePoolError> {
        if self.storage_id == 0 {
            self.storage_id = allocate_storage_id()?;
        }
        self.materials.insert(material).map(|dense| MaterialId {
            storage: self.storage_id,
            dense,
        })
    }

    pub fn get(&self, id: MaterialId) -> Option<&PhysicsMaterial> {
        (id.storage == self.storage_id)
            .then(|| self.materials.get(id.dense))
            .flatten()
    }

    pub fn get_mut(&mut self, id: MaterialId) -> Option<&mut PhysicsMaterial> {
        (id.storage == self.storage_id)
            .then(|| self.materials.get_mut(id.dense))
            .flatten()
    }

    pub fn remove(&mut self, id: MaterialId) -> Option<PhysicsMaterial> {
        (id.storage == self.storage_id)
            .then(|| self.materials.remove(id.dense))
            .flatten()
    }

    pub fn contains(&self, id: MaterialId) -> bool {
        id.storage == self.storage_id && self.materials.contains(id.dense)
    }

    pub fn len(&self) -> usize {
        self.materials.len()
    }

    pub fn is_empty(&self) -> bool {
        self.materials.is_empty()
    }

    pub fn values(&self) -> &[PhysicsMaterial] {
        self.materials.values()
    }

    pub fn values_mut(&mut self) -> &mut [PhysicsMaterial] {
        self.materials.values_mut()
    }
}

impl Default for MaterialStorage {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "storage_tests.rs"]
mod tests;
