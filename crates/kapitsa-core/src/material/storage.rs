use crate::containers::{DenseId, DensePool, DensePoolError};

use super::PhysicsMaterial;

/// Identifies a material in the storage instance that created it.
///
/// The underlying [`DenseId`] does not include an owner identifier, so IDs must not be used with a
/// different [`MaterialStorage`]. Obtain IDs through [`MaterialStorage::insert`].
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MaterialId(DenseId);

/// Owns physics materials and assigns each one a typed generational ID.
pub struct MaterialStorage {
    materials: DensePool<PhysicsMaterial>,
}

impl MaterialStorage {
    pub const fn new() -> Self {
        Self {
            materials: DensePool::new(),
        }
    }

    pub fn insert(&mut self, material: PhysicsMaterial) -> Result<MaterialId, DensePoolError> {
        self.materials.insert(material).map(MaterialId)
    }

    pub fn get(&self, id: MaterialId) -> Option<&PhysicsMaterial> {
        self.materials.get(id.0)
    }

    pub fn get_mut(&mut self, id: MaterialId) -> Option<&mut PhysicsMaterial> {
        self.materials.get_mut(id.0)
    }

    pub fn remove(&mut self, id: MaterialId) -> Option<PhysicsMaterial> {
        self.materials.remove(id.0)
    }

    pub fn contains(&self, id: MaterialId) -> bool {
        self.materials.contains(id.0)
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
