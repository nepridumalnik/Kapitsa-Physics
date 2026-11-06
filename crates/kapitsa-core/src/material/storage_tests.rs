use crate::{containers::DensePoolError, material::PhysicsMaterial};

use super::MaterialStorage;

#[test]
fn reuses_slot_with_a_new_generation() -> Result<(), DensePoolError> {
    let mut storage = MaterialStorage::new();
    let old_id = storage.insert(PhysicsMaterial::default())?;
    storage.remove(old_id);

    let new_id = storage.insert(PhysicsMaterial::default())?;

    assert_eq!(new_id.dense.index(), old_id.dense.index());
    assert_eq!(new_id.dense.generation(), old_id.dense.generation() + 1);
    Ok(())
}

#[test]
fn material_ids_do_not_resolve_in_a_different_storage() -> Result<(), DensePoolError> {
    let mut first = MaterialStorage::new();
    let mut second = MaterialStorage::new();
    let id = first.insert(PhysicsMaterial::default())?;
    let other = second.insert(PhysicsMaterial::default())?;

    assert_ne!(id, other);
    assert!(first.contains(id));
    assert!(!second.contains(id));
    assert!(second.get(id).is_none());
    assert!(second.remove(id).is_none());
    Ok(())
}
