use crate::{containers::DensePoolError, material::PhysicsMaterial};

use super::MaterialStorage;

#[test]
fn reuses_slot_with_a_new_generation() -> Result<(), DensePoolError> {
    let mut storage = MaterialStorage::new();
    let old_id = storage.insert(PhysicsMaterial::default())?;
    storage.remove(old_id);

    let new_id = storage.insert(PhysicsMaterial::default())?;

    assert_eq!(new_id.0.index(), old_id.0.index());
    assert_eq!(new_id.0.generation(), old_id.0.generation() + 1);
    Ok(())
}
