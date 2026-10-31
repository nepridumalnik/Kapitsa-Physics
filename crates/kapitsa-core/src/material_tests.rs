use std::error::Error;

use crate::containers::DensePoolError;

use super::{MaterialStorage, PhysicsMaterial, PhysicsMaterialError};

#[test]
fn creates_material_with_valid_parameters() -> Result<(), PhysicsMaterialError> {
    let material = PhysicsMaterial::new(0.8, 0.3, 0.6, 2.5)?;

    assert_eq!(material.friction(), 0.8);
    assert_eq!(material.static_friction(), 0.3);
    assert_eq!(material.restitution(), 0.6);
    assert_eq!(material.density(), 2.5);
    Ok(())
}

#[test]
fn default_uses_chaos_material_values() {
    let material = PhysicsMaterial::default();

    assert_eq!(material.friction(), 0.5);
    assert_eq!(material.static_friction(), 0.0);
    assert_eq!(material.restitution(), 0.1);
    assert_eq!(material.density(), 1.0);
}

#[test]
fn rejects_nan_for_each_parameter() {
    assert_eq!(
        PhysicsMaterial::new(f64::NAN, 0.0, 0.5, 1.0),
        Err(PhysicsMaterialError::NonFiniteFriction)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, f64::NAN, 0.5, 1.0),
        Err(PhysicsMaterialError::NonFiniteStaticFriction)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, f64::NAN, 1.0),
        Err(PhysicsMaterialError::NonFiniteRestitution)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, 0.5, f64::NAN),
        Err(PhysicsMaterialError::NonFiniteDensity)
    );
}

#[test]
fn rejects_positive_and_negative_infinity_for_each_parameter() {
    let cases = [
        (
            [f64::INFINITY, 0.0, 0.5, 1.0],
            PhysicsMaterialError::NonFiniteFriction,
        ),
        (
            [f64::NEG_INFINITY, 0.0, 0.5, 1.0],
            PhysicsMaterialError::NonFiniteFriction,
        ),
        (
            [0.5, f64::INFINITY, 0.5, 1.0],
            PhysicsMaterialError::NonFiniteStaticFriction,
        ),
        (
            [0.5, f64::NEG_INFINITY, 0.5, 1.0],
            PhysicsMaterialError::NonFiniteStaticFriction,
        ),
        (
            [0.5, 0.0, f64::INFINITY, 1.0],
            PhysicsMaterialError::NonFiniteRestitution,
        ),
        (
            [0.5, 0.0, f64::NEG_INFINITY, 1.0],
            PhysicsMaterialError::NonFiniteRestitution,
        ),
        (
            [0.5, 0.0, 0.5, f64::INFINITY],
            PhysicsMaterialError::NonFiniteDensity,
        ),
        (
            [0.5, 0.0, 0.5, f64::NEG_INFINITY],
            PhysicsMaterialError::NonFiniteDensity,
        ),
    ];

    for (parameters, expected) in cases {
        assert_eq!(
            PhysicsMaterial::new(parameters[0], parameters[1], parameters[2], parameters[3]),
            Err(expected)
        );
    }
}

#[test]
fn rejects_negative_friction_coefficients() {
    assert_eq!(
        PhysicsMaterial::new(-0.1, 0.0, 0.5, 1.0),
        Err(PhysicsMaterialError::NegativeFriction)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, -0.1, 0.5, 1.0),
        Err(PhysicsMaterialError::NegativeStaticFriction)
    );
}

#[test]
fn rejects_restitution_outside_unit_interval() {
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, -0.1, 1.0),
        Err(PhysicsMaterialError::RestitutionOutOfRange)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, 1.1, 1.0),
        Err(PhysicsMaterialError::RestitutionOutOfRange)
    );
}

#[test]
fn rejects_non_positive_density() {
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, 0.5, 0.0),
        Err(PhysicsMaterialError::NonPositiveDensity)
    );
    assert_eq!(
        PhysicsMaterial::new(0.5, 0.0, 0.5, -1.0),
        Err(PhysicsMaterialError::NonPositiveDensity)
    );
}

#[test]
fn accepts_boundaries_without_friction_ordering() -> Result<(), PhysicsMaterialError> {
    let material = PhysicsMaterial::new(0.8, 0.0, 0.0, f64::MIN_POSITIVE)?;
    let maximum_restitution = PhysicsMaterial::new(0.0, 0.5, 1.0, 1.0)?;

    assert_eq!(material.friction(), 0.8);
    assert_eq!(material.static_friction(), 0.0);
    assert_eq!(material.restitution(), 0.0);
    assert_eq!(material.density(), f64::MIN_POSITIVE);
    assert_eq!(maximum_restitution.restitution(), 1.0);
    Ok(())
}

#[test]
fn material_storage_starts_empty() {
    let storage = MaterialStorage::new();
    let default_storage = MaterialStorage::default();

    assert!(storage.is_empty());
    assert_eq!(storage.len(), 0);
    assert!(storage.values().is_empty());
    assert!(default_storage.is_empty());
}

#[test]
fn material_storage_inserts_and_gets_materials() -> Result<(), DensePoolError> {
    let mut storage = MaterialStorage::new();
    let material = PhysicsMaterial::default();
    let id = storage.insert(material)?;

    assert_eq!(storage.get(id), Some(&material));
    assert!(storage.contains(id));
    assert_eq!(storage.len(), 1);
    Ok(())
}

#[test]
fn material_storage_updates_materials() -> Result<(), Box<dyn Error>> {
    let mut storage = MaterialStorage::new();
    let id = storage.insert(PhysicsMaterial::default())?;
    let updated = PhysicsMaterial::new(0.8, 0.2, 0.7, 2.0)?;

    if let Some(material) = storage.get_mut(id) {
        *material = updated;
    }

    assert_eq!(storage.get(id), Some(&updated));
    Ok(())
}

#[test]
fn material_storage_removes_and_reuses_slots() -> Result<(), DensePoolError> {
    let mut storage = MaterialStorage::new();
    let old_id = storage.insert(PhysicsMaterial::default())?;

    assert_eq!(storage.remove(old_id), Some(PhysicsMaterial::default()));
    let new_id = storage.insert(PhysicsMaterial::default())?;

    assert_ne!(new_id, old_id);
    assert!(!storage.contains(old_id));
    assert_eq!(storage.get(old_id), None);
    assert_eq!(storage.get_mut(old_id), None);
    assert_eq!(storage.remove(old_id), None);
    assert!(storage.contains(new_id));
    Ok(())
}

#[test]
fn material_ids_remain_valid_after_swap_remove() -> Result<(), Box<dyn Error>> {
    let mut storage = MaterialStorage::new();
    let first = storage.insert(PhysicsMaterial::new(0.1, 0.0, 0.0, 1.0)?)?;
    let middle = storage.insert(PhysicsMaterial::new(0.2, 0.0, 0.0, 1.0)?)?;
    let last = storage.insert(PhysicsMaterial::new(0.3, 0.0, 0.0, 1.0)?)?;

    storage.remove(first);

    assert_eq!(
        storage.get(middle).map(PhysicsMaterial::friction),
        Some(0.2)
    );
    assert_eq!(storage.get(last).map(PhysicsMaterial::friction), Some(0.3));
    Ok(())
}

#[test]
fn material_storage_exposes_dense_values() -> Result<(), Box<dyn Error>> {
    let mut storage = MaterialStorage::new();
    let first = storage.insert(PhysicsMaterial::default())?;
    let second = storage.insert(PhysicsMaterial::default())?;

    assert_eq!(storage.values().len(), 2);
    storage.values_mut()[0] = PhysicsMaterial::new(0.8, 0.1, 0.2, 3.0)?;

    assert_eq!(storage.values()[0].friction(), 0.8);
    assert_eq!(storage.values()[1], PhysicsMaterial::default());
    assert_eq!(storage.get(first).map(PhysicsMaterial::friction), Some(0.8));
    assert_eq!(storage.get(second), Some(&PhysicsMaterial::default()));
    Ok(())
}

#[test]
fn material_storage_length_tracks_insertions_and_removals() -> Result<(), DensePoolError> {
    let mut storage = MaterialStorage::new();
    let first = storage.insert(PhysicsMaterial::default())?;
    let second = storage.insert(PhysicsMaterial::default())?;
    assert_eq!(storage.values().len(), storage.len());

    let third = storage.insert(PhysicsMaterial::default())?;
    assert_eq!(storage.values().len(), storage.len());
    storage.remove(second);
    assert_eq!(storage.values().len(), storage.len());
    storage.remove(first);
    assert_eq!(storage.values().len(), storage.len());
    storage.remove(third);
    assert_eq!(storage.values().len(), storage.len());
    assert!(storage.is_empty());
    Ok(())
}
