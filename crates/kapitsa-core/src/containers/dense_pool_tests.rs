use super::{DenseId, DensePool, DensePoolError, SlotState};

#[test]
fn empty_pool_has_no_values() {
    let pool = DensePool::<i32>::new();

    assert!(pool.is_empty());
    assert_eq!(pool.len(), 0);
    assert_eq!(pool.iter().count(), 0);
}

#[test]
fn insert_get_mut_and_remove() -> Result<(), DensePoolError> {
    let mut pool = DensePool::default();
    let id = pool.insert(10)?;

    assert_eq!(pool.get(id), Some(&10));
    assert!(pool.contains(id));
    if let Some(value) = pool.get_mut(id) {
        *value = 20;
    }
    assert_eq!(pool.get(id), Some(&20));
    assert_eq!(pool.remove(id), Some(20));
    assert!(!pool.contains(id));
    assert!(pool.is_empty());
    Ok(())
}

#[test]
fn removes_first_and_last_elements() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    let middle = pool.insert(2)?;
    let last = pool.insert(3)?;

    assert_eq!(pool.remove(first), Some(1));
    assert_eq!(pool.remove(last), Some(3));
    assert_eq!(pool.get(middle), Some(&2));
    assert_eq!(pool.len(), 1);
    Ok(())
}

#[test]
fn removes_the_only_element() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let id = pool.insert(1)?;

    assert_eq!(pool.remove(id), Some(1));
    assert!(pool.is_empty());
    Ok(())
}

#[test]
fn swap_remove_updates_the_moved_elements_slot() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert("first")?;
    let middle = pool.insert("middle")?;
    let last = pool.insert("last")?;

    assert_eq!(pool.remove(first), Some("first"));
    assert_eq!(pool.get(last), Some(&"last"));
    assert_eq!(pool.get(middle), Some(&"middle"));
    assert_eq!(last.index(), 2);
    assert_eq!(
        pool.iter().map(|(id, _)| id).collect::<Vec<_>>(),
        [last, middle]
    );
    Ok(())
}

#[test]
fn reuses_free_slots_and_rejects_stale_ids() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let old_id = pool.insert(1)?;
    assert_eq!(pool.remove(old_id), Some(1));

    let new_id = pool.insert(2)?;
    assert_eq!(new_id.index(), old_id.index());
    assert_eq!(new_id.generation(), old_id.generation() + 1);
    assert!(!pool.contains(old_id));
    assert_eq!(pool.get(old_id), None);
    assert_eq!(pool.get(new_id), Some(&2));
    Ok(())
}

#[test]
fn supports_multiple_generations_of_one_slot() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let mut previous = pool.insert(0)?;

    for value in 1..8 {
        assert_eq!(pool.remove(previous), Some(value - 1));
        let current = pool.insert(value)?;
        assert_eq!(current.index(), previous.index());
        assert_eq!(current.generation(), previous.generation() + 1);
        assert!(!pool.contains(previous));
        previous = current;
    }
    Ok(())
}

#[test]
fn retires_a_slot_when_its_generation_overflows() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let initial = pool.insert(1)?;
    let slot = pool
        .slots
        .get_mut(initial.index() as usize)
        .ok_or(DensePoolError::InvalidFreeList)?;
    slot.generation = u32::MAX;
    let max_generation_id = DenseId {
        index: initial.index(),
        generation: u32::MAX,
    };

    assert_eq!(pool.remove(max_generation_id), Some(1));
    assert!(matches!(
        pool.slots
            .get(initial.index() as usize)
            .map(|slot| &slot.state),
        Some(SlotState::Retired)
    ));

    let replacement = pool.insert(2)?;
    assert_ne!(replacement.index(), initial.index());
    assert_eq!(pool.get(max_generation_id), None);
    Ok(())
}

#[test]
fn iteration_has_no_skips_or_repeats() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(10)?;
    let removed = pool.insert(20)?;
    let last = pool.insert(30)?;
    assert_eq!(pool.remove(removed), Some(20));

    let items = pool
        .iter()
        .map(|(id, value)| (id, *value))
        .collect::<Vec<_>>();
    assert_eq!(items.len(), 2);
    assert!(items.contains(&(first, 10)));
    assert!(items.contains(&(last, 30)));
    assert_ne!(items[0].0, items[1].0);
    Ok(())
}

#[test]
fn iter_mut_updates_every_live_value() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    let second = pool.insert(2)?;

    for (_, value) in pool.iter_mut() {
        *value *= 10;
    }

    assert_eq!(pool.get(first), Some(&10));
    assert_eq!(pool.get(second), Some(&20));
    Ok(())
}

#[test]
fn inserting_does_not_reuse_a_live_id() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    let second = pool.insert(2)?;

    assert_ne!(first, second);
    assert_ne!(first.index(), second.index());
    assert!(pool.contains(first));
    assert!(pool.contains(second));
    Ok(())
}
