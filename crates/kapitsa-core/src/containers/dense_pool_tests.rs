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

#[test]
fn values_returns_an_empty_slice_for_an_empty_pool() {
    let mut pool = DensePool::<i32>::new();

    assert!(pool.values().is_empty());
    assert!(pool.values_mut().is_empty());
}

#[test]
fn values_match_the_inserted_elements() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    pool.insert(10)?;
    pool.insert(20)?;
    pool.insert(30)?;

    assert_eq!(pool.values(), &[10, 20, 30]);
    Ok(())
}

#[test]
fn values_mut_changes_each_element() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    pool.insert(1)?;
    pool.insert(2)?;
    pool.insert(3)?;

    for value in pool.values_mut() {
        *value *= 10;
    }

    assert_eq!(pool.values(), &[10, 20, 30]);
    Ok(())
}

#[test]
fn values_reflect_swap_remove() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    pool.insert(2)?;
    pool.insert(3)?;

    assert_eq!(pool.remove(first), Some(1));
    assert_eq!(pool.values(), &[3, 2]);
    Ok(())
}

#[test]
fn dense_ids_remain_valid_after_slice_mutation() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    let second = pool.insert(2)?;

    for value in pool.values_mut() {
        *value += 10;
    }

    assert_eq!(pool.get(first), Some(&11));
    assert_eq!(pool.get(second), Some(&12));
    assert!(pool.contains(first));
    assert!(pool.contains(second));
    Ok(())
}

#[test]
fn values_length_stays_synchronized_with_pool_length() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    let second = pool.insert(2)?;
    assert_eq!(pool.values().len(), pool.len());

    let third = pool.insert(3)?;
    assert_eq!(pool.values().len(), pool.len());
    assert_eq!(pool.remove(second), Some(2));
    assert_eq!(pool.values().len(), pool.len());
    assert_eq!(pool.remove(first), Some(1));
    assert_eq!(pool.values().len(), pool.len());
    assert_eq!(pool.remove(third), Some(3));
    assert_eq!(pool.values().len(), pool.len());
    assert!(pool.values().is_empty());
    Ok(())
}

#[test]
fn try_insert_many_commits_as_a_batch_and_reuses_free_slots() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    let second = pool.insert(2)?;
    let third = pool.insert(3)?;
    assert_eq!(pool.remove(second), Some(2));
    assert_eq!(pool.remove(first), Some(1));

    let ids = pool.try_insert_many(vec![10, 11, 12, 13])?;

    assert_eq!(ids.len(), 4);
    assert_eq!(ids[0].index(), first.index());
    assert_eq!(ids[1].index(), second.index());
    assert_eq!(ids[2].index(), 3);
    assert_eq!(ids[3].index(), 4);
    for (id, expected) in ids.iter().copied().zip([10, 11, 12, 13]) {
        assert_eq!(pool.get(id), Some(&expected));
    }
    assert_eq!(pool.get(third), Some(&3));
    assert!(!pool.contains(first));
    assert!(!pool.contains(second));
    Ok(())
}

#[test]
fn try_insert_many_with_no_values_is_a_no_op() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let id = pool.insert(1)?;

    assert!(pool.try_insert_many(Vec::new())?.is_empty());
    assert_eq!(pool.get(id), Some(&1));
    Ok(())
}

#[test]
fn try_insert_many_rolls_back_values_slots_and_generations() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    let second = pool.insert(2)?;
    let third = pool.insert(3)?;
    assert_eq!(pool.remove(first), Some(1));
    assert_eq!(pool.remove(second), Some(2));

    let values_before = pool.values().to_vec();
    let free_head_before = pool.free_head;
    let slots_before = pool
        .slots
        .iter()
        .map(|slot| (slot.generation, slot.state))
        .collect::<Vec<_>>();
    let first_generation = pool.slots[first.index() as usize].generation;
    let second_generation = pool.slots[second.index() as usize].generation;

    let result = pool.try_insert_many_failing_after(vec![10, 11, 12], 2);

    assert_eq!(result, Err(DensePoolError::CapacityExceeded));
    assert_eq!(pool.values(), values_before);
    assert_eq!(pool.free_head, free_head_before);
    assert_eq!(
        pool.slots
            .iter()
            .map(|slot| (slot.generation, slot.state))
            .collect::<Vec<_>>(),
        slots_before
    );
    assert!(!pool.values().contains(&10));
    assert!(!pool.values().contains(&11));
    assert_eq!(pool.get(third), Some(&3));

    let ids = pool.try_insert_many(vec![20, 21])?;
    assert_eq!(ids[0].index(), second.index());
    assert_eq!(ids[0].generation(), second_generation);
    assert_eq!(ids[1].index(), first.index());
    assert_eq!(ids[1].generation(), first_generation);
    Ok(())
}

#[test]
fn reserves_insertions_for_reused_and_new_slots() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let first = pool.insert(1)?;
    let second = pool.insert(2)?;
    let third = pool.insert(3)?;
    assert_eq!(pool.remove(second), Some(2));
    assert_eq!(pool.remove(first), Some(1));

    pool.try_reserve_insertions(4)?;
    let ids = [
        pool.insert(10)?,
        pool.insert(11)?,
        pool.insert(12)?,
        pool.insert(13)?,
    ];

    assert_eq!(ids[0].index(), first.index());
    assert_eq!(ids[1].index(), second.index());
    assert_eq!(ids[2].index(), 3);
    assert_eq!(ids[3].index(), 4);
    assert_eq!(pool.get(third), Some(&3));
    Ok(())
}

#[test]
fn reserve_overflow_does_not_change_pool() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let id = pool.insert(7)?;
    let values = pool.values().to_vec();
    let dense_to_slot = pool.dense_to_slot.clone();
    let slots = pool
        .slots
        .iter()
        .map(|slot| (slot.generation, slot.state))
        .collect::<Vec<_>>();
    let free_head = pool.free_head;

    assert_eq!(
        pool.try_reserve_insertions(usize::MAX),
        Err(DensePoolError::IndexOverflow)
    );
    assert_eq!(pool.values(), values.as_slice());
    assert_eq!(pool.dense_to_slot, dense_to_slot);
    assert_eq!(
        pool.slots
            .iter()
            .map(|slot| (slot.generation, slot.state))
            .collect::<Vec<_>>(),
        slots
    );
    assert_eq!(pool.free_head, free_head);
    assert_eq!(pool.get(id), Some(&7));
    Ok(())
}

#[test]
fn reserve_rejects_a_cycle_before_insertion() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let id = pool.insert(7)?;
    assert_eq!(pool.remove(id), Some(7));
    pool.slots[id.index() as usize].state = SlotState::Free {
        next: Some(id.index()),
    };
    assert_eq!(
        pool.try_reserve_insertions(2),
        Err(DensePoolError::InvalidFreeList)
    );
    assert_eq!(pool.len(), 0);
    Ok(())
}

#[test]
fn failed_insertion_returns_owned_value() -> Result<(), DensePoolError> {
    let mut pool = DensePool::new();
    let id = pool.insert(String::from("previous"))?;
    assert!(pool.remove(id).is_some());
    let slot = pool
        .slots
        .get_mut(id.index() as usize)
        .ok_or(DensePoolError::InvalidFreeList)?;
    slot.state = SlotState::Retired;
    let result = pool.insert_recover(String::from("retained"));
    assert_eq!(
        result,
        Err((DensePoolError::InvalidFreeList, String::from("retained")))
    );
    assert_eq!(pool.len(), 0);
    Ok(())
}

#[test]
fn recoverable_batch_restores_values_and_free_slots_after_failure() -> Result<(), DensePoolError> {
    for fail_after in 0..3 {
        let mut pool = DensePool::new();
        let survivor = pool.insert(String::from("survivor"))?;
        let freed = pool.insert(String::from("freed"))?;
        assert!(pool.remove(freed).is_some());
        let free_head = pool.free_head;
        let mut values = vec![String::from("a"), String::from("b"), String::from("c")];
        assert_eq!(
            pool.try_insert_many_recover_inner(&mut values, Some(fail_after)),
            Err(DensePoolError::CapacityExceeded)
        );
        assert_eq!(values, ["a", "b", "c"]);
        assert_eq!(pool.get(survivor).map(String::as_str), Some("survivor"));
        assert_eq!(pool.len(), 1);
        assert_eq!(pool.free_head, free_head);
        let ids = pool.try_insert_many_recover(&mut values)?;
        assert!(values.is_empty());
        let stored: Vec<_> = ids
            .iter()
            .map(|&id| pool.get(id).map(String::as_str))
            .collect();
        assert_eq!(stored, [Some("a"), Some("b"), Some("c")]);
        assert_eq!(ids.last().map(|id| id.index()), Some(freed.index()));
        assert_eq!(
            ids.last().map(|id| id.generation()),
            Some(freed.generation() + 1)
        );
    }
    Ok(())
}
