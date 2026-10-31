use std::fmt;

/// Identifies an element in a [`DensePool`].
///
/// An ID is valid only in the pool that created it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DenseId {
    index: u32,
    generation: u32,
}

impl DenseId {
    pub fn index(self) -> u32 {
        self.index
    }

    pub fn generation(self) -> u32 {
        self.generation
    }
}

/// Reason an element could not be inserted into a dense pool.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DensePoolError {
    IndexOverflow,
    CapacityExceeded,
    InvalidFreeList,
}

impl fmt::Display for DensePoolError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::IndexOverflow => "dense pool index exceeds the u32 range",
            Self::CapacityExceeded => "dense pool capacity could not be reserved",
            Self::InvalidFreeList => "dense pool free list is invalid",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for DensePoolError {}

#[derive(Debug)]
struct Slot {
    generation: u32,
    state: SlotState,
}

#[derive(Clone, Copy, Debug)]
enum SlotState {
    Occupied { dense_index: u32 },
    Free { next: Option<u32> },
    Retired,
}

/// Stores live values densely and identifies them with stable generational IDs.
pub struct DensePool<T> {
    values: Vec<T>,
    dense_to_slot: Vec<u32>,
    slots: Vec<Slot>,
    free_head: Option<u32>,
}

impl<T> DensePool<T> {
    pub const fn new() -> Self {
        Self {
            values: Vec::new(),
            dense_to_slot: Vec::new(),
            slots: Vec::new(),
            free_head: None,
        }
    }

    pub fn insert(&mut self, value: T) -> Result<DenseId, DensePoolError> {
        let dense_index =
            u32::try_from(self.values.len()).map_err(|_| DensePoolError::IndexOverflow)?;
        let (slot_index, slot_position, generation, next_free) = match self.free_head {
            Some(slot_index) => {
                let slot_position =
                    usize::try_from(slot_index).map_err(|_| DensePoolError::InvalidFreeList)?;
                let slot = self
                    .slots
                    .get(slot_position)
                    .ok_or(DensePoolError::InvalidFreeList)?;
                let next_free = match slot.state {
                    SlotState::Free { next } => next,
                    SlotState::Occupied { .. } | SlotState::Retired => {
                        return Err(DensePoolError::InvalidFreeList);
                    }
                };
                (slot_index, slot_position, slot.generation, next_free)
            }
            None => {
                let slot_index =
                    u32::try_from(self.slots.len()).map_err(|_| DensePoolError::IndexOverflow)?;
                (slot_index, self.slots.len(), 0, None)
            }
        };

        self.values
            .try_reserve(1)
            .map_err(|_| DensePoolError::CapacityExceeded)?;
        self.dense_to_slot
            .try_reserve(1)
            .map_err(|_| DensePoolError::CapacityExceeded)?;
        if self.free_head.is_none() {
            self.slots
                .try_reserve(1)
                .map_err(|_| DensePoolError::CapacityExceeded)?;
        }

        if self.free_head.is_some() {
            let slot = self
                .slots
                .get_mut(slot_position)
                .ok_or(DensePoolError::InvalidFreeList)?;
            slot.state = SlotState::Occupied { dense_index };
            self.free_head = next_free;
        } else {
            self.slots.push(Slot {
                generation,
                state: SlotState::Occupied { dense_index },
            });
        }
        self.values.push(value);
        self.dense_to_slot.push(slot_index);

        Ok(DenseId {
            index: slot_index,
            generation,
        })
    }

    pub fn get(&self, id: DenseId) -> Option<&T> {
        let (_, dense_position) = self.indices(id)?;
        self.values.get(dense_position)
    }

    pub fn get_mut(&mut self, id: DenseId) -> Option<&mut T> {
        let (_, dense_position) = self.indices(id)?;
        self.values.get_mut(dense_position)
    }

    pub fn remove(&mut self, id: DenseId) -> Option<T> {
        let (slot_position, dense_position) = self.indices(id)?;
        let moved_dense_index = u32::try_from(dense_position).ok()?;
        let next_generation = self.slots.get(slot_position)?.generation.checked_add(1);
        let moved_slot_position = if dense_position < self.values.len().saturating_sub(1) {
            let moved_slot_index = *self.dense_to_slot.last()?;
            let moved_slot_position = usize::try_from(moved_slot_index).ok()?;
            let moved_slot = self.slots.get(moved_slot_position)?;
            match moved_slot.state {
                SlotState::Occupied { dense_index }
                    if usize::try_from(dense_index).ok()? == self.values.len() - 1 =>
                {
                    Some(moved_slot_position)
                }
                SlotState::Occupied { .. } | SlotState::Free { .. } | SlotState::Retired => {
                    return None;
                }
            }
        } else {
            None
        };

        let slot = self.slots.get_mut(slot_position)?;
        if let Some(generation) = next_generation {
            slot.generation = generation;
            slot.state = SlotState::Free {
                next: self.free_head,
            };
            self.free_head = Some(id.index);
        } else {
            slot.state = SlotState::Retired;
        }

        let removed = self.values.swap_remove(dense_position);
        self.dense_to_slot.swap_remove(dense_position);
        if let Some(moved_slot_position) = moved_slot_position
            && let Some(slot) = self.slots.get_mut(moved_slot_position)
        {
            slot.state = SlotState::Occupied {
                dense_index: moved_dense_index,
            };
        }
        Some(removed)
    }

    pub fn contains(&self, id: DenseId) -> bool {
        self.indices(id).is_some()
    }

    /// Returns the live values in dense storage order.
    ///
    /// The order is not guaranteed and may change after `remove`.
    pub fn values(&self) -> &[T] {
        &self.values
    }

    /// Mutably borrows the live values in dense storage order.
    ///
    /// The order is not guaranteed and may change after `remove`.
    pub fn values_mut(&mut self) -> &mut [T] {
        &mut self.values
    }

    pub fn iter(&self) -> impl Iterator<Item = (DenseId, &T)> {
        self.values
            .iter()
            .enumerate()
            .filter_map(move |(dense_position, value)| {
                let slot_index = *self.dense_to_slot.get(dense_position)?;
                let slot_position = usize::try_from(slot_index).ok()?;
                let slot = self.slots.get(slot_position)?;
                match slot.state {
                    SlotState::Occupied { dense_index }
                        if usize::try_from(dense_index).ok()? == dense_position =>
                    {
                        Some((
                            DenseId {
                                index: slot_index,
                                generation: slot.generation,
                            },
                            value,
                        ))
                    }
                    SlotState::Occupied { .. } | SlotState::Free { .. } | SlotState::Retired => {
                        None
                    }
                }
            })
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (DenseId, &mut T)> {
        let dense_to_slot = &self.dense_to_slot;
        let slots = &self.slots;
        self.values
            .iter_mut()
            .enumerate()
            .filter_map(move |(dense_position, value)| {
                let slot_index = *dense_to_slot.get(dense_position)?;
                let slot_position = usize::try_from(slot_index).ok()?;
                let slot = slots.get(slot_position)?;
                match slot.state {
                    SlotState::Occupied { dense_index }
                        if usize::try_from(dense_index).ok()? == dense_position =>
                    {
                        Some((
                            DenseId {
                                index: slot_index,
                                generation: slot.generation,
                            },
                            value,
                        ))
                    }
                    SlotState::Occupied { .. } | SlotState::Free { .. } | SlotState::Retired => {
                        None
                    }
                }
            })
    }

    pub fn len(&self) -> usize {
        self.values.len()
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    fn indices(&self, id: DenseId) -> Option<(usize, usize)> {
        let slot_position = usize::try_from(id.index).ok()?;
        let slot = self.slots.get(slot_position)?;
        if slot.generation != id.generation {
            return None;
        }
        let dense_index = match slot.state {
            SlotState::Occupied { dense_index } => dense_index,
            SlotState::Free { .. } | SlotState::Retired => return None,
        };
        let dense_position = usize::try_from(dense_index).ok()?;
        (self.values.get(dense_position).is_some()
            && self.dense_to_slot.get(dense_position) == Some(&id.index))
        .then_some((slot_position, dense_position))
    }
}

impl<T> Default for DensePool<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "dense_pool_tests.rs"]
mod tests;
