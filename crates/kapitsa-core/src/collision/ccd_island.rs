use crate::rigid_body::BodyType;

use super::{CollisionError, island};

/// Groups CCD candidates through dynamic bodies while leaving shared static and
/// kinematic surfaces outside the connectivity graph.
#[derive(Default)]
pub(crate) struct CcdIslandLayout {
    parents: Vec<usize>,
    root_islands: Vec<Option<usize>>,
    body_islands: Vec<Option<usize>>,
    connected: Vec<bool>,
    candidate_islands: Vec<Option<usize>>,
    candidate_offsets: Vec<usize>,
    candidate_indices: Vec<usize>,
    island_body_offsets: Vec<usize>,
    island_body_indices: Vec<usize>,
    write_offsets: Vec<usize>,
    body_candidate_offsets: Vec<usize>,
    body_candidate_indices: Vec<usize>,
    island_count: usize,
}

impl CcdIslandLayout {
    pub(crate) const fn new() -> Self {
        Self {
            parents: Vec::new(),
            root_islands: Vec::new(),
            body_islands: Vec::new(),
            connected: Vec::new(),
            candidate_islands: Vec::new(),
            candidate_offsets: Vec::new(),
            candidate_indices: Vec::new(),
            island_body_offsets: Vec::new(),
            island_body_indices: Vec::new(),
            write_offsets: Vec::new(),
            body_candidate_offsets: Vec::new(),
            body_candidate_indices: Vec::new(),
            island_count: 0,
        }
    }

    #[cfg(test)]
    pub(crate) fn build(
        &mut self,
        body_types: &[BodyType],
        continuous_bodies: &[bool],
        candidate_bodies: &[(usize, usize)],
        force_single_island: bool,
    ) -> Result<(), CollisionError> {
        self.build_inner(
            body_types,
            continuous_bodies,
            candidate_bodies,
            None,
            force_single_island,
        )
    }

    pub(crate) fn build_active(
        &mut self,
        body_types: &[BodyType],
        continuous_bodies: &[bool],
        candidate_bodies: &[(usize, usize)],
        active_candidates: &[bool],
        force_single_island: bool,
    ) -> Result<(), CollisionError> {
        self.build_inner(
            body_types,
            continuous_bodies,
            candidate_bodies,
            Some(active_candidates),
            force_single_island,
        )
    }

    fn build_inner(
        &mut self,
        body_types: &[BodyType],
        continuous_bodies: &[bool],
        candidate_bodies: &[(usize, usize)],
        active_candidates: Option<&[bool]>,
        force_single_island: bool,
    ) -> Result<(), CollisionError> {
        if body_types.len() != continuous_bodies.len() {
            return Err(CollisionError::InvalidContact);
        }
        if active_candidates.is_some_and(|active| active.len() != candidate_bodies.len()) {
            return Err(CollisionError::InvalidContact);
        }

        self.parents.clear();
        self.parents
            .try_reserve(body_types.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.parents.extend(0..body_types.len());
        resize_with_reserve(&mut self.root_islands, body_types.len(), None)?;
        resize_with_reserve(&mut self.body_islands, body_types.len(), None)?;
        resize_with_reserve(&mut self.connected, body_types.len(), false)?;
        resize_with_reserve(&mut self.candidate_islands, candidate_bodies.len(), None)?;
        self.root_islands.fill(None);
        self.body_islands.fill(None);
        self.connected.fill(false);
        self.candidate_islands.fill(None);
        self.island_count = 0;

        for (candidate_index, &(body_a, body_b)) in candidate_bodies.iter().enumerate() {
            let type_a = *body_types
                .get(body_a)
                .ok_or(CollisionError::InvalidContact)?;
            let type_b = *body_types
                .get(body_b)
                .ok_or(CollisionError::InvalidContact)?;
            if body_a == body_b
                || (continuous_bodies[body_a] && type_a != BodyType::Dynamic)
                || (continuous_bodies[body_b] && type_b != BodyType::Dynamic)
            {
                return Err(CollisionError::InvalidContact);
            }
            if !active_candidates.is_none_or(|active| active[candidate_index])
                || (!continuous_bodies[body_a] && !continuous_bodies[body_b])
            {
                continue;
            }

            if type_a == BodyType::Dynamic {
                self.connected[body_a] = true;
            }
            if type_b == BodyType::Dynamic {
                self.connected[body_b] = true;
            }
            if type_a == BodyType::Dynamic && type_b == BodyType::Dynamic {
                island::union(&mut self.parents, body_a, body_b)?;
            }
        }

        for (body, body_type) in body_types.iter().copied().enumerate() {
            if body_type != BodyType::Dynamic || !self.connected[body] {
                continue;
            }
            let root = island::find(&mut self.parents, body)?;
            let island_index = match self.root_islands[root] {
                Some(island_index) => island_index,
                None => {
                    let island_index = self.island_count;
                    self.island_count += 1;
                    self.root_islands[root] = Some(island_index);
                    island_index
                }
            };
            self.body_islands[body] = Some(island_index);
        }

        for (candidate_index, &(body_a, body_b)) in candidate_bodies.iter().enumerate() {
            if !active_candidates.is_none_or(|active| active[candidate_index])
                || (!continuous_bodies[body_a] && !continuous_bodies[body_b])
            {
                continue;
            }
            let island_a = (body_types[body_a] == BodyType::Dynamic)
                .then(|| self.body_islands[body_a])
                .flatten();
            let island_b = (body_types[body_b] == BodyType::Dynamic)
                .then(|| self.body_islands[body_b])
                .flatten();
            let island_index = match (island_a, island_b) {
                (Some(a), Some(b)) if a == b => a,
                (Some(_), Some(_)) => return Err(CollisionError::InvalidContact),
                (Some(island_index), None) | (None, Some(island_index)) => island_index,
                (None, None) => return Err(CollisionError::InvalidContact),
            };
            self.candidate_islands[candidate_index] = Some(island_index);
        }

        if force_single_island && self.candidate_islands.iter().any(Option::is_some) {
            self.island_count = 1;
            for (body, body_type) in body_types.iter().copied().enumerate() {
                self.body_islands[body] =
                    (body_type == BodyType::Dynamic && self.connected[body]).then_some(0);
            }
            for island in self.candidate_islands.iter_mut().flatten() {
                *island = 0;
            }
        }

        resize_with_reserve(&mut self.island_body_offsets, self.island_count + 1, 0)?;
        self.island_body_offsets.fill(0);
        for island in self.body_islands.iter().flatten().copied() {
            self.island_body_offsets[island + 1] = self.island_body_offsets[island + 1]
                .checked_add(1)
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        for island in 0..self.island_count {
            self.island_body_offsets[island + 1] = self.island_body_offsets[island + 1]
                .checked_add(self.island_body_offsets[island])
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        let island_body_count = *self
            .island_body_offsets
            .last()
            .ok_or(CollisionError::InvalidContact)?;
        resize_with_reserve(&mut self.island_body_indices, island_body_count, 0)?;
        resize_with_reserve(&mut self.write_offsets, self.island_count, 0)?;
        self.write_offsets
            .copy_from_slice(&self.island_body_offsets[..self.island_count]);
        for (body, island) in self.body_islands.iter().copied().enumerate() {
            let Some(island) = island else {
                continue;
            };
            let write_index = self.write_offsets[island];
            self.island_body_indices[write_index] = body;
            self.write_offsets[island] += 1;
        }

        resize_with_reserve(&mut self.candidate_offsets, self.island_count + 1, 0)?;
        self.candidate_offsets.fill(0);
        for island_index in self.candidate_islands.iter().flatten().copied() {
            let count = self
                .candidate_offsets
                .get_mut(island_index + 1)
                .ok_or(CollisionError::InvalidContact)?;
            *count = count
                .checked_add(1)
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        for island_index in 0..self.island_count {
            self.candidate_offsets[island_index + 1] = self.candidate_offsets[island_index + 1]
                .checked_add(self.candidate_offsets[island_index])
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        let candidate_count = *self
            .candidate_offsets
            .last()
            .ok_or(CollisionError::InvalidContact)?;
        resize_with_reserve(&mut self.candidate_indices, candidate_count, 0)?;
        resize_with_reserve(&mut self.write_offsets, self.island_count, 0)?;
        self.write_offsets
            .copy_from_slice(&self.candidate_offsets[..self.island_count]);
        for (candidate_index, island_index) in self.candidate_islands.iter().copied().enumerate() {
            let Some(island_index) = island_index else {
                continue;
            };
            let write_index = self.write_offsets[island_index];
            self.candidate_indices[write_index] = candidate_index;
            self.write_offsets[island_index] += 1;
        }

        resize_with_reserve(&mut self.body_candidate_offsets, body_types.len() + 1, 0)?;
        self.body_candidate_offsets.fill(0);
        for &(body_a, body_b) in candidate_bodies {
            if !continuous_bodies[body_a] && !continuous_bodies[body_b] {
                continue;
            }
            if body_types[body_a] == BodyType::Dynamic {
                self.body_candidate_offsets[body_a + 1] = self.body_candidate_offsets[body_a + 1]
                    .checked_add(1)
                    .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
            }
            if body_types[body_b] == BodyType::Dynamic {
                self.body_candidate_offsets[body_b + 1] = self.body_candidate_offsets[body_b + 1]
                    .checked_add(1)
                    .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
            }
        }
        for body in 0..body_types.len() {
            self.body_candidate_offsets[body + 1] = self.body_candidate_offsets[body + 1]
                .checked_add(self.body_candidate_offsets[body])
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        let body_candidate_count = *self
            .body_candidate_offsets
            .last()
            .ok_or(CollisionError::InvalidContact)?;
        resize_with_reserve(&mut self.body_candidate_indices, body_candidate_count, 0)?;
        resize_with_reserve(&mut self.write_offsets, body_types.len(), 0)?;
        self.write_offsets
            .copy_from_slice(&self.body_candidate_offsets[..body_types.len()]);
        for (candidate_index, &(body_a, body_b)) in candidate_bodies.iter().enumerate() {
            if !continuous_bodies[body_a] && !continuous_bodies[body_b] {
                continue;
            }
            for body in [body_a, body_b] {
                if body_types[body] != BodyType::Dynamic {
                    continue;
                }
                let write_index = self.write_offsets[body];
                self.body_candidate_indices[write_index] = candidate_index;
                self.write_offsets[body] += 1;
            }
        }
        Ok(())
    }

    pub(crate) fn island_count(&self) -> usize {
        self.island_count
    }

    #[cfg(test)]
    pub(crate) fn candidate_island(&self, candidate: usize) -> Option<usize> {
        self.candidate_islands.get(candidate).copied().flatten()
    }

    pub(crate) fn candidates(&self, island: usize) -> Option<&[usize]> {
        let start = *self.candidate_offsets.get(island)?;
        let end = *self.candidate_offsets.get(island + 1)?;
        self.candidate_indices.get(start..end)
    }

    pub(crate) fn bodies(&self, island: usize) -> Option<&[usize]> {
        let start = *self.island_body_offsets.get(island)?;
        let end = *self.island_body_offsets.get(island + 1)?;
        self.island_body_indices.get(start..end)
    }

    pub(crate) fn body_island(&self, body: usize) -> Option<usize> {
        self.body_islands.get(body).copied().flatten()
    }

    pub(crate) fn body_candidates(&self, body: usize) -> Option<&[usize]> {
        let start = *self.body_candidate_offsets.get(body)?;
        let end = *self.body_candidate_offsets.get(body + 1)?;
        self.body_candidate_indices.get(start..end)
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        self.parents.capacity() * size_of::<usize>()
            + self.root_islands.capacity() * size_of::<Option<usize>>()
            + self.body_islands.capacity() * size_of::<Option<usize>>()
            + self.connected.capacity() * size_of::<bool>()
            + self.candidate_islands.capacity() * size_of::<Option<usize>>()
            + self.candidate_offsets.capacity() * size_of::<usize>()
            + self.candidate_indices.capacity() * size_of::<usize>()
            + self.island_body_offsets.capacity() * size_of::<usize>()
            + self.island_body_indices.capacity() * size_of::<usize>()
            + self.write_offsets.capacity() * size_of::<usize>()
            + self.body_candidate_offsets.capacity() * size_of::<usize>()
            + self.body_candidate_indices.capacity() * size_of::<usize>()
    }
}

fn resize_with_reserve<T: Copy>(
    values: &mut Vec<T>,
    len: usize,
    value: T,
) -> Result<(), CollisionError> {
    if len > values.len() {
        values
            .try_reserve(len - values.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    values.resize(len, value);
    Ok(())
}

#[cfg(test)]
#[path = "ccd_island_tests.rs"]
mod tests;
