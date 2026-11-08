use std::{collections::HashSet, fmt};

use super::{BodyId, ChunkId, ColliderEntry, ColliderId, PhysicsWorld, WorldError};
use crate::{chunk::Chunk, collision::CollisionError, rigid_body::BodyType};

pub(super) struct ChunkEntry {
    body: BodyId,
    chunk: Chunk,
    ids: Vec<ColliderId>,
}

/// Reason a chunk operation was rejected before changing registered geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkOperationError {
    World(WorldError),
    InvalidChunkId,
    DuplicateChunkId,
}

impl fmt::Display for ChunkOperationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::World(error) => write!(f, "{error}"),
            Self::InvalidChunkId => f.write_str("chunk ID is invalid for this world"),
            Self::DuplicateChunkId => f.write_str("chunk removal contains a duplicate ID"),
        }
    }
}

impl std::error::Error for ChunkOperationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::World(error) => Some(error),
            _ => None,
        }
    }
}

impl From<WorldError> for ChunkOperationError {
    fn from(error: WorldError) -> Self {
        Self::World(error)
    }
}

/// Failed registration, retaining the unaccepted prepared chunk.
pub struct AddChunkError {
    pub cause: ChunkOperationError,
    pub chunk: Chunk,
}

/// Failed batch registration, retaining every input chunk in input order.
pub struct AddChunkBatchError {
    pub cause: ChunkOperationError,
    pub chunks: Vec<Chunk>,
}

impl fmt::Debug for AddChunkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AddChunkError")
            .field("cause", &self.cause)
            .finish_non_exhaustive()
    }
}

impl fmt::Debug for AddChunkBatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AddChunkBatchError")
            .field("cause", &self.cause)
            .field("chunks", &self.chunks.len())
            .finish()
    }
}

impl fmt::Display for AddChunkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.cause.fmt(f)
    }
}
impl fmt::Display for AddChunkBatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.cause.fmt(f)
    }
}
impl std::error::Error for AddChunkError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.cause)
    }
}
impl std::error::Error for AddChunkBatchError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.cause)
    }
}

pub type RemoveChunkBatchError = ChunkOperationError;

fn capacity() -> ChunkOperationError {
    WorldError::DensePoolCapacityExceeded.into()
}

impl PhysicsWorld {
    pub fn chunk_count(&self) -> usize {
        self.chunks.len()
    }

    pub fn chunk_body(&self, id: ChunkId) -> Option<BodyId> {
        (id.world == self.world_id)
            .then(|| self.chunks.get(id.dense))
            .flatten()
            .map(|entry| entry.body)
    }

    /// Chunk-owned colliders cannot be removed or mutably borrowed individually.
    pub fn chunk_colliders(&self, id: ChunkId) -> Option<&[ColliderId]> {
        (id.world == self.world_id)
            .then(|| self.chunks.get(id.dense))
            .flatten()
            .map(|entry| entry.ids.as_slice())
    }

    pub(super) fn has_body_chunks(&self, body: BodyId) -> bool {
        self.chunks.iter().any(|(_, entry)| entry.body == body)
    }

    pub(super) fn remove_body_chunks(&mut self, body: BodyId) {
        loop {
            let id = self
                .chunks
                .iter()
                .find_map(|(id, entry)| (entry.body == body).then_some(id));
            let Some(id) = id else {
                break;
            };
            self.chunks.remove(id);
        }
    }

    // Returning the owned chunk avoids allocating a box on the failure path.
    #[allow(clippy::result_large_err)]
    pub fn add_chunk(&mut self, body: BodyId, chunk: Chunk) -> Result<ChunkId, AddChunkError> {
        let mut entry = ChunkEntry {
            body,
            chunk,
            ids: Vec::new(),
        };
        let mut colliders = Vec::new();
        let preflight = (|| -> Result<(), ChunkOperationError> {
            self.validate_chunk(body, &entry.chunk)?;
            self.chunks
                .try_reserve_insertions(1)
                .map_err(WorldError::from)?;
            self.colliders
                .try_reserve_insertions(entry.chunk.colliders.len())
                .map_err(WorldError::from)?;
            entry
                .ids
                .try_reserve_exact(entry.chunk.colliders.len())
                .map_err(|_| capacity())?;
            colliders
                .try_reserve_exact(entry.chunk.colliders.len())
                .map_err(|_| capacity())?;
            Ok(())
        })();
        if let Err(cause) = preflight {
            return Err(AddChunkError {
                cause,
                chunk: entry.chunk,
            });
        }
        for collider in entry.chunk.colliders.drain(..) {
            colliders.push(ColliderEntry {
                collider,
                body,
                prev: None,
                next: None,
                chunk: None,
            });
        }
        let ids = match self.colliders.try_insert_many_recover(&mut colliders) {
            Ok(ids) => ids,
            Err(error) => {
                entry
                    .chunk
                    .colliders
                    .extend(colliders.into_iter().map(|entry| entry.collider));
                return Err(AddChunkError {
                    cause: WorldError::from(error).into(),
                    chunk: entry.chunk,
                });
            }
        };
        match self.chunks.insert_recover(entry) {
            Ok(dense) => {
                let id = ChunkId {
                    world: self.world_id,
                    dense,
                };
                for collider in ids {
                    self.link_chunk_collider(body, id, collider);
                }
                self.mark_query_index_dirty();
                self.collision_system.mark_spatial_dirty();
                Ok(id)
            }
            Err((error, mut entry)) => {
                for id in ids {
                    if let Some(collider) = self.colliders.remove(id) {
                        entry.chunk.colliders.push(collider.collider);
                    }
                }
                Err(AddChunkError {
                    cause: WorldError::from(error).into(),
                    chunk: entry.chunk,
                })
            }
        }
    }

    fn validate_chunk(&self, body: BodyId, chunk: &Chunk) -> Result<(), ChunkOperationError> {
        let entry = self
            .bodies
            .get(body.dense)
            .filter(|_| body.world == self.world_id)
            .ok_or(WorldError::InvalidBodyId)?;
        if entry.cluster.is_some() {
            return Err(WorldError::InvalidBodyId.into());
        }
        if entry.body.physics().body_type() == BodyType::Dynamic {
            return Err(WorldError::Collision(CollisionError::UnsupportedDynamicGeometry).into());
        }
        if let Some(first) = entry.first_collider {
            let first = self
                .colliders
                .get(first.dense)
                .ok_or(WorldError::InvalidColliderId)?;
            if first.body != body || first.prev.is_some() {
                return Err(WorldError::InvalidColliderId.into());
            }
        }
        for collider in &chunk.colliders {
            if collider
                .material()
                .is_some_and(|id| !self.materials.contains(id))
            {
                return Err(WorldError::InvalidMaterialId.into());
            }
            collider
                .world_bounds(&entry.body.state().transform())
                .map_err(|_| WorldError::Collision(CollisionError::GeometryBounds))?;
        }
        Ok(())
    }

    fn link_chunk_collider(
        &mut self,
        body: BodyId,
        chunk: ChunkId,
        dense: crate::containers::DenseId,
    ) {
        let id = ColliderId {
            world: self.world_id,
            dense,
        };
        let first = self
            .bodies
            .get(body.dense)
            .and_then(|entry| entry.first_collider);
        if let Some(entry) = self.colliders.get_mut(dense) {
            entry.chunk = Some(chunk);
            entry.next = first;
        }
        if let Some(first) = first
            && let Some(entry) = self.colliders.get_mut(first.dense)
        {
            entry.prev = Some(id);
        }
        if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.first_collider = Some(id);
        }
        if let Some(entry) = self.chunks.get_mut(chunk.dense) {
            entry.ids.push(id);
        }
    }

    /// Registers all chunks or retains the previous registered geometry.
    /// On error every input chunk and its prepared BVHs are returned.
    pub fn add_chunk_batch(
        &mut self,
        body: BodyId,
        chunks: Vec<Chunk>,
    ) -> Result<Vec<ChunkId>, AddChunkBatchError> {
        self.add_chunk_batch_inner(
            body,
            chunks,
            #[cfg(test)]
            None,
        )
    }

    fn add_chunk_batch_inner(
        &mut self,
        body: BodyId,
        mut chunks: Vec<Chunk>,
        #[cfg(test)] fail_after: Option<usize>,
    ) -> Result<Vec<ChunkId>, AddChunkBatchError> {
        let mut entries = Vec::new();
        let mut collider_entries = Vec::new();
        let mut counts = Vec::new();
        let mut result = Vec::new();
        let preflight = (|| -> Result<(), ChunkOperationError> {
            if chunks.is_empty() {
                return Ok(());
            }
            let mut count = 0usize;
            for chunk in &chunks {
                self.validate_chunk(body, chunk)?;
                count = count
                    .checked_add(chunk.colliders.len())
                    .ok_or_else(capacity)?;
            }
            self.chunks
                .try_reserve_insertions(chunks.len())
                .map_err(WorldError::from)?;
            self.colliders
                .try_reserve_insertions(count)
                .map_err(WorldError::from)?;
            entries
                .try_reserve_exact(chunks.len())
                .map_err(|_| capacity())?;
            counts
                .try_reserve_exact(chunks.len())
                .map_err(|_| capacity())?;
            result
                .try_reserve_exact(chunks.len())
                .map_err(|_| capacity())?;
            collider_entries
                .try_reserve_exact(count)
                .map_err(|_| capacity())?;
            for chunk in &chunks {
                let mut ids = Vec::new();
                ids.try_reserve_exact(chunk.colliders.len())
                    .map_err(|_| capacity())?;
                counts.push((chunk.colliders.len(), ids));
            }
            Ok(())
        })();
        if let Err(cause) = preflight {
            return Err(AddChunkBatchError { cause, chunks });
        }
        if chunks.is_empty() {
            return Ok(Vec::new());
        }
        for (chunk, (_, ids)) in chunks.drain(..).zip(counts.iter_mut()) {
            entries.push(ChunkEntry {
                body,
                chunk,
                ids: std::mem::take(ids),
            });
        }
        let chunk_ids = match self.chunks.try_insert_many_recover(&mut entries) {
            Ok(ids) => ids,
            Err(error) => {
                chunks.extend(entries.into_iter().map(|entry| entry.chunk));
                return Err(AddChunkBatchError {
                    cause: WorldError::from(error).into(),
                    chunks,
                });
            }
        };
        for &dense in &chunk_ids {
            if let Some(entry) = self.chunks.get_mut(dense) {
                for collider in entry.chunk.colliders.drain(..) {
                    collider_entries.push(ColliderEntry {
                        collider,
                        body,
                        prev: None,
                        next: None,
                        chunk: Some(ChunkId {
                            world: self.world_id,
                            dense,
                        }),
                    });
                }
            }
        }
        #[cfg(test)]
        let inserted = match fail_after {
            Some(count) => self
                .colliders
                .try_insert_many_recover_failing_after(&mut collider_entries, count),
            None => self
                .colliders
                .try_insert_many_recover(&mut collider_entries),
        };
        #[cfg(not(test))]
        let inserted = self
            .colliders
            .try_insert_many_recover(&mut collider_entries);
        let collider_ids = match inserted {
            Ok(ids) => ids,
            Err(error) => {
                let mut colliders = collider_entries.into_iter();
                for (&dense, (count, _)) in chunk_ids.iter().zip(&counts) {
                    if let Some(mut entry) = self.chunks.remove(dense) {
                        entry
                            .chunk
                            .colliders
                            .extend(colliders.by_ref().take(*count).map(|entry| entry.collider));
                        chunks.push(entry.chunk);
                    }
                }
                return Err(AddChunkBatchError {
                    cause: WorldError::from(error).into(),
                    chunks,
                });
            }
        };
        // All allocations and validation are complete. The IDs below were just
        // created in exclusively borrowed pools, so each lookup must resolve.
        let mut colliders = collider_ids.into_iter();
        for (dense, (count, _)) in chunk_ids.into_iter().zip(counts) {
            let id = ChunkId {
                world: self.world_id,
                dense,
            };
            for collider_dense in colliders.by_ref().take(count) {
                self.link_chunk_collider(body, id, collider_dense);
            }
            result.push(id);
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        Ok(result)
    }

    pub fn remove_chunk(&mut self, id: ChunkId) -> Option<Chunk> {
        self.validate_chunk_removal(id).ok()?;
        self.mark_query_index_dirty();
        let mut entry = self.chunks.remove(id.dense)?;
        for collider in entry.ids {
            entry
                .chunk
                .colliders
                .push(self.remove_collider_internal(collider)?);
        }
        Some(entry.chunk)
    }

    fn validate_chunk_removal(&self, id: ChunkId) -> Result<(), ChunkOperationError> {
        let entry = self
            .chunks
            .get(id.dense)
            .filter(|_| id.world == self.world_id)
            .ok_or(ChunkOperationError::InvalidChunkId)?;
        let body = self
            .bodies
            .get(entry.body.dense)
            .ok_or(WorldError::InvalidBodyId)?;
        if body.cluster.is_some() {
            return Err(WorldError::InvalidBodyId.into());
        }
        for &collider in &entry.ids {
            let node = self
                .colliders
                .get(collider.dense)
                .filter(|node| node.chunk == Some(id) && node.body == entry.body)
                .ok_or(WorldError::InvalidColliderId)?;
            if node.prev.is_none() && body.first_collider != Some(collider) {
                return Err(WorldError::InvalidColliderId.into());
            }
            if node.prev.is_some_and(|prev| {
                self.colliders
                    .get(prev.dense)
                    .is_none_or(|prev| prev.body != entry.body || prev.next != Some(collider))
            }) || node.next.is_some_and(|next| {
                self.colliders
                    .get(next.dense)
                    .is_none_or(|next| next.body != entry.body || next.prev != Some(collider))
            }) {
                return Err(WorldError::InvalidColliderId.into());
            }
        }
        Ok(())
    }

    /// Validates every ID (including duplicates) before removing any geometry.
    /// Returned chunks keep their original mesh order, transforms and BVHs.
    pub fn remove_chunk_batch(
        &mut self,
        ids: Vec<ChunkId>,
    ) -> Result<Vec<Chunk>, RemoveChunkBatchError> {
        let mut seen = HashSet::new();
        seen.try_reserve(ids.len()).map_err(|_| capacity())?;
        let mut result = Vec::new();
        result
            .try_reserve_exact(ids.len())
            .map_err(|_| capacity())?;
        for &id in &ids {
            if !seen.insert(id) {
                return Err(ChunkOperationError::DuplicateChunkId);
            }
            self.validate_chunk_removal(id)?;
        }
        if ids.is_empty() {
            return Ok(result);
        }
        // Preflight proves every list link is valid. Removing a node preserves
        // that invariant for the remaining nodes, including adjacent removals.
        self.mark_query_index_dirty();
        for id in ids {
            let mut entry = self
                .chunks
                .remove(id.dense)
                .ok_or(ChunkOperationError::InvalidChunkId)?;
            for collider in entry.ids {
                let collider = self
                    .remove_collider_internal(collider)
                    .ok_or(WorldError::InvalidColliderId)?;
                entry.chunk.colliders.push(collider);
            }
            result.push(entry.chunk);
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests;
