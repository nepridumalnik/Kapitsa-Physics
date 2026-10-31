use crate::rigid_body::BodyType;

use super::{CollisionError, solver::SolverManifold};

/// Reusable deterministic body and constraint labels for independent islands.
#[derive(Default)]
pub(crate) struct IslandLayout {
    parents: Vec<usize>,
    root_islands: Vec<Option<usize>>,
    body_islands: Vec<Option<usize>>,
    connected: Vec<bool>,
    manifold_islands: Vec<Option<usize>>,
    island_count: usize,
}

impl IslandLayout {
    pub(crate) fn build(
        &mut self,
        body_types: &[BodyType],
        manifolds: &[SolverManifold],
    ) -> Result<(), CollisionError> {
        self.parents.clear();
        self.parents
            .try_reserve(body_types.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.parents.extend(0..body_types.len());
        self.root_islands.clear();
        self.root_islands
            .try_reserve(body_types.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.root_islands.resize(body_types.len(), None);
        self.body_islands.clear();
        self.body_islands
            .try_reserve(body_types.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.body_islands.resize(body_types.len(), None);
        self.connected.clear();
        self.connected
            .try_reserve(body_types.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.connected.resize(body_types.len(), false);
        self.manifold_islands.clear();
        self.manifold_islands
            .try_reserve(manifolds.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.manifold_islands.resize(manifolds.len(), None);
        self.island_count = 0;

        for manifold in manifolds {
            let a = manifold.body_a;
            let b = manifold.body_b;
            if a == b || a >= body_types.len() || b >= body_types.len() {
                return Err(CollisionError::InvalidContact);
            }
            if body_types[a] == BodyType::Dynamic {
                self.connected[a] = true;
            }
            if body_types[b] == BodyType::Dynamic {
                self.connected[b] = true;
            }
            if body_types[a] == BodyType::Dynamic && body_types[b] == BodyType::Dynamic {
                union(&mut self.parents, a, b)?;
            }
        }

        for (body, body_type) in body_types.iter().copied().enumerate() {
            if body_type != BodyType::Dynamic || !self.connected[body] {
                continue;
            }
            let root = find(&mut self.parents, body)?;
            let island = match self.root_islands.get(root).copied().flatten() {
                Some(island) => island,
                None => {
                    let island = self.island_count;
                    self.island_count += 1;
                    *self
                        .root_islands
                        .get_mut(root)
                        .ok_or(CollisionError::InvalidContact)? = Some(island);
                    island
                }
            };
            *self
                .body_islands
                .get_mut(body)
                .ok_or(CollisionError::InvalidContact)? = Some(island);
        }

        for (index, manifold) in manifolds.iter().enumerate() {
            let body_a = manifold.body_a;
            let body_b = manifold.body_b;
            let a_dynamic = body_types[body_a] == BodyType::Dynamic;
            let b_dynamic = body_types[body_b] == BodyType::Dynamic;
            let island = match (a_dynamic, b_dynamic) {
                (true, true) => {
                    let root_a = find(&mut self.parents, body_a)?;
                    let root_b = find(&mut self.parents, body_b)?;
                    if root_a != root_b {
                        return Err(CollisionError::InvalidContact);
                    }
                    self.root_islands
                        .get(root_a)
                        .copied()
                        .flatten()
                        .ok_or(CollisionError::InvalidContact)?
                }
                (true, false) => self
                    .body_islands
                    .get(body_a)
                    .copied()
                    .flatten()
                    .ok_or(CollisionError::InvalidContact)?,
                (false, true) => self
                    .body_islands
                    .get(body_b)
                    .copied()
                    .flatten()
                    .ok_or(CollisionError::InvalidContact)?,
                (false, false) => continue,
            };
            *self
                .manifold_islands
                .get_mut(index)
                .ok_or(CollisionError::InvalidContact)? = Some(island);
        }
        Ok(())
    }

    pub(crate) fn island_count(&self) -> usize {
        self.island_count
    }

    pub(crate) fn body_island(&self, body: usize) -> Option<usize> {
        self.body_islands.get(body).copied().flatten()
    }

    pub(crate) fn manifold_island(&self, manifold: usize) -> Option<usize> {
        self.manifold_islands.get(manifold).copied().flatten()
    }
}

fn find(parents: &mut [usize], mut body: usize) -> Result<usize, CollisionError> {
    let mut root = body;
    loop {
        let parent = *parents.get(root).ok_or(CollisionError::InvalidContact)?;
        if parent == root {
            break;
        }
        root = parent;
    }
    while body != root {
        let parent = *parents.get(body).ok_or(CollisionError::InvalidContact)?;
        *parents
            .get_mut(body)
            .ok_or(CollisionError::InvalidContact)? = root;
        body = parent;
    }
    Ok(root)
}

fn union(parents: &mut [usize], left: usize, right: usize) -> Result<(), CollisionError> {
    let root_left = find(parents, left)?;
    let root_right = find(parents, right)?;
    if root_left == root_right {
        return Ok(());
    }
    let (root, child) = if root_left < root_right {
        (root_left, root_right)
    } else {
        (root_right, root_left)
    };
    *parents
        .get_mut(child)
        .ok_or(CollisionError::InvalidContact)? = root;
    Ok(())
}

#[cfg(test)]
#[path = "island_tests.rs"]
mod tests;
