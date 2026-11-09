use super::CollisionError;

/// Stable conflict-free groups for one solver island.
///
/// Contacts and joints share the same endpoint list. Dynamic bodies define
/// conflicts, so constraints on static or kinematic bodies can share a color.
#[derive(Default)]
pub(crate) struct ConstraintColoring {
    dynamic_bodies: Vec<bool>,
    endpoints: Vec<[usize; 2]>,
    body_offsets: Vec<usize>,
    body_constraints: Vec<usize>,
    scratch_offsets: Vec<usize>,
    constraint_colors: Vec<usize>,
    color_stamps: Vec<usize>,
    body_stamps: Vec<usize>,
    color_offsets: Vec<usize>,
    colored_constraints: Vec<usize>,
    color_body_offsets: Vec<usize>,
    color_bodies: Vec<usize>,
    color_count: usize,
    max_color_constraint_count: usize,
}

impl ConstraintColoring {
    pub(crate) const fn new() -> Self {
        Self {
            dynamic_bodies: Vec::new(),
            endpoints: Vec::new(),
            body_offsets: Vec::new(),
            body_constraints: Vec::new(),
            scratch_offsets: Vec::new(),
            constraint_colors: Vec::new(),
            color_stamps: Vec::new(),
            body_stamps: Vec::new(),
            color_offsets: Vec::new(),
            colored_constraints: Vec::new(),
            color_body_offsets: Vec::new(),
            color_bodies: Vec::new(),
            color_count: 0,
            max_color_constraint_count: 0,
        }
    }

    pub(crate) fn update(
        &mut self,
        dynamic_bodies: &[bool],
        endpoints: &[[usize; 2]],
    ) -> Result<bool, CollisionError> {
        if self.dynamic_bodies == dynamic_bodies && self.endpoints == endpoints {
            return Ok(false);
        }
        if endpoints
            .iter()
            .any(|[a, b]| a == b || *a >= dynamic_bodies.len() || *b >= dynamic_bodies.len())
        {
            return Err(CollisionError::InvalidContact);
        }

        reserve(&mut self.dynamic_bodies, dynamic_bodies.len())?;
        self.dynamic_bodies.clear();
        self.dynamic_bodies.extend_from_slice(dynamic_bodies);
        reserve(&mut self.endpoints, endpoints.len())?;
        self.endpoints.clear();
        self.endpoints.extend_from_slice(endpoints);
        match self.rebuild() {
            Ok(()) => Ok(true),
            Err(error) => {
                self.dynamic_bodies.clear();
                self.endpoints.clear();
                self.color_count = 0;
                self.max_color_constraint_count = 0;
                Err(error)
            }
        }
    }

    #[cfg(any(feature = "bench-internals", test))]
    pub(crate) fn color_count(&self) -> usize {
        self.color_count
    }

    pub(crate) fn schedule_group_count(&self) -> usize {
        self.color_count
    }

    pub(crate) fn max_color_constraint_count(&self) -> usize {
        self.max_color_constraint_count
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn constraint_count(&self) -> usize {
        self.endpoints.len()
    }

    pub(crate) fn constraints_in_schedule_group(&self, group: usize) -> Option<&[usize]> {
        let start = *self.color_offsets.get(group)?;
        let end = *self.color_offsets.get(group + 1)?;
        self.colored_constraints.get(start..end)
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        self.dynamic_bodies.capacity() * size_of::<bool>()
            + self.endpoints.capacity() * size_of::<[usize; 2]>()
            + self.body_offsets.capacity() * size_of::<usize>()
            + self.body_constraints.capacity() * size_of::<usize>()
            + self.scratch_offsets.capacity() * size_of::<usize>()
            + self.constraint_colors.capacity() * size_of::<usize>()
            + self.color_stamps.capacity() * size_of::<usize>()
            + self.body_stamps.capacity() * size_of::<usize>()
            + self.color_offsets.capacity() * size_of::<usize>()
            + self.colored_constraints.capacity() * size_of::<usize>()
            + self.color_body_offsets.capacity() * size_of::<usize>()
            + self.color_bodies.capacity() * size_of::<usize>()
    }

    fn rebuild(&mut self) -> Result<(), CollisionError> {
        let body_count = self.dynamic_bodies.len();
        let constraint_count = self.endpoints.len();
        reserve(&mut self.body_offsets, body_count.saturating_add(1))?;
        self.body_offsets.clear();
        self.body_offsets.resize(body_count + 1, 0);
        for [a, b] in self.endpoints.iter().copied() {
            for body in [a, b] {
                if self.dynamic_bodies[body] {
                    self.body_offsets[body + 1] = self.body_offsets[body + 1]
                        .checked_add(1)
                        .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
                }
            }
        }
        for body in 0..body_count {
            self.body_offsets[body + 1] = self.body_offsets[body + 1]
                .checked_add(self.body_offsets[body])
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        let adjacency_count = *self
            .body_offsets
            .last()
            .ok_or(CollisionError::InvalidContact)?;
        reserve(&mut self.body_constraints, adjacency_count)?;
        self.body_constraints.resize(adjacency_count, 0);
        reserve(&mut self.scratch_offsets, body_count)?;
        self.scratch_offsets.clear();
        self.scratch_offsets
            .extend_from_slice(&self.body_offsets[..body_count]);
        for (constraint, [a, b]) in self.endpoints.iter().copied().enumerate() {
            for body in [a, b] {
                if self.dynamic_bodies[body] {
                    let slot = self.scratch_offsets[body];
                    self.body_constraints[slot] = constraint;
                    self.scratch_offsets[body] += 1;
                }
            }
        }

        reserve(&mut self.constraint_colors, constraint_count)?;
        self.constraint_colors.resize(constraint_count, usize::MAX);
        reserve(&mut self.color_stamps, constraint_count)?;
        self.color_stamps.resize(constraint_count, 0);
        self.color_stamps.fill(0);
        let mut color_count = 0;
        for constraint in 0..constraint_count {
            let stamp = constraint + 1;
            for body in self.endpoints[constraint] {
                if !self.dynamic_bodies[body] {
                    continue;
                }
                for neighbor in self.body_constraints
                    [self.body_offsets[body]..self.body_offsets[body + 1]]
                    .iter()
                    .copied()
                {
                    if neighbor < constraint {
                        self.color_stamps[self.constraint_colors[neighbor]] = stamp;
                    }
                }
            }
            let color = (0..=color_count)
                .find(|color| self.color_stamps[*color] != stamp)
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
            self.constraint_colors[constraint] = color;
            color_count = color_count.max(color + 1);
        }

        reserve(&mut self.color_offsets, color_count.saturating_add(1))?;
        self.color_offsets.clear();
        self.color_offsets.resize(color_count + 1, 0);
        for color in self.constraint_colors.iter().copied() {
            self.color_offsets[color + 1] = self.color_offsets[color + 1]
                .checked_add(1)
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        for color in 0..color_count {
            self.color_offsets[color + 1] = self.color_offsets[color + 1]
                .checked_add(self.color_offsets[color])
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        reserve(&mut self.colored_constraints, constraint_count)?;
        self.colored_constraints.resize(constraint_count, 0);
        self.scratch_offsets.clear();
        self.scratch_offsets
            .extend_from_slice(&self.color_offsets[..color_count]);
        for constraint in 0..constraint_count {
            let color = self.constraint_colors[constraint];
            let slot = self.scratch_offsets[color];
            self.colored_constraints[slot] = constraint;
            self.scratch_offsets[color] += 1;
        }
        self.max_color_constraint_count = self
            .color_offsets
            .windows(2)
            .map(|offsets| offsets[1] - offsets[0])
            .max()
            .unwrap_or(0);
        self.color_count = color_count;
        self.rebuild_color_body_plan(body_count)?;

        // This check is unconditional because the solver's concurrent body access
        // relies on each dynamic body having exactly one owner per color.
        if !self.has_unique_dynamic_body_ownership(body_count)? {
            return Err(CollisionError::InvalidContact);
        }
        Ok(())
    }

    /// Reusable index-only ownership plan for the safe binding phase.
    /// References are intentionally not stored: they must be materialized
    /// while one color exclusively borrows the solver-body slice.
    fn rebuild_color_body_plan(&mut self, body_count: usize) -> Result<(), CollisionError> {
        reserve(
            &mut self.color_body_offsets,
            self.color_count.saturating_add(1),
        )?;
        self.color_body_offsets.clear();
        self.color_body_offsets.resize(self.color_count + 1, 0);
        reserve(
            &mut self.color_bodies,
            self.endpoints.len().saturating_mul(2),
        )?;
        self.color_bodies.clear();
        reserve(&mut self.body_stamps, body_count)?;
        self.body_stamps.resize(body_count, 0);
        self.body_stamps.fill(0);

        for color in 0..self.color_count {
            let stamp = color + 1;
            let start = self.color_offsets[color];
            let end = self.color_offsets[color + 1];
            for &constraint in &self.colored_constraints[start..end] {
                for body in self.endpoints[constraint] {
                    if self.body_stamps[body] != stamp {
                        self.body_stamps[body] = stamp;
                        self.color_bodies.push(body);
                    }
                }
            }
            let color_start = self.color_body_offsets[color];
            self.color_bodies[color_start..].sort_unstable();
            self.color_body_offsets[color + 1] = self.color_bodies.len();
        }
        Ok(())
    }

    pub(crate) fn bodies_in_schedule_group(&self, group: usize) -> Option<&[usize]> {
        let start = *self.color_body_offsets.get(group)?;
        let end = *self.color_body_offsets.get(group + 1)?;
        self.color_bodies.get(start..end)
    }

    fn has_unique_dynamic_body_ownership(
        &mut self,
        body_count: usize,
    ) -> Result<bool, CollisionError> {
        reserve(&mut self.body_stamps, body_count)?;
        self.body_stamps.resize(body_count, 0);
        self.body_stamps.fill(0);
        for color in 0..self.color_count {
            let start = self.color_offsets[color];
            let end = self.color_offsets[color + 1];
            let stamp = color + 1;
            for slot in start..end {
                let constraint = self.colored_constraints[slot];
                for body in self.endpoints[constraint] {
                    if !self.dynamic_bodies[body] {
                        continue;
                    }
                    if self.body_stamps[body] == stamp {
                        return Ok(false);
                    }
                    self.body_stamps[body] = stamp;
                }
            }
        }
        Ok(true)
    }
}

fn reserve<T>(values: &mut Vec<T>, target_len: usize) -> Result<(), CollisionError> {
    if values.capacity() < target_len {
        values
            .try_reserve(target_len - values.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::ConstraintColoring;

    #[test]
    fn colors_constraints_across_the_whole_island() {
        let dynamic_bodies = [true, true, true, false];
        let endpoints = [[0, 3], [1, 3], [0, 1], [1, 2]];
        let mut coloring = ConstraintColoring::default();

        assert!(coloring.update(&dynamic_bodies, &endpoints).unwrap());
        assert_eq!(coloring.color_count(), 3);
        assert_eq!(coloring.schedule_group_count(), 3);
        assert_eq!(coloring.constraints_in_schedule_group(0), Some(&[0, 1][..]));
        assert_eq!(coloring.constraints_in_schedule_group(1), Some(&[2][..]));
        assert_eq!(coloring.constraints_in_schedule_group(2), Some(&[3][..]));
        assert_eq!(coloring.bodies_in_schedule_group(0), Some(&[0, 1, 3][..]));
        assert_eq!(coloring.bodies_in_schedule_group(1), Some(&[0, 1][..]));
        assert_eq!(coloring.bodies_in_schedule_group(2), Some(&[1, 2][..]));
        assert!(
            coloring
                .has_unique_dynamic_body_ownership(dynamic_bodies.len())
                .unwrap()
        );
    }

    #[test]
    fn colors_adjacent_chain_constraints_in_two_deterministic_groups() {
        let mut coloring = ConstraintColoring::default();
        let endpoints = [[0, 1], [1, 2], [2, 3], [3, 4]];

        assert!(
            coloring
                .update(&[true, true, true, true, true], &endpoints)
                .unwrap()
        );
        assert_eq!(coloring.color_count(), 2);
        assert_eq!(coloring.schedule_group_count(), 2);
        assert_eq!(coloring.constraints_in_schedule_group(0), Some(&[0, 2][..]));
        assert_eq!(coloring.constraints_in_schedule_group(1), Some(&[1, 3][..]));
        assert_eq!(
            coloring.bodies_in_schedule_group(0),
            Some(&[0, 1, 2, 3][..])
        );
        assert_eq!(
            coloring.bodies_in_schedule_group(1),
            Some(&[1, 2, 3, 4][..])
        );
        assert!(coloring.has_unique_dynamic_body_ownership(5).unwrap());
    }

    #[test]
    fn reuses_colors_until_dynamic_types_or_constraint_endpoints_change() {
        let mut coloring = ConstraintColoring::default();
        let endpoints = [[0, 1], [1, 2]];
        assert!(coloring.update(&[true, true, true], &endpoints).unwrap());
        assert!(!coloring.update(&[true, true, true], &endpoints).unwrap());
        assert!(coloring.update(&[true, false, true], &endpoints).unwrap());
        assert!(coloring.update(&[true, false, true], &[[0, 1]]).unwrap());
    }

    #[test]
    fn rejects_invalid_constraint_endpoints() {
        let mut coloring = ConstraintColoring::default();
        assert!(coloring.update(&[true], &[[0, 1]]).is_err());
        assert!(coloring.update(&[true], &[[0, 0]]).is_err());
    }
}
