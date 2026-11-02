use super::CollisionError;

/// Stable level-ordered, conflict-free groups for one solver island.
///
/// Contacts and joints share the same endpoint list. Dynamic bodies define
/// conflicts; static and kinematic endpoints seed level zero. Unanchored
/// components use their first dynamic body as a deterministic root.
#[derive(Default)]
pub(crate) struct ConstraintColoring {
    dynamic_bodies: Vec<bool>,
    endpoints: Vec<[usize; 2]>,
    body_offsets: Vec<usize>,
    body_constraints: Vec<usize>,
    scratch_offsets: Vec<usize>,
    body_levels: Vec<usize>,
    level_queue: Vec<usize>,
    constraint_levels: Vec<usize>,
    level_order: Vec<usize>,
    level_offsets: Vec<usize>,
    level_color_counts: Vec<usize>,
    level_group_offsets: Vec<usize>,
    constraint_colors: Vec<usize>,
    color_stamps: Vec<usize>,
    body_stamps: Vec<usize>,
    color_offsets: Vec<usize>,
    colored_constraints: Vec<usize>,
    color_count: usize,
    schedule_group_count: usize,
    max_level_constraint_count: usize,
}

impl ConstraintColoring {
    pub(crate) const fn new() -> Self {
        Self {
            dynamic_bodies: Vec::new(),
            endpoints: Vec::new(),
            body_offsets: Vec::new(),
            body_constraints: Vec::new(),
            scratch_offsets: Vec::new(),
            body_levels: Vec::new(),
            level_queue: Vec::new(),
            constraint_levels: Vec::new(),
            level_order: Vec::new(),
            level_offsets: Vec::new(),
            level_color_counts: Vec::new(),
            level_group_offsets: Vec::new(),
            constraint_colors: Vec::new(),
            color_stamps: Vec::new(),
            body_stamps: Vec::new(),
            color_offsets: Vec::new(),
            colored_constraints: Vec::new(),
            color_count: 0,
            schedule_group_count: 0,
            max_level_constraint_count: 0,
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
            Ok(rebuilt) => Ok(rebuilt),
            Err(error) => {
                self.dynamic_bodies.clear();
                self.endpoints.clear();
                self.color_count = 0;
                self.schedule_group_count = 0;
                self.max_level_constraint_count = 0;
                Err(error)
            }
        }
    }

    /// Maximum number of colors used by any one dependency level.
    #[cfg(any(feature = "bench-internals", test))]
    pub(crate) fn color_count(&self) -> usize {
        self.color_count
    }

    pub(crate) fn schedule_group_count(&self) -> usize {
        self.schedule_group_count
    }

    pub(crate) fn max_level_constraint_count(&self) -> usize {
        self.max_level_constraint_count
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
            + self.body_levels.capacity() * size_of::<usize>()
            + self.level_queue.capacity() * size_of::<usize>()
            + self.constraint_levels.capacity() * size_of::<usize>()
            + self.level_order.capacity() * size_of::<usize>()
            + self.level_offsets.capacity() * size_of::<usize>()
            + self.level_color_counts.capacity() * size_of::<usize>()
            + self.level_group_offsets.capacity() * size_of::<usize>()
            + self.constraint_colors.capacity() * size_of::<usize>()
            + self.color_stamps.capacity() * size_of::<usize>()
            + self.body_stamps.capacity() * size_of::<usize>()
            + self.color_offsets.capacity() * size_of::<usize>()
            + self.colored_constraints.capacity() * size_of::<usize>()
    }

    fn rebuild(&mut self) -> Result<bool, CollisionError> {
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
                if !self.dynamic_bodies[body] {
                    continue;
                }
                let slot = self.scratch_offsets[body];
                self.body_constraints[slot] = constraint;
                self.scratch_offsets[body] += 1;
            }
        }

        self.build_levels(body_count, constraint_count)?;
        self.build_level_coloring(body_count, constraint_count)?;
        debug_assert!(self.has_unique_dynamic_body_ownership());
        Ok(true)
    }

    fn build_levels(
        &mut self,
        body_count: usize,
        constraint_count: usize,
    ) -> Result<(), CollisionError> {
        reserve(&mut self.body_levels, body_count)?;
        self.body_levels.resize(body_count, usize::MAX);
        reserve(&mut self.level_queue, body_count)?;
        self.level_queue.clear();

        for [a, b] in self.endpoints.iter().copied() {
            let dynamic_a = self.dynamic_bodies[a];
            let dynamic_b = self.dynamic_bodies[b];
            if dynamic_a != dynamic_b {
                let dynamic_body = if dynamic_a { a } else { b };
                if self.body_levels[dynamic_body] == usize::MAX {
                    self.body_levels[dynamic_body] = 0;
                    self.level_queue.push(dynamic_body);
                }
            }
        }
        self.visit_level_queue();

        for body in 0..body_count {
            if !self.dynamic_bodies[body]
                || self.body_levels[body] != usize::MAX
                || self.body_offsets[body] == self.body_offsets[body + 1]
            {
                continue;
            }
            self.body_levels[body] = 0;
            self.level_queue.push(body);
            self.visit_level_queue();
        }

        reserve(&mut self.constraint_levels, constraint_count)?;
        self.constraint_levels.resize(constraint_count, 0);
        let mut max_level = 0;
        for (constraint, [a, b]) in self.endpoints.iter().copied().enumerate() {
            let level_a = if self.dynamic_bodies[a] {
                self.body_levels[a]
            } else {
                0
            };
            let level_b = if self.dynamic_bodies[b] {
                self.body_levels[b]
            } else {
                0
            };
            let level = level_a.max(level_b);
            if level == usize::MAX {
                return Err(CollisionError::InvalidContact);
            }
            self.constraint_levels[constraint] = level;
            max_level = max_level.max(level);
        }

        let level_count = if constraint_count == 0 {
            0
        } else {
            max_level
                .checked_add(1)
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?
        };
        reserve(&mut self.level_offsets, level_count.saturating_add(1))?;
        self.level_offsets.clear();
        self.level_offsets.resize(level_count + 1, 0);
        for level in self.constraint_levels.iter().copied() {
            self.level_offsets[level + 1] = self.level_offsets[level + 1]
                .checked_add(1)
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        for level in 0..level_count {
            self.level_offsets[level + 1] = self.level_offsets[level + 1]
                .checked_add(self.level_offsets[level])
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        reserve(&mut self.level_order, constraint_count)?;
        self.level_order.resize(constraint_count, 0);
        reserve(&mut self.scratch_offsets, level_count)?;
        self.scratch_offsets.clear();
        self.scratch_offsets
            .extend_from_slice(&self.level_offsets[..level_count]);
        for (constraint, level) in self.constraint_levels.iter().copied().enumerate() {
            let slot = self.scratch_offsets[level];
            self.level_order[slot] = constraint;
            self.scratch_offsets[level] += 1;
        }
        Ok(())
    }

    fn visit_level_queue(&mut self) {
        let mut head = 0;
        while head < self.level_queue.len() {
            let body = self.level_queue[head];
            head += 1;
            let next_level = self.body_levels[body] + 1;
            for constraint in self.body_constraints
                [self.body_offsets[body]..self.body_offsets[body + 1]]
                .iter()
                .copied()
            {
                let [a, b] = self.endpoints[constraint];
                let neighbor = if a == body { b } else { a };
                if self.dynamic_bodies[neighbor] && self.body_levels[neighbor] == usize::MAX {
                    self.body_levels[neighbor] = next_level;
                    self.level_queue.push(neighbor);
                }
            }
        }
        self.level_queue.clear();
    }

    fn build_level_coloring(
        &mut self,
        body_count: usize,
        constraint_count: usize,
    ) -> Result<(), CollisionError> {
        let level_count = self.level_offsets.len().saturating_sub(1);
        reserve(&mut self.constraint_colors, constraint_count)?;
        self.constraint_colors.resize(constraint_count, usize::MAX);
        reserve(&mut self.color_stamps, constraint_count)?;
        self.color_stamps.resize(constraint_count, 0);
        self.color_stamps.fill(0);
        reserve(&mut self.body_stamps, body_count)?;
        self.body_stamps.resize(body_count, 0);
        reserve(&mut self.level_color_counts, level_count)?;
        self.level_color_counts.resize(level_count, 0);

        let mut color_count = 0;
        self.max_level_constraint_count = 0;
        for level in 0..level_count {
            let start = self.level_offsets[level];
            let end = self.level_offsets[level + 1];
            self.max_level_constraint_count = self.max_level_constraint_count.max(end - start);
            let mut level_colors = 0;
            for slot in start..end {
                let constraint = self.level_order[slot];
                let stamp = constraint + 1;
                let [a, b] = self.endpoints[constraint];
                for body in [a, b] {
                    if !self.dynamic_bodies[body] {
                        continue;
                    }
                    for neighbor in self.body_constraints
                        [self.body_offsets[body]..self.body_offsets[body + 1]]
                        .iter()
                        .copied()
                    {
                        if neighbor < constraint && self.constraint_levels[neighbor] == level {
                            self.color_stamps[self.constraint_colors[neighbor]] = stamp;
                        }
                    }
                }
                let color = (0..constraint_count)
                    .find(|color| self.color_stamps[*color] != stamp)
                    .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
                self.constraint_colors[constraint] = color;
                level_colors = level_colors.max(color + 1);
            }
            self.level_color_counts[level] = level_colors;
            color_count = color_count.max(level_colors);
        }

        reserve(&mut self.level_group_offsets, level_count.saturating_add(1))?;
        self.level_group_offsets.clear();
        self.level_group_offsets.resize(level_count + 1, 0);
        for level in 0..level_count {
            self.level_group_offsets[level + 1] = self.level_group_offsets[level]
                .checked_add(self.level_color_counts[level])
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        let group_count = *self
            .level_group_offsets
            .last()
            .ok_or(CollisionError::InvalidContact)?;

        reserve(&mut self.color_offsets, group_count.saturating_add(1))?;
        self.color_offsets.clear();
        self.color_offsets.resize(group_count + 1, 0);
        for constraint in 0..constraint_count {
            let group = self.level_group_offsets[self.constraint_levels[constraint]]
                + self.constraint_colors[constraint];
            self.color_offsets[group + 1] = self.color_offsets[group + 1]
                .checked_add(1)
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        for group in 0..group_count {
            self.color_offsets[group + 1] = self.color_offsets[group + 1]
                .checked_add(self.color_offsets[group])
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        }
        reserve(&mut self.colored_constraints, constraint_count)?;
        self.colored_constraints.resize(constraint_count, 0);
        self.scratch_offsets.clear();
        self.scratch_offsets
            .extend_from_slice(&self.color_offsets[..group_count]);
        for constraint in 0..constraint_count {
            let group = self.level_group_offsets[self.constraint_levels[constraint]]
                + self.constraint_colors[constraint];
            let slot = self.scratch_offsets[group];
            self.colored_constraints[slot] = constraint;
            self.scratch_offsets[group] += 1;
        }
        self.color_count = color_count;
        self.schedule_group_count = group_count;
        Ok(())
    }

    fn has_unique_dynamic_body_ownership(&mut self) -> bool {
        self.body_stamps.fill(0);
        for group in 0..self.schedule_group_count {
            let start = self.color_offsets[group];
            let end = self.color_offsets[group + 1];
            let stamp = group + 1;
            for slot in start..end {
                let constraint = self.colored_constraints[slot];
                for body in self.endpoints[constraint] {
                    if !self.dynamic_bodies[body] {
                        continue;
                    }
                    if self.body_stamps[body] == stamp {
                        return false;
                    }
                    self.body_stamps[body] = stamp;
                }
            }
        }
        true
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
    fn colors_constraints_within_stable_dependency_levels() {
        let dynamic_bodies = [true, true, true, false];
        // Contacts against one static surface seed two bodies at level zero;
        // the final joint extends the dependency graph by one level.
        let endpoints = [[0, 3], [1, 3], [0, 1], [1, 2]];
        let mut coloring = ConstraintColoring::default();

        assert!(coloring.update(&dynamic_bodies, &endpoints).unwrap());
        assert_eq!(coloring.color_count(), 2);
        assert_eq!(coloring.schedule_group_count(), 3);
        assert_eq!(coloring.constraints_in_schedule_group(0), Some(&[0, 1][..]));
        assert_eq!(coloring.constraints_in_schedule_group(1), Some(&[2][..]));
        assert_eq!(coloring.constraints_in_schedule_group(2), Some(&[3][..]));
        assert!(coloring.has_unique_dynamic_body_ownership());
    }

    #[test]
    fn orders_unanchored_chain_from_deterministic_root() {
        let mut coloring = ConstraintColoring::default();
        let endpoints = [[0, 1], [1, 2], [2, 3], [3, 4]];

        assert!(
            coloring
                .update(&[true, true, true, true, true], &endpoints)
                .unwrap()
        );
        assert_eq!(coloring.color_count(), 1);
        assert_eq!(coloring.schedule_group_count(), 4);
        for group in 0..4 {
            assert_eq!(
                coloring.constraints_in_schedule_group(group),
                Some(&[group][..])
            );
        }
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
