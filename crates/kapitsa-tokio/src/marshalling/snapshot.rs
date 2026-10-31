use std::{
    collections::HashMap,
    mem::{self, size_of},
    sync::Arc,
    time::Duration,
};

use kapitsa_core::{
    rigid_body::BodyState,
    world::{BodyId, PhysicsWorld},
};

const DENSE_ENTER_RATIO: usize = 3;
const DENSE_EXIT_RATIO: usize = 6;
const CAPACITY_SHRINK_RATIO: usize = 4;
const MIN_RETAINED_CAPACITY: usize = 256;

#[derive(Clone, Copy)]
struct SnapshotEntry {
    id: BodyId,
    state: BodyState,
}

enum SnapshotIndex {
    Dense(Vec<usize>),
    Sparse(HashMap<BodyId, usize>),
}

/// Immutable body states published after a successful simulation step.
///
/// Body lookup uses the complete world-scoped generational ID, so IDs from
/// other worlds and stale IDs cannot resolve to a live body.
pub struct PhysicsSnapshot {
    step: u64,
    simulation_time: Duration,
    entries: Vec<SnapshotEntry>,
    index: SnapshotIndex,
}

impl PhysicsSnapshot {
    fn empty() -> Self {
        Self {
            step: 0,
            simulation_time: Duration::ZERO,
            entries: Vec::new(),
            index: SnapshotIndex::Sparse(HashMap::new()),
        }
    }

    pub(crate) fn from_world(world: &PhysicsWorld, step: u64, simulation_time: Duration) -> Self {
        let mut snapshot = Self::empty();
        snapshot.rebuild(world, step, simulation_time);
        snapshot
    }

    pub(crate) fn take_reusable(reusable: &mut Option<Self>) -> (Self, bool) {
        match reusable.take() {
            Some(snapshot) => (snapshot, true),
            None => (Self::empty(), false),
        }
    }

    pub(crate) fn recycle(previous: Arc<Self>, reusable: &mut Option<Self>) -> bool {
        match Arc::try_unwrap(previous) {
            Ok(snapshot) => {
                *reusable = Some(snapshot);
                true
            }
            Err(_) => false,
        }
    }

    #[allow(dead_code)]
    pub(crate) fn buffer_capacities(&self) -> (usize, usize) {
        let index_capacity = match &self.index {
            SnapshotIndex::Dense(index) => index.capacity(),
            SnapshotIndex::Sparse(index) => index.capacity(),
        };
        (self.entries.capacity(), index_capacity)
    }

    #[allow(dead_code)]
    pub(crate) fn estimated_storage_bytes(&self) -> usize {
        let index_bytes = match &self.index {
            SnapshotIndex::Dense(index) => index.capacity() * size_of::<usize>(),
            SnapshotIndex::Sparse(index) => index.capacity() * size_of::<(BodyId, usize)>(),
        };
        self.entries.capacity() * size_of::<SnapshotEntry>() + index_bytes
    }

    #[allow(dead_code)]
    pub(crate) fn legacy_slot_storage_bytes(max_index: u32) -> usize {
        match usize::try_from(max_index) {
            Ok(index) => index
                .saturating_add(1)
                .saturating_mul(size_of::<Option<(u32, u32, BodyState)>>()),
            Err(_) => usize::MAX,
        }
    }

    pub(crate) fn rebuild(&mut self, world: &PhysicsWorld, step: u64, simulation_time: Duration) {
        let body_count = world.body_count();
        prepare_entries(&mut self.entries, body_count);
        for (id, body) in world.bodies() {
            self.entries.push(SnapshotEntry {
                id,
                state: *body.state(),
            });
        }

        let live = self.entries.len();
        let span = slot_span(&self.entries);
        let current_dense = matches!(&self.index, SnapshotIndex::Dense(_));
        let use_dense = should_use_dense(current_dense, live, span);

        if use_dense != current_dense {
            self.index = if use_dense {
                SnapshotIndex::Dense(Vec::new())
            } else {
                SnapshotIndex::Sparse(HashMap::new())
            };
        }
        match &mut self.index {
            SnapshotIndex::Dense(index) => rebuild_dense_index(index, &self.entries, span),
            SnapshotIndex::Sparse(index) => rebuild_sparse_index(index, &self.entries),
        }
        self.step = step;
        self.simulation_time = simulation_time;
    }

    /// Returns the total number of successful steps represented by this snapshot.
    pub fn step(&self) -> u64 {
        self.step
    }

    /// Returns accumulated simulated time represented by this snapshot.
    pub fn simulation_time(&self) -> Duration {
        self.simulation_time
    }

    /// Looks up a body by its complete world-scoped generational ID in expected O(1) time.
    pub fn body(&self, id: BodyId) -> Option<&BodyState> {
        let position = match &self.index {
            SnapshotIndex::Dense(index) => {
                let slot = usize::try_from(id.index()).ok()?;
                let position = *index.get(slot)?;
                (position != usize::MAX).then_some(position)?
            }
            SnapshotIndex::Sparse(index) => *index.get(&id)?,
        };
        let entry = self.entries.get(position)?;
        (entry.id == id).then_some(&entry.state)
    }

    /// Iterates over current bodies as `(ID, state)` pairs.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &BodyState)> + '_ {
        self.entries.iter().map(|entry| (entry.id, &entry.state))
    }

    /// Returns the number of live bodies in this snapshot.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns whether this snapshot contains no live bodies.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

fn slot_span(entries: &[SnapshotEntry]) -> Option<usize> {
    let max_index = entries.iter().map(|entry| entry.id.index()).max();
    match max_index {
        Some(index) => usize::try_from(index).ok()?.checked_add(1),
        None => Some(0),
    }
}

fn should_use_dense(current_dense: bool, live: usize, span: Option<usize>) -> bool {
    if live == 0 {
        return true;
    }
    let Some(span) = span else {
        return false;
    };
    let ratio = if current_dense {
        DENSE_EXIT_RATIO
    } else {
        DENSE_ENTER_RATIO
    };
    span <= live.saturating_mul(ratio)
}

fn rebuild_dense_index(index: &mut Vec<usize>, entries: &[SnapshotEntry], span: Option<usize>) {
    let Some(span) = span else {
        index.clear();
        return;
    };
    if index.capacity() > capacity_limit(span) {
        drop(mem::take(index));
    }
    index.clear();
    index.reserve(span);
    index.resize(span, usize::MAX);
    for (position, entry) in entries.iter().enumerate() {
        if let Ok(slot) = usize::try_from(entry.id.index())
            && let Some(indexed_position) = index.get_mut(slot)
        {
            *indexed_position = position;
        }
    }
}

fn rebuild_sparse_index(index: &mut HashMap<BodyId, usize>, entries: &[SnapshotEntry]) {
    if index.capacity() > capacity_limit(entries.len()) {
        drop(mem::take(index));
    }
    index.clear();
    index.reserve(entries.len());
    for (position, entry) in entries.iter().enumerate() {
        index.insert(entry.id, position);
    }
}

fn prepare_entries(entries: &mut Vec<SnapshotEntry>, live: usize) {
    if entries.capacity() > capacity_limit(live) {
        drop(mem::take(entries));
    }
    entries.clear();
    entries.reserve(live);
}

fn capacity_limit(required: usize) -> usize {
    required
        .saturating_mul(CAPACITY_SHRINK_RATIO)
        .max(MIN_RETAINED_CAPACITY)
}

#[cfg(test)]
mod tests {
    use std::{
        collections::{HashMap, HashSet},
        sync::Arc,
        time::Duration,
    };

    use kapitsa_core::{rigid_body::RigidBody, transform::Transform, world::PhysicsWorld};
    use kapitsa_math::Vec3;

    use super::PhysicsSnapshot;

    fn add_bodies(world: &mut PhysicsWorld, count: usize) -> Vec<kapitsa_core::world::BodyId> {
        (0..count)
            .map(|_| {
                world
                    .add_body(RigidBody::new_kinematic(Transform::identity()))
                    .expect("body insertion")
            })
            .collect()
    }

    fn world_with_live_tail(slots: usize, live: usize) -> PhysicsWorld {
        let mut world = PhysicsWorld::new();
        let ids = add_bodies(&mut world, slots);
        for id in &ids[..slots - live] {
            assert!(world.remove_body(*id).is_some());
        }
        world
    }

    fn index_len(snapshot: &PhysicsSnapshot) -> usize {
        match &snapshot.index {
            super::SnapshotIndex::Dense(index) => index.len(),
            super::SnapshotIndex::Sparse(index) => index.len(),
        }
    }

    fn is_dense(snapshot: &PhysicsSnapshot) -> bool {
        matches!(&snapshot.index, super::SnapshotIndex::Dense(_))
    }

    #[test]
    fn empty_snapshot_has_no_bodies() {
        let snapshot = PhysicsSnapshot::from_world(&PhysicsWorld::new(), 0, Duration::ZERO);
        assert!(snapshot.is_empty());
        assert_eq!(snapshot.len(), 0);
        assert_eq!(snapshot.entries.len(), 0);
        assert!(is_dense(&snapshot));
        assert_eq!(index_len(&snapshot), 0);
        assert_eq!(snapshot.bodies().count(), 0);
    }

    #[test]
    fn snapshots_store_only_live_bodies_after_removals() {
        let mut world = PhysicsWorld::new();
        let ids = add_bodies(&mut world, 100);
        let mut snapshot = PhysicsSnapshot::from_world(&world, 0, Duration::ZERO);
        assert!(is_dense(&snapshot));
        assert_eq!(snapshot.entries.len(), 100);
        assert_eq!(index_len(&snapshot), 100);

        for id in &ids[..90] {
            assert!(world.remove_body(*id).is_some());
        }
        snapshot.rebuild(&world, 1, Duration::from_millis(10));

        assert_eq!(snapshot.len(), 10);
        assert_eq!(snapshot.entries.len(), 10);
        assert!(!is_dense(&snapshot));
        assert_eq!(index_len(&snapshot), 10);
        assert!(ids[..90].iter().all(|id| snapshot.body(*id).is_none()));
        assert!(ids[90..].iter().all(|id| snapshot.body(*id).is_some()));
    }

    #[test]
    fn dense_index_marks_removed_slots_with_the_empty_sentinel() {
        let mut world = PhysicsWorld::new();
        let ids = add_bodies(&mut world, 3);
        assert!(world.remove_body(ids[1]).is_some());

        let snapshot = PhysicsSnapshot::from_world(&world, 1, Duration::ZERO);
        let super::SnapshotIndex::Dense(index) = &snapshot.index else {
            panic!("three slots with two live bodies should use Dense");
        };

        assert_eq!(index.get(1), Some(&usize::MAX));
        assert!(snapshot.body(ids[1]).is_none());
        assert!(snapshot.body(ids[0]).is_some());
        assert!(snapshot.body(ids[2]).is_some());
    }

    #[test]
    fn sparse_slot_history_does_not_expand_snapshot_to_maximum_slot() {
        let mut world = PhysicsWorld::new();
        let ids = add_bodies(&mut world, 20_000);
        for id in &ids[..19_900] {
            assert!(world.remove_body(*id).is_some());
        }
        let live_ids = &ids[19_900..];
        assert_eq!(live_ids.len(), 100);
        assert!(live_ids.iter().all(|id| id.index() >= 19_900));

        let snapshot = PhysicsSnapshot::from_world(&world, 4, Duration::ZERO);

        assert!(!is_dense(&snapshot));
        assert_eq!(snapshot.entries.len(), 100);
        assert_eq!(index_len(&snapshot), 100);
        assert_eq!(snapshot.entries.len(), world.body_count());
        assert!(
            snapshot
                .entries
                .iter()
                .all(|entry| entry.id.index() >= 19_900)
        );
        assert!(live_ids.iter().all(|id| snapshot.body(*id).is_some()));
    }

    #[test]
    fn dense_sparse_transitions_preserve_all_live_body_states() {
        let mut sparse_world = PhysicsWorld::new();
        let sparse_ids = add_bodies(&mut sparse_world, 1_000);
        let mut snapshot = PhysicsSnapshot::from_world(&sparse_world, 0, Duration::ZERO);
        assert!(is_dense(&snapshot));
        for id in &sparse_ids[..900] {
            assert!(sparse_world.remove_body(*id).is_some());
        }

        snapshot.rebuild(&sparse_world, 1, Duration::from_millis(10));
        assert!(!is_dense(&snapshot));
        assert_eq!(snapshot.len(), 100);
        assert!(
            sparse_ids[..900]
                .iter()
                .all(|id| snapshot.body(*id).is_none())
        );
        assert!(
            sparse_ids[900..]
                .iter()
                .all(|id| snapshot.body(*id).is_some())
        );

        let mut dense_world = PhysicsWorld::new();
        let dense_ids = add_bodies(&mut dense_world, 100);
        snapshot.rebuild(&dense_world, 2, Duration::from_millis(20));
        assert!(is_dense(&snapshot));
        assert_eq!(snapshot.len(), dense_world.body_count());
        assert!(dense_ids.iter().all(|id| snapshot.body(*id).is_some()));
        assert_eq!(snapshot.bodies().count(), 100);
    }

    #[test]
    fn reused_snapshot_releases_capacity_after_shrink_and_reallocates_on_growth() {
        let mut world = PhysicsWorld::new();
        let initial_ids = add_bodies(&mut world, 20_000);
        let initial = PhysicsSnapshot::from_world(&world, 0, Duration::ZERO);
        let (initial_entries, initial_index) = initial.buffer_capacities();
        assert!(initial_entries >= 20_000);
        assert!(initial_index >= 20_000);

        let mut reusable = None;
        assert!(PhysicsSnapshot::recycle(Arc::new(initial), &mut reusable));
        let (mut snapshot, reused) = PhysicsSnapshot::take_reusable(&mut reusable);
        assert!(reused);
        for id in &initial_ids[..19_900] {
            assert!(world.remove_body(*id).is_some());
        }
        snapshot.rebuild(&world, 1, Duration::from_millis(10));

        let (shrunk_entries, shrunk_index) = snapshot.buffer_capacities();
        assert!(!is_dense(&snapshot));
        assert_eq!(snapshot.len(), 100);
        assert!(shrunk_entries < initial_entries);
        assert!(shrunk_entries <= super::capacity_limit(100));
        assert!(shrunk_index >= snapshot.len());
        assert!(shrunk_index <= super::capacity_limit(100));
        assert!(
            initial_ids[..19_900]
                .iter()
                .all(|id| snapshot.body(*id).is_none())
        );
        assert!(
            initial_ids[19_900..]
                .iter()
                .all(|id| snapshot.body(*id).is_some())
        );

        for _ in 0..19_900 {
            world
                .add_body(RigidBody::new_kinematic(Transform::identity()))
                .expect("body insertion after growth");
        }
        snapshot.rebuild(&world, 2, Duration::from_millis(20));
        let (grown_entries, grown_index) = snapshot.buffer_capacities();
        assert!(is_dense(&snapshot));
        assert_eq!(snapshot.len(), 20_000);
        assert!(grown_entries >= 20_000);
        assert!(grown_index >= 20_000);
        assert!(snapshot.bodies().all(|(id, _)| snapshot.body(id).is_some()));
    }

    #[test]
    fn shrinking_a_world_does_not_mutate_a_snapshot_held_by_a_reader() {
        let mut world = PhysicsWorld::new();
        let ids = add_bodies(&mut world, 20_000);
        let retained = Arc::new(PhysicsSnapshot::from_world(&world, 7, Duration::ZERO));
        let reader = Arc::clone(&retained);
        let retained_capacities = reader.buffer_capacities();
        let mut reusable = None;

        assert!(!PhysicsSnapshot::recycle(retained, &mut reusable));
        for id in &ids[..19_900] {
            assert!(world.remove_body(*id).is_some());
        }
        let shrunk = PhysicsSnapshot::from_world(&world, 8, Duration::from_millis(10));

        assert!(!is_dense(&shrunk));
        assert_eq!(shrunk.len(), 100);
        assert_eq!(reader.step(), 7);
        assert_eq!(reader.len(), 20_000);
        assert_eq!(reader.buffer_capacities(), retained_capacities);
        assert!(ids.iter().all(|id| reader.body(*id).is_some()));
        assert!(ids[..19_900].iter().all(|id| shrunk.body(*id).is_none()));
        assert!(ids[19_900..].iter().all(|id| shrunk.body(*id).is_some()));
    }

    #[test]
    fn stable_and_oscillating_scenes_do_not_shrink_or_toggle_repeatedly() {
        let mut full = PhysicsWorld::new();
        add_bodies(&mut full, 400);
        let medium = world_with_live_tail(400, 100);
        let sparse = world_with_live_tail(400, 60);
        let mut snapshot = PhysicsSnapshot::from_world(&full, 0, Duration::ZERO);
        assert!(is_dense(&snapshot));

        snapshot.rebuild(&medium, 1, Duration::ZERO);
        assert!(is_dense(&snapshot));
        snapshot.rebuild(&sparse, 2, Duration::ZERO);
        assert!(!is_dense(&snapshot));
        snapshot.rebuild(&medium, 3, Duration::ZERO);
        assert!(!is_dense(&snapshot));
        let warmed_capacities = snapshot.buffer_capacities();

        for step in 4..20 {
            let world = if step % 2 == 0 { &sparse } else { &medium };
            snapshot.rebuild(world, step, Duration::ZERO);
            assert!(!is_dense(&snapshot));
            assert_eq!(snapshot.buffer_capacities(), warmed_capacities);
            assert_eq!(snapshot.len(), world.body_count());
        }

        let dense_capacities =
            PhysicsSnapshot::from_world(&full, 0, Duration::ZERO).buffer_capacities();
        let mut stable = PhysicsSnapshot::from_world(&full, 0, Duration::ZERO);
        for step in 1..6 {
            stable.rebuild(&full, step, Duration::ZERO);
            assert_eq!(stable.buffer_capacities(), dense_capacities);
        }
    }

    #[test]
    fn index_hysteresis_keeps_the_current_kind_between_thresholds() {
        assert!(super::should_use_dense(false, 100, Some(300)));
        assert!(!super::should_use_dense(false, 100, Some(301)));
        assert!(super::should_use_dense(true, 100, Some(600)));
        assert!(!super::should_use_dense(true, 100, Some(601)));
        assert!(!super::should_use_dense(false, 100, Some(400)));
        assert!(super::should_use_dense(true, 100, Some(400)));
        assert!(!super::should_use_dense(true, usize::MAX, None));
    }

    #[test]
    fn empty_world_uses_dense_and_empty_sparse_lookup_is_safe() {
        let empty = PhysicsWorld::new();
        let dense = PhysicsSnapshot::from_world(&empty, 0, Duration::ZERO);
        assert!(is_dense(&dense));
        assert!(dense.is_empty());

        let mut foreign_world = PhysicsWorld::new();
        let foreign = foreign_world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("foreign body");
        let mut sparse = PhysicsSnapshot::empty();
        assert!(sparse.body(foreign).is_none());
        sparse.index = super::SnapshotIndex::Sparse(HashMap::new());
        assert!(sparse.body(foreign).is_none());
        assert!(sparse.bodies().next().is_none());
    }

    #[test]
    fn lookup_uses_full_ids_and_bodies_are_iterated_once() {
        let mut world = PhysicsWorld::new();
        let first = world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("first body");
        let mut moving = RigidBody::new_kinematic(Transform::identity());
        moving.set_linear_velocity(Vec3::X).expect("velocity");
        let second = world.add_body(moving).expect("second body");
        let mut other_world = PhysicsWorld::new();
        other_world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("foreign first body");
        let foreign = other_world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("foreign body");
        assert_eq!(second.index(), foreign.index());

        let snapshot = PhysicsSnapshot::from_world(&world, 7, Duration::from_millis(120));
        let body_ids = snapshot.bodies().map(|(id, _)| id).collect::<HashSet<_>>();

        assert_eq!(snapshot.step(), 7);
        assert_eq!(snapshot.simulation_time(), Duration::from_millis(120));
        assert_eq!(snapshot.len(), world.body_count());
        assert_eq!(body_ids, HashSet::from([first, second]));
        assert_eq!(snapshot.bodies().count(), 2);
        assert!(snapshot.body(first).is_some());
        assert_eq!(
            snapshot.body(second).map(|state| state.linear_velocity()),
            Some(Vec3::X)
        );
        assert!(snapshot.body(foreign).is_none());
    }

    #[test]
    fn rebuild_discards_stale_generations_and_old_index_entries() {
        let mut world = PhysicsWorld::new();
        let stale = world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("first body");
        let mut snapshot = PhysicsSnapshot::from_world(&world, 0, Duration::ZERO);
        assert!(world.remove_body(stale).is_some());
        let replacement = world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("replacement body");
        assert_eq!(stale.index(), replacement.index());
        assert_ne!(stale.generation(), replacement.generation());

        snapshot.rebuild(&world, 1, Duration::from_millis(10));

        assert!(is_dense(&snapshot));
        assert_eq!(index_len(&snapshot), 1);
        assert_eq!(snapshot.entries.len(), 1);
        assert!(snapshot.body(stale).is_none());
        assert!(snapshot.body(replacement).is_some());
    }

    #[test]
    fn arc_recycling_skips_readers_and_reuses_released_capacity() {
        let mut world = PhysicsWorld::new();
        let ids = add_bodies(&mut world, 100);
        let original = PhysicsSnapshot::from_world(&world, 3, Duration::from_millis(30));
        let original_velocity = original
            .body(ids[0])
            .expect("body in snapshot")
            .linear_velocity();
        let retained = Arc::new(original);
        let weak = Arc::downgrade(&retained);
        let reader = Arc::clone(&retained);
        let mut reusable = None;

        assert!(!PhysicsSnapshot::recycle(retained, &mut reusable));
        assert!(reusable.is_none());
        assert_eq!(reader.step(), 3);
        assert_eq!(
            reader.body(ids[0]).map(|state| state.linear_velocity()),
            Some(original_velocity)
        );

        let fresh = PhysicsSnapshot::from_world(&world, 4, Duration::from_millis(40));
        let expected_capacity = fresh.buffer_capacities();
        assert!(PhysicsSnapshot::recycle(Arc::new(fresh), &mut reusable));
        let (mut recovered, reused) = PhysicsSnapshot::take_reusable(&mut reusable);
        assert!(reused);
        assert_eq!(recovered.buffer_capacities(), expected_capacity);

        recovered.rebuild(&world, 5, Duration::from_millis(50));
        assert_eq!(recovered.buffer_capacities(), expected_capacity);
        assert_eq!(
            recovered.estimated_storage_bytes(),
            reader.estimated_storage_bytes()
        );
        assert_eq!(reader.step(), 3);
        assert_eq!(
            reader.body(ids[0]).map(|state| state.linear_velocity()),
            Some(original_velocity)
        );
        drop(reader);
        assert!(weak.upgrade().is_none());
    }
}
