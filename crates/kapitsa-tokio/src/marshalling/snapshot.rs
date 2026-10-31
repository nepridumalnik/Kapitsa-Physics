use std::{collections::HashMap, mem::size_of, sync::Arc, time::Duration};

use kapitsa_core::{
    rigid_body::BodyState,
    world::{BodyId, PhysicsWorld},
};

#[derive(Clone, Copy)]
struct SnapshotEntry {
    id: BodyId,
    state: BodyState,
}

/// Immutable body states published after a successful simulation step.
///
/// Body lookup uses the complete world-scoped generational ID, so IDs from
/// other worlds and stale IDs cannot resolve to a live body.
pub struct PhysicsSnapshot {
    step: u64,
    simulation_time: Duration,
    entries: Vec<SnapshotEntry>,
    indices: HashMap<BodyId, usize>,
}

impl PhysicsSnapshot {
    fn empty() -> Self {
        Self {
            step: 0,
            simulation_time: Duration::ZERO,
            entries: Vec::new(),
            indices: HashMap::new(),
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
        (self.entries.capacity(), self.indices.capacity())
    }

    #[allow(dead_code)]
    pub(crate) fn estimated_storage_bytes(&self) -> usize {
        self.entries.capacity() * size_of::<SnapshotEntry>()
            + self.indices.capacity() * size_of::<(BodyId, usize)>()
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
        self.entries.clear();
        self.indices.clear();
        self.entries.reserve(body_count);
        self.indices.reserve(body_count);
        self.step = step;
        self.simulation_time = simulation_time;

        for (id, body) in world.bodies() {
            let index = self.entries.len();
            self.entries.push(SnapshotEntry {
                id,
                state: *body.state(),
            });
            self.indices.insert(id, index);
        }
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
        self.indices
            .get(&id)
            .and_then(|index| self.entries.get(*index))
            .map(|entry| &entry.state)
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

#[cfg(test)]
mod tests {
    use std::{collections::HashSet, sync::Arc, time::Duration};

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

    #[test]
    fn empty_snapshot_has_no_bodies() {
        let snapshot = PhysicsSnapshot::from_world(&PhysicsWorld::new(), 0, Duration::ZERO);
        assert!(snapshot.is_empty());
        assert_eq!(snapshot.len(), 0);
        assert_eq!(snapshot.entries.len(), 0);
        assert_eq!(snapshot.indices.len(), 0);
        assert_eq!(snapshot.bodies().count(), 0);
    }

    #[test]
    fn snapshots_store_only_live_bodies_after_removals() {
        let mut world = PhysicsWorld::new();
        let ids = add_bodies(&mut world, 100);
        let mut snapshot = PhysicsSnapshot::from_world(&world, 0, Duration::ZERO);
        assert_eq!(snapshot.entries.len(), 100);
        assert_eq!(snapshot.indices.len(), 100);

        for id in &ids[..90] {
            assert!(world.remove_body(*id).is_some());
        }
        snapshot.rebuild(&world, 1, Duration::from_millis(10));

        assert_eq!(snapshot.len(), 10);
        assert_eq!(snapshot.entries.len(), 10);
        assert_eq!(snapshot.indices.len(), 10);
        assert!(ids[..90].iter().all(|id| snapshot.body(*id).is_none()));
        assert!(ids[90..].iter().all(|id| snapshot.body(*id).is_some()));
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

        assert_eq!(snapshot.entries.len(), 100);
        assert_eq!(snapshot.indices.len(), 100);
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
    fn rebuild_discards_stale_generations_and_old_map_entries() {
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

        assert_eq!(snapshot.indices.len(), 1);
        assert_eq!(snapshot.entries.len(), 1);
        assert!(!snapshot.indices.contains_key(&stale));
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
