use std::time::Duration;

use kapitsa_core::{
    rigid_body::BodyState,
    world::{BodyId, PhysicsWorld},
};

#[derive(Clone, Copy)]
struct SnapshotEntry {
    id: BodyId,
    state: BodyState,
}

/// Immutable body state published after a successful simulation step.
pub struct PhysicsSnapshot {
    step: u64,
    simulation_time: Duration,
    bodies: Vec<Option<SnapshotEntry>>,
    len: usize,
}

impl PhysicsSnapshot {
    pub(crate) fn from_world(world: &PhysicsWorld, step: u64, simulation_time: Duration) -> Self {
        let mut bodies = Vec::new();
        for (id, body) in world.bodies() {
            let index = usize::try_from(id.index()).unwrap_or(usize::MAX);
            if index >= bodies.len() {
                bodies.resize_with(index.saturating_add(1), || None);
            }
            bodies[index] = Some(SnapshotEntry {
                id,
                state: *body.state(),
            });
        }
        Self {
            step,
            simulation_time,
            bodies,
            len: world.body_count(),
        }
    }

    pub fn step(&self) -> u64 {
        self.step
    }

    pub fn simulation_time(&self) -> Duration {
        self.simulation_time
    }

    pub fn body(&self, id: BodyId) -> Option<&BodyState> {
        let index = usize::try_from(id.index()).ok()?;
        let entry = self.bodies.get(index)?.as_ref()?;
        (entry.id == id).then_some(&entry.state)
    }

    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &BodyState)> + '_ {
        self.bodies
            .iter()
            .filter_map(|entry| entry.as_ref().map(|entry| (entry.id, &entry.state)))
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use kapitsa_core::{rigid_body::RigidBody, transform::Transform, world::PhysicsWorld};
    use kapitsa_math::Vec3;

    use super::PhysicsSnapshot;

    #[test]
    fn snapshot_uses_slot_indices_and_tracks_exact_body_count() {
        let mut world = PhysicsWorld::new();
        let first = world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("first body");
        let second = world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .expect("second body");
        world.remove_body(first).expect("remove first");
        let mut body = RigidBody::new_kinematic(Transform::identity());
        body.set_linear_velocity(Vec3::X).expect("velocity");
        let replacement = world.add_body(body).expect("reuse slot");

        let snapshot = PhysicsSnapshot::from_world(&world, 7, Duration::from_millis(120));

        assert_eq!(snapshot.step(), 7);
        assert_eq!(snapshot.simulation_time(), Duration::from_millis(120));
        assert_eq!(snapshot.len(), 2);
        assert!(!snapshot.is_empty());
        assert_eq!(snapshot.body(first), None);
        assert_eq!(
            snapshot.body(replacement).unwrap().linear_velocity(),
            Vec3::X
        );
        assert!(snapshot.body(second).is_some());
        assert_eq!(snapshot.bodies().count(), 2);
    }

    #[test]
    fn empty_snapshot_has_no_bodies() {
        let snapshot = PhysicsSnapshot::from_world(&PhysicsWorld::new(), 0, Duration::ZERO);
        assert!(snapshot.is_empty());
        assert_eq!(snapshot.len(), 0);
        assert_eq!(snapshot.bodies().count(), 0);
    }
}
