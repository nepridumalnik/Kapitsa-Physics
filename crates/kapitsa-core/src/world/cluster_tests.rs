use super::*;
use crate::{
    cluster::{ClusterPart, RigidCluster, Weld, WeldDesc, WeldStiffness, WeldStrength},
    joint::JointFrame,
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
};
use kapitsa_math::{Mat3, Vec3};

fn part(world: &mut PhysicsWorld, body: BodyId) -> PartId {
    let mass = match MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY) {
        Ok(value) => value,
        Err(error) => panic!("mass: {error}"),
    };
    let dense = match world.parts.insert(ClusterPart {
        body,
        local_transform: Transform::identity(),
        mass,
        colliders: Vec::new(),
    }) {
        Ok(value) => value,
        Err(error) => panic!("part: {error}"),
    };
    PartId {
        world: world.world_id,
        dense,
    }
}

fn weld(world: &mut PhysicsWorld, a: PartId, b: PartId) -> WeldId {
    let desc = WeldDesc {
        frame_a: JointFrame::IDENTITY,
        frame_b: JointFrame::IDENTITY,
        strength: WeldStrength::Unbreakable,
        stiffness: WeldStiffness {
            linear: 1.0,
            angular: 1.0,
        },
    };
    let dense = match world.welds.insert(Weld {
        part_a: a,
        part_b: b,
        desc,
    }) {
        Ok(value) => value,
        Err(error) => panic!("weld: {error}"),
    };
    WeldId {
        world: world.world_id,
        dense,
    }
}

#[test]
fn world_scoped_ids_reject_foreign_and_reused_slots() {
    let mut world = PhysicsWorld::new();
    let body = match world.add_body(RigidBody::new_static(Transform::identity())) {
        Ok(value) => value,
        Err(error) => panic!("body: {error}"),
    };
    let old = part(&mut world, body);
    let old_weld = weld(&mut world, old, old);
    assert!(world.parts.remove(old.dense).is_some());
    assert!(world.welds.remove(old_weld.dense).is_some());
    let new = part(&mut world, body);
    let new_weld = weld(&mut world, new, new);
    assert_eq!(old.index(), new.index());
    assert_ne!(old.generation(), new.generation());
    assert_eq!(old_weld.index(), new_weld.index());
    assert_ne!(old_weld.generation(), new_weld.generation());
    assert!(world.cluster_part(old).is_none());
    assert!(world.weld(old_weld).is_none());
    let mut foreign = PhysicsWorld::new();
    assert!(
        foreign
            .add_body(RigidBody::new_static(Transform::identity()))
            .is_ok()
    );
    assert!(foreign.cluster_part(new).is_none());
    assert!(foreign.weld(new_weld).is_none());
}

#[test]
fn validation_rejects_disconnected_and_invalid_graphs() {
    let mut world = PhysicsWorld::new();
    let body = match world.add_body(RigidBody::new_static(Transform::identity())) {
        Ok(value) => value,
        Err(error) => panic!("body: {error}"),
    };
    let a = part(&mut world, body);
    let b = part(&mut world, body);
    let entry = match world.bodies.get_mut(body.dense) {
        Some(value) => value,
        None => panic!("missing body"),
    };
    entry.cluster = Some(RigidCluster {
        root: a,
        parts: vec![a, b],
        welds: Vec::new(),
    });
    assert_eq!(
        world.validate_clusters(),
        Err(ClusterError::InvalidTopology)
    );
    let edge = weld(&mut world, a, b);
    if let Some(cluster) = world
        .bodies
        .get_mut(body.dense)
        .and_then(|entry| entry.cluster.as_mut())
    {
        cluster.welds.push(edge);
    }
    assert_eq!(world.validate_clusters(), Ok(()));
    if let Some(cluster) = world
        .bodies
        .get_mut(body.dense)
        .and_then(|entry| entry.cluster.as_mut())
    {
        cluster.parts.clear();
    }
    assert_eq!(
        world.validate_clusters(),
        Err(ClusterError::InvalidTopology)
    );
}
