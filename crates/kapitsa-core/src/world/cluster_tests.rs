use super::*;
use crate::{
    cluster::{ClusterPart, RigidCluster, Weld, WeldDesc, WeldStiffness, WeldStrength},
    collider::Collider,
    joint::JointFrame,
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
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

#[test]
fn managed_body_mutation_is_validated() {
    let mut world = PhysicsWorld::new();
    let mass = match MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY) {
        Ok(value) => value,
        Err(error) => panic!("mass: {error}"),
    };
    let dynamic = match RigidBody::new_dynamic(Transform::identity(), mass) {
        Ok(value) => value,
        Err(error) => panic!("body: {error}"),
    };
    let body = match world.add_body(dynamic) {
        Ok(value) => value,
        Err(error) => panic!("body: {error}"),
    };
    let root = part(&mut world, body);
    if let Some(entry) = world.bodies.get_mut(body.dense) {
        entry.cluster = Some(RigidCluster {
            root,
            parts: vec![root],
            welds: Vec::new(),
        });
    }
    assert!(world.body_mut(body).is_none());
    assert!(!world.bodies_mut().any(|(id, _)| id == body));
    let replacement_mass = match MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY) {
        Ok(value) => value,
        Err(error) => panic!("mass: {error}"),
    };
    assert_eq!(
        world.edit_body(body, |body| body.set_mass(replacement_mass)),
        Err(ClusterError::ManagedMass)
    );
    assert_eq!(
        world.replace_body_state(body, RigidBody::new_static(Transform::identity())),
        Err(ClusterError::ManagedMass)
    );
    assert_eq!(
        world.edit_body(body, |body| body.set_linear_velocity(Vec3::X)),
        Ok(())
    );
    assert_eq!(
        world.body(body).map(|body| body.state().linear_velocity()),
        Some(Vec3::X)
    );
    assert_eq!(
        world.edit_body(body, |body| body.apply_force(Vec3::Y)),
        Ok(())
    );
    assert_eq!(
        world.body(body).map(|body| body.forces().force()),
        Some(Vec3::Y)
    );
    assert_eq!(world.validate_clusters(), Ok(()));
}

#[test]
fn rebind_preserves_collider_identity_and_world_pose() {
    let mut world = PhysicsWorld::new();
    let origin = Transform::identity();
    let translated = match Transform::new(Vec3::X, kapitsa_math::Quat::IDENTITY) {
        Ok(value) => value,
        Err(error) => panic!("transform: {error}"),
    };
    let a = match world.add_body(RigidBody::new_static(origin)) {
        Ok(value) => value,
        Err(error) => panic!("body: {error}"),
    };
    let b = match world.add_body(RigidBody::new_static(translated)) {
        Ok(value) => value,
        Err(error) => panic!("body: {error}"),
    };
    let sphere = match Sphere::new(1.0) {
        Ok(value) => value,
        Err(error) => panic!("sphere: {error}"),
    };
    let collider = match world.add_collider(a, Collider::new(Shape::Sphere(sphere), translated)) {
        Ok(value) => value,
        Err(error) => panic!("collider: {error}"),
    };
    assert_eq!(world.rebind_collider(collider, b, origin), Ok(()));
    assert_eq!(world.collider_body(collider), Some(b));
    assert_eq!(
        world
            .collider(collider)
            .map(|shape| shape.local_transform()),
        Some(origin)
    );
    assert_eq!(
        world
            .bodies
            .get(a.dense)
            .and_then(|entry| entry.first_collider),
        None
    );
    assert_eq!(
        world
            .bodies
            .get(b.dense)
            .and_then(|entry| entry.first_collider),
        Some(collider)
    );
    assert!(world.remove_collider(collider).is_some());
}
