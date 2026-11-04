use super::*;
use crate::{
    cluster::{
        ClusterPart, ClusterPartDesc, RigidCluster, Weld, WeldDesc, WeldStiffness, WeldStrength,
    },
    collider::Collider,
    joint::JointFrame,
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::{ForceSource, ForceTarget},
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
    if let Some(weld) = world.welds.get_mut(edge.dense) {
        weld.desc.stiffness.linear = f64::NAN;
    }
    assert_eq!(world.validate_clusters(), Err(ClusterError::InvalidWeld));
    if let Some(weld) = world.welds.get_mut(edge.dense) {
        weld.desc.stiffness.linear = 1.0;
    }
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
fn changing_mass_preserves_rotating_velocity_field_and_force() {
    let mut world = PhysicsWorld::new();
    let mass = match MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY) {
        Ok(value) => value,
        Err(error) => panic!("mass: {error}"),
    };
    let mut dynamic = match RigidBody::new_dynamic(Transform::identity(), mass) {
        Ok(value) => value,
        Err(error) => panic!("body: {error}"),
    };
    assert!(dynamic.set_angular_velocity(Vec3::Z).is_ok());
    assert!(dynamic.apply_force(Vec3::Y).is_ok());
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
    assert_eq!(
        world.set_part_mass(
            root,
            mass,
            crate::cluster::MassChangeMode::KeepVelocityField
        ),
        Ok(())
    );
    assert_eq!(
        world.body(body).map(|value| value.forces().force()),
        Some(Vec3::Y)
    );
    let shifted = match MassProperties::new(0.5, Vec3::X, Mat3::IDENTITY) {
        Ok(value) => value,
        Err(error) => panic!("mass: {error}"),
    };
    assert_eq!(
        world.set_part_mass(
            root,
            shifted,
            crate::cluster::MassChangeMode::KeepVelocityField
        ),
        Ok(())
    );
    let body = match world.body(body) {
        Some(value) => value,
        None => panic!("missing body"),
    };
    assert_eq!(body.state().linear_velocity(), Vec3::Y);
    assert_eq!(body.state().angular_velocity(), Vec3::Z);
    assert_eq!(body.forces().force(), Vec3::Y);
    assert_eq!(body.forces().torque(), -Vec3::Z);
    assert_eq!(body.state().transform(), Transform::identity());
}

#[test]
fn explicit_mass_transfer_updates_momentum_and_rejects_invalid_input() {
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
    let transfer = crate::cluster::MassTransfer {
        world_origin: Vec3::ZERO,
        linear_impulse: Vec3::X,
        angular_impulse: Vec3::Z,
    };
    assert_eq!(
        world.set_part_mass_with_transfer(root, mass, transfer),
        Ok(())
    );
    let updated = match world.body(body) {
        Some(value) => *value,
        None => panic!("missing body"),
    };
    assert_eq!(updated.state().linear_velocity(), Vec3::X);
    assert_eq!(updated.state().angular_velocity(), Vec3::Z);
    assert_eq!(
        world.set_part_mass(
            root,
            mass,
            crate::cluster::MassChangeMode::ConserveRemainingMomentum
        ),
        Err(ClusterError::InvalidMotion)
    );
    assert_eq!(world.body(body), Some(&updated));
    let invalid = crate::cluster::MassTransfer {
        linear_impulse: Vec3::splat(f64::NAN),
        ..transfer
    };
    assert_eq!(
        world.set_part_mass_with_transfer(root, mass, invalid),
        Err(ClusterError::InvalidMotion)
    );
    assert_eq!(world.body(body), Some(&updated));
    let overflow = crate::cluster::MassTransfer {
        world_origin: Vec3::splat(f64::MAX),
        linear_impulse: Vec3::splat(f64::MAX),
        angular_impulse: Vec3::splat(f64::MAX),
    };
    assert!(
        world
            .set_part_mass_with_transfer(root, mass, overflow)
            .is_err()
    );
    assert_eq!(world.body(body), Some(&updated));
    assert_eq!(world.cluster_part(root).map(ClusterPart::mass), Some(mass));
    let unlinked = part(&mut world, body);
    assert_eq!(
        world.set_part_mass(
            unlinked,
            mass,
            crate::cluster::MassChangeMode::KeepVelocityField
        ),
        Err(ClusterError::InvalidTopology)
    );
    assert_eq!(world.body(body), Some(&updated));
}

#[test]
fn changing_static_cluster_mass_rejects_world_center_overflow() {
    let mut world = PhysicsWorld::new();
    let pose = match Transform::new(Vec3::splat(f64::MAX), kapitsa_math::Quat::IDENTITY) {
        Ok(value) => value,
        Err(error) => panic!("pose: {error}"),
    };
    let body = match world.add_body(RigidBody::new_static(pose)) {
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
    let before = world.cluster_part(root).map(ClusterPart::mass);
    let replacement = match MassProperties::new(1.0, Vec3::splat(f64::MAX), Mat3::IDENTITY) {
        Ok(value) => value,
        Err(error) => panic!("mass: {error}"),
    };
    assert_eq!(
        world.set_part_mass(
            root,
            replacement,
            crate::cluster::MassChangeMode::KeepVelocityField
        ),
        Err(ClusterError::ArithmeticOverflow)
    );
    assert_eq!(world.cluster_part(root).map(ClusterPart::mass), before);
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

#[test]
fn enable_cluster_assigns_existing_colliders_to_identity_root() {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("body"))
        .expect("body id");
    let collider = world
        .add_collider(
            body,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).expect("sphere")),
                Transform::identity(),
            ),
        )
        .expect("collider");

    let root = world.enable_cluster(body).expect("enable cluster");
    assert_eq!(world.part_body(root), Some(body));
    assert_eq!(world.cluster(body).map(RigidCluster::part_count), Some(1));
    assert_eq!(
        world.cluster_part(root).map(ClusterPart::colliders),
        Some([collider].as_slice())
    );
    assert_eq!(world.validate_clusters(), Ok(()));
    assert_eq!(
        world.enable_cluster(body),
        Err(ClusterError::AlreadyClustered)
    );
}

#[test]
fn enable_cluster_rejects_incompatible_mass_without_mutation() {
    let mut world = PhysicsWorld::new();
    let original = MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let replacement = MassProperties::new(3.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), original).expect("body"))
        .expect("body id");

    assert_eq!(
        world.enable_cluster_with_mass(body, replacement),
        Err(ClusterError::InvalidMass)
    );
    assert!(world.cluster(body).is_none());
    assert_eq!(
        world.body(body).and_then(|body| body.physics().mass()),
        Some(original)
    );
}

#[test]
fn add_cluster_part_keeps_part_local_geometry_and_updates_mass() {
    let mut world = PhysicsWorld::new();
    let root_mass = MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("root mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), root_mass).expect("body"))
        .expect("body id");
    let root = world.enable_cluster(body).expect("root");
    let part_mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("part mass");
    let part = world
        .add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::identity(),
                mass: part_mass,
                colliders: vec![Collider::new(
                    Shape::Sphere(Sphere::new(0.5).expect("sphere")),
                    Transform::new(Vec3::X, kapitsa_math::Quat::IDENTITY).expect("collider pose"),
                )],
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame::IDENTITY,
                frame_b: JointFrame::IDENTITY,
                strength: WeldStrength::Unbreakable,
                stiffness: WeldStiffness {
                    linear: 1.0,
                    angular: 1.0,
                },
            },
        )
        .expect("part");

    let collider = world
        .cluster_part(part)
        .and_then(|part| part.colliders().first().copied())
        .expect("part collider");
    assert_eq!(world.collider_body(collider), Some(body));
    assert_eq!(
        world
            .collider(collider)
            .map(|value| value.local_transform().translation()),
        Some(Vec3::X)
    );
    assert_eq!(world.cluster(body).map(RigidCluster::part_count), Some(2));
    assert_eq!(world.cluster(body).map(RigidCluster::weld_count), Some(1));
    assert_eq!(
        world
            .body(body)
            .and_then(|body| body.physics().mass())
            .map(|mass| mass.mass()),
        Some(3.0)
    );
    assert_eq!(world.validate_clusters(), Ok(()));
}

#[test]
fn add_cluster_part_rejects_mismatched_anchors_without_mutation() {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("body"))
        .expect("body id");
    let root = world.enable_cluster(body).expect("root");
    let before = world.cluster(body).cloned();
    let result = world.add_cluster_part(
        root,
        ClusterPartDesc {
            local_transform: Transform::identity(),
            mass,
            colliders: Vec::new(),
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        },
        WeldDesc {
            frame_a: JointFrame::new(Vec3::ZERO, kapitsa_math::Quat::IDENTITY),
            frame_b: JointFrame::new(Vec3::X, kapitsa_math::Quat::IDENTITY),
            strength: WeldStrength::Unbreakable,
            stiffness: WeldStiffness {
                linear: 1.0,
                angular: 1.0,
            },
        },
    );
    assert_eq!(result, Err(ClusterError::IncompatibleAnchors));
    assert_eq!(world.cluster(body).cloned(), before);
    assert_eq!(world.parts.len(), 1);
    assert_eq!(world.welds.len(), 0);
}

#[test]
fn part_force_source_uses_part_transform_and_keeps_target_identity() {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("body"))
        .expect("body id");
    let root = world.enable_cluster(body).expect("root");
    let part = world
        .add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::new(Vec3::X, kapitsa_math::Quat::IDENTITY)
                    .expect("part transform"),
                mass,
                colliders: Vec::new(),
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame {
                    local_anchor: Vec3::X,
                    local_rotation: kapitsa_math::Quat::IDENTITY,
                },
                frame_b: JointFrame::IDENTITY,
                strength: WeldStrength::Unbreakable,
                stiffness: WeldStiffness {
                    linear: 1.0,
                    angular: 1.0,
                },
            },
        )
        .expect("part");
    let source = world
        .add_force_source_for_part(part, Vec3::ZERO, Vec3::Y)
        .expect("force source");

    assert_eq!(
        world.force_source(source).map(ForceSource::target),
        Some(ForceTarget::Part(part))
    );
    world
        .prepare_force_source_updates()
        .expect("prepare force source");
    assert_eq!(
        world.body(body).map(|body| body.forces().torque()),
        Some(Vec3::Z * 0.5)
    );
}

#[test]
fn removing_cluster_body_removes_owned_parts_and_welds() {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("body"))
        .expect("body id");
    let root = world.enable_cluster(body).expect("root");
    let child = world
        .add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::identity(),
                mass,
                colliders: Vec::new(),
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame::IDENTITY,
                frame_b: JointFrame::IDENTITY,
                strength: WeldStrength::Unbreakable,
                stiffness: WeldStiffness {
                    linear: 1.0,
                    angular: 1.0,
                },
            },
        )
        .expect("child");
    let weld = world.cluster(body).expect("cluster").welds()[0];
    assert!(world.remove_body(body).is_some());
    assert!(world.cluster_part(root).is_none());
    assert!(world.cluster_part(child).is_none());
    assert!(world.weld(weld).is_none());
    assert_eq!(world.validate_clusters(), Ok(()));
}

#[test]
fn removing_cluster_root_cleans_part_targets_and_selects_stable_root() {
    let mut world = PhysicsWorld::new();
    let root_mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("root mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), root_mass).expect("body"))
        .expect("body id");
    let root = world.enable_cluster(body).expect("root");
    let child_mass = MassProperties::new(2.0, Vec3::ZERO, Mat3::IDENTITY).expect("child mass");
    let child = world
        .add_cluster_part(
            root,
            ClusterPartDesc {
                local_transform: Transform::new(Vec3::X, kapitsa_math::Quat::IDENTITY)
                    .expect("part transform"),
                mass: child_mass,
                colliders: Vec::new(),
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
            },
            WeldDesc {
                frame_a: JointFrame {
                    local_anchor: Vec3::X,
                    local_rotation: kapitsa_math::Quat::IDENTITY,
                },
                frame_b: JointFrame::IDENTITY,
                strength: WeldStrength::Unbreakable,
                stiffness: WeldStiffness {
                    linear: 1.0,
                    angular: 1.0,
                },
            },
        )
        .expect("child");
    let source = world
        .add_force_source_for_part(root, Vec3::ZERO, Vec3::Y)
        .expect("root force source");

    assert_eq!(world.remove_cluster_part(root), Ok(()));
    assert!(world.cluster_part(root).is_none());
    assert_eq!(world.cluster(body).map(RigidCluster::root), Some(child));
    assert_eq!(world.cluster(body).map(RigidCluster::weld_count), Some(0));
    assert_eq!(
        world
            .body(body)
            .and_then(|body| body.physics().mass())
            .map(|mass| mass.mass()),
        Some(2.0)
    );
    assert!(world.force_source(source).is_none());
    assert_eq!(world.validate_clusters(), Ok(()));
    assert_eq!(
        world.remove_cluster_part(child),
        Err(ClusterError::InvalidTopology)
    );
    assert_eq!(world.cluster(body).map(RigidCluster::root), Some(child));
}
