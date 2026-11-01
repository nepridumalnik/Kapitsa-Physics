use crate::{
    collider::Collider,
    collision::{
        CollisionPair,
        island::IslandLayout,
        manifold::ContactManifold,
        solver::{SolverManifold, SolverMaterial},
    },
    joint::{FixedJoint, Joint, JointCommon, JointConstraint, JointFrame},
    rigid_body::{BodyType, RigidBody},
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};

fn pair() -> CollisionPair {
    let mut world = PhysicsWorld::new();
    let body_a = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let body_b = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let collider_a = world
        .add_collider(
            body_a,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    let collider_b = world
        .add_collider(
            body_b,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    CollisionPair::new(collider_a, collider_b).unwrap()
}

fn manifold(body_a: usize, body_b: usize) -> SolverManifold {
    SolverManifold {
        body_a,
        body_b,
        manifold: ContactManifold::new(pair()),
        material: SolverMaterial {
            restitution: 0.0,
            static_friction: 0.0,
            dynamic_friction: 0.0,
        },
    }
}

fn joint(body_a: usize, body_b: usize, enabled: bool) -> JointConstraint {
    let mut common = JointCommon::new(JointFrame::IDENTITY, JointFrame::IDENTITY);
    common.enabled = enabled;
    JointConstraint {
        body_a,
        body_b,
        joint: Joint::Fixed(FixedJoint::new(common)),
    }
}

#[test]
fn static_and_kinematic_bodies_do_not_merge_dynamic_islands() {
    let body_types = [
        BodyType::Dynamic,
        BodyType::Static,
        BodyType::Dynamic,
        BodyType::Kinematic,
        BodyType::Dynamic,
    ];
    let manifolds = [
        manifold(0, 1),
        manifold(2, 1),
        manifold(2, 3),
        manifold(4, 3),
    ];
    let mut layout = IslandLayout::default();
    layout.build(&body_types, &manifolds).unwrap();

    assert_eq!(layout.island_count(), 3);
    assert_eq!(layout.body_island(0), Some(0));
    assert_eq!(layout.body_island(2), Some(1));
    assert_eq!(layout.body_island(4), Some(2));
    assert_eq!(layout.body_island(1), None);
    assert_eq!(layout.body_island(3), None);
    assert_eq!(layout.manifold_island(0), Some(0));
    assert_eq!(layout.manifold_island(1), Some(1));
    assert_eq!(layout.manifold_island(2), Some(1));
    assert_eq!(layout.manifold_island(3), Some(2));
}

#[test]
fn dynamic_contact_chains_share_one_deterministic_island() {
    let body_types = [BodyType::Dynamic; 6];
    let manifolds = [
        manifold(3, 4),
        manifold(1, 2),
        manifold(2, 3),
        manifold(0, 1),
    ];
    let mut layout = IslandLayout::default();
    layout.build(&body_types, &manifolds).unwrap();
    assert_eq!(layout.island_count(), 1);
    assert!((0..5).all(|body| layout.body_island(body) == Some(0)));
    assert_eq!(layout.body_island(5), None);
    assert!((0..4).all(|manifold| layout.manifold_island(manifold) == Some(0)));
}

#[test]
fn constraints_without_dynamic_bodies_have_no_solver_island() {
    let body_types = [BodyType::Static, BodyType::Kinematic];
    let manifolds = [manifold(0, 1)];
    let mut layout = IslandLayout::default();
    layout.build(&body_types, &manifolds).unwrap();
    assert_eq!(layout.island_count(), 0);
    assert_eq!(layout.manifold_island(0), None);
}

#[test]
fn joint_and_contact_edges_form_one_component_without_merging_shared_static_supports() {
    let body_types = [
        BodyType::Dynamic,
        BodyType::Dynamic,
        BodyType::Dynamic,
        BodyType::Static,
        BodyType::Dynamic,
    ];
    let manifolds = [manifold(1, 2)];
    let joints = [joint(0, 1, true), joint(0, 3, true), joint(4, 3, true)];
    let mut layout = IslandLayout::default();
    layout
        .build_with_joints(&body_types, &manifolds, &joints)
        .unwrap();

    assert_eq!(layout.island_count(), 2);
    assert_eq!(layout.body_island(0), Some(0));
    assert_eq!(layout.body_island(1), Some(0));
    assert_eq!(layout.body_island(2), Some(0));
    assert_eq!(layout.body_island(4), Some(1));
    assert_eq!(layout.joint_island(0), Some(0));
    assert_eq!(layout.joint_island(1), Some(0));
    assert_eq!(layout.joint_island(2), Some(1));
    assert_eq!(layout.manifold_island(0), Some(0));
}

#[test]
fn disabled_joints_do_not_create_or_merge_solver_islands() {
    let body_types = [BodyType::Dynamic, BodyType::Dynamic];
    let joints = [joint(0, 1, false)];
    let mut layout = IslandLayout::default();
    layout.build_with_joints(&body_types, &[], &joints).unwrap();

    assert_eq!(layout.island_count(), 0);
    assert_eq!(layout.body_island(0), None);
    assert_eq!(layout.body_island(1), None);
    assert_eq!(layout.joint_island(0), None);
}
