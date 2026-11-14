use super::*;
use crate::{
    aerodynamics::hull::HullSection,
    aerodynamics::{Aerodynamics, AtmosphericState},
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
};
use kapitsa_math::{Mat3, Quat, Vec3};

fn section(point: Vec3) -> HullSection {
    HullSection::new(
        Transform::new(point, Quat::IDENTITY).expect("test transform"),
        Vec3::X,
        2.0,
        3.0,
        4.0,
        5.0,
        0.2,
        0.8,
        None,
    )
    .expect("test hull section")
}

#[test]
fn primary_hull_transfers_to_root_part_without_changing_source_coordinates() {
    let mut world = PhysicsWorld::new();
    world.set_atmosphere(test_atmosphere(
        AtmosphericState::new(1.2, Vec3::ZERO).expect("state"),
    ));
    let hull = vec![section(Vec3::X * 2.0), section(-Vec3::Y)];
    let mut body = RigidBody::new_dynamic(
        Transform::identity(),
        MassProperties::new(10.0, Vec3::ZERO, Mat3::IDENTITY * 10.0).expect("mass"),
    )
    .expect("body");
    body.set_aerodynamics(Aerodynamics::Hull(hull.clone()));
    let id = world.add_body(body).expect("body id");
    let part = world.enable_cluster(id).expect("cluster");
    assert_eq!(world.part_aerodynamic_hull(part), Some(hull.as_slice()));
    assert!(matches!(
        world.body(id).expect("body").aerodynamics(),
        Aerodynamics::None
    ));
}

#[test]
fn part_hull_accessor_and_late_failure_preserve_configuration() {
    let mut world = PhysicsWorld::new();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("mass");
    let body = world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("body"))
        .expect("body id");
    let part = world.enable_cluster(body).expect("cluster");
    let hull = vec![section(Vec3::Z)];
    world
        .set_part_aerodynamic_hull(part, hull.clone())
        .expect("setter");
    assert_eq!(world.part_aerodynamic_hull(part), Some(hull.as_slice()));
    let mut foreign_world = PhysicsWorld::new();
    let foreign_body = foreign_world
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass).expect("foreign body"))
        .expect("foreign id");
    let foreign_part = foreign_world
        .enable_cluster(foreign_body)
        .expect("foreign part");
    assert_eq!(
        world.set_part_aerodynamic_hull(foreign_part, hull),
        Err(ClusterError::InvalidPartId)
    );
    assert_eq!(
        world.part_aerodynamic_hull(part),
        Some([section(Vec3::Z)].as_slice())
    );
}
