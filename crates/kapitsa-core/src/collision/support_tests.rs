use crate::{
    collision::{CollisionError, support::support_world},
    shape::{
        Capsule, Convex, Cuboid, Cylinder, Shape, Sphere, TaperedCapsule, TaperedCylinder, Triangle,
    },
    transform::Transform,
};
use kapitsa_math::{Quat, Vec3};

#[test]
fn support_mapping_covers_all_seven_convex_shapes() {
    let tetra = Convex::new(
        vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z],
        vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
    )
    .expect("tetrahedron");
    let shapes = [
        (
            Shape::Sphere(Sphere::new(2.0).expect("sphere")),
            Vec3::X,
            Vec3::new(2.0, 0.0, 0.0),
        ),
        (
            Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0).expect("cuboid")),
            Vec3::new(1.0, -1.0, 0.0),
            Vec3::new(1.0, -2.0, 3.0),
        ),
        (
            Shape::Capsule(
                Capsule::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 2.0), 0.5).expect("capsule"),
            ),
            Vec3::Z,
            Vec3::new(0.0, 0.0, 2.5),
        ),
        (
            Shape::Cylinder(
                Cylinder::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 2.0), 0.5).expect("cylinder"),
            ),
            Vec3::X,
            Vec3::new(0.5, 0.0, 0.0),
        ),
        (
            Shape::TaperedCapsule(
                TaperedCapsule::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 2.0), 0.25, 0.5)
                    .expect("tapered capsule"),
            ),
            Vec3::Z,
            Vec3::new(0.0, 0.0, 2.5),
        ),
        (
            Shape::TaperedCylinder(
                TaperedCylinder::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 2.0), 0.25, 0.5)
                    .expect("tapered cylinder"),
            ),
            Vec3::X,
            Vec3::new(0.5, 0.0, 2.0),
        ),
        (Shape::Convex(tetra), Vec3::X, Vec3::X),
    ];

    for (shape, direction, expected) in shapes {
        let (point, _) =
            support_world(&shape, &Transform::identity(), direction).expect("support point");
        assert_eq!(point, expected);
    }
}

#[test]
fn support_mapping_applies_rotation_and_translation() {
    let shape = Shape::Cuboid(Cuboid::new(2.0, 4.0, 6.0).expect("cuboid"));
    let rotation = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2).expect("rotation");
    let transform = Transform::new(Vec3::new(3.0, 4.0, 5.0), rotation).expect("transform");
    let (point, _) = support_world(&shape, &transform, Vec3::Y).expect("support");
    assert!((point.y - 5.0).abs() < 1.0e-12);
}

#[test]
fn triangle_support_mapping_selects_the_farthest_vertex() {
    let shape = Shape::Triangle(
        Triangle::new(
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .expect("triangle"),
    );
    let (point, feature) =
        support_world(&shape, &Transform::identity(), Vec3::X).expect("support point");
    assert_eq!(point, Vec3::new(2.0, 0.0, 0.0));
    assert_eq!(feature, 1);
}

#[test]
fn support_mapping_reports_world_transform_overflow() {
    let shape = Shape::Sphere(Sphere::new(f64::MAX).expect("sphere"));
    let transform = Transform::new(Vec3::splat(f64::MAX), Quat::IDENTITY).expect("transform");
    assert_eq!(
        support_world(&shape, &transform, Vec3::X),
        Err(CollisionError::ArithmeticOverflow)
    );
}
