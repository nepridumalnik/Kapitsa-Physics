use crate::{
    collision::{epa::EpaWorkspace, gjk::intersect},
    shape::{Cuboid, Shape},
    transform::Transform,
};
use kapitsa_math::Vec3;

#[test]
fn epa_finds_penetration_and_a_to_b_normal_for_overlapping_cuboids() {
    let cuboid = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("cuboid"));
    let first = Transform::identity();
    let second =
        Transform::new(Vec3::new(1.5, 0.0, 0.0), kapitsa_math::Quat::IDENTITY).expect("transform");
    let gjk = intersect(&cuboid, &first, &cuboid, &second).expect("GJK");
    assert!(gjk.intersects, "overlapping cuboids must intersect");
    let result = EpaWorkspace::new()
        .solve(&gjk.simplex, &cuboid, &first, &cuboid, &second)
        .expect("EPA");
    assert!((result.penetration - 0.5).abs() < 1.0e-8);
    assert!(result.normal.dot(Vec3::X) > 0.99);
    assert!((result.normal.length() - 1.0).abs() < 1.0e-12);
    assert!(result.point_a.is_finite() && result.point_b.is_finite());
}
