use crate::{
    collision::{epa::EpaWorkspace, gjk::Simplex, gjk::intersect},
    shape::{Capsule, Cuboid, Shape, Sphere},
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

#[test]
fn epa_reports_finite_depth_for_identical_and_nested_shapes() {
    let cuboid = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("cuboid"));
    let sphere = Shape::Sphere(Sphere::new(0.25).expect("sphere"));
    let identity = Transform::identity();

    for (shape_a, shape_b, expected_depth) in [(&cuboid, &cuboid, 2.0), (&cuboid, &sphere, 1.25)] {
        let result = intersect(shape_a, &identity, shape_b, &identity).expect("GJK");
        assert!(result.intersects);
        let contact = EpaWorkspace::new()
            .solve(&result.simplex, shape_a, &identity, shape_b, &identity)
            .expect("EPA");
        assert!((contact.penetration - expected_depth).abs() < 1.0e-6);
        assert!((contact.normal.length() - 1.0).abs() < 1.0e-10);
        assert!(contact.point_a.is_finite() && contact.point_b.is_finite());
        assert!(
            ((contact.point_a - contact.point_b).dot(contact.normal) - expected_depth).abs()
                < 1.0e-6
        );
    }
}

#[test]
fn epa_handles_perpendicular_capsules_and_near_touching_boxes() {
    let horizontal = Shape::Capsule(
        Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.25)
            .expect("horizontal capsule"),
    );
    let vertical = Shape::Capsule(
        Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.25)
            .expect("vertical capsule"),
    );
    let identity = Transform::identity();
    let crossed = intersect(&horizontal, &identity, &vertical, &identity).expect("GJK");
    assert!(crossed.intersects);
    let contact = EpaWorkspace::new()
        .solve(
            &crossed.simplex,
            &horizontal,
            &identity,
            &vertical,
            &identity,
        )
        .expect("capsule EPA");
    assert!(contact.penetration >= 0.0 && contact.penetration.is_finite());
    assert!((contact.normal.length() - 1.0).abs() < 1.0e-10);
    assert!(contact.point_a.is_finite() && contact.point_b.is_finite());
    assert!(
        ((contact.point_a - contact.point_b).dot(contact.normal) - contact.penetration).abs()
            < 1.0e-6
    );

    let box_a = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("box A"));
    let box_b = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("box B"));
    let slight_overlap = Transform::new(
        Vec3::new(2.0 - 1.0e-5, 0.0, 0.0),
        kapitsa_math::Quat::IDENTITY,
    )
    .expect("near-contact transform");
    let near = intersect(&box_a, &identity, &box_b, &slight_overlap).expect("near GJK");
    assert!(near.intersects);
    let contact = EpaWorkspace::new()
        .solve(&near.simplex, &box_a, &identity, &box_b, &slight_overlap)
        .expect("near EPA");
    assert!((contact.penetration - 1.0e-5).abs() < 1.0e-7);
    assert!(contact.normal.dot(Vec3::X) > 0.99);
    assert!(
        ((contact.point_a - contact.point_b).dot(contact.normal) - contact.penetration).abs()
            < 1.0e-7
    );

    let thin = Shape::Cuboid(Cuboid::new(2.0, 1.0e-4, 1.0e-4).expect("thin box"));
    let thin_offset = Transform::new(Vec3::new(1.5, 0.0, 0.0), kapitsa_math::Quat::IDENTITY)
        .expect("thin-box offset");
    let thin_gjk = intersect(&thin, &identity, &thin, &thin_offset).expect("thin GJK");
    assert!(thin_gjk.intersects);
    let thin_contact = EpaWorkspace::new()
        .solve(&thin_gjk.simplex, &thin, &identity, &thin, &thin_offset)
        .expect("thin EPA");
    assert!((thin_contact.penetration - 1.0e-4).abs() < 1.0e-7);
    assert!((thin_contact.normal.length() - 1.0).abs() < 1.0e-10);
    assert!(thin_contact.point_a.is_finite() && thin_contact.point_b.is_finite());
}

#[test]
fn epa_face_orientation_is_independent_of_simplex_winding() {
    let cuboid = Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("cuboid"));
    let identity = Transform::identity();
    let offset =
        Transform::new(Vec3::new(1.5, 0.1, -0.05), kapitsa_math::Quat::IDENTITY).expect("offset");
    let result = intersect(&cuboid, &identity, &cuboid, &offset).expect("GJK");
    assert!(result.intersects && result.simplex.count == 4);

    let mut checked = 0;
    for a in 0..4 {
        for b in 0..4 {
            for c in 0..4 {
                for d in 0..4 {
                    if a == b || a == c || a == d || b == c || b == d || c == d {
                        continue;
                    }
                    let simplex = Simplex {
                        points: [
                            result.simplex.points[a],
                            result.simplex.points[b],
                            result.simplex.points[c],
                            result.simplex.points[d],
                        ],
                        count: 4,
                    };
                    let contact = EpaWorkspace::new()
                        .solve(&simplex, &cuboid, &identity, &cuboid, &offset)
                        .expect("permuted EPA");
                    assert!(contact.penetration.is_finite() && contact.penetration > 0.0);
                    assert!((contact.penetration - 0.5).abs() < 1.0e-6);
                    assert!(contact.normal.dot(Vec3::X) > 0.99);
                    assert!(contact.point_a.is_finite() && contact.point_b.is_finite());
                    assert!(
                        ((contact.point_a - contact.point_b).dot(contact.normal)
                            - contact.penetration)
                            .abs()
                            < 1.0e-6
                    );
                    checked += 1;
                }
            }
        }
    }
    assert_eq!(checked, 24);
}
