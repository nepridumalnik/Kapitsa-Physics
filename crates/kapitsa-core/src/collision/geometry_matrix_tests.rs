use crate::{
    collider::CollisionMode,
    collision::{CollisionPair, mesh::TriangleBvh, narrow_phase::NarrowPhase},
    rigid_body::RigidBody,
    shape::{
        BoneInfluence, Capsule, Convex, Cuboid, Cylinder, HeightField, LevelSet, Plane, Shape,
        SkinnedTriangleMesh, Sphere, TaperedCapsule, TaperedCylinder, Triangle, TriangleMesh,
    },
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Affine3, Quat, Vec2, Vec3};

struct Geometry {
    name: &'static str,
    shape: Shape,
    transform: Transform,
    bvh: Option<TriangleBvh>,
}

#[test]
fn all_28_convex_shape_pairs_generate_symmetric_contacts() {
    let pair = pair();
    let shapes = convex_shapes();
    let mut tested = 0;
    for left_index in 0..shapes.len() {
        for right_index in left_index..shapes.len() {
            let (left_name, left_shape) = &shapes[left_index];
            let (right_name, right_shape) = &shapes[right_index];
            let left = Geometry {
                name: left_name,
                shape: make_convex(left_shape),
                transform: at(-0.2, 0.0, 0.0),
                bvh: None,
            };
            let right = Geometry {
                name: right_name,
                shape: make_convex(right_shape),
                transform: at(0.2, 0.0, 0.0),
                bvh: None,
            };
            assert_symmetric_contact(pair, &left, &right);
            assert_known_penetration(pair, &left, &right);
            tested += 1;
        }
    }
    assert_eq!(tested, 28);
}

#[test]
fn all_42_convex_surface_shape_pairs_generate_symmetric_contacts() {
    let pair = pair();
    let convexes = convex_shapes();
    let surfaces = surface_shapes();
    let mut tested = 0;
    for (convex_name, convex_shape) in &convexes {
        for surface in &surfaces {
            let convex = Geometry {
                name: convex_name,
                shape: make_convex(convex_shape),
                transform: at(1.0, 1.0, 0.25),
                bvh: None,
            };
            assert_symmetric_contact(pair, surface, &convex);
            tested += 1;
        }
    }
    assert_eq!(tested, 42);
}

#[test]
fn triangle_cuboid_contacts_match_independent_sat_reference() {
    let pair = pair();
    let triangle_local = [
        Vec3::ZERO,
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(0.0, 2.0, 0.0),
    ];
    let triangle_rotation =
        Quat::from_axis_angle(Vec3::new(1.0, 0.4, -0.2), 0.37).expect("triangle rotation");
    let triangle_transform =
        Transform::new(Vec3::new(0.3, -0.2, 0.7), triangle_rotation).expect("triangle transform");
    let triangle = Geometry {
        name: "finite triangle",
        shape: Shape::Triangle(Triangle::from_vertices(triangle_local).expect("triangle")),
        transform: triangle_transform,
        bvh: None,
    };

    let mut cases = vec![
        (
            "face tangent",
            Vec3::new(0.6, 0.5, 0.1),
            Vec3::new(0.2, 0.2, 0.1),
            Quat::IDENTITY,
        ),
        (
            "edge tangent",
            Vec3::new(1.0, -0.1, 0.0),
            Vec3::new(0.3, 0.1, 0.1),
            Quat::IDENTITY,
        ),
        (
            "vertex tangent",
            Vec3::new(-0.1, -0.1, 0.0),
            Vec3::splat(0.1),
            Quat::IDENTITY,
        ),
        (
            "face",
            Vec3::new(0.6, 0.5, 0.05),
            Vec3::splat(0.3),
            Quat::IDENTITY,
        ),
        (
            "vertex",
            Vec3::new(-0.05, -0.05, 0.0),
            Vec3::splat(0.2),
            Quat::IDENTITY,
        ),
        (
            "edge",
            Vec3::new(1.9, 0.05, 0.0),
            Vec3::splat(0.3),
            Quat::IDENTITY,
        ),
        (
            "outside face",
            Vec3::new(1.9, 1.9, 0.0),
            Vec3::splat(0.2),
            Quat::IDENTITY,
        ),
        (
            "outside vertex",
            Vec3::new(2.25, 0.0, 0.0),
            Vec3::splat(0.2),
            Quat::IDENTITY,
        ),
    ];
    let rotated_box = Quat::from_axis_angle(Vec3::Z, 0.61).expect("box rotation");
    cases.push((
        "rotated pair",
        Vec3::new(0.6, 0.5, 0.05),
        Vec3::splat(0.3),
        rotated_box,
    ));

    for (name, local_center, half_size, box_rotation) in cases {
        let center = triangle_transform.transform_point(local_center);
        let box_transform =
            Transform::new(center, box_rotation * triangle_rotation).expect("box transform");
        let cuboid = Geometry {
            name: "cuboid",
            shape: Shape::Cuboid(Cuboid::from_vec(half_size * 2.0).expect("cuboid")),
            transform: box_transform,
            bvh: None,
        };
        let expected = triangle_cuboid_sat(&triangle, triangle_local, &cuboid, half_size);
        let mut narrow_phase = NarrowPhase::new();
        let actual = detect(&mut narrow_phase, pair, &triangle, &cuboid)
            .unwrap_or_else(|error| panic!("{name}: {error}"));
        assert_eq!(actual.is_some(), expected, "{name}");
        if let Some(manifold) = actual {
            let contact = manifold.points().first().expect("contact point");
            assert!(contact.normal.is_finite(), "{name}: invalid normal");
            assert!(contact.penetration.is_finite(), "{name}: invalid depth");
            let mut reverse_narrow_phase = NarrowPhase::new();
            let reverse = detect(&mut reverse_narrow_phase, pair, &cuboid, &triangle)
                .unwrap_or_else(|error| panic!("{name} reversed: {error}"))
                .expect("reversed contact");
            let reverse_normal = reverse.points().first().expect("contact point").normal;
            assert!(
                (contact.normal + reverse_normal).length() < 1.0e-6,
                "{name}"
            );
        }
    }
}

#[test]
fn triangle_sphere_contacts_match_independent_distance_reference() {
    let pair = pair();
    let triangle_vertices = [
        Vec3::ZERO,
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(0.0, 2.0, 0.0),
    ];
    let triangle_rotation =
        Quat::from_axis_angle(Vec3::new(0.3, 1.0, -0.4), 0.52).expect("triangle rotation");
    let triangle_transform =
        Transform::new(Vec3::new(-0.4, 0.8, 0.2), triangle_rotation).expect("triangle transform");
    let triangle = Geometry {
        name: "finite triangle",
        shape: Shape::Triangle(Triangle::from_vertices(triangle_vertices).expect("triangle")),
        transform: triangle_transform,
        bvh: None,
    };
    let cases = [
        ("face", Vec3::new(0.5, 0.5, 0.1), 0.2),
        ("edge", Vec3::new(1.0, -0.1, 0.0), 0.2),
        ("vertex", Vec3::new(-0.1, -0.1, 0.0), 0.2),
        ("outside edge", Vec3::new(2.2, -0.2, 0.0), 0.2),
        ("outside central region", Vec3::new(1.5, 1.5, 0.0), 0.2),
        ("zero radial distance", Vec3::new(0.5, 0.5, 0.0), 0.2),
    ];

    for (name, local_center, radius) in cases {
        let sphere_transform = Transform::new(
            triangle_transform.transform_point(local_center),
            Quat::from_axis_angle(Vec3::Y, 0.31).expect("sphere rotation"),
        )
        .expect("sphere transform");
        let sphere = Geometry {
            name: "sphere",
            shape: Shape::Sphere(Sphere::new(radius).expect("sphere")),
            transform: sphere_transform,
            bvh: None,
        };
        let world_triangle =
            triangle_vertices.map(|vertex| triangle_transform.transform_point(vertex));
        let expected_distance =
            point_triangle_distance_reference(sphere_transform.translation(), world_triangle);
        let expected_contact = expected_distance <= radius;
        let mut narrow_phase = NarrowPhase::new();
        let manifold = detect(&mut narrow_phase, pair, &triangle, &sphere)
            .unwrap_or_else(|error| panic!("{name}: {error}"));
        assert_eq!(manifold.is_some(), expected_contact, "{name}");
        if let Some(manifold) = manifold {
            let contact = manifold.points().first().expect("contact point");
            let expected_penetration = (radius - expected_distance).max(0.0);
            assert!(
                (contact.penetration - expected_penetration).abs() < 1.0e-9,
                "{name}: expected {expected_penetration}, got {}",
                contact.penetration
            );
            let mut reverse_narrow_phase = NarrowPhase::new();
            let reverse = detect(&mut reverse_narrow_phase, pair, &sphere, &triangle)
                .unwrap_or_else(|error| panic!("{name} reversed: {error}"))
                .expect("reversed contact");
            let reverse_normal = reverse.points().first().expect("contact point").normal;
            assert!(
                (contact.normal + reverse_normal).length() < 1.0e-6,
                "{name}"
            );
        }
    }
}

#[test]
fn triangle_contacts_match_independent_samples_for_supported_convex_shapes() {
    let pair = pair();
    let triangle_vertices = [
        Vec3::ZERO,
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(1.0, 2.0, 0.0),
    ];
    let triangle = Geometry {
        name: "finite triangle",
        shape: Shape::Triangle(Triangle::from_vertices(triangle_vertices).expect("triangle")),
        transform: Transform::identity(),
        bvh: None,
    };
    let convexes = convex_shapes().into_iter().filter(|(name, _)| {
        matches!(
            *name,
            "sphere" | "cuboid" | "capsule" | "cylinder" | "convex hull"
        )
    });

    for (name, shape) in convexes {
        for (center, expected) in [
            (Vec3::new(1.0, 0.5, 0.0), true),
            (Vec3::new(3.0, 0.5, 0.0), false),
        ] {
            let convex = Geometry {
                name,
                shape: make_convex(&shape),
                transform: Transform::new(center, Quat::IDENTITY).expect("convex transform"),
                bvh: None,
            };
            let reference = triangle_samples_intersect_shape(
                triangle_vertices,
                &convex.shape,
                &convex.transform,
            );
            assert_eq!(reference, expected, "reference case for {name}");
            let mut narrow_phase = NarrowPhase::new();
            let actual = detect(&mut narrow_phase, pair, &triangle, &convex)
                .unwrap_or_else(|error| panic!("{name}: {error}"));
            assert_eq!(actual.is_some(), reference, "narrow phase for {name}");
        }
    }
}

fn triangle_samples_intersect_shape(
    triangle: [Vec3; 3],
    shape: &Shape,
    transform: &Transform,
) -> bool {
    let Some(inverse) = transform.inverse().ok() else {
        return false;
    };
    const SUBDIVISIONS: usize = 64;
    for first in 0..=SUBDIVISIONS {
        for second in 0..=(SUBDIVISIONS - first) {
            let u = first as f64 / SUBDIVISIONS as f64;
            let v = second as f64 / SUBDIVISIONS as f64;
            let point =
                triangle[0] + (triangle[1] - triangle[0]) * u + (triangle[2] - triangle[0]) * v;
            if shape_contains_point_reference(shape, &inverse, point) {
                return true;
            }
        }
    }
    false
}

fn shape_contains_point_reference(shape: &Shape, inverse: &Transform, world_point: Vec3) -> bool {
    let point = inverse.transform_point(world_point);
    match shape {
        Shape::Sphere(sphere) => point.length_squared() <= sphere.radius().powi(2),
        Shape::Cuboid(cuboid) => {
            let half = cuboid.half_extents();
            point.abs().x <= half.x && point.abs().y <= half.y && point.abs().z <= half.z
        }
        Shape::Capsule(capsule) => {
            let segment = capsule.end() - capsule.start();
            let amount =
                ((point - capsule.start()).dot(segment) / segment.length_squared()).clamp(0.0, 1.0);
            (point - (capsule.start() + segment * amount)).length_squared()
                <= capsule.radius().powi(2)
        }
        Shape::Cylinder(cylinder) => {
            let axis = cylinder.end() - cylinder.start();
            let axis_length = axis.length();
            let direction = axis / axis_length;
            let offset = point - cylinder.start();
            let axial = offset.dot(direction);
            let radial = offset - direction * axial;
            axial >= 0.0
                && axial <= axis_length
                && radial.length_squared() <= cylinder.radius().powi(2)
        }
        Shape::Convex(convex) => convex.faces().iter().all(|&[a, b, c]| {
            let vertices = convex.vertices();
            let origin = vertices[a];
            let normal = (vertices[b] - origin).cross(vertices[c] - origin);
            (point - origin).dot(normal) <= 1.0e-10 * normal.length()
        }),
        _ => false,
    }
}

fn point_triangle_distance_reference(point: Vec3, triangle: [Vec3; 3]) -> f64 {
    let [a, b, c] = triangle;
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);
    let normal_length = normal.length();
    let signed_distance = (point - a).dot(normal) / normal_length;
    let projected = point - normal * (signed_distance / normal_length);
    let ap = projected - a;
    let d00 = ab.dot(ab);
    let d01 = ab.dot(ac);
    let d11 = ac.dot(ac);
    let d20 = ap.dot(ab);
    let d21 = ap.dot(ac);
    let denominator = d00 * d11 - d01 * d01;
    let beta = (d11 * d20 - d01 * d21) / denominator;
    let gamma = (d00 * d21 - d01 * d20) / denominator;
    if beta >= 0.0 && gamma >= 0.0 && beta + gamma <= 1.0 {
        return signed_distance.abs();
    }
    triangle
        .into_iter()
        .zip([triangle[1], triangle[2], triangle[0]])
        .map(|(start, end)| {
            let edge = end - start;
            let amount = ((point - start).dot(edge) / edge.dot(edge)).clamp(0.0, 1.0);
            (point - (start + edge * amount)).length()
        })
        .fold(f64::INFINITY, f64::min)
}

fn triangle_cuboid_sat(
    triangle: &Geometry,
    triangle_local: [Vec3; 3],
    cuboid: &Geometry,
    half_extents: Vec3,
) -> bool {
    let triangle_vertices = triangle_local.map(|vertex| triangle.transform.transform_point(vertex));
    let box_axes = [
        cuboid.transform.transform_vector(Vec3::X),
        cuboid.transform.transform_vector(Vec3::Y),
        cuboid.transform.transform_vector(Vec3::Z),
    ];
    let triangle_edges = [
        triangle_vertices[1] - triangle_vertices[0],
        triangle_vertices[2] - triangle_vertices[1],
        triangle_vertices[0] - triangle_vertices[2],
    ];
    let mut axes = Vec::with_capacity(13);
    axes.extend(box_axes);
    axes.push(triangle_edges[0].cross(triangle_edges[1]));
    for edge in triangle_edges {
        for box_axis in box_axes {
            axes.push(edge.cross(box_axis));
        }
    }

    axes.into_iter().all(|axis| {
        let Some(axis) = axis.try_normalize() else {
            return true;
        };
        let triangle_projection = triangle_vertices.map(|vertex| vertex.dot(axis));
        let triangle_min = triangle_projection
            .into_iter()
            .fold(f64::INFINITY, f64::min);
        let triangle_max = triangle_projection
            .into_iter()
            .fold(f64::NEG_INFINITY, f64::max);
        let box_center = cuboid.transform.translation().dot(axis);
        let box_radius = box_axes
            .iter()
            .zip([half_extents.x, half_extents.y, half_extents.z])
            .map(|(box_axis, half)| half * box_axis.dot(axis).abs())
            .sum::<f64>();
        triangle_max >= box_center - box_radius && triangle_min <= box_center + box_radius
    })
}

#[test]
fn all_28_convex_shape_pairs_reject_separated_shapes_symmetrically() {
    let pair = pair();
    let shapes = convex_shapes();
    for left_index in 0..shapes.len() {
        for right_index in left_index..shapes.len() {
            let (left_name, left_shape) = &shapes[left_index];
            let (right_name, right_shape) = &shapes[right_index];
            let left = Geometry {
                name: left_name,
                shape: make_convex(left_shape),
                transform: at(-10.0, 0.0, 0.0),
                bvh: None,
            };
            let right = Geometry {
                name: right_name,
                shape: make_convex(right_shape),
                transform: at(10.0, 0.0, 0.0),
                bvh: None,
            };
            assert_no_contact(pair, &left, &right);
        }
    }
}

#[test]
fn all_28_convex_pairs_remain_symmetric_when_rotated_and_deeply_overlapping() {
    let pair = pair();
    let shapes = convex_shapes();
    let rotation_a = Quat::from_axis_angle(Vec3::Z, 0.37).expect("rotation A");
    let rotation_b = Quat::from_axis_angle(Vec3::X, -0.53).expect("rotation B");
    for left_index in 0..shapes.len() {
        for right_index in left_index..shapes.len() {
            let (left_name, left_shape) = &shapes[left_index];
            let (right_name, right_shape) = &shapes[right_index];
            let left_shape = make_convex(left_shape);
            let right_shape = make_convex(right_shape);
            let left = Geometry {
                name: left_name,
                shape: make_convex(&left_shape),
                transform: Transform::new(Vec3::new(-0.15, 0.0, 0.0), rotation_a)
                    .expect("rotated transform A"),
                bvh: None,
            };
            let right = Geometry {
                name: right_name,
                shape: make_convex(&right_shape),
                transform: Transform::new(Vec3::new(0.15, 0.0, 0.0), rotation_b)
                    .expect("rotated transform B"),
                bvh: None,
            };
            assert_symmetric_contact(pair, &left, &right);

            let overlap_left = Geometry {
                transform: Transform::new(Vec3::new(-0.02, 0.0, 0.0), rotation_a)
                    .expect("overlap transform A"),
                ..left
            };
            let overlap_right = Geometry {
                transform: Transform::new(Vec3::new(0.02, 0.0, 0.0), rotation_b)
                    .expect("overlap transform B"),
                ..right
            };
            assert_symmetric_contact(pair, &overlap_left, &overlap_right);
        }
    }
}

#[test]
fn small_sphere_fully_embedded_in_rotated_cuboid_has_symmetric_contact() {
    let pair = pair();
    let rotation_a =
        Quat::from_axis_angle(Vec3::new(0.3, 1.0, -0.2), 0.6).expect("cuboid rotation");
    let rotation_b = Quat::from_axis_angle(Vec3::Z, -0.4).expect("sphere rotation");
    let cuboid = Geometry {
        name: "large cuboid",
        shape: Shape::Cuboid(Cuboid::new(4.0, 3.0, 2.0).expect("cuboid")),
        transform: Transform::new(Vec3::ZERO, rotation_a).expect("cuboid transform"),
        bvh: None,
    };
    let sphere = Geometry {
        name: "embedded sphere",
        shape: Shape::Sphere(Sphere::new(0.2).expect("sphere")),
        transform: Transform::new(Vec3::new(0.1, -0.1, 0.05), rotation_b)
            .expect("sphere transform"),
        bvh: None,
    };
    assert_symmetric_contact(pair, &cuboid, &sphere);
}

#[test]
fn all_42_convex_surface_pairs_remain_symmetric_when_both_are_rotated() {
    let pair = pair();
    let convexes = convex_shapes();
    let rotation =
        Quat::from_axis_angle(Vec3::new(1.0, 0.5, -0.25), 0.41).expect("surface rotation");
    for (convex_name, convex_shape) in &convexes {
        for mut surface in surface_shapes() {
            let surface_transform =
                Transform::new(Vec3::ZERO, rotation).expect("rotated surface transform");
            let local_contact = Transform::new(Vec3::new(1.0, 1.0, 0.25), rotation)
                .expect("rotated convex transform");
            let convex_transform = surface_transform
                .compose(&local_contact)
                .expect("composed convex transform");
            surface.transform = surface_transform;
            let convex = Geometry {
                name: convex_name,
                shape: make_convex(convex_shape),
                transform: convex_transform,
                bvh: None,
            };
            assert_symmetric_contact(pair, &surface, &convex);
        }
    }
}

#[test]
fn all_42_convex_surface_pairs_reject_separated_shapes() {
    let pair = pair();
    for (convex_name, convex_shape) in convex_shapes() {
        for surface in surface_shapes() {
            let convex_position = if surface.name == "plane" {
                Vec3::new(0.0, 0.0, 10.0)
            } else {
                Vec3::new(100.0, 0.0, 0.0)
            };
            let convex = Geometry {
                name: convex_name,
                shape: make_convex(&convex_shape),
                transform: Transform::new(convex_position, Quat::IDENTITY)
                    .expect("separated transform"),
                bvh: None,
            };
            assert_no_contact(pair, &surface, &convex);
        }
    }
}

fn assert_no_contact(pair: CollisionPair, left: &Geometry, right: &Geometry) {
    for (first, second) in [(left, right), (right, left)] {
        let mut narrow_phase = NarrowPhase::new();
        let result = detect(&mut narrow_phase, pair, first, second)
            .unwrap_or_else(|error| panic!("{} vs {}: {error}", first.name, second.name));
        assert!(
            result.is_none(),
            "{} vs {} unexpectedly contacted while separated",
            first.name,
            second.name
        );
    }
}

fn assert_symmetric_contact(pair: CollisionPair, left: &Geometry, right: &Geometry) {
    let mut forward_narrow_phase = NarrowPhase::new();
    let forward = detect(&mut forward_narrow_phase, pair, left, right)
        .unwrap_or_else(|error| panic!("{} vs {}: {error}", left.name, right.name))
        .unwrap_or_else(|| panic!("{} vs {}: expected contact", left.name, right.name));
    let mut reverse_narrow_phase = NarrowPhase::new();
    let reverse = detect(&mut reverse_narrow_phase, pair, right, left)
        .unwrap_or_else(|error| panic!("{} vs {} reversed: {error}", left.name, right.name))
        .unwrap_or_else(|| panic!("{} vs {} reversed: expected contact", left.name, right.name));
    let normal_forward = forward.points().first().expect("contact point").normal;
    let normal_reverse = reverse.points().first().expect("contact point").normal;
    assert!(
        (normal_forward + normal_reverse).length() <= 1.0e-6,
        "{} vs {}: normals did not reverse ({normal_forward:?}, {normal_reverse:?})",
        left.name,
        right.name
    );
    assert_eq!(
        forward.len(),
        reverse.len(),
        "{} vs {}",
        left.name,
        right.name
    );
    for point in forward.points() {
        let world_a = left.transform.transform_point(point.local_a);
        let world_b = right.transform.transform_point(point.local_b);
        assert!(
            reverse.points().iter().any(|reverse_point| {
                let reverse_a = right.transform.transform_point(reverse_point.local_a);
                let reverse_b = left.transform.transform_point(reverse_point.local_b);
                (world_a - reverse_b).length() <= 1.0e-6
                    && (world_b - reverse_a).length() <= 1.0e-6
                    && (point.normal + reverse_point.normal).length() <= 1.0e-6
            }),
            "{} vs {}: reversed contacts changed body-local anchors",
            left.name,
            right.name
        );
    }
}

fn assert_known_penetration(pair: CollisionPair, left: &Geometry, right: &Geometry) {
    let expected = match (left.name, right.name) {
        ("sphere", "capsule") | ("sphere", "cylinder") => Some(0.5),
        ("capsule", "capsule") => Some(0.4),
        _ => None,
    };
    if let Some(expected) = expected {
        let mut narrow_phase = NarrowPhase::new();
        let manifold = detect(&mut narrow_phase, pair, left, right)
            .expect("known geometry should be valid")
            .expect("known geometry should contact");
        let contact = manifold.points().first().expect("contact point");
        assert!((contact.penetration - expected).abs() <= 1.0e-5);
        assert!((contact.normal - Vec3::X).length() <= 1.0e-5);
    }
}

fn detect(
    narrow_phase: &mut NarrowPhase,
    pair: CollisionPair,
    left: &Geometry,
    right: &Geometry,
) -> Result<Option<crate::collision::manifold::ContactManifold>, crate::collision::CollisionError> {
    narrow_phase.detect_with_geometry(
        pair,
        &left.shape,
        &left.transform,
        &left.transform,
        CollisionMode::Solid,
        left.bvh.as_ref(),
        None,
        &right.shape,
        &right.transform,
        &right.transform,
        CollisionMode::Solid,
        right.bvh.as_ref(),
        None,
        0.01,
    )
}

fn convex_shapes() -> Vec<(&'static str, Shape)> {
    vec![
        ("sphere", Shape::Sphere(Sphere::new(0.5).expect("sphere"))),
        (
            "cuboid",
            Shape::Cuboid(Cuboid::new(0.8, 0.8, 0.8).expect("cuboid")),
        ),
        (
            "capsule",
            Shape::Capsule(
                Capsule::new(Vec3::new(0.0, -0.25, 0.0), Vec3::new(0.0, 0.25, 0.0), 0.4)
                    .expect("capsule"),
            ),
        ),
        (
            "cylinder",
            Shape::Cylinder(
                Cylinder::new(Vec3::new(0.0, -0.25, 0.0), Vec3::new(0.0, 0.25, 0.0), 0.4)
                    .expect("cylinder"),
            ),
        ),
        (
            "tapered capsule",
            Shape::TaperedCapsule(
                TaperedCapsule::new(
                    Vec3::new(0.0, -0.25, 0.0),
                    Vec3::new(0.0, 0.25, 0.0),
                    0.4,
                    0.35,
                )
                .expect("tapered capsule"),
            ),
        ),
        (
            "tapered cylinder",
            Shape::TaperedCylinder(
                TaperedCylinder::new(
                    Vec3::new(0.0, -0.25, 0.0),
                    Vec3::new(0.0, 0.25, 0.0),
                    0.4,
                    0.35,
                )
                .expect("tapered cylinder"),
            ),
        ),
        ("convex hull", Shape::Convex(convex_cube())),
    ]
}

fn make_convex(shape: &Shape) -> Shape {
    match shape {
        Shape::Sphere(sphere) => Shape::Sphere(Sphere::new(sphere.radius()).expect("sphere")),
        Shape::Cuboid(cuboid) => {
            Shape::Cuboid(Cuboid::from_vec(cuboid.size()).expect("cuboid dimensions"))
        }
        Shape::Capsule(capsule) => Shape::Capsule(
            Capsule::new(capsule.start(), capsule.end(), capsule.radius()).expect("capsule"),
        ),
        Shape::Cylinder(cylinder) => Shape::Cylinder(
            Cylinder::new(cylinder.start(), cylinder.end(), cylinder.radius()).expect("cylinder"),
        ),
        Shape::TaperedCapsule(capsule) => Shape::TaperedCapsule(
            TaperedCapsule::new(
                capsule.start(),
                capsule.end(),
                capsule.start_radius(),
                capsule.end_radius(),
            )
            .expect("tapered capsule"),
        ),
        Shape::TaperedCylinder(cylinder) => Shape::TaperedCylinder(
            TaperedCylinder::new(
                cylinder.start(),
                cylinder.end(),
                cylinder.start_radius(),
                cylinder.end_radius(),
            )
            .expect("tapered cylinder"),
        ),
        Shape::Convex(_) => Shape::Convex(convex_cube()),
        _ => panic!("expected a convex fixture"),
    }
}

fn convex_cube() -> Convex {
    let vertices = vec![
        Vec3::new(-0.4, -0.4, -0.4),
        Vec3::new(0.4, -0.4, -0.4),
        Vec3::new(0.4, 0.4, -0.4),
        Vec3::new(-0.4, 0.4, -0.4),
        Vec3::new(-0.4, -0.4, 0.4),
        Vec3::new(0.4, -0.4, 0.4),
        Vec3::new(0.4, 0.4, 0.4),
        Vec3::new(-0.4, 0.4, 0.4),
    ];
    let faces = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [3, 7, 6],
        [3, 6, 2],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ];
    Convex::new(vertices, faces).expect("convex cube")
}

fn surface_shapes() -> Vec<Geometry> {
    let triangle_vertices = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
        Vec3::new(1.0, 2.0, 0.0),
    ];
    let triangle = Triangle::from_vertices(triangle_vertices).expect("triangle");
    let mesh = TriangleMesh::new(triangle_vertices.to_vec(), vec![[0, 1, 2]]).expect("mesh");
    let mesh_bvh = TriangleBvh::build(&mesh).expect("mesh BVH");
    let skinned_mesh = SkinnedTriangleMesh::new(
        TriangleMesh::new(triangle_vertices.to_vec(), vec![[0, 1, 2]]).expect("skin mesh"),
        vec![String::from("root")],
        (0..3)
            .map(|_| vec![BoneInfluence::new(0, 1.0).expect("bone influence")])
            .collect(),
        Affine3::IDENTITY,
        vec![Affine3::IDENTITY],
    )
    .expect("skinned mesh");
    let skin_bvh = TriangleBvh::build(skinned_mesh.mesh()).expect("skin BVH");
    let field = HeightField::from_vec(2, 2, Vec2::splat(2.0), vec![0.0; 4]).expect("heightfield");
    let mut distances = Vec::with_capacity(27);
    for z in 0..3 {
        for _y in 0..3 {
            for _x in 0..3 {
                distances.push(z as f64 - 1.0);
            }
        }
    }
    let level_set = LevelSet::from_vec(Vec3::new(0.0, 0.0, -1.0), [3, 3, 3], Vec3::ONE, distances)
        .expect("level set");
    let identity = Transform::identity();

    vec![
        Geometry {
            name: "plane",
            shape: Shape::Plane(Plane::new(0.0, 0.0, 1.0, 0.0).expect("plane")),
            transform: identity,
            bvh: None,
        },
        Geometry {
            name: "triangle",
            shape: Shape::Triangle(triangle),
            transform: identity,
            bvh: None,
        },
        Geometry {
            name: "triangle mesh",
            shape: Shape::TriangleMesh(mesh),
            transform: identity,
            bvh: Some(mesh_bvh),
        },
        Geometry {
            name: "heightfield",
            shape: Shape::HeightField(field),
            transform: identity,
            bvh: None,
        },
        Geometry {
            name: "level set",
            shape: Shape::LevelSet(level_set),
            transform: identity,
            bvh: None,
        },
        Geometry {
            name: "skinned triangle mesh",
            shape: Shape::SkinnedTriangleMesh(skinned_mesh),
            transform: identity,
            bvh: Some(skin_bvh),
        },
    ]
}

fn pair() -> CollisionPair {
    let mut world = PhysicsWorld::new();
    let body_a = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body A");
    let body_b = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body B");
    let collider = || {
        crate::collider::Collider::new(
            Shape::Sphere(Sphere::new(1.0).expect("sphere")),
            Transform::identity(),
        )
    };
    let collider_a = world.add_collider(body_a, collider()).expect("collider A");
    let collider_b = world.add_collider(body_b, collider()).expect("collider B");
    CollisionPair::new(collider_a, collider_b).expect("collision pair")
}

fn at(x: f64, y: f64, z: f64) -> Transform {
    Transform::new(Vec3::new(x, y, z), Quat::IDENTITY).expect("transform")
}
