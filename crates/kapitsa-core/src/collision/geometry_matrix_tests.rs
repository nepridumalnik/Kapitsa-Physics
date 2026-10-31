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

fn assert_symmetric_contact(pair: CollisionPair, left: &Geometry, right: &Geometry) {
    let mut forward_narrow_phase = NarrowPhase::new();
    let forward = detect(&mut forward_narrow_phase, pair, left, right)
        .unwrap_or_else(|error| panic!("{} vs {}: {error}", left.name, right.name))
        .unwrap_or_else(|| panic!("{} vs {}: expected contact", left.name, right.name));
    let mut reverse_narrow_phase = NarrowPhase::new();
    let reverse = detect(&mut reverse_narrow_phase, pair, right, left)
        .unwrap_or_else(|error| panic!("{} vs {} reversed: {error}", left.name, right.name))
        .unwrap_or_else(|| panic!("{} vs {} reversed: expected contact", left.name, right.name));
    let normal_forward = forward.points().next().expect("contact point").normal;
    let normal_reverse = reverse.points().next().expect("contact point").normal;
    assert!(
        (normal_forward + normal_reverse).length() <= 1.0e-6,
        "{} vs {}: normals did not reverse ({normal_forward:?}, {normal_reverse:?})",
        left.name,
        right.name
    );
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
