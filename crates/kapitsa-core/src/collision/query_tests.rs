use crate::{
    collider::{Collider, CollisionMode},
    collision::{OverlapHit, QueryFilter, Ray},
    rigid_body::RigidBody,
    shape::{
        BoneInfluence, Capsule, Convex, Cuboid, Cylinder, HeightField, LevelSet, Plane, Shape,
        SkinnedTriangleMesh, Sphere, TaperedCapsule, TaperedCylinder, Triangle, TriangleMesh,
    },
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Affine3, Quat, Vec2, Vec3};

fn transform(position: Vec3) -> Transform {
    Transform::new(position, Quat::IDENTITY).expect("transform")
}

fn add_static_shape(
    world: &mut PhysicsWorld,
    shape: Shape,
    transform: Transform,
) -> crate::world::ColliderId {
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body");
    world
        .add_collider(body, Collider::new(shape, transform))
        .expect("collider")
}

#[test]
fn ray_constructor_normalizes_direction_and_rejects_bad_inputs() {
    let ray = Ray::new(Vec3::ZERO, Vec3::new(3.0, 0.0, 0.0), 5.0).expect("ray");
    assert_eq!(ray.direction, Vec3::X);
    assert_eq!(
        Ray::new(Vec3::ZERO, Vec3::ZERO, 1.0),
        Err(crate::collision::QueryError::InvalidInput)
    );
    assert_eq!(
        Ray::new(Vec3::ZERO, Vec3::X, f64::INFINITY),
        Err(crate::collision::QueryError::InvalidInput)
    );
}

#[test]
fn raycast_returns_nearest_hit_and_observes_filters() {
    let mut world = PhysicsWorld::new();
    let first_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body");
    let first = world
        .add_collider(
            first_body,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).expect("sphere")),
                transform(Vec3::new(3.0, 0.0, 0.0)),
            ),
        )
        .expect("collider");
    let second_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .expect("body");
    let mut second = Collider::new(
        Shape::Sphere(Sphere::new(1.0).expect("sphere")),
        transform(Vec3::new(6.0, 0.0, 0.0)),
    );
    second.set_filter(crate::collider::CollisionFilter::new(2, u32::MAX));
    let second = world.add_collider(second_body, second).expect("collider");
    let ray = Ray::new(Vec3::ZERO, Vec3::X, 10.0).expect("ray");
    let hit = world
        .raycast(ray, QueryFilter::default())
        .expect("raycast")
        .expect("hit");
    assert_eq!(hit.collider, first);
    assert!((hit.distance - 2.0).abs() < 1.0e-10);

    let filter = QueryFilter {
        layers: 2,
        mask: 2,
        ..QueryFilter::default()
    };
    let hit = world
        .raycast(ray, filter)
        .expect("filtered raycast")
        .expect("hit");
    assert_eq!(hit.collider, second);
    assert!((hit.distance - 5.0).abs() < 1.0e-10);
}

#[test]
fn raycast_handles_all_convex_shape_categories() {
    let tetra = Convex::new(
        vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z],
        vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
    )
    .expect("tetrahedron");
    let shapes = [
        Shape::Sphere(Sphere::new(1.0).expect("sphere")),
        Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).expect("cuboid")),
        Shape::Capsule(Capsule::new(-Vec3::Z, Vec3::Z, 0.5).expect("capsule")),
        Shape::Cylinder(Cylinder::new(-Vec3::Z, Vec3::Z, 0.5).expect("cylinder")),
        Shape::TaperedCapsule(
            TaperedCapsule::new(-Vec3::Z, Vec3::Z, 0.25, 0.5).expect("tapered capsule"),
        ),
        Shape::TaperedCylinder(
            TaperedCylinder::new(-Vec3::Z, Vec3::Z, 0.25, 0.5).expect("tapered cylinder"),
        ),
        Shape::Convex(tetra),
    ];
    let rays = [
        Ray::new(-Vec3::X * 5.0, Vec3::X, 10.0).unwrap(),
        Ray::new(-Vec3::X * 5.0, Vec3::X, 10.0).unwrap(),
        Ray::new(-Vec3::X * 5.0, Vec3::X, 10.0).unwrap(),
        Ray::new(-Vec3::X * 5.0, Vec3::X, 10.0).unwrap(),
        Ray::new(-Vec3::X * 5.0, Vec3::X, 10.0).unwrap(),
        Ray::new(-Vec3::X * 5.0, Vec3::X, 10.0).unwrap(),
        Ray::new(Vec3::new(-1.0, 0.1, 0.1), Vec3::X, 5.0).unwrap(),
    ];
    for (shape, ray) in shapes.into_iter().zip(rays) {
        let mut world = PhysicsWorld::new();
        add_static_shape(&mut world, shape, Transform::identity());
        assert!(
            world
                .raycast(ray, QueryFilter::default())
                .unwrap()
                .is_some()
        );
    }
}

#[test]
fn raycast_handles_plane_triangle_mesh_heightfield_levelset_and_skin() {
    let ray_down = Ray::new(Vec3::new(0.0, 0.0, 5.0), -Vec3::Z, 10.0).unwrap();
    let triangle_vertices = [
        Vec3::new(-2.0, -2.0, 0.0),
        Vec3::new(2.0, -2.0, 0.0),
        Vec3::new(0.0, 2.0, 0.0),
    ];
    let mesh = TriangleMesh::new(triangle_vertices.to_vec(), vec![[0, 1, 2]]).unwrap();
    let skinned_mesh = SkinnedTriangleMesh::new(
        TriangleMesh::new(triangle_vertices.to_vec(), vec![[0, 1, 2]]).unwrap(),
        vec![String::from("root")],
        (0..3)
            .map(|_| vec![BoneInfluence::new(0, 1.0).unwrap()])
            .collect(),
        Affine3::IDENTITY,
        vec![Affine3::IDENTITY],
    )
    .unwrap();
    let cases = [
        Shape::Plane(Plane::from_vec(Vec3::Z, 0.0).unwrap()),
        Shape::Triangle(Triangle::from_vertices(triangle_vertices).unwrap()),
        Shape::TriangleMesh(mesh),
        Shape::HeightField(HeightField::from_vec(2, 2, Vec2::new(4.0, 4.0), vec![0.0; 4]).unwrap()),
        Shape::LevelSet(
            LevelSet::from_vec(
                Vec3::new(-2.0, -2.0, -1.0),
                [2, 2, 2],
                Vec3::new(4.0, 4.0, 2.0),
                vec![-1.0; 4].into_iter().chain([1.0; 4]).collect(),
            )
            .unwrap(),
        ),
        Shape::SkinnedTriangleMesh(skinned_mesh),
    ];
    for shape in cases {
        let mut world = PhysicsWorld::new();
        add_static_shape(&mut world, shape, Transform::identity());
        let hit = world
            .raycast(ray_down, QueryFilter::default())
            .expect("raycast")
            .expect("hit");
        assert!((hit.distance - 5.0).abs() < 1.0e-10);
    }
}

#[test]
fn raycast_respects_sensor_and_excluded_body_options() {
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let mut collider = Collider::new(
        Shape::Sphere(Sphere::new(1.0).unwrap()),
        transform(Vec3::new(3.0, 0.0, 0.0)),
    );
    collider.set_mode(CollisionMode::Sensor);
    world.add_collider(body, collider).unwrap();
    let ray = Ray::new(Vec3::ZERO, Vec3::X, 10.0).unwrap();
    assert!(
        world
            .raycast(ray, QueryFilter::default())
            .unwrap()
            .is_none()
    );
    let filter = QueryFilter {
        include_sensors: true,
        ..QueryFilter::default()
    };
    assert!(world.raycast(ray, filter).unwrap().is_some());
    let filter = QueryFilter {
        include_sensors: true,
        exclude_body: Some(body),
        ..QueryFilter::default()
    };
    assert!(world.raycast(ray, filter).unwrap().is_none());
}

#[test]
fn raycast_uses_mutated_body_transform_without_syncing_the_spatial_index() {
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    world
        .add_collider(
            body,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                Transform::identity(),
            ),
        )
        .unwrap();
    world
        .body_mut(body)
        .unwrap()
        .set_transform(transform(Vec3::new(5.0, 0.0, 0.0)))
        .unwrap();
    let ray = Ray::new(Vec3::ZERO, Vec3::X, 10.0).unwrap();
    let hit = world.raycast(ray, QueryFilter::default()).unwrap().unwrap();
    assert!((hit.distance - 4.0).abs() < 1.0e-10);
}

#[test]
fn sweep_returns_first_impact_and_supports_surface_geometry() {
    let query = Shape::Sphere(Sphere::new(0.5).unwrap());
    let start = transform(Vec3::new(0.0, 0.0, 5.0));
    let displacement = -Vec3::Z * 10.0;
    let triangle = [
        Vec3::new(-5.0, -5.0, 0.0),
        Vec3::new(5.0, -5.0, 0.0),
        Vec3::new(0.0, 5.0, 0.0),
    ];
    let mesh = TriangleMesh::new(triangle.to_vec(), vec![[0, 1, 2]]).unwrap();
    let level_set = LevelSet::from_vec(
        Vec3::new(-5.0, -5.0, -5.0),
        [2, 2, 2],
        Vec3::splat(10.0),
        vec![-1.0; 4].into_iter().chain([1.0; 4]).collect(),
    )
    .unwrap();
    let skinned_mesh = SkinnedTriangleMesh::new(
        TriangleMesh::new(triangle.to_vec(), vec![[0, 1, 2]]).unwrap(),
        vec![String::from("root")],
        (0..3)
            .map(|_| vec![BoneInfluence::new(0, 1.0).unwrap()])
            .collect(),
        Affine3::IDENTITY,
        vec![Affine3::IDENTITY],
    )
    .unwrap();
    let targets = [
        Shape::Plane(Plane::from_vec(Vec3::Z, 0.0).unwrap()),
        Shape::Triangle(Triangle::from_vertices(triangle).unwrap()),
        Shape::TriangleMesh(mesh),
        Shape::HeightField(
            HeightField::from_vec(2, 2, Vec2::new(10.0, 10.0), vec![0.0; 4]).unwrap(),
        ),
        Shape::LevelSet(level_set),
        Shape::SkinnedTriangleMesh(skinned_mesh),
    ];
    for target in targets {
        let mut world = PhysicsWorld::new();
        add_static_shape(&mut world, target, Transform::identity());
        let hit = world
            .sweep(&query, start, displacement, QueryFilter::default())
            .unwrap()
            .expect("impact");
        assert!((hit.toi - 0.45).abs() < 1.0e-9, "{hit:?}");
    }

    let mut world = PhysicsWorld::new();
    let first_body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let first = world
        .add_collider(
            first_body,
            Collider::new(
                Shape::Sphere(Sphere::new(1.0).unwrap()),
                transform(Vec3::new(4.0, 0.0, 0.0)),
            ),
        )
        .unwrap();
    add_static_shape(
        &mut world,
        Shape::Sphere(Sphere::new(1.0).unwrap()),
        transform(Vec3::new(7.0, 0.0, 0.0)),
    );
    let hit = world
        .sweep(
            &Shape::Sphere(Sphere::new(0.5).unwrap()),
            Transform::identity(),
            Vec3::new(10.0, 0.0, 0.0),
            QueryFilter::default(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.collider, first);
    assert!((hit.toi - 0.25).abs() < 1.0e-9);
}

#[test]
fn sweep_reports_initial_overlap_and_rejects_nonconvex_query_shapes() {
    let mut world = PhysicsWorld::new();
    add_static_shape(
        &mut world,
        Shape::Sphere(Sphere::new(1.0).unwrap()),
        Transform::identity(),
    );
    let sphere = Shape::Sphere(Sphere::new(0.5).unwrap());
    let hit = world
        .sweep(
            &sphere,
            Transform::identity(),
            Vec3::X,
            QueryFilter::default(),
        )
        .unwrap()
        .unwrap();
    assert_eq!(hit.toi, 0.0);
    let triangle =
        Shape::Triangle(Triangle::from_vertices([Vec3::ZERO, Vec3::X, Vec3::Y]).unwrap());
    assert_eq!(
        world.sweep(
            &triangle,
            Transform::identity(),
            Vec3::X,
            QueryFilter::default(),
        ),
        Err(crate::collision::QueryError::UnsupportedQueryShape)
    );
    let empty_world = PhysicsWorld::new();
    assert_eq!(
        empty_world.sweep(
            &triangle,
            Transform::identity(),
            Vec3::X,
            QueryFilter::default(),
        ),
        Err(crate::collision::QueryError::UnsupportedQueryShape)
    );
    assert_eq!(
        empty_world.overlaps(
            &triangle,
            Transform::identity(),
            QueryFilter::default(),
            &mut Vec::new(),
        ),
        Err(crate::collision::QueryError::UnsupportedQueryShape)
    );
}

#[test]
fn overlaps_detects_all_shape_categories_and_preserves_output_on_error() {
    let tetra = Convex::new(
        vec![Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z],
        vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
    )
    .unwrap();
    let vertices = [
        Vec3::new(-2.0, -2.0, 0.0),
        Vec3::new(2.0, -2.0, 0.0),
        Vec3::new(0.0, 2.0, 0.0),
    ];
    let skin = SkinnedTriangleMesh::new(
        TriangleMesh::new(vertices.to_vec(), vec![[0, 1, 2]]).unwrap(),
        vec![String::from("root")],
        (0..3)
            .map(|_| vec![BoneInfluence::new(0, 1.0).unwrap()])
            .collect(),
        Affine3::IDENTITY,
        vec![Affine3::IDENTITY],
    )
    .unwrap();
    let shapes = [
        Shape::Sphere(Sphere::new(1.0).unwrap()),
        Shape::Cuboid(Cuboid::new(2.0, 2.0, 2.0).unwrap()),
        Shape::Plane(Plane::from_vec(Vec3::Z, 0.0).unwrap()),
        Shape::Capsule(Capsule::new(-Vec3::Z, Vec3::Z, 0.5).unwrap()),
        Shape::Cylinder(Cylinder::new(-Vec3::Z, Vec3::Z, 0.5).unwrap()),
        Shape::TaperedCapsule(TaperedCapsule::new(-Vec3::Z, Vec3::Z, 0.25, 0.5).unwrap()),
        Shape::TaperedCylinder(TaperedCylinder::new(-Vec3::Z, Vec3::Z, 0.25, 0.5).unwrap()),
        Shape::Triangle(Triangle::from_vertices(vertices).unwrap()),
        Shape::Convex(tetra),
        Shape::TriangleMesh(TriangleMesh::new(vertices.to_vec(), vec![[0, 1, 2]]).unwrap()),
        Shape::HeightField(HeightField::from_vec(2, 2, Vec2::new(4.0, 4.0), vec![0.0; 4]).unwrap()),
        Shape::LevelSet(
            LevelSet::from_vec(
                Vec3::new(-2.0, -2.0, -1.0),
                [2, 2, 2],
                Vec3::new(4.0, 4.0, 2.0),
                vec![-1.0; 4].into_iter().chain([1.0; 4]).collect(),
            )
            .unwrap(),
        ),
        Shape::SkinnedTriangleMesh(skin),
    ];
    for target in shapes {
        let mut world = PhysicsWorld::new();
        let id = add_static_shape(&mut world, target, Transform::identity());
        let mut output = Vec::new();
        world
            .overlaps(
                &Shape::Sphere(Sphere::new(0.5).unwrap()),
                Transform::identity(),
                QueryFilter::default(),
                &mut output,
            )
            .unwrap();
        assert_eq!(output.len(), 1);
        assert_eq!(output[0].collider, id);

        let nonconvex = Shape::Plane(Plane::from_vec(Vec3::Z, 0.0).unwrap());
        assert_eq!(
            world.overlaps(
                &nonconvex,
                Transform::identity(),
                QueryFilter::default(),
                &mut output,
            ),
            Err(crate::collision::QueryError::UnsupportedQueryShape)
        );
        assert_eq!(output.len(), 1);
    }
}

#[test]
fn overlaps_respects_sensors_and_query_filters() {
    let mut world = PhysicsWorld::new();
    let body = world
        .add_body(RigidBody::new_static(Transform::identity()))
        .unwrap();
    let mut collider = Collider::new(
        Shape::Sphere(Sphere::new(1.0).unwrap()),
        Transform::identity(),
    );
    collider.set_mode(CollisionMode::Sensor);
    collider.set_filter(crate::collider::CollisionFilter::new(2, 2));
    world.add_collider(body, collider).unwrap();
    let query = Shape::Sphere(Sphere::new(0.5).unwrap());
    let mut output = vec![OverlapHit {
        body,
        collider: world.colliders().next().unwrap().0,
    }];
    world
        .overlaps(
            &query,
            Transform::identity(),
            QueryFilter::default(),
            &mut output,
        )
        .unwrap();
    assert!(output.is_empty());
    world
        .overlaps(
            &query,
            Transform::identity(),
            QueryFilter {
                layers: 2,
                mask: 2,
                include_sensors: true,
                ..QueryFilter::default()
            },
            &mut output,
        )
        .unwrap();
    assert_eq!(output.len(), 1);
    assert_eq!(output[0].body, body);
    world
        .overlaps(
            &query,
            Transform::identity(),
            QueryFilter {
                layers: 2,
                mask: 2,
                include_sensors: true,
                exclude_body: Some(body),
            },
            &mut output,
        )
        .unwrap();
    assert!(output.is_empty());
}
