use crate::{
    collision::{
        ccd::{
            CcdMotion, convex_toi, height_field_toi, level_set_toi, plane_toi, plane_toi_motion,
            swept_bounds, triangle_mesh_toi,
        },
        height_field::HeightFieldTriangle,
        mesh::{MeshQueryWorkspace, TriangleBvh},
    },
    mass::MassProperties,
    rigid_body::{RigidBody, predict, predict_from_state, trajectory_bounds},
    shape::{Cuboid, HeightField, LevelSet, Plane, Shape, Sphere, TriangleMesh},
    transform::Transform,
};
use kapitsa_math::{Quat, Vec2, Vec3};

fn transform(position: Vec3, rotation: Quat) -> Transform {
    Transform::new(position, rotation).expect("valid transform")
}

#[test]
fn convex_toi_finds_fast_translation_impact() {
    let sphere = Shape::Sphere(Sphere::new(1.0).expect("sphere"));
    let start = transform(Vec3::new(5.0, 0.0, 0.0), Quat::IDENTITY);
    let end = transform(Vec3::ZERO, Quat::IDENTITY);
    let result = convex_toi(
        &sphere,
        Transform::identity(),
        Transform::identity(),
        &sphere,
        start,
        end,
    )
    .expect("TOI")
    .expect("impact");
    assert!((result.toi - 0.6).abs() < 1.0e-10);
    assert!((result.normal - Vec3::X).length() < 1.0e-10);
}

#[test]
fn convex_toi_finds_fast_sphere_impact_against_a_thin_cuboid() {
    let sphere = Shape::Sphere(Sphere::new(0.5).expect("sphere"));
    let cuboid = Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0).expect("cuboid"));
    let start = transform(Vec3::new(-5.0, 0.0, 0.0), Quat::IDENTITY);
    let end = transform(Vec3::new(5.0, 0.0, 0.0), Quat::IDENTITY);
    let hit = convex_toi(
        &sphere,
        start,
        end,
        &cuboid,
        Transform::identity(),
        Transform::identity(),
    )
    .expect("TOI")
    .expect("impact");
    assert!(hit.toi > 0.0 && hit.toi < 1.0);
}

#[test]
fn convex_toi_uses_relative_motion_for_a_near_miss_after_shared_translation() {
    let cuboid = Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).expect("cuboid"));
    let start_a = Transform::identity();
    let end_a = transform(Vec3::new(100.0, 0.0, 0.0), Quat::IDENTITY);
    let start_b = transform(Vec3::new(0.0, 1.1, 0.0), Quat::IDENTITY);
    let end_b = transform(Vec3::new(100.0, 1.01, 0.0), Quat::IDENTITY);

    assert!(
        convex_toi(&cuboid, start_a, end_a, &cuboid, start_b, end_b)
            .expect("relative TOI")
            .is_none()
    );
}

#[test]
fn convex_toi_accounts_for_rotation_without_endpoint_overlap() {
    let rod = Shape::Cuboid(Cuboid::new(4.0, 0.2, 0.2).expect("cuboid"));
    let sphere = Shape::Sphere(Sphere::new(0.2).expect("sphere"));
    let start = Transform::identity();
    let end = transform(
        Vec3::ZERO,
        Quat::from_axis_angle(Vec3::Z, std::f64::consts::PI).expect("rotation"),
    );
    let target = transform(Vec3::new(0.0, 1.5, 0.0), Quat::IDENTITY);
    assert!(
        convex_toi(&rod, start, start, &sphere, target, target)
            .expect("start query")
            .is_none()
    );
    let result = convex_toi(&rod, start, end, &sphere, target, target)
        .expect("rotational TOI")
        .expect("impact");
    assert!(result.toi > 0.0 && result.toi < 1.0);
}

#[test]
fn plane_toi_is_analytic_for_linear_sphere_motion() {
    let sphere = Shape::Sphere(Sphere::new(1.0).expect("sphere"));
    let plane = Plane::from_vec(Vec3::Z, 0.0).expect("plane");
    let start = transform(Vec3::new(0.0, 0.0, 5.0), Quat::IDENTITY);
    let end = transform(Vec3::new(0.0, 0.0, -5.0), Quat::IDENTITY);
    let result = plane_toi(
        &sphere,
        start,
        end,
        &plane,
        Transform::identity(),
        Transform::identity(),
        false,
        true,
    )
    .expect("plane TOI")
    .expect("impact");
    assert!((result.toi - 0.4).abs() < 1.0e-12);
    assert_eq!(result.normal, Vec3::Z);
}

#[test]
fn triangle_mesh_and_height_field_toi_search_local_surface_geometry() {
    let sphere = Shape::Sphere(Sphere::new(0.5).expect("sphere"));
    let start = transform(Vec3::new(0.0, 0.0, 5.0), Quat::IDENTITY);
    let end = transform(Vec3::new(0.0, 0.0, -5.0), Quat::IDENTITY);
    let vertices = vec![
        Vec3::new(-5.0, -5.0, 0.0),
        Vec3::new(5.0, -5.0, 0.0),
        Vec3::new(0.0, 5.0, 0.0),
    ];
    let mesh = TriangleMesh::new(vertices, vec![[0, 1, 2]]).expect("mesh");
    let bvh = TriangleBvh::build(&mesh).expect("BVH");
    let mut workspace = MeshQueryWorkspace::default();
    let mut triangle_ids = Vec::new();
    let mesh_hit = triangle_mesh_toi(
        &sphere,
        start,
        end,
        &mesh,
        None,
        &bvh,
        Transform::identity(),
        Transform::identity(),
        &mut workspace,
        &mut triangle_ids,
    )
    .expect("mesh TOI")
    .expect("mesh impact");
    assert!((mesh_hit.toi - 0.45).abs() < 1.0e-10);

    let field =
        HeightField::from_vec(2, 2, Vec2::new(10.0, 10.0), vec![0.0; 4]).expect("height field");
    let mut cell_triangles: Vec<HeightFieldTriangle> = Vec::new();
    let field_hit = height_field_toi(
        &sphere,
        start,
        end,
        &field,
        Transform::identity(),
        Transform::identity(),
        &mut cell_triangles,
    )
    .expect("height field TOI")
    .expect("height field impact");
    assert!((field_hit.toi - 0.45).abs() < 1.0e-10);
}

#[test]
fn level_set_toi_uses_interpolation_and_a_conservative_gradient_bound() {
    let sphere = Shape::Sphere(Sphere::new(0.5).expect("sphere"));
    let start = transform(Vec3::new(0.0, 0.0, 15.0), Quat::IDENTITY);
    let end = transform(Vec3::new(0.0, 0.0, -5.0), Quat::IDENTITY);
    let field = LevelSet::from_vec(
        Vec3::new(-5.0, -5.0, 0.0),
        [2, 2, 2],
        Vec3::splat(10.0),
        vec![-50.0; 4].into_iter().chain([50.0; 4]).collect(),
    )
    .expect("level set");
    let mut cells = Vec::new();
    let result = level_set_toi(
        &sphere,
        start,
        end,
        &field,
        Transform::identity(),
        Transform::identity(),
        &mut cells,
    )
    .expect("level-set TOI")
    .expect("impact");
    assert!((result.toi - 0.475).abs() < 1.0e-10);
    assert!((result.normal + Vec3::Z).length() < 1.0e-10);
    assert!((result.point_b.z - 5.0).abs() < 1.0e-12);
}

#[test]
fn skinned_mesh_toi_uses_the_current_deformed_vertices() {
    let sphere = Shape::Sphere(Sphere::new(0.5).expect("sphere"));
    let start = transform(Vec3::new(0.0, 0.0, 5.0), Quat::IDENTITY);
    let end = transform(Vec3::new(0.0, 0.0, -5.0), Quat::IDENTITY);
    let vertices = vec![
        Vec3::new(-5.0, -5.0, 0.0),
        Vec3::new(5.0, -5.0, 0.0),
        Vec3::new(0.0, 5.0, 0.0),
    ];
    let mesh = TriangleMesh::new(vertices, vec![[0, 1, 2]]).expect("mesh");
    let mut bvh = TriangleBvh::build(&mesh).expect("BVH");
    let deformed = mesh
        .vertices()
        .iter()
        .map(|vertex| *vertex + Vec3::Z)
        .collect::<Vec<_>>();
    bvh.refit_vertices(&mesh, &deformed).expect("refit");
    let mut workspace = MeshQueryWorkspace::default();
    let mut triangle_ids = Vec::new();
    let result = triangle_mesh_toi(
        &sphere,
        start,
        end,
        &mesh,
        Some(&deformed),
        &bvh,
        Transform::identity(),
        Transform::identity(),
        &mut workspace,
        &mut triangle_ids,
    )
    .expect("skinned TOI")
    .expect("impact");
    assert!((result.toi - 0.35).abs() < 1.0e-10);
}

#[test]
fn swept_bounds_cover_all_intermediate_rotations() {
    let rod = Shape::Cuboid(Cuboid::new(4.0, 0.2, 0.2).expect("cuboid"));
    let start = Transform::identity();
    let end = transform(
        Vec3::new(3.0, 0.0, 0.0),
        Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2).expect("rotation"),
    );
    let bounds = swept_bounds(&rod, start, end)
        .expect("swept bounds")
        .expect("finite bounds");
    assert!(bounds.contains(Vec3::new(1.5, 1.5, 0.0)));
    assert!(bounds.contains(Vec3::new(3.0, 2.0, 0.0)));
}

#[test]
fn rotational_ccd_follows_integrator_samples_across_multiple_turns() {
    let rod = Shape::Cuboid(Cuboid::new(4.0, 0.2, 0.2).expect("cuboid"));
    let plane = Plane::from_vec(Vec3::Y, 0.0).expect("plane");
    let target = transform(Vec3::new(0.0, 1.5, 0.0), Quat::IDENTITY);
    let target_motion = CcdMotion::linear(target, target).expect("stationary target");
    let mut full_turn_toi = None;

    for angle in [
        std::f64::consts::FRAC_PI_2,
        3.0 * std::f64::consts::FRAC_PI_2,
        std::f64::consts::TAU,
        2.0 * std::f64::consts::TAU,
    ] {
        let mass = MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY)
            .expect("mass properties");
        let mut body = RigidBody::new_dynamic(Transform::identity(), mass).expect("body");
        body.set_angular_velocity(Vec3::Z * (angle - 1.0))
            .expect("finite angular velocity");
        body.apply_torque(Vec3::Z).expect("finite torque");
        let start_state = *body.state();
        let end_state = predict(&body, 1.0).expect("integrator endpoint");
        let bounds =
            trajectory_bounds(&body, start_state, end_state, 1.0).expect("trajectory bounds");
        let sample = |amount: f64| {
            if amount == 0.0 {
                return Ok(start_state.transform());
            }
            predict_from_state(&body, start_state, amount)
                .map(|state| state.transform())
                .map_err(|_| crate::collision::CollisionError::ArithmeticOverflow)
        };
        let motion = CcdMotion::trajectory(
            start_state.transform(),
            end_state.transform(),
            bounds.translation,
            true,
            bounds.rotation,
            &sample,
        )
        .expect("trajectory motion");
        let hit = plane_toi_motion(&rod, &motion, &plane, &target_motion, true, false)
            .unwrap_or_else(|error| panic!("rotational TOI failed at angle {angle}: {error:?}"))
            .expect("rotation crosses target");

        const SAMPLE_COUNT: usize = 20_000;
        let sample_toi = (1..=SAMPLE_COUNT)
            .find_map(|index| {
                let amount = index as f64 / SAMPLE_COUNT as f64;
                let transform = sample(amount).expect("integrator sample");
                let gap =
                    super::plane_gap(&rod, transform, &plane, target, true).expect("plane sample");
                (gap.separation <= 1.0e-8).then_some(amount)
            })
            .expect("sampled collision");
        assert!((hit.toi - sample_toi).abs() <= 2.0 / SAMPLE_COUNT as f64);
        assert_eq!(
            motion.at(hit.toi).expect("trajectory sample"),
            sample(hit.toi).unwrap()
        );

        if angle == std::f64::consts::TAU {
            full_turn_toi = Some(hit.toi);
        } else if angle == 2.0 * std::f64::consts::TAU {
            assert!(hit.toi < full_turn_toi.expect("one-turn TOI") * 0.5);
            let endpoint_dot = end_state.transform().rotation().x * Quat::IDENTITY.x
                + end_state.transform().rotation().y * Quat::IDENTITY.y
                + end_state.transform().rotation().z * Quat::IDENTITY.z
                + end_state.transform().rotation().w * Quat::IDENTITY.w;
            assert!(endpoint_dot.abs() > 1.0 - 1.0e-12);
        }
    }
}
