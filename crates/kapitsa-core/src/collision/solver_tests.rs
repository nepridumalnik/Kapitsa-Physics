use crate::{
    collider::Collider,
    collision::{
        CollisionPair,
        cache::ContactCache,
        manifold::{ContactManifold, ContactPoint},
        solver::{SolverBody, SolverManifold, SolverMaterial, SolverWorkspace},
    },
    mass::MassProperties,
    rigid_body::RigidBody,
    shape::{Shape, Sphere},
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Vec3};

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

fn dynamic_body(translation: Vec3) -> RigidBody {
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).unwrap();
    RigidBody::new_dynamic(
        Transform::new(translation, kapitsa_math::Quat::IDENTITY).unwrap(),
        mass,
    )
    .unwrap()
}

fn ball_on_plane(linear_velocity: Vec3) -> (Vec<SolverBody>, Vec<SolverManifold>) {
    let ground = RigidBody::new_static(Transform::identity());
    let mut ball = dynamic_body(Vec3::new(0.0, 0.4, 0.0));
    ball.set_linear_velocity(linear_velocity).unwrap();
    let ground_state = *ground.state();
    let ball_state = *ball.state();
    let point_a = Vec3::ZERO;
    let point_b = Vec3::new(0.0, -0.1, 0.0);
    let pair = pair();
    let contact = ContactPoint::from_world(
        &Transform::identity(),
        &ball.state().transform(),
        point_a,
        point_b,
        Vec3::Y,
        0.1,
        1,
    )
    .unwrap();
    let mut manifold = ContactManifold::new(pair);
    manifold
        .insert(contact, point_a * 0.5 + point_b * 0.5)
        .unwrap();
    (
        vec![
            SolverBody::from_prediction(ground, ground_state),
            SolverBody::from_prediction(ball, ball_state),
        ],
        vec![SolverManifold {
            body_a: 0,
            body_b: 1,
            manifold,
            material: SolverMaterial {
                restitution: 0.5,
                static_friction: 0.0,
                dynamic_friction: 0.0,
            },
        }],
    )
}

#[test]
fn normal_impulses_apply_restitution_and_separate_position_correction() {
    let (mut bodies, mut manifolds) = ball_on_plane(-Vec3::Y * 2.0);
    let ground_transform = bodies[0].body().state().transform();
    let mut solver = SolverWorkspace::default();
    solver
        .solve(
            &mut bodies,
            &mut manifolds,
            super::CollisionConfig::default(),
        )
        .unwrap();

    let ball = bodies[1].body();
    assert!((ball.state().linear_velocity().y - 1.0).abs() < 1.0e-10);
    assert!(ball.state().transform().translation().y > 0.4);
    assert_eq!(bodies[0].body().state().transform(), ground_transform);
    assert!(
        manifolds[0]
            .manifold
            .points()
            .next()
            .unwrap()
            .normal_impulse
            > 0.0
    );
}

#[test]
fn friction_limits_tangent_impulses_and_creates_angular_response() {
    let (mut bodies, mut manifolds) = ball_on_plane(Vec3::new(10.0, -1.0, 0.0));
    manifolds[0].material.static_friction = 0.4;
    manifolds[0].material.dynamic_friction = 0.3;
    let mut solver = SolverWorkspace::default();
    solver
        .solve(
            &mut bodies,
            &mut manifolds,
            super::CollisionConfig::default(),
        )
        .unwrap();

    let ball = bodies[1].body();
    assert!(ball.state().linear_velocity().x < 10.0);
    assert!(ball.state().angular_velocity().z.abs() > 0.0);
    let contact = manifolds[0].manifold.points().next().unwrap();
    let friction_limit = 0.3 * contact.normal_impulse;
    assert!(
        contact.tangent_impulse[0].hypot(contact.tangent_impulse[1]) <= friction_limit + 1.0e-12
    );
}

#[test]
fn rotated_contact_warm_start_does_not_increase_energy_vs_cold_start() {
    fn solve(warm_start: bool) -> (f64, f64) {
        let pair = pair();
        let mut previous = ContactManifold::new(pair);
        let mut cached_contact = ContactPoint::from_world(
            &Transform::identity(),
            &Transform::identity(),
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::Y,
            0.0,
            5,
        )
        .unwrap();
        cached_contact.normal_impulse = 0.1;
        cached_contact.tangent_impulse = [0.05, 0.0];
        previous.insert(cached_contact, Vec3::ZERO).unwrap();
        let mut cache = ContactCache::new();
        cache.stage(&[previous]).unwrap();
        cache.commit();

        let normal = kapitsa_math::Quat::from_axis_angle(Vec3::Z, 0.1)
            .unwrap()
            .rotate_vector(Vec3::Y)
            .unwrap();
        let body_transform = Transform::new(normal * 0.45, kapitsa_math::Quat::IDENTITY).unwrap();
        let mut ball = dynamic_body(body_transform.translation());
        ball.set_linear_velocity(Vec3::Z - normal * 0.1).unwrap();
        let point_a = Vec3::ZERO;
        let point_b = body_transform.translation() - normal * 0.5;
        let contact = ContactPoint::from_world(
            &Transform::identity(),
            &body_transform,
            point_a,
            point_b,
            normal,
            0.05,
            5,
        )
        .unwrap();
        let mut manifold = ContactManifold::new(pair);
        manifold.insert(contact, point_b).unwrap();
        let ground = RigidBody::new_static(Transform::identity());
        let ball_state = *ball.state();
        let mut bodies = vec![
            SolverBody::from_prediction(ground, *ground.state()),
            SolverBody::from_prediction(ball, ball_state),
        ];
        let mut manifolds = vec![SolverManifold {
            body_a: 0,
            body_b: 1,
            manifold,
            material: SolverMaterial {
                restitution: 0.0,
                static_friction: 0.6,
                dynamic_friction: 0.6,
            },
        }];
        if warm_start {
            cache.warm_start_solver(&mut manifolds).unwrap();
        }
        let initial_energy = kinetic_energy(bodies[1].body());
        SolverWorkspace::default()
            .solve(
                &mut bodies,
                &mut manifolds,
                super::CollisionConfig::default(),
            )
            .unwrap();
        (initial_energy, kinetic_energy(bodies[1].body()))
    }

    let (warm_initial, warm_final) = solve(true);
    let (cold_initial, cold_final) = solve(false);
    assert!((warm_initial - cold_initial).abs() < 1.0e-12);
    assert!(warm_final <= warm_initial + 1.0e-8);
    assert!(cold_final <= cold_initial + 1.0e-8);
    assert!(warm_final <= cold_final + 1.0e-5);
}

fn kinetic_energy(body: &RigidBody) -> f64 {
    let mass = body.physics().mass().unwrap();
    let state = body.state();
    let rotation = state.transform().rotation().to_mat3().unwrap();
    let inertia_world = rotation * mass.inertia() * rotation.transpose();
    0.5 * mass.mass() * state.linear_velocity().length_squared()
        + 0.5
            * state
                .angular_velocity()
                .dot(inertia_world * state.angular_velocity())
}

#[test]
fn invalid_contact_material_leaves_solver_bodies_unchanged() {
    let (mut bodies, mut manifolds) = ball_on_plane(-Vec3::Y);
    manifolds[0].material.static_friction = f64::NAN;
    let initial = bodies[1].body().state().linear_velocity();
    let mut solver = SolverWorkspace::default();
    assert_eq!(
        solver.solve(
            &mut bodies,
            &mut manifolds,
            super::CollisionConfig::default()
        ),
        Err(crate::collision::CollisionError::InvalidMaterial)
    );
    assert_eq!(bodies[1].body().state().linear_velocity(), initial);
}

#[test]
fn kinematic_velocity_contributes_without_kinematic_reaction() {
    let mut surface = RigidBody::new_kinematic(Transform::identity());
    surface.set_linear_velocity(Vec3::Y).unwrap();
    let ball = dynamic_body(Vec3::new(0.0, 0.4, 0.0));
    let surface_state = *surface.state();
    let ball_state = *ball.state();
    let point_a = Vec3::ZERO;
    let point_b = Vec3::new(0.0, -0.1, 0.0);
    let contact = ContactPoint::from_world(
        &surface_state.transform(),
        &ball_state.transform(),
        point_a,
        point_b,
        Vec3::Y,
        0.1,
        2,
    )
    .unwrap();
    let mut manifold = ContactManifold::new(pair());
    manifold.insert(contact, point_a).unwrap();
    let mut bodies = vec![
        SolverBody::from_prediction(surface, surface_state),
        SolverBody::from_prediction(ball, ball_state),
    ];
    let mut manifolds = vec![SolverManifold {
        body_a: 0,
        body_b: 1,
        manifold,
        material: SolverMaterial {
            restitution: 0.0,
            static_friction: 0.0,
            dynamic_friction: 0.0,
        },
    }];
    SolverWorkspace::default()
        .solve(
            &mut bodies,
            &mut manifolds,
            super::CollisionConfig::default(),
        )
        .unwrap();

    assert_eq!(bodies[0].body().state().linear_velocity(), Vec3::Y);
    assert_eq!(bodies[0].body().state().transform(), Transform::identity());
    assert!(bodies[1].body().state().linear_velocity().y > 0.0);
}
