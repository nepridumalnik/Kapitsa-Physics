use super::*;
use crate::{
    constraint::SolverBody,
    joint::{BallJoint, Joint, JointCommon, JointConstraint, JointFrame},
    mass::MassProperties,
    rigid_body::RigidBody,
    transform::Transform,
    world::PhysicsWorld,
};
use kapitsa_math::{Mat3, Quat, Vec3};

fn fixture(count: usize) -> (Vec<SolverBody>, Vec<JointConstraint>) {
    let mut world = PhysicsWorld::new();
    let omega = Vec3::new(0.05, 0.1, 0.02);
    let mut ids = Vec::new();
    for i in 0..count {
        let mass = MassProperties::new(
            0.8 + (i % 3) as f64 * 0.3,
            Vec3::new(0.01, -0.02, 0.03),
            Mat3::from_rows([[0.6, 0.08, -0.03], [0.08, 0.8, 0.04], [-0.03, 0.04, 1.0]]),
        )
        .unwrap();
        let rotation = Quat::from_axis_angle(Vec3::Y, (i % 4) as f64 * 0.1).unwrap();
        let origin = Vec3::X * i as f64;
        let mut body =
            RigidBody::new_dynamic(Transform::new(origin, rotation).unwrap(), mass).unwrap();
        let center = body.world_center_of_mass().unwrap().unwrap();
        body.set_linear_velocity(Vec3::new(0.2, -0.1, 0.05) + omega.cross(center))
            .unwrap();
        body.set_angular_velocity(omega).unwrap();
        ids.push(world.add_body(body).unwrap());
    }
    for i in 1..count {
        let point = Vec3::X * (i as f64 - 0.5);
        let frames = [i - 1, i].map(|j| {
            let transform = world.body(ids[j]).unwrap().state().transform();
            JointFrame::new(
                transform
                    .rotation()
                    .conjugate()
                    .rotate_vector(point - transform.translation())
                    .unwrap(),
                transform.rotation().conjugate(),
            )
        });
        world
            .add_joint(
                ids[i - 1],
                ids[i],
                Joint::Ball(BallJoint::new(JointCommon::new(frames[0], frames[1]))),
            )
            .unwrap();
    }
    let bodies = world
        .bodies()
        .map(|(_, body)| SolverBody { body: *body })
        .collect();
    let joints = world
        .joints()
        .enumerate()
        .map(|(i, (id, _, _, joint))| JointConstraint {
            id,
            body_a: i,
            body_b: i + 1,
            joint: *joint,
        })
        .collect();
    (bodies, joints)
}
fn invariants(bodies: &[SolverBody]) -> (Vec3, Vec3, f64) {
    let mut p = Vec3::ZERO;
    let mut l = Vec3::ZERO;
    let mut e = 0.0;
    for body in bodies {
        let body = MomentumBody::new(body.body()).unwrap();
        p += body.momentum;
        l += body.center.cross(body.momentum) + body.spin;
        e += 0.5 * body.momentum.length_squared() / body.mass.mass()
            + 0.5 * body.omega().unwrap().dot(body.spin);
    }
    (p, l, e)
}
#[test]
fn ball_kernel_preserves_momentum_and_anchor_compatibility() {
    for count in [2, 10, 31] {
        let (mut bodies, joints) = fixture(count);
        let before = invariants(&bodies);
        let mut scratch = ConstrainedWorkspace::default();
        assert!(ConstrainedIntegrator::eligible(
            &bodies,
            &joints,
            IslandEligibility::default()
        ));
        for _ in 0..10 {
            ConstrainedIntegrator::solve(&mut bodies, &joints, 1.0 / 120.0, &mut scratch).unwrap();
        }
        let after = invariants(&bodies);
        assert!((after.0 - before.0).length() < 1.0e-10);
        assert!((after.1 - before.1).length() < 1.0e-9);
        assert!(((after.2 - before.2) / before.2).abs() < 1.0e-7);
    }
}
#[test]
fn ball_rows_are_invariant_under_translation_and_rotation() {
    let (bodies, joints) = fixture(2);
    let mut bodies: Vec<_> = bodies
        .iter()
        .map(|body| MomentumBody::new(body.body()).unwrap())
        .collect();
    bodies[1].center += Vec3::new(0.1, -0.2, 0.3);
    let mut rows = [EqualityRow::ZERO; MAX_EQUALITY_ROWS];
    let count = write_equality_rows(joints[0], &bodies, &mut rows).unwrap();
    for row in &rows[..count] {
        assert!((row.linear_a + row.linear_b).length() < 1.0e-14);
        assert!(
            (bodies[row.a].center.cross(row.linear_a)
                + row.angular_a
                + bodies[row.b].center.cross(row.linear_b)
                + row.angular_b)
                .length()
                < 1.0e-14
        );
    }
}

fn joint_variants(common: JointCommon) -> Vec<(Joint, usize)> {
    use crate::joint::*;
    let mut partial = D6Joint::new(common);
    partial.linear_motion[1] = JointMotion::Free;
    partial.angular_motion[2] = JointMotion::Free;
    let mut free = D6Joint::new(common);
    free.linear_motion = [JointMotion::Free; 3];
    free.angular_motion = [JointMotion::Free; 3];
    vec![
        (Joint::Ball(BallJoint::new(common)), 3),
        (Joint::Fixed(FixedJoint::new(common)), 6),
        (Joint::Hinge(HingeJoint::new(common)), 5),
        (Joint::Slider(SliderJoint::new(common)), 5),
        (Joint::Cylindrical(CylindricalJoint::new(common)), 4),
        (Joint::Planar(PlanarJoint::new(common)), 3),
        (Joint::Distance(DistanceJoint::fixed_length(common, 0.7)), 1),
        (Joint::D6(partial), 4),
        (Joint::D6(free), 0),
        (
            Joint::ConeTwist(ConeTwistJoint::new(
                common,
                [0.0; 2],
                JointLimit::new(0.3, 0.3),
            )),
            6,
        ),
        (
            Joint::ConeTwist(ConeTwistJoint::new(
                common,
                [0.0; 2],
                JointLimit::new(-std::f64::consts::PI, std::f64::consts::PI),
            )),
            5,
        ),
        (
            Joint::ConeTwist(ConeTwistJoint::new(
                common,
                [std::f64::consts::PI; 2],
                JointLimit::new(-std::f64::consts::PI, std::f64::consts::PI),
            )),
            3,
        ),
    ]
}

#[test]
fn all_equality_jacobians_match_finite_differences_and_rigid_invariance() {
    let (bodies, joints) = fixture(2);
    let mut bodies: Vec<_> = bodies
        .iter()
        .map(|body| MomentumBody::new(body.body()).unwrap())
        .collect();
    bodies[1].center += Vec3::new(0.1, -0.2, 0.3);
    bodies[1].rotation = bodies[1]
        .rotation
        .integrate_world_angular_velocity(Vec3::new(0.2, -0.1, 0.3), 1.0)
        .unwrap();
    let mut common = *joints[0].joint.common();
    let rotation = Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0), 0.4).unwrap();
    common.frame_a.local_rotation = common.frame_a.local_rotation * rotation;
    common.frame_b.local_rotation = common.frame_b.local_rotation * rotation;
    for (joint, expected_count) in joint_variants(common) {
        assert!(joint.supports_free_bilateral());
        let constraint = JointConstraint { joint, ..joints[0] };
        let mut rows = [EqualityRow::ZERO; MAX_EQUALITY_ROWS];
        let count = write_equality_rows(constraint, &bodies, &mut rows).unwrap();
        assert_eq!(count, expected_count);
        for row in &rows[..count] {
            assert!((row.linear_a + row.linear_b).length() < 1.0e-13);
            assert!(
                (bodies[row.a].center.cross(row.linear_a)
                    + row.angular_a
                    + bodies[row.b].center.cross(row.linear_b)
                    + row.angular_b)
                    .length()
                    < 1.0e-13,
                "{joint:?}"
            );
        }
        for body in 0..2 {
            for angular in [false, true] {
                for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                    let epsilon = 1.0e-6;
                    let mut perturbed = [bodies.clone(), bodies.clone()];
                    for (state, sign) in perturbed.iter_mut().zip([1.0, -1.0]) {
                        if angular {
                            state[body].rotation = state[body]
                                .rotation
                                .integrate_world_angular_velocity(axis, sign * epsilon)
                                .unwrap();
                        } else {
                            state[body].center += axis * (sign * epsilon);
                        }
                    }
                    let mut plus = [EqualityRow::ZERO; MAX_EQUALITY_ROWS];
                    let mut minus = plus;
                    write_equality_rows(constraint, &perturbed[0], &mut plus).unwrap();
                    write_equality_rows(constraint, &perturbed[1], &mut minus).unwrap();
                    for i in 0..count {
                        let row = rows[i];
                        let gradient = match (body, angular) {
                            (0, false) => row.linear_a,
                            (0, true) => row.angular_a,
                            (1, false) => row.linear_b,
                            _ => row.angular_b,
                        };
                        let numerical = (plus[i].value - minus[i].value) / (2.0 * epsilon);
                        assert!(
                            (numerical - gradient.dot(axis)).abs() < 2.0e-8,
                            "{joint:?} row={i} body={body} angular={angular} numerical={numerical} analytic={}",
                            gradient.dot(axis)
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn every_bilateral_kind_has_five_second_momentum_and_energy_convergence() {
    let (template, links) = fixture(2);
    for (kind, _) in joint_variants(*links[0].joint.common()) {
        let mut previous = f64::INFINITY;
        for frequency in [60, 120, 240] {
            let mut bodies = template.clone();
            let mut constraint = JointConstraint {
                joint: kind,
                ..links[0]
            };
            if let Joint::Distance(ref mut distance) = constraint.joint {
                distance.common.frame_a.local_anchor = Vec3::ZERO;
                distance.common.frame_b.local_anchor = Vec3::ZERO;
                distance.distance_limit.lower = 1.0;
                distance.distance_limit.upper = 1.0;
            }
            if let Joint::ConeTwist(ref mut cone) = constraint.joint
                && cone.twist_limit.lower == cone.twist_limit.upper
            {
                cone.common.frame_b.local_rotation = cone.common.frame_b.local_rotation
                    * Quat::from_axis_angle(Vec3::Z, cone.twist_limit.lower).unwrap();
            }
            let initial = invariants(&bodies);
            let mut scratch = ConstrainedWorkspace::default();
            for _ in 0..5 * frequency {
                ConstrainedIntegrator::solve(
                    &mut bodies,
                    &[constraint],
                    1.0 / f64::from(frequency),
                    &mut scratch,
                )
                .unwrap_or_else(|error| panic!("kind={kind:?} Hz={frequency}: {error}"));
            }
            let end = invariants(&bodies);
            let energy = ((end.2 - initial.2) / initial.2).abs();
            println!(
                "kind={:?} Hz={frequency} dP={:.6e} dL={:.6e} dE/E={energy:.6e}",
                std::mem::discriminant(&kind),
                (end.0 - initial.0).length(),
                (end.1 - initial.1).length()
            );
            assert!((end.0 - initial.0).length() < 1.0e-10);
            assert!((end.1 - initial.1).length() < 1.0e-10);
            assert!(energy < 1.0e-7 && (energy < 0.35 * previous || energy < 1.0e-12));
            previous = energy;
        }
    }
}

#[test]
fn redundant_closed_fixed_loop_preserves_invariants() {
    let (mut bodies, mut links) = fixture(3);
    for link in &mut links {
        link.joint = Joint::Fixed(crate::joint::FixedJoint::new(*link.joint.common()));
    }
    let frames = [0, 2].map(|i| {
        let transform = bodies[i].state().transform();
        JointFrame::new(
            transform
                .rotation()
                .conjugate()
                .rotate_vector(Vec3::X - transform.translation())
                .unwrap(),
            transform.rotation().conjugate(),
        )
    });
    links.push(JointConstraint {
        body_a: 0,
        body_b: 2,
        joint: Joint::Fixed(crate::joint::FixedJoint::new(JointCommon::new(
            frames[0], frames[1],
        ))),
        ..links[0]
    });
    let initial = invariants(&bodies);
    let mut scratch = ConstrainedWorkspace::default();
    for _ in 0..600 {
        ConstrainedIntegrator::solve(&mut bodies, &links, 1.0 / 120.0, &mut scratch).unwrap();
    }
    let end = invariants(&bodies);
    assert!((end.0 - initial.0).length() < 1.0e-10);
    assert!((end.1 - initial.1).length() < 1.0e-10);
}

fn production_fixture(count: usize) -> (PhysicsWorld, Vec<crate::world::BodyId>) {
    let (bodies, joints) = fixture(count);
    let mut world = PhysicsWorld::new();
    let ids: Vec<_> = bodies
        .iter()
        .map(|body| world.add_body(*body.body()).unwrap())
        .collect();
    for joint in joints {
        world
            .add_joint(ids[joint.body_a], ids[joint.body_b], joint.joint)
            .unwrap();
    }
    (world, ids)
}

#[test]
fn eligible_island_is_independent_of_legacy_contacts_and_external_work() {
    use crate::{
        collider::Collider,
        shape::{Shape, Sphere},
    };
    let (mut reference, reference_ids) = production_fixture(10);
    let (mut mixed, ids) = production_fixture(10);
    let mass = *mixed
        .body(ids[0])
        .unwrap()
        .physics()
        .mass()
        .as_ref()
        .unwrap();
    let static_id = mixed
        .add_body(RigidBody::new_static(
            Transform::new(Vec3::X * 100.0, Quat::IDENTITY).unwrap(),
        ))
        .unwrap();
    let moving = mixed
        .add_body(
            RigidBody::new_dynamic(
                Transform::new(Vec3::X * 100.9, Quat::IDENTITY).unwrap(),
                mass,
            )
            .unwrap(),
        )
        .unwrap();
    for id in [static_id, moving] {
        mixed
            .add_collider(
                id,
                Collider::new(
                    Shape::Sphere(Sphere::new(0.5).unwrap()),
                    Transform::identity(),
                ),
            )
            .unwrap();
    }
    mixed
        .add_force_source(moving, Vec3::ZERO, Vec3::Y * 0.01)
        .unwrap();
    for _ in 0..600 {
        reference.step(1.0 / 120.0).unwrap();
        mixed.step(1.0 / 120.0).unwrap();
        for (&a, &b) in ids.iter().zip(&reference_ids) {
            assert_eq!(mixed.body(a), reference.body(b));
        }
    }
}

#[test]
fn constrained_chain_sizes_retain_invariants_and_report_cost() {
    for count in [2, 10, 31, 100, 500, 1000] {
        let (mut bodies, joints) = fixture(count);
        let initial = invariants(&bodies);
        let mut scratch = ConstrainedWorkspace::default();
        let start = std::time::Instant::now();
        let result = ConstrainedIntegrator::solve(&mut bodies, &joints, 1.0 / 120.0, &mut scratch);
        println!(
            "chain n={count} result={result:?} us={} linear_iterations={} nonlinear_iterations={}",
            start.elapsed().as_micros(),
            scratch.linear_iterations,
            scratch.nonlinear_iterations
        );
        result.unwrap();
        let end = invariants(&bodies);
        assert!((end.0 - initial.0).length() < 1.0e-10 * (1.0 + initial.0.length()));
        assert!((end.1 - initial.1).length() < 1.0e-10 * (1.0 + initial.1.length()));
    }
}

#[cfg(feature = "bench-internals")]
mod rayon_executor {
    use crate as kapitsa_core;
    include!("../../../benches/support/rayon_executor.rs");
}

#[cfg(feature = "bench-internals")]
#[test]
#[ignore = "Release timing comparison with persistent Rayon pools"]
fn constrained_parallel_scaling_and_single_island_cost() {
    use crate::execution::SequentialExecutor;
    for (islands, count) in [(1, 100), (1, 500), (1, 1000), (8, 100)] {
        let mut reference = None;
        for workers in [1, 2, 4, 8] {
            let mut world = PhysicsWorld::new();
            for island in 0..islands {
                let (bodies, joints) = fixture(count);
                let ids: Vec<_> = bodies
                    .into_iter()
                    .map(|body| {
                        let mut body = body.body;
                        let transform = body.state().transform();
                        body.set_transform(
                            Transform::new(
                                transform.translation() + Vec3::Y * (island as f64 * 30.0),
                                transform.rotation(),
                            )
                            .unwrap(),
                        )
                        .unwrap();
                        world.add_body(body).unwrap()
                    })
                    .collect();
                for joint in joints {
                    world
                        .add_joint(ids[joint.body_a], ids[joint.body_b], joint.joint)
                        .unwrap();
                }
            }
            let pool = rayon_executor::RayonExecutor::new(workers).unwrap();
            for _ in 0..8 {
                world
                    .step_with_executor(1.0 / 120.0, &pool, workers)
                    .unwrap();
            }
            let mut samples = Vec::with_capacity(32);
            for _ in 0..32 {
                let start = std::time::Instant::now();
                if workers == 1 {
                    world
                        .step_with_executor(1.0 / 120.0, &SequentialExecutor, 1)
                        .unwrap();
                } else {
                    world
                        .step_with_executor(1.0 / 120.0, &pool, workers)
                        .unwrap();
                }
                samples.push(start.elapsed().as_nanos());
            }
            samples.sort_unstable();
            let states: Vec<_> = world.bodies().map(|(_, body)| *body.state()).collect();
            if let Some(reference) = &reference {
                assert_eq!(&states, reference);
            } else {
                reference = Some(states);
            }
            println!(
                "islands={islands} bodies_per_island={count} workers={workers} p50_us={:.3} p95_us={:.3}",
                samples[16] as f64 / 1000.0,
                samples[30] as f64 / 1000.0
            );
        }
    }
}
