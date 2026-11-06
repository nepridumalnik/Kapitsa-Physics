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
