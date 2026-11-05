use crate::mass::MassProperties;
use kapitsa_math::{Mat3, Quat, Vec3};

// A reference for a free two-body Ball step. The two kicks are the only
// constraint impulses; the drift carries world linear and spin momenta.
#[derive(Clone, Copy)]
struct Body {
    center: Vec3,
    rotation: Quat,
    momentum: Vec3,
    spin: Vec3,
    mass: MassProperties,
    anchor: Vec3,
}

impl Body {
    fn anchor_world(self) -> Vec3 {
        self.center
            + self
                .rotation
                .rotate_vector(self.anchor - self.mass.center_of_mass())
                .unwrap()
    }

    fn omega(self) -> Vec3 {
        let local = self.rotation.conjugate().rotate_vector(self.spin).unwrap();
        self.rotation
            .rotate_vector(self.mass.inertia().solve_spd(local).unwrap())
            .unwrap()
    }

    fn velocity_at_anchor(self) -> Vec3 {
        self.momentum / self.mass.mass() + self.omega().cross(self.anchor_world() - self.center)
    }

    fn kick(&mut self, impulse: Vec3, point: Vec3) {
        self.momentum += impulse;
        self.spin += (point - self.center).cross(impulse);
    }

    fn drift(&mut self, dt: f64) {
        self.center += self.momentum * (dt / self.mass.mass());
        let initial_rotation = self.rotation;
        let mut omega = self.omega();
        let mut converged = false;
        for _ in 0..16 {
            self.rotation = initial_rotation
                .integrate_world_angular_velocity(omega, dt * 0.5)
                .unwrap();
            let next = self.omega();
            if (next - omega).length() < 1.0e-14 {
                omega = next;
                converged = true;
                break;
            }
            omega = next;
        }
        assert!(converged, "rotational midpoint did not converge");
        self.rotation = initial_rotation
            .integrate_world_angular_velocity(omega, dt)
            .unwrap();
    }
}

fn position_after_kick(start: [Body; 2], impulse: Vec3, dt: f64) -> ([Body; 2], Vec3) {
    let mut bodies = start;
    let anchor = (bodies[0].anchor_world() + bodies[1].anchor_world()) * 0.5;
    bodies[0].kick(-impulse, anchor);
    bodies[1].kick(impulse, anchor);
    bodies[0].drift(dt);
    bodies[1].drift(dt);
    let error = bodies[1].anchor_world() - bodies[0].anchor_world();
    (bodies, error)
}

fn solve_columns(columns: [Vec3; 3], rhs: Vec3) -> Vec3 {
    let determinant = columns[0].dot(columns[1].cross(columns[2]));
    assert!(determinant.abs() > 1.0e-16, "singular constraint Jacobian");
    Vec3::new(
        rhs.dot(columns[1].cross(columns[2])),
        columns[0].dot(rhs.cross(columns[2])),
        columns[0].dot(columns[1].cross(rhs)),
    ) / determinant
}

fn constrained_step(start: [Body; 2], dt: f64) -> [Body; 2] {
    let mut impulse = Vec3::ZERO;
    let epsilon = 1.0e-5;
    let mut next = start;
    let mut converged = false;
    for _ in 0..12 {
        let (candidate, error) = position_after_kick(start, impulse, dt);
        next = candidate;
        if error.length() < 1.0e-13 {
            converged = true;
            break;
        }
        let columns = [Vec3::X, Vec3::Y, Vec3::Z].map(|axis| {
            let (_, plus) = position_after_kick(start, impulse + axis * epsilon, dt);
            let (_, minus) = position_after_kick(start, impulse - axis * epsilon, dt);
            (plus - minus) / (2.0 * epsilon)
        });
        impulse += solve_columns(columns, -error);
    }
    assert!(converged, "position constraint did not converge");

    let common_anchor = (next[0].anchor_world() + next[1].anchor_world()) * 0.5;
    let relative = next[1].velocity_at_anchor() - next[0].velocity_at_anchor();
    let columns = [Vec3::X, Vec3::Y, Vec3::Z].map(|axis| {
        let mut candidate = next;
        candidate[0].kick(-axis, common_anchor);
        candidate[1].kick(axis, common_anchor);
        candidate[1].velocity_at_anchor() - candidate[0].velocity_at_anchor() - relative
    });
    let final_impulse = solve_columns(columns, -relative);
    next[0].kick(-final_impulse, common_anchor);
    next[1].kick(final_impulse, common_anchor);
    next
}

fn metrics(bodies: [Body; 2]) -> (Vec3, Vec3, f64, f64, f64) {
    let mut linear = Vec3::ZERO;
    let mut angular = Vec3::ZERO;
    let mut energy = 0.0;
    for body in bodies {
        linear += body.momentum;
        angular += body.center.cross(body.momentum) + body.spin;
        energy += 0.5 * body.momentum.length_squared() / body.mass.mass()
            + 0.5 * body.omega().dot(body.spin);
    }
    (
        linear,
        angular,
        energy,
        (bodies[1].anchor_world() - bodies[0].anchor_world()).length(),
        (bodies[1].velocity_at_anchor() - bodies[0].velocity_at_anchor()).length(),
    )
}

fn fixture(consistent: bool) -> [Body; 2] {
    let omega = Vec3::new(0.05, 0.1, 0.02);
    let anchor = Vec3::X * 0.5;
    [
        (
            Vec3::ZERO,
            Quat::from_axis_angle(Vec3::Y, 0.2).unwrap(),
            0.8,
            Vec3::new(0.01, -0.02, 0.03),
            Vec3::new(0.6, 0.8, 1.0),
        ),
        (
            Vec3::X,
            Quat::from_axis_angle(Vec3::Z, -0.3).unwrap(),
            1.3,
            Vec3::new(-0.03, 0.01, 0.02),
            Vec3::new(0.9, 1.2, 1.4),
        ),
    ]
    .map(|(origin, rotation, mass, local_center, inertia)| {
        let mass = MassProperties::new(mass, local_center, Mat3::from_diagonal(inertia)).unwrap();
        let center = origin + rotation.rotate_vector(local_center).unwrap();
        let local_anchor = rotation.conjugate().rotate_vector(anchor - origin).unwrap();
        let linear = if consistent {
            Vec3::new(0.2, -0.1, 0.05) + omega.cross(center)
        } else if origin == Vec3::ZERO {
            Vec3::new(-0.4, -0.15, -0.3)
        } else {
            Vec3::new(-0.2, 0.0, -0.2)
        };
        let angular = if consistent {
            omega
        } else if origin == Vec3::ZERO {
            Vec3::new(-0.075, -0.125, -0.025)
        } else {
            Vec3::new(-0.025, -0.075, 0.025)
        };
        let local_omega = rotation.conjugate().rotate_vector(angular).unwrap();
        let spin = rotation
            .rotate_vector(mass.inertia() * local_omega)
            .unwrap();
        Body {
            center,
            rotation,
            momentum: linear * mass.mass(),
            spin,
            mass,
            anchor: local_anchor,
        }
    })
}

fn production_world(bodies: [Body; 2]) -> crate::world::PhysicsWorld {
    use crate::{
        joint::{BallJoint, Joint, JointCommon, JointFrame},
        rigid_body::RigidBody,
        transform::Transform,
    };
    let mut world = crate::world::PhysicsWorld::new();
    let ids: Vec<_> = bodies
        .iter()
        .map(|body| {
            let origin = body.center
                - body
                    .rotation
                    .rotate_vector(body.mass.center_of_mass())
                    .unwrap();
            let mut rigid =
                RigidBody::new_dynamic(Transform::new(origin, body.rotation).unwrap(), body.mass)
                    .unwrap();
            rigid
                .set_linear_velocity(body.momentum / body.mass.mass())
                .unwrap();
            rigid.set_angular_velocity(body.omega()).unwrap();
            world.add_body(rigid).unwrap()
        })
        .collect();
    world
        .add_joint(
            ids[0],
            ids[1],
            Joint::Ball(BallJoint::new(JointCommon::new(
                JointFrame::new(bodies[0].anchor, Quat::IDENTITY),
                JointFrame::new(bodies[1].anchor, Quat::IDENTITY),
            ))),
        )
        .unwrap();
    world
}

fn production_metrics(
    world: &crate::world::PhysicsWorld,
    initial: [Body; 2],
) -> (Vec3, Vec3, f64, f64, f64) {
    let bodies: Vec<_> = world
        .bodies()
        .zip(initial)
        .map(|((_, rigid), initial)| {
            let state = rigid.state();
            let rotation = state.transform().rotation();
            let local_omega = rotation
                .conjugate()
                .rotate_vector(state.angular_velocity())
                .unwrap();
            Body {
                center: rigid.world_center_of_mass().unwrap().unwrap(),
                rotation,
                momentum: state.linear_velocity() * initial.mass.mass(),
                spin: rotation
                    .rotate_vector(initial.mass.inertia() * local_omega)
                    .unwrap(),
                mass: initial.mass,
                anchor: initial.anchor,
            }
        })
        .collect();
    metrics([bodies[0], bodies[1]])
}

#[test]
fn two_body_constrained_reference_conserves_momentum_and_converges_in_time() {
    for consistent in [true, false] {
        let mut previous_error = f64::INFINITY;
        for (subdivision, dt) in [(1, 1.0 / 60.0), (2, 1.0 / 120.0), (4, 1.0 / 240.0)] {
            let mut bodies = fixture(consistent);
            let fixture = bodies;
            let mut world = production_world(fixture);
            let initial = metrics(bodies);
            assert!(initial.3 < 1.0e-14);
            if consistent {
                assert!(initial.4 < 1.0e-14);
            } else {
                assert!(initial.4 > 0.1);
            }
            bodies = constrained_step(bodies, dt);
            world.step(dt).unwrap();
            let after_first = metrics(bodies);
            assert!(after_first.3 < 1.0e-12 && after_first.4 < 1.0e-12);
            if !consistent {
                assert!(after_first.2 < initial.2);
            }
            for _ in 1..300 * subdivision {
                bodies = constrained_step(bodies, dt);
                world.step(dt).unwrap();
            }
            let final_metrics = metrics(bodies);
            let production = production_metrics(&world, fixture);
            let p_drift = (final_metrics.0 - initial.0).length();
            let l_drift = (final_metrics.1 - initial.1).length();
            let energy_change = (final_metrics.2 - initial.2) / initial.2;
            let production_l_drift = (production.1 - initial.1).length();
            let production_energy = (production.2 - initial.2) / initial.2;
            println!(
                "production consistent={consistent} dt={dt:.6} dL={production_l_drift:.6e} dE/E={production_energy:.6e}"
            );
            assert!(production_l_drift.is_finite() && production_energy.is_finite());
            println!(
                "reference consistent={consistent} dt={dt:.6} dP={p_drift:.6e} dL={l_drift:.6e} dE/E={energy_change:.6e} first_E={:.9e} anchor={:.3e} vel={:.3e}",
                after_first.2, final_metrics.3, final_metrics.4
            );
            assert!(p_drift < 1.0e-10 && l_drift < 1.0e-10);
            assert!(final_metrics.3 < 1.0e-12 && final_metrics.4 < 1.0e-12);
            let energy_error = if consistent {
                (final_metrics.2 - initial.2).abs()
            } else {
                (final_metrics.2 - after_first.2).abs()
            };
            assert!(energy_error < previous_error);
            previous_error = energy_error;
        }
    }
}
