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

#[derive(Clone, Copy)]
struct BallLink {
    a: usize,
    b: usize,
    anchor_a: Vec3,
    anchor_b: Vec3,
}

fn tree_anchor(body: Body, local: Vec3) -> Vec3 {
    body.center
        + body
            .rotation
            .rotate_vector(local - body.mass.center_of_mass())
            .unwrap()
}

fn tree_anchor_velocity(body: Body, local: Vec3) -> Vec3 {
    body.momentum / body.mass.mass() + body.omega().cross(tree_anchor(body, local) - body.center)
}

fn tree_kick(bodies: &mut [Body], link: BallLink, impulse: Vec3, point: Vec3) {
    let (left, right) = bodies.split_at_mut(link.b);
    left[link.a].kick(-impulse, point);
    right[0].kick(impulse, point);
}

fn tree_values(bodies: &[Body], links: &[BallLink], velocity: bool) -> Vec<f64> {
    let mut result = Vec::with_capacity(links.len() * 3);
    for &link in links {
        let value = if velocity {
            tree_anchor_velocity(bodies[link.b], link.anchor_b)
                - tree_anchor_velocity(bodies[link.a], link.anchor_a)
        } else {
            tree_anchor(bodies[link.b], link.anchor_b) - tree_anchor(bodies[link.a], link.anchor_a)
        };
        result.extend([value.x, value.y, value.z]);
    }
    result
}

fn solve_dense(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Vec<f64> {
    let n = rhs.len();
    for column in 0..n {
        let pivot = (column..n)
            .max_by(|&a, &b| matrix[a][column].abs().total_cmp(&matrix[b][column].abs()))
            .unwrap();
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        assert!(
            matrix[column][column].abs() > 1.0e-16,
            "singular reference system"
        );
        let pivot_row = matrix[column].clone();
        for row in column + 1..n {
            let factor = matrix[row][column] / pivot_row[column];
            rhs[row] -= factor * rhs[column];
            for index in column + 1..n {
                matrix[row][index] -= factor * pivot_row[index];
            }
        }
    }
    let mut result = vec![0.0; n];
    for row in (0..n).rev() {
        let rest: f64 = (row + 1..n)
            .map(|column| matrix[row][column] * result[column])
            .sum();
        result[row] = (rhs[row] - rest) / matrix[row][row];
        assert!(result[row].is_finite());
    }
    result
}

fn tree_predict(start: &[Body], links: &[BallLink], impulses: &[f64], dt: f64) -> Vec<Body> {
    let mut bodies = start.to_vec();
    for (index, &link) in links.iter().enumerate() {
        let point = (tree_anchor(start[link.a], link.anchor_a)
            + tree_anchor(start[link.b], link.anchor_b))
            * 0.5;
        let impulse = Vec3::new(
            impulses[3 * index],
            impulses[3 * index + 1],
            impulses[3 * index + 2],
        );
        tree_kick(&mut bodies, link, impulse, point);
    }
    for body in &mut bodies {
        body.drift(dt);
    }
    bodies
}

fn tree_constrained_step(start: &[Body], links: &[BallLink], dt: f64) -> Vec<Body> {
    let dimensions = links.len() * 3;
    let mut impulses = vec![0.0; dimensions];
    let mut next = Vec::new();
    let epsilon = 1.0e-5;
    let mut converged = false;
    for _ in 0..12 {
        next = tree_predict(start, links, &impulses, dt);
        let error = tree_values(&next, links, false);
        if error.iter().all(|value| value.abs() < 1.0e-13) {
            converged = true;
            break;
        }
        let mut jacobian = vec![vec![0.0; dimensions]; dimensions];
        for column in 0..dimensions {
            impulses[column] += epsilon;
            let plus = tree_values(&tree_predict(start, links, &impulses, dt), links, false);
            impulses[column] -= 2.0 * epsilon;
            let minus = tree_values(&tree_predict(start, links, &impulses, dt), links, false);
            impulses[column] += epsilon;
            for row in 0..dimensions {
                jacobian[row][column] = (plus[row] - minus[row]) / (2.0 * epsilon);
            }
        }
        let delta = solve_dense(jacobian, error.iter().map(|value| -*value).collect());
        for (value, correction) in impulses.iter_mut().zip(delta) {
            *value += correction;
        }
    }
    assert!(converged, "tree position constraints did not converge");
    let residual = tree_values(&next, links, true);
    let mut jacobian = vec![vec![0.0; dimensions]; dimensions];
    for column in 0..dimensions {
        let link = links[column / 3];
        let axis = [Vec3::X, Vec3::Y, Vec3::Z][column % 3];
        let point = (tree_anchor(next[link.a], link.anchor_a)
            + tree_anchor(next[link.b], link.anchor_b))
            * 0.5;
        let mut displaced = next.clone();
        tree_kick(&mut displaced, link, axis, point);
        let changed = tree_values(&displaced, links, true);
        for row in 0..dimensions {
            jacobian[row][column] = changed[row] - residual[row];
        }
    }
    let impulses = solve_dense(jacobian, residual.iter().map(|value| -*value).collect());
    for (index, &link) in links.iter().enumerate() {
        let point = (tree_anchor(next[link.a], link.anchor_a)
            + tree_anchor(next[link.b], link.anchor_b))
            * 0.5;
        tree_kick(
            &mut next,
            link,
            Vec3::new(
                impulses[3 * index],
                impulses[3 * index + 1],
                impulses[3 * index + 2],
            ),
            point,
        );
    }
    next
}

fn tree_fixture(count: usize) -> (Vec<Body>, Vec<BallLink>) {
    let mut bodies: Vec<Body> = Vec::with_capacity(count);
    let mut origins = vec![Vec3::ZERO; count];
    let mut links = Vec::new();
    let omega = Vec3::new(0.05, 0.1, 0.02);
    for index in 0..count {
        if index > 0 {
            origins[index] = origins[(index - 1) / 2]
                + if index.is_multiple_of(2) {
                    Vec3::X
                } else {
                    Vec3::Y
                };
        }
        let angle = if index.is_multiple_of(2) {
            0.001
        } else {
            -0.001
        };
        let rotation = Quat::from_axis_angle(Vec3::new(0.2, 0.7, -0.3), angle).unwrap();
        let mass_value = 0.8 + (index % 7) as f64 * 0.17;
        let center_local = Vec3::new(
            (index % 3) as f64 * 0.002,
            -((index % 5) as f64) * 0.001,
            (index % 2) as f64 * 0.0015,
        );
        let scale = 0.7 + (index % 5) as f64 * 0.13;
        let mass = MassProperties::new(
            mass_value,
            center_local,
            Mat3::from_diagonal(Vec3::new(scale * 0.6, scale * 0.8, scale)),
        )
        .unwrap();
        let center = origins[index] + rotation.rotate_vector(center_local).unwrap();
        let local_omega = rotation.conjugate().rotate_vector(omega).unwrap();
        let spin = rotation
            .rotate_vector(mass.inertia() * local_omega)
            .unwrap();
        bodies.push(Body {
            center,
            rotation,
            momentum: (Vec3::new(0.2, -0.1, 0.05) + omega.cross(center)) * mass_value,
            spin,
            mass,
            anchor: Vec3::ZERO,
        });
        if index > 0 {
            let parent = (index - 1) / 2;
            let anchor_a = bodies[parent]
                .rotation
                .conjugate()
                .rotate_vector(origins[index] - origins[parent])
                .unwrap();
            links.push(BallLink {
                a: parent,
                b: index,
                anchor_a,
                anchor_b: Vec3::ZERO,
            });
        }
    }
    (bodies, links)
}

fn tree_invariants(bodies: &[Body], links: &[BallLink]) -> (Vec3, Vec3, f64, f64, f64) {
    let mut linear = Vec3::ZERO;
    let mut angular = Vec3::ZERO;
    let mut energy = 0.0;
    for &body in bodies {
        linear += body.momentum;
        angular += body.center.cross(body.momentum) + body.spin;
        energy += 0.5 * body.momentum.length_squared() / body.mass.mass()
            + 0.5 * body.omega().dot(body.spin);
    }
    let position = tree_values(bodies, links, false)
        .into_iter()
        .fold(0.0_f64, |max, value| max.max(value.abs()));
    let velocity = tree_values(bodies, links, true)
        .into_iter()
        .fold(0.0_f64, |max, value| max.max(value.abs()));
    (linear, angular, energy, position, velocity)
}

fn tree_production_world(bodies: &[Body], links: &[BallLink]) -> crate::world::PhysicsWorld {
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
    for link in links {
        world
            .add_joint(
                ids[link.a],
                ids[link.b],
                Joint::Ball(BallJoint::new(JointCommon::new(
                    JointFrame::new(link.anchor_a, Quat::IDENTITY),
                    JointFrame::new(link.anchor_b, Quat::IDENTITY),
                ))),
            )
            .unwrap();
    }
    world
}

fn tree_production_state(world: &crate::world::PhysicsWorld, template: &[Body]) -> Vec<Body> {
    world
        .bodies()
        .zip(template)
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
        .collect()
}

#[test]
fn ball_trees_constrained_reference_preserves_momentum_and_converges() {
    for count in [10usize, 31] {
        let mut previous_error = f64::INFINITY;
        for (subdivision, dt) in [(1, 1.0 / 60.0), (2, 1.0 / 120.0), (4, 1.0 / 240.0)] {
            let (mut bodies, links) = tree_fixture(count);
            let template = bodies.clone();
            let mut world = tree_production_world(&bodies, &links);
            let initial = tree_invariants(&bodies, &links);
            assert!(initial.3 < 1.0e-14 && initial.4 < 1.0e-14);
            // One fixed physical second, independent of dt.
            for _ in 0..60 * subdivision {
                bodies = tree_constrained_step(&bodies, &links, dt);
                world.step(dt).unwrap();
            }
            let end = tree_invariants(&bodies, &links);
            let production = tree_invariants(&tree_production_state(&world, &template), &links);
            println!(
                "production tree n={count} dt={dt:.6}: dP={:.6e} dL={:.6e} dE/E={:.6e} anchor={:.3e} velocity={:.3e}",
                (production.0 - initial.0).length(),
                (production.1 - initial.1).length(),
                (production.2 - initial.2) / initial.2,
                production.3,
                production.4
            );
            assert!(
                production.0.is_finite() && production.1.is_finite() && production.2.is_finite()
            );
            let p_drift = (end.0 - initial.0).length();
            let l_drift = (end.1 - initial.1).length();
            let energy_error = (end.2 - initial.2).abs();
            println!(
                "reference tree n={count} dt={dt:.6}: dP={p_drift:.6e} dL={l_drift:.6e} dE/E={:.6e} anchor={:.3e} velocity={:.3e}",
                (end.2 - initial.2) / initial.2,
                end.3,
                end.4
            );
            assert!(p_drift < 1.0e-9 && l_drift < 1.0e-9);
            assert!(end.3 < 1.0e-12 && end.4 < 1.0e-12);
            assert!(energy_error / initial.2 < 1.0e-3 && energy_error < 0.35 * previous_error);
            previous_error = energy_error;
        }
    }
}
