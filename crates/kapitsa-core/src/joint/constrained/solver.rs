use super::{EqualityRow, MAX_EQUALITY_ROWS, MomentumBody, finite, write_equality_rows};
use crate::{
    collision::CollisionError,
    constraint::SolverBody,
    joint::JointConstraint,
    rigid_body::{BodyState, BodyType},
};
use kapitsa_math::{Mat3, Vec3};

#[derive(Clone, Copy, Default)]
pub(crate) struct IslandEligibility {
    pub(crate) has_colliders: bool,
    pub(crate) has_contacts: bool,
    pub(crate) has_ccd: bool,
    pub(crate) has_clusters: bool,
    pub(crate) external_work: bool,
}

#[derive(Clone, Copy)]
struct MassResponse {
    linear: f64,
    angular: Mat3,
}

#[derive(Default)]
pub(crate) struct ConstrainedWorkspace {
    start: Vec<MomentumBody>,
    candidate: Vec<MomentumBody>,
    rows: Vec<EqualityRow>,
    start_rows: Vec<EqualityRow>,
    masses: Vec<MassResponse>,
    generalized: Vec<[Vec3; 2]>,
    impulses: Vec<f64>,
    rhs: Vec<f64>,
    solution: Vec<f64>,
    residual: Vec<f64>,
    direction: Vec<f64>,
    product: Vec<f64>,
    diagonal: Vec<f64>,
    states: Vec<BodyState>,
    pub(crate) linear_iterations: usize,
    pub(crate) nonlinear_iterations: usize,
}

pub(crate) struct ConstrainedIntegrator;
impl ConstrainedIntegrator {
    pub(crate) fn eligible(
        bodies: &[SolverBody],
        joints: &[JointConstraint],
        flags: IslandEligibility,
    ) -> bool {
        !bodies.is_empty()
            && !joints.is_empty()
            && !flags.has_colliders
            && !flags.has_contacts
            && !flags.has_ccd
            && !flags.has_clusters
            && !flags.external_work
            && bodies.iter().all(|body| {
                let body = body.body();
                body.physics().body_type() == BodyType::Dynamic
                    && !body.state().is_sleeping()
                    && body.forces().force() == Vec3::ZERO
                    && body.forces().torque() == Vec3::ZERO
            })
            && joints
                .iter()
                .all(|joint| joint.joint.supports_free_bilateral())
    }

    pub(crate) fn solve(
        bodies: &mut [SolverBody],
        joints: &[JointConstraint],
        dt: f64,
        scratch: &mut ConstrainedWorkspace,
    ) -> Result<(), CollisionError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(CollisionError::InvalidConfig);
        }
        scratch.linear_iterations = 0;
        scratch.nonlinear_iterations = 0;
        scratch.start.clear();
        reserve(&mut scratch.start, bodies.len())?;
        reserve(&mut scratch.candidate, bodies.len())?;
        reserve(&mut scratch.states, bodies.len())?;
        for body in bodies.iter() {
            scratch.start.push(MomentumBody::new(body.body())?);
        }
        write_rows(joints, &scratch.start, &mut scratch.start_rows)?;
        resize(&mut scratch.impulses, scratch.start_rows.len(), 0.0)?;
        scratch.impulses.fill(0.0);
        scratch.prepare_masses(true)?;
        let mut converged = false;
        for iteration in 0..32 {
            scratch.nonlinear_iterations = iteration + 1;
            scratch.candidate.clear();
            scratch.candidate.extend_from_slice(&scratch.start);
            transpose(
                &scratch.start_rows,
                &scratch.impulses,
                &mut scratch.generalized,
                bodies.len(),
            )?;
            for (body, kick) in scratch.candidate.iter_mut().zip(&scratch.generalized) {
                body.kick(kick[0], kick[1])?;
                body.drift(dt)?;
            }
            write_rows(joints, &scratch.candidate, &mut scratch.rows)?;
            if scratch.rows.iter().all(|row| row.value.abs() <= 1.0e-13) {
                converged = true;
                break;
            }
            resize(&mut scratch.rhs, scratch.rows.len(), 0.0)?;
            for (rhs, row) in scratch.rhs.iter_mut().zip(&scratch.rows) {
                *rhs = -row.value / dt;
            }
            scratch.linear_solve(true)?;
            for (impulse, delta) in scratch.impulses.iter_mut().zip(&scratch.solution) {
                *impulse += delta;
                if !impulse.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
            }
        }
        if !converged {
            return Err(CollisionError::NonConvergent);
        }
        scratch.prepare_masses(false)?;
        let mut compatible = false;
        for _ in 0..3 {
            resize(&mut scratch.rhs, scratch.rows.len(), 0.0)?;
            for (rhs, row) in scratch.rhs.iter_mut().zip(&scratch.rows) {
                *rhs = -velocity(*row, &scratch.candidate)?;
            }
            if scratch.rhs.iter().all(|value| value.abs() <= 1.0e-12) {
                compatible = true;
                break;
            }
            scratch.linear_solve(false)?;
            transpose(
                &scratch.rows,
                &scratch.solution,
                &mut scratch.generalized,
                bodies.len(),
            )?;
            for (body, kick) in scratch.candidate.iter_mut().zip(&scratch.generalized) {
                body.kick(kick[0], kick[1])?;
            }
        }
        if !compatible {
            return Err(CollisionError::NonConvergent);
        }
        scratch.states.clear();
        for (body, original) in scratch.candidate.iter().zip(bodies.iter()) {
            scratch.states.push(body.state(*original.body())?);
        }
        for (body, state) in bodies.iter_mut().zip(&scratch.states) {
            body.body_mut().set_state_preserving_forces(*state);
        }
        Ok(())
    }
}

impl ConstrainedWorkspace {
    fn prepare_masses(&mut self, start: bool) -> Result<(), CollisionError> {
        self.masses.clear();
        let bodies = if start { &self.start } else { &self.candidate };
        reserve(&mut self.masses, bodies.len())?;
        for body in bodies {
            let rotation = body
                .rotation
                .to_mat3()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let angular = rotation * body.inverse_inertia * rotation.transpose();
            let linear = 1.0 / body.mass.mass();
            if !angular.is_finite() || !linear.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            self.masses.push(MassResponse { linear, angular });
        }
        Ok(())
    }

    fn linear_solve(&mut self, start: bool) -> Result<(), CollisionError> {
        let rows = if start { &self.start_rows } else { &self.rows };
        let n = rows.len();
        resize(&mut self.solution, n, 0.0)?;
        self.solution.fill(0.0);
        resize(&mut self.residual, n, 0.0)?;
        self.residual.copy_from_slice(&self.rhs);
        resize(&mut self.direction, n, 0.0)?;
        resize(&mut self.product, n, 0.0)?;
        resize(&mut self.diagonal, n, 0.0)?;
        let mut rho = 0.0;
        for (i, row) in rows.iter().enumerate() {
            let ma = self
                .masses
                .get(row.a)
                .ok_or(CollisionError::InvalidContact)?;
            let mb = self
                .masses
                .get(row.b)
                .ok_or(CollisionError::InvalidContact)?;
            let d = ma.linear * row.linear_a.length_squared()
                + row.angular_a.dot(ma.angular * row.angular_a)
                + mb.linear * row.linear_b.length_squared()
                + row.angular_b.dot(mb.angular * row.angular_b);
            if !d.is_finite() || d <= 0.0 {
                return Err(CollisionError::NonConvergent);
            }
            self.diagonal[i] = d;
            self.direction[i] = self.residual[i] / d;
            rho += self.residual[i] * self.direction[i];
        }
        let rhs_norm = dot(&self.rhs, &self.rhs)?.sqrt();
        let tolerance = 1.0e-14 * (1.0 + rhs_norm);
        for _ in 0..n.saturating_mul(2).saturating_add(32) {
            if dot(&self.residual, &self.residual)?.sqrt() <= tolerance {
                return Ok(());
            }
            operator(
                rows,
                &self.masses,
                &self.direction,
                &mut self.product,
                &mut self.generalized,
            )?;
            let curvature = dot(&self.direction, &self.product)?;
            if curvature <= 0.0 || !rho.is_finite() {
                return Err(CollisionError::NonConvergent);
            }
            let alpha = rho / curvature;
            for i in 0..n {
                self.solution[i] += alpha * self.direction[i];
                self.residual[i] -= alpha * self.product[i];
            }
            // Recompute the true residual to prevent a recurrence-only success.
            if dot(&self.residual, &self.residual)?.sqrt() <= tolerance {
                operator(
                    rows,
                    &self.masses,
                    &self.solution,
                    &mut self.product,
                    &mut self.generalized,
                )?;
                for i in 0..n {
                    self.residual[i] = self.rhs[i] - self.product[i];
                }
                if dot(&self.residual, &self.residual)?.sqrt() <= tolerance {
                    return Ok(());
                }
            }
            let next: f64 = self
                .residual
                .iter()
                .zip(&self.diagonal)
                .map(|(r, d)| r * r / d)
                .sum();
            let beta = next / rho;
            for i in 0..n {
                self.direction[i] = self.residual[i] / self.diagonal[i] + beta * self.direction[i];
            }
            rho = next;
            self.linear_iterations += 1;
        }
        Err(CollisionError::NonConvergent)
    }
}

fn velocity(row: EqualityRow, bodies: &[MomentumBody]) -> Result<f64, CollisionError> {
    let a = bodies.get(row.a).ok_or(CollisionError::InvalidContact)?;
    let b = bodies.get(row.b).ok_or(CollisionError::InvalidContact)?;
    let value = row.linear_a.dot(a.momentum / a.mass.mass())
        + row.angular_a.dot(a.omega()?)
        + row.linear_b.dot(b.momentum / b.mass.mass())
        + row.angular_b.dot(b.omega()?);
    if !value.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(value)
}
fn write_rows(
    joints: &[JointConstraint],
    bodies: &[MomentumBody],
    rows: &mut Vec<EqualityRow>,
) -> Result<(), CollisionError> {
    rows.clear();
    reserve(rows, joints.len().saturating_mul(MAX_EQUALITY_ROWS))?;
    let mut local = [EqualityRow::ZERO; MAX_EQUALITY_ROWS];
    for joint in joints {
        let count = write_equality_rows(*joint, bodies, &mut local)?;
        rows.extend_from_slice(&local[..count]);
    }
    Ok(())
}
fn transpose(
    rows: &[EqualityRow],
    values: &[f64],
    out: &mut Vec<[Vec3; 2]>,
    count: usize,
) -> Result<(), CollisionError> {
    resize(out, count, [Vec3::ZERO; 2])?;
    out.fill([Vec3::ZERO; 2]);
    for (row, value) in rows.iter().zip(values) {
        for (index, linear, angular) in [
            (row.a, row.linear_a, row.angular_a),
            (row.b, row.linear_b, row.angular_b),
        ] {
            let body = out.get_mut(index).ok_or(CollisionError::InvalidContact)?;
            body[0] = finite(body[0] + linear * *value)?;
            body[1] = finite(body[1] + angular * *value)?;
        }
    }
    Ok(())
}
fn operator(
    rows: &[EqualityRow],
    masses: &[MassResponse],
    input: &[f64],
    out: &mut [f64],
    generalized: &mut Vec<[Vec3; 2]>,
) -> Result<(), CollisionError> {
    transpose(rows, input, generalized, masses.len())?;
    for (value, mass) in generalized.iter_mut().zip(masses) {
        value[0] = finite(value[0] * mass.linear)?;
        value[1] = finite(mass.angular * value[1])?;
    }
    for (row, value) in rows.iter().zip(out) {
        let a = generalized[row.a];
        let b = generalized[row.b];
        *value = row.linear_a.dot(a[0])
            + row.angular_a.dot(a[1])
            + row.linear_b.dot(b[0])
            + row.angular_b.dot(b[1]);
        if !value.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
    }
    Ok(())
}
fn dot(a: &[f64], b: &[f64]) -> Result<f64, CollisionError> {
    let value: f64 = a.iter().zip(b).map(|(a, b)| a * b).sum();
    value
        .is_finite()
        .then_some(value)
        .ok_or(CollisionError::ArithmeticOverflow)
}
fn reserve<T>(values: &mut Vec<T>, count: usize) -> Result<(), CollisionError> {
    values
        .try_reserve(count.saturating_sub(values.len()))
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)
}
fn resize<T: Clone>(values: &mut Vec<T>, count: usize, value: T) -> Result<(), CollisionError> {
    reserve(values, count)?;
    values.resize(count, value);
    Ok(())
}
