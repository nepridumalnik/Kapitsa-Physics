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
    pub(crate) legacy_requested: bool,
}

#[derive(Clone, Copy)]
struct MassResponse {
    linear: f64,
    angular: Mat3,
}

#[derive(Clone, Copy)]
struct RowBlock {
    a: usize,
    b: usize,
    start: usize,
    end: usize,
}
const BAND_WIDTH: usize = 12; // At most two adjacent six-row joints on a path.

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
    normal: Vec<f64>,
    cyclic: bool,
    path_blocks: Vec<RowBlock>,
    path_edges: Vec<[usize; 2]>,
    path_visited: Vec<bool>,
    path_order: Vec<usize>,
    path_band: Vec<[f64; BAND_WIDTH]>,
    path_scale: Vec<f64>,
    path_rhs: Vec<f64>,
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
        let basic = !bodies.is_empty()
            && !joints.is_empty()
            && !flags.has_colliders
            && !flags.has_contacts
            && !flags.has_ccd
            && !flags.has_clusters
            && !flags.external_work
            && !flags.legacy_requested
            && bodies.iter().all(|body| {
                let body = body.body();
                body.physics().body_type() == BodyType::Dynamic
                    && !body.state().is_sleeping()
                    && body.forces().force() == Vec3::ZERO
                    && body.forces().torque() == Vec3::ZERO
            })
            && joints
                .iter()
                .all(|joint| joint.joint.supports_free_bilateral());
        if !basic {
            return false;
        }
        // RATTLE starts on the constraint manifold. Legacy projections may
        // leave a geometric defect after external work; admitting such a state
        // would convert that pre-existing defect into an unphysical start kick.
        // This is a preflight scope check, never a fallback after solve failure.
        joints.iter().all(|joint| {
            let (Some(a), Some(b)) = (bodies.get(joint.body_a), bodies.get(joint.body_b)) else {
                return true;
            };
            let (Ok(a), Ok(b)) = (MomentumBody::new(a.body()), MomentumBody::new(b.body())) else {
                return true;
            };
            let mut rows = [EqualityRow::ZERO; MAX_EQUALITY_ROWS];
            let local = JointConstraint {
                body_a: 0,
                body_b: 1,
                ..*joint
            };
            match write_equality_rows(local, &[a, b], &mut rows) {
                // Degeneracy and arithmetic failures must be reported by solve.
                Err(_) => true,
                Ok(count) => rows[..count].iter().all(|row| row.value.abs() <= 1.0e-9),
            }
        })
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
        scratch.cyclic = joints.len() >= bodies.len();
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
            if scratch
                .rows
                .iter()
                .all(|row| row.value.abs() <= row.tolerance)
            {
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
    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;
        (self.start.capacity() + self.candidate.capacity()) * size_of::<MomentumBody>()
            + (self.rows.capacity() + self.start_rows.capacity()) * size_of::<EqualityRow>()
            + self.masses.capacity() * size_of::<MassResponse>()
            + self.generalized.capacity() * size_of::<[Vec3; 2]>()
            + (self.impulses.capacity()
                + self.rhs.capacity()
                + self.solution.capacity()
                + self.residual.capacity()
                + self.direction.capacity()
                + self.product.capacity()
                + self.diagonal.capacity()
                + self.normal.capacity())
                * size_of::<f64>()
            + self.states.capacity() * size_of::<BodyState>()
            + self.path_blocks.capacity() * size_of::<RowBlock>()
            + self.path_edges.capacity() * size_of::<[usize; 2]>()
            + self.path_visited.capacity() * size_of::<bool>()
            + self.path_order.capacity() * size_of::<usize>()
            + self.path_band.capacity() * size_of::<[f64; BAND_WIDTH]>()
            + (self.path_scale.capacity() + self.path_rhs.capacity()) * size_of::<f64>()
    }

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

    // A path's Schur matrix is block tridiagonal after ordering joints along
    // the path. Its Cholesky bandwidth is at most 11 regardless of body count.
    // This exact O(N) structural solve avoids the O(N^2) propagation cost of
    // Jacobi-PCG on long chains. Branches keep the matrix-free PCG operator.
    fn path_solve(&mut self, start: bool) -> Result<bool, CollisionError> {
        let rows = if start { &self.start_rows } else { &self.rows };
        let n = rows.len();
        self.path_blocks.clear();
        reserve(&mut self.path_blocks, n)?;
        for (i, row) in rows.iter().enumerate() {
            if let Some(block) = self.path_blocks.last_mut()
                && block.a == row.a
                && block.b == row.b
            {
                block.end = i + 1;
                continue;
            }
            self.path_blocks.push(RowBlock {
                a: row.a,
                b: row.b,
                start: i,
                end: i + 1,
            });
        }
        resize(&mut self.path_edges, self.masses.len(), [usize::MAX; 2])?;
        self.path_edges.fill([usize::MAX; 2]);
        for (index, block) in self.path_blocks.iter().enumerate() {
            for body in [block.a, block.b] {
                let edges = &mut self.path_edges[body];
                if edges[0] == usize::MAX {
                    edges[0] = index;
                } else if edges[1] == usize::MAX {
                    edges[1] = index;
                } else {
                    return Ok(false);
                }
            }
        }
        let neighbors = |block: usize| {
            let block_data = self.path_blocks[block];
            [block_data.a, block_data.b].map(|body| {
                let edges = self.path_edges[body];
                if edges[0] == block {
                    edges[1]
                } else {
                    edges[0]
                }
            })
        };
        resize(&mut self.path_visited, self.path_blocks.len(), false)?;
        self.path_visited.fill(false);
        self.path_order.clear();
        reserve(&mut self.path_order, n)?;
        for first in 0..self.path_blocks.len() {
            if self.path_visited[first] || neighbors(first).iter().all(|v| *v != usize::MAX) {
                continue;
            }
            let mut current = first;
            let mut previous = usize::MAX;
            loop {
                if self.path_visited[current] {
                    return Ok(false);
                }
                self.path_visited[current] = true;
                let block = self.path_blocks[current];
                self.path_order.extend(block.start..block.end);
                let next = neighbors(current)
                    .into_iter()
                    .find(|next| *next != usize::MAX && *next != previous);
                let Some(next) = next else {
                    break;
                };
                previous = current;
                current = next;
            }
        }
        if self.path_order.len() != n {
            return Ok(false);
        }
        resize(&mut self.path_band, n, [0.0; BAND_WIDTH])?;
        self.path_band.fill([0.0; BAND_WIDTH]);
        resize(&mut self.path_scale, n, 0.0)?;
        resize(&mut self.path_rhs, n, 0.0)?;
        for (i, old) in self.path_order.iter().copied().enumerate() {
            let value = cross_mass(rows[old], rows[old], &self.masses)?;
            if value <= 0.0 {
                return Err(CollisionError::NonConvergent);
            }
            self.path_scale[i] = value.sqrt();
        }
        for i in 0..n {
            for j in i.saturating_sub(BAND_WIDTH - 1)..=i {
                let mut value = cross_mass(
                    rows[self.path_order[i]],
                    rows[self.path_order[j]],
                    &self.masses,
                )? / (self.path_scale[i] * self.path_scale[j]);
                for k in i.saturating_sub(BAND_WIDTH - 1)..j {
                    value -= self.path_band[i][i - k] * self.path_band[j][j - k];
                }
                if j == i {
                    if !value.is_finite() || value <= 0.0 {
                        return Err(CollisionError::NonConvergent);
                    }
                    self.path_band[i][0] = value.sqrt();
                } else {
                    self.path_band[i][i - j] = value / self.path_band[j][0];
                }
            }
        }
        let tolerance = 1.0e-12 * (1.0 + dot(&self.rhs, &self.rhs)?.sqrt());
        self.residual.copy_from_slice(&self.rhs);
        for _ in 0..4 {
            for i in 0..n {
                let mut value = self.residual[self.path_order[i]] / self.path_scale[i];
                for j in i.saturating_sub(BAND_WIDTH - 1)..i {
                    value -= self.path_band[i][i - j] * self.path_rhs[j];
                }
                self.path_rhs[i] = value / self.path_band[i][0];
            }
            for i in (0..n).rev() {
                let mut value = self.path_rhs[i];
                for j in i + 1..n.min(i + BAND_WIDTH) {
                    value -= self.path_band[j][j - i] * self.path_rhs[j];
                }
                self.path_rhs[i] = value / self.path_band[i][0];
                self.solution[self.path_order[i]] += self.path_rhs[i] / self.path_scale[i];
            }
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
            self.linear_iterations += 1;
            if dot(&self.residual, &self.residual)?.sqrt() <= tolerance {
                return Ok(true);
            }
        }
        Err(CollisionError::NonConvergent)
    }

    // Cyclic graphs can have a singular Schur operator and an off-manifold
    // residual outside its range. CGLS finds a minimum-residual reaction without
    // regularizing the physical constraints. The nonlinear solve still checks
    // every original geometric equality before accepting a step.
    fn least_squares(&mut self, start: bool) -> Result<(), CollisionError> {
        let rows = if start { &self.start_rows } else { &self.rows };
        let n = rows.len();
        resize(&mut self.normal, n, 0.0)?;
        operator(
            rows,
            &self.masses,
            &self.residual,
            &mut self.normal,
            &mut self.generalized,
        )?;
        self.direction.copy_from_slice(&self.normal);
        let mut norm = dot(&self.normal, &self.normal)?;
        let tolerance = 1.0e-14 * (1.0 + dot(&self.rhs, &self.rhs)?.sqrt());
        for _ in 0..n.saturating_mul(4).saturating_add(64) {
            if norm.sqrt() <= tolerance {
                return Ok(());
            }
            operator(
                rows,
                &self.masses,
                &self.direction,
                &mut self.product,
                &mut self.generalized,
            )?;
            let denominator = dot(&self.product, &self.product)?;
            if denominator <= 0.0 {
                return Err(CollisionError::NonConvergent);
            }
            let alpha = norm / denominator;
            for i in 0..n {
                self.solution[i] += alpha * self.direction[i];
                self.residual[i] -= alpha * self.product[i];
            }
            operator(
                rows,
                &self.masses,
                &self.residual,
                &mut self.normal,
                &mut self.generalized,
            )?;
            let next = dot(&self.normal, &self.normal)?;
            let beta = next / norm;
            for i in 0..n {
                self.direction[i] = self.normal[i] + beta * self.direction[i];
            }
            norm = next;
            self.linear_iterations += 1;
        }
        Err(CollisionError::NonConvergent)
    }

    fn linear_solve(&mut self, start: bool) -> Result<(), CollisionError> {
        let n = if start {
            self.start_rows.len()
        } else {
            self.rows.len()
        };
        resize(&mut self.solution, n, 0.0)?;
        self.solution.fill(0.0);
        resize(&mut self.residual, n, 0.0)?;
        self.residual.copy_from_slice(&self.rhs);
        resize(&mut self.direction, n, 0.0)?;
        resize(&mut self.product, n, 0.0)?;
        resize(&mut self.diagonal, n, 0.0)?;
        if self.cyclic {
            return self.least_squares(start);
        }
        if self.path_solve(start)? {
            return Ok(());
        }
        let rows = if start { &self.start_rows } else { &self.rows };
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
        let tolerance = 1.0e-12 * (1.0 + rhs_norm);
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

fn cross_mass(
    a: EqualityRow,
    b: EqualityRow,
    masses: &[MassResponse],
) -> Result<f64, CollisionError> {
    let mut value = 0.0;
    for (ai, al, aa) in [
        (a.a, a.linear_a, a.angular_a),
        (a.b, a.linear_b, a.angular_b),
    ] {
        for (bi, bl, ba) in [
            (b.a, b.linear_a, b.angular_a),
            (b.b, b.linear_b, b.angular_b),
        ] {
            if ai == bi {
                let mass = masses.get(ai).ok_or(CollisionError::InvalidContact)?;
                value += mass.linear * al.dot(bl) + aa.dot(mass.angular * ba);
            }
        }
    }
    value
        .is_finite()
        .then_some(value)
        .ok_or(CollisionError::ArithmeticOverflow)
}
