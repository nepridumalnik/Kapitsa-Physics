use kapitsa_math::Vec3;

use crate::{
    collision::CollisionError,
    constraint::SolverBody,
    rigid_body::{
        BodyType,
        solver_mass::{SolverMass, SolverMassSlot},
    },
};

use super::{
    row::ConstraintRow,
    solver::{SolverMassCacheStats, map_rigid_body_error, row_velocity},
};

type RowTerm = (usize, Vec3, Vec3);
type MatrixEntry = (usize, f64);

/// Reused sparse factorization buffers for hard bilateral path and cycle islands.
#[derive(Default)]
pub(super) struct DirectVelocityScratch {
    neighbors: Vec<[usize; 2]>,
    terms: Vec<Vec<RowTerm>>,
    matrix: Vec<Vec<MatrixEntry>>,
    remaining: Vec<MatrixEntry>,
    masses: Vec<SolverMass>,
    rhs: Vec<f64>,
    diagonal: Vec<f64>,
    solution: Vec<f64>,
    generalized: Vec<(Vec3, Vec3)>,
    candidate: Vec<SolverBody>,
}

impl DirectVelocityScratch {
    pub(super) const fn new() -> Self {
        Self {
            neighbors: Vec::new(),
            terms: Vec::new(),
            matrix: Vec::new(),
            remaining: Vec::new(),
            masses: Vec::new(),
            rhs: Vec::new(),
            diagonal: Vec::new(),
            solution: Vec::new(),
            generalized: Vec::new(),
            candidate: Vec::new(),
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(super) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;
        self.neighbors.capacity() * size_of::<[usize; 2]>()
            + self.terms.capacity() * size_of::<Vec<RowTerm>>()
            + self
                .terms
                .iter()
                .map(|entries| entries.capacity() * size_of::<RowTerm>())
                .sum::<usize>()
            + self.matrix.capacity() * size_of::<Vec<MatrixEntry>>()
            + self
                .matrix
                .iter()
                .map(|entries| entries.capacity() * size_of::<MatrixEntry>())
                .sum::<usize>()
            + self.remaining.capacity() * size_of::<MatrixEntry>()
            + self.masses.capacity() * size_of::<SolverMass>()
            + (self.rhs.capacity() + self.diagonal.capacity() + self.solution.capacity())
                * size_of::<f64>()
            + self.generalized.capacity() * size_of::<(Vec3, Vec3)>()
            + self.candidate.capacity() * size_of::<SolverBody>()
    }

    pub(super) fn solve(
        &mut self,
        rows: &mut [ConstraintRow],
        bodies: &mut [SolverBody],
        slots: &mut [SolverMassSlot],
        mass_cache: bool,
        cache_stats: &mut SolverMassCacheStats,
    ) -> Result<bool, CollisionError> {
        if rows.is_empty()
            || rows.iter().any(|row| {
                !row.position_enabled
                    || row.compliance != 0.0
                    || row.damping != 0.0
                    || row.force_driven
                    || row.target_velocity != 0.0
                    || row.min_impulse != f64::NEG_INFINITY
                    || row.max_impulse != f64::INFINITY
            })
        {
            return Ok(false);
        }
        grow(&mut self.neighbors, bodies.len())?;
        self.neighbors.resize(bodies.len(), [usize::MAX; 2]);
        self.neighbors.fill([usize::MAX; 2]);
        reset_nested(&mut self.terms, bodies.len())?;
        for (index, row) in rows.iter().enumerate() {
            if !row.is_valid(bodies.len()) {
                return Err(CollisionError::InvalidContact);
            }
            for (body, other, linear, angular) in [
                (row.body_a, row.body_b, row.linear_a, row.angular_a),
                (row.body_b, row.body_a, row.linear_b, row.angular_b),
            ] {
                let neighbors = &mut self.neighbors[body];
                if !neighbors.contains(&other) {
                    let Some(slot) = neighbors
                        .iter_mut()
                        .find(|neighbor| **neighbor == usize::MAX)
                    else {
                        return Ok(false);
                    };
                    *slot = other;
                }
                let terms = &mut self.terms[body];
                // Two hard joints have at most twelve locked rows per body.
                // This also avoids unbounded fill for repeated parallel joints.
                if terms.len() == 12 {
                    return Ok(false);
                }
                grow(terms, terms.len() + 1)?;
                terms.push((index, linear, angular));
            }
        }
        // A single body pair already uses the local solver. This path addresses
        // propagation across at least two connected joints.
        if !self
            .neighbors
            .iter()
            .any(|neighbors| neighbors[1] != usize::MAX)
        {
            return Ok(false);
        }
        self.masses.clear();
        grow(&mut self.masses, bodies.len())?;
        for (index, body) in bodies.iter().enumerate() {
            let mass = if mass_cache {
                let (mass, status) = slots
                    .get_mut(index)
                    .ok_or(CollisionError::InvalidContact)?
                    .resolve(body.body())?;
                cache_stats.record(status);
                mass
            } else {
                SolverMass::from_body(body.body())?
            };
            self.masses.push(mass);
        }
        reset_nested(&mut self.matrix, rows.len())?;
        self.rhs.clear();
        grow(&mut self.rhs, rows.len())?;
        for row in rows.iter() {
            self.rhs
                .push(row.target_velocity - row_velocity(&bodies[..], *row)?);
        }
        for (mass, body_terms) in self.masses.iter().zip(&self.terms) {
            for (i, &(row, linear, angular)) in body_terms.iter().enumerate() {
                for &(column, other_linear, other_angular) in &body_terms[i..] {
                    let value = mass.cross(linear, angular, other_linear, other_angular)?;
                    add_entry(&mut self.matrix[row], column, value)?;
                }
            }
        }
        grow(&mut self.diagonal, rows.len())?;
        self.diagonal.resize(rows.len(), 0.0);
        for row in 0..rows.len() {
            let Some(&(column, pivot)) = self.matrix[row].first() else {
                return Ok(false);
            };
            if column != row {
                return Ok(false);
            }
            if !pivot.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            if pivot <= f64::MIN_POSITIVE {
                return Ok(false);
            }
            self.diagonal[row] = pivot;
            self.remaining.clear();
            grow(
                &mut self.remaining,
                self.matrix[row].len().saturating_sub(1),
            )?;
            self.remaining.extend_from_slice(&self.matrix[row][1..]);
            for (i, &(target, value)) in self.remaining.iter().enumerate() {
                let factor = value / pivot;
                self.rhs[target] -= factor * self.rhs[row];
                if !factor.is_finite() || !self.rhs[target].is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                for &(column, other) in &self.remaining[i..] {
                    add_entry(&mut self.matrix[target], column, -factor * other)?;
                }
            }
        }
        grow(&mut self.solution, rows.len())?;
        self.solution.resize(rows.len(), 0.0);
        self.solution.fill(0.0);
        for row in (0..rows.len()).rev() {
            let mut cross = 0.0;
            for &(column, value) in &self.matrix[row][1..] {
                cross += value * self.solution[column];
            }
            self.solution[row] = (self.rhs[row] - cross) / self.diagonal[row];
            if !self.solution[row].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
        grow(&mut self.generalized, bodies.len())?;
        self.generalized
            .resize(bodies.len(), (Vec3::ZERO, Vec3::ZERO));
        self.generalized.fill((Vec3::ZERO, Vec3::ZERO));
        for (row, &impulse) in rows.iter().zip(&self.solution) {
            self.generalized[row.body_a].0 += row.linear_a * impulse;
            self.generalized[row.body_a].1 += row.angular_a * impulse;
            self.generalized[row.body_b].0 += row.linear_b * impulse;
            self.generalized[row.body_b].1 += row.angular_b * impulse;
            if !(row.accumulated_impulse + impulse).is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
        self.candidate.clear();
        grow(&mut self.candidate, bodies.len())?;
        self.candidate.extend_from_slice(bodies);
        for ((body, &(linear, angular)), &mass) in self
            .candidate
            .iter_mut()
            .zip(&self.generalized)
            .zip(&self.masses)
        {
            if body.body().physics().body_type() == BodyType::Dynamic {
                body.body_mut()
                    .apply_constraint_impulse_cached(linear, angular, mass)
                    .map_err(map_rigid_body_error)?;
            }
        }
        // Do not commit an ill-conditioned solution. Fallback leaves both the
        // bodies and their accumulated reactions untouched.
        for row in rows.iter() {
            let residual = row_velocity(&self.candidate[..], *row)? - row.target_velocity;
            if residual.abs() > 1.0e-9 * (1.0 + row.target_velocity.abs()) {
                return Ok(false);
            }
        }
        // In a hard, unpowered island with stationary boundaries the exact
        // velocity projection is dissipative. Reject numerically unstable
        // factorizations before they can introduce kinetic energy.
        if bodies
            .iter()
            .all(|body| body.body().physics().body_type() != BodyType::Kinematic)
        {
            let before = kinetic_energy(bodies)?;
            let after = kinetic_energy(&self.candidate)?;
            let roundoff = 64.0
                * f64::EPSILON
                * (bodies.len() as f64 + 1.0)
                * (1.0 + before.abs() + after.abs());
            if after > before + roundoff {
                return Ok(false);
            }
        }
        for (row, &impulse) in rows.iter_mut().zip(&self.solution) {
            row.accumulated_impulse += impulse;
        }
        bodies.copy_from_slice(&self.candidate);
        Ok(true)
    }
}

fn kinetic_energy(bodies: &[SolverBody]) -> Result<f64, CollisionError> {
    let mut total = 0.0;
    for body in bodies {
        if body.body().physics().body_type() != BodyType::Dynamic {
            continue;
        }
        let mass = body
            .body()
            .physics()
            .mass()
            .ok_or(CollisionError::InvalidContact)?;
        let state = body.state();
        let omega = state
            .transform()
            .rotation()
            .conjugate()
            .rotate_vector(state.angular_velocity())
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let spin = mass
            .inertia()
            .checked_mul_vector(omega)
            .and_then(|momentum| omega.checked_dot(momentum))
            .ok_or(CollisionError::ArithmeticOverflow)?;
        let speed_squared = state
            .linear_velocity()
            .checked_dot(state.linear_velocity())
            .ok_or(CollisionError::ArithmeticOverflow)?;
        total += 0.5 * (mass.mass() * speed_squared + spin);
        if !total.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
    }
    Ok(total)
}

fn grow<T>(values: &mut Vec<T>, required: usize) -> Result<(), CollisionError> {
    if values.capacity() < required {
        values
            .try_reserve(required - values.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    Ok(())
}

fn reset_nested<T>(values: &mut Vec<Vec<T>>, required: usize) -> Result<(), CollisionError> {
    grow(values, required)?;
    if values.len() < required {
        values.resize_with(required, Vec::new);
    }
    for value in values.iter_mut() {
        value.clear();
    }
    Ok(())
}

fn add_entry(
    entries: &mut Vec<MatrixEntry>,
    column: usize,
    value: f64,
) -> Result<(), CollisionError> {
    match entries.binary_search_by_key(&column, |entry| entry.0) {
        Ok(index) => {
            entries[index].1 += value;
            if !entries[index].1.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
        Err(index) => {
            if !value.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            grow(entries, entries.len() + 1)?;
            entries.insert(index, (column, value));
        }
    }
    Ok(())
}
