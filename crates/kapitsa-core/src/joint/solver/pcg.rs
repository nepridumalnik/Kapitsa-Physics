use super::{
    ConstraintRow, GeneralizedImpulse, IncompleteCholeskyPreconditioner, PositionPcgResult,
    RowDimension, SolverBody, SolverMass, SolverMassCacheStats, SolverMassSlot, StepExecutor,
    VelocityPcgResult, VelocityPcgScratch, VelocityRowTerm, map_rigid_body_error, row_velocity,
};
use crate::collision::CollisionError;
use kapitsa_math::Vec3;

#[cfg(feature = "bench-internals")]
#[allow(clippy::too_many_arguments)]
pub(super) fn solve_position_pcg_prepared(
    rows: &mut [ConstraintRow],
    slots: &mut [SolverMassSlot],
    bodies: &mut [SolverBody],
    max_iterations: usize,
    tolerance: crate::joint::JointTolerance,
    incomplete_cholesky_preconditioner: bool,
    scratch: &mut VelocityPcgScratch,
    cache_stats: &mut SolverMassCacheStats,
) -> Result<Option<PositionPcgResult>, CollisionError> {
    if rows.is_empty() {
        return Ok(Some(PositionPcgResult {
            converged: true,
            ..PositionPcgResult::default()
        }));
    }
    if max_iterations == 0
        || rows.iter().any(|row| {
            !row.position_enabled
                || row.compliance != 0.0
                || row.damping != 0.0
                || row.force_driven
                || row.min_multiplier != f64::NEG_INFINITY
                || row.max_multiplier != f64::INFINITY
        })
    {
        return Ok(None);
    }

    if slots.len() < bodies.len() {
        return Err(CollisionError::InvalidContact);
    }
    let preparation_started = std::time::Instant::now();
    scratch.masses.clear();
    scratch
        .masses
        .try_reserve(bodies.len())
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    for (body, slot) in bodies.iter().zip(slots) {
        let (mass, status) = slot.resolve(body.body())?;
        cache_stats.record(status);
        scratch.masses.push(mass);
    }

    resize_f64_buffer(&mut scratch.residual, rows.len())?;
    resize_f64_buffer(&mut scratch.diagonal, rows.len())?;
    for (index, row) in rows.iter().copied().enumerate() {
        if !row.is_valid(bodies.len()) {
            return Err(CollisionError::InvalidContact);
        }
        scratch.residual[index] = -row.error;
        let mass_a = *scratch
            .masses
            .get(row.body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let mass_b = *scratch
            .masses
            .get(row.body_b)
            .ok_or(CollisionError::InvalidContact)?;
        scratch.diagonal[index] = mass_a.diagonal(row.linear_a, row.angular_a)?
            + mass_b.diagonal(row.linear_b, row.angular_b)?;
        if !scratch.residual[index].is_finite() || !scratch.diagonal[index].is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if scratch.diagonal[index] <= f64::MIN_POSITIVE {
            if scratch.residual[index].abs() > position_row_tolerance(row, tolerance) {
                return Ok(None);
            }
            scratch.diagonal[index] = 1.0;
            scratch.residual[index] = 0.0;
        }
    }
    if position_vector_converged(rows, &scratch.residual, tolerance) {
        scratch.position_system_prepare_time += preparation_started.elapsed();
        return Ok(Some(position_pcg_result(rows, &scratch.residual, 0, true)));
    }

    if incomplete_cholesky_preconditioner
        && !build_incomplete_cholesky_preconditioner_into(
            rows,
            &scratch.masses,
            &scratch.diagonal,
            &mut scratch.row_terms_by_body,
            &mut scratch.lower_pattern,
            &mut scratch.preconditioner,
        )?
    {
        return Ok(None);
    }
    scratch.position_system_prepare_time += preparation_started.elapsed();

    resize_f64_buffer(&mut scratch.solution, rows.len())?;
    scratch.solution.fill(0.0);
    resize_f64_buffer(&mut scratch.preconditioned, rows.len())?;
    resize_f64_buffer(&mut scratch.direction, rows.len())?;
    resize_f64_buffer(&mut scratch.product, rows.len())?;
    let preconditioner = incomplete_cholesky_preconditioner.then_some(&scratch.preconditioner);
    let preconditioner_started = std::time::Instant::now();
    apply_velocity_preconditioner(
        &scratch.residual,
        &scratch.diagonal,
        preconditioner,
        &mut scratch.preconditioned,
    )?;
    scratch.position_preconditioner_time += preconditioner_started.elapsed();
    scratch.direction.copy_from_slice(&scratch.preconditioned);
    let reduction_started = std::time::Instant::now();
    let mut residual_product = checked_dot_product(&scratch.residual, &scratch.preconditioned)?;
    scratch.position_reduction_time += reduction_started.elapsed();
    if residual_product <= f64::MIN_POSITIVE {
        return Ok(None);
    }

    resize_impulse_buffer(&mut scratch.body_impulses, bodies.len())?;
    resize_impulse_buffer(&mut scratch.body_responses, bodies.len())?;
    let mut completed_iterations = 0;
    let mut converged = false;
    for iteration in 0..max_iterations {
        let matrix_vector_started = std::time::Instant::now();
        global_velocity_matrix_vector(
            rows,
            &scratch.direction,
            &scratch.masses,
            &mut scratch.body_impulses,
            &mut scratch.body_responses,
            &mut scratch.product,
        )?;
        scratch.position_matrix_vector_time += matrix_vector_started.elapsed();
        let reduction_started = std::time::Instant::now();
        let denominator = checked_dot_product(&scratch.direction, &scratch.product)?;
        scratch.position_reduction_time += reduction_started.elapsed();
        if denominator <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let alpha = residual_product / denominator;
        if !alpha.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        for index in 0..rows.len() {
            scratch.solution[index] += alpha * scratch.direction[index];
            scratch.residual[index] -= alpha * scratch.product[index];
            if !scratch.solution[index].is_finite() || !scratch.residual[index].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
        completed_iterations = iteration + 1;
        if position_vector_converged(rows, &scratch.residual, tolerance) {
            converged = true;
            break;
        }
        let preconditioner_started = std::time::Instant::now();
        apply_velocity_preconditioner(
            &scratch.residual,
            &scratch.diagonal,
            preconditioner,
            &mut scratch.preconditioned,
        )?;
        scratch.position_preconditioner_time += preconditioner_started.elapsed();
        let reduction_started = std::time::Instant::now();
        let next_residual_product =
            checked_dot_product(&scratch.residual, &scratch.preconditioned)?;
        scratch.position_reduction_time += reduction_started.elapsed();
        if next_residual_product <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let beta = next_residual_product / residual_product;
        if !beta.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        for index in 0..rows.len() {
            scratch.direction[index] =
                scratch.preconditioned[index] + beta * scratch.direction[index];
            if !scratch.direction[index].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
        residual_product = next_residual_product;
    }

    global_velocity_impulse(rows, &scratch.solution, &mut scratch.body_impulses)?;
    for ((body, impulse), mass) in bodies
        .iter_mut()
        .zip(scratch.body_impulses.iter().copied())
        .zip(scratch.masses.iter().copied())
    {
        if impulse.linear == Vec3::ZERO && impulse.angular == Vec3::ZERO {
            continue;
        }
        if body.body().physics().body_type() == crate::rigid_body::BodyType::Dynamic {
            body.body_mut()
                .apply_constraint_position_impulse_cached(impulse.linear, impulse.angular, mass)
                .map_err(map_rigid_body_error)?;
        }
    }
    for (row, delta) in rows.iter_mut().zip(scratch.solution.iter().copied()) {
        let next = row.accumulated_multiplier + delta;
        if !next.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        row.accumulated_multiplier = next;
    }
    Ok(Some(position_pcg_result(
        rows,
        &scratch.residual,
        completed_iterations,
        converged,
    )))
}

#[cfg(feature = "bench-internals")]
#[allow(clippy::too_many_arguments)]
pub(super) fn solve_velocity_pcg_prepared(
    rows: &mut [ConstraintRow],
    row_joint_indices: &[usize],
    bodies: &mut [SolverBody],
    max_iterations: usize,
    tolerance: crate::joint::JointTolerance,
    incomplete_cholesky_preconditioner: bool,
    block_jacobi_preconditioner: bool,
    executor: &impl StepExecutor,
    worker_count: usize,
    scratch: &mut VelocityPcgScratch,
) -> Result<Option<VelocityPcgResult>, CollisionError> {
    if rows.is_empty() {
        return Ok(Some(VelocityPcgResult {
            converged: true,
            ..VelocityPcgResult::default()
        }));
    }
    if scratch.unavailable {
        return Ok(None);
    }
    if max_iterations == 0
        || rows.iter().any(|row| {
            row.compliance != 0.0
                || row.damping != 0.0
                || row.force_driven
                || row.min_impulse != f64::NEG_INFINITY
                || row.max_impulse != f64::INFINITY
        })
    {
        scratch.unavailable = true;
        return Ok(None);
    }

    resize_f64_buffer(&mut scratch.residual, rows.len())?;
    if rows.iter().any(|row| !row.is_valid(bodies.len())) {
        return Err(CollisionError::InvalidContact);
    }
    prepare_velocity_residual_parallel(
        rows,
        bodies,
        &mut scratch.residual,
        executor,
        worker_count,
    )?;
    let system_prepare_started = std::time::Instant::now();
    if !scratch.system_prepared {
        clear_mass_buffer(&mut scratch.masses, bodies.len())?;
        for body in bodies.iter() {
            scratch.masses.push(SolverMass::from_body(body.body())?);
        }
        resize_f64_buffer(&mut scratch.diagonal, rows.len())?;
    }
    if !scratch.system_prepared {
        for (index, row) in rows.iter().copied().enumerate() {
            let mass_a = *scratch
                .masses
                .get(row.body_a)
                .ok_or(CollisionError::InvalidContact)?;
            let mass_b = *scratch
                .masses
                .get(row.body_b)
                .ok_or(CollisionError::InvalidContact)?;
            scratch.diagonal[index] = mass_a.diagonal(row.linear_a, row.angular_a)?
                + mass_b.diagonal(row.linear_b, row.angular_b)?;
            if !scratch.diagonal[index].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            if scratch.diagonal[index] <= f64::MIN_POSITIVE {
                if scratch.residual[index].abs() > velocity_row_tolerance(row, tolerance) {
                    scratch.unavailable = true;
                    return Ok(None);
                }
                scratch.diagonal[index] = 1.0;
                scratch.residual[index] = 0.0;
            }
        }
    }
    if !scratch.system_prepared {
        scratch.uses_incomplete_cholesky = incomplete_cholesky_preconditioner;
        scratch.uses_block_jacobi = block_jacobi_preconditioner;
        if incomplete_cholesky_preconditioner && block_jacobi_preconditioner {
            scratch.unavailable = true;
            return Ok(None);
        }
        if block_jacobi_preconditioner
            && !build_velocity_block_jacobi_preconditioner(
                rows,
                row_joint_indices,
                &scratch.masses,
                &mut scratch.block_jacobi,
            )?
        {
            scratch.unavailable = true;
            return Ok(None);
        }
        if incomplete_cholesky_preconditioner
            && !build_incomplete_cholesky_preconditioner_into(
                rows,
                &scratch.masses,
                &scratch.diagonal,
                &mut scratch.row_terms_by_body,
                &mut scratch.lower_pattern,
                &mut scratch.preconditioner,
            )?
        {
            scratch.unavailable = true;
            return Ok(None);
        }
        if !incomplete_cholesky_preconditioner
            && !build_velocity_row_terms_by_body(
                rows,
                scratch.masses.len(),
                &mut scratch.row_terms_by_body,
            )?
        {
            scratch.unavailable = true;
            return Ok(None);
        }
        scratch.system_prepared = true;
        scratch.system_prepare_time += system_prepare_started.elapsed();
    }
    if velocity_vector_converged_parallel(
        rows,
        &scratch.residual,
        &mut scratch.reduction_partials,
        tolerance,
        executor,
        worker_count,
    )? {
        return Ok(Some(velocity_pcg_result(rows, &scratch.residual, 0, true)));
    }

    let preconditioner = if scratch.uses_incomplete_cholesky {
        Some(&scratch.preconditioner)
    } else {
        None
    };
    resize_f64_buffer(&mut scratch.solution, rows.len())?;
    resize_f64_buffer(&mut scratch.preconditioned, rows.len())?;
    resize_f64_buffer(&mut scratch.direction, rows.len())?;
    resize_f64_buffer(&mut scratch.product, rows.len())?;
    let stage_started = std::time::Instant::now();
    apply_velocity_pcg_preconditioner(
        &scratch.residual,
        &scratch.diagonal,
        preconditioner,
        scratch.uses_block_jacobi.then_some(&scratch.block_jacobi),
        &mut scratch.preconditioned,
        executor,
        worker_count,
    )?;
    scratch.preconditioner_time += stage_started.elapsed();
    scratch.direction.copy_from_slice(&scratch.preconditioned);
    let stage_started = std::time::Instant::now();
    let mut residual_product = checked_dot_product_parallel(
        &scratch.residual,
        &scratch.preconditioned,
        &mut scratch.reduction_partials,
        executor,
        worker_count,
    )?;
    scratch.reduction_time += stage_started.elapsed();
    if residual_product <= f64::MIN_POSITIVE {
        return Ok(None);
    }

    resize_impulse_buffer(&mut scratch.body_impulses, bodies.len())?;
    resize_impulse_buffer(&mut scratch.body_responses, bodies.len())?;
    let mut completed_iterations = 0;
    let mut converged = false;
    for iteration in 0..max_iterations {
        let stage_started = std::time::Instant::now();
        global_velocity_matrix_vector_parallel(
            rows,
            &scratch.direction,
            &scratch.masses,
            &scratch.row_terms_by_body,
            &mut scratch.body_impulses,
            &mut scratch.body_responses,
            &mut scratch.product,
            executor,
            worker_count,
        )?;
        scratch.matrix_vector_time += stage_started.elapsed();
        let stage_started = std::time::Instant::now();
        let denominator = checked_dot_product_parallel(
            &scratch.direction,
            &scratch.product,
            &mut scratch.reduction_partials,
            executor,
            worker_count,
        )?;
        scratch.reduction_time += stage_started.elapsed();
        if denominator <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let alpha = residual_product / denominator;
        if !alpha.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        velocity_pcg_scaled_add(
            &mut scratch.solution,
            &scratch.direction,
            alpha,
            executor,
            worker_count,
        )?;
        velocity_pcg_scaled_add(
            &mut scratch.residual,
            &scratch.product,
            -alpha,
            executor,
            worker_count,
        )?;
        completed_iterations = iteration + 1;
        if velocity_vector_converged_parallel(
            rows,
            &scratch.residual,
            &mut scratch.reduction_partials,
            tolerance,
            executor,
            worker_count,
        )? {
            converged = true;
            break;
        }

        let stage_started = std::time::Instant::now();
        apply_velocity_pcg_preconditioner(
            &scratch.residual,
            &scratch.diagonal,
            preconditioner,
            scratch.uses_block_jacobi.then_some(&scratch.block_jacobi),
            &mut scratch.preconditioned,
            executor,
            worker_count,
        )?;
        scratch.preconditioner_time += stage_started.elapsed();
        let stage_started = std::time::Instant::now();
        let next_residual_product = checked_dot_product_parallel(
            &scratch.residual,
            &scratch.preconditioned,
            &mut scratch.reduction_partials,
            executor,
            worker_count,
        )?;
        scratch.reduction_time += stage_started.elapsed();
        if next_residual_product <= f64::MIN_POSITIVE {
            return Ok(None);
        }
        let beta = next_residual_product / residual_product;
        if !beta.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        velocity_pcg_update_direction(
            &mut scratch.direction,
            &scratch.preconditioned,
            beta,
            executor,
            worker_count,
        )?;
        residual_product = next_residual_product;
    }

    global_velocity_impulse_parallel(
        &scratch.solution,
        &scratch.row_terms_by_body,
        &mut scratch.body_impulses,
        executor,
        worker_count,
    )?;
    for (body, impulse) in bodies.iter_mut().zip(scratch.body_impulses.iter().copied()) {
        if impulse.linear == Vec3::ZERO && impulse.angular == Vec3::ZERO {
            continue;
        }
        if body.body().physics().body_type() == crate::rigid_body::BodyType::Dynamic {
            body.body_mut()
                .apply_constraint_impulse(impulse.linear, impulse.angular)
                .map_err(map_rigid_body_error)?;
        }
    }
    for (row, delta) in rows.iter_mut().zip(scratch.solution.iter().copied()) {
        let next = row.accumulated_impulse + delta;
        if !next.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        row.accumulated_impulse = next;
    }
    Ok(Some(velocity_pcg_result(
        rows,
        &scratch.residual,
        completed_iterations,
        converged,
    )))
}

#[cfg(feature = "bench-internals")]
fn build_velocity_row_terms_by_body(
    rows: &[ConstraintRow],
    body_count: usize,
    row_terms_by_body: &mut Vec<Vec<VelocityRowTerm>>,
) -> Result<bool, CollisionError> {
    resize_nested_buffer(row_terms_by_body, body_count)?;
    for (row_index, row) in rows.iter().copied().enumerate() {
        for (body_index, linear, angular) in [
            (row.body_a, row.linear_a, row.angular_a),
            (row.body_b, row.linear_b, row.angular_b),
        ] {
            let Some(terms) = row_terms_by_body.get_mut(body_index) else {
                return Ok(false);
            };
            terms.push(VelocityRowTerm {
                row: row_index,
                linear,
                angular,
            });
        }
    }
    Ok(true)
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy)]
pub(super) struct VelocityBlockFactor {
    start: usize,
    len: usize,
    lower: [f64; 36],
}

#[cfg(feature = "bench-internals")]
fn build_velocity_block_jacobi_preconditioner(
    rows: &[ConstraintRow],
    row_joint_indices: &[usize],
    masses: &[SolverMass],
    blocks: &mut Vec<VelocityBlockFactor>,
) -> Result<bool, CollisionError> {
    if row_joint_indices.len() != rows.len() {
        return Ok(false);
    }
    let block_count = row_joint_indices
        .iter()
        .enumerate()
        .filter(|(index, owner)| *index == 0 || row_joint_indices[*index - 1] != **owner)
        .count();
    if block_count > blocks.capacity() {
        blocks
            .try_reserve_exact(block_count - blocks.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    blocks.clear();
    let mut start = 0;
    while start < rows.len() {
        let owner = row_joint_indices[start];
        let end = row_joint_indices[start..]
            .iter()
            .position(|candidate| *candidate != owner)
            .map_or(rows.len(), |offset| start + offset);
        let len = end - start;
        if len == 0 || len > 6 {
            return Ok(false);
        }
        let mut block = VelocityBlockFactor {
            start,
            len,
            lower: [0.0; 36],
        };
        for row in 0..len {
            let row_i = rows[start + row];
            for column in 0..=row {
                let row_j = rows[start + column];
                if row_i.body_a != row_j.body_a || row_i.body_b != row_j.body_b {
                    return Ok(false);
                }
                let mass_a = *masses
                    .get(row_i.body_a)
                    .ok_or(CollisionError::InvalidContact)?;
                let mass_b = *masses
                    .get(row_i.body_b)
                    .ok_or(CollisionError::InvalidContact)?;
                let mut value = mass_a.cross(
                    row_i.linear_a,
                    row_i.angular_a,
                    row_j.linear_a,
                    row_j.angular_a,
                )? + mass_b.cross(
                    row_i.linear_b,
                    row_i.angular_b,
                    row_j.linear_b,
                    row_j.angular_b,
                )?;
                for previous in 0..column {
                    value -= block.lower[row * 6 + previous] * block.lower[column * 6 + previous];
                }
                if !value.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                if row == column {
                    if value <= f64::MIN_POSITIVE {
                        return Ok(false);
                    }
                    block.lower[row * 6 + column] = value.sqrt();
                } else {
                    let factor = value / block.lower[column * 6 + column];
                    if !factor.is_finite() {
                        return Ok(false);
                    }
                    block.lower[row * 6 + column] = factor;
                }
            }
        }
        blocks.push(block);
        start = end;
    }
    Ok(true)
}

#[cfg(feature = "bench-internals")]
fn apply_velocity_pcg_preconditioner(
    residual: &[f64],
    diagonal: &[f64],
    incomplete_cholesky: Option<&IncompleteCholeskyPreconditioner>,
    block_jacobi: Option<&[VelocityBlockFactor]>,
    output: &mut [f64],
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if let Some(blocks) = block_jacobi {
        apply_block_jacobi_preconditioner(residual, blocks, output)
    } else if incomplete_cholesky.is_some() {
        apply_velocity_preconditioner(residual, diagonal, incomplete_cholesky, output)
    } else {
        if residual.len() != diagonal.len() || residual.len() != output.len() {
            return Err(CollisionError::InvalidContact);
        }
        let chunk_size = velocity_pcg_chunk_size(output.len(), worker_count);
        velocity_pcg_for_each_chunk_mut(output, chunk_size, executor, |chunk_index, chunk| {
            let start = chunk_index * chunk_size;
            for (offset, value) in chunk.iter_mut().enumerate() {
                let index = start + offset;
                *value = residual[index] / diagonal[index];
            }
        })?;
        if output.iter().any(|value| !value.is_finite()) {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(())
    }
}

#[cfg(feature = "bench-internals")]
fn apply_block_jacobi_preconditioner(
    residual: &[f64],
    blocks: &[VelocityBlockFactor],
    output: &mut [f64],
) -> Result<(), CollisionError> {
    if residual.len() != output.len() {
        return Err(CollisionError::InvalidContact);
    }
    for block in blocks {
        for row in 0..block.len {
            let mut value = residual[block.start + row];
            for column in 0..row {
                value -= block.lower[row * 6 + column] * output[block.start + column];
            }
            output[block.start + row] = value / block.lower[row * 6 + row];
        }
        for row in (0..block.len).rev() {
            let mut value = output[block.start + row];
            for lower_row in row + 1..block.len {
                value -= block.lower[lower_row * 6 + row] * output[block.start + lower_row];
            }
            output[block.start + row] = value / block.lower[row * 6 + row];
            if !output[block.start + row].is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn build_incomplete_cholesky_preconditioner_into(
    rows: &[ConstraintRow],
    masses: &[SolverMass],
    diagonal: &[f64],
    row_terms_by_body: &mut Vec<Vec<VelocityRowTerm>>,
    lower_pattern: &mut Vec<Vec<(usize, f64)>>,
    preconditioner: &mut IncompleteCholeskyPreconditioner,
) -> Result<bool, CollisionError> {
    if diagonal.len() != rows.len() {
        return Ok(false);
    }
    if !build_velocity_row_terms_by_body(rows, masses.len(), row_terms_by_body)? {
        return Ok(false);
    }
    resize_nested_buffer(lower_pattern, rows.len())?;

    for (body_index, terms) in row_terms_by_body.iter().enumerate() {
        let mass = *masses
            .get(body_index)
            .ok_or(CollisionError::InvalidContact)?;
        for right in 0..terms.len() {
            for left in 0..right {
                let left_term = terms[left];
                let right_term = terms[right];
                let (row, column) = if left_term.row > right_term.row {
                    (left_term.row, right_term.row)
                } else {
                    (right_term.row, left_term.row)
                };
                let value = mass.cross(
                    left_term.linear,
                    left_term.angular,
                    right_term.linear,
                    right_term.angular,
                )?;
                if !value.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                lower_pattern[row].push((column, value));
            }
        }
    }

    for entries in lower_pattern.iter_mut() {
        entries.sort_unstable_by_key(|(column, _)| *column);
        let mut write = 0;
        for read in 0..entries.len() {
            let (column, value) = entries[read];
            if write > 0 && entries[write - 1].0 == column {
                entries[write - 1].1 += value;
                if !entries[write - 1].1.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
            } else {
                entries[write] = (column, value);
                write += 1;
            }
        }
        entries.truncate(write);
    }

    resize_nested_buffer(&mut preconditioner.lower_rows, rows.len())?;
    resize_nested_buffer(&mut preconditioner.lower_columns, rows.len())?;
    resize_f64_buffer(&mut preconditioner.diagonal, rows.len())?;

    for row in 0..rows.len() {
        for &(column, matrix_value) in &lower_pattern[row] {
            let mut value = matrix_value;
            let row_factors = &preconditioner.lower_rows[row];
            let column_factors = &preconditioner.lower_rows[column];
            let (mut row_factor_index, mut column_factor_index) = (0, 0);
            while row_factor_index < row_factors.len() && column_factor_index < column_factors.len()
            {
                let (row_column, row_factor) = row_factors[row_factor_index];
                if row_column >= column {
                    break;
                }
                let (column_column, column_factor) = column_factors[column_factor_index];
                match row_column.cmp(&column_column) {
                    std::cmp::Ordering::Less => row_factor_index += 1,
                    std::cmp::Ordering::Greater => column_factor_index += 1,
                    std::cmp::Ordering::Equal => {
                        value -= row_factor * column_factor;
                        row_factor_index += 1;
                        column_factor_index += 1;
                    }
                }
            }
            let factor = value / preconditioner.diagonal[column];
            if !factor.is_finite() {
                return Ok(false);
            }
            preconditioner.lower_rows[row].push((column, factor));
            preconditioner.lower_columns[column].push((row, factor));
        }
        let mut diagonal_value = diagonal[row];
        for &(_, factor) in &preconditioner.lower_rows[row] {
            diagonal_value -= factor * factor;
        }
        if !diagonal_value.is_finite() || diagonal_value <= f64::MIN_POSITIVE {
            return Ok(false);
        }
        preconditioner.diagonal[row] = diagonal_value.sqrt();
    }
    Ok(true)
}

#[cfg(feature = "bench-internals")]
fn apply_velocity_preconditioner(
    residual: &[f64],
    diagonal: &[f64],
    preconditioner: Option<&IncompleteCholeskyPreconditioner>,
    output: &mut [f64],
) -> Result<(), CollisionError> {
    if residual.len() != diagonal.len() || residual.len() != output.len() {
        return Err(CollisionError::InvalidContact);
    }
    let Some(preconditioner) = preconditioner else {
        for index in 0..residual.len() {
            output[index] = residual[index] / diagonal[index];
        }
        return Ok(());
    };
    if preconditioner.diagonal.len() != residual.len() {
        return Err(CollisionError::InvalidContact);
    }
    for row in 0..residual.len() {
        let mut value = residual[row];
        for &(column, factor) in &preconditioner.lower_rows[row] {
            value -= factor * output[column];
        }
        output[row] = value / preconditioner.diagonal[row];
        if !output[row].is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
    }
    for row in (0..residual.len()).rev() {
        let mut value = output[row];
        for &(lower_row, factor) in &preconditioner.lower_columns[row] {
            value -= factor * output[lower_row];
        }
        output[row] = value / preconditioner.diagonal[row];
        if !output[row].is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn global_velocity_matrix_vector(
    rows: &[ConstraintRow],
    vector: &[f64],
    masses: &[SolverMass],
    body_impulses: &mut [GeneralizedImpulse],
    body_responses: &mut [GeneralizedImpulse],
    result: &mut [f64],
) -> Result<(), CollisionError> {
    if rows.len() != vector.len() || rows.len() != result.len() {
        return Err(CollisionError::InvalidContact);
    }
    body_impulses.fill(GeneralizedImpulse::default());
    body_responses.fill(GeneralizedImpulse::default());
    for (row, coefficient) in rows.iter().copied().zip(vector.iter().copied()) {
        add_generalized_impulse(
            body_impulses
                .get_mut(row.body_a)
                .ok_or(CollisionError::InvalidContact)?,
            row.linear_a,
            row.angular_a,
            coefficient,
        )?;
        add_generalized_impulse(
            body_impulses
                .get_mut(row.body_b)
                .ok_or(CollisionError::InvalidContact)?,
            row.linear_b,
            row.angular_b,
            coefficient,
        )?;
    }
    for index in 0..masses.len() {
        let mass = masses[index];
        let impulse = body_impulses[index];
        body_responses[index] = GeneralizedImpulse {
            linear: impulse.linear * mass.inverse_mass,
            angular: mass
                .inverse_inertia
                .checked_mul_vector(impulse.angular)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        };
        if !body_responses[index].linear.is_finite() || !body_responses[index].angular.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
    }
    for (index, row) in rows.iter().copied().enumerate() {
        let response_a = body_responses
            .get(row.body_a)
            .ok_or(CollisionError::InvalidContact)?;
        let response_b = body_responses
            .get(row.body_b)
            .ok_or(CollisionError::InvalidContact)?;
        let value = row
            .linear_a
            .checked_dot(response_a.linear)
            .ok_or(CollisionError::ArithmeticOverflow)?
            + row
                .angular_a
                .checked_dot(response_a.angular)
                .ok_or(CollisionError::ArithmeticOverflow)?
            + row
                .linear_b
                .checked_dot(response_b.linear)
                .ok_or(CollisionError::ArithmeticOverflow)?
            + row
                .angular_b
                .checked_dot(response_b.angular)
                .ok_or(CollisionError::ArithmeticOverflow)?;
        if !value.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        result[index] = value;
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn global_velocity_impulse(
    rows: &[ConstraintRow],
    vector: &[f64],
    body_impulses: &mut [GeneralizedImpulse],
) -> Result<(), CollisionError> {
    if rows.len() != vector.len() {
        return Err(CollisionError::InvalidContact);
    }
    body_impulses.fill(GeneralizedImpulse::default());
    for (row, coefficient) in rows.iter().copied().zip(vector.iter().copied()) {
        add_generalized_impulse(
            body_impulses
                .get_mut(row.body_a)
                .ok_or(CollisionError::InvalidContact)?,
            row.linear_a,
            row.angular_a,
            coefficient,
        )?;
        add_generalized_impulse(
            body_impulses
                .get_mut(row.body_b)
                .ok_or(CollisionError::InvalidContact)?,
            row.linear_b,
            row.angular_b,
            coefficient,
        )?;
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
#[allow(clippy::too_many_arguments)]
fn global_velocity_matrix_vector_parallel(
    rows: &[ConstraintRow],
    vector: &[f64],
    masses: &[SolverMass],
    row_terms_by_body: &[Vec<VelocityRowTerm>],
    body_impulses: &mut [GeneralizedImpulse],
    body_responses: &mut [GeneralizedImpulse],
    result: &mut [f64],
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if rows.len() != vector.len()
        || rows.len() != result.len()
        || masses.len() != row_terms_by_body.len()
        || masses.len() != body_impulses.len()
        || masses.len() != body_responses.len()
    {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(masses.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(body_impulses, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, impulse) in chunk.iter_mut().enumerate() {
            let mut sum = GeneralizedImpulse::default();
            for term in &row_terms_by_body[start + offset] {
                let coefficient = vector[term.row];
                sum.linear += term.linear * coefficient;
                sum.angular += term.angular * coefficient;
            }
            *impulse = sum;
        }
    })?;
    if body_impulses
        .iter()
        .any(|impulse| !impulse.linear.is_finite() || !impulse.angular.is_finite())
    {
        return Err(CollisionError::ArithmeticOverflow);
    }

    velocity_pcg_for_each_chunk_mut(
        body_responses,
        chunk_size,
        executor,
        |chunk_index, chunk| {
            let start = chunk_index * chunk_size;
            for (offset, response) in chunk.iter_mut().enumerate() {
                let index = start + offset;
                let mass = masses[index];
                let impulse = body_impulses[index];
                *response = GeneralizedImpulse {
                    linear: impulse.linear * mass.inverse_mass,
                    angular: mass
                        .inverse_inertia
                        .checked_mul_vector(impulse.angular)
                        .unwrap_or(Vec3::new(f64::NAN, f64::NAN, f64::NAN)),
                };
            }
        },
    )?;
    if body_responses
        .iter()
        .any(|response| !response.linear.is_finite() || !response.angular.is_finite())
    {
        return Err(CollisionError::ArithmeticOverflow);
    }

    let chunk_size = velocity_pcg_chunk_size(rows.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(result, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, value) in chunk.iter_mut().enumerate() {
            let row = rows[start + offset];
            let response_a = body_responses[row.body_a];
            let response_b = body_responses[row.body_b];
            *value = row
                .linear_a
                .checked_dot(response_a.linear)
                .unwrap_or(f64::NAN)
                + row
                    .angular_a
                    .checked_dot(response_a.angular)
                    .unwrap_or(f64::NAN)
                + row
                    .linear_b
                    .checked_dot(response_b.linear)
                    .unwrap_or(f64::NAN)
                + row
                    .angular_b
                    .checked_dot(response_b.angular)
                    .unwrap_or(f64::NAN);
        }
    })?;
    if result.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn global_velocity_impulse_parallel(
    vector: &[f64],
    row_terms_by_body: &[Vec<VelocityRowTerm>],
    body_impulses: &mut [GeneralizedImpulse],
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if body_impulses.len() != row_terms_by_body.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(body_impulses.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(body_impulses, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, impulse) in chunk.iter_mut().enumerate() {
            let mut sum = GeneralizedImpulse::default();
            for term in &row_terms_by_body[start + offset] {
                let coefficient = vector[term.row];
                sum.linear += term.linear * coefficient;
                sum.angular += term.angular * coefficient;
            }
            *impulse = sum;
        }
    })?;
    if body_impulses
        .iter()
        .any(|impulse| !impulse.linear.is_finite() || !impulse.angular.is_finite())
    {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
const PCG_MIN_PARALLEL_CHUNK: usize = 256;

#[cfg(feature = "bench-internals")]
fn velocity_pcg_chunk_size(length: usize, worker_count: usize) -> usize {
    if worker_count <= 1 || length < PCG_MIN_PARALLEL_CHUNK {
        return length.max(1);
    }
    let task_count = worker_count.min((length / PCG_MIN_PARALLEL_CHUNK).max(1));
    length.div_ceil(task_count)
}

#[cfg(feature = "bench-internals")]
fn velocity_pcg_for_each_chunk_mut<T, F>(
    values: &mut [T],
    chunk_size: usize,
    executor: &impl StepExecutor,
    operation: F,
) -> Result<(), CollisionError>
where
    T: Send,
    F: Fn(usize, &mut [T]) + Send + Sync,
{
    if values.is_empty() {
        return Ok(());
    }
    if chunk_size >= values.len() {
        operation(0, values);
        return Ok(());
    }
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        executor.for_each_chunk_mut(values, chunk_size, operation)
    }));
    if matches!(result, Ok(Ok(()))) {
        Ok(())
    } else {
        Err(CollisionError::WorkerFailed)
    }
}

#[cfg(feature = "bench-internals")]
fn velocity_pcg_scaled_add(
    destination: &mut [f64],
    source: &[f64],
    scale: f64,
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if destination.len() != source.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(destination.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(destination, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, value) in chunk.iter_mut().enumerate() {
            *value += scale * source[start + offset];
        }
    })?;
    if destination.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn velocity_pcg_update_direction(
    direction: &mut [f64],
    preconditioned: &[f64],
    beta: f64,
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if direction.len() != preconditioned.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(direction.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(direction, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, value) in chunk.iter_mut().enumerate() {
            *value = preconditioned[start + offset] + beta * *value;
        }
    })?;
    if direction.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn add_generalized_impulse(
    impulse: &mut GeneralizedImpulse,
    linear: Vec3,
    angular: Vec3,
    coefficient: f64,
) -> Result<(), CollisionError> {
    impulse.linear += linear * coefficient;
    impulse.angular += angular * coefficient;
    if impulse.linear.is_finite() && impulse.angular.is_finite() {
        Ok(())
    } else {
        Err(CollisionError::ArithmeticOverflow)
    }
}

#[cfg(feature = "bench-internals")]
fn velocity_vector_converged(
    rows: &[ConstraintRow],
    residual: &[f64],
    tolerance: crate::joint::JointTolerance,
) -> bool {
    rows.iter()
        .zip(residual)
        .all(|(row, value)| value.abs() <= velocity_row_tolerance(*row, tolerance))
}

#[cfg(feature = "bench-internals")]
fn velocity_vector_converged_parallel(
    rows: &[ConstraintRow],
    residual: &[f64],
    partial_results: &mut Vec<f64>,
    tolerance: crate::joint::JointTolerance,
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<bool, CollisionError> {
    if rows.len() != residual.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(rows.len(), worker_count);
    if chunk_size >= rows.len().max(1) {
        return Ok(velocity_vector_converged(rows, residual, tolerance));
    }
    let task_count = rows.len().div_ceil(chunk_size);
    resize_f64_buffer(partial_results, task_count)?;
    velocity_pcg_for_each_chunk_mut(partial_results, 1, executor, |task_index, output| {
        let start = task_index * chunk_size;
        let end = (start + chunk_size).min(rows.len());
        output[0] = if rows[start..end]
            .iter()
            .zip(&residual[start..end])
            .all(|(row, value)| value.abs() <= velocity_row_tolerance(*row, tolerance))
        {
            1.0
        } else {
            0.0
        };
    })?;
    Ok(partial_results[..task_count]
        .iter()
        .all(|result| *result == 1.0))
}

#[cfg(feature = "bench-internals")]
fn velocity_pcg_result(
    rows: &[ConstraintRow],
    residual: &[f64],
    iterations: usize,
    converged: bool,
) -> VelocityPcgResult {
    let mut result = VelocityPcgResult {
        iterations,
        converged,
        ..VelocityPcgResult::default()
    };
    for (row, residual) in rows.iter().zip(residual) {
        match row.dimension {
            RowDimension::Linear => {
                result.linear_residual_mps = result.linear_residual_mps.max(residual.abs())
            }
            RowDimension::Angular => {
                result.angular_residual_radps = result.angular_residual_radps.max(residual.abs())
            }
        }
    }
    result
}

#[cfg(feature = "bench-internals")]
fn position_pcg_result(
    rows: &[ConstraintRow],
    residual: &[f64],
    iterations: usize,
    converged: bool,
) -> PositionPcgResult {
    let mut result = PositionPcgResult {
        iterations,
        converged,
        ..PositionPcgResult::default()
    };
    for (row, residual) in rows.iter().zip(residual) {
        match row.dimension {
            RowDimension::Linear => {
                result.linear_residual_m = result.linear_residual_m.max(residual.abs())
            }
            RowDimension::Angular => {
                result.angular_residual_rad = result.angular_residual_rad.max(residual.abs())
            }
        }
    }
    result
}

#[cfg(feature = "bench-internals")]
fn prepare_velocity_residual_parallel(
    rows: &[ConstraintRow],
    bodies: &[SolverBody],
    residual: &mut [f64],
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<(), CollisionError> {
    if rows.len() != residual.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(rows.len(), worker_count);
    velocity_pcg_for_each_chunk_mut(residual, chunk_size, executor, |chunk_index, chunk| {
        let start = chunk_index * chunk_size;
        for (offset, value) in chunk.iter_mut().enumerate() {
            let row = rows[start + offset];
            *value = row.target_velocity - row_velocity(bodies, row).unwrap_or(f64::NAN);
        }
    })?;
    if residual.iter().any(|value| !value.is_finite()) {
        return Err(CollisionError::ArithmeticOverflow);
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn velocity_row_tolerance(row: ConstraintRow, tolerance: crate::joint::JointTolerance) -> f64 {
    match row.dimension {
        RowDimension::Linear => tolerance.velocity_mps,
        RowDimension::Angular => tolerance.angular_velocity_radps,
    }
}

#[cfg(feature = "bench-internals")]
fn position_vector_converged(
    rows: &[ConstraintRow],
    residual: &[f64],
    tolerance: crate::joint::JointTolerance,
) -> bool {
    rows.iter()
        .zip(residual)
        .all(|(row, value)| value.abs() <= position_row_tolerance(*row, tolerance))
}

#[cfg(feature = "bench-internals")]
fn position_row_tolerance(row: ConstraintRow, tolerance: crate::joint::JointTolerance) -> f64 {
    match row.dimension {
        RowDimension::Linear => tolerance.linear_m,
        RowDimension::Angular => tolerance.angular_rad,
    }
}

#[cfg(feature = "bench-internals")]
fn checked_dot_product(left: &[f64], right: &[f64]) -> Result<f64, CollisionError> {
    if left.len() != right.len() {
        return Err(CollisionError::InvalidContact);
    }
    left.iter().zip(right).try_fold(0.0, |sum, (left, right)| {
        let next = sum + left * right;
        next.is_finite()
            .then_some(next)
            .ok_or(CollisionError::ArithmeticOverflow)
    })
}

#[cfg(feature = "bench-internals")]
fn checked_dot_product_parallel(
    left: &[f64],
    right: &[f64],
    partial_sums: &mut Vec<f64>,
    executor: &impl StepExecutor,
    worker_count: usize,
) -> Result<f64, CollisionError> {
    if left.len() != right.len() {
        return Err(CollisionError::InvalidContact);
    }
    let chunk_size = velocity_pcg_chunk_size(left.len(), worker_count);
    if chunk_size >= left.len().max(1) {
        return checked_dot_product(left, right);
    }
    let task_count = left.len().div_ceil(chunk_size);
    resize_f64_buffer(partial_sums, task_count)?;
    velocity_pcg_for_each_chunk_mut(partial_sums, 1, executor, |task_index, output| {
        let start = task_index * chunk_size;
        let end = (start + chunk_size).min(left.len());
        let mut sum = 0.0;
        for index in start..end {
            sum += left[index] * right[index];
        }
        output[0] = sum;
    })?;
    partial_sums[..task_count]
        .iter()
        .try_fold(0.0, |sum, partial| {
            let next = sum + partial;
            next.is_finite()
                .then_some(next)
                .ok_or(CollisionError::ArithmeticOverflow)
        })
}

#[cfg(feature = "bench-internals")]
fn zero_f64_buffer(length: usize) -> Result<Vec<f64>, CollisionError> {
    let mut buffer = Vec::new();
    buffer
        .try_reserve_exact(length)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    buffer.resize(length, 0.0);
    Ok(buffer)
}

#[cfg(feature = "bench-internals")]
fn zero_impulse_buffer(length: usize) -> Result<Vec<GeneralizedImpulse>, CollisionError> {
    let mut buffer = Vec::new();
    buffer
        .try_reserve_exact(length)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    buffer.resize(length, GeneralizedImpulse::default());
    Ok(buffer)
}

#[cfg(feature = "bench-internals")]
fn resize_f64_buffer(buffer: &mut Vec<f64>, length: usize) -> Result<(), CollisionError> {
    if length > buffer.capacity() {
        buffer
            .try_reserve_exact(length - buffer.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    buffer.resize(length, 0.0);
    buffer.fill(0.0);
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn resize_impulse_buffer(
    buffer: &mut Vec<GeneralizedImpulse>,
    length: usize,
) -> Result<(), CollisionError> {
    if length > buffer.capacity() {
        buffer
            .try_reserve_exact(length - buffer.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    buffer.resize(length, GeneralizedImpulse::default());
    buffer.fill(GeneralizedImpulse::default());
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn clear_mass_buffer(buffer: &mut Vec<SolverMass>, length: usize) -> Result<(), CollisionError> {
    buffer.clear();
    if length > buffer.capacity() {
        buffer
            .try_reserve_exact(length)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    }
    Ok(())
}

#[cfg(feature = "bench-internals")]
fn resize_nested_buffer<T>(buffer: &mut Vec<Vec<T>>, length: usize) -> Result<(), CollisionError> {
    if length > buffer.len() {
        buffer
            .try_reserve_exact(length - buffer.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        buffer.resize_with(length, Vec::new);
    } else {
        buffer.truncate(length);
    }
    for entries in buffer {
        entries.clear();
    }
    Ok(())
}
