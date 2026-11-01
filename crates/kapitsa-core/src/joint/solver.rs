use super::{row::ConstraintRow, workspace::JointWorkspace};
use crate::{
    collision::{CollisionError, solver::SolverBody},
    constraint,
};

/// Shared sequential Gauss–Seidel/XPBD solver for prepared joint rows.
pub(crate) struct JointSolver;

impl JointSolver {
    pub(crate) fn solve<F>(
        workspace: &mut JointWorkspace,
        bodies: &mut [SolverBody],
        dt: f64,
        position_iterations: u32,
        velocity_iterations: u32,
        mut prepare_rows: F,
    ) -> Result<(), CollisionError>
    where
        F: FnMut(&[SolverBody], &mut JointWorkspace) -> Result<(), CollisionError>,
    {
        if !dt.is_finite() || dt <= 0.0 || position_iterations == 0 || velocity_iterations == 0 {
            return Err(CollisionError::InvalidConfig);
        }
        workspace.begin_step();
        for _ in 0..position_iterations {
            workspace.begin_prepare();
            prepare_rows(bodies, workspace)?;
            workspace.finish_prepare();
            solve_position_iteration(workspace, bodies, dt)?;
        }
        for _ in 0..velocity_iterations {
            workspace.begin_prepare();
            prepare_rows(bodies, workspace)?;
            workspace.finish_prepare();
            solve_velocity_iteration(workspace, bodies, dt)?;
        }
        Ok(())
    }
}

fn solve_position_iteration(
    workspace: &mut JointWorkspace,
    bodies: &mut [SolverBody],
    dt: f64,
) -> Result<(), CollisionError> {
    let dt_squared = dt * dt;
    if !dt_squared.is_finite() || dt_squared <= 0.0 {
        return Err(CollisionError::ArithmeticOverflow);
    }
    let alpha_scale = dt_squared.recip();
    for index in 0..workspace.rows().len() {
        let row = *workspace
            .rows()
            .get(index)
            .ok_or(CollisionError::InvalidContact)?;
        if !row.is_valid(bodies.len()) {
            return Err(CollisionError::InvalidContact);
        }
        let effective_mass = effective_mass_for_row(bodies, row)?;
        let alpha = row.compliance * alpha_scale;
        if !alpha.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let denominator = effective_mass + alpha;
        if denominator <= f64::MIN_POSITIVE {
            continue;
        }
        let delta = -(row.error + alpha * row.accumulated_multiplier) / denominator;
        let next =
            (row.accumulated_multiplier + delta).clamp(row.min_multiplier, row.max_multiplier);
        let applied = next - row.accumulated_multiplier;
        if !next.is_finite() || !applied.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if applied != 0.0 {
            apply_position_row(bodies, row, applied)?;
            workspace.rows_mut()[index].accumulated_multiplier = next;
        }
    }
    Ok(())
}

fn solve_velocity_iteration(
    workspace: &mut JointWorkspace,
    bodies: &mut [SolverBody],
    dt: f64,
) -> Result<(), CollisionError> {
    for index in 0..workspace.rows().len() {
        let row = *workspace
            .rows()
            .get(index)
            .ok_or(CollisionError::InvalidContact)?;
        if !row.is_valid(bodies.len()) {
            return Err(CollisionError::InvalidContact);
        }
        let effective_mass = effective_mass_for_row(bodies, row)?;
        if effective_mass <= f64::MIN_POSITIVE || row.damping == 0.0 {
            continue;
        }
        let velocity = row_velocity(bodies, row)?;
        let speed_error = row.target_velocity - velocity;
        let scaled_damping = row.damping * dt;
        let denominator = 1.0_f64 + scaled_damping * effective_mass;
        let delta = scaled_damping * speed_error / denominator;
        let next = (row.accumulated_impulse + delta).clamp(row.min_impulse, row.max_impulse);
        let applied = next - row.accumulated_impulse;
        if !denominator.is_finite() || !next.is_finite() || !applied.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        if applied != 0.0 {
            apply_velocity_row(bodies, row, applied)?;
            workspace.rows_mut()[index].accumulated_impulse = next;
        }
    }
    Ok(())
}

fn effective_mass_for_row(
    bodies: &[SolverBody],
    row: ConstraintRow,
) -> Result<f64, CollisionError> {
    let (body_a, body_b) = solver_bodies_at(bodies, row.body_a, row.body_b)?;
    let jacobians = [
        constraint::Jacobian {
            linear: row.linear_a,
            angular: row.angular_a,
        },
        constraint::Jacobian {
            linear: row.linear_b,
            angular: row.angular_b,
        },
    ];
    constraint::effective_cross_mass(body_a.body(), body_b.body(), jacobians, jacobians)
        .map_err(map_constraint_error)
}

fn row_velocity(bodies: &[SolverBody], row: ConstraintRow) -> Result<f64, CollisionError> {
    let (body_a, body_b) = solver_bodies_at(bodies, row.body_a, row.body_b)?;
    let state_a = body_a.state();
    let state_b = body_b.state();
    let value = row.linear_a.dot(state_a.linear_velocity())
        + row.angular_a.dot(state_a.angular_velocity())
        + row.linear_b.dot(state_b.linear_velocity())
        + row.angular_b.dot(state_b.angular_velocity());
    value
        .is_finite()
        .then_some(value)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn apply_position_row(
    bodies: &mut [SolverBody],
    row: ConstraintRow,
    multiplier: f64,
) -> Result<(), CollisionError> {
    let (body_a, body_b) = solver_bodies_at_mut(bodies, row.body_a, row.body_b)?;
    constraint::apply_generalized_position_impulse_pair(
        body_a.body_mut(),
        body_b.body_mut(),
        row.linear_a * multiplier,
        row.angular_a * multiplier,
        row.linear_b * multiplier,
        row.angular_b * multiplier,
    )
    .map_err(map_constraint_error)
}

fn apply_velocity_row(
    bodies: &mut [SolverBody],
    row: ConstraintRow,
    impulse: f64,
) -> Result<(), CollisionError> {
    let (body_a, body_b) = solver_bodies_at_mut(bodies, row.body_a, row.body_b)?;
    constraint::apply_generalized_impulse_pair(
        body_a.body_mut(),
        body_b.body_mut(),
        row.linear_a * impulse,
        row.angular_a * impulse,
        row.linear_b * impulse,
        row.angular_b * impulse,
    )
    .map_err(map_constraint_error)
}

fn solver_bodies_at(
    bodies: &[SolverBody],
    body_a: usize,
    body_b: usize,
) -> Result<(&SolverBody, &SolverBody), CollisionError> {
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    Ok((
        bodies.get(body_a).ok_or(CollisionError::InvalidContact)?,
        bodies.get(body_b).ok_or(CollisionError::InvalidContact)?,
    ))
}

fn solver_bodies_at_mut(
    bodies: &mut [SolverBody],
    body_a: usize,
    body_b: usize,
) -> Result<(&mut SolverBody, &mut SolverBody), CollisionError> {
    if body_a == body_b {
        return Err(CollisionError::InvalidContact);
    }
    if body_a < body_b {
        let (left, right) = bodies.split_at_mut(body_b);
        Ok((
            left.get_mut(body_a).ok_or(CollisionError::InvalidContact)?,
            right.first_mut().ok_or(CollisionError::InvalidContact)?,
        ))
    } else {
        let (left, right) = bodies.split_at_mut(body_a);
        Ok((
            right.first_mut().ok_or(CollisionError::InvalidContact)?,
            left.get_mut(body_b).ok_or(CollisionError::InvalidContact)?,
        ))
    }
}

fn map_constraint_error(error: constraint::ConstraintError) -> CollisionError {
    match error {
        constraint::ConstraintError::InvalidMass => CollisionError::InvalidContact,
        constraint::ConstraintError::ArithmeticOverflow => CollisionError::ArithmeticOverflow,
        constraint::ConstraintError::NonConvergent => CollisionError::NonConvergent,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{mass::MassProperties, rigid_body::RigidBody, transform::Transform};
    use kapitsa_math::Vec3;

    fn solver_body(position: Vec3) -> SolverBody {
        let mass =
            MassProperties::new(1.0, Vec3::ZERO, kapitsa_math::Mat3::IDENTITY).expect("valid mass");
        let body = RigidBody::new_dynamic(
            Transform::new(position, kapitsa_math::Quat::IDENTITY).expect("transform"),
            mass,
        )
        .expect("dynamic body");
        let state = *body.state();
        SolverBody::from_prediction(body, state)
    }

    fn anchor_row(error: f64) -> ConstraintRow {
        ConstraintRow {
            body_a: 0,
            body_b: 1,
            linear_a: -Vec3::X,
            angular_a: Vec3::ZERO,
            linear_b: Vec3::X,
            angular_b: Vec3::ZERO,
            error,
            target_velocity: 0.0,
            compliance: 0.0,
            damping: 0.0,
            min_multiplier: f64::NEG_INFINITY,
            max_multiplier: f64::INFINITY,
            min_impulse: f64::NEG_INFINITY,
            max_impulse: f64::INFINITY,
            accumulated_multiplier: 0.0,
            accumulated_impulse: 0.0,
        }
    }

    #[test]
    fn xpbd_row_corrects_position_and_reuses_rows_across_iterations() {
        let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
        let mut workspace = JointWorkspace::new();
        let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
            let distance = bodies[1].state().transform().translation().x
                - bodies[0].state().transform().translation().x;
            workspace.push(anchor_row(distance - 1.0));
            Ok(())
        };
        JointSolver::solve(&mut workspace, &mut bodies, 0.01, 4, 1, prepare).expect("solve row");
        let distance = bodies[1].state().transform().translation().x
            - bodies[0].state().transform().translation().x;
        assert!((distance - 1.0).abs() < 1.0e-12, "distance={distance}");
        assert_eq!(workspace.rows().len(), 1);
    }

    #[test]
    fn compliant_row_scales_with_dt_and_preserves_finite_body_state() {
        let mut bodies = [solver_body(Vec3::ZERO), solver_body(Vec3::X * 2.0)];
        let mut workspace = JointWorkspace::new();
        let mut row = anchor_row(0.0);
        row.compliance = 1.0e-3;
        let prepare = |bodies: &[SolverBody], workspace: &mut JointWorkspace| {
            let distance = bodies[1].state().transform().translation().x
                - bodies[0].state().transform().translation().x;
            workspace.push(ConstraintRow {
                error: distance - 1.0,
                ..row
            });
            Ok(())
        };
        JointSolver::solve(&mut workspace, &mut bodies, 0.1, 8, 1, prepare)
            .expect("solve compliant row");
        assert!(bodies.iter().all(|body| {
            body.state().transform().translation().is_finite()
                && body.state().linear_velocity().is_finite()
                && body.state().angular_velocity().is_finite()
        }));
    }
}
