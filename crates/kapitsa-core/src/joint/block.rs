use crate::{
    collision::CollisionError, joint::row::ConstraintRow, rigid_body::solver_mass::SolverMass,
};
use kapitsa_math::{Mat3, Vec3};

/// Coupled 3-axis correction with an SPD effective inverse mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Block3 {
    pub(crate) inverse_effective_mass: Mat3,
    pub(crate) errors: Vec3,
    pub(crate) multipliers: Vec3,
    pub(crate) compliance: f64,
}

impl Block3 {
    /// Returns delta multipliers; the caller must apply the three Jacobians together.
    pub(crate) fn solve_position(self, dt: f64) -> Result<Vec3, CollisionError> {
        if !dt.is_finite() || dt <= 0.0 || !self.compliance.is_finite() || self.compliance < 0.0 {
            return Err(CollisionError::InvalidConfig);
        }

        let alpha = if self.compliance == 0.0 {
            0.0
        } else {
            self.compliance / dt / dt
        };
        if !alpha.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }

        let lhs = self.inverse_effective_mass + Mat3::from_diagonal(Vec3::splat(alpha));
        let rhs = -(self.errors + self.multipliers * alpha);
        if !rhs.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        lhs.solve_spd(rhs)
            .map_err(|_| CollisionError::NonConvergent)
    }
}

/// Builds the 3x3 generalized inverse-mass matrix for three rows on one body pair.
/// `masses` is ordered as the rows' body A and body B. `dt` must match the step
/// passed later to `Block3::solve_position`.
pub(crate) fn prepare_block3(
    rows: &[ConstraintRow; 3],
    masses: [SolverMass; 2],
    dt: f64,
) -> Result<Block3, CollisionError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(CollisionError::InvalidConfig);
    }
    let body_a = rows[0].body_a;
    let body_b = rows[0].body_b;
    let body_count = body_a
        .max(body_b)
        .checked_add(1)
        .ok_or(CollisionError::InvalidContact)?;
    if rows
        .iter()
        .any(|row| row.body_a != body_a || row.body_b != body_b || !row.is_valid(body_count))
    {
        return Err(CollisionError::InvalidContact);
    }

    let compliance = rows[0].compliance;
    if rows.iter().any(|row| row.compliance != compliance) {
        return Err(CollisionError::InvalidConfig);
    }

    let mut entries = [[0.0; 3]; 3];
    for (row_index, row) in rows.iter().enumerate() {
        // J M^-1 J^T is symmetric. Compute each independent entry once
        // and mirror it so roundoff cannot make the assembled matrix asymmetric.
        for (column_index, column) in rows.iter().enumerate().skip(row_index) {
            let value = masses[0].cross(
                row.linear_a,
                row.angular_a,
                column.linear_a,
                column.angular_a,
            )? + masses[1].cross(
                row.linear_b,
                row.angular_b,
                column.linear_b,
                column.angular_b,
            )?;
            if !value.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            entries[row_index][column_index] = value;
            entries[column_index][row_index] = value;
        }
    }

    Ok(Block3 {
        inverse_effective_mass: Mat3::from_rows(entries),
        errors: Vec3::new(rows[0].error, rows[1].error, rows[2].error),
        multipliers: Vec3::new(
            rows[0].accumulated_multiplier,
            rows[1].accumulated_multiplier,
            rows[2].accumulated_multiplier,
        ),
        compliance,
    })
}

#[cfg(test)]
mod tests {
    use super::{Block3, prepare_block3};
    use crate::{
        collision::CollisionError,
        joint::row::{ConstraintRow, RowDimension},
        mass::MassProperties,
        rigid_body::{RigidBody, solver_mass::SolverMass},
        transform::Transform,
    };
    use kapitsa_math::{Mat3, Quat, Vec3};

    fn close(actual: Vec3, expected: Vec3, absolute: f64, relative: f64) {
        let error = (actual - expected).length();
        let tolerance = absolute + relative * expected.length();
        assert!(
            error <= tolerance,
            "{actual:?} != {expected:?}; error={error}"
        );
    }

    fn row(
        key: u8,
        linear_a: Vec3,
        angular_a: Vec3,
        linear_b: Vec3,
        angular_b: Vec3,
        error: f64,
    ) -> ConstraintRow {
        ConstraintRow {
            key,
            dimension: RowDimension::Linear,
            body_a: 0,
            body_b: 1,
            linear_a,
            angular_a,
            linear_b,
            angular_b,
            error,
            target_velocity: 0.0,
            force_bias: 0.0,
            position_enabled: true,
            compliance: 0.0,
            damping: 0.0,
            min_multiplier: f64::NEG_INFINITY,
            max_multiplier: f64::INFINITY,
            min_impulse: f64::NEG_INFINITY,
            max_impulse: f64::INFINITY,
            accumulated_multiplier: 0.0,
            accumulated_impulse: 0.0,
            cached_velocity_effective_mass: 0.0,
        }
    }

    fn gaussian_solve(matrix: [[f64; 3]; 3], rhs: Vec3) -> Vec3 {
        let mut augmented = [
            [matrix[0][0], matrix[0][1], matrix[0][2], rhs.x],
            [matrix[1][0], matrix[1][1], matrix[1][2], rhs.y],
            [matrix[2][0], matrix[2][1], matrix[2][2], rhs.z],
        ];
        for pivot in 0..3 {
            let pivot_row = (pivot..3)
                .max_by(|&left, &right| {
                    augmented[left][pivot]
                        .abs()
                        .total_cmp(&augmented[right][pivot].abs())
                })
                .expect("pivot row");
            augmented.swap(pivot, pivot_row);
            let pivot_values = augmented[pivot];
            for row in (pivot + 1)..3 {
                let factor = augmented[row][pivot] / augmented[pivot][pivot];
                for (column, value) in augmented[row].iter_mut().enumerate().skip(pivot) {
                    *value -= factor * pivot_values[column];
                }
            }
        }
        let mut solution = [0.0; 3];
        for row in (0..3).rev() {
            let mut value = augmented[row][3];
            for column in (row + 1)..3 {
                value -= augmented[row][column] * solution[column];
            }
            solution[row] = value / augmented[row][row];
        }
        Vec3::new(solution[0], solution[1], solution[2])
    }

    fn scalar_gauss_seidel(matrix: [[f64; 3]; 3], rhs: Vec3, iterations: usize) -> Vec3 {
        let right = [rhs.x, rhs.y, rhs.z];
        let mut solution = [0.0; 3];
        for _ in 0..iterations {
            for row in 0..3 {
                let mut value = right[row];
                for column in 0..3 {
                    if column != row {
                        value -= matrix[row][column] * solution[column];
                    }
                }
                solution[row] = value / matrix[row][row];
            }
        }
        Vec3::new(solution[0], solution[1], solution[2])
    }

    #[test]
    fn diagonal_spd_block_has_exact_known_solution() {
        let block = Block3 {
            inverse_effective_mass: Mat3::from_diagonal(Vec3::new(2.0, 3.0, 5.0)),
            errors: Vec3::new(2.0, 3.0, 5.0),
            multipliers: Vec3::ZERO,
            compliance: 0.0,
        };
        close(
            block.solve_position(1.0).expect("SPD"),
            -Vec3::ONE,
            1.0e-12,
            0.0,
        );
    }

    #[test]
    fn compliant_block_obeys_alpha_over_dt_squared() {
        let block = Block3 {
            inverse_effective_mass: Mat3::IDENTITY,
            errors: Vec3::new(2.0, 0.0, 0.0),
            multipliers: Vec3::ZERO,
            compliance: 0.25,
        };
        close(
            block.solve_position(0.5).expect("SPD"),
            -Vec3::X,
            1.0e-12,
            0.0,
        );
    }

    #[test]
    fn invalid_dt_and_non_spd_blocks_are_rejected() {
        let mut block = Block3 {
            inverse_effective_mass: Mat3::IDENTITY,
            errors: Vec3::X,
            multipliers: Vec3::ZERO,
            compliance: 0.0,
        };
        assert_eq!(
            block.solve_position(0.0),
            Err(CollisionError::InvalidConfig)
        );
        assert_eq!(
            block.solve_position(f64::NAN),
            Err(CollisionError::InvalidConfig)
        );

        block.inverse_effective_mass =
            Mat3::from_rows([[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        assert_eq!(
            block.solve_position(1.0),
            Err(CollisionError::NonConvergent)
        );
    }

    #[test]
    fn coupled_spd_block_matches_independent_solve_and_scalar_convergence() {
        let matrix = [[4.0, 2.5, 1.5], [2.5, 4.0, 1.0], [1.5, 1.0, 3.0]];
        let expected = Vec3::new(1.0, -2.0, 0.5);
        let rhs = Vec3::new(
            matrix[0][0] * expected.x + matrix[0][1] * expected.y + matrix[0][2] * expected.z,
            matrix[1][0] * expected.x + matrix[1][1] * expected.y + matrix[1][2] * expected.z,
            matrix[2][0] * expected.x + matrix[2][1] * expected.y + matrix[2][2] * expected.z,
        );
        let block = Block3 {
            inverse_effective_mass: Mat3::from_rows(matrix),
            errors: -rhs,
            multipliers: Vec3::ZERO,
            compliance: 0.0,
        };

        let block_solution = block.solve_position(1.0).expect("coupled SPD block");
        let reference = gaussian_solve(matrix, rhs);
        let scalar_solution = scalar_gauss_seidel(matrix, rhs, 256);
        close(block_solution, reference, 1.0e-12, 1.0e-12);
        close(scalar_solution, reference, 1.0e-12, 1.0e-12);
    }

    #[test]
    fn solver_accepts_small_asymmetric_roundoff_perturbation() {
        let block = Block3 {
            inverse_effective_mass: Mat3::from_rows([
                [4.0, 1.0 + 8.0 * f64::EPSILON, 0.0],
                [1.0, 3.0, 0.0],
                [0.0, 0.0, 2.0],
            ]),
            errors: Vec3::new(-5.0, -4.0, -2.0),
            multipliers: Vec3::ZERO,
            compliance: 0.0,
        };
        close(
            block
                .solve_position(1.0)
                .expect("roundoff-symmetric SPD block"),
            Vec3::ONE,
            1.0e-12,
            1.0e-12,
        );
    }

    #[test]
    fn solve_remains_finite_at_extreme_matrix_scales() {
        let expected = Vec3::new(1.25, -2.0, 0.5);
        for scale in [1.0e-200, 1.0e200] {
            let inverse_effective_mass = Mat3::from_diagonal(Vec3::splat(scale));
            let block = Block3 {
                inverse_effective_mass,
                errors: -(inverse_effective_mass * expected),
                multipliers: Vec3::ZERO,
                compliance: 0.0,
            };
            close(
                block.solve_position(1.0).expect("scaled SPD block"),
                expected,
                1.0e-12,
                1.0e-12,
            );
        }
    }

    #[test]
    fn prepare_block3_retains_all_generalized_off_diagonal_terms() {
        let rows = [
            row(0, Vec3::X, Vec3::Y, -Vec3::X, Vec3::ZERO, 1.0),
            row(1, Vec3::X + Vec3::Y, Vec3::Z, -Vec3::Y, Vec3::ZERO, 2.0),
            row(
                2,
                Vec3::X + Vec3::Z,
                Vec3::X + Vec3::Y,
                -Vec3::Z,
                Vec3::ZERO,
                3.0,
            ),
        ];
        let body_a = SolverMass {
            inverse_mass: 2.0,
            inverse_inertia: Mat3::from_diagonal(Vec3::new(1.0, 2.0, 3.0)),
            center: Vec3::ZERO,
        };
        let block = prepare_block3(&rows, [body_a, SolverMass::STATIC], 0.01)
            .expect("valid effective mass block");

        assert_eq!(
            block.inverse_effective_mass,
            Mat3::from_rows([[4.0, 2.0, 4.0], [2.0, 7.0, 2.0], [4.0, 2.0, 7.0]])
        );
        assert_eq!(block.errors, Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn prepare_block3_handles_static_side_and_large_mass_ratio() {
        let rows = [
            row(0, Vec3::X, Vec3::ZERO, -Vec3::X, Vec3::ZERO, 1.0e-6),
            row(1, Vec3::Y, Vec3::ZERO, -Vec3::Y, Vec3::ZERO, 2.0e-6),
            row(2, Vec3::Z, Vec3::ZERO, -Vec3::Z, Vec3::ZERO, 3.0e-6),
        ];
        let light = SolverMass {
            inverse_mass: 1.0,
            inverse_inertia: Mat3::ZERO,
            center: Vec3::ZERO,
        };
        let heavy = SolverMass {
            inverse_mass: 1.0e-6,
            inverse_inertia: Mat3::ZERO,
            center: Vec3::ZERO,
        };
        let block = prepare_block3(&rows, [SolverMass::STATIC, heavy], 0.1)
            .expect("static plus dynamic body");
        assert_eq!(
            block.inverse_effective_mass,
            Mat3::from_diagonal(Vec3::splat(1.0e-6))
        );
        let ratio_block =
            prepare_block3(&rows, [light, heavy], 0.1).expect("large dynamic mass ratio");
        assert_eq!(
            ratio_block.inverse_effective_mass,
            Mat3::from_diagonal(Vec3::splat(1.000001))
        );
        assert!(
            ratio_block
                .solve_position(0.1)
                .expect("ratio solve")
                .is_finite()
        );
    }

    #[test]
    fn prepared_effective_mass_matches_finite_difference_of_row_velocity() {
        let mass_a = MassProperties::new(
            2.0,
            Vec3::new(0.2, -0.1, 0.15),
            Mat3::from_diagonal(Vec3::new(3.0, 4.0, 5.0)),
        )
        .expect("mass A");
        let mass_b = MassProperties::new(
            3.5,
            Vec3::new(-0.1, 0.25, 0.05),
            Mat3::from_diagonal(Vec3::new(6.0, 4.0, 3.0)),
        )
        .expect("mass B");
        let body_a = RigidBody::new_dynamic(
            Transform::new(
                Vec3::ZERO,
                Quat::from_axis_angle(Vec3::Z, 0.15).expect("rotation A"),
            )
            .expect("transform A"),
            mass_a,
        )
        .expect("body A");
        let body_b = RigidBody::new_dynamic(
            Transform::new(
                Vec3::new(1.0, 0.2, -0.1),
                Quat::from_axis_angle(Vec3::new(0.2, 1.0, -0.1), 0.3).expect("rotation B"),
            )
            .expect("transform B"),
            mass_b,
        )
        .expect("body B");
        let rows = [
            row(
                0,
                Vec3::X,
                Vec3::new(0.0, 0.3, 0.1),
                -Vec3::X,
                Vec3::new(0.2, 0.0, 0.1),
                0.0,
            ),
            row(
                1,
                Vec3::Y,
                Vec3::new(0.1, 0.0, 0.5),
                -Vec3::Y,
                Vec3::new(0.0, 0.4, 0.2),
                0.0,
            ),
            row(
                2,
                Vec3::Z,
                Vec3::new(0.7, 0.2, 0.0),
                -Vec3::Z,
                Vec3::new(0.1, 0.6, 0.0),
                0.0,
            ),
        ];
        let solver_mass_a = SolverMass::from_body(&body_a).expect("mass A cache");
        let solver_mass_b = SolverMass::from_body(&body_b).expect("mass B cache");
        let block =
            prepare_block3(&rows, [solver_mass_a, solver_mass_b], 0.01).expect("prepared block");

        for column in 0..3 {
            let mut applied_a = body_a;
            let mut applied_b = body_b;
            let row = rows[column];
            applied_a
                .apply_constraint_impulse_cached(row.linear_a, row.angular_a, solver_mass_a)
                .expect("apply impulse to A");
            applied_b
                .apply_constraint_impulse_cached(row.linear_b, row.angular_b, solver_mass_b)
                .expect("apply impulse to B");

            for (row_index, row) in rows.iter().enumerate() {
                let response = row.linear_a.dot(applied_a.state().linear_velocity())
                    + row.angular_a.dot(applied_a.state().angular_velocity())
                    + row.linear_b.dot(applied_b.state().linear_velocity())
                    + row.angular_b.dot(applied_b.state().angular_velocity());
                let row = block
                    .inverse_effective_mass
                    .row(row_index)
                    .expect("effective-mass row");
                let expected = match column {
                    0 => row.x,
                    1 => row.y,
                    2 => row.z,
                    _ => unreachable!("column is in 0..3"),
                };
                assert!((response - expected).abs() <= 1.0e-12 * expected.abs().max(1.0));
            }
        }
    }
}
