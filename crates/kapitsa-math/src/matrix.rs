use std::ops::{Add, Mul, Neg, Sub};

use crate::Vec3;

/// A 3 by 3 matrix stored as three columns and applied to column vectors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    columns: [Vec3; 3],
}

impl Mat3 {
    pub const ZERO: Self = Self::from_cols(Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
    pub const IDENTITY: Self = Self::from_cols(Vec3::X, Vec3::Y, Vec3::Z);

    pub const fn from_cols(x: Vec3, y: Vec3, z: Vec3) -> Self {
        Self { columns: [x, y, z] }
    }

    pub const fn from_rows(rows: [[f64; 3]; 3]) -> Self {
        Self::from_cols(
            Vec3::new(rows[0][0], rows[1][0], rows[2][0]),
            Vec3::new(rows[0][1], rows[1][1], rows[2][1]),
            Vec3::new(rows[0][2], rows[1][2], rows[2][2]),
        )
    }

    /// Constructs a matrix from a flat array in column-major order.
    pub const fn from_cols_array(values: [f64; 9]) -> Self {
        Self::from_cols(
            Vec3::new(values[0], values[1], values[2]),
            Vec3::new(values[3], values[4], values[5]),
            Vec3::new(values[6], values[7], values[8]),
        )
    }

    pub const fn to_cols_array(self) -> [f64; 9] {
        [
            self.columns[0].x,
            self.columns[0].y,
            self.columns[0].z,
            self.columns[1].x,
            self.columns[1].y,
            self.columns[1].z,
            self.columns[2].x,
            self.columns[2].y,
            self.columns[2].z,
        ]
    }

    pub const fn column(self, index: usize) -> Option<Vec3> {
        match index {
            0 => Some(self.columns[0]),
            1 => Some(self.columns[1]),
            2 => Some(self.columns[2]),
            _ => None,
        }
    }

    pub const fn row(self, index: usize) -> Option<Vec3> {
        match index {
            0 => Some(Vec3::new(
                self.columns[0].x,
                self.columns[1].x,
                self.columns[2].x,
            )),
            1 => Some(Vec3::new(
                self.columns[0].y,
                self.columns[1].y,
                self.columns[2].y,
            )),
            2 => Some(Vec3::new(
                self.columns[0].z,
                self.columns[1].z,
                self.columns[2].z,
            )),
            _ => None,
        }
    }

    pub fn transpose(self) -> Self {
        Self::from_cols(
            Vec3::new(self.columns[0].x, self.columns[1].x, self.columns[2].x),
            Vec3::new(self.columns[0].y, self.columns[1].y, self.columns[2].y),
            Vec3::new(self.columns[0].z, self.columns[1].z, self.columns[2].z),
        )
    }

    pub fn abs(self) -> Self {
        Self::from_cols(
            self.columns[0].abs(),
            self.columns[1].abs(),
            self.columns[2].abs(),
        )
    }

    pub const fn is_finite(self) -> bool {
        self.columns[0].is_finite() && self.columns[1].is_finite() && self.columns[2].is_finite()
    }

    pub fn checked_mul_vector(self, rhs: Vec3) -> Option<Vec3> {
        let result = self * rhs;
        (self.is_finite() && rhs.is_finite() && result.is_finite()).then_some(result)
    }

    pub fn checked_mul_matrix(self, rhs: Self) -> Option<Self> {
        let result = self * rhs;
        (self.is_finite() && rhs.is_finite() && result.is_finite()).then_some(result)
    }

    /// Solves `self * x = rhs` using Cholesky factorization.
    ///
    /// The matrix must be finite, symmetric to a relative tolerance of 32
    /// machine epsilons, and positive definite. Matrix scaling keeps the
    /// factorization stable for very large and very small finite coefficients.
    pub fn solve_spd(self, rhs: Vec3) -> Result<Vec3, CholeskyError> {
        if !self.is_finite() || !rhs.is_finite() {
            return Err(CholeskyError::NonFiniteInput);
        }

        let mut scale: f64 = 0.0;
        for value in self.to_cols_array() {
            scale = scale.max(value.abs());
        }
        if scale == 0.0 {
            return Err(CholeskyError::NotPositiveDefinite);
        }

        let rows = self.to_rows();
        let mut normalized = [[0.0; 3]; 3];
        for row in 0..3 {
            for column in 0..3 {
                normalized[row][column] = rows[row][column] / scale;
            }
        }

        let symmetry_tolerance = 32.0 * f64::EPSILON;
        for (row_index, row) in normalized.iter().enumerate() {
            for (column, earlier_row) in normalized.iter().enumerate().take(row_index) {
                if (row[column] - earlier_row[row_index]).abs() > symmetry_tolerance {
                    return Err(CholeskyError::NotSymmetric);
                }
            }
        }

        let mut lower = [[0.0; 3]; 3];
        for row in 0..3 {
            for column in 0..=row {
                let mut value = normalized[row][column];
                let correction = lower[row][..column]
                    .iter()
                    .zip(lower[column][..column].iter())
                    .map(|(left, right)| left * right)
                    .sum::<f64>();
                value -= correction;

                if row == column {
                    if !value.is_finite() || value <= 0.0 {
                        return Err(CholeskyError::NotPositiveDefinite);
                    }
                    lower[row][column] = value.sqrt();
                } else {
                    lower[row][column] = value / lower[column][column];
                }
            }
        }

        let normalized_rhs = [rhs.x / scale, rhs.y / scale, rhs.z / scale];
        if normalized_rhs.iter().any(|value| !value.is_finite()) {
            return Err(CholeskyError::ResultOutOfRange);
        }

        let mut intermediate = [0.0; 3];
        for row in 0..3 {
            let mut value = normalized_rhs[row];
            for column in 0..row {
                value -= lower[row][column] * intermediate[column];
            }
            intermediate[row] = value / lower[row][row];
        }

        let mut solution = [0.0; 3];
        for row in (0..3).rev() {
            let mut value = intermediate[row];
            for column in row + 1..3 {
                value -= lower[column][row] * solution[column];
            }
            solution[row] = value / lower[row][row];
        }

        let solution = Vec3::new(solution[0], solution[1], solution[2]);
        if !solution.is_finite() {
            return Err(CholeskyError::ResultOutOfRange);
        }
        Ok(solution)
    }

    fn to_rows(self) -> [[f64; 3]; 3] {
        [
            [self.columns[0].x, self.columns[1].x, self.columns[2].x],
            [self.columns[0].y, self.columns[1].y, self.columns[2].y],
            [self.columns[0].z, self.columns[1].z, self.columns[2].z],
        ]
    }
}

impl Add for Mat3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::from_cols(
            self.columns[0] + rhs.columns[0],
            self.columns[1] + rhs.columns[1],
            self.columns[2] + rhs.columns[2],
        )
    }
}

impl Sub for Mat3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::from_cols(
            self.columns[0] - rhs.columns[0],
            self.columns[1] - rhs.columns[1],
            self.columns[2] - rhs.columns[2],
        )
    }
}

impl Neg for Mat3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::from_cols(-self.columns[0], -self.columns[1], -self.columns[2])
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        self.columns[0] * rhs.x + self.columns[1] * rhs.y + self.columns[2] * rhs.z
    }
}

impl Mul for Mat3 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self::from_cols(
            self * rhs.columns[0],
            self * rhs.columns[1],
            self * rhs.columns[2],
        )
    }
}

impl Mul<f64> for Mat3 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::from_cols(
            self.columns[0] * rhs,
            self.columns[1] * rhs,
            self.columns[2] * rhs,
        )
    }
}

/// Failure returned by [`Mat3::solve_spd`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CholeskyError {
    NonFiniteInput,
    NotSymmetric,
    NotPositiveDefinite,
    ResultOutOfRange,
}

impl std::fmt::Display for CholeskyError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::NonFiniteInput => "matrix and right-hand side must be finite",
            Self::NotSymmetric => "matrix must be symmetric",
            Self::NotPositiveDefinite => "matrix must be positive definite",
            Self::ResultOutOfRange => "solution is outside the finite f64 range",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for CholeskyError {}

#[cfg(test)]
mod tests {
    use super::{CholeskyError, Mat3};
    use crate::Vec3;

    fn assert_vec3_close(actual: Vec3, expected: Vec3, tolerance: f64) {
        assert!((actual.x - expected.x).abs() <= tolerance);
        assert!((actual.y - expected.y).abs() <= tolerance);
        assert!((actual.z - expected.z).abs() <= tolerance);
    }

    #[test]
    fn stores_columns_and_rows_in_column_major_order() {
        let matrix = Mat3::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert_eq!(matrix.column(0), Some(Vec3::new(1.0, 4.0, 7.0)));
        assert_eq!(matrix.row(1), Some(Vec3::new(4.0, 5.0, 6.0)));
        assert_eq!(
            matrix.to_cols_array(),
            [1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0]
        );
        assert_eq!(matrix.column(3), None);
        assert_eq!(matrix.row(3), None);
    }

    #[test]
    fn vector_and_matrix_products_follow_column_vector_convention() {
        let matrix = Mat3::from_rows([[1.0, 2.0, 3.0], [0.0, 1.0, 4.0], [5.0, 6.0, 0.0]]);
        assert_eq!(
            matrix * Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(14.0, 14.0, 17.0)
        );

        let rotation = Mat3::from_rows([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]);
        assert_eq!(rotation * Mat3::IDENTITY, rotation);
        assert_eq!(rotation * Vec3::X, Vec3::Y);
    }

    #[test]
    fn transpose_is_involutive_and_reverses_product_order() {
        let a = Mat3::from_rows([[1.0, 2.0, 0.0], [0.0, 1.0, 3.0], [4.0, 0.0, 1.0]]);
        let b = Mat3::from_rows([[2.0, 0.0, 1.0], [1.0, 3.0, 0.0], [0.0, 4.0, 1.0]]);

        assert_eq!(a.transpose().transpose(), a);
        assert_eq!((a * b).transpose(), b.transpose() * a.transpose());
    }

    #[test]
    fn solves_nondiagonal_spd_system_with_cholesky() {
        let matrix = Mat3::from_rows([[4.0, 1.0, 1.0], [1.0, 3.0, 0.0], [1.0, 0.0, 2.0]]);
        let expected = Vec3::new(1.5, -2.0, 0.25);
        let solution = matrix.solve_spd(matrix * expected).expect("SPD solve");

        assert_vec3_close(solution, expected, 1e-14);
    }

    #[test]
    fn solves_spd_systems_across_extreme_scales() {
        let expected = Vec3::new(2.0, -3.0, 4.0);
        for scale in [1e-200, 1e200] {
            let matrix = Mat3::from_cols(
                Vec3::new(4.0 * scale, scale, 0.0),
                Vec3::new(scale, 3.0 * scale, scale),
                Vec3::new(0.0, scale, 2.0 * scale),
            );
            let solution = matrix
                .solve_spd(matrix * expected)
                .expect("scaled SPD solve");
            assert_vec3_close(solution, expected, 1e-13);
        }
    }

    #[test]
    fn rejects_invalid_spd_inputs() {
        assert_eq!(
            Mat3::IDENTITY.solve_spd(Vec3::new(f64::NAN, 0.0, 0.0)),
            Err(CholeskyError::NonFiniteInput)
        );
        assert_eq!(
            Mat3::from_rows([[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                .solve_spd(Vec3::ONE),
            Err(CholeskyError::NotSymmetric)
        );
        assert_eq!(
            Mat3::from_rows([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
                .solve_spd(Vec3::ONE),
            Err(CholeskyError::NotPositiveDefinite)
        );
    }

    #[test]
    fn checked_products_reject_overflow() {
        let matrix = Mat3::from_cols(Vec3::splat(f64::MAX), Vec3::ZERO, Vec3::ZERO);
        assert_eq!(matrix.checked_mul_vector(Vec3::new(2.0, 0.0, 0.0)), None);
        assert_eq!(matrix.checked_mul_matrix(matrix), None);
    }
}
