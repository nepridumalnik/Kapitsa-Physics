#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(dead_code)]
pub(crate) struct JointTolerance {
    pub(crate) linear_m: f64,
    pub(crate) angular_rad: f64,
    pub(crate) velocity_mps: f64,
    pub(crate) angular_velocity_radps: f64,
}

#[allow(dead_code)]
impl JointTolerance {
    pub(crate) fn validate(self) -> bool {
        [
            self.linear_m,
            self.angular_rad,
            self.velocity_mps,
            self.angular_velocity_radps,
        ]
        .into_iter()
        .all(|value| value.is_finite() && value > 0.0)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct JointResidual {
    pub(crate) max_linear_m: f64,
    pub(crate) max_angular_rad: f64,
    pub(crate) max_velocity_mps: f64,
    pub(crate) max_angular_velocity_radps: f64,
}

impl JointResidual {
    pub(crate) const ZERO: Self = Self {
        max_linear_m: 0.0,
        max_angular_rad: 0.0,
        max_velocity_mps: 0.0,
        max_angular_velocity_radps: 0.0,
    };

    pub(crate) fn is_valid(self) -> bool {
        [
            self.max_linear_m,
            self.max_angular_rad,
            self.max_velocity_mps,
            self.max_angular_velocity_radps,
        ]
        .into_iter()
        .all(|value| value.is_finite() && value >= 0.0)
    }

    #[allow(dead_code)]
    pub(crate) fn converged(self, tolerance: JointTolerance) -> bool {
        self.is_valid()
            && tolerance.validate()
            && self.max_linear_m <= tolerance.linear_m
            && self.max_angular_rad <= tolerance.angular_rad
            && self.max_velocity_mps <= tolerance.velocity_mps
            && self.max_angular_velocity_radps <= tolerance.angular_velocity_radps
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub(crate) struct IterationRange {
    pub(crate) min: u32,
    pub(crate) max: u32,
}

#[allow(dead_code)]
impl IterationRange {
    pub(crate) const fn new(min: u32, max: u32) -> Self {
        Self { min, max }
    }

    pub(crate) const fn is_valid(self) -> bool {
        self.min > 0 && self.min <= self.max
    }
}

/// Project a scalar constraint gradient onto its active multiplier bounds.
pub(super) fn projected_kkt_residual(
    gradient: f64,
    multiplier: f64,
    lower: f64,
    upper: f64,
) -> f64 {
    if lower == upper {
        gradient.abs()
    } else if multiplier <= lower {
        (-gradient).max(0.0)
    } else if multiplier >= upper {
        gradient.max(0.0)
    } else {
        gradient.abs()
    }
}

/// Work performed by the joint solver during one physics step.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JointSolverMetrics {
    pub prepared_rows: u64,
    pub inverse_inertia_updates: u64,
    pub scalar_rows_solved: u64,
    pub block_rows_solved: u64,
    pub velocity_warm_starts: u64,
    pub color_batches_processed: u64,
    pub max_colors_per_island: u32,
    pub position_sweeps: u64,
    pub velocity_sweeps: u64,
}

impl JointSolverMetrics {
    pub(crate) const ZERO: Self = Self {
        prepared_rows: 0,
        inverse_inertia_updates: 0,
        scalar_rows_solved: 0,
        block_rows_solved: 0,
        velocity_warm_starts: 0,
        color_batches_processed: 0,
        max_colors_per_island: 0,
        position_sweeps: 0,
        velocity_sweeps: 0,
    };

    pub(crate) fn accumulate(&mut self, other: Self) {
        self.prepared_rows = self.prepared_rows.saturating_add(other.prepared_rows);
        self.inverse_inertia_updates = self
            .inverse_inertia_updates
            .saturating_add(other.inverse_inertia_updates);
        self.scalar_rows_solved = self
            .scalar_rows_solved
            .saturating_add(other.scalar_rows_solved);
        self.block_rows_solved = self
            .block_rows_solved
            .saturating_add(other.block_rows_solved);
        self.velocity_warm_starts = self
            .velocity_warm_starts
            .saturating_add(other.velocity_warm_starts);
        self.color_batches_processed = self
            .color_batches_processed
            .saturating_add(other.color_batches_processed);
        self.max_colors_per_island = self.max_colors_per_island.max(other.max_colors_per_island);
        self.position_sweeps = self.position_sweeps.saturating_add(other.position_sweeps);
        self.velocity_sweeps = self.velocity_sweeps.saturating_add(other.velocity_sweeps);
    }
}

#[cfg(test)]
mod metrics_tests {
    use super::JointSolverMetrics;

    #[test]
    fn joint_solver_metrics_default_is_zero() {
        assert_eq!(JointSolverMetrics::default(), JointSolverMetrics::ZERO);
    }

    #[test]
    fn joint_solver_metrics_accumulate() {
        let mut metrics = JointSolverMetrics::default();
        metrics.accumulate(JointSolverMetrics {
            prepared_rows: 2,
            inverse_inertia_updates: 3,
            scalar_rows_solved: 4,
            block_rows_solved: 5,
            velocity_warm_starts: 6,
            color_batches_processed: 7,
            max_colors_per_island: 2,
            position_sweeps: 8,
            velocity_sweeps: 9,
        });
        assert_eq!(
            metrics,
            JointSolverMetrics {
                prepared_rows: 2,
                inverse_inertia_updates: 3,
                scalar_rows_solved: 4,
                block_rows_solved: 5,
                velocity_warm_starts: 6,
                color_batches_processed: 7,
                max_colors_per_island: 2,
                position_sweeps: 8,
                velocity_sweeps: 9,
            }
        );
    }

    #[test]
    fn joint_solver_metrics_preserves_max_color_count() {
        let mut metrics = JointSolverMetrics {
            color_batches_processed: 2,
            max_colors_per_island: 2,
            ..JointSolverMetrics::default()
        };
        metrics.accumulate(JointSolverMetrics {
            color_batches_processed: 3,
            max_colors_per_island: 3,
            ..JointSolverMetrics::default()
        });
        assert_eq!(metrics.color_batches_processed, 5);
        assert_eq!(metrics.max_colors_per_island, 3);
    }

    #[test]
    fn joint_solver_metrics_saturates() {
        let mut metrics = JointSolverMetrics {
            prepared_rows: u64::MAX,
            scalar_rows_solved: u64::MAX,
            color_batches_processed: u64::MAX,
            max_colors_per_island: u32::MAX,
            ..JointSolverMetrics::default()
        };
        metrics.accumulate(JointSolverMetrics {
            prepared_rows: 1,
            scalar_rows_solved: 1,
            color_batches_processed: 1,
            max_colors_per_island: 1,
            ..JointSolverMetrics::default()
        });
        assert_eq!(metrics.prepared_rows, u64::MAX);
        assert_eq!(metrics.scalar_rows_solved, u64::MAX);
        assert_eq!(metrics.color_batches_processed, u64::MAX);
        assert_eq!(metrics.max_colors_per_island, u32::MAX);
    }

    #[test]
    fn joint_solver_metrics_accumulation_is_order_independent() {
        let islands = [
            JointSolverMetrics {
                prepared_rows: 11,
                color_batches_processed: 2,
                max_colors_per_island: 2,
                position_sweeps: 3,
                ..JointSolverMetrics::default()
            },
            JointSolverMetrics {
                prepared_rows: 17,
                color_batches_processed: 5,
                max_colors_per_island: 3,
                velocity_sweeps: 7,
                ..JointSolverMetrics::default()
            },
            JointSolverMetrics {
                prepared_rows: 23,
                color_batches_processed: 1,
                max_colors_per_island: 1,
                scalar_rows_solved: 31,
                ..JointSolverMetrics::default()
            },
        ];
        let mut forward = JointSolverMetrics::default();
        for island in islands {
            forward.accumulate(island);
        }
        let mut reverse = JointSolverMetrics::default();
        for island in islands.into_iter().rev() {
            reverse.accumulate(island);
        }
        assert_eq!(forward, reverse);
    }
}

#[cfg(test)]
mod residual_tests {
    use super::{IterationRange, JointResidual, JointTolerance, projected_kkt_residual};

    #[test]
    fn tolerances_and_iteration_ranges_reject_invalid_values() {
        assert!(
            JointTolerance {
                linear_m: 1.0e-3,
                angular_rad: 1.0e-3,
                velocity_mps: 1.0e-2,
                angular_velocity_radps: 1.0e-2,
            }
            .validate()
        );
        assert!(
            !JointTolerance {
                linear_m: 0.0,
                angular_rad: 1.0,
                velocity_mps: 1.0,
                angular_velocity_radps: 1.0,
            }
            .validate()
        );
        assert!(
            !JointTolerance {
                linear_m: f64::NAN,
                angular_rad: 1.0,
                velocity_mps: 1.0,
                angular_velocity_radps: 1.0,
            }
            .validate()
        );
        assert!(IterationRange::new(2, 8).is_valid());
        assert!(IterationRange::new(1, 1).is_valid());
        assert!(!IterationRange::new(0, 1).is_valid());
        assert!(!IterationRange::new(4, 3).is_valid());
    }

    #[test]
    fn convergence_rejects_non_finite_residuals_and_keeps_units_independent() {
        let tolerance = JointTolerance {
            linear_m: 1.0e-3,
            angular_rad: 1.0e-2,
            velocity_mps: 1.0e-2,
            angular_velocity_radps: 1.0e-1,
        };
        let residual = JointResidual {
            max_linear_m: 5.0e-4,
            max_angular_rad: 5.0e-3,
            max_velocity_mps: 5.0e-3,
            max_angular_velocity_radps: 5.0e-2,
        };
        assert!(residual.converged(tolerance));
        assert!(
            !JointResidual {
                max_linear_m: 2.0e-3,
                ..residual
            }
            .converged(tolerance)
        );
        assert!(
            !JointResidual {
                max_angular_velocity_radps: f64::INFINITY,
                ..residual
            }
            .converged(tolerance)
        );
    }

    #[test]
    fn projected_residual_obeys_lower_upper_and_bilateral_kkt_conditions() {
        assert_eq!(projected_kkt_residual(1.0, 0.0, 0.0, f64::INFINITY), 0.0);
        assert_eq!(projected_kkt_residual(-1.0, 0.0, 0.0, f64::INFINITY), 1.0);
        assert_eq!(
            projected_kkt_residual(-1.0, 0.0, f64::NEG_INFINITY, 0.0),
            0.0
        );
        assert_eq!(
            projected_kkt_residual(1.0, 0.0, f64::NEG_INFINITY, 0.0),
            1.0
        );
        assert_eq!(projected_kkt_residual(-2.0, 0.0, -1.0, 1.0), 2.0);
        assert_eq!(projected_kkt_residual(2.0, 0.0, 1.0, 1.0), 2.0);
    }
}
