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
mod tests {
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
