//! Benchmark-only work counters owned by the joint solver.

/// Work performed by the joint solver during one physics step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(not(feature = "bench-internals"), derive(Eq))]
pub struct JointSolverMetrics {
    /// Number of scalar constraint rows prepared.
    pub prepared_rows: u64,
    /// Number of cached inverse inertia tensors recomputed.
    pub inverse_inertia_updates: u64,
    /// Solver mass cache hits retained for the joint stress benchmark.
    pub solver_mass_cache_hits: u64,
    /// Solver mass cache misses retained for the joint stress benchmark.
    pub solver_mass_cache_misses: u64,
    /// Number of scalar constraint rows processed by the solver.
    pub scalar_rows_solved: u64,
    /// Number of rows processed by the block solver.
    pub block_rows_solved: u64,
    /// Number of rows that used scalar fallback from a block group.
    pub block_fallback_rows: u64,
    /// Number of cached velocity impulses actually applied.
    pub velocity_warm_starts: u64,
    /// Graph-color batches actually executed; zero if no colored pass runs.
    pub color_batches_processed: u64,
    /// Maximum graph colors used by an island; zero if no colored pass runs.
    pub max_colors_per_island: u32,
    /// Number of position constraint sweeps actually executed.
    pub position_sweeps: u64,
    /// Number of velocity constraint sweeps actually executed.
    pub velocity_sweeps: u64,
    /// Maximum anchor error before velocity constraints are solved.
    #[cfg(feature = "bench-internals")]
    pub pre_velocity_anchor_error_m: f64,
    /// Maximum frame angle before velocity constraints are solved.
    #[cfg(feature = "bench-internals")]
    pub pre_velocity_frame_error_rad: f64,
    /// Maximum connector residual after the velocity phase.
    #[cfg(feature = "bench-internals")]
    pub post_velocity_anchor_error_m: f64,
    /// Maximum connector frame angle after the velocity phase.
    #[cfg(feature = "bench-internals")]
    pub post_velocity_frame_error_rad: f64,
    /// Maximum connector residual after contact correction and before joint projection.
    #[cfg(feature = "bench-internals")]
    pub pre_joint_anchor_error_m: f64,
    /// Maximum connector frame angle after contact correction and before joint projection.
    #[cfg(feature = "bench-internals")]
    pub pre_joint_frame_error_rad: f64,
    /// Maximum connector residual after joint position projection.
    #[cfg(feature = "bench-internals")]
    pub post_joint_anchor_error_m: f64,
    /// Maximum connector frame angle after joint position projection.
    #[cfg(feature = "bench-internals")]
    pub post_joint_frame_error_rad: f64,
    /// Maximum linear velocity row residual after velocity solving.
    #[cfg(feature = "bench-internals")]
    pub post_velocity_linear_residual_mps: f64,
    /// Maximum angular velocity row residual after velocity solving.
    #[cfg(feature = "bench-internals")]
    pub post_velocity_angular_residual_radps: f64,
    /// Maximum linear velocity row residual after position projection.
    #[cfg(feature = "bench-internals")]
    pub post_position_linear_residual_mps: f64,
    /// Maximum angular velocity row residual after position projection.
    #[cfg(feature = "bench-internals")]
    pub post_position_angular_residual_radps: f64,
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct JointGeometryResidual {
    pub(crate) anchor_error_m: f64,
    pub(crate) frame_error_rad: f64,
}

impl JointSolverMetrics {
    pub(crate) const ZERO: Self = Self {
        prepared_rows: 0,
        inverse_inertia_updates: 0,
        solver_mass_cache_hits: 0,
        solver_mass_cache_misses: 0,
        scalar_rows_solved: 0,
        block_rows_solved: 0,
        block_fallback_rows: 0,
        velocity_warm_starts: 0,
        color_batches_processed: 0,
        max_colors_per_island: 0,
        position_sweeps: 0,
        velocity_sweeps: 0,
        #[cfg(feature = "bench-internals")]
        pre_velocity_anchor_error_m: 0.0,
        #[cfg(feature = "bench-internals")]
        pre_velocity_frame_error_rad: 0.0,
        #[cfg(feature = "bench-internals")]
        post_velocity_anchor_error_m: 0.0,
        #[cfg(feature = "bench-internals")]
        post_velocity_frame_error_rad: 0.0,
        #[cfg(feature = "bench-internals")]
        pre_joint_anchor_error_m: 0.0,
        #[cfg(feature = "bench-internals")]
        pre_joint_frame_error_rad: 0.0,
        #[cfg(feature = "bench-internals")]
        post_joint_anchor_error_m: 0.0,
        #[cfg(feature = "bench-internals")]
        post_joint_frame_error_rad: 0.0,
        #[cfg(feature = "bench-internals")]
        post_velocity_linear_residual_mps: 0.0,
        #[cfg(feature = "bench-internals")]
        post_velocity_angular_residual_radps: 0.0,
        #[cfg(feature = "bench-internals")]
        post_position_linear_residual_mps: 0.0,
        #[cfg(feature = "bench-internals")]
        post_position_angular_residual_radps: 0.0,
    };

    pub(crate) fn accumulate(&mut self, other: Self) {
        self.prepared_rows = self.prepared_rows.saturating_add(other.prepared_rows);
        self.inverse_inertia_updates = self
            .inverse_inertia_updates
            .saturating_add(other.inverse_inertia_updates);
        self.solver_mass_cache_hits = self
            .solver_mass_cache_hits
            .saturating_add(other.solver_mass_cache_hits);
        self.solver_mass_cache_misses = self
            .solver_mass_cache_misses
            .saturating_add(other.solver_mass_cache_misses);
        self.scalar_rows_solved = self
            .scalar_rows_solved
            .saturating_add(other.scalar_rows_solved);
        self.block_rows_solved = self
            .block_rows_solved
            .saturating_add(other.block_rows_solved);
        self.block_fallback_rows = self
            .block_fallback_rows
            .saturating_add(other.block_fallback_rows);
        self.velocity_warm_starts = self
            .velocity_warm_starts
            .saturating_add(other.velocity_warm_starts);
        self.color_batches_processed = self
            .color_batches_processed
            .saturating_add(other.color_batches_processed);
        self.max_colors_per_island = self.max_colors_per_island.max(other.max_colors_per_island);
        self.position_sweeps = self.position_sweeps.saturating_add(other.position_sweeps);
        self.velocity_sweeps = self.velocity_sweeps.saturating_add(other.velocity_sweeps);
        #[cfg(feature = "bench-internals")]
        {
            self.pre_velocity_anchor_error_m = self
                .pre_velocity_anchor_error_m
                .max(other.pre_velocity_anchor_error_m);
            self.pre_velocity_frame_error_rad = self
                .pre_velocity_frame_error_rad
                .max(other.pre_velocity_frame_error_rad);
            self.post_velocity_anchor_error_m = self
                .post_velocity_anchor_error_m
                .max(other.post_velocity_anchor_error_m);
            self.post_velocity_frame_error_rad = self
                .post_velocity_frame_error_rad
                .max(other.post_velocity_frame_error_rad);
            self.pre_joint_anchor_error_m = self
                .pre_joint_anchor_error_m
                .max(other.pre_joint_anchor_error_m);
            self.pre_joint_frame_error_rad = self
                .pre_joint_frame_error_rad
                .max(other.pre_joint_frame_error_rad);
            self.post_joint_anchor_error_m = self
                .post_joint_anchor_error_m
                .max(other.post_joint_anchor_error_m);
            self.post_joint_frame_error_rad = self
                .post_joint_frame_error_rad
                .max(other.post_joint_frame_error_rad);
            self.post_velocity_linear_residual_mps = self
                .post_velocity_linear_residual_mps
                .max(other.post_velocity_linear_residual_mps);
            self.post_velocity_angular_residual_radps = self
                .post_velocity_angular_residual_radps
                .max(other.post_velocity_angular_residual_radps);
            self.post_position_linear_residual_mps = self
                .post_position_linear_residual_mps
                .max(other.post_position_linear_residual_mps);
            self.post_position_angular_residual_radps = self
                .post_position_angular_residual_radps
                .max(other.post_position_angular_residual_radps);
        }
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
            solver_mass_cache_hits: 10,
            solver_mass_cache_misses: 11,
            scalar_rows_solved: 4,
            block_rows_solved: 5,
            block_fallback_rows: 6,
            velocity_warm_starts: 6,
            color_batches_processed: 7,
            max_colors_per_island: 2,
            position_sweeps: 8,
            velocity_sweeps: 9,
            ..JointSolverMetrics::default()
        });
        assert_eq!(
            metrics,
            JointSolverMetrics {
                prepared_rows: 2,
                inverse_inertia_updates: 3,
                solver_mass_cache_hits: 10,
                solver_mass_cache_misses: 11,
                scalar_rows_solved: 4,
                block_rows_solved: 5,
                block_fallback_rows: 6,
                velocity_warm_starts: 6,
                color_batches_processed: 7,
                max_colors_per_island: 2,
                position_sweeps: 8,
                velocity_sweeps: 9,
                ..JointSolverMetrics::default()
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
            inverse_inertia_updates: u64::MAX,
            solver_mass_cache_hits: u64::MAX,
            solver_mass_cache_misses: u64::MAX,
            scalar_rows_solved: u64::MAX,
            block_rows_solved: u64::MAX,
            block_fallback_rows: u64::MAX,
            velocity_warm_starts: u64::MAX,
            color_batches_processed: u64::MAX,
            max_colors_per_island: u32::MAX,
            position_sweeps: u64::MAX,
            velocity_sweeps: u64::MAX,
            #[cfg(feature = "bench-internals")]
            pre_velocity_anchor_error_m: f64::MAX,
            #[cfg(feature = "bench-internals")]
            pre_velocity_frame_error_rad: f64::MAX,
            #[cfg(feature = "bench-internals")]
            post_velocity_anchor_error_m: f64::MAX,
            #[cfg(feature = "bench-internals")]
            post_velocity_frame_error_rad: f64::MAX,
            #[cfg(feature = "bench-internals")]
            pre_joint_anchor_error_m: f64::MAX,
            #[cfg(feature = "bench-internals")]
            pre_joint_frame_error_rad: f64::MAX,
            #[cfg(feature = "bench-internals")]
            post_joint_anchor_error_m: f64::MAX,
            #[cfg(feature = "bench-internals")]
            post_joint_frame_error_rad: f64::MAX,
            #[cfg(feature = "bench-internals")]
            post_velocity_linear_residual_mps: f64::MAX,
            #[cfg(feature = "bench-internals")]
            post_velocity_angular_residual_radps: f64::MAX,
            #[cfg(feature = "bench-internals")]
            post_position_linear_residual_mps: f64::MAX,
            #[cfg(feature = "bench-internals")]
            post_position_angular_residual_radps: f64::MAX,
        };
        metrics.accumulate(JointSolverMetrics {
            prepared_rows: 1,
            inverse_inertia_updates: 1,
            solver_mass_cache_hits: 1,
            solver_mass_cache_misses: 1,
            scalar_rows_solved: 1,
            block_rows_solved: 1,
            block_fallback_rows: 1,
            velocity_warm_starts: 1,
            color_batches_processed: 1,
            max_colors_per_island: 1,
            position_sweeps: 1,
            velocity_sweeps: 1,
            ..JointSolverMetrics::default()
        });
        assert_eq!(metrics.prepared_rows, u64::MAX);
        assert_eq!(metrics.inverse_inertia_updates, u64::MAX);
        assert_eq!(metrics.solver_mass_cache_hits, u64::MAX);
        assert_eq!(metrics.solver_mass_cache_misses, u64::MAX);
        assert_eq!(metrics.scalar_rows_solved, u64::MAX);
        assert_eq!(metrics.block_rows_solved, u64::MAX);
        assert_eq!(metrics.block_fallback_rows, u64::MAX);
        assert_eq!(metrics.velocity_warm_starts, u64::MAX);
        assert_eq!(metrics.color_batches_processed, u64::MAX);
        assert_eq!(metrics.max_colors_per_island, u32::MAX);
        assert_eq!(metrics.position_sweeps, u64::MAX);
        assert_eq!(metrics.velocity_sweeps, u64::MAX);
        #[cfg(feature = "bench-internals")]
        {
            assert_eq!(metrics.pre_velocity_anchor_error_m, f64::MAX);
            assert_eq!(metrics.pre_velocity_frame_error_rad, f64::MAX);
            assert_eq!(metrics.post_velocity_anchor_error_m, f64::MAX);
            assert_eq!(metrics.post_velocity_frame_error_rad, f64::MAX);
            assert_eq!(metrics.pre_joint_anchor_error_m, f64::MAX);
            assert_eq!(metrics.pre_joint_frame_error_rad, f64::MAX);
            assert_eq!(metrics.post_joint_anchor_error_m, f64::MAX);
            assert_eq!(metrics.post_joint_frame_error_rad, f64::MAX);
            assert_eq!(metrics.post_velocity_linear_residual_mps, f64::MAX);
            assert_eq!(metrics.post_velocity_angular_residual_radps, f64::MAX);
            assert_eq!(metrics.post_position_linear_residual_mps, f64::MAX);
            assert_eq!(metrics.post_position_angular_residual_radps, f64::MAX);
        }
    }

    #[test]
    fn joint_solver_metrics_accumulation_is_order_independent() {
        let islands = [
            JointSolverMetrics {
                prepared_rows: 11,
                color_batches_processed: 2,
                max_colors_per_island: 2,
                position_sweeps: 3,
                #[cfg(feature = "bench-internals")]
                pre_velocity_anchor_error_m: 0.25,
                #[cfg(feature = "bench-internals")]
                pre_velocity_frame_error_rad: 0.1,
                #[cfg(feature = "bench-internals")]
                post_velocity_anchor_error_m: 0.25,
                #[cfg(feature = "bench-internals")]
                post_joint_frame_error_rad: 0.5,
                #[cfg(feature = "bench-internals")]
                post_position_linear_residual_mps: 0.4,
                ..JointSolverMetrics::default()
            },
            JointSolverMetrics {
                prepared_rows: 17,
                color_batches_processed: 5,
                max_colors_per_island: 3,
                velocity_sweeps: 7,
                #[cfg(feature = "bench-internals")]
                pre_velocity_anchor_error_m: 0.75,
                #[cfg(feature = "bench-internals")]
                pre_velocity_frame_error_rad: 0.2,
                #[cfg(feature = "bench-internals")]
                post_velocity_anchor_error_m: 0.75,
                #[cfg(feature = "bench-internals")]
                post_joint_frame_error_rad: 0.2,
                #[cfg(feature = "bench-internals")]
                post_position_linear_residual_mps: 0.2,
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
        #[cfg(feature = "bench-internals")]
        {
            assert_eq!(forward.pre_velocity_anchor_error_m, 0.75);
            assert_eq!(forward.pre_velocity_frame_error_rad, 0.2);
            assert_eq!(forward.post_velocity_anchor_error_m, 0.75);
            assert_eq!(forward.post_position_linear_residual_mps, 0.4);
            assert_eq!(forward.post_joint_frame_error_rad, 0.5);
        }
    }
}
