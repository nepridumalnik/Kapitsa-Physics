//! Benchmark-only work counters owned by the joint solver.

/// Kinetic and gravitational energy at a solver stage.
#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnergyComponents {
    pub translational_j: f64,
    pub rotational_j: f64,
    pub potential_j: f64,
}

#[cfg(feature = "bench-internals")]
impl EnergyComponents {
    pub(crate) fn add_assign(&mut self, other: Self) {
        self.translational_j += other.translational_j;
        self.rotational_j += other.rotational_j;
        self.potential_j += other.potential_j;
    }

    pub fn delta(self, before: Self) -> Self {
        Self {
            translational_j: self.translational_j - before.translational_j,
            rotational_j: self.rotational_j - before.rotational_j,
            potential_j: self.potential_j - before.potential_j,
        }
    }
}

#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy)]
pub(crate) enum EnergyStage {
    BeforeVelocityPcg,
    AfterVelocityPcg,
    VelocityPcgDelta,
    AfterVelocitySolver,
    BeforePositionPcg,
    AfterPositionPcg,
    PositionPcgDelta,
    AfterPositionSolver,
}

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
    /// Inner iterations used by the benchmark-only global velocity PCG solve.
    #[cfg(feature = "bench-internals")]
    pub velocity_pcg_iterations: u64,
    /// Inner iterations used by the benchmark-only global position PCG solve.
    #[cfg(feature = "bench-internals")]
    pub position_pcg_iterations: u64,
    /// Whole-world energy before prediction/integration.
    #[cfg(feature = "bench-internals")]
    pub energy_before_integration: EnergyComponents,
    /// Whole-world energy after prediction/integration and before solving.
    #[cfg(feature = "bench-internals")]
    pub energy_after_integration: EnergyComponents,
    /// Island energy before the first global velocity PCG invocation.
    #[cfg(feature = "bench-internals")]
    pub energy_before_velocity_pcg: EnergyComponents,
    /// Island energy after the final global velocity PCG invocation.
    #[cfg(feature = "bench-internals")]
    pub energy_after_velocity_pcg: EnergyComponents,
    /// Sum of signed component energy changes across individual velocity PCG calls.
    #[cfg(feature = "bench-internals")]
    pub velocity_pcg_energy_delta: EnergyComponents,
    /// Island energy after all velocity constraints.
    #[cfg(feature = "bench-internals")]
    pub energy_after_velocity_solver: EnergyComponents,
    /// Island energy before the first global position PCG invocation.
    #[cfg(feature = "bench-internals")]
    pub energy_before_position_pcg: EnergyComponents,
    /// Island energy after the final global position PCG invocation.
    #[cfg(feature = "bench-internals")]
    pub energy_after_position_pcg: EnergyComponents,
    /// Sum of signed component energy changes across individual position PCG calls.
    #[cfg(feature = "bench-internals")]
    pub position_pcg_energy_delta: EnergyComponents,
    /// First-order work v·Δp + ω·ΔL done by the velocity projection impulses.
    #[cfg(feature = "bench-internals")]
    pub velocity_projection_impulse_work_j: f64,
    /// Quadratic kinetic term 0.5·Δv·Δp + 0.5·Δω·ΔL of the velocity projection.
    #[cfg(feature = "bench-internals")]
    pub velocity_projection_quadratic_work_j: f64,
    /// Island energy after all position constraints.
    #[cfg(feature = "bench-internals")]
    pub energy_after_position_solver: EnergyComponents,
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
    /// Maximum linear velocity residual after contact solving, before joint solving.
    #[cfg(feature = "bench-internals")]
    pub pre_joint_velocity_linear_residual_mps: f64,
    /// Maximum |Jv-target| immediately after integration, before solver impulses.
    #[cfg(feature = "bench-internals")]
    pub initial_velocity_linear_residual_mps: f64,
    /// Maximum angular |Jv-target| immediately after integration, before solver impulses.
    #[cfg(feature = "bench-internals")]
    pub initial_velocity_angular_residual_radps: f64,
    /// Maximum angular velocity residual after contact solving, before joint solving.
    #[cfg(feature = "bench-internals")]
    pub pre_joint_velocity_angular_residual_radps: f64,
    /// Maximum linear velocity residual after the first velocity sweep.
    #[cfg(feature = "bench-internals")]
    pub first_velocity_linear_residual_mps: f64,
    /// Maximum angular velocity residual after the first velocity sweep.
    #[cfg(feature = "bench-internals")]
    pub first_velocity_angular_residual_radps: f64,
    /// Maximum anchor error after the first joint position sweep.
    #[cfg(feature = "bench-internals")]
    pub first_post_joint_anchor_error_m: f64,
    /// Maximum frame angle after the first joint position sweep.
    #[cfg(feature = "bench-internals")]
    pub first_post_joint_frame_error_rad: f64,
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
        velocity_pcg_iterations: 0,
        #[cfg(feature = "bench-internals")]
        position_pcg_iterations: 0,
        #[cfg(feature = "bench-internals")]
        energy_before_integration: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        energy_after_integration: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        energy_before_velocity_pcg: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        energy_after_velocity_pcg: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        velocity_pcg_energy_delta: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        energy_after_velocity_solver: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        energy_before_position_pcg: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        energy_after_position_pcg: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        position_pcg_energy_delta: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
        #[cfg(feature = "bench-internals")]
        velocity_projection_impulse_work_j: 0.0,
        #[cfg(feature = "bench-internals")]
        velocity_projection_quadratic_work_j: 0.0,
        #[cfg(feature = "bench-internals")]
        energy_after_position_solver: EnergyComponents {
            translational_j: 0.0,
            rotational_j: 0.0,
            potential_j: 0.0,
        },
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
        #[cfg(feature = "bench-internals")]
        pre_joint_velocity_linear_residual_mps: 0.0,
        #[cfg(feature = "bench-internals")]
        initial_velocity_linear_residual_mps: 0.0,
        #[cfg(feature = "bench-internals")]
        initial_velocity_angular_residual_radps: 0.0,
        #[cfg(feature = "bench-internals")]
        pre_joint_velocity_angular_residual_radps: 0.0,
        #[cfg(feature = "bench-internals")]
        first_velocity_linear_residual_mps: 0.0,
        #[cfg(feature = "bench-internals")]
        first_velocity_angular_residual_radps: 0.0,
        #[cfg(feature = "bench-internals")]
        first_post_joint_anchor_error_m: 0.0,
        #[cfg(feature = "bench-internals")]
        first_post_joint_frame_error_rad: 0.0,
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
            self.velocity_pcg_iterations = self
                .velocity_pcg_iterations
                .saturating_add(other.velocity_pcg_iterations);
            self.energy_before_integration
                .add_assign(other.energy_before_integration);
            self.energy_after_integration
                .add_assign(other.energy_after_integration);
            self.energy_before_velocity_pcg
                .add_assign(other.energy_before_velocity_pcg);
            self.energy_after_velocity_pcg
                .add_assign(other.energy_after_velocity_pcg);
            self.velocity_pcg_energy_delta
                .add_assign(other.velocity_pcg_energy_delta);
            self.energy_after_velocity_solver
                .add_assign(other.energy_after_velocity_solver);
            self.energy_before_position_pcg
                .add_assign(other.energy_before_position_pcg);
            self.energy_after_position_pcg
                .add_assign(other.energy_after_position_pcg);
            self.position_pcg_energy_delta
                .add_assign(other.position_pcg_energy_delta);
            self.velocity_projection_impulse_work_j += other.velocity_projection_impulse_work_j;
            self.velocity_projection_quadratic_work_j += other.velocity_projection_quadratic_work_j;
            self.energy_after_position_solver
                .add_assign(other.energy_after_position_solver);
        }
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
            self.pre_joint_velocity_linear_residual_mps = self
                .pre_joint_velocity_linear_residual_mps
                .max(other.pre_joint_velocity_linear_residual_mps);
            self.initial_velocity_linear_residual_mps = self
                .initial_velocity_linear_residual_mps
                .max(other.initial_velocity_linear_residual_mps);
            self.initial_velocity_angular_residual_radps = self
                .initial_velocity_angular_residual_radps
                .max(other.initial_velocity_angular_residual_radps);
            self.pre_joint_velocity_angular_residual_radps = self
                .pre_joint_velocity_angular_residual_radps
                .max(other.pre_joint_velocity_angular_residual_radps);
            self.first_velocity_linear_residual_mps = self
                .first_velocity_linear_residual_mps
                .max(other.first_velocity_linear_residual_mps);
            self.first_velocity_angular_residual_radps = self
                .first_velocity_angular_residual_radps
                .max(other.first_velocity_angular_residual_radps);
            self.first_post_joint_anchor_error_m = self
                .first_post_joint_anchor_error_m
                .max(other.first_post_joint_anchor_error_m);
            self.first_post_joint_frame_error_rad = self
                .first_post_joint_frame_error_rad
                .max(other.first_post_joint_frame_error_rad);
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
            velocity_pcg_iterations: u64::MAX,
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
            #[cfg(feature = "bench-internals")]
            first_velocity_linear_residual_mps: f64::MAX,
            #[cfg(feature = "bench-internals")]
            first_velocity_angular_residual_radps: f64::MAX,
            #[cfg(feature = "bench-internals")]
            first_post_joint_anchor_error_m: f64::MAX,
            #[cfg(feature = "bench-internals")]
            first_post_joint_frame_error_rad: f64::MAX,
            #[cfg(feature = "bench-internals")]
            pre_joint_velocity_linear_residual_mps: f64::MAX,
            #[cfg(feature = "bench-internals")]
            pre_joint_velocity_angular_residual_radps: f64::MAX,
            ..JointSolverMetrics::default()
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
            assert_eq!(metrics.first_velocity_linear_residual_mps, f64::MAX);
            assert_eq!(metrics.first_velocity_angular_residual_radps, f64::MAX);
            assert_eq!(metrics.first_post_joint_anchor_error_m, f64::MAX);
            assert_eq!(metrics.first_post_joint_frame_error_rad, f64::MAX);
            assert_eq!(metrics.pre_joint_velocity_linear_residual_mps, f64::MAX);
            assert_eq!(metrics.pre_joint_velocity_angular_residual_radps, f64::MAX);
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
                #[cfg(feature = "bench-internals")]
                first_velocity_linear_residual_mps: 0.8,
                #[cfg(feature = "bench-internals")]
                first_post_joint_anchor_error_m: 0.6,
                #[cfg(feature = "bench-internals")]
                pre_joint_velocity_linear_residual_mps: 0.9,
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
                #[cfg(feature = "bench-internals")]
                first_velocity_linear_residual_mps: 0.4,
                #[cfg(feature = "bench-internals")]
                first_post_joint_anchor_error_m: 0.7,
                #[cfg(feature = "bench-internals")]
                pre_joint_velocity_linear_residual_mps: 0.3,
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
            assert_eq!(forward.first_velocity_linear_residual_mps, 0.8);
            assert_eq!(forward.first_post_joint_anchor_error_m, 0.7);
            assert_eq!(forward.pre_joint_velocity_linear_residual_mps, 0.9);
        }
    }
}
