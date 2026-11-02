/// Counts joint solver work in benchmark builds without affecting the runtime path.
#[cfg(feature = "bench-internals")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct SolverCounters {
    pub(crate) prepared_rows: u64,
    pub(crate) inverse_inertia_updates: u64,
    pub(crate) scalar_rows_solved: u64,
    pub(crate) block_rows_solved: u64,
    pub(crate) velocity_warm_starts: u64,
    pub(crate) colors: u32,
    pub(crate) position_iterations: u32,
    pub(crate) velocity_iterations: u32,
}

#[cfg(feature = "bench-internals")]
impl SolverCounters {
    pub(crate) const ZERO: Self = Self {
        prepared_rows: 0,
        inverse_inertia_updates: 0,
        scalar_rows_solved: 0,
        block_rows_solved: 0,
        velocity_warm_starts: 0,
        colors: 0,
        position_iterations: 0,
        velocity_iterations: 0,
    };
}
