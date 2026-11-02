/// Internal joint-solver strategy selection. Public physics configuration
/// stays stable while individual solver strategies are validated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct JointSolverOptions {
    mass_cache: bool,
}

impl JointSolverOptions {
    pub(crate) const fn legacy() -> Self {
        Self { mass_cache: false }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) const fn with_mass_cache(mass_cache: bool) -> Self {
        Self { mass_cache }
    }

    pub(crate) const fn mass_cache(self) -> bool {
        self.mass_cache
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JointSolverStep {
    dt: f64,
    options: JointSolverOptions,
}

impl JointSolverStep {
    pub(crate) const fn new(dt: f64, options: JointSolverOptions) -> Self {
        Self { dt, options }
    }

    pub(crate) const fn dt(self) -> f64 {
        self.dt
    }

    pub(crate) const fn options(self) -> JointSolverOptions {
        self.options
    }
}
