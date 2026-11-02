/// Internal joint-solver strategy selection. Public physics configuration
/// stays stable while individual solver strategies are validated.
use super::JointImpulseCache;
use super::reaction::ReactionMode;
use super::warm_start::WarmStartConfig;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct JointSolverOptions {
    mass_cache: bool,
    warm_start: WarmStartConfig,
    reaction_mode: ReactionMode,
}

impl JointSolverOptions {
    pub(crate) const fn legacy() -> Self {
        Self {
            mass_cache: false,
            warm_start: WarmStartConfig::disabled(),
            reaction_mode: ReactionMode::VelocityOnly,
        }
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) const fn with_mass_cache(mut self, mass_cache: bool) -> Self {
        self.mass_cache = mass_cache;
        self
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) const fn with_warm_start(mut self, warm_start: WarmStartConfig) -> Self {
        self.warm_start = warm_start;
        self
    }

    pub(crate) const fn mass_cache(self) -> bool {
        self.mass_cache
    }

    pub(crate) const fn warm_start(self) -> WarmStartConfig {
        self.warm_start
    }

    pub(crate) const fn reaction_mode(self) -> ReactionMode {
        self.reaction_mode
    }
}

#[derive(Clone, Copy)]
pub(crate) struct JointSolverStep<'a> {
    dt: f64,
    options: JointSolverOptions,
    impulse_cache: Option<&'a JointImpulseCache>,
}

impl<'a> JointSolverStep<'a> {
    pub(crate) const fn new(dt: f64, options: JointSolverOptions) -> Self {
        Self {
            dt,
            options,
            impulse_cache: None,
        }
    }

    pub(crate) const fn with_impulse_cache(mut self, impulse_cache: &'a JointImpulseCache) -> Self {
        self.impulse_cache = Some(impulse_cache);
        self
    }

    pub(crate) const fn dt(self) -> f64 {
        self.dt
    }

    pub(crate) const fn options(self) -> JointSolverOptions {
        self.options
    }

    pub(crate) const fn impulse_cache(self) -> Option<&'a JointImpulseCache> {
        self.impulse_cache
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_solver_options_use_velocity_only_reactions() {
        assert_eq!(
            JointSolverOptions::legacy().reaction_mode(),
            ReactionMode::VelocityOnly
        );
    }
}
