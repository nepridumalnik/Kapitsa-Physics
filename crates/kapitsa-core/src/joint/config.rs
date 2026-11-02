/// Internal joint-solver strategy selection. Public physics configuration
/// stays stable while individual solver strategies are validated.
use super::JointImpulseCache;
use super::reaction::ReactionMode;
use super::warm_start::WarmStartConfig;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct JointTolerance {
    pub(crate) linear_m: f64,
    pub(crate) angular_rad: f64,
    pub(crate) velocity_mps: f64,
    pub(crate) angular_velocity_radps: f64,
}

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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct IterationRange {
    pub(crate) min: u32,
    pub(crate) max: u32,
}

impl IterationRange {
    #[allow(dead_code)]
    pub(crate) const fn new(min: u32, max: u32) -> Self {
        Self { min, max }
    }

    pub(crate) const fn is_valid(self) -> bool {
        self.min > 0 && self.min <= self.max
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum JointProjection {
    #[default]
    ScalarGaussSeidel,
    FixedBlock3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[allow(dead_code)] // Adaptive policy is only selected by tests and benchmark controls.
pub(crate) enum JointIterations {
    #[default]
    Fixed,
    Adaptive {
        position: IterationRange,
        velocity: IterationRange,
        tolerance: JointTolerance,
    },
}

impl JointIterations {
    pub(crate) fn validate(self) -> bool {
        match self {
            Self::Fixed => true,
            Self::Adaptive {
                position,
                velocity,
                tolerance,
            } => position.is_valid() && velocity.is_valid() && tolerance.validate(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct JointSolverOptions {
    mass_cache: bool,
    warm_start: WarmStartConfig,
    reaction_mode: ReactionMode,
    projection: JointProjection,
    iterations: JointIterations,
}

impl JointSolverOptions {
    pub(crate) const fn legacy() -> Self {
        Self {
            mass_cache: false,
            warm_start: WarmStartConfig::disabled(),
            reaction_mode: ReactionMode::VelocityOnly,
            projection: JointProjection::ScalarGaussSeidel,
            iterations: JointIterations::Fixed,
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

    #[cfg(any(test, feature = "bench-internals"))]
    #[allow(dead_code)] // Exposed only to tests and benchmark strategy controls.
    pub(crate) const fn with_projection(mut self, projection: JointProjection) -> Self {
        self.projection = projection;
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

    pub(crate) const fn projection(self) -> JointProjection {
        self.projection
    }

    #[cfg(any(test, feature = "bench-internals"))]
    #[allow(dead_code)] // Exposed only to tests and benchmark strategy controls.
    pub(crate) const fn with_iterations(mut self, iterations: JointIterations) -> Self {
        self.iterations = iterations;
        self
    }

    pub(crate) const fn iterations(self) -> JointIterations {
        self.iterations
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
    fn adaptive_iteration_ranges_and_tolerances_validate() {
        assert!(IterationRange::new(2, 8).is_valid());
        assert!(IterationRange::new(1, 1).is_valid());
        assert!(!IterationRange::new(0, 1).is_valid());
        assert!(!IterationRange::new(4, 3).is_valid());

        let tolerance = JointTolerance {
            linear_m: 1.0e-3,
            angular_rad: 1.0e-3,
            velocity_mps: 1.0e-2,
            angular_velocity_radps: 1.0e-2,
        };
        assert!(tolerance.validate());
        assert!(
            !JointTolerance {
                linear_m: f64::NAN,
                ..tolerance
            }
            .validate()
        );
    }

    #[test]
    fn legacy_solver_options_use_velocity_only_reactions() {
        assert_eq!(
            JointSolverOptions::legacy().reaction_mode(),
            ReactionMode::VelocityOnly
        );
    }

    #[test]
    fn adaptive_iteration_policy_rejects_invalid_ranges_and_tolerances() {
        let tolerance = JointTolerance {
            linear_m: 1.0e-3,
            angular_rad: 1.0e-3,
            velocity_mps: 1.0e-2,
            angular_velocity_radps: 1.0e-2,
        };
        assert!(
            JointIterations::Adaptive {
                position: IterationRange::new(1, 4),
                velocity: IterationRange::new(2, 6),
                tolerance,
            }
            .validate()
        );
        assert!(
            !JointIterations::Adaptive {
                position: IterationRange::new(0, 4),
                velocity: IterationRange::new(2, 6),
                tolerance,
            }
            .validate()
        );
        assert!(
            !JointIterations::Adaptive {
                position: IterationRange::new(1, 4),
                velocity: IterationRange::new(2, 6),
                tolerance: JointTolerance {
                    angular_velocity_radps: f64::INFINITY,
                    ..tolerance
                },
            }
            .validate()
        );
    }
}
