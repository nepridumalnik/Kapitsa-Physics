//! Spherical density fields and their validated configuration.
use super::{AerodynamicsError, AtmosphericState};
use kapitsa_math::Vec3;

/// Atmosphere configured for a world. Vacuum is the default.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Atmosphere {
    #[default]
    Vacuum,
    Sphere(SphericalAtmosphere),
}

impl Atmosphere {
    /// Samples conditions at a world-space point.
    pub fn state_at(self, point: Vec3) -> Result<AtmosphericState, AerodynamicsError> {
        if !point.is_finite() {
            return Err(AerodynamicsError::InvalidAtmosphere(
                "sample point must be finite",
            ));
        }
        match self {
            Self::Vacuum => AtmosphericState::new(0.0, Vec3::ZERO),
            Self::Sphere(sphere) => sphere.state_at(point),
        }
    }

    pub(crate) fn wind_velocity(self) -> Vec3 {
        match self {
            Self::Vacuum => Vec3::ZERO,
            Self::Sphere(sphere) => sphere.wind,
        }
    }
}

/// Density decay above the reference radius; heights are in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DensityProfile {
    /// `density = reference_density * exp(-altitude / scale_height)`.
    Exponential {
        /// Positive, finite height in metres over which density decreases by a factor of `e`.
        scale_height: f64,
    },
}

/// A spherical density field with constant wind, configured with
/// [`SphericalAtmosphere::builder`]. Distances are measured from its world-space centre.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphericalAtmosphere {
    center: Vec3,
    reference_radius: f64,
    reference_density: f64,
    height: f64,
    outer_radius: f64,
    profile: DensityProfile,
    smooth_edge: Option<f64>,
    wind: Vec3,
}

impl SphericalAtmosphere {
    /// Starts a builder; see [`SphericalAtmosphereBuilder`] for a complete example.
    pub fn builder() -> SphericalAtmosphereBuilder {
        SphericalAtmosphereBuilder::default()
    }

    /// Density is constant below the reference radius and zero at/beyond the outer radius.
    pub fn state_at(self, point: Vec3) -> Result<AtmosphericState, AerodynamicsError> {
        if !point.is_finite() {
            return Err(AerodynamicsError::InvalidAtmosphere(
                "sample point must be finite",
            ));
        }
        let distance = point
            .checked_sub(self.center)
            .and_then(|v| v.checked_length())
            .ok_or(AerodynamicsError::ArithmeticOverflow)?;
        if distance >= self.outer_radius {
            return AtmosphericState::new(0.0, self.wind);
        }
        let altitude = (distance - self.reference_radius).max(0.0);
        let factor = match self.profile {
            DensityProfile::Exponential { scale_height } => (-altitude / scale_height).exp(),
        };
        let edge = match self.smooth_edge {
            Some(width) => {
                let t = ((self.outer_radius - distance) / width).clamp(0.0, 1.0);
                t * t * (3.0 - 2.0 * t)
            }
            None => 1.0,
        };
        AtmosphericState::new(self.reference_density * factor * edge, self.wind)
    }

    /// World-space centre in metres.
    pub fn center(self) -> Vec3 {
        self.center
    }
    /// Reference radius from the centre, in metres.
    pub fn reference_radius(self) -> f64 {
        self.reference_radius
    }
    /// Density at and below the reference radius, in kg/m^3.
    pub fn reference_density(self) -> f64 {
        self.reference_density
    }
    /// Atmosphere height above the reference radius, in metres.
    pub fn height(self) -> f64 {
        self.height
    }
    /// Cutoff radius from the centre, in metres.
    pub fn outer_radius(self) -> f64 {
        self.outer_radius
    }
    /// Configured density decay formula.
    pub fn profile(self) -> DensityProfile {
        self.profile
    }
    /// Optional smoothing width in metres; `None` means a hard cutoff.
    pub fn smooth_edge(self) -> Option<f64> {
        self.smooth_edge
    }
    /// Constant world-space wind velocity in m/s.
    pub fn wind(self) -> Vec3 {
        self.wind
    }
}

/// Required: reference radius, reference density, height and profile.
/// Defaults: origin, no wind, hard cutoff. Repeated setters replace the previous value;
/// validation happens in [`Self::build`].
///
/// # Example
///
/// ```
/// use kapitsa_core::aerodynamics::{Atmosphere, DensityProfile, SphericalAtmosphere};
/// use kapitsa_core::world::PhysicsWorld;
/// use kapitsa_math::Vec3;
///
/// let sphere = SphericalAtmosphere::builder()
///     .center(Vec3::ZERO)
///     .reference_radius(600_000.0)
///     .reference_density(1.225)
///     .height(80_000.0)
///     .profile(DensityProfile::Exponential { scale_height: 5_600.0 })
///     .smooth_edge(5_600.0)
///     .wind(Vec3::X * 10.0)
///     .build()?;
///
/// let mut world = PhysicsWorld::new();
/// world.set_atmosphere(Atmosphere::Sphere(sphere));
/// let surface = sphere.state_at(Vec3::Y * 600_000.0)?;
/// assert_eq!(surface.density(), 1.225);
/// world.set_atmosphere(Atmosphere::Vacuum);
/// # Ok::<(), kapitsa_core::aerodynamics::AerodynamicsError>(())
/// ```
#[derive(Clone, Copy, Debug, Default)]
pub struct SphericalAtmosphereBuilder {
    center: Vec3,
    reference_radius: Option<f64>,
    reference_density: Option<f64>,
    height: Option<f64>,
    profile: Option<DensityProfile>,
    smooth_edge: Option<f64>,
    wind: Vec3,
}

impl SphericalAtmosphereBuilder {
    /// Sets the centre in world coordinates, in metres. Components must be finite.
    /// Defaults to `Vec3::ZERO`.
    pub fn center(mut self, value: Vec3) -> Self {
        self.center = value;
        self
    }
    /// Sets the reference radius from the centre, in metres. Required, finite and non-negative.
    /// Density is constant at the reference density at and below this radius.
    pub fn reference_radius(mut self, value: f64) -> Self {
        self.reference_radius = Some(value);
        self
    }
    /// Sets density at the reference radius, in kg/m^3. Required, finite and non-negative.
    /// Zero density disables aerodynamic forces throughout the sphere.
    pub fn reference_density(mut self, value: f64) -> Self {
        self.reference_density = Some(value);
        self
    }
    /// Sets the atmosphere height above the reference radius, in metres. Required and positive.
    /// The sum with the reference radius must be finite and strictly greater than that radius.
    /// At and beyond this outer radius, density is zero.
    pub fn height(mut self, value: f64) -> Self {
        self.height = Some(value);
        self
    }
    /// Selects the density decay formula above the reference radius. Required.
    /// Profile parameters are validated by `build()`.
    pub fn profile(mut self, value: DensityProfile) -> Self {
        self.profile = Some(value);
        self
    }
    /// Enables smooth density tapering over the final `value` metres inside the outer boundary.
    /// The width must be finite, positive and no greater than the atmosphere height.
    /// Multiplies the profile by `t * t * (3.0 - 2.0 * t)`, where `t` is the remaining distance
    /// to the boundary divided by the width, clamped to `[0, 1]`. Defaults to a hard cutoff.
    pub fn smooth_edge(mut self, value: f64) -> Self {
        self.smooth_edge = Some(value);
        self
    }
    /// Sets constant wind velocity in world coordinates, in m/s. Components must be finite.
    /// Defaults to `Vec3::ZERO`.
    pub fn wind(mut self, value: Vec3) -> Self {
        self.wind = value;
        self
    }

    /// Validates all settings and constructs the atmosphere.
    ///
    /// # Errors
    /// Returns [`AerodynamicsError`] for missing required fields, non-finite values,
    /// invalid profile/edge parameters, or an unrepresentable outer radius.
    pub fn build(self) -> Result<SphericalAtmosphere, AerodynamicsError> {
        let invalid = AerodynamicsError::InvalidAtmosphere;
        let reference_radius = self
            .reference_radius
            .ok_or(invalid("reference radius is required"))?;
        let reference_density = self
            .reference_density
            .ok_or(invalid("reference density is required"))?;
        let height = self.height.ok_or(invalid("height is required"))?;
        let profile = self.profile.ok_or(invalid("density profile is required"))?;
        if !self.center.is_finite() {
            return Err(invalid("center must be finite"));
        }
        AtmosphericState::new(reference_density, self.wind)?;
        if !reference_radius.is_finite() || reference_radius < 0.0 {
            return Err(invalid("reference radius must be finite and non-negative"));
        }
        let outer_radius = reference_radius + height;
        if !height.is_finite()
            || height <= 0.0
            || !outer_radius.is_finite()
            || outer_radius <= reference_radius
        {
            return Err(invalid(
                "height must produce a finite outer radius greater than the reference radius",
            ));
        }
        match profile {
            DensityProfile::Exponential { scale_height }
                if !scale_height.is_finite() || scale_height <= 0.0 =>
            {
                return Err(invalid("scale height must be finite and positive"));
            }
            _ => {}
        }
        if let Some(width) = self.smooth_edge
            && (!width.is_finite() || width <= 0.0 || width > height)
        {
            return Err(invalid(
                "edge width must be positive and no greater than atmosphere height",
            ));
        }
        Ok(SphericalAtmosphere {
            center: self.center,
            reference_radius,
            reference_density,
            height,
            outer_radius,
            profile,
            smooth_edge: self.smooth_edge,
            wind: self.wind,
        })
    }
}

#[cfg(test)]
pub(crate) fn test_atmosphere(state: AtmosphericState) -> Atmosphere {
    Atmosphere::Sphere(SphericalAtmosphere {
        center: Vec3::ZERO,
        reference_radius: 1.0e100,
        reference_density: state.density(),
        height: 1.0e100,
        outer_radius: 2.0e100,
        profile: DensityProfile::Exponential {
            scale_height: 1.0e100,
        },
        smooth_edge: None,
        wind: state.wind_velocity(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn builder() -> SphericalAtmosphereBuilder {
        SphericalAtmosphere::builder()
            .center(Vec3::X * 10.0)
            .reference_radius(100.0)
            .reference_density(2.0)
            .height(50.0)
            .profile(DensityProfile::Exponential { scale_height: 10.0 })
    }

    #[test]
    fn samples_reference_profile_and_cutoff() -> Result<(), AerodynamicsError> {
        let sphere = builder().build()?;
        assert_eq!(sphere.state_at(Vec3::X * 10.0)?.density(), 2.0);
        assert_eq!(sphere.state_at(Vec3::X * 110.0)?.density(), 2.0);
        assert!(
            (sphere.state_at(Vec3::X * 120.0)?.density() - 2.0 / std::f64::consts::E).abs() < 1e-14
        );
        assert_eq!(sphere.state_at(Vec3::X * 160.0)?.density(), 0.0);
        assert_eq!(sphere.state_at(Vec3::X * 170.0)?.density(), 0.0);
        Ok(())
    }

    #[test]
    fn smooth_edge_preserves_reference_and_tapers_inside_top() -> Result<(), AerodynamicsError> {
        let sphere = builder().smooth_edge(10.0).build()?;
        assert_eq!(sphere.state_at(Vec3::X * 110.0)?.density(), 2.0);
        let hard = builder().build()?;
        let point = Vec3::X * 155.0;
        assert!(
            (sphere.state_at(point)?.density() / hard.state_at(point)?.density() - 0.5).abs()
                < 1e-14
        );
        Ok(())
    }

    #[test]
    fn validates_required_fields_and_numeric_limits() {
        assert!(SphericalAtmosphere::builder().build().is_err());
        for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(builder().height(value).build().is_err());
            assert!(builder().smooth_edge(value).build().is_err());
            assert!(
                builder()
                    .profile(DensityProfile::Exponential {
                        scale_height: value
                    })
                    .build()
                    .is_err()
            );
        }
        assert!(builder().smooth_edge(51.0).build().is_err());
        assert!(
            builder()
                .reference_radius(f64::MAX)
                .height(f64::MAX)
                .build()
                .is_err()
        );
        assert!(builder().reference_density(-1.0).build().is_err());
        assert!(builder().center(Vec3::X * f64::INFINITY).build().is_err());
    }
}
