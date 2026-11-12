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
    Exponential { scale_height: f64 },
}

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

    pub fn center(self) -> Vec3 {
        self.center
    }
    pub fn reference_radius(self) -> f64 {
        self.reference_radius
    }
    pub fn reference_density(self) -> f64 {
        self.reference_density
    }
    pub fn height(self) -> f64 {
        self.height
    }
    pub fn outer_radius(self) -> f64 {
        self.outer_radius
    }
    pub fn profile(self) -> DensityProfile {
        self.profile
    }
    pub fn smooth_edge(self) -> Option<f64> {
        self.smooth_edge
    }
    pub fn wind(self) -> Vec3 {
        self.wind
    }
}

/// Required: reference radius, reference density, height and profile.
/// Defaults: origin, no wind, hard cutoff.
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
    pub fn center(mut self, value: Vec3) -> Self {
        self.center = value;
        self
    }
    pub fn reference_radius(mut self, value: f64) -> Self {
        self.reference_radius = Some(value);
        self
    }
    pub fn reference_density(mut self, value: f64) -> Self {
        self.reference_density = Some(value);
        self
    }
    pub fn height(mut self, value: f64) -> Self {
        self.height = Some(value);
        self
    }
    pub fn profile(mut self, value: DensityProfile) -> Self {
        self.profile = Some(value);
        self
    }
    pub fn smooth_edge(mut self, value: f64) -> Self {
        self.smooth_edge = Some(value);
        self
    }
    pub fn wind(mut self, value: Vec3) -> Self {
        self.wind = value;
        self
    }

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
