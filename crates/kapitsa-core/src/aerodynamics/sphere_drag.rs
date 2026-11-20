//! Continuum drag coefficients for an isolated sphere.
//!
//! Implements the continuum component of Singh et al., equations 5–13 and
//! 23–26: <https://arxiv.org/abs/2012.04813v1>. This is not their full
//! rarefaction-corrected correlation, nor a general hull or rocket model.
//! The empirical constants are fitted, not independent validation data.

use kapitsa_math::Vec3;

use super::AtmosphericState;

const GAMMA: f64 = 1.4;
const VISCOSITY_EXPONENT: f64 = 0.74;
const BOUNDARY_LAYER_SCALE: f64 = 9.4;
const SHOCK_CURVATURE: f64 = 0.356;
const HIGH_MACH_DRAG: f64 = 0.9;
const STOKES_FACTOR: f64 = 24.0;
const MIN_MACH: f64 = 0.3;
const MAX_MACH: f64 = 2.0;
const MIN_REYNOLDS: f64 = 300.0;
const MAX_REYNOLDS: f64 = 10_000.0;
const MAX_KNUDSEN: f64 = 0.01;
const BROWN_LAWLER_MAX_REYNOLDS: f64 = 200_000.0;
const BROWN_LAWLER_VISCOUS_SCALE: f64 = 0.150;
const BROWN_LAWLER_VISCOUS_EXPONENT: f64 = 0.681;
const BROWN_LAWLER_INERTIAL_SCALE: f64 = 0.407;
const BROWN_LAWLER_TRANSITION_REYNOLDS: f64 = 8_710.0;
const INCOMPRESSIBLE_MAX_MACH: f64 = 0.2;

/// Invalid or outside the deliberately restricted approximation domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SphereDragError {
    InvalidInput,
    UnsupportedDomain,
    ArithmeticOverflow,
}

impl std::fmt::Display for SphereDragError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "sphere drag evaluation failed: {self:?}")
    }
}
impl std::error::Error for SphereDragError {}

/// Drag model for a nonrotating sphere centered at the body's center of mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphereDrag {
    diameter: f64,
    dynamic_viscosity: f64,
}

/// Parameters for [`super::Aerodynamics::IncompressibleSphere`].
pub type IncompressibleSphereDrag = SphereDrag;

#[derive(Clone, Copy)]
pub(crate) enum SphereDragRegime {
    Incompressible,
    Continuum,
    CompressionDominatedParticle,
    SlipCorrectedParticle,
    CreepingParticle,
}

impl SphereDrag {
    /// Sets diameter in metres and fallback dynamic viscosity in Pa·s. Both
    /// must be finite and positive. The selected aerodynamic model determines
    /// the Mach and Reynolds limits. The caller supplies a sphere whose
    /// geometry remains unchanged during the run.
    pub fn new(diameter: f64, dynamic_viscosity: f64) -> Result<Self, crate::Error> {
        if !diameter.is_finite()
            || diameter <= 0.0
            || !dynamic_viscosity.is_finite()
            || dynamic_viscosity <= 0.0
        {
            return Err(SphereDragError::InvalidInput.into());
        }
        Ok(Self {
            diameter,
            dynamic_viscosity,
        })
    }

    pub(crate) fn evaluate(
        self,
        atmosphere: AtmosphericState,
        sound_speed: f64,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
        regime: SphereDragRegime,
    ) -> Result<Vec3, crate::Error> {
        if atmosphere.density() == 0.0 {
            return Ok(Vec3::ZERO);
        }
        let relative_velocity = linear_velocity
            .checked_sub(atmosphere.wind_velocity())
            .ok_or(SphereDragError::ArithmeticOverflow)?;
        if angular_velocity != Vec3::ZERO {
            return Err(SphereDragError::UnsupportedDomain.into());
        }
        let viscosity = atmosphere
            .dynamic_viscosity()
            .unwrap_or(self.dynamic_viscosity);
        if matches!(regime, SphereDragRegime::CreepingParticle) {
            const MAX_CREEPING_REYNOLDS: f64 = 0.1;
            const MAX_CREEPING_MACH: f64 = 0.05;
            return sphere_drag_force(
                self.diameter,
                atmosphere.density(),
                viscosity,
                relative_velocity,
                |speed, reynolds| {
                    let mach = speed / sound_speed;
                    if mach > MAX_CREEPING_MACH || reynolds > MAX_CREEPING_REYNOLDS {
                        return Err(SphereDragError::UnsupportedDomain.into());
                    }
                    // Mach/Re cancels relative speed analytically. Form Kn
                    // directly so underflow in Mach cannot erase rarefaction.
                    let denominator = (atmosphere.density() * sound_speed) * self.diameter;
                    if !denominator.is_finite() || denominator <= 0.0 {
                        return Err(SphereDragError::ArithmeticOverflow.into());
                    }
                    let radius_knudsen =
                        viscosity / denominator * (2.0 * air_mean_free_path_factor());
                    if !radius_knudsen.is_finite() {
                        return Err(SphereDragError::ArithmeticOverflow.into());
                    }
                    creeping_particle_drag_ratio(radius_knudsen)
                },
            );
        }
        let coefficient = match regime {
            SphereDragRegime::Incompressible | SphereDragRegime::CreepingParticle => None,
            SphereDragRegime::SlipCorrectedParticle => {
                Some(slip_corrected_particle_drag_coefficient as fn(f64, f64) -> _)
            }
            SphereDragRegime::Continuum => Some(continuum_drag_coefficient as fn(f64, f64) -> _),
            SphereDragRegime::CompressionDominatedParticle => {
                Some(compression_dominated_particle_drag_coefficient as fn(f64, f64) -> _)
            }
        };
        if let Some(coefficient) = coefficient {
            return compressible_drag_force(
                self.diameter,
                atmosphere.density(),
                viscosity,
                sound_speed,
                relative_velocity,
                coefficient,
            );
        }
        let speed = relative_velocity
            .checked_length()
            .ok_or(SphereDragError::ArithmeticOverflow)?;
        if speed / sound_speed > INCOMPRESSIBLE_MAX_MACH {
            return Err(SphereDragError::UnsupportedDomain.into());
        }
        incompressible_drag_force(
            self.diameter,
            atmosphere.density(),
            viscosity,
            relative_velocity,
        )
    }
}

/// Incompressible drag coefficient for a smooth isolated sphere, referenced
/// to frontal area. Implements Brown and Lawler (2003), equation (19):
/// <https://doi.org/10.1061/(ASCE)0733-9372(2003)129:3(222)>.
///
/// `reynolds` is diameter-based and must satisfy `0 < Re < 200_000`.
/// The caller must establish incompressible continuum conditions. The fit
/// includes the Stokes limit and excludes the drag crisis. It returns a
/// coefficient only. At rest the force is zero and the coefficient is undefined.
pub fn incompressible_drag_coefficient(reynolds: f64) -> Result<f64, crate::Error> {
    if !reynolds.is_finite() || reynolds <= 0.0 {
        return Err(crate::Error::from(SphereDragError::InvalidInput));
    }
    let cd = (STOKES_FACTOR / reynolds) * stokes_multiplier(reynolds)?;
    if !cd.is_finite() {
        return Err(crate::Error::from(SphereDragError::ArithmeticOverflow));
    }
    Ok(cd)
}

/// Drag force in newtons for an isolated sphere in incompressible continuum
/// flow, using Brown and Lawler (2003), equation (19). Reynolds number based
/// on `diameter` must be below 200,000. The force is applied at the center of
/// the sphere and acts opposite `relative_velocity` (body minus fluid, m/s).
/// Diameter is in metres, density in kg/m³ and dynamic viscosity in Pa·s.
/// Diameter and viscosity must be positive. Zero density or zero relative
/// velocity produces zero force. The caller establishes the flow regime.
pub fn incompressible_drag_force(
    diameter: f64,
    density: f64,
    dynamic_viscosity: f64,
    relative_velocity: Vec3,
) -> Result<Vec3, crate::Error> {
    sphere_drag_force(
        diameter,
        density,
        dynamic_viscosity,
        relative_velocity,
        |_, reynolds| Ok(stokes_multiplier(reynolds)?),
    )
}

/// Force in newtons from the restricted compressible continuum sphere model.
/// Uses the domain of [`continuum_drag_coefficient`] and acts at the center of
/// the nonrotating sphere, opposite `relative_velocity` (body minus air, m/s).
/// Diameter is in metres, density in kg/m³, dynamic viscosity in Pa·s and
/// sound speed in m/s. Zero density or zero relative speed produces zero force.
pub fn continuum_drag_force(
    diameter: f64,
    density: f64,
    dynamic_viscosity: f64,
    sound_speed: f64,
    relative_velocity: Vec3,
) -> Result<Vec3, crate::Error> {
    compressible_drag_force(
        diameter,
        density,
        dynamic_viscosity,
        sound_speed,
        relative_velocity,
        continuum_drag_coefficient,
    )
}

fn compressible_drag_force(
    diameter: f64,
    density: f64,
    dynamic_viscosity: f64,
    sound_speed: f64,
    relative_velocity: Vec3,
    coefficient: impl FnOnce(f64, f64) -> Result<f64, crate::Error>,
) -> Result<Vec3, crate::Error> {
    if !sound_speed.is_finite() || sound_speed <= 0.0 {
        return Err(SphereDragError::InvalidInput.into());
    }
    sphere_drag_force(
        diameter,
        density,
        dynamic_viscosity,
        relative_velocity,
        |speed, reynolds| {
            let mach = speed / sound_speed;
            if !mach.is_finite() || mach == 0.0 {
                return Err(SphereDragError::ArithmeticOverflow.into());
            }
            Ok(coefficient(mach, reynolds)? * reynolds / STOKES_FACTOR)
        },
    )
}

fn sphere_drag_force(
    diameter: f64,
    density: f64,
    dynamic_viscosity: f64,
    relative_velocity: Vec3,
    drag_relative_to_stokes: impl FnOnce(f64, f64) -> Result<f64, crate::Error>,
) -> Result<Vec3, crate::Error> {
    if !diameter.is_finite()
        || diameter <= 0.0
        || !density.is_finite()
        || density < 0.0
        || !dynamic_viscosity.is_finite()
        || dynamic_viscosity <= 0.0
        || !relative_velocity.is_finite()
    {
        return Err(SphereDragError::InvalidInput.into());
    }
    if density == 0.0 || relative_velocity == Vec3::ZERO {
        return Ok(Vec3::ZERO);
    }
    let speed = relative_velocity
        .checked_length()
        .ok_or(SphereDragError::ArithmeticOverflow)?;
    let reynolds = (density * speed) * diameter / dynamic_viscosity;
    if !reynolds.is_finite() || reynolds == 0.0 {
        return Err(SphereDragError::ArithmeticOverflow.into());
    }
    // Factor out Stokes drag so slow flow needs neither 1/Re nor speed².
    let correction = drag_relative_to_stokes(speed, reynolds)?;
    let scale = (dynamic_viscosity * diameter) * (3.0 * std::f64::consts::PI * correction);
    let force = relative_velocity
        .checked_mul_scalar(-scale)
        .ok_or(SphereDragError::ArithmeticOverflow)?;
    if force == Vec3::ZERO {
        return Err(SphereDragError::ArithmeticOverflow.into());
    }
    Ok(force)
}

fn stokes_multiplier(reynolds: f64) -> Result<f64, SphereDragError> {
    if reynolds >= BROWN_LAWLER_MAX_REYNOLDS {
        return Err(SphereDragError::UnsupportedDomain);
    }
    Ok(1.0
        + BROWN_LAWLER_VISCOUS_SCALE * reynolds.powf(BROWN_LAWLER_VISCOUS_EXPONENT)
        + (BROWN_LAWLER_INERTIAL_SCALE / STOKES_FACTOR)
            * reynolds
            * (reynolds / (reynolds + BROWN_LAWLER_TRANSITION_REYNOLDS)))
}

/// Low-Mach air drag coefficient of an isolated spherical particle, referenced
/// to frontal area. Uses Schiller–Naumann with the Clift–Gauvin slip correction
/// from Palmer and Sahai (2023), equations 13–14.
///
/// Accepts `0 < Mach ≤ 0.2`, `0 < Re ≤ 45`, and diameter Knudsen number
/// `Kn ≤ 0.1`. These are conservative implementation limits for low-Mach slip
/// flow. All dimensionless inputs refer to the undisturbed air and particle
/// diameter. Kn is inferred using the paper's air relation (gamma 1.4).
pub fn slip_corrected_particle_drag_coefficient(
    mach: f64,
    reynolds: f64,
) -> Result<f64, crate::Error> {
    const MAX_SLIP_KNUDSEN: f64 = 0.1;
    const MAX_RAREFACTION_REYNOLDS: f64 = 45.0;
    const SLIP_LINEAR: f64 = 2.514;
    const SLIP_EXPONENTIAL: f64 = 0.8;
    const SLIP_DECAY: f64 = 0.55;

    if !mach.is_finite() || mach <= 0.0 || !reynolds.is_finite() || reynolds <= 0.0 {
        return Err(SphereDragError::InvalidInput.into());
    }
    let knudsen = air_knudsen(mach, reynolds);
    if mach > INCOMPRESSIBLE_MAX_MACH
        || reynolds > MAX_RAREFACTION_REYNOLDS
        || knudsen > MAX_SLIP_KNUDSEN
    {
        return Err(SphereDragError::UnsupportedDomain.into());
    }
    let slip = 1.0 + knudsen * (SLIP_LINEAR + SLIP_EXPONENTIAL * (-SLIP_DECAY / knudsen).exp());
    let cd = (STOKES_FACTOR / reynolds) * (1.0 + 0.15 * reynolds.powf(0.687)) / slip;
    if !cd.is_finite() {
        return Err(SphereDragError::ArithmeticOverflow.into());
    }
    Ok(cd)
}

fn air_knudsen(mach: f64, reynolds: f64) -> f64 {
    mach / reynolds * air_mean_free_path_factor()
}

/// Frontal-area drag coefficient of an isolated spherical particle in the
/// free-molecular limit. Assumes diffuse reflection and equal gas and particle
/// temperatures. Uses Loth et al. (2021), equation (13), with a series at low
/// speed to avoid cancellation. `speed_ratio` is relative speed divided by
/// the gas's most probable molecular speed and must be finite and positive.
/// `diameter_knudsen` is the mean free path divided by diameter. This model
/// requires `Kn >= 10` as a free-molecular approximation policy.
/// <https://doi.org/10.2514/1.J060153>
pub fn free_molecular_particle_drag_coefficient(
    speed_ratio: f64,
    diameter_knudsen: f64,
) -> Result<f64, crate::Error> {
    const MIN_FREE_MOLECULAR_KNUDSEN: f64 = 10.0;
    const SERIES_MAX_SPEED_RATIO: f64 = 0.05;
    if !speed_ratio.is_finite()
        || speed_ratio <= 0.0
        || !diameter_knudsen.is_finite()
        || diameter_knudsen < 0.0
    {
        return Err(SphereDragError::InvalidInput.into());
    }
    if diameter_knudsen < MIN_FREE_MOLECULAR_KNUDSEN {
        return Err(SphereDragError::UnsupportedDomain.into());
    }
    let sqrt_pi = std::f64::consts::PI.sqrt();
    let square = speed_ratio * speed_ratio;
    let thermal_reemission = 2.0 * sqrt_pi / 3.0;
    let cd = if speed_ratio < SERIES_MAX_SPEED_RATIO {
        // Taylor expansion of s * Cd through s^6. The constant term is
        // the diffuse Epstein limit.
        let incident =
            16.0 / 3.0 + square * (16.0 / 15.0 + square * (-8.0 / 105.0 + square * 8.0 / 945.0));
        (incident / sqrt_pi + thermal_reemission) / speed_ratio
    } else {
        let inverse = speed_ratio.recip();
        let inverse_square = inverse * inverse;
        (-square).exp() * inverse * (2.0 + inverse_square) / sqrt_pi
            + (2.0 + inverse_square * (2.0 - 0.5 * inverse_square)) * libm::erf(speed_ratio)
            + thermal_reemission * inverse
    };
    if !cd.is_finite() || cd <= 0.0 {
        return Err(SphereDragError::ArithmeticOverflow.into());
    }
    Ok(cd)
}

fn air_mean_free_path_factor() -> f64 {
    (GAMMA * std::f64::consts::PI / 2.0).sqrt()
}

/// Ratio of drag to Stokes drag for a slowly translating isolated sphere with
/// diffuse molecular reflection. `radius_knudsen` is the gas mean free path
/// divided by particle radius, and must be finite and nonnegative.
///
/// Implements equations (2.1), (2.7) and section 2.2 of
/// [A correction tensor for approximating drag on slow-moving particles](https://www.cambridge.org/core/journals/journal-of-fluid-mechanics/article/correction-tensor-for-approximating-drag-on-slowmoving-particles-of-arbitrary-shape-and-knudsen-number/24A36A6C4A6CB3301E18A6A596756095).
/// This heuristic joins the continuum and free-molecular limits in creeping
/// flow. The caller establishes negligible inertia and compressibility.
pub fn creeping_particle_drag_ratio(radius_knudsen: f64) -> Result<f64, crate::Error> {
    const MAXWELL_SLIP: f64 = 1.0;
    const EPSTEIN_LIMIT: f64 = 18.0 / (8.0 + std::f64::consts::PI);
    if !radius_knudsen.is_finite() || radius_knudsen < 0.0 {
        return Err(SphereDragError::InvalidInput.into());
    }
    let continuum_term = (-(EPSTEIN_LIMIT - MAXWELL_SLIP) * radius_knudsen).exp();
    // Divide before multiplying by Kn when the denominator could overflow.
    let ratio = if radius_knudsen > 1.0 {
        radius_knudsen.recip() / (EPSTEIN_LIMIT + continuum_term / radius_knudsen)
    } else {
        (continuum_term + EPSTEIN_LIMIT * radius_knudsen).recip()
    };
    Ok(ratio)
}

/// Compression-dominated, precritical drag coefficient of an isolated spherical
/// particle, referenced to frontal area. Uses Loth et al. (2021), equations
/// 6–8, as reproduced by Palmer and Sahai (2023), equations 19–22.
///
/// Accepts Mach 0.25–10 and diameter Reynolds number `45 < Re < 200_000`.
/// This is the compression-dominated branch, excluding the rarefaction branch
/// and drag crisis. At each piecewise breakpoint the lower-Mach formula applies.
/// <https://doi.org/10.2514/1.J060153>
pub fn compression_dominated_particle_drag_coefficient(
    mach: f64,
    reynolds: f64,
) -> Result<f64, crate::Error> {
    const MIN_MACH: f64 = 0.25;
    const MAX_MACH: f64 = 10.0;
    const MIN_REYNOLDS: f64 = 45.0;
    const MAX_REYNOLDS: f64 = 200_000.0;

    if !mach.is_finite() || mach <= 0.0 || !reynolds.is_finite() || reynolds <= 0.0 {
        return Err(SphereDragError::InvalidInput.into());
    }
    if !(MIN_MACH..=MAX_MACH).contains(&mach)
        || reynolds <= MIN_REYNOLDS
        || reynolds >= MAX_REYNOLDS
    {
        return Err(SphereDragError::UnsupportedDomain.into());
    }
    // Empirical Mach functions C_M, G_M and H_M from the published fit.
    let pressure_scale = if mach <= 1.5 {
        1.65 + 0.65 * (4.0 * mach - 3.4).tanh()
    } else {
        2.18 - 0.13 * (0.9 * mach - 2.7).tanh()
    };
    let pressure_reynolds_correction = if mach <= 0.8 {
        ((166.0 * mach + 3.29) * mach - 10.9) * mach + 20.0
    } else {
        5.0 + 40.0 / mach.powi(3)
    };
    let viscous_scale = if mach <= 1.0 {
        ((0.0239 * mach + 0.212) * mach - 0.074) * mach + 1.0
    } else {
        0.93 + 1.0 / (3.5 + mach.powi(5))
    };
    // The bounded inputs keep every intermediate finite and positive where
    // used as a denominator or power base.
    let viscous = STOKES_FACTOR / reynolds * (1.0 + 0.15 * reynolds.powf(0.687)) * viscous_scale;
    let pressure = 0.42 * pressure_scale
        / (1.0
            + 42_500.0 / reynolds.powf(1.16 * pressure_scale)
            + pressure_reynolds_correction / reynolds.sqrt());
    Ok(viscous + pressure)
}

/// Continuum drag coefficient, referenced to the sphere's frontal area.
///
/// Inputs are freestream Mach and diameter-based Reynolds numbers. Only air
/// (gamma 1.4, viscosity power 0.74), Mach 0.3–2, Re 300–10000 and Kn < 0.01
/// are accepted. These conservative limits are implementation policy, not an
/// accuracy guarantee. The caller must ensure an isolated stationary or
/// translating non-rotating sphere in uniform flow; wall temperature,
/// rarefaction corrections, interference and drag crisis are not modeled.
/// This returns a coefficient only and is not integrated into the world.
pub fn continuum_drag_coefficient(mach: f64, reynolds: f64) -> Result<f64, crate::Error> {
    if !mach.is_finite() || !reynolds.is_finite() || mach <= 0.0 || reynolds <= 0.0 {
        return Err(crate::Error::from(SphereDragError::InvalidInput));
    }
    let knudsen = air_knudsen(mach, reynolds);
    if !(MIN_MACH..=MAX_MACH).contains(&mach)
        || !(MIN_REYNOLDS..=MAX_REYNOLDS).contains(&reynolds)
        || knudsen >= MAX_KNUDSEN
    {
        return Err(crate::Error::from(SphereDragError::UnsupportedDomain));
    }
    let theta = |m: f64| (1.0 + (GAMMA - 1.0) * m * m / 2.0).powf(GAMMA / (GAMMA - 1.0));
    let exponent = (GAMMA + 1.0) / (2.0 * GAMMA) - (GAMMA - 1.0) * VISCOSITY_EXPONENT / GAMMA;
    let c0 = STOKES_FACTOR / BOUNDARY_LAYER_SCALE.powi(2);
    let (effective_mach, reynolds_factor, pressure) = if mach <= 1.0 {
        (mach, 1.0, 0.0)
    } else {
        let m2 = mach * mach;
        let velocity_ratio = (2.0 + (GAMMA - 1.0) * m2) / ((GAMMA + 1.0) * m2);
        let temperature_ratio = ((GAMMA - 1.0) * m2 + 2.0) * (2.0 * GAMMA * m2 - (GAMMA - 1.0))
            / ((GAMMA + 1.0).powi(2) * m2);
        let post_mach = (((GAMMA - 1.0) * m2 + 2.0) / (2.0 * GAMMA * m2 - (GAMMA - 1.0))).sqrt();
        let alpha = 1.0 / (SHOCK_CURVATURE * mach + 1.0 - SHOCK_CURVATURE);
        let theta_inf = (1.0 + (GAMMA - 1.0).powi(2) / (4.0 * GAMMA)).powf(GAMMA / (GAMMA - 1.0));
        let c1 = (HIGH_MACH_DRAG - c0 * theta_inf)
            / (1.0 - (GAMMA - 1.0) / ((GAMMA + 1.0) * SHOCK_CURVATURE * mach));
        (
            post_mach,
            (1.0 / (alpha * alpha * temperature_ratio)).powf(VISCOSITY_EXPONENT),
            c1 * (1.0 - alpha * velocity_ratio),
        )
    };
    let theta = theta(effective_mach);
    let effective_re = reynolds * reynolds_factor * theta.powf(exponent);
    let cd = pressure + c0 * theta * (1.0 + BOUNDARY_LAYER_SCALE / effective_re.sqrt()).powi(2);
    if !cd.is_finite() || cd < 0.0 {
        return Err(crate::Error::from(SphereDragError::ArithmeticOverflow));
    }
    Ok(cd)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_molecular_particle_drag_matches_kinetic_limits() -> Result<(), crate::Error> {
        let kn = 10.0;
        let slow = 1e-150;
        let sqrt_pi = std::f64::consts::PI.sqrt();
        let epstein = 16.0 / (3.0 * sqrt_pi) + 2.0 * sqrt_pi / 3.0;
        assert!(
            (slow * free_molecular_particle_drag_coefficient(slow, kn)? - epstein).abs() < 1e-14
        );
        assert!(
            (free_molecular_particle_drag_coefficient(1.0, kn)? - 4.753_749_922_058_572).abs()
                < 1e-14
        );
        assert_eq!(free_molecular_particle_drag_coefficient(f64::MAX, kn)?, 2.0);
        let boundary = 0.05_f64;
        let before = free_molecular_particle_drag_coefficient(boundary.next_down(), kn)?;
        let after = free_molecular_particle_drag_coefficient(boundary, kn)?;
        assert!((before - after).abs() < 1e-10);
        assert_eq!(
            free_molecular_particle_drag_coefficient(1.0, kn.next_down()),
            Err(SphereDragError::UnsupportedDomain.into())
        );
        assert_eq!(
            free_molecular_particle_drag_coefficient(0.0, kn),
            Err(SphereDragError::InvalidInput.into())
        );
        Ok(())
    }

    #[test]
    fn creeping_particle_keeps_rarefaction_when_mach_underflows() -> Result<(), crate::Error> {
        let speed = 1e-320;
        let sound_speed = 1e4;
        let diameter = 0.001;
        let density = 1e4;
        let viscosity = 1e4;
        assert_eq!(speed / sound_speed, 0.0);
        let force = SphereDrag::new(diameter, viscosity)?.evaluate(
            AtmosphericState::new(density, Vec3::ZERO)?,
            sound_speed,
            Vec3::X * speed,
            Vec3::ZERO,
            SphereDragRegime::CreepingParticle,
        )?;
        let radius_kn = (0.7 * std::f64::consts::PI).sqrt() / 5.0;
        let expected = 3.0
            * std::f64::consts::PI
            * viscosity
            * diameter
            * creeping_particle_drag_ratio(radius_kn)?;
        assert!((force.x / -speed - expected).abs() < 0.001);
        Ok(())
    }

    #[test]
    fn creeping_particle_drag_recovers_continuum_and_free_molecular_limits()
    -> Result<(), crate::Error> {
        assert_eq!(creeping_particle_drag_ratio(0.0)?, 1.0);
        let low_kn = 1e-5;
        let slip_slope = (1.0 - creeping_particle_drag_ratio(low_kn)?) / low_kn;
        assert!((slip_slope - 1.0).abs() < 1e-4);
        let transition = creeping_particle_drag_ratio(1.0)?;
        assert!(transition < 1.0 && transition > creeping_particle_drag_ratio(10.0)?);
        let high_kn = f64::MAX;
        let scaled_drag = creeping_particle_drag_ratio(high_kn)? * high_kn;
        assert!((scaled_drag - (8.0 + std::f64::consts::PI) / 18.0).abs() < 1e-14);
        for kn in [-1.0, f64::INFINITY, f64::NAN] {
            assert_eq!(
                creeping_particle_drag_ratio(kn),
                Err(SphereDragError::InvalidInput.into())
            );
        }
        Ok(())
    }

    #[test]
    fn slip_correction_recovers_continuum_limit_and_rejects_transitional_flow()
    -> Result<(), crate::Error> {
        let continuum = slip_corrected_particle_drag_coefficient(f64::MIN_POSITIVE, 1.0)?;
        assert!((continuum - 27.6).abs() < 1e-13);
        let slip = slip_corrected_particle_drag_coefficient(0.05, 1.0)?;
        assert!((slip - 23.262_844_124_467_1).abs() < 1e-13);
        assert!(slip < continuum);
        for (mach, re) in [(0.1, 1.0), (0.21, 45.0), (0.05, 45.01)] {
            assert_eq!(
                slip_corrected_particle_drag_coefficient(mach, re),
                Err(SphereDragError::UnsupportedDomain.into())
            );
        }
        assert_eq!(
            slip_corrected_particle_drag_coefficient(0.0, 1.0),
            Err(SphereDragError::InvalidInput.into())
        );
        Ok(())
    }

    #[test]
    fn compression_dominated_particle_fit_covers_published_mach_branches()
    -> Result<(), crate::Error> {
        // Equation values, not measurements.
        for (mach, expected) in [
            (0.25, 0.573_600_708_844_753_6),
            (0.8, 0.716_824_977_946_283),
            (1.0, 0.930_324_194_486_603_4),
            (1.5, 1.122_263_737_312_958_7),
            (2.0, 1.197_339_242_085_612_4),
            (10.0, 1.216_732_110_402_699_1),
        ] {
            let cd = compression_dominated_particle_drag_coefficient(mach, 500.0)?;
            assert!((cd - expected).abs() < 1e-13);
        }
        for (mach, re) in [(0.24, 500.0), (10.01, 500.0), (1.0, 45.0), (1.0, 200_000.0)] {
            assert_eq!(
                compression_dominated_particle_drag_coefficient(mach, re),
                Err(SphereDragError::UnsupportedDomain.into())
            );
        }
        Ok(())
    }

    #[test]
    fn continuum_force_matches_frontal_area_drag_through_mach_one() -> Result<(), crate::Error> {
        let diameter = 0.01;
        let viscosity = 0.001;
        let density = 1.0;
        let sound_speed = 100.0;
        for mach in [0.3, 0.99, 1.0, 1.01, 2.0] {
            let speed = mach * sound_speed;
            let cd = continuum_drag_coefficient(mach, speed * 10.0)?;
            let expected = 0.5
                * density
                * speed
                * speed
                * std::f64::consts::PI
                * (diameter / 2.0_f64).powi(2)
                * cd;
            let force =
                continuum_drag_force(diameter, density, viscosity, sound_speed, Vec3::Y * speed)?;
            assert!((force.y / expected + 1.0).abs() < 1e-14);
            assert_eq!(force.x, 0.0);
            assert_eq!(force.z, 0.0);
        }
        assert_eq!(
            continuum_drag_force(diameter, 0.0, viscosity, sound_speed, Vec3::X),
            Ok(Vec3::ZERO)
        );
        assert_eq!(
            continuum_drag_force(diameter, density, viscosity, sound_speed, Vec3::ZERO),
            Ok(Vec3::ZERO)
        );
        assert_eq!(
            continuum_drag_force(diameter, density, viscosity, sound_speed, Vec3::X * 210.0),
            Err(SphereDragError::UnsupportedDomain.into())
        );
        Ok(())
    }

    #[test]
    fn incompressible_force_matches_dynamic_pressure_and_stokes_limit() -> Result<(), crate::Error>
    {
        let velocity = Vec3::new(3.0, -4.0, 0.0);
        let diameter = 0.1;
        let viscosity = 0.001;
        let density = 2.0;
        let cd = incompressible_drag_coefficient(1000.0)?;
        let area = std::f64::consts::PI * diameter * diameter / 4.0;
        let expected = -velocity * (0.5 * density * 5.0 * area * cd);
        let force = incompressible_drag_force(diameter, density, viscosity, velocity)?;
        assert!((force - expected).length() < 1e-15);

        let speed = 1e-200;
        let creeping = incompressible_drag_force(1.0, 1.0, 1.0, Vec3::X * speed)?;
        assert!((creeping.x / speed + 3.0 * std::f64::consts::PI).abs() < 1e-14);
        assert_eq!(
            incompressible_drag_force(1.0, 0.0, 1.0, velocity)?,
            Vec3::ZERO
        );
        assert_eq!(
            incompressible_drag_force(1.0, 1.0, 1.0, Vec3::ZERO)?,
            Vec3::ZERO
        );
        assert_eq!(
            incompressible_drag_force(1.0, 1.0, 0.0, velocity),
            Err(SphereDragError::InvalidInput.into())
        );
        assert_eq!(
            incompressible_drag_force(1.0, 1.0, 1.0, Vec3::X * 200_000.0),
            Err(SphereDragError::UnsupportedDomain.into())
        );
        Ok(())
    }

    #[test]
    fn incompressible_fit_reaches_stokes_limit_and_precrisis_domain() -> Result<(), crate::Error> {
        let creeping_re = 1e-12;
        assert!((incompressible_drag_coefficient(creeping_re)? * creeping_re - 24.0).abs() < 1e-7);
        let cd_at_1000 = incompressible_drag_coefficient(1_000.0)?;
        assert!((cd_at_1000 - 0.463_383_854_143_119_2).abs() < 1e-14);
        assert!(incompressible_drag_coefficient(BROWN_LAWLER_MAX_REYNOLDS.next_down())? > 0.0);
        Ok(())
    }

    #[test]
    fn incompressible_fit_checks_input_and_representability() {
        for re in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(
                incompressible_drag_coefficient(re),
                Err(SphereDragError::InvalidInput.into())
            );
        }
        assert_eq!(
            incompressible_drag_coefficient(200_000.0),
            Err(SphereDragError::UnsupportedDomain.into())
        );
        assert_eq!(
            incompressible_drag_coefficient(f64::MIN_POSITIVE),
            Err(SphereDragError::ArithmeticOverflow.into())
        );
    }

    // Diagnostic comparison, not an accuracy acceptance test. Nagata et al.,
    // JFM 904 A36 (2020), table 3, https://doi.org/10.1017/jfm.2020.629.
    // DNS uses an adiabatic wall and Sutherland viscosity; this implementation
    // uses a fixed viscosity exponent and omits rarefaction and wall heating.
    // The two grids do not provide a complete DNS uncertainty estimate.
    #[test]
    fn compare_nagata_dns_grid_resolutions() -> Result<(), crate::Error> {
        for (re, mach, base, fine) in [
            (300.0, 0.3, 0.6587, 0.6644),
            (300.0, 2.0, 1.452, 1.458),
            (500.0, 0.3, 0.5658, 0.5679),
            (500.0, 2.0, 1.331, 1.336),
            (750.0, 0.3, 0.5078, 0.5085),
            (750.0, 2.0, 1.251, 1.254),
        ] {
            let predicted = continuum_drag_coefficient(mach, re)?;
            let relative_error = 100.0 * (predicted - fine) / fine;
            let particle = compression_dominated_particle_drag_coefficient(mach, re)?;
            let particle_error = 100.0 * (particle - fine) / fine;
            println!(
                "Re={re:.0} M={mach:.1} DNS={base:.4}/{fine:.4} model={predicted:.6} error={relative_error:+.2}%"
            );
            println!("  Loth={particle:.6} error={particle_error:+.2}%");
            if mach == MIN_MACH {
                let cd = incompressible_drag_coefficient(re)?;
                let error = 100.0 * (cd - fine) / fine;
                let model_delta = 100.0 * (predicted / cd - 1.0);
                println!(
                    "  incompressible={cd:.6} error={error:+.2}% model delta={model_delta:+.2}%"
                );
            }
            assert!(predicted.is_finite());
            assert!(particle.is_finite());
        }
        Ok(())
    }

    #[test]
    fn continuum_branches_meet_at_mach_one() {
        for (re, expected) in [
            (300.0, 1.068_607_600),
            (500.0, 0.926_120_036),
            (750.0, 0.841_505_917),
            (10_000.0, 0.595_822_325),
        ] {
            let center = continuum_drag_coefficient(1.0, re).unwrap();
            assert!((center - expected).abs() < 1e-9);
            for mach in [1.0 - 1e-8, 1.0 + 1e-8] {
                assert!((continuum_drag_coefficient(mach, re).unwrap() - center).abs() < 1e-7);
            }
        }
    }

    #[test]
    fn rejects_inputs_outside_documented_domain() {
        for (m, re) in [(f64::NAN, 300.0), (1.0, 0.0), (-1.0, 300.0)] {
            assert_eq!(
                continuum_drag_coefficient(m, re),
                Err(crate::Error::from(SphereDragError::InvalidInput))
            );
        }
        for (m, re) in [(0.29, 300.0), (2.01, 300.0), (1.0, 299.0), (1.0, 10_001.0)] {
            assert_eq!(
                continuum_drag_coefficient(m, re),
                Err(crate::Error::from(SphereDragError::UnsupportedDomain))
            );
        }
        for (m, re) in [(0.3, 300.0), (2.0, 300.0), (1.0, 10_000.0)] {
            assert!(continuum_drag_coefficient(m, re).is_ok());
        }
    }
}
