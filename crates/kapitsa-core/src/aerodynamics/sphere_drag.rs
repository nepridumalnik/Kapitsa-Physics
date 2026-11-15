//! Restricted continuum approximation for drag on an isolated sphere.
//!
//! Implements the continuum component of Singh et al., equations 5–13 and
//! 23–26: <https://arxiv.org/abs/2012.04813v1>. This is not their full
//! rarefaction-corrected correlation, nor a general hull or rocket model.
//! The empirical constants are fitted, not independent validation data.

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

/// Continuum drag coefficient, referenced to the sphere's frontal area.
///
/// Inputs are freestream Mach and diameter-based Reynolds numbers. Only air
/// (gamma 1.4, viscosity power 0.74), Mach 0.3–2, Re 300–10000 and Kn < 0.01
/// are accepted. These conservative limits are implementation policy, not an
/// accuracy guarantee. The caller must ensure an isolated stationary or
/// translating non-rotating sphere in uniform flow; wall temperature,
/// rarefaction corrections, interference and drag crisis are not modeled.
/// This returns a coefficient only and is not integrated into the world.
pub fn continuum_drag_coefficient(mach: f64, reynolds: f64) -> Result<f64, SphereDragError> {
    if !mach.is_finite() || !reynolds.is_finite() || mach <= 0.0 || reynolds <= 0.0 {
        return Err(SphereDragError::InvalidInput);
    }
    let knudsen = mach / reynolds * (GAMMA * std::f64::consts::PI / 2.0).sqrt();
    if !(MIN_MACH..=MAX_MACH).contains(&mach)
        || !(MIN_REYNOLDS..=MAX_REYNOLDS).contains(&reynolds)
        || knudsen >= MAX_KNUDSEN
    {
        return Err(SphereDragError::UnsupportedDomain);
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
        return Err(SphereDragError::ArithmeticOverflow);
    }
    Ok(cd)
}

#[cfg(test)]
mod tests {
    use super::*;

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
                Err(SphereDragError::InvalidInput)
            );
        }
        for (m, re) in [(0.29, 300.0), (2.01, 300.0), (1.0, 299.0), (1.0, 10_001.0)] {
            assert_eq!(
                continuum_drag_coefficient(m, re),
                Err(SphereDragError::UnsupportedDomain)
            );
        }
        for (m, re) in [(0.3, 300.0), (2.0, 300.0), (1.0, 10_000.0)] {
            assert!(continuum_drag_coefficient(m, re).is_ok());
        }
    }
}
