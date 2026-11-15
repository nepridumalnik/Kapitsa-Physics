//! Wave drag of a slender axisymmetric body in linearized supersonic flow.
//!
//! This is an independent numerical primitive, not a transonic closure or a
//! complete aircraft model. The inputs are samples of dA/dx, in metres, at
//! equally spaced positions. The endpoints must both be zero. Between samples
//! dA/dx is linear, so the second derivative is piecewise constant. The
//! logarithmic singularity is integrated analytically for each interval pair.
//! See NACA TN 3478, equations (1) and (2):
//! <https://ntrs.nasa.gov/api/citations/19930084271/downloads/19930084271.pdf>.

use std::{f64::consts::PI, sync::Arc};

/// Errors in the input or numerical evaluation of wave drag.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaveDragError {
    InvalidInput,
    UnsupportedMach,
    ArithmeticOverflow,
    PrecisionLoss,
    WorkLimitExceeded,
    AllocationFailed,
}

impl std::fmt::Display for WaveDragError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "wave drag evaluation failed: {self:?}")
    }
}
impl std::error::Error for WaveDragError {}

/// A caller-supplied multiplier for the slender-body wave-drag factor.
///
/// Values are dimensionless and must come from data or a separately validated
/// model. They do not make the slender-body approximation valid for other shapes
/// or nonzero incidence. The domain is inclusive and never extrapolated.
#[derive(Clone, Debug, PartialEq)]
pub struct WaveMachCurve {
    knots: Arc<[(f64, f64)]>,
}

impl WaveMachCurve {
    /// Creates a curve from at least two strictly increasing `(Mach, multiplier)`
    /// pairs. Both values must be finite and nonnegative.
    pub fn new(knots: Vec<(f64, f64)>) -> Result<Self, WaveDragError> {
        if knots.len() < 2
            || knots.iter().any(|(mach, factor)| {
                !mach.is_finite() || *mach < 0.0 || !factor.is_finite() || *factor < 0.0
            })
            || knots.windows(2).any(|pair| pair[0].0 >= pair[1].0)
        {
            return Err(WaveDragError::InvalidInput);
        }
        Ok(Self {
            knots: knots.into(),
        })
    }

    /// Returns the linearly interpolated multiplier, without extrapolation.
    pub fn multiplier(&self, mach: f64) -> Result<f64, WaveDragError> {
        if !mach.is_finite() || mach < 0.0 {
            return Err(WaveDragError::InvalidInput);
        }
        for pair in self.knots.windows(2) {
            let (x0, y0) = pair[0];
            let (x1, y1) = pair[1];
            if mach == x0 {
                return Ok(y0);
            }
            if mach == x1 {
                return Ok(y1);
            }
            if mach > x0 && mach < x1 {
                let t = (mach - x0) / (x1 - x0);
                return Ok(if y0 <= y1 {
                    y0 + t * (y1 - y0)
                } else {
                    y1 + (1.0 - t) * (y0 - y1)
                });
            }
        }
        Err(WaveDragError::UnsupportedMach)
    }
}

/// Computes uniformly spaced `dA/dx` samples from voxel cross sections.
///
/// `cell_size` is the spacing in metres and `smoothing_radius` is the radius
/// of an explicitly chosen centered box filter in cells. Zeros are padded
/// outside the input, and the returned array includes that padding; its first
/// and last derivatives are zero. The moving average is applied to areas
/// before centered differencing. Work and output are bounded by 2048 intervals.
///
/// Voxel sections overestimate area and differentiating them amplifies noise.
/// The caller must choose and validate the smoothing scale and grid resolution
/// for the shape; this routine does not establish axisymmetry, slenderness,
/// convergence, or the validity of a wave-drag calculation.
pub fn area_slopes_from_sections(
    sections: &[super::geometry::GeometrySection],
    cell_size: f64,
    smoothing_radius: usize,
) -> Result<Vec<f64>, WaveDragError> {
    if sections.is_empty()
        || !cell_size.is_finite()
        || cell_size <= 0.0
        || sections
            .iter()
            .any(|section| !section.area.is_finite() || section.area < 0.0)
    {
        return Err(WaveDragError::InvalidInput);
    }
    let padding = smoothing_radius
        .checked_add(2)
        .ok_or(WaveDragError::WorkLimitExceeded)?;
    let length = padding
        .checked_mul(2)
        .and_then(|n| sections.len().checked_add(n))
        .ok_or(WaveDragError::WorkLimitExceeded)?;
    if length > 2049 {
        return Err(WaveDragError::WorkLimitExceeded);
    }
    let width = smoothing_radius
        .checked_mul(2)
        .and_then(|n| n.checked_add(1))
        .ok_or(WaveDragError::WorkLimitExceeded)?;
    let mut smoothed = Vec::new();
    smoothed
        .try_reserve_exact(length)
        .map_err(|_| WaveDragError::AllocationFailed)?;
    for index in 0..length {
        let start = index
            .saturating_sub(padding + smoothing_radius)
            .min(sections.len());
        let end = (index + smoothing_radius + 1)
            .saturating_sub(padding)
            .min(sections.len());
        let window = sections
            .get(start..end)
            .ok_or(WaveDragError::PrecisionLoss)?;
        let max_area = window
            .iter()
            .fold(0.0_f64, |max, section| max.max(section.area));
        let area = if max_area == 0.0 {
            0.0
        } else {
            let mut sum = 0.0;
            let mut correction = 0.0;
            for section in window {
                let value = section.area / max_area - correction;
                let next = sum + value;
                correction = (next - sum) - value;
                sum = next;
            }
            super::hull::scaled_product(&[max_area, sum], width as f64)
                .map_err(|_| WaveDragError::ArithmeticOverflow)?
        };
        smoothed.push(area);
    }
    let mut slopes = Vec::new();
    slopes
        .try_reserve_exact(length)
        .map_err(|_| WaveDragError::AllocationFailed)?;
    slopes.push(0.0);
    for pair in smoothed.windows(3) {
        let delta = pair[2] - pair[0];
        let slope = super::hull::scaled_product(&[delta.abs(), 0.5], cell_size)
            .map_err(|_| WaveDragError::ArithmeticOverflow)?;
        slopes.push(slope.copysign(delta));
    }
    slopes.push(0.0);
    Ok(slopes)
}

/// Calculates wave drag in newtons from equally spaced samples of `dA/dx`.
///
/// `density` is in kg/m³, `speed` in m/s, and `sound_speed` in m/s. The
/// distribution must have at least three finite samples with zero endpoints.
/// The body's length cancels when the source strength `dA/dx` is supplied.
/// This formula assumes an axisymmetric slender body aligned with the flow,
/// linearized supersonic conditions, and zero source strength at both ends.
/// It does not predict transonic drag rise, viscosity, separation or lift.
/// Work is quadratic and limited to 2048 intervals. The caller must ensure
/// the source describes a physically valid slender axisymmetric body.
pub fn wave_drag(
    area_slopes: &[f64],
    density: f64,
    speed: f64,
    sound_speed: f64,
) -> Result<f64, WaveDragError> {
    if area_slopes.len() < 3
        || !density.is_finite()
        || density < 0.0
        || !speed.is_finite()
        || speed < 0.0
        || !sound_speed.is_finite()
        || sound_speed <= 0.0
        || area_slopes.iter().any(|value| !value.is_finite())
        || area_slopes.first() != Some(&0.0)
        || area_slopes.last() != Some(&0.0)
    {
        return Err(WaveDragError::InvalidInput);
    }
    let intervals = area_slopes.len() - 1;
    if intervals > 2048 {
        return Err(WaveDragError::WorkLimitExceeded);
    }
    if density == 0.0 || speed == 0.0 {
        return Ok(0.0);
    }
    if speed <= sound_speed {
        return Err(WaveDragError::UnsupportedMach);
    }
    // g''(x) = ln|x|. Coordinates are in interval widths; the constant
    // ln(h) cancels because the source strength vanishes at both ends.
    fn g(x: f64) -> f64 {
        if x == 0.0 {
            0.0
        } else {
            x * x * (0.5 * x.abs().ln() - 0.75)
        }
    }
    let scale = area_slopes
        .iter()
        .fold(0.0_f64, |max, value| max.max(value.abs()));
    if scale == 0.0 {
        return Ok(0.0);
    }
    let mut integral = 0.0;
    let mut compensation = 0.0;
    for (i, pair_i) in area_slopes.windows(2).enumerate() {
        let di = pair_i[1] / scale - pair_i[0] / scale;
        for (j, pair_j) in area_slopes.windows(2).enumerate() {
            let dj = pair_j[1] / scale - pair_j[0] / scale;
            let k = i.abs_diff(j) as f64;
            let kernel = if k == 0.0 {
                -1.5
            } else if k >= 16.0 {
                // The direct second difference loses precision at large k.
                let t = 1.0 / (k * k);
                k.ln() - t * (1.0 / 12.0 + t * (1.0 / 60.0 + t / 168.0))
            } else {
                g(k + 1.0) + g(k - 1.0) - 2.0 * g(k)
            };
            let term = di * dj * kernel;
            let corrected = term - compensation;
            let next = integral + corrected;
            compensation = (next - integral) - corrected;
            integral = next;
        }
    }
    if !integral.is_finite() {
        return Err(WaveDragError::ArithmeticOverflow);
    }
    if integral > 0.0 {
        return Err(WaveDragError::PrecisionLoss);
    }
    super::hull::scaled_product(&[density, speed, speed, scale, scale, -integral], 4.0 * PI)
        .map_err(|_| WaveDragError::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use super::{WaveDragError, WaveMachCurve, area_slopes_from_sections, wave_drag};
    use crate::aerodynamics::geometry::GeometrySection;
    use std::f64::consts::PI;

    #[test]
    fn section_wave_drag_converges_for_smooth_slender_body() -> Result<(), WaveDragError> {
        let mut errors = Vec::new();
        for n in [128, 256, 512] {
            let sections: Vec<_> = (0..n)
                .map(|i| {
                    let x = (i as f64 + 0.5) / n as f64;
                    GeometrySection {
                        area: (128.0 / (3.0 * PI)) * (x * (1.0 - x)).powf(1.5),
                        centroid: None,
                    }
                })
                .collect();
            let slopes = area_slopes_from_sections(&sections, 1.0 / n as f64, 1)?;
            assert_eq!(slopes.first(), Some(&0.0));
            assert_eq!(slopes.last(), Some(&0.0));
            let drag = wave_drag(&slopes, 1.0, 2.0, 1.0)?;
            errors.push((drag - 256.0 / PI).abs());
        }
        assert!(errors[2] < errors[1] && errors[1] < errors[0], "{errors:?}");
        assert!(errors[2] < (256.0 / PI) * 0.02, "{errors:?}");
        Ok(())
    }

    #[test]
    fn section_smoothing_preserves_subnormal_area_when_average_is_representable()
    -> Result<(), WaveDragError> {
        let tiny = f64::from_bits(1);
        let sections = [GeometrySection {
            area: tiny,
            centroid: None,
        }; 3];
        let slopes = area_slopes_from_sections(&sections, tiny, 1)?;
        assert!(slopes.iter().any(|value| *value > 0.0));
        assert!(slopes.iter().any(|value| *value < 0.0));
        Ok(())
    }

    #[test]
    fn section_smoothing_validates_resources_and_numeric_input() {
        let invalid = [GeometrySection {
            area: -1.0,
            centroid: None,
        }];
        assert_eq!(
            area_slopes_from_sections(&invalid, 1.0, 0),
            Err(WaveDragError::InvalidInput)
        );
        let valid = [GeometrySection {
            area: 1.0,
            centroid: None,
        }];
        assert_eq!(
            area_slopes_from_sections(&valid, 0.0, 0),
            Err(WaveDragError::InvalidInput)
        );
        assert_eq!(
            area_slopes_from_sections(&valid, 1.0, usize::MAX),
            Err(WaveDragError::WorkLimitExceeded)
        );
    }

    #[test]
    fn mach_curve_validates_data_and_endpoints() -> Result<(), WaveDragError> {
        assert!(WaveMachCurve::new(vec![(0.0, 0.0)]).is_err());
        assert!(WaveMachCurve::new(vec![(0.0, 1.0), (0.0, 2.0)]).is_err());
        assert!(WaveMachCurve::new(vec![(0.0, 1.0), (1.0, f64::NAN)]).is_err());
        let curve = WaveMachCurve::new(vec![(0.0, f64::MAX), (1.0, 0.0)])?;
        assert_eq!(curve.multiplier(0.0)?, f64::MAX);
        assert_eq!(curve.multiplier(1.0)?, 0.0);
        assert_eq!(curve.multiplier(0.5)?, f64::MAX * 0.5);
        assert_eq!(curve.multiplier(2.0), Err(WaveDragError::UnsupportedMach));
        Ok(())
    }

    #[test]
    fn sears_haack_converges() -> Result<(), WaveDragError> {
        let expected = 64.0 / PI;
        let mut errors = Vec::new();
        for n in [256, 512, 1024] {
            let samples: Vec<f64> = (0..=n)
                .map(|i| {
                    let x = i as f64 / n as f64;
                    64.0 / PI * (x * (1.0 - x)).sqrt() * (1.0 - 2.0 * x)
                })
                .collect();
            errors.push((wave_drag(&samples, 1.0, 2.0, 1.0)? - expected * 4.0).abs());
        }
        assert!(errors[2] < errors[1] && errors[1] < errors[0], "{errors:?}");
        assert!(errors[2] < expected * 4.0 * 0.01, "{errors:?}");
        Ok(())
    }

    #[test]
    fn scaling_preserves_representable_extremes() -> Result<(), WaveDragError> {
        let reference = wave_drag(&[0.0, 1.0, -1.0, 0.0], 1.0, 2.0, 1.0)?;
        let extreme = wave_drag(&[0.0, 1e200, -1e200, 0.0], 1e-300, 2e-50, 1e-50)?;
        assert!((extreme / reference - 1.0).abs() < 1e-12);
        let reflected = wave_drag(&[0.0, -1.0, 1.0, 0.0], 1.0, 2.0, 1.0)?;
        assert_eq!(reference, reflected);
        Ok(())
    }

    #[test]
    fn rejects_invalid_and_subsonic_inputs() {
        assert_eq!(
            wave_drag(&[0.0, 1.0, 0.0], 1.0, 1.0, 1.0),
            Err(WaveDragError::UnsupportedMach)
        );
        assert_eq!(
            wave_drag(&[1.0, 1.0, 0.0], 1.0, 2.0, 1.0),
            Err(WaveDragError::InvalidInput)
        );
        assert_eq!(wave_drag(&[0.0, 1.0, 0.0], 0.0, 0.0, 1.0), Ok(0.0));
    }
}
