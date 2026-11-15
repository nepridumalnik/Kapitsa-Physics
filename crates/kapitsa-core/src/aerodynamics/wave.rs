//! Wave drag of a slender axisymmetric body in linearized supersonic flow.
//!
//! This is an independent numerical primitive, not a transonic closure or a
//! complete aircraft model. The inputs are samples of dA/dx, in metres, at
//! equally spaced positions. The endpoints must both be zero. Between samples
//! dA/dx is linear, so the second derivative is piecewise constant. The
//! logarithmic singularity is integrated analytically for each interval pair.
//! See NACA TN 3478, equations (1) and (2):
//! <https://ntrs.nasa.gov/api/citations/19930084271/downloads/19930084271.pdf>.

use std::f64::consts::PI;

/// Errors in the input or numerical evaluation of wave drag.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaveDragError {
    InvalidInput,
    UnsupportedMach,
    ArithmeticOverflow,
    PrecisionLoss,
    WorkLimitExceeded,
}

impl std::fmt::Display for WaveDragError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "wave drag evaluation failed: {self:?}")
    }
}
impl std::error::Error for WaveDragError {}

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
    if speed / sound_speed <= 1.0 {
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
    let mut integral = 0.0;
    for (i, pair_i) in area_slopes.windows(2).enumerate() {
        let di = pair_i[1] - pair_i[0];
        for (j, pair_j) in area_slopes.windows(2).enumerate() {
            let dj = pair_j[1] - pair_j[0];
            let k = i.abs_diff(j) as f64;
            let kernel = if k == 0.0 {
                -1.5
            } else {
                g(k + 1.0) + g(k - 1.0) - 2.0 * g(k)
            };
            integral += di * dj * kernel;
        }
    }
    let drag = -density * speed * speed * integral / (4.0 * PI);
    if !drag.is_finite() || !integral.is_finite() {
        return Err(WaveDragError::ArithmeticOverflow);
    }
    if drag < 0.0 {
        return Err(WaveDragError::PrecisionLoss);
    }
    Ok(drag)
}

#[cfg(test)]
mod tests {
    use super::{WaveDragError, wave_drag};
    use std::f64::consts::PI;

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
