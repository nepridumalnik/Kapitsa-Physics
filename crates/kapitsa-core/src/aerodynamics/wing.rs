//! Independent finite-wing, attached-flow reference model.
//!
//! Uses lifting-line slope a = a0 / (1 + a0 / (pi e AR)) and
//! CDi = CL² / (pi e AR). No stall, compressibility or wing interference.
//! Coefficients refer to planform area; angles are in radians.

use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WingError {
    InvalidParameters,
    InvalidFlow,
    UnsupportedFlow,
    ArithmeticOverflow,
}

impl std::fmt::Display for WingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "finite-wing evaluation failed: {self:?}")
    }
}
impl std::error::Error for WingError {}

/// Explicit parameters, not inferred from voxel geometry.
#[derive(Clone, Copy, Debug)]
pub struct FiniteWing {
    aspect_ratio: f64,
    efficiency: f64,
    profile_drag: f64,
    zero_lift_angle: f64,
    max_attached_angle: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WingCoefficients {
    pub lift: f64,
    pub drag: f64,
}

impl FiniteWing {
    /// `efficiency` is in (0, 1]; `max_attached_angle` is in (0, pi/2).
    /// The caller selects an attached-flow validity limit from airfoil data.
    pub fn new(
        aspect_ratio: f64,
        efficiency: f64,
        profile_drag: f64,
        zero_lift_angle: f64,
        max_attached_angle: f64,
    ) -> Result<Self, WingError> {
        if !aspect_ratio.is_finite()
            || aspect_ratio <= 0.0
            || !efficiency.is_finite()
            || efficiency <= 0.0
            || efficiency > 1.0
            || !profile_drag.is_finite()
            || profile_drag < 0.0
            || !zero_lift_angle.is_finite()
            || zero_lift_angle.abs() >= PI / 2.0
            || !max_attached_angle.is_finite()
            || max_attached_angle <= 0.0
            || max_attached_angle >= PI / 2.0
        {
            return Err(WingError::InvalidParameters);
        }
        Ok(Self {
            aspect_ratio,
            efficiency,
            profile_drag,
            zero_lift_angle,
            max_attached_angle,
        })
    }

    /// Incompressible engineering policy: Mach <= 0.3, attached flow only.
    /// Rejection outside that domain is intentional; no implicit stall clamp.
    pub fn coefficients(
        self,
        angle_of_attack: f64,
        mach: f64,
    ) -> Result<WingCoefficients, WingError> {
        if !angle_of_attack.is_finite() || !mach.is_finite() || mach < 0.0 {
            return Err(WingError::InvalidFlow);
        }
        let alpha = angle_of_attack - self.zero_lift_angle;
        if mach > 0.3 || alpha.abs() > self.max_attached_angle {
            return Err(WingError::UnsupportedFlow);
        }
        let denominator = PI * self.efficiency * self.aspect_ratio;
        let slope = 2.0 * PI / (1.0 + 2.0 * PI / denominator);
        let lift = slope * alpha;
        let drag = self.profile_drag + lift * lift / denominator;
        if denominator == 0.0 || !denominator.is_finite() || !lift.is_finite() || !drag.is_finite()
        {
            return Err(WingError::ArithmeticOverflow);
        }
        Ok(WingCoefficients { lift, drag })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wing(ar: f64) -> FiniteWing {
        FiniteWing::new(ar, 0.8, 0.02, 0.0, 0.2).unwrap()
    }

    #[test]
    fn analytic_slope_and_induced_drag() {
        let c = wing(8.0).coefficients(0.1, 0.2).unwrap();
        let expected = (2.0 * PI / (1.0 + 2.0 / (0.8 * 8.0))) * 0.1;
        assert!((c.lift - expected).abs() < 1e-14);
        assert!((c.drag - 0.02 - expected * expected / (PI * 0.8 * 8.0)).abs() < 1e-14);
    }

    #[test]
    fn symmetry_zero_lift_and_infinite_span_limit() {
        let positive = wing(8.0).coefficients(0.1, 0.0).unwrap();
        let negative = wing(8.0).coefficients(-0.1, 0.0).unwrap();
        assert_eq!(positive.lift, -negative.lift);
        assert_eq!(positive.drag, negative.drag);
        assert_eq!(
            wing(8.0).coefficients(0.0, 0.0).unwrap(),
            WingCoefficients {
                lift: 0.0,
                drag: 0.02
            }
        );
        let broad = wing(1e10).coefficients(0.1, 0.0).unwrap();
        assert!((broad.lift - 2.0 * PI * 0.1).abs() < 1e-9);
    }

    #[test]
    fn validity_is_explicit() {
        assert!(wing(8.0).coefficients(0.2, 0.3).is_ok());
        assert_eq!(
            wing(8.0).coefficients(0.21, 0.0),
            Err(WingError::UnsupportedFlow)
        );
        assert_eq!(
            wing(8.0).coefficients(0.0, 0.31),
            Err(WingError::UnsupportedFlow)
        );
        assert_eq!(
            wing(8.0).coefficients(f64::NAN, 0.0),
            Err(WingError::InvalidFlow)
        );
        assert!(FiniteWing::new(0.0, 0.8, 0.02, 0.0, 0.2).is_err());
    }
}
