//! Finite-wing loads with an analytic attached-flow primitive or explicit polar.
//!
//! `FiniteWing` uses lifting-line slope a = a0 / (1 + a0 / (pi e AR)) and
//! CDi = CL² / (pi e AR), without stall or compressibility continuation.
//! `WingSurface` optionally replaces that closure with caller-supplied TOTAL
//! CL/CD data and a quasi-steady control incidence shift. No wing interference.
//! Coefficients refer to planform area; angles are in radians.
//!
//! Configure two surfaces in a body's stable local frame:
//! ```
//! use kapitsa_core::{aerodynamics::{Aerodynamics, wing::{FiniteWing, WingSurface}}, transform::Transform};
//! use kapitsa_math::{Quat, Vec3};
//! let model = FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.2)?;
//! let mut wings = Vec::new();
//! for y in [-2.0, 2.0] {
//!     wings.push(WingSurface::new(model,
//!         Transform::new(Vec3::Y * y, Quat::IDENTITY)?, 1.0)?);
//! }
//! let aerodynamics = Aerodynamics::Wings(wings);
//! # let _ = aerodynamics;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use std::{f64::consts::PI, sync::Arc};

/// Caller-supplied TOTAL finite-wing CL/CD on a rectangular Mach/AoA grid.
/// Values are Mach-major (`mach_index * angles.len() + angle_index`).
/// CD already includes induced and profile drag; these are not 2D section data.
/// Bilinear interpolation is supported only inside the closed supplied domain.
/// No stall or transonic behavior is inferred beyond the supplied data.
#[derive(Clone, Debug, PartialEq)]
pub struct WingPolar {
    mach: Box<[f64]>,
    angles: Box<[f64]>,
    coefficients: Box<[WingCoefficients]>,
}

impl WingPolar {
    pub fn new(
        mach: Vec<f64>,
        angles: Vec<f64>,
        coefficients: Vec<WingCoefficients>,
    ) -> Result<Self, WingError> {
        let valid_axis = |axis: &[f64]| {
            axis.len() >= 2
                && axis.iter().all(|v| v.is_finite())
                && axis.windows(2).all(|pair| pair[0] < pair[1])
        };
        if !valid_axis(&mach)
            || mach[0] < 0.0
            || !valid_axis(&angles)
            || mach.len().checked_mul(angles.len()) != Some(coefficients.len())
            || coefficients
                .iter()
                .any(|c| !c.lift.is_finite() || !c.drag.is_finite() || c.drag < 0.0)
        {
            return Err(WingError::InvalidParameters);
        }
        Ok(Self {
            mach: mach.into_boxed_slice(),
            angles: angles.into_boxed_slice(),
            coefficients: coefficients.into_boxed_slice(),
        })
    }

    pub fn coefficients(&self, angle: f64, mach: f64) -> Result<WingCoefficients, WingError> {
        if !angle.is_finite() || !mach.is_finite() || mach < 0.0 {
            return Err(WingError::InvalidFlow);
        }
        fn interval(axis: &[f64], value: f64) -> Result<(usize, f64), WingError> {
            if value < axis[0] || value > axis[axis.len() - 1] {
                return Err(WingError::UnsupportedFlow);
            }
            let index = axis
                .partition_point(|knot| *knot <= value)
                .saturating_sub(1)
                .min(axis.len() - 2);
            let lo = axis[index];
            let hi = axis[index + 1];
            // Halve only when subtraction overflows, preserving tiny intervals.
            let span = hi - lo;
            let t = if span.is_finite() {
                (value - lo) / span
            } else {
                (value * 0.5 - lo * 0.5) / (hi * 0.5 - lo * 0.5)
            };
            Ok((index, t))
        }
        let (m, mt) = interval(&self.mach, mach)?;
        let (a, at) = interval(&self.angles, angle)?;
        let mix = |x: WingCoefficients, y: WingCoefficients, t: f64| WingCoefficients {
            lift: x.lift * (1.0 - t) + y.lift * t,
            drag: x.drag * (1.0 - t) + y.drag * t,
        };
        let n = self.angles.len();
        let result = mix(
            mix(
                self.coefficients[m * n + a],
                self.coefficients[m * n + a + 1],
                at,
            ),
            mix(
                self.coefficients[(m + 1) * n + a],
                self.coefficients[(m + 1) * n + a + 1],
                at,
            ),
            mt,
        );
        if !result.lift.is_finite() || !result.drag.is_finite() {
            return Err(WingError::ArithmeticOverflow);
        }
        Ok(result)
    }
}

/// Quasi-steady incidence shift, NOT a universal hinge, flap or slat model.
/// All angles are radians. Effective AoA = physical AoA + effectiveness *
/// deflection; positive effectiveness and deflection increase effective AoA.
/// Force directions still follow the physical flow. No actuator dynamics or Cm.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WingControl {
    deflection: f64,
    max_deflection: f64,
    effectiveness: f64,
}

impl WingControl {
    pub fn new(
        deflection: f64,
        max_deflection: f64,
        effectiveness: f64,
    ) -> Result<Self, WingError> {
        if !deflection.is_finite()
            || !max_deflection.is_finite()
            || max_deflection < 0.0
            || max_deflection >= PI / 2.0
            || deflection.abs() > max_deflection
            || !effectiveness.is_finite()
            || !(deflection * effectiveness).is_finite()
        {
            return Err(WingError::InvalidParameters);
        }
        Ok(Self {
            deflection,
            max_deflection,
            effectiveness,
        })
    }
}

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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteWing {
    aspect_ratio: f64,
    efficiency: f64,
    profile_drag: f64,
    zero_lift_angle: f64,
    max_attached_angle: f64,
    compressible_max_mach: Option<f64>,
    drag_rise: Option<DragRise>,
}

/// Explicit parameters for an empirical wing compressibility drag increment.
#[derive(Clone, Copy, Debug, PartialEq)]
struct DragRise {
    critical_mach: f64,
    scale: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WingCoefficients {
    pub lift: f64,
    pub drag: f64,
}

/// One explicitly configured wing. Transform maps wing coordinates to the
/// stable body-local frame; its origin is the force application point.
#[derive(Clone, Debug, PartialEq)]
pub struct WingSurface {
    model: FiniteWing,
    polar: Option<Arc<WingPolar>>,
    control: Option<WingControl>,
    local_transform: crate::transform::Transform,
    planform_area: f64,
}

impl WingSurface {
    pub fn local_transform(&self) -> crate::transform::Transform {
        self.local_transform
    }

    /// Replaces the analytic coefficient source; clones share immutable data.
    pub fn with_polar(mut self, polar: Arc<WingPolar>) -> Self {
        self.polar = Some(polar);
        self
    }

    pub fn with_control(mut self, control: WingControl) -> Self {
        self.control = Some(control);
        self
    }

    /// Evaluate at physical AoA, with an optional configured incidence shift.
    pub fn coefficients(&self, angle: f64, mach: f64) -> Result<WingCoefficients, WingError> {
        let effective = angle + self.control.map_or(0.0, |c| c.deflection * c.effectiveness);
        if !angle.is_finite() {
            return Err(WingError::InvalidFlow);
        }
        if !effective.is_finite() {
            return Err(WingError::ArithmeticOverflow);
        }
        match &self.polar {
            Some(polar) => polar.coefficients(effective, mach),
            None => self.model.coefficients(effective, mach),
        }
    }

    pub fn new(
        model: FiniteWing,
        local_transform: crate::transform::Transform,
        planform_area: f64,
    ) -> Result<Self, WingError> {
        if !planform_area.is_finite() || planform_area < 0.0 {
            return Err(WingError::InvalidParameters);
        }
        Ok(Self {
            model,
            polar: None,
            control: None,
            local_transform,
            planform_area,
        })
    }

    /// Returns world force and COM moment using atmosphere supplied at the wing.
    /// No intrinsic airfoil pitching moment or mesh load exclusion is implied.
    pub fn evaluate(
        &self,
        flow: super::pressure::PressureFlow,
    ) -> Result<super::pressure::PressureWrench, WingError> {
        if !flow.center_of_mass.is_finite()
            || !flow.linear_velocity.is_finite()
            || !flow.angular_velocity.is_finite()
            || !flow.sound_speed.is_finite()
            || flow.sound_speed <= 0.0
        {
            return Err(WingError::InvalidFlow);
        }
        if flow.atmosphere.density() == 0.0 || self.planform_area == 0.0 {
            return Ok(super::pressure::PressureWrench {
                force: kapitsa_math::Vec3::ZERO,
                moment: kapitsa_math::Vec3::ZERO,
            });
        }
        let overflow = WingError::ArithmeticOverflow;
        let arm = flow.transform.transform_vector(
            self.local_transform
                .translation()
                .checked_sub(flow.center_of_mass)
                .ok_or(overflow)?,
        );
        let velocity = flow
            .angular_velocity
            .checked_cross(arm)
            .and_then(|v| flow.linear_velocity.checked_add(v))
            .and_then(|v| v.checked_sub(flow.atmosphere.wind_velocity()))
            .ok_or(overflow)?;
        let body_velocity = flow
            .transform
            .rotation()
            .conjugate()
            .rotate_vector(velocity)
            .ok_or(overflow)?;
        let wing_velocity = self
            .local_transform
            .rotation()
            .conjugate()
            .rotate_vector(body_velocity)
            .ok_or(overflow)?;
        let local_force = planar_force(
            wing_velocity,
            flow.atmosphere.density(),
            flow.sound_speed,
            self.planform_area,
            |alpha, mach| self.coefficients(alpha, mach),
        )?;
        let force = flow
            .transform
            .transform_vector(self.local_transform.transform_vector(local_force));
        let moment = arm.checked_cross(force).ok_or(overflow)?;
        if !force.is_finite() {
            return Err(overflow);
        }
        Ok(super::pressure::PressureWrench { force, moment })
    }
}

impl FiniteWing {
    /// Evaluate a planar wing in its local orthonormal frame: +X points
    /// toward the leading edge, +Y along span, +Z toward positive lift.
    /// Spanwise flow is outside this unswept reference model's domain.
    /// The returned force is local; application-point moments are caller-owned.
    pub fn force(
        self,
        relative_velocity: kapitsa_math::Vec3,
        density: f64,
        sound_speed: f64,
        planform_area: f64,
    ) -> Result<kapitsa_math::Vec3, WingError> {
        planar_force(
            relative_velocity,
            density,
            sound_speed,
            planform_area,
            |alpha, mach| self.coefficients(alpha, mach),
        )
    }

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
            compressible_max_mach: None,
            drag_rise: None,
        })
    }

    /// Enables a subsonic Prandtl-Glauert lifting-line approximation.
    ///
    /// `max_mach` must be finite and between 0 and 1, exclusive. The caller
    /// must select a limit below the critical Mach for this wing and flow;
    /// this approximation does not predict shock formation, wave drag or stall.
    /// The default model remains incompressible with a Mach limit of 0.3.
    pub fn with_subsonic_compressibility(mut self, max_mach: f64) -> Result<Self, WingError> {
        if !max_mach.is_finite() || max_mach <= 0.0 || max_mach >= 1.0 {
            return Err(WingError::InvalidParameters);
        }
        self.compressible_max_mach = Some(max_mach);
        Ok(self)
    }

    /// Adds the empirical NDARC compressibility drag increment.
    ///
    /// `critical_mach` is dimensionless, finite, and in (0, 1); `scale` is a
    /// finite nonnegative drag coefficient, referenced to the wing planform.
    /// Both must be selected for the wing and lift conditions by the caller.
    /// This option does not infer critical Mach from shape or lift. The
    /// increment is zero below Mach/critical Mach = 0.7 and is supported up to
    /// 1.1; outside the upper limit evaluation fails. The wing's configured
    /// Mach and attached-flow limits still apply independently.
    /// See NASA TP-2015-218751, Appendix 3, section 8-8.
    pub fn with_drag_rise(mut self, critical_mach: f64, scale: f64) -> Result<Self, WingError> {
        if !critical_mach.is_finite()
            || critical_mach <= 0.0
            || critical_mach >= 1.0
            || !scale.is_finite()
            || scale < 0.0
        {
            return Err(WingError::InvalidParameters);
        }
        self.drag_rise = (scale != 0.0).then_some(DragRise {
            critical_mach,
            scale,
        });
        Ok(self)
    }

    /// The default incompressible model supports Mach <= 0.3. An optional
    /// subsonic compressibility approximation uses its caller-specified limit.
    /// Both require attached flow; there is no implicit stall clamp.
    pub fn coefficients(
        self,
        angle_of_attack: f64,
        mach: f64,
    ) -> Result<WingCoefficients, WingError> {
        if !angle_of_attack.is_finite() || !mach.is_finite() || mach < 0.0 {
            return Err(WingError::InvalidFlow);
        }
        let alpha = angle_of_attack - self.zero_lift_angle;
        if mach > self.compressible_max_mach.unwrap_or(0.3) || alpha.abs() > self.max_attached_angle
        {
            return Err(WingError::UnsupportedFlow);
        }
        let effective_aspect = self.efficiency * self.aspect_ratio;
        let beta = if self.compressible_max_mach.is_some() {
            (1.0 - mach * mach).sqrt()
        } else {
            1.0
        };
        // Algebraically equivalent form without overflowing pi*e*AR, or
        // underflowing the slope by first dividing 2*pi by a tiny denominator.
        let slope = if effective_aspect >= 2.0 {
            2.0 * PI / (beta + 2.0 / effective_aspect)
        } else {
            PI * effective_aspect / (1.0 + beta * effective_aspect / 2.0)
        };
        let lift = slope * alpha;
        let drag_rise = if let Some(model) = self.drag_rise {
            let ratio = mach / model.critical_mach;
            if ratio > 1.1 {
                return Err(WingError::UnsupportedFlow);
            }
            let x = (ratio - 0.7).max(0.0);
            let x2 = x * x;
            let x4 = x2 * x2;
            model.scale * (6.5 * x2 + 28.0 * x4 + 2870.0 * x4 * x4)
        } else {
            0.0
        };
        let drag = self.profile_drag + (lift / effective_aspect) * (lift / PI) + drag_rise;
        if effective_aspect == 0.0 || !lift.is_finite() || !drag.is_finite() {
            return Err(WingError::ArithmeticOverflow);
        }
        Ok(WingCoefficients { lift, drag })
    }
}

/// Shared physical-flow domain and force projection, independent of polar source.
fn planar_force(
    relative_velocity: kapitsa_math::Vec3,
    density: f64,
    sound_speed: f64,
    planform_area: f64,
    coefficients: impl FnOnce(f64, f64) -> Result<WingCoefficients, WingError>,
) -> Result<kapitsa_math::Vec3, WingError> {
    use kapitsa_math::Vec3;
    if !relative_velocity.is_finite()
        || !density.is_finite()
        || density < 0.0
        || !sound_speed.is_finite()
        || sound_speed <= 0.0
        || !planform_area.is_finite()
        || planform_area < 0.0
    {
        return Err(WingError::InvalidFlow);
    }
    if density == 0.0 || planform_area == 0.0 || relative_velocity == Vec3::ZERO {
        return Ok(Vec3::ZERO);
    }
    // Permit rigid-frame roundoff, never physical sideslip or reversed flow.
    let planar_scale = relative_velocity.x.abs().max(relative_velocity.z.abs());
    if relative_velocity.y.abs() > 64.0 * f64::EPSILON * planar_scale || relative_velocity.x <= 0.0
    {
        return Err(WingError::UnsupportedFlow);
    }
    let velocity = Vec3::new(relative_velocity.x, 0.0, relative_velocity.z);
    let speed = velocity
        .checked_length()
        .ok_or(WingError::ArithmeticOverflow)?;
    let alpha = (-velocity.z).atan2(velocity.x);
    let c = coefficients(alpha, speed / sound_speed)?;
    let direction = velocity / speed;
    let lift_direction = Vec3::new(-direction.z, 0.0, direction.x);
    let scale = (0.5 * density * speed) * speed * planform_area;
    let force = (lift_direction * c.lift - direction * c.drag) * scale;
    if !scale.is_finite() || !force.is_finite() {
        return Err(WingError::ArithmeticOverflow);
    }
    Ok(force)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wing(ar: f64) -> FiniteWing {
        FiniteWing::new(ar, 0.8, 0.02, 0.0, 0.2).unwrap()
    }

    // Synthetic regression values, NOT measured data or a default aerodynamic model.
    fn polar() -> Arc<WingPolar> {
        Arc::new(
            WingPolar::new(
                vec![0.0, 0.8, 1.2],
                vec![-0.5, 0.0, 0.2, 0.5],
                [1.0, 1.5, 2.0]
                    .into_iter()
                    .flat_map(|factor| {
                        [-0.4, 0.0, 1.0, 0.4]
                            .into_iter()
                            .map(move |lift| WingCoefficients {
                                lift: lift * factor,
                                drag: 0.1 * factor,
                            })
                    })
                    .collect(),
            )
            .unwrap(),
        )
    }

    #[test]
    fn supplied_polar_interpolates_transonic_and_poststall_values_only() {
        let p = polar();
        let c = p.coefficients(0.1, 1.0).unwrap();
        assert!((c.lift - 0.875).abs() < 1e-14);
        assert!((c.drag - 0.175).abs() < 1e-14);
        assert_eq!(p.coefficients(0.5, 1.2).unwrap().lift, 0.8);
        assert!(p.coefficients(0.35, 1.0).unwrap().lift < p.coefficients(0.2, 1.0).unwrap().lift);
        assert!(p.coefficients(-0.25, 0.0).unwrap().lift < 0.0);
        for (angle, mach) in [(0.51, 1.0), (-0.51, 1.0), (0.0, 1.21)] {
            assert_eq!(p.coefficients(angle, mach), Err(WingError::UnsupportedFlow));
        }
        assert_eq!(p.coefficients(f64::NAN, 0.0), Err(WingError::InvalidFlow));
    }

    #[test]
    fn polar_rejects_invalid_rectangles_and_coefficients() {
        let c = WingCoefficients {
            lift: -1.0,
            drag: 0.0,
        };
        for (mach, angles, values) in [
            (vec![0.0], vec![0.0, 0.1], vec![c; 2]),
            (vec![0.0, 0.0], vec![0.0, 0.1], vec![c; 4]),
            (vec![-0.1, 1.0], vec![0.0, 0.1], vec![c; 4]),
            (vec![0.0, 1.0], vec![0.1, 0.0], vec![c; 4]),
            (vec![0.0, f64::NAN], vec![0.0, 0.1], vec![c; 4]),
            (vec![0.0, 1.0], vec![0.0, f64::INFINITY], vec![c; 4]),
            (vec![0.0, 1.0], vec![0.0, 0.1], vec![c; 3]),
            (
                vec![0.0, 1.0],
                vec![0.0, 0.1],
                vec![
                    WingCoefficients {
                        lift: 0.0,
                        drag: -0.1
                    };
                    4
                ],
            ),
            (
                vec![0.0, 1.0],
                vec![0.0, 0.1],
                vec![
                    WingCoefficients {
                        lift: f64::INFINITY,
                        drag: 0.0
                    };
                    4
                ],
            ),
        ] {
            assert_eq!(
                WingPolar::new(mach, angles, values),
                Err(WingError::InvalidParameters)
            );
        }
        assert!(WingControl::new(0.21, 0.2, 1.0).is_err());
        assert!(WingControl::new(0.0, 0.2, f64::NAN).is_err());
    }

    #[test]
    fn control_shift_preserves_physical_force_axes_total_drag_and_com_arm() {
        use crate::{
            aerodynamics::{AtmosphericState, pressure::PressureFlow},
            transform::Transform,
        };
        use kapitsa_math::{Quat, Vec3};
        let p = polar();
        let base = WingSurface::new(
            // Deliberately huge analytic profile drag: must not enter table forces.
            FiniteWing::new(1.0, 0.1, 100.0, 0.0, 0.1).unwrap(),
            Transform::new(Vec3::X * 2.0, Quat::IDENTITY).unwrap(),
            2.0,
        )
        .unwrap()
        .with_polar(Arc::clone(&p));
        let positive = base
            .clone()
            .with_control(WingControl::new(0.1, 0.2, 1.0).unwrap());
        let negative = base
            .clone()
            .with_control(WingControl::new(-0.1, 0.2, 1.0).unwrap());
        assert!(Arc::ptr_eq(
            base.polar.as_ref().unwrap(),
            positive.polar.as_ref().unwrap()
        ));
        let flow = PressureFlow {
            atmosphere: AtmosphericState::new(1.0, Vec3::ZERO).unwrap(),
            sound_speed: 100.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::X,
            linear_velocity: Vec3::X * 100.0,
            angular_velocity: Vec3::ZERO,
        };
        let load = positive.evaluate(flow).unwrap();
        assert!(load.force.z > 0.0);
        assert!(negative.evaluate(flow).unwrap().force.z < 0.0);
        let c = p.coefficients(0.1, 1.0).unwrap();
        assert!((load.force.x + 10000.0 * c.drag).abs() < 1e-9);
        assert!((load.force.z - 10000.0 * c.lift).abs() < 1e-9);
        assert_eq!(load.moment, Vec3::X.cross(load.force));
        assert!(load.moment.y < 0.0);
        for velocity in [Vec3::new(100.0, 1.0, 0.0), -Vec3::X * 100.0] {
            assert_eq!(
                positive.evaluate(PressureFlow {
                    linear_velocity: velocity,
                    ..flow
                }),
                Err(WingError::UnsupportedFlow)
            );
        }
        let beyond = base.with_control(WingControl::new(0.2, 0.2, 3.0).unwrap());
        assert_eq!(beyond.evaluate(flow), Err(WingError::UnsupportedFlow));
    }

    #[test]
    fn empirical_drag_rise_uses_configured_wing_data() -> Result<(), WingError> {
        let base = FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.4)?.with_subsonic_compressibility(0.9)?;
        let model = base.with_drag_rise(0.8, 0.0011)?;
        let below = model.coefficients(0.0, 0.5)?;
        assert_eq!(below, base.coefficients(0.0, 0.5)?);
        let at_critical = model.coefficients(0.0, 0.8)?;
        let x = 0.3_f64;
        let expected = 0.0011 * (6.5 * x.powi(2) + 28.0 * x.powi(4) + 2870.0 * x.powi(8));
        assert!((at_critical.drag - 0.02 - expected).abs() < 1e-12);
        assert_eq!(
            model.coefficients(0.0, 0.89),
            Err(WingError::UnsupportedFlow)
        );
        assert!(base.with_drag_rise(0.0, 0.001).is_err());
        assert!(base.with_drag_rise(0.8, f64::NAN).is_err());
        assert_eq!(
            base.with_drag_rise(0.8, 0.0)?.coefficients(0.0, 0.89)?,
            base.coefficients(0.0, 0.89)?
        );
        Ok(())
    }

    #[test]
    fn compressible_lifting_line_has_explicit_domain() -> Result<(), WingError> {
        let base = FiniteWing::new(8.0, 0.8, 0.02, 0.0, 0.4)?;
        let corrected = base.with_subsonic_compressibility(0.7)?;
        let alpha = 0.1;
        let low = base.coefficients(alpha, 0.0)?;
        let corrected_low = corrected.coefficients(alpha, 0.0)?;
        assert_eq!(low, corrected_low);
        let coefficients = corrected.coefficients(alpha, 0.6)?;
        let beta = (1.0_f64 - 0.6_f64.powi(2)).sqrt();
        let expected_lift = 2.0 * PI * alpha / (beta + 2.0 / (0.8 * 8.0));
        assert!((coefficients.lift - expected_lift).abs() < 1e-12);
        assert!(coefficients.lift > low.lift);
        assert!(corrected.coefficients(alpha, 0.7).is_ok());
        assert_eq!(
            corrected.coefficients(alpha, 0.7001),
            Err(WingError::UnsupportedFlow)
        );
        assert_eq!(
            base.coefficients(alpha, 0.6),
            Err(WingError::UnsupportedFlow)
        );
        assert_eq!(
            base.with_subsonic_compressibility(1.0),
            Err(WingError::InvalidParameters)
        );
        assert_eq!(
            base.with_subsonic_compressibility(f64::NAN),
            Err(WingError::InvalidParameters)
        );
        Ok(())
    }

    #[test]
    fn analytic_slope_and_induced_drag() {
        let c = wing(8.0).coefficients(0.1, 0.2).unwrap();
        let expected = (2.0 * PI / (1.0 + 2.0 / (0.8 * 8.0))) * 0.1;
        assert!((c.lift - expected).abs() < 1e-14);
        assert!((c.drag - 0.02 - expected * expected / (PI * 0.8 * 8.0)).abs() < 1e-14);
    }

    #[test]
    fn representable_extreme_aspect_ratios_do_not_overflow_intermediates() {
        for ar in [1e-300, 1e300, f64::MAX] {
            let coefficients = wing(ar).coefficients(0.1, 0.0).unwrap();
            assert!(coefficients.lift.is_finite() && coefficients.lift > 0.0);
            assert!(coefficients.drag.is_finite() && coefficients.drag >= 0.02);
        }
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

    #[test]
    fn force_has_dissipative_drag_and_perpendicular_lift() {
        use kapitsa_math::Vec3;
        let velocity = Vec3::new(50.0, 0.0, 5.0);
        let speed = velocity.length();
        let c = wing(8.0)
            .coefficients((-5.0_f64).atan2(50.0), speed / 340.0)
            .unwrap();
        let force = wing(8.0).force(velocity, 1.2, 340.0, 2.0).unwrap();
        let q_area = 0.5 * 1.2 * speed * speed * 2.0;
        assert!((force.dot(velocity / speed) + q_area * c.drag).abs() < 1e-10);
        assert!(force.z < 0.0);
        let reverse = wing(8.0)
            .force(Vec3::new(50.0, 0.0, -5.0), 1.2, 340.0, 2.0)
            .unwrap();
        assert!((force.x - reverse.x).abs() < 1e-10);
        assert!((force.z + reverse.z).abs() < 1e-10);
        assert_eq!(wing(8.0).force(Vec3::X, 0.0, 340.0, 2.0), Ok(Vec3::ZERO));
        assert_eq!(
            wing(8.0).force(Vec3::Y, 1.2, 340.0, 2.0),
            Err(WingError::UnsupportedFlow)
        );
    }

    #[test]
    fn nose_up_attitude_generates_positive_world_lift_and_com_moment() {
        use crate::transform::Transform;
        use kapitsa_math::{Quat, Vec3};
        let attitude =
            Transform::new(Vec3::ZERO, Quat::from_axis_angle(Vec3::Y, -0.1).unwrap()).unwrap();
        let surface = WingSurface::new(
            wing(8.0),
            Transform::new(Vec3::Y * 2.0, Quat::IDENTITY).unwrap(),
            2.0,
        )
        .unwrap();
        let flow = super::super::pressure::PressureFlow {
            atmosphere: super::super::AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
            sound_speed: 340.0,
            transform: attitude,
            center_of_mass: Vec3::ZERO,
            linear_velocity: Vec3::X * 50.0,
            angular_velocity: Vec3::ZERO,
        };
        let load = surface.evaluate(flow).unwrap();
        assert!(load.force.z > 0.0);
        assert!(load.force.x < 0.0);
        assert!((load.moment - (Vec3::Y * 2.0).cross(load.force)).length() < 1e-10);
        let centered = surface
            .evaluate(super::super::pressure::PressureFlow {
                center_of_mass: Vec3::Y * 2.0,
                ..flow
            })
            .unwrap();
        assert_eq!(centered.force, load.force);
        assert_eq!(centered.moment, Vec3::ZERO);
    }

    #[test]
    fn surface_uses_rotational_velocity_and_is_world_frame_covariant() {
        use crate::transform::Transform;
        use kapitsa_math::{Quat, Vec3};
        let surface = WingSurface::new(
            wing(8.0),
            Transform::new(Vec3::X * 2.0, Quat::IDENTITY).unwrap(),
            2.0,
        )
        .unwrap();
        let flow = super::super::pressure::PressureFlow {
            atmosphere: super::super::AtmosphericState::new(1.2, Vec3::X * 3.0).unwrap(),
            sound_speed: 340.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::ZERO,
            linear_velocity: Vec3::X * 53.0,
            angular_velocity: Vec3::Y,
        };
        let load = surface.evaluate(flow).unwrap();
        assert!(load.force.z > 0.0);
        assert!(load.moment.dot(flow.angular_velocity) < 0.0);
        let rotation = Transform::new(
            Vec3::splat(1e10),
            Quat::from_axis_angle(Vec3::Z, 0.7).unwrap(),
        )
        .unwrap();
        let rotated = surface
            .evaluate(super::super::pressure::PressureFlow {
                transform: rotation,
                atmosphere: super::super::AtmosphericState::new(
                    1.2,
                    rotation.transform_vector(flow.atmosphere.wind_velocity()),
                )
                .unwrap(),
                linear_velocity: rotation.transform_vector(flow.linear_velocity),
                angular_velocity: rotation.transform_vector(flow.angular_velocity),
                ..flow
            })
            .unwrap();
        assert!((rotated.force - rotation.transform_vector(load.force)).length() < 1e-9);
        assert!((rotated.moment - rotation.transform_vector(load.moment)).length() < 1e-9);
    }

    #[test]
    fn vacuum_skips_overflowing_motion_but_validates_inputs() {
        use crate::transform::Transform;
        use kapitsa_math::Vec3;
        let surface = WingSurface::new(wing(8.0), Transform::identity(), 2.0).unwrap();
        let flow = super::super::pressure::PressureFlow {
            atmosphere: super::super::AtmosphericState::new(0.0, Vec3::X * f64::MAX).unwrap(),
            sound_speed: 340.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::Y,
            linear_velocity: -Vec3::X * f64::MAX,
            angular_velocity: Vec3::Z * f64::MAX,
        };
        let result = surface.evaluate(flow).unwrap();
        assert_eq!(result.force, Vec3::ZERO);
        assert_eq!(result.moment, Vec3::ZERO);
        assert_eq!(
            surface.evaluate(super::super::pressure::PressureFlow {
                sound_speed: 0.0,
                ..flow
            }),
            Err(WingError::InvalidFlow)
        );
    }

    #[test]
    fn world_translation_is_not_inverted_for_vector_only_evaluation() {
        use crate::transform::Transform;
        use kapitsa_math::{Quat, Vec3};
        let surface = WingSurface::new(wing(8.0), Transform::identity(), 2.0).unwrap();
        let rotation = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_4).unwrap();
        let transform = Transform::new(Vec3::new(f64::MAX, f64::MAX, 0.0), rotation).unwrap();
        let flow = super::super::pressure::PressureFlow {
            atmosphere: super::super::AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
            sound_speed: 340.0,
            transform,
            center_of_mass: Vec3::ZERO,
            linear_velocity: transform.transform_vector(Vec3::new(50.0, 0.0, -5.0)),
            angular_velocity: Vec3::ZERO,
        };
        assert!(transform.inverse().is_err());
        let translated = surface.evaluate(flow).unwrap();
        let origin = surface
            .evaluate(super::super::pressure::PressureFlow {
                transform: Transform::new(Vec3::ZERO, rotation).unwrap(),
                ..flow
            })
            .unwrap();
        assert_eq!(translated, origin);
    }
}
