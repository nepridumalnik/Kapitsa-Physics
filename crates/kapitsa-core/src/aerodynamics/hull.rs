//! Caller-parameterized distributed subsonic hull drag (Mach <= 0.3).
//!
//! This is an empirical sectional resistance model, not potential-flow normal
//! force, lift, area-rule, interference, shielding, separation or FAR prediction.
//! Each section's transform locates its load point in the stable body frame;
//! its axis is expressed in section coordinates. Choose enough sections to
//! resolve angular motion and spatial atmospheric variation. Areas and Cd must
//! use the same convention: axial reference area for axial/form Cd, projected
//! crossflow area for crossflow Cd, and wetted area for skin friction.
//! See <https://en.wikipedia.org/wiki/Drag_equation> (F = q Cd A).
//!
//! Optional smooth flat-plate friction uses Re_L = rho |U_axial| L / mu.
//! Integrating local Cf_x = .664/sqrt(Re_x) or .0576/Re_x^.2 from 0 to L
//! gives mean Cf_L = 1.328/sqrt(Re_L) or .072/Re_L^.2 respectively:
//! integral_0^1 s^-p ds = 1/(1-p). See
//! <https://en.wikipedia.org/wiki/Skin_friction_drag>. The caller explicitly
//! chooses the boundary-layer regime and inclusive Reynolds validity interval;
//! no universal transition threshold, viscosity, or mixed-regime closure is
//! assumed. Turbulent means the whole plate, not transition partway along it.
//! Friction opposes axial flow only, never total perpendicular speed. Zero
//! axial speed skips friction and its Reynolds check. None is inviscid.
//! Finite kinematics and positive sound speed are validated even in vacuum or
//! with disabled areas; those cases then skip Mach and Reynolds domain checks.
//! Mach is based on the full local relative speed, including omega cross r.
//! Each component uses its own speed squared in q, not full-flow q. Length is
//! the caller's friction development length, not automatically accumulated across
//! sections. No end-cap loads or intrinsic pitching moments are inferred.
//!
//! `Aerodynamics::Hull` owns these sections for ordinary body world stepping;
//! each point samples its own atmosphere and uses the world's existing sound
//! speed setting. Cluster conversion transfers them to the identity root part;
//! part-owned sections use the same evaluator and remain additive to explicit
//! wings and body surfaces. There is no runtime setter for body-owned Hull.
//! Simple, Wings and hypersonic Newtonian Mesh retain their existing semantics.
//!
//! ```
//! use kapitsa_core::{aerodynamics::{Aerodynamics, hull::HullSection}, transform::Transform};
//! use kapitsa_math::{Quat, Vec3};
//! let mut sections = Vec::new();
//! for x in [-2.0, 2.0] {
//!     sections.push(HullSection::new(
//!         Transform::new(Vec3::X * x, Quat::IDENTITY)?, Vec3::X,
//!         2.0, 0.5, 2.0, 4.0, 0.1, 0.8, None,
//!     )?);
//! }
//! let model = Aerodynamics::Hull(sections);
//! # let _ = model;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use super::pressure::{PressureFlow, PressureWrench};
use crate::transform::Transform;
use kapitsa_math::Vec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HullError {
    InvalidParameters,
    InvalidFlow,
    UnsupportedMach,
    UnsupportedReynolds,
    ArithmeticOverflow,
}
impl std::fmt::Display for HullError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "subsonic hull evaluation failed: {self:?}")
    }
}
impl std::error::Error for HullError {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoundaryLayer {
    Laminar,
    Turbulent,
}

/// Explicit dynamic viscosity (Pa s), regime, and caller-selected validity bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkinFriction {
    dynamic_viscosity: f64,
    regime: BoundaryLayer,
    min_reynolds: f64,
    max_reynolds: f64,
}
impl SkinFriction {
    pub fn new(
        dynamic_viscosity: f64,
        regime: BoundaryLayer,
        min_reynolds: f64,
        max_reynolds: f64,
    ) -> Result<Self, HullError> {
        if !dynamic_viscosity.is_finite()
            || dynamic_viscosity <= 0.0
            || !min_reynolds.is_finite()
            || min_reynolds <= 0.0
            || !max_reynolds.is_finite()
            || max_reynolds < min_reynolds
        {
            return Err(HullError::InvalidParameters);
        }
        Ok(Self {
            dynamic_viscosity,
            regime,
            min_reynolds,
            max_reynolds,
        })
    }
    fn mean_coefficient(self, reynolds: f64) -> Result<f64, HullError> {
        if !reynolds.is_finite() {
            return Err(HullError::ArithmeticOverflow);
        }
        if reynolds < self.min_reynolds || reynolds > self.max_reynolds {
            return Err(HullError::UnsupportedReynolds);
        }
        Ok(match self.regime {
            BoundaryLayer::Laminar => 1.328 / reynolds.sqrt(),
            BoundaryLayer::Turbulent => 0.072 / reynolds.powf(0.2),
        })
    }
}

/// Geometry and empirical coefficients for one load point, not inferred from mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HullSection {
    local_transform: Transform,
    axis: Vec3,
    length: f64,
    axial_area: f64,
    crossflow_area: f64,
    wetted_area: f64,
    axial_cd: f64,
    crossflow_cd: f64,
    skin_friction: Option<SkinFriction>,
}
impl HullSection {
    /// Length must be positive; areas and coefficients must be nonnegative.
    /// The local axis is normalized. Reversing it does not change resistance.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        local_transform: Transform,
        axis: Vec3,
        length: f64,
        axial_area: f64,
        crossflow_area: f64,
        wetted_area: f64,
        axial_cd: f64,
        crossflow_cd: f64,
        skin_friction: Option<SkinFriction>,
    ) -> Result<Self, HullError> {
        let axis = axis.try_normalize().ok_or(HullError::InvalidParameters)?;
        if !length.is_finite()
            || length <= 0.0
            || [
                axial_area,
                crossflow_area,
                wetted_area,
                axial_cd,
                crossflow_cd,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err(HullError::InvalidParameters);
        }
        Ok(Self {
            local_transform,
            axis,
            length,
            axial_area,
            crossflow_area,
            wetted_area,
            axial_cd,
            crossflow_cd,
            skin_friction,
        })
    }
    pub fn local_transform(self) -> Transform {
        self.local_transform
    }

    /// Independent evaluator; atmosphere must be sampled at this section's point.
    /// Returns world force and moment about the current COM (body-local in flow).
    pub fn evaluate(self, flow: PressureFlow) -> Result<PressureWrench, HullError> {
        if !flow.center_of_mass.is_finite()
            || !flow.linear_velocity.is_finite()
            || !flow.angular_velocity.is_finite()
            || !flow.sound_speed.is_finite()
            || flow.sound_speed <= 0.0
        {
            return Err(HullError::InvalidFlow);
        }
        let zero = PressureWrench {
            force: Vec3::ZERO,
            moment: Vec3::ZERO,
        };
        if flow.atmosphere.density() == 0.0
            || (self.axial_area == 0.0 || self.axial_cd == 0.0)
                && (self.crossflow_area == 0.0 || self.crossflow_cd == 0.0)
                && (self.wetted_area == 0.0 || self.skin_friction.is_none())
        {
            return Ok(zero);
        }
        let overflow = HullError::ArithmeticOverflow;
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
        let speed = velocity.checked_length().ok_or(overflow)?;
        if speed / flow.sound_speed > 0.3 {
            return Err(HullError::UnsupportedMach);
        }
        if speed == 0.0 {
            return Ok(zero);
        }
        let axis = flow
            .transform
            .transform_vector(self.local_transform.transform_vector(self.axis));
        let axial_speed = velocity.checked_dot(axis).ok_or(overflow)?;
        let axial = axis.checked_mul_scalar(axial_speed).ok_or(overflow)?;
        let crossflow = velocity.checked_sub(axial).ok_or(overflow)?;
        let cross_speed = crossflow.checked_length().ok_or(overflow)?;
        let density = flow.atmosphere.density();
        let mut axial_force = resistance(
            axial,
            axial_speed.abs(),
            density,
            self.axial_area,
            self.axial_cd,
        )?;
        if axial_speed != 0.0
            && self.wetted_area != 0.0
            && let Some(friction) = self.skin_friction
        {
            let reynolds = scaled_product(
                &[density, axial_speed.abs(), self.length],
                friction.dynamic_viscosity,
            )?;
            let cf = friction.mean_coefficient(reynolds)?;
            axial_force = axial_force
                .checked_add(resistance(
                    axial,
                    axial_speed.abs(),
                    density,
                    self.wetted_area,
                    cf,
                )?)
                .ok_or(overflow)?;
        }
        let force = axial_force
            .checked_add(resistance(
                crossflow,
                cross_speed,
                density,
                self.crossflow_area,
                self.crossflow_cd,
            )?)
            .ok_or(overflow)?;
        let moment = arm.checked_cross(force).ok_or(overflow)?;
        Ok(PressureWrench { force, moment })
    }
}

fn resistance(
    velocity: Vec3,
    speed: f64,
    density: f64,
    area: f64,
    cd: f64,
) -> Result<Vec3, HullError> {
    if speed == 0.0 || area == 0.0 || cd == 0.0 {
        return Ok(Vec3::ZERO);
    }
    // Include each velocity component before restoring the exponent: even the
    // shared scalar can overflow while the complete force is representable.
    let component = |v: f64| {
        scaled_product(&[0.5, density, speed, area, cd, v.abs()], 1.0)
            .map(|magnitude| -magnitude.copysign(v))
    };
    Ok(Vec3::new(
        component(velocity.x)?,
        component(velocity.y)?,
        component(velocity.z)?,
    ))
}

// Products/quotients here have finite nonnegative factors and a positive divisor.
// Keep binary exponents separate so neither large nor tiny intermediate products
// lose a representable result. Unlike logarithms, power-of-two scaling is exact.
fn scaled_product(factors: &[f64], divisor: f64) -> Result<f64, HullError> {
    fn split(mut value: f64) -> (f64, i32) {
        let mut adjustment = 0;
        if value < f64::MIN_POSITIVE {
            value *= 2.0_f64.powi(54);
            adjustment = -54;
        }
        let bits = value.to_bits();
        let exponent = ((bits >> 52) & 0x7ff) as i32 - 1023 + adjustment;
        let mantissa = f64::from_bits((bits & ((1_u64 << 52) - 1)) | (1023_u64 << 52));
        (mantissa, exponent)
    }

    if factors.contains(&0.0) {
        return Ok(0.0);
    }
    let (denominator, denominator_exponent) = split(divisor);
    let mut mantissa = 1.0 / denominator;
    let mut exponent = -denominator_exponent;
    for &factor in factors {
        let (fraction, power) = split(factor);
        mantissa *= fraction;
        exponent += power;
        if mantissa >= 2.0 {
            mantissa *= 0.5;
            exponent += 1;
        }
    }
    if mantissa < 1.0 {
        mantissa *= 2.0;
        exponent -= 1;
    }
    // Chunking also handles subnormal outputs without constructing an infinite
    // or zero power of two before applying the mantissa.
    while exponent != 0 {
        let power = exponent.clamp(-1022, 1023);
        mantissa *= 2.0_f64.powi(power);
        exponent -= power;
    }
    mantissa
        .is_finite()
        .then_some(mantissa)
        .ok_or(HullError::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::AtmosphericState;
    use kapitsa_math::Quat;

    fn section(point: Vec3) -> HullSection {
        HullSection::new(
            Transform::new(point, Quat::IDENTITY).unwrap(),
            Vec3::X,
            2.0,
            3.0,
            4.0,
            5.0,
            0.2,
            0.8,
            None,
        )
        .unwrap()
    }
    fn flow(velocity: Vec3) -> PressureFlow {
        PressureFlow {
            atmosphere: AtmosphericState::new(1.2, Vec3::ZERO).unwrap(),
            sound_speed: 340.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::ZERO,
            linear_velocity: velocity,
            angular_velocity: Vec3::ZERO,
        }
    }
    fn close(a: Vec3, b: Vec3) {
        assert!((a - b).length() < 1e-9, "{a:?} != {b:?}");
    }

    #[test]
    fn analytic_components_reverse_flow_and_axis() {
        let s = section(Vec3::Y * 2.0);
        let v = Vec3::new(20.0, 10.0, 0.0);
        let w = s.evaluate(flow(v)).unwrap();
        close(w.force, Vec3::new(-144.0, -192.0, 0.0));
        close(w.moment, Vec3::Z * 288.0);
        assert!(w.force.dot(v) < 0.0);
        close(s.evaluate(flow(-v)).unwrap().force, -w.force);
        let mut reversed = s;
        reversed.axis = -Vec3::X;
        assert_eq!(reversed.evaluate(flow(v)), Ok(w));
        reversed.local_transform = Transform::new(
            Vec3::Y * 2.0,
            Quat::from_axis_angle(Vec3::Z, std::f64::consts::PI).unwrap(),
        )
        .unwrap();
        close(reversed.evaluate(flow(v)).unwrap().force, w.force);
    }

    #[test]
    fn mean_friction_uses_axial_reynolds_and_skips_zero_axial() {
        for (regime, cf) in [
            (BoundaryLayer::Laminar, 1.328 / 10000.0_f64.sqrt()),
            (BoundaryLayer::Turbulent, 0.072 / 10000.0_f64.powf(0.2)),
        ] {
            let mut s = section(Vec3::ZERO);
            s.axial_cd = 0.0;
            s.crossflow_cd = 0.0;
            s.skin_friction = Some(SkinFriction::new(0.0048, regime, 10000.0, 10000.0).unwrap());
            close(
                s.evaluate(flow(Vec3::new(20.0, 30.0, 0.0))).unwrap().force,
                Vec3::X * (-0.5 * 1.2 * 400.0 * 5.0 * cf),
            );
            assert_eq!(s.evaluate(flow(Vec3::Y * 30.0)).unwrap().force, Vec3::ZERO);
            assert_eq!(
                s.evaluate(flow(Vec3::X * 10.0)),
                Err(HullError::UnsupportedReynolds)
            );
        }
    }

    #[test]
    fn extreme_representable_loads_and_reynolds() {
        let mut s = section(Vec3::ZERO);
        s.length = 1.0;
        s.axial_area = 1e-4;
        s.crossflow_area = 1e-4;
        s.axial_cd = 1e-2;
        s.crossflow_cd = 1e-2;
        let mut f = flow(Vec3::X * 100.0);
        f.atmosphere = AtmosphericState::new(1e308, Vec3::ZERO).unwrap();
        for velocity in [Vec3::X * 100.0, Vec3::Y * 100.0, -Vec3::X * 100.0] {
            f.linear_velocity = velocity;
            let wrench = s.evaluate(f).unwrap();
            let component = if velocity.x != 0.0 {
                wrench.force.x / -velocity.x.signum()
            } else {
                -wrench.force.y
            };
            assert!((component / 5e305 - 1.0).abs() < 1e-14);
            assert_eq!(wrench.moment, Vec3::ZERO);
        }
        f.linear_velocity = Vec3::X * 100.0;
        s.axial_cd = 0.0;
        s.wetted_area = 1e-4;
        s.skin_friction =
            Some(SkinFriction::new(1e308, BoundaryLayer::Laminar, 100.0, 100.0).unwrap());
        let wrench = s.evaluate(f).unwrap();
        assert!((wrench.force.x / -6.64e306 - 1.0).abs() < 1e-14);
        s.skin_friction =
            Some(SkinFriction::new(1.0, BoundaryLayer::Laminar, 1.0, f64::MAX).unwrap());
        assert_eq!(s.evaluate(f), Err(HullError::ArithmeticOverflow));
        s.skin_friction = None;
        s.axial_cd = 1.0;
        s.axial_area = 1.0;
        assert_eq!(s.evaluate(f), Err(HullError::ArithmeticOverflow));
    }

    #[test]
    fn scaled_products_keep_small_factors_and_reject_overflow() {
        assert_eq!(scaled_product(&[1e308, 100.0, 1.0], 1e308), Ok(100.0));
        assert_eq!(
            scaled_product(&[f64::MAX, 2.0], 1.0),
            Err(HullError::ArithmeticOverflow)
        );
        let tiny = f64::from_bits(1);
        assert_eq!(scaled_product(&[tiny, 0.5, 2.0], 1.0), Ok(tiny));
        assert_eq!(scaled_product(&[tiny, f64::MAX], tiny), Ok(f64::MAX));
        let force = resistance(Vec3::X * 1e-100, 1e-100, 1e308, 1e308, 1e-200).unwrap();
        assert!((force.x / -5e215 - 1.0).abs() < 1e-14);
    }

    #[test]
    fn distributed_symmetry_and_angular_damping() {
        let mut f = flow(Vec3::ZERO);
        f.angular_velocity = Vec3::Z * 2.0;
        let a = section(-Vec3::X * 3.0).evaluate(f).unwrap();
        let b = section(Vec3::X * 3.0).evaluate(f).unwrap();
        close(a.force + b.force, Vec3::ZERO);
        assert!((a.moment + b.moment).dot(f.angular_velocity) < 0.0);
        let f = flow(Vec3::X * 20.0);
        close(
            section(-Vec3::Y).evaluate(f).unwrap().moment
                + section(Vec3::Y).evaluate(f).unwrap().moment,
            Vec3::ZERO,
        );
    }

    #[test]
    fn rigid_transform_and_com_frame_invariance() {
        let s = section(Vec3::new(1.0, 2.0, 3.0));
        let mut f = flow(Vec3::new(20.0, 10.0, 0.0));
        f.center_of_mass = Vec3::X * 0.5;
        f.angular_velocity = Vec3::Z;
        let w = s.evaluate(f).unwrap();
        let rotation = Quat::from_axis_angle(Vec3::Y, 0.7).unwrap();
        f.transform = Transform::new(Vec3::new(100.0, -200.0, 300.0), rotation).unwrap();
        f.linear_velocity = f.transform.transform_vector(f.linear_velocity);
        f.angular_velocity = f.transform.transform_vector(f.angular_velocity);
        let rotated = s.evaluate(f).unwrap();
        close(rotated.force, f.transform.transform_vector(w.force));
        close(rotated.moment, f.transform.transform_vector(w.moment));
        let mut shifted = s;
        shifted.local_transform = Transform::new(
            s.local_transform.translation() + Vec3::Y * 7.0,
            Quat::IDENTITY,
        )
        .unwrap();
        f.center_of_mass += Vec3::Y * 7.0;
        assert_eq!(shifted.evaluate(f), Ok(rotated));
    }

    #[test]
    fn domain_disabled_validation_and_overflow() {
        let s = section(Vec3::ZERO);
        assert!(s.evaluate(flow(Vec3::X * 102.0)).is_ok());
        assert_eq!(
            s.evaluate(flow(Vec3::X * 102.0001)),
            Err(HullError::UnsupportedMach)
        );
        let mut f = flow(Vec3::X * 200.0);
        f.atmosphere = AtmosphericState::new(0.0, Vec3::ZERO).unwrap();
        assert_eq!(s.evaluate(f).unwrap().force, Vec3::ZERO);
        f.sound_speed = f64::NAN;
        assert_eq!(s.evaluate(f), Err(HullError::InvalidFlow));
        let mut disabled = s;
        disabled.axial_cd = 0.0;
        disabled.crossflow_cd = 0.0;
        assert_eq!(
            disabled.evaluate(flow(Vec3::X * 200.0)).unwrap().force,
            Vec3::ZERO
        );
        assert_eq!(
            s.evaluate(flow(Vec3::X * f64::MAX)),
            Err(HullError::UnsupportedMach)
        );
        let mut huge = s;
        huge.axial_area = f64::MAX;
        assert_eq!(
            huge.evaluate(flow(Vec3::X * 50.0)),
            Err(HullError::ArithmeticOverflow)
        );
        assert!(SkinFriction::new(0.0, BoundaryLayer::Laminar, 1.0, 2.0).is_err());
        assert!(SkinFriction::new(1.0, BoundaryLayer::Laminar, 2.0, 1.0).is_err());
        assert!(
            HullSection::new(
                Transform::identity(),
                Vec3::ZERO,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                1.0,
                None
            )
            .is_err()
        );
    }
}
