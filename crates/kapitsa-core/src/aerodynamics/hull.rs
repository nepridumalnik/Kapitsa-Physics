//! Caller-parameterized distributed hull resistance.
//!
//! The default constant coefficients retain Mach <= 0.3. Explicitly attaching a
//! [`MachCurve`] enables a DATA-DRIVEN transonic resistance interface, not a
//! predictive FAR solver. No transonic coefficients are supplied by the engine.
//! Callers must supply measured/CFD coefficients consistent with each section's
//! reference areas, Reynolds range, angle of attack and component-speed convention
//! below; a Mach-only table cannot infer their dependence on Re or angle of attack.
//! The table replaces both constant Cd values throughout its inclusive domain;
//! outside that domain evaluation fails, without fallback or extrapolation.
//! Mach 1 is ordinary finite interpolation, not a singular correction factor.
//! Basic definitions (not coefficient data):
//! <https://en.wikipedia.org/wiki/Drag_coefficient>,
//! <https://en.wikipedia.org/wiki/Mach_number>,
//! <https://en.wikipedia.org/wiki/Transonic>.
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
//! These incompressible correlations have no compressible viscosity closure:
//! enabled wetted friction above total Mach 0.3 is explicitly rejected even when
//! a Mach curve is configured, including pure crossflow.
//! Finite kinematics and positive sound speed are validated even in vacuum or
//! with disabled areas; those cases then skip Mach and Reynolds domain checks.
//! Mach is based on the full local relative speed, including omega cross r.
//! Drag components use their own speed squared in q, not full-flow q. Optional
//! [`NormalForceCurve`] data instead uses full-flow q and unsigned axisymmetric
//! incidence in [0, pi]; positive Cn opposes crossflow, not pure lift. Length is
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
use std::sync::Arc;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HullError {
    InvalidParameters,
    InvalidFlow,
    UnsupportedMach,
    UnsupportedReynolds,
    /// The configured incompressible flat-plate correlation has no compressible closure.
    UnsupportedSkinFrictionMach,
    /// The wave drag approximation requires flow aligned with the body axis.
    UnsupportedWaveIncidence,
    /// The wave drag distribution could not be evaluated.
    Wave(super::wave::WaveDragError),
    ArithmeticOverflow,
}
impl std::fmt::Display for HullError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "hull evaluation failed: {self:?}")
    }
}
impl std::error::Error for HullError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Wave(error) => Some(error),
            _ => None,
        }
    }
}

/// A caller-supplied Mach knot and dimensionless sectional resistance coefficients.
/// Fields are validated when constructing a [`MachCurve`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MachKnot {
    pub mach: f64,
    pub axial_cd: f64,
    pub crossflow_cd: f64,
}

/// Immutable, cheaply shared, piecewise-linear measured/CFD coefficient table.
/// At least two finite, nonnegative, strictly increasing Mach knots are required;
/// both Cd values must be finite and nonnegative. No extrapolation is performed.
#[derive(Clone, Debug, PartialEq)]
pub struct MachCurve {
    knots: Arc<[MachKnot]>,
}
impl MachCurve {
    pub fn new(knots: Vec<MachKnot>) -> Result<Self, HullError> {
        if knots.len() < 2
            || knots.iter().any(|k| {
                [k.mach, k.axial_cd, k.crossflow_cd]
                    .iter()
                    .any(|v| !v.is_finite() || *v < 0.0)
            })
            || knots.windows(2).any(|pair| pair[0].mach >= pair[1].mach)
        {
            return Err(HullError::InvalidParameters);
        }
        Ok(Self {
            knots: knots.into(),
        })
    }

    pub fn knots(&self) -> &[MachKnot] {
        &self.knots
    }

    /// Returns (axial Cd, crossflow Cd), including both endpoints exactly.
    /// Nonfinite or negative input is invalid; finite out-of-domain Mach is unsupported.
    pub fn coefficients(&self, mach: f64) -> Result<(f64, f64), HullError> {
        if !mach.is_finite() || mach < 0.0 {
            return Err(HullError::InvalidFlow);
        }
        for pair in self.knots.windows(2) {
            let [a, b] = [pair[0], pair[1]];
            if mach == a.mach {
                return Ok((a.axial_cd, a.crossflow_cd));
            }
            if mach == b.mach {
                return Ok((b.axial_cd, b.crossflow_cd));
            }
            if mach > a.mach && mach < b.mach {
                let t = (mach - a.mach) / (b.mach - a.mach);
                // Interpolate from the smaller coefficient to avoid overflowing
                // an otherwise representable convex combination near f64::MAX.
                let interpolate = |x: f64, y: f64| {
                    if x <= y {
                        x + t * (y - x)
                    } else {
                        y + (1.0 - t) * (x - y)
                    }
                };
                return Ok((
                    interpolate(a.axial_cd, b.axial_cd),
                    interpolate(a.crossflow_cd, b.crossflow_cd),
                ));
            }
        }
        Err(HullError::UnsupportedMach)
    }

    fn has_axial_resistance(&self) -> bool {
        self.knots.iter().any(|k| k.axial_cd != 0.0)
    }
    fn has_crossflow_resistance(&self) -> bool {
        self.knots.iter().any(|k| k.crossflow_cd != 0.0)
    }
}

/// A caller-supplied axisymmetric normal-force coefficient at Mach and incidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalForceKnot {
    pub mach: f64,
    /// Unsigned incidence in radians: atan2(|v_perp|, v_axial), in [0, pi].
    pub angle_of_attack: f64,
    /// Positive Cn opposes crossflow; negative Cn intentionally reverses that force.
    pub normal_cn: f64,
}

/// Immutable rectangular CFD/experimental axisymmetric normal-force table.
/// Values are bilinearly interpolated only inside its inclusive Mach/incidence
/// domain. There is no fixed reference side or signed planar angle of attack.
/// Force uses full-speed dynamic pressure and points opposite crossflow for
/// positive Cn: it is normal to the axis, not to the velocity (not pure lift).
/// Supply measured coefficients matching this convention; none are provided.
#[derive(Clone, Debug, PartialEq)]
pub struct NormalForceCurve {
    reference_area: f64,
    knots: Arc<[NormalForceKnot]>,
    mach_count: usize,
    angle_count: usize,
}
impl NormalForceCurve {
    pub fn new(reference_area: f64, knots: Vec<NormalForceKnot>) -> Result<Self, HullError> {
        if !reference_area.is_finite() || reference_area <= 0.0 || knots.len() < 4 {
            return Err(HullError::InvalidParameters);
        }
        if knots.iter().any(|k| {
            !k.mach.is_finite()
                || k.mach < 0.0
                || !k.angle_of_attack.is_finite()
                || k.angle_of_attack < 0.0
                || k.angle_of_attack > std::f64::consts::PI
                || !k.normal_cn.is_finite()
        }) {
            return Err(HullError::InvalidParameters);
        }
        let mut machs = Vec::new();
        let mut angles = Vec::new();
        for k in &knots {
            if !machs.contains(&k.mach) {
                machs.push(k.mach);
            }
            if !angles.contains(&k.angle_of_attack) {
                angles.push(k.angle_of_attack);
            }
        }
        machs.sort_by(f64::total_cmp);
        angles.sort_by(f64::total_cmp);
        if machs.len() < 2
            || angles.len() < 2
            || knots.len() != machs.len() * angles.len()
            || knots.iter().any(|k| {
                knots
                    .iter()
                    .filter(|x| x.mach == k.mach && x.angle_of_attack == k.angle_of_attack)
                    .count()
                    != 1
            })
        {
            return Err(HullError::InvalidParameters);
        }
        let mut ordered = knots;
        ordered.sort_by(|a, b| {
            a.mach
                .total_cmp(&b.mach)
                .then(a.angle_of_attack.total_cmp(&b.angle_of_attack))
        });
        Ok(Self {
            reference_area,
            knots: ordered.into(),
            mach_count: machs.len(),
            angle_count: angles.len(),
        })
    }
    pub fn reference_area(&self) -> f64 {
        self.reference_area
    }
    pub fn knots(&self) -> &[NormalForceKnot] {
        &self.knots
    }
    fn coefficient(&self, mach: f64, alpha: f64) -> Result<f64, HullError> {
        if !mach.is_finite() || mach < 0.0 || !alpha.is_finite() {
            return Err(HullError::InvalidFlow);
        }
        let m = (0..self.mach_count - 1)
            .find_map(|i| {
                let a = self.knots[i * self.angle_count].mach;
                let b = self.knots[(i + 1) * self.angle_count].mach;
                (mach >= a && mach <= b).then_some((i, (mach - a) / (b - a)))
            })
            .ok_or(HullError::UnsupportedMach)?;
        let j = (0..self.angle_count - 1)
            .find_map(|j| {
                let a = self.knots[j].angle_of_attack;
                let b = self.knots[j + 1].angle_of_attack;
                (alpha >= a && alpha <= b).then_some((j, (alpha - a) / (b - a)))
            })
            .ok_or(HullError::UnsupportedMach)?;
        // Opposite-sign differences can overflow even for a finite convex result.
        let lerp = |x: f64, y: f64, t: f64| {
            if x.is_sign_negative() != y.is_sign_negative() {
                (1.0 - t) * x + t * y
            } else if x <= y {
                x + t * (y - x)
            } else {
                y + (1.0 - t) * (x - y)
            }
        };
        let a = lerp(
            self.knots[m.0 * self.angle_count + j.0].normal_cn,
            self.knots[m.0 * self.angle_count + j.0 + 1].normal_cn,
            j.1,
        );
        let b = lerp(
            self.knots[(m.0 + 1) * self.angle_count + j.0].normal_cn,
            self.knots[(m.0 + 1) * self.angle_count + j.0 + 1].normal_cn,
            j.1,
        );
        Ok(lerp(a, b, m.1))
    }
}

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
/// Clone shares immutable Mach data; sections are no longer Copy.
#[derive(Clone, Debug, PartialEq)]
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
    mach_curve: Option<MachCurve>,
    normal_curve: Option<NormalForceCurve>,
    wave_drag_factor: Option<f64>,
    wave_mach_curve: Option<super::wave::WaveMachCurve>,
}
impl HullSection {
    /// Length must be positive; areas and coefficients must be nonnegative.
    /// The local axis is normalized. Reversing it leaves drag unchanged but maps
    /// normal-table incidence to pi - alpha, so asymmetric data can change Cn.
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
            mach_curve: None,
            normal_curve: None,
            wave_drag_factor: None,
            wave_mach_curve: None,
        })
    }

    /// Add zero-incidence supersonic wave drag for a slender axisymmetric body.
    /// Samples are equally spaced values of dA/dx in metres; the first and last
    /// values must be zero. The force acts at this section's origin. This model
    /// does not infer geometry or a center of pressure. Other configured drag is
    /// additive, so callers must avoid counting wave drag twice. Evaluation is
    /// limited to Mach > 1 and flow aligned with the section axis unless an
    /// explicit Mach multiplier curve supplies a different range.
    pub fn with_wave_drag(mut self, area_slopes: &[f64]) -> Result<Self, HullError> {
        let factor =
            super::wave::wave_drag(area_slopes, 0.25, 2.0, 1.0).map_err(HullError::Wave)?;
        self.wave_drag_factor = (factor != 0.0).then_some(factor);
        Ok(self)
    }

    /// Configures wave drag from one axis of a prepared voxel union.
    ///
    /// `axis` selects the body-local X, Y or Z sections (0, 1 or 2).
    /// `smoothing_radius` is an explicit box-filter radius in cells. The caller
    /// must choose the force application point and validate resolution,
    /// smoothing and applicability
    /// to a slender axisymmetric body. No automatic center of pressure or Mach
    /// transition is inferred, and geometry changes require reconfiguration.
    pub fn with_voxel_wave_drag(
        self,
        geometry: &super::geometry::PreparedGeometry,
        axis: usize,
        smoothing_radius: usize,
    ) -> Result<Self, HullError> {
        let sections = geometry
            .sections()
            .get(axis)
            .ok_or(HullError::InvalidParameters)?;
        let expected_axis = match axis {
            0 => Vec3::X,
            1 => Vec3::Y,
            2 => Vec3::Z,
            _ => return Err(HullError::InvalidParameters),
        };
        let actual_axis = self.local_transform.transform_vector(self.axis);
        let misalignment = actual_axis
            .checked_cross(expected_axis)
            .and_then(Vec3::checked_length)
            .ok_or(HullError::ArithmeticOverflow)?;
        if misalignment > 64.0 * f64::EPSILON {
            return Err(HullError::InvalidParameters);
        }
        let slopes = super::wave::area_slopes_from_sections(
            sections,
            geometry.cell_size(),
            smoothing_radius,
        )
        .map_err(HullError::Wave)?;
        self.with_wave_drag(&slopes)
    }

    /// Attach caller-supplied Mach multipliers to the wave-drag factor.
    ///
    /// The curve applies only when wave drag has also been configured. It can
    /// represent a transition only when backed by suitable data; no universal
    /// transition is inferred. The curve's domain is enforced without fallback.
    pub fn with_wave_mach_curve(mut self, curve: super::wave::WaveMachCurve) -> Self {
        self.wave_mach_curve = Some(curve);
        self
    }

    /// Replace constant Cd values with explicit caller data, without changing geometry.
    /// Cloning the curve shares its immutable allocation across sections.
    /// SkinFriction remains incompressible and cannot be used above Mach 0.3.
    pub fn with_mach_curve(mut self, curve: MachCurve) -> Self {
        self.mach_curve = Some(curve);
        self
    }

    pub fn mach_curve(&self) -> Option<&MachCurve> {
        self.mach_curve.as_ref()
    }

    /// Attach axisymmetric normal-force data using unsigned incidence [0, pi].
    /// Positive Cn opposes crossflow and dissipates relative-flow energy; negative
    /// Cn reverses it intentionally. This is not a pure lift model.
    /// With zero crossflow the force is zero and the normal table is not sampled
    /// (including zero total speed); its Mach/incidence domain is then skipped.
    pub fn with_normal_curve(mut self, curve: NormalForceCurve) -> Self {
        self.normal_curve = Some(curve);
        self
    }

    pub fn normal_curve(&self) -> Option<&NormalForceCurve> {
        self.normal_curve.as_ref()
    }

    pub fn local_transform(&self) -> Transform {
        self.local_transform
    }

    /// Independent evaluator; atmosphere must be sampled at this section's point.
    /// Returns world force and moment about the current COM (body-local in flow).
    pub fn evaluate(&self, flow: PressureFlow) -> Result<PressureWrench, HullError> {
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
        let axial_enabled = self
            .mach_curve
            .as_ref()
            .map_or(self.axial_cd != 0.0, MachCurve::has_axial_resistance);
        let crossflow_enabled = self.mach_curve.as_ref().map_or(
            self.crossflow_cd != 0.0,
            MachCurve::has_crossflow_resistance,
        );
        let drag_enabled = (self.axial_area != 0.0 && axial_enabled)
            || (self.crossflow_area != 0.0 && crossflow_enabled);
        if flow.atmosphere.density() == 0.0
            || !drag_enabled
                && self.normal_curve.is_none()
                && self.wave_drag_factor.is_none()
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
        let mach = speed / flow.sound_speed;
        if !mach.is_finite() {
            return Err(HullError::UnsupportedMach);
        }
        // Unused drag coefficients must not gate normal-only or friction loads.
        let (axial_cd, crossflow_cd) = if !drag_enabled {
            (0.0, 0.0)
        } else if let Some(curve) = &self.mach_curve {
            curve.coefficients(mach)?
        } else {
            if mach > 0.3 {
                return Err(HullError::UnsupportedMach);
            }
            (self.axial_cd, self.crossflow_cd)
        };
        // No compressible viscosity correlation is supplied. Reject explicit
        // friction above its domain even if this point's axial speed is zero.
        if mach > 0.3 && self.wetted_area != 0.0 && self.skin_friction.is_some() {
            return Err(HullError::UnsupportedSkinFrictionMach);
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
        let wave_force = if let Some(factor) = self.wave_drag_factor {
            let multiplier = match &self.wave_mach_curve {
                Some(curve) => curve.multiplier(mach).map_err(HullError::Wave)?,
                None if mach > 1.0 => 1.0,
                None => return Err(HullError::UnsupportedMach),
            };
            if multiplier == 0.0 {
                Vec3::ZERO
            } else {
                if cross_speed > speed * (64.0 * f64::EPSILON) {
                    return Err(HullError::UnsupportedWaveIncidence);
                }
                {
                    let component = |v: f64| {
                        scaled_product(
                            &[density, axial_speed.abs(), factor, multiplier, v.abs()],
                            1.0,
                        )
                        .map(|magnitude| -magnitude.copysign(v))
                    };
                    Vec3::new(
                        component(axial.x)?,
                        component(axial.y)?,
                        component(axial.z)?,
                    )
                }
            }
        } else {
            Vec3::ZERO
        };
        let mut axial_force =
            resistance(axial, axial_speed.abs(), density, self.axial_area, axial_cd)?;
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
        let cross_force = resistance(
            crossflow,
            cross_speed,
            density,
            self.crossflow_area,
            crossflow_cd,
        )?;
        let normal_force = if let Some(curve) = &self.normal_curve
            && cross_speed != 0.0
        {
            let alpha = cross_speed.atan2(axial_speed);
            let cn = curve.coefficient(mach, alpha)?;
            // Scale each complete component, including the normalization divisor.
            // Neither q*A*Cn nor magnitude/|v_perp| need fit on their own.
            let component = |v: f64| {
                scaled_product(
                    &[
                        0.5,
                        density,
                        speed,
                        speed,
                        curve.reference_area,
                        cn.abs(),
                        v.abs(),
                    ],
                    cross_speed,
                )
                .map(|magnitude| -magnitude.copysign(v) * cn.signum())
            };
            Vec3::new(
                component(crossflow.x)?,
                component(crossflow.y)?,
                component(crossflow.z)?,
            )
        } else {
            Vec3::ZERO
        };
        let force = axial_force
            .checked_add(cross_force)
            .and_then(|f| f.checked_add(normal_force))
            .and_then(|f| f.checked_add(wave_force))
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
pub(super) fn scaled_product(factors: &[f64], divisor: f64) -> Result<f64, HullError> {
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
    fn wave_mach_curve_uses_explicit_transition_without_extrapolation()
    -> Result<(), Box<dyn std::error::Error>> {
        use crate::aerodynamics::wave::{WaveDragError, WaveMachCurve};
        let curve = WaveMachCurve::new(vec![(0.8, 0.0), (1.0, 0.5), (1.2, 1.0)])?;
        assert!((curve.multiplier(0.9)? - 0.25).abs() < 1e-12);
        assert_eq!(curve.multiplier(1.3), Err(WaveDragError::UnsupportedMach));
        let section = HullSection::new(
            Transform::identity(),
            Vec3::X,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            None,
        )?
        .with_wave_drag(&[0.0, 1.0, -1.0, 0.0])?
        .with_wave_mach_curve(curve);
        assert_eq!(
            section
                .evaluate(flow(Vec3::new(0.8 * 340.0, 0.0, 0.0)))?
                .force,
            Vec3::ZERO
        );
        let at_one = section.evaluate(flow(Vec3::X * 340.0))?;
        let reference = super::super::wave::wave_drag(&[0.0, 1.0, -1.0, 0.0], 1.2, 680.0, 340.0)?;
        assert!((at_one.force.x + reference / 8.0).abs() < reference * 1e-12);
        assert!(matches!(
            section.evaluate(flow(Vec3::X * 500.0)),
            Err(HullError::Wave(WaveDragError::UnsupportedMach))
        ));
        Ok(())
    }

    #[test]
    fn zero_wave_distribution_does_not_restrict_other_loads()
    -> Result<(), Box<dyn std::error::Error>> {
        let section = HullSection::new(
            Transform::identity(),
            Vec3::X,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            None,
        )?
        .with_wave_drag(&[0.0, 0.0, 0.0])?;
        let result = section.evaluate(flow(Vec3::new(50.0, 1.0, 0.0)))?;
        assert_eq!(result.force, Vec3::ZERO);
        assert_eq!(result.moment, Vec3::ZERO);
        Ok(())
    }

    #[test]
    fn wave_drag_section_force_moment_and_domain() -> Result<(), Box<dyn std::error::Error>> {
        let section = HullSection::new(
            Transform::new(Vec3::Y, Quat::IDENTITY)?,
            Vec3::X,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            None,
        )?
        .with_wave_drag(&[0.0, 1.0, -1.0, 0.0])?;
        let moving = flow(Vec3::X * 680.0);
        let wrench = section.evaluate(moving)?;
        let expected = super::super::wave::wave_drag(&[0.0, 1.0, -1.0, 0.0], 1.2, 680.0, 340.0)?;
        assert!((wrench.force.x + expected).abs() < expected * 1e-12);
        assert_eq!(wrench.force.y, 0.0);
        assert!((wrench.moment.z - expected).abs() < expected * 1e-12);
        assert_eq!(
            section.evaluate(flow(Vec3::X * 340.0)),
            Err(HullError::UnsupportedMach)
        );
        assert_eq!(
            section.evaluate(flow(Vec3::new(680.0, 1.0, 0.0))),
            Err(HullError::UnsupportedWaveIncidence)
        );
        let reverse = section.evaluate(flow(-Vec3::X * 680.0))?;
        assert!((reverse.force.x - expected).abs() < expected * 1e-12);
        Ok(())
    }

    // Synthetic arithmetic fixtures only: these are not physical coefficient data.
    fn curve() -> MachCurve {
        MachCurve::new(vec![
            MachKnot {
                mach: 0.5,
                axial_cd: 0.2,
                crossflow_cd: 0.4,
            },
            MachKnot {
                mach: 1.0,
                axial_cd: 0.6,
                crossflow_cd: 0.8,
            },
            MachKnot {
                mach: 1.5,
                axial_cd: 0.4,
                crossflow_cd: 0.6,
            },
        ])
        .unwrap()
    }

    // Synthetic axisymmetric fixtures, not measured aerodynamic data.
    fn normal_curve(cn: f64) -> NormalForceCurve {
        NormalForceCurve::new(
            2.0,
            [0.5, 1.5]
                .into_iter()
                .flat_map(|mach| {
                    [0.0, std::f64::consts::PI]
                        .into_iter()
                        .map(move |angle_of_attack| NormalForceKnot {
                            mach,
                            angle_of_attack,
                            normal_cn: cn,
                        })
                })
                .collect(),
        )
        .unwrap()
    }

    fn normal_only(cn: f64) -> HullSection {
        let mut s = section(Vec3::ZERO).with_normal_curve(normal_curve(cn));
        s.axial_area = 0.0;
        s.crossflow_area = 0.0;
        s
    }

    #[test]
    fn normal_only_transonic_crossflow_sign_and_axis_incidence() {
        let s = normal_only(0.5);
        for sign in [-1.0, 1.0] {
            let v = Vec3::new(204.0, sign * 272.0, 0.0);
            let w = s.evaluate(flow(v)).unwrap();
            close(w.force, Vec3::Y * (-sign * 0.5 * 1.2 * 340.0 * 340.0));
            assert!(w.force.dot(v) < 0.0);
            let negative = normal_only(-0.5).evaluate(flow(v)).unwrap();
            close(negative.force, -w.force);
        }
        // A zero constant Cd also disables drag without disabling a normal table.
        let mut zero_cd = s.clone();
        zero_cd.axial_area = 3.0;
        zero_cd.crossflow_area = 4.0;
        zero_cd.axial_cd = 0.0;
        zero_cd.crossflow_cd = 0.0;
        assert_eq!(
            zero_cd.evaluate(flow(Vec3::Y * 340.0)),
            s.evaluate(flow(Vec3::Y * 340.0))
        );

        let mut asymmetric = normal_curve(0.0).knots().to_vec();
        for k in &mut asymmetric {
            k.normal_cn = k.angle_of_attack;
        }
        let mut s = s.with_normal_curve(NormalForceCurve::new(2.0, asymmetric).unwrap());
        let v = Vec3::new(204.0, 272.0, 0.0);
        let alpha = 272.0_f64.atan2(204.0);
        let first = s.evaluate(flow(v)).unwrap();
        s.axis = -Vec3::X;
        let reversed = s.evaluate(flow(v)).unwrap();
        assert!(
            (reversed.force.y / first.force.y - (std::f64::consts::PI - alpha) / alpha).abs()
                < 1e-14
        );
        assert!(first.force.y < 0.0 && reversed.force.y < 0.0);
    }

    #[test]
    fn normal_zero_crossflow_skips_only_its_domain_and_domains_are_independent() {
        let s = normal_only(0.5);
        for v in [Vec3::ZERO, Vec3::X * 680.0, -Vec3::X * 680.0] {
            assert_eq!(s.evaluate(flow(v)).unwrap().force, Vec3::ZERO);
        }
        assert_eq!(
            s.evaluate(flow(Vec3::Y * 680.0)),
            Err(HullError::UnsupportedMach)
        );
        let mut drag = s.clone().with_mach_curve(curve());
        drag.axial_area = 3.0;
        assert_eq!(
            drag.evaluate(flow(Vec3::ZERO)),
            Err(HullError::UnsupportedMach)
        );
        let narrow = NormalForceCurve::new(
            2.0,
            normal_curve(0.5)
                .knots()
                .iter()
                .map(|k| NormalForceKnot {
                    mach: k.mach + 1.0,
                    ..*k
                })
                .collect(),
        )
        .unwrap();
        let drag = section(Vec3::ZERO)
            .with_mach_curve(curve())
            .with_normal_curve(narrow);
        assert!(drag.evaluate(flow(Vec3::X * 340.0)).is_ok());
        assert_eq!(
            drag.evaluate(flow(Vec3::Y * 340.0)),
            Err(HullError::UnsupportedMach)
        );
        // Normal data cannot extend the domain of enabled constant drag.
        let constant = section(Vec3::ZERO).with_normal_curve(normal_curve(0.5));
        assert_eq!(
            constant.evaluate(flow(Vec3::Y * 340.0)),
            Err(HullError::UnsupportedMach)
        );
    }

    #[test]
    fn normal_tables_reject_invalid_data_and_interpolate_signed_extremes() {
        let valid = normal_curve(0.5).knots().to_vec();
        for knots in [vec![], valid[..3].to_vec(), vec![valid[0]; 4]] {
            assert_eq!(
                NormalForceCurve::new(2.0, knots),
                Err(HullError::InvalidParameters)
            );
        }
        for area in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(
                NormalForceCurve::new(area, valid.clone()),
                Err(HullError::InvalidParameters)
            );
        }
        for bad in [-0.1, std::f64::consts::PI + 0.1, f64::NAN, f64::INFINITY] {
            let mut knots = valid.clone();
            knots[0].angle_of_attack = bad;
            assert_eq!(
                NormalForceCurve::new(2.0, knots),
                Err(HullError::InvalidParameters)
            );
        }
        for bad in [-1.0, f64::NAN, f64::INFINITY] {
            let mut knots = valid.clone();
            knots[0].mach = bad;
            assert_eq!(
                NormalForceCurve::new(2.0, knots),
                Err(HullError::InvalidParameters)
            );
        }
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut knots = valid.clone();
            knots[0].normal_cn = bad;
            assert_eq!(
                NormalForceCurve::new(2.0, knots),
                Err(HullError::InvalidParameters)
            );
        }
        let mut knots = valid;
        for k in &mut knots {
            k.normal_cn = if k.angle_of_attack == 0.0 {
                -f64::MAX
            } else {
                f64::MAX
            };
        }
        let c = NormalForceCurve::new(2.0, knots).unwrap();
        assert_eq!(c.coefficient(1.0, std::f64::consts::FRAC_PI_2), Ok(0.0));
        for k in c.knots() {
            assert_eq!(c.coefficient(k.mach, k.angle_of_attack), Ok(k.normal_cn));
        }
        assert_eq!(c.coefficient(1.0, -0.1), Err(HullError::UnsupportedMach));
        assert_eq!(c.coefficient(1.0, f64::NAN), Err(HullError::InvalidFlow));
    }

    #[test]
    fn normal_extreme_components_remain_finite_or_report_true_overflow() {
        let mut s = normal_only(1.0);
        s.normal_curve =
            Some(NormalForceCurve::new(1e-4, normal_curve(1e-2).knots().to_vec()).unwrap());
        let mut f = flow(Vec3::new(1e100, 1e-100, 0.0));
        f.sound_speed = 1e100;
        // Magnitude/|v_perp| overflows, but the complete component fits.
        f.atmosphere = AtmosphericState::new(1e100, Vec3::ZERO).unwrap();
        let w = s.evaluate(f).unwrap();
        assert!((w.force.y / -5e293 - 1.0).abs() < 1e-14);
        // A vector magnitude may overflow while each output component fits.
        let s = normal_only(1.0);
        let mut f = flow(Vec3::new(0.0, 1.0, 1.0));
        f.sound_speed = f.linear_velocity.length();
        f.atmosphere = AtmosphericState::new(1e308, Vec3::ZERO).unwrap();
        let w = s.evaluate(f).unwrap();
        assert!(w.force.is_finite());
        assert!((w.force.y / -std::f64::consts::SQRT_2 / 1e308 - 1.0).abs() < 1e-14);
        f.atmosphere = AtmosphericState::new(f64::MAX, Vec3::ZERO).unwrap();
        assert_eq!(s.evaluate(f), Err(HullError::ArithmeticOverflow));
    }

    #[test]
    fn mach_curve_knots_interpolation_and_explicit_domain() {
        let c = curve();
        for k in c.knots() {
            assert_eq!(c.coefficients(k.mach), Ok((k.axial_cd, k.crossflow_cd)));
        }
        for (mach, expected) in [(0.75, (0.4, 0.6)), (1.25, (0.5, 0.7))] {
            let (a, b) = c.coefficients(mach).unwrap();
            assert!((a - expected.0).abs() < 1e-14);
            assert!((b - expected.1).abs() < 1e-14);
        }
        // Mach 1 need not itself be a knot; there is no sonic singularity.
        let across = MachCurve::new(vec![c.knots()[0], c.knots()[2]]).unwrap();
        let (a, b) = across.coefficients(1.0).unwrap();
        assert!((a - 0.3).abs() < 1e-14 && (b - 0.5).abs() < 1e-14);
        for mach in [0.0, 0.4999, 1.5001, f64::MAX] {
            assert_eq!(c.coefficients(mach), Err(HullError::UnsupportedMach));
        }
        for mach in [f64::NAN, f64::INFINITY, -0.1] {
            assert_eq!(c.coefficients(mach), Err(HullError::InvalidFlow));
        }
        let large = MachCurve::new(vec![
            MachKnot {
                mach: 0.0,
                axial_cd: f64::MAX,
                crossflow_cd: f64::MAX,
            },
            MachKnot {
                mach: 2.0,
                axial_cd: f64::MAX,
                crossflow_cd: 0.0,
            },
        ])
        .unwrap();
        assert_eq!(large.coefficients(1.0), Ok((f64::MAX, f64::MAX * 0.5)));
    }

    #[test]
    fn mach_curve_rejects_invalid_tables_and_shares_immutable_storage() {
        let valid = curve().knots().to_vec();
        for knots in [
            vec![],
            vec![valid[0]],
            vec![valid[1], valid[0]],
            vec![valid[0], valid[0]],
        ] {
            assert_eq!(MachCurve::new(knots), Err(HullError::InvalidParameters));
        }
        for bad in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for field in 0..3 {
                let mut knots = valid.clone();
                match field {
                    0 => knots[1].mach = bad,
                    1 => knots[1].axial_cd = bad,
                    _ => knots[1].crossflow_cd = bad,
                }
                assert_eq!(MachCurve::new(knots), Err(HullError::InvalidParameters));
            }
        }
        let c = curve();
        let a = section(Vec3::ZERO).with_mach_curve(c.clone());
        let b = section(Vec3::Y).with_mach_curve(c);
        assert!(Arc::ptr_eq(
            &a.mach_curve().unwrap().knots,
            &b.mach_curve().unwrap().knots
        ));
    }

    #[test]
    fn configured_transonic_resistance_uses_local_full_speed_and_com() {
        let mut s = section(Vec3::Y * 2.0).with_mach_curve(curve());
        // Constant zero Cd must not disable a configured nonzero table.
        s.axial_cd = 0.0;
        s.crossflow_cd = 0.0;
        for mach in [0.5, 0.75, 1.0, 1.25, 1.5] {
            let speed = 340.0 * mach;
            let w = s.evaluate(flow(Vec3::X * speed)).unwrap();
            let cd = s.mach_curve().unwrap().coefficients(mach).unwrap().0;
            close(w.force, Vec3::X * (-0.5 * 1.2 * speed * speed * 3.0 * cd));
            close(w.moment, (Vec3::Y * 2.0).cross(w.force));
            assert!(w.force.is_finite() && w.force.x < 0.0);
        }
        let mut f = flow(Vec3::new(204.0, 272.0, 0.0));
        f.center_of_mass = Vec3::Y;
        f.angular_velocity = Vec3::Z * 20.0;
        let local_velocity = f.linear_velocity + f.angular_velocity.cross(Vec3::Y);
        let mach = local_velocity.length() / f.sound_speed;
        let (a, b) = curve().coefficients(mach).unwrap();
        let w = s.evaluate(f).unwrap();
        close(
            w.force,
            Vec3::new(
                -0.5 * 1.2 * 184.0_f64.powi(2) * 3.0 * a,
                -0.5 * 1.2 * 272.0_f64.powi(2) * 4.0 * b,
                0.0,
            ),
        );
        close(w.moment, Vec3::Y.cross(w.force));
        let rotation = Quat::from_axis_angle(Vec3::Z, 0.7).unwrap();
        f.transform = Transform::new(Vec3::new(100.0, -20.0, 4.0), rotation).unwrap();
        f.linear_velocity = f.transform.transform_vector(f.linear_velocity);
        f.angular_velocity = f.transform.transform_vector(f.angular_velocity);
        let rotated = s.evaluate(f).unwrap();
        close(rotated.force, f.transform.transform_vector(w.force));
        close(rotated.moment, f.transform.transform_vector(w.moment));
    }

    #[test]
    fn transonic_vacuum_disabled_and_incompressible_friction_domains() {
        let s = section(Vec3::ZERO).with_mach_curve(curve());
        assert_eq!(
            s.evaluate(flow(Vec3::X * 100.0)),
            Err(HullError::UnsupportedMach)
        );
        assert_eq!(
            s.evaluate(flow(Vec3::X * 600.0)),
            Err(HullError::UnsupportedMach)
        );
        let mut f = flow(Vec3::X * 600.0);
        f.atmosphere = AtmosphericState::new(0.0, Vec3::ZERO).unwrap();
        assert_eq!(s.evaluate(f).unwrap().force, Vec3::ZERO);
        let mut disabled = s.clone();
        disabled.axial_area = 0.0;
        disabled.crossflow_area = 0.0;
        assert_eq!(
            disabled.evaluate(flow(Vec3::X * 600.0)).unwrap().force,
            Vec3::ZERO
        );
        let mut viscous = s;
        viscous.skin_friction =
            Some(SkinFriction::new(1e-5, BoundaryLayer::Laminar, 1.0, 1e12).unwrap());
        for velocity in [Vec3::X * 340.0, Vec3::Y * 340.0] {
            assert_eq!(
                viscous.evaluate(flow(velocity)),
                Err(HullError::UnsupportedSkinFrictionMach)
            );
        }
        // Vacuum still skips correlation domains, not finite-flow validation.
        assert_eq!(viscous.evaluate(f).unwrap().force, Vec3::ZERO);
        f.sound_speed = f64::NAN;
        assert_eq!(viscous.evaluate(f), Err(HullError::InvalidFlow));
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
        let mut shifted = s.clone();
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
        let mut disabled = s.clone();
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
