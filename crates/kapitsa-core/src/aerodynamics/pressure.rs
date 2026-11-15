//! Newtonian hypersonic impact pressure on caller-selected physical surface patches.
//!
//! Source: <https://en.wikipedia.org/wiki/Pressure_coefficient>, Cp = 2 sin²(theta).
//! Leeward excess pressure is zero. Mach >= 5 is an explicit implementation
//! policy, not a universal accuracy guarantee. No suction, friction, wake,
//! circulation, transonic correction or shock interaction is resolved.
//! Callers must supply exposed original-surface normals and areas. In particular,
//! voxel staircase normals are not a consistent replacement for smooth normals.
//! Overlap removal and upstream shielding are the caller's responsibility.

use super::{AtmosphericState, geometry::PreparedSurfacePatches};
use crate::transform::Transform;
use kapitsa_math::Vec3;
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PressureError {
    InvalidPatch,
    InvalidMotion,
    InvalidSoundSpeed,
    InvalidCoefficient,
    InvalidBaseOrientation,
    BaseNotFullyVisible,
    UnsupportedMach,
    ArithmeticOverflow,
}
impl fmt::Display for PressureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Newtonian pressure evaluation failed: {self:?}")
    }
}
impl std::error::Error for PressureError {}

fn same_triangle(a: &[Vec3; 3], b: &[Vec3; 3]) -> bool {
    (0..3).any(|shift| {
        (0..3).all(|i| a[i] == b[(i + shift) % 3]) || (0..3).all(|i| a[i] == b[(shift + 3 - i) % 3])
    })
}

/// Reject selected triangles that do not uniquely identify a source face.
pub(crate) fn verify_unique_base_sources(
    selected: &[[Vec3; 3]],
    mesh: &crate::shape::TriangleMesh,
    max_work: usize,
) -> Result<(), super::AerodynamicsError> {
    if selected
        .len()
        .checked_mul(mesh.triangles().len())
        .is_none_or(|work| work > max_work)
    {
        return Err(super::AerodynamicsError::Geometry(
            super::geometry::GeometryError::WorkBudgetExceeded,
        ));
    }
    for triangle in selected {
        if mesh
            .triangles()
            .iter()
            .filter(|face| same_triangle(triangle, &face.map(|index| mesh.vertices()[index])))
            .count()
            != 1
        {
            return Err(super::AerodynamicsError::Pressure(
                PressureError::BaseNotFullyVisible,
            ));
        }
    }
    Ok(())
}

/// Verifies that each selected source triangle appears unchanged exactly once
/// in an exposure result. The caller must prepare `visible` from the complete
/// current Mesh/Colliders geometry, in the direction from the base towards the
/// region being checked, and repeat this check whenever geometry or direction
/// changes. This is a geometric line-of-sight check, not a wake model.
///
/// Partial fragments, absent faces, duplicated selections and multiple matching
/// output faces are rejected. Coincident input faces may already be deduplicated
/// by geometry preparation and cannot be distinguished by this check. Exact vertex equality is deliberate: rounding or clipping may
/// conservatively reject a visible face rather than certify a blocked one.
pub fn verify_full_base_visibility(
    selected: &[[Vec3; 3]],
    visible: &PreparedSurfacePatches,
) -> Result<(), PressureError> {
    for (index, triangle) in selected.iter().enumerate() {
        if selected[..index]
            .iter()
            .any(|other| same_triangle(triangle, other))
            || visible
                .iter()
                .filter(|patch| same_triangle(triangle, &patch.vertices))
                .count()
                != 1
        {
            return Err(PressureError::BaseNotFullyVisible);
        }
    }
    Ok(())
}

/// Physical patch in a stable body-local frame, independent of center of mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressurePatch {
    center: Vec3,
    normal: Vec3,
    area: f64,
}

/// Net world-space force and moment about the current center of mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressureWrench {
    pub force: Vec3,
    pub moment: Vec3,
}

/// Motion and local flow at a patch. Linear velocity refers to the COM.
#[derive(Clone, Copy, Debug)]
pub struct PressureFlow {
    pub atmosphere: AtmosphericState,
    pub sound_speed: f64,
    pub transform: Transform,
    pub center_of_mass: Vec3,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

/// Caller-supplied constant base pressure coefficient and inclusive Mach domain.
///
/// `cp` is excess pressure relative to freestream divided by dynamic pressure;
/// it must be nonpositive. This is not a wake model or a surface classifier:
/// the caller must select actual exposed base patches and provide data suitable
/// for the geometry and flow, including Reynolds number and incidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BasePressure {
    cp_magnitude: f64,
    min_mach: f64,
    max_mach: f64,
}

/// Explicit signed Cp and inclusive local Mach domain, not a predictive model.
/// The caller is responsible for choosing an applicable coefficient and surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfacePressure {
    cp: f64,
    min_mach: f64,
    max_mach: f64,
}

impl SurfacePressure {
    pub fn new(cp: f64, min_mach: f64, max_mach: f64) -> Result<Self, PressureError> {
        if !cp.is_finite()
            || !min_mach.is_finite()
            || min_mach < 0.0
            || !max_mach.is_finite()
            || max_mach < min_mach
        {
            return Err(PressureError::InvalidCoefficient);
        }
        Ok(Self {
            cp,
            min_mach,
            max_mach,
        })
    }
}

#[derive(Clone, Copy)]
enum ExplicitPressure {
    Base(BasePressure),
    Surface(SurfacePressure),
}

impl BasePressure {
    pub fn new(cp: f64, min_mach: f64, max_mach: f64) -> Result<Self, PressureError> {
        if !cp.is_finite()
            || cp > 0.0
            || !min_mach.is_finite()
            || min_mach < 0.0
            || !max_mach.is_finite()
            || max_mach < min_mach
        {
            return Err(PressureError::InvalidCoefficient);
        }
        Ok(Self {
            cp_magnitude: -cp,
            min_mach,
            max_mach,
        })
    }
}

impl PressurePatch {
    pub fn new(center: Vec3, outward_normal: Vec3, area: f64) -> Result<Self, PressureError> {
        if !center.is_finite() || !area.is_finite() || area < 0.0 {
            return Err(PressureError::InvalidPatch);
        }
        let normal = outward_normal
            .try_normalize()
            .ok_or(PressureError::InvalidPatch)?;
        Ok(Self {
            center,
            normal,
            area,
        })
    }

    pub fn center(&self) -> Vec3 {
        self.center
    }
    pub fn normal(&self) -> Vec3 {
        self.normal
    }
    pub fn area(&self) -> f64 {
        self.area
    }

    /// Evaluates a centroid load sample; supply atmosphere sampled at this patch.
    /// COM is body-local; velocities, force and moment are world-space.
    /// No allocation, geometry preparation or body mutation occurs here.
    pub fn evaluate(&self, flow: PressureFlow) -> Result<PressureWrench, PressureError> {
        self.evaluate_inner(flow, None)
    }

    /// Applies a caller-supplied constant base Cp to this explicitly selected
    /// physical patch. Force acts outward for negative Cp. A patch whose outward
    /// normal points into its local body-relative velocity is rejected; callers
    /// must still establish that the patch is a base and is not shielded.
    pub fn evaluate_base(
        &self,
        flow: PressureFlow,
        base: BasePressure,
    ) -> Result<PressureWrench, PressureError> {
        self.evaluate_inner(flow, Some(ExplicitPressure::Base(base)))
    }

    /// Applies a supplied signed Cp without inferring exposure or wake physics.
    /// Pressure acts along the inward normal for positive Cp and outward for negative Cp.
    pub fn evaluate_surface(
        &self,
        flow: PressureFlow,
        pressure: SurfacePressure,
    ) -> Result<PressureWrench, PressureError> {
        self.evaluate_inner(flow, Some(ExplicitPressure::Surface(pressure)))
    }

    fn evaluate_inner(
        &self,
        flow: PressureFlow,
        explicit: Option<ExplicitPressure>,
    ) -> Result<PressureWrench, PressureError> {
        if !flow.center_of_mass.is_finite()
            || !flow.linear_velocity.is_finite()
            || !flow.angular_velocity.is_finite()
        {
            return Err(PressureError::InvalidMotion);
        }
        if !flow.sound_speed.is_finite() || flow.sound_speed <= 0.0 {
            return Err(PressureError::InvalidSoundSpeed);
        }
        let zero = PressureWrench {
            force: Vec3::ZERO,
            moment: Vec3::ZERO,
        };
        if self.area == 0.0 || flow.atmosphere.density() == 0.0 {
            return Ok(zero);
        }
        let overflow = PressureError::ArithmeticOverflow;
        let offset = self
            .center
            .checked_sub(flow.center_of_mass)
            .ok_or(overflow)?;
        let arm = flow.transform.transform_vector(offset);
        let normal = flow.transform.transform_vector(self.normal);
        if !arm.is_finite() || !normal.is_finite() {
            return Err(overflow);
        }
        let relative = flow
            .angular_velocity
            .checked_cross(arm)
            .and_then(|v| flow.linear_velocity.checked_add(v))
            .and_then(|v| v.checked_sub(flow.atmosphere.wind_velocity()))
            .ok_or(overflow)?;
        let speed = relative.checked_length().ok_or(overflow)?;
        if speed == 0.0 {
            return Ok(zero);
        }
        let mach = speed / flow.sound_speed;
        if !mach.is_finite() {
            return Err(overflow);
        }
        let domain = match explicit {
            Some(ExplicitPressure::Base(b)) => Some((b.min_mach, b.max_mach)),
            Some(ExplicitPressure::Surface(p)) => Some((p.min_mach, p.max_mach)),
            None => None,
        };
        if domain.map_or(mach < 5.0, |(min, max)| mach < min || mach > max) {
            return Err(PressureError::UnsupportedMach);
        }
        if let Some(explicit) = explicit {
            let cp = match explicit {
                ExplicitPressure::Base(base) => {
                    let alignment = normal.checked_dot(relative / speed).ok_or(overflow)?;
                    if alignment > 0.0 {
                        return Err(PressureError::InvalidBaseOrientation);
                    }
                    -base.cp_magnitude
                }
                ExplicitPressure::Surface(pressure) => pressure.cp,
            };
            let magnitude = super::hull::scaled_product(
                &[
                    0.5,
                    flow.atmosphere.density(),
                    speed,
                    speed,
                    cp.abs(),
                    self.area,
                ],
                1.0,
            )
            .map_err(|_| overflow)?;
            let force = normal
                .checked_mul_scalar(-magnitude.copysign(cp))
                .ok_or(overflow)?;
            let moment = arm.checked_cross(force).ok_or(overflow)?;
            return Ok(PressureWrench { force, moment });
        }
        // `relative` is the body velocity through the fluid, pointing from the
        // oncoming fluid toward the body.  Windward patches have outward normals
        // aligned with it; the impact pressure acts back along `-normal`.
        let relative_direction = relative / speed;
        let incidence = normal
            .checked_dot(relative_direction)
            .ok_or(overflow)?
            .max(0.0);
        if incidence == 0.0 {
            return Ok(zero);
        }
        // The factors 1/2 in dynamic pressure and 2 in Newtonian Cp cancel.
        // Scale the complete product so an unrepresentable intermediate pressure
        // does not reject a representable force on a small patch.
        let magnitude = super::hull::scaled_product(
            &[
                flow.atmosphere.density(),
                speed,
                speed,
                incidence,
                incidence,
                self.area,
            ],
            1.0,
        )
        .map_err(|_| overflow)?;
        let force = normal.checked_mul_scalar(-magnitude).ok_or(overflow)?;
        let moment = arm.checked_cross(force).ok_or(overflow)?;
        Ok(PressureWrench { force, moment })
    }
}

/// Sums centroid pressure loads over prepared physical mesh patches.
///
/// `flow` supplies one uniform atmospheric state and sound speed for all patches,
/// a body-to-world transform, body-local COM, and world-space COM/angular velocity.
/// The returned wrench is world-space and its moment is about that COM.
///
/// This allocation-free evaluation preserves `PressurePatch`'s Mach policy and
/// force convention. It does not prepare geometry or update exposure/shielding:
/// callers must select patches for the current body-local incoming direction.
/// Rotation-induced local flow can invalidate that single-direction visibility
/// approximation. No world integration or per-owner load distribution occurs.
/// An error from any patch or a nonfinite sum rejects the entire result.
/// Empty sets return zero after validating motion and sound speed.
pub fn evaluate_prepared_mesh(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
) -> Result<PressureWrench, PressureError> {
    evaluate_prepared(patches, flow, None)
}

/// Applies the supplied base Cp to every explicitly selected prepared patch.
/// The set must represent actual exposed base surfaces, not windward impact
/// patches; this function does not determine which surfaces are in a wake.
/// It is independent of whether the patches originated from a mesh or collider.
pub fn evaluate_prepared_base(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    base: BasePressure,
) -> Result<PressureWrench, PressureError> {
    evaluate_prepared(patches, flow, Some(ExplicitPressure::Base(base)))
}

/// Applies an explicitly supplied signed Cp to selected mesh- or collider-derived
/// patches. This does not predict Cp, determine exposure, or model a wake. A
/// single-direction exposure set generally omits suction on other surfaces.
/// No world integration or per-owner load distribution occurs.
pub fn evaluate_prepared_surface(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    pressure: SurfacePressure,
) -> Result<PressureWrench, PressureError> {
    evaluate_prepared(patches, flow, Some(ExplicitPressure::Surface(pressure)))
}

fn evaluate_prepared(
    patches: &PreparedSurfacePatches,
    flow: PressureFlow,
    explicit: Option<ExplicitPressure>,
) -> Result<PressureWrench, PressureError> {
    // Validate even an empty set, using the same rules as patch evaluation.
    let mut total = PressurePatch::new(Vec3::ZERO, Vec3::X, 0.0)?.evaluate(flow)?;
    for patch in patches.iter() {
        let load = PressurePatch::new(patch.center, patch.normal, patch.area)?
            .evaluate_inner(flow, explicit)?;
        total.force = total
            .force
            .checked_add(load.force)
            .ok_or(PressureError::ArithmeticOverflow)?;
        total.moment = total
            .moment
            .checked_add(load.moment)
            .ok_or(PressureError::ArithmeticOverflow)?;
    }
    Ok(total)
}

#[cfg(test)]
#[path = "pressure_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "mesh_pressure_tests.rs"]
mod mesh_tests;
