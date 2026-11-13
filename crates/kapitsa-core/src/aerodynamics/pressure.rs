//! Newtonian hypersonic impact pressure on caller-selected physical surface patches.
//!
//! Source: <https://en.wikipedia.org/wiki/Pressure_coefficient>, Cp = 2 sin²(theta).
//! Leeward excess pressure is zero. Mach >= 5 is an explicit implementation
//! policy, not a universal accuracy guarantee. No suction, friction, wake,
//! circulation, transonic correction or shock interaction is resolved.
//! Callers must supply exposed original-surface normals and areas. In particular,
//! voxel staircase normals are not a consistent replacement for smooth normals.
//! Overlap removal and upstream shielding are the caller's responsibility.

use super::AtmosphericState;
use crate::transform::Transform;
use kapitsa_math::Vec3;
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PressureError {
    InvalidPatch,
    InvalidMotion,
    InvalidSoundSpeed,
    UnsupportedMach,
    ArithmeticOverflow,
}
impl fmt::Display for PressureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Newtonian pressure evaluation failed: {self:?}")
    }
}
impl std::error::Error for PressureError {}

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
        if mach < 5.0 {
            return Err(PressureError::UnsupportedMach);
        }
        let incidence = normal
            .checked_dot(relative / speed)
            .ok_or(overflow)?
            .max(0.0);
        if incidence == 0.0 {
            return Ok(zero);
        }
        let q = (0.5 * flow.atmosphere.density() * speed) * speed;
        let magnitude = q * (2.0 * incidence * incidence) * self.area;
        if !q.is_finite() || !magnitude.is_finite() {
            return Err(overflow);
        }
        let force = normal.checked_mul_scalar(-magnitude).ok_or(overflow)?;
        let moment = arm.checked_cross(force).ok_or(overflow)?;
        Ok(PressureWrench { force, moment })
    }
}

#[cfg(test)]
#[path = "pressure_tests.rs"]
mod tests;
