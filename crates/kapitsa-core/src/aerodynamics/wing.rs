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

/// One explicitly configured wing. Transform maps wing coordinates to the
/// stable body-local frame; its origin is the force application point.
#[derive(Clone, Copy, Debug)]
pub struct WingSurface {
    model: FiniteWing,
    local_transform: crate::transform::Transform,
    planform_area: f64,
}

impl WingSurface {
    pub fn local_transform(self) -> crate::transform::Transform {
        self.local_transform
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
            local_transform,
            planform_area,
        })
    }

    /// Returns world force and COM moment using atmosphere supplied at the wing.
    /// No intrinsic airfoil pitching moment or mesh load exclusion is implied.
    pub fn evaluate(
        self,
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
            .inverse()
            .map_err(|_| overflow)?
            .transform_vector(velocity);
        let wing_velocity = self
            .local_transform
            .inverse()
            .map_err(|_| overflow)?
            .transform_vector(body_velocity);
        let local_force = self.model.force(
            wing_velocity,
            flow.atmosphere.density(),
            flow.sound_speed,
            self.planform_area,
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
        // Permit roundoff from rigid frame transforms, not physical sideslip.
        let planar_scale = relative_velocity.x.abs().max(relative_velocity.z.abs());
        if relative_velocity.y.abs() > 64.0 * f64::EPSILON * planar_scale
            || relative_velocity.x <= 0.0
        {
            return Err(WingError::UnsupportedFlow);
        }
        let relative_velocity = Vec3::new(relative_velocity.x, 0.0, relative_velocity.z);
        let speed = relative_velocity
            .checked_length()
            .ok_or(WingError::ArithmeticOverflow)?;
        // Nose-up attitude at horizontal flight gives negative local Z velocity.
        let alpha = (-relative_velocity.z).atan2(relative_velocity.x);
        let coefficients = self.coefficients(alpha, speed / sound_speed)?;
        let direction = relative_velocity / speed;
        let lift_direction = Vec3::new(-direction.z, 0.0, direction.x);
        let scale = (0.5 * density * speed) * speed * planform_area;
        let force = (lift_direction * coefficients.lift - direction * coefficients.drag) * scale;
        if !scale.is_finite() || !force.is_finite() {
            return Err(WingError::ArithmeticOverflow);
        }
        Ok(force)
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
}
