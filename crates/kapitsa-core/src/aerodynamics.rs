//! Aerodynamic force models.

use std::fmt;

use kapitsa_math::Vec3;

use crate::transform::Transform;

pub use crate::world::{AerodynamicSurface, AerodynamicTarget, AerodynamicsTask};

/// Local atmospheric conditions used by an aerodynamic force model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AtmosphericState {
    density: f64,
    wind_velocity: Vec3,
}

impl AtmosphericState {
    /// Creates atmospheric conditions with density in kg/m³ and wind velocity in m/s.
    pub fn new(density: f64, wind_velocity: Vec3) -> Result<Self, AerodynamicsError> {
        if !density.is_finite() || density < 0.0 {
            return Err(AerodynamicsError::InvalidDensity);
        }
        if !wind_velocity.is_finite() {
            return Err(AerodynamicsError::NonFiniteWindVelocity);
        }
        Ok(Self {
            density,
            wind_velocity,
        })
    }

    pub fn density(self) -> f64 {
        self.density
    }

    pub fn wind_velocity(self) -> Vec3 {
        self.wind_velocity
    }
}

/// Parameters for a quadratic drag model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AerodynamicProperties {
    reference_area: f64,
    drag_coefficient: f64,
    local_application_offset: Vec3,
    lift_coefficient: f64,
    local_lift_axis: Option<Vec3>,
}

impl AerodynamicProperties {
    /// Creates drag parameters. For body surfaces the offset is relative to the
    /// body center of mass; for part surfaces it is relative to the part origin.
    /// Both the offset and lift axis are expressed in the target's local frame.
    pub fn new(
        reference_area: f64,
        drag_coefficient: f64,
        local_application_offset: Vec3,
    ) -> Result<Self, AerodynamicsError> {
        if !reference_area.is_finite() || reference_area < 0.0 {
            return Err(AerodynamicsError::InvalidReferenceArea);
        }
        if !drag_coefficient.is_finite() || drag_coefficient < 0.0 {
            return Err(AerodynamicsError::InvalidDragCoefficient);
        }
        if !local_application_offset.is_finite() {
            return Err(AerodynamicsError::NonFiniteApplicationOffset);
        }
        Ok(Self {
            reference_area,
            drag_coefficient,
            local_application_offset,
            lift_coefficient: 0.0,
            local_lift_axis: None,
        })
    }

    pub fn reference_area(self) -> f64 {
        self.reference_area
    }

    pub fn drag_coefficient(self) -> f64 {
        self.drag_coefficient
    }

    pub fn local_application_offset(self) -> Vec3 {
        self.local_application_offset
    }

    pub(crate) fn with_local_application_offset(
        mut self,
        local_application_offset: Vec3,
    ) -> Result<Self, AerodynamicsError> {
        if !local_application_offset.is_finite() {
            return Err(AerodynamicsError::NonFiniteApplicationOffset);
        }
        self.local_application_offset = local_application_offset;
        Ok(self)
    }

    pub fn lift_coefficient(self) -> f64 {
        self.lift_coefficient
    }

    pub fn local_lift_axis(self) -> Option<Vec3> {
        self.local_lift_axis
    }

    /// Adds an optional lift model using a body-local surface normal.
    ///
    /// The coefficient may be negative to reverse the lift direction. The axis
    /// is normalized when the properties are configured.
    pub fn with_lift(
        mut self,
        lift_coefficient: f64,
        local_lift_axis: Vec3,
    ) -> Result<Self, AerodynamicsError> {
        if !lift_coefficient.is_finite() {
            return Err(AerodynamicsError::InvalidLiftCoefficient);
        }
        let local_lift_axis = local_lift_axis
            .try_normalize()
            .ok_or(AerodynamicsError::InvalidLiftAxis)?;
        self.lift_coefficient = lift_coefficient;
        self.local_lift_axis = Some(local_lift_axis);
        Ok(self)
    }

    /// Evaluates aerodynamic forces for a rigid body's current motion.
    ///
    /// `linear_velocity` is the center-of-mass velocity and `angular_velocity` is
    /// expressed in world coordinates. The returned force is expressed in world
    /// coordinates and acts at the configured point relative to the center of mass.
    pub fn drag_force(
        self,
        atmosphere: AtmosphericState,
        transform: Transform,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    ) -> Result<AerodynamicForce, AerodynamicsError> {
        let (drag, lift) =
            self.force_components(atmosphere, transform, linear_velocity, angular_velocity)?;
        let force = drag
            .checked_add(lift)
            .ok_or(AerodynamicsError::ArithmeticOverflow)?;
        Ok(AerodynamicForce {
            force,
            local_application_offset: self.local_application_offset,
        })
    }

    pub(crate) fn force_components(
        self,
        atmosphere: AtmosphericState,
        transform: Transform,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    ) -> Result<(Vec3, Vec3), AerodynamicsError> {
        if !linear_velocity.is_finite() {
            return Err(AerodynamicsError::NonFiniteLinearVelocity);
        }
        if !angular_velocity.is_finite() {
            return Err(AerodynamicsError::NonFiniteAngularVelocity);
        }

        if self.reference_area == 0.0
            || (self.drag_coefficient == 0.0
                && (self.lift_coefficient == 0.0 || self.local_lift_axis.is_none()))
        {
            return Ok((Vec3::ZERO, Vec3::ZERO));
        }

        let world_offset = transform.transform_vector(self.local_application_offset);
        if !world_offset.is_finite() {
            return Err(AerodynamicsError::ArithmeticOverflow);
        }
        let rotational_velocity = angular_velocity
            .checked_cross(world_offset)
            .ok_or(AerodynamicsError::ArithmeticOverflow)?;
        let body_point_velocity = linear_velocity
            .checked_add(rotational_velocity)
            .ok_or(AerodynamicsError::ArithmeticOverflow)?;
        let relative_velocity = body_point_velocity
            .checked_sub(atmosphere.wind_velocity)
            .ok_or(AerodynamicsError::ArithmeticOverflow)?;
        let speed = relative_velocity
            .checked_length()
            .ok_or(AerodynamicsError::ArithmeticOverflow)?;

        if speed == 0.0 || atmosphere.density == 0.0 {
            return Ok((Vec3::ZERO, Vec3::ZERO));
        }

        let speed_squared = speed * speed;
        if !speed_squared.is_finite() {
            return Err(AerodynamicsError::ArithmeticOverflow);
        }
        let dynamic_pressure = 0.5 * atmosphere.density * speed_squared;
        let drag_magnitude = dynamic_pressure * self.reference_area * self.drag_coefficient;
        if !drag_magnitude.is_finite() {
            return Err(AerodynamicsError::ArithmeticOverflow);
        }
        let flow_direction = relative_velocity
            .checked_mul_scalar(1.0 / speed)
            .ok_or(AerodynamicsError::ArithmeticOverflow)?;
        let drag_force = flow_direction
            .checked_mul_scalar(-drag_magnitude)
            .ok_or(AerodynamicsError::ArithmeticOverflow)?;
        let lift_force = match self.local_lift_axis {
            Some(local_axis) if self.lift_coefficient != 0.0 => {
                let world_axis = transform.transform_vector(local_axis);
                let projection = world_axis
                    .checked_dot(flow_direction)
                    .ok_or(AerodynamicsError::ArithmeticOverflow)?;
                let lift_factor = (1.0 - projection * projection).max(0.0).sqrt();
                let lift_direction = world_axis
                    .checked_sub(
                        flow_direction
                            .checked_mul_scalar(projection)
                            .ok_or(AerodynamicsError::ArithmeticOverflow)?,
                    )
                    .ok_or(AerodynamicsError::ArithmeticOverflow)?;
                let Some(lift_direction) = lift_direction.try_normalize() else {
                    return Ok((drag_force, Vec3::ZERO));
                };
                let lift_magnitude =
                    dynamic_pressure * self.reference_area * self.lift_coefficient * lift_factor;
                if !lift_magnitude.is_finite() {
                    return Err(AerodynamicsError::ArithmeticOverflow);
                }
                lift_direction
                    .checked_mul_scalar(lift_magnitude)
                    .ok_or(AerodynamicsError::ArithmeticOverflow)?
            }
            _ => Vec3::ZERO,
        };
        Ok((drag_force, lift_force))
    }
}

/// A force calculated by an aerodynamic model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AerodynamicForce {
    force: Vec3,
    local_application_offset: Vec3,
}

impl AerodynamicForce {
    pub fn force(self) -> Vec3 {
        self.force
    }

    pub fn local_application_offset(self) -> Vec3 {
        self.local_application_offset
    }
}

/// Invalid aerodynamic parameters or an unrepresentable force calculation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AerodynamicsError {
    InvalidDensity,
    InvalidReferenceArea,
    InvalidDragCoefficient,
    InvalidLiftCoefficient,
    InvalidLiftAxis,
    NonFiniteWindVelocity,
    NonFiniteApplicationOffset,
    NonFiniteLinearVelocity,
    NonFiniteAngularVelocity,
    ArithmeticOverflow,
}

impl fmt::Display for AerodynamicsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidDensity => "air density must be finite and non-negative",
            Self::InvalidReferenceArea => "reference area must be finite and non-negative",
            Self::InvalidDragCoefficient => "drag coefficient must be finite and non-negative",
            Self::InvalidLiftCoefficient => "lift coefficient must be finite",
            Self::InvalidLiftAxis => "lift axis must be finite and non-zero",
            Self::NonFiniteWindVelocity => "wind velocity must be finite",
            Self::NonFiniteApplicationOffset => "application offset must be finite",
            Self::NonFiniteLinearVelocity => "linear velocity must be finite",
            Self::NonFiniteAngularVelocity => "angular velocity must be finite",
            Self::ArithmeticOverflow => "aerodynamic calculation exceeds the finite f64 range",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for AerodynamicsError {}

#[cfg(test)]
mod tests {
    use kapitsa_math::Vec3;

    use super::{AerodynamicProperties, AerodynamicsError, AtmosphericState};
    use crate::transform::Transform;

    #[test]
    fn disabled_surface_avoids_overflow_and_components_match_total() {
        let atmosphere = AtmosphericState::new(1.0, Vec3::ZERO).unwrap();
        for properties in [
            AerodynamicProperties::new(0.0, 1.0, Vec3::ZERO).unwrap(),
            AerodynamicProperties::new(1.0, 0.0, Vec3::ZERO)
                .unwrap()
                .with_lift(0.0, Vec3::Y)
                .unwrap(),
        ] {
            assert_eq!(
                properties
                    .drag_force(
                        atmosphere,
                        Transform::identity(),
                        Vec3::X * f64::MAX,
                        Vec3::ZERO
                    )
                    .unwrap()
                    .force(),
                Vec3::ZERO
            );
        }
        let properties = AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO)
            .unwrap()
            .with_lift(0.7, Vec3::Y)
            .unwrap();
        let velocity = Vec3::new(10.0, 1.0, 0.0);
        let (drag, lift) = properties
            .force_components(atmosphere, Transform::identity(), velocity, Vec3::ZERO)
            .unwrap();
        assert_eq!(
            properties
                .drag_force(atmosphere, Transform::identity(), velocity, Vec3::ZERO)
                .unwrap()
                .force(),
            drag + lift
        );
    }

    #[test]
    fn drag_points_against_relative_flow() {
        let atmosphere = AtmosphericState::new(1.0, Vec3::ZERO).unwrap();
        let properties = AerodynamicProperties::new(2.0, 0.5, Vec3::ZERO).unwrap();
        let drag = properties
            .drag_force(
                atmosphere,
                Transform::identity(),
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::ZERO,
            )
            .unwrap();

        assert_eq!(drag.force(), Vec3::new(-50.0, 0.0, 0.0));
    }

    #[test]
    fn rotational_motion_changes_point_velocity() {
        let atmosphere = AtmosphericState::new(1.0, Vec3::ZERO).unwrap();
        let properties = AerodynamicProperties::new(1.0, 1.0, Vec3::X).unwrap();
        let drag = properties
            .drag_force(atmosphere, Transform::identity(), Vec3::ZERO, Vec3::Z)
            .unwrap();

        assert_eq!(drag.force(), Vec3::new(0.0, -0.5, 0.0));
    }

    #[test]
    fn zero_flow_produces_zero_force() {
        let atmosphere = AtmosphericState::new(1.2, Vec3::new(4.0, 0.0, 0.0)).unwrap();
        let properties = AerodynamicProperties::new(3.0, 0.8, Vec3::ZERO).unwrap();
        let drag = properties
            .drag_force(
                atmosphere,
                Transform::identity(),
                Vec3::new(4.0, 0.0, 0.0),
                Vec3::ZERO,
            )
            .unwrap();

        assert_eq!(drag.force(), Vec3::ZERO);
    }

    #[test]
    fn lift_acts_along_the_projected_surface_axis() {
        let atmosphere = AtmosphericState::new(1.0, Vec3::ZERO).unwrap();
        let properties = AerodynamicProperties::new(1.0, 0.0, Vec3::ZERO)
            .unwrap()
            .with_lift(1.0, Vec3::Y)
            .unwrap();
        let force = properties
            .drag_force(
                atmosphere,
                Transform::identity(),
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::ZERO,
            )
            .unwrap();

        assert_eq!(force.force(), Vec3::new(0.0, 50.0, 0.0));
    }

    #[test]
    fn lift_vanishes_continuously_when_axis_aligns_with_flow() {
        let atmosphere = AtmosphericState::new(1.0, Vec3::ZERO).unwrap();
        let properties = AerodynamicProperties::new(1.0, 0.0, Vec3::ZERO)
            .unwrap()
            .with_lift(1.0, Vec3::new(1.0, 1.0e-6, 0.0))
            .unwrap();
        let force = properties
            .drag_force(
                atmosphere,
                Transform::identity(),
                Vec3::new(10.0, 0.0, 0.0),
                Vec3::ZERO,
            )
            .unwrap();

        assert!(force.force().length() < 1.0e-4);
    }

    #[test]
    fn constructors_reject_invalid_values() {
        assert_eq!(
            AtmosphericState::new(-1.0, Vec3::ZERO),
            Err(AerodynamicsError::InvalidDensity)
        );
        assert_eq!(
            AerodynamicProperties::new(1.0, f64::NAN, Vec3::ZERO),
            Err(AerodynamicsError::InvalidDragCoefficient)
        );
        assert_eq!(
            AerodynamicProperties::new(1.0, 1.0, Vec3::new(f64::INFINITY, 0.0, 0.0)),
            Err(AerodynamicsError::NonFiniteApplicationOffset)
        );
        assert_eq!(
            AerodynamicProperties::new(1.0, 0.0, Vec3::ZERO)
                .unwrap()
                .with_lift(1.0, Vec3::ZERO),
            Err(AerodynamicsError::InvalidLiftAxis)
        );
    }
}
