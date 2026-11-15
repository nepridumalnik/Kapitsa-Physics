//! Small-incidence normal loads for a slender axisymmetric body.
//!
//! This is the incompressible, steady, uniform-flow body term of Barrowman,
//! equation 3-59: <https://ntrs.nasa.gov/api/citations/20010047838/downloads/20010047838.pdf>.
//! It is not a stall, transonic, viscous, wing-interference or general body model.

use kapitsa_math::{Quat, Vec3};

use super::{
    geometry::{GeometrySection, PreparedGeometry},
    hull::{HullError, HullSection, scaled_product},
    wave::area_slopes_from_sections,
};
use crate::transform::Transform;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct PotentialCell {
    slope: f64,
    width: f64,
    max_incidence: f64,
}

impl PotentialCell {
    pub(super) fn force(
        self,
        density: f64,
        axial_speed: f64,
        crossflow: Vec3,
        cross_speed: f64,
    ) -> Result<Vec3, HullError> {
        if cross_speed == 0.0 {
            return Ok(Vec3::ZERO);
        }
        if axial_speed <= 0.0 || cross_speed.atan2(axial_speed) > self.max_incidence {
            return Err(HullError::UnsupportedSlenderIncidence);
        }
        // Positive body axis points toward the nose; the cited source uses a
        // downstream coordinate. Its area derivative therefore changes sign.
        // The aerodynamic load opposes crossflow on an expanding nose section.
        let component = |value: f64| {
            scaled_product(
                &[
                    density,
                    axial_speed,
                    self.slope.abs(),
                    self.width,
                    value.abs(),
                ],
                1.0,
            )
            .map(|magnitude| magnitude.copysign(value) * self.slope.signum())
        };
        Ok(Vec3::new(
            component(crossflow.x)?,
            component(crossflow.y)?,
            component(crossflow.z)?,
        ))
    }
}

/// Creates body-local normal-load sections from one axis of a prepared voxel union.
///
/// `axis` is the body-local positive nose direction (0 = X, 1 = Y, 2 = Z).
/// `centerline` is a point in body-local metres on the body's longitudinal
/// centerline. `smoothing_radius` is the radius in cells of the area filter;
/// `max_incidence` is the caller-selected small-angle limit in radians, strictly
/// between zero and pi/2. Geometry and filter convergence must be established
/// by the caller. The model assumes a slender axisymmetric body, uniform steady
/// flow and no angular velocity, except pure axial spin in aligned flow;
/// local Mach must not exceed 0.3. It does not
/// determine viscous resistance or wave drag. Source and return sections use
/// stable body-local coordinates, independent of center of mass.
///
/// Errors are returned for invalid inputs, allocation or arithmetic failure,
/// and geometry work-budget exhaustion. The returned sections can be supplied
/// to `Aerodynamics::Hull` or converted to the appropriate part-local frame.
pub fn hull_sections_from_voxels(
    geometry: &PreparedGeometry,
    axis: usize,
    centerline: Vec3,
    smoothing_radius: usize,
    max_incidence: f64,
) -> Result<Vec<HullSection>, HullError> {
    let sections = geometry
        .sections()
        .get(axis)
        .ok_or(HullError::InvalidParameters)?;
    sections_from_areas(
        sections,
        geometry.origin(),
        geometry.cell_size(),
        axis,
        centerline,
        smoothing_radius,
        max_incidence,
    )
}

#[allow(clippy::too_many_arguments)]
fn sections_from_areas(
    sections: &[GeometrySection],
    origin: Vec3,
    cell_size: f64,
    axis: usize,
    centerline: Vec3,
    smoothing_radius: usize,
    max_incidence: f64,
) -> Result<Vec<HullSection>, HullError> {
    if !origin.is_finite()
        || !centerline.is_finite()
        || !max_incidence.is_finite()
        || max_incidence <= 0.0
        || max_incidence >= std::f64::consts::FRAC_PI_2
    {
        return Err(HullError::InvalidParameters);
    }
    let direction = match axis {
        0 => Vec3::X,
        1 => Vec3::Y,
        2 => Vec3::Z,
        _ => return Err(HullError::InvalidParameters),
    };
    let slopes = area_slopes_from_sections(sections, cell_size, smoothing_radius)
        .map_err(HullError::Wave)?;
    let padding = smoothing_radius
        .checked_add(2)
        .ok_or(HullError::ArithmeticOverflow)?;
    let mut result = Vec::new();
    result
        .try_reserve_exact(slopes.len())
        .map_err(|_| HullError::AllocationFailed)?;
    for (index, slope) in slopes.into_iter().enumerate() {
        if slope == 0.0 {
            continue;
        }
        let offset = (index as f64 - padding as f64 + 0.5) * cell_size;
        let coordinate = match axis {
            0 => origin.x,
            1 => origin.y,
            _ => origin.z,
        } + offset;
        if !coordinate.is_finite() {
            return Err(HullError::ArithmeticOverflow);
        }
        let mut point = centerline;
        match axis {
            0 => point.x = coordinate,
            1 => point.y = coordinate,
            _ => point.z = coordinate,
        }
        let transform =
            Transform::new(point, Quat::IDENTITY).map_err(|_| HullError::ArithmeticOverflow)?;
        let section = HullSection::new(
            transform, direction, cell_size, 0.0, 0.0, 0.0, 0.0, 0.0, None,
        )?;
        result.push(section.with_potential(PotentialCell {
            slope,
            width: cell_size,
            max_incidence,
        }));
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aerodynamics::{AtmosphericState, pressure::PressureFlow};

    fn sample_sections(count: usize) -> Vec<GeometrySection> {
        (0..count)
            .map(|i| {
                let x = (i as f64 + 0.5) / count as f64;
                GeometrySection {
                    area: x * (1.0 - x),
                    centroid: None,
                }
            })
            .collect()
    }

    fn flow(velocity: Vec3) -> Result<PressureFlow, Box<dyn std::error::Error>> {
        Ok(PressureFlow {
            atmosphere: AtmosphericState::new(1.0, Vec3::ZERO)?,
            sound_speed: 340.0,
            transform: Transform::identity(),
            center_of_mass: Vec3::ZERO,
            linear_velocity: velocity,
            angular_velocity: Vec3::ZERO,
        })
    }

    #[test]
    fn closed_body_normal_load_and_moment_converge() -> Result<(), Box<dyn std::error::Error>> {
        let mut errors = Vec::new();
        for count in [16, 32, 64] {
            let input = sample_sections(count);
            let sections = sections_from_areas(
                &input,
                Vec3::ZERO,
                1.0 / count as f64,
                0,
                Vec3::ZERO,
                0,
                0.2,
            )?;
            let mut force = Vec3::ZERO;
            let mut moment = Vec3::ZERO;
            for section in sections {
                let wrench = section.evaluate(flow(Vec3::new(10.0, 0.0, -1.0))?)?;
                force += wrench.force;
                moment += wrench.moment;
            }
            assert!(force.length() < 1e-11, "{force:?}");
            assert!(moment.y < 0.0, "{moment:?}");
            errors.push((moment.y + 10.0 / 6.0).abs());
        }
        assert!(errors[2] < errors[1] && errors[1] < errors[0], "{errors:?}");
        assert!(errors[2] < 0.001, "{errors:?}");
        Ok(())
    }

    #[test]
    fn potential_cell_is_covariant_under_rigid_transform() -> Result<(), Box<dyn std::error::Error>>
    {
        let section = HullSection::new(
            Transform::new(Vec3::X * 2.0, Quat::IDENTITY)?,
            Vec3::X,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            None,
        )?
        .with_potential(PotentialCell {
            slope: -0.5,
            width: 1.0,
            max_incidence: 0.2,
        });
        let initial = flow(Vec3::new(10.0, 0.0, -1.0))?;
        let reference = section.evaluate(initial)?;
        let rotation = Quat::from_axis_angle(Vec3::Y, 0.7).ok_or("rotation")?;
        let transform = Transform::new(Vec3::new(100.0, 200.0, -300.0), rotation)?;
        let rotated_velocity = rotation
            .rotate_vector(initial.linear_velocity)
            .ok_or("velocity")?;
        let result = section.evaluate(PressureFlow {
            transform,
            linear_velocity: rotated_velocity,
            ..initial
        })?;
        let expected_force = rotation.rotate_vector(reference.force).ok_or("force")?;
        let expected_moment = rotation.rotate_vector(reference.moment).ok_or("moment")?;
        assert!((result.force - expected_force).length() < 1e-12);
        assert!((result.moment - expected_moment).length() < 1e-12);
        Ok(())
    }

    #[test]
    fn potential_cell_preserves_representable_extreme_loads()
    -> Result<(), Box<dyn std::error::Error>> {
        for (slope, axial, cross, sound) in [
            (1e-170, 1e150, -1e140, 5e150),
            (1e170, 1e-150, -1e-151, 5e-150),
        ] {
            let section = HullSection::new(
                Transform::identity(),
                Vec3::X,
                0.1,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                None,
            )?
            .with_potential(PotentialCell {
                slope,
                width: 0.1,
                max_incidence: 0.2,
            });
            let mut state = flow(Vec3::new(axial, 0.0, cross))?;
            state.sound_speed = sound;
            let force = section.evaluate(state)?.force;
            let expected = scaled_product(&[axial, slope, 0.1, cross.abs()], 1.0)?;
            assert!(expected > 0.0);
            assert!((force.z / -expected - 1.0).abs() < 1e-12);
        }
        Ok(())
    }

    #[test]
    fn normal_load_respects_flow_and_rotation_domains() -> Result<(), Box<dyn std::error::Error>> {
        let section = sections_from_areas(
            &sample_sections(16),
            Vec3::ZERO,
            1.0 / 16.0,
            0,
            Vec3::ZERO,
            0,
            0.2,
        )?
        .into_iter()
        .next()
        .ok_or("section")?;
        assert!(matches!(
            section.evaluate(flow(Vec3::new(-10.0, 0.0, -1.0))?),
            Err(HullError::UnsupportedSlenderIncidence)
        ));
        assert!(matches!(
            section.evaluate(flow(Vec3::new(10.0, 0.0, -5.0))?),
            Err(HullError::UnsupportedSlenderIncidence)
        ));
        assert!(matches!(
            section.evaluate(flow(Vec3::new(200.0, 0.0, -1.0))?),
            Err(HullError::UnsupportedMach)
        ));
        let mut rotating = flow(Vec3::X * 10.0)?;
        rotating.angular_velocity = Vec3::Z * 0.01;
        assert!(matches!(
            section.evaluate(rotating),
            Err(HullError::UnsupportedSlenderRotation)
        ));
        rotating.angular_velocity = Vec3::X;
        assert_eq!(section.evaluate(rotating)?.force, Vec3::ZERO);
        rotating.linear_velocity = Vec3::new(10.0, 0.0, 1.0);
        assert!(matches!(
            section.evaluate(rotating),
            Err(HullError::UnsupportedSlenderRotation)
        ));
        rotating.linear_velocity = Vec3::ZERO;
        assert_eq!(section.evaluate(rotating)?.force, Vec3::ZERO);
        assert_eq!(section.evaluate(flow(Vec3::X * 200.0)?)?.force, Vec3::ZERO);
        Ok(())
    }
}
