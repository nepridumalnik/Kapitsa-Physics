use crate::{
    mass::{MassError, MassProperties},
    transform::Transform,
};

use super::ClusterError;

fn map_mass_error(error: MassError) -> ClusterError {
    match error {
        MassError::ArithmeticOverflow => ClusterError::ArithmeticOverflow,
        other => ClusterError::Mass(other),
    }
}

/// Combines part properties in cluster-local coordinates.
pub(crate) fn aggregate_mass<'a>(
    parts: impl IntoIterator<Item = (&'a MassProperties, &'a Transform)>,
) -> Result<MassProperties, ClusterError> {
    let mut parts = parts.into_iter();
    let (first, transform) = parts
        .next()
        .ok_or(ClusterError::Mass(MassError::EmptyComposition))?;
    let mut result = MassProperties::combine(&[(*first, *transform)]).map_err(map_mass_error)?;
    for (part, transform) in parts {
        result.push(part, transform).map_err(map_mass_error)?;
    }
    if !result.mass().is_finite()
        || result.mass() <= 0.0
        || !result.center_of_mass().is_finite()
        || !result.inertia().is_finite()
    {
        return Err(ClusterError::ArithmeticOverflow);
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_math::{Mat3, Quat, Vec3};

    fn close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() <= 1e-12 * expected.abs().max(1.0),
            "expected {expected}, got {actual}"
        );
    }

    fn properties_close(actual: &MassProperties, expected: &MassProperties) {
        close(actual.mass(), expected.mass());
        let a = actual.center_of_mass();
        let b = expected.center_of_mass();
        for (x, y) in [(a.x, b.x), (a.y, b.y), (a.z, b.z)] {
            close(x, y);
        }
        for (x, y) in actual
            .inertia()
            .to_cols_array()
            .into_iter()
            .zip(expected.inertia().to_cols_array())
        {
            close(x, y);
        }
    }

    #[test]
    fn empty_and_error_mapping() {
        assert_eq!(
            aggregate_mass([]),
            Err(ClusterError::Mass(MassError::EmptyComposition))
        );
        assert_eq!(
            map_mass_error(MassError::ArithmeticOverflow),
            ClusterError::ArithmeticOverflow
        );
        assert_eq!(
            map_mass_error(MassError::NonPhysicalInertia),
            ClusterError::Mass(MassError::NonPhysicalInertia)
        );
    }

    #[test]
    fn translates_single_and_multiple_parts() -> Result<(), Box<dyn std::error::Error>> {
        let a = MassProperties::new(1.0, Vec3::X, Mat3::IDENTITY)?;
        let b = MassProperties::new(3.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let ta = Transform::new(Vec3::X, Quat::IDENTITY)?;
        let tb = Transform::new(6.0 * Vec3::X, Quat::IDENTITY)?;
        let single = aggregate_mass([(&a, &ta)])?;
        close(single.center_of_mass().x, 2.0);
        let combined = aggregate_mass([(&a, &ta), (&b, &tb)])?;
        close(combined.mass(), 4.0);
        close(combined.center_of_mass().x, 5.0);
        let values = combined.inertia().to_cols_array();
        close(values[0], 2.0);
        close(values[4], 14.0);
        close(values[8], 14.0);
        Ok(())
    }

    #[test]
    fn rotates_center_and_tensor() -> Result<(), Box<dyn std::error::Error>> {
        let part =
            MassProperties::new(1.0, Vec3::X, Mat3::from_diagonal(Vec3::new(1.0, 2.0, 3.0)))?;
        let rotation = Quat::from_axis_angle(Vec3::Z, std::f64::consts::FRAC_PI_2)
            .ok_or("invalid rotation")?;
        let transform = Transform::new(Vec3::new(2.0, 3.0, 0.0), rotation)?;
        let actual = aggregate_mass([(&part, &transform)])?;
        close(actual.center_of_mass().x, 2.0);
        close(actual.center_of_mass().y, 4.0);
        let inertia = actual.inertia().to_cols_array();
        close(inertia[0], 2.0);
        close(inertia[4], 1.0);
        close(inertia[8], 3.0);
        close(inertia[1], 0.0);
        Ok(())
    }

    #[test]
    fn order_is_consistent_with_roundoff() -> Result<(), Box<dyn std::error::Error>> {
        let a = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let b = MassProperties::new(2.0, Vec3::X, Mat3::IDENTITY)?;
        let c = MassProperties::new(3.0, Vec3::Y, Mat3::IDENTITY)?;
        let ta = Transform::identity();
        let tb = Transform::new(Vec3::new(2.0, -1.0, 0.0), Quat::IDENTITY)?;
        let tc = Transform::new(Vec3::new(-1.0, 0.0, 2.0), Quat::IDENTITY)?;
        let forward = aggregate_mass([(&a, &ta), (&b, &tb), (&c, &tc)])?;
        let reverse = aggregate_mass([(&c, &tc), (&b, &tb), (&a, &ta)])?;
        properties_close(&forward, &reverse);
        Ok(())
    }

    #[test]
    fn detects_overflow_in_first_and_later_parts() -> Result<(), Box<dyn std::error::Error>> {
        let large = MassProperties::new(f64::MAX, Vec3::X, Mat3::IDENTITY)?;
        let identity = Transform::identity();
        let huge_translation = Transform::new(Vec3::new(f64::MAX, 0.0, 0.0), Quat::IDENTITY)?;
        let far = MassProperties::new(1.0, Vec3::new(f64::MAX, 0.0, 0.0), Mat3::IDENTITY)?;
        assert_eq!(
            aggregate_mass([(&far, &huge_translation)]),
            Err(ClusterError::ArithmeticOverflow)
        );
        assert_eq!(
            aggregate_mass([(&large, &identity), (&large, &identity)]),
            Err(ClusterError::ArithmeticOverflow)
        );
        Ok(())
    }
}
