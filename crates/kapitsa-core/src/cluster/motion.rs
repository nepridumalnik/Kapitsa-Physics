use kapitsa_math::{CholeskyError, Mat3, Vec3};

use crate::{mass::MassProperties, transform::Transform};

use super::ClusterError;

/// Evaluates the rigid velocity field at another world-space center.
pub(crate) fn velocity_field(
    parent_center: Vec3,
    parent_velocity: Vec3,
    omega: Vec3,
    new_center: Vec3,
) -> Result<(Vec3, Vec3), ClusterError> {
    if !parent_center.is_finite()
        || !parent_velocity.is_finite()
        || !omega.is_finite()
        || !new_center.is_finite()
    {
        return Err(ClusterError::InvalidMotion);
    }
    let velocity = new_center
        .checked_sub(parent_center)
        .and_then(|offset| omega.checked_cross(offset))
        .and_then(|relative| parent_velocity.checked_add(relative))
        .ok_or(ClusterError::ArithmeticOverflow)?;
    Ok((velocity, omega))
}

fn world_properties(mass: MassProperties, pose: Transform) -> Result<(Vec3, Mat3), ClusterError> {
    let rotation = pose
        .rotation()
        .to_mat3()
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let center = rotation
        .checked_mul_vector(mass.center_of_mass())
        .and_then(|value| value.checked_add(pose.translation()))
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let inertia = rotation
        .checked_mul_matrix(mass.inertia())
        .and_then(|value| value.checked_mul_matrix(rotation.transpose()))
        .ok_or(ClusterError::ArithmeticOverflow)?;
    Ok((center, inertia))
}

fn map_cholesky(error: CholeskyError) -> ClusterError {
    match error {
        CholeskyError::NonFiniteInput | CholeskyError::ResultOutOfRange => {
            ClusterError::ArithmeticOverflow
        }
        CholeskyError::NotPositiveDefinite | CholeskyError::NotSymmetric => {
            ClusterError::InvalidMass
        }
    }
}

/// Transfers total linear and angular momentum about a world-space origin.
#[allow(clippy::too_many_arguments)]
pub(crate) fn transfer_motion(
    old_mass: MassProperties,
    old_pose: Transform,
    velocity: Vec3,
    omega: Vec3,
    new_mass: MassProperties,
    new_pose: Transform,
    origin: Vec3,
    delta_p: Vec3,
    delta_h: Vec3,
) -> Result<(Vec3, Vec3), ClusterError> {
    if !velocity.is_finite()
        || !omega.is_finite()
        || !origin.is_finite()
        || !delta_p.is_finite()
        || !delta_h.is_finite()
    {
        return Err(ClusterError::InvalidMotion);
    }
    let (old_center, old_inertia) = world_properties(old_mass, old_pose)?;
    let (new_center, new_inertia) = world_properties(new_mass, new_pose)?;
    let old_p = velocity
        .checked_mul_scalar(old_mass.mass())
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let new_p = old_p
        .checked_add(delta_p)
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let old_spin = old_inertia
        .checked_mul_vector(omega)
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let old_orbital = old_center
        .checked_sub(origin)
        .and_then(|offset| offset.checked_cross(old_p))
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let new_h = old_spin
        .checked_add(old_orbital)
        .and_then(|value| value.checked_add(delta_h))
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let new_orbital = new_center
        .checked_sub(origin)
        .and_then(|offset| offset.checked_cross(new_p))
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let new_spin = new_h
        .checked_sub(new_orbital)
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let new_velocity = Vec3::new(
        new_p.x / new_mass.mass(),
        new_p.y / new_mass.mass(),
        new_p.z / new_mass.mass(),
    );
    if !new_velocity.is_finite() {
        return Err(ClusterError::ArithmeticOverflow);
    }
    let new_omega = new_inertia.solve_spd(new_spin).map_err(map_cholesky)?;
    if !new_omega.is_finite() {
        return Err(ClusterError::ArithmeticOverflow);
    }
    Ok((new_velocity, new_omega))
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_math::Quat;

    fn close(actual: Vec3, expected: Vec3) {
        for (a, b) in [
            (actual.x, expected.x),
            (actual.y, expected.y),
            (actual.z, expected.z),
        ] {
            assert!(
                (a - b).abs() <= 1e-11 * a.abs().max(b.abs()).max(1.0),
                "{a} != {b}"
            );
        }
    }

    #[test]
    fn velocity_field_rotates_about_shifted_center() -> Result<(), ClusterError> {
        let (velocity, angular) = velocity_field(Vec3::X, Vec3::Y, 2.0 * Vec3::Z, 3.0 * Vec3::X)?;
        close(velocity, 5.0 * Vec3::Y);
        close(angular, 2.0 * Vec3::Z);
        assert_eq!(
            velocity_field(
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::Z,
                Vec3::new(f64::NAN, 0.0, 0.0)
            ),
            Err(ClusterError::InvalidMotion)
        );
        assert_eq!(
            velocity_field(-Vec3::X * f64::MAX, Vec3::ZERO, Vec3::Z, Vec3::X * f64::MAX),
            Err(ClusterError::ArithmeticOverflow)
        );
        Ok(())
    }

    #[test]
    fn unchanged_body_does_not_accelerate() -> Result<(), Box<dyn std::error::Error>> {
        let mass =
            MassProperties::new(3.0, Vec3::X, Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0)))?;
        let rotation = Quat::from_axis_angle(Vec3::Z, 0.7).ok_or("rotation")?;
        let pose = Transform::new(Vec3::new(4.0, -2.0, 1.0), rotation)?;
        let v = Vec3::new(2.0, -1.0, 0.5);
        let w = Vec3::new(0.2, 0.7, -0.4);
        let (actual_v, actual_w) = transfer_motion(
            mass,
            pose,
            v,
            w,
            mass,
            pose,
            Vec3::new(-2.0, 3.0, 1.0),
            Vec3::ZERO,
            Vec3::ZERO,
        )?;
        close(actual_v, v);
        close(actual_w, w);
        Ok(())
    }

    #[test]
    fn fuel_removal_without_reaction_preserves_velocity() -> Result<(), ClusterError> {
        let original_center = Vec3::new(10.0, 2.0, -3.0);
        let velocity = Vec3::new(20.0, -4.0, 1.0);
        let (new_velocity, new_omega) = velocity_field(
            original_center,
            velocity,
            Vec3::ZERO,
            original_center + Vec3::X,
        )?;
        close(new_velocity, velocity);
        close(new_omega, Vec3::ZERO);
        Ok(())
    }

    #[test]
    fn transfer_conserves_momentum_with_shift_rotation_and_impulses()
    -> Result<(), Box<dyn std::error::Error>> {
        let old = MassProperties::new(2.0, Vec3::X, Mat3::from_diagonal(Vec3::new(2.0, 3.0, 4.0)))?;
        let new = MassProperties::new(3.0, Vec3::Y, Mat3::from_diagonal(Vec3::new(3.0, 4.0, 5.0)))?;
        let old_pose = Transform::new(Vec3::new(1.0, 2.0, -1.0), Quat::IDENTITY)?;
        let rotation = Quat::from_axis_angle(Vec3::Z, 0.8).ok_or("rotation")?;
        let new_pose = Transform::new(Vec3::new(-1.0, 0.5, 2.0), rotation)?;
        let origin = Vec3::new(0.5, -1.0, 3.0);
        let v = Vec3::new(1.0, -2.0, 0.5);
        let w = Vec3::new(0.3, 0.4, -0.2);
        let dp = Vec3::new(-0.4, 1.0, 0.7);
        let dh = Vec3::new(0.8, -0.3, 1.2);
        let (nv, nw) = transfer_motion(old, old_pose, v, w, new, new_pose, origin, dp, dh)?;
        let (oc, oi) = world_properties(old, old_pose)?;
        let (nc, ni) = world_properties(new, new_pose)?;
        let op = v * old.mass();
        let np = nv * new.mass();
        close(np, op + dp);
        close(
            ni * nw + (nc - origin).cross(np),
            oi * w + (oc - origin).cross(op) + dh,
        );
        Ok(())
    }

    #[test]
    fn cholesky_errors_are_distinct() {
        assert_eq!(
            map_cholesky(CholeskyError::NotPositiveDefinite),
            ClusterError::InvalidMass
        );
        assert_eq!(
            map_cholesky(CholeskyError::NotSymmetric),
            ClusterError::InvalidMass
        );
        assert_eq!(
            map_cholesky(CholeskyError::NonFiniteInput),
            ClusterError::ArithmeticOverflow
        );
        assert_eq!(
            map_cholesky(CholeskyError::ResultOutOfRange),
            ClusterError::ArithmeticOverflow
        );
    }
}
