use crate::collision::CollisionError;
use kapitsa_math::Vec3;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct JointWrench {
    /// Average force in newtons, applied at the selected connector anchor.
    pub(crate) force: Vec3,
    /// Average torque in newton-meters, about the selected connector anchor.
    pub(crate) torque: Vec3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum ReactionMode {
    /// Uses velocity impulses only; position XPBD multipliers are not included.
    #[default]
    VelocityOnly,
    /// Reserved until a non-double-counted constraint-load estimator is defined.
    #[allow(dead_code)]
    ConstraintEstimate,
}

/// Converts generalized impulse at the center of mass to average wrench at an anchor.
/// `torque_com_impulse` is the angular Jacobian impulse, before shifting its reference point.
pub(crate) fn wrench_from_impulse(
    linear_impulse: Vec3,
    torque_com_impulse: Vec3,
    com_world: Vec3,
    anchor_world: Vec3,
    dt: f64,
) -> Result<JointWrench, CollisionError> {
    if !dt.is_finite()
        || dt <= 0.0
        || !linear_impulse.is_finite()
        || !torque_com_impulse.is_finite()
        || !com_world.is_finite()
        || !anchor_world.is_finite()
    {
        return Err(CollisionError::InvalidContact);
    }

    let shift = anchor_world - com_world;
    let force = linear_impulse / dt;
    let torque = (torque_com_impulse - shift.cross(linear_impulse)) / dt;
    if !force.is_finite() || !torque.is_finite() {
        return Err(CollisionError::ArithmeticOverflow);
    }

    Ok(JointWrench { force, torque })
}

#[cfg(test)]
mod tests {
    use super::wrench_from_impulse;
    use crate::collision::CollisionError;
    use kapitsa_math::Vec3;

    #[test]
    fn force_applied_at_anchor_has_zero_torque_at_anchor() {
        let wrench = wrench_from_impulse(
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -20.0),
            Vec3::ZERO,
            Vec3::new(0.0, 2.0, 0.0),
            0.5,
        )
        .expect("valid reaction");

        assert_eq!(wrench.force, Vec3::new(20.0, 0.0, 0.0));
        assert_eq!(wrench.torque, Vec3::ZERO);
    }

    #[test]
    fn pure_couple_preserves_torque_under_anchor_shift() {
        let wrench = wrench_from_impulse(
            Vec3::ZERO,
            Vec3::new(0.0, 0.0, 3.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::ZERO,
            0.5,
        )
        .expect("valid wrench");

        assert_eq!(wrench.force, Vec3::ZERO);
        assert_eq!(wrench.torque, Vec3::new(0.0, 0.0, 6.0));
    }

    #[test]
    fn equal_and_opposite_impulses_match_at_a_shared_anchor() {
        let anchor = Vec3::ZERO;
        let side_a = wrench_from_impulse(
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, -1.0, 0.0),
            anchor,
            1.0,
        )
        .expect("side A wrench");
        let side_b = wrench_from_impulse(
            Vec3::new(-2.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -5.0),
            Vec3::new(0.0, 1.0, 0.0),
            anchor,
            1.0,
        )
        .expect("side B wrench");

        assert_eq!(side_a.force, -side_b.force);
        assert_eq!(side_a.torque, -side_b.torque);
        assert_eq!(side_a.torque, Vec3::new(0.0, 0.0, 3.0));
    }

    #[test]
    fn wrench_conversion_rejects_invalid_inputs_and_overflow() {
        assert_eq!(
            wrench_from_impulse(Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, 0.0),
            Err(CollisionError::InvalidContact)
        );
        assert_eq!(
            wrench_from_impulse(
                Vec3::splat(f64::NAN),
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
                1.0,
            ),
            Err(CollisionError::InvalidContact)
        );
        assert_eq!(
            wrench_from_impulse(
                Vec3::splat(f64::MAX),
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
                f64::MIN_POSITIVE,
            ),
            Err(CollisionError::ArithmeticOverflow)
        );
    }
}
