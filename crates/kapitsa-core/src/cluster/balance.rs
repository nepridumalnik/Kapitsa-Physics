//! Internal instantaneous tree balance; not connected to world diagnostics.

use kapitsa_math::Vec3;

use crate::{mass::MassProperties, transform::Transform};

use super::{ClusterError, motion::part_momentum_rate};

#[derive(Clone, Copy)]
pub(crate) struct PartBalanceInput {
    pub mass: MassProperties,
    pub pose: Transform,
    /// World-space force and moment about the common fixed origin.
    pub external_force: Vec3,
    pub external_moment: Vec3,
    /// Caller must establish completeness, including all force and contact paths.
    pub loads_complete: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum BalanceError {
    IncompleteLoads,
    InvalidTree,
    InvalidTolerance,
    Unbalanced,
    Physics(ClusterError),
}

/// Express a reaction's moment at a weld anchor rather than the common
/// balance origin. Force and both points are world-space; the result is N·m.
/// No strength decision is valid until the inputs and load ledger are complete.
pub(crate) fn reaction_moment_at_anchor(
    force: Vec3,
    moment_at_origin: Vec3,
    origin: Vec3,
    anchor: Vec3,
) -> Result<Vec3, BalanceError> {
    if [force, moment_at_origin, origin, anchor]
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err(BalanceError::Physics(ClusterError::InvalidMotion));
    }
    anchor
        .checked_sub(origin)
        .and_then(|offset| offset.checked_cross(force))
        .and_then(|shift| moment_at_origin.checked_sub(shift))
        .ok_or(BalanceError::Physics(ClusterError::ArithmeticOverflow))
}

/// Reaction exerted by each parent on its child, with moment about `origin`.
/// Parents must precede children; the root is index zero. This ordering allows
/// a linear reverse traversal without adjacency allocations. `force_tolerance`
/// and `moment_tolerance` are absolute tolerances in N and N·m respectively.
pub(crate) fn tree_reactions(
    parts: &[PartBalanceInput],
    parents: &[usize],
    reference_center: Vec3,
    reference_acceleration: Vec3,
    omega: Vec3,
    alpha: Vec3,
    origin: Vec3,
    force_tolerance: f64,
    moment_tolerance: f64,
) -> Result<Vec<(Vec3, Vec3)>, BalanceError> {
    if parts.is_empty()
        || parents.len() != parts.len()
        || parents[0] != 0
        || parents.iter().enumerate().skip(1).any(|(i, &p)| p >= i)
    {
        return Err(BalanceError::InvalidTree);
    }
    if !force_tolerance.is_finite()
        || force_tolerance < 0.0
        || !moment_tolerance.is_finite()
        || moment_tolerance < 0.0
    {
        return Err(BalanceError::InvalidTolerance);
    }
    if parts.iter().any(|part| !part.loads_complete) {
        return Err(BalanceError::IncompleteLoads);
    }
    let mut reactions = Vec::new();
    reactions
        .try_reserve_exact(parts.len())
        .map_err(|_| BalanceError::Physics(ClusterError::StorageCapacityExceeded))?;
    for part in parts {
        if !part.external_force.is_finite() || !part.external_moment.is_finite() {
            return Err(BalanceError::Physics(ClusterError::InvalidMotion));
        }
        let (force, moment) = part_momentum_rate(
            part.mass,
            part.pose,
            reference_center,
            reference_acceleration,
            omega,
            alpha,
            origin,
        )
        .map_err(BalanceError::Physics)?;
        reactions.push((
            force
                .checked_sub(part.external_force)
                .ok_or(BalanceError::Physics(ClusterError::ArithmeticOverflow))?,
            moment
                .checked_sub(part.external_moment)
                .ok_or(BalanceError::Physics(ClusterError::ArithmeticOverflow))?,
        ));
    }
    for child in (1..parts.len()).rev() {
        let parent = parents[child];
        reactions[parent].0 = reactions[parent]
            .0
            .checked_add(reactions[child].0)
            .ok_or(BalanceError::Physics(ClusterError::ArithmeticOverflow))?;
        reactions[parent].1 = reactions[parent]
            .1
            .checked_add(reactions[child].1)
            .ok_or(BalanceError::Physics(ClusterError::ArithmeticOverflow))?;
    }
    let (force, moment) = reactions[0];
    if !force.length().is_finite() || !moment.length().is_finite() {
        return Err(BalanceError::Physics(ClusterError::ArithmeticOverflow));
    }
    if force.length() > force_tolerance || moment.length() > moment_tolerance {
        return Err(BalanceError::Unbalanced);
    }
    Ok(reactions)
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_math::{Mat3, Quat};

    #[test]
    fn tree_reactions_validate_completeness_and_equilibrium()
    -> Result<(), Box<dyn std::error::Error>> {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let make = |x, force| -> Result<PartBalanceInput, Box<dyn std::error::Error>> {
            Ok(PartBalanceInput {
                mass,
                pose: Transform::new(Vec3::X * x, Quat::IDENTITY)?,
                external_force: force,
                external_moment: Vec3::ZERO,
                loads_complete: true,
            })
        };
        let mut parts = [
            make(0.0, Vec3::X * 6.0)?,
            make(1.0, Vec3::ZERO)?,
            make(2.0, Vec3::ZERO)?,
        ];
        let solve = |parts: &[PartBalanceInput], parents: &[usize]| {
            tree_reactions(
                parts,
                parents,
                Vec3::X,
                Vec3::X * 2.0,
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
                1e-12,
                1e-12,
            )
        };
        let result = solve(&parts, &[0, 0, 1]).map_err(|e| format!("{e:?}"))?;
        assert_eq!(result[1], (Vec3::X * 4.0, Vec3::ZERO));
        assert_eq!(result[2], (Vec3::X * 2.0, Vec3::ZERO));
        parts[2].loads_complete = false;
        assert_eq!(
            solve(&parts, &[0, 0, 1]),
            Err(BalanceError::IncompleteLoads)
        );
        parts[2].loads_complete = true;
        assert_eq!(solve(&parts, &[0, 2, 1]), Err(BalanceError::InvalidTree));
        parts[0].external_force = Vec3::ZERO;
        assert_eq!(solve(&parts, &[0, 0, 1]), Err(BalanceError::Unbalanced));
        Ok(())
    }

    #[test]
    fn offset_load_reaction_transforms_between_moment_origins()
    -> Result<(), Box<dyn std::error::Error>> {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        // Equal and opposite forces form a couple. The child is at (2, 0, 0).
        // Its static cut reaction is -Y, with moment -2 Z about the origin.
        let parts = [
            PartBalanceInput {
                mass,
                pose: Transform::identity(),
                external_force: -Vec3::Y,
                external_moment: -Vec3::Z * 2.0,
                loads_complete: true,
            },
            PartBalanceInput {
                mass,
                pose: Transform::new(Vec3::X * 2.0, Quat::IDENTITY)?,
                external_force: Vec3::Y,
                external_moment: Vec3::Z * 2.0,
                loads_complete: true,
            },
        ];
        let solve = |parts: &[PartBalanceInput], origin| {
            tree_reactions(
                parts,
                &[0, 0],
                Vec3::X,
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
                origin,
                1e-12,
                1e-12,
            )
        };
        let about_zero = solve(&parts, Vec3::ZERO).map_err(|e| format!("{e:?}"))?;
        assert_eq!(about_zero[1], (-Vec3::Y, -Vec3::Z * 2.0));
        assert_eq!(
            reaction_moment_at_anchor(about_zero[1].0, about_zero[1].1, Vec3::ZERO, Vec3::X)
                .map_err(|e| format!("{e:?}"))?,
            -Vec3::Z
        );
        assert_eq!(
            reaction_moment_at_anchor(
                Vec3::X,
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::new(f64::NAN, 0.0, 0.0)
            ),
            Err(BalanceError::Physics(ClusterError::InvalidMotion))
        );
        let mut shifted = parts;
        for part in &mut shifted {
            part.external_moment -= Vec3::X.cross(part.external_force);
        }
        let about_x = solve(&shifted, Vec3::X).map_err(|e| format!("{e:?}"))?;
        assert_eq!(about_x[1], (-Vec3::Y, -Vec3::Z));
        Ok(())
    }
}
