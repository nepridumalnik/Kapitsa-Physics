//! Impulsive momentum balance for a rigid tree at one fixed configuration.

use kapitsa_math::Vec3;

use crate::{mass::MassProperties, transform::Transform};

use super::balance::{self, BalanceError, PartBalanceInput};

/// A part's simultaneous external impulse at a fixed world-space configuration.
/// Angular impulse is in N·m·s about the common `origin`, not the part COM.
#[derive(Clone, Copy, Debug)]
pub struct PartImpulse {
    pub mass: MassProperties,
    pub pose: Transform,
    /// External linear impulse in world-space N·s.
    pub linear: Vec3,
    /// External angular impulse about `origin` in world-space N·m·s.
    pub angular: Vec3,
}

/// Parent-on-child constraint impulse, with angular impulse about `origin`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpulseReaction {
    pub linear: Vec3,
    pub angular: Vec3,
}

impl ImpulseReaction {
    /// Expresses angular impulse at `anchor` instead of `origin`.
    ///
    /// Points are world-space metres and both impulse vectors are world-space.
    /// The linear impulse is unchanged. Use the returned value for an angular
    /// [`ImpulseStrength`] limit specified at the weld anchor.
    pub fn at_anchor(self, origin: Vec3, anchor: Vec3) -> Result<Self, ImpulseBalanceError> {
        let angular =
            balance::reaction_moment_at_anchor(self.linear, self.angular, origin, anchor)?;
        Ok(Self { angular, ..self })
    }
}

/// Independent instantaneous impulse limits, not force or torque limits.
/// Angular impulse must be expressed at the weld anchor before comparison.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImpulseStrength {
    /// Maximum linear impulse magnitude in N·s.
    pub max_linear: Option<f64>,
    /// Maximum angular impulse magnitude at the weld anchor in N·m·s.
    pub max_angular: Option<f64>,
}

impl ImpulseStrength {
    /// Tests a reaction already expressed about its weld anchor. Equality does
    /// not break the weld. Invalid limits and non-finite reactions are errors.
    pub fn exceeded_by(self, reaction: ImpulseReaction) -> Result<bool, ImpulseBalanceError> {
        if self.max_linear.is_none() && self.max_angular.is_none()
            || [self.max_linear, self.max_angular]
                .into_iter()
                .flatten()
                .any(|limit| !limit.is_finite() || limit <= 0.0)
        {
            return Err(ImpulseBalanceError::InvalidTolerance);
        }
        if !reaction.linear.is_finite() || !reaction.angular.is_finite() {
            return Err(ImpulseBalanceError::Physics(
                super::ClusterError::InvalidMotion,
            ));
        }
        let linear = reaction.linear.length();
        let angular = reaction.angular.length();
        if !linear.is_finite() || !angular.is_finite() {
            return Err(ImpulseBalanceError::Physics(
                super::ClusterError::ArithmeticOverflow,
            ));
        }
        Ok(self.max_linear.is_some_and(|limit| linear > limit)
            || self.max_angular.is_some_and(|limit| angular > limit))
    }
}

/// Solves simultaneous impulsive tree balance at one fixed configuration.
///
/// Parents must precede children and root's parent is zero. All external
/// impulses, including constraints and contacts, must be supplied per part;
/// `delta_linear_velocity` and `delta_angular_velocity` are the corresponding
/// change of the assembly COM velocity and world angular velocity. The caller
/// must establish both completeness and simultaneity: a sequence of CCD impacts
/// at different poses cannot be combined here. Reaction zero is the residual,
/// verified against absolute tolerances in N·s and N·m·s. Remaining entries
/// are parent-on-child impulses. This solves ideal rigid constraints, not
/// impact duration, fracture propagation, contact generation, or strength.
pub fn tree_impulse_reactions(
    parts: &[PartImpulse],
    parents: &[usize],
    reference_center: Vec3,
    delta_linear_velocity: Vec3,
    delta_angular_velocity: Vec3,
    origin: Vec3,
    linear_tolerance: f64,
    angular_tolerance: f64,
) -> Result<Vec<ImpulseReaction>, BalanceError> {
    let mut inputs = Vec::new();
    inputs
        .try_reserve_exact(parts.len())
        .map_err(|_| BalanceError::Physics(super::ClusterError::StorageCapacityExceeded))?;
    for part in parts {
        inputs.push(PartBalanceInput {
            mass: part.mass,
            pose: part.pose,
            external_force: part.linear,
            external_moment: part.angular,
            loads_complete: true,
        });
    }
    let reactions = balance::tree_reactions(
        &inputs,
        parents,
        reference_center,
        delta_linear_velocity,
        Vec3::ZERO,
        delta_angular_velocity,
        origin,
        linear_tolerance,
        angular_tolerance,
    )?;
    let mut output = Vec::new();
    output
        .try_reserve_exact(reactions.len())
        .map_err(|_| BalanceError::Physics(super::ClusterError::StorageCapacityExceeded))?;
    output.extend(
        reactions
            .into_iter()
            .map(|(linear, angular)| ImpulseReaction { linear, angular }),
    );
    Ok(output)
}

/// Errors returned by impulsive tree balance.
pub use super::balance::BalanceError as ImpulseBalanceError;

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_math::Mat3;

    #[test]
    fn simultaneous_impulse_transmits_momentum_without_timestep()
    -> Result<(), Box<dyn std::error::Error>> {
        let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
        let part = PartImpulse {
            mass,
            pose: Transform::identity(),
            linear: Vec3::ZERO,
            angular: Vec3::ZERO,
        };
        let parts = [
            PartImpulse {
                linear: Vec3::X * 2.0,
                ..part
            },
            part,
        ];
        let result = tree_impulse_reactions(
            &parts,
            &[0, 0],
            Vec3::ZERO,
            Vec3::X,
            Vec3::ZERO,
            Vec3::ZERO,
            0.0,
            0.0,
        )
        .map_err(|e| format!("{e:?}"))?;
        assert_eq!(result[1].linear, Vec3::X);
        assert_eq!(result[1].angular, Vec3::ZERO);
        assert!(
            !ImpulseStrength {
                max_linear: Some(1.0),
                max_angular: None
            }
            .exceeded_by(result[1])
            .map_err(|e| format!("{e:?}"))?
        );
        assert!(
            ImpulseStrength {
                max_linear: Some(0.5),
                max_angular: None
            }
            .exceeded_by(result[1])
            .map_err(|e| format!("{e:?}"))?
        );
        assert_eq!(
            tree_impulse_reactions(
                &parts,
                &[0, 0],
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
                0.0,
                0.0
            ),
            Err(ImpulseBalanceError::Unbalanced)
        );
        Ok(())
    }

    #[test]
    fn strength_uses_angular_impulse_at_weld_anchor() -> Result<(), Box<dyn std::error::Error>> {
        let reaction = ImpulseReaction {
            linear: Vec3::Y * 2.0,
            angular: Vec3::Z * 6.0,
        };
        let anchored = reaction
            .at_anchor(Vec3::ZERO, Vec3::X * 2.0)
            .map_err(|e| format!("{e:?}"))?;
        assert_eq!(anchored.angular, Vec3::Z * 2.0);
        let limit = ImpulseStrength {
            max_linear: None,
            max_angular: Some(3.0),
        };
        assert!(limit.exceeded_by(reaction).map_err(|e| format!("{e:?}"))?);
        assert!(!limit.exceeded_by(anchored).map_err(|e| format!("{e:?}"))?);
        assert_eq!(
            reaction.at_anchor(Vec3::ZERO, Vec3::new(f64::NAN, 0.0, 0.0)),
            Err(ImpulseBalanceError::Physics(
                super::super::ClusterError::InvalidMotion
            ))
        );
        Ok(())
    }
}
