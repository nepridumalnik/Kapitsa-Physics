//! Internal instantaneous tree balance; not connected to world diagnostics.

use kapitsa_math::Vec3;
use std::collections::{HashMap, HashSet};

use crate::{
    mass::MassProperties,
    transform::Transform,
    world::{
        ClusterLoadPart, ClusterLoadState, ClusterLoadWeld, PartForceSourceWrench,
        PartGravityWrench, PartHullWrench, PartId, PartWingWrench, WeldId,
    },
};

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

/// Attributed force snapshots, all expressed about the evaluation-time body COM.
/// Other force and impulse paths are deliberately not represented here.
pub(crate) struct RecordedPartLoads<'a> {
    pub sources: &'a [PartForceSourceWrench],
    pub gravity: &'a [PartGravityWrench],
    pub hull: &'a [PartHullWrench],
    pub wings: &'a [PartWingWrench],
}

/// Accumulates recorded contributions about `origin` without certifying the
/// ledger as complete. Slices must come from the same full-step snapshot.
pub(crate) fn prepare_recorded_loads(
    state: &ClusterLoadState,
    tree: &[TreePart],
    loads: RecordedPartLoads<'_>,
    body_center: Vec3,
    origin: Vec3,
) -> Result<Vec<PartBalanceInput>, crate::Error> {
    if !body_center.is_finite() || !origin.is_finite() {
        return Err(crate::Error::from(BalanceError::Physics(
            crate::Error::from(ClusterError::InvalidMotion),
        )));
    }
    let capacity_error =
        || BalanceError::Physics(crate::Error::from(ClusterError::StorageCapacityExceeded));
    let mut indices = HashMap::new();
    indices
        .try_reserve(tree.len())
        .map_err(|_| capacity_error())?;
    let mut inputs = Vec::new();
    inputs
        .try_reserve_exact(tree.len())
        .map_err(|_| capacity_error())?;
    for part in tree {
        if indices.insert(part.part, inputs.len()).is_some() {
            return Err(crate::Error::from(BalanceError::InvalidTree));
        }
        inputs.push(part.incomplete_balance_input());
    }
    let offset = body_center.checked_sub(origin).ok_or_else(|| {
        crate::Error::from(BalanceError::Physics(crate::Error::from(
            ClusterError::ArithmeticOverflow,
        )))
    })?;
    let mut add = |part, force: Vec3, moment: Vec3| -> Result<(), crate::Error> {
        let index = *indices
            .get(&part)
            .ok_or_else(|| crate::Error::from(BalanceError::InvalidTree))?;
        if !force.is_finite() || !moment.is_finite() {
            return Err(crate::Error::from(BalanceError::Physics(
                crate::Error::from(ClusterError::InvalidMotion),
            )));
        }
        let shifted = offset
            .checked_cross(force)
            .and_then(|shift| moment.checked_add(shift))
            .ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
        let input = &mut inputs[index];
        input.external_force = input.external_force.checked_add(force).ok_or_else(|| {
            crate::Error::from(BalanceError::Physics(crate::Error::from(
                ClusterError::ArithmeticOverflow,
            )))
        })?;
        input.external_moment = input.external_moment.checked_add(shifted).ok_or_else(|| {
            crate::Error::from(BalanceError::Physics(crate::Error::from(
                ClusterError::ArithmeticOverflow,
            )))
        })?;
        Ok(())
    };
    for load in loads.sources.iter().filter(|load| load.body == state.body) {
        add(load.part, load.force, load.moment)?;
    }
    for load in loads.gravity.iter().filter(|load| load.body == state.body) {
        add(load.part, load.force, load.moment)?;
    }
    for load in loads.hull.iter().filter(|load| load.body == state.body) {
        add(load.part, load.wrench.force, load.wrench.moment)?;
    }
    for load in loads.wings.iter().filter(|load| load.body == state.body) {
        add(load.part, load.wrench.force, load.wrench.moment)?;
    }
    Ok(inputs)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BalanceError {
    IncompleteLoads,
    InvalidTree,
    InvalidTolerance,
    Unbalanced,
    Physics(crate::Error),
}

impl std::fmt::Display for BalanceError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IncompleteLoads => formatter.write_str("weld load snapshot is incomplete"),
            Self::InvalidTree => formatter.write_str("weld tree topology is invalid"),
            Self::InvalidTolerance => formatter.write_str("weld balance tolerance is invalid"),
            Self::Unbalanced => formatter.write_str("weld loads do not balance"),
            Self::Physics(error) => write!(formatter, "weld balance physics error: {error}"),
        }
    }
}

impl std::error::Error for BalanceError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Physics(error) => Some(error),
            _ => None,
        }
    }
}

/// Parent-on-child reaction in world coordinates, at the child's weld anchor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldReaction {
    pub weld: WeldId,
    pub child: PartId,
    pub force: Vec3,
    pub moment: Vec3,
    pub anchor: Vec3,
}

/// Root-first topology from one evaluation-time snapshot. This establishes no
/// claim about force attribution or whether a snapshot is complete.
pub(crate) struct TreePart {
    pub part: PartId,
    pub parent: usize,
    pub weld: Option<WeldId>,
    pub mass: MassProperties,
    pub pose: Transform,
}

impl TreePart {
    /// Initializes a part without asserting load completeness. The ledger
    /// must be populated and independently certified before solving.
    pub(crate) fn incomplete_balance_input(&self) -> PartBalanceInput {
        PartBalanceInput {
            mass: self.mass,
            pose: self.pose,
            external_force: Vec3::ZERO,
            external_moment: Vec3::ZERO,
            loads_complete: false,
        }
    }
}

/// Certifies only the conservative force-free regime established by the world
/// before integration. Any other regime remains unsupported, even if all
/// diagnostic load slices happen to be empty.
pub(crate) fn prepare_force_free_inputs(
    state: &ClusterLoadState,
    tree: &[TreePart],
) -> Result<Vec<PartBalanceInput>, crate::Error> {
    if !state.force_free_certified || state.collision_loads_uncertified {
        return Err(crate::Error::from(BalanceError::IncompleteLoads));
    }
    let mut inputs = Vec::new();
    inputs.try_reserve_exact(tree.len()).map_err(|_| {
        BalanceError::Physics(crate::Error::from(ClusterError::StorageCapacityExceeded))
    })?;
    for part in tree {
        let mut input = part.incomplete_balance_input();
        input.loads_complete = true;
        inputs.push(input);
    }
    Ok(inputs)
}

/// Builds an ordered tree in expected O(parts + welds), rejecting disconnected,
/// cyclic, duplicate, and cross-body edges. Inputs must belong to one snapshot.
pub(crate) fn prepare_tree(
    state: &ClusterLoadState,
    parts: &[ClusterLoadPart],
    welds: &[ClusterLoadWeld],
) -> Result<Vec<TreePart>, crate::Error> {
    let capacity_error =
        || BalanceError::Physics(crate::Error::from(ClusterError::StorageCapacityExceeded));
    let mut indices = HashMap::new();
    indices
        .try_reserve(parts.len())
        .map_err(|_| capacity_error())?;
    let mut selected = Vec::new();
    selected
        .try_reserve(parts.len())
        .map_err(|_| capacity_error())?;
    for part in parts.iter().filter(|part| part.body == state.body) {
        if indices.insert(part.part, selected.len()).is_some() {
            return Err(crate::Error::from(BalanceError::InvalidTree));
        }
        selected.push(part);
    }
    let root = *indices
        .get(&state.root)
        .ok_or_else(|| crate::Error::from(BalanceError::InvalidTree))?;
    let mut adjacency = Vec::new();
    adjacency
        .try_reserve_exact(selected.len())
        .map_err(|_| capacity_error())?;
    adjacency.resize_with(selected.len(), Vec::new);
    let mut weld_ids = HashSet::new();
    weld_ids
        .try_reserve(welds.len())
        .map_err(|_| capacity_error())?;
    let mut edge_count = 0usize;
    for weld in welds {
        let (a, b) = weld.value.parts();
        match (indices.get(&a), indices.get(&b)) {
            (None, None) => continue,
            (Some(&a), Some(&b)) if a != b && weld_ids.insert(weld.weld) => {
                adjacency[a].try_reserve(1).map_err(|_| capacity_error())?;
                adjacency[b].try_reserve(1).map_err(|_| capacity_error())?;
                adjacency[a].push((b, weld.weld));
                adjacency[b].push((a, weld.weld));
                edge_count += 1;
            }
            _ => return Err(crate::Error::from(BalanceError::InvalidTree)),
        }
    }
    if edge_count != selected.len() - 1 {
        return Err(crate::Error::from(BalanceError::InvalidTree));
    }
    let mut seen = Vec::new();
    seen.try_reserve_exact(selected.len())
        .map_err(|_| capacity_error())?;
    seen.resize(selected.len(), false);
    let mut queue = Vec::new();
    queue
        .try_reserve_exact(selected.len())
        .map_err(|_| capacity_error())?;
    queue.push((root, 0, None));
    seen[root] = true;
    let mut result = Vec::new();
    result
        .try_reserve_exact(selected.len())
        .map_err(|_| capacity_error())?;
    let mut cursor = 0;
    while cursor < queue.len() {
        let (index, parent, weld) = queue[cursor];
        let part = selected[index];
        let pose = state
            .state
            .transform()
            .compose(&part.local_transform)
            .map_err(|error| {
                BalanceError::Physics(crate::Error::from(ClusterError::Transform(error)))
            })?;
        result.push(TreePart {
            part: part.part,
            parent,
            weld,
            mass: part.mass,
            pose,
        });
        for &(next, edge) in &adjacency[index] {
            if !seen[next] {
                seen[next] = true;
                queue.push((next, cursor, Some(edge)));
            }
        }
        cursor += 1;
    }
    if result.len() != selected.len() {
        return Err(crate::Error::from(BalanceError::InvalidTree));
    }
    Ok(result)
}

/// Express a reaction's moment at a weld anchor rather than the common
/// balance origin. Force and both points are world-space; the result is N·m.
/// No strength decision is valid until the inputs and load ledger are complete.
pub(crate) fn reaction_moment_at_anchor(
    force: Vec3,
    moment_at_origin: Vec3,
    origin: Vec3,
    anchor: Vec3,
) -> Result<Vec3, crate::Error> {
    if [force, moment_at_origin, origin, anchor]
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err(crate::Error::from(BalanceError::Physics(
            crate::Error::from(ClusterError::InvalidMotion),
        )));
    }
    anchor
        .checked_sub(origin)
        .and_then(|offset| offset.checked_cross(force))
        .and_then(|shift| moment_at_origin.checked_sub(shift))
        .ok_or_else(|| {
            crate::Error::from(BalanceError::Physics(crate::Error::from(
                ClusterError::ArithmeticOverflow,
            )))
        })
}

/// Reaction exerted by each parent on its child, with moment about `origin`.
/// Parents must precede children; the root is index zero. This ordering allows
/// a linear reverse traversal without adjacency allocations. `force_tolerance`
/// and `moment_tolerance` are absolute tolerances in N and N·m respectively.
#[allow(clippy::too_many_arguments)] // The physical reference frame and both tolerances are independent inputs.
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
) -> Result<Vec<(Vec3, Vec3)>, crate::Error> {
    if parts.is_empty()
        || parents.len() != parts.len()
        || parents[0] != 0
        || parents.iter().enumerate().skip(1).any(|(i, &p)| p >= i)
    {
        return Err(crate::Error::from(BalanceError::InvalidTree));
    }
    if !force_tolerance.is_finite()
        || force_tolerance < 0.0
        || !moment_tolerance.is_finite()
        || moment_tolerance < 0.0
    {
        return Err(crate::Error::from(BalanceError::InvalidTolerance));
    }
    if parts.iter().any(|part| !part.loads_complete) {
        return Err(crate::Error::from(BalanceError::IncompleteLoads));
    }
    let mut reactions = Vec::new();
    reactions.try_reserve_exact(parts.len()).map_err(|_| {
        BalanceError::Physics(crate::Error::from(ClusterError::StorageCapacityExceeded))
    })?;
    for part in parts {
        if !part.external_force.is_finite() || !part.external_moment.is_finite() {
            return Err(crate::Error::from(BalanceError::Physics(
                crate::Error::from(ClusterError::InvalidMotion),
            )));
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
            force.checked_sub(part.external_force).ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?,
            moment.checked_sub(part.external_moment).ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?,
        ));
    }
    for child in (1..parts.len()).rev() {
        let parent = parents[child];
        reactions[parent].0 = reactions[parent]
            .0
            .checked_add(reactions[child].0)
            .ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
        reactions[parent].1 = reactions[parent]
            .1
            .checked_add(reactions[child].1)
            .ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
    }
    let (force, moment) = reactions[0];
    if !force.length().is_finite() || !moment.length().is_finite() {
        return Err(crate::Error::from(BalanceError::Physics(
            crate::Error::from(ClusterError::ArithmeticOverflow),
        )));
    }
    if force.length() > force_tolerance || moment.length() > moment_tolerance {
        return Err(crate::Error::from(BalanceError::Unbalanced));
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
            Err(crate::Error::from(BalanceError::IncompleteLoads))
        );
        parts[2].loads_complete = true;
        assert_eq!(
            solve(&parts, &[0, 2, 1]),
            Err(crate::Error::from(BalanceError::InvalidTree))
        );
        parts[0].external_force = Vec3::ZERO;
        assert_eq!(
            solve(&parts, &[0, 0, 1]),
            Err(crate::Error::from(BalanceError::Unbalanced))
        );
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
            Err(crate::Error::from(BalanceError::Physics(
                crate::Error::from(ClusterError::InvalidMotion)
            )))
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
