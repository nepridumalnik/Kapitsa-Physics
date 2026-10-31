use crate::{collision::CollisionPair, transform::Transform};
use kapitsa_math::Vec3;

use super::CollisionError;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ContactPoint {
    pub(crate) local_a: Vec3,
    pub(crate) local_b: Vec3,
    pub(crate) normal: Vec3,
    pub(crate) penetration: f64,
    pub(crate) feature_id: u64,
    pub(crate) normal_impulse: f64,
    pub(crate) tangent_impulse: [f64; 2],
}

impl ContactPoint {
    pub(crate) fn from_world(
        body_a: &Transform,
        body_b: &Transform,
        point_a: Vec3,
        point_b: Vec3,
        normal: Vec3,
        penetration: f64,
        feature_id: u64,
    ) -> Result<Self, CollisionError> {
        if !point_a.is_finite()
            || !point_b.is_finite()
            || !penetration.is_finite()
            || penetration < 0.0
        {
            return Err(CollisionError::InvalidContact);
        }
        let normal = normal
            .try_normalize()
            .ok_or(CollisionError::InvalidContact)?;
        let inverse_a = body_a
            .inverse()
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let inverse_b = body_b
            .inverse()
            .map_err(|_| CollisionError::ArithmeticOverflow)?;
        let local_a = inverse_a.transform_point(point_a);
        let local_b = inverse_b.transform_point(point_b);
        if !local_a.is_finite() || !local_b.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(Self {
            local_a,
            local_b,
            normal,
            penetration,
            feature_id,
            normal_impulse: 0.0,
            tangent_impulse: [0.0; 2],
        })
    }
}

/// A fixed-size contact patch for one canonical collider pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ContactManifold {
    pub(crate) pair: CollisionPair,
    points: [Option<ContactPoint>; 4],
    world_points: [Vec3; 4],
    count: usize,
}

impl ContactManifold {
    pub(crate) const fn new(pair: CollisionPair) -> Self {
        Self {
            pair,
            points: [None; 4],
            world_points: [Vec3::ZERO; 4],
            count: 0,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.count
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub(crate) fn points(&self) -> impl Iterator<Item = ContactPoint> + '_ {
        self.points
            .iter()
            .take(self.count)
            .filter_map(|point| *point)
    }

    pub(crate) fn insert(
        &mut self,
        point: ContactPoint,
        world_position: Vec3,
    ) -> Result<(), CollisionError> {
        if !world_position.is_finite()
            || !point.local_a.is_finite()
            || !point.local_b.is_finite()
            || !point.normal.is_finite()
            || !point.penetration.is_finite()
            || point.penetration < 0.0
        {
            return Err(CollisionError::InvalidContact);
        }
        for index in 0..self.count {
            let existing = self
                .points
                .get(index)
                .and_then(|entry| *entry)
                .ok_or(CollisionError::InvalidContact)?;
            if existing.feature_id == point.feature_id {
                let mut updated = point;
                updated.normal_impulse = existing.normal_impulse;
                updated.tangent_impulse = existing.tangent_impulse;
                *self
                    .points
                    .get_mut(index)
                    .ok_or(CollisionError::InvalidContact)? = Some(updated);
                *self
                    .world_points
                    .get_mut(index)
                    .ok_or(CollisionError::InvalidContact)? = world_position;
                return Ok(());
            }
        }
        if self.count < self.points.len() {
            let index = self.count;
            *self
                .points
                .get_mut(index)
                .ok_or(CollisionError::InvalidContact)? = Some(point);
            *self
                .world_points
                .get_mut(index)
                .ok_or(CollisionError::InvalidContact)? = world_position;
            self.count += 1;
            return Ok(());
        }

        self.reduce_with(point, world_position)
    }

    fn reduce_with(
        &mut self,
        point: ContactPoint,
        world_position: Vec3,
    ) -> Result<(), CollisionError> {
        let mut candidates = [(None, Vec3::ZERO); 5];
        let mut deepest = 0;
        let mut deepest_penetration = f64::NEG_INFINITY;
        for (index, candidate) in candidates.iter_mut().take(4).enumerate() {
            let contact = self
                .points
                .get(index)
                .and_then(|entry| *entry)
                .ok_or(CollisionError::InvalidContact)?;
            let position = *self
                .world_points
                .get(index)
                .ok_or(CollisionError::InvalidContact)?;
            *candidate = (Some(contact), position);
            if contact.penetration > deepest_penetration {
                deepest = index;
                deepest_penetration = contact.penetration;
            }
        }
        *candidates
            .get_mut(4)
            .ok_or(CollisionError::InvalidContact)? = (Some(point), world_position);
        if point.penetration > deepest_penetration {
            deepest = 4;
        }

        let mut excluded = None;
        let mut best_spread = f64::NEG_INFINITY;
        for omitted in 0..5 {
            if omitted == deepest {
                continue;
            }
            let spread = patch_spread(&candidates, omitted)?;
            if spread > best_spread {
                best_spread = spread;
                excluded = Some(omitted);
            }
        }
        let excluded = excluded.ok_or(CollisionError::InvalidContact)?;
        let mut destination = 0;
        for (index, (candidate, position)) in candidates.iter().copied().enumerate() {
            if index == excluded {
                continue;
            }
            let candidate = candidate.ok_or(CollisionError::InvalidContact)?;
            *self
                .points
                .get_mut(destination)
                .ok_or(CollisionError::InvalidContact)? = Some(candidate);
            *self
                .world_points
                .get_mut(destination)
                .ok_or(CollisionError::InvalidContact)? = position;
            destination += 1;
        }
        self.count = 4;
        Ok(())
    }
}

fn patch_spread(
    candidates: &[(Option<ContactPoint>, Vec3); 5],
    omitted: usize,
) -> Result<f64, CollisionError> {
    let mut maximum_distance = 0.0_f64;
    for left in 0..5 {
        if left == omitted {
            continue;
        }
        for right in (left + 1)..5 {
            if right == omitted {
                continue;
            }
            let delta = candidates[left]
                .1
                .checked_sub(candidates[right].1)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let distance = delta
                .checked_length()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            maximum_distance = maximum_distance.max(distance);
        }
    }
    if maximum_distance == 0.0 {
        return Ok(0.0);
    }

    let mut spread = 0.0;
    for left in 0..5 {
        if left == omitted {
            continue;
        }
        for right in (left + 1)..5 {
            if right == omitted {
                continue;
            }
            let delta = candidates[left]
                .1
                .checked_sub(candidates[right].1)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let distance = delta
                .checked_length()
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let normalized = distance / maximum_distance;
            spread += normalized * normalized;
        }
    }
    spread
        .is_finite()
        .then_some(spread)
        .ok_or(CollisionError::ArithmeticOverflow)
}

pub(crate) fn combine_feature_ids(feature_a: u64, feature_b: u64) -> u64 {
    let mut value = feature_a ^ feature_b.rotate_left(32);
    value ^= value >> 30;
    value = value.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value ^= value >> 27;
    value = value.wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

#[cfg(test)]
#[path = "manifold_tests.rs"]
mod tests;
