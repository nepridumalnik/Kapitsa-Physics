use crate::{collision::CollisionError, world::JointId};
use kapitsa_math::Vec3;

use super::{Joint, row::ConstraintRow};

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct WarmStartConfig {
    pub(crate) enabled: bool,
    pub(crate) scale_min: f64,
    pub(crate) scale_max: f64,
    pub(crate) minimum_axis_dot: f64,
}

impl WarmStartConfig {
    pub(crate) const fn disabled() -> Self {
        Self {
            enabled: false,
            scale_min: 0.5,
            scale_max: 2.0,
            minimum_axis_dot: 0.95,
        }
    }

    #[cfg(any(test, feature = "bench-internals"))]
    pub(crate) const fn enabled() -> Self {
        Self {
            enabled: true,
            ..Self::disabled()
        }
    }

    pub(crate) fn validate(self) -> bool {
        self.scale_min.is_finite()
            && self.scale_min > 0.0
            && self.scale_max.is_finite()
            && self.scale_max >= self.scale_min
            && self.minimum_axis_dot.is_finite()
            && (0.0..=1.0).contains(&self.minimum_axis_dot)
    }
}

impl Default for WarmStartConfig {
    fn default() -> Self {
        Self::disabled()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum JointKind {
    Fixed,
    Hinge,
    Slider,
    Ball,
    Cylindrical,
    Planar,
    Distance,
    Spring,
    ConeTwist,
    D6,
}

impl From<Joint> for JointKind {
    fn from(joint: Joint) -> Self {
        match joint {
            Joint::Fixed(_) => Self::Fixed,
            Joint::Hinge(_) => Self::Hinge,
            Joint::Slider(_) => Self::Slider,
            Joint::Ball(_) => Self::Ball,
            Joint::Cylindrical(_) => Self::Cylindrical,
            Joint::Planar(_) => Self::Planar,
            Joint::Distance(_) => Self::Distance,
            Joint::Spring(_) => Self::Spring,
            Joint::ConeTwist(_) => Self::ConeTwist,
            Joint::D6(_) => Self::D6,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct JointImpulse {
    joint: JointId,
    row_key: u8,
    impulse: f64,
    dt: f64,
    axis_a: Vec3,
    axis_b: Vec3,
    kind: JointKind,
}

#[derive(Default)]
struct JointImpulseSlot {
    joint: Option<JointId>,
    impulses: Vec<JointImpulse>,
}

/// Transactional velocity impulse cache, indexed by the stable joint slot.
#[derive(Default)]
pub(crate) struct JointImpulseCache {
    current: Vec<JointImpulseSlot>,
    next: Vec<JointImpulseSlot>,
}

impl JointImpulseCache {
    pub(crate) const fn new() -> Self {
        Self {
            current: Vec::new(),
            next: Vec::new(),
        }
    }

    pub(crate) fn begin_stage(&mut self) {
        clear_slots(&mut self.next);
    }

    pub(crate) fn lookup(
        &self,
        joint: JointId,
        kind: Joint,
        row: ConstraintRow,
        dt: f64,
        config: WarmStartConfig,
    ) -> Option<f64> {
        if !config.enabled || !config.validate() || !dt.is_finite() || dt <= 0.0 {
            return None;
        }
        let slot = self.current.get(joint.index() as usize)?;
        if slot.joint != Some(joint) {
            return None;
        }
        let kind = JointKind::from(kind);
        let entry = slot
            .impulses
            .iter()
            .find(|entry| entry.joint == joint && entry.row_key == row.key && entry.kind == kind)?;
        let (axis_a, axis_b) = row_axes(row)?;
        if axis_a.dot(entry.axis_a) < config.minimum_axis_dot
            || axis_b.dot(entry.axis_b) < config.minimum_axis_dot
        {
            return None;
        }
        let scale = dt / entry.dt;
        if !scale.is_finite() || scale < config.scale_min || scale > config.scale_max {
            return None;
        }
        let impulse = entry.impulse * scale;
        impulse.is_finite().then_some(impulse)
    }

    pub(crate) fn stage_joint(
        &mut self,
        joint: JointId,
        kind: Joint,
        dt: f64,
        rows: &[ConstraintRow],
    ) -> Result<(), CollisionError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(CollisionError::InvalidConfig);
        }
        let index = joint.index() as usize;
        ensure_slot(&mut self.current, index, rows.len())?;
        ensure_slot(&mut self.next, index, rows.len())?;
        let slot = self
            .next
            .get_mut(index)
            .ok_or(CollisionError::InvalidContact)?;
        if slot.joint.is_some() {
            return Err(CollisionError::InvalidContact);
        }
        slot.impulses
            .try_reserve(rows.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let kind = JointKind::from(kind);
        for row in rows {
            if !row.accumulated_impulse.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            let Some((axis_a, axis_b)) = row_axes(*row) else {
                continue;
            };
            slot.impulses.push(JointImpulse {
                joint,
                row_key: row.key,
                impulse: row.accumulated_impulse,
                dt,
                axis_a,
                axis_b,
                kind,
            });
        }
        slot.joint = Some(joint);
        Ok(())
    }

    pub(crate) fn commit(&mut self) {
        std::mem::swap(&mut self.current, &mut self.next);
        clear_slots(&mut self.next);
    }

    pub(crate) fn abort(&mut self) {
        clear_slots(&mut self.next);
    }

    pub(crate) fn invalidate(&mut self, joint: JointId) {
        for slot in [&mut self.current, &mut self.next] {
            if slot
                .get(joint.index() as usize)
                .is_some_and(|slot| slot.joint == Some(joint))
            {
                let entry = &mut slot[joint.index() as usize];
                entry.joint = None;
                entry.impulses.clear();
            }
        }
    }
}

fn ensure_slot(
    slots: &mut Vec<JointImpulseSlot>,
    index: usize,
    row_count: usize,
) -> Result<(), CollisionError> {
    if index >= slots.len() {
        let new_len = index
            .checked_add(1)
            .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        slots
            .try_reserve(new_len - slots.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        slots.resize_with(new_len, JointImpulseSlot::default);
    }
    let impulses = &mut slots
        .get_mut(index)
        .ok_or(CollisionError::InvalidContact)?
        .impulses;
    impulses
        .try_reserve(row_count.saturating_sub(impulses.len()))
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)
}

fn row_axes(row: ConstraintRow) -> Option<(Vec3, Vec3)> {
    let (axis_a, axis_b) = match row.dimension {
        super::row::RowDimension::Linear => (row.linear_a, row.linear_b),
        super::row::RowDimension::Angular => (row.angular_a, row.angular_b),
    };
    Some((axis_a.try_normalize()?, axis_b.try_normalize()?))
}

fn clear_slots(slots: &mut [JointImpulseSlot]) {
    for slot in slots {
        slot.joint = None;
        slot.impulses.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        joint::{FixedJoint, JointCommon, JointFrame},
        rigid_body::RigidBody,
        transform::Transform,
        world::PhysicsWorld,
    };

    fn joint() -> (JointId, Joint) {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let body_b = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let joint = Joint::Fixed(FixedJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        )));
        let id = world.add_joint(body_a, body_b, joint).unwrap();
        (id, joint)
    }

    fn row() -> ConstraintRow {
        ConstraintRow {
            key: 2,
            dimension: super::super::row::RowDimension::Linear,
            body_a: 0,
            body_b: 1,
            linear_a: -Vec3::X,
            angular_a: Vec3::ZERO,
            linear_b: Vec3::X,
            angular_b: Vec3::ZERO,
            error: 0.0,
            target_velocity: 0.0,
            force_bias: 0.0,
            position_enabled: true,
            compliance: 0.0,
            damping: 0.0,
            min_multiplier: f64::NEG_INFINITY,
            max_multiplier: f64::INFINITY,
            min_impulse: f64::NEG_INFINITY,
            max_impulse: f64::INFINITY,
            accumulated_multiplier: 0.0,
            accumulated_impulse: 2.0,
        }
    }

    fn stage(cache: &mut JointImpulseCache, id: JointId, kind: Joint, impulse: f64) {
        let mut row = row();
        row.accumulated_impulse = impulse;
        cache.begin_stage();
        cache.stage_joint(id, kind, 0.01, &[row]).unwrap();
    }

    #[test]
    fn warm_start_defaults_disabled_and_rejects_invalid_config() {
        let config = WarmStartConfig::default();
        assert!(!config.enabled);
        assert!(config.validate());
        assert!(
            !WarmStartConfig {
                scale_min: 0.0,
                ..config
            }
            .validate()
        );
        assert!(
            !WarmStartConfig {
                scale_max: 0.25,
                ..config
            }
            .validate()
        );
        assert!(
            !WarmStartConfig {
                minimum_axis_dot: 1.01,
                ..config
            }
            .validate()
        );
    }

    #[test]
    fn cache_publishes_only_on_commit_and_scales_impulse_with_dt() {
        let (id, kind) = joint();
        let mut cache = JointImpulseCache::new();
        let row = row();
        let config = WarmStartConfig::enabled();
        assert_eq!(cache.lookup(id, kind, row, 0.01, config), None);

        stage(&mut cache, id, kind, 2.0);
        assert_eq!(cache.lookup(id, kind, row, 0.01, config), None);
        cache.commit();

        assert_eq!(cache.lookup(id, kind, row, 0.02, config), Some(4.0));
        assert_eq!(cache.lookup(id, kind, row, 0.005, config), Some(1.0));
        assert_eq!(cache.lookup(id, kind, row, 0.03, config), None);
        assert_eq!(cache.lookup(id, kind, row, 0.004, config), None);
        let mut different_key = row;
        different_key.key += 1;
        assert_eq!(cache.lookup(id, kind, different_key, 0.01, config), None);
        assert_eq!(
            cache.lookup(id, kind, row, 0.01, WarmStartConfig::default()),
            None
        );
    }

    #[test]
    fn cache_rejects_axis_inversion_joint_kind_and_stale_generation() {
        let (id, kind) = joint();
        let mut cache = JointImpulseCache::new();
        stage(&mut cache, id, kind, 2.0);
        cache.commit();

        let (foreign_id, foreign_kind) = joint();
        assert_eq!(foreign_id.index(), id.index());
        assert_ne!(foreign_id, id);
        assert_eq!(
            cache.lookup(
                foreign_id,
                foreign_kind,
                row(),
                0.01,
                WarmStartConfig::enabled()
            ),
            None
        );

        let mut inverted = row();
        inverted.linear_a = -inverted.linear_a;
        inverted.linear_b = -inverted.linear_b;
        assert_eq!(
            cache.lookup(id, kind, inverted, 0.01, WarmStartConfig::enabled()),
            None
        );
        let replacement_kind = Joint::Ball(crate::joint::BallJoint::new(JointCommon::new(
            JointFrame::IDENTITY,
            JointFrame::IDENTITY,
        )));
        assert_eq!(
            cache.lookup(
                id,
                replacement_kind,
                row(),
                0.01,
                WarmStartConfig::enabled()
            ),
            None
        );

        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let body_b = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let old_id = world.add_joint(body_a, body_b, kind).unwrap();
        let mut generation_cache = JointImpulseCache::new();
        stage(&mut generation_cache, old_id, kind, 3.0);
        generation_cache.commit();
        world.remove_joint(old_id).unwrap();
        let replacement = world.add_joint(body_a, body_b, kind).unwrap();
        assert_eq!(old_id.index(), replacement.index());
        assert_ne!(old_id.generation(), replacement.generation());
        assert_eq!(
            generation_cache.lookup(replacement, kind, row(), 0.01, WarmStartConfig::enabled()),
            None
        );
    }

    #[test]
    fn abort_and_invalidate_preserve_or_remove_committed_impulses() {
        let (id, kind) = joint();
        let mut cache = JointImpulseCache::new();
        stage(&mut cache, id, kind, 2.0);
        cache.commit();

        stage(&mut cache, id, kind, 9.0);
        cache.abort();
        assert_eq!(
            cache.lookup(id, kind, row(), 0.01, WarmStartConfig::enabled()),
            Some(2.0)
        );

        cache.invalidate(id);
        assert_eq!(
            cache.lookup(id, kind, row(), 0.01, WarmStartConfig::enabled()),
            None
        );
    }

    #[test]
    fn cache_reuses_staged_capacity_after_warmup() {
        let (id, kind) = joint();
        let mut cache = JointImpulseCache::new();
        stage(&mut cache, id, kind, 2.0);
        cache.commit();
        stage(&mut cache, id, kind, 2.0);
        cache.commit();
        let capacities = (
            cache.current.capacity(),
            cache.next.capacity(),
            cache.current[0].impulses.capacity(),
            cache.next[0].impulses.capacity(),
        );

        for _ in 0..8 {
            stage(&mut cache, id, kind, 2.0);
            assert_eq!(
                (
                    cache.current.capacity(),
                    cache.next.capacity(),
                    cache.current[0].impulses.capacity(),
                    cache.next[0].impulses.capacity(),
                ),
                capacities
            );
            cache.commit();
        }
    }
}
