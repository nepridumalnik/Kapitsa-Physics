use super::{JointBreakEvent, row::ConstraintRow, solver::JointConstraint};
use crate::collision::CollisionError;
use kapitsa_math::Vec3;

#[derive(Clone, Copy, Debug)]
struct JointReaction {
    id: crate::world::JointId,
    force_impulse: Vec3,
    torque_impulse: Vec3,
}

/// Reusable rows for one solver island, retained between physics steps.
#[derive(Default)]
pub(crate) struct JointWorkspace {
    rows: Vec<ConstraintRow>,
    row_joint_indices: Vec<usize>,
    row_joint_ids: Vec<Option<crate::world::JointId>>,
    preparing_rows: Vec<ConstraintRow>,
    preparing_joint_indices: Vec<usize>,
    preparing_joint_ids: Vec<Option<crate::world::JointId>>,
    prepared_rows: usize,
    active_joint_index: usize,
    active_joint_id: Option<crate::world::JointId>,
    previous_row_start: usize,
    previous_row_end: usize,
    reactions: Vec<JointReaction>,
    break_events: Vec<JointBreakEvent>,
}

impl JointWorkspace {
    pub(crate) const fn new() -> Self {
        Self {
            rows: Vec::new(),
            row_joint_indices: Vec::new(),
            row_joint_ids: Vec::new(),
            preparing_rows: Vec::new(),
            preparing_joint_indices: Vec::new(),
            preparing_joint_ids: Vec::new(),
            prepared_rows: 0,
            active_joint_index: 0,
            active_joint_id: None,
            previous_row_start: 0,
            previous_row_end: 0,
            reactions: Vec::new(),
            break_events: Vec::new(),
        }
    }

    pub(crate) fn begin_step(&mut self) {
        self.prepared_rows = 0;
        self.active_joint_index = 0;
        self.active_joint_id = None;
        self.previous_row_start = 0;
        self.previous_row_end = 0;
        self.reactions.clear();
        self.break_events.clear();
        for row in &mut self.rows {
            row.accumulated_multiplier = 0.0;
            row.accumulated_impulse = 0.0;
        }
    }

    pub(crate) fn clear(&mut self) {
        self.rows.clear();
        self.row_joint_indices.clear();
        self.row_joint_ids.clear();
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
        self.prepared_rows = 0;
        self.active_joint_index = 0;
        self.active_joint_id = None;
        self.previous_row_start = 0;
        self.previous_row_end = 0;
        self.reactions.clear();
        self.break_events.clear();
    }

    pub(crate) fn begin_prepare(&mut self) {
        self.prepared_rows = 0;
        self.active_joint_index = 0;
        self.active_joint_id = None;
        self.previous_row_start = 0;
        self.previous_row_end = self.rows.len();
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
    }

    pub(crate) fn push(&mut self, mut row: ConstraintRow) {
        let previous = (self.previous_row_start..self.previous_row_end).find_map(|index| {
            (self.row_joint_indices.get(index) == Some(&self.active_joint_index)
                && self.row_joint_ids.get(index) == Some(&self.active_joint_id)
                && self
                    .rows
                    .get(index)
                    .is_some_and(|previous| previous.key == row.key))
            .then(|| self.rows[index])
        });
        if let Some(previous) = previous {
            row.accumulated_multiplier = previous.accumulated_multiplier;
            row.accumulated_impulse = previous.accumulated_impulse;
        }
        self.preparing_rows.push(row);
        self.preparing_joint_indices.push(self.active_joint_index);
        self.preparing_joint_ids.push(self.active_joint_id);
        self.prepared_rows += 1;
    }

    pub(crate) fn finish_prepare(&mut self) {
        std::mem::swap(&mut self.rows, &mut self.preparing_rows);
        std::mem::swap(
            &mut self.row_joint_indices,
            &mut self.preparing_joint_indices,
        );
        std::mem::swap(&mut self.row_joint_ids, &mut self.preparing_joint_ids);
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
    }

    pub(crate) fn begin_joint_prepare(
        &mut self,
        joint_index: usize,
        joint_id: crate::world::JointId,
    ) {
        let start = self
            .row_joint_indices
            .partition_point(|&owner| owner < joint_index);
        let end = self
            .row_joint_indices
            .partition_point(|&owner| owner <= joint_index);
        self.previous_row_start = start;
        self.previous_row_end = end;
        self.prepared_rows = 0;
        self.active_joint_index = joint_index;
        self.active_joint_id = Some(joint_id);
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
    }

    pub(crate) fn prepared_rows_mut(&mut self) -> &mut [ConstraintRow] {
        &mut self.preparing_rows
    }

    pub(crate) fn finish_joint_prepare(&mut self) -> Result<(), CollisionError> {
        let joint_id = self.active_joint_id.ok_or(CollisionError::InvalidContact)?;
        let new_count = self.preparing_rows.len();
        let old_count = self.previous_row_end - self.previous_row_start;
        if new_count > old_count {
            let additional = new_count - old_count;
            self.rows
                .try_reserve(additional)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.row_joint_indices
                .try_reserve(additional)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            self.row_joint_ids
                .try_reserve(additional)
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        }
        self.rows.splice(
            self.previous_row_start..self.previous_row_end,
            self.preparing_rows.iter().copied(),
        );
        self.row_joint_indices.splice(
            self.previous_row_start..self.previous_row_end,
            std::iter::repeat(self.active_joint_index).take(new_count),
        );
        self.row_joint_ids.splice(
            self.previous_row_start..self.previous_row_end,
            std::iter::repeat(Some(joint_id)).take(new_count),
        );
        self.preparing_rows.clear();
        self.preparing_joint_indices.clear();
        self.preparing_joint_ids.clear();
        Ok(())
    }

    pub(crate) fn rows(&self) -> &[ConstraintRow] {
        &self.rows
    }

    pub(crate) fn rows_mut(&mut self) -> &mut [ConstraintRow] {
        &mut self.rows
    }

    pub(crate) fn set_joint_index(&mut self, index: usize, id: crate::world::JointId) {
        self.previous_row_start = self
            .row_joint_indices
            .partition_point(|&owner| owner < index);
        self.previous_row_end = self
            .row_joint_indices
            .partition_point(|&owner| owner <= index);
        self.active_joint_index = index;
        self.active_joint_id = Some(id);
    }

    pub(crate) fn collect_break_events(
        &mut self,
        joints: &[JointConstraint],
        dt: f64,
    ) -> Result<(), CollisionError> {
        self.break_events.clear();
        if !joints.iter().any(|joint| {
            let common = joint.joint.common();
            common.enabled && (common.break_force.is_some() || common.break_torque.is_some())
        }) {
            return Ok(());
        }

        self.reactions.clear();
        self.reactions
            .try_reserve(joints.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.break_events
            .try_reserve(joints.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.reactions
            .extend(joints.iter().map(|joint| JointReaction {
                id: joint.id,
                force_impulse: Vec3::ZERO,
                torque_impulse: Vec3::ZERO,
            }));

        for (row, joint_index) in self.rows.iter().zip(&self.row_joint_indices) {
            let reaction = self
                .reactions
                .get_mut(*joint_index)
                .ok_or(CollisionError::InvalidContact)?;
            if row.linear_a != Vec3::ZERO || row.linear_b != Vec3::ZERO {
                reaction.force_impulse += row.linear_a * row.accumulated_impulse;
            } else {
                reaction.torque_impulse += row.angular_a * row.accumulated_impulse;
            }
            if !reaction.force_impulse.is_finite() || !reaction.torque_impulse.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
        }

        for (joint, reaction) in joints.iter().zip(&self.reactions) {
            let common = joint.joint.common();
            if !common.enabled || (common.break_force.is_none() && common.break_torque.is_none()) {
                continue;
            }
            let force = reaction.force_impulse.length() / dt;
            let torque = reaction.torque_impulse.length() / dt;
            if !force.is_finite() || !torque.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            if common
                .break_force
                .is_some_and(|threshold| force > threshold)
                || common
                    .break_torque
                    .is_some_and(|threshold| torque > threshold)
            {
                self.break_events
                    .push(JointBreakEvent::new(reaction.id, force, torque));
            }
        }
        Ok(())
    }

    pub(crate) fn break_events(&self) -> &[JointBreakEvent] {
        &self.break_events
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        self.rows.capacity() * std::mem::size_of::<ConstraintRow>()
            + self.row_joint_indices.capacity() * std::mem::size_of::<usize>()
            + self.row_joint_ids.capacity() * std::mem::size_of::<Option<crate::world::JointId>>()
            + self.preparing_rows.capacity() * std::mem::size_of::<ConstraintRow>()
            + self.preparing_joint_indices.capacity() * std::mem::size_of::<usize>()
            + self.preparing_joint_ids.capacity()
                * std::mem::size_of::<Option<crate::world::JointId>>()
            + self.reactions.capacity() * std::mem::size_of::<JointReaction>()
            + self.break_events.capacity() * std::mem::size_of::<JointBreakEvent>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        joint::{FixedJoint, Joint, JointCommon, JointFrame},
        rigid_body::RigidBody,
        transform::Transform,
        world::PhysicsWorld,
    };
    use kapitsa_math::Vec3;

    fn test_joint_id() -> crate::world::JointId {
        let mut world = PhysicsWorld::new();
        let body_a = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let body_b = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        world
            .add_joint(
                body_a,
                body_b,
                Joint::Fixed(FixedJoint::new(JointCommon::new(
                    JointFrame::IDENTITY,
                    JointFrame::IDENTITY,
                ))),
            )
            .unwrap()
    }

    fn row(key: u8) -> ConstraintRow {
        ConstraintRow {
            key,
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
            accumulated_impulse: 0.0,
        }
    }

    #[test]
    fn clear_reuses_row_and_owner_capacity_for_many_joints() {
        let mut workspace = JointWorkspace::new();
        let joint_id = test_joint_id();
        let add_rows = |workspace: &mut JointWorkspace| {
            workspace.begin_prepare();
            for joint_index in 0..32 {
                workspace.set_joint_index(joint_index, joint_id);
                workspace.push(ConstraintRow {
                    key: joint_index as u8,
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
                    accumulated_impulse: 0.0,
                });
            }
            workspace.finish_prepare();
        };
        add_rows(&mut workspace);
        let row_capacity = workspace.rows.capacity();
        let owner_capacity = workspace.row_joint_indices.capacity();
        let id_capacity = workspace.row_joint_ids.capacity();
        assert!(row_capacity >= 32);
        assert!(owner_capacity >= 32);
        assert!(id_capacity >= 32);

        workspace.clear();

        assert!(workspace.rows.is_empty());
        assert!(workspace.row_joint_indices.is_empty());
        assert!(workspace.row_joint_ids.is_empty());
        assert_eq!(workspace.rows.capacity(), row_capacity);
        assert_eq!(workspace.row_joint_indices.capacity(), owner_capacity);
        assert_eq!(workspace.row_joint_ids.capacity(), id_capacity);

        add_rows(&mut workspace);
        assert_eq!(workspace.rows.capacity(), row_capacity);
        assert_eq!(workspace.row_joint_indices.capacity(), owner_capacity);
        assert_eq!(workspace.row_joint_ids.capacity(), id_capacity);
    }

    #[test]
    fn multipliers_follow_joint_and_row_keys_and_reset_at_step_boundaries() {
        let mut workspace = JointWorkspace::new();
        let first_joint = test_joint_id();
        let replacement_joint = test_joint_id();

        workspace.begin_prepare();
        workspace.set_joint_index(0, first_joint);
        workspace.push(row(4));
        workspace.finish_prepare();
        workspace.rows_mut()[0].accumulated_multiplier = 3.0;
        workspace.rows_mut()[0].accumulated_impulse = 7.0;

        workspace.begin_prepare();
        workspace.set_joint_index(0, replacement_joint);
        workspace.push(row(4));
        workspace.finish_prepare();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
        assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);

        workspace.rows_mut()[0].accumulated_multiplier = 5.0;
        workspace.rows_mut()[0].accumulated_impulse = 11.0;
        workspace.begin_prepare();
        workspace.set_joint_index(0, replacement_joint);
        workspace.push(row(5));
        workspace.finish_prepare();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
        assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);

        workspace.rows_mut()[0].accumulated_multiplier = 13.0;
        workspace.rows_mut()[0].accumulated_impulse = 17.0;
        workspace.begin_prepare();
        workspace.set_joint_index(0, replacement_joint);
        workspace.push(row(5));
        workspace.finish_prepare();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 13.0);
        assert_eq!(workspace.rows()[0].accumulated_impulse, 17.0);

        workspace.begin_step();
        assert_eq!(workspace.rows()[0].accumulated_multiplier, 0.0);
        assert_eq!(workspace.rows()[0].accumulated_impulse, 0.0);
    }
}
