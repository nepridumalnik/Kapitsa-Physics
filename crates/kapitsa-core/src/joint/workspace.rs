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
    prepared_rows: usize,
    active_joint_index: usize,
    reactions: Vec<JointReaction>,
    break_events: Vec<JointBreakEvent>,
}

impl JointWorkspace {
    pub(crate) const fn new() -> Self {
        Self {
            rows: Vec::new(),
            row_joint_indices: Vec::new(),
            prepared_rows: 0,
            active_joint_index: 0,
            reactions: Vec::new(),
            break_events: Vec::new(),
        }
    }

    pub(crate) fn begin_step(&mut self) {
        self.prepared_rows = 0;
        self.active_joint_index = 0;
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
        self.prepared_rows = 0;
        self.active_joint_index = 0;
        self.reactions.clear();
        self.break_events.clear();
    }

    pub(crate) fn begin_prepare(&mut self) {
        self.prepared_rows = 0;
    }

    pub(crate) fn push(&mut self, mut row: ConstraintRow) {
        if let Some(previous) = self.rows.get(self.prepared_rows) {
            row.accumulated_multiplier = previous.accumulated_multiplier;
            row.accumulated_impulse = previous.accumulated_impulse;
            self.rows[self.prepared_rows] = row;
            self.row_joint_indices[self.prepared_rows] = self.active_joint_index;
        } else {
            self.rows.push(row);
            self.row_joint_indices.push(self.active_joint_index);
        }
        self.prepared_rows += 1;
    }

    pub(crate) fn finish_prepare(&mut self) {
        self.rows.truncate(self.prepared_rows);
        self.row_joint_indices.truncate(self.prepared_rows);
    }

    pub(crate) fn rows(&self) -> &[ConstraintRow] {
        &self.rows
    }

    pub(crate) fn rows_mut(&mut self) -> &mut [ConstraintRow] {
        &mut self.rows
    }

    pub(crate) fn set_joint_index(&mut self, index: usize) {
        self.active_joint_index = index;
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
            + self.reactions.capacity() * std::mem::size_of::<JointReaction>()
            + self.break_events.capacity() * std::mem::size_of::<JointBreakEvent>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_math::Vec3;

    #[test]
    fn clear_reuses_row_and_owner_capacity_for_many_joints() {
        let mut workspace = JointWorkspace::new();
        let add_rows = |workspace: &mut JointWorkspace| {
            for joint_index in 0..32 {
                workspace.set_joint_index(joint_index);
                workspace.push(ConstraintRow {
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
        };
        add_rows(&mut workspace);
        let row_capacity = workspace.rows.capacity();
        let owner_capacity = workspace.row_joint_indices.capacity();
        assert!(row_capacity >= 32);
        assert!(owner_capacity >= 32);

        workspace.clear();

        assert!(workspace.rows.is_empty());
        assert!(workspace.row_joint_indices.is_empty());
        assert_eq!(workspace.rows.capacity(), row_capacity);
        assert_eq!(workspace.row_joint_indices.capacity(), owner_capacity);

        add_rows(&mut workspace);
        assert_eq!(workspace.rows.capacity(), row_capacity);
        assert_eq!(workspace.row_joint_indices.capacity(), owner_capacity);
    }
}
