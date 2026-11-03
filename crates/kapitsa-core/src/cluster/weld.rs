use crate::{joint::JointFrame, world::PartId};

/// Thresholds governing whether a weld can break.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WeldStrength {
    Unbreakable,
    Breakable {
        max_force: Option<f64>,
        max_torque: Option<f64>,
    },
}

/// Linear and angular stiffness of a weld.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldStiffness {
    pub linear: f64,
    pub angular: f64,
}

/// Weld frames and mechanical properties.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldDesc {
    pub frame_a: JointFrame,
    pub frame_b: JointFrame,
    pub strength: WeldStrength,
    pub stiffness: WeldStiffness,
}

impl WeldDesc {
    pub(crate) fn validate(&self) -> Result<(), crate::world::ClusterError> {
        use crate::world::ClusterError;
        self.frame_a
            .validate()
            .map_err(|_| ClusterError::InvalidWeld)?;
        self.frame_b
            .validate()
            .map_err(|_| ClusterError::InvalidWeld)?;
        if !self.stiffness.linear.is_finite()
            || self.stiffness.linear <= 0.0
            || !self.stiffness.angular.is_finite()
            || self.stiffness.angular <= 0.0
        {
            return Err(ClusterError::InvalidWeld);
        }
        if let WeldStrength::Breakable {
            max_force,
            max_torque,
        } = self.strength
        {
            if max_force.is_none() && max_torque.is_none() {
                return Err(ClusterError::InvalidWeld);
            }
            if [max_force, max_torque]
                .into_iter()
                .flatten()
                .any(|limit| !limit.is_finite() || limit <= 0.0)
            {
                return Err(ClusterError::InvalidWeld);
            }
        }
        Ok(())
    }
}

/// A weld joining two cluster parts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Weld {
    pub(crate) part_a: PartId,
    pub(crate) part_b: PartId,
    pub(crate) desc: WeldDesc,
}

impl Weld {
    pub fn parts(&self) -> (PartId, PartId) {
        (self.part_a, self.part_b)
    }

    pub fn part_a(&self) -> PartId {
        self.part_a
    }

    pub fn part_b(&self) -> PartId {
        self.part_b
    }

    pub fn desc(&self) -> &WeldDesc {
        &self.desc
    }
}
