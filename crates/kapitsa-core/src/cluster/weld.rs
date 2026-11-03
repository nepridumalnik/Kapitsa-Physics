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
