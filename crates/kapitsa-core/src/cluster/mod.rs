//! Metadata for rigid assemblies made up of parts and welds.

mod part;
mod weld;

pub use part::{ClusterPart, ClusterPartDesc};
pub use weld::{Weld, WeldDesc, WeldStiffness, WeldStrength};

pub use crate::world::{ClusterError, PartId, WeldId};

/// A connected rigid assembly represented by a single body.
#[derive(Clone, Debug, PartialEq)]
pub struct RigidCluster {
    pub(crate) root: PartId,
    pub(crate) parts: Vec<PartId>,
    pub(crate) welds: Vec<WeldId>,
}

impl RigidCluster {
    pub fn root(&self) -> PartId {
        self.root
    }

    pub fn parts(&self) -> &[PartId] {
        &self.parts
    }

    pub fn welds(&self) -> &[WeldId] {
        &self.welds
    }

    pub fn part_count(&self) -> usize {
        self.parts.len()
    }

    pub fn weld_count(&self) -> usize {
        self.welds.len()
    }
}
