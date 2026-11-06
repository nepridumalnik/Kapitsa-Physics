//! Metadata for rigid assemblies made up of parts and welds.

pub(crate) mod mass;
#[allow(dead_code)]
pub(crate) mod motion;
mod part;
mod weld;

pub use part::{ClusterPart, ClusterPartDesc};
pub use weld::{Weld, WeldDesc, WeldStiffness, WeldStrength};

pub use crate::world::{ClusterError, PartId, WeldId};

/// Policy for editing a part's mass without implicitly modelling exhaust.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MassChangeMode {
    KeepVelocityField,
    ConserveRemainingMomentum,
}

/// Signed momentum carried into a body by matter and accompanying impulses.
/// Angular momentum is expressed about `world_origin` in world coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassTransfer {
    pub world_origin: kapitsa_math::Vec3,
    pub linear_impulse: kapitsa_math::Vec3,
    pub angular_impulse: kapitsa_math::Vec3,
}

impl MassTransfer {
    pub fn validate(self) -> Result<(), ClusterError> {
        if !self.world_origin.is_finite()
            || !self.linear_impulse.is_finite()
            || !self.angular_impulse.is_finite()
        {
            return Err(ClusterError::InvalidMotion);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{ClusterError, MassTransfer};
    use kapitsa_math::Vec3;

    #[test]
    fn mass_transfer_rejects_non_finite_components() {
        let valid = MassTransfer {
            world_origin: Vec3::ZERO,
            linear_impulse: Vec3::X,
            angular_impulse: -Vec3::Y,
        };
        assert_eq!(valid.validate(), Ok(()));
        let invalid = Vec3::new(f64::NAN, 0.0, 0.0);
        for transfer in [
            MassTransfer {
                world_origin: invalid,
                ..valid
            },
            MassTransfer {
                linear_impulse: invalid,
                ..valid
            },
            MassTransfer {
                angular_impulse: invalid,
                ..valid
            },
        ] {
            assert_eq!(transfer.validate(), Err(ClusterError::InvalidMotion));
        }
    }
}

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
