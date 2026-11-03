use kapitsa_math::Vec3;

use crate::{
    collider::Collider,
    mass::MassProperties,
    transform::Transform,
    world::{BodyId, ColliderId},
};

/// Data used to prepare a part for insertion into a rigid cluster.
pub struct ClusterPartDesc {
    pub local_transform: Transform,
    pub mass: MassProperties,
    pub colliders: Vec<Collider>,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

/// A part and the body and colliders currently representing it.
#[derive(Clone, Debug, PartialEq)]
pub struct ClusterPart {
    pub(crate) body: BodyId,
    pub(crate) local_transform: Transform,
    pub(crate) mass: MassProperties,
    pub(crate) colliders: Vec<ColliderId>,
}

impl ClusterPart {
    pub fn body(&self) -> BodyId {
        self.body
    }

    pub fn local_transform(&self) -> Transform {
        self.local_transform
    }

    pub fn mass(&self) -> MassProperties {
        self.mass
    }

    pub fn colliders(&self) -> &[ColliderId] {
        &self.colliders
    }
}
