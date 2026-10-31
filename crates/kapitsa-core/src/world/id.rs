use crate::containers::DenseId;

/// Identifies a rigid body in the [`super::PhysicsWorld`] that created it.
///
/// `DenseId` does not contain an owner identifier; a `BodyId` must only be used
/// with its originating world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BodyId(pub(super) DenseId);

/// Identifies a collider in the [`super::PhysicsWorld`] that created it.
///
/// `DenseId` does not contain an owner identifier; a `ColliderId` must only be
/// used with its originating world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ColliderId(pub(super) DenseId);
