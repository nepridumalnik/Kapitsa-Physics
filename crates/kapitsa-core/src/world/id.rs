use crate::containers::DenseId;

/// Identifies a rigid body in the [`super::PhysicsWorld`] that created it.
///
/// `DenseId` does not contain an owner identifier; a `BodyId` must only be used
/// with its originating world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BodyId(pub(super) DenseId);

impl BodyId {
    /// Returns the stable slot index within the world that created this ID.
    pub fn index(self) -> u32 {
        self.0.index()
    }

    /// Returns the slot generation used to reject stale IDs after reuse.
    pub fn generation(self) -> u32 {
        self.0.generation()
    }
}

/// Identifies a collider in the [`super::PhysicsWorld`] that created it.
///
/// `DenseId` does not contain an owner identifier; a `ColliderId` must only be
/// used with its originating world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ColliderId(pub(super) DenseId);
