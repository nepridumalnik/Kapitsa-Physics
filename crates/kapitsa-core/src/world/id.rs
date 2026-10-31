use crate::containers::DenseId;

/// Identifies a rigid body in the [`super::PhysicsWorld`] that created it.
///
/// The opaque owner identifier prevents IDs from resolving in another world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BodyId {
    pub(super) world: u64,
    pub(super) dense: DenseId,
}

impl BodyId {
    /// Returns the stable slot index within the world that created this ID.
    pub fn index(self) -> u32 {
        self.dense.index()
    }

    /// Returns the slot generation used to reject stale IDs after reuse.
    pub fn generation(self) -> u32 {
        self.dense.generation()
    }
}

/// Identifies a collider in the [`super::PhysicsWorld`] that created it.
///
/// The opaque owner identifier prevents IDs from resolving in another world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ColliderId {
    pub(super) world: u64,
    pub(super) dense: DenseId,
}
