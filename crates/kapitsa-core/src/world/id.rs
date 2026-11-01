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

/// Identifies a persistent force source owned by a physics world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ForceSourceId {
    pub(super) world: u64,
    pub(super) dense: DenseId,
}

impl ForceSourceId {
    /// Returns the stable slot index within the owning world.
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

/// Identifies a joint in the world that created it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct JointId {
    pub(super) world: u64,
    pub(super) dense: DenseId,
}

impl JointId {
    /// Returns the stable slot index within the owning world.
    pub fn index(self) -> u32 {
        self.dense.index()
    }

    /// Returns the slot generation used to reject stale IDs after reuse.
    pub fn generation(self) -> u32 {
        self.dense.generation()
    }
}
