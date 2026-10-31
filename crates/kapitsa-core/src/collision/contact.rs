use crate::world::ColliderId;

/// A canonical, unordered pair of distinct colliders.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CollisionPair {
    pub a: ColliderId,
    pub b: ColliderId,
}

impl CollisionPair {
    /// Creates a pair ordered by the complete world and generational IDs.
    pub fn new(a: ColliderId, b: ColliderId) -> Option<Self> {
        match a.cmp(&b) {
            std::cmp::Ordering::Less => Some(Self { a, b }),
            std::cmp::Ordering::Greater => Some(Self { a: b, b: a }),
            std::cmp::Ordering::Equal => None,
        }
    }
}
