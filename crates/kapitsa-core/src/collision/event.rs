use kapitsa_math::Vec3;

use super::CollisionPair;

/// Change in the interaction state for a collider pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum CollisionEventKind {
    Begin,
    Persist,
    End,
    Impact,
}

/// Contact data associated with a collision event.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactInfo {
    pub point: Vec3,
    pub normal: Vec3,
    pub penetration: f64,
    pub normal_impulse: f64,
}

/// A collision lifecycle notification for one canonical collider pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollisionEvent {
    pub kind: CollisionEventKind,
    pub pair: CollisionPair,
    pub sensor: bool,
    pub contact: Option<ContactInfo>,
}
