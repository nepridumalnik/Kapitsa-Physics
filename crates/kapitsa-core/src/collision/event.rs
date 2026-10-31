use kapitsa_math::Vec3;

use super::CollisionError;
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CollisionInteraction {
    pub(crate) pair: CollisionPair,
    pub(crate) sensor: bool,
    pub(crate) contact: Option<ContactInfo>,
    pub(crate) impact: bool,
}

#[derive(Clone, Copy)]
struct ActivePair {
    pair: CollisionPair,
    sensor: bool,
}

/// Transactional lifecycle state, committed alongside the world step.
#[allow(dead_code)]
pub(crate) struct CollisionLifecycle {
    active: Vec<ActivePair>,
    next: Vec<ActivePair>,
}

#[allow(dead_code)]
impl CollisionLifecycle {
    pub(crate) const fn new() -> Self {
        Self {
            active: Vec::new(),
            next: Vec::new(),
        }
    }

    pub(crate) fn stage(
        &mut self,
        interactions: &mut [CollisionInteraction],
        events: &mut Vec<CollisionEvent>,
    ) -> Result<(), CollisionError> {
        interactions.sort_by_key(|interaction| interaction.pair);
        self.next.clear();
        events.clear();
        if interactions
            .windows(2)
            .any(|pair| pair[0].pair == pair[1].pair)
        {
            return Err(CollisionError::InvalidContact);
        }
        for interaction in interactions.iter().copied() {
            if interaction.impact
                && (interaction.sensor
                    || interaction
                        .contact
                        .is_none_or(|contact| contact.normal_impulse <= 0.0))
            {
                return Err(CollisionError::InvalidContact);
            }
            if let Some(contact) = interaction.contact
                && (!contact.point.is_finite()
                    || !contact.normal.is_finite()
                    || !contact.penetration.is_finite()
                    || contact.penetration < 0.0
                    || !contact.normal_impulse.is_finite()
                    || contact.normal_impulse < 0.0
                    || contact.normal.try_normalize().is_none())
            {
                return Err(CollisionError::InvalidContact);
            }
        }
        self.next
            .try_reserve(interactions.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let event_capacity = self
            .active
            .len()
            .checked_add(
                interactions
                    .len()
                    .checked_mul(2)
                    .ok_or(CollisionError::SpatialIndexCapacityExceeded)?,
            )
            .ok_or(CollisionError::SpatialIndexCapacityExceeded)?;
        events
            .try_reserve(event_capacity)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;

        let mut old_index = 0;
        let mut new_index = 0;
        while old_index < self.active.len() || new_index < interactions.len() {
            let old = self.active.get(old_index).copied();
            let current = interactions.get(new_index).copied();
            match (old, current) {
                (Some(old), Some(current)) if old.pair == current.pair => {
                    self.push_current(current, CollisionEventKind::Persist, events);
                    old_index += 1;
                    new_index += 1;
                }
                (Some(old), Some(current)) if old.pair < current.pair => {
                    events.push(CollisionEvent {
                        kind: CollisionEventKind::End,
                        pair: old.pair,
                        sensor: old.sensor,
                        contact: None,
                    });
                    old_index += 1;
                }
                (Some(_), Some(current)) => {
                    self.push_current(current, CollisionEventKind::Begin, events);
                    new_index += 1;
                }
                (Some(old), None) => {
                    events.push(CollisionEvent {
                        kind: CollisionEventKind::End,
                        pair: old.pair,
                        sensor: old.sensor,
                        contact: None,
                    });
                    old_index += 1;
                }
                (None, Some(current)) => {
                    self.push_current(current, CollisionEventKind::Begin, events);
                    new_index += 1;
                }
                (None, None) => break,
            }
        }
        events.sort_by(|left, right| left.kind.cmp(&right.kind).then(left.pair.cmp(&right.pair)));
        Ok(())
    }

    fn push_current(
        &mut self,
        current: CollisionInteraction,
        kind: CollisionEventKind,
        events: &mut Vec<CollisionEvent>,
    ) {
        self.next.push(ActivePair {
            pair: current.pair,
            sensor: current.sensor,
        });
        events.push(CollisionEvent {
            kind,
            pair: current.pair,
            sensor: current.sensor,
            contact: current.contact,
        });
        if current.impact {
            events.push(CollisionEvent {
                kind: CollisionEventKind::Impact,
                pair: current.pair,
                sensor: current.sensor,
                contact: current.contact,
            });
        }
    }

    pub(crate) fn commit(&mut self) {
        std::mem::swap(&mut self.active, &mut self.next);
        self.next.clear();
    }

    pub(crate) fn abort(&mut self) {
        self.next.clear();
    }
}

#[cfg(test)]
#[path = "event_tests.rs"]
mod tests;
