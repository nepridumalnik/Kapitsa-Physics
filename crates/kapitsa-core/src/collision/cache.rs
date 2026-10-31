use std::collections::HashMap;

use kapitsa_math::Vec3;

use super::{CollisionError, CollisionPair, manifold::ContactManifold};

const MINIMUM_NORMAL_DOT: f64 = 0.95;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct ContactKey {
    pair: CollisionPair,
    feature: u64,
}

#[derive(Clone, Copy)]
struct CachedImpulse {
    normal: Vec3,
    normal_impulse: f64,
    tangent_impulse: [f64; 2],
}

/// Transactional warm-start cache for regenerated contact manifolds.
pub(crate) struct ContactCache {
    current: HashMap<ContactKey, CachedImpulse>,
    next: HashMap<ContactKey, CachedImpulse>,
}

impl ContactCache {
    pub(crate) fn new() -> Self {
        Self {
            current: HashMap::new(),
            next: HashMap::new(),
        }
    }

    pub(crate) fn warm_start(&self, manifolds: &mut [ContactManifold]) {
        for manifold in manifolds {
            let pair = manifold.pair;
            for index in 0..manifold.len() {
                let Some(contact) = manifold.point_mut(index) else {
                    continue;
                };
                let key = ContactKey {
                    pair,
                    feature: contact.feature_id,
                };
                let Some(cached) = self.current.get(&key).copied() else {
                    continue;
                };
                let Some(normal) = contact.normal.try_normalize() else {
                    continue;
                };
                if normal.dot(cached.normal) < MINIMUM_NORMAL_DOT {
                    continue;
                }
                contact.normal_impulse = cached.normal_impulse;
                contact.tangent_impulse = cached.tangent_impulse;
            }
        }
    }

    pub(crate) fn stage(&mut self, manifolds: &[ContactManifold]) -> Result<(), CollisionError> {
        self.next.clear();
        let contact_count = manifolds.iter().try_fold(0_usize, |count, manifold| {
            count
                .checked_add(manifold.len())
                .ok_or(CollisionError::SpatialIndexCapacityExceeded)
        })?;
        self.next
            .try_reserve(contact_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for manifold in manifolds {
            for contact in manifold.points() {
                if !contact.normal.is_finite()
                    || !contact.normal_impulse.is_finite()
                    || contact.normal_impulse < 0.0
                    || contact
                        .tangent_impulse
                        .iter()
                        .any(|impulse| !impulse.is_finite())
                {
                    self.next.clear();
                    return Err(CollisionError::InvalidContact);
                }
                let Some(normal) = contact.normal.try_normalize() else {
                    self.next.clear();
                    return Err(CollisionError::InvalidContact);
                };
                self.next.insert(
                    ContactKey {
                        pair: manifold.pair,
                        feature: contact.feature_id,
                    },
                    CachedImpulse {
                        normal,
                        normal_impulse: contact.normal_impulse,
                        tangent_impulse: contact.tangent_impulse,
                    },
                );
            }
        }
        Ok(())
    }

    pub(crate) fn commit(&mut self) {
        std::mem::swap(&mut self.current, &mut self.next);
        self.next.clear();
    }

    pub(crate) fn abort(&mut self) {
        self.next.clear();
    }

    pub(crate) fn invalidate_collider(&mut self, collider: crate::world::ColliderId) {
        self.current
            .retain(|key, _| key.pair.a != collider && key.pair.b != collider);
        self.next
            .retain(|key, _| key.pair.a != collider && key.pair.b != collider);
    }

    pub(crate) fn clear(&mut self) {
        self.current.clear();
        self.next.clear();
    }

    #[cfg(test)]
    fn len(&self) -> usize {
        self.current.len()
    }
}

#[cfg(test)]
#[path = "cache_tests.rs"]
mod tests;
