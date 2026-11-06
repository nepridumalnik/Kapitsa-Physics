use std::collections::HashMap;

use kapitsa_math::Vec3;

use super::{
    CollisionError, CollisionPair,
    manifold::ContactManifold,
    solver::{SolverManifold, tangent_basis},
};

const MINIMUM_NORMAL_DOT: f64 = 0.95;
// A large timestep change makes the previous step's impulse a poor estimate.
const MINIMUM_TIMESTEP_RATIO: f64 = 0.5;
const MAXIMUM_TIMESTEP_RATIO: f64 = 2.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct ContactKey {
    pair: CollisionPair,
    feature: u64,
}

#[derive(Clone, Copy)]
struct CachedImpulse {
    normal: Vec3,
    local_a: Vec3,
    local_b: Vec3,
    normal_impulse: f64,
    tangent_world: Vec3,
}

/// Transactional warm-start cache for regenerated contact manifolds.
pub(crate) struct ContactCache {
    current: HashMap<ContactKey, CachedImpulse>,
    next: HashMap<ContactKey, CachedImpulse>,
    current_dt: Option<f64>,
    next_dt: Option<f64>,
}

impl ContactCache {
    pub(crate) fn new() -> Self {
        Self {
            current: HashMap::new(),
            next: HashMap::new(),
            current_dt: None,
            next_dt: None,
        }
    }

    pub(crate) fn warm_start_solver(
        &self,
        manifolds: &mut [SolverManifold],
        dt: f64,
        contact_offset: f64,
    ) -> Result<(), CollisionError> {
        if !dt.is_finite() || dt <= 0.0 || !contact_offset.is_finite() || contact_offset < 0.0 {
            return Err(CollisionError::ArithmeticOverflow);
        }
        let impulse_scale = match self.current_dt {
            None => 1.0,
            Some(previous_dt) if previous_dt.is_finite() && previous_dt > 0.0 => {
                let ratio = dt / previous_dt;
                if !(MINIMUM_TIMESTEP_RATIO..=MAXIMUM_TIMESTEP_RATIO).contains(&ratio) {
                    return Ok(());
                }
                ratio
            }
            Some(_) => return Err(CollisionError::ArithmeticOverflow),
        };
        for manifold in manifolds {
            self.warm_start_manifold(manifold, impulse_scale, contact_offset)?;
        }
        Ok(())
    }

    fn warm_start_manifold(
        &self,
        solver_manifold: &mut SolverManifold,
        impulse_scale: f64,
        contact_offset: f64,
    ) -> Result<(), CollisionError> {
        let pair = solver_manifold.manifold.pair;
        for index in 0..solver_manifold.manifold.len() {
            let Some(contact) = solver_manifold.manifold.point_mut(index) else {
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
            // A feature can be reused for a different point, especially on
            // smooth shapes. Reject the impulse when either body-local anchor
            // has moved beyond the contact generation distance.
            let anchor_matches = |current: Vec3, previous: Vec3| {
                current
                    .checked_sub(previous)
                    .and_then(Vec3::checked_length)
                    .is_some_and(|distance| distance <= contact_offset)
            };
            if !anchor_matches(contact.local_a, cached.local_a)
                || !anchor_matches(contact.local_b, cached.local_b)
            {
                continue;
            }
            let (tangent_a, tangent_b) = tangent_basis(normal)?;
            let normal_impulse = cached.normal_impulse * impulse_scale;
            let tangent_world_cached = cached
                .tangent_world
                .checked_mul_scalar(impulse_scale)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            if !normal_impulse.is_finite() || normal_impulse < 0.0 {
                return Err(CollisionError::ArithmeticOverflow);
            }
            let normal_component = tangent_world_cached
                .checked_dot(normal)
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let tangent_world = tangent_world_cached
                .checked_sub(
                    normal
                        .checked_mul_scalar(normal_component)
                        .ok_or(CollisionError::ArithmeticOverflow)?,
                )
                .ok_or(CollisionError::ArithmeticOverflow)?;
            let mut tangent_impulse = [
                tangent_world
                    .checked_dot(tangent_a)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
                tangent_world
                    .checked_dot(tangent_b)
                    .ok_or(CollisionError::ArithmeticOverflow)?,
            ];
            let friction = solver_manifold
                .material
                .static_friction
                .max(solver_manifold.material.dynamic_friction);
            if !friction.is_finite() || friction < 0.0 {
                return Err(CollisionError::InvalidMaterial);
            }
            let limit = normal_impulse * friction;
            let magnitude = tangent_impulse[0].hypot(tangent_impulse[1]);
            if !limit.is_finite() || !magnitude.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            if magnitude > limit && magnitude > 0.0 {
                let scale = limit / magnitude;
                tangent_impulse[0] *= scale;
                tangent_impulse[1] *= scale;
            }
            contact.normal_impulse = normal_impulse;
            contact.tangent_impulse = tangent_impulse;
        }
        Ok(())
    }

    pub(crate) fn stage(
        &mut self,
        manifolds: &[ContactManifold],
        dt: f64,
    ) -> Result<(), CollisionError> {
        self.next.clear();
        self.next_dt = None;
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
                    || !contact.normal_impulse_increase_sum.is_finite()
                    || contact.normal_impulse_increase_sum < 0.0
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
                let (tangent_a, tangent_b) = match tangent_basis(normal) {
                    Ok(basis) => basis,
                    Err(error) => {
                        self.next.clear();
                        return Err(error);
                    }
                };
                let Some(tangent_world) = tangent_a
                    .checked_mul_scalar(contact.tangent_impulse[0])
                    .and_then(|impulse| {
                        tangent_b
                            .checked_mul_scalar(contact.tangent_impulse[1])
                            .and_then(|other| impulse.checked_add(other))
                    })
                else {
                    self.next.clear();
                    return Err(CollisionError::ArithmeticOverflow);
                };
                self.next.insert(
                    ContactKey {
                        pair: manifold.pair,
                        feature: contact.feature_id,
                    },
                    CachedImpulse {
                        normal,
                        local_a: contact.local_a,
                        local_b: contact.local_b,
                        normal_impulse: contact.normal_impulse,
                        tangent_world,
                    },
                );
            }
        }
        self.next_dt = Some(dt);
        Ok(())
    }

    pub(crate) fn commit(&mut self) {
        std::mem::swap(&mut self.current, &mut self.next);
        self.next.clear();
        self.current_dt = self.next_dt.take();
    }

    pub(crate) fn abort(&mut self) {
        self.next.clear();
        self.next_dt = None;
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
        self.current_dt = None;
        self.next_dt = None;
    }

    #[cfg(feature = "bench-internals")]
    pub(crate) fn retained_capacity_bytes(&self) -> usize {
        use std::mem::size_of;

        self.current.capacity() * size_of::<(ContactKey, CachedImpulse)>()
            + self.next.capacity() * size_of::<(ContactKey, CachedImpulse)>()
    }

    #[cfg(test)]
    fn len(&self) -> usize {
        self.current.len()
    }
}

#[cfg(test)]
#[path = "cache_tests.rs"]
mod tests;
