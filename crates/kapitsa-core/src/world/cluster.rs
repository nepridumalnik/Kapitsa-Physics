use kapitsa_math::Vec3;
use std::collections::{HashMap, HashSet};

use super::{BodyId, ClusterError, ColliderId, ForceTarget, PartId, PhysicsWorld, WeldId};
use crate::{
    cluster::{
        ClusterPart, MassChangeMode, MassTransfer, RigidCluster, Weld,
        mass::aggregate_mass,
        motion::{transfer_motion, velocity_field},
    },
    mass::MassProperties,
    rigid_body::BodyType,
    transform::Transform,
};

impl PhysicsWorld {
    /// Converts an existing dynamic body into a one-part cluster.
    ///
    /// Existing colliders become owned by the identity-transform root part.
    /// Static and kinematic bodies can use [`Self::enable_cluster_with_mass`]
    /// because they do not carry mass properties in `RigidBody`.
    pub fn enable_cluster(&mut self, body: BodyId) -> Result<PartId, ClusterError> {
        let mass = self
            .body(body)
            .ok_or(ClusterError::InvalidBodyId)?
            .physics()
            .mass()
            .ok_or(ClusterError::InvalidMass)?;
        self.enable_cluster_with_mass(body, mass)
    }

    /// Converts an existing body into a one-part cluster with explicit root mass.
    pub fn enable_cluster_with_mass(
        &mut self,
        body: BodyId,
        mass: MassProperties,
    ) -> Result<PartId, ClusterError> {
        let body_entry = self
            .bodies
            .get(body.dense)
            .filter(|_| body.world == self.world_id)
            .ok_or(ClusterError::InvalidBodyId)?;
        if body_entry.cluster.is_some() {
            return Err(ClusterError::AlreadyClustered);
        }
        if body_entry.body.physics().body_type() == BodyType::Dynamic
            && body_entry.body.physics().mass() != Some(mass)
        {
            return Err(ClusterError::InvalidMass);
        }

        let mut colliders = Vec::new();
        let mut next = body_entry.first_collider;
        while let Some(collider) = next {
            let entry = self
                .colliders
                .get(collider.dense)
                .filter(|entry| entry.body == body)
                .ok_or(ClusterError::InvalidTopology)?;
            colliders.push(collider);
            next = entry.next;
        }
        colliders
            .try_reserve(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        self.parts
            .try_reserve_insertions(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;

        let dense = self
            .parts
            .insert(ClusterPart {
                body,
                local_transform: Transform::identity(),
                mass,
                colliders,
            })
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let part = PartId {
            world: self.world_id,
            dense,
        };
        let entry = self
            .bodies
            .get_mut(body.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        entry.cluster = Some(RigidCluster {
            root: part,
            parts: vec![part],
            welds: Vec::new(),
        });
        Ok(part)
    }

    /// Adds a part and welds it to an existing cluster part.
    ///
    /// The velocity fields in `desc` describe the new part before attachment.
    /// They must already agree with the receiving body's rigid velocity field;
    /// a different incoming state belongs to the explicit merge operation.
    pub fn add_cluster_part(
        &mut self,
        parent: PartId,
        mut desc: crate::cluster::ClusterPartDesc,
        weld_desc: crate::cluster::WeldDesc,
    ) -> Result<PartId, ClusterError> {
        weld_desc.validate()?;
        if !desc.local_transform.translation().is_finite()
            || !desc.local_transform.rotation().is_normalized()
            || !desc.linear_velocity.is_finite()
            || !desc.angular_velocity.is_finite()
        {
            return Err(ClusterError::InvalidMotion);
        }
        let body = self
            .cluster_part(parent)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
        if !cluster.parts().contains(&parent) {
            return Err(ClusterError::InvalidTopology);
        }
        let body_pose = self
            .body(body)
            .ok_or(ClusterError::InvalidBodyId)?
            .state()
            .transform();
        let parent_pose = body_pose
            .compose(
                &self
                    .cluster_part(parent)
                    .ok_or(ClusterError::InvalidPartId)?
                    .local_transform(),
            )
            .map_err(ClusterError::Transform)?;
        let part_pose = body_pose
            .compose(&desc.local_transform)
            .map_err(ClusterError::Transform)?;
        let anchor_a = parent_pose.transform_point(weld_desc.frame_a.local_anchor);
        let anchor_b = part_pose.transform_point(weld_desc.frame_b.local_anchor);
        let anchor_error = anchor_a
            .checked_sub(anchor_b)
            .and_then(Vec3::checked_length)
            .ok_or(ClusterError::ArithmeticOverflow)?;
        let anchor_scale = anchor_a
            .abs()
            .max_element()
            .max(anchor_b.abs().max_element())
            .max(1.0);
        if anchor_error > 128.0 * f64::EPSILON * anchor_scale {
            return Err(ClusterError::IncompatibleAnchors);
        }

        let original = *self.body(body).ok_or(ClusterError::InvalidBodyId)?;
        let mut values = Vec::new();
        values
            .try_reserve(cluster.parts().len().saturating_add(1))
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &id in cluster.parts() {
            let part = self.cluster_part(id).ok_or(ClusterError::InvalidTopology)?;
            values.push((part.mass(), part.local_transform()));
        }
        values.push((desc.mass, desc.local_transform));
        let aggregate = aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;

        let prepared_body = if original.physics().body_type() == BodyType::Dynamic {
            let old_center = original
                .world_center_of_mass()
                .map_err(ClusterError::Body)?
                .ok_or(ClusterError::InvalidMass)?;
            let new_center = body_pose.transform_point(aggregate.center_of_mass());
            if !new_center.is_finite() {
                return Err(ClusterError::ArithmeticOverflow);
            }
            let (velocity, omega) = velocity_field(
                old_center,
                original.state().linear_velocity(),
                original.state().angular_velocity(),
                new_center,
            )?;
            let force = original.forces().force();
            let torque = old_center
                .checked_sub(new_center)
                .and_then(|offset| offset.checked_cross(force))
                .and_then(|shift| original.forces().torque().checked_add(shift))
                .ok_or(ClusterError::ArithmeticOverflow)?;
            let mut prepared = original;
            prepared.set_mass(aggregate).map_err(ClusterError::Body)?;
            prepared
                .set_linear_velocity(velocity)
                .map_err(ClusterError::Body)?;
            prepared
                .set_angular_velocity(omega)
                .map_err(ClusterError::Body)?;
            prepared.apply_force(force).map_err(ClusterError::Body)?;
            prepared.apply_torque(torque).map_err(ClusterError::Body)?;
            Some(prepared)
        } else {
            None
        };

        if original.physics().body_type() == BodyType::Dynamic {
            let old_center = original
                .world_center_of_mass()
                .map_err(ClusterError::Body)?
                .ok_or(ClusterError::InvalidMass)?;
            let part_center = part_pose.transform_point(desc.mass.center_of_mass());
            let expected_linear = part_center
                .checked_sub(old_center)
                .and_then(|offset| original.state().angular_velocity().checked_cross(offset))
                .and_then(|relative| original.state().linear_velocity().checked_add(relative))
                .ok_or(ClusterError::ArithmeticOverflow)?;
            if (desc.linear_velocity - expected_linear).length()
                > 128.0 * f64::EPSILON * expected_linear.abs().max_element().max(1.0)
                || (desc.angular_velocity - original.state().angular_velocity()).length()
                    > 128.0
                        * f64::EPSILON
                        * original
                            .state()
                            .angular_velocity()
                            .abs()
                            .max_element()
                            .max(1.0)
            {
                return Err(ClusterError::IncompatibleMotion);
            }
        }

        self.parts
            .try_reserve_insertions(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        self.welds
            .try_reserve_insertions(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        if let Some(cluster) = self
            .bodies
            .get_mut(body.dense)
            .and_then(|entry| entry.cluster.as_mut())
        {
            cluster
                .parts
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
            cluster
                .welds
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        } else {
            return Err(ClusterError::NotClustered);
        }
        if desc.colliders.len() > 0 {
            self.colliders
                .try_reserve_insertions(desc.colliders.len())
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        }
        let part_dense = self
            .parts
            .insert(ClusterPart {
                body,
                local_transform: desc.local_transform,
                mass: desc.mass,
                colliders: Vec::new(),
            })
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let part = PartId {
            world: self.world_id,
            dense: part_dense,
        };
        let mut collider_ids = Vec::new();
        for mut collider in desc.colliders.drain(..) {
            let local_transform = desc
                .local_transform
                .compose(&collider.local_transform())
                .map_err(ClusterError::Transform)?;
            collider.set_local_transform(local_transform);
            match self.add_collider(body, collider) {
                Ok(collider_id) => collider_ids.push(collider_id),
                Err(error) => {
                    for collider_id in collider_ids {
                        self.remove_collider(collider_id);
                    }
                    self.parts.remove(part_dense);
                    return Err(map_cluster_world_error(error));
                }
            }
        }
        if let Some(entry) = self.parts.get_mut(part_dense) {
            entry.colliders = collider_ids;
        }

        let weld_dense = match self.welds.insert(Weld {
            part_a: parent,
            part_b: part,
            desc: weld_desc,
        }) {
            Ok(value) => value,
            Err(_) => {
                let colliders = self
                    .parts
                    .get(part_dense)
                    .map(|entry| entry.colliders.clone())
                    .unwrap_or_default();
                for collider_id in colliders {
                    self.remove_collider(collider_id);
                }
                self.parts.remove(part_dense);
                return Err(ClusterError::StorageCapacityExceeded);
            }
        };
        let weld = WeldId {
            world: self.world_id,
            dense: weld_dense,
        };

        if let Some(prepared) = prepared_body {
            self.invalidate_body_edit(body);
            if let Some(entry) = self.bodies.get_mut(body.dense) {
                entry.body = prepared;
            }
        }
        if let Some(cluster) = self
            .bodies
            .get_mut(body.dense)
            .and_then(|entry| entry.cluster.as_mut())
        {
            cluster.parts.push(part);
            cluster.welds.push(weld);
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        Ok(part)
    }

    /// Removes a part and its incident welds from a rigid cluster.
    ///
    /// The remaining parts must stay connected. Removing the final part is
    /// rejected so the owning body cannot outlive an empty cluster.
    pub fn remove_cluster_part(&mut self, part: PartId) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
        if !cluster.parts().contains(&part) {
            return Err(ClusterError::InvalidTopology);
        }
        if cluster.parts().len() == 1 {
            return Err(ClusterError::InvalidTopology);
        }

        let mut remaining_parts = Vec::new();
        remaining_parts
            .try_reserve(cluster.parts().len() - 1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &id in cluster.parts() {
            if id != part {
                remaining_parts.push(id);
            }
        }
        let remaining_set = remaining_parts.iter().copied().collect::<HashSet<_>>();

        let mut removed_welds = Vec::new();
        let mut remaining_welds = Vec::new();
        remaining_welds
            .try_reserve(cluster.welds().len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &weld_id in cluster.welds() {
            let weld = self.weld(weld_id).ok_or(ClusterError::InvalidTopology)?;
            let (a, b) = weld.parts();
            if a == part || b == part {
                removed_welds.push(weld_id);
            } else if remaining_set.contains(&a) && remaining_set.contains(&b) {
                remaining_welds.push(weld_id);
            } else {
                return Err(ClusterError::InvalidTopology);
            }
        }

        let mut adjacency = HashMap::new();
        for &id in &remaining_parts {
            adjacency.insert(id, Vec::new());
        }
        for &weld_id in &remaining_welds {
            let weld = self.weld(weld_id).ok_or(ClusterError::InvalidTopology)?;
            let (a, b) = weld.parts();
            adjacency
                .get_mut(&a)
                .ok_or(ClusterError::InvalidTopology)?
                .push(b);
            adjacency
                .get_mut(&b)
                .ok_or(ClusterError::InvalidTopology)?
                .push(a);
        }
        let mut visited = HashSet::new();
        let mut pending = vec![remaining_parts[0]];
        while let Some(id) = pending.pop() {
            if !visited.insert(id) {
                continue;
            }
            if let Some(neighbors) = adjacency.get(&id) {
                pending.extend(neighbors.iter().copied());
            }
        }
        if visited.len() != remaining_parts.len() {
            return Err(ClusterError::InvalidTopology);
        }

        let removed_entry = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidTopology)?;
        let removed_colliders = removed_entry.colliders().to_vec();
        for &collider in &removed_colliders {
            if self.collider_body(collider) != Some(body) {
                return Err(ClusterError::InvalidTopology);
            }
        }

        let mut values = Vec::new();
        values
            .try_reserve(remaining_parts.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &id in &remaining_parts {
            let entry = self.cluster_part(id).ok_or(ClusterError::InvalidTopology)?;
            values.push((entry.mass(), entry.local_transform()));
        }
        let aggregate = aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;
        let original = *self.body(body).ok_or(ClusterError::InvalidBodyId)?;
        let new_center = original
            .state()
            .transform()
            .transform_point(aggregate.center_of_mass());
        if !new_center.is_finite() {
            return Err(ClusterError::ArithmeticOverflow);
        }
        let mut prepared = original;
        if original.physics().body_type() == BodyType::Dynamic {
            let old_center = original
                .world_center_of_mass()
                .map_err(ClusterError::Body)?
                .ok_or(ClusterError::InvalidMass)?;
            let (velocity, omega) = velocity_field(
                old_center,
                original.state().linear_velocity(),
                original.state().angular_velocity(),
                new_center,
            )?;
            let force = original.forces().force();
            let torque = old_center
                .checked_sub(new_center)
                .and_then(|offset| offset.checked_cross(force))
                .and_then(|shift| original.forces().torque().checked_add(shift))
                .ok_or(ClusterError::ArithmeticOverflow)?;
            prepared.set_mass(aggregate).map_err(ClusterError::Body)?;
            prepared
                .set_linear_velocity(velocity)
                .map_err(ClusterError::Body)?;
            prepared
                .set_angular_velocity(omega)
                .map_err(ClusterError::Body)?;
            if original.physics().mass() != Some(aggregate) {
                prepared.apply_force(force).map_err(ClusterError::Body)?;
                prepared.apply_torque(torque).map_err(ClusterError::Body)?;
            }
        }

        let removed_sources = self
            .force_sources
            .iter()
            .filter_map(|(dense, source)| {
                (source.target == ForceTarget::Part(part)).then_some(dense)
            })
            .collect::<Vec<_>>();
        for dense in removed_sources {
            self.force_sources.remove(dense);
        }
        for collider in removed_colliders {
            self.remove_collider(collider)
                .ok_or(ClusterError::InvalidTopology)?;
        }
        for weld in &removed_welds {
            self.welds
                .remove(weld.dense)
                .ok_or(ClusterError::InvalidTopology)?;
        }
        self.parts
            .remove(part.dense)
            .ok_or(ClusterError::InvalidTopology)?;

        let root = remaining_parts
            .iter()
            .copied()
            .min()
            .ok_or(ClusterError::InvalidTopology)?;
        let cluster = self
            .bodies
            .get_mut(body.dense)
            .and_then(|entry| entry.cluster.as_mut())
            .ok_or(ClusterError::NotClustered)?;
        cluster.root = root;
        cluster.parts.retain(|id| *id != part);
        cluster.welds.retain(|id| !removed_welds.contains(id));
        if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.body = prepared;
        }
        self.invalidate_body_edit(body);
        Ok(())
    }

    /// Changes a part's mass while preserving the body's world-space velocity field.
    ///
    /// `ConserveRemainingMomentum` requires explicit impulses and is rejected here;
    /// use [`Self::set_part_mass_with_transfer`] for that operation.
    pub fn set_part_mass(
        &mut self,
        part: PartId,
        mass: MassProperties,
        mode: MassChangeMode,
    ) -> Result<(), ClusterError> {
        if mode == MassChangeMode::ConserveRemainingMomentum {
            return Err(ClusterError::InvalidMotion);
        }
        self.set_part_mass_inner(part, mass, None)
    }

    /// Changes a dynamic cluster's mass with explicitly supplied signed impulses.
    pub fn set_part_mass_with_transfer(
        &mut self,
        part: PartId,
        mass: MassProperties,
        transfer: MassTransfer,
    ) -> Result<(), ClusterError> {
        transfer.validate()?;
        self.set_part_mass_inner(part, mass, Some(transfer))
    }

    fn set_part_mass_inner(
        &mut self,
        part: PartId,
        mass: MassProperties,
        transfer: Option<MassTransfer>,
    ) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
        if !cluster.parts().contains(&part) {
            return Err(ClusterError::InvalidTopology);
        }
        let mut values = Vec::new();
        values
            .try_reserve(cluster.parts().len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &id in cluster.parts() {
            let entry = self.cluster_part(id).ok_or(ClusterError::InvalidTopology)?;
            values.push((
                if id == part { mass } else { entry.mass() },
                entry.local_transform(),
            ));
        }
        let aggregate = aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;
        let original = *self.body(body).ok_or(ClusterError::InvalidBodyId)?;
        let new_center = original
            .state()
            .transform()
            .transform_point(aggregate.center_of_mass());
        if !new_center.is_finite() {
            return Err(ClusterError::ArithmeticOverflow);
        }
        if transfer.is_none()
            && self
                .cluster_part(part)
                .is_some_and(|entry| entry.mass() == mass)
        {
            return Ok(());
        }
        if transfer.is_some() && original.physics().body_type() != BodyType::Dynamic {
            return Err(ClusterError::IncompatibleMotion);
        }
        let mut prepared = original;
        if original.physics().body_type() == BodyType::Dynamic {
            let old_center = original
                .world_center_of_mass()
                .map_err(ClusterError::Body)?
                .ok_or(ClusterError::InvalidMass)?;
            let (velocity, omega) = if let Some(transfer) = transfer {
                let old_mass = original.physics().mass().ok_or(ClusterError::InvalidMass)?;
                let pose = original.state().transform();
                transfer_motion(
                    old_mass,
                    pose,
                    original.state().linear_velocity(),
                    original.state().angular_velocity(),
                    aggregate,
                    pose,
                    transfer.world_origin,
                    transfer.linear_impulse,
                    transfer.angular_impulse,
                )?
            } else {
                velocity_field(
                    old_center,
                    original.state().linear_velocity(),
                    original.state().angular_velocity(),
                    new_center,
                )?
            };
            let force = original.forces().force();
            let torque = old_center
                .checked_sub(new_center)
                .and_then(|offset| offset.checked_cross(force))
                .and_then(|shift| original.forces().torque().checked_add(shift))
                .ok_or(ClusterError::ArithmeticOverflow)?;
            let aggregate_changed = original.physics().mass() != Some(aggregate);
            prepared.set_mass(aggregate).map_err(ClusterError::Body)?;
            prepared
                .set_linear_velocity(velocity)
                .map_err(ClusterError::Body)?;
            prepared
                .set_angular_velocity(omega)
                .map_err(ClusterError::Body)?;
            if aggregate_changed {
                prepared.apply_force(force).map_err(ClusterError::Body)?;
                prepared.apply_torque(torque).map_err(ClusterError::Body)?;
            }
        }
        let target = self
            .parts
            .get_mut(part.dense)
            .ok_or(ClusterError::InvalidTopology)?;
        let entry = self
            .bodies
            .get_mut(body.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        target.mass = mass;
        entry.body = prepared;
        let mut collider = entry.first_collider;
        while let Some(id) = collider {
            collider = self.colliders.get(id.dense).and_then(|entry| entry.next);
            self.collision_system.invalidate_collider_contacts(id);
        }
        self.invalidate_body_edit(body);
        Ok(())
    }

    #[allow(dead_code)]
    pub(crate) fn rebind_collider(
        &mut self,
        collider: ColliderId,
        body: BodyId,
        local_transform: Transform,
    ) -> Result<(), ClusterError> {
        if collider.world != self.world_id || body.world != self.world_id {
            return Err(ClusterError::InvalidTopology);
        }
        let target = self
            .bodies
            .get(body.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        let target_first = target.first_collider;
        let entry = self
            .colliders
            .get(collider.dense)
            .ok_or(ClusterError::InvalidTopology)?;
        let old = entry.body;
        let prev = entry.prev;
        let next = entry.next;
        if target.body.physics().body_type() == BodyType::Dynamic
            && !entry.collider.shape().is_convex()
        {
            return Err(ClusterError::UnsupportedGeometry);
        }
        let old_body = self
            .bodies
            .get(old.dense)
            .ok_or(ClusterError::InvalidTopology)?;
        if old.world != self.world_id
            || (prev.is_none() && old_body.first_collider != Some(collider))
            || prev.is_some_and(|id| {
                self.colliders
                    .get(id.dense)
                    .is_none_or(|other| other.body != old || other.next != Some(collider))
            })
            || next.is_some_and(|id| {
                self.colliders
                    .get(id.dense)
                    .is_none_or(|other| other.body != old || other.prev != Some(collider))
            })
            || target_first.is_some_and(|id| {
                self.colliders
                    .get(id.dense)
                    .is_none_or(|other| other.body != body || other.prev.is_some())
            })
        {
            return Err(ClusterError::InvalidTopology);
        }
        if old != body {
            if let Some(id) = prev {
                if let Some(other) = self.colliders.get_mut(id.dense) {
                    other.next = next;
                }
            } else if let Some(old_body) = self.bodies.get_mut(old.dense) {
                old_body.first_collider = next;
            }
            if let Some(id) = next
                && let Some(other) = self.colliders.get_mut(id.dense)
            {
                other.prev = prev;
            }
            if let Some(id) = target_first
                && let Some(other) = self.colliders.get_mut(id.dense)
            {
                other.prev = Some(collider);
            }
            if let Some(target) = self.bodies.get_mut(body.dense) {
                target.first_collider = Some(collider);
            }
        }
        if let Some(entry) = self.colliders.get_mut(collider.dense) {
            if old != body {
                entry.body = body;
                entry.prev = None;
                entry.next = target_first;
            }
            entry.collider.set_local_transform(local_transform);
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(collider);
        Ok(())
    }

    pub fn cluster(&self, body: BodyId) -> Option<&RigidCluster> {
        if body.world != self.world_id {
            return None;
        }
        self.bodies.get(body.dense)?.cluster.as_ref()
    }

    pub fn cluster_part(&self, part: PartId) -> Option<&ClusterPart> {
        if part.world != self.world_id {
            return None;
        }
        self.parts.get(part.dense)
    }

    pub fn weld(&self, weld: WeldId) -> Option<&Weld> {
        if weld.world != self.world_id {
            return None;
        }
        self.welds.get(weld.dense)
    }

    pub fn part_body(&self, part: PartId) -> Option<BodyId> {
        Some(self.cluster_part(part)?.body())
    }

    pub fn validate_clusters(&self) -> Result<(), ClusterError> {
        let mut all_parts = HashSet::new();
        let mut all_welds = HashSet::new();
        let mut all_colliders = HashSet::new();
        for (dense, entry) in self.bodies.iter() {
            let Some(cluster) = &entry.cluster else {
                continue;
            };
            let body = BodyId {
                world: self.world_id,
                dense,
            };
            let mut adjacency = HashMap::new();
            for &part_id in cluster.parts() {
                let part = self
                    .cluster_part(part_id)
                    .ok_or(ClusterError::InvalidTopology)?;
                if part.body() != body || !all_parts.insert(part_id) {
                    return Err(ClusterError::InvalidTopology);
                }
                adjacency.insert(part_id, Vec::new());
                for &collider in part.colliders() {
                    if self.collider_body(collider) != Some(body) || !all_colliders.insert(collider)
                    {
                        return Err(ClusterError::InvalidTopology);
                    }
                }
            }
            if !adjacency.contains_key(&cluster.root()) {
                return Err(ClusterError::InvalidTopology);
            }
            for &weld_id in cluster.welds() {
                let weld = self.weld(weld_id).ok_or(ClusterError::InvalidTopology)?;
                weld.desc().validate()?;
                let (a, b) = weld.parts();
                if a == b
                    || !all_welds.insert(weld_id)
                    || !adjacency.contains_key(&a)
                    || !adjacency.contains_key(&b)
                {
                    return Err(ClusterError::InvalidTopology);
                }
                if let Some(edges) = adjacency.get_mut(&a) {
                    edges.push(b);
                }
                if let Some(edges) = adjacency.get_mut(&b) {
                    edges.push(a);
                }
            }
            let mut visited = HashSet::new();
            let mut stack = vec![cluster.root()];
            while let Some(part) = stack.pop() {
                if visited.insert(part)
                    && let Some(neighbors) = adjacency.get(&part)
                {
                    stack.extend(neighbors.iter().copied());
                }
            }
            if visited.len() != adjacency.len() {
                return Err(ClusterError::InvalidTopology);
            }
            let mut collider = entry.first_collider;
            let mut previous = None;
            let mut linked_colliders = HashSet::new();
            while let Some(id) = collider {
                if !all_colliders.contains(&id) || !linked_colliders.insert(id) {
                    return Err(ClusterError::InvalidTopology);
                }
                let collider_entry = self
                    .colliders
                    .get(id.dense)
                    .ok_or(ClusterError::InvalidTopology)?;
                if collider_entry.body != body || collider_entry.prev != previous {
                    return Err(ClusterError::InvalidTopology);
                }
                previous = Some(id);
                collider = collider_entry.next;
            }
            if adjacency
                .keys()
                .filter_map(|id| self.cluster_part(*id))
                .map(|part| part.colliders().len())
                .sum::<usize>()
                != linked_colliders.len()
            {
                return Err(ClusterError::InvalidTopology);
            }
        }
        if all_parts.len() != self.parts.len() || all_welds.len() != self.welds.len() {
            return Err(ClusterError::InvalidTopology);
        }
        Ok(())
    }
}

fn map_cluster_world_error(error: super::WorldError) -> ClusterError {
    match error {
        super::WorldError::Collision(error) => ClusterError::Collision(error),
        super::WorldError::DensePoolCapacityExceeded
        | super::WorldError::DensePoolIndexOverflow
        | super::WorldError::DensePoolInvalidFreeList
        | super::WorldError::WorldIdExhausted => ClusterError::StorageCapacityExceeded,
        super::WorldError::InvalidBodyId => ClusterError::InvalidBodyId,
        _ => ClusterError::InvalidTopology,
    }
}

#[cfg(test)]
#[path = "cluster_tests.rs"]
mod tests;
