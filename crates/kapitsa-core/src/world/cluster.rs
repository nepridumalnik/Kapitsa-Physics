use kapitsa_math::Vec3;
use std::collections::{HashMap, HashSet};

use super::{
    BodyEntry, BodyId, ClusterError, ColliderId, ForceTarget, JointId, JointTarget, PartId,
    PhysicsWorld, WeldId,
};
use crate::{
    cluster::{
        ClusterPart, MassChangeMode, MassTransfer, RigidCluster, Weld,
        mass::aggregate_mass,
        motion::{transfer_motion, velocity_field, world_properties},
    },
    mass::MassProperties,
    rigid_body::BodyType,
    transform::Transform,
};

impl PhysicsWorld {
    /// Adds a collider owned by a cluster part.
    pub fn add_part_collider(
        &mut self,
        part: PartId,
        mut collider: crate::collider::Collider,
    ) -> Result<ColliderId, ClusterError> {
        let part_entry = self.cluster_part(part).ok_or(ClusterError::InvalidPartId)?;
        let body = part_entry.body();
        let local_transform = part_entry
            .local_transform()
            .compose(&collider.local_transform())
            .map_err(ClusterError::Transform)?;
        let colliders = part_entry.colliders().len();
        self.parts
            .get_mut(part.dense)
            .ok_or(ClusterError::InvalidPartId)?
            .colliders
            .try_reserve(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        collider.set_local_transform(local_transform);
        let id = self
            .add_collider_internal(body, collider)
            .map_err(map_cluster_world_error)?;
        if let Some(entry) = self.parts.get_mut(part.dense) {
            if entry.colliders.len() != colliders {
                return Err(ClusterError::InvalidTopology);
            }
            entry.colliders.push(id);
        }
        Ok(id)
    }

    /// Removes a collider from its owning cluster part.
    pub fn remove_part_collider(
        &mut self,
        id: ColliderId,
    ) -> Result<crate::collider::Collider, ClusterError> {
        let body = self.collider_body(id).ok_or(ClusterError::InvalidPartId)?;
        let part = self
            .cluster(body)
            .ok_or(ClusterError::NotClustered)?
            .parts()
            .iter()
            .copied()
            .find(|part| {
                self.cluster_part(*part)
                    .is_some_and(|entry| entry.colliders().contains(&id))
            })
            .ok_or(ClusterError::InvalidTopology)?;
        let collider = self
            .remove_collider_internal(id)
            .ok_or(ClusterError::InvalidTopology)?;
        let entry = self
            .parts
            .get_mut(part.dense)
            .ok_or(ClusterError::InvalidTopology)?;
        if let Some(index) = entry.colliders.iter().position(|value| *value == id) {
            entry.colliders.remove(index);
            Ok(collider)
        } else {
            Err(ClusterError::InvalidTopology)
        }
    }

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
        let parent_transform = self
            .cluster_part(parent)
            .ok_or(ClusterError::InvalidPartId)?
            .local_transform();
        let part_pose = body_pose
            .compose(&desc.local_transform)
            .map_err(ClusterError::Transform)?;
        validate_weld_frames(parent_transform, desc.local_transform, weld_desc)?;

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
            match self.add_collider_internal(body, collider) {
                Ok(collider_id) => collider_ids.push(collider_id),
                Err(error) => {
                    for collider_id in collider_ids {
                        self.remove_collider_internal(collider_id);
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
                    self.remove_collider_internal(collider_id);
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
    pub fn destroy_cluster_part(&mut self, part: PartId) -> Result<(), ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
        let original_root = cluster.root();
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
        let removed_joints = self
            .joints
            .iter()
            .filter_map(|(dense, entry)| {
                (entry.target_a == super::JointTarget::Part(part)
                    || entry.target_b == super::JointTarget::Part(part))
                .then_some(dense)
            })
            .collect::<Vec<_>>();
        for dense in removed_joints {
            self.joints.remove(dense);
            self.collision_system.invalidate_joint_warm_start(JointId {
                world: self.world_id,
                dense,
            });
        }
        for collider in removed_colliders {
            self.remove_collider_internal(collider)
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

        let root = if original_root == part {
            remaining_parts
                .iter()
                .copied()
                .min()
                .ok_or(ClusterError::InvalidTopology)?
        } else {
            original_root
        };
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

    /// Backward-compatible alias for [`Self::destroy_cluster_part`].
    pub fn remove_cluster_part(&mut self, part: PartId) -> Result<(), ClusterError> {
        self.destroy_cluster_part(part)
    }

    /// Removes one weld and splits the graph when that edge disconnects it.
    pub fn break_weld(&mut self, weld: WeldId) -> Result<Vec<BodyId>, ClusterError> {
        let weld_entry = self.weld(weld).ok_or(ClusterError::InvalidWeldId)?;
        let body = self
            .cluster_part(weld_entry.part_a())
            .ok_or(ClusterError::InvalidTopology)?
            .body();
        let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
        if !cluster.welds().contains(&weld) {
            return Err(ClusterError::InvalidTopology);
        }
        self.split_after_weld_removal(body, &[weld])
    }

    /// Removes all welds incident to a part and splits any resulting components.
    pub fn detach_part(&mut self, part: PartId) -> Result<Vec<BodyId>, ClusterError> {
        let body = self
            .cluster_part(part)
            .ok_or(ClusterError::InvalidPartId)?
            .body();
        let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
        if !cluster.parts().contains(&part) {
            return Err(ClusterError::InvalidTopology);
        }
        let welds = cluster
            .welds()
            .iter()
            .copied()
            .filter(|weld_id| {
                self.weld(*weld_id)
                    .is_some_and(|weld| weld.part_a() == part || weld.part_b() == part)
            })
            .collect::<Vec<_>>();
        self.split_after_weld_removal(body, &welds)
    }

    /// Merges two existing clusters into the first body's cluster.
    pub fn merge_clusters(
        &mut self,
        first: BodyId,
        second: BodyId,
        weld_desc: crate::cluster::WeldDesc,
    ) -> Result<(), ClusterError> {
        if first == second {
            return Err(ClusterError::InvalidBodyId);
        }
        weld_desc.validate()?;
        let first_cluster = self.cluster(first).ok_or(ClusterError::NotClustered)?;
        let second_cluster = self.cluster(second).ok_or(ClusterError::NotClustered)?;
        let first_root = first_cluster.root();
        let second_root = second_cluster.root();
        let first_parts = first_cluster.parts().to_vec();
        let second_parts = second_cluster.parts().to_vec();
        let second_welds = second_cluster.welds().to_vec();
        let first_body = *self.body(first).ok_or(ClusterError::InvalidBodyId)?;
        let second_body = *self.body(second).ok_or(ClusterError::InvalidBodyId)?;
        let first_type = first_body.physics().body_type();
        if first_type != second_body.physics().body_type() {
            return Err(ClusterError::IncompatibleMotion);
        }
        let first_pose = first_body.state().transform();
        let second_pose = second_body.state().transform();
        let inverse_first = first_pose.inverse().map_err(ClusterError::Transform)?;
        let relative = inverse_first
            .compose(&second_pose)
            .map_err(ClusterError::Transform)?;

        let first_root_transform = self
            .cluster_part(first_root)
            .ok_or(ClusterError::InvalidTopology)?
            .local_transform();
        let second_root_transform = self
            .cluster_part(second_root)
            .ok_or(ClusterError::InvalidTopology)?
            .local_transform();
        let second_root_in_first = relative
            .compose(&second_root_transform)
            .map_err(ClusterError::Transform)?;
        validate_weld_frames(first_root_transform, second_root_in_first, weld_desc)?;

        let mut transformed_parts = Vec::new();
        let mut transformed_colliders = Vec::new();
        transformed_parts
            .try_reserve(second_parts.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &part in &second_parts {
            let entry = self
                .cluster_part(part)
                .ok_or(ClusterError::InvalidTopology)?;
            let local_transform = relative
                .compose(&entry.local_transform())
                .map_err(ClusterError::Transform)?;
            transformed_parts.push((part, local_transform));
            for &collider in entry.colliders() {
                let collider_entry = self
                    .colliders
                    .get(collider.dense)
                    .filter(|entry| entry.body == second)
                    .ok_or(ClusterError::InvalidTopology)?;
                if first_type == BodyType::Dynamic && !collider_entry.collider.shape().is_convex() {
                    return Err(ClusterError::UnsupportedGeometry);
                }
                let local_transform = relative
                    .compose(&collider_entry.collider.local_transform())
                    .map_err(ClusterError::Transform)?;
                transformed_colliders.push((collider, local_transform));
            }
        }

        let mut values = Vec::new();
        values
            .try_reserve(first_parts.len() + second_parts.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &part in &first_parts {
            let entry = self
                .cluster_part(part)
                .ok_or(ClusterError::InvalidTopology)?;
            values.push((entry.mass(), entry.local_transform()));
        }
        for &(part, local_transform) in &transformed_parts {
            let entry = self
                .cluster_part(part)
                .ok_or(ClusterError::InvalidTopology)?;
            values.push((entry.mass(), local_transform));
        }
        let aggregate = aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;
        let mut prepared = first_body;
        if first_type == BodyType::Dynamic {
            let first_mass = first_body
                .physics()
                .mass()
                .ok_or(ClusterError::InvalidMass)?;
            let second_mass = second_body
                .physics()
                .mass()
                .ok_or(ClusterError::InvalidMass)?;
            let (second_center, second_inertia) = world_properties(second_mass, second_pose)?;
            let second_p = second_body
                .state()
                .linear_velocity()
                .checked_mul_scalar(second_mass.mass())
                .ok_or(ClusterError::ArithmeticOverflow)?;
            let second_h = second_inertia
                .checked_mul_vector(second_body.state().angular_velocity())
                .and_then(|spin| {
                    second_center
                        .checked_cross(second_p)
                        .and_then(|orbital| spin.checked_add(orbital))
                })
                .ok_or(ClusterError::ArithmeticOverflow)?;
            let (velocity, omega) = transfer_motion(
                first_mass,
                first_pose,
                first_body.state().linear_velocity(),
                first_body.state().angular_velocity(),
                aggregate,
                first_pose,
                Vec3::ZERO,
                second_p,
                second_h,
            )?;
            let first_center = first_body
                .world_center_of_mass()
                .map_err(ClusterError::Body)?
                .ok_or(ClusterError::InvalidMass)?;
            let new_center = first_pose.transform_point(aggregate.center_of_mass());
            let force = first_body
                .forces()
                .force()
                .checked_add(second_body.forces().force())
                .ok_or(ClusterError::ArithmeticOverflow)?;
            let torque = first_body
                .forces()
                .torque()
                .checked_add(second_body.forces().torque())
                .and_then(|value| {
                    first_center
                        .checked_sub(new_center)
                        .and_then(|offset| offset.checked_cross(first_body.forces().force()))
                        .and_then(|shift| value.checked_add(shift))
                })
                .and_then(|value| {
                    second_center
                        .checked_sub(new_center)
                        .and_then(|offset| offset.checked_cross(second_body.forces().force()))
                        .and_then(|shift| value.checked_add(shift))
                })
                .ok_or(ClusterError::ArithmeticOverflow)?;
            prepared.set_mass(aggregate).map_err(ClusterError::Body)?;
            prepared
                .set_linear_velocity(velocity)
                .map_err(ClusterError::Body)?;
            prepared
                .set_angular_velocity(omega)
                .map_err(ClusterError::Body)?;
            prepared.apply_force(force).map_err(ClusterError::Body)?;
            prepared.apply_torque(torque).map_err(ClusterError::Body)?;
        }

        self.welds
            .try_reserve_insertions(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        self.bodies
            .get_mut(first.dense)
            .and_then(|entry| entry.cluster.as_mut())
            .ok_or(ClusterError::NotClustered)?
            .parts
            .try_reserve(second_parts.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        self.bodies
            .get_mut(first.dense)
            .and_then(|entry| entry.cluster.as_mut())
            .ok_or(ClusterError::NotClustered)?
            .welds
            .try_reserve(second_welds.len() + 1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;

        let bridge_dense = self
            .welds
            .insert(Weld {
                part_a: first_root,
                part_b: second_root,
                desc: weld_desc,
            })
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let bridge = WeldId {
            world: self.world_id,
            dense: bridge_dense,
        };
        for &(part, local_transform) in &transformed_parts {
            let entry = self
                .parts
                .get_mut(part.dense)
                .ok_or(ClusterError::InvalidTopology)?;
            entry.body = first;
            entry.local_transform = local_transform;
        }
        for &(collider, local_transform) in &transformed_colliders {
            self.rebind_collider(collider, first, local_transform)?;
        }
        for (_, entry) in self.joints.iter_mut() {
            let mut frame_a = entry.joint.common().frame_a;
            let mut frame_b = entry.joint.common().frame_b;
            let target_a_moves = entry.target_a == JointTarget::Body(second)
                || matches!(entry.target_a, JointTarget::Part(part) if second_parts.contains(&part));
            let target_b_moves = entry.target_b == JointTarget::Body(second)
                || matches!(entry.target_b, JointTarget::Part(part) if second_parts.contains(&part));
            if target_a_moves {
                frame_a = super::transform_joint_frame(relative, frame_a)
                    .map_err(ClusterError::Transform)?;
                if entry.target_a == JointTarget::Body(second) {
                    entry.target_a = JointTarget::Body(first);
                }
                entry.body_a = first;
            }
            if target_b_moves {
                frame_b = super::transform_joint_frame(relative, frame_b)
                    .map_err(ClusterError::Transform)?;
                if entry.target_b == JointTarget::Body(second) {
                    entry.target_b = JointTarget::Body(first);
                }
                entry.body_b = first;
            }
            if target_a_moves || target_b_moves {
                entry.joint = entry.joint.with_frames(frame_a, frame_b);
            }
        }
        for (_, source) in self.force_sources.iter_mut() {
            if source.target == ForceTarget::Body(second) {
                let world_point = second_pose.transform_point(source.local_point);
                let world_force = second_pose.transform_vector(source.local_force);
                source.local_point = inverse_first.transform_point(world_point);
                source.local_force = inverse_first.transform_vector(world_force);
                source.target = ForceTarget::Body(first);
                source.body = first;
            } else if source.body == second {
                source.body = first;
            }
        }
        let cluster = self
            .bodies
            .get_mut(first.dense)
            .and_then(|entry| entry.cluster.as_mut())
            .ok_or(ClusterError::NotClustered)?;
        cluster.parts.extend(second_parts);
        cluster.welds.extend(second_welds);
        cluster.welds.push(bridge);
        if let Some(entry) = self.bodies.get_mut(first.dense) {
            entry.body = prepared;
        }
        self.bodies
            .remove(second.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        self.invalidate_body_edit(first);
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        Ok(())
    }

    fn split_after_weld_removal(
        &mut self,
        body: BodyId,
        removed_welds: &[WeldId],
    ) -> Result<Vec<BodyId>, ClusterError> {
        let cluster = self.cluster(body).ok_or(ClusterError::NotClustered)?;
        let original_root = cluster.root();
        let removed_set = removed_welds.iter().copied().collect::<HashSet<_>>();
        if removed_set.len() != removed_welds.len()
            || removed_welds
                .iter()
                .any(|weld| !cluster.welds().contains(weld))
        {
            return Err(ClusterError::InvalidWeldId);
        }
        let mut adjacency = HashMap::<PartId, Vec<PartId>>::new();
        for &part in cluster.parts() {
            adjacency.insert(part, Vec::new());
        }
        let mut remaining_welds = Vec::new();
        for &weld_id in cluster.welds() {
            let weld = self.weld(weld_id).ok_or(ClusterError::InvalidTopology)?;
            if removed_set.contains(&weld_id) {
                continue;
            }
            let (a, b) = weld.parts();
            adjacency
                .get_mut(&a)
                .ok_or(ClusterError::InvalidTopology)?
                .push(b);
            adjacency
                .get_mut(&b)
                .ok_or(ClusterError::InvalidTopology)?
                .push(a);
            remaining_welds.push(weld_id);
        }
        for neighbors in adjacency.values_mut() {
            neighbors.sort_unstable();
        }
        let mut parts = cluster.parts().to_vec();
        parts.sort_unstable();
        let mut components = Vec::<Vec<PartId>>::new();
        let mut visited = HashSet::new();
        for start in parts {
            if visited.contains(&start) {
                continue;
            }
            let mut component = Vec::new();
            let mut pending = vec![start];
            while let Some(part) = pending.pop() {
                if !visited.insert(part) {
                    continue;
                }
                component.push(part);
                if let Some(neighbors) = adjacency.get(&part) {
                    pending.extend(neighbors.iter().rev().copied());
                }
            }
            component.sort_unstable();
            components.push(component);
        }
        components.sort_by_key(|component| component[0]);
        let root_index = components
            .iter()
            .position(|component| component.contains(&original_root))
            .ok_or(ClusterError::InvalidTopology)?;
        components.swap(0, root_index);

        let original = *self.body(body).ok_or(ClusterError::InvalidBodyId)?;
        let mut prepared_bodies = Vec::new();
        prepared_bodies
            .try_reserve(components.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for component in &components {
            let mut values = Vec::new();
            values
                .try_reserve(component.len())
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
            for &part in component {
                let entry = self
                    .cluster_part(part)
                    .ok_or(ClusterError::InvalidTopology)?;
                for &collider in entry.colliders() {
                    let collider_entry = self
                        .colliders
                        .get(collider.dense)
                        .filter(|entry| entry.body == body)
                        .ok_or(ClusterError::InvalidTopology)?;
                    if original.physics().body_type() == BodyType::Dynamic
                        && !collider_entry.collider.shape().is_convex()
                    {
                        return Err(ClusterError::UnsupportedGeometry);
                    }
                }
                values.push((entry.mass(), entry.local_transform()));
            }
            let aggregate =
                aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;
            let mut prepared = original;
            if original.physics().body_type() == BodyType::Dynamic {
                let old_center = original
                    .world_center_of_mass()
                    .map_err(ClusterError::Body)?
                    .ok_or(ClusterError::InvalidMass)?;
                let new_center = original
                    .state()
                    .transform()
                    .transform_point(aggregate.center_of_mass());
                if !new_center.is_finite() {
                    return Err(ClusterError::ArithmeticOverflow);
                }
                let (velocity, omega) = velocity_field(
                    old_center,
                    original.state().linear_velocity(),
                    original.state().angular_velocity(),
                    new_center,
                )?;
                prepared.set_mass(aggregate).map_err(ClusterError::Body)?;
                prepared
                    .set_linear_velocity(velocity)
                    .map_err(ClusterError::Body)?;
                prepared
                    .set_angular_velocity(omega)
                    .map_err(ClusterError::Body)?;
            }
            prepared_bodies.push(prepared);
        }

        self.bodies
            .try_reserve_insertions(components.len().saturating_sub(1))
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let mut new_bodies = Vec::<BodyId>::new();
        new_bodies
            .try_reserve(components.len().saturating_sub(1))
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for prepared in prepared_bodies.iter().skip(1).copied() {
            let dense = match self.bodies.insert(BodyEntry {
                body: prepared,
                first_collider: None,
                cluster: None,
            }) {
                Ok(dense) => dense,
                Err(_) => {
                    for id in new_bodies {
                        self.bodies.remove(id.dense);
                    }
                    return Err(ClusterError::StorageCapacityExceeded);
                }
            };
            new_bodies.push(BodyId {
                world: self.world_id,
                dense,
            });
        }

        for &weld in removed_welds {
            self.welds
                .remove(weld.dense)
                .ok_or(ClusterError::InvalidTopology)?;
        }
        for (component_index, component) in components.iter().enumerate().skip(1) {
            let target_body = new_bodies[component_index - 1];
            for &part in component {
                let colliders = self
                    .cluster_part(part)
                    .ok_or(ClusterError::InvalidTopology)?
                    .colliders()
                    .to_vec();
                if let Some(entry) = self.parts.get_mut(part.dense) {
                    entry.body = target_body;
                }
                for collider in colliders {
                    let local_transform = self
                        .collider(collider)
                        .ok_or(ClusterError::InvalidTopology)?
                        .local_transform();
                    self.rebind_collider(collider, target_body, local_transform)?;
                }
            }
        }
        let root_component = &components[0];
        let root_welds = remaining_welds
            .iter()
            .copied()
            .filter(|weld_id| {
                self.weld(*weld_id).is_some_and(|weld| {
                    root_component.contains(&weld.part_a())
                        && root_component.contains(&weld.part_b())
                })
            })
            .collect::<Vec<_>>();
        let root = root_component[0];
        if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.body = prepared_bodies[0];
            entry.cluster = Some(RigidCluster {
                root,
                parts: root_component.clone(),
                welds: root_welds,
            });
        }
        for (component_index, component) in components.iter().enumerate().skip(1) {
            let target_body = new_bodies[component_index - 1];
            let welds = remaining_welds
                .iter()
                .copied()
                .filter(|weld_id| {
                    self.weld(*weld_id).is_some_and(|weld| {
                        component.contains(&weld.part_a()) && component.contains(&weld.part_b())
                    })
                })
                .collect::<Vec<_>>();
            if let Some(entry) = self.bodies.get_mut(target_body.dense) {
                entry.cluster = Some(RigidCluster {
                    root: component[0],
                    parts: component.clone(),
                    welds,
                });
            }
        }
        let part_body = components
            .iter()
            .enumerate()
            .flat_map(|(component_index, component)| {
                let target_body = if component_index == 0 {
                    body
                } else {
                    new_bodies[component_index - 1]
                };
                component
                    .iter()
                    .copied()
                    .map(move |part| (part, target_body))
            })
            .collect::<HashMap<_, _>>();
        for entry in self.joints.iter_mut().map(|(_, entry)| entry) {
            if let JointTarget::Part(part) = entry.target_a {
                if let Some(&target_body) = part_body.get(&part) {
                    entry.body_a = target_body;
                }
            }
            if let JointTarget::Part(part) = entry.target_b {
                if let Some(&target_body) = part_body.get(&part) {
                    entry.body_b = target_body;
                }
            }
        }
        for (_, source) in self.force_sources.iter_mut() {
            if let ForceTarget::Part(part) = source.target {
                if let Some(&target_body) = part_body.get(&part) {
                    source.body = target_body;
                }
            }
        }
        self.invalidate_body_edit(body);
        for &new_body in &new_bodies {
            self.invalidate_body_edit(new_body);
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        let mut result = Vec::with_capacity(new_bodies.len() + 1);
        result.push(body);
        result.extend(new_bodies);
        Ok(result)
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
            let mut part_colliders = HashSet::new();
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
                    if !part_colliders.insert(collider) {
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
            if part_colliders != linked_colliders {
                return Err(ClusterError::InvalidTopology);
            }
        }
        if all_parts.len() != self.parts.len() || all_welds.len() != self.welds.len() {
            return Err(ClusterError::InvalidTopology);
        }
        Ok(())
    }
}

fn validate_weld_frames(
    transform_a: Transform,
    transform_b: Transform,
    weld: crate::cluster::WeldDesc,
) -> Result<(), ClusterError> {
    let frame_a =
        super::transform_joint_frame(transform_a, weld.frame_a).map_err(ClusterError::Transform)?;
    let frame_b =
        super::transform_joint_frame(transform_b, weld.frame_b).map_err(ClusterError::Transform)?;
    let anchor_delta = frame_a
        .local_anchor
        .checked_sub(frame_b.local_anchor)
        .ok_or(ClusterError::ArithmeticOverflow)?;
    let anchor_scale = frame_a
        .local_anchor
        .abs()
        .max_element()
        .max(frame_b.local_anchor.abs().max_element())
        .max(1.0);
    let tolerance = 128.0 * f64::EPSILON * anchor_scale;
    if anchor_delta.length() > tolerance {
        return Err(ClusterError::IncompatibleAnchors);
    }
    let rotation_dot = frame_a.local_rotation.x.mul_add(
        frame_b.local_rotation.x,
        frame_a.local_rotation.y.mul_add(
            frame_b.local_rotation.y,
            frame_a.local_rotation.z.mul_add(
                frame_b.local_rotation.z,
                frame_a.local_rotation.w * frame_b.local_rotation.w,
            ),
        ),
    );
    if (rotation_dot.abs() - 1.0).abs() > 128.0 * f64::EPSILON {
        return Err(ClusterError::IncompatibleAnchors);
    }
    Ok(())
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
