use kapitsa_math::Vec3;

use crate::Error;
use std::collections::{HashMap, HashSet};

use super::{
    BodyEntry, BodyId, ClusterError, ColliderId, ForceTarget, JointId, JointTarget, PartId,
    PhysicsWorld, WeldId,
};
use crate::{
    aerodynamics::{AerodynamicSurface, AerodynamicTarget, Aerodynamics},
    cluster::{
        ClusterPart, MassChangeMode, MassTransfer, RigidCluster, Weld,
        mass::aggregate_mass,
        motion::{transfer_motion, velocity_field, world_properties},
    },
    mass::MassProperties,
    rigid_body::{BodyMotion, BodyType, RigidBody},
    transform::Transform,
};

/// Rebind of one part's owned colliders after a split.
type PartRebind = (usize, PartId, Vec<(ColliderId, Transform)>);

/// Prepared against an unchanged world; commit must immediately follow it.
pub(super) struct PreparedWeldSplit {
    body: BodyId,
    removed_welds: Vec<WeldId>,
    entries: Vec<BodyEntry>,
    prepared_bodies: Vec<BodyMotion>,
    part_rebinds: Vec<PartRebind>,
    part_components: HashMap<PartId, usize>,
    root_aerodynamics: Vec<AerodynamicSurface>,
    root_cluster: RigidCluster,
    other_clusters: std::vec::IntoIter<RigidCluster>,
    affected_bodies: Vec<BodyId>,
}

/// Inserted entries and plans; must be finalized without intervening fallible
/// operations. This type is internal and cannot outlive the transaction.
pub(super) struct InsertedWeldSplits {
    plans: Vec<PreparedWeldSplit>,
    results: Vec<Vec<BodyId>>,
}

#[derive(Debug)]
pub(super) enum WeldSplitPreparationError {
    Balance(crate::Error),
    Cluster(crate::Error),
}

impl PhysicsWorld {
    fn pending_contact_weld_impulses(
        &self,
        state: &super::ClusterLoadState,
    ) -> Result<Vec<WeldId>, crate::Error> {
        use super::WeldBalanceError as BalanceError;
        use crate::cluster::{
            balance::prepare_tree,
            impulse::{PartImpulse, tree_impulse_reactions},
            motion::world_properties,
        };

        let (linear_tolerance, angular_tolerance) = self
            .automatic_weld_impulse_tolerances
            .ok_or_else(|| crate::Error::from(BalanceError::IncompleteLoads))?;
        let context = &self.collision_system.context;
        if !context.ccd_solved_manifolds.is_empty()
            || (!context.ccd_clipped_bodies.is_empty()
                && context.ccd_clipped_bodies.len() != context.predicted_states.len())
            || context.manifolds.iter().any(|manifold| {
                self.collider_body(manifold.pair.a).is_none()
                    || self.collider_body(manifold.pair.b).is_none()
            })
        {
            return Err(crate::Error::from(BalanceError::IncompleteLoads));
        }
        let mut matching = self.contact_impulses_pending.iter().filter_map(|contact| {
            let a = self.collider_body(contact.pair.a);
            let b = self.collider_body(contact.pair.b);
            if a == Some(state.body) {
                Some((contact, 0))
            } else if b == Some(state.body) {
                Some((contact, 1))
            } else {
                None
            }
        });
        let (contact, side) = matching
            .next()
            .ok_or_else(|| crate::Error::from(BalanceError::IncompleteLoads))?;
        if contact.ccd
            || self.contact_impulses_omitted_pending != 0
            || self.contact_ccd_clipped_pending
        {
            return Err(crate::Error::from(BalanceError::IncompleteLoads));
        }
        let states = contact
            .isolated_velocity_states
            .ok_or_else(|| crate::Error::from(BalanceError::IncompleteLoads))?;
        let other_body = self
            .collider_body(if side == 0 {
                contact.pair.b
            } else {
                contact.pair.a
            })
            .ok_or_else(|| crate::Error::from(BalanceError::IncompleteLoads))?;
        let before = states[0][side];
        let after = states[1][side];
        let mut impact_state = *state;
        impact_state.state = before;
        let tree = prepare_tree(
            &impact_state,
            &self.cluster_load_parts_pending,
            &self.cluster_load_welds_pending,
        )?;
        let mass = state
            .mass
            .ok_or_else(|| crate::Error::from(BalanceError::IncompleteLoads))?;
        let (origin, _) =
            world_properties(mass, before.transform()).map_err(BalanceError::Physics)?;
        let delta_linear = after
            .linear_velocity()
            .checked_sub(before.linear_velocity())
            .ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
        let delta_angular = after
            .angular_velocity()
            .checked_sub(before.angular_velocity())
            .ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
        let mut inputs = Vec::new();
        let mut parents = Vec::new();
        let mut indices = HashMap::new();
        inputs.try_reserve_exact(tree.len()).map_err(|_| {
            BalanceError::Physics(crate::Error::from(ClusterError::StorageCapacityExceeded))
        })?;
        parents.try_reserve_exact(tree.len()).map_err(|_| {
            BalanceError::Physics(crate::Error::from(ClusterError::StorageCapacityExceeded))
        })?;
        indices.try_reserve(tree.len()).map_err(|_| {
            BalanceError::Physics(crate::Error::from(ClusterError::StorageCapacityExceeded))
        })?;
        for (index, part) in tree.iter().enumerate() {
            indices.insert(part.part, index);
            inputs.push(PartImpulse {
                mass: part.mass,
                pose: part.pose,
                linear: Vec3::ZERO,
                angular: Vec3::ZERO,
            });
            parents.push(part.parent);
        }
        let mut count = 0usize;
        for (point, point_side) in std::iter::once((contact, side)).chain(matching) {
            let point_states = point
                .isolated_velocity_states
                .ok_or_else(|| crate::Error::from(BalanceError::IncompleteLoads))?;
            let point_other = self.collider_body(if point_side == 0 {
                point.pair.b
            } else {
                point.pair.a
            });
            if point.ccd
                || point_other != Some(other_body)
                || point_states[0][point_side] != before
                || point_states[1][point_side] != after
            {
                return Err(crate::Error::from(BalanceError::IncompleteLoads));
            }
            let part = point.parts[point_side]
                .ok_or_else(|| crate::Error::from(BalanceError::IncompleteLoads))?;
            let index = *indices
                .get(&part)
                .ok_or_else(|| crate::Error::from(BalanceError::IncompleteLoads))?;
            let contribution = if point_side == 0 {
                point.impulse_on_b.checked_mul_scalar(-1.0)
            } else {
                Some(point.impulse_on_b)
            }
            .ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
            let moment = point.points[point_side]
                .checked_sub(origin)
                .and_then(|arm| arm.checked_cross(contribution))
                .ok_or_else(|| {
                    crate::Error::from(BalanceError::Physics(crate::Error::from(
                        ClusterError::ArithmeticOverflow,
                    )))
                })?;
            inputs[index].linear =
                inputs[index]
                    .linear
                    .checked_add(contribution)
                    .ok_or_else(|| {
                        crate::Error::from(BalanceError::Physics(crate::Error::from(
                            ClusterError::ArithmeticOverflow,
                        )))
                    })?;
            inputs[index].angular = inputs[index].angular.checked_add(moment).ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
            count = count.checked_add(1).ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
        }
        let expected = context
            .manifolds
            .iter()
            .filter(|manifold| {
                self.collider_body(manifold.pair.a) == Some(state.body)
                    || self.collider_body(manifold.pair.b) == Some(state.body)
            })
            .try_fold(0usize, |count, manifold| count.checked_add(manifold.len()))
            .ok_or_else(|| {
                crate::Error::from(BalanceError::Physics(crate::Error::from(
                    ClusterError::ArithmeticOverflow,
                )))
            })?;
        if expected != count {
            return Err(crate::Error::from(BalanceError::IncompleteLoads));
        }
        let reactions = tree_impulse_reactions(
            &inputs,
            &parents,
            origin,
            delta_linear,
            delta_angular,
            origin,
            linear_tolerance,
            angular_tolerance,
        )?;
        let mut candidates = Vec::new();
        candidates
            .try_reserve(tree.len().saturating_sub(1))
            .map_err(|_| {
                BalanceError::Physics(crate::Error::from(ClusterError::StorageCapacityExceeded))
            })?;
        for (part, reaction) in tree.iter().zip(reactions).skip(1) {
            let id = part
                .weld
                .ok_or_else(|| crate::Error::from(BalanceError::InvalidTree))?;
            let weld = self
                .weld(id)
                .ok_or_else(|| crate::Error::from(BalanceError::InvalidTree))?;
            let Some(strength) = weld.impulse_strength() else {
                continue;
            };
            let frame = if weld.part_a() == part.part {
                weld.desc().frame_a
            } else if weld.part_b() == part.part {
                weld.desc().frame_b
            } else {
                return Err(crate::Error::from(BalanceError::InvalidTree));
            };
            let reaction =
                reaction.at_anchor(origin, part.pose.transform_point(frame.local_anchor))?;
            if strength.exceeded_by(reaction)? {
                candidates.push(id);
            }
        }
        Ok(candidates)
    }

    /// Prepare all breakable clusters from the active step's pending snapshots
    /// and final staged predictions. No topology or body state is changed.
    pub(super) fn prepare_pending_weld_splits(
        &mut self,
        force_tolerance: f64,
        moment_tolerance: f64,
    ) -> Result<Vec<PreparedWeldSplit>, WeldSplitPreparationError> {
        let mut plans = Vec::new();
        plans
            .try_reserve(self.cluster_load_states_pending.len())
            .map_err(|_| {
                WeldSplitPreparationError::Cluster(crate::Error::from(
                    ClusterError::StorageCapacityExceeded,
                ))
            })?;
        for state in &self.cluster_load_states_pending {
            let cluster = self
                .cluster(state.body)
                .ok_or(WeldSplitPreparationError::Cluster(crate::Error::from(
                    ClusterError::InvalidTopology,
                )))?;
            if !cluster.welds().iter().any(|id| {
                self.weld(*id).is_some_and(|weld| {
                    matches!(
                        weld.desc().strength,
                        crate::cluster::WeldStrength::Breakable { .. }
                    )
                })
            }) {
                continue;
            }
            let evaluation = (|| {
                let mut welds = if state.collision_loads_uncertified {
                    self.pending_contact_weld_impulses(state)?
                } else {
                    Vec::new()
                };
                let candidates = self.pending_weld_failure_candidates(
                    state.body,
                    force_tolerance,
                    moment_tolerance,
                    state.collision_loads_uncertified,
                )?;
                welds.try_reserve(candidates.len()).map_err(|_| {
                    crate::Error::from(super::WeldBalanceError::Physics(crate::Error::from(
                        ClusterError::StorageCapacityExceeded,
                    )))
                })?;
                let mut seen = HashSet::new();
                seen.try_reserve(welds.len().saturating_add(candidates.len()))
                    .map_err(|_| {
                        crate::Error::from(super::WeldBalanceError::Physics(crate::Error::from(
                            ClusterError::StorageCapacityExceeded,
                        )))
                    })?;
                seen.extend(welds.iter().copied());
                for candidate in candidates {
                    if seen.insert(candidate.reaction.weld) {
                        welds.push(candidate.reaction.weld);
                    }
                }
                Ok::<_, crate::Error>(welds)
            })();
            let welds = match evaluation {
                Ok(welds) => welds,
                Err(error)
                    if error.downcast_ref::<super::WeldBalanceError>()
                        == Some(&super::WeldBalanceError::IncompleteLoads)
                        && self.error_policy == super::ErrorPolicy::Realtime =>
                {
                    self.pending_step_diagnostics.weld_incomplete_load_clusters = self
                        .pending_step_diagnostics
                        .weld_incomplete_load_clusters
                        .checked_add(1)
                        .ok_or(WeldSplitPreparationError::Cluster(crate::Error::from(
                            ClusterError::StorageCapacityExceeded,
                        )))?;
                    if state.collision_loads_uncertified {
                        self.pending_step_diagnostics.weld_collision_load_clusters = self
                            .pending_step_diagnostics
                            .weld_collision_load_clusters
                            .checked_add(1)
                            .ok_or(WeldSplitPreparationError::Cluster(crate::Error::from(
                                ClusterError::StorageCapacityExceeded,
                            )))?;
                    }
                    continue;
                }
                Err(error) => return Err(WeldSplitPreparationError::Balance(error)),
            };
            if welds.is_empty() {
                continue;
            }
            let predicted = self
                .collision_system
                .context
                .predicted_states
                .iter()
                .find(|(body, _)| *body == state.body)
                .map(|(_, state)| *state)
                .ok_or(WeldSplitPreparationError::Cluster(crate::Error::from(
                    ClusterError::InvalidMotion,
                )))?;
            let mut motion = self
                .body(state.body)
                .ok_or(WeldSplitPreparationError::Cluster(crate::Error::from(
                    ClusterError::InvalidBodyId,
                )))?
                .motion();
            motion.commit_integrated_state(predicted);
            plans.push(
                self.prepare_weld_split_for_motion(state.body, &welds, motion)
                    .map_err(WeldSplitPreparationError::Cluster)?,
            );
        }
        Ok(plans)
    }
    /// Returns the part owning a collider, or `None` for an unclustered body
    /// or an invalid collider. Lookup scans the owning cluster's part and
    /// collider lists; it does not cache ownership across topology changes.
    pub fn collider_part(&self, collider: ColliderId) -> Option<PartId> {
        let body = self.collider_body(collider)?;
        self.cluster(body)?.parts().iter().copied().find(|part| {
            self.cluster_part(*part)
                .is_some_and(|entry| entry.colliders().contains(&collider))
        })
    }

    /// Adds a collider owned by a cluster part.
    pub fn add_part_collider(
        &mut self,
        part: PartId,
        collider: crate::collider::Collider,
    ) -> Result<ColliderId, Error> {
        self.add_part_collider_inner(part, collider)
    }
    fn add_part_collider_inner(
        &mut self,
        part: PartId,
        mut collider: crate::collider::Collider,
    ) -> Result<ColliderId, crate::Error> {
        let part_entry = self
            .cluster_part(part)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?;
        let body = part_entry.body();
        let local_transform = part_entry
            .local_transform()
            .compose(&collider.local_transform())
            .map_err(ClusterError::Transform)?;
        self.parts
            .get_mut(part.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?
            .colliders
            .try_reserve(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        collider.set_local_transform(local_transform);
        let id = self.add_collider_internal(body, collider)?;
        self.parts
            .get_mut(part.dense)
            .expect("validated cluster part must exist")
            .colliders
            .push(id);
        Ok(id)
    }

    /// Changes a collider's rigid transform relative to its owning part.
    /// Validates ownership and composition before changing collision or aero state.
    pub fn set_part_collider_transform(
        &mut self,
        part: PartId,
        collider: ColliderId,
        part_local_transform: Transform,
    ) -> Result<(), Error> {
        self.set_part_collider_transform_inner(part, collider, part_local_transform)
    }
    fn set_part_collider_transform_inner(
        &mut self,
        part: PartId,
        collider: ColliderId,
        part_local_transform: Transform,
    ) -> Result<(), crate::Error> {
        let owner = self
            .cluster_part(part)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?;
        if !owner.colliders().contains(&collider) {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        let body = owner.body();
        let local = owner
            .local_transform()
            .compose(&part_local_transform)
            .map_err(ClusterError::Transform)?;
        self.rebind_collider(collider, body, local)?;
        let entry = self
            .bodies
            .get_mut(body.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?;
        entry.collider_aerodynamic_mesh = None;
        entry.mesh_aerodynamics = None;
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&body);
        }
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(())
    }

    /// Removes a collider from its owning cluster part.
    pub fn remove_part_collider(
        &mut self,
        id: ColliderId,
    ) -> Result<crate::collider::Collider, Error> {
        self.remove_part_collider_inner(id)
    }
    fn remove_part_collider_inner(
        &mut self,
        id: ColliderId,
    ) -> Result<crate::collider::Collider, crate::Error> {
        let body = self
            .collider_body(id)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?;
        self.cluster(body)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        let part = self
            .collider_part(id)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        let collider = self
            .remove_collider_internal(id)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        let entry = self
            .parts
            .get_mut(part.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        if let Some(index) = entry.colliders.iter().position(|value| *value == id) {
            entry.colliders.remove(index);
            Ok(collider)
        } else {
            Err(crate::Error::from(ClusterError::InvalidTopology))
        }
    }

    /// Converts an existing dynamic body into a one-part cluster.
    ///
    /// Existing colliders become owned by the identity-transform root part.
    /// Static and kinematic bodies can use [`Self::enable_cluster_with_mass`]
    /// because they do not carry mass properties in `RigidBody`.
    pub fn enable_cluster(&mut self, body: BodyId) -> Result<PartId, Error> {
        self.enable_cluster_inner(body)
    }
    fn enable_cluster_inner(&mut self, body: BodyId) -> Result<PartId, crate::Error> {
        let mass = self
            .body(body)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .physics()
            .mass()
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
        self.enable_cluster_with_mass_inner(body, mass)
    }

    /// Converts an existing body into a one-part cluster with explicit root mass.
    pub fn enable_cluster_with_mass(
        &mut self,
        body: BodyId,
        mass: MassProperties,
    ) -> Result<PartId, Error> {
        self.enable_cluster_with_mass_inner(body, mass)
    }
    fn enable_cluster_with_mass_inner(
        &mut self,
        body: BodyId,
        mass: MassProperties,
    ) -> Result<PartId, crate::Error> {
        if self.has_body_chunks(body) {
            return Err(crate::Error::from(ClusterError::UnsupportedGeometry));
        }
        let body_entry = self
            .bodies
            .get(body.dense)
            .filter(|_| body.world == self.world_id)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?;
        if body_entry.cluster.is_some() {
            return Err(crate::Error::from(ClusterError::AlreadyClustered));
        }
        // Base patches are body-local and are not yet transferred to a part.
        // Reject before changing topology rather than strand or lose loads.
        if !body_entry.body.base_pressure_patches.is_empty()
            || body_entry.body.potential_sphere.is_some()
            || matches!(
                body_entry.body.aerodynamics(),
                Aerodynamics::IncompressibleSphere(_)
                    | Aerodynamics::ContinuumSphere(_)
                    | Aerodynamics::CompressionDominatedParticle(_)
                    | Aerodynamics::SlipCorrectedParticle(_)
                    | Aerodynamics::CreepingParticle(_)
                    | Aerodynamics::FreeMolecularParticle(_)
                    | Aerodynamics::TransitionalParticle(_)
            )
        {
            return Err(crate::Error::from(ClusterError::UnsupportedGeometry));
        }
        let mut root_hull = Vec::new();
        if let Aerodynamics::Hull(sections) = body_entry.body.aerodynamics() {
            root_hull
                .try_reserve(sections.len())
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        }
        let mut root_wings = Vec::new();
        if matches!(body_entry.body.aerodynamics(), Aerodynamics::Wing(_)) {
            root_wings
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        }
        let mesh_mode = matches!(body_entry.body.aerodynamics(), Aerodynamics::Mesh(_));
        let collider_mode = matches!(body_entry.body.aerodynamics(), Aerodynamics::Colliders);
        let wave_mode = matches!(body_entry.body.aerodynamics(), Aerodynamics::VoxelWave(_));
        if body_entry.body.physics().body_type() == BodyType::Dynamic
            && body_entry.body.physics().mass() != Some(mass)
        {
            return Err(crate::Error::from(ClusterError::InvalidMass));
        }

        let mut colliders = Vec::new();
        let mut next = body_entry.first_collider;
        while let Some(collider) = next {
            let entry = self
                .colliders
                .get(collider.dense)
                .filter(|entry| entry.body == body)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            colliders.push(collider);
            next = entry.next;
        }
        colliders
            .try_reserve(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        self.parts
            .try_reserve_insertions(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        if mesh_mode || collider_mode || wave_mode {
            self.part_aerodynamic_geometry
                .get_or_insert_with(HashMap::new)
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        }

        let dense = self
            .parts
            .insert(ClusterPart {
                body,
                local_transform: Transform::identity(),
                mass,
                colliders,
                aerodynamic_wings: Vec::new(),
                aerodynamic_hull: Vec::new(),
            })
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let part = PartId {
            world: self.world_id,
            dense,
        };
        let entry = self
            .bodies
            .get_mut(body.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?;
        entry.cluster = Some(RigidCluster {
            root: part,
            parts: vec![part],
            welds: Vec::new(),
        });
        // Capacity was reserved before inserting the part; these moves cannot
        // lose the original body model on a fallible preparation operation.
        match entry.body.take_primary_aerodynamics() {
            Aerodynamics::Mesh(mesh) => {
                self.part_aerodynamic_geometry
                    .get_or_insert_with(HashMap::new)
                    .insert(
                        part,
                        super::assembly_aerodynamics::PartGeometry::Mesh(std::sync::Arc::new(mesh)),
                    );
                entry.mesh_aerodynamics = None;
            }
            Aerodynamics::VoxelWave(model) => {
                self.part_aerodynamic_geometry
                    .get_or_insert_with(HashMap::new)
                    .insert(
                        part,
                        super::assembly_aerodynamics::PartGeometry::Wave(Box::new(model)),
                    );
                entry.mesh_aerodynamics = None;
            }
            Aerodynamics::Colliders => {
                self.part_aerodynamic_geometry
                    .get_or_insert_with(HashMap::new)
                    .insert(part, super::assembly_aerodynamics::PartGeometry::Colliders);
                entry.mesh_aerodynamics = None;
                entry.collider_aerodynamic_mesh = None;
            }
            Aerodynamics::Wing(wing) => root_wings.push(wing),
            Aerodynamics::Wings(wings) => root_wings = wings,
            Aerodynamics::Hull(sections) => root_hull = sections,
            model => entry.body.set_primary_aerodynamics(model),
        }
        if let Some(root) = self.parts.get_mut(part.dense) {
            root.aerodynamic_wings = root_wings;
            root.aerodynamic_hull = root_hull;
        }
        self.body_revision = self.body_revision.wrapping_add(1);
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
        desc: crate::cluster::ClusterPartDesc,
        weld_desc: crate::cluster::WeldDesc,
    ) -> Result<PartId, Error> {
        self.add_cluster_part_inner(parent, desc, weld_desc)
    }
    fn add_cluster_part_inner(
        &mut self,
        parent: PartId,
        mut desc: crate::cluster::ClusterPartDesc,
        weld_desc: crate::cluster::WeldDesc,
    ) -> Result<PartId, crate::Error> {
        weld_desc.validate()?;
        if !desc.local_transform.translation().is_finite()
            || !desc.local_transform.rotation().is_normalized()
            || !desc.linear_velocity.is_finite()
            || !desc.angular_velocity.is_finite()
        {
            return Err(crate::Error::from(ClusterError::InvalidMotion));
        }
        let body = self
            .cluster_part(parent)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?
            .body();
        let cluster = self
            .cluster(body)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        if !cluster.parts().contains(&parent) {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        let body_pose = self
            .body(body)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .state()
            .transform();
        let parent_transform = self
            .cluster_part(parent)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?
            .local_transform();
        let part_pose = body_pose
            .compose(&desc.local_transform)
            .map_err(ClusterError::Transform)?;
        validate_weld_frames(parent_transform, desc.local_transform, weld_desc)?;

        let original = self
            .body(body)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .motion();
        let mut values = Vec::new();
        values
            .try_reserve(cluster.parts().len().saturating_add(1))
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &id in cluster.parts() {
            let part = self
                .cluster_part(id)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            values.push((part.mass(), part.local_transform()));
        }
        values.push((desc.mass, desc.local_transform));
        let aggregate = aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;

        let prepared_body = if original.physics().body_type() == BodyType::Dynamic {
            let old_center = original
                .world_center_of_mass()
                .map_err(map_body_to_cluster)?
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
            let new_center = body_pose.transform_point(aggregate.center_of_mass());
            if !new_center.is_finite() {
                return Err(crate::Error::from(ClusterError::ArithmeticOverflow));
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
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
            let mut prepared = original;
            prepared.set_mass(aggregate).map_err(map_body_to_cluster)?;
            prepared
                .set_linear_velocity(velocity)
                .map_err(map_body_to_cluster)?;
            prepared
                .set_angular_velocity(omega)
                .map_err(map_body_to_cluster)?;
            prepared.apply_force(force).map_err(map_body_to_cluster)?;
            prepared.apply_torque(torque).map_err(map_body_to_cluster)?;
            Some(prepared)
        } else {
            None
        };

        if original.physics().body_type() == BodyType::Dynamic {
            let old_center = original
                .world_center_of_mass()
                .map_err(map_body_to_cluster)?
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
            let part_center = part_pose.transform_point(desc.mass.center_of_mass());
            let expected_linear = part_center
                .checked_sub(old_center)
                .and_then(|offset| original.state().angular_velocity().checked_cross(offset))
                .and_then(|relative| original.state().linear_velocity().checked_add(relative))
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
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
                return Err(crate::Error::from(ClusterError::IncompatibleMotion));
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
            return Err(crate::Error::from(ClusterError::NotClustered));
        }
        if !desc.colliders.is_empty() {
            self.colliders
                .try_reserve_insertions(desc.colliders.len())
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        }
        let mut prepared_colliders = Vec::new();
        prepared_colliders
            .try_reserve(desc.colliders.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for mut collider in desc.colliders.drain(..) {
            let local_transform = desc
                .local_transform
                .compose(&collider.local_transform())
                .map_err(ClusterError::Transform)?;
            collider.set_local_transform(local_transform);
            prepared_colliders.push(collider);
        }
        let prepared_surfaces = if let Some(prepared) = &prepared_body {
            Some(self.prepare_cluster_surface_rebase(body, &original, prepared)?)
        } else {
            self.cluster_aerodynamic_surfaces(body)?;
            None
        };
        let part_dense = self
            .parts
            .insert(ClusterPart {
                body,
                local_transform: desc.local_transform,
                mass: desc.mass,
                colliders: Vec::new(),
                aerodynamic_wings: Vec::new(),
                aerodynamic_hull: Vec::new(),
            })
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let part = PartId {
            world: self.world_id,
            dense: part_dense,
        };
        let mut collider_ids = Vec::new();
        for collider in prepared_colliders {
            match self.add_collider_internal(body, collider) {
                Ok(collider_id) => collider_ids.push(collider_id),
                Err(error) => {
                    for collider_id in collider_ids {
                        self.remove_collider_internal(collider_id);
                    }
                    self.parts.remove(part_dense);
                    return Err(error);
                }
            }
        }
        self.parts
            .get_mut(part_dense)
            .expect("inserted cluster part must exist")
            .colliders = collider_ids.clone();

        let weld_dense = match self.welds.insert(Weld {
            part_a: parent,
            part_b: part,
            desc: weld_desc,
            impulse_strength: None,
        }) {
            Ok(value) => value,
            Err(_) => {
                for collider_id in collider_ids {
                    self.remove_collider_internal(collider_id);
                }
                self.parts.remove(part_dense);
                return Err(crate::Error::from(ClusterError::StorageCapacityExceeded));
            }
        };
        let weld = WeldId {
            world: self.world_id,
            dense: weld_dense,
        };

        if let Some(prepared) = prepared_body {
            let entry = self
                .bodies
                .get_mut(body.dense)
                .expect("cluster body must exist");
            if let Some(surfaces) = prepared_surfaces {
                Self::commit_cluster_surface_rebase(&mut entry.body, surfaces);
            }
            entry.body.set_motion(prepared);
        }
        let cluster = self
            .bodies
            .get_mut(body.dense)
            .and_then(|entry| entry.cluster.as_mut())
            .expect("validated cluster must exist");
        cluster.parts.push(part);
        cluster.welds.push(weld);
        self.synchronize_cluster_physics_state(&[body]);
        Ok(part)
    }

    /// Removes a part and its incident welds from a rigid cluster.
    ///
    /// The remaining parts must stay connected. Removing the final part is
    /// rejected so the owning body cannot outlive an empty cluster.
    pub fn destroy_cluster_part(&mut self, part: PartId) -> Result<(), Error> {
        self.destroy_cluster_part_inner(part)
    }
    fn destroy_cluster_part_inner(&mut self, part: PartId) -> Result<(), crate::Error> {
        let body = self
            .cluster_part(part)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?
            .body();
        let cluster = self
            .cluster(body)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        let original_root = cluster.root();
        if !cluster.parts().contains(&part) {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        if cluster.parts().len() == 1 {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
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
            let weld = self
                .weld(weld_id)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            let (a, b) = weld.parts();
            if a == part || b == part {
                removed_welds.push(weld_id);
            } else if remaining_set.contains(&a) && remaining_set.contains(&b) {
                remaining_welds.push(weld_id);
            } else {
                return Err(crate::Error::from(ClusterError::InvalidTopology));
            }
        }

        let mut adjacency = HashMap::new();
        for &id in &remaining_parts {
            adjacency.insert(id, Vec::new());
        }
        for &weld_id in &remaining_welds {
            let weld = self
                .weld(weld_id)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            let (a, b) = weld.parts();
            adjacency
                .get_mut(&a)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
                .push(b);
            adjacency
                .get_mut(&b)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
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
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }

        let removed_entry = self
            .cluster_part(part)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        let removed_colliders = removed_entry.colliders().to_vec();
        for &collider in &removed_colliders {
            if self.collider_body(collider) != Some(body) {
                return Err(crate::Error::from(ClusterError::InvalidTopology));
            }
        }

        let mut values = Vec::new();
        values
            .try_reserve(remaining_parts.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &id in &remaining_parts {
            let entry = self
                .cluster_part(id)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            values.push((entry.mass(), entry.local_transform()));
        }
        let aggregate = aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;
        let original = self
            .body(body)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .motion();
        let new_center = original
            .state()
            .transform()
            .transform_point(aggregate.center_of_mass());
        if !new_center.is_finite() {
            return Err(crate::Error::from(ClusterError::ArithmeticOverflow));
        }
        let mut prepared = original;
        if original.physics().body_type() == BodyType::Dynamic {
            let old_center = original
                .world_center_of_mass()
                .map_err(map_body_to_cluster)?
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
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
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
            prepared.set_mass(aggregate).map_err(map_body_to_cluster)?;
            prepared
                .set_linear_velocity(velocity)
                .map_err(map_body_to_cluster)?;
            prepared
                .set_angular_velocity(omega)
                .map_err(map_body_to_cluster)?;
            if original.physics().mass() != Some(aggregate) {
                prepared.apply_force(force).map_err(map_body_to_cluster)?;
                prepared.apply_torque(torque).map_err(map_body_to_cluster)?;
            }
        }

        let mut prepared_surfaces =
            self.prepare_cluster_surface_rebase(body, &original, &prepared)?;
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
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        }
        for weld in &removed_welds {
            self.welds
                .remove(weld.dense)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        }
        self.parts
            .remove(part.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;

        let root = if original_root == part {
            remaining_parts
                .iter()
                .copied()
                .min()
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
        } else {
            original_root
        };
        let cluster = self
            .bodies
            .get_mut(body.dense)
            .and_then(|entry| entry.cluster.as_mut())
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        cluster.root = root;
        cluster.parts.retain(|id| *id != part);
        cluster.welds.retain(|id| !removed_welds.contains(id));
        let entry = self
            .bodies
            .get_mut(body.dense)
            .expect("cluster body must exist");
        prepared_surfaces.retain(|surface| surface.target() != AerodynamicTarget::Part(part));
        Self::commit_cluster_surface_rebase(&mut entry.body, prepared_surfaces);
        entry.body.set_motion(prepared);
        self.synchronize_cluster_physics_state(&[body]);
        Ok(())
    }

    /// Backward-compatible alias for [`Self::destroy_cluster_part`].
    pub fn remove_cluster_part(&mut self, part: PartId) -> Result<(), Error> {
        self.remove_cluster_part_inner(part)
    }
    fn remove_cluster_part_inner(&mut self, part: PartId) -> Result<(), crate::Error> {
        self.destroy_cluster_part_inner(part)
    }

    /// Removes one weld and splits the graph when that edge disconnects it.
    pub fn break_weld(&mut self, weld: WeldId) -> Result<Vec<BodyId>, Error> {
        self.break_weld_inner(weld)
    }
    fn break_weld_inner(&mut self, weld: WeldId) -> Result<Vec<BodyId>, crate::Error> {
        let weld_entry = self
            .weld(weld)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidWeldId))?;
        let body = self
            .cluster_part(weld_entry.part_a())
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
            .body();
        let cluster = self
            .cluster(body)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        if !cluster.welds().contains(&weld) {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        self.split_after_weld_removal(body, &[weld])
    }

    /// Removes all welds incident to a part and splits any resulting components.
    pub fn detach_part(&mut self, part: PartId) -> Result<Vec<BodyId>, Error> {
        self.detach_part_inner(part)
    }
    fn detach_part_inner(&mut self, part: PartId) -> Result<Vec<BodyId>, crate::Error> {
        let body = self
            .cluster_part(part)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?
            .body();
        let cluster = self
            .cluster(body)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        if !cluster.parts().contains(&part) {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
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
    ) -> Result<(), Error> {
        self.merge_clusters_inner(first, second, weld_desc)
    }
    fn merge_clusters_inner(
        &mut self,
        first: BodyId,
        second: BodyId,
        weld_desc: crate::cluster::WeldDesc,
    ) -> Result<(), crate::Error> {
        if first == second {
            return Err(crate::Error::from(ClusterError::InvalidBodyId));
        }
        weld_desc.validate()?;
        let first_cluster = self
            .cluster(first)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        let second_cluster = self
            .cluster(second)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        let has_wave = |cluster: &RigidCluster| {
            cluster.parts().iter().any(|part| {
                matches!(
                    self.part_aerodynamic_geometry
                        .as_ref()
                        .and_then(|geometry| geometry.get(part)),
                    Some(super::assembly_aerodynamics::PartGeometry::Wave(_))
                )
            })
        };
        if has_wave(first_cluster) && has_wave(second_cluster) {
            return Err(crate::Error::from(ClusterError::UnsupportedGeometry));
        }
        let first_tessellation = self
            .bodies
            .get(first.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .collider_tessellation;
        let second_tessellation = self
            .bodies
            .get(second.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .collider_tessellation;
        if first_tessellation != second_tessellation {
            return Err(crate::Error::from(ClusterError::UnsupportedGeometry));
        }
        let first_root = first_cluster.root();
        let second_root = second_cluster.root();
        let first_parts = first_cluster.parts().to_vec();
        let second_parts = second_cluster.parts().to_vec();
        let second_welds = second_cluster.welds().to_vec();
        let first_body = self
            .body(first)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .motion();
        let second_body = self
            .body(second)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .motion();
        let first_type = first_body.physics().body_type();
        if first_type != second_body.physics().body_type() {
            return Err(crate::Error::from(ClusterError::IncompatibleMotion));
        }
        let first_pose = first_body.state().transform();
        let second_pose = second_body.state().transform();
        let inverse_first = first_pose.inverse().map_err(ClusterError::Transform)?;
        let relative = inverse_first
            .compose(&second_pose)
            .map_err(ClusterError::Transform)?;

        let first_root_transform = self
            .cluster_part(first_root)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
            .local_transform();
        let second_root_transform = self
            .cluster_part(second_root)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
            .local_transform();
        let second_root_in_first = relative
            .compose(&second_root_transform)
            .map_err(ClusterError::Transform)?;
        validate_weld_frames(first_root_transform, second_root_in_first, weld_desc)?;
        if first_type == BodyType::Kinematic {
            let weld_point = first_pose.transform_point(
                super::transform_joint_frame(first_root_transform, weld_desc.frame_a)
                    .map_err(ClusterError::Transform)?
                    .local_anchor,
            );
            let first_velocity = first_body
                .state()
                .angular_velocity()
                .checked_cross(
                    weld_point
                        .checked_sub(first_pose.translation())
                        .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?,
                )
                .and_then(|value| first_body.state().linear_velocity().checked_add(value))
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
            let second_velocity = second_body
                .state()
                .angular_velocity()
                .checked_cross(
                    weld_point
                        .checked_sub(second_pose.translation())
                        .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?,
                )
                .and_then(|value| second_body.state().linear_velocity().checked_add(value))
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
            let scale = first_velocity
                .abs()
                .max_element()
                .max(second_velocity.abs().max_element())
                .max(1.0);
            if first_velocity
                .checked_sub(second_velocity)
                .and_then(Vec3::checked_length)
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?
                > 128.0 * f64::EPSILON * scale
            {
                return Err(crate::Error::from(ClusterError::IncompatibleMotion));
            }
            let first_omega = first_body.state().angular_velocity();
            let second_omega = second_body.state().angular_velocity();
            let omega_scale = first_omega
                .abs()
                .max_element()
                .max(second_omega.abs().max_element())
                .max(1.0);
            if first_omega
                .checked_sub(second_omega)
                .and_then(Vec3::checked_length)
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?
                > 128.0 * f64::EPSILON * omega_scale
            {
                return Err(crate::Error::from(ClusterError::IncompatibleMotion));
            }
        }

        let mut transformed_parts = Vec::new();
        let mut transformed_colliders = Vec::new();
        transformed_parts
            .try_reserve(second_parts.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &part in &second_parts {
            let entry = self
                .cluster_part(part)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            let local_transform = relative
                .compose(&entry.local_transform())
                .map_err(ClusterError::Transform)?;
            transformed_parts.push((part, local_transform));
            for &collider in entry.colliders() {
                let collider_entry = self
                    .colliders
                    .get(collider.dense)
                    .filter(|entry| entry.body == second)
                    .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
                if first_type == BodyType::Dynamic && !collider_entry.collider.shape().is_convex() {
                    return Err(crate::Error::from(ClusterError::UnsupportedGeometry));
                }
                self.validate_collider_rebind(collider, first)?;
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
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            values.push((entry.mass(), entry.local_transform()));
        }
        for &(part, local_transform) in &transformed_parts {
            let entry = self
                .cluster_part(part)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            values.push((entry.mass(), local_transform));
        }
        let aggregate = aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;
        let mut prepared = first_body;
        if first_type == BodyType::Dynamic {
            let first_mass = first_body
                .physics()
                .mass()
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
            let second_mass = second_body
                .physics()
                .mass()
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
            let origin = first_body
                .world_center_of_mass()
                .map_err(map_body_to_cluster)?
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
            let (second_center, second_inertia) = world_properties(second_mass, second_pose)?;
            let second_p = second_body
                .state()
                .linear_velocity()
                .checked_mul_scalar(second_mass.mass())
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
            let second_h = second_inertia
                .checked_mul_vector(second_body.state().angular_velocity())
                .and_then(|spin| {
                    second_center
                        .checked_sub(origin)
                        .and_then(|offset| offset.checked_cross(second_p))
                        .and_then(|orbital| spin.checked_add(orbital))
                })
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
            let (velocity, omega) = transfer_motion(
                first_mass,
                first_pose,
                first_body.state().linear_velocity(),
                first_body.state().angular_velocity(),
                aggregate,
                first_pose,
                origin,
                second_p,
                second_h,
            )?;
            let first_center = origin;
            let new_center = first_pose.transform_point(aggregate.center_of_mass());
            let force = first_body
                .forces()
                .force()
                .checked_add(second_body.forces().force())
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
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
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
            prepared.set_mass(aggregate).map_err(map_body_to_cluster)?;
            prepared
                .set_linear_velocity(velocity)
                .map_err(map_body_to_cluster)?;
            prepared
                .set_angular_velocity(omega)
                .map_err(map_body_to_cluster)?;
            prepared.apply_force(force).map_err(map_body_to_cluster)?;
            prepared.apply_torque(torque).map_err(map_body_to_cluster)?;
        }

        for &(part, _) in &transformed_parts {
            if self.parts.get(part.dense).is_none() {
                return Err(crate::Error::from(ClusterError::InvalidTopology));
            }
        }
        if self
            .bodies
            .get(first.dense)
            .and_then(|entry| entry.cluster.as_ref())
            .is_none()
        {
            return Err(crate::Error::from(ClusterError::NotClustered));
        }
        let mut joint_updates = Vec::new();
        for (dense, entry) in self.joints.iter() {
            let target_a_moves = entry.target_a == JointTarget::Body(second)
                || matches!(entry.target_a, JointTarget::Part(part) if second_parts.contains(&part));
            let target_b_moves = entry.target_b == JointTarget::Body(second)
                || matches!(entry.target_b, JointTarget::Part(part) if second_parts.contains(&part));
            if !target_a_moves && !target_b_moves {
                continue;
            }
            let mut frame_a = entry.joint.common().frame_a;
            let mut frame_b = entry.joint.common().frame_b;
            if target_a_moves {
                frame_a = super::transform_joint_frame(relative, frame_a)
                    .map_err(ClusterError::Transform)?;
            }
            if target_b_moves {
                frame_b = super::transform_joint_frame(relative, frame_b)
                    .map_err(ClusterError::Transform)?;
            }
            joint_updates.push((dense, target_a_moves, target_b_moves, frame_a, frame_b));
        }

        let first_aerodynamics = self.cluster_aerodynamic_surfaces(first)?;
        let second_aerodynamics = self.cluster_aerodynamic_surfaces(second)?;
        let mut merged_aerodynamics = Vec::new();
        merged_aerodynamics
            .try_reserve(
                first_aerodynamics
                    .len()
                    .saturating_add(second_aerodynamics.len()),
            )
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for surface in first_aerodynamics {
            merged_aerodynamics.push(
                PhysicsWorld::rebase_aerodynamic_surface(surface, &first_body, &prepared)
                    .map_err(|_| ClusterError::ArithmeticOverflow)?,
            );
        }
        for surface in second_aerodynamics {
            merged_aerodynamics.push(
                PhysicsWorld::rebase_aerodynamic_surface(surface, &second_body, &prepared)
                    .map_err(|_| ClusterError::ArithmeticOverflow)?,
            );
        }

        self.welds
            .try_reserve_insertions(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        self.bodies
            .get_mut(first.dense)
            .and_then(|entry| entry.cluster.as_mut())
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?
            .parts
            .try_reserve(second_parts.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        self.bodies
            .get_mut(first.dense)
            .and_then(|entry| entry.cluster.as_mut())
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?
            .welds
            .try_reserve(second_welds.len() + 1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;

        let bridge_dense = self
            .welds
            .insert(Weld {
                part_a: first_root,
                part_b: second_root,
                desc: weld_desc,
                impulse_strength: None,
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
                .expect("cluster part must be validated before commit");
            entry.body = first;
            entry.local_transform = local_transform;
        }
        for &(collider, local_transform) in &transformed_colliders {
            self.rebind_collider_prepared(collider, first, local_transform);
        }
        for (dense, target_a_moves, target_b_moves, frame_a, frame_b) in joint_updates {
            let entry = self
                .joints
                .get_mut(dense)
                .expect("joint must exist during merge commit");
            if target_a_moves {
                if entry.target_a == JointTarget::Body(second) {
                    entry.target_a = JointTarget::Body(first);
                }
                entry.body_a = first;
            }
            if target_b_moves {
                if entry.target_b == JointTarget::Body(second) {
                    entry.target_b = JointTarget::Body(first);
                }
                entry.body_b = first;
            }
            entry.joint = entry.joint.with_frames(frame_a, frame_b);
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
            .expect("first cluster must be validated before merge commit");
        cluster.parts.extend(second_parts);
        cluster.welds.extend(second_welds);
        cluster.welds.push(bridge);
        let entry = self
            .bodies
            .get_mut(first.dense)
            .expect("first body must exist during merge commit");
        entry.body.set_motion(prepared);
        entry.body.set_primary_aerodynamics(Aerodynamics::None);
        entry.body.aerodynamic_surfaces = merged_aerodynamics;
        self.bodies
            .remove(second.dense)
            .expect("second body must exist during merge commit");
        if let Some(caches) = &mut self.assembly_aerodynamics {
            caches.remove(&second);
        }
        self.synchronize_cluster_physics_state(&[first]);
        Ok(())
    }

    fn split_after_weld_removal(
        &mut self,
        body: BodyId,
        removed_welds: &[WeldId],
    ) -> Result<Vec<BodyId>, crate::Error> {
        let prepared = self.prepare_weld_split(body, removed_welds)?;
        self.commit_weld_split(prepared)
    }

    /// Performs all geometry, topology, and motion preparation without
    /// changing world state. The result is valid only for this exact world.
    fn prepare_weld_split(
        &self,
        body: BodyId,
        removed_welds: &[WeldId],
    ) -> Result<PreparedWeldSplit, crate::Error> {
        let motion = self
            .body(body)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .motion();
        self.prepare_weld_split_for_motion(body, removed_welds, motion)
    }

    /// Prepare from the motion to be committed, which may be a staged end-of-step
    /// motion rather than the current one. The caller must supply motion for the
    /// same body and must not mutate topology between preparation and commit.
    #[allow(dead_code)]
    fn prepare_weld_split_for_motion(
        &self,
        body: BodyId,
        removed_welds: &[WeldId],
        original: BodyMotion,
    ) -> Result<PreparedWeldSplit, crate::Error> {
        let cluster = self
            .cluster(body)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        let original_root = cluster.root();
        let removed_set = removed_welds.iter().copied().collect::<HashSet<_>>();
        if removed_set.len() != removed_welds.len()
            || removed_welds
                .iter()
                .any(|weld| !cluster.welds().contains(weld))
        {
            return Err(crate::Error::from(ClusterError::InvalidWeldId));
        }
        let mut adjacency = HashMap::<PartId, Vec<PartId>>::new();
        for &part in cluster.parts() {
            adjacency.insert(part, Vec::new());
        }
        let mut remaining_welds = Vec::new();
        for &weld_id in cluster.welds() {
            let weld = self
                .weld(weld_id)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            if removed_set.contains(&weld_id) {
                continue;
            }
            let (a, b) = weld.parts();
            adjacency
                .get_mut(&a)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
                .push(b);
            adjacency
                .get_mut(&b)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
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
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        components.swap(0, root_index);

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
                    .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
                for &collider in entry.colliders() {
                    let collider_entry = self
                        .colliders
                        .get(collider.dense)
                        .filter(|entry| entry.body == body)
                        .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
                    if original.physics().body_type() == BodyType::Dynamic
                        && !collider_entry.collider.shape().is_convex()
                    {
                        return Err(crate::Error::from(ClusterError::UnsupportedGeometry));
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
                    .map_err(map_body_to_cluster)?
                    .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
                let new_center = original
                    .state()
                    .transform()
                    .transform_point(aggregate.center_of_mass());
                if !new_center.is_finite() {
                    return Err(crate::Error::from(ClusterError::ArithmeticOverflow));
                }
                let (velocity, omega) = velocity_field(
                    old_center,
                    original.state().linear_velocity(),
                    original.state().angular_velocity(),
                    new_center,
                )?;
                prepared.set_mass(aggregate).map_err(map_body_to_cluster)?;
                prepared
                    .set_linear_velocity(velocity)
                    .map_err(map_body_to_cluster)?;
                prepared
                    .set_angular_velocity(omega)
                    .map_err(map_body_to_cluster)?;
            }
            prepared_bodies.push(prepared);
        }

        let mut part_rebinds = Vec::new();
        for (component_index, component) in components.iter().enumerate().skip(1) {
            for &part in component {
                let colliders = self
                    .cluster_part(part)
                    .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
                    .colliders()
                    .to_vec();
                let mut collider_transforms = Vec::with_capacity(colliders.len());
                for collider in colliders {
                    let local_transform = self
                        .collider(collider)
                        .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?
                        .local_transform();
                    self.validate_collider_source(collider)?;
                    collider_transforms.push((collider, local_transform));
                }
                part_rebinds.push((component_index - 1, part, collider_transforms));
            }
        }

        let mut part_components = HashMap::new();
        part_components
            .try_reserve(cluster.parts().len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for (index, component) in components.iter().enumerate() {
            for &part in component {
                part_components.insert(part, index);
            }
        }
        // Build every component's final topology before inserting any bodies or
        // removing welds. No fallible topology allocation belongs in commit.
        let mut component_welds = Vec::new();
        component_welds
            .try_reserve_exact(components.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        component_welds.resize_with(components.len(), Vec::new);
        for &weld_id in &remaining_welds {
            let weld = self
                .weld(weld_id)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            let index = *part_components
                .get(&weld.part_a())
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            if part_components.get(&weld.part_b()) != Some(&index) {
                return Err(crate::Error::from(ClusterError::InvalidTopology));
            }
            component_welds[index]
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
            component_welds[index].push(weld_id);
        }
        let mut prepared_clusters = Vec::new();
        prepared_clusters
            .try_reserve_exact(components.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for (index, (parts, welds)) in components.into_iter().zip(component_welds).enumerate() {
            let root = if index == 0 { original_root } else { parts[0] };
            prepared_clusters.push(RigidCluster { root, parts, welds });
        }
        if prepared_clusters.is_empty() {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        let original_aerodynamics = self.cluster_aerodynamic_surfaces(body)?;
        let mut component_aerodynamics = vec![Vec::new(); prepared_clusters.len()];
        for surface in original_aerodynamics {
            let (index, surface) = match surface.target() {
                AerodynamicTarget::Body => (
                    0,
                    PhysicsWorld::rebase_aerodynamic_surface(
                        surface,
                        &original,
                        &prepared_bodies[0],
                    )
                    .map_err(|_| ClusterError::ArithmeticOverflow)?,
                ),
                AerodynamicTarget::Part(part) => (
                    *part_components
                        .get(&part)
                        .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?,
                    surface,
                ),
            };
            component_aerodynamics[index]
                .try_reserve(1)
                .map_err(|_| ClusterError::StorageCapacityExceeded)?;
            component_aerodynamics[index].push(surface);
        }
        let mut component_aerodynamics = component_aerodynamics.into_iter();
        let root_aerodynamics = component_aerodynamics.next().unwrap();
        let collider_tessellation = self
            .bodies
            .get(body.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .collider_tessellation;
        let entries = prepared_bodies
            .iter()
            .skip(1)
            .copied()
            .zip(component_aerodynamics)
            .map(|(motion, aerodynamics)| {
                let mut body = RigidBody::from_motion(motion);
                body.aerodynamic_surfaces = aerodynamics;
                BodyEntry {
                    body,
                    first_collider: None,
                    cluster: None,
                    mesh_aerodynamics: None,
                    collider_aerodynamic_mesh: None,
                    collider_tessellation,
                }
            })
            .collect::<Vec<_>>();
        let mut affected_bodies = Vec::new();
        affected_bodies
            .try_reserve_exact(prepared_clusters.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let mut clusters = prepared_clusters.into_iter();
        let root_cluster = clusters
            .next()
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        let mut removed = Vec::new();
        removed
            .try_reserve_exact(removed_welds.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        removed.extend_from_slice(removed_welds);
        Ok(PreparedWeldSplit {
            body,
            removed_welds: removed,
            entries,
            prepared_bodies,
            part_rebinds,
            part_components,
            root_aerodynamics,
            root_cluster,
            other_clusters: clusters,
            affected_bodies,
        })
    }

    /// Commits a plan before any other world mutation. Batch insertion fails
    /// without mutation; subsequent operations rely on preparation invariants.
    fn commit_weld_split(&mut self, plan: PreparedWeldSplit) -> Result<Vec<BodyId>, crate::Error> {
        let mut plans = Vec::new();
        plans
            .try_reserve_exact(1)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        plans.push(plan);
        let mut results = self.commit_weld_splits(plans)?;
        results
            .pop()
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))
    }

    /// Plans must refer to distinct clusters prepared against the same unchanged
    /// world. All body entries are inserted in one rollback-capable pool batch.
    fn commit_weld_splits(
        &mut self,
        plans: Vec<PreparedWeldSplit>,
    ) -> Result<Vec<Vec<BodyId>>, crate::Error> {
        let staged = self.insert_weld_split_bodies(plans)?;
        Ok(self.finish_weld_splits(staged))
    }

    pub(super) fn insert_weld_split_bodies(
        &mut self,
        mut plans: Vec<PreparedWeldSplit>,
    ) -> Result<InsertedWeldSplits, crate::Error> {
        let mut bodies = HashSet::new();
        bodies
            .try_reserve(plans.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let mut count = 0usize;
        for plan in &plans {
            if !bodies.insert(plan.body) {
                return Err(crate::Error::from(ClusterError::InvalidTopology));
            }
            count = count
                .checked_add(plan.entries.len())
                .ok_or_else(|| crate::Error::from(ClusterError::StorageCapacityExceeded))?;
        }
        let mut entries = Vec::new();
        entries
            .try_reserve_exact(count)
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let mut results = Vec::new();
        results
            .try_reserve_exact(plans.len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for plan in &mut plans {
            entries.append(&mut plan.entries);
        }
        #[cfg(test)]
        let inserted = if let Some(fail_after) = self.weld_split_fail_after.take() {
            self.bodies
                .try_insert_many_failing_after(entries, fail_after)
        } else {
            self.bodies.try_insert_many(entries)
        };
        #[cfg(not(test))]
        let inserted = self.bodies.try_insert_many(entries);
        let inserted = inserted.map_err(|_| ClusterError::StorageCapacityExceeded)?;
        let mut ids = inserted.into_iter();
        for plan in &mut plans {
            plan.affected_bodies.push(plan.body);
            for dense in ids.by_ref().take(plan.other_clusters.len()) {
                plan.affected_bodies.push(BodyId {
                    world: self.world_id,
                    dense,
                });
            }
        }
        Ok(InsertedWeldSplits { plans, results })
    }

    pub(super) fn finish_weld_splits(&mut self, staged: InsertedWeldSplits) -> Vec<Vec<BodyId>> {
        let InsertedWeldSplits { plans, mut results } = staged;
        for plan in plans {
            results.push(self.commit_inserted_weld_split(plan));
        }
        results
    }

    /// Applies a validated plan after its new body entries have been inserted.
    fn commit_inserted_weld_split(&mut self, plan: PreparedWeldSplit) -> Vec<BodyId> {
        let PreparedWeldSplit {
            body,
            removed_welds,
            entries: _,
            prepared_bodies,
            part_rebinds,
            part_components,
            root_aerodynamics,
            root_cluster,
            other_clusters: clusters,
            affected_bodies,
        } = plan;
        let new_bodies = &affected_bodies[1..];
        let entry = self.bodies.get_mut(body.dense).unwrap();
        entry.body.set_primary_aerodynamics(Aerodynamics::None);
        entry.body.aerodynamic_surfaces = root_aerodynamics;

        for weld in removed_welds {
            self.welds
                .remove(weld.dense)
                .expect("weld must be validated before split commit");
        }
        for (target_index, part, collider_transforms) in part_rebinds {
            let target_body = new_bodies[target_index];
            self.parts
                .get_mut(part.dense)
                .expect("cluster part must be validated before split commit")
                .body = target_body;
            for (collider, local_transform) in collider_transforms {
                self.rebind_collider_prepared(collider, target_body, local_transform);
            }
        }
        let entry = self
            .bodies
            .get_mut(body.dense)
            .expect("root body must exist during split commit");
        entry.body.set_motion(prepared_bodies[0]);
        entry.cluster = Some(root_cluster);
        for (component_index, cluster) in clusters.enumerate() {
            let component_index = component_index + 1;
            let target_body = new_bodies[component_index - 1];
            let entry = self
                .bodies
                .get_mut(target_body.dense)
                .expect("split body must exist during commit");
            entry.cluster = Some(cluster);
        }
        let target_body = |index: usize| {
            if index == 0 {
                body
            } else {
                new_bodies[index - 1]
            }
        };
        for entry in self.joints.iter_mut().map(|(_, entry)| entry) {
            if let JointTarget::Part(part) = entry.target_a
                && let Some(&index) = part_components.get(&part)
            {
                entry.body_a = target_body(index);
            }
            if let JointTarget::Part(part) = entry.target_b
                && let Some(&index) = part_components.get(&part)
            {
                entry.body_b = target_body(index);
            }
        }
        for (_, source) in self.force_sources.iter_mut() {
            if let ForceTarget::Part(part) = source.target
                && let Some(&index) = part_components.get(&part)
            {
                source.body = target_body(index);
            }
        }
        self.synchronize_cluster_physics_state(&affected_bodies);
        affected_bodies
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
    ) -> Result<(), Error> {
        self.set_part_mass_checked(part, mass, mode)
    }
    fn set_part_mass_checked(
        &mut self,
        part: PartId,
        mass: MassProperties,
        mode: MassChangeMode,
    ) -> Result<(), crate::Error> {
        if mode == MassChangeMode::ConserveRemainingMomentum {
            return Err(crate::Error::from(ClusterError::InvalidMotion));
        }
        self.set_part_mass_inner(part, mass, None)
    }

    /// Changes a dynamic cluster's mass with explicitly supplied signed impulses.
    pub fn set_part_mass_with_transfer(
        &mut self,
        part: PartId,
        mass: MassProperties,
        transfer: MassTransfer,
    ) -> Result<(), Error> {
        self.set_part_mass_with_transfer_inner(part, mass, transfer)
    }
    fn set_part_mass_with_transfer_inner(
        &mut self,
        part: PartId,
        mass: MassProperties,
        transfer: MassTransfer,
    ) -> Result<(), crate::Error> {
        transfer.validate()?;
        self.set_part_mass_inner(part, mass, Some(transfer))
    }

    fn set_part_mass_inner(
        &mut self,
        part: PartId,
        mass: MassProperties,
        transfer: Option<MassTransfer>,
    ) -> Result<(), crate::Error> {
        let body = self
            .cluster_part(part)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidPartId))?
            .body();
        let cluster = self
            .cluster(body)
            .ok_or_else(|| crate::Error::from(ClusterError::NotClustered))?;
        if !cluster.parts().contains(&part) {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        let mut values = Vec::new();
        values
            .try_reserve(cluster.parts().len())
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        for &id in cluster.parts() {
            let entry = self
                .cluster_part(id)
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
            values.push((
                if id == part { mass } else { entry.mass() },
                entry.local_transform(),
            ));
        }
        let aggregate = aggregate_mass(values.iter().map(|(mass, transform)| (mass, transform)))?;
        let original = self
            .body(body)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?
            .motion();
        let new_center = original
            .state()
            .transform()
            .transform_point(aggregate.center_of_mass());
        if !new_center.is_finite() {
            return Err(crate::Error::from(ClusterError::ArithmeticOverflow));
        }
        if transfer.is_none()
            && self
                .cluster_part(part)
                .is_some_and(|entry| entry.mass() == mass)
        {
            return Ok(());
        }
        if transfer.is_some() && original.physics().body_type() != BodyType::Dynamic {
            return Err(crate::Error::from(ClusterError::IncompatibleMotion));
        }
        let mut prepared = original;
        if original.physics().body_type() == BodyType::Dynamic {
            let old_center = original
                .world_center_of_mass()
                .map_err(map_body_to_cluster)?
                .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
            let (velocity, omega) = if let Some(transfer) = transfer {
                let old_mass = original
                    .physics()
                    .mass()
                    .ok_or_else(|| crate::Error::from(ClusterError::InvalidMass))?;
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
                .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
            let aggregate_changed = original.physics().mass() != Some(aggregate);
            prepared.set_mass(aggregate).map_err(map_body_to_cluster)?;
            prepared
                .set_linear_velocity(velocity)
                .map_err(map_body_to_cluster)?;
            prepared
                .set_angular_velocity(omega)
                .map_err(map_body_to_cluster)?;
            if aggregate_changed {
                prepared.apply_force(force).map_err(map_body_to_cluster)?;
                prepared.apply_torque(torque).map_err(map_body_to_cluster)?;
            }
        }
        let prepared_surfaces = self.prepare_cluster_surface_rebase(body, &original, &prepared)?;
        let target = self
            .parts
            .get_mut(part.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        let entry = self
            .bodies
            .get_mut(body.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?;
        target.mass = mass;
        Self::commit_cluster_surface_rebase(&mut entry.body, prepared_surfaces);
        entry.body.set_motion(prepared);
        let mut collider = entry.first_collider;
        while let Some(id) = collider {
            collider = self.colliders.get(id.dense).and_then(|entry| entry.next);
            self.collision_system.invalidate_collider_contacts(id);
        }
        self.synchronize_cluster_physics_state(&[body]);
        Ok(())
    }

    fn commit_cluster_surface_rebase(body: &mut RigidBody, mut prepared: Vec<AerodynamicSurface>) {
        if matches!(body.aerodynamics(), Aerodynamics::Simple(_))
            && let Some(primary) = prepared.pop()
        {
            body.set_primary_aerodynamics(Aerodynamics::Simple(primary.properties()));
        }
        if prepared.len() <= body.aerodynamic_surfaces.len() {
            body.aerodynamic_surfaces.truncate(prepared.len());
            body.aerodynamic_surfaces.copy_from_slice(&prepared);
        } else {
            body.aerodynamic_surfaces = prepared;
        }
    }

    fn cluster_aerodynamic_surfaces(
        &self,
        body: BodyId,
    ) -> Result<Vec<AerodynamicSurface>, crate::Error> {
        let body = self
            .body(body)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?;
        let primary = match body.aerodynamics() {
            Aerodynamics::None => None,
            Aerodynamics::Simple(properties) => Some(AerodynamicSurface::new(*properties)),
            Aerodynamics::Mesh(_)
            | Aerodynamics::VoxelWave(_)
            | Aerodynamics::IncompressibleSphere(_)
            | Aerodynamics::ContinuumSphere(_)
            | Aerodynamics::CompressionDominatedParticle(_)
            | Aerodynamics::SlipCorrectedParticle(_)
            | Aerodynamics::CreepingParticle(_)
            | Aerodynamics::FreeMolecularParticle(_)
            | Aerodynamics::TransitionalParticle(_)
            | Aerodynamics::Colliders
            | Aerodynamics::Wing(_)
            | Aerodynamics::Wings(_)
            | Aerodynamics::Hull(_) => {
                return Err(crate::Error::from(ClusterError::UnsupportedGeometry));
            }
        };
        let mut surfaces = Vec::new();
        surfaces
            .try_reserve(
                body.aerodynamic_surfaces
                    .len()
                    .saturating_add(usize::from(primary.is_some())),
            )
            .map_err(|_| ClusterError::StorageCapacityExceeded)?;
        surfaces.extend(body.aerodynamic_surfaces.iter().copied());
        surfaces.extend(primary);
        Ok(surfaces)
    }

    fn prepare_cluster_surface_rebase(
        &self,
        body: BodyId,
        original: &BodyMotion,
        prepared: &BodyMotion,
    ) -> Result<Vec<AerodynamicSurface>, crate::Error> {
        let mut surfaces = self.cluster_aerodynamic_surfaces(body)?;
        if original.physics().body_type() == BodyType::Dynamic {
            for surface in &mut surfaces {
                *surface = Self::rebase_aerodynamic_surface(*surface, original, prepared)
                    .map_err(|_| ClusterError::ArithmeticOverflow)?;
            }
        }
        Ok(surfaces)
    }

    #[allow(dead_code)]
    pub(crate) fn rebind_collider(
        &mut self,
        collider: ColliderId,
        body: BodyId,
        local_transform: Transform,
    ) -> Result<(), crate::Error> {
        self.validate_collider_rebind(collider, body)?;
        self.rebind_collider_prepared(collider, body, local_transform);
        Ok(())
    }

    fn validate_collider_source(&self, collider: ColliderId) -> Result<(), crate::Error> {
        if collider.world != self.world_id {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        let entry = self
            .colliders
            .get(collider.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        let old = entry.body;
        let prev = entry.prev;
        let next = entry.next;
        let old_body = self
            .bodies
            .get(old.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
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
        {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        Ok(())
    }

    fn validate_collider_rebind(
        &self,
        collider: ColliderId,
        body: BodyId,
    ) -> Result<(), crate::Error> {
        if collider.world != self.world_id || body.world != self.world_id {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        let target = self
            .bodies
            .get(body.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidBodyId))?;
        let target_first = target.first_collider;
        let entry = self
            .colliders
            .get(collider.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
        if target.body.physics().body_type() == BodyType::Dynamic
            && !entry.collider.shape().is_convex()
        {
            return Err(crate::Error::from(ClusterError::UnsupportedGeometry));
        }
        self.validate_collider_source(collider)?;
        if target_first.is_some_and(|id| {
            self.colliders
                .get(id.dense)
                .is_none_or(|other| other.body != body || other.prev.is_some())
        }) {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        Ok(())
    }

    /// Rebinds a collider whose existence, ownership, and links were validated
    /// by the caller before the commit phase.
    ///
    /// # Panics
    ///
    /// Panics if the collider, one of its linked-list neighbours, or the
    /// target body is missing. This enforces the preconditions instead of
    /// silently leaving the world in a partially updated state.
    fn rebind_collider_prepared(
        &mut self,
        collider: ColliderId,
        body: BodyId,
        local_transform: Transform,
    ) {
        let (old, prev, next, target_first) = {
            let entry = self
                .colliders
                .get(collider.dense)
                .expect("collider must be validated before rebinding");
            (
                entry.body,
                entry.prev,
                entry.next,
                self.bodies
                    .get(body.dense)
                    .expect("rebind target body must exist")
                    .first_collider,
            )
        };
        if old != body {
            if let Some(id) = prev {
                self.colliders
                    .get_mut(id.dense)
                    .expect("previous collider link must exist")
                    .next = next;
            } else {
                self.bodies
                    .get_mut(old.dense)
                    .expect("collider source body must exist")
                    .first_collider = next;
            }
            if let Some(id) = next {
                self.colliders
                    .get_mut(id.dense)
                    .expect("next collider link must exist")
                    .prev = prev;
            }
            if let Some(id) = target_first {
                self.colliders
                    .get_mut(id.dense)
                    .expect("target collider link must exist")
                    .prev = Some(collider);
            }
            self.bodies
                .get_mut(body.dense)
                .expect("rebind target body must exist")
                .first_collider = Some(collider);
        }
        let entry = self
            .colliders
            .get_mut(collider.dense)
            .expect("collider must be validated before rebinding");
        if old != body {
            entry.body = body;
            entry.prev = None;
            entry.next = target_first;
        }
        entry.collider.set_local_transform(local_transform);
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(collider);
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

    /// Stores independent impulse limits on a weld, or clears them with `None`.
    ///
    /// Linear and angular limits are in N·s and N·m·s. This configuration is
    /// checked only for supported isolated discrete contacts when automatic
    /// weld failure and impulse-balance tolerances are configured. An
    /// unbreakable weld cannot be given impulse limits. Invalid input leaves
    /// both the existing limits and prepared step results unchanged.
    pub fn set_weld_impulse_strength(
        &mut self,
        weld: WeldId,
        strength: Option<crate::cluster::impulse::ImpulseStrength>,
    ) -> Result<(), Error> {
        self.set_weld_impulse_strength_inner(weld, strength)
    }
    fn set_weld_impulse_strength_inner(
        &mut self,
        weld: WeldId,
        strength: Option<crate::cluster::impulse::ImpulseStrength>,
    ) -> Result<(), crate::Error> {
        let entry = self
            .weld(weld)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidWeldId))?;
        if let Some(value) = strength
            && (matches!(
                entry.desc().strength,
                crate::cluster::WeldStrength::Unbreakable
            ) || value
                .exceeded_by(crate::cluster::impulse::ImpulseReaction {
                    linear: Vec3::ZERO,
                    angular: Vec3::ZERO,
                })
                .is_err())
        {
            return Err(crate::Error::from(ClusterError::InvalidWeld));
        }
        if entry.impulse_strength() == strength {
            return Ok(());
        }
        self.welds
            .get_mut(weld.dense)
            .ok_or_else(|| crate::Error::from(ClusterError::InvalidWeldId))?
            .impulse_strength = strength;
        self.body_revision = self.body_revision.wrapping_add(1);
        Ok(())
    }

    pub fn part_body(&self, part: PartId) -> Option<BodyId> {
        Some(self.cluster_part(part)?.body())
    }

    pub fn validate_clusters(&self) -> Result<(), Error> {
        self.validate_clusters_inner()
    }
    fn validate_clusters_inner(&self) -> Result<(), crate::Error> {
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
                    .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
                if part.body() != body || !all_parts.insert(part_id) {
                    return Err(crate::Error::from(ClusterError::InvalidTopology));
                }
                adjacency.insert(part_id, Vec::new());
                for &collider in part.colliders() {
                    if self.collider_body(collider) != Some(body) || !all_colliders.insert(collider)
                    {
                        return Err(crate::Error::from(ClusterError::InvalidTopology));
                    }
                    if !part_colliders.insert(collider) {
                        return Err(crate::Error::from(ClusterError::InvalidTopology));
                    }
                }
            }
            if !adjacency.contains_key(&cluster.root()) {
                return Err(crate::Error::from(ClusterError::InvalidTopology));
            }
            for &weld_id in cluster.welds() {
                let weld = self
                    .weld(weld_id)
                    .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
                weld.desc().validate()?;
                let (a, b) = weld.parts();
                if a == b
                    || !all_welds.insert(weld_id)
                    || !adjacency.contains_key(&a)
                    || !adjacency.contains_key(&b)
                {
                    return Err(crate::Error::from(ClusterError::InvalidTopology));
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
                return Err(crate::Error::from(ClusterError::InvalidTopology));
            }
            let mut collider = entry.first_collider;
            let mut previous = None;
            let mut linked_colliders = HashSet::new();
            while let Some(id) = collider {
                if !all_colliders.contains(&id) || !linked_colliders.insert(id) {
                    return Err(crate::Error::from(ClusterError::InvalidTopology));
                }
                let collider_entry = self
                    .colliders
                    .get(id.dense)
                    .ok_or_else(|| crate::Error::from(ClusterError::InvalidTopology))?;
                if collider_entry.body != body || collider_entry.prev != previous {
                    return Err(crate::Error::from(ClusterError::InvalidTopology));
                }
                previous = Some(id);
                collider = collider_entry.next;
            }
            if part_colliders != linked_colliders {
                return Err(crate::Error::from(ClusterError::InvalidTopology));
            }
        }
        if all_parts.len() != self.parts.len() || all_welds.len() != self.welds.len() {
            return Err(crate::Error::from(ClusterError::InvalidTopology));
        }
        Ok(())
    }
}

fn validate_weld_frames(
    transform_a: Transform,
    transform_b: Transform,
    weld: crate::cluster::WeldDesc,
) -> Result<(), crate::Error> {
    let frame_a =
        super::transform_joint_frame(transform_a, weld.frame_a).map_err(ClusterError::Transform)?;
    let frame_b =
        super::transform_joint_frame(transform_b, weld.frame_b).map_err(ClusterError::Transform)?;
    let anchor_delta = frame_a
        .local_anchor
        .checked_sub(frame_b.local_anchor)
        .ok_or_else(|| crate::Error::from(ClusterError::ArithmeticOverflow))?;
    let anchor_scale = frame_a
        .local_anchor
        .abs()
        .max_element()
        .max(frame_b.local_anchor.abs().max_element())
        .max(1.0);
    let tolerance = 128.0 * f64::EPSILON * anchor_scale;
    if anchor_delta.length() > tolerance {
        return Err(crate::Error::from(ClusterError::IncompatibleAnchors));
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
        return Err(crate::Error::from(ClusterError::IncompatibleAnchors));
    }
    Ok(())
}

#[cfg(test)]
#[path = "cluster_tests.rs"]
mod tests;

fn map_body_to_cluster(error: crate::Error) -> ClusterError {
    match error.downcast_ref::<crate::rigid_body::BodyError>() {
        Some(error) => ClusterError::Body(crate::Error::from(*error)),
        None => ClusterError::Body(crate::Error::from(
            crate::rigid_body::BodyError::NonFiniteValue,
        )),
    }
}
