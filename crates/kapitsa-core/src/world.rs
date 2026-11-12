use kapitsa_math::{Affine3, Vec3};
use std::{
    collections::HashMap,
    panic::{AssertUnwindSafe, catch_unwind},
    sync::atomic::{AtomicU64, Ordering},
};

#[cfg(feature = "bench-internals")]
use crate::collision::ContactManifold;
#[cfg(feature = "bench-internals")]
use std::time::Instant;

mod aerodynamics {
    use super::{BodyId, PartId, PhysicsWorld, StepError};
    use crate::aerodynamics::{AerodynamicProperties, Atmosphere};

    /// Identifies the rigid object that owns an aerodynamic surface.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum AerodynamicTarget {
        Body,
        Part(PartId),
    }

    /// Aerodynamic properties attached to a body or one of its cluster parts.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct AerodynamicSurface {
        properties: AerodynamicProperties,
        target: AerodynamicTarget,
    }

    impl AerodynamicSurface {
        pub fn new(properties: AerodynamicProperties) -> Self {
            Self {
                properties,
                target: AerodynamicTarget::Body,
            }
        }

        pub fn for_part(part: PartId, properties: AerodynamicProperties) -> Self {
            Self {
                properties,
                target: AerodynamicTarget::Part(part),
            }
        }

        pub fn properties(self) -> AerodynamicProperties {
            self.properties
        }

        pub fn target(self) -> AerodynamicTarget {
            self.target
        }

        pub(crate) fn with_properties(self, properties: AerodynamicProperties) -> Self {
            Self { properties, ..self }
        }
    }

    /// Checks the current aerodynamic force calculation without advancing the world.
    ///
    /// This preflight does not apply forces or check timestep-dependent integration.
    /// Use [`PhysicsWorld::step`] to advance the simulation.
    #[derive(Clone, Copy, Debug, Default)]
    pub struct AerodynamicsTask;

    impl AerodynamicsTask {
        pub fn execute(self, world: &PhysicsWorld) -> Result<(), StepError> {
            let atmosphere = world.atmosphere;
            if matches!(atmosphere, Atmosphere::Vacuum) {
                return Ok(());
            };
            for (dense, entry) in world.bodies.iter() {
                if entry.aerodynamics.is_empty() {
                    continue;
                }
                let mut body = entry.body;
                PhysicsWorld::apply_aerodynamics(
                    atmosphere,
                    &entry.aerodynamics,
                    BodyId {
                        world: world.world_id,
                        dense,
                    },
                    &mut body,
                    &world.parts,
                    None,
                )?;
            }
            Ok(())
        }
    }
}
#[cfg(test)]
mod aerodynamics_tests;
mod ccd;
mod chunk;
mod cluster;
mod error;
#[cfg(test)]
mod gravity_fields_tests;
mod id;
#[cfg(feature = "bench-internals")]
mod metrics;
mod query;

pub use aerodynamics::{AerodynamicSurface, AerodynamicTarget, AerodynamicsTask};
pub use chunk::{AddChunkBatchError, AddChunkError, ChunkOperationError, RemoveChunkBatchError};
pub use error::{ClusterError, ForceSourceError, StepError, WorldError};
pub use id::{BodyId, ChunkId, ColliderId, ForceSourceId, GravityId, JointId, PartId, WeldId};
#[cfg(feature = "bench-internals")]
pub use metrics::PhysicsStepMetrics;

static NEXT_WORLD_ID: AtomicU64 = AtomicU64::new(1);

fn allocate_world_id(counter: &AtomicU64) -> Result<u64, WorldError> {
    counter
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |next| {
            next.checked_add(1)
        })
        .map_err(|_| WorldError::WorldIdExhausted)
}

use crate::{
    aabb::Aabb,
    aerodynamics::{AerodynamicProperties, Atmosphere},
    cluster::{ClusterPart, RigidCluster, Weld},
    collider::Collider,
    collision::{
        CcdMotion, CcdTrajectory, CollisionConfig, CollisionError, CollisionEvent,
        CollisionInteraction, CollisionPair, CollisionSystem, ContactInfo, SolverBody,
        SolverManifold, SolverMaterial, StepContext, spatial_index::SpatialIndex,
    },
    containers::DensePool,
    execution::{SequentialExecutor, StepExecutor},
    gravity::{CentralGravity, Gravity, GravityError},
    joint::{Joint, JointBreakEvent, JointFrame},
    material::{MaterialId, MaterialStorage, PhysicsMaterial},
    rigid_body::{BodyState, BodyType, CcdMode, RigidBody, predict},
};

struct BodyEntry {
    body: RigidBody,
    aerodynamics: Vec<AerodynamicSurface>,
    first_collider: Option<ColliderId>,
    cluster: Option<RigidCluster>,
}

/// Object receiving a persistent force source.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ForceTarget {
    Body(BodyId),
    Part(PartId),
}

/// Persistent force applied at a point expressed in body-local coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ForceSource {
    body: BodyId,
    target: ForceTarget,
    local_point: Vec3,
    local_force: Vec3,
    enabled: bool,
}

impl ForceSource {
    /// Returns the dynamic body receiving this force.
    pub fn body(&self) -> BodyId {
        self.body
    }

    /// Returns the body or cluster part receiving this force.
    pub fn target(&self) -> ForceTarget {
        self.target
    }

    /// Returns the application point relative to the body's origin, in meters.
    pub fn local_point(&self) -> Vec3 {
        self.local_point
    }

    /// Returns the force vector in body-local coordinates, in newtons.
    pub fn local_force(&self) -> Vec3 {
        self.local_force
    }

    /// Returns whether this source is applied on each physics step.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

struct ForceSourceBodyUpdate {
    id: BodyId,
    original: RigidBody,
    prepared: RigidBody,
}

struct JointEntry {
    target_a: JointTarget,
    target_b: JointTarget,
    body_a: BodyId,
    body_b: BodyId,
    joint: Joint,
}

/// Object used as one endpoint of a persistent joint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointTarget {
    Body(BodyId),
    Part(PartId),
}

struct GravityBodyUpdate {
    id: BodyId,
    error: Option<StepError>,
    original: RigidBody,
    prepared: RigidBody,
}

struct AerodynamicsBodyUpdate {
    id: BodyId,
    error: Option<StepError>,
    original: RigidBody,
    prepared: RigidBody,
}

const FORCE_SOURCE_BODY_LINEAR_SEARCH_LIMIT: usize = 128;

struct ColliderEntry {
    collider: Collider,
    body: BodyId,
    prev: Option<ColliderId>,
    next: Option<ColliderId>,
    chunk: Option<ChunkId>,
}

#[derive(Clone, Copy)]
struct CcdBodyReplaySnapshot {
    prediction: usize,
    ccd_state: BodyState,
    predicted_state: BodyState,
    elapsed: f64,
    impact_count: u32,
    clipped: bool,
    completed: bool,
}

fn swept_bounds_for_trajectory(
    collider: &Collider,
    trajectory: &CcdTrajectory,
    end: BodyState,
) -> Result<Option<crate::aabb::Aabb>, CollisionError> {
    let local_bounds = collider.local_bounds()?;
    let start_transform = trajectory.transform_at(0.0)?;
    let end_transform = end
        .transform()
        .compose(&trajectory.local_transform())
        .map_err(|_| CollisionError::ArithmeticOverflow)?;
    if trajectory.has_linear_collider_origin(end) {
        let motion = CcdMotion::linear(start_transform, end_transform)?;
        return crate::collision::swept_bounds_motion(local_bounds, &motion);
    }
    let (translation_bound, rotation_bound, _) = trajectory.path_bounds(end)?;
    let motion = CcdMotion::trajectory(
        start_transform,
        end_transform,
        translation_bound,
        false,
        rotation_bound,
        *trajectory,
    )?;
    crate::collision::swept_bounds_motion(local_bounds, &motion)
}

/// Owns rigid bodies and colliders and assigns IDs scoped to this world instance.
pub struct PhysicsWorld {
    world_id: u64,
    bodies: DensePool<BodyEntry>,
    parts: DensePool<ClusterPart>,
    welds: DensePool<Weld>,
    colliders: DensePool<ColliderEntry>,
    chunks: DensePool<chunk::ChunkEntry>,
    force_sources: DensePool<ForceSource>,
    joints: DensePool<JointEntry>,
    force_source_body_updates: Vec<ForceSourceBodyUpdate>,
    force_source_body_indices: Option<HashMap<BodyId, usize>>,
    central_gravity: Option<CentralGravity>,
    gravity_fields: DensePool<Gravity>,
    gravity_body_updates: Vec<GravityBodyUpdate>,
    atmosphere: Atmosphere,
    aerodynamics_body_updates: Vec<AerodynamicsBodyUpdate>,
    materials: MaterialStorage,
    collision_system: CollisionSystem,
    joint_events: Vec<JointBreakEvent>,
    query_index: Option<SpatialIndex>,
    query_index_dirty: bool,
    query_proxies: Vec<(ColliderId, Aabb)>,
    unbounded_query_colliders: Vec<ColliderId>,
}

/// Opaque copied body data used to predict and atomically commit a world step.
#[doc(hidden)]
#[derive(Clone, Copy, Debug)]
pub struct BodyStep {
    body_id: BodyId,
    source: RigidBody,
    predicted: Option<BodyState>,
    dt: Option<f64>,
    error: Option<StepError>,
    prepared: RigidBody,
}

impl BodyStep {
    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    pub fn integrate(&mut self, dt: f64) -> Result<(), StepError> {
        self.predicted = None;
        self.dt = None;
        self.error = None;
        let predicted = match predict(&self.prepared, dt) {
            Ok(predicted) => predicted,
            Err(cause) => {
                let error = StepError::BodyIntegration {
                    body_id: self.body_id,
                    cause,
                };
                self.error = Some(error);
                return Err(error);
            }
        };
        self.predicted = Some(predicted);
        self.dt = Some(dt);
        Ok(())
    }
}

impl PhysicsWorld {
    pub const fn new() -> Self {
        Self {
            world_id: 0,
            bodies: DensePool::new(),
            parts: DensePool::new(),
            welds: DensePool::new(),
            colliders: DensePool::new(),
            chunks: DensePool::new(),
            force_sources: DensePool::new(),
            joints: DensePool::new(),
            force_source_body_updates: Vec::new(),
            force_source_body_indices: None,
            central_gravity: None,
            gravity_fields: DensePool::new(),
            gravity_body_updates: Vec::new(),
            atmosphere: Atmosphere::Vacuum,
            aerodynamics_body_updates: Vec::new(),
            materials: MaterialStorage::new(),
            collision_system: CollisionSystem::new(),
            joint_events: Vec::new(),
            query_index: None,
            query_index_dirty: false,
            query_proxies: Vec::new(),
            unbounded_query_colliders: Vec::new(),
        }
    }

    pub fn collision_config(&self) -> CollisionConfig {
        self.collision_system.config()
    }

    pub fn central_gravity(&self) -> Option<CentralGravity> {
        self.central_gravity
    }

    pub fn set_central_gravity(&mut self, gravity: Option<CentralGravity>) {
        self.central_gravity = gravity;
    }

    pub fn atmosphere(&self) -> Atmosphere {
        self.atmosphere
    }

    pub fn set_atmosphere(&mut self, atmosphere: Atmosphere) {
        self.atmosphere = atmosphere;
    }

    pub fn aerodynamics(&self, id: BodyId) -> Option<AerodynamicProperties> {
        if id.world != self.world_id {
            return None;
        }
        self.bodies.get(id.dense).and_then(|entry| {
            entry
                .aerodynamics
                .iter()
                .find(|surface| surface.target() == AerodynamicTarget::Body)
                .map(|surface| surface.properties())
        })
    }

    /// Replaces body surfaces. All previously returned surface indices are invalidated.
    pub fn set_aerodynamics(
        &mut self,
        id: BodyId,
        properties: Option<AerodynamicProperties>,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        let surfaces = &mut self
            .bodies
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .aerodynamics;
        if properties.is_some()
            && !surfaces
                .iter()
                .any(|surface| surface.target() == AerodynamicTarget::Body)
        {
            surfaces
                .try_reserve(1)
                .map_err(|_| WorldError::DensePoolCapacityExceeded)?;
        }
        surfaces.retain(|surface| surface.target() != AerodynamicTarget::Body);
        if let Some(properties) = properties {
            surfaces.push(AerodynamicSurface::new(properties));
        }
        Ok(())
    }

    pub fn aerodynamic_surfaces(&self, id: BodyId) -> Option<&[AerodynamicSurface]> {
        if id.world != self.world_id {
            return None;
        }
        self.bodies
            .get(id.dense)
            .map(|entry| entry.aerodynamics.as_slice())
    }

    /// Returns an index valid until the surface list changes or the cluster is split or merged.
    pub fn add_aerodynamic_surface(
        &mut self,
        id: BodyId,
        properties: AerodynamicProperties,
    ) -> Result<usize, WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        let surfaces = &mut self
            .bodies
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .aerodynamics;
        surfaces
            .try_reserve(1)
            .map_err(|_| WorldError::DensePoolCapacityExceeded)?;
        surfaces.push(AerodynamicSurface::new(properties));
        Ok(surfaces.len() - 1)
    }

    /// Removes a surface by its current index, invalidating previously returned indices.
    pub fn remove_aerodynamic_surface(
        &mut self,
        id: BodyId,
        index: usize,
    ) -> Result<AerodynamicProperties, WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        let surfaces = &mut self
            .bodies
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .aerodynamics;
        if index >= surfaces.len() {
            return Err(WorldError::InvalidAerodynamicSurface);
        }
        Ok(surfaces.remove(index).properties())
    }

    /// Returns an index valid until the surface list changes or the cluster is split or merged.
    pub fn add_aerodynamic_surface_for_part(
        &mut self,
        part: PartId,
        properties: AerodynamicProperties,
    ) -> Result<usize, WorldError> {
        let part_entry = self.cluster_part(part).ok_or(WorldError::InvalidBodyId)?;
        let body = part_entry.body();
        let surfaces = &mut self
            .bodies
            .get_mut(body.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .aerodynamics;
        surfaces
            .try_reserve(1)
            .map_err(|_| WorldError::DensePoolCapacityExceeded)?;
        surfaces.push(AerodynamicSurface::for_part(part, properties));
        Ok(surfaces.len() - 1)
    }

    pub(crate) fn rebase_aerodynamic_surface(
        surface: AerodynamicSurface,
        old_body: &RigidBody,
        new_body: &RigidBody,
    ) -> Result<AerodynamicSurface, crate::aerodynamics::AerodynamicsError> {
        if !matches!(surface.target(), AerodynamicTarget::Body) {
            return Ok(surface);
        }
        let old_center = old_body
            .world_center_of_mass()
            .map_err(|_| crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?
            .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
        let new_center = new_body
            .world_center_of_mass()
            .map_err(|_| crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?
            .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
        let world_point = old_center
            .checked_add(
                old_body
                    .state()
                    .transform()
                    .transform_vector(surface.properties().local_application_offset()),
            )
            .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
        let inverse_rotation = new_body
            .state()
            .transform()
            .rotation()
            .conjugate()
            .rotate_vector(
                world_point
                    .checked_sub(new_center)
                    .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?,
            )
            .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
        let mut properties = surface
            .properties()
            .with_local_application_offset(inverse_rotation)?;
        if let Some(axis) = surface.properties().local_lift_axis() {
            let world_axis = old_body
                .state()
                .transform()
                .rotation()
                .rotate_vector(axis)
                .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
            let local_axis = new_body
                .state()
                .transform()
                .rotation()
                .conjugate()
                .rotate_vector(world_axis)
                .ok_or(crate::aerodynamics::AerodynamicsError::ArithmeticOverflow)?;
            properties =
                properties.with_lift(surface.properties().lift_coefficient(), local_axis)?;
        }
        Ok(surface.with_properties(properties))
    }

    /// Adds an independently managed gravity field.
    pub fn add_gravity(&mut self, gravity: impl Into<Gravity>) -> Result<GravityId, WorldError> {
        if self.world_id == 0 {
            self.world_id = allocate_world_id(&NEXT_WORLD_ID)?;
        }
        let dense = self.gravity_fields.insert(gravity.into())?;
        Ok(GravityId {
            world: self.world_id,
            dense,
        })
    }

    pub fn gravity(&self, id: GravityId) -> Option<Gravity> {
        (id.world == self.world_id)
            .then(|| self.gravity_fields.get(id.dense).copied())
            .flatten()
    }

    pub fn gravity_fields(&self) -> impl Iterator<Item = (GravityId, Gravity)> + '_ {
        self.gravity_fields.iter().map(|(dense, field)| {
            (
                GravityId {
                    world: self.world_id,
                    dense,
                },
                *field,
            )
        })
    }

    pub fn update_gravity(
        &mut self,
        id: GravityId,
        gravity: impl Into<Gravity>,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidGravityId);
        }
        *self
            .gravity_fields
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidGravityId)? = gravity.into();
        Ok(())
    }

    pub fn remove_gravity(&mut self, id: GravityId) -> Option<Gravity> {
        if id.world != self.world_id {
            return None;
        }
        self.gravity_fields.remove(id.dense)
    }

    pub fn set_collision_config(&mut self, config: CollisionConfig) -> Result<(), CollisionError> {
        self.collision_system.set_config(config)
    }

    pub fn collision_events(&self) -> &[CollisionEvent] {
        self.collision_system.events()
    }

    /// Returns joint-break events committed by the latest successful step.
    pub fn joint_events(&self) -> &[JointBreakEvent] {
        &self.joint_events
    }

    pub fn add_material(&mut self, material: PhysicsMaterial) -> Result<MaterialId, WorldError> {
        self.materials.insert(material).map_err(WorldError::from)
    }

    pub fn material(&self, id: MaterialId) -> Option<&PhysicsMaterial> {
        self.materials.get(id)
    }

    pub fn material_mut(&mut self, id: MaterialId) -> Option<&mut PhysicsMaterial> {
        self.materials.get_mut(id)
    }

    /// Replaces a material and invalidates cached contacts that used it.
    pub fn update_material(
        &mut self,
        id: MaterialId,
        material: PhysicsMaterial,
    ) -> Result<(), WorldError> {
        let current = self
            .materials
            .get_mut(id)
            .ok_or(WorldError::InvalidMaterialId)?;
        if *current == material {
            return Ok(());
        }
        *current = material;
        for (dense, entry) in self.colliders.iter() {
            if entry.collider.material() == Some(id) {
                self.collision_system
                    .invalidate_collider_contacts(ColliderId {
                        world: self.world_id,
                        dense,
                    });
            }
        }
        Ok(())
    }

    pub fn remove_material(&mut self, id: MaterialId) -> Result<PhysicsMaterial, WorldError> {
        if !self.materials.contains(id) {
            return Err(WorldError::InvalidMaterialId);
        }
        if self
            .colliders
            .iter()
            .any(|(_, entry)| entry.collider.material() == Some(id))
        {
            return Err(WorldError::MaterialInUse);
        }
        self.materials
            .remove(id)
            .ok_or(WorldError::InvalidMaterialId)
    }

    pub fn set_collider_material(
        &mut self,
        id: ColliderId,
        material: Option<MaterialId>,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id || self.colliders.get(id.dense).is_none() {
            return Err(WorldError::InvalidColliderId);
        }
        if material.is_some_and(|material| !self.materials.contains(material)) {
            return Err(WorldError::InvalidMaterialId);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidColliderId)?;
        entry.collider.set_material(material);
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Ok(())
    }

    /// Sets whether an attached collider responds physically or only reports intersections.
    pub fn set_collider_mode(
        &mut self,
        id: ColliderId,
        mode: crate::collider::CollisionMode,
    ) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidColliderId);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidColliderId)?;
        if entry.collider.mode() != mode {
            entry.collider.set_mode(mode);
            self.collision_system.invalidate_collider_contacts(id);
        }
        Ok(())
    }

    pub fn add_body(&mut self, body: RigidBody) -> Result<BodyId, WorldError> {
        if self.world_id == 0 {
            self.world_id = allocate_world_id(&NEXT_WORLD_ID)?;
        }
        let dense = self
            .bodies
            .insert(BodyEntry {
                body,
                aerodynamics: Vec::new(),
                first_collider: None,
                cluster: None,
            })
            .map_err(WorldError::from)?;
        Ok(BodyId {
            world: self.world_id,
            dense,
        })
    }

    pub fn body(&self, id: BodyId) -> Option<&RigidBody> {
        if id.world != self.world_id {
            return None;
        }
        self.bodies.get(id.dense).map(|entry| &entry.body)
    }

    /// Returns mutable access only for bodies whose mass is not cluster-managed.
    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut RigidBody> {
        if id.world != self.world_id || self.bodies.get(id.dense)?.cluster.is_some() {
            return None;
        }
        self.invalidate_body_edit(id);
        self.bodies.get_mut(id.dense).map(|entry| &mut entry.body)
    }

    /// Applies a validated edit to a copy before updating a body in this world.
    /// Cluster-owned mass and motion type must remain unchanged.
    pub fn edit_body(
        &mut self,
        id: BodyId,
        edit: impl FnOnce(&mut RigidBody) -> Result<(), crate::rigid_body::BodyError>,
    ) -> Result<(), ClusterError> {
        let mut body = *self.body(id).ok_or(ClusterError::InvalidBodyId)?;
        edit(&mut body).map_err(ClusterError::Body)?;
        self.replace_body_state(id, body)
    }

    /// Replaces body state while protecting cluster-managed physical properties.
    pub fn replace_body_state(&mut self, id: BodyId, body: RigidBody) -> Result<(), ClusterError> {
        if id.world != self.world_id {
            return Err(ClusterError::InvalidBodyId);
        }
        let entry = self
            .bodies
            .get(id.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        if entry.cluster.is_some() && entry.body.physics() != body.physics() {
            return Err(ClusterError::ManagedMass);
        }
        self.invalidate_body_edit(id);
        let entry = self
            .bodies
            .get_mut(id.dense)
            .ok_or(ClusterError::InvalidBodyId)?;
        entry.body = body;
        Ok(())
    }

    fn invalidate_body_edit(&mut self, id: BodyId) {
        self.synchronize_cluster_physics_state(&[id]);
    }

    pub(crate) fn synchronize_cluster_physics_state(&mut self, bodies: &[BodyId]) {
        let mut colliders = Vec::new();
        for &body in bodies {
            let mut next = self
                .bodies
                .get(body.dense)
                .and_then(|entry| entry.first_collider);
            while let Some(id) = next {
                colliders.push(id);
                next = self.colliders.get(id.dense).and_then(|entry| entry.next);
            }
        }
        for collider in colliders {
            self.collision_system.invalidate_collider_contacts(collider);
        }
        if self.collision_system.joint_warm_start_enabled() {
            for (dense, entry) in self.joints.iter() {
                if bodies.contains(&entry.body_a) || bodies.contains(&entry.body_b) {
                    self.collision_system.invalidate_joint_warm_start(JointId {
                        world: self.world_id,
                        dense,
                    });
                }
            }
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Adds a constraint between two existing, distinct bodies.
    pub fn add_joint(
        &mut self,
        body_a: BodyId,
        body_b: BodyId,
        joint: Joint,
    ) -> Result<JointId, WorldError> {
        if body_a == body_b || self.body(body_a).is_none() || self.body(body_b).is_none() {
            return Err(WorldError::InvalidJointBodies);
        }
        joint.validate()?;
        let dense = self
            .joints
            .insert(JointEntry {
                target_a: JointTarget::Body(body_a),
                target_b: JointTarget::Body(body_b),
                body_a,
                body_b,
                joint,
            })
            .map_err(WorldError::from)?;
        Ok(JointId {
            world: self.world_id,
            dense,
        })
    }

    /// Adds a constraint between two body or cluster-part targets.
    pub fn add_joint_targets(
        &mut self,
        target_a: JointTarget,
        target_b: JointTarget,
        joint: Joint,
    ) -> Result<JointId, WorldError> {
        let (body_a, frame_a) = self.resolve_joint_target(target_a)?;
        let (body_b, frame_b) = self.resolve_joint_target(target_b)?;
        if target_a == target_b {
            return Err(WorldError::InvalidJointBodies);
        }
        joint.validate()?;
        let frame_a = transform_joint_frame(frame_a, joint.common().frame_a)
            .map_err(|_| WorldError::InvalidJointTarget)?;
        let frame_b = transform_joint_frame(frame_b, joint.common().frame_b)
            .map_err(|_| WorldError::InvalidJointTarget)?;
        let dense = self
            .joints
            .insert(JointEntry {
                target_a,
                target_b,
                body_a,
                body_b,
                joint: joint.with_frames(frame_a, frame_b),
            })
            .map_err(WorldError::from)?;
        Ok(JointId {
            world: self.world_id,
            dense,
        })
    }

    fn resolve_joint_target(
        &self,
        target: JointTarget,
    ) -> Result<(BodyId, crate::transform::Transform), WorldError> {
        match target {
            JointTarget::Body(body) => self
                .body(body)
                .map(|_| (body, crate::transform::Transform::identity()))
                .ok_or(WorldError::InvalidJointTarget),
            JointTarget::Part(part) => self
                .cluster_part(part)
                .and_then(|entry| {
                    self.cluster(entry.body())
                        .map(|_| (entry.body(), entry.local_transform()))
                })
                .ok_or(WorldError::InvalidJointTarget),
        }
    }

    /// Returns the configuration of a live joint in this world.
    pub fn joint(&self, id: JointId) -> Option<&Joint> {
        if id.world != self.world_id {
            return None;
        }
        self.joints.get(id.dense).map(|entry| &entry.joint)
    }

    /// Returns the two bodies connected by a live joint.
    pub fn joint_bodies(&self, id: JointId) -> Option<(BodyId, BodyId)> {
        if id.world != self.world_id {
            return None;
        }
        self.joints
            .get(id.dense)
            .map(|entry| (entry.body_a, entry.body_b))
    }

    pub fn joint_targets(&self, id: JointId) -> Option<(JointTarget, JointTarget)> {
        if id.world != self.world_id {
            return None;
        }
        self.joints
            .get(id.dense)
            .map(|entry| (entry.target_a, entry.target_b))
    }

    /// Replaces a joint's settings while retaining its ID and attached bodies.
    pub fn update_joint(&mut self, id: JointId, joint: Joint) -> Result<(), WorldError> {
        joint.validate()?;
        if id.world != self.world_id {
            return Err(WorldError::InvalidJointId);
        }
        let entry = self
            .joints
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidJointId)?;
        entry.joint = joint;
        self.collision_system.invalidate_joint_warm_start(id);
        Ok(())
    }

    /// Replaces a joint using frames expressed in the current target spaces.
    pub fn update_joint_targets(&mut self, id: JointId, joint: Joint) -> Result<(), WorldError> {
        joint.validate()?;
        if id.world != self.world_id {
            return Err(WorldError::InvalidJointId);
        }
        let (target_a, target_b) = self
            .joints
            .get(id.dense)
            .map(|entry| (entry.target_a, entry.target_b))
            .ok_or(WorldError::InvalidJointId)?;
        let (_, frame_a_target) = self.resolve_joint_target(target_a)?;
        let (_, frame_b_target) = self.resolve_joint_target(target_b)?;
        let frame_a = transform_joint_frame(frame_a_target, joint.common().frame_a)
            .map_err(|_| WorldError::InvalidJointTarget)?;
        let frame_b = transform_joint_frame(frame_b_target, joint.common().frame_b)
            .map_err(|_| WorldError::InvalidJointTarget)?;
        let entry = self
            .joints
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidJointId)?;
        entry.joint = joint.with_frames(frame_a, frame_b);
        self.collision_system.invalidate_joint_warm_start(id);
        Ok(())
    }

    /// Enables or disables a live joint without changing its settings.
    pub fn set_joint_enabled(&mut self, id: JointId, enabled: bool) -> Result<(), WorldError> {
        if id.world != self.world_id {
            return Err(WorldError::InvalidJointId);
        }
        self.joints
            .get_mut(id.dense)
            .ok_or(WorldError::InvalidJointId)?
            .joint
            .set_enabled(enabled);
        if !enabled {
            self.collision_system.invalidate_joint_warm_start(id);
        }
        Ok(())
    }

    /// Removes a live joint and returns its final configuration.
    pub fn remove_joint(&mut self, id: JointId) -> Option<Joint> {
        if id.world != self.world_id {
            return None;
        }
        let removed = self.joints.remove(id.dense).map(|entry| entry.joint);
        if removed.is_some() {
            self.collision_system.invalidate_joint_warm_start(id);
        }
        removed
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Iterates over joints in dense storage order, which may change after removal.
    pub fn joints(&self) -> impl Iterator<Item = (JointId, BodyId, BodyId, &Joint)> {
        self.joints.iter().map(|(dense, entry)| {
            (
                JointId {
                    world: self.world_id,
                    dense,
                },
                entry.body_a,
                entry.body_b,
                &entry.joint,
            )
        })
    }

    /// Adds a persistent force source to an existing dynamic body.
    pub fn add_force_source(
        &mut self,
        body: BodyId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<ForceSourceId, ForceSourceError> {
        let Some(body_state) = self.body(body) else {
            return Err(ForceSourceError::InvalidBodyId);
        };
        if body_state.physics().body_type() != BodyType::Dynamic {
            return Err(ForceSourceError::BodyMustBeDynamic);
        }
        validate_force_source_values(local_point, local_force)?;
        let dense = self
            .force_sources
            .insert(ForceSource {
                body,
                target: ForceTarget::Body(body),
                local_point,
                local_force,
                enabled: true,
            })
            .map_err(|_| ForceSourceError::StorageCapacityExceeded)?;
        Ok(ForceSourceId {
            world: self.world_id,
            dense,
        })
    }

    /// Adds a persistent force source attached to a part of a rigid cluster.
    pub fn add_force_source_for_part(
        &mut self,
        part: PartId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<ForceSourceId, ForceSourceError> {
        let part_entry = self
            .cluster_part(part)
            .ok_or(ForceSourceError::InvalidPartId)?;
        let body = part_entry.body();
        let Some(cluster) = self.cluster(body) else {
            return Err(ForceSourceError::InvalidPartId);
        };
        if !cluster.parts().contains(&part) {
            return Err(ForceSourceError::InvalidPartId);
        }
        let Some(body_state) = self.body(body) else {
            return Err(ForceSourceError::InvalidBodyId);
        };
        if body_state.physics().body_type() != BodyType::Dynamic {
            return Err(ForceSourceError::BodyMustBeDynamic);
        }
        validate_force_source_values(local_point, local_force)?;
        let dense = self
            .force_sources
            .insert(ForceSource {
                body,
                target: ForceTarget::Part(part),
                local_point,
                local_force,
                enabled: true,
            })
            .map_err(|_| ForceSourceError::StorageCapacityExceeded)?;
        Ok(ForceSourceId {
            world: self.world_id,
            dense,
        })
    }

    /// Looks up a force source by its world-scoped ID.
    pub fn force_source(&self, id: ForceSourceId) -> Option<&ForceSource> {
        (id.world == self.world_id)
            .then(|| self.force_sources.get(id.dense))
            .flatten()
    }

    /// Iterates over persistent force sources in dense storage order.
    pub fn force_sources(&self) -> impl Iterator<Item = (ForceSourceId, &ForceSource)> {
        self.force_sources.iter().map(|(dense, source)| {
            (
                ForceSourceId {
                    world: self.world_id,
                    dense,
                },
                source,
            )
        })
    }

    /// Replaces the local application point and force vector of a source.
    pub fn set_force_source(
        &mut self,
        id: ForceSourceId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<(), ForceSourceError> {
        validate_force_source_values(local_point, local_force)?;
        let source = self
            .force_source_mut(id)
            .ok_or(ForceSourceError::InvalidForceSourceId)?;
        source.local_point = local_point;
        source.local_force = local_force;
        Ok(())
    }

    /// Enables or disables a source without removing it.
    pub fn set_force_source_enabled(
        &mut self,
        id: ForceSourceId,
        enabled: bool,
    ) -> Result<(), ForceSourceError> {
        self.force_source_mut(id)
            .ok_or(ForceSourceError::InvalidForceSourceId)?
            .enabled = enabled;
        Ok(())
    }

    /// Removes a source and returns its last configuration.
    pub fn remove_force_source(
        &mut self,
        id: ForceSourceId,
    ) -> Result<ForceSource, ForceSourceError> {
        if id.world != self.world_id {
            return Err(ForceSourceError::InvalidForceSourceId);
        }
        self.force_sources
            .remove(id.dense)
            .ok_or(ForceSourceError::InvalidForceSourceId)
    }

    fn force_source_mut(&mut self, id: ForceSourceId) -> Option<&mut ForceSource> {
        if id.world != self.world_id {
            return None;
        }
        self.force_sources.get_mut(id.dense)
    }

    /// Advances every body by one fixed timestep, atomically committing the results.
    pub fn step(&mut self, dt: f64) -> Result<(), StepError> {
        self.step_with_executor(dt, &SequentialExecutor, 1)
    }

    /// Advances one step using the supplied statically dispatched task executor.
    pub fn step_with_executor<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if worker_count <= 1 {
            return self.step_with_executor_inner(dt, executor, worker_count);
        }
        match catch_unwind(AssertUnwindSafe(|| {
            self.step_with_executor_inner(dt, executor, worker_count)
        })) {
            Ok(result) => result,
            Err(_) => {
                self.collision_system.abort_step();
                self.rollback_gravity_updates();
                self.rollback_aerodynamics_updates();
                self.rollback_force_source_updates();
                Err(StepError::Collision(CollisionError::WorkerFailed))
            }
        }
    }

    fn step_with_executor_inner<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(StepError::InvalidDeltaTime);
        }
        self.with_force_source_updates(|world| {
            world.prepare_aerodynamics_updates(executor, worker_count, dt)?;
            world.prepare_gravity_updates(executor, worker_count)?;
            world.step_prepared(dt, executor, worker_count)
        })
    }

    fn with_force_source_updates<T>(
        &mut self,
        operation: impl FnOnce(&mut Self) -> Result<T, StepError>,
    ) -> Result<T, StepError> {
        self.validate_collision_inputs()
            .map_err(StepError::Collision)?;
        if let Err((source_id, cause)) = self.prepare_force_source_updates() {
            return Err(StepError::ForceSource { source_id, cause });
        }

        let result = operation(self);
        if result.is_err() {
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            self.rollback_force_source_updates();
        } else {
            self.aerodynamics_body_updates.clear();
            self.clear_gravity_updates();
            self.clear_force_source_updates();
        }
        result
    }

    /// Advances one full step through reusable prediction shards and the supplied executor.
    ///
    /// Unlike the low-level `fill_step_shards` and `commit_step_shards` pair, this method
    /// prepares persistent force sources and rolls prepared force and gravity back on failure.
    #[doc(hidden)]
    pub fn step_with_shards<E: StepExecutor>(
        &mut self,
        dt: f64,
        shards: &mut [Vec<BodyStep>],
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if self.body_count() == 0 {
            return self.step_with_executor_inner(dt, executor, worker_count);
        }
        let result = catch_unwind(AssertUnwindSafe(|| {
            self.step_with_shards_inner(dt, shards, executor, worker_count)
        }));
        match result {
            Ok(result) => result,
            Err(_) => {
                self.collision_system.abort_step();
                self.rollback_gravity_updates();
                self.rollback_aerodynamics_updates();
                self.rollback_force_source_updates();
                Err(StepError::Collision(CollisionError::WorkerFailed))
            }
        }
    }

    fn step_with_shards_inner<E: StepExecutor>(
        &mut self,
        dt: f64,
        shards: &mut [Vec<BodyStep>],
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(StepError::InvalidDeltaTime);
        }

        self.with_force_source_updates(|world| {
            if shards.is_empty() {
                return Err(StepError::InvalidStepResults);
            }
            world.fill_step_shards_with_executor_at_dt(shards, executor, Some(dt))?;
            executor
                .for_each_chunk_mut(shards, 1, |_, chunk| {
                    for shard in chunk {
                        for step in shard {
                            if step.integrate(dt).is_err() {
                                break;
                            }
                        }
                    }
                })
                .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
            world.commit_step_shards_with_executor(shards, executor, worker_count)
        })
    }

    fn step_prepared<E: StepExecutor>(
        &mut self,
        dt: f64,
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        self.collision_system.begin_step();
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_energy_before_integration(self.energy_components());
        if let Err(error) = self.prepare_joint_collision_pairs() {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }

        #[cfg(feature = "bench-internals")]
        let mut integrated_energy = crate::joint::diagnostics::EnergyComponents::default();
        for (dense_id, entry) in self.bodies.iter() {
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            match predict(&entry.body, dt) {
                Ok(state) => {
                    #[cfg(feature = "bench-internals")]
                    integrated_energy.add_assign(crate::collision::solver::body_energy_components(
                        &entry.body,
                        state,
                        self.central_gravity,
                    ));
                    self.collision_system.stage_prediction(body_id, state)
                }
                Err(cause) => {
                    self.collision_system.abort_step();
                    return Err(StepError::BodyIntegration { body_id, cause });
                }
            }
        }
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_energy_after_integration(integrated_energy);

        let worker_count = worker_count.max(1);
        if let Err(error) = self.prepare_collision_candidates(dt, worker_count, executor) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(worker_count, executor, dt) {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.prepare_joint_break_commit() {
            self.collision_system.abort_step();
            return Err(StepError::Collision(error));
        }

        #[cfg(feature = "bench-internals")]
        let commit_started = Instant::now();
        self.commit_predicted_states();
        self.commit_joint_breaks();
        self.collision_system.commit_step();
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_commit(commit_started.elapsed());
        Ok(())
    }

    fn prepare_joint_break_commit(&mut self) -> Result<(), CollisionError> {
        self.joint_events
            .try_reserve(self.collision_system.joint_break_events().len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)
    }

    fn commit_joint_breaks(&mut self) {
        let world_id = self.world_id;
        let break_events = self.collision_system.joint_break_events();
        let joints = &mut self.joints;
        let committed_events = &mut self.joint_events;
        committed_events.clear();
        for event in break_events.iter().copied() {
            let id = event.joint();
            if id.world == world_id && joints.remove(id.dense).is_some() {
                committed_events.push(event);
            }
        }
    }

    fn prepare_force_source_updates(&mut self) -> Result<(), (ForceSourceId, ForceSourceError)> {
        self.clear_force_source_updates();
        if self.force_sources.is_empty() {
            return Ok(());
        }

        for (dense, source) in self.force_sources.iter() {
            let source_id = ForceSourceId {
                world: self.world_id,
                dense,
            };
            let source = *source;
            if !source.enabled {
                continue;
            }
            let Some(body) = self.body(source.body).copied() else {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::InvalidBodyId));
            };
            if body.physics().body_type() != BodyType::Dynamic {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::BodyMustBeDynamic));
            }

            let index_active = self
                .force_source_body_indices
                .as_ref()
                .is_some_and(|indices| !indices.is_empty());
            let update_index = if index_active {
                self.force_source_body_indices
                    .as_ref()
                    .and_then(|indices| indices.get(&source.body).copied())
            } else {
                self.force_source_body_updates
                    .iter()
                    .position(|update| update.id == source.body)
            };
            let update_index = match update_index {
                Some(index) => index,
                None => {
                    if self.force_source_body_updates.try_reserve(1).is_err() {
                        self.force_source_body_updates.clear();
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.clear();
                        }
                        return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                    }
                    if index_active {
                        let map_reserve_failed = self
                            .force_source_body_indices
                            .as_mut()
                            .is_none_or(|indices| indices.try_reserve(1).is_err());
                        if map_reserve_failed {
                            self.force_source_body_updates.clear();
                            if let Some(indices) = &mut self.force_source_body_indices {
                                indices.clear();
                            }
                            return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                        }
                    }
                    let index = self.force_source_body_updates.len();
                    self.force_source_body_updates.push(ForceSourceBodyUpdate {
                        id: source.body,
                        original: body,
                        prepared: body,
                    });
                    if index_active {
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.insert(source.body, index);
                        }
                    } else if self.force_source_body_updates.len()
                        > FORCE_SOURCE_BODY_LINEAR_SEARCH_LIMIT
                    {
                        let indices = self
                            .force_source_body_indices
                            .get_or_insert_with(HashMap::new);
                        if indices
                            .try_reserve(self.force_source_body_updates.len())
                            .is_err()
                        {
                            self.force_source_body_updates.clear();
                            indices.clear();
                            return Err((source_id, ForceSourceError::StorageCapacityExceeded));
                        }
                        for (index, update) in self.force_source_body_updates.iter().enumerate() {
                            indices.insert(update.id, index);
                        }
                    }
                    index
                }
            };

            let transform = match source.target {
                ForceTarget::Body(target) if target == source.body => self
                    .force_source_body_updates[update_index]
                    .prepared
                    .state()
                    .transform(),
                ForceTarget::Part(part) => {
                    let Some(part_entry) = self.cluster_part(part) else {
                        self.force_source_body_updates.clear();
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.clear();
                        }
                        return Err((source_id, ForceSourceError::InvalidPartId));
                    };
                    if part_entry.body() != source.body {
                        self.force_source_body_updates.clear();
                        if let Some(indices) = &mut self.force_source_body_indices {
                            indices.clear();
                        }
                        return Err((source_id, ForceSourceError::InvalidPartId));
                    }
                    self.force_source_body_updates[update_index]
                        .prepared
                        .state()
                        .transform()
                        .compose(&part_entry.local_transform())
                        .map_err(|_| (source_id, ForceSourceError::ArithmeticOverflow))?
                }
                ForceTarget::Body(_) => {
                    self.force_source_body_updates.clear();
                    if let Some(indices) = &mut self.force_source_body_indices {
                        indices.clear();
                    }
                    return Err((source_id, ForceSourceError::InvalidBodyId));
                }
            };
            let body = &mut self.force_source_body_updates[update_index].prepared;
            let force = transform.transform_vector(source.local_force);
            let point = transform.transform_point(source.local_point);
            if let Err(error) = body.apply_force_at_point(force, point) {
                self.force_source_body_updates.clear();
                if let Some(indices) = &mut self.force_source_body_indices {
                    indices.clear();
                }
                return Err((source_id, ForceSourceError::from(error)));
            }
        }

        for update in &self.force_source_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body = update.prepared;
            }
        }
        Ok(())
    }

    fn clear_force_source_updates(&mut self) {
        self.force_source_body_updates.clear();
        if let Some(indices) = &mut self.force_source_body_indices {
            indices.clear();
        }
    }

    fn apply_aerodynamics(
        atmosphere: Atmosphere,
        surfaces: &[AerodynamicSurface],
        body_id: BodyId,
        body: &mut RigidBody,
        parts: &DensePool<ClusterPart>,
        dt: Option<f64>,
    ) -> Result<(), StepError> {
        if matches!(atmosphere, Atmosphere::Vacuum)
            || body.physics().body_type() != BodyType::Dynamic
        {
            return Ok(());
        }
        let error = || StepError::Aerodynamics {
            body_id,
            cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
        };
        let center = body
            .world_center_of_mass()
            .map_err(|_| error())?
            .ok_or_else(error)?;
        let mut drag = Vec3::ZERO;
        let mut drag_torque = Vec3::ZERO;
        let mut lift_force = Vec3::ZERO;
        let mut lift_torque = Vec3::ZERO;
        for surface in surfaces {
            let properties = surface.properties();
            let target_transform = match surface.target() {
                AerodynamicTarget::Body => body.state().transform(),
                AerodynamicTarget::Part(part) => parts
                    .get(part.dense)
                    .filter(|entry| part.world == body_id.world && entry.body() == body_id)
                    .and_then(|entry| {
                        body.state()
                            .transform()
                            .compose(&entry.local_transform())
                            .ok()
                    })
                    .ok_or_else(error)?,
            };
            let point = match surface.target() {
                AerodynamicTarget::Body => center.checked_add(
                    target_transform.transform_vector(properties.local_application_offset()),
                ),
                AerodynamicTarget::Part(_) => {
                    Some(target_transform.transform_point(properties.local_application_offset()))
                }
            }
            .filter(|point| point.is_finite())
            .ok_or_else(error)?;
            let local_atmosphere = atmosphere
                .state_at(point)
                .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
            if local_atmosphere.density() == 0.0 {
                continue;
            }
            let offset = point.checked_sub(center).ok_or_else(error)?;
            let point_velocity = body
                .state()
                .angular_velocity()
                .checked_cross(offset)
                .and_then(|velocity| velocity.checked_add(body.state().linear_velocity()))
                .ok_or_else(error)?;
            let point_properties = properties
                .with_local_application_offset(Vec3::ZERO)
                .map_err(|_| error())?;
            let (surface_drag, lift) = point_properties
                .force_components(
                    local_atmosphere,
                    target_transform,
                    point_velocity,
                    Vec3::ZERO,
                )
                .map_err(|cause| StepError::Aerodynamics { body_id, cause })?;
            drag = drag.checked_add(surface_drag).ok_or_else(error)?;
            drag_torque = offset
                .checked_cross(surface_drag)
                .and_then(|torque| drag_torque.checked_add(torque))
                .ok_or_else(error)?;
            lift_force = lift_force.checked_add(lift).ok_or_else(error)?;
            lift_torque = offset
                .checked_cross(lift)
                .and_then(|torque| lift_torque.checked_add(torque))
                .ok_or_else(error)?;
        }
        if drag == Vec3::ZERO
            && drag_torque == Vec3::ZERO
            && lift_force == Vec3::ZERO
            && lift_torque == Vec3::ZERO
        {
            return Ok(());
        }
        let mut scale = 1.0_f64;
        if let Some(dt) = dt {
            let mass = body.physics().mass().ok_or_else(error)?;
            let rotation = body.state().transform().rotation();
            let local_torque = rotation
                .conjugate()
                .rotate_vector(drag_torque)
                .ok_or_else(error)?;
            let local_angular_acceleration = mass
                .inertia()
                .solve_spd(local_torque)
                .map_err(|_| error())?;
            let angular_acceleration = rotation
                .rotate_vector(local_angular_acceleration)
                .ok_or_else(error)?;
            let relative_velocity = body
                .state()
                .linear_velocity()
                .checked_sub(atmosphere.wind_velocity())
                .ok_or_else(error)?;
            let power = relative_velocity
                .checked_dot(drag)
                .and_then(|linear| {
                    body.state()
                        .angular_velocity()
                        .checked_dot(drag_torque)
                        .map(|angular| linear + angular)
                })
                .ok_or_else(error)?;
            let curvature = drag
                .checked_dot(drag)
                .and_then(|linear| {
                    drag_torque
                        .checked_dot(angular_acceleration)
                        .map(|angular| linear / mass.mass() + angular)
                })
                .ok_or_else(error)?;
            if !power.is_finite() || !curvature.is_finite() {
                return Err(error());
            }
            if curvature > 0.0 {
                scale = (-power / dt / curvature).clamp(0.0, 1.0);
            }
        }
        body.apply_force(drag.checked_mul_scalar(scale).ok_or_else(error)?)
            .map_err(|_| error())?;
        body.apply_torque(drag_torque.checked_mul_scalar(scale).ok_or_else(error)?)
            .map_err(|_| error())?;
        if let Some(dt) = dt
            && (lift_force != Vec3::ZERO || lift_torque != Vec3::ZERO)
        {
            let mass = body.physics().mass().ok_or_else(error)?;
            let rotation = body.state().transform().rotation();
            let inverse = rotation.conjugate();
            let drag_angular_acceleration = rotation
                .rotate_vector(
                    mass.inertia()
                        .solve_spd(inverse.rotate_vector(drag_torque).ok_or_else(error)?)
                        .map_err(|_| error())?,
                )
                .ok_or_else(error)?;
            let angular_velocity = body
                .state()
                .angular_velocity()
                .checked_add(
                    drag_angular_acceleration
                        .checked_mul_scalar(scale * dt)
                        .ok_or_else(error)?,
                )
                .ok_or_else(error)?;
            let local_omega = inverse.rotate_vector(angular_velocity).ok_or_else(error)?;
            let angular_momentum = rotation
                .rotate_vector(
                    mass.inertia()
                        .checked_mul_vector(local_omega)
                        .ok_or_else(error)?,
                )
                .ok_or_else(error)?;
            let relative_velocity = body
                .state()
                .linear_velocity()
                .checked_sub(atmosphere.wind_velocity())
                .and_then(|velocity| {
                    drag.checked_mul_scalar(scale * dt / mass.mass())
                        .and_then(|change| velocity.checked_add(change))
                })
                .ok_or_else(error)?;
            let linear_momentum = relative_velocity
                .checked_mul_scalar(mass.mass())
                .ok_or_else(error)?;
            let metric = relative_velocity
                .checked_dot(linear_momentum)
                .and_then(|linear| {
                    angular_velocity
                        .checked_dot(angular_momentum)
                        .map(|angular| linear + angular)
                })
                .filter(|value| value.is_finite() && *value >= 0.0)
                .ok_or_else(error)?;
            if metric == 0.0 {
                return Ok(());
            }
            let power = relative_velocity
                .checked_dot(lift_force)
                .and_then(|linear| {
                    angular_velocity
                        .checked_dot(lift_torque)
                        .map(|angular| linear + angular)
                })
                .filter(|value| value.is_finite())
                .ok_or_else(error)?;
            let projection = power / metric;
            lift_force = lift_force
                .checked_sub(
                    linear_momentum
                        .checked_mul_scalar(projection)
                        .ok_or_else(error)?,
                )
                .ok_or_else(error)?;
            lift_torque = lift_torque
                .checked_sub(
                    angular_momentum
                        .checked_mul_scalar(projection)
                        .ok_or_else(error)?,
                )
                .ok_or_else(error)?;
            let local_lift_torque = inverse.rotate_vector(lift_torque).ok_or_else(error)?;
            let angular_acceleration = rotation
                .rotate_vector(
                    mass.inertia()
                        .solve_spd(local_lift_torque)
                        .map_err(|_| error())?,
                )
                .ok_or_else(error)?;
            let curvature = lift_force
                .checked_dot(lift_force)
                .and_then(|linear| {
                    lift_torque
                        .checked_dot(angular_acceleration)
                        .map(|angular| linear / mass.mass() + angular)
                })
                .filter(|value| value.is_finite() && *value >= 0.0)
                .ok_or_else(error)?;
            let rate = (curvature / metric).sqrt();
            let half_angle = 0.5 * dt * rate;
            if !half_angle.is_finite() {
                return Err(error());
            }
            let (factor, damping) = if half_angle <= 1.0 {
                let factor = 1.0 / (1.0 + half_angle * half_angle);
                (factor, rate * half_angle * factor)
            } else {
                let inverse_angle = 1.0 / half_angle;
                let denominator = 1.0 + inverse_angle * inverse_angle;
                (
                    inverse_angle * inverse_angle / denominator,
                    rate * inverse_angle / denominator,
                )
            };
            lift_force = lift_force
                .checked_mul_scalar(factor)
                .and_then(|force| {
                    linear_momentum
                        .checked_mul_scalar(damping)
                        .and_then(|correction| force.checked_sub(correction))
                })
                .ok_or_else(error)?;
            lift_torque = lift_torque
                .checked_mul_scalar(factor)
                .and_then(|torque| {
                    angular_momentum
                        .checked_mul_scalar(damping)
                        .and_then(|correction| torque.checked_sub(correction))
                })
                .ok_or_else(error)?;
        }
        body.apply_force(lift_force).map_err(|_| error())?;
        body.apply_torque(lift_torque).map_err(|_| error())?;
        Ok(())
    }

    fn prepare_aerodynamics_updates<E: StepExecutor>(
        &mut self,
        executor: &E,
        workers: usize,
        dt: f64,
    ) -> Result<(), StepError> {
        self.aerodynamics_body_updates.clear();
        let atmosphere = self.atmosphere;
        if matches!(atmosphere, Atmosphere::Vacuum) {
            return Ok(());
        };

        for (dense_id, entry) in self.bodies.iter() {
            if entry.aerodynamics.is_empty()
                || entry.body.physics().body_type() != BodyType::Dynamic
            {
                continue;
            }
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            if self.aerodynamics_body_updates.try_reserve(1).is_err() {
                self.aerodynamics_body_updates.clear();
                return Err(StepError::Aerodynamics {
                    body_id,
                    cause: crate::aerodynamics::AerodynamicsError::ArithmeticOverflow,
                });
            }
            self.aerodynamics_body_updates.push(AerodynamicsBodyUpdate {
                id: body_id,
                error: None,
                original: entry.body,
                prepared: entry.body,
            });
        }

        let bodies = &self.bodies;
        let chunk_size = self
            .aerodynamics_body_updates
            .len()
            .div_ceil(workers.max(1))
            .max(1);
        executor
            .for_each_chunk_mut(
                &mut self.aerodynamics_body_updates,
                chunk_size,
                |_, updates| {
                    for update in updates {
                        update.error = bodies
                            .get(update.id.dense)
                            .map(|entry| {
                                Self::apply_aerodynamics(
                                    atmosphere,
                                    &entry.aerodynamics,
                                    update.id,
                                    &mut update.prepared,
                                    &self.parts,
                                    Some(dt),
                                )
                            })
                            .unwrap_or(Ok(()))
                            .err();
                    }
                },
            )
            .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
        if let Some(error) = self
            .aerodynamics_body_updates
            .iter()
            .find_map(|update| update.error)
        {
            self.aerodynamics_body_updates.clear();
            return Err(error);
        }

        for update in &self.aerodynamics_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body = update.prepared;
            }
        }
        Ok(())
    }

    fn rollback_aerodynamics_updates(&mut self) {
        for update in &self.aerodynamics_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body = update.original;
            }
        }
        self.aerodynamics_body_updates.clear();
    }

    fn gravity_force_for_body(
        gravity: Option<CentralGravity>,
        fields: &DensePool<Gravity>,
        body_id: BodyId,
        body: &RigidBody,
    ) -> Result<Vec3, StepError> {
        if gravity.is_none() && fields.is_empty() {
            return Ok(Vec3::ZERO);
        }
        if body.physics().body_type() != BodyType::Dynamic {
            return Ok(Vec3::ZERO);
        }
        let center = body
            .world_center_of_mass()
            .map_err(|_| StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })?
            .ok_or(StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })?;
        let mut acceleration = match gravity {
            Some(gravity) => gravity
                .acceleration_at(center)
                .map_err(|cause| StepError::Gravity { body_id, cause })?,
            None => Vec3::ZERO,
        };
        for (_, field) in fields.iter() {
            let next = field
                .acceleration_at(center)
                .map_err(|cause| StepError::Gravity { body_id, cause })?;
            acceleration = acceleration.checked_add(next).ok_or(StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })?;
        }
        let mass = body.physics().mass().ok_or(StepError::Gravity {
            body_id,
            cause: GravityError::ArithmeticOverflow,
        })?;
        acceleration
            .checked_mul_scalar(mass.mass())
            .ok_or(StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })
    }

    fn apply_gravity(
        gravity: Option<CentralGravity>,
        fields: &DensePool<Gravity>,
        body_id: BodyId,
        body: &mut RigidBody,
    ) -> Result<(), StepError> {
        let force = Self::gravity_force_for_body(gravity, fields, body_id, body)?;
        if force != Vec3::ZERO {
            body.apply_force(force).map_err(|_| StepError::Gravity {
                body_id,
                cause: GravityError::ArithmeticOverflow,
            })?;
        }
        Ok(())
    }

    fn prepare_gravity_updates<E: StepExecutor>(
        &mut self,
        executor: &E,
        workers: usize,
    ) -> Result<(), StepError> {
        self.gravity_body_updates.clear();
        if self.central_gravity.is_none() && self.gravity_fields.is_empty() {
            return Ok(());
        }

        for (dense_id, entry) in self.bodies.iter() {
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            if entry.body.physics().body_type() != BodyType::Dynamic {
                continue;
            }
            if self.gravity_body_updates.try_reserve(1).is_err() {
                self.gravity_body_updates.clear();
                return Err(StepError::Gravity {
                    body_id,
                    cause: GravityError::StorageCapacityExceeded,
                });
            }
            self.gravity_body_updates.push(GravityBodyUpdate {
                id: body_id,
                error: None,
                original: entry.body,
                prepared: entry.body,
            });
        }

        let gravity = self.central_gravity;
        let fields = &self.gravity_fields;
        let chunk_size = self
            .gravity_body_updates
            .len()
            .div_ceil(workers.max(1))
            .max(1);
        executor
            .for_each_chunk_mut(&mut self.gravity_body_updates, chunk_size, |_, updates| {
                for update in updates {
                    update.error =
                        Self::apply_gravity(gravity, fields, update.id, &mut update.prepared).err();
                }
            })
            .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
        if let Some(error) = self
            .gravity_body_updates
            .iter()
            .find_map(|update| update.error)
        {
            self.gravity_body_updates.clear();
            return Err(error);
        }

        for update in &self.gravity_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body = update.prepared;
            }
        }
        Ok(())
    }

    fn rollback_gravity_updates(&mut self) {
        for update in &self.gravity_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body = update.original;
            }
        }
        self.clear_gravity_updates();
    }

    fn clear_gravity_updates(&mut self) {
        self.gravity_body_updates.clear();
    }

    fn rollback_force_source_updates(&mut self) {
        for update in &self.force_source_body_updates {
            if let Some(entry) = self
                .bodies
                .get_mut(update.id.dense)
                .filter(|_| update.id.world == self.world_id)
            {
                entry.body = update.original;
            }
        }
        self.clear_force_source_updates();
    }

    fn stage_spatial_proxies(&mut self, dt: f64) -> Result<(), CollisionError> {
        for ((body_dense, body_entry), (predicted_id, predicted_state)) in self.bodies.iter().zip(
            self.collision_system
                .context
                .predicted_states
                .iter()
                .copied(),
        ) {
            let body_id = BodyId {
                world: self.world_id,
                dense: body_dense,
            };
            if predicted_id != body_id {
                return Err(CollisionError::InvalidContact);
            }
            let body_type = body_entry.body.physics().body_type();
            let mut next = body_entry.first_collider;
            while let Some(collider_id) = next {
                if collider_id.world != self.world_id {
                    return Err(CollisionError::InvalidContact);
                }
                let collider_entry = self
                    .colliders
                    .get(collider_id.dense)
                    .ok_or(CollisionError::InvalidContact)?;
                if collider_entry.body != body_id {
                    return Err(CollisionError::InvalidContact);
                }
                let bounds = if body_type != BodyType::Static {
                    let trajectory = CcdTrajectory::new(
                        body_entry.body,
                        *body_entry.body.state(),
                        0.0,
                        0.0,
                        dt,
                        collider_entry.collider.local_transform(),
                        false,
                    );
                    swept_bounds_for_trajectory(
                        &collider_entry.collider,
                        &trajectory,
                        predicted_state,
                    )?
                } else {
                    collider_entry
                        .collider
                        .world_bounds(&predicted_state.transform())
                        .map_err(|_| CollisionError::GeometryBounds)?
                };
                self.collision_system
                    .context
                    .colliders
                    .try_reserve(1)
                    .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                self.collision_system
                    .context
                    .colliders
                    .push(crate::collision::CollisionProxy {
                        collider: collider_id,
                        body: body_id,
                        bounds,
                        body_type,
                        mode: collider_entry.collider.mode(),
                        filter: collider_entry.collider.filter(),
                        convex: collider_entry.collider.shape().is_convex(),
                    });
                if let Some(bounds) = bounds {
                    self.collision_system
                        .context
                        .spatial_proxies
                        .try_reserve(1)
                        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
                    self.collision_system
                        .context
                        .spatial_proxies
                        .push((collider_id, bounds));
                }
                next = collider_entry.next;
            }
        }
        Ok(())
    }

    fn commit_predicted_states(&mut self) {
        {
            let world_id = self.world_id;
            let colliders = &self.colliders;
            let query_index = &mut self.query_index;
            let query_index_dirty = &mut self.query_index_dirty;
            for ((dense_id, entry), (body_id, state)) in self.bodies.iter_mut().zip(
                self.collision_system
                    .context
                    .predicted_states
                    .iter()
                    .copied(),
            ) {
                let current_id = BodyId {
                    world: world_id,
                    dense: dense_id,
                };
                debug_assert_eq!(current_id, body_id);
                let transform_changed = entry.body.state().transform() != state.transform();
                entry.body.commit_integrated_state(state);
                if transform_changed
                    && !*query_index_dirty
                    && let Some(index) = query_index.as_mut()
                    && update_query_index_for_body(index, colliders, world_id, body_id, entry)
                        .is_err()
                {
                    *query_index_dirty = true;
                    index.mark_dirty();
                }
            }
        }
        if self.query_index_dirty {
            self.rebuild_query_index();
        }
    }

    fn prepare_collision_candidates(
        &mut self,
        dt: f64,
        worker_count: usize,
        executor: &impl StepExecutor,
    ) -> Result<(), CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.stage_spatial_proxies(dt)?;
        self.collision_system.synchronize_spatial_index()?;
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_spatial_index(started.elapsed());

        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.collision_system.generate_candidates()?;
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_broad_phase(started.elapsed());

        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        self.process_continuous_collisions(dt, worker_count, executor)?;
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_ccd(started.elapsed());
        Ok(())
    }

    fn prepare_joint_collision_pairs(&mut self) -> Result<(), CollisionError> {
        let pairs = &mut self.collision_system.context.joint_collision_pairs;
        pairs.clear();
        pairs
            .try_reserve(self.joints.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for (_, entry) in self.joints.iter() {
            if entry.body_a == entry.body_b
                || !entry.joint.common().enabled
                || entry.joint.common().collide_connected
            {
                continue;
            }
            let pair = if entry.body_a < entry.body_b {
                (entry.body_a, entry.body_b)
            } else {
                (entry.body_b, entry.body_a)
            };
            pairs.push(pair);
        }
        pairs.sort_unstable();
        pairs.dedup();
        Ok(())
    }

    fn prepare_joint_constraints(&mut self) -> Result<(), CollisionError> {
        self.collision_system.context.joint_constraints.clear();
        self.collision_system
            .context
            .joint_constraints
            .try_reserve(self.joints.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        let world_id = self.world_id;
        let bodies = &self.bodies;
        let joints = &self.joints;
        let context = &mut self.collision_system.context;
        for (dense, entry) in joints.iter() {
            let (body_a_id, body_b_id, joint) = (entry.body_a, entry.body_b, entry.joint);
            if !joint.common().enabled {
                continue;
            }
            if body_a_id == body_b_id {
                continue;
            }
            let body_type_a = bodies
                .get(body_a_id.dense)
                .filter(|_| body_a_id.world == world_id)
                .map(|body| body.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            let body_type_b = bodies
                .get(body_b_id.dense)
                .filter(|_| body_b_id.world == world_id)
                .map(|body| body.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            if body_type_a != BodyType::Dynamic && body_type_b != BodyType::Dynamic {
                continue;
            }
            let prediction_a =
                Self::predicted_body_index(&context.body_indices, world_id, body_a_id)?;
            let prediction_b =
                Self::predicted_body_index(&context.body_indices, world_id, body_b_id)?;
            let body_a = Self::solver_body_index_in_context(
                bodies,
                world_id,
                context,
                body_a_id,
                prediction_a,
            )?;
            let body_b = Self::solver_body_index_in_context(
                bodies,
                world_id,
                context,
                body_b_id,
                prediction_b,
            )?;
            context
                .joint_constraints
                .push(crate::joint::JointConstraint {
                    id: crate::world::JointId {
                        world: world_id,
                        dense,
                    },
                    body_a,
                    body_b,
                    joint,
                });
        }
        Ok(())
    }

    fn stage_collision_events(&mut self) -> Result<(), CollisionError> {
        #[cfg(feature = "bench-internals")]
        let started = Instant::now();
        let result = self.collision_system.stage_lifecycle_events();
        #[cfg(feature = "bench-internals")]
        self.collision_system.record_events(started.elapsed());
        result
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_mass_cache_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_mass_cache_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_warm_start_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_warm_start_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_partial_coloring_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_partial_coloring_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_reduced_fixed_tree_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_reduced_fixed_tree_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_projection_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_projection_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_light_diagnostics_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_light_diagnostics_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_adaptive_iterations_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_adaptive_iterations_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_alternating_velocity_sweeps_for_bench(&mut self, enabled: bool) {
        self.collision_system
            .set_joint_alternating_velocity_sweeps_for_bench(enabled);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_adaptive_velocity_max_for_bench(&mut self, max: u32) {
        self.collision_system
            .set_joint_adaptive_velocity_max_for_bench(max);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_global_velocity_pcg_for_bench(
        &mut self,
        max_iterations: u32,
        incomplete_cholesky_preconditioner: bool,
    ) {
        self.collision_system
            .set_joint_global_velocity_pcg_for_bench(
                max_iterations,
                incomplete_cholesky_preconditioner,
            );
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_global_velocity_block_pcg_for_bench(&mut self, max_iterations: u32) {
        self.collision_system
            .set_joint_global_velocity_block_pcg_for_bench(max_iterations);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_velocity_relaxation_for_bench(&mut self, relaxation: f64) {
        self.collision_system
            .set_joint_velocity_relaxation_for_bench(relaxation);
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn set_joint_global_position_pcg_for_bench(
        &mut self,
        max_iterations: u32,
        incomplete_cholesky_preconditioner: bool,
    ) {
        self.collision_system
            .set_joint_global_position_pcg_for_bench(
                max_iterations,
                incomplete_cholesky_preconditioner,
            );
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn collision_step_metrics(&self) -> crate::collision::CollisionStepMetrics {
        let mut metrics = self.collision_system.benchmark_metrics();
        let query_index_bytes = self
            .query_index
            .as_ref()
            .map_or(0, SpatialIndex::retained_capacity_bytes)
            .saturating_add(
                self.query_proxies.capacity() * std::mem::size_of::<(ColliderId, Aabb)>(),
            )
            .saturating_add(
                self.unbounded_query_colliders.capacity() * std::mem::size_of::<ColliderId>(),
            );
        metrics.committed_query_index_bytes = query_index_bytes;
        metrics
    }

    #[cfg(feature = "bench-internals")]
    #[doc(hidden)]
    pub fn physics_step_metrics(&self) -> PhysicsStepMetrics {
        if !self.collision_system.step_metrics_committed() {
            return PhysicsStepMetrics::default();
        }
        PhysicsStepMetrics {
            collision: self.collision_step_metrics(),
            joints: self.collision_system.joint_solver_metrics(),
        }
    }

    #[cfg(feature = "bench-internals")]
    fn energy_components(&self) -> crate::joint::diagnostics::EnergyComponents {
        let mut energy = crate::joint::diagnostics::EnergyComponents::default();
        for (_, entry) in self.bodies.iter() {
            energy.add_assign(crate::collision::solver::body_energy_components(
                &entry.body,
                *entry.body.state(),
                self.central_gravity,
            ));
        }
        energy
    }

    fn required_body_index_slots(context: &StepContext) -> Result<usize, CollisionError> {
        context
            .predicted_states
            .iter()
            .try_fold(0_usize, |required, (body_id, _)| {
                let slot =
                    usize::try_from(body_id.index()).map_err(|_| CollisionError::GeometryBounds)?;
                let end = slot.checked_add(1).ok_or(CollisionError::GeometryBounds)?;
                Ok(required.max(end))
            })
    }

    fn rebuild_body_indices(context: &mut StepContext) -> Result<(), CollisionError> {
        if context.body_indices_ready {
            return Ok(());
        }
        let required_slots = Self::required_body_index_slots(context)?;
        if context.body_indices.len() < required_slots {
            context
                .body_indices
                .try_reserve(required_slots - context.body_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context.body_indices.resize(required_slots, None);
        }
        context
            .body_indices
            .iter_mut()
            .for_each(|slot| *slot = None);
        for (index, (body_id, _)) in context.predicted_states.iter().copied().enumerate() {
            let slot =
                usize::try_from(body_id.index()).map_err(|_| CollisionError::GeometryBounds)?;
            *context
                .body_indices
                .get_mut(slot)
                .ok_or(CollisionError::InvalidContact)? = Some((body_id, index));
        }
        context.body_indices_ready = true;
        Ok(())
    }

    fn prepare_solver_body_indices(context: &mut StepContext) -> Result<(), CollisionError> {
        let required_slots = Self::required_body_index_slots(context)?;
        if context.solver_body_indices.len() < required_slots {
            context
                .solver_body_indices
                .try_reserve(required_slots - context.solver_body_indices.len())
                .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
            context.solver_body_indices.resize(required_slots, None);
        }
        context
            .solver_body_indices
            .iter_mut()
            .for_each(|slot| *slot = None);
        context.body_types.clear();
        context.solver_bodies.clear();
        context.constrained_starts.clear();
        context.solver_body_predictions.clear();
        Ok(())
    }

    fn process_collision_candidates(
        &mut self,
        worker_count: usize,
        executor: &impl StepExecutor,
        dt: f64,
    ) -> Result<(), CollisionError> {
        let candidate_count = self.collision_system.candidate_count();
        let has_active_joints = self
            .joints
            .iter()
            .any(|(_, entry)| entry.joint.common().enabled);
        if candidate_count == 0 && !has_active_joints {
            self.collision_system.solve_contacts(
                worker_count,
                executor,
                dt,
                self.central_gravity,
            )?;
            self.stage_collision_events()?;
            return Ok(());
        }
        let context = &mut self.collision_system.context;
        Self::rebuild_body_indices(context)?;
        Self::prepare_solver_body_indices(context)?;
        if has_active_joints {
            #[cfg(feature = "bench-internals")]
            let joint_prepare_started = Instant::now();
            self.prepare_joint_constraints()?;
            #[cfg(feature = "bench-internals")]
            self.collision_system
                .record_joint_constraint_prepare(joint_prepare_started.elapsed());
        }

        if candidate_count == 0 {
            self.collision_system.solve_contacts(
                worker_count,
                executor,
                dt,
                self.central_gravity,
            )?;
            self.stage_collision_events()?;
            return Ok(());
        }

        self.collision_system.context.manifolds.clear();
        self.collision_system
            .context
            .manifolds
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        self.collision_system
            .context
            .interactions
            .try_reserve(candidate_count)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for candidate_index in 0..candidate_count {
            let pair = self
                .collision_system
                .candidate(candidate_index)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_a = self
                .colliders
                .get(pair.a.dense)
                .filter(|_| pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(pair.b.dense)
                .filter(|_| pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            if self
                .collision_system
                .context
                .suppresses_joint_collision(collider_a.body, collider_b.body)
            {
                continue;
            }
            let body_a_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_a.body,
            )?;
            let body_b_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_b.body,
            )?;
            let body_a = self
                .collision_system
                .context
                .predicted_states
                .get(body_a_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let body_b = self
                .collision_system
                .context
                .predicted_states
                .get(body_b_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let transform_a = body_a
                .compose(&collider_a.collider.local_transform())
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            let transform_b = body_b
                .compose(&collider_b.collider.local_transform())
                .map_err(|_| CollisionError::ArithmeticOverflow)?;
            #[cfg(feature = "bench-internals")]
            let started = Instant::now();
            let manifold = self.collision_system.detect(
                pair,
                collider_a.collider.shape(),
                &transform_a,
                &body_a,
                collider_a.collider.mode(),
                collider_a.collider.query_bvh(),
                collider_a.collider.query_vertices(),
                collider_b.collider.shape(),
                &transform_b,
                &body_b,
                collider_b.collider.mode(),
                collider_b.collider.query_bvh(),
                collider_b.collider.query_vertices(),
            )?;
            #[cfg(feature = "bench-internals")]
            self.collision_system.record_narrow_phase(
                started.elapsed(),
                manifold.as_ref().map_or(0, ContactManifold::len),
            );
            if let Some(manifold) = manifold {
                let sensor = collider_a.collider.mode() == crate::collider::CollisionMode::Sensor
                    || collider_b.collider.mode() == crate::collider::CollisionMode::Sensor;
                if sensor {
                    upsert_interaction(
                        &mut self.collision_system.context.interactions,
                        &mut self.collision_system.context.interaction_indices,
                        CollisionInteraction {
                            pair,
                            sensor: true,
                            transient: false,
                            contact: None,
                            impact: false,
                        },
                    )?;
                } else {
                    self.collision_system.context.manifolds.push(manifold);
                }
            }
        }

        self.collision_system.context.solver_manifolds.clear();
        self.collision_system
            .context
            .solver_manifolds
            .try_reserve(self.collision_system.context.manifolds.len())
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        for manifold_index in 0..self.collision_system.context.manifolds.len() {
            let manifold = self.collision_system.context.manifolds[manifold_index];
            let (body_id_a, body_id_b, mode_a, mode_b, material_a, material_b) = {
                let collider_a = self
                    .colliders
                    .get(manifold.pair.a.dense)
                    .filter(|_| manifold.pair.a.world == self.world_id)
                    .ok_or(CollisionError::InvalidContact)?;
                let collider_b = self
                    .colliders
                    .get(manifold.pair.b.dense)
                    .filter(|_| manifold.pair.b.world == self.world_id)
                    .ok_or(CollisionError::InvalidContact)?;
                (
                    collider_a.body,
                    collider_b.body,
                    collider_a.collider.mode(),
                    collider_b.collider.mode(),
                    collider_a.collider.material(),
                    collider_b.collider.material(),
                )
            };
            let prediction_a = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                body_id_a,
            )?;
            let prediction_b = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                body_id_b,
            )?;
            let type_a = self
                .bodies
                .get(body_id_a.dense)
                .filter(|_| body_id_a.world == self.world_id)
                .map(|entry| entry.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            let type_b = self
                .bodies
                .get(body_id_b.dense)
                .filter(|_| body_id_b.world == self.world_id)
                .map(|entry| entry.body.physics().body_type())
                .ok_or(CollisionError::InvalidContact)?;
            if type_a != BodyType::Dynamic && type_b != BodyType::Dynamic {
                continue;
            }
            if mode_a == crate::collider::CollisionMode::Sensor
                || mode_b == crate::collider::CollisionMode::Sensor
            {
                return Err(CollisionError::InvalidContact);
            }
            let material = self.combined_solver_material(material_a, material_b)?;
            let body_a = self.solver_body_index(body_id_a, prediction_a)?;
            let body_b = self.solver_body_index(body_id_b, prediction_b)?;
            self.collision_system
                .context
                .solver_manifolds
                .push(SolverManifold {
                    body_a,
                    body_b,
                    manifold,
                    material,
                });
        }
        self.collision_system
            .solve_contacts(worker_count, executor, dt, self.central_gravity)?;

        for manifold in self.collision_system.context.manifolds.iter().copied() {
            let collider_a = self
                .colliders
                .get(manifold.pair.a.dense)
                .filter(|_| manifold.pair.a.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let collider_b = self
                .colliders
                .get(manifold.pair.b.dense)
                .filter(|_| manifold.pair.b.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            let body_a_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_a.body,
            )?;
            let body_b_index = Self::predicted_body_index(
                &self.collision_system.context.body_indices,
                self.world_id,
                collider_b.body,
            )?;
            let body_a = self
                .collision_system
                .context
                .predicted_states
                .get(body_a_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let body_b = self
                .collision_system
                .context
                .predicted_states
                .get(body_b_index)
                .ok_or(CollisionError::InvalidContact)?
                .1
                .transform();
            let mut deepest = None;
            let mut total_impulse = 0.0;
            let mut impact = false;
            for point in manifold.points().iter().copied() {
                total_impulse += point.normal_impulse;
                if !total_impulse.is_finite() {
                    return Err(CollisionError::ArithmeticOverflow);
                }
                impact |= point.normal_impulse_increase_sum > 0.0;
                if deepest.is_none_or(|current: crate::collision::ContactPoint| {
                    point.penetration > current.penetration
                }) {
                    deepest = Some(point);
                }
            }
            // Solver updates can add and then completely cancel an impulse.
            // Such a manifold has no resultant impact to publish.
            impact &= total_impulse > 0.0;
            let deepest = deepest.ok_or(CollisionError::InvalidContact)?;
            let point_a = body_a.transform_point(deepest.local_a);
            let point_b = body_b.transform_point(deepest.local_b);
            let point = point_a * 0.5 + point_b * 0.5;
            if !point.is_finite() {
                return Err(CollisionError::ArithmeticOverflow);
            }
            upsert_interaction(
                &mut self.collision_system.context.interactions,
                &mut self.collision_system.context.interaction_indices,
                CollisionInteraction {
                    pair: manifold.pair,
                    sensor: false,
                    transient: false,
                    contact: Some(ContactInfo {
                        point,
                        normal: deepest.normal,
                        penetration: deepest.penetration,
                        normal_impulse: total_impulse,
                    }),
                    impact,
                },
            )?;
        }
        self.stage_collision_events()?;
        Ok(())
    }

    fn predicted_body_index(
        body_indices: &[Option<(BodyId, usize)>],
        world_id: u64,
        id: BodyId,
    ) -> Result<usize, CollisionError> {
        if id.world != world_id {
            return Err(CollisionError::InvalidContact);
        }
        let slot = usize::try_from(id.index()).map_err(|_| CollisionError::InvalidContact)?;
        body_indices
            .get(slot)
            .and_then(|entry| *entry)
            .filter(|(current, _)| *current == id)
            .map(|(_, index)| index)
            .ok_or(CollisionError::InvalidContact)
    }

    fn solver_body_index(
        &mut self,
        body_id: BodyId,
        prediction_index: usize,
    ) -> Result<usize, CollisionError> {
        Self::solver_body_index_in_context(
            &self.bodies,
            self.world_id,
            &mut self.collision_system.context,
            body_id,
            prediction_index,
        )
    }

    fn solver_body_index_in_context(
        bodies: &DensePool<BodyEntry>,
        world_id: u64,
        context: &mut StepContext,
        body_id: BodyId,
        prediction_index: usize,
    ) -> Result<usize, CollisionError> {
        let slot = usize::try_from(body_id.index()).map_err(|_| CollisionError::InvalidContact)?;
        if let Some((current, index)) = context.solver_body_indices.get(slot).copied().flatten()
            && current == body_id
        {
            return Ok(index);
        }
        let entry = bodies
            .get(body_id.dense)
            .filter(|_| body_id.world == world_id)
            .ok_or(CollisionError::InvalidContact)?;
        let body = entry.body;
        let constrained_start = (entry.first_collider.is_none()
            && entry.cluster.is_none()
            && body.ccd_mode() == CcdMode::Discrete)
            .then_some(SolverBody { body });
        let state = context
            .predicted_states
            .get(prediction_index)
            .filter(|(current, _)| *current == body_id)
            .map(|(_, state)| *state)
            .ok_or(CollisionError::InvalidContact)?;
        let index = context.solver_bodies.len();
        context
            .solver_bodies
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context
            .solver_body_predictions
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context
            .body_types
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        *context
            .solver_body_indices
            .get_mut(slot)
            .ok_or(CollisionError::InvalidContact)? = Some((body_id, index));
        context
            .solver_bodies
            .push(SolverBody::from_prediction(body, state));
        context
            .constrained_starts
            .try_reserve(1)
            .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
        context.constrained_starts.push(constrained_start);
        context.solver_body_predictions.push(prediction_index);
        context.body_types.push(body.physics().body_type());
        Ok(index)
    }

    fn combined_solver_material(
        &self,
        material_a: Option<MaterialId>,
        material_b: Option<MaterialId>,
    ) -> Result<SolverMaterial, CollisionError> {
        let fallback = PhysicsMaterial::default();
        let material_a = material_a
            .and_then(|id| self.materials.get(id))
            .copied()
            .unwrap_or(fallback);
        let material_b = material_b
            .and_then(|id| self.materials.get(id))
            .copied()
            .unwrap_or(fallback);
        let dynamic_friction = material_a.friction().sqrt() * material_b.friction().sqrt();
        let static_friction =
            material_a.static_friction().sqrt() * material_b.static_friction().sqrt();
        if !dynamic_friction.is_finite() || !static_friction.is_finite() {
            return Err(CollisionError::ArithmeticOverflow);
        }
        Ok(SolverMaterial {
            restitution: material_a.restitution().max(material_b.restitution()),
            static_friction,
            dynamic_friction,
        })
    }

    fn validate_collision_inputs(&self) -> Result<(), CollisionError> {
        self.collision_system.config().validate()?;
        for (_, entry) in self.colliders.iter() {
            if entry
                .collider
                .material()
                .is_some_and(|id| !self.materials.contains(id))
            {
                return Err(CollisionError::InvalidMaterial);
            }
            let body = self
                .bodies
                .get(entry.body.dense)
                .filter(|_| entry.body.world == self.world_id)
                .ok_or(CollisionError::InvalidContact)?;
            if body.body.physics().body_type() == crate::rigid_body::BodyType::Dynamic
                && !entry.collider.shape().is_convex()
            {
                return Err(CollisionError::UnsupportedDynamicGeometry);
            }
        }
        Ok(())
    }

    /// Fills balanced, ordered body snapshots into reusable shard buffers.
    ///
    /// This low-level helper does not prepare persistent force sources. Use
    /// [`Self::step_with_shards`] to execute a complete step.
    #[doc(hidden)]
    pub fn fill_step_shards(&self, shards: &mut [Vec<BodyStep>]) -> Result<(), StepError> {
        self.fill_step_shards_with_executor(shards, &SequentialExecutor)
    }

    fn fill_step_shards_with_executor<E: StepExecutor>(
        &self,
        shards: &mut [Vec<BodyStep>],
        executor: &E,
    ) -> Result<(), StepError> {
        self.fill_step_shards_with_executor_at_dt(shards, executor, None)
    }

    fn fill_step_shards_with_executor_at_dt<E: StepExecutor>(
        &self,
        shards: &mut [Vec<BodyStep>],
        executor: &E,
        dt: Option<f64>,
    ) -> Result<(), StepError> {
        if shards.is_empty() {
            return Err(StepError::InvalidStepResults);
        }
        for shard in shards.iter_mut() {
            shard.clear();
        }

        let base_size = self.body_count() / shards.len();
        let remainder = self.body_count() % shards.len();
        for (index, shard) in shards.iter_mut().enumerate() {
            let required = base_size + usize::from(index < remainder);
            shard
                .try_reserve(required)
                .map_err(|_| StepError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        }
        let mut shard_index = 0;
        let mut shard_remaining = base_size + usize::from(remainder > 0);

        for (dense_id, entry) in self.bodies.iter() {
            while shard_remaining == 0 {
                shard_index += 1;
                shard_remaining = base_size + usize::from(shard_index < remainder);
            }
            let body_id = BodyId {
                world: self.world_id,
                dense: dense_id,
            };
            shards[shard_index].push(BodyStep {
                body_id,
                source: entry.body,
                predicted: None,
                dt: None,
                error: None,
                prepared: entry.body,
            });
            shard_remaining -= 1;
        }
        if !matches!(self.atmosphere, Atmosphere::Vacuum)
            || self.central_gravity.is_some()
            || !self.gravity_fields.is_empty()
        {
            let atmosphere = self.atmosphere;
            executor
                .for_each_chunk_mut(shards, 1, |_, chunks| {
                    for step in chunks.iter_mut().flatten() {
                        if !matches!(atmosphere, Atmosphere::Vacuum)
                            && let Some(properties) = self
                                .bodies
                                .get(step.body_id.dense)
                                .map(|entry| entry.aerodynamics.as_slice())
                                .filter(|surfaces| !surfaces.is_empty())
                        {
                            step.error = Self::apply_aerodynamics(
                                atmosphere,
                                properties,
                                step.body_id,
                                &mut step.prepared,
                                &self.parts,
                                dt,
                            )
                            .err();
                        }
                        if step.error.is_none() {
                            step.error = Self::apply_gravity(
                                self.central_gravity,
                                &self.gravity_fields,
                                step.body_id,
                                &mut step.prepared,
                            )
                            .err();
                        }
                    }
                })
                .map_err(|_| StepError::Collision(CollisionError::WorkerFailed))?;
            if let Some(error) = shards.iter().flatten().find_map(|step| step.error) {
                return Err(error);
            }
        }
        Ok(())
    }

    /// Validates all shard results before committing any body state.
    #[doc(hidden)]
    pub fn commit_step_shards(&mut self, shards: &[Vec<BodyStep>]) -> Result<(), StepError> {
        self.commit_step_shards_with_workers(shards, shards.len().max(1))
    }

    /// Validates shard results using the built-in sequential executor.
    ///
    /// Use [`Self::commit_step_shards_with_executor`] to select another executor.
    #[doc(hidden)]
    pub fn commit_step_shards_with_workers(
        &mut self,
        shards: &[Vec<BodyStep>],
        _worker_count: usize,
    ) -> Result<(), StepError> {
        self.commit_step_shards_with_executor(shards, &SequentialExecutor, 1)
    }

    /// Validates shard results and delegates independent collision work to an executor.
    ///
    /// This low-level commit helper does not prepare persistent force sources. Use
    /// [`Self::step_with_shards`] to execute a complete step.
    #[doc(hidden)]
    pub fn commit_step_shards_with_executor<E: StepExecutor>(
        &mut self,
        shards: &[Vec<BodyStep>],
        executor: &E,
        worker_count: usize,
    ) -> Result<(), StepError> {
        let mut steps = shards.iter().flatten();
        let mut step_dt = None;
        for (dense_id, entry) in self.bodies.iter() {
            let Some(step) = steps.next() else {
                return Err(StepError::InvalidStepResults);
            };
            if let Some(error) = step.error {
                return Err(error);
            }
            if step.body_id
                != (BodyId {
                    world: self.world_id,
                    dense: dense_id,
                })
                || step.source != entry.body
                || step.predicted.is_none()
                || step.dt.is_none_or(|dt| !dt.is_finite() || dt <= 0.0)
            {
                return Err(StepError::InvalidStepResults);
            }
            if step_dt.is_some_and(|dt| Some(dt) != step.dt) {
                return Err(StepError::InvalidStepResults);
            }
            step_dt = step.dt;
        }
        let dt = step_dt.unwrap_or(0.0);
        if steps.next().is_some() {
            return Err(StepError::InvalidStepResults);
        }

        self.validate_collision_inputs()
            .map_err(StepError::Collision)?;
        self.prepare_gravity_updates(executor, worker_count)?;
        self.collision_system.begin_step();
        if let Err(error) = self.prepare_joint_collision_pairs() {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }
        for step in shards.iter().flatten() {
            if let Some(state) = step.predicted {
                self.collision_system.stage_prediction(step.body_id, state);
            }
        }
        let worker_count = worker_count.max(1);
        if let Err(error) = self.prepare_collision_candidates(dt, worker_count, executor) {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.process_collision_candidates(worker_count, executor, dt) {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }
        if let Err(error) = self.prepare_joint_break_commit() {
            self.collision_system.abort_step();
            self.rollback_gravity_updates();
            self.rollback_aerodynamics_updates();
            return Err(StepError::Collision(error));
        }

        #[cfg(feature = "bench-internals")]
        let commit_started = Instant::now();
        self.commit_predicted_states();
        self.commit_joint_breaks();
        self.collision_system.commit_step();
        self.clear_gravity_updates();
        #[cfg(feature = "bench-internals")]
        self.collision_system
            .record_commit(commit_started.elapsed());
        Ok(())
    }

    pub fn add_collider(
        &mut self,
        body: BodyId,
        collider: Collider,
    ) -> Result<ColliderId, WorldError> {
        if self
            .bodies
            .get(body.dense)
            .is_some_and(|entry| entry.cluster.is_some())
        {
            return Err(WorldError::InvalidBodyId);
        }
        self.add_collider_internal(body, collider)
    }

    fn add_collider_internal(
        &mut self,
        body: BodyId,
        mut collider: Collider,
    ) -> Result<ColliderId, WorldError> {
        if body.world != self.world_id {
            return Err(WorldError::InvalidBodyId);
        }
        if collider
            .material()
            .is_some_and(|material| !self.materials.contains(material))
        {
            return Err(WorldError::InvalidMaterialId);
        }
        let first = self
            .bodies
            .get(body.dense)
            .ok_or(WorldError::InvalidBodyId)?
            .first_collider;
        if self.bodies.get(body.dense).is_some_and(|entry| {
            entry.body.physics().body_type() == crate::rigid_body::BodyType::Dynamic
                && !collider.shape().is_convex()
        }) {
            return Err(WorldError::Collision(
                CollisionError::UnsupportedDynamicGeometry,
            ));
        }
        if first.is_some_and(|id| self.colliders.get(id.dense).is_none()) {
            return Err(WorldError::InvalidColliderId);
        }
        collider.prepare_query_geometry()?;
        let has_finite_bounds = collider.local_bounds().ok().flatten().is_some();
        if !has_finite_bounds {
            self.unbounded_query_colliders
                .try_reserve(1)
                .map_err(|_| WorldError::Collision(CollisionError::SpatialIndexCapacityExceeded))?;
        }

        let dense = self
            .colliders
            .insert(ColliderEntry {
                collider,
                body,
                prev: None,
                next: first,
                chunk: None,
            })
            .map_err(WorldError::from)?;
        let id = ColliderId {
            world: self.world_id,
            dense,
        };

        if let Some(first) = first
            && let Some(entry) = self.colliders.get_mut(first.dense)
        {
            entry.prev = Some(id);
        }
        if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.first_collider = Some(id);
        }
        if has_finite_bounds {
            self.update_query_index_for_collider(id);
        } else {
            self.unbounded_query_colliders.push(id);
        }
        self.collision_system.mark_spatial_dirty();
        Ok(id)
    }

    /// Updates the deformed local vertices for a skinned-mesh collider.
    ///
    /// The pose is fixed for each physics step. Changes between steps are
    /// discrete, and CCD does not interpolate unknown bone trajectories.
    pub fn set_skin_pose(
        &mut self,
        id: ColliderId,
        matrices: &[Affine3],
    ) -> Result<(), CollisionError> {
        if id.world != self.world_id {
            return Err(CollisionError::InvalidSkinPose);
        }
        let entry = self
            .colliders
            .get_mut(id.dense)
            .ok_or(CollisionError::InvalidSkinPose)?;
        entry.collider.set_skin_pose(matrices)?;
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Ok(())
    }

    pub fn collider(&self, id: ColliderId) -> Option<&Collider> {
        if id.world != self.world_id {
            return None;
        }
        self.colliders.get(id.dense).map(|entry| &entry.collider)
    }

    pub fn collider_mut(&mut self, id: ColliderId) -> Option<&mut Collider> {
        if id.world != self.world_id {
            return None;
        }
        let collider = self.colliders.get(id.dense)?;
        if collider.chunk.is_some() {
            return None;
        }
        let body = collider.body;
        if self
            .bodies
            .get(body.dense)
            .is_some_and(|entry| entry.cluster.is_some())
        {
            return None;
        }
        if self.colliders.get(id.dense).is_some() {
            self.collision_system.invalidate_collider_contacts(id);
        }
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.colliders
            .get_mut(id.dense)
            .map(|entry| &mut entry.collider)
    }

    pub fn collider_body(&self, id: ColliderId) -> Option<BodyId> {
        if id.world != self.world_id {
            return None;
        }
        self.colliders.get(id.dense).map(|entry| entry.body)
    }

    pub fn remove_collider(&mut self, id: ColliderId) -> Option<Collider> {
        if id.world != self.world_id {
            return None;
        }
        let collider = self.colliders.get(id.dense)?;
        if collider.chunk.is_some() {
            return None;
        }
        let body = collider.body;
        if self
            .bodies
            .get(body.dense)
            .is_some_and(|entry| entry.cluster.is_some())
        {
            return None;
        }
        self.remove_collider_internal(id)
    }

    fn remove_collider_internal(&mut self, id: ColliderId) -> Option<Collider> {
        if id.world != self.world_id {
            return None;
        }
        let (body, prev, next) = {
            let entry = self.colliders.get(id.dense)?;
            (entry.body, entry.prev, entry.next)
        };
        let body_entry = self.bodies.get(body.dense)?;
        if prev.is_none() && body_entry.first_collider != Some(id) {
            return None;
        }
        if prev.is_some_and(|prev| {
            self.colliders
                .get(prev.dense)
                .is_none_or(|entry| entry.body != body || entry.next != Some(id))
        }) || next.is_some_and(|next| {
            self.colliders
                .get(next.dense)
                .is_none_or(|entry| entry.body != body || entry.prev != Some(id))
        }) {
            return None;
        }

        if !self.query_index_dirty {
            let remove_failed = self
                .query_index
                .as_mut()
                .is_some_and(|index| index.remove(id).is_err());
            if remove_failed {
                self.mark_query_index_dirty();
            }
        }
        self.unbounded_query_colliders
            .retain(|collider| *collider != id);

        let removed = self.colliders.remove(id.dense)?;
        if let Some(prev) = prev {
            if let Some(entry) = self.colliders.get_mut(prev.dense) {
                entry.next = next;
            }
        } else if let Some(entry) = self.bodies.get_mut(body.dense) {
            entry.first_collider = next;
        }
        if let Some(next) = next
            && let Some(entry) = self.colliders.get_mut(next.dense)
        {
            entry.prev = prev;
        }
        self.collision_system.mark_spatial_dirty();
        self.collision_system.invalidate_collider_contacts(id);
        Some(removed.collider)
    }

    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    pub fn remove_body(&mut self, id: BodyId) -> Option<RigidBody> {
        if id.world != self.world_id {
            return None;
        }
        let cluster = self.bodies.get(id.dense)?.cluster.clone();
        let mut next = self.bodies.get(id.dense)?.first_collider;
        while let Some(collider) = next {
            next = self.colliders.get(collider.dense)?.next;
            self.remove_collider_internal(collider)?;
        }
        self.remove_body_chunks(id);
        loop {
            let joint = self.joints.iter().find_map(|(dense, entry)| {
                (entry.body_a == id || entry.body_b == id).then_some(JointId {
                    world: self.world_id,
                    dense,
                })
            });
            let Some(joint) = joint else {
                break;
            };
            self.joints.remove(joint.dense)?;
            self.collision_system.invalidate_joint_warm_start(joint);
        }
        loop {
            let source = self.force_sources.iter().find_map(|(dense, source)| {
                (source.body == id).then_some(ForceSourceId {
                    world: self.world_id,
                    dense,
                })
            });
            let Some(source) = source else {
                break;
            };
            self.force_sources.remove(source.dense)?;
        }
        let removed = self.bodies.remove(id.dense).map(|entry| entry.body)?;
        if let Some(cluster) = cluster {
            for weld in cluster.welds {
                self.welds.remove(weld.dense);
            }
            for part in cluster.parts {
                self.parts.remove(part.dense);
            }
        }
        Some(removed)
    }

    /// Iterates over bodies in dense storage order, which may change after removal.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &RigidBody)> {
        self.bodies.iter().map(|(dense, entry)| {
            (
                BodyId {
                    world: self.world_id,
                    dense,
                },
                &entry.body,
            )
        })
    }

    /// Mutably iterates over non-cluster bodies in dense storage order.
    /// The order may change after removal; cluster-managed bodies are omitted.
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = (BodyId, &mut RigidBody)> {
        self.mark_query_index_dirty();
        self.collision_system.mark_spatial_dirty();
        self.bodies.iter_mut().filter_map(|(dense, entry)| {
            entry.cluster.is_none().then_some((
                BodyId {
                    world: self.world_id,
                    dense,
                },
                &mut entry.body,
            ))
        })
    }

    /// Iterates over colliders in dense storage order, which may change after removal.
    pub fn colliders(&self) -> impl Iterator<Item = (ColliderId, BodyId, &Collider)> {
        self.colliders.iter().map(|(dense, entry)| {
            (
                ColliderId {
                    world: self.world_id,
                    dense,
                },
                entry.body,
                &entry.collider,
            )
        })
    }

    /// Iterates over the colliders attached to `body` in linked-list order.
    pub fn colliders_of(
        &self,
        body: BodyId,
    ) -> Option<impl Iterator<Item = (ColliderId, &Collider)> + '_> {
        collider_entries_of(self.world_id, &self.bodies, &self.colliders, body)
            .map(|entries| entries.map(|(id, _, collider)| (id, collider)))
    }
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

fn validate_force_source_values(
    local_point: Vec3,
    local_force: Vec3,
) -> Result<(), ForceSourceError> {
    if !local_point.is_finite() || !local_force.is_finite() {
        return Err(ForceSourceError::NonFiniteValue);
    }
    Ok(())
}

pub(crate) fn transform_joint_frame(
    target_transform: crate::transform::Transform,
    frame: JointFrame,
) -> Result<JointFrame, crate::transform::TransformError> {
    let frame_transform =
        crate::transform::Transform::new(frame.local_anchor, frame.local_rotation)?;
    let transformed = target_transform.compose(&frame_transform)?;
    Ok(JointFrame::new(
        transformed.translation(),
        transformed.rotation(),
    ))
}

fn body_point_velocity(
    mut body: RigidBody,
    state: BodyState,
    point: Vec3,
) -> Result<Vec3, CollisionError> {
    body.set_state_preserving_forces(state);
    let center = body
        .world_center_of_mass()
        .map_err(|_| CollisionError::ArithmeticOverflow)?
        .unwrap_or(state.transform().translation());
    let angular_velocity = state
        .angular_velocity()
        .checked_cross(
            point
                .checked_sub(center)
                .ok_or(CollisionError::ArithmeticOverflow)?,
        )
        .ok_or(CollisionError::ArithmeticOverflow)?;
    state
        .linear_velocity()
        .checked_add(angular_velocity)
        .ok_or(CollisionError::ArithmeticOverflow)
}

fn collider_entries_of<'a>(
    world_id: u64,
    bodies: &DensePool<BodyEntry>,
    colliders: &'a DensePool<ColliderEntry>,
    body: BodyId,
) -> Option<impl Iterator<Item = (ColliderId, BodyId, &'a Collider)> + 'a> {
    if body.world != world_id {
        return None;
    }
    let mut next = bodies.get(body.dense)?.first_collider;
    Some(std::iter::from_fn(move || {
        let id = next?;
        let entry = colliders.get(id.dense)?;
        next = entry.next;
        Some((id, entry.body, &entry.collider))
    }))
}

fn update_query_index_for_body(
    index: &mut SpatialIndex,
    colliders: &DensePool<ColliderEntry>,
    world_id: u64,
    body_id: BodyId,
    body: &BodyEntry,
) -> Result<(), CollisionError> {
    let mut next = body.first_collider;
    while let Some(collider_id) = next {
        if collider_id.world != world_id {
            return Err(CollisionError::InvalidContact);
        }
        let entry = colliders
            .get(collider_id.dense)
            .filter(|entry| entry.body == body_id)
            .ok_or(CollisionError::InvalidContact)?;
        if let Some(bounds) = entry
            .collider
            .world_bounds(&body.body.state().transform())
            .map_err(|_| CollisionError::GeometryBounds)?
        {
            if !index.contains_bounds(collider_id, bounds)? {
                index.update(collider_id, expanded_query_bounds(bounds)?)?;
            }
        } else {
            index.remove(collider_id)?;
        }
        next = entry.next;
    }
    Ok(())
}

fn expanded_query_bounds(bounds: Aabb) -> Result<Aabb, CollisionError> {
    const MARGIN: f64 = 0.1;
    let margin = Vec3::splat(MARGIN);
    let min = bounds
        .min()
        .checked_sub(margin)
        .ok_or(CollisionError::GeometryBounds)?;
    let max = bounds
        .max()
        .checked_add(margin)
        .ok_or(CollisionError::GeometryBounds)?;
    Aabb::new(min, max).map_err(|_| CollisionError::GeometryBounds)
}

fn upsert_interaction(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    interaction: CollisionInteraction,
) -> Result<(), CollisionError> {
    let indices = indices.get_or_insert_with(HashMap::new);
    if let Some(&index) = indices.get(&interaction.pair) {
        let existing = interactions
            .get_mut(index)
            .ok_or(CollisionError::InvalidContact)?;
        merge_interaction(existing, interaction)?;
        return Ok(());
    }
    interactions
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    indices
        .try_reserve(1)
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    let index = interactions.len();
    interactions.push(interaction);
    indices.insert(interaction.pair, index);
    Ok(())
}

fn remove_transient_sensor_interaction(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    pair: CollisionPair,
) -> Result<(), CollisionError> {
    let Some(index) = indices
        .as_ref()
        .and_then(|indices| indices.get(&pair).copied())
    else {
        return Ok(());
    };
    let interaction = interactions
        .get(index)
        .ok_or(CollisionError::InvalidContact)?;
    if !interaction.sensor || !interaction.transient {
        return Ok(());
    }
    interactions.remove(index);
    let indices = indices.as_mut().ok_or(CollisionError::InvalidContact)?;
    indices.clear();
    indices
        .try_reserve(interactions.len())
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    for (index, interaction) in interactions.iter().enumerate() {
        if indices.insert(interaction.pair, index).is_some() {
            return Err(CollisionError::InvalidContact);
        }
    }
    Ok(())
}

fn remove_interactions_for_pairs(
    interactions: &mut Vec<CollisionInteraction>,
    indices: &mut Option<HashMap<CollisionPair, usize>>,
    pairs: &[CollisionPair],
) -> Result<(), CollisionError> {
    interactions.retain(|interaction| pairs.binary_search(&interaction.pair).is_err());
    let Some(indices) = indices else {
        return Ok(());
    };
    indices.clear();
    indices
        .try_reserve(interactions.len())
        .map_err(|_| CollisionError::SpatialIndexCapacityExceeded)?;
    for (index, interaction) in interactions.iter().enumerate() {
        if indices.insert(interaction.pair, index).is_some() {
            return Err(CollisionError::InvalidContact);
        }
    }
    Ok(())
}

fn merge_interaction(
    existing: &mut CollisionInteraction,
    incoming: CollisionInteraction,
) -> Result<(), CollisionError> {
    if existing.sensor != incoming.sensor {
        return Err(CollisionError::InvalidContact);
    }
    existing.transient &= incoming.transient;
    if let Some(contact) = incoming.contact
        && (!existing.impact || contact.normal_impulse > 0.0)
    {
        // A later resting manifold must not erase the contact that proves an
        // earlier CCD impact. Impact events require a positive actual impulse.
        existing.contact = Some(contact);
    }
    existing.impact |= incoming.impact;
    Ok(())
}

#[cfg(test)]
#[path = "world_tests.rs"]
mod tests;

#[cfg(test)]
fn test_atmosphere(state: crate::aerodynamics::AtmosphericState) -> Atmosphere {
    crate::aerodynamics::atmosphere::test_atmosphere(state)
}
