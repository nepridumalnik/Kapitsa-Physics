//! Host-driven command data and atomic command application.
//!
//! The async service reuses these types; they never depend on Tokio or Rayon.

use std::collections::HashMap;

use kapitsa_math::{Affine3, Vec3};

use crate::Error;

use crate::{
    cluster::{
        ClusterPart, ClusterPartDesc, MassChangeMode, MassTransfer, RigidCluster, Weld, WeldDesc,
    },
    collider::{Collider, CollisionFilter, CollisionMode},
    collision::{CollisionConfig, OverlapHit, QueryError, QueryFilter, Ray, RayHit, ShapeHit},
    gravity::Gravity,
    joint::Joint,
    mass::MassProperties,
    material::{MaterialId, PhysicsMaterial},
    rigid_body::{BodyError, BodyMotion, CcdMode, RigidBody},
    shape::Shape,
    transform::Transform,
    world::{
        BodyId, ClusterError, ColliderId, ForceSource, ForceSourceError, ForceSourceId, GravityId,
        JointId, JointTarget, PartId, PhysicsWorld, WeldId, WorldError,
    },
};

/// Maximum number of commands accepted in one atomic batch.
pub const MAX_COMMANDS_PER_BATCH: usize = 128;

/// Bound small-set scans to avoid hashing overhead on repeated-target batches.
pub const SMALL_STAGING_LINEAR_SEARCH_LIMIT: usize = 64;

/// Failure returned while applying one command to a world.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CommandError {
    World(WorldError),
    Body(BodyError),
    Cluster(ClusterError),
    Query(QueryError),
    ForceSource(ForceSourceError),
    /// The command cannot be used in an atomic mutation batch.
    UnsupportedBatchCommand,
    /// The buffer exceeded [`MAX_COMMANDS_PER_BATCH`].
    TooManyCommands {
        max_commands: usize,
    },
    /// A unified engine error that has no narrower command variant.
    Unified(Error),
}

impl std::fmt::Display for CommandError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::World(error) => write!(formatter, "physics world command failed: {error}"),
            Self::Body(error) => write!(formatter, "rigid body command failed: {error}"),
            Self::Cluster(error) => write!(formatter, "rigid cluster command failed: {error}"),
            Self::Query(error) => write!(formatter, "physics query failed: {error}"),
            Self::ForceSource(error) => write!(formatter, "force source command failed: {error}"),
            Self::UnsupportedBatchCommand => {
                formatter.write_str("command cannot be used in an atomic body batch")
            }
            Self::TooManyCommands { max_commands } => write!(
                formatter,
                "command batch exceeds the maximum of {max_commands} commands"
            ),
            Self::Unified(error) => error.fmt(formatter),
        }
    }
}

impl std::error::Error for CommandError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::World(error) => Some(error),
            Self::Body(error) => Some(error),
            Self::Cluster(error) => Some(error),
            Self::Query(error) => Some(error),
            Self::ForceSource(error) => Some(error),
            Self::Unified(error) => Some(error),
            Self::UnsupportedBatchCommand | Self::TooManyCommands { .. } => None,
        }
    }
}

impl CommandError {
    /// Converts a unified error, recovering a typed command variant when possible.
    pub fn from_unified(error: Error) -> Self {
        if let Some(error) = error.downcast_ref::<WorldError>() {
            return Self::World(error.clone());
        }
        if let Some(error) = error.downcast_ref::<BodyError>() {
            return Self::Body(*error);
        }
        if let Some(error) = error.downcast_ref::<ClusterError>() {
            return Self::from(error.clone());
        }
        if let Some(error) = error.downcast_ref::<QueryError>() {
            return Self::Query(*error);
        }
        if let Some(error) = error.downcast_ref::<ForceSourceError>() {
            return Self::ForceSource(*error);
        }
        Self::Unified(error)
    }
}

impl From<Error> for CommandError {
    fn from(error: Error) -> Self {
        Self::from_unified(error)
    }
}

impl From<WorldError> for CommandError {
    fn from(error: WorldError) -> Self {
        Self::World(error)
    }
}

impl From<BodyError> for CommandError {
    fn from(error: BodyError) -> Self {
        Self::Body(error)
    }
}

impl From<ClusterError> for CommandError {
    fn from(error: ClusterError) -> Self {
        match error {
            ClusterError::InvalidBodyId => Self::World(WorldError::InvalidBodyId),
            error => Self::Cluster(error),
        }
    }
}

impl From<QueryError> for CommandError {
    fn from(error: QueryError) -> Self {
        Self::Query(error)
    }
}

impl From<ForceSourceError> for CommandError {
    fn from(error: ForceSourceError) -> Self {
        Self::ForceSource(error)
    }
}

/// A batch failure with the index of the command that rejected it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommandBatchError {
    /// `None` when the whole buffer was rejected before per-command validation.
    pub command_index: Option<usize>,
    pub error: CommandError,
}

impl std::fmt::Display for CommandBatchError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.command_index {
            Some(index) => write!(formatter, "command {index} failed: {}", self.error),
            None => write!(formatter, "{}", self.error),
        }
    }
}

impl std::error::Error for CommandBatchError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.error)
    }
}

/// An operation applied to the simulation world at a physics-step boundary.
#[non_exhaustive]
pub enum PhysicsCommand {
    /// Queues cooking of the current assembly meshes.
    QueueBodyShape {
        body: BodyId,
        policy: crate::aerodynamics::shape::worker::PendingAero,
        worker: crate::aerodynamics::shape::worker::ShapeWorker,
    },
    /// Publishes available background results and reads the body's status.
    PollBodyShape {
        body: BodyId,
    },
    /// Cancels a tracked cook without replacing its binding.
    CancelBodyShape {
        body: BodyId,
    },
    /// Adds a material and returns its ID.
    AddMaterial {
        material: PhysicsMaterial,
    },
    /// Replaces a material, invalidating affected cached contacts.
    UpdateMaterial {
        id: MaterialId,
        material: PhysicsMaterial,
    },
    /// Removes an unassigned material.
    RemoveMaterial {
        id: MaterialId,
    },
    /// Reads a material at this request's position in the queue.
    GetMaterial {
        id: MaterialId,
    },
    /// Converts a dynamic body into a cluster and returns its root part.
    EnableCluster {
        body: BodyId,
    },
    /// Converts a body into a cluster with explicit root mass.
    EnableClusterWithMass {
        body: BodyId,
        mass: MassProperties,
    },
    /// Adds a welded part to a cluster.
    AddClusterPart {
        parent: PartId,
        desc: ClusterPartDesc,
        weld: WeldDesc,
    },
    /// Adds a collider in part-local coordinates.
    AddPartCollider {
        part: PartId,
        collider: Collider,
    },
    /// Replaces owned finite wings in the stable part-local frame; empty clears.
    /// Applied in queue order, including while paused; not an atomic-motion command.
    SetPartAerodynamicWings {
        part: PartId,
        wings: Vec<crate::aerodynamics::wing::WingSurface>,
    },
    /// Replaces owned hull sections in the stable part-local frame; empty clears.
    /// Applied in queue order, including while paused; not an atomic-motion command.
    SetPartAerodynamicHull {
        part: PartId,
        sections: Vec<crate::aerodynamics::hull::HullSection>,
    },
    /// Selects a part-local physical aerodynamic Mesh; None disables geometry.
    SetPartAerodynamicMesh {
        part: PartId,
        mesh: Option<crate::shape::TriangleMesh>,
    },
    /// Selects collider-derived part geometry, replacing Mesh mode when enabled.
    SetPartAerodynamicColliders {
        part: PartId,
        enabled: bool,
    },
    /// Selects the assembly wave model owned by this part.
    SetPartVoxelWave {
        part: PartId,
        model: Option<crate::aerodynamics::wave::VoxelWave>,
    },
    /// Selects a mesh as geometry-only input to the assembly wave union.
    SetPartWaveSourceMesh {
        part: PartId,
        mesh: Option<crate::shape::TriangleMesh>,
    },
    /// Selects part colliders as geometry-only input to the wave union.
    SetPartWaveSourceColliders {
        part: PartId,
        enabled: bool,
    },
    /// Changes an owned collider's transform in the part-local frame.
    SetPartColliderTransform {
        part: PartId,
        collider: ColliderId,
        transform: Transform,
    },
    /// Changes geometric aerodynamic tessellation for one solver body.
    SetColliderTessellation {
        body: BodyId,
        config: crate::aerodynamics::ColliderTessellation,
    },
    /// Replaces a body's primary aerodynamic model.
    SetAerodynamics {
        body: BodyId,
        model: crate::aerodynamics::Aerodynamics,
    },
    /// Removes a part-owned collider.
    RemovePartCollider {
        id: ColliderId,
    },
    /// Destroys a part when topology allows it.
    DestroyClusterPart {
        part: PartId,
    },
    /// Breaks a weld and returns the component body IDs.
    BreakWeld {
        weld: WeldId,
    },
    /// Detaches a part and returns the component body IDs.
    DetachPart {
        part: PartId,
    },
    /// Merges the second cluster into the first.
    MergeClusters {
        first: BodyId,
        second: BodyId,
        weld: WeldDesc,
    },
    /// Changes a part's mass using a mass-change policy.
    SetPartMass {
        part: PartId,
        mass: MassProperties,
        mode: MassChangeMode,
    },
    /// Changes a part's mass with explicit momentum transfer.
    SetPartMassWithTransfer {
        part: PartId,
        mass: MassProperties,
        transfer: MassTransfer,
    },
    /// Reads the cluster at this request's position in the queue.
    GetCluster {
        body: BodyId,
    },
    /// Reads the part at this request's position in the queue.
    GetClusterPart {
        part: PartId,
    },
    /// Reads the weld at this request's position in the queue.
    GetWeld {
        weld: WeldId,
    },
    /// Adds a joint between body or cluster-part targets.
    AddJointTargets {
        target_a: JointTarget,
        target_b: JointTarget,
        joint: Joint,
    },
    /// Updates a joint using its target-local frames.
    UpdateJointTargets {
        id: JointId,
        joint: Joint,
    },
    /// Adds a persistent force expressed in part-local coordinates.
    AddPartForceSource {
        part: PartId,
        local_point: Vec3,
        local_force: Vec3,
    },
    AddGravity {
        gravity: Gravity,
    },
    UpdateGravity {
        id: GravityId,
        gravity: Gravity,
    },
    RemoveGravity {
        id: GravityId,
    },
    /// Inserts a rigid body and returns its new [`BodyId`].
    AddBody {
        body: RigidBody,
    },
    /// Removes a body and its attached colliders.
    RemoveBody {
        id: BodyId,
    },
    /// Attaches a collider to a body and returns its new [`ColliderId`].
    AddCollider {
        body: BodyId,
        collider: Collider,
    },
    /// Creates a joint between two bodies and returns its new [`JointId`].
    AddJoint {
        body_a: BodyId,
        body_b: BodyId,
        joint: Joint,
    },
    /// Removes a joint.
    RemoveJoint {
        id: JointId,
    },
    /// Replaces a joint's settings while retaining its ID and attached bodies.
    UpdateJoint {
        id: JointId,
        joint: Joint,
    },
    /// Enables or disables a joint without changing its settings.
    SetJointEnabled {
        id: JointId,
        enabled: bool,
    },
    /// Removes a collider.
    RemoveCollider {
        id: ColliderId,
    },
    /// Creates a persistent local-space force source and returns its ID.
    AddForceSource {
        body: BodyId,
        local_point: Vec3,
        local_force: Vec3,
    },
    /// Removes a persistent force source.
    RemoveForceSource {
        id: ForceSourceId,
    },
    /// Updates the local point and force of an existing persistent source.
    SetForceSource {
        id: ForceSourceId,
        local_point: Vec3,
        local_force: Vec3,
    },
    /// Enables or disables a persistent force source.
    SetForceSourceEnabled {
        id: ForceSourceId,
        enabled: bool,
    },
    /// Replaces a body's transform while preserving its velocities.
    SetTransform {
        id: BodyId,
        transform: Transform,
    },
    /// Sets the world-space linear velocity.
    SetLinearVelocity {
        id: BodyId,
        velocity: Vec3,
    },
    /// Sets the world-space angular velocity.
    SetAngularVelocity {
        id: BodyId,
        velocity: Vec3,
    },
    /// Replaces the body's dynamic mass properties.
    SetMass {
        id: BodyId,
        mass: MassProperties,
    },
    /// Replaces the world's validated collision configuration.
    SetCollisionConfig {
        config: CollisionConfig,
    },
    /// Replaces the validated world atmosphere at this position in the command queue.
    SetAtmosphere {
        atmosphere: crate::aerodynamics::Atmosphere,
    },
    /// Moves, replaces or disables the world's point-mass gravity field.
    SetCentralGravity {
        gravity: Option<crate::gravity::CentralGravity>,
    },
    /// Sets continuous collision detection for a body.
    SetCcdMode {
        id: BodyId,
        mode: CcdMode,
    },
    /// Sleeps a dynamic body and clears its motion and forces.
    Sleep {
        id: BodyId,
    },
    /// Wakes a dynamic body.
    WakeUp {
        id: BodyId,
    },
    /// Accumulates a world-space force.
    ApplyForce {
        id: BodyId,
        force: Vec3,
    },
    /// Accumulates a world-space force at a world-space point.
    ApplyForceAtPoint {
        id: BodyId,
        force: Vec3,
        point: Vec3,
    },
    /// Accumulates a world-space torque.
    ApplyTorque {
        id: BodyId,
        torque: Vec3,
    },
    /// Applies a world-space impulse immediately.
    ApplyImpulse {
        id: BodyId,
        impulse: Vec3,
    },
    /// Applies a world-space impulse at a world-space point immediately.
    ApplyImpulseAtPoint {
        id: BodyId,
        impulse: Vec3,
        point: Vec3,
    },
    /// Clears accumulated force and torque.
    ClearForces {
        id: BodyId,
    },
    /// Replaces a collider's collision filter.
    SetColliderFilter {
        id: ColliderId,
        filter: CollisionFilter,
    },
    /// Sets whether an attached collider responds physically or only reports intersections.
    SetColliderMode {
        id: ColliderId,
        mode: CollisionMode,
    },
    /// Updates a skinned mesh collider's bone pose.
    SetSkinPose {
        id: ColliderId,
        matrices: Vec<Affine3>,
    },
    /// Assigns a material ID after resolving it in the world material store.
    SetColliderMaterial {
        id: ColliderId,
        material: Option<MaterialId>,
    },
    /// Casts a ray against the current world at this command's FIFO position.
    Raycast {
        ray: Ray,
        filter: QueryFilter,
    },
    /// Sweeps a convex shape against the current world at this command's FIFO position.
    Sweep {
        shape: Shape,
        transform: Transform,
        displacement: Vec3,
        filter: QueryFilter,
    },
    /// Finds colliders overlapping a convex shape at this command's FIFO position.
    Overlaps {
        shape: Shape,
        transform: Transform,
        filter: QueryFilter,
    },
}

/// Ordered body mutations submitted together before a physics step.
#[derive(Default)]
pub struct PhysicsCommandBuffer {
    pub(crate) commands: Vec<PhysicsCommand>,
}

impl std::fmt::Debug for PhysicsCommandBuffer {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("PhysicsCommandBuffer")
            .field("command_count", &self.commands.len())
            .finish()
    }
}

impl PhysicsCommandBuffer {
    /// Maximum number of commands accepted in one atomic batch.
    pub const MAX_COMMANDS: usize = MAX_COMMANDS_PER_BATCH;

    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            commands: Vec::with_capacity(capacity),
        }
    }

    pub fn reserve(&mut self, additional: usize) {
        self.commands.reserve(additional);
    }

    pub fn len(&self) -> usize {
        self.commands.len()
    }

    pub fn is_empty(&self) -> bool {
        self.commands.is_empty()
    }

    pub fn set_transform(&mut self, id: BodyId, transform: Transform) {
        self.commands
            .push(PhysicsCommand::SetTransform { id, transform });
    }

    pub fn set_linear_velocity(&mut self, id: BodyId, velocity: Vec3) {
        self.commands
            .push(PhysicsCommand::SetLinearVelocity { id, velocity });
    }

    pub fn set_angular_velocity(&mut self, id: BodyId, velocity: Vec3) {
        self.commands
            .push(PhysicsCommand::SetAngularVelocity { id, velocity });
    }

    pub fn set_mass(&mut self, id: BodyId, mass: MassProperties) {
        self.commands.push(PhysicsCommand::SetMass { id, mass });
    }

    pub fn set_ccd_mode(&mut self, id: BodyId, mode: CcdMode) {
        self.commands.push(PhysicsCommand::SetCcdMode { id, mode });
    }

    pub fn sleep(&mut self, id: BodyId) {
        self.commands.push(PhysicsCommand::Sleep { id });
    }

    pub fn wake_up(&mut self, id: BodyId) {
        self.commands.push(PhysicsCommand::WakeUp { id });
    }

    pub fn apply_force(&mut self, id: BodyId, force: Vec3) {
        self.commands.push(PhysicsCommand::ApplyForce { id, force });
    }

    pub fn apply_force_at_point(&mut self, id: BodyId, force: Vec3, point: Vec3) {
        self.commands
            .push(PhysicsCommand::ApplyForceAtPoint { id, force, point });
    }

    pub fn apply_torque(&mut self, id: BodyId, torque: Vec3) {
        self.commands
            .push(PhysicsCommand::ApplyTorque { id, torque });
    }

    pub fn apply_impulse(&mut self, id: BodyId, impulse: Vec3) {
        self.commands
            .push(PhysicsCommand::ApplyImpulse { id, impulse });
    }

    pub fn apply_impulse_at_point(&mut self, id: BodyId, impulse: Vec3, point: Vec3) {
        self.commands
            .push(PhysicsCommand::ApplyImpulseAtPoint { id, impulse, point });
    }

    pub fn clear_forces(&mut self, id: BodyId) {
        self.commands.push(PhysicsCommand::ClearForces { id });
    }

    /// Replaces a persistent force source's local point and force.
    pub fn set_force_source(&mut self, id: ForceSourceId, local_point: Vec3, local_force: Vec3) {
        self.commands.push(PhysicsCommand::SetForceSource {
            id,
            local_point,
            local_force,
        });
    }

    /// Enables or disables a persistent force source.
    pub fn set_force_source_enabled(&mut self, id: ForceSourceId, enabled: bool) {
        self.commands
            .push(PhysicsCommand::SetForceSourceEnabled { id, enabled });
    }

    /// Removes a joint as part of this atomic batch.
    pub fn remove_joint(&mut self, id: JointId) {
        self.commands.push(PhysicsCommand::RemoveJoint { id });
    }

    /// Replaces a joint's settings as part of this atomic batch.
    pub fn update_joint(&mut self, id: JointId, joint: Joint) {
        self.commands
            .push(PhysicsCommand::UpdateJoint { id, joint });
    }

    /// Enables or disables a joint as part of this atomic batch.
    pub fn set_joint_enabled(&mut self, id: JointId, enabled: bool) {
        self.commands
            .push(PhysicsCommand::SetJointEnabled { id, enabled });
    }

    /// Borrows the queued commands.
    #[doc(hidden)]
    pub fn commands(&self) -> &[PhysicsCommand] {
        &self.commands
    }

    /// Returns the command vector's capacity in elements.
    #[doc(hidden)]
    pub fn capacity(&self) -> usize {
        self.commands.capacity()
    }

    /// Removes all queued commands while preserving capacity.
    #[doc(hidden)]
    pub fn clear(&mut self) {
        self.commands.clear();
    }

    /// Takes the queued command vector, leaving an empty buffer.
    #[doc(hidden)]
    pub fn take_commands(&mut self) -> Vec<PhysicsCommand> {
        std::mem::take(&mut self.commands)
    }

    /// Rebuilds a buffer from a recycled command vector.
    #[doc(hidden)]
    pub fn from_commands(commands: Vec<PhysicsCommand>) -> Self {
        Self { commands }
    }
}

/// Successful result returned by a physics command.
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum PhysicsCommandResult {
    /// Latest background preparation status.
    BodyShapeStatus(crate::world::BodyShapeStatus),
    /// Whether a tracked cook was cancelled.
    BodyShapeCancelled(bool),
    /// The command completed without returning a newly allocated ID.
    Done,
    /// A body was inserted.
    Body(BodyId),
    /// A collider was inserted.
    Collider(ColliderId),
    /// A persistent force source was inserted.
    ForceSource(ForceSourceId),
    Gravity(GravityId),
    /// A newly created material ID.
    MaterialId(MaterialId),
    /// Material found by a read-only query.
    Material(Option<PhysicsMaterial>),
    /// A newly created cluster part ID.
    Part(PartId),
    /// Component body IDs after a split.
    Bodies(Vec<BodyId>),
    /// Cluster found by a read-only query.
    Cluster(Option<RigidCluster>),
    /// Cluster part found by a read-only query.
    ClusterPart(Option<ClusterPart>),
    /// Weld found by a read-only query.
    Weld(Option<Weld>),
    /// A joint was inserted.
    Joint(JointId),
    /// The result of a ray query.
    Raycast(Option<RayHit>),
    /// The result of a shape sweep.
    Sweep(Option<ShapeHit>),
    /// All colliders overlapping the requested shape.
    Overlaps(Vec<OverlapHit>),
}

struct StagedForceSource {
    id: ForceSourceId,
    local_point: Vec3,
    local_force: Vec3,
    enabled: bool,
    command_index: usize,
}

struct StagedJoint {
    id: JointId,
    joint: Joint,
    removed: bool,
    command_index: usize,
}

// Bound small-set scans to avoid hashing overhead on repeated-target batches.

#[derive(Default)]
pub struct BatchScratch {
    updated_bodies: Vec<(BodyId, BodyMotion)>,
    body_indices: HashMap<BodyId, usize>,
    updated_force_sources: Vec<StagedForceSource>,
    force_source_indices: HashMap<ForceSourceId, usize>,
    updated_joints: Vec<StagedJoint>,
    joint_indices: HashMap<JointId, usize>,
    joint_removals: Vec<(JointId, usize)>,
}

impl BatchScratch {
    fn clear(&mut self) {
        self.updated_bodies.clear();
        self.body_indices.clear();
        self.updated_force_sources.clear();
        self.force_source_indices.clear();
        self.updated_joints.clear();
        self.joint_indices.clear();
        self.joint_removals.clear();
    }

    /// Whether no staged command state remains.
    #[doc(hidden)]
    pub fn is_empty(&self) -> bool {
        self.updated_bodies.is_empty()
            && self.body_indices.is_empty()
            && self.updated_force_sources.is_empty()
            && self.force_source_indices.is_empty()
            && self.updated_joints.is_empty()
            && self.joint_indices.is_empty()
            && self.joint_removals.is_empty()
    }

    /// Retained capacities: bodies, body indices, force sources, source indices.
    #[doc(hidden)]
    pub fn capacities(&self) -> [usize; 4] {
        [
            self.updated_bodies.capacity(),
            self.body_indices.capacity(),
            self.updated_force_sources.capacity(),
            self.force_source_indices.capacity(),
        ]
    }
}

pub fn apply_command_batch(
    world: &mut PhysicsWorld,
    commands: &[PhysicsCommand],
    scratch: &mut BatchScratch,
) -> Result<(), crate::Error> {
    scratch.clear();
    let result = apply_command_batch_staged(world, commands, scratch);
    scratch.clear();
    result
}

fn apply_command_batch_staged(
    world: &mut PhysicsWorld,
    commands: &[PhysicsCommand],
    scratch: &mut BatchScratch,
) -> Result<(), crate::Error> {
    for (command_index, command) in commands.iter().enumerate() {
        if let PhysicsCommand::SetForceSource {
            id,
            local_point,
            local_force,
        } = command
        {
            let source_index = staged_force_source_index(world, scratch, *id, command_index)?;
            if !local_point.is_finite() || !local_force.is_finite() {
                return Err(crate::Error::from(CommandBatchError {
                    command_index: Some(command_index),
                    error: CommandError::ForceSource(ForceSourceError::NonFiniteValue),
                }));
            }
            let source = &mut scratch.updated_force_sources[source_index];
            source.local_point = *local_point;
            source.local_force = *local_force;
            source.command_index = command_index;
            continue;
        }
        if let PhysicsCommand::SetForceSourceEnabled { id, enabled } = command {
            let source_index = staged_force_source_index(world, scratch, *id, command_index)?;
            let source = &mut scratch.updated_force_sources[source_index];
            source.enabled = *enabled;
            source.command_index = command_index;
            continue;
        }
        match command {
            PhysicsCommand::UpdateJoint { id, joint } => {
                let index = staged_joint_index(world, scratch, *id, command_index)?;
                let staged = &mut scratch.updated_joints[index];
                if staged.removed {
                    return Err(crate::Error::from(invalid_joint_batch_command(
                        command_index,
                    )));
                }
                if let Err(error) = joint.validate() {
                    return Err(crate::Error::from(CommandBatchError {
                        command_index: Some(command_index),
                        error: CommandError::from_unified(error),
                    }));
                }
                staged.joint = *joint;
                staged.command_index = command_index;
                continue;
            }
            PhysicsCommand::SetJointEnabled { id, enabled } => {
                let index = staged_joint_index(world, scratch, *id, command_index)?;
                let staged = &mut scratch.updated_joints[index];
                if staged.removed {
                    return Err(crate::Error::from(invalid_joint_batch_command(
                        command_index,
                    )));
                }
                staged.joint.set_enabled(*enabled);
                staged.command_index = command_index;
                continue;
            }
            PhysicsCommand::RemoveJoint { id } => {
                let index = staged_joint_index(world, scratch, *id, command_index)?;
                let staged = &mut scratch.updated_joints[index];
                if staged.removed {
                    return Err(crate::Error::from(invalid_joint_batch_command(
                        command_index,
                    )));
                }
                staged.removed = true;
                staged.command_index = command_index;
                scratch.joint_removals.push((*id, command_index));
                continue;
            }
            _ => {}
        }

        let Some(id) = body_command_id(command) else {
            return Err(crate::Error::from(CommandBatchError {
                command_index: Some(command_index),
                error: CommandError::UnsupportedBatchCommand,
            }));
        };

        if matches!(command, PhysicsCommand::SetMass { .. }) && world.cluster(id).is_some() {
            return Err(crate::Error::from(CommandBatchError {
                command_index: Some(command_index),
                error: CommandError::Cluster(ClusterError::ManagedMass),
            }));
        }

        let body_index = if scratch.updated_bodies.len() <= SMALL_STAGING_LINEAR_SEARCH_LIMIT {
            scratch
                .updated_bodies
                .iter()
                .position(|(updated_id, _)| *updated_id == id)
        } else {
            scratch.body_indices.get(&id).copied()
        };
        let body_index = match body_index {
            Some(index) => index,
            None => {
                let Some(body) = world.body(id).map(RigidBody::motion) else {
                    return Err(crate::Error::from(CommandBatchError {
                        command_index: Some(command_index),
                        error: CommandError::World(WorldError::InvalidBodyId),
                    }));
                };
                let index = scratch.updated_bodies.len();
                if index >= SMALL_STAGING_LINEAR_SEARCH_LIMIT {
                    if scratch.body_indices.is_empty() {
                        for (existing_index, (existing_id, _)) in
                            scratch.updated_bodies.iter().enumerate()
                        {
                            scratch.body_indices.insert(*existing_id, existing_index);
                        }
                    }
                    scratch.body_indices.insert(id, index);
                }
                scratch.updated_bodies.push((id, body));
                index
            }
        };

        if let Err(error) = apply_body_command(&mut scratch.updated_bodies[body_index].1, command) {
            return Err(crate::Error::from(CommandBatchError {
                command_index: Some(command_index),
                error: error.into(),
            }));
        }
    }

    for source in &scratch.updated_force_sources {
        world
            .set_force_source(source.id, source.local_point, source.local_force)
            .and_then(|()| world.set_force_source_enabled(source.id, source.enabled))
            .map_err(|error| CommandBatchError {
                command_index: Some(source.command_index),
                error: CommandError::from_unified(error),
            })?;
    }

    for (id, command_index) in &scratch.joint_removals {
        world.remove_joint(*id).ok_or(CommandBatchError {
            command_index: Some(*command_index),
            error: CommandError::World(WorldError::InvalidJointId),
        })?;
    }
    for joint in &scratch.updated_joints {
        if !joint.removed {
            world
                .update_joint(joint.id, joint.joint)
                .map_err(|error| CommandBatchError {
                    command_index: Some(joint.command_index),
                    error: CommandError::from_unified(error),
                })?;
        }
    }

    // Batch commands do not change body membership, so each staged body ID
    // remains live until the final copyback.
    for (id, body) in &scratch.updated_bodies {
        world
            .edit_body(*id, |motion| {
                *motion = *body;
                Ok(())
            })
            .map_err(|error| {
                crate::Error::from(CommandBatchError {
                    command_index: None,
                    error: error.into(),
                })
            })?;
    }
    Ok(())
}

fn staged_force_source_index(
    world: &PhysicsWorld,
    scratch: &mut BatchScratch,
    id: ForceSourceId,
    command_index: usize,
) -> Result<usize, crate::Error> {
    let source_index = if scratch.updated_force_sources.len() <= SMALL_STAGING_LINEAR_SEARCH_LIMIT {
        scratch
            .updated_force_sources
            .iter()
            .position(|source| source.id == id)
    } else {
        scratch.force_source_indices.get(&id).copied()
    };
    if let Some(index) = source_index {
        return Ok(index);
    }
    let Some(source) = world.force_source(id) else {
        return Err(crate::Error::from(CommandBatchError {
            command_index: Some(command_index),
            error: CommandError::ForceSource(ForceSourceError::InvalidForceSourceId),
        }));
    };
    let source: ForceSource = *source;
    let index = scratch.updated_force_sources.len();
    if index >= SMALL_STAGING_LINEAR_SEARCH_LIMIT {
        if scratch.force_source_indices.is_empty() {
            for (existing_index, source) in scratch.updated_force_sources.iter().enumerate() {
                scratch
                    .force_source_indices
                    .insert(source.id, existing_index);
            }
        }
        scratch.force_source_indices.insert(id, index);
    }
    scratch.updated_force_sources.push(StagedForceSource {
        id,
        local_point: source.local_point(),
        local_force: source.local_force(),
        enabled: source.is_enabled(),
        command_index,
    });
    Ok(index)
}

fn staged_joint_index(
    world: &PhysicsWorld,
    scratch: &mut BatchScratch,
    id: JointId,
    command_index: usize,
) -> Result<usize, crate::Error> {
    let index = if scratch.updated_joints.len() <= SMALL_STAGING_LINEAR_SEARCH_LIMIT {
        scratch
            .updated_joints
            .iter()
            .position(|joint| joint.id == id)
    } else {
        scratch.joint_indices.get(&id).copied()
    };
    if let Some(index) = index {
        return Ok(index);
    }
    let Some(joint) = world.joint(id).copied() else {
        return Err(crate::Error::from(invalid_joint_batch_command(
            command_index,
        )));
    };
    let index = scratch.updated_joints.len();
    if index >= SMALL_STAGING_LINEAR_SEARCH_LIMIT {
        if scratch.joint_indices.is_empty() {
            for (existing_index, existing) in scratch.updated_joints.iter().enumerate() {
                scratch.joint_indices.insert(existing.id, existing_index);
            }
        }
        scratch.joint_indices.insert(id, index);
    }
    scratch.updated_joints.push(StagedJoint {
        id,
        joint,
        removed: false,
        command_index,
    });
    Ok(index)
}

fn invalid_joint_batch_command(command_index: usize) -> CommandBatchError {
    CommandBatchError {
        command_index: Some(command_index),
        error: CommandError::World(WorldError::InvalidJointId),
    }
}

fn body_command_id(command: &PhysicsCommand) -> Option<BodyId> {
    match command {
        PhysicsCommand::SetTransform { id, .. }
        | PhysicsCommand::SetLinearVelocity { id, .. }
        | PhysicsCommand::SetAngularVelocity { id, .. }
        | PhysicsCommand::SetMass { id, .. }
        | PhysicsCommand::SetCcdMode { id, .. }
        | PhysicsCommand::Sleep { id }
        | PhysicsCommand::WakeUp { id }
        | PhysicsCommand::ApplyForce { id, .. }
        | PhysicsCommand::ApplyForceAtPoint { id, .. }
        | PhysicsCommand::ApplyTorque { id, .. }
        | PhysicsCommand::ApplyImpulse { id, .. }
        | PhysicsCommand::ApplyImpulseAtPoint { id, .. }
        | PhysicsCommand::ClearForces { id } => Some(*id),
        PhysicsCommand::AddJointTargets { .. }
        | PhysicsCommand::UpdateJointTargets { .. }
        | PhysicsCommand::AddPartForceSource { .. }
        | PhysicsCommand::AddMaterial { .. }
        | PhysicsCommand::UpdateMaterial { .. }
        | PhysicsCommand::RemoveMaterial { .. }
        | PhysicsCommand::GetMaterial { .. }
        | PhysicsCommand::EnableCluster { .. }
        | PhysicsCommand::EnableClusterWithMass { .. }
        | PhysicsCommand::AddClusterPart { .. }
        | PhysicsCommand::AddPartCollider { .. }
        | PhysicsCommand::SetPartAerodynamicWings { .. }
        | PhysicsCommand::SetPartAerodynamicHull { .. }
        | PhysicsCommand::SetPartAerodynamicMesh { .. }
        | PhysicsCommand::SetPartAerodynamicColliders { .. }
        | PhysicsCommand::SetPartVoxelWave { .. }
        | PhysicsCommand::SetPartWaveSourceMesh { .. }
        | PhysicsCommand::SetPartWaveSourceColliders { .. }
        | PhysicsCommand::SetPartColliderTransform { .. }
        | PhysicsCommand::SetColliderTessellation { .. }
        | PhysicsCommand::SetAerodynamics { .. }
        | PhysicsCommand::RemovePartCollider { .. }
        | PhysicsCommand::DestroyClusterPart { .. }
        | PhysicsCommand::BreakWeld { .. }
        | PhysicsCommand::DetachPart { .. }
        | PhysicsCommand::MergeClusters { .. }
        | PhysicsCommand::SetPartMass { .. }
        | PhysicsCommand::SetPartMassWithTransfer { .. }
        | PhysicsCommand::GetCluster { .. }
        | PhysicsCommand::GetClusterPart { .. }
        | PhysicsCommand::QueueBodyShape { .. }
        | PhysicsCommand::PollBodyShape { .. }
        | PhysicsCommand::CancelBodyShape { .. }
        | PhysicsCommand::GetWeld { .. }
        | PhysicsCommand::AddGravity { .. }
        | PhysicsCommand::UpdateGravity { .. }
        | PhysicsCommand::RemoveGravity { .. }
        | PhysicsCommand::AddBody { .. }
        | PhysicsCommand::RemoveBody { .. }
        | PhysicsCommand::AddCollider { .. }
        | PhysicsCommand::RemoveCollider { .. }
        | PhysicsCommand::AddJoint { .. }
        | PhysicsCommand::RemoveJoint { .. }
        | PhysicsCommand::UpdateJoint { .. }
        | PhysicsCommand::SetJointEnabled { .. }
        | PhysicsCommand::AddForceSource { .. }
        | PhysicsCommand::RemoveForceSource { .. }
        | PhysicsCommand::SetForceSource { .. }
        | PhysicsCommand::SetForceSourceEnabled { .. }
        | PhysicsCommand::SetCollisionConfig { .. }
        | PhysicsCommand::SetAtmosphere { .. }
        | PhysicsCommand::SetCentralGravity { .. }
        | PhysicsCommand::SetColliderFilter { .. }
        | PhysicsCommand::SetColliderMode { .. }
        | PhysicsCommand::SetSkinPose { .. }
        | PhysicsCommand::SetColliderMaterial { .. }
        | PhysicsCommand::Raycast { .. }
        | PhysicsCommand::Sweep { .. }
        | PhysicsCommand::Overlaps { .. } => None,
    }
}

fn apply_body_command(body: &mut BodyMotion, command: &PhysicsCommand) -> Result<(), Error> {
    match command {
        PhysicsCommand::SetTransform { transform, .. } => body.set_transform(*transform)?,
        PhysicsCommand::SetLinearVelocity { velocity, .. } => {
            body.set_linear_velocity(*velocity)?;
        }
        PhysicsCommand::SetAngularVelocity { velocity, .. } => {
            body.set_angular_velocity(*velocity)?;
        }
        PhysicsCommand::SetMass { mass, .. } => body.set_mass(*mass)?,
        PhysicsCommand::SetCcdMode { mode, .. } => body.set_ccd_mode(*mode),
        PhysicsCommand::Sleep { .. } => body.sleep()?,
        PhysicsCommand::WakeUp { .. } => body.wake_up(),
        PhysicsCommand::ApplyForce { force, .. } => body.apply_force(*force)?,
        PhysicsCommand::ApplyForceAtPoint { force, point, .. } => {
            body.apply_force_at_point(*force, *point)?;
        }
        PhysicsCommand::ApplyTorque { torque, .. } => body.apply_torque(*torque)?,
        PhysicsCommand::ApplyImpulse { impulse, .. } => body.apply_impulse(*impulse)?,
        PhysicsCommand::ApplyImpulseAtPoint { impulse, point, .. } => {
            body.apply_impulse_at_point(*impulse, *point)?;
        }
        PhysicsCommand::ClearForces { .. } => body.clear_forces(),
        PhysicsCommand::AddJointTargets { .. }
        | PhysicsCommand::UpdateJointTargets { .. }
        | PhysicsCommand::AddPartForceSource { .. }
        | PhysicsCommand::AddMaterial { .. }
        | PhysicsCommand::UpdateMaterial { .. }
        | PhysicsCommand::RemoveMaterial { .. }
        | PhysicsCommand::GetMaterial { .. }
        | PhysicsCommand::EnableCluster { .. }
        | PhysicsCommand::EnableClusterWithMass { .. }
        | PhysicsCommand::AddClusterPart { .. }
        | PhysicsCommand::AddPartCollider { .. }
        | PhysicsCommand::SetPartAerodynamicWings { .. }
        | PhysicsCommand::SetPartAerodynamicHull { .. }
        | PhysicsCommand::SetPartAerodynamicMesh { .. }
        | PhysicsCommand::SetPartAerodynamicColliders { .. }
        | PhysicsCommand::SetPartVoxelWave { .. }
        | PhysicsCommand::SetPartWaveSourceMesh { .. }
        | PhysicsCommand::SetPartWaveSourceColliders { .. }
        | PhysicsCommand::SetPartColliderTransform { .. }
        | PhysicsCommand::SetColliderTessellation { .. }
        | PhysicsCommand::SetAerodynamics { .. }
        | PhysicsCommand::RemovePartCollider { .. }
        | PhysicsCommand::DestroyClusterPart { .. }
        | PhysicsCommand::BreakWeld { .. }
        | PhysicsCommand::DetachPart { .. }
        | PhysicsCommand::MergeClusters { .. }
        | PhysicsCommand::SetPartMass { .. }
        | PhysicsCommand::SetPartMassWithTransfer { .. }
        | PhysicsCommand::GetCluster { .. }
        | PhysicsCommand::GetClusterPart { .. }
        | PhysicsCommand::QueueBodyShape { .. }
        | PhysicsCommand::PollBodyShape { .. }
        | PhysicsCommand::CancelBodyShape { .. }
        | PhysicsCommand::GetWeld { .. }
        | PhysicsCommand::AddGravity { .. }
        | PhysicsCommand::UpdateGravity { .. }
        | PhysicsCommand::RemoveGravity { .. }
        | PhysicsCommand::AddBody { .. }
        | PhysicsCommand::RemoveBody { .. }
        | PhysicsCommand::AddCollider { .. }
        | PhysicsCommand::RemoveCollider { .. }
        | PhysicsCommand::AddJoint { .. }
        | PhysicsCommand::RemoveJoint { .. }
        | PhysicsCommand::UpdateJoint { .. }
        | PhysicsCommand::SetJointEnabled { .. }
        | PhysicsCommand::AddForceSource { .. }
        | PhysicsCommand::RemoveForceSource { .. }
        | PhysicsCommand::SetForceSource { .. }
        | PhysicsCommand::SetForceSourceEnabled { .. }
        | PhysicsCommand::SetCollisionConfig { .. }
        | PhysicsCommand::SetAtmosphere { .. }
        | PhysicsCommand::SetCentralGravity { .. }
        | PhysicsCommand::SetColliderFilter { .. }
        | PhysicsCommand::SetColliderMode { .. }
        | PhysicsCommand::SetSkinPose { .. }
        | PhysicsCommand::SetColliderMaterial { .. }
        | PhysicsCommand::Raycast { .. }
        | PhysicsCommand::Sweep { .. }
        | PhysicsCommand::Overlaps { .. } => {
            unreachable!("only body commands reach apply_body_command");
        }
    }
    Ok(())
}

pub(crate) fn apply_command_inner(
    world: &mut PhysicsWorld,
    command: PhysicsCommand,
) -> Result<PhysicsCommandResult, crate::Error> {
    let result = match command {
        PhysicsCommand::AddJointTargets {
            target_a,
            target_b,
            joint,
        } => PhysicsCommandResult::Joint(world.add_joint_targets(target_a, target_b, joint)?),
        PhysicsCommand::UpdateJointTargets { id, joint } => {
            world.update_joint_targets(id, joint)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::AddPartForceSource {
            part,
            local_point,
            local_force,
        } => PhysicsCommandResult::ForceSource(world.add_force_source_for_part(
            part,
            local_point,
            local_force,
        )?),
        PhysicsCommand::AddMaterial { material } => {
            PhysicsCommandResult::MaterialId(world.add_material(material)?)
        }
        PhysicsCommand::UpdateMaterial { id, material } => {
            world.update_material(id, material)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::RemoveMaterial { id } => {
            world.remove_material(id)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::GetMaterial { id } => {
            PhysicsCommandResult::Material(world.material(id).copied())
        }
        PhysicsCommand::EnableCluster { body } => {
            PhysicsCommandResult::Part(world.enable_cluster(body)?)
        }
        PhysicsCommand::EnableClusterWithMass { body, mass } => {
            PhysicsCommandResult::Part(world.enable_cluster_with_mass(body, mass)?)
        }
        PhysicsCommand::AddClusterPart { parent, desc, weld } => {
            PhysicsCommandResult::Part(world.add_cluster_part(parent, desc, weld)?)
        }
        PhysicsCommand::AddPartCollider { part, collider } => {
            PhysicsCommandResult::Collider(world.add_part_collider(part, collider)?)
        }
        PhysicsCommand::SetPartAerodynamicWings { part, wings } => {
            world.set_part_aerodynamic_wings(part, wings)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartAerodynamicHull { part, sections } => {
            world.set_part_aerodynamic_hull(part, sections)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartAerodynamicMesh { part, mesh } => {
            world.set_part_aerodynamic_mesh(part, mesh)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartAerodynamicColliders { part, enabled } => {
            world.set_part_aerodynamic_colliders(part, enabled)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartVoxelWave { part, model } => {
            world.set_part_voxel_wave(part, model)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartWaveSourceMesh { part, mesh } => {
            world.set_part_wave_source_mesh(part, mesh)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartWaveSourceColliders { part, enabled } => {
            world.set_part_wave_source_colliders(part, enabled)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartColliderTransform {
            part,
            collider,
            transform,
        } => {
            world.set_part_collider_transform(part, collider, transform)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetColliderTessellation { body, config } => {
            world.set_collider_tessellation(body, config)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetAerodynamics { body, model } => {
            world.set_aerodynamics(body, model)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::RemovePartCollider { id } => {
            world.remove_part_collider(id)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::DestroyClusterPart { part } => {
            world.destroy_cluster_part(part)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::BreakWeld { weld } => PhysicsCommandResult::Bodies(world.break_weld(weld)?),
        PhysicsCommand::DetachPart { part } => {
            PhysicsCommandResult::Bodies(world.detach_part(part)?)
        }
        PhysicsCommand::MergeClusters {
            first,
            second,
            weld,
        } => {
            world.merge_clusters(first, second, weld)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartMass { part, mass, mode } => {
            world.set_part_mass(part, mass, mode)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetPartMassWithTransfer {
            part,
            mass,
            transfer,
        } => {
            world.set_part_mass_with_transfer(part, mass, transfer)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::GetCluster { body } => {
            PhysicsCommandResult::Cluster(world.cluster(body).cloned())
        }
        PhysicsCommand::GetClusterPart { part } => {
            PhysicsCommandResult::ClusterPart(world.cluster_part(part).cloned())
        }
        PhysicsCommand::QueueBodyShape {
            body,
            policy,
            worker,
        } => {
            world.queue_body_shape(body, policy, &worker)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::CancelBodyShape { body } => {
            PhysicsCommandResult::BodyShapeCancelled(world.cancel_body_shape(body))
        }
        PhysicsCommand::PollBodyShape { body } => {
            world.poll_body_shapes();
            PhysicsCommandResult::BodyShapeStatus(world.body_shape_status(body))
        }
        PhysicsCommand::GetWeld { weld } => PhysicsCommandResult::Weld(world.weld(weld).copied()),
        PhysicsCommand::AddGravity { gravity } => {
            PhysicsCommandResult::Gravity(world.add_gravity(gravity)?)
        }
        PhysicsCommand::UpdateGravity { id, gravity } => {
            world.update_gravity(id, gravity)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::RemoveGravity { id } => {
            world
                .remove_gravity(id)
                .ok_or(WorldError::InvalidGravityId)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::AddBody { body } => PhysicsCommandResult::Body(world.add_body(body)?),
        PhysicsCommand::RemoveBody { id } => {
            world.remove_body(id).ok_or(WorldError::InvalidBodyId)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::AddCollider { body, collider } => {
            PhysicsCommandResult::Collider(world.add_collider(body, collider)?)
        }
        PhysicsCommand::AddJoint {
            body_a,
            body_b,
            joint,
        } => PhysicsCommandResult::Joint(world.add_joint(body_a, body_b, joint)?),
        PhysicsCommand::RemoveJoint { id } => {
            world.remove_joint(id).ok_or(WorldError::InvalidJointId)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::UpdateJoint { id, joint } => {
            world.update_joint(id, joint)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetJointEnabled { id, enabled } => {
            world.set_joint_enabled(id, enabled)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::RemoveCollider { id } => {
            world
                .remove_collider(id)
                .ok_or(WorldError::InvalidColliderId)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::AddForceSource {
            body,
            local_point,
            local_force,
        } => PhysicsCommandResult::ForceSource(world.add_force_source(
            body,
            local_point,
            local_force,
        )?),
        PhysicsCommand::RemoveForceSource { id } => {
            world.remove_force_source(id)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetForceSource {
            id,
            local_point,
            local_force,
        } => {
            world.set_force_source(id, local_point, local_force)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetForceSourceEnabled { id, enabled } => {
            world.set_force_source_enabled(id, enabled)?;
            PhysicsCommandResult::Done
        }
        command @ (PhysicsCommand::SetTransform { .. }
        | PhysicsCommand::SetLinearVelocity { .. }
        | PhysicsCommand::SetAngularVelocity { .. }
        | PhysicsCommand::SetMass { .. }
        | PhysicsCommand::SetCcdMode { .. }
        | PhysicsCommand::Sleep { .. }
        | PhysicsCommand::WakeUp { .. }
        | PhysicsCommand::ApplyForce { .. }
        | PhysicsCommand::ApplyForceAtPoint { .. }
        | PhysicsCommand::ApplyTorque { .. }
        | PhysicsCommand::ApplyImpulse { .. }
        | PhysicsCommand::ApplyImpulseAtPoint { .. }
        | PhysicsCommand::ClearForces { .. }) => {
            let Some(id) = body_command_id(&command) else {
                return Err(crate::Error::from(CommandError::UnsupportedBatchCommand));
            };
            if matches!(command, PhysicsCommand::SetMass { .. }) && world.cluster(id).is_some() {
                return Err(crate::Error::from(CommandError::Cluster(
                    ClusterError::ManagedMass,
                )));
            }
            world.edit_body(id, |body| apply_body_command(body, &command))?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetAtmosphere { atmosphere } => {
            world.set_atmosphere(atmosphere);
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetCentralGravity { gravity } => {
            world.set_central_gravity(gravity);
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetCollisionConfig { config } => {
            world
                .set_collision_config(config)
                .map_err(CommandError::from_unified)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetColliderFilter { id, filter } => {
            world
                .collider_mut(id)
                .ok_or(WorldError::InvalidColliderId)?
                .set_filter(filter);
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetColliderMode { id, mode } => {
            world.set_collider_mode(id, mode)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetSkinPose { id, matrices } => {
            world
                .set_skin_pose(id, &matrices)
                .map_err(CommandError::from_unified)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetColliderMaterial { id, material } => {
            world.set_collider_material(id, material)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::Raycast { ray, filter } => {
            PhysicsCommandResult::Raycast(world.raycast(ray, filter)?)
        }
        PhysicsCommand::Sweep {
            shape,
            transform,
            displacement,
            filter,
        } => PhysicsCommandResult::Sweep(world.sweep(&shape, transform, displacement, filter)?),
        PhysicsCommand::Overlaps {
            shape,
            transform,
            filter,
        } => {
            let mut overlaps = Vec::new();
            world.overlaps(&shape, transform, filter, &mut overlaps)?;
            PhysicsCommandResult::Overlaps(overlaps)
        }
    };
    Ok(result)
}

impl PhysicsWorld {
    /// Creates an empty command buffer for this world.
    pub fn commands(&self) -> PhysicsCommandBuffer {
        PhysicsCommandBuffer::new()
    }

    /// Applies a command buffer atomically without stepping the world.
    ///
    /// Validation follows buffer order and reports the rejecting command index.
    /// No partial mutation is committed on failure. This method never steps the
    /// world or publishes a snapshot; call [`PhysicsWorld::step`] afterwards.
    pub fn apply(&mut self, commands: &PhysicsCommandBuffer) -> Result<(), crate::Error> {
        if commands.commands.len() > PhysicsCommandBuffer::MAX_COMMANDS {
            return Err(crate::Error::from(CommandBatchError {
                command_index: None,
                error: CommandError::TooManyCommands {
                    max_commands: PhysicsCommandBuffer::MAX_COMMANDS,
                },
            }));
        }
        if commands.commands.is_empty() {
            return Ok(());
        }
        let mut scratch = self.command_scratch.take().unwrap_or_default();
        let result = apply_command_batch(self, &commands.commands, &mut scratch);
        self.command_scratch = Some(scratch);
        result
    }

    /// Applies an ordered command slice with caller-provided reusable scratch.
    ///
    /// This is the low-level form used by the async service. Prefer
    /// [`PhysicsWorld::apply`] for host-driven loops.
    #[doc(hidden)]
    pub fn apply_commands(
        &mut self,
        commands: &[PhysicsCommand],
        scratch: &mut BatchScratch,
    ) -> Result<(), crate::Error> {
        if commands.is_empty() {
            return Ok(());
        }
        apply_command_batch(self, commands, scratch)
    }

    /// Applies one command immediately, preserving its typed failure.
    pub fn apply_command(
        &mut self,
        command: PhysicsCommand,
    ) -> Result<PhysicsCommandResult, crate::Error> {
        apply_command_inner(self, command)
    }
}
