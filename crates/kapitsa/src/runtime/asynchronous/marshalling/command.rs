use std::collections::HashMap;

use kapitsa_core::{
    collider::{Collider, CollisionFilter, CollisionMode},
    collision::{CollisionConfig, OverlapHit, QueryFilter, Ray, RayHit, ShapeHit},
    mass::MassProperties,
    material::MaterialId,
    rigid_body::{CcdMode, RigidBody},
    shape::Shape,
    transform::Transform,
    world::{
        BodyId, ColliderId, ForceSource, ForceSourceError, ForceSourceId, PhysicsWorld, WorldError,
    },
};
use kapitsa_math::{Affine3, Vec3};
use tokio::sync::oneshot;

use super::PhysicsClientError;
use super::events::PhysicsBatchError;

/// Correlates an accepted command buffer with its asynchronous result.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BatchId(u64);

impl BatchId {
    pub(crate) fn new(value: u64) -> Self {
        Self(value)
    }

    pub fn get(self) -> u64 {
        self.0
    }
}

/// An operation applied to the simulation world at a physics-step boundary.
pub enum PhysicsCommand {
    /// Inserts a rigid body and returns its new [`BodyId`].
    AddBody { body: RigidBody },
    /// Removes a body and its attached colliders.
    RemoveBody { id: BodyId },
    /// Attaches a collider to a body and returns its new [`ColliderId`].
    AddCollider { body: BodyId, collider: Collider },
    /// Removes a collider.
    RemoveCollider { id: ColliderId },
    /// Creates a persistent local-space force source and returns its ID.
    AddForceSource {
        body: BodyId,
        local_point: Vec3,
        local_force: Vec3,
    },
    /// Removes a persistent force source.
    RemoveForceSource { id: ForceSourceId },
    /// Updates the local point and force of an existing persistent source.
    SetForceSource {
        id: ForceSourceId,
        local_point: Vec3,
        local_force: Vec3,
    },
    /// Enables or disables a persistent force source.
    SetForceSourceEnabled { id: ForceSourceId, enabled: bool },
    /// Replaces a body's transform while preserving its velocities.
    SetTransform { id: BodyId, transform: Transform },
    /// Sets the world-space linear velocity.
    SetLinearVelocity { id: BodyId, velocity: Vec3 },
    /// Sets the world-space angular velocity.
    SetAngularVelocity { id: BodyId, velocity: Vec3 },
    /// Replaces the body's dynamic mass properties.
    SetMass { id: BodyId, mass: MassProperties },
    /// Replaces the world's validated collision configuration.
    SetCollisionConfig { config: CollisionConfig },
    /// Sets continuous collision detection for a body.
    SetCcdMode { id: BodyId, mode: CcdMode },
    /// Sleeps a dynamic body and clears its motion and forces.
    Sleep { id: BodyId },
    /// Wakes a dynamic body.
    WakeUp { id: BodyId },
    /// Accumulates a world-space force.
    ApplyForce { id: BodyId, force: Vec3 },
    /// Accumulates a world-space force at a world-space point.
    ApplyForceAtPoint {
        id: BodyId,
        force: Vec3,
        point: Vec3,
    },
    /// Accumulates a world-space torque.
    ApplyTorque { id: BodyId, torque: Vec3 },
    /// Applies a world-space impulse immediately.
    ApplyImpulse { id: BodyId, impulse: Vec3 },
    /// Applies a world-space impulse at a world-space point immediately.
    ApplyImpulseAtPoint {
        id: BodyId,
        impulse: Vec3,
        point: Vec3,
    },
    /// Clears accumulated force and torque.
    ClearForces { id: BodyId },
    /// Replaces a collider's collision filter.
    SetColliderFilter {
        id: ColliderId,
        filter: CollisionFilter,
    },
    /// Sets whether an attached collider responds physically or only reports intersections.
    SetColliderMode { id: ColliderId, mode: CollisionMode },
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
    Raycast { ray: Ray, filter: QueryFilter },
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
    pub const MAX_COMMANDS: usize = 128;

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
}

/// A synchronous failure while submitting a command buffer.
#[derive(Debug)]
pub enum PhysicsSubmitError {
    EmptyBuffer(PhysicsCommandBuffer),
    TooManyCommands {
        buffer: PhysicsCommandBuffer,
        max_commands: usize,
    },
    BatchIdExhausted(PhysicsCommandBuffer),
    NotRunning(PhysicsCommandBuffer),
    QueueFull(PhysicsCommandBuffer),
    ChannelClosed(PhysicsCommandBuffer),
}

impl PhysicsSubmitError {
    pub fn into_buffer(self) -> PhysicsCommandBuffer {
        match self {
            Self::EmptyBuffer(buffer)
            | Self::BatchIdExhausted(buffer)
            | Self::NotRunning(buffer)
            | Self::QueueFull(buffer)
            | Self::ChannelClosed(buffer) => buffer,
            Self::TooManyCommands { buffer, .. } => buffer,
        }
    }
}

impl std::fmt::Display for PhysicsSubmitError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message = match self {
            Self::EmptyBuffer(_) => "physics command buffer is empty",
            Self::TooManyCommands { max_commands, .. } => {
                return write!(
                    formatter,
                    "physics command buffer exceeds the {max_commands}-command limit"
                );
            }
            Self::BatchIdExhausted(_) => "physics batch identifier space is exhausted",
            Self::NotRunning(_) => "physics simulation is not running",
            Self::QueueFull(_) => "physics command queue is full",
            Self::ChannelClosed(_) => "physics command channel is closed",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for PhysicsSubmitError {}

/// Successful result returned by a physics command.
#[derive(Debug, Clone, PartialEq)]
pub enum PhysicsCommandResult {
    /// The command completed without returning a newly allocated ID.
    Done,
    /// A body was inserted.
    Body(BodyId),
    /// A collider was inserted.
    Collider(ColliderId),
    /// A persistent force source was inserted.
    ForceSource(ForceSourceId),
    /// The result of a ray query.
    Raycast(Option<RayHit>),
    /// The result of a shape sweep.
    Sweep(Option<ShapeHit>),
    /// All colliders overlapping the requested shape.
    Overlaps(Vec<OverlapHit>),
}

// Keep single-command requests inline so the existing async path adds no per-command allocation.
#[allow(clippy::large_enum_variant)]
pub(crate) enum CommandRequest {
    Single {
        command: PhysicsCommand,
        reply: oneshot::Sender<Result<PhysicsCommandResult, PhysicsClientError>>,
    },
    Batch {
        id: BatchId,
        commands: Vec<PhysicsCommand>,
    },
}

impl CommandRequest {
    pub(crate) fn command_count(&self) -> usize {
        match self {
            Self::Single { .. } => 1,
            Self::Batch { commands, .. } => commands.len(),
        }
    }
}

struct StagedForceSource {
    id: ForceSourceId,
    local_point: Vec3,
    local_force: Vec3,
    enabled: bool,
    command_index: usize,
}

// Bound small-set scans to avoid hashing overhead on repeated-target batches.
const SMALL_STAGING_LINEAR_SEARCH_LIMIT: usize = 64;

#[derive(Default)]
pub(crate) struct BatchScratch {
    updated_bodies: Vec<(BodyId, RigidBody)>,
    body_indices: HashMap<BodyId, usize>,
    updated_force_sources: Vec<StagedForceSource>,
    force_source_indices: HashMap<ForceSourceId, usize>,
}

impl BatchScratch {
    fn clear(&mut self) {
        self.updated_bodies.clear();
        self.body_indices.clear();
        self.updated_force_sources.clear();
        self.force_source_indices.clear();
    }
}

pub(crate) fn apply_command_batch(
    world: &mut PhysicsWorld,
    commands: &[PhysicsCommand],
    scratch: &mut BatchScratch,
) -> Result<(), PhysicsBatchError> {
    scratch.clear();
    let result = apply_command_batch_staged(world, commands, scratch);
    scratch.clear();
    result
}

fn apply_command_batch_staged(
    world: &mut PhysicsWorld,
    commands: &[PhysicsCommand],
    scratch: &mut BatchScratch,
) -> Result<(), PhysicsBatchError> {
    for (command_index, command) in commands.iter().enumerate() {
        if let PhysicsCommand::SetForceSource {
            id,
            local_point,
            local_force,
        } = command
        {
            let source_index = staged_force_source_index(world, scratch, *id, command_index)?;
            if !local_point.is_finite() || !local_force.is_finite() {
                return Err(PhysicsBatchError {
                    command_index: Some(command_index),
                    error: PhysicsClientError::ForceSource(ForceSourceError::NonFiniteValue),
                });
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

        let Some(id) = body_command_id(command) else {
            return Err(PhysicsBatchError {
                command_index: Some(command_index),
                error: PhysicsClientError::UnsupportedBatchCommand,
            });
        };

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
                let Some(body) = world.body(id).copied() else {
                    return Err(PhysicsBatchError {
                        command_index: Some(command_index),
                        error: PhysicsClientError::World(WorldError::InvalidBodyId),
                    });
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
            return Err(PhysicsBatchError {
                command_index: Some(command_index),
                error,
            });
        }
    }

    for source in &scratch.updated_force_sources {
        world
            .set_force_source(source.id, source.local_point, source.local_force)
            .and_then(|()| world.set_force_source_enabled(source.id, source.enabled))
            .map_err(|error| PhysicsBatchError {
                command_index: Some(source.command_index),
                error: PhysicsClientError::ForceSource(error),
            })?;
    }

    // These commands cannot change world membership, so every ID validated above
    // remains live until the staged body copies are committed.
    for (id, body) in &scratch.updated_bodies {
        if let Some(target) = world.body_mut(*id) {
            *target = *body;
        }
    }
    Ok(())
}

fn staged_force_source_index(
    world: &PhysicsWorld,
    scratch: &mut BatchScratch,
    id: ForceSourceId,
    command_index: usize,
) -> Result<usize, PhysicsBatchError> {
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
        return Err(PhysicsBatchError {
            command_index: Some(command_index),
            error: PhysicsClientError::ForceSource(ForceSourceError::InvalidForceSourceId),
        });
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
        PhysicsCommand::AddBody { .. }
        | PhysicsCommand::RemoveBody { .. }
        | PhysicsCommand::AddCollider { .. }
        | PhysicsCommand::RemoveCollider { .. }
        | PhysicsCommand::AddForceSource { .. }
        | PhysicsCommand::RemoveForceSource { .. }
        | PhysicsCommand::SetForceSource { .. }
        | PhysicsCommand::SetForceSourceEnabled { .. }
        | PhysicsCommand::SetCollisionConfig { .. }
        | PhysicsCommand::SetColliderFilter { .. }
        | PhysicsCommand::SetColliderMode { .. }
        | PhysicsCommand::SetSkinPose { .. }
        | PhysicsCommand::SetColliderMaterial { .. }
        | PhysicsCommand::Raycast { .. }
        | PhysicsCommand::Sweep { .. }
        | PhysicsCommand::Overlaps { .. } => None,
    }
}

fn apply_body_command(
    body: &mut RigidBody,
    command: &PhysicsCommand,
) -> Result<(), PhysicsClientError> {
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
        PhysicsCommand::AddBody { .. }
        | PhysicsCommand::RemoveBody { .. }
        | PhysicsCommand::AddCollider { .. }
        | PhysicsCommand::RemoveCollider { .. }
        | PhysicsCommand::AddForceSource { .. }
        | PhysicsCommand::RemoveForceSource { .. }
        | PhysicsCommand::SetForceSource { .. }
        | PhysicsCommand::SetForceSourceEnabled { .. }
        | PhysicsCommand::SetCollisionConfig { .. }
        | PhysicsCommand::SetColliderFilter { .. }
        | PhysicsCommand::SetColliderMode { .. }
        | PhysicsCommand::SetSkinPose { .. }
        | PhysicsCommand::SetColliderMaterial { .. }
        | PhysicsCommand::Raycast { .. }
        | PhysicsCommand::Sweep { .. }
        | PhysicsCommand::Overlaps { .. } => {
            return Err(PhysicsClientError::UnsupportedBatchCommand);
        }
    }
    Ok(())
}

pub(crate) fn apply_command(
    world: &mut PhysicsWorld,
    command: PhysicsCommand,
) -> Result<PhysicsCommandResult, PhysicsClientError> {
    let result = match command {
        PhysicsCommand::AddBody { body } => PhysicsCommandResult::Body(world.add_body(body)?),
        PhysicsCommand::RemoveBody { id } => {
            world.remove_body(id).ok_or(WorldError::InvalidBodyId)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::AddCollider { body, collider } => {
            PhysicsCommandResult::Collider(world.add_collider(body, collider)?)
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
        PhysicsCommand::SetTransform { id, transform } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_transform(transform)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetLinearVelocity { id, velocity } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_linear_velocity(velocity)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetAngularVelocity { id, velocity } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_angular_velocity(velocity)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetMass { id, mass } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_mass(mass)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetCollisionConfig { config } => {
            world
                .set_collision_config(config)
                .map_err(WorldError::Collision)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::SetCcdMode { id, mode } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .set_ccd_mode(mode);
            PhysicsCommandResult::Done
        }
        PhysicsCommand::Sleep { id } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .sleep()?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::WakeUp { id } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .wake_up();
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyForce { id, force } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_force(force)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyForceAtPoint { id, force, point } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_force_at_point(force, point)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyTorque { id, torque } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_torque(torque)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyImpulse { id, impulse } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_impulse(impulse)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ApplyImpulseAtPoint { id, impulse, point } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .apply_impulse_at_point(impulse, point)?;
            PhysicsCommandResult::Done
        }
        PhysicsCommand::ClearForces { id } => {
            world
                .body_mut(id)
                .ok_or(WorldError::InvalidBodyId)?
                .clear_forces();
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
                .map_err(WorldError::Collision)?;
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

#[cfg(test)]
mod tests {
    #[cfg(feature = "bench-internals")]
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::collections::HashMap;
    use std::hint::black_box;
    use std::time::{Duration, Instant};

    use kapitsa_core::{
        collider::{Collider, CollisionFilter, CollisionMode},
        collision::{QueryError, QueryFilter, Ray},
        mass::MassProperties,
        material::{MaterialStorage, PhysicsMaterial},
        rigid_body::{BodyError, RigidBody},
        shape::{Shape, Sphere},
        transform::Transform,
        world::{ForceSourceError, PhysicsWorld, WorldError},
    };
    use kapitsa_math::{Mat3, Quat, Vec3};

    use super::super::PhysicsClientError;
    use super::{
        BatchScratch, PhysicsCommand, PhysicsCommandResult, SMALL_STAGING_LINEAR_SEARCH_LIMIT,
        StagedForceSource, apply_command, apply_command_batch,
    };

    #[cfg(feature = "bench-internals")]
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    #[cfg(feature = "bench-internals")]
    static TRACK_ALLOCATIONS: AtomicBool = AtomicBool::new(false);
    #[cfg(feature = "bench-internals")]
    static ALLOCATION_COUNT: AtomicUsize = AtomicUsize::new(0);
    #[cfg(feature = "bench-internals")]
    static ALLOCATION_BYTES: AtomicUsize = AtomicUsize::new(0);

    #[cfg(feature = "bench-internals")]
    struct TrackingAllocator;

    #[cfg(feature = "bench-internals")]
    unsafe impl GlobalAlloc for TrackingAllocator {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let pointer = unsafe { System.alloc(layout) };
            if !pointer.is_null() && TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
                ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
                ALLOCATION_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            }
            pointer
        }

        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            let pointer = unsafe { System.alloc_zeroed(layout) };
            if !pointer.is_null() && TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
                ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
                ALLOCATION_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
            }
            pointer
        }

        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let new_pointer = unsafe { System.realloc(pointer, layout, size) };
            if !new_pointer.is_null() && TRACK_ALLOCATIONS.load(Ordering::Relaxed) {
                ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
                ALLOCATION_BYTES.fetch_add(size, Ordering::Relaxed);
            }
            new_pointer
        }

        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            unsafe { System.dealloc(pointer, layout) };
        }
    }

    #[cfg(feature = "bench-internals")]
    #[global_allocator]
    static TEST_ALLOCATOR: TrackingAllocator = TrackingAllocator;

    #[cfg(feature = "bench-internals")]
    struct AllocationTracking;

    #[cfg(feature = "bench-internals")]
    impl AllocationTracking {
        fn start() -> Self {
            ALLOCATION_COUNT.store(0, Ordering::Relaxed);
            ALLOCATION_BYTES.store(0, Ordering::Relaxed);
            TRACK_ALLOCATIONS.store(true, Ordering::Relaxed);
            Self
        }
    }

    #[cfg(feature = "bench-internals")]
    impl Drop for AllocationTracking {
        fn drop(&mut self) {
            TRACK_ALLOCATIONS.store(false, Ordering::Relaxed);
        }
    }

    #[cfg(feature = "bench-internals")]
    #[derive(Clone, Copy)]
    struct BatchAllocationStats {
        count: usize,
        bytes: usize,
    }

    #[cfg(feature = "bench-internals")]
    fn tracked_batch_allocations(
        world: &mut PhysicsWorld,
        commands: &[PhysicsCommand],
        scratch: &mut BatchScratch,
    ) -> BatchAllocationStats {
        let tracking = AllocationTracking::start();
        let result = apply_command_batch(world, commands, scratch);
        drop(tracking);
        result.expect("benchmark batch applies");
        BatchAllocationStats {
            count: ALLOCATION_COUNT.load(Ordering::Relaxed),
            bytes: ALLOCATION_BYTES.load(Ordering::Relaxed),
        }
    }

    #[derive(Clone, Copy)]
    enum BatchScenario {
        RepeatedBody,
        DistinctBodies,
        MixedBodiesAndSources,
    }

    impl BatchScenario {
        const ALL: [Self; 3] = [
            Self::RepeatedBody,
            Self::DistinctBodies,
            Self::MixedBodiesAndSources,
        ];

        fn name(self) -> &'static str {
            match self {
                Self::RepeatedBody => "same-body",
                Self::DistinctBodies => "distinct-bodies",
                Self::MixedBodiesAndSources => "mixed-body-source",
            }
        }
    }

    struct BatchCase {
        world: PhysicsWorld,
        commands: Vec<PhysicsCommand>,
        scratch: BatchScratch,
        staged_bodies: Vec<kapitsa_core::world::BodyId>,
        staged_sources: Vec<kapitsa_core::world::ForceSourceId>,
    }

    fn batch_case(scenario: BatchScenario, command_count: usize) -> BatchCase {
        let mut world = PhysicsWorld::new();
        let mut commands = Vec::with_capacity(command_count);
        let mut staged_bodies = Vec::new();
        let mut staged_sources = Vec::new();

        match scenario {
            BatchScenario::RepeatedBody => {
                let body = world.add_body(dynamic_body()).expect("body");
                staged_bodies.push(body);
                for index in 0..command_count {
                    commands.push(PhysicsCommand::SetLinearVelocity {
                        id: body,
                        velocity: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
                    });
                }
            }
            BatchScenario::DistinctBodies => {
                for index in 0..command_count {
                    let body = world.add_body(dynamic_body()).expect("body");
                    staged_bodies.push(body);
                    commands.push(PhysicsCommand::SetLinearVelocity {
                        id: body,
                        velocity: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
                    });
                }
            }
            BatchScenario::MixedBodiesAndSources => {
                let body = world.add_body(dynamic_body()).expect("body");
                staged_bodies.push(body);
                for index in 0..command_count / 2 {
                    let source = world
                        .add_force_source(body, Vec3::ZERO, Vec3::X)
                        .expect("force source");
                    staged_sources.push(source);
                    commands.push(PhysicsCommand::SetLinearVelocity {
                        id: body,
                        velocity: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
                    });
                    commands.push(PhysicsCommand::SetForceSource {
                        id: source,
                        local_point: Vec3::Y,
                        local_force: Vec3::new(index as f64 + 2.0, 0.0, 0.0),
                    });
                }
            }
        }

        BatchCase {
            world,
            commands,
            scratch: BatchScratch::default(),
            staged_bodies,
            staged_sources,
        }
    }

    fn apply_batch(case: &mut BatchCase) {
        apply_command_batch(
            &mut case.world,
            black_box(&case.commands),
            &mut case.scratch,
        )
        .expect("benchmark batch applies");
    }

    fn staged_buffer_control(case: &BatchCase) {
        let mut bodies = Vec::with_capacity(case.commands.len());
        for id in &case.staged_bodies {
            bodies.push((*id, *case.world.body(*id).expect("staged body")));
        }

        let mut sources = Vec::new();
        for (command_index, id) in case.staged_sources.iter().copied().enumerate() {
            let source = *case.world.force_source(id).expect("staged source");
            sources.push(StagedForceSource {
                id,
                local_point: source.local_point(),
                local_force: source.local_force(),
                enabled: source.is_enabled(),
                command_index,
            });
        }
        black_box((&bodies, &sources));
    }

    fn average_ns(samples: &[f64]) -> (f64, f64, f64) {
        let mut sorted = samples.to_vec();
        sorted.sort_by(f64::total_cmp);
        (
            sorted[0],
            sorted[sorted.len() / 2],
            sorted[sorted.len() - 1],
        )
    }

    fn repeats_for(target: Duration, pilot: Duration) -> usize {
        let target_ns = target.as_nanos().max(1);
        let pilot_ns = pilot.as_nanos().max(1);
        usize::try_from(target_ns.div_ceil(pilot_ns))
            .unwrap_or(usize::MAX)
            .clamp(1, 65_536)
    }

    fn calibrated_repeats(
        scenario: BatchScenario,
        command_count: usize,
        target: Duration,
        mut run: impl FnMut(&mut BatchCase),
    ) -> usize {
        const CALIBRATION_REPEATS: usize = 4;
        let mut case = batch_case(scenario, command_count);
        run(&mut case);
        let started = Instant::now();
        for _ in 0..CALIBRATION_REPEATS {
            run(&mut case);
        }
        repeats_for(target, started.elapsed() / CALIBRATION_REPEATS as u32)
    }

    fn run_samples(
        scenario: BatchScenario,
        command_count: usize,
        repeats: usize,
        mut run: impl FnMut(&mut BatchCase),
        mut verify: impl FnMut(&BatchCase),
    ) -> (f64, f64, f64) {
        const SAMPLES: usize = 5;
        let mut samples = Vec::with_capacity(SAMPLES);
        for _ in 0..SAMPLES {
            let mut case = batch_case(scenario, command_count);
            run(&mut case);
            let started = Instant::now();
            for _ in 0..repeats {
                run(&mut case);
            }
            samples.push(started.elapsed().as_secs_f64() * 1.0e9 / repeats as f64);
            verify(&case);
        }
        average_ns(&samples)
    }

    fn verify_batch_result(case: &BatchCase) {
        let mut expected_bodies = HashMap::new();
        let mut expected_sources = HashMap::new();
        for command in &case.commands {
            match command {
                PhysicsCommand::SetLinearVelocity { id, velocity } => {
                    expected_bodies.insert(*id, *velocity);
                }
                PhysicsCommand::SetForceSource {
                    id,
                    local_point,
                    local_force,
                } => {
                    expected_sources.insert(*id, (*local_point, *local_force));
                }
                _ => panic!("unexpected command in benchmark batch"),
            }
        }
        for (id, expected) in expected_bodies {
            assert_eq!(
                case.world
                    .body(id)
                    .expect("updated body")
                    .state()
                    .linear_velocity(),
                expected
            );
        }
        for (id, (expected_point, expected_force)) in expected_sources {
            let source = case.world.force_source(id).expect("updated source");
            assert_eq!(source.local_point(), expected_point);
            assert_eq!(source.local_force(), expected_force);
        }
    }

    fn mass(value: f64) -> MassProperties {
        MassProperties::new(value, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass")
    }

    fn dynamic_body() -> RigidBody {
        RigidBody::new_dynamic(Transform::identity(), mass(1.0)).expect("dynamic body")
    }

    fn collider() -> Collider {
        Collider::new(
            Shape::Sphere(Sphere::new(1.0).expect("valid sphere")),
            Transform::identity(),
        )
    }

    fn assert_done(result: Result<PhysicsCommandResult, PhysicsClientError>) {
        assert!(matches!(result, Ok(PhysicsCommandResult::Done)));
    }

    #[test]
    fn add_remove_and_reused_body_ids_are_handled_by_world() {
        let mut world = PhysicsWorld::new();
        let PhysicsCommandResult::Body(first) = apply_command(
            &mut world,
            PhysicsCommand::AddBody {
                body: dynamic_body(),
            },
        )
        .expect("add body") else {
            panic!("add body must return its ID");
        };
        assert!(world.body(first).is_some());
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::RemoveBody { id: first },
        ));
        assert!(world.body(first).is_none());

        let PhysicsCommandResult::Body(replacement) = apply_command(
            &mut world,
            PhysicsCommand::AddBody {
                body: dynamic_body(),
            },
        )
        .expect("reuse body slot") else {
            panic!("add body must return its ID");
        };
        assert_eq!(first.index(), replacement.index());
        assert_ne!(first.generation(), replacement.generation());
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::SetTransform {
                    id: first,
                    transform: Transform::identity(),
                }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
    }

    #[test]
    fn commands_reject_body_ids_from_another_world() {
        let mut source = PhysicsWorld::new();
        let foreign = source
            .add_body(dynamic_body())
            .expect("source body insertion");
        let mut target = PhysicsWorld::new();
        let local = target
            .add_body(dynamic_body())
            .expect("target body insertion");

        assert!(matches!(
            apply_command(
                &mut target,
                PhysicsCommand::SetLinearVelocity {
                    id: foreign,
                    velocity: Vec3::X,
                }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
        assert!(matches!(
            apply_command(&mut target, PhysicsCommand::RemoveBody { id: foreign }),
            Err(PhysicsClientError::World(WorldError::InvalidBodyId))
        ));
        assert_eq!(target.body_count(), 1);
        assert!(target.body(local).is_some());
        assert_eq!(
            target
                .body(local)
                .expect("local body remains")
                .state()
                .linear_velocity(),
            Vec3::ZERO
        );
    }

    #[test]
    fn body_commands_delegate_to_existing_body_operations() {
        let mut world = PhysicsWorld::new();
        let id = world.add_body(dynamic_body()).expect("body");
        let transform = Transform::new(Vec3::X, Quat::IDENTITY).expect("transform");
        let commands = [
            PhysicsCommand::SetTransform { id, transform },
            PhysicsCommand::SetLinearVelocity {
                id,
                velocity: Vec3::X,
            },
            PhysicsCommand::SetAngularVelocity {
                id,
                velocity: Vec3::Y,
            },
            PhysicsCommand::SetMass {
                id,
                mass: mass(2.0),
            },
            PhysicsCommand::ApplyForce { id, force: Vec3::X },
            PhysicsCommand::ApplyForceAtPoint {
                id,
                force: Vec3::Y,
                point: Vec3::Z,
            },
            PhysicsCommand::ApplyTorque {
                id,
                torque: Vec3::Z,
            },
            PhysicsCommand::ApplyImpulse {
                id,
                impulse: Vec3::X,
            },
            PhysicsCommand::ApplyImpulseAtPoint {
                id,
                impulse: Vec3::Y,
                point: Vec3::Z,
            },
        ];
        for command in commands {
            assert_done(apply_command(&mut world, command));
        }
        let body = world.body(id).expect("body remains");
        assert_eq!(body.state().transform(), transform);
        assert_eq!(body.physics().mass(), Some(mass(2.0)));
        assert_ne!(body.forces().force(), Vec3::ZERO);
        assert_ne!(body.forces().torque(), Vec3::ZERO);
        assert_eq!(body.state().linear_velocity(), Vec3::new(1.5, 0.5, 0.0));

        assert_done(apply_command(&mut world, PhysicsCommand::Sleep { id }));
        assert!(world.body(id).expect("body").state().is_sleeping());
        assert_done(apply_command(&mut world, PhysicsCommand::WakeUp { id }));
        assert!(!world.body(id).expect("body").state().is_sleeping());

        apply_command(
            &mut world,
            PhysicsCommand::ApplyForce { id, force: Vec3::X },
        )
        .expect("apply force");
        apply_command(&mut world, PhysicsCommand::ClearForces { id }).expect("clear forces");
        assert_eq!(world.body(id).expect("body").forces().force(), Vec3::ZERO);
        assert_eq!(world.body(id).expect("body").forces().torque(), Vec3::ZERO);
    }

    #[test]
    fn invalid_body_and_numeric_commands_return_errors_without_partial_changes() {
        let mut world = PhysicsWorld::new();
        let id = world.add_body(dynamic_body()).expect("body");
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::ApplyForce {
                    id,
                    force: Vec3::new(f64::INFINITY, 0.0, 0.0),
                }
            ),
            Err(PhysicsClientError::Body(BodyError::NonFiniteValue))
        ));
        let body = world.body(id).expect("body unchanged");
        assert_eq!(body.forces().force(), Vec3::ZERO);

        let invalid = world
            .add_body(RigidBody::new_kinematic(Transform::identity()))
            .unwrap();
        assert!(matches!(
            apply_command(&mut world, PhysicsCommand::Sleep { id: invalid }),
            Err(PhysicsClientError::Body(BodyError::BodyMustBeDynamic))
        ));
    }

    #[test]
    fn collider_commands_update_remove_and_follow_body_removal() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let PhysicsCommandResult::Collider(collider_id) = apply_command(
            &mut world,
            PhysicsCommand::AddCollider {
                body,
                collider: collider(),
            },
        )
        .expect("add collider") else {
            panic!("add collider must return its ID");
        };
        let filter = CollisionFilter::new(3, 7);
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::SetColliderFilter {
                id: collider_id,
                filter,
            },
        ));
        assert_eq!(
            world.collider(collider_id).expect("collider").filter(),
            filter
        );
        assert_done(apply_command(
            &mut world,
            PhysicsCommand::SetColliderMode {
                id: collider_id,
                mode: CollisionMode::Sensor,
            },
        ));
        assert_eq!(
            world.collider(collider_id).expect("collider").mode(),
            CollisionMode::Sensor
        );
        apply_command(
            &mut world,
            PhysicsCommand::SetColliderMaterial {
                id: collider_id,
                material: None,
            },
        )
        .expect("clear material");
        assert_eq!(
            world.collider(collider_id).expect("collider").material(),
            None
        );
        apply_command(&mut world, PhysicsCommand::RemoveBody { id: body })
            .expect("remove body and attached collider");
        assert!(world.collider(collider_id).is_none());
        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::RemoveCollider { id: collider_id }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidColliderId))
        ));
    }

    #[test]
    fn collider_material_command_rejects_ids_from_other_storages() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let collider = world.add_collider(body, collider()).expect("collider");
        let mut foreign_materials = MaterialStorage::new();
        let foreign_id = foreign_materials
            .insert(PhysicsMaterial::default())
            .expect("foreign material");

        assert!(matches!(
            apply_command(
                &mut world,
                PhysicsCommand::SetColliderMaterial {
                    id: collider,
                    material: Some(foreign_id),
                }
            ),
            Err(PhysicsClientError::World(WorldError::InvalidMaterialId))
        ));
        assert_eq!(world.collider(collider).expect("collider").material(), None);
    }

    #[test]
    fn spatial_queries_run_through_the_world_command_handler() {
        use kapitsa_core::collision::{OverlapHit, RayHit, ShapeHit};

        let mut world = PhysicsWorld::new();
        let body = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .unwrap();
        let collider = world
            .add_collider(
                body,
                Collider::new(
                    Shape::Sphere(Sphere::new(1.0).unwrap()),
                    Transform::new(Vec3::new(3.0, 0.0, 0.0), kapitsa_math::Quat::IDENTITY).unwrap(),
                ),
            )
            .unwrap();
        let filter = QueryFilter::default();
        let ray = Ray::new(Vec3::ZERO, Vec3::X, 10.0).unwrap();
        let result = apply_command(&mut world, PhysicsCommand::Raycast { ray, filter }).unwrap();
        assert!(
            matches!(result, PhysicsCommandResult::Raycast(Some(RayHit { collider: id, .. })) if id == collider)
        );

        let result = apply_command(
            &mut world,
            PhysicsCommand::Sweep {
                shape: Shape::Sphere(Sphere::new(0.5).unwrap()),
                transform: Transform::identity(),
                displacement: Vec3::new(5.0, 0.0, 0.0),
                filter,
            },
        )
        .unwrap();
        assert!(
            matches!(result, PhysicsCommandResult::Sweep(Some(ShapeHit { collider: id, .. })) if id == collider)
        );

        let result = apply_command(
            &mut world,
            PhysicsCommand::Overlaps {
                shape: Shape::Sphere(Sphere::new(1.0).unwrap()),
                transform: Transform::new(Vec3::new(3.0, 0.0, 0.0), kapitsa_math::Quat::IDENTITY)
                    .unwrap(),
                filter,
            },
        )
        .unwrap();
        assert_eq!(
            result,
            PhysicsCommandResult::Overlaps(vec![OverlapHit { body, collider }])
        );
    }

    #[test]
    fn invalid_ray_is_rejected_before_it_can_be_submitted() {
        assert_eq!(
            Ray::new(Vec3::ZERO, Vec3::ZERO, 1.0),
            Err(QueryError::InvalidInput)
        );
    }

    #[test]
    fn indexed_batch_keeps_order_for_repeated_body_and_source_ids() {
        const UNIQUE_IDS: usize = SMALL_STAGING_LINEAR_SEARCH_LIMIT + 1;

        let mut world = PhysicsWorld::new();
        let bodies: Vec<_> = (0..UNIQUE_IDS)
            .map(|_| world.add_body(dynamic_body()).expect("body"))
            .collect();
        let sources: Vec<_> = (0..UNIQUE_IDS)
            .map(|_| {
                world
                    .add_force_source(bodies[0], Vec3::ZERO, Vec3::X)
                    .expect("force source")
            })
            .collect();
        let mut commands = Vec::with_capacity(UNIQUE_IDS * 2 + 3);
        for (index, id) in bodies.iter().copied().enumerate() {
            commands.push(PhysicsCommand::SetLinearVelocity {
                id,
                velocity: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
            });
        }
        for (index, id) in sources.iter().copied().enumerate() {
            commands.push(PhysicsCommand::SetForceSource {
                id,
                local_point: Vec3::ZERO,
                local_force: Vec3::new(index as f64 + 1.0, 0.0, 0.0),
            });
        }
        commands.push(PhysicsCommand::SetLinearVelocity {
            id: bodies[0],
            velocity: -Vec3::X,
        });
        commands.push(PhysicsCommand::SetForceSource {
            id: sources[0],
            local_point: Vec3::Y,
            local_force: Vec3::Z,
        });
        commands.push(PhysicsCommand::SetForceSourceEnabled {
            id: sources[0],
            enabled: false,
        });

        apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect("indexed batch applies");

        assert_eq!(
            world
                .body(bodies[0])
                .expect("first body")
                .state()
                .linear_velocity(),
            -Vec3::X
        );
        assert_eq!(
            world
                .body(bodies[1])
                .expect("second body")
                .state()
                .linear_velocity(),
            Vec3::new(2.0, 0.0, 0.0)
        );
        let first_source = world.force_source(sources[0]).expect("first source");
        assert_eq!(first_source.local_point(), Vec3::Y);
        assert_eq!(first_source.local_force(), Vec3::Z);
        assert!(!first_source.is_enabled());
        assert_eq!(
            world
                .force_source(sources[1])
                .expect("second source")
                .local_force(),
            Vec3::new(2.0, 0.0, 0.0)
        );
    }

    #[test]
    fn body_batch_preserves_order_and_applies_impulse_with_updated_mass() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let commands = [
            PhysicsCommand::SetMass {
                id: body,
                mass: mass(2.0),
            },
            PhysicsCommand::ApplyImpulse {
                id: body,
                impulse: Vec3::new(10.0, 0.0, 0.0),
            },
        ];

        apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect("atomic batch");

        assert_eq!(
            world.body(body).expect("body").state().linear_velocity(),
            Vec3::new(5.0, 0.0, 0.0)
        );
    }

    #[test]
    fn failed_body_batch_does_not_commit_earlier_body_changes() {
        let mut world = PhysicsWorld::new();
        let dynamic = world.add_body(dynamic_body()).expect("dynamic body");
        let static_body = world
            .add_body(RigidBody::new_static(Transform::identity()))
            .expect("static body");
        let commands = [
            PhysicsCommand::SetLinearVelocity {
                id: dynamic,
                velocity: Vec3::X,
            },
            PhysicsCommand::SetMass {
                id: static_body,
                mass: mass(2.0),
            },
        ];

        let mut scratch = BatchScratch::default();
        let error = apply_command_batch(&mut world, &commands, &mut scratch)
            .expect_err("static mass rejected");

        assert_eq!(error.command_index, Some(1));
        assert_eq!(
            error.error,
            PhysicsClientError::Body(BodyError::BodyMustBeDynamic)
        );
        assert_eq!(
            world
                .body(dynamic)
                .expect("dynamic body")
                .state()
                .linear_velocity(),
            Vec3::ZERO
        );
        assert!(scratch.updated_bodies.is_empty());
        assert!(scratch.body_indices.is_empty());
        let retained_body_capacity = scratch.updated_bodies.capacity();
        let retained_index_capacity = scratch.body_indices.capacity();

        apply_command_batch(
            &mut world,
            &[PhysicsCommand::SetLinearVelocity {
                id: dynamic,
                velocity: Vec3::Y,
            }],
            &mut scratch,
        )
        .expect("scratch can be reused after a failed batch");
        assert_eq!(scratch.updated_bodies.capacity(), retained_body_capacity);
        assert_eq!(scratch.body_indices.capacity(), retained_index_capacity);
        assert_eq!(
            world
                .body(dynamic)
                .expect("updated body")
                .state()
                .linear_velocity(),
            Vec3::Y
        );
    }

    #[test]
    fn force_source_updates_share_atomic_body_batch_commit() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let source = world
            .add_force_source(body, Vec3::ZERO, Vec3::X)
            .expect("force source");
        let commands = [
            PhysicsCommand::SetForceSource {
                id: source,
                local_point: Vec3::Y,
                local_force: Vec3::Z,
            },
            PhysicsCommand::SetForceSourceEnabled {
                id: source,
                enabled: false,
            },
            PhysicsCommand::SetLinearVelocity {
                id: body,
                velocity: Vec3::X,
            },
        ];

        apply_command_batch(&mut world, &commands, &mut BatchScratch::default())
            .expect("atomic batch");

        let source = world.force_source(source).expect("updated source");
        assert_eq!(source.local_point(), Vec3::Y);
        assert_eq!(source.local_force(), Vec3::Z);
        assert!(!source.is_enabled());
        assert_eq!(
            world.body(body).expect("body").state().linear_velocity(),
            Vec3::X
        );
    }

    #[test]
    fn failed_force_source_batch_does_not_commit_source_or_body_changes() {
        let mut world = PhysicsWorld::new();
        let body = world.add_body(dynamic_body()).expect("body");
        let source = world
            .add_force_source(body, Vec3::ZERO, Vec3::X)
            .expect("force source");
        let mut other_world = PhysicsWorld::new();
        let other_body = other_world.add_body(dynamic_body()).expect("other body");
        let foreign_source = other_world
            .add_force_source(other_body, Vec3::ZERO, Vec3::Y)
            .expect("foreign source");
        let commands = [
            PhysicsCommand::SetForceSource {
                id: source,
                local_point: Vec3::Y,
                local_force: Vec3::Z,
            },
            PhysicsCommand::SetLinearVelocity {
                id: body,
                velocity: Vec3::X,
            },
            PhysicsCommand::SetForceSourceEnabled {
                id: foreign_source,
                enabled: false,
            },
        ];

        let mut scratch = BatchScratch::default();
        let error =
            apply_command_batch(&mut world, &commands, &mut scratch).expect_err("foreign source");

        assert_eq!(error.command_index, Some(2));
        assert_eq!(
            error.error,
            PhysicsClientError::ForceSource(ForceSourceError::InvalidForceSourceId)
        );
        let source = world.force_source(source).expect("unchanged source");
        assert_eq!(source.local_point(), Vec3::ZERO);
        assert_eq!(source.local_force(), Vec3::X);
        assert!(source.is_enabled());
        assert_eq!(
            world.body(body).expect("body").state().linear_velocity(),
            Vec3::ZERO
        );
        assert!(scratch.updated_bodies.is_empty());
        assert!(scratch.body_indices.is_empty());
        assert!(scratch.updated_force_sources.is_empty());
        assert!(scratch.force_source_indices.is_empty());
    }

    #[test]
    #[ignore = "Release benchmark; run with the command batch scaling filter"]
    fn benchmark_apply_command_batch_scaling() {
        const COMMAND_COUNTS: [usize; 7] = [128, 256, 512, 1_024, 2_048, 4_096, 8_192];
        const TARGET_SAMPLE_TIME: Duration = Duration::from_millis(5);

        println!(
            "scenario,commands,repeats,min_us_per_batch,median_us_per_batch,max_us_per_batch,median_ns_per_command,one_shot_buffer_control_us,one_shot_control_share_pct"
        );
        for scenario in BatchScenario::ALL {
            for command_count in COMMAND_COUNTS {
                let repeats =
                    calibrated_repeats(scenario, command_count, TARGET_SAMPLE_TIME, apply_batch);
                let (min_ns, median_ns, max_ns) = run_samples(
                    scenario,
                    command_count,
                    repeats,
                    apply_batch,
                    verify_batch_result,
                );
                let temp_buffer_repeats =
                    calibrated_repeats(scenario, command_count, TARGET_SAMPLE_TIME, |case| {
                        staged_buffer_control(case)
                    });
                let (_, temp_median_ns, _) = run_samples(
                    scenario,
                    command_count,
                    temp_buffer_repeats,
                    |case| staged_buffer_control(case),
                    |_| {},
                );

                println!(
                    "{},{command_count},{repeats},{:.3},{:.3},{:.3},{:.2},{:.3},{:.1}",
                    scenario.name(),
                    min_ns / 1_000.0,
                    median_ns / 1_000.0,
                    max_ns / 1_000.0,
                    median_ns / command_count as f64,
                    temp_median_ns / 1_000.0,
                    temp_median_ns * 100.0 / median_ns,
                );
            }
        }
    }

    #[cfg(feature = "bench-internals")]
    #[test]
    #[ignore = "allocation counts; run separately with bench-internals"]
    fn benchmark_apply_command_batch_allocations() {
        const COMMAND_COUNTS: [usize; 7] = [128, 256, 512, 1_024, 2_048, 4_096, 8_192];

        println!(
            "scenario,commands,warm_allocations,warm_requested_bytes,body_vec_capacity,body_index_capacity,source_vec_capacity,source_index_capacity"
        );
        for scenario in BatchScenario::ALL {
            for command_count in COMMAND_COUNTS {
                let mut case = batch_case(scenario, command_count);
                apply_batch(&mut case);
                let allocations =
                    tracked_batch_allocations(&mut case.world, &case.commands, &mut case.scratch);
                verify_batch_result(&case);
                println!(
                    "{},{command_count},{},{},{},{},{},{}",
                    scenario.name(),
                    allocations.count,
                    allocations.bytes,
                    case.scratch.updated_bodies.capacity(),
                    case.scratch.body_indices.capacity(),
                    case.scratch.updated_force_sources.capacity(),
                    case.scratch.force_source_indices.capacity(),
                );
            }
        }
    }
}
