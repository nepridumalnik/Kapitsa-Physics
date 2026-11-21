use super::{PhysicsClient, PhysicsClientError, PhysicsCommand, PhysicsCommandResult, expect_done};
use kapitsa_core::{
    Error,
    cluster::{
        ClusterPart, ClusterPartDesc, MassChangeMode, MassTransfer, RigidCluster, Weld, WeldDesc,
    },
    collider::Collider,
    joint::Joint,
    mass::MassProperties,
    world::{BodyId, ColliderId, ForceSourceId, JointId, JointTarget, PartId, WeldId},
};
use kapitsa_math::Vec3;

impl PhysicsClient {
    /// Adds a joint between body or cluster-part targets.
    pub async fn add_joint_targets(
        &self,
        target_a: JointTarget,
        target_b: JointTarget,
        joint: Joint,
    ) -> Result<JointId, Error> {
        match self
            .submit(PhysicsCommand::AddJointTargets {
                target_a,
                target_b,
                joint,
            })
            .await?
        {
            PhysicsCommandResult::Joint(id) => Ok(id),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Updates a joint using its original target-local frames.
    pub async fn update_joint_targets(&self, id: JointId, joint: Joint) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::UpdateJointTargets { id, joint })
                .await?,
        )
    }
    /// Adds a persistent force in part-local coordinates.
    pub async fn add_part_force_source(
        &self,
        part: PartId,
        local_point: Vec3,
        local_force: Vec3,
    ) -> Result<ForceSourceId, Error> {
        match self
            .submit(PhysicsCommand::AddPartForceSource {
                part,
                local_point,
                local_force,
            })
            .await?
        {
            PhysicsCommandResult::ForceSource(id) => Ok(id),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Converts a dynamic body into a cluster and returns its root part.
    pub async fn enable_cluster(&self, body: BodyId) -> Result<PartId, Error> {
        match self.submit(PhysicsCommand::EnableCluster { body }).await? {
            PhysicsCommandResult::Part(id) => Ok(id),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Converts a body into a cluster using explicit root mass.
    pub async fn enable_cluster_with_mass(
        &self,
        body: BodyId,
        mass: MassProperties,
    ) -> Result<PartId, Error> {
        match self
            .submit(PhysicsCommand::EnableClusterWithMass { body, mass })
            .await?
        {
            PhysicsCommandResult::Part(id) => Ok(id),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Adds a part connected to its parent by a weld.
    pub async fn add_cluster_part(
        &self,
        parent: PartId,
        desc: ClusterPartDesc,
        weld: WeldDesc,
    ) -> Result<PartId, Error> {
        match self
            .submit(PhysicsCommand::AddClusterPart { parent, desc, weld })
            .await?
        {
            PhysicsCommandResult::Part(id) => Ok(id),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Adds a collider using part-local coordinates.
    pub async fn add_part_collider(
        &self,
        part: PartId,
        collider: Collider,
    ) -> Result<ColliderId, Error> {
        match self
            .submit(PhysicsCommand::AddPartCollider { part, collider })
            .await?
        {
            PhysicsCommandResult::Collider(id) => Ok(id),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Queues background preparation of the body's current assembly geometry.
    pub async fn queue_body_shape(
        &self,
        body: BodyId,
        policy: kapitsa_core::aerodynamics::shape::worker::PendingAero,
        worker: &kapitsa_core::aerodynamics::shape::worker::ShapeWorker,
    ) -> Result<(), Error> {
        self.queue_body_shape_with_completion(body, policy, worker, None)
            .await
    }

    /// Queues a cook and sends its outcome when the world processes publication.
    /// While paused, call `poll_body_shape` to drive publication. Superseding
    /// or cancelling the request disconnects its completion channel.
    pub async fn queue_body_shape_with_completion(
        &self,
        body: BodyId,
        policy: kapitsa_core::aerodynamics::shape::worker::PendingAero,
        worker: &kapitsa_core::aerodynamics::shape::worker::ShapeWorker,
        completion: Option<kapitsa_core::world::BodyShapeCompletion>,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::QueueBodyShape {
                body,
                policy,
                worker: worker.clone(),
                completion,
            })
            .await?,
        )
    }

    /// Cancels a tracked background cook without replacing the installed binding.
    pub async fn cancel_body_shape(&self, body: BodyId) -> Result<bool, Error> {
        match self
            .submit(PhysicsCommand::CancelBodyShape { body })
            .await?
        {
            PhysicsCommandResult::BodyShapeCancelled(cancelled) => Ok(cancelled),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }

    /// Publishes available cook results and reads the latest status, even while paused.
    pub async fn poll_body_shape(
        &self,
        body: BodyId,
    ) -> Result<kapitsa_core::world::BodyShapeStatus, Error> {
        match self.submit(PhysicsCommand::PollBodyShape { body }).await? {
            PhysicsCommandResult::BodyShapeStatus(status) => Ok(status),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }

    /// Replaces explicit finite wings in the stable part-local frame; empty clears.
    /// Takes owned configuration and applies it in queue order, even while paused.
    /// Loads are additive to hull, geometry and other explicit surfaces; callers
    /// must avoid double counting. Ownership follows PartId through topology edits.
    /// This metadata edit is unsupported in atomic motion batches.
    pub async fn set_part_aerodynamic_wings(
        &self,
        part: PartId,
        wings: Vec<kapitsa_core::aerodynamics::wing::WingSurface>,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartAerodynamicWings { part, wings })
                .await?,
        )
    }
    /// Replaces explicit hull sections in the stable part-local frame; empty clears.
    /// Takes owned configuration (including shared immutable Mach data) and applies
    /// it in queue order, even while paused. Loads remain additive; no coefficients
    /// or interference are inferred. Ownership follows PartId through topology edits.
    /// This metadata edit is unsupported in atomic motion batches.
    pub async fn set_part_aerodynamic_hull(
        &self,
        part: PartId,
        sections: Vec<kapitsa_core::aerodynamics::hull::HullSection>,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartAerodynamicHull { part, sections })
                .await?,
        )
    }
    /// Replaces a part's Mesh in its local frame; None disables geometric loads.
    /// Explicit aerodynamic surfaces remain additive and must not duplicate it.
    pub async fn set_part_aerodynamic_mesh(
        &self,
        part: PartId,
        mesh: Option<kapitsa_core::shape::TriangleMesh>,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartAerodynamicMesh { part, mesh })
                .await?,
        )
    }
    /// Enables collider geometry in place of the part's Mesh configuration.
    pub async fn set_part_aerodynamic_colliders(
        &self,
        part: PartId,
        enabled: bool,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartAerodynamicColliders { part, enabled })
                .await?,
        )
    }
    /// Replaces the part-owned assembly wave model. The command is applied in
    /// queue order and is not supported in atomic motion batches.
    pub async fn set_part_voxel_wave(
        &self,
        part: PartId,
        model: Option<kapitsa_core::aerodynamics::wave::VoxelWave>,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartVoxelWave { part, model })
                .await?,
        )
    }

    /// Sets a part-local mesh used only as geometry for the assembly wave model.
    pub async fn set_part_wave_source_mesh(
        &self,
        part: PartId,
        mesh: Option<kapitsa_core::shape::TriangleMesh>,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartWaveSourceMesh { part, mesh })
                .await?,
        )
    }

    /// Selects part-owned colliders as geometry-only inputs to the wave model.
    pub async fn set_part_wave_source_colliders(
        &self,
        part: PartId,
        enabled: bool,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartWaveSourceColliders { part, enabled })
                .await?,
        )
    }

    /// Changes the collider transform in its owner's part-local frame (metres).
    pub async fn set_part_collider_transform(
        &self,
        part: PartId,
        collider: ColliderId,
        transform: kapitsa_core::transform::Transform,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartColliderTransform {
                part,
                collider,
                transform,
            })
            .await?,
        )
    }
    /// Sets the solver body's collider-derived aerodynamic resolution and budget.
    pub async fn set_collider_tessellation(
        &self,
        body: BodyId,
        config: kapitsa_core::aerodynamics::ColliderTessellation,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetColliderTessellation { body, config })
                .await?,
        )
    }

    /// Replaces the body's primary aerodynamic model.
    pub async fn set_aerodynamics(
        &self,
        body: BodyId,
        model: kapitsa_core::aerodynamics::Aerodynamics,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetAerodynamics { body, model })
                .await?,
        )
    }

    /// Removes a collider owned by a cluster part.
    pub async fn remove_part_collider(&self, id: ColliderId) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::RemovePartCollider { id })
                .await?,
        )
    }
    /// Destroys a part when the cluster topology permits it.
    pub async fn destroy_cluster_part(&self, part: PartId) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::DestroyClusterPart { part })
                .await?,
        )
    }
    /// Removes a weld and returns the resulting component body IDs.
    pub async fn break_weld(&self, weld: WeldId) -> Result<Vec<BodyId>, Error> {
        match self.submit(PhysicsCommand::BreakWeld { weld }).await? {
            PhysicsCommandResult::Bodies(ids) => Ok(ids),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Removes incident welds and returns the resulting component body IDs.
    pub async fn detach_part(&self, part: PartId) -> Result<Vec<BodyId>, Error> {
        match self.submit(PhysicsCommand::DetachPart { part }).await? {
            PhysicsCommandResult::Bodies(ids) => Ok(ids),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Merges the second cluster into the first.
    pub async fn merge_clusters(
        &self,
        first: BodyId,
        second: BodyId,
        weld: WeldDesc,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::MergeClusters {
                first,
                second,
                weld,
            })
            .await?,
        )
    }
    /// Changes a part mass with the specified mass-change policy.
    pub async fn set_part_mass(
        &self,
        part: PartId,
        mass: MassProperties,
        mode: MassChangeMode,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartMass { part, mass, mode })
                .await?,
        )
    }
    /// Changes a part mass with explicit momentum transfer.
    pub async fn set_part_mass_with_transfer(
        &self,
        part: PartId,
        mass: MassProperties,
        transfer: MassTransfer,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::SetPartMassWithTransfer {
                part,
                mass,
                transfer,
            })
            .await?,
        )
    }
    /// Reads the cluster at the request's position in the command queue.
    pub async fn cluster(&self, body: BodyId) -> Result<Option<RigidCluster>, Error> {
        match self.submit(PhysicsCommand::GetCluster { body }).await? {
            PhysicsCommandResult::Cluster(value) => Ok(value),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Reads a cluster part at the request's position in the command queue.
    pub async fn cluster_part(&self, part: PartId) -> Result<Option<ClusterPart>, Error> {
        match self.submit(PhysicsCommand::GetClusterPart { part }).await? {
            PhysicsCommandResult::ClusterPart(value) => Ok(value),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Reads a weld at the request's position in the command queue.
    pub async fn weld(&self, weld: WeldId) -> Result<Option<Weld>, Error> {
        match self.submit(PhysicsCommand::GetWeld { weld }).await? {
            PhysicsCommandResult::Weld(value) => Ok(value),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
}
