use super::{PhysicsClient, PhysicsClientError, PhysicsCommand, PhysicsCommandResult, expect_done};
use kapitsa_core::Error;
use kapitsa_core::material::{MaterialId, PhysicsMaterial};

impl PhysicsClient {
    /// Adds a material and returns its ID.
    pub async fn add_material(&self, material: PhysicsMaterial) -> Result<MaterialId, Error> {
        match self
            .submit(PhysicsCommand::AddMaterial { material })
            .await?
        {
            PhysicsCommandResult::MaterialId(id) => Ok(id),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
    /// Replaces a material and invalidates affected cached contacts.
    pub async fn update_material(
        &self,
        id: MaterialId,
        material: PhysicsMaterial,
    ) -> Result<(), Error> {
        expect_done(
            self.submit(PhysicsCommand::UpdateMaterial { id, material })
                .await?,
        )
    }
    /// Removes an unassigned material.
    pub async fn remove_material(&self, id: MaterialId) -> Result<(), Error> {
        expect_done(self.submit(PhysicsCommand::RemoveMaterial { id }).await?)
    }
    /// Reads a material at the request's position in the command queue.
    pub async fn material(&self, id: MaterialId) -> Result<Option<PhysicsMaterial>, Error> {
        match self.submit(PhysicsCommand::GetMaterial { id }).await? {
            PhysicsCommandResult::Material(material) => Ok(material),
            _ => Err(Error::from(PhysicsClientError::UnexpectedResult)),
        }
    }
}
