use std::error::Error;

use kapitsa::math::{Mat3, Quat, Vec3};
use kapitsa::query::{QueryFilter, Ray};
use kapitsa::shape::TriangleMesh;
use kapitsa::{
    Chunk, ChunkBuilder, ChunkClientError, MassProperties, PhysicsClientError, RigidBody,
    Simulation, Transform, WorldError,
};

fn chunk(x: f64, meshes: usize) -> Result<Chunk, Box<dyn Error>> {
    let mut builder = ChunkBuilder::new();
    for _ in 0..meshes {
        builder.add_mesh(TriangleMesh::new(
            vec![
                Vec3::new(-2.0, 0.0, -2.0),
                Vec3::new(-2.0, 0.0, 2.0),
                Vec3::new(2.0, 0.0, -2.0),
            ],
            vec![[0, 1, 2]],
        )?)?;
    }
    Ok(builder.build(Transform::new(Vec3::X * x, Quat::IDENTITY)?)?)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn paused_client_registers_queries_unloads_and_reuses_chunks() -> Result<(), Box<dyn Error>> {
    let mut simulation = Simulation::builder()
        .frequency(120)
        .max_workers(1)
        .spawn_on_tokio(&tokio::runtime::Handle::current())?;
    simulation.pause()?;
    let client = simulation.client();
    let body = client
        .add_body(RigidBody::new_static(Transform::identity()))
        .await?;
    let input = vec![chunk(0.0, 1)?, chunk(10.0, 2)?];
    #[cfg(feature = "bench-internals")]
    let original: Vec<_> = input
        .iter()
        .flat_map(|chunk| {
            chunk
                .colliders()
                .iter()
                .map(|collider| collider.mesh_bvh_serial())
        })
        .collect();
    let ids = client.add_chunk_batch(body, input).await?;
    let first = *ids.first().ok_or("missing first chunk")?;
    assert_eq!(client.chunk_count().await?, 2);
    assert_eq!(client.chunk_body(first).await?, Some(body));
    let first_colliders = client
        .chunk_colliders(first)
        .await?
        .ok_or("missing chunk")?;
    assert_eq!(first_colliders.len(), 1);
    let collider = *first_colliders.first().ok_or("missing collider")?;
    assert!(matches!(
        client.remove_collider(collider).await,
        Err(PhysicsClientError::World(WorldError::InvalidColliderId))
    ));
    let ray = Ray::new(Vec3::new(-1.0, 3.0, -1.0), -Vec3::Y, 6.0)?;
    assert!(client.raycast(ray, QueryFilter::default()).await?.is_some());
    let removed = client.remove_chunk_batch(ids.clone()).await?;
    #[cfg(feature = "bench-internals")]
    {
        let returned: Vec<_> = removed
            .iter()
            .flat_map(|chunk| {
                chunk
                    .colliders()
                    .iter()
                    .map(|collider| collider.mesh_bvh_serial())
            })
            .collect();
        assert_eq!(returned, original);
        assert!(returned.iter().all(Option::is_some));
    }
    assert_eq!(
        removed.first().map(|chunk| chunk.colliders().len()),
        Some(1)
    );
    assert_eq!(removed.last().map(|chunk| chunk.colliders().len()), Some(2));
    assert!(client.raycast(ray, QueryFilter::default()).await?.is_none());
    assert_eq!(client.chunk_body(first).await?, None);
    let replacements = client.add_chunk_batch(body, removed).await?;
    assert_eq!(client.chunk_count().await?, 2);
    let replacement = *replacements.first().ok_or("missing replacement")?;
    assert_ne!(replacement, first);
    assert!(client.remove_chunk(first).await?.is_none());
    let duplicate = client
        .remove_chunk_batch(vec![replacement, replacement])
        .await;
    match duplicate {
        Err(ChunkClientError::Rejected {
            cause: kapitsa::ChunkOperationError::DuplicateChunkId,
            input,
        }) => assert_eq!(input, vec![replacement, replacement]),
        _ => return Err("duplicate removal was not rejected".into()),
    }
    assert_eq!(client.chunk_count().await?, 2);
    assert_eq!(client.remove_chunk_batch(replacements).await?.len(), 2);
    assert!(client.add_chunk_batch(body, Vec::new()).await?.is_empty());
    assert!(client.remove_chunk_batch(Vec::new()).await?.is_empty());
    simulation.stop().await?;
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn rejection_and_stopped_service_return_prepared_chunks() -> Result<(), Box<dyn Error>> {
    let mut simulation = Simulation::builder()
        .frequency(120)
        .max_workers(1)
        .spawn_on_tokio(&tokio::runtime::Handle::current())?;
    simulation.pause()?;
    let client = simulation.client();
    let mass = MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?;
    let dynamic = client
        .add_body(RigidBody::new_dynamic(Transform::identity(), mass)?)
        .await?;
    let input = vec![chunk(0.0, 1)?, chunk(20.0, 2)?];
    #[cfg(feature = "bench-internals")]
    let serials: Vec<_> = input
        .iter()
        .flat_map(|chunk| {
            chunk
                .colliders()
                .iter()
                .map(|collider| collider.mesh_bvh_serial())
        })
        .collect();
    let recovered = match client.add_chunk_batch(dynamic, input).await {
        Err(ChunkClientError::Rejected {
            cause:
                kapitsa::ChunkOperationError::World(WorldError::Collision(
                    kapitsa::CollisionError::UnsupportedDynamicGeometry,
                )),
            input,
        }) => input,
        _ => return Err("dynamic mesh batch was not rejected".into()),
    };
    assert_eq!(recovered.len(), 2);
    #[cfg(feature = "bench-internals")]
    assert_eq!(
        serials,
        recovered
            .iter()
            .flat_map(|chunk| chunk
                .colliders()
                .iter()
                .map(|collider| collider.mesh_bvh_serial()))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        recovered.last().map(|chunk| chunk.colliders().len()),
        Some(2)
    );
    assert_eq!(client.chunk_count().await?, 0);
    simulation.stop().await?;
    let recovered = match client.add_chunk_batch(dynamic, recovered).await {
        Err(ChunkClientError::NotSubmitted {
            cause: PhysicsClientError::NotRunning,
            input,
        }) => input,
        _ => return Err("stopped service did not return the batch".into()),
    };
    assert_eq!(recovered.len(), 2);
    #[cfg(feature = "bench-internals")]
    assert_eq!(
        serials,
        recovered
            .iter()
            .flat_map(|chunk| chunk
                .colliders()
                .iter()
                .map(|collider| collider.mesh_bvh_serial()))
            .collect::<Vec<_>>()
    );
    assert_eq!(
        recovered.first().map(|chunk| chunk.colliders().len()),
        Some(1)
    );
    Ok(())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn clients_reject_foreign_ids_without_partial_changes() -> Result<(), Box<dyn Error>> {
    let mut first = Simulation::builder()
        .max_workers(1)
        .spawn_on_tokio(&tokio::runtime::Handle::current())?;
    let mut second = Simulation::builder()
        .max_workers(1)
        .spawn_on_tokio(&tokio::runtime::Handle::current())?;
    first.pause()?;
    second.pause()?;
    let client = first.client();
    let other = second.client();
    let body = client
        .add_body(RigidBody::new_static(Transform::identity()))
        .await?;
    let other_body = other
        .add_body(RigidBody::new_static(Transform::identity()))
        .await?;
    let id = client.add_chunk(body, chunk(0.0, 1)?).await?;
    let foreign = other.add_chunk(other_body, chunk(0.0, 1)?).await?;
    match client.add_chunk(other_body, chunk(10.0, 1)?).await {
        Err(ChunkClientError::Rejected {
            cause: kapitsa::ChunkOperationError::World(WorldError::InvalidBodyId),
            input,
        }) => assert_eq!(input.colliders().len(), 1),
        _ => return Err("foreign body was not rejected with its chunk".into()),
    }
    match client.remove_chunk_batch(vec![id, foreign]).await {
        Err(ChunkClientError::Rejected {
            cause: kapitsa::ChunkOperationError::InvalidChunkId,
            input,
        }) => assert_eq!(input, vec![id, foreign]),
        _ => return Err("foreign chunk batch was not rejected".into()),
    }
    assert_eq!(client.chunk_count().await?, 1);
    assert_eq!(client.chunk_body(id).await?, Some(body));
    assert_eq!(other.chunk_body(foreign).await?, Some(other_body));
    first.stop().await?;
    second.stop().await?;
    Ok(())
}
