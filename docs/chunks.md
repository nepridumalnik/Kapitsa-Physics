# Prepared mesh chunks

`kapitsa_core::chunk::{Chunk, ChunkBuilder}` prepares a nonempty group of triangle
meshes. `build(transform)` is synchronous and does not access a world; ownership
can be transferred to a worker thread. Vertices and `local_bounds()` are in
chunk-local coordinates. The shared transform is relative to the owning body.
An empty builder returns `ChunkBuildError::Empty`.

`PhysicsWorld` supports:

```rust,ignore
add_chunk(body: BodyId, chunk: Chunk) -> Result<ChunkId, AddChunkError>
remove_chunk(id: ChunkId) -> Option<Chunk>
add_chunk_batch(body: BodyId, chunks: Vec<Chunk>) -> Result<Vec<ChunkId>, AddChunkBatchError>
remove_chunk_batch(ids: Vec<ChunkId>) -> Result<Vec<Chunk>, RemoveChunkBatchError>
```

Each batch is atomic with respect to registered geometry. Validation and fallible
reservations happen before publishing changes. Addition errors retain all input
chunks in input order (`error.chunk` / `error.chunks`). Removal rejects foreign,
stale and duplicate IDs before removing anything. Output order follows input
order. Empty batches are no-ops. Capacity reservation may grow internal buffers
on failure; atomicity does not promise byte-for-byte storage identity or handle
process aborts/panics. No step can interleave with these exclusive synchronous
operations. There is no transaction spanning separate add and remove calls.

Removed chunks retain transforms, mesh order, material/filter settings and BVHs.
Re-registering them does not rebuild geometry. `ChunkId` is scoped to its world
and uses slot generations. `chunk_body`, `chunk_colliders` and `chunk_count`
provide read-only registration information. Chunks use ordinary world colliders,
contact invalidation, queries and CCD; there is no separate collision system.

Triangle meshes require a static or kinematic body. Cluster bodies cannot accept
chunks, and bodies with chunks cannot be converted into clusters. Chunk-owned
colliders cannot be individually removed or borrowed through `collider_mut`.
Material/mode changes through the checked world APIs remain available. Removing
a body destroys its chunks and invalidates their IDs, just as it destroys its
ordinary colliders. Use explicit chunk removal first to retain geometry.

Chunk identity in a terrain database, geometry generation, job cancellation,
readiness, caching and LOD selection belong to the caller. Apply complete batches
between physical steps. If separate add/remove calls are used for a transition,
the caller must keep `step` and queries out of the intermediate interval and
handle failure of each operation explicitly.
