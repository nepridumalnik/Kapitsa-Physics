# Prepared mesh chunks

`kapitsa::chunk::{Chunk, ChunkBuilder}` prepares a nonempty group of triangle
meshes. `build(transform)` is synchronous and does not access a world; ownership
can be transferred to a worker thread. Vertices and `local_bounds()` are in
chunk-local coordinates. The shared transform is relative to the owning body.
An empty builder returns `ChunkBuildError::Empty`.

Use `PhysicsClient` to control registered chunks in a simulation:

```rust,ignore
async fn add_chunk(&self, body: BodyId, chunk: Chunk)
    -> Result<ChunkId, ChunkClientError<Chunk, ChunkId>>;
async fn remove_chunk(&self, id: ChunkId)
    -> Result<Option<Chunk>, ChunkClientError<ChunkId, Option<Chunk>>>;
async fn add_chunk_batch(&self, body: BodyId, chunks: Vec<Chunk>)
    -> Result<Vec<ChunkId>, ChunkClientError<Vec<Chunk>, Vec<ChunkId>>>;
async fn remove_chunk_batch(&self, ids: Vec<ChunkId>)
    -> Result<Vec<Chunk>, ChunkClientError<Vec<ChunkId>, Vec<Chunk>>>;
```

Each batch is atomic with respect to registered geometry. Validation and fallible
reservations happen before publishing changes. Rejected additions return every
input chunk in order. Removal rejects foreign, stale and duplicate IDs before
removing anything. Output order follows input order. Empty batches are no-ops
when the service is running; like other client requests, they require a live service.
Reservation may grow internal buffers on failure; atomicity does not promise
byte-for-byte storage identity or handle process aborts or panics.

Each call enters the bounded FIFO command queue as a single operation, so no
step can interleave within a batch. Separate client calls are separate operations
and a step may occur between them; pause the simulation if the application must
control that interval. There is no transaction spanning calls.

`ChunkClientError::NotSubmitted { cause, input }` returns the input when the
service rejects a queued operation during shutdown or the queue is unavailable.
`Rejected { cause, input }` returns it when world validation or reservation
fails. `StepFailedAfterApply { step, cause, output }` returns the operation's
actual output: the mutation happened and was not rolled back, even though the
next physics step failed. `ResponseLost` means the response channel closed after
enqueueing and the operation's outcome is unknown; do not blindly retry it.

As with other client methods, cancelling a future after enqueue does not cancel
the operation. The returned ID or removed chunk may then be dropped with the
unreceived response. Cancelling earlier also drops the input held by the future.
Keep and await the operation future when the result must be retained. Query
results describe their FIFO position before integration and remain valid if the
subsequent step fails. Successful mutations are acknowledged after snapshot
publication, and work while the simulation is paused.

Removed chunks retain transforms, mesh order, material/filter settings and BVHs.
Re-registering them does not rebuild geometry. `ChunkId` is scoped to its world
and uses slot generations. `client.chunk_body`, `client.chunk_colliders` and
`client.chunk_count` provide read-only registration information. The body
snapshot does not contain chunk IDs. Chunks use ordinary world colliders,
contact invalidation, queries and CCD.

Triangle meshes require a static or kinematic body. Cluster bodies cannot accept
chunks, and bodies with chunks cannot be converted into clusters. Chunk-owned
colliders cannot be individually removed through the client. Material and mode
can be changed with the checked client methods. Removing a body destroys its
chunks and invalidates their IDs, just as it destroys its ordinary colliders.
Use explicit chunk removal first to retain geometry.

Chunk identity in a terrain database, geometry generation, job cancellation,
readiness, caching and LOD selection belong to the caller. The synchronous
`kapitsa-core::world::PhysicsWorld` operations remain available to low-level
engine integrations; application simulation commands use the client API.
