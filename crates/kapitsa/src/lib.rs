mod runtime;

#[cfg(test)]
extern crate self as kapitsa;

pub mod execution {
    pub use crate::runtime::{RayonExecutor, RayonPool, RayonPoolError};
    pub use kapitsa_core::execution::{ExecutionError, SequentialExecutor, StepExecutor};
}

pub mod aerodynamics {
    pub use kapitsa_core::aerodynamics::{
        AerodynamicForce, AerodynamicProperties, AerodynamicSurface, AerodynamicTarget,
        AerodynamicsError, AerodynamicsTask, AtmosphericState,
    };
}

pub mod hydrodynamics {
    pub use kapitsa_core::hydrodynamics::HydrodynamicsTask;
}

pub mod aabb {
    pub use kapitsa_core::aabb::{Aabb, AabbError};
}

pub mod collider {
    pub use kapitsa_core::collider::{Collider, ColliderError, CollisionFilter, CollisionMode};
}

pub mod chunk {
    pub use kapitsa_core::chunk::{Chunk, ChunkBuildError, ChunkBuilder};
}

pub mod collision {
    pub use kapitsa_core::collision::{
        CcdLimitPolicy, CollisionConfig, CollisionError, CollisionEvent, CollisionEventKind,
        CollisionPair, ContactInfo,
    };
}

pub mod cluster {
    pub use kapitsa_core::cluster::*;
}

pub mod gravity {
    pub use kapitsa_core::gravity::{
        CentralGravity, Gravity, GravityError, SphericalGravity, UniformGravity,
    };
}

pub mod joint {
    pub use kapitsa_core::joint::*;
}

pub mod mass {
    pub use kapitsa_core::mass::{MassError, MassProperties};
}

pub mod material {
    pub use kapitsa_core::material::{MaterialId, PhysicsMaterial, PhysicsMaterialError};
}

pub mod math {
    pub use kapitsa_math::{Affine3, Mat3, Quat, Vec2, Vec3};
}

pub mod query {
    pub use kapitsa_core::collision::{OverlapHit, QueryError, QueryFilter, Ray, RayHit, ShapeHit};
}

pub mod rigid_body {
    pub use kapitsa_core::rigid_body::{
        BodyError, BodyForces, BodyPhysics, BodyState, BodyType, CcdMode, IntegrationError,
        RigidBody,
    };
}

pub mod shape {
    pub use kapitsa_core::shape::*;
}

pub mod transform {
    pub use kapitsa_core::transform::{Transform, TransformError};
}

pub mod world {
    pub use kapitsa_core::world::{
        AddChunkBatchError, AddChunkError, ChunkId, ChunkOperationError, RemoveChunkBatchError,
    };
    pub use kapitsa_core::world::{
        BodyId, ClusterError, ColliderId, ForceSource, ForceSourceError, ForceSourceId, GravityId,
        JointId, JointTarget, PartId, StepError, WeldId, WorldError,
    };
}

pub use kapitsa_core::{
    chunk::{Chunk, ChunkBuildError, ChunkBuilder},
    collider::{Collider, CollisionFilter, CollisionMode},
    collision::{
        CcdLimitPolicy, CollisionConfig, CollisionError, CollisionEvent, CollisionEventKind,
        CollisionPair, ContactInfo,
    },
    gravity::{CentralGravity, Gravity, GravityError, SphericalGravity, UniformGravity},
    joint::{Joint, JointBreakEvent},
    mass::MassProperties,
    material::{MaterialId, PhysicsMaterial},
    rigid_body::{BodyState, CcdMode, RigidBody},
    shape::Shape,
    transform::Transform,
    world::{
        AddChunkBatchError, AddChunkError, BodyId, ChunkId, ChunkOperationError, ClusterError,
        ColliderId, ForceSourceId, GravityId, JointId, JointTarget, PartId, RemoveChunkBatchError,
        StepError, WeldId, WorldError,
    },
};

pub use runtime::{
    AdaptiveTimeStepConfig, BatchId, ChunkClientError, CollisionEventBatch,
    CollisionEventStreamError, CollisionEvents, ConfigError, JointBreakEventBatch,
    JointBreakEventStreamError, JointBreakEvents, PhysicsBatchError, PhysicsBatchResult,
    PhysicsBatchResultStreamError, PhysicsBatchResults, PhysicsClient, PhysicsClientError,
    PhysicsCommand, PhysicsCommandBuffer, PhysicsCommandResult, PhysicsSnapshot,
    PhysicsSubmitError, SchedulerConfig, Simulation, SimulationBuilder, SimulationConfig,
    SimulationError, SimulationMetrics, SimulationState, StepMeasurement, StepStats,
    TimeStepPolicy, WorkerConfig, WorkerConfigError,
};
