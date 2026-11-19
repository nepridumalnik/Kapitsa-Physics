mod runtime;
mod unified_error;

#[cfg(test)]
extern crate self as kapitsa;

pub mod execution {
    #[cfg(feature = "parallel")]
    pub use crate::runtime::RayonExecutor;
    #[cfg(feature = "service")]
    pub use crate::runtime::{RayonPool, RayonPoolError};
    pub use kapitsa_core::execution::{ExecutionError, SequentialExecutor, StepExecutor};
}

pub mod aerodynamics {
    pub use kapitsa_core::aerodynamics::{
        AerodynamicForce, AerodynamicProperties, AerodynamicSurface, AerodynamicTarget,
        Aerodynamics, AerodynamicsError, AerodynamicsTask, Atmosphere, AtmosphericState,
        ColliderTessellation, DensityProfile, SphericalAtmosphere, SphericalAtmosphereBuilder,
        cache, colliders, geometry, hull, pressure, slender, wave, wing,
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
        AddChunkBatchError, AddChunkError, ChunkId, ChunkOperationError, PhysicsWorld,
        RemoveChunkBatchError, StepResults,
    };
    pub use kapitsa_core::world::{
        BodyId, ClusterError, ColliderId, ErrorPolicy, ForceSource, ForceSourceError,
        ForceSourceId, GravityId, JointId, JointTarget, PartId, StepDiagnostics, StepError, WeldId,
        WorldError,
    };
}

pub use kapitsa_core::{
    Error, ErrorKind,
    chunk::{Chunk, ChunkBuildError, ChunkBuilder},
    collider::{Collider, CollisionFilter, CollisionMode},
    collision::{
        CcdLimitPolicy, CollisionConfig, CollisionEvent, CollisionEventKind, CollisionPair,
        ContactInfo,
    },
    command::{
        CommandBatchError, CommandError, PhysicsCommand, PhysicsCommandBuffer, PhysicsCommandResult,
    },
    gravity::{CentralGravity, Gravity, SphericalGravity, UniformGravity},
    joint::{Joint, JointBreakEvent},
    mass::MassProperties,
    material::{MaterialId, PhysicsMaterial},
    rigid_body::{BodyMotion, BodyState, CcdMode, RigidBody},
    shape::Shape,
    transform::Transform,
    world::{
        AddChunkBatchError, AddChunkError, BodyId, ChunkId, ChunkOperationError, ColliderId,
        ErrorPolicy, ForceSourceId, GravityId, JointId, JointTarget, PartId, PhysicsWorld,
        RemoveChunkBatchError, StepDiagnostics, StepResults, WeldId,
    },
};

pub use runtime::{
    AdaptiveTimeStepConfig, ConfigError, SchedulerConfig, SimulationConfig, SimulationMetrics,
    SimulationState, StepMeasurement, StepStats, TimeStepPolicy, WorkerConfig, WorkerConfigError,
};

#[cfg(feature = "service")]
pub use runtime::{
    BatchId, ChunkClientError, CollisionEventBatch, CollisionEvents, JointBreakEventBatch,
    JointBreakEvents, PhysicsBatchResult, PhysicsBatchResults, PhysicsClient, PhysicsSnapshot,
    PhysicsSubmitError, Simulation, SimulationBuilder,
};

/// Common imports for typical simulation code.
///
/// ```
/// use kapitsa::prelude::*;
///
/// let mut world = PhysicsWorld::new();
/// let body = world
///     .add_body(
///         RigidBody::new_dynamic(
///             Transform::identity(),
///             MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY).expect("valid mass"),
///         )
///         .expect("valid body"),
///     )
///     .expect("valid body");
/// assert!(world.body(body).is_some());
/// ```
pub mod prelude {
    pub use crate::{
        Error, ErrorKind,
        mass::MassProperties,
        material::PhysicsMaterial,
        math::{Affine3, Mat3, Quat, Vec2, Vec3},
        rigid_body::{BodyState, BodyType, RigidBody},
        shape::{Capsule, Cuboid, Cylinder, Shape, Sphere, TriangleMesh},
        transform::Transform,
        world::{BodyId, ColliderId, PartId, PhysicsWorld},
    };

    #[cfg(feature = "service")]
    pub use crate::{
        PhysicsClient, PhysicsSnapshot, Simulation, SimulationBuilder, SimulationConfig,
    };
}
