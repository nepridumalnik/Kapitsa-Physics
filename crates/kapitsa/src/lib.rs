//! Public facade for the Kapitsa physics engine.
//!
//! Re-exports the synchronous world, math primitives, runtime configuration,
//! and (with the `service` feature) the asynchronous simulation client.
#![deny(missing_docs)]

// Runtime internals are documented incrementally; the published facade modules below
// are checked with `deny(missing_docs)`.
#[allow(missing_docs)]
mod runtime;
pub use runtime::{PhysicsSystem, PhysicsSystemBuilder};
#[allow(missing_docs)]
mod unified_error;

#[cfg(test)]
extern crate self as kapitsa;

/// Execution strategies for stepping and parallel work.
pub mod execution {
    #[cfg(feature = "parallel")]
    pub use crate::runtime::RayonExecutor;
    #[cfg(feature = "service")]
    pub use crate::runtime::{RayonPool, RayonPoolError};
    pub use kapitsa_core::execution::{ExecutionError, SequentialExecutor, StepExecutor};
}

/// Aerodynamic models, atmospheres, and force inputs.
pub mod aerodynamics {
    pub use kapitsa_core::aerodynamics::{
        AerodynamicForce, AerodynamicProperties, AerodynamicSurface, AerodynamicTarget,
        Aerodynamics, AerodynamicsError, AerodynamicsTask, Atmosphere, AtmosphericState,
        ColliderTessellation, DensityProfile, MeshPressureModel, SphericalAtmosphere,
        SphericalAtmosphereBuilder, cache, colliders, geometry, hull, potential, pressure, shape,
        slender, vlm, wave, wing,
    };
}

/// Fluid and hydrodynamic tasks applied to a world.
pub mod hydrodynamics {
    pub use kapitsa_core::hydrodynamics::HydrodynamicsTask;
}

/// Axis-aligned bounding boxes.
pub mod aabb {
    pub use kapitsa_core::aabb::{Aabb, AabbError};
}

/// Collider shapes, filters, and collision modes.
pub mod collider {
    pub use kapitsa_core::collider::{Collider, ColliderError, CollisionFilter, CollisionMode};
}

/// Chunk geometry assembly and registration.
pub mod chunk {
    pub use kapitsa_core::chunk::{Chunk, ChunkBuildError, ChunkBuilder};
}

/// Collision detection, contacts, and query types.
pub mod collision {
    pub use kapitsa_core::collision::{
        CcdLimitPolicy, CollisionConfig, CollisionError, CollisionEvent, CollisionEventKind,
        CollisionPair, ContactInfo,
    };
}

/// Rigid clusters, welds, and mass transfer.
pub mod cluster {
    pub use kapitsa_core::cluster::{
        ClusterPart, ClusterPartDesc, MassChangeMode, MassTransfer, RigidCluster, Weld, WeldDesc,
        WeldStiffness, WeldStrength,
    };
    pub use kapitsa_core::world::{ClusterError, PartId, WeldId};
}

/// Gravitational fields and parameters.
pub mod gravity {
    pub use kapitsa_core::gravity::{
        CentralGravity, Gravity, GravityError, SphericalGravity, UniformGravity,
    };
}

/// Joint types, frames, limits, drives, and break events.
pub mod joint {
    pub use kapitsa_core::joint::{
        BallJoint, ConeTwistJoint, CylindricalJoint, D6Joint, DistanceJoint, FixedJoint,
        HingeJoint, Joint, JointBreakEvent, JointCommon, JointDrive, JointError, JointFrame,
        JointLimit, JointMotion, PlanarJoint, SliderJoint, SpringJoint,
    };
}

/// Mass, center of mass, and inertia properties.
pub mod mass {
    pub use kapitsa_core::mass::{MassError, MassProperties};
}

/// Physics material properties and material storage.
pub mod material {
    pub use kapitsa_core::material::{MaterialId, PhysicsMaterial, PhysicsMaterialError};
}

/// Vector, matrix, quaternion, and affine primitives.
pub mod math {
    pub use kapitsa_math::{Affine3, Mat3, Quat, Vec2, Vec3};
}

/// Ray casts, sweeps, and overlap queries.
pub mod query {
    pub use kapitsa_core::collision::{OverlapHit, QueryError, QueryFilter, Ray, RayHit, ShapeHit};
}

/// Rigid body state, forces, and integration modes.
pub mod rigid_body {
    pub use kapitsa_core::rigid_body::{
        BodyError, BodyForces, BodyPhysics, BodyState, BodyType, CcdMode, IntegrationError,
        RigidBody,
    };
}

/// Collision shapes and their bounds.
pub mod shape {
    pub use kapitsa_core::shape::{
        BoneInfluence, BoundsError, Capsule, Convex, Cuboid, Cylinder, HeightField, LevelSet,
        Plane, Shape, ShapeError, SkinnedTriangleMesh, Sphere, TaperedCapsule, TaperedCylinder,
        Triangle, TriangleMesh,
    };
}

/// Rigid transforms with translation and rotation.
pub mod transform {
    pub use kapitsa_core::transform::{Transform, TransformError};
}

/// The synchronous physics world and its identifiers.
pub mod world {
    pub use kapitsa_core::world::BodyShapeStatus;
    pub use kapitsa_core::world::{
        AddChunkBatchError, AddChunkError, ChunkId, ChunkOperationError, PhysicsWorld,
        RemoveChunkBatchError, StepResults,
    };
    pub use kapitsa_core::world::{
        BodyId, ClusterError, ColliderId, ErrorPolicy, ForceSource, ForceSourceError,
        ForceSourceId, GravityId, JointId, JointTarget, PartId, PartWingId, PartWingWrench,
        StepDiagnostics, StepError, WeldId, WorldError,
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
    AdaptiveTimeStepConfig, ConfigError, RecoveryBlock, RecoveryConfig, RecoveryStatus,
    SchedulerConfig, SimulationConfig, SimulationMetrics, SimulationState, StepMeasurement,
    StepRecoveryPolicy, StepStats, TimeStepPolicy, WorkerConfig, WorkerConfigError,
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
/// Common imports for typical simulation code.
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
