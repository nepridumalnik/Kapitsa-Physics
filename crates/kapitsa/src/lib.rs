mod runtime;

pub mod aabb {
    pub use kapitsa_core::aabb::{Aabb, AabbError};
}

pub mod collider {
    pub use kapitsa_core::collider::{Collider, ColliderError, CollisionFilter, CollisionMode};
}

pub mod collision {
    pub use kapitsa_core::collision::{
        CcdLimitPolicy, CollisionConfig, CollisionError, CollisionEvent, CollisionEventKind,
        CollisionPair, ContactInfo,
    };
}

pub mod gravity {
    pub use kapitsa_core::gravity::{CentralGravity, GravityError};
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
        BodyId, ColliderId, ForceSource, ForceSourceError, ForceSourceId, JointId, StepError,
        WorldError,
    };
}

pub use kapitsa_core::{
    collider::{Collider, CollisionFilter, CollisionMode},
    collision::{
        CcdLimitPolicy, CollisionConfig, CollisionError, CollisionEvent, CollisionEventKind,
        CollisionPair, ContactInfo,
    },
    gravity::{CentralGravity, GravityError},
    joint::{Joint, JointBreakEvent},
    mass::MassProperties,
    material::{MaterialId, PhysicsMaterial},
    rigid_body::{BodyState, CcdMode, RigidBody},
    shape::Shape,
    transform::Transform,
    world::{BodyId, ColliderId, ForceSourceId, JointId, StepError, WorldError},
};

pub use runtime::{
    AdaptiveTimeStepConfig, BatchId, CollisionEventBatch, CollisionEventStreamError,
    CollisionEvents, ConfigError, JointBreakEventBatch, JointBreakEventStreamError,
    JointBreakEvents, PhysicsBatchError, PhysicsBatchResult, PhysicsBatchResultStreamError,
    PhysicsBatchResults, PhysicsClient, PhysicsClientError, PhysicsCommand, PhysicsCommandBuffer,
    PhysicsCommandResult, PhysicsSnapshot, PhysicsSubmitError, SchedulerConfig, Simulation,
    SimulationConfig, SimulationError, SimulationMetrics, SimulationState, StepMeasurement,
    StepStats, TimeStepPolicy, WorkerConfig, WorkerConfigError,
};
