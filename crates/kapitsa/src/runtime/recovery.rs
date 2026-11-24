//! Pure classification and bounded numerical retry decisions.

use std::time::Duration;

use kapitsa_core::{
    Error,
    collision::CollisionError,
    world::{ErrorPolicy, StepError},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FailureClass {
    ReduceStep,
    CorrectiveInput,
    TransientWorker,
    Fatal,
}

/// `intact_worker` is asserted only by an executor boundary that guarantees
/// ownership and rollback; `WorkerFailed` by itself does not establish it.
pub(crate) fn classify(error: &Error, _policy: ErrorPolicy, intact_worker: bool) -> FailureClass {
    let Some(step) = error.downcast_ref::<StepError>() else {
        return FailureClass::Fatal;
    };
    match step {
        StepError::Collision(cause) => match cause.downcast_ref::<CollisionError>() {
            Some(CollisionError::CcdLimitExceeded) => FailureClass::ReduceStep,
            Some(CollisionError::WorkerFailed) if intact_worker => FailureClass::TransientWorker,
            Some(CollisionError::WorkerFailed) | None => FailureClass::Fatal,
            Some(_) => FailureClass::CorrectiveInput,
        },
        StepError::InvalidStepResults | StepError::WeldSplit(_) => FailureClass::Fatal,
        StepError::InvalidDeltaTime
        | StepError::BodyIntegration { .. }
        | StepError::ForceSource { .. }
        | StepError::Gravity { .. }
        | StepError::Aerodynamics { .. }
        | StepError::WeldBalance(_) => FailureClass::CorrectiveInput,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RetryDecision {
    Retry(Duration),
    Blocked,
    Fatal,
}

pub(crate) fn decide(
    class: FailureClass,
    attempted: Duration,
    physical_bound: Duration,
    floor: Duration,
    retries: usize,
    max_retries: usize,
) -> RetryDecision {
    if class == FailureClass::Fatal {
        return RetryDecision::Fatal;
    }
    if class != FailureClass::ReduceStep
        || floor.is_zero()
        || attempted.is_zero()
        || retries >= max_retries
    {
        return RetryDecision::Blocked;
    }
    let next = (attempted / 2).min(physical_bound);
    if next < floor || next >= attempted {
        RetryDecision::Blocked
    } else {
        RetryDecision::Retry(next)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kapitsa_core::{ErrorKind, rigid_body::IntegrationError};

    #[test]
    fn strict_ccd_limit_can_retry_a_smaller_step_without_changing_policy()
    -> Result<(), Box<dyn std::error::Error>> {
        use kapitsa_core::{
            collider::Collider,
            collision::{CcdLimitPolicy, CollisionConfig},
            mass::MassProperties,
            material::PhysicsMaterial,
            rigid_body::{CcdMode, RigidBody},
            shape::{Cuboid, Shape, Sphere},
            transform::Transform,
            world::PhysicsWorld,
        };
        use kapitsa_math::{Mat3, Quat, Vec3};

        let mut world = PhysicsWorld::new();
        world.set_error_policy(ErrorPolicy::Strict);
        let material = world.add_material(PhysicsMaterial::new(0.0, 0.0, 1.0, 1.0)?)?;
        for x in [0.0, 10.0] {
            let wall = world.add_body(RigidBody::new_static(Transform::new(
                Vec3::X * x,
                Quat::IDENTITY,
            )?))?;
            let collider = world.add_collider(
                wall,
                Collider::new(
                    Shape::Cuboid(Cuboid::new(0.1, 4.0, 4.0)?),
                    Transform::identity(),
                ),
            )?;
            world.set_collider_material(collider, Some(material))?;
        }
        let mut body = RigidBody::new_dynamic(
            Transform::new(Vec3::X * 5.0, Quat::IDENTITY)?,
            MassProperties::new(1.0, Vec3::ZERO, Mat3::IDENTITY)?,
        )?;
        body.set_linear_velocity(Vec3::X * -2000.0)?;
        body.set_ccd_mode(CcdMode::Continuous);
        let id = world.add_body(body)?;
        let collider = world.add_collider(
            id,
            Collider::new(Shape::Sphere(Sphere::new(0.5)?), Transform::identity()),
        )?;
        world.set_collider_material(collider, Some(material))?;
        world.set_collision_config(
            CollisionConfig::builder()
                .max_ccd_impacts(1)
                .ccd_limit_policy(CcdLimitPolicy::AbortStep)
                .build(),
        )?;
        let before = world.body(id).ok_or("body")?.motion();
        let error = world.step(0.01).err().ok_or("expected CCD limit")?;
        assert_eq!(
            classify(&error, ErrorPolicy::Strict, false),
            FailureClass::ReduceStep
        );
        assert_eq!(world.body(id).ok_or("body")?.motion(), before);
        world.step(0.005)?;
        assert_eq!(world.error_policy(), ErrorPolicy::Strict);
        assert_eq!(
            world.collision_config().ccd_limit_policy(),
            CcdLimitPolicy::AbortStep
        );
        Ok(())
    }

    #[test]
    fn only_proven_causes_are_retryable() {
        let collision = |cause| Error::from(StepError::Collision(Error::from(cause)));
        assert_eq!(
            classify(
                &collision(CollisionError::CcdLimitExceeded),
                ErrorPolicy::Realtime,
                false
            ),
            FailureClass::ReduceStep
        );
        assert_eq!(
            classify(
                &collision(CollisionError::CcdLimitExceeded),
                ErrorPolicy::Strict,
                false
            ),
            FailureClass::ReduceStep
        );
        assert_eq!(
            classify(
                &collision(CollisionError::NonConvergent),
                ErrorPolicy::Realtime,
                false
            ),
            FailureClass::CorrectiveInput
        );
        assert_eq!(
            classify(
                &collision(CollisionError::WorkerFailed),
                ErrorPolicy::Realtime,
                false
            ),
            FailureClass::Fatal
        );
        assert_eq!(
            classify(
                &collision(CollisionError::WorkerFailed),
                ErrorPolicy::Realtime,
                true
            ),
            FailureClass::TransientWorker
        );
        assert_eq!(
            classify(
                &Error::from(StepError::InvalidDeltaTime),
                ErrorPolicy::Realtime,
                false
            ),
            FailureClass::CorrectiveInput
        );
        assert_eq!(
            classify(
                &Error::from(IntegrationError::ArithmeticOverflow),
                ErrorPolicy::Realtime,
                false
            ),
            FailureClass::Fatal
        );
        assert_eq!(
            classify(
                &Error::new(ErrorKind::Step, "unknown"),
                ErrorPolicy::Realtime,
                false
            ),
            FailureClass::Fatal
        );
    }

    #[test]
    fn retries_strictly_shrink_and_respect_all_bounds() {
        let ms = Duration::from_millis;
        assert_eq!(
            decide(FailureClass::ReduceStep, ms(10), ms(3), ms(1), 0, 8),
            RetryDecision::Retry(ms(3))
        );
        assert_eq!(
            decide(FailureClass::ReduceStep, ms(2), ms(9), ms(1), 0, 8),
            RetryDecision::Retry(ms(1))
        );
        assert_eq!(
            decide(FailureClass::ReduceStep, ms(1), ms(9), ms(1), 0, 8),
            RetryDecision::Blocked
        );
        assert_eq!(
            decide(
                FailureClass::ReduceStep,
                Duration::from_nanos(1),
                Duration::MAX,
                Duration::from_nanos(1),
                0,
                8
            ),
            RetryDecision::Blocked
        );
        assert_eq!(
            decide(
                FailureClass::ReduceStep,
                Duration::MAX,
                Duration::MAX,
                ms(1),
                usize::MAX,
                usize::MAX
            ),
            RetryDecision::Blocked
        );
        assert_eq!(
            decide(FailureClass::CorrectiveInput, ms(10), ms(10), ms(1), 0, 8),
            RetryDecision::Blocked
        );
        assert_eq!(
            decide(FailureClass::Fatal, ms(10), ms(10), ms(1), 0, 8),
            RetryDecision::Fatal
        );
        assert_eq!(
            decide(
                FailureClass::ReduceStep,
                ms(10),
                ms(10),
                Duration::ZERO,
                0,
                8
            ),
            RetryDecision::Blocked
        );
    }
}
